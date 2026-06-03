//! hvn-profiler v0.4.0 — JSON-driven 2D grid + 3D trajectory view.
//!
//! Backends:
//! - `mock://`              → synthetic sine wave (v0.0.1 demo, preserved)
//! - `zmq://host:port`      → subscribe to the HVN-SITL msgpack streamer
//! - `mavlink://host:port`  → direct MAVLink UDP (udpin / listen): real drone or ArduPilot SITL, no Python streamer (v0.4.0)
//! - `mavlinkout://host:port` → direct MAVLink UDP (udpout / send-first)
//!
//! Rendering:
//! - With `--template <PATH>`: load the JSON template. A top toolbar offers a
//!   view-mode switch — `2D grid` / `3D view` / `Split` — when the template
//!   carries a `view_3d` block (default `Split`, else `2D grid`):
//!   - `2D grid` lays the template's `cells` out on a `grid.rows × grid.cols`
//!     grid, one static auto-scaling `egui_plot::Plot` per visible panel. The
//!     2D panels have NO live controls by design (unchanged from v0.2.0).
//!   - `3D view` projects the `view_3d` trails through an orbit-camera painter
//!     and exposes the 3D-only live controls (view / trail-length / zoom /
//!     decimation / realtime / per-trail visibility). See `profiler_render::view3d`.
//!   - `Split` shows the 2D grid and 3D view side-by-side.
//! - Without `--template`: fall back to v0.1.0 single-trace mode (prefers
//!   `accel[0]`, else the busiest channel) so the binary works even when no
//!   template file is present.

use std::time::Instant;

use clap::{Parser, ValueEnum};
use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints};
use std::collections::HashMap;

use profiler_render::{
    add_panel_draft, apply_trail_draft, collect_source_keys, compact_cells, first_available_slot,
    group_source_keys, relocate_cell, remove_cell_at, replace_cell_at, render_faults_panel,
    render_gen_panel, render_view3d_with_override, swap_cells,
    CellMenuAction, EditHistory, FaultsPanelState, GeneratorPanelState, GridRenderOptions,
    LabelOverride, PanelDraft, PanelState, PendingCommand, SeenDrones, TraceStore, TrailDraft,
    View3dState,
};
use profiler_source::{
    DiscoveredSource, DiscoveryStatus, FaultCommand, FaultPublisher, MavlinkConfig, Source,
    SourceListEntry, SourceRegistry, Value as SampleValue, DEFAULT_PROBE_MS,
};
use profiler_template::{
    discover as discover_templates, ensure_user_templates_dir, load_entry_json, LabelMode,
    Template, TemplateEntry, TemplateOrigin, UiState,
};

/// Max samples drained from the source per render frame. Caps wall-clock
/// time spent in `update` when the ZMQ backend is wildly ahead.
const MAX_DRAIN_PER_FRAME: usize = 5_000;

/// Preferred key to render when present in the store.
const PREFERRED_KEY: &str = "accel[0]";

/// HVN profiler — GPU-accelerated telemetry viewer.
#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
struct Cli {
    /// Telemetry source URI. Repeatable (v0.9.0).
    ///
    /// Pass `--source URI` once per drone you want to ingest from. Each URI
    /// is opened with its own backend, and the per-drone TraceStore is keyed
    /// by the `drone_name` carried on each envelope (or, for sources that
    /// don't supply one, by a fallback derived from the URI). The toolbar's
    /// "Drone" dropdown picks which drone's data the 2D / 3D panels display.
    ///
    /// Supported schemes:
    /// - `mock://`               — synthetic sine wave (default)
    /// - `zmq://host:port`       — subscribe to the SITL msgpack streamer
    /// - `mavlink://host:port` — direct MAVLink UDP, bind/listen (udpin)
    /// - `mavlinkout://host:port` — direct MAVLink UDP, send-first (udpout)
    ///
    /// Examples:
    /// - `--source zmq://127.0.0.1:9005`
    /// - `--source zmq://127.0.0.1:9005 --source zmq://127.0.0.1:9006`
    /// - `--source zmq://127.0.0.1:9005 --source mavlink://0.0.0.0:14550`
    ///
    /// v0.16.0 — when omitted, the profiler runs a one-shot localhost
    /// discovery scan (`zmq://127.0.0.1:9005..=9020` + canonical MAVLink
    /// ports) and auto-connects to every Live source it finds. If none are
    /// found, it falls back to `mock://`. Passing `--source` at least once
    /// disables auto-discovery — explicit URIs are honoured verbatim.
    #[arg(long)]
    source: Vec<String>,

    /// Path to a JSON template describing the panel grid.
    ///
    /// When given, renders the multi-panel 2D layout. When omitted, falls back
    /// to v0.1.0 single-trace mode.
    #[arg(long)]
    template: Option<String>,

    /// Global per-panel label overlay mode override.
    ///
    /// `off` (default, v0.9.0) suppresses every cell's overlay regardless of
    /// what the template asked for — launching with no flag produces clean,
    /// label-free panels. `template` honours each cell's own `label_mode`
    /// from the JSON template (the v0.5.0–v0.8.0 default). `data`/`metadata`
    /// force every cell into that mode at startup. The toolbar selector
    /// can flip the override at runtime.
    #[arg(long, value_enum, default_value_t = LabelArg::Off)]
    labels: LabelArg,

    /// Show the Faults & Interference side panel (v0.6.0).
    ///
    /// `off` (default) keeps the v0.5.0 read-only behaviour intact.
    /// `on` opens an outbound ZMQ PUB to `--fault-channel` and renders a
    /// collapsible right-side panel with GPS / IMU / Mag / Baro sliders +
    /// one-shot dropout / freeze / spike buttons. The panel can also be
    /// toggled at runtime via the toolbar button.
    #[arg(long, value_enum, default_value_t = FaultPanelArg::Off)]
    fault_panel: FaultPanelArg,

    /// Endpoint the FaultPublisher PUBs to when `--fault-panel on` (v0.6.0).
    ///
    /// Defaults to the SITL `runtime_control` dispatcher frontend
    /// (`tcp://127.0.0.1:9003`) — the PUB → XSUB proxy that fans commands
    /// out to all running sims.
    #[arg(long, default_value = "tcp://127.0.0.1:9003")]
    fault_channel: String,

    /// Force a specific drone as the initial Target in the Faults panel (v0.7.0).
    ///
    /// Useful for headless / scripted runs where you want commands aimed at
    /// a particular drone before the streamer's first envelope from it has
    /// arrived. The name is appended to the dropdown's discovered list
    /// (deduplicated). Falls back to `"all"` when omitted.
    #[arg(long)]
    drone: Option<String>,

    /// Show the Signal Generators panel (v0.7.0).
    ///
    /// `off` (default) keeps the v0.6.0 behaviour intact. `on` opens the
    /// panel for adding waveform-driven generators that write into the
    /// Faults sliders at 20 Hz. Implies `--fault-panel on` because
    /// generators feed the Faults publisher.
    #[arg(long, value_enum, default_value_t = GeneratorsArg::Off)]
    generators: GeneratorsArg,

    /// MAVLink passive-listener mode (v0.8.0).
    ///
    /// `off` (default) — when the source is `mavlink://` or `mavlinkout://`,
    /// the profiler acts as an active GCS: sends a 1 Hz HEARTBEAT on the
    /// same socket and a one-shot `REQUEST_DATA_STREAM(ALL, 10 Hz)` after
    /// the vehicle's first inbound HEARTBEAT. This is what wakes the rich
    /// messages on real ArduPilot serials whose stock stream is just
    /// `GLOBAL_POSITION_INT` / `GPS_RAW_INT` / `SYS_STATUS` / `HEARTBEAT`.
    /// `on` — restores the v0.4.0 listen-only behaviour (no outgoing
    /// traffic), useful when sharing a port with another GCS that already
    /// drives stream requests, or for sniffing via `mavlinkrouter`.
    #[arg(long, value_enum, default_value_t = MavlinkPassiveArg::Off)]
    mavlink_passive: MavlinkPassiveArg,

    /// v0.16.2 — log one line per drained envelope showing how the router
    /// resolved its `drone_name` → per-drone TraceStore mapping.
    ///
    /// Output format (per sample, info level):
    ///
    /// ```text
    /// [routing] sample drone='eric_1' key='ap_attitude[0]' value=Scalar(-0.12) → store='eric_1'
    /// ```
    ///
    /// Used to confirm whether the bug "panel shows wrong drone" lives in
    /// the routing layer (this trace will show samples landing in the wrong
    /// bucket) or in the renderer (routing is right, but the renderer reads
    /// from the wrong store). Off by default — the trace is firehose-level
    /// on a 10 Hz multi-drone fleet.
    #[arg(long, default_value_t = false)]
    debug_routing: bool,

    /// v0.16.4 — explicit MAVLink `system_id` → drone-name mapping.
    ///
    /// Comma-separated list of `SYSID=NAME` pairs, e.g.
    /// `--drone-map "1=eric_1,2=eric_2,3=eric_3"`. Used by MAVLink sources
    /// to label samples consistently with the ZMQ envelope's `drone_name`
    /// so the picker merges samples from the same physical drone across
    /// both transports (otherwise a `udpin:14560` link would produce a
    /// `drone_1` entry alongside the ZMQ `eric_1` entry, double-counting
    /// every vehicle).
    ///
    /// When absent or empty, MAVLink samples fall back to `drone_<sysid>`.
    /// Unmapped sysids in a non-empty map ALSO fall back to `drone_<sysid>`
    /// (the operator's mapping is an allowlist of labels, not a filter).
    ///
    /// Has no effect on ZMQ sources (the streamer envelope already carries
    /// the authoritative `drone_name`).
    #[arg(long, value_parser = parse_drone_map, default_value = "")]
    drone_map: DroneMap,

    /// v0.16.4 — disable the auto stream-request that fires on every newly
    /// seen MAVLink sysid.
    ///
    /// Default (flag absent): on every previously-unseen `system_id` on a
    /// `mavlink://` source, the profiler sends one `REQUEST_DATA_STREAM(ALL,
    /// 10 Hz)` aimed at that sysid. This wakes the rich vocabulary
    /// (EKF_STATUS_REPORT, ATTITUDE, RAW_IMU, …) on every drone sharing the
    /// port — including the 25-drone fan-out on a single secondary GCS
    /// mirror (e.g. `:14560`).
    ///
    /// Pass this flag to suppress the auto-request — use when another GCS
    /// is already requesting streams from the same vehicle, or when
    /// listening through a `mavlink-router` fan-out that mustn't see
    /// profiler-originated traffic.
    ///
    /// Implies nothing about `--mavlink-passive` (which is a stricter
    /// listen-only mode that also suppresses the 1 Hz heartbeat).
    #[arg(long, default_value_t = false)]
    no_mavlink_active_gcs: bool,
}

/// v0.16.4 — parsed `--drone-map` value: a `system_id → name` table.
#[derive(Debug, Clone, Default)]
struct DroneMap(std::collections::HashMap<u8, String>);

fn parse_drone_map(raw: &str) -> Result<DroneMap, String> {
    let mut map = std::collections::HashMap::new();
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return Ok(DroneMap(map));
    }
    for entry in trimmed.split(',') {
        let entry = entry.trim();
        if entry.is_empty() {
            continue;
        }
        let (sysid_str, name) = entry.split_once('=').ok_or_else(|| {
            format!("invalid --drone-map entry '{entry}': expected SYSID=NAME")
        })?;
        let sysid: u8 = sysid_str.trim().parse().map_err(|e| {
            format!("invalid sysid in --drone-map entry '{entry}': {e}")
        })?;
        let name = name.trim();
        if name.is_empty() {
            return Err(format!(
                "empty drone name in --drone-map entry '{entry}'"
            ));
        }
        map.insert(sysid, name.to_string());
    }
    Ok(DroneMap(map))
}

/// CLI on/off for `--mavlink-passive`. (v0.8.0)
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum MavlinkPassiveArg {
    /// Active GCS (heartbeat + REQUEST_DATA_STREAM) — v0.8.0 default.
    Off,
    /// Listen-only — restores v0.4.0 behaviour.
    On,
}

/// CLI on/off for `--generators`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum GeneratorsArg {
    /// Panel hidden, no auto-drive (v0.6.0 default).
    Off,
    /// Open the Signal Generators panel; implies `--fault-panel on`.
    On,
}

/// CLI on/off for `--fault-panel`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum FaultPanelArg {
    /// Read-only (v0.5.0 default) — no ZMQ PUB opened, no panel shown.
    Off,
    /// Open the PUB socket and render the panel. Toolbar can toggle visibility.
    On,
}

/// CLI value for `--labels`. Maps onto [`LabelOverride`] / [`LabelMode`].
///
/// v0.9.0 flipped the default from `Template` → `Off`: launching the profiler
/// with no flag now shows clean, label-free panels. Templates can still opt
/// individual cells in via their own `label_mode` field, but the user has to
/// pick `--labels template` (or the toolbar's "template" item) to honour them.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, ValueEnum)]
enum LabelArg {
    /// Honour each cell's own `label_mode` from the template.
    Template,
    /// Force every cell to draw no overlay. v0.9.0 global default.
    #[default]
    Off,
    /// Force every cell to draw the data overlay.
    Data,
    /// Force every cell to draw the metadata overlay.
    Metadata,
}

impl LabelArg {
    fn to_override(self) -> LabelOverride {
        match self {
            LabelArg::Template => LabelOverride::Respect,
            LabelArg::Off => LabelOverride::Force(LabelMode::Off),
            LabelArg::Data => LabelOverride::Force(LabelMode::Data),
            LabelArg::Metadata => LabelOverride::Force(LabelMode::Metadata),
        }
    }
}

fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let cli = Cli::parse();
    log::info!(
        "hvn-profiler v{} starting (sources={:?}, template={:?}, labels={:?}, fault_panel={:?}, generators={:?}, drone={:?})",
        env!("CARGO_PKG_VERSION"),
        if cli.source.is_empty() { vec!["(auto-discover)".to_string()] } else { cli.source.clone() },
        cli.template,
        cli.labels,
        cli.fault_panel,
        cli.generators,
        cli.drone,
    );

    let mav_cfg = MavlinkConfig {
        passive: matches!(cli.mavlink_passive, MavlinkPassiveArg::On),
        // v0.10.0 — when the operator passes `--drone NAME`, pin every MAVLink
        // sample's `drone_name` to that string instead of the default
        // `drone_{id}` demux. v0.16.4 — when `--drone-map` is non-empty it
        // takes precedence; `--drone NAME` is only meaningful for single-
        // vehicle links.
        drone_name_override: cli.drone.clone(),
        // v0.16.4 — explicit sysid → drone_name table from `--drone-map`.
        // Empty by default → MAVLink samples carry `drone_{sysid}`. Non-empty
        // → SITL-style names like `eric_1` are stamped per sysid so the
        // picker merges samples with ZMQ sources by sysid.
        sysid_map: cli.drone_map.0.clone(),
        // v0.16.4 — default ON. `--no-mavlink-active-gcs` turns it off so
        // shared / router-fanned ports get a strictly-passive listener.
        active_gcs: !cli.no_mavlink_active_gcs,
    };
    if !mav_cfg.sysid_map.is_empty() {
        log::info!(
            "MAVLink sysid map: {} entr{} ({:?})",
            mav_cfg.sysid_map.len(),
            if mav_cfg.sysid_map.len() == 1 { "y" } else { "ies" },
            mav_cfg.sysid_map,
        );
    }
    if !mav_cfg.active_gcs {
        log::info!("--no-mavlink-active-gcs: per-sysid REQUEST_DATA_STREAM disabled");
    }

    // v0.16.0 — shared async runtime used for the Add-Source dialog's
    // localhost discovery scan AND the bare-launch auto-connect path
    // below. Multi-thread flavour so a scan-in-flight doesn't block the
    // egui paint thread when it grabs the runtime to spawn the next one.
    let runtime = std::sync::Arc::new(
        tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .thread_name("hvn-profiler-discovery")
            .build()
            .map_err(|e| anyhow::anyhow!("building discovery tokio runtime: {e}"))?,
    );

    // v0.16.0 — ALWAYS run a one-shot localhost discovery scan at startup
    // and auto-connect to every Live source it finds (in addition to any
    // explicit --source URIs). The scan budget is `DEFAULT_PROBE_MS`
    // (typically 500 ms), so launching the profiler stays snappy.
    //
    // Explicit URIs take precedence: discovery hits that match an explicit
    // URI are not added a second time (the `SourceRegistry::add*` calls
    // would no-op anyway, but skipping them keeps the startup log tidy).
    //
    // If neither the operator nor discovery provided any sources, fall
    // back to `mock://` so the toolchain-proof demo still works.
    let mut startup_uris: Vec<String> = cli.source.clone();
    log::info!(
        "running localhost discovery scan (≤ {} ms)",
        profiler_source::DEFAULT_PROBE_MS,
    );
    let scan = runtime.block_on(profiler_source::discover_localhost_sources(
        &startup_uris,
        profiler_source::DEFAULT_PROBE_MS,
    ));
    let mut autodiscovered: Vec<String> = Vec::new();
    for d in &scan {
        if matches!(d.status, profiler_source::DiscoveryStatus::Live { .. })
            && !startup_uris.iter().any(|u| u == &d.uri)
        {
            autodiscovered.push(d.uri.clone());
        }
    }
    if autodiscovered.is_empty() {
        if startup_uris.is_empty() {
            log::info!(
                "discovery found no live sources and no --source given → mock://",
            );
            startup_uris.push("mock://".to_string());
        } else {
            log::info!(
                "discovery found no additional live sources beyond {:?}",
                startup_uris,
            );
        }
    } else {
        log::info!(
            "discovery: auto-connecting to {} additional live source(s): {:?}",
            autodiscovered.len(),
            autodiscovered,
        );
        startup_uris.extend(autodiscovered.iter().cloned());
    }

    // v0.15.0 — `SourceRegistry` replaces `MultiSource` as the runtime
    // source container. CLI-supplied URIs are added at startup; the in-app
    // Sources dropdown can add/remove more after that. ZMQ sources are
    // routed through `add_zmq` so the toolbar can read the live
    // `last_drone_name` slot for the dropdown's "this is eric_1" label.
    let mut registry = SourceRegistry::new(mav_cfg.clone());
    for uri in &startup_uris {
        let res = if uri.starts_with("zmq://") {
            registry.add_zmq(uri)
        } else {
            registry.add(uri)
        };
        if let Err(e) = res {
            log::warn!("failed to open startup source '{uri}': {e}");
        }
    }
    let seen_drones = Some(registry.merged_seen_drones());
    let source_desc = registry.describe();

    // --generators on implies --fault-panel on (generators feed the Faults
    // publisher). Resolve the effective fault_panel state up-front so the
    // rest of the wiring sees a single source of truth.
    let generators_on = matches!(cli.generators, GeneratorsArg::On);
    let effective_fault_panel = match (cli.fault_panel, generators_on) {
        (FaultPanelArg::On, _) | (_, true) => FaultPanelArg::On,
        _ => FaultPanelArg::Off,
    };

    // Open the outbound fault channel only when explicitly requested.
    // `--fault-panel off` (the default) keeps the v0.5.0 read-only contract.
    let fault_publisher = match effective_fault_panel {
        FaultPanelArg::On => {
            let pubr = FaultPublisher::new(&cli.fault_channel).map_err(|e| {
                anyhow::anyhow!("opening fault channel {}: {e}", cli.fault_channel)
            })?;
            log::info!("FaultPublisher ready on {}", cli.fault_channel);
            Some(pubr)
        }
        FaultPanelArg::Off => None,
    };
    let fault_panel_initial = matches!(effective_fault_panel, FaultPanelArg::On);
    let drone_override = cli.drone.clone();

    // Load the template if given. A parse failure is fatal (the user asked for
    // a specific layout) — surface it rather than silently falling back.
    //
    // v0.14.0 — when no `--template` is provided, fall back to the BUNDLED
    // default (`tutorial`). This replaces the v0.1.0 single-trace fallback as
    // the first-run experience: a 3x3 layout mixing a few key live plots with
    // inline `info_text` instructions. The single-trace path stays reachable
    // for fully bare-bones runs by passing `--template ""` is not supported;
    // operators who genuinely want the old behaviour can ship an empty
    // template file. `hvn-default` remains available in the picker dropdown.
    let template = match &cli.template {
        Some(path) => {
            let tpl = Template::from_path(path)?;
            log::info!(
                "loaded template '{}' ({}x{} grid, {} cells, {} visible)",
                tpl.name,
                tpl.grid.rows,
                tpl.grid.cols,
                tpl.cells.len(),
                tpl.visible_cells().count(),
            );
            Some(tpl)
        }
        None => {
            // v0.14.0 — implicit-default: parse the bundled `tutorial` JSON.
            // Parse errors here are programmer errors (the JSON is compiled
            // into the binary and unit-tested), so surface them rather than
            // silently swallowing.
            match profiler_template::bundled::by_name(
                profiler_template::bundled::DEFAULT_BUNDLED_NAME,
            ) {
                Some(b) => match Template::from_str(b.json) {
                    Ok(tpl) => {
                        log::info!(
                            "no --template given → loading bundled default '{}' ({}x{} grid, {} cells)",
                            tpl.name,
                            tpl.grid.rows,
                            tpl.grid.cols,
                            tpl.cells.len(),
                        );
                        Some(tpl)
                    }
                    Err(e) => {
                        log::warn!(
                            "bundled default template failed to parse ({e}); falling back to single-trace mode",
                        );
                        None
                    }
                },
                None => {
                    log::info!("no bundled default available → single-trace fallback mode");
                    None
                }
            }
        }
    };

    // v0.8.0 — discover bundled + user templates and pick the index for the
    // currently-loaded one (so the picker dropdown shows it as selected).
    // The user templates directory is created lazily so a fresh install can
    // do its first "Save as..." without manual `mkdir`.
    if let Err(e) = ensure_user_templates_dir() {
        log::warn!("could not create user templates dir: {e}");
    }
    let cli_template_path = cli.template.as_ref().map(std::path::PathBuf::from);
    let templates =
        discover_templates(cli_template_path.as_deref());
    let current_template_idx = template.as_ref().and_then(|tpl| {
        let by_name = templates.iter().position(|t| t.name == tpl.name);
        // Prefer matching by CLI path (so "current" pinning is exact when
        // a CLI template's name collides with a bundled one).
        if let Some(path) = cli_template_path.as_deref() {
            let abs = std::fs::canonicalize(path).ok();
            for (i, e) in templates.iter().enumerate() {
                if let Some(p) = e.origin.path() {
                    if abs.is_some()
                        && std::fs::canonicalize(p).ok() == abs
                    {
                        return Some(i);
                    }
                }
            }
        }
        by_name
    });
    log::info!(
        "template picker: {} entries discovered (current: {:?})",
        templates.len(),
        current_template_idx
    );

    // Compact "src1 + src2 + …" for the title bar (Vec<String> in v0.9.0).
    // v0.16.0 — `startup_uris` is `cli.source` with any auto-discovered
    // URIs appended (or `["mock://"]` when nothing was found).
    let sources_summary = if startup_uris.len() == 1 {
        startup_uris[0].clone()
    } else {
        format!("{} sources: {}", startup_uris.len(), startup_uris.join(" + "))
    };
    let autodiscovered_for_app = autodiscovered.clone();
    let title = match &template {
        Some(t) => format!(
            "hvn-profiler v{} — {} — {}",
            env!("CARGO_PKG_VERSION"),
            t.name,
            sources_summary,
        ),
        None => format!(
            "hvn-profiler v{} — {}",
            env!("CARGO_PKG_VERSION"),
            sources_summary,
        ),
    };

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 900.0])
            .with_title(&title),
        ..Default::default()
    };

    eframe::run_native(
        "hvn-profiler",
        native_options,
        Box::new(move |_cc| {
            Ok(Box::new(App::new(
                registry,
                source_desc,
                template,
                cli.labels,
                fault_publisher,
                fault_panel_initial,
                seen_drones,
                drone_override,
                generators_on,
                templates,
                current_template_idx,
                runtime,
                autodiscovered_for_app,
                cli.debug_routing,
            )))
        }),
    )
    .map_err(|e| anyhow::anyhow!("eframe::run_native failed: {e}"))
}

/// Which layout the toolbar is currently showing.
///
/// `Split` and `View3d` are only reachable when the template carries a
/// `view_3d` block; otherwise the toolbar pins to `Grid` (or, with no template
/// at all, the v0.1.0 single-trace fallback runs regardless of this field).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ViewMode {
    /// v0.2.0 multi-panel 2D grid (unchanged, control-free).
    Grid,
    /// 3D trajectory view + its live controls.
    View3d,
    /// 2D grid and 3D view side-by-side.
    Split,
}

/// Drone-name used in the per-drone store map when an envelope arrives with
/// `drone_name == None`. Keeps single-source / single-drone flows working
/// even when the streamer doesn't tag envelopes (older SITL, raw MAVLink).
const UNNAMED_DRONE: &str = "(unnamed)";

/// v0.16.4 — `true` when `name` is the synthetic `drone_<sysid>` label
/// emitted by `MavlinkSource` when no `--drone-map` is supplied. Used by
/// the drain loop's picker-identity merge to decide whether to upgrade the
/// canonical name for a sysid (synthetic loses to meaningful).
fn is_synthetic_drone_name(name: &str, sysid: u8) -> bool {
    let expected = format!("drone_{sysid}");
    name == expected
}

struct App {
    /// v0.15.0 — runtime-mutable source registry. Replaces the v0.9.0
    /// `Box<dyn Source>` (MultiSource) so the toolbar Sources dropdown can
    /// add/remove sources after startup. The drain loop calls
    /// `registry.try_recv()` exactly as before.
    registry: SourceRegistry,
    source_desc: String,
    /// v0.15.0 — last status message from the Sources toolbar (Add /
    /// Remove). Shown briefly next to the Sources button. Cleared when the
    /// next action runs.
    last_source_action: Option<String>,
    /// v0.15.0 — `+ Add Source...` dialog: in-progress URI text. `None`
    /// means the dialog is closed.
    add_source_uri: Option<String>,
    /// v0.16.0 — shared tokio runtime used to spawn the Add-Source
    /// dialog's localhost discovery scan. Same runtime that the bare-launch
    /// auto-connect path used at boot.
    discovery_runtime: std::sync::Arc<tokio::runtime::Runtime>,
    /// v0.16.0 — shared slot for the in-progress / latest discovery scan
    /// result. `(scanning, results)`: while a task is in flight `scanning`
    /// is `true`; once it completes the results land in the `Vec`. The
    /// dialog reads this each frame; the `[🔄 Rescan]` button kicks off a
    /// fresh scan that overwrites the previous result.
    discovery_state: std::sync::Arc<std::sync::Mutex<DiscoveryState>>,
    /// v0.16.0 — banner toast for the bare-launch auto-discovery hit.
    /// `Some` for the first ~5 s after start when ≥1 source was auto-
    /// connected; cleared after the timeout fires so the toolbar relaxes.
    autodiscover_toast: Option<(Instant, String)>,
    /// v0.15.0 — most recent template-fallback warning. Set whenever a
    /// cell's declared `source_uri` doesn't match any connected source;
    /// the toolbar paints it next to the source counter.
    template_fallback_warning: Option<String>,
    /// v0.9.0: per-drone trace storage. Keyed by `Sample.drone_name`. Each
    /// drone gets its own ring buffer so cross-drone keys don't collide
    /// (every drone has its own `accel[0]`, `pos_ekf_ned[i]`, etc.).
    ///
    /// Samples without a `drone_name` fall under [`UNNAMED_DRONE`].
    stores: std::collections::HashMap<String, TraceStore>,
    /// v0.9.0: drone whose data is currently displayed by the renderer.
    /// `None` until the first sample arrives; then bound to the first-seen
    /// drone. The toolbar dropdown lets the user switch this independently
    /// of the Faults panel's target.
    view_drone: Option<String>,
    /// v0.9.0: insertion-ordered list of discovered drone names. Used to
    /// populate the toolbar dropdown — sorted by first-seen so the layout
    /// stays stable as new drones appear.
    discovered_drones: Vec<String>,
    /// Optional template — when present, render the multi-panel grid.
    template: Option<Template>,
    /// Current toolbar view mode.
    mode: ViewMode,
    /// Persisted 3D camera + control state (only used when a `view_3d` exists).
    view3d_state: View3dState,
    /// `true` once the 3D state has been seeded from the template defaults.
    view3d_inited: bool,
    /// Global label-mode override (toolbar / CLI flag).
    label_arg: LabelArg,
    /// v0.6.0: outbound runtime-control publisher. `None` when `--fault-panel off`.
    fault_publisher: Option<FaultPublisher>,
    /// v0.6.0: Faults panel UI state.
    faults_state: FaultsPanelState,
    /// v0.6.0: scratch buffer for commands emitted by the panel each frame.
    pending_fault_cmds: Vec<PendingCommand>,
    /// v0.6.0: cumulative count of fault commands published (for the status row).
    fault_sent: u64,
    /// v0.7.0: Signal Generators panel UI state.
    gen_state: GeneratorPanelState,
    started: Instant,
    drained_total: u64,
    /// Wall-clock of the last "samples-in-store" status log (1 Hz).
    last_status_log: Instant,
    /// v0.8.0: known templates surfaced by the picker dropdown.
    templates: Vec<TemplateEntry>,
    /// v0.8.0: index into `templates` for the currently-active entry.
    current_template: Option<usize>,
    /// v0.8.0: most recent status text from a Save / Save-as / Open action,
    /// shown briefly in the toolbar.
    last_template_action: Option<String>,
    // ── v0.10.0 state ────────────────────────────────────────────────────
    /// Per-cell zoom/pan state. Persists across frames; the entry for
    /// `(row, col)` is created lazily by the renderer on first paint.
    panel_states: HashMap<(usize, usize), PanelState>,
    /// Runtime visibility override (right-click "Hide panel"). Keyed by
    /// `(row, col)`. Absent → use the template's `visible` flag.
    cell_visibility_override: HashMap<(usize, usize), bool>,
    /// Runtime per-cell `label_mode` override (right-click "Label: …").
    cell_label_override: HashMap<(usize, usize), profiler_template::LabelMode>,
    /// Sink for context-menu actions emitted by the renderer this frame.
    /// Drained at the end of `update` so the actions take effect next frame.
    pending_cell_actions: Vec<CellMenuAction>,
    /// Open editor mode: `None` means no modal; otherwise the operator is
    /// editing a panel / trail (Add or Edit). Cleared on Cancel/Add.
    editor: Option<EditorMode>,
    /// Source-key dropdown contents, refreshed on each opens of the editor
    /// from the union of live store keys across every drone.
    editor_source_keys_cache: Vec<String>,
    /// `true` when the loaded template has been mutated since the last save.
    /// Toolbar paints a `●` next to the template name. Ctrl+S clears it.
    template_dirty: bool,
    /// v0.11.0 — undo/redo history. Every editor mutation records the
    /// pre-change template; Ctrl+Z / Ctrl+Y walk back and forth. Capacity
    /// defaults to 64 snapshots; oldest evicted on overflow.
    history: EditHistory,
    /// v0.11.0 — per-category collapsed state for the editor's grouped
    /// source-key dropdown. Shared across every `source_key_combo` widget
    /// (Add Panel, Edit Panel, Add Trail) so toggling "DT physics" in one
    /// form also collapses it in the next — matches the operator's mental
    /// model and avoids re-toggling on every modal open.
    ///
    /// Replaces the v0.10.2 `egui::CollapsingHeader` inside the ComboBox
    /// popup, which dismissed the entire popup whenever the operator clicked
    /// the ▶/▼ arrow.
    editor_combo_collapse: profiler_render::ComboCollapseState,
    /// v0.12.0 — last monotonic seconds at which a non-null value arrived
    /// for each known key. Drives the editor's freshness coloring (Live /
    /// Stale / Schema-only / Custom) in the source-key picker dropdown.
    /// Refreshed on every `drain()` call.
    last_seen_keys: HashMap<String, f64>,
    /// v0.12.0 — picker type-filter row state (Status / 2D scalar / 2D
    /// vector / 3D). Persists across modal opens so the operator keeps
    /// their preferred filter set.
    picker_filter: profiler_render::PickerTypeFilter,
    /// v0.11.0 — when the window is too narrow for a 50/50 Split layout
    /// (< RESPONSIVE_3D_COLLAPSE_W), the 3D view is rendered as a floating
    /// overlay instead. This flag tracks whether the overlay is currently
    /// open. Defaults `true` so the user sees the 3D view at least once on
    /// first narrow-window launch; the operator can close it to reclaim
    /// pixels for the 2D grid.
    split_3d_overlay_open: bool,
    /// v0.16.2 — when `true`, log one line per drained envelope showing
    /// the routing decision (drone_name → store key). Set from
    /// `--debug-routing` and consulted by [`Self::drain`].
    debug_routing: bool,
    /// v0.16.4 — `system_id` → canonical drone-name resolution table.
    ///
    /// When a sample arrives carrying `sysid = Some(s)`, the drain loop
    /// looks up the canonical drone name for `s` and rewrites the sample's
    /// `drone_name` to that string before routing into `stores`. This is
    /// what merges samples from ZMQ (drone_name `"eric_1"`, sysid 1) and
    /// MAVLink (drone_name `"drone_1"`, sysid 1) into a single picker
    /// entry — they both end up keyed under `"eric_1"` in `stores`, and
    /// the picker shows ONE row, not two.
    ///
    /// Conflict resolution: the first non-synthetic name observed wins (a
    /// name matching `drone_<sysid>` is treated as synthetic; anything
    /// else — `"eric_1"`, an operator-supplied `--drone-map` label — is
    /// authoritative and pins the canonical entry).
    ///
    /// When a sample lacks a sysid (mock / older streamer), it routes by
    /// `drone_name` alone — same as the pre-v0.16.4 behaviour.
    sysid_to_drone: HashMap<u8, String>,
}

/// v0.10.0 — what the modal editor is currently doing.
enum EditorMode {
    /// "+ Add Panel" toolbar click — empty draft.
    AddPanel(PanelDraft),
    /// Per-cell "Edit panel..." right-click — pre-filled draft + original
    /// `(row, col)` so the apply step calls `replace_cell_at`.
    EditPanel { row: usize, col: usize, draft: PanelDraft },
    /// "+ Add Trail" 3D-view click — empty draft.
    AddTrail(TrailDraft),
}

impl App {
    #[allow(clippy::too_many_arguments)]
    fn new(
        registry: SourceRegistry,
        source_desc: String,
        template: Option<Template>,
        label_arg: LabelArg,
        fault_publisher: Option<FaultPublisher>,
        fault_panel_initial: bool,
        seen_drones: Option<SeenDrones>,
        drone_override: Option<String>,
        generators_initial: bool,
        templates: Vec<TemplateEntry>,
        current_template: Option<usize>,
        discovery_runtime: std::sync::Arc<tokio::runtime::Runtime>,
        autodiscovered: Vec<String>,
        debug_routing: bool,
    ) -> Self {
        let now = Instant::now();
        // Default mode: Split when the template ships a 3D view, else Grid.
        let has_3d = template
            .as_ref()
            .and_then(|t| t.view_3d.as_ref())
            .is_some();
        let mode = if has_3d { ViewMode::Split } else { ViewMode::Grid };
        let mut faults_state = FaultsPanelState::default();
        faults_state.visible = fault_panel_initial && fault_publisher.is_some();
        faults_state.seen_drones = seen_drones;
        if let Some(name) = drone_override {
            faults_state.extras.push(name.clone());
            faults_state.drone = name;
        }
        let gen_state = GeneratorPanelState {
            visible: generators_initial && fault_publisher.is_some(),
            ..Default::default()
        };
        let autodiscover_toast = if autodiscovered.is_empty() {
            None
        } else {
            Some((
                now,
                format!(
                    "Auto-connected to {} source(s) discovered on localhost",
                    autodiscovered.len(),
                ),
            ))
        };
        Self {
            registry,
            source_desc,
            last_source_action: None,
            add_source_uri: None,
            discovery_runtime,
            discovery_state: std::sync::Arc::new(std::sync::Mutex::new(
                DiscoveryState::default(),
            )),
            autodiscover_toast,
            template_fallback_warning: None,
            stores: std::collections::HashMap::new(),
            view_drone: None,
            discovered_drones: Vec::new(),
            template,
            mode,
            view3d_state: View3dState::default(),
            view3d_inited: false,
            label_arg,
            fault_publisher,
            faults_state,
            pending_fault_cmds: Vec::new(),
            fault_sent: 0,
            gen_state,
            started: now,
            drained_total: 0,
            last_status_log: now,
            templates,
            current_template,
            last_template_action: None,
            panel_states: HashMap::new(),
            cell_visibility_override: HashMap::new(),
            cell_label_override: HashMap::new(),
            pending_cell_actions: Vec::new(),
            editor: None,
            editor_source_keys_cache: Vec::new(),
            template_dirty: false,
            history: EditHistory::default(),
            editor_combo_collapse: profiler_render::ComboCollapseState::default(),
            last_seen_keys: HashMap::new(),
            picker_filter: profiler_render::PickerTypeFilter::default(),
            split_3d_overlay_open: true,
            debug_routing,
            sysid_to_drone: HashMap::new(),
        }
    }

    /// Drain the source, push into the per-drone store. Caps work per frame.
    ///
    /// Each sample is routed to `stores[drone_name]`, creating a fresh
    /// `TraceStore` for any drone we hear for the first time. The first-seen
    /// drone also seeds `view_drone` so launching with only one source picks
    /// the right drone automatically.
    fn drain(&mut self) {
        let mut n = 0;
        while n < MAX_DRAIN_PER_FRAME {
            match self.registry.try_recv() {
                Some(s) => {
                    // v0.10.1 — Sample.drone_name is `Arc<str>`; route via a
                    // single `to_string()` for the HashMap key.
                    let raw_name = s
                        .drone_name
                        .as_deref()
                        .map(str::to_string)
                        .unwrap_or_else(|| UNNAMED_DRONE.to_string());
                    // v0.16.4 — sysid-based picker identity merge.
                    //
                    // When the sample carries a sysid, register / look up
                    // the canonical drone name for that sysid. The first
                    // NON-SYNTHETIC name observed pins the canonical entry
                    // (a synthetic name matches `drone_<sysid>` exactly —
                    // produced by `MavlinkSource` when no `--drone-map` is
                    // supplied; the ZMQ envelope or operator-mapped MAVLink
                    // label otherwise wins).
                    //
                    // Without a sysid (mock, older streamer envelope), the
                    // route falls back to drone_name alone (pre-v0.16.4
                    // behaviour).
                    let drone_key = match s.sysid {
                        Some(sid) => {
                            let synthetic = is_synthetic_drone_name(&raw_name, sid);
                            match self.sysid_to_drone.get(&sid) {
                                Some(canonical) => {
                                    // Upgrade the canonical entry if we
                                    // currently have a synthetic name AND
                                    // the new sample carries a meaningful
                                    // one (ZMQ arrived second after MAVLink
                                    // synthetic).
                                    let canonical_synthetic =
                                        is_synthetic_drone_name(canonical, sid);
                                    if canonical_synthetic && !synthetic {
                                        // Migrate the existing store under
                                        // the synthetic key to the new
                                        // canonical key. Cheap: we just
                                        // rename the HashMap entry.
                                        let prev = canonical.clone();
                                        self.sysid_to_drone
                                            .insert(sid, raw_name.clone());
                                        if let Some(prev_store) =
                                            self.stores.remove(&prev)
                                        {
                                            self.stores.insert(
                                                raw_name.clone(),
                                                prev_store,
                                            );
                                        }
                                        // Patch the discovered list / view
                                        // selection so the picker shows the
                                        // upgraded name.
                                        for d in self.discovered_drones.iter_mut() {
                                            if *d == prev {
                                                *d = raw_name.clone();
                                            }
                                        }
                                        if self.view_drone.as_deref() == Some(&prev) {
                                            self.view_drone = Some(raw_name.clone());
                                        }
                                        raw_name.clone()
                                    } else {
                                        canonical.clone()
                                    }
                                }
                                None => {
                                    self.sysid_to_drone
                                        .insert(sid, raw_name.clone());
                                    raw_name.clone()
                                }
                            }
                        }
                        None => raw_name.clone(),
                    };
                    // v0.16.2 — `--debug-routing`: one log line per drained
                    // envelope so the operator can audit the drone_name →
                    // store mapping when a panel renders the wrong drone's
                    // data. Off in production (firehose-level at 10 Hz × N
                    // drones).
                    if self.debug_routing {
                        log::info!(
                            "[routing] sample drone={:?} key={:?} value={:?} -> store={:?}",
                            s.drone_name.as_deref().unwrap_or("<none>"),
                            s.key,
                            s.value,
                            drone_key,
                        );
                    }
                    let is_new = !self.stores.contains_key(&drone_key);
                    let store = self
                        .stores
                        .entry(drone_key.clone())
                        .or_default();
                    // v0.11.0 / v0.13.0 — route by payload variant. Schema-
                    // only registrations land in the editor's null-key set;
                    // everything else routes to the matching `TraceStore`
                    // helper (numeric / string / bool / text-log / vector).
                    match &s.value {
                        SampleValue::Null => {
                            store.note_null_key(&s.key);
                        }
                        SampleValue::Scalar(v) => {
                            // NaN sentinel from older paths still triggers
                            // the schema-only registration (legacy v0.11.0
                            // contract — keeps SCHEMA_ONLY_SENTINEL alive).
                            if v.is_nan() {
                                store.note_null_key(&s.key);
                            } else {
                                store.push(s.ts, &s.key, *v);
                                self.last_seen_keys.insert(s.key.clone(), s.ts);
                            }
                        }
                        SampleValue::Bool(b) => {
                            store.push_bool(s.ts, &s.key, *b);
                            self.last_seen_keys.insert(s.key.clone(), s.ts);
                        }
                        SampleValue::String(text) => {
                            store.push_string(s.ts, &s.key, text.as_ref());
                            self.last_seen_keys.insert(s.key.clone(), s.ts);
                        }
                        SampleValue::IntVector(values) => {
                            // The msgpack decoder ALSO emits per-component
                            // scalars for legacy template wiring, but the
                            // base key itself (`rc_channels`) deserves the
                            // typed view too — `push_vec_int` is a no-op
                            // for the per-index keys already pushed.
                            store.push_vec_int(s.ts, &s.key, values);
                            self.last_seen_keys.insert(s.key.clone(), s.ts);
                        }
                        SampleValue::Vector(values) => {
                            // Same rationale as IntVector — the per-index
                            // scalars are emitted by the decoder; we
                            // mirror once into the numeric channel for the
                            // base key so naive scalar plots still work.
                            for (i, v) in values.iter().enumerate() {
                                let key = format!("{}[{}]", s.key, i);
                                store.push(s.ts, &key, *v);
                            }
                            self.last_seen_keys.insert(s.key.clone(), s.ts);
                        }
                        SampleValue::TextLog(entries) => {
                            for entry in entries {
                                store.push_text_log(
                                    s.ts,
                                    &s.key,
                                    profiler_render::TextLogEntry {
                                        ts: entry.ts,
                                        text: entry.text.to_string(),
                                        severity: entry.severity,
                                    },
                                );
                            }
                            self.last_seen_keys.insert(s.key.clone(), s.ts);
                        }
                    }
                    if is_new {
                        self.discovered_drones.push(drone_key.clone());
                        log::info!(
                            "discovered drone '{drone_key}' (now {} known)",
                            self.discovered_drones.len(),
                        );
                        if self.view_drone.is_none() {
                            self.view_drone = Some(drone_key.clone());
                        }
                    }
                    n += 1;
                }
                None => break,
            }
        }
        self.drained_total += n as u64;
    }

    /// Reference to the store currently being displayed, or an empty default
    /// when no samples have arrived yet. Always returns a usable store so the
    /// renderers don't have to special-case the "no data" path beyond what
    /// they already handle for empty rings.
    fn view_store(&self) -> &TraceStore {
        // The static empty store lives in a OnceLock so the borrow lifetime
        // matches `&self`. Cheap (constructed once, retained for process life).
        use std::sync::OnceLock;
        static EMPTY: OnceLock<TraceStore> = OnceLock::new();
        let empty = EMPTY.get_or_init(TraceStore::default);
        match self.view_drone.as_deref().and_then(|d| self.stores.get(d)) {
            Some(s) => s,
            None => empty,
        }
    }

    /// Choose which trace to render this frame (single-trace fallback mode).
    fn select_key(&self) -> Option<String> {
        let store = self.view_store();
        if store.len(PREFERRED_KEY) > 0 {
            Some(PREFERRED_KEY.to_string())
        } else {
            store.busiest_key()
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // Keep the render loop hot.
        ui.ctx().request_repaint();
        self.drain();

        let elapsed = self.started.elapsed().as_secs_f64();
        let has_3d = self
            .template
            .as_ref()
            .and_then(|t| t.view_3d.as_ref())
            .is_some();

        // Ctrl+S — save the current template's UI state in place (v0.8.0).
        let ctrl_s = ui
            .ctx()
            .input(|i| i.modifiers.command && i.key_pressed(egui::Key::S));
        if ctrl_s {
            self.handle_save_in_place();
        }

        // v0.11.0 — Ctrl+Z / Ctrl+Y (and Ctrl+Shift+Z) undo / redo. Captured
        // before the toolbar renders so the click never accidentally triggers
        // an Add Panel modal via the editor sink.
        let (ctrl_z, ctrl_y) = ui.ctx().input(|i| {
            let modifiers = i.modifiers;
            let z = modifiers.command && !modifiers.shift && i.key_pressed(egui::Key::Z);
            let y = (modifiers.command && i.key_pressed(egui::Key::Y))
                || (modifiers.command && modifiers.shift && i.key_pressed(egui::Key::Z));
            (z, y)
        });
        if ctrl_z {
            self.apply_undo();
        } else if ctrl_y {
            self.apply_redo();
        }

        // ── Top toolbar: status + view-mode switch ────────────────────────
        //
        // v0.11.0 — `horizontal_wrapped` lets the toolbar overflow onto a
        // second line when the window is narrow rather than truncating. We
        // also reduce the inter-button spacing so each row fits a few more
        // buttons before wrapping.
        let mut picker_action: Option<TemplateAction> = None;
        // v0.15.0 — Sources toolbar action accumulators. The Sources popup
        // and Add Source dialog mutate `self.registry`, but the borrow
        // checker forbids that inside the `horizontal_wrapped` closure
        // (which already borrows `self`). We collect actions here and apply
        // them after the closure exits.
        let mut source_action: Option<SourceAction> = None;
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 6.0;
            ui.heading(format!("hvn-profiler v{}", env!("CARGO_PKG_VERSION")));
            ui.separator();
            // v0.8.0 — template picker dropdown.
            picker_action = self.render_template_picker(ui);
            ui.separator();
            // v0.9.0 — drone selector. Hidden until ≥2 drones are known
            // (single-drone runs don't need the clutter).
            self.render_drone_selector(ui);
            ui.label(format!("t = {elapsed:7.2} s"));
            ui.separator();
            // v0.15.0 — Sources toolbar button + popup. Replaces the static
            // source description (`self.source_desc`) so the operator can
            // add/remove sources at runtime.
            source_action = self.render_sources_toolbar(ui);
            ui.separator();
            ui.label(format!("samples={}", self.drained_total));

            // The view-mode switch is only meaningful with a 3D block.
            if has_3d {
                ui.separator();
                ui.label("view:");
                ui.selectable_value(&mut self.mode, ViewMode::Grid, "2D grid");
                ui.selectable_value(&mut self.mode, ViewMode::View3d, "3D view");
                ui.selectable_value(&mut self.mode, ViewMode::Split, "Split");
                // v0.11.0 — manual 3D side-panel toggle. Only meaningful in
                // Split mode (in 2D-grid mode there's no 3D pane to hide).
                if self.mode == ViewMode::Split {
                    let icon = if self.split_3d_overlay_open { "3D ◧ on" } else { "3D ◧ off" };
                    if ui
                        .button(icon)
                        .on_hover_text(
                            "Toggle the 3D pane in Split view (lets you reclaim the full window for 2D plots).",
                        )
                        .clicked()
                    {
                        self.split_3d_overlay_open = !self.split_3d_overlay_open;
                    }
                }
            }

            // Labels override (v0.5.0). Always available when a template is loaded.
            if self.template.is_some() {
                ui.separator();
                ui.label("labels:");
                ui.selectable_value(&mut self.label_arg, LabelArg::Template, "template");
                ui.selectable_value(&mut self.label_arg, LabelArg::Off, "off");
                ui.selectable_value(&mut self.label_arg, LabelArg::Data, "data");
                ui.selectable_value(&mut self.label_arg, LabelArg::Metadata, "metadata");
            }

            // v0.10.0 — in-app editor entry points + dirty flag.
            if self.template.is_some() {
                ui.separator();
                if ui.button("+ Add Panel").clicked() {
                    self.refresh_editor_source_keys();
                    // v0.16.0 — default to the first unoccupied (row, col)
                    // instead of always (0, 0). Repeated Add clicks used to
                    // stack new cells on top of an existing (0, 0) panel,
                    // causing overlapping plots to fight for the same rect.
                    let (row, col) = self
                        .template
                        .as_ref()
                        .map(first_available_slot)
                        .unwrap_or((0, 0));
                    self.editor = Some(EditorMode::AddPanel(PanelDraft {
                        row,
                        col,
                        ..PanelDraft::default()
                    }));
                }
                if has_3d && ui.button("+ Add Trail").clicked() {
                    self.refresh_editor_source_keys();
                    self.editor = Some(EditorMode::AddTrail(TrailDraft::default()));
                }
                // v0.11.0 — Undo / Redo (Ctrl+Z / Ctrl+Y).
                let undo_btn = ui.add_enabled(
                    self.history.can_undo(),
                    egui::Button::new("⟲ Undo"),
                );
                if undo_btn
                    .on_hover_text("Undo last edit (Ctrl+Z)")
                    .clicked()
                {
                    self.apply_undo();
                }
                let redo_btn = ui.add_enabled(
                    self.history.can_redo(),
                    egui::Button::new("⟳ Redo"),
                );
                if redo_btn
                    .on_hover_text("Redo last undone edit (Ctrl+Y)")
                    .clicked()
                {
                    self.apply_redo();
                }
                // Hidden-panels button: only visible when at least one cell
                // is currently hidden via the runtime override map.
                let hidden_n = self
                    .cell_visibility_override
                    .values()
                    .filter(|v| !**v)
                    .count();
                if hidden_n > 0
                    && ui
                        .button(format!("Hidden panels ({hidden_n})"))
                        .on_hover_text("Click to restore all hidden panels")
                        .clicked()
                {
                    self.cell_visibility_override.clear();
                    self.template_dirty = true;
                }
                if self.template_dirty {
                    ui.label(egui::RichText::new("●").color(egui::Color32::from_rgb(0xff, 0x99, 0x33)))
                        .on_hover_text("Template has unsaved changes (Ctrl+S to save)");
                }
            }

            // Faults panel toggle (v0.6.0). Only available when --fault-panel on
            // opened the publisher; if it didn't, the button is grayed out with
            // an explanatory tooltip — restarting with the flag is required.
            ui.separator();
            let has_publisher = self.fault_publisher.is_some();
            let label = if self.faults_state.visible {
                "Hide Faults"
            } else {
                "Show Faults"
            };
            let btn = ui.add_enabled(has_publisher, egui::Button::new(label));
            if !has_publisher {
                btn.on_hover_text(
                    "Restart with `--fault-panel on` to enable the Faults panel.",
                );
            } else if btn.clicked() {
                self.faults_state.visible = !self.faults_state.visible;
            }
            if has_publisher {
                ui.label(format!("fault_sent={}", self.fault_sent));
            }

            // v0.7.0 — Signal Generators toggle. Same enable-gate as Faults
            // (the generators feed the Faults publisher).
            let gen_label = if self.gen_state.visible {
                "Hide Generators"
            } else {
                "Show Generators"
            };
            let gbtn = ui.add_enabled(has_publisher, egui::Button::new(gen_label));
            if !has_publisher {
                gbtn.on_hover_text(
                    "Restart with `--generators on` (or `--fault-panel on`) to enable Signal Generators.",
                );
            } else if gbtn.clicked() {
                self.gen_state.visible = !self.gen_state.visible;
            }
            if has_publisher && !self.gen_state.rows.is_empty() {
                let running = self.gen_state.rows.iter().filter(|g| g.running).count();
                ui.label(format!(
                    "gen={}/{}",
                    running,
                    self.gen_state.rows.len()
                ));
            }
        });
        // v0.8.0 — apply any action the picker queued during the toolbar
        // closure (kept outside so we can borrow `self` mutably for the
        // load / save handlers).
        if let Some(action) = picker_action {
            self.handle_template_action(action);
        }
        // v0.15.0 — Sources toolbar action (Add / Remove / open dialog).
        if let Some(action) = source_action {
            self.handle_source_action(action);
        }
        ui.separator();

        // Status-log bookkeeping is mode-specific.
        let mut grid_log: Option<(usize, usize, usize)> = None;
        let mut v3d_log: Option<profiler_render::View3dStats> = None;

        // No template → v0.1.0 single-trace fallback (mode is irrelevant).
        if self.template.is_none() {
            self.render_single_trace(ui);
        } else {
            // No 3D block → always the plain grid.
            let mode = if has_3d { self.mode } else { ViewMode::Grid };
            match mode {
                ViewMode::Grid => {
                    grid_log = Some(self.render_grid(ui));
                }
                ViewMode::View3d => {
                    v3d_log = self.render_3d(ui);
                }
                ViewMode::Split => {
                    // v0.11.0 — responsive: at narrow window widths the
                    // 50/50 split squeezes both views below usable. Below
                    // RESPONSIVE_3D_COLLAPSE_W (1100 px) we fall back to a
                    // grid-only view and render the 3D scene as a floating
                    // overlay window the operator can toggle with the
                    // toolbar's "view: 3D view" button.
                    let avail_w = ui.available_rect_before_wrap().width();
                    if avail_w < profiler_render::RESPONSIVE_3D_COLLAPSE_W {
                        grid_log = Some(self.render_grid(ui));
                        // 3D in a floating window — closeable via the X.
                        // Stored visibility flag is sticky so the operator
                        // re-opens once and it stays.
                        let ctx = ui.ctx().clone();
                        let mut open = self.split_3d_overlay_open;
                        if open {
                            egui::Window::new("3D view")
                                .open(&mut open)
                                .default_size([720.0, 520.0])
                                .default_pos([avail_w * 0.5 - 360.0, 80.0])
                                .resizable(true)
                                .show(&ctx, |ui| {
                                    v3d_log = self.render_3d(ui);
                                });
                        }
                        self.split_3d_overlay_open = open;
                    } else if self.split_3d_overlay_open {
                        // Left/right split via equal columns: 2D grid | 3D view.
                        ui.columns(2, |cols| {
                            grid_log = Some(self.render_grid(&mut cols[0]));
                            v3d_log = self.render_3d(&mut cols[1]);
                        });
                    } else {
                        // Operator chose to hide 3D pane → full-width 2D grid.
                        grid_log = Some(self.render_grid(ui));
                    }
                }
            }
        }

        // ── v0.6.0 Faults panel ─────────────────────────────────────────
        // Rendered as a floating Window so it overlays without disturbing
        // the existing CentralPanel layout. Visibility is toggled by the
        // toolbar button; once open it stays open until closed.
        //
        // v0.7.0: The Signal Generators panel is rendered FIRST so it can
        // tick its running generators into the Faults state before the
        // Faults panel's debounce flush picks them up the same frame.
        if self.fault_publisher.is_some() && self.gen_state.visible {
            let ctx = ui.ctx().clone();
            let now_ms = self.started.elapsed().as_millis() as u64;
            let mut visible = self.gen_state.visible;
            egui::Window::new("Signal Generators")
                .open(&mut visible)
                .default_width(720.0)
                .default_pos([60.0, 480.0])
                .resizable(true)
                .show(&ctx, |ui| {
                    render_gen_panel(
                        ui,
                        &mut self.gen_state,
                        &mut self.faults_state,
                        now_ms,
                    );
                });
            self.gen_state.visible = visible;
        } else if self.fault_publisher.is_some() {
            // Panel hidden but generators may still be running (Pause is
            // per-row, Hide doesn't auto-pause). Continue ticking so the
            // operator's last waveform keeps driving the slider.
            let now_ms = self.started.elapsed().as_millis() as u64;
            self.gen_state.tick_and_apply(now_ms, &mut self.faults_state);
        }

        if self.fault_publisher.is_some() && self.faults_state.visible {
            let ctx = ui.ctx().clone();
            let now_s = ctx.input(|i| i.time);
            self.pending_fault_cmds.clear();
            let mut visible = self.faults_state.visible;
            egui::Window::new("Faults & Interference")
                .open(&mut visible)
                .default_width(340.0)
                .default_pos([900.0, 80.0])
                .resizable(true)
                .show(&ctx, |ui| {
                    render_faults_panel(
                        ui,
                        &mut self.faults_state,
                        &mut self.pending_fault_cmds,
                        now_s,
                    );
                });
            self.faults_state.visible = visible;

            // Forward emitted commands to the publisher.
            if !self.pending_fault_cmds.is_empty() {
                if let Some(pubr) = &self.fault_publisher {
                    for pc in self.pending_fault_cmds.drain(..) {
                        let cmd = FaultCommand {
                            feature: pc.feature,
                            drone: pc.drone,
                            command: pc.label,
                            args: pc.args,
                            reset: pc.reset,
                        };
                        match pubr.send(&cmd) {
                            Ok(()) => self.fault_sent += 1,
                            Err(e) => log::warn!("fault publish failed: {e}"),
                        }
                    }
                }
            }
        } else if self.fault_publisher.is_some() && !self.gen_state.rows.is_empty() {
            // Faults panel closed but generators are running — we still
            // need to drain the dirty bookkeeping into the publisher so
            // generated values don't queue up unsent. Re-invoke the panel
            // headlessly (no Window).
            let ctx = ui.ctx().clone();
            let now_s = ctx.input(|i| i.time);
            self.pending_fault_cmds.clear();
            // Stand-alone Area lets us drive `render_faults_panel` without
            // actually showing it. Cheap (one frame).
            egui::Area::new(egui::Id::new("faults_headless"))
                .interactable(false)
                .fixed_pos([-10000.0, -10000.0])
                .show(&ctx, |ui| {
                    render_faults_panel(
                        ui,
                        &mut self.faults_state,
                        &mut self.pending_fault_cmds,
                        now_s,
                    );
                });
            if !self.pending_fault_cmds.is_empty() {
                if let Some(pubr) = &self.fault_publisher {
                    for pc in self.pending_fault_cmds.drain(..) {
                        let cmd = FaultCommand {
                            feature: pc.feature,
                            drone: pc.drone,
                            command: pc.label,
                            args: pc.args,
                            reset: pc.reset,
                        };
                        match pubr.send(&cmd) {
                            Ok(()) => self.fault_sent += 1,
                            Err(e) => log::warn!("fault publish failed: {e}"),
                        }
                    }
                }
            }
        }

        // ── v0.10.0 — drain per-cell context-menu actions queued by the grid
        // renderer this frame. The renderer pushed entries into
        // `self.pending_cell_actions` while drawing; we apply them here so
        // the next frame sees the mutated template / state.
        if !self.pending_cell_actions.is_empty() {
            let actions: Vec<CellMenuAction> = self.pending_cell_actions.drain(..).collect();
            for action in actions {
                self.apply_cell_menu_action(action);
            }
        }

        // ── v0.10.0 — in-app editor modal (Add Panel / Add Trail / Edit Panel).
        // Driven by the toolbar buttons and the per-cell "Edit panel..." menu;
        // the modal mutates the loaded template in-memory and flips the dirty
        // bit so the toolbar's `●` indicator and Ctrl+S save flow notice.
        if self.editor.is_some() {
            let ctx = ui.ctx().clone();
            self.render_editor_modal(&ctx);
        }

        // ── v0.15.0 — in-app Add Source modal. Open / closed via
        // `self.add_source_uri`; the Connect / Cancel buttons return an
        // action we then apply through the same plumbing as the toolbar.
        if self.add_source_uri.is_some() {
            let ctx = ui.ctx().clone();
            if let Some(action) = self.render_add_source_modal(&ctx) {
                self.handle_source_action(action);
            }
        }

        // ── v0.15.0 — template fallback warning. Re-scanned each frame
        // because the connected source list can change underneath us. We
        // compute the warning AFTER any add/remove from this frame so the
        // toolbar reflects the resolution that's about to happen.
        self.recompute_template_fallback_warning();

        // 1 Hz status log — proof-of-life when running headless. The smoke
        // test greps for `view3d:` (3D modes) and `grid:` (2D / split).
        let now = Instant::now();
        if now.duration_since(self.last_status_log).as_secs_f32() >= 1.0 {
            // v0.9.0: log against the *view* drone's store (multi-drone aware).
            let view_store = self.view_store();
            let drone = self.view_drone.as_deref().unwrap_or("-");
            let drones_known = self.discovered_drones.len();
            if let Some((panels, with_data, keys_with_data)) = grid_log {
                log::info!(
                    "grid: drone={drone} drones_known={drones_known} \
                     panels={panels} panels_with_data={with_data} \
                     keys_with_data={keys_with_data} drained_total={} latest_ts={:.2}",
                    self.drained_total,
                    view_store.latest_ts(),
                );
            }
            if let Some(stats) = &v3d_log {
                log::info!(
                    "view3d: drone={drone} drones_known={drones_known} \
                     trails_visible={} truth_pts={} gps_pts={} ekf_pts={} dr_pts={} \
                     drained_total={} latest_ts={:.2}",
                    stats.trails_visible,
                    stats.pts("truth"),
                    stats.pts("gps"),
                    stats.pts("ekf"),
                    stats.pts("dr"),
                    self.drained_total,
                    view_store.latest_ts(),
                );
            }
            if grid_log.is_none() && v3d_log.is_none() {
                log::info!(
                    "store: drone={drone} drones_known={drones_known} \
                     keys={} drained_total={} latest_ts={:.2}",
                    view_store.keys().len(),
                    self.drained_total,
                    view_store.latest_ts(),
                );
            }
            self.last_status_log = now;
        }
    }
}

impl App {
    /// v0.16.2 — build the URI→drone-name map the renderer uses to resolve
    /// per-cell `source_uri` pins each frame. Reads the live source registry
    /// (which knows each connected URI's most-recently-observed drone name)
    /// and falls back to the URI-derived synthetic name when no envelope has
    /// arrived yet for a source.
    fn build_uri_to_drone(&self) -> HashMap<String, String> {
        // 3.0s freshness cutoff is the same threshold the Sources toolbar
        // uses; we don't actually care about live-ness here, just the
        // currently-known name. Pass it through so `list()` returns the
        // metadata in the same call.
        let entries = self.registry.list(3.0);
        let mut map: HashMap<String, String> = HashMap::with_capacity(entries.len());
        for e in entries {
            // Prefer the envelope-discovered name; fall back to the synthetic
            // URI-derived name when no envelope has arrived yet. This matches
            // the `MultiSource`/`SourceRegistry` fallback path used by the
            // drain loop, so the resolution map agrees with the drone-name
            // bucket the samples actually landed in.
            let drone_name = e
                .drone_name
                .as_deref()
                .map(str::to_string)
                .unwrap_or_else(|| e.fallback_name.to_string());
            map.insert(e.uri, drone_name);
        }
        map
    }

    /// Render the 2D grid against the multi-drone [`StoresView`]. Each cell-
    /// source resolves its own [`TraceStore`] via its `source_uri` pin, falling
    /// back to the view-drone when unpinned.
    /// Returns the grid stats tuple `(panels, panels_with_data, keys_with_data)`
    /// for the status log.
    fn render_grid(&mut self, ui: &mut egui::Ui) -> (usize, usize, usize) {
        let tpl = self.template.take().expect("render_grid called with template");
        // Empty fallback store used when the view-drone has no `TraceStore`
        // yet (first frame, before any sample arrives).
        let empty_store: TraceStore = TraceStore::default();
        let uri_to_drone = self.build_uri_to_drone();
        let view_drone: String = self
            .view_drone
            .as_deref()
            .map(str::to_string)
            .unwrap_or_default();
        let view = profiler_render::StoresView::multi(
            &self.stores,
            &uri_to_drone,
            &view_drone,
            &empty_store,
        );
        // v0.11.0 — stable_dt is the egui-smoothed frame delta; clamped
        // by the renderer to skip stale frames after a window-minimise pause.
        let frame_dt = ui.ctx().input(|i| i.stable_dt);
        let opts = GridRenderOptions {
            panel_states: Some(&mut self.panel_states),
            menu_sink: Some(&mut self.pending_cell_actions),
            visibility_override: Some(&self.cell_visibility_override),
            // v0.10.2 — hidden cells are compacted out of the visible grid by
            // default. The override map still drives WHAT is hidden, but the
            // renderer no longer reserves a blank slot for it.
            compact_hidden: true,
            // v0.11.0 — drag-to-reorder + animated reflow ON by default.
            drag_to_reorder: true,
            animate_reflow: true,
            frame_dt,
        };
        let stats = profiler_render::render_template_grid_multi(
            ui,
            &tpl,
            &view,
            self.label_arg.to_override(),
            opts,
        );
        self.template = Some(tpl);
        (stats.panels, stats.panels_with_data, stats.keys_with_data)
    }

    /// Render the 3D trajectory view + its controls. Returns the per-frame
    /// stats, or `None` if the template has no `view_3d` block.
    fn render_3d(&mut self, ui: &mut egui::Ui) -> Option<profiler_render::View3dStats> {
        let tpl = self.template.take()?;
        let result = tpl.view_3d.as_ref().map(|view| {
            if !self.view3d_inited {
                let (min_w, valinit) = tpl
                    .view_slider
                    .as_ref()
                    .map(|s| (Some(s.min_window_s), Some(s.valinit)))
                    .unwrap_or((None, None));
                self.view3d_state.init_from(view, min_w, valinit);
                self.view3d_inited = true;
            }
            // Read the view-drone's store inline (split borrow with view3d_state).
            let store = match self.view_drone.as_deref().and_then(|d| self.stores.get(d)) {
                Some(s) => s,
                None => {
                    use std::sync::OnceLock;
                    static EMPTY: OnceLock<TraceStore> = OnceLock::new();
                    EMPTY.get_or_init(TraceStore::default)
                }
            };
            render_view3d_with_override(
                ui,
                view,
                store,
                &mut self.view3d_state,
                self.label_arg.to_override(),
            )
        });
        self.template = Some(tpl);
        result
    }

    /// v0.1.0 single-trace fallback (no template given). Reads from the
    /// view-drone's store (or empty default before first sample).
    fn render_single_trace(&mut self, ui: &mut egui::Ui) {
        let key = self.select_key();
        let title = match key.as_deref() {
            Some(k) => format!("trace: {k}"),
            None => "waiting for data…".to_string(),
        };
        ui.label(&title);

        let store = self.view_store();
        let points: Vec<[f64; 2]> = key
            .as_deref()
            .map(|k| store.points(k))
            .unwrap_or_default();
        let count = points.len();

        Plot::new("trace")
            .legend(Legend::default())
            .show_axes([true, true])
            .show_grid([true, true])
            .show(ui, |plot_ui| {
                if !points.is_empty() {
                    let label = key.clone().unwrap_or_else(|| "trace".to_string());
                    plot_ui.line(Line::new(label, PlotPoints::from(points)));
                }
            });

        ui.separator();
        ui.label(format!(
            "store: keys={} points-in-current-trace={}",
            self.view_store().keys().len(),
            count,
        ));
    }
}

// ─── v0.15.0 in-app source management ────────────────────────────────────

/// Action requested by the Sources toolbar dropdown — applied by `App`
/// after the toolbar closure exits (so we can borrow `self.registry` mutably).
#[derive(Debug, Clone)]
enum SourceAction {
    /// Open the `+ Add Source...` modal with an empty URI buffer.
    OpenAddDialog,
    /// Operator clicked `[Connect]` in the dialog with the given URI.
    Connect(String),
    /// Operator cancelled the `+ Add Source...` dialog.
    CancelAdd,
    /// Operator clicked `[×]` next to the given URI.
    Remove(String),
    /// v0.16.0 — operator clicked `[🔄 Rescan]` in the Add-Source dialog.
    /// Triggers a fresh localhost discovery scan without closing the dialog.
    Rescan,
    /// v0.16.0 — operator clicked `[+ Connect]` on a discovered row.
    /// Adds the URI to the registry; dialog stays open so the operator can
    /// pick more rows in succession.
    ConnectDiscovered(String),
}

/// Live-threshold (seconds) for the `●`/`◌` indicator in the Sources popup.
const SOURCES_LIVE_THRESHOLD_S: f64 = 3.0;

/// v0.16.0 — how long the bare-launch auto-discovery toast stays visible
/// before fading. Long enough to read, short enough not to clutter.
const AUTODISCOVER_TOAST_S: f32 = 5.0;

/// v0.16.0 — shared discovery scan state between the egui paint thread and
/// the background tokio task that runs [`profiler_source::discover_localhost_sources`].
///
/// The dialog reads `scanning` to decide whether to draw a spinner, and
/// `results` to render the row list. The kick-off helper sets `scanning =
/// true` synchronously and clears it once the task completes; the
/// `results` slot is replaced wholesale, never appended to.
#[derive(Debug, Default)]
struct DiscoveryState {
    /// `true` while a scan task is in flight. The dialog draws "Scanning…"
    /// while this is set.
    scanning: bool,
    /// Most recent scan result. `None` before the first scan, otherwise
    /// the full Vec from [`profiler_source::discover_localhost_sources`].
    /// Empty Vec means the scan ran but found nothing.
    results: Option<Vec<DiscoveredSource>>,
}

// ─── v0.8.0 template picker + save / save-as plumbing ─────────────────────

/// Action requested by the template picker dropdown — applied by `App` after
/// the toolbar closure finishes (so we can borrow `self` mutably).
enum TemplateAction {
    /// Switch to entry at index in `App.templates`.
    Select(usize),
    /// "Open template file..." — show a file dialog, then load.
    OpenFile,
    /// "Save as..." — show a save dialog, write to chosen path.
    SaveAs,
    /// v0.10.1 — "+ New blank template..." — prompt for a filename, then
    /// bootstrap an empty 1×1 grid the operator populates via "+ Add Panel".
    NewBlank,
}

impl App {
    /// v0.15.0 — render the toolbar's Sources button + connected-source list.
    ///
    /// Always shows `[ Sources (N) ▾ ]` with `N` = number of connected
    /// sources. Clicking the button toggles a popup that lists each source's
    /// `(● live | ◌ stale)` indicator, drone-name (auto-detected from
    /// envelope), URI, and a `[×]` removal button. The popup footer offers
    /// `+ Add source...` which opens the modal dialog.
    ///
    /// Returns the action the operator triggered (or `None` if they only
    /// hovered) so the caller can mutate `self.registry` outside the
    /// borrow-locked closure body.
    fn render_sources_toolbar(&mut self, ui: &mut egui::Ui) -> Option<SourceAction> {
        let mut action: Option<SourceAction> = None;

        let n = self.registry.len();
        let warning_open = self.template_fallback_warning.is_some();
        let last_status = self.last_source_action.clone();

        // v0.15.0 — use a ComboBox-style popup for the Sources dropdown.
        // ComboBox's selected_text shows the count and its `show_ui` body is
        // the popup interior. This dodges the egui 0.34 `popup_below_widget`
        // API churn while still giving us the click-to-open + click-outside-
        // to-close behaviour the spec asks for.
        let btn_label = format!("Sources ({n}) ▾");
        egui::ComboBox::from_id_salt("hvn-profiler-sources-popup")
            .selected_text(btn_label)
            .width(140.0)
            .show_ui(ui, |ui| {
                ui.set_min_width(360.0);
                ui.label(egui::RichText::new("Connected sources").strong());
                ui.separator();
                let entries = self.registry.list(SOURCES_LIVE_THRESHOLD_S);
                if entries.is_empty() {
                    ui.weak("(no sources connected)");
                } else {
                    for e in &entries {
                        ui.horizontal(|ui: &mut egui::Ui| {
                            // Live / stale indicator.
                            let (glyph, color) = if e.is_live {
                                (
                                    "●",
                                    egui::Color32::from_rgb(80, 200, 120),
                                )
                            } else {
                                ("◌", egui::Color32::from_gray(140))
                            };
                            ui.label(
                                egui::RichText::new(glyph).color(color).strong(),
                            );
                            ui.label(&e.uri);
                            ui.weak(
                                e.drone_name
                                    .as_ref()
                                    .map(|s| s.to_string())
                                    .unwrap_or_else(|| {
                                        format!("({})", e.fallback_name)
                                    }),
                            );
                            // Spacer so the [×] button right-aligns.
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui: &mut egui::Ui| {
                                    if ui
                                        .button("×")
                                        .on_hover_text(format!("Remove {}", e.uri))
                                        .clicked()
                                    {
                                        action = Some(SourceAction::Remove(e.uri.clone()));
                                    }
                                },
                            );
                        });
                    }
                }
                ui.separator();
                if ui.button("+ Add source...").clicked() {
                    action = Some(SourceAction::OpenAddDialog);
                }
            });
        // Status message — fades by being overwritten on the next action.
        if let Some(txt) = &last_status {
            ui.weak(txt);
        }
        // v0.16.0 — bare-launch auto-discovery toast. Fades after
        // AUTODISCOVER_TOAST_S so it doesn't clutter the toolbar forever.
        let toast_clear = if let Some((shown_at, txt)) = &self.autodiscover_toast {
            if shown_at.elapsed().as_secs_f32() < AUTODISCOVER_TOAST_S {
                ui.label(
                    egui::RichText::new(format!("✨ {txt}"))
                        .color(egui::Color32::from_rgb(120, 200, 240)),
                );
                false
            } else {
                true
            }
        } else {
            false
        };
        if toast_clear {
            self.autodiscover_toast = None;
        }
        if warning_open {
            if let Some(w) = &self.template_fallback_warning {
                ui.label(
                    egui::RichText::new(format!("⚠ {w}"))
                        .color(egui::Color32::from_rgb(240, 180, 60)),
                )
                .on_hover_text("A template cell pinned to a source URI that isn't connected; rendering from the first available source.");
            }
        }

        action
    }

    /// v0.15.0 — scan the loaded template for cell sources that declare a
    /// `source_uri` not matching any currently-connected source. When at
    /// least one such cell exists, set `template_fallback_warning` so the
    /// toolbar paints a `⚠` next to the Sources button.
    ///
    /// Called every frame after source mutations are applied, so the
    /// indicator stays in sync as the operator adds / removes sources.
    fn recompute_template_fallback_warning(&mut self) {
        let connected: Vec<String> = self.registry.uris();
        let mut missing: Vec<String> = Vec::new();
        if let Some(tpl) = &self.template {
            for cell in &tpl.cells {
                for src in &cell.sources {
                    if let Some(uri) = &src.source_uri {
                        if !connected.iter().any(|c| c == uri)
                            && !missing.iter().any(|m| m == uri)
                        {
                            missing.push(uri.clone());
                        }
                    }
                }
            }
        }
        self.template_fallback_warning = if missing.is_empty() {
            None
        } else if missing.len() == 1 {
            Some(format!(
                "Template references source `{}` which isn't connected — using first available.",
                missing[0],
            ))
        } else {
            Some(format!(
                "Template references {} sources that aren't connected — using first available.",
                missing.len(),
            ))
        };
    }

    /// v0.15.0 — apply a `SourceAction` collected by the toolbar closure.
    /// Mutates `self.registry` and surfaces status messages.
    fn handle_source_action(&mut self, action: SourceAction) {
        match action {
            SourceAction::OpenAddDialog => {
                self.add_source_uri = Some(String::new());
                // v0.16.0 — kick off the localhost discovery scan in the
                // background so the dialog has results to show by the time
                // the operator is done reading the header.
                self.kick_discovery_scan();
            }
            SourceAction::CancelAdd => {
                self.add_source_uri = None;
            }
            SourceAction::Rescan => {
                self.kick_discovery_scan();
            }
            SourceAction::ConnectDiscovered(uri) => {
                // Same path as a manual Connect, but DON'T close the dialog:
                // the operator may want to click [+ Connect] on several rows
                // back-to-back without re-opening the modal.
                let res = if uri.starts_with("zmq://") {
                    self.registry.add_zmq(&uri)
                } else {
                    self.registry.add(&uri)
                };
                match res {
                    Ok(()) => {
                        self.last_source_action = Some(format!("Connected to {uri}"));
                        self.source_desc = self.registry.describe();
                    }
                    Err(e) => {
                        self.last_source_action = Some(format!("Connect failed: {e}"));
                    }
                }
            }
            SourceAction::Connect(uri) => {
                let uri = uri.trim().to_string();
                self.add_source_uri = None;
                if uri.is_empty() {
                    self.last_source_action =
                        Some("Connect failed: URI must not be empty".into());
                    return;
                }
                let res = if uri.starts_with("zmq://") {
                    self.registry.add_zmq(&uri)
                } else {
                    self.registry.add(&uri)
                };
                match res {
                    Ok(()) => {
                        self.last_source_action = Some(format!("Connected to {uri}"));
                        // Refresh source_desc so the title-bar / status row
                        // reflects the new set.
                        self.source_desc = self.registry.describe();
                    }
                    Err(e) => {
                        self.last_source_action =
                            Some(format!("Connect failed: {e}"));
                    }
                }
            }
            SourceAction::Remove(uri) => {
                let removed = self.registry.remove(&uri);
                if removed {
                    self.last_source_action = Some(format!("Removed {uri}"));
                } else {
                    self.last_source_action =
                        Some(format!("Remove failed: {uri} not in list"));
                }
                self.source_desc = self.registry.describe();
            }
        }
    }

    /// v0.16.0 — kick off a background localhost discovery scan. Snapshots
    /// the connected-source list under the registry's lock-free read, then
    /// spawns the scan on the shared tokio runtime. The result lands in
    /// `self.discovery_state` once the task completes.
    ///
    /// Idempotent: if a scan is already in flight, this is a no-op (the
    /// in-flight task will refresh the slot when it finishes). The
    /// `[🔄 Rescan]` button still calls this each click; the second call
    /// during an in-flight scan just no-ops and the operator's existing
    /// scan completes normally.
    fn kick_discovery_scan(&mut self) {
        // No-op when a scan is already pending.
        if let Ok(g) = self.discovery_state.lock() {
            if g.scanning {
                return;
            }
        }
        let connected = self.registry.uris();
        let state = std::sync::Arc::clone(&self.discovery_state);
        // Mark scanning BEFORE spawning so the dialog draws "Scanning..."
        // immediately on the very next frame.
        if let Ok(mut g) = state.lock() {
            g.scanning = true;
        }
        let rt = std::sync::Arc::clone(&self.discovery_runtime);
        rt.spawn(async move {
            let results = profiler_source::discover_localhost_sources(
                &connected,
                DEFAULT_PROBE_MS,
            )
            .await;
            if let Ok(mut g) = state.lock() {
                g.results = Some(results);
                g.scanning = false;
            }
        });
    }

    /// v0.16.0 — render the `+ Add Source...` modal dialog. Driven by
    /// `self.add_source_uri`: `None` means the dialog is closed; `Some(buf)`
    /// keeps it open and tracks the in-progress URI text.
    ///
    /// v0.16.0 adds a "Detected on localhost:" section above the URI input
    /// that lists every source the background discovery scan found. The
    /// scan kicks off automatically when the dialog opens; the `[🔄 Rescan]`
    /// button refreshes the list in place.
    fn render_add_source_modal(&mut self, ctx: &egui::Context) -> Option<SourceAction> {
        let buf_owned = self.add_source_uri.clone()?;
        let mut buf = buf_owned;
        let mut connect_clicked = false;
        let mut cancel_clicked = false;
        let mut rescan_clicked = false;
        let mut discovered_connect: Option<String> = None;
        // Snapshot the discovery slot under the lock so we don't hold it
        // through the egui closure (which may panic-on-poison).
        let (scanning, results_snapshot): (bool, Option<Vec<DiscoveredSource>>) =
            match self.discovery_state.lock() {
                Ok(g) => (g.scanning, g.results.clone()),
                Err(_) => (false, None),
            };
        // Pre-compute the set of URIs already in the registry so we can
        // grey out their [+ Connect] buttons even when the result entry
        // was Live / Silent (e.g. operator clicked Connect, the entry got
        // added, but the scan hasn't been re-run yet).
        let connected_uris: std::collections::HashSet<String> =
            self.registry.uris().into_iter().collect();

        egui::Window::new("+ Add Source")
            .collapsible(false)
            .resizable(false)
            .default_width(460.0)
            .show(ctx, |ui| {
                // ─ Header: "Detected on localhost:" + Rescan button ──────
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("Detected on localhost:").strong());
                    ui.with_layout(
                        egui::Layout::right_to_left(egui::Align::Center),
                        |ui| {
                            if ui
                                .button("🔄 Rescan")
                                .on_hover_text(
                                    "Re-run the localhost discovery scan",
                                )
                                .clicked()
                            {
                                rescan_clicked = true;
                            }
                        },
                    );
                });

                // ─ Discovery list ────────────────────────────────────────
                if scanning && results_snapshot.is_none() {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.weak("Scanning…");
                    });
                } else if let Some(rows) = &results_snapshot {
                    if rows.is_empty() {
                        ui.weak("(none detected; enter a custom URI below)");
                    } else {
                        // v0.16.0 — re-render the spinner inline next to
                        // the previous results when a fresh scan is in
                        // flight, so the operator can still click rows
                        // from the prior result while the next one cooks.
                        if scanning {
                            ui.horizontal(|ui| {
                                ui.spinner();
                                ui.weak("Re-scanning…");
                            });
                        }
                        for d in rows {
                            let is_connected = connected_uris.contains(&d.uri)
                                || matches!(d.status, DiscoveryStatus::InUse);
                            ui.horizontal(|ui| {
                                let (glyph, color) = match &d.status {
                                    DiscoveryStatus::Live { .. } => (
                                        "●",
                                        egui::Color32::from_rgb(80, 200, 120),
                                    ),
                                    DiscoveryStatus::Silent => (
                                        "◌",
                                        egui::Color32::from_gray(140),
                                    ),
                                    DiscoveryStatus::InUse => (
                                        "✓",
                                        egui::Color32::from_rgb(100, 160, 220),
                                    ),
                                };
                                if is_connected
                                    && !matches!(d.status, DiscoveryStatus::InUse)
                                {
                                    // The scan said Live/Silent but the
                                    // operator already added the URI in
                                    // this session — re-tag as ✓.
                                    ui.label(
                                        egui::RichText::new("✓")
                                            .color(egui::Color32::from_rgb(100, 160, 220))
                                            .strong(),
                                    );
                                } else {
                                    ui.label(
                                        egui::RichText::new(glyph).color(color).strong(),
                                    );
                                }
                                ui.label(&d.uri);
                                // Drone name / status descriptor.
                                let descriptor = match &d.status {
                                    DiscoveryStatus::Live { drone_name } => {
                                        drone_name
                                            .clone()
                                            .unwrap_or_else(|| "(live)".to_string())
                                    }
                                    DiscoveryStatus::Silent => "(silent)".into(),
                                    DiscoveryStatus::InUse => "(already connected)".into(),
                                };
                                ui.weak(descriptor);
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        let btn_label = if is_connected {
                                            "✓ Connected"
                                        } else {
                                            "+ Connect"
                                        };
                                        let btn = ui.add_enabled(
                                            !is_connected,
                                            egui::Button::new(btn_label),
                                        );
                                        if btn.clicked() {
                                            discovered_connect = Some(d.uri.clone());
                                            // Pre-fill the URI box too for
                                            // visual feedback.
                                            buf = d.uri.clone();
                                        }
                                    },
                                );
                            });
                        }
                        // Per-kind summary (helps the operator notice when
                        // their MAVLink port range is exotic).
                        let live_n = rows
                            .iter()
                            .filter(|r| matches!(r.status, DiscoveryStatus::Live { .. }))
                            .count();
                        let silent_n = rows
                            .iter()
                            .filter(|r| matches!(r.status, DiscoveryStatus::Silent))
                            .count();
                        let inuse_n = rows
                            .iter()
                            .filter(|r| matches!(r.status, DiscoveryStatus::InUse))
                            .count();
                        ui.weak(format!(
                            "{live_n} live / {silent_n} silent / {inuse_n} connected",
                        ));
                    }
                } else {
                    ui.weak("(scan pending)");
                }

                ui.separator();
                ui.label(egui::RichText::new("Or enter a custom URI:").strong());
                egui::Grid::new("add_source_grid")
                    .num_columns(2)
                    .spacing([8.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("URI:");
                        ui.text_edit_singleline(&mut buf);
                        ui.end_row();
                        ui.label("Type:");
                        let scheme_hint = if buf.starts_with("zmq://") {
                            "ZMQ (HVN-SITL streamer)"
                        } else if buf.starts_with("mavlink://") {
                            "MAVLink UDP (udpin / listen)"
                        } else if buf.starts_with("mavlinkout://") {
                            "MAVLink UDP (udpout / send-first)"
                        } else if buf.starts_with("mock://") {
                            "Synthetic (sine wave)"
                        } else if buf.is_empty() {
                            "(auto-detected from URI scheme)"
                        } else {
                            "(unrecognised scheme — will default to mock://)"
                        };
                        ui.weak(scheme_hint);
                        ui.end_row();
                    });
                ui.weak("zmq:// or mavlink:// or mock://");
                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("Cancel").clicked() {
                        cancel_clicked = true;
                    }
                    if ui.button("Connect").clicked() {
                        connect_clicked = true;
                    }
                });
            });
        // Persist the in-progress text back into self so future frames see it.
        self.add_source_uri = Some(buf.clone());
        if let Some(uri) = discovered_connect {
            return Some(SourceAction::ConnectDiscovered(uri));
        }
        if rescan_clicked {
            return Some(SourceAction::Rescan);
        }
        if connect_clicked {
            return Some(SourceAction::Connect(buf));
        }
        if cancel_clicked {
            return Some(SourceAction::CancelAdd);
        }
        None
    }

    /// Render the toolbar's template picker. Returns the user's choice, if
    /// any — applied after the toolbar closure exits.
    fn render_template_picker(&mut self, ui: &mut egui::Ui) -> Option<TemplateAction> {
        let mut action: Option<TemplateAction> = None;
        // Selected label for the button face.
        let current_label = self
            .current_template
            .and_then(|i| self.templates.get(i))
            .map(|e| {
                format!("Template: {} ({})", e.name, e.origin_label())
            })
            .unwrap_or_else(|| "Template: (none)".to_string());

        egui::ComboBox::from_id_salt("hvn-profiler-template-picker")
            .selected_text(current_label)
            .show_ui(ui, |ui| {
                for (i, entry) in self.templates.iter().enumerate() {
                    let label = format!("{} ({})", entry.name, entry.origin_label());
                    let selected = Some(i) == self.current_template;
                    if ui.selectable_label(selected, label).clicked() {
                        action = Some(TemplateAction::Select(i));
                    }
                }
                ui.separator();
                if ui.button("📁 Open template file…").clicked() {
                    action = Some(TemplateAction::OpenFile);
                }
                // v0.10.1 — bootstrap an empty template the user populates
                // via "+ Add Panel".
                if ui.button("✨ + New blank template…").clicked() {
                    action = Some(TemplateAction::NewBlank);
                }
                // The Save button is enabled only when a savable entry is
                // selected. Bundled entries pop "Save as..." instead.
                let is_savable = self
                    .current_template
                    .and_then(|i| self.templates.get(i))
                    .map(|e| e.origin.is_savable_in_place())
                    .unwrap_or(false);
                if is_savable && ui.button("💾 Save (Ctrl+S)").clicked() {
                    // Tunnel through the same picker_action plumbing by
                    // reusing the existing "in-place save" code path —
                    // close the menu then run save.
                    self.handle_save_in_place();
                }
                if ui.button("💾 Save as…").clicked() {
                    action = Some(TemplateAction::SaveAs);
                }
            });

        if let Some(txt) = &self.last_template_action {
            ui.weak(txt);
        }

        action
    }

    /// v0.9.0 — render the toolbar's drone selector dropdown.
    ///
    /// The dropdown only appears once ≥2 drones have been discovered; while
    /// only one is known we don't clutter the toolbar. The list is populated
    /// from `App.discovered_drones` (insertion-ordered), and the active
    /// selection sits on `App.view_drone`.
    ///
    /// This is INDEPENDENT of the Faults panel's "Target" dropdown: the user
    /// can watch drone A while injecting faults on drone B.
    fn render_drone_selector(&mut self, ui: &mut egui::Ui) {
        if self.discovered_drones.len() < 2 {
            return;
        }
        let current = self
            .view_drone
            .clone()
            .unwrap_or_else(|| self.discovered_drones[0].clone());
        ui.label("Drone:");
        egui::ComboBox::from_id_salt("hvn-profiler-drone-selector")
            .selected_text(&current)
            .show_ui(ui, |ui| {
                for name in &self.discovered_drones {
                    let selected = Some(name) == self.view_drone.as_ref();
                    if ui.selectable_label(selected, name).clicked() {
                        self.view_drone = Some(name.clone());
                        // Reset 3D camera fit so the new drone's trail is centred.
                        self.view3d_inited = false;
                    }
                }
            });
        ui.separator();
    }

    fn handle_template_action(&mut self, action: TemplateAction) {
        match action {
            TemplateAction::Select(i) => self.load_template_at(i),
            TemplateAction::OpenFile => self.open_template_dialog(),
            TemplateAction::SaveAs => self.save_as_dialog(),
            TemplateAction::NewBlank => self.new_blank_template_dialog(),
        }
    }

    /// Switch the active template to `templates[i]`, preserving the store
    /// (so live data keeps flowing), the discovered-drone roster, and the
    /// operator's `view_drone` selection (when that drone is still known),
    /// plus toolbar state (labels, fault panel, generators). Resets only
    /// the bits that depend on the template itself: the view3d init flag
    /// (so the 3D camera re-fits to the new trail set) and the picked index.
    ///
    /// v0.10.0 — explicit `view_drone` capture-and-restore: prior to this we
    /// relied on the implicit invariant that nothing here touches the field;
    /// the explicit step pins the contract and is what the
    /// `view_drone_persist_test` integration test asserts.
    fn load_template_at(&mut self, i: usize) {
        let captured_view_drone = self.view_drone.clone();
        let entry = match self.templates.get(i).cloned() {
            Some(e) => e,
            None => return,
        };
        let json = match load_entry_json(&entry) {
            Ok(s) => s,
            Err(e) => {
                let msg = format!("Open failed: {e}");
                log::warn!("{msg}");
                self.last_template_action = Some(msg);
                return;
            }
        };
        let mut tpl = match Template::from_str(&json) {
            Ok(t) => t,
            Err(e) => {
                let msg = format!("Parse failed: {e}");
                log::warn!("{msg}");
                self.last_template_action = Some(msg);
                return;
            }
        };
        // Apply any persisted UI state.
        if let Some(ui) = tpl.ui_state.clone() {
            tpl.apply_ui_state(&ui);
        }
        log::info!(
            "switched to template '{}' ({})",
            tpl.name,
            entry.origin_label()
        );
        self.last_template_action =
            Some(format!("Loaded '{}' ({})", tpl.name, entry.origin_label()));
        // Default view mode: Split if the template carries a 3D block.
        let has_3d = tpl.view_3d.is_some();
        self.mode = if has_3d { ViewMode::Split } else { ViewMode::Grid };
        self.template = Some(tpl);
        self.view3d_inited = false;
        self.current_template = Some(i);
        // v0.10.0 — restore the captured view-drone selection if that drone
        // is still known. If it's gone (the operator unplugged it between
        // reloads), fall back to the first-seen drone so the renderer
        // always has a valid target.
        self.view_drone = match captured_view_drone {
            Some(d) if self.stores.contains_key(&d) => Some(d),
            _ => self.discovered_drones.first().cloned(),
        };
    }

    fn open_template_dialog(&mut self) {
        let res = rfd::FileDialog::new()
            .add_filter("HVN profiler template", &["json"])
            .set_directory(profiler_template::user_templates_dir())
            .pick_file();
        let Some(path) = res else { return };
        // Insert (or reuse) an entry pointing at this path.
        match load_path_into_templates(&mut self.templates, &path) {
            Ok(i) => self.load_template_at(i),
            Err(e) => {
                let msg = format!("Open failed: {e}");
                log::warn!("{msg}");
                self.last_template_action = Some(msg);
            }
        }
    }

    /// v0.10.1 — "+ New blank template…": prompt for a filename in the user
    /// templates directory, then bootstrap an in-memory `Template::blank()`.
    /// The file is NOT written here — Ctrl+S writes the JSON once the
    /// operator has added at least one panel. Cancelling the dialog is a
    /// no-op (no state mutation, no status-bar message).
    ///
    /// After load:
    /// - The new template is registered as a `Cli`-origin entry so the
    ///   picker selects it (in-place Save targets the chosen path).
    /// - `template_dirty` is set to `true` so the toolbar shows `●` even
    ///   before the first edit, signalling "this template has not been
    ///   written to disk yet."
    fn new_blank_template_dialog(&mut self) {
        let dir = profiler_template::user_templates_dir();
        let res = rfd::FileDialog::new()
            .add_filter("HVN profiler template", &["json"])
            .set_directory(&dir)
            .set_file_name("untitled.json")
            .save_file();
        let Some(path) = res else { return };

        let stem = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("untitled")
            .to_string();
        let tpl = Template::blank(stem);

        // Append a fresh Cli-origin entry pointing at the chosen path. We do
        // not re-discover here so the operator's choice survives even if the
        // path sits outside the user-templates dir.
        let entry = TemplateEntry {
            name: tpl.name.clone(),
            origin: TemplateOrigin::Cli { path: path.clone() },
        };
        self.templates.push(entry);
        let idx = self.templates.len() - 1;

        // Default 2D-grid view (blank template has no 3D block).
        self.mode = ViewMode::Grid;
        self.template = Some(tpl);
        self.current_template = Some(idx);
        self.view3d_inited = false;
        self.template_dirty = true;
        self.last_template_action = Some(format!(
            "New blank template (save to {})",
            path.display(),
        ));
        log::info!("created blank template at {}", path.display());
    }

    fn save_as_dialog(&mut self) {
        let res = rfd::FileDialog::new()
            .add_filter("HVN profiler template", &["json"])
            .set_directory(profiler_template::user_templates_dir())
            .set_file_name("my-template.json")
            .save_file();
        let Some(path) = res else { return };
        self.write_current_template_to(&path);
    }

    /// Ctrl+S — overwrite the currently-loaded user/CLI template. Bundled
    /// templates trigger a Save-as instead.
    fn handle_save_in_place(&mut self) {
        let entry_path: Option<std::path::PathBuf> = self
            .current_template
            .and_then(|i| self.templates.get(i))
            .and_then(|e| e.origin.path().map(|p| p.to_path_buf()));
        match entry_path {
            Some(p) => self.write_current_template_to(&p),
            None => self.save_as_dialog(),
        }
    }

    fn write_current_template_to(&mut self, path: &std::path::Path) {
        let mut tpl = match self.template.clone() {
            Some(t) => t,
            None => {
                self.last_template_action =
                    Some("Save failed: no template loaded".into());
                return;
            }
        };
        tpl.ui_state = Some(self.capture_ui_state(&tpl));
        let json = match tpl.to_pretty_json() {
            Ok(s) => s,
            Err(e) => {
                let msg = format!("Save failed: {e}");
                log::warn!("{msg}");
                self.last_template_action = Some(msg);
                return;
            }
        };
        if let Some(parent) = path.parent() {
            if !parent.exists() {
                let _ = std::fs::create_dir_all(parent);
            }
        }
        if let Err(e) = std::fs::write(path, json.as_bytes()) {
            let msg = format!("Save failed: {e}");
            log::warn!("{msg}");
            self.last_template_action = Some(msg);
            return;
        }
        log::info!("saved template to {}", path.display());
        self.last_template_action = Some(format!("Saved to {}", path.display()));
        // Refresh discovery so the new file (if any) shows up.
        let cli_path_dummy: Option<&std::path::Path> = None;
        let mut new_list = profiler_template::discover(cli_path_dummy);
        // Pin to the just-saved path so the dropdown highlights it.
        let abs = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
        let mut idx = None;
        for (i, e) in new_list.iter().enumerate() {
            if let Some(p) = e.origin.path() {
                if std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf()) == abs {
                    idx = Some(i);
                    break;
                }
            }
        }
        // If we couldn't find it (path outside the user dir), insert as Cli.
        if idx.is_none() {
            new_list.push(TemplateEntry {
                name: tpl.name.clone(),
                origin: TemplateOrigin::Cli { path: path.to_path_buf() },
            });
            idx = Some(new_list.len() - 1);
        }
        self.templates = new_list;
        self.current_template = idx;
    }

    /// Snapshot the live UI state into a [`UiState`].
    fn capture_ui_state(&self, tpl: &Template) -> UiState {
        let mut ui = UiState::default();
        // Cell visibility — persist any cell whose current `visible` flag
        // overrides the original JSON (in v0.8.0 we don't expose runtime
        // visibility toggles outside save-as yet, so this is a faithful copy
        // of the loaded template's state).
        for cell in &tpl.cells {
            let key = format!("{},{}", cell.row, cell.col);
            ui.cell_visibility.insert(key.clone(), cell.visible);
            ui.cell_label_mode.insert(key, cell.label_mode);
        }
        // If the global label override is forcing a mode, stamp it onto
        // every entry (round-trip the "I want metadata everywhere" flag).
        if let profiler_render::LabelOverride::Force(mode) = self.label_arg.to_override() {
            for v in ui.cell_label_mode.values_mut() {
                *v = mode;
            }
        }
        // 3D trail visibility + trail length (only if view3d state was
        // touched; otherwise the snapshot omits trail keys).
        if tpl.view_3d.is_some() {
            for (k, v) in &self.view3d_state.visible {
                ui.trail_visibility.insert(k.clone(), *v);
            }
            ui.trail_frac = Some(self.view3d_state.trail_frac);
            ui.view_frac = Some(self.view3d_state.view_frac);
        }
        ui
    }
}

// ─── v0.10.0 — in-app editor + per-cell context menu plumbing ────────────────

impl App {
    /// v0.10.0 — repopulate the source-key dropdown shown by the "+ Add Panel"
    /// modal. Walks every per-drone [`TraceStore`] in `self.stores`, collects
    /// the union of observed keys (with vector-base shorthand inserted so the
    /// operator can pick `ap_attitude` for an `AttitudeRpy` panel without
    /// remembering it expands to `ap_attitude[0..2]`), sorts + dedupes, and
    /// stores the result in `editor_source_keys_cache`.
    ///
    /// Called whenever the operator opens the editor (toolbar "+ Add Panel"
    /// / "+ Add Trail" or per-cell "Edit panel...") so the dropdown reflects
    /// the keys observed *up to that moment* — a drone that started streaming
    /// after a previous open will still have its keys in the list.
    fn refresh_editor_source_keys(&mut self) {
        self.editor_source_keys_cache = collect_source_keys(self.stores.values());
    }

    /// v0.10.0 — apply one [`CellMenuAction`] emitted by the per-cell
    /// right-click menu. Called once per action drained at the end of each
    /// frame so the next frame sees the mutated state.
    ///
    /// - `Edit` → open the editor pre-filled from the cell.
    /// - `HideToggle` → flip `cell_visibility_override[(r,c)]`.
    /// - `ResetZoom` → clear `panel_states[(r,c)].locked` so auto-scale-Y resumes.
    /// - `SetLabelMode` → mutate the cell's own `label_mode` (template dirties).
    /// - `Delete` → drop the cell from the template (template dirties).
    fn apply_cell_menu_action(&mut self, action: CellMenuAction) {
        match action {
            CellMenuAction::Edit { row, col } => {
                // Pre-fill the editor draft from the existing cell at (row,col).
                let cell_opt = self
                    .template
                    .as_ref()
                    .and_then(|t| t.cells.iter().find(|c| c.row == row && c.col == col).cloned());
                let draft = match cell_opt {
                    Some(cell) => panel_draft_from_cell(&cell),
                    None => PanelDraft { row, col, ..Default::default() },
                };
                self.refresh_editor_source_keys();
                self.editor = Some(EditorMode::EditPanel { row, col, draft });
            }
            CellMenuAction::HideToggle { row, col } => {
                self.record_history();
                let cur = self
                    .cell_visibility_override
                    .get(&(row, col))
                    .copied()
                    .unwrap_or(true);
                self.cell_visibility_override.insert((row, col), !cur);
                self.template_dirty = true;
            }
            CellMenuAction::ResetZoom { row, col } => {
                if let Some(st) = self.panel_states.get_mut(&(row, col)) {
                    st.locked = false;
                }
            }
            CellMenuAction::SetLabelMode { row, col, mode } => {
                self.record_history();
                self.cell_label_override.insert((row, col), mode);
                // Also mutate the template's own cell so the renderer
                // (which honours `cell.label_mode` when LabelOverride::Respect
                // is in effect) picks it up immediately.
                if let Some(tpl) = self.template.as_mut() {
                    for cell in tpl.cells.iter_mut() {
                        if cell.row == row && cell.col == col {
                            cell.label_mode = mode;
                        }
                    }
                }
                self.template_dirty = true;
            }
            CellMenuAction::Delete { row, col } => {
                self.record_history();
                if let Some(tpl) = self.template.as_mut() {
                    if remove_cell_at(tpl, row, col).is_ok() {
                        // v0.10.2 — reflow the remaining cells so the grid
                        // stays tightly packed. Visual ordering (top-to-bottom,
                        // left-to-right) is preserved by `compact_cells`.
                        compact_cells(tpl);
                    }
                    self.template_dirty = true;
                }
            }
            // v0.11.0 — drag-to-reorder: swap occupied target.
            CellMenuAction::SwapTo { from, to } => {
                self.record_history();
                if let Some(tpl) = self.template.as_mut() {
                    if swap_cells(tpl, from, to).is_ok() {
                        compact_cells(tpl);
                        self.template_dirty = true;
                        self.last_template_action = Some(format!(
                            "Swapped ({}, {}) ↔ ({}, {})",
                            from.0, from.1, to.0, to.1,
                        ));
                    }
                }
            }
            // v0.11.0 — drag-to-reorder: drop onto an empty slot.
            CellMenuAction::MoveTo { from, to } => {
                self.record_history();
                if let Some(tpl) = self.template.as_mut() {
                    if relocate_cell(tpl, from, to).is_ok() {
                        compact_cells(tpl);
                        self.template_dirty = true;
                        self.last_template_action = Some(format!(
                            "Moved ({}, {}) → ({}, {})",
                            from.0, from.1, to.0, to.1,
                        ));
                    }
                }
            }
        }
    }

    /// v0.11.0 — snapshot the current template into the undo history before
    /// any editor mutation. No-op when no template is loaded.
    fn record_history(&mut self) {
        if let Some(tpl) = self.template.as_ref() {
            self.history.record(tpl.clone());
        }
    }

    /// v0.11.0 — apply an undo. Swaps the current template with the most
    /// recent past snapshot; the displaced state goes on the redo stack.
    /// Marks dirty so the operator notices the change wasn't saved.
    fn apply_undo(&mut self) {
        let Some(current) = self.template.take() else {
            return;
        };
        match self.history.undo(current.clone()) {
            Some(prev) => {
                self.template = Some(prev);
                self.template_dirty = true;
                self.last_template_action = Some("Undo".into());
            }
            None => {
                self.template = Some(current);
            }
        }
    }

    /// v0.11.0 — apply a redo. Symmetric inverse of [`Self::apply_undo`].
    fn apply_redo(&mut self) {
        let Some(current) = self.template.take() else {
            return;
        };
        match self.history.redo(current.clone()) {
            Some(next) => {
                self.template = Some(next);
                self.template_dirty = true;
                self.last_template_action = Some("Redo".into());
            }
            None => {
                self.template = Some(current);
            }
        }
    }

    /// v0.10.0 — render the open editor modal (Add Panel / Edit Panel /
    /// Add Trail). The modal lives in its own `egui::Window` so it overlays
    /// the grid without disturbing the central layout. On Apply / Add, the
    /// modal mutates the in-memory template and sets `template_dirty = true`;
    /// on Cancel / close, the editor is dropped without mutation.
    fn render_editor_modal(&mut self, ctx: &egui::Context) {
        // Take ownership so we can mutate without borrowing `self.editor`
        // across the closure body.
        let Some(mode) = self.editor.take() else {
            return;
        };

        // The modal's open-state is driven entirely by `next`: error branches
        // re-emit the editor mode so the modal stays open with the failure
        // message in `last_template_action`; success branches let it drop.
        let mut next: Option<EditorMode> = None;

        // v0.15.0 — snapshot the connected source list ONCE per modal frame
        // so the closure body doesn't need to borrow `self.registry`.
        let connected_sources: Vec<SourceListEntry> =
            self.registry.list(SOURCES_LIVE_THRESHOLD_S);

        match mode {
            EditorMode::AddPanel(mut draft) => {
                let mut commit = false;
                let mut cancel = false;
                // v0.12.0 — build the picker context for freshness coloring
                // + type filter row. `observed_keys` is materialised once
                // per modal render from the live stores so the classifier
                // can disambiguate SchemaOnly vs Custom.
                let observed_keys: std::collections::HashSet<String> = self
                    .stores
                    .values()
                    .flat_map(|s| {
                        s.keys()
                            .into_iter()
                            .chain(s.null_keys().iter().cloned())
                    })
                    .collect();
                let now_s = self.started.elapsed().as_secs_f64();
                let source_keys = &self.editor_source_keys_cache;
                let collapse = &mut self.editor_combo_collapse;
                let last_seen = &self.last_seen_keys;
                let filter = &mut self.picker_filter;
                let sources_ref: &[SourceListEntry] = &connected_sources;
                // v0.16.8 — snapshot the discovered-drone list so the
                // drone-pin dropdown in `panel_form` can offer the same
                // entries the toolbar picker shows.
                let drones_ref: &[String] = &self.discovered_drones;
                egui::Window::new("+ Add Panel")
                    .collapsible(false)
                    .resizable(true)
                    .default_width(360.0)
                    .show(ctx, |ui| {
                        let mut pctx = PickerContext {
                            last_seen,
                            now_s,
                            filter,
                            observed: &observed_keys,
                        };
                        panel_form(
                            ui,
                            &mut draft,
                            source_keys,
                            collapse,
                            Some(&mut pctx),
                            Some(sources_ref),
                            Some(drones_ref),
                        );
                        ui.separator();
                        ui.horizontal(|ui| {
                            if ui.button("Add").clicked() {
                                commit = true;
                            }
                            if ui.button("Cancel").clicked() {
                                cancel = true;
                            }
                        });
                    });
                if commit {
                    // v0.11.0 — snapshot pre-mutation for undo. Done before
                    // the apply so a failed apply still leaves history clean
                    // (we re-emit the editor below on Err).
                    self.record_history();
                    if let Some(tpl) = self.template.as_mut() {
                        match add_panel_draft(tpl, &draft) {
                            Ok(()) => {
                                self.template_dirty = true;
                                self.last_template_action = Some(format!(
                                    "Added cell at ({}, {})", draft.row, draft.col,
                                ));
                            }
                            Err(e) => {
                                // Drop the snapshot we just took — apply was
                                // a no-op, no undo entry needed.
                                let _ = self.history.undo(tpl.clone());
                                self.last_template_action = Some(format!("Add failed: {e}"));
                                // Keep the modal open so operator can fix.
                                next = Some(EditorMode::AddPanel(draft));
                            }
                        }
                    }
                } else if cancel {
                    // closed
                } else {
                    next = Some(EditorMode::AddPanel(draft));
                }
            }
            EditorMode::EditPanel { row, col, mut draft } => {
                let mut commit = false;
                let mut cancel = false;
                let observed_keys: std::collections::HashSet<String> = self
                    .stores
                    .values()
                    .flat_map(|s| {
                        s.keys()
                            .into_iter()
                            .chain(s.null_keys().iter().cloned())
                    })
                    .collect();
                let now_s = self.started.elapsed().as_secs_f64();
                let source_keys = &self.editor_source_keys_cache;
                let collapse = &mut self.editor_combo_collapse;
                let last_seen = &self.last_seen_keys;
                let filter = &mut self.picker_filter;
                let sources_ref: &[SourceListEntry] = &connected_sources;
                // v0.16.8 — see AddPanel above; same drone-pin dropdown plumb.
                let drones_ref: &[String] = &self.discovered_drones;
                egui::Window::new(format!("Edit panel ({row}, {col})"))
                    .collapsible(false)
                    .resizable(true)
                    .default_width(360.0)
                    .show(ctx, |ui| {
                        let mut pctx = PickerContext {
                            last_seen,
                            now_s,
                            filter,
                            observed: &observed_keys,
                        };
                        panel_form(
                            ui,
                            &mut draft,
                            source_keys,
                            collapse,
                            Some(&mut pctx),
                            Some(sources_ref),
                            Some(drones_ref),
                        );
                        ui.separator();
                        ui.horizontal(|ui| {
                            if ui.button("Apply").clicked() {
                                commit = true;
                            }
                            if ui.button("Cancel").clicked() {
                                cancel = true;
                            }
                        });
                    });
                if commit {
                    self.record_history();
                    if let Some(tpl) = self.template.as_mut() {
                        match replace_cell_at(tpl, row, col, &draft) {
                            Ok(()) => {
                                self.template_dirty = true;
                                // v0.10.1 — if the operator changed Row/Col in
                                // the form, the cell has relocated; surface
                                // both the source and destination so it's
                                // obvious from the status bar.
                                let msg = if (draft.row, draft.col) == (row, col) {
                                    format!("Updated cell at ({row}, {col})")
                                } else {
                                    format!(
                                        "Moved cell ({row}, {col}) → ({}, {})",
                                        draft.row, draft.col,
                                    )
                                };
                                self.last_template_action = Some(msg);
                            }
                            Err(e) => {
                                self.last_template_action = Some(format!("Apply failed: {e}"));
                                next = Some(EditorMode::EditPanel { row, col, draft });
                            }
                        }
                    }
                } else if cancel {
                    // closed
                } else {
                    next = Some(EditorMode::EditPanel { row, col, draft });
                }
            }
            EditorMode::AddTrail(mut draft) => {
                let mut commit = false;
                let mut cancel = false;
                let source_keys = &self.editor_source_keys_cache;
                let collapse = &mut self.editor_combo_collapse;
                egui::Window::new("+ Add Trail (3D)")
                    .collapsible(false)
                    .resizable(true)
                    .default_width(360.0)
                    .show(ctx, |ui| {
                        trail_form(ui, &mut draft, source_keys, collapse);
                        ui.separator();
                        ui.horizontal(|ui| {
                            if ui.button("Add").clicked() {
                                commit = true;
                            }
                            if ui.button("Cancel").clicked() {
                                cancel = true;
                            }
                        });
                    });
                if commit {
                    self.record_history();
                    if let Some(tpl) = self.template.as_mut() {
                        match apply_trail_draft(tpl, &draft) {
                            Ok(()) => {
                                self.template_dirty = true;
                                self.last_template_action = Some(format!(
                                    "Added trail '{}'", draft.name,
                                ));
                                self.view3d_inited = false;
                            }
                            Err(e) => {
                                self.last_template_action = Some(format!("Add failed: {e}"));
                                next = Some(EditorMode::AddTrail(draft));
                            }
                        }
                    }
                } else if cancel {
                    // closed
                } else {
                    next = Some(EditorMode::AddTrail(draft));
                }
            }
        }

        // Reinstate the editor if the operator didn't commit/cancel this
        // frame. Error paths re-emit the mode via `next` so the modal stays
        // open with the failure message in `last_template_action`.
        self.editor = next;
    }
}

/// Build a [`PanelDraft`] from an existing cell — used by the "Edit panel..."
/// flow so the modal opens pre-filled with the current values.
fn panel_draft_from_cell(cell: &profiler_template::Cell) -> PanelDraft {
    let (source_key, fallback, minus, color, overlay_extra) = match cell.sources.first() {
        Some(src) => (
            src.key.clone(),
            src.fallback.clone().unwrap_or_default(),
            src.minus.clone().unwrap_or_default(),
            if !src.color.is_empty() {
                src.color.clone()
            } else {
                cell.color.clone().unwrap_or_else(|| "#1f77b4".into())
            },
            cell.sources.iter().skip(1).map(|s| s.key.clone()).collect(),
        ),
        None => (
            String::new(),
            String::new(),
            String::new(),
            cell.color.clone().unwrap_or_else(|| "#1f77b4".into()),
            Vec::new(),
        ),
    };
    // v0.13.0 — for Status cells the canonical source key lives in
    // `cell.source`, not the first `cell.sources` entry; mirror it back
    // into the draft so the editor opens pre-filled with the right key.
    let source_key = if cell.primitive == profiler_template::Primitive::Status
        && !cell.source.is_empty()
    {
        cell.source.clone()
    } else {
        source_key
    };
    let status_color_map: Vec<(String, String)> = cell
        .color_map
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();
    // v0.15.0 — preserve the cell's source_uri pin (if any) into the draft.
    let source_uri = cell
        .sources
        .first()
        .and_then(|s| s.source_uri.clone())
        .unwrap_or_default();
    // v0.16.8 — preserve the cell's drone-pin (if any) into the draft.
    let source_drone = cell
        .sources
        .first()
        .and_then(|s| s.source_drone.clone())
        .unwrap_or_default();
    PanelDraft {
        row: cell.row,
        col: cell.col,
        primitive: cell.primitive,
        title: cell.title.clone(),
        source_key,
        source_uri,
        source_drone,
        fallback,
        minus,
        color,
        label_mode: cell.label_mode,
        overlay_extra_keys: overlay_extra,
        status_kind: cell.kind.unwrap_or_default(),
        status_color_map,
        status_default_color: cell
            .default_color
            .clone()
            .unwrap_or_else(|| "#aaaaaa".to_string()),
    }
}

/// Shared form widget for both Add Panel and Edit Panel modals — mutates a
/// [`PanelDraft`] in place. Renders one labelled row per field.
///
/// v0.12.0 — `picker` carries the freshness registry + type filter so the
/// source-key dropdown can colorize entries and skip filtered classes. When
/// the operator picks a source key whose [`ValueShape`] is known and the
/// existing primitive default matches `Scalar`, the form auto-selects the
/// inferred primitive (the user can still change it via the dropdown).
///
/// v0.15.0 — `connected_sources` carries the list of currently-connected
/// source URIs (with their drone names) so the form can render a source
/// dropdown ABOVE the key dropdown. The operator picks `(any)` (the empty
/// default) to defer source binding to render time, or a specific URI to pin
/// the cell to one source. Passing `None` hides the source row entirely
/// (used by tests and the legacy single-source flow).
fn panel_form(
    ui: &mut egui::Ui,
    draft: &mut PanelDraft,
    source_keys: &[String],
    collapse: &mut profiler_render::ComboCollapseState,
    picker: Option<&mut PickerContext<'_>>,
    connected_sources: Option<&[SourceListEntry]>,
    // v0.16.8 — list of every drone NAME the CLI has discovered so far
    // (insertion-ordered, matches the toolbar picker). `None` hides the
    // "Drone" dropdown entirely; `Some(&[])` shows the dropdown with only
    // `(any)` available (no discovered drones yet).
    discovered_drones: Option<&[String]>,
) {
    use profiler_template::{LabelMode, Primitive};

    egui::Grid::new("panel_form_grid")
        .num_columns(2)
        .spacing([8.0, 4.0])
        .show(ui, |ui| {
            ui.label("Row:");
            ui.add(egui::DragValue::new(&mut draft.row).speed(1.0).range(0..=64));
            ui.end_row();
            ui.label("Col:");
            ui.add(egui::DragValue::new(&mut draft.col).speed(1.0).range(0..=64));
            ui.end_row();

            ui.label("Title:");
            ui.text_edit_singleline(&mut draft.title);
            ui.end_row();

            ui.label("Primitive:");
            egui::ComboBox::from_id_salt("panel_form_primitive")
                .selected_text(format!("{:?}", draft.primitive))
                .show_ui(ui, |ui| {
                    for p in [
                        Primitive::Scalar,
                        Primitive::Vector,
                        Primitive::Overlay,
                        Primitive::Magnitude,
                        Primitive::Diff,
                        Primitive::MagInterference,
                        Primitive::AttitudeRpy,
                        Primitive::Status,
                        Primitive::InfoText,
                    ] {
                        ui.selectable_value(&mut draft.primitive, p, format!("{p:?}"));
                    }
                });
            ui.end_row();

            // v0.16.8 — drone dropdown row appears ABOVE both the source-key
            // row and the source-URI row when the caller supplies a list of
            // discovered drones. Drone-pin wins over URI-pin at render time
            // (see `StoresView::for_source`), so the Drone row is shown first
            // to match its precedence in the resolver. Empty
            // `draft.source_drone` (`""`) renders as `(Any)`; a non-empty
            // value pins the cell to that drone NAME (stable across runs).
            //
            // Why a dropdown and not free-text: a typo in a drone name would
            // silently route to the view-drone (the operator's "(Any)" path)
            // with no warning. Limiting selection to `discovered_drones`
            // keeps the pin honest.
            if let Some(drones) = discovered_drones {
                ui.label("Drone:");
                egui::ComboBox::from_id_salt("panel_form_source_drone")
                    .selected_text(if draft.source_drone.is_empty() {
                        "(Any)".to_string()
                    } else {
                        draft.source_drone.clone()
                    })
                    .show_ui(ui, |ui| {
                        // `(Any)` first — defer to view-drone at render time.
                        let selected = draft.source_drone.is_empty();
                        if ui
                            .selectable_label(selected, "(Any)")
                            .on_hover_text("Follow the toolbar's view-drone selection")
                            .clicked()
                        {
                            draft.source_drone.clear();
                        }
                        for name in drones {
                            let selected = draft.source_drone == *name;
                            if ui.selectable_label(selected, name).clicked() {
                                draft.source_drone = name.clone();
                            }
                        }
                    });
                ui.end_row();
            }

            // v0.15.0 — source dropdown row appears ABOVE the source-key
            // row when the caller supplies a list of connected sources.
            // Empty `draft.source_uri` (`""`) renders as `(any)`; a
            // non-empty value pins the cell to that URI.
            //
            // v0.16.8 — the Drone row above takes precedence at render time.
            // This URI row is kept for backward compat with v0.15.0 templates
            // and for the legacy single-drone case where URI-pin still works.
            if let Some(sources) = connected_sources {
                ui.label("Source:");
                egui::ComboBox::from_id_salt("panel_form_source_uri")
                    .selected_text(format_source_combo_label(&draft.source_uri, sources))
                    .show_ui(ui, |ui| {
                        // `(any)` first — defer binding to render time.
                        let selected = draft.source_uri.is_empty();
                        if ui
                            .selectable_label(selected, "(any)")
                            .on_hover_text("Use the first connected source at render time")
                            .clicked()
                        {
                            draft.source_uri.clear();
                        }
                        for entry in sources {
                            let label = format_source_combo_entry(entry);
                            let selected = draft.source_uri == entry.uri;
                            if ui.selectable_label(selected, label).clicked() {
                                draft.source_uri = entry.uri.clone();
                            }
                        }
                    });
                ui.end_row();
            }

            ui.label("Source key:");
            // v0.12.0 — capture the source key BEFORE the combo runs so we
            // can detect a change and auto-infer the primitive.
            let prev_key = draft.source_key.clone();
            source_key_combo(
                ui,
                "panel_form_src",
                &mut draft.source_key,
                source_keys,
                collapse,
                picker,
            );
            if draft.source_key != prev_key && !draft.source_key.is_empty() {
                if let Some(shape) = profiler_render::known_value_shape(&draft.source_key) {
                    let inferred = profiler_render::infer_primitive(&shape);
                    // Only auto-overwrite when the current primitive is the
                    // dropdown default (Scalar) — operator-picked primitives
                    // are respected.
                    if draft.primitive == Primitive::Scalar {
                        draft.primitive = match inferred {
                            "vector" => Primitive::Vector,
                            "status" => Primitive::Status,
                            _ => Primitive::Scalar,
                        };
                    }
                    // v0.13.0 — when we just landed on Status (either by
                    // auto-inference or because the operator pre-selected
                    // it), pick a sensible kind from the key name + shape.
                    if draft.primitive == Primitive::Status {
                        if let Some(kind) = profiler_render::default_status_kind(
                            &draft.source_key,
                            &shape,
                        ) {
                            draft.status_kind = kind;
                        }
                    }
                }
            }
            ui.end_row();

            ui.label("Fallback:");
            ui.text_edit_singleline(&mut draft.fallback);
            ui.end_row();

            if draft.primitive == Primitive::Diff {
                ui.label("Minus key:");
                ui.text_edit_singleline(&mut draft.minus);
                ui.end_row();
            }

            ui.label("Color:");
            ui.text_edit_singleline(&mut draft.color);
            ui.end_row();

            ui.label("Label mode:");
            egui::ComboBox::from_id_salt("panel_form_label_mode")
                .selected_text(format!("{:?}", draft.label_mode))
                .show_ui(ui, |ui| {
                    for m in [LabelMode::Off, LabelMode::Data, LabelMode::Metadata] {
                        ui.selectable_value(&mut draft.label_mode, m, format!("{m:?}"));
                    }
                });
            ui.end_row();
        });

    // Overlay-only: editable list of extra source keys (one per line).
    if draft.primitive == profiler_template::Primitive::Overlay {
        ui.separator();
        ui.label("Overlay extra keys (one per line):");
        let mut joined = draft.overlay_extra_keys.join("\n");
        if ui
            .add(egui::TextEdit::multiline(&mut joined).desired_rows(3))
            .changed()
        {
            draft.overlay_extra_keys = joined
                .lines()
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect();
        }
    }

    // v0.13.0 — Status-only: kind selector, default color, and the
    // `color_map` row editor.
    if draft.primitive == Primitive::Status {
        use profiler_template::StatusKind;
        ui.separator();
        egui::Grid::new("panel_form_status_grid")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("Kind:");
                egui::ComboBox::from_id_salt("panel_form_status_kind")
                    .selected_text(format!("{:?}", draft.status_kind))
                    .show_ui(ui, |ui| {
                        for k in [
                            StatusKind::Text,
                            StatusKind::Badge,
                            StatusKind::FixType,
                            StatusKind::ArmedBool,
                            StatusKind::TextLog,
                            StatusKind::EkfFlags,
                        ] {
                            ui.selectable_value(
                                &mut draft.status_kind,
                                k,
                                format!("{k:?}"),
                            );
                        }
                    });
                ui.end_row();

                ui.label("Default col:");
                ui.text_edit_singleline(&mut draft.status_default_color);
                ui.end_row();
            });

        // `armed_bool` and `fix_type` are preset; collapse the editor to
        // a read-only hint so the operator isn't tempted to author a
        // custom map (which would silently override the renderer's
        // built-in semantics).
        let preset = matches!(
            draft.status_kind,
            StatusKind::ArmedBool | StatusKind::FixType
        );
        ui.separator();
        if preset {
            ui.label(format!(
                "Color map: (preset — `{:?}` uses fixed colors)",
                draft.status_kind,
            ));
        } else {
            ui.label("Color map:");
            let mut remove_idx: Option<usize> = None;
            // Render rows. Each is `[text edit | text edit | × button]`.
            for (i, (k, v)) in draft.status_color_map.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    ui.add_sized(
                        [120.0, 20.0],
                        egui::TextEdit::singleline(k).hint_text("value"),
                    );
                    ui.add_sized(
                        [90.0, 20.0],
                        egui::TextEdit::singleline(v).hint_text("#rrggbb"),
                    );
                    if ui
                        .button("×")
                        .on_hover_text("Remove this row")
                        .clicked()
                    {
                        remove_idx = Some(i);
                    }
                });
            }
            if let Some(idx) = remove_idx {
                draft.status_color_map.remove(idx);
            }
            if ui.button("+ Add row").clicked() {
                draft
                    .status_color_map
                    .push((String::new(), "#1f77b4".to_string()));
            }
        }
    }
}

/// v0.15.0 — format the selected source-URI as the dropdown's button face.
fn format_source_combo_label(uri: &str, sources: &[SourceListEntry]) -> String {
    if uri.is_empty() {
        return "(any)".to_string();
    }
    for e in sources {
        if e.uri == uri {
            return format_source_combo_entry(e);
        }
    }
    // URI doesn't match any connected source — surface it as a missing pin
    // so the operator notices.
    format!("{uri} (not connected)")
}

/// v0.15.0 — render one entry in the source-URI dropdown.
fn format_source_combo_entry(e: &SourceListEntry) -> String {
    let name = e
        .drone_name
        .as_ref()
        .map(|s| s.to_string())
        .unwrap_or_else(|| format!("({})", e.fallback_name));
    let live = if e.is_live { "●" } else { "◌" };
    format!("{live} {} — {name}", e.uri)
}

/// Form widget for the Add Trail modal.
fn trail_form(
    ui: &mut egui::Ui,
    draft: &mut TrailDraft,
    source_keys: &[String],
    collapse: &mut profiler_render::ComboCollapseState,
) {
    egui::Grid::new("trail_form_grid")
        .num_columns(2)
        .spacing([8.0, 4.0])
        .show(ui, |ui| {
            ui.label("Name:");
            ui.text_edit_singleline(&mut draft.name);
            ui.end_row();

            ui.label("Label:");
            ui.text_edit_singleline(&mut draft.label);
            ui.end_row();

            ui.label("Color:");
            ui.text_edit_singleline(&mut draft.color);
            ui.end_row();

            ui.label("Dead-reckon?");
            ui.checkbox(&mut draft.use_deadreckon, "synthesise from accel + quat");
            ui.end_row();
        });

    ui.separator();
    if draft.use_deadreckon {
        egui::Grid::new("trail_form_dr_grid")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("Accel base:");
                source_key_combo(ui, "trail_form_accel", &mut draft.accel_key, source_keys, collapse, None);
                ui.end_row();
                ui.label("Quat base:");
                source_key_combo(ui, "trail_form_quat", &mut draft.quat_key, source_keys, collapse, None);
                ui.end_row();
                ui.label("Seed-from base:");
                source_key_combo(ui, "trail_form_seed", &mut draft.seed_key, source_keys, collapse, None);
                ui.end_row();
            });
    } else {
        egui::Grid::new("trail_form_direct_grid")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("X (East) key:");
                source_key_combo(ui, "trail_form_x", &mut draft.x_key, source_keys, collapse, None);
                ui.end_row();
                ui.label("Y (North) key:");
                source_key_combo(ui, "trail_form_y", &mut draft.y_key, source_keys, collapse, None);
                ui.end_row();
                ui.label("Z (NED-down) key:");
                source_key_combo(ui, "trail_form_z", &mut draft.z_neg_key, source_keys, collapse, None);
                ui.end_row();
            });
    }
}

/// Free-form text input + dropdown to pick from observed source keys.
///
/// v0.10.2 — the dropdown is grouped by category (DT physics, AP MAVLink,
/// Position (NED), Timing, Other). Within a group, keys keep the alphabetical
/// order returned by `collect_source_keys`.
///
/// v0.11.0 — the per-category header is a MANUAL ▶/▼ toggle (rather than
/// `egui::CollapsingHeader`), because the latter's click interaction escaped
/// the `ComboBox` popup-rect tracking and dismissed the entire popup on
/// every collapse / expand. The toggle now only flips
/// `category_collapsed[category]` in the editor state — the popup stays open
/// and the operator can keep browsing. Collapsed state is held by the caller
/// via [`profiler_render::ComboCollapseState`] so it persists across re-opens
/// of the SAME modal.
/// v0.12.0 — optional picker context for [`source_key_combo`].
///
/// Carries the freshness registry + type filter + observed-key set so the
/// dropdown can colorize entries and skip filtered classes. Passed by
/// reference so each modal opens with the operator's current filter row
/// state.
struct PickerContext<'a> {
    last_seen: &'a HashMap<String, f64>,
    now_s: f64,
    filter: &'a mut profiler_render::PickerTypeFilter,
    /// Set of keys that have ever been observed in any store (includes
    /// schema-only null keys). Used to disambiguate `SchemaOnly` vs
    /// `Custom` for the freshness classifier.
    observed: &'a std::collections::HashSet<String>,
}

fn source_key_combo(
    ui: &mut egui::Ui,
    salt: &str,
    value: &mut String,
    source_keys: &[String],
    collapse: &mut profiler_render::ComboCollapseState,
    mut ctx: Option<&mut PickerContext<'_>>,
) {
    use profiler_render::{classify_key, KeyFreshness};
    ui.horizontal(|ui| {
        ui.text_edit_singleline(value);
        egui::ComboBox::from_id_salt(salt)
            .selected_text("▼")
            .width(20.0)
            .show_ui(ui, |ui| {
                // v0.12.0 — filter row at the top of the popup. Operator can
                // toggle off classes they don't want to see. The toggles
                // stay inside the ComboBox popup-rect tracking (plain
                // checkboxes, same trick as the category headers).
                if let Some(ref mut pc) = ctx.as_ref() {
                    // Render once via a re-borrow that doesn't consume ctx.
                    let _ = pc; // satisfy borrow checker; we use the real one below
                }
                if let Some(pc) = ctx.as_deref_mut() {
                    ui.horizontal_wrapped(|ui| {
                        ui.label(egui::RichText::new("Show:").weak());
                        ui.checkbox(&mut pc.filter.status, "Status");
                        ui.checkbox(&mut pc.filter.scalar_2d, "2D scalar");
                        ui.checkbox(&mut pc.filter.vector_2d, "2D vector");
                        ui.checkbox(&mut pc.filter.d3, "3D");
                    });
                    ui.separator();
                }
                // Cap at 256 entries so a noisy run doesn't lock up the UI.
                // We slice BEFORE grouping so the cap applies uniformly.
                // v0.12.0 — apply the type filter row BEFORE the cap so the
                // dropdown can still show 256 of the unfiltered classes
                // when the operator has narrowed the view.
                let filtered: Vec<String> = match ctx.as_deref() {
                    Some(pc) => source_keys
                        .iter()
                        .filter(|k| {
                            let shape = profiler_render::known_value_shape(k);
                            pc.filter.allows(k, shape)
                        })
                        .cloned()
                        .collect(),
                    None => source_keys.to_vec(),
                };
                let limited: Vec<String> = filtered.into_iter().take(256).collect();
                let grouped = group_source_keys(&limited);
                for (group, keys) in grouped {
                    let collapsed = collapse.is_collapsed(group);
                    // Manual toggle row: label `▶ Category` / `▼ Category`.
                    // We deliberately render only a `Label` (no Button, no
                    // CollapsingHeader) so the click stays inside the
                    // ComboBox's popup-rect interaction tracking and does
                    // NOT dismiss the popup.
                    let icon = if collapsed { "▶" } else { "▼" };
                    let header = ui.add(
                        egui::Label::new(
                            egui::RichText::new(format!("{icon}  {group}")).strong(),
                        )
                        .sense(egui::Sense::click()),
                    );
                    if header.clicked() {
                        collapse.toggle(group);
                    }
                    if !collapsed {
                        ui.indent(format!("{salt}_{group}_body"), |ui| {
                            for k in &keys {
                                // v0.12.0 — colorize by freshness when a
                                // picker context is supplied.
                                let label_text = match ctx.as_deref() {
                                    Some(pc) => {
                                        let observed = pc.observed.contains(k);
                                        let fresh = classify_key(k, pc.last_seen, pc.now_s, observed);
                                        let mut rt = egui::RichText::new(k);
                                        rt = match fresh {
                                            KeyFreshness::Live => {
                                                rt.color(egui::Color32::from_gray(220))
                                            }
                                            KeyFreshness::Stale => {
                                                rt.color(egui::Color32::from_gray(140))
                                            }
                                            KeyFreshness::SchemaOnly => rt
                                                .color(egui::Color32::from_gray(120))
                                                .italics(),
                                            KeyFreshness::Custom => rt.color(
                                                egui::Color32::from_rgb(220, 180, 100),
                                            ),
                                        };
                                        rt
                                    }
                                    None => egui::RichText::new(k),
                                };
                                let selected = value == k;
                                if ui.selectable_label(selected, label_text).clicked() {
                                    *value = k.clone();
                                }
                            }
                        });
                    }
                }
            });
    });
}

/// Insert (or find-and-reuse) a `TemplateEntry` for `path` in `templates`,
/// returning its index. Used by the "Open template file..." dialog.
fn load_path_into_templates(
    templates: &mut Vec<TemplateEntry>,
    path: &std::path::Path,
) -> std::io::Result<usize> {
    let abs = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
    for (i, e) in templates.iter().enumerate() {
        if let Some(p) = e.origin.path() {
            if std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf()) == abs {
                return Ok(i);
            }
        }
    }
    let text = std::fs::read_to_string(path)?;
    let name = serde_json::from_str::<serde_json::Value>(&text)
        .ok()
        .and_then(|v| v["name"].as_str().map(str::to_string))
        .unwrap_or_else(|| {
            path.file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("opened")
                .to_string()
        });
    templates.push(TemplateEntry {
        name,
        origin: TemplateOrigin::Cli { path: path.to_path_buf() },
    });
    Ok(templates.len() - 1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    /// v0.9.0 — the GLOBAL `--labels` default is `off` so launching the
    /// profiler with no flag shows clean panels. Templates can still opt
    /// cells in via `label_mode` by picking `--labels template`.
    #[test]
    fn cli_labels_default_is_off() {
        // Parse with no arguments — `clap` applies the `default_value_t`.
        let cli = Cli::parse_from(["hvn-profiler"]);
        assert_eq!(cli.labels, LabelArg::Off, "v0.9.0: --labels defaults to Off");
        // `LabelArg::default()` is also `Off` (used by the toolbar selector).
        assert_eq!(LabelArg::default(), LabelArg::Off);
        // The resolved override forces every cell to `Off`, suppressing any
        // per-cell `label_mode` that the JSON template asked for.
        assert_eq!(
            LabelArg::default().to_override(),
            LabelOverride::Force(LabelMode::Off),
        );
    }

    #[test]
    fn cli_labels_template_still_available() {
        // Explicit opt-in still honours per-cell modes.
        let cli = Cli::parse_from(["hvn-profiler", "--labels", "template"]);
        assert_eq!(cli.labels, LabelArg::Template);
        assert_eq!(cli.labels.to_override(), LabelOverride::Respect);
    }
}
