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
use profiler_render::{
    render_faults_panel, render_gen_panel, render_template_grid_with_override,
    render_view3d_with_override, FaultsPanelState, GeneratorPanelState, LabelOverride,
    PendingCommand, SeenDrones, TraceStore, View3dState,
};
use profiler_source::{
    multi_from_uris_with_discovery_opts, FaultCommand, FaultPublisher, MavlinkConfig, Source,
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
    #[arg(long, default_values_t = vec!["mock://".to_string()])]
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
        cli.source,
        cli.template,
        cli.labels,
        cli.fault_panel,
        cli.generators,
        cli.drone,
    );

    let mav_cfg = MavlinkConfig {
        passive: matches!(cli.mavlink_passive, MavlinkPassiveArg::On),
    };
    // v0.9.0 — multi-source fan-in. Single-URI is the fast path
    // (preserves v0.8.0 behaviour bit-for-bit); >1 URI wraps every leg into
    // a MultiSource with a merged SeenDrones set.
    let (source, seen_drones) = multi_from_uris_with_discovery_opts(&cli.source, mav_cfg)?;
    let source_desc = source.describe();

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
            log::info!("no --template given → single-trace fallback mode");
            None
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
    let sources_summary = if cli.source.len() == 1 {
        cli.source[0].clone()
    } else {
        format!("{} sources: {}", cli.source.len(), cli.source.join(" + "))
    };
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
                source,
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

struct App {
    source: Box<dyn Source>,
    source_desc: String,
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
}

impl App {
    #[allow(clippy::too_many_arguments)]
    fn new(
        source: Box<dyn Source>,
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
        Self {
            source,
            source_desc,
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
            match self.source.try_recv() {
                Some(s) => {
                    let drone_key = s
                        .drone_name
                        .clone()
                        .unwrap_or_else(|| UNNAMED_DRONE.to_string());
                    let is_new = !self.stores.contains_key(&drone_key);
                    let store = self
                        .stores
                        .entry(drone_key.clone())
                        .or_default();
                    store.push(s.ts, &s.key, s.value);
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

        // ── Top toolbar: status + view-mode switch ────────────────────────
        let mut picker_action: Option<TemplateAction> = None;
        ui.horizontal(|ui| {
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
            ui.label(&self.source_desc);
            ui.separator();
            ui.label(format!("samples={}", self.drained_total));

            // The view-mode switch is only meaningful with a 3D block.
            if has_3d {
                ui.separator();
                ui.label("view:");
                ui.selectable_value(&mut self.mode, ViewMode::Grid, "2D grid");
                ui.selectable_value(&mut self.mode, ViewMode::View3d, "3D view");
                ui.selectable_value(&mut self.mode, ViewMode::Split, "Split");
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
                    // Left/right split via equal columns: 2D grid | 3D view.
                    ui.columns(2, |cols| {
                        grid_log = Some(self.render_grid(&mut cols[0]));
                        v3d_log = self.render_3d(&mut cols[1]);
                    });
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
    /// Render the 2D grid against the currently-selected drone's store.
    /// Returns the grid stats tuple `(panels, panels_with_data, keys_with_data)`
    /// for the status log.
    fn render_grid(&mut self, ui: &mut egui::Ui) -> (usize, usize, usize) {
        let tpl = self.template.take().expect("render_grid called with template");
        let store = self.view_store();
        let stats = render_template_grid_with_override(
            ui,
            &tpl,
            store,
            self.label_arg.to_override(),
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
}

impl App {
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
        }
    }

    /// Switch the active template to `templates[i]`, preserving the store
    /// (so live data keeps flowing) and toolbar state (labels, fault panel,
    /// generators). Resets only the bits that depend on the template
    /// itself: the view3d init flag (so the 3D camera re-fits to the new
    /// trail set) and the picked index.
    fn load_template_at(&mut self, i: usize) {
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
