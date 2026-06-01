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
use profiler_source::{from_uri_with_discovery, FaultCommand, FaultPublisher, Source};
use profiler_template::{LabelMode, Template};

/// Max samples drained from the source per render frame. Caps wall-clock
/// time spent in `update` when the ZMQ backend is wildly ahead.
const MAX_DRAIN_PER_FRAME: usize = 5_000;

/// Preferred key to render when present in the store.
const PREFERRED_KEY: &str = "accel[0]";

/// HVN profiler — GPU-accelerated telemetry viewer.
#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
struct Cli {
    /// Telemetry source URI.
    ///
    /// Supported:
    /// - `mock://`               — synthetic sine wave (default)
    /// - `zmq://host:port`       — subscribe to the SITL msgpack streamer
    /// - `mavlink://host:port` — direct MAVLink UDP, bind/listen (udpin)
    /// - `mavlinkout://host:port` — direct MAVLink UDP, send-first (udpout)
    #[arg(long, default_value = "mock://")]
    source: String,

    /// Path to a JSON template describing the panel grid.
    ///
    /// When given, renders the multi-panel 2D layout. When omitted, falls back
    /// to v0.1.0 single-trace mode.
    #[arg(long)]
    template: Option<String>,

    /// Global per-panel label overlay mode override (v0.5.0).
    ///
    /// `template` (default) honours each cell's own `label_mode` from the JSON
    /// template. `off`/`data`/`metadata` force every cell into that mode at
    /// startup; the toolbar selector can flip it at runtime.
    #[arg(long, value_enum, default_value_t = LabelArg::Template)]
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
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum LabelArg {
    /// Honour each cell's own `label_mode` from the template.
    Template,
    /// Force every cell to draw no overlay.
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
        "hvn-profiler v{} starting (source={}, template={:?}, labels={:?}, fault_panel={:?}, generators={:?}, drone={:?})",
        env!("CARGO_PKG_VERSION"),
        cli.source,
        cli.template,
        cli.labels,
        cli.fault_panel,
        cli.generators,
        cli.drone,
    );

    let (source, seen_drones) = from_uri_with_discovery(&cli.source)?;
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

    let title = match &template {
        Some(t) => format!(
            "hvn-profiler v{} — {} — {}",
            env!("CARGO_PKG_VERSION"),
            t.name,
            cli.source
        ),
        None => format!(
            "hvn-profiler v{} — {}",
            env!("CARGO_PKG_VERSION"),
            cli.source
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

struct App {
    source: Box<dyn Source>,
    source_desc: String,
    store: TraceStore,
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
            store: TraceStore::default(),
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
        }
    }

    /// Drain the source, push into the store. Caps work per frame.
    fn drain(&mut self) {
        let mut n = 0;
        while n < MAX_DRAIN_PER_FRAME {
            match self.source.try_recv() {
                Some(s) => {
                    self.store.push(s.ts, &s.key, s.value);
                    n += 1;
                }
                None => break,
            }
        }
        self.drained_total += n as u64;
    }

    /// Choose which trace to render this frame.
    fn select_key(&self) -> Option<String> {
        if self.store.len(PREFERRED_KEY) > 0 {
            Some(PREFERRED_KEY.to_string())
        } else {
            self.store.busiest_key()
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

        // ── Top toolbar: status + view-mode switch ────────────────────────
        ui.horizontal(|ui| {
            ui.heading(format!("hvn-profiler v{}", env!("CARGO_PKG_VERSION")));
            ui.separator();
            if let Some(t) = &self.template {
                ui.label(format!("template: {}", t.name));
                ui.separator();
            }
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
            if let Some((panels, with_data, keys_with_data)) = grid_log {
                log::info!(
                    "grid: panels={panels} panels_with_data={with_data} \
                     keys_with_data={keys_with_data} drained_total={} latest_ts={:.2}",
                    self.drained_total,
                    self.store.latest_ts(),
                );
            }
            if let Some(stats) = &v3d_log {
                log::info!(
                    "view3d: trails_visible={} truth_pts={} gps_pts={} ekf_pts={} dr_pts={} \
                     drained_total={} latest_ts={:.2}",
                    stats.trails_visible,
                    stats.pts("truth"),
                    stats.pts("gps"),
                    stats.pts("ekf"),
                    stats.pts("dr"),
                    self.drained_total,
                    self.store.latest_ts(),
                );
            }
            if grid_log.is_none() && v3d_log.is_none() {
                log::info!(
                    "store: keys={} drained_total={} latest_ts={:.2}",
                    self.store.keys().len(),
                    self.drained_total,
                    self.store.latest_ts(),
                );
            }
            self.last_status_log = now;
        }
    }
}

impl App {
    /// Render the 2D grid (unchanged from v0.2.0). Returns the grid stats
    /// tuple `(panels, panels_with_data, keys_with_data)` for the status log.
    fn render_grid(&mut self, ui: &mut egui::Ui) -> (usize, usize, usize) {
        // `take` to avoid borrowing `self` immutably while `render_template_grid`
        // also borrows `self.store` — same pattern as v0.2.0.
        let tpl = self.template.take().expect("render_grid called with template");
        let stats = render_template_grid_with_override(
            ui,
            &tpl,
            &self.store,
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
            render_view3d_with_override(
                ui,
                view,
                &self.store,
                &mut self.view3d_state,
                self.label_arg.to_override(),
            )
        });
        self.template = Some(tpl);
        result
    }

    /// v0.1.0 single-trace fallback (no template given).
    fn render_single_trace(&mut self, ui: &mut egui::Ui) {
        let key = self.select_key();
        let title = match key.as_deref() {
            Some(k) => format!("trace: {k}"),
            None => "waiting for data…".to_string(),
        };
        ui.label(&title);

        let points: Vec<[f64; 2]> = key
            .as_deref()
            .map(|k| self.store.points(k))
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
            self.store.keys().len(),
            count,
        ));
    }
}
