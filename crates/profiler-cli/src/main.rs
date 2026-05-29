//! hvn-profiler v0.3.0 — JSON-driven 2D grid + 3D trajectory view.
//!
//! Backends (unchanged from v0.1.0):
//! - `mock://`            → synthetic sine wave (v0.0.1 demo, preserved)
//! - `zmq://host:port`    → subscribe to the HVN-SITL msgpack streamer
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

use clap::Parser;
use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints};
use profiler_render::{render_view3d, TraceStore, View3dState};
use profiler_source::{from_uri, Source};
use profiler_template::Template;

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
    /// Supported in v0.1.0:
    /// - `mock://`             — synthetic sine wave (default)
    /// - `zmq://host:port`     — subscribe to the SITL msgpack streamer
    #[arg(long, default_value = "mock://")]
    source: String,

    /// Path to a JSON template describing the panel grid.
    ///
    /// When given, renders the multi-panel 2D layout. When omitted, falls back
    /// to v0.1.0 single-trace mode.
    #[arg(long)]
    template: Option<String>,
}

fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let cli = Cli::parse();
    log::info!(
        "hvn-profiler v{} starting (source={}, template={:?})",
        env!("CARGO_PKG_VERSION"),
        cli.source,
        cli.template,
    );

    let source = from_uri(&cli.source)?;
    let source_desc = source.describe();

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
        Box::new(move |_cc| Ok(Box::new(App::new(source, source_desc, template)))),
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
    started: Instant,
    drained_total: u64,
    /// Wall-clock of the last "samples-in-store" status log (1 Hz).
    last_status_log: Instant,
}

impl App {
    fn new(source: Box<dyn Source>, source_desc: String, template: Option<Template>) -> Self {
        let now = Instant::now();
        // Default mode: Split when the template ships a 3D view, else Grid.
        let has_3d = template
            .as_ref()
            .and_then(|t| t.view_3d.as_ref())
            .is_some();
        let mode = if has_3d { ViewMode::Split } else { ViewMode::Grid };
        Self {
            source,
            source_desc,
            store: TraceStore::default(),
            template,
            mode,
            view3d_state: View3dState::default(),
            view3d_inited: false,
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
        let stats = profiler_render::render_template_grid(ui, &tpl, &self.store);
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
            render_view3d(ui, view, &self.store, &mut self.view3d_state)
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
