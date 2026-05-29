//! hvn-profiler v0.2.0 — JSON-driven multi-panel 2D layout.
//!
//! Backends (unchanged from v0.1.0):
//! - `mock://`            → synthetic sine wave (v0.0.1 demo, preserved)
//! - `zmq://host:port`    → subscribe to the HVN-SITL msgpack streamer
//!
//! Rendering:
//! - With `--template <PATH>`: load the JSON template, lay its `cells` out on
//!   a `grid.rows × grid.cols` grid, one static auto-scaling `egui_plot::Plot`
//!   per visible panel. The `view_3d` block is parsed but NOT rendered (3D is
//!   a later milestone). The 2D panels have no live controls by design.
//! - Without `--template`: fall back to v0.1.0 single-trace mode (prefers
//!   `accel[0]`, else the busiest channel) so the binary works even when no
//!   template file is present.

use std::time::Instant;

use clap::Parser;
use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints};
use profiler_render::TraceStore;
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

struct App {
    source: Box<dyn Source>,
    source_desc: String,
    store: TraceStore,
    /// Optional template — when present, render the multi-panel grid.
    template: Option<Template>,
    started: Instant,
    drained_total: u64,
    /// Wall-clock of the last "samples-in-store" status log (1 Hz).
    last_status_log: Instant,
}

impl App {
    fn new(source: Box<dyn Source>, source_desc: String, template: Option<Template>) -> Self {
        let now = Instant::now();
        Self {
            source,
            source_desc,
            store: TraceStore::default(),
            template,
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
        });
        ui.separator();

        // Status-log bookkeeping is mode-specific; capture a line then emit it
        // after the immutable borrow of `self.template` ends.
        let mut grid_log: Option<(usize, usize, usize)> = None;

        if let Some(tpl) = self.template.take() {
            let stats = profiler_render::render_template_grid(ui, &tpl, &self.store);
            grid_log = Some((stats.panels, stats.panels_with_data, stats.keys_with_data));
            self.template = Some(tpl);
        } else {
            self.render_single_trace(ui);
        }

        // 1 Hz status log — proof-of-life when running headless (smoke test /
        // CI greps for `panels=` in template mode, `store:` otherwise).
        let now = Instant::now();
        if now.duration_since(self.last_status_log).as_secs_f32() >= 1.0 {
            match grid_log {
                Some((panels, with_data, keys_with_data)) => log::info!(
                    "grid: panels={panels} panels_with_data={with_data} \
                     keys_with_data={keys_with_data} drained_total={} latest_ts={:.2}",
                    self.drained_total,
                    self.store.latest_ts(),
                ),
                None => log::info!(
                    "store: keys={} drained_total={} latest_ts={:.2}",
                    self.store.keys().len(),
                    self.drained_total,
                    self.store.latest_ts(),
                ),
            }
            self.last_status_log = now;
        }
    }
}

impl App {
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
