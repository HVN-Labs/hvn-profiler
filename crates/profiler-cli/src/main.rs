//! hvn-profiler v0.1.0 — first live data.
//!
//! This release wires the CLI to two backends:
//! - `mock://`            → synthetic sine wave (v0.0.1 demo, preserved)
//! - `zmq://host:port`    → subscribe to the HVN-SITL msgpack streamer
//!
//! One trace is rendered (multi-panel layout is v0.2.0). The trace selection
//! prefers `accel[0]` when present, else the busiest channel in the store.

use std::time::Instant;

use clap::Parser;
use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints};
use profiler_render::TraceStore;
use profiler_source::{from_uri, Source};

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

    /// Path to a JSON template describing panels / traces / units.
    /// v0.1.0 ignores this — multi-panel layout lands in v0.2.0.
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
    let title = format!(
        "hvn-profiler v{} — {}",
        env!("CARGO_PKG_VERSION"),
        cli.source
    );

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([960.0, 540.0])
            .with_title(&title),
        ..Default::default()
    };

    eframe::run_native(
        "hvn-profiler",
        native_options,
        Box::new(move |_cc| Ok(Box::new(App::new(source, source_desc)))),
    )
    .map_err(|e| anyhow::anyhow!("eframe::run_native failed: {e}"))
}

struct App {
    source: Box<dyn Source>,
    source_desc: String,
    store: TraceStore,
    started: Instant,
    drained_total: u64,
    /// Wall-clock of the last "samples-in-store" status log (1 Hz).
    last_status_log: Instant,
}

impl App {
    fn new(source: Box<dyn Source>, source_desc: String) -> Self {
        let now = Instant::now();
        Self {
            source,
            source_desc,
            store: TraceStore::default(),
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

        // 1 Hz status log — handy proof-of-life when running headless (smoke
        // test in CI / verification scripts greps for this).
        let now = Instant::now();
        if now.duration_since(self.last_status_log).as_secs_f32() >= 1.0 {
            log::info!(
                "store: keys={} drained_total={} latest_ts={:.2}",
                self.store.keys().len(),
                self.drained_total,
                self.store.latest_ts(),
            );
            self.last_status_log = now;
        }

        let elapsed = self.started.elapsed().as_secs_f64();

        ui.horizontal(|ui| {
            ui.heading(format!("hvn-profiler v{}", env!("CARGO_PKG_VERSION")));
            ui.separator();
            ui.label(format!("t = {elapsed:7.2} s"));
            ui.separator();
            ui.label(&self.source_desc);
            ui.separator();
            ui.label(format!("samples={}", self.drained_total));
        });
        ui.separator();

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
