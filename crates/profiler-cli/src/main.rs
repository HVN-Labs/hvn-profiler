//! hvn-profiler v0.0.1 — toolchain proof.
//!
//! Opens an `eframe` window containing a live-animated sine wave drawn with
//! `egui_plot`. This is intentionally minimal — its only job is to prove that
//! `egui` + `wgpu` build and run on the developer's machine.
//!
//! Real telemetry sources land in v0.1.0.

use std::time::Instant;

use clap::Parser;
use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints};

/// HVN profiler — GPU-accelerated telemetry viewer (v0.0.1 demo).
#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
struct Cli {
    /// Telemetry source URI. Schemes: `mock://`, `mavlink://host:port`,
    /// `zmq://host:port`, `csv://path`. v0.0.1 ignores this and always
    /// renders the mock sine wave.
    #[arg(long, default_value = "mock://")]
    source: String,

    /// Path to a JSON template describing panels / traces / units.
    /// v0.0.1 ignores this — multi-panel layout lands in v0.2.0.
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
    if cli.source != "mock://" {
        log::warn!(
            "Source '{}' is not implemented yet — falling back to mock sine wave (v0.0.1).",
            cli.source
        );
    }

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([960.0, 540.0])
            .with_title("hvn-profiler v0.0.1 — sine wave demo"),
        ..Default::default()
    };

    eframe::run_native(
        "hvn-profiler",
        native_options,
        Box::new(|_cc| Ok(Box::new(App::new()))),
    )
    .map_err(|e| anyhow::anyhow!("eframe::run_native failed: {e}"))
}

struct App {
    started: Instant,
}

impl App {
    fn new() -> Self {
        Self {
            started: Instant::now(),
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // Keep redrawing every frame so the wave animates.
        ui.ctx().request_repaint();

        let elapsed = self.started.elapsed().as_secs_f64();

        ui.horizontal(|ui| {
            ui.heading("hvn-profiler v0.0.1");
            ui.separator();
            ui.label(format!("t = {elapsed:7.2} s"));
            ui.separator();
            ui.label("mock://sine — toolchain proof");
        });
        ui.separator();

        // 1 kHz density across a 4-second window — exercises the renderer
        // more than a token 100-sample line would.
        const N: usize = 4_000;
        let samples: PlotPoints = (0..N)
            .map(|i| {
                let t = elapsed + (i as f64) * 0.001;
                let y = (t * std::f64::consts::TAU * 0.5).sin();
                [t, y]
            })
            .collect();

        Plot::new("sine")
            .legend(Legend::default())
            .show_axes([true, true])
            .show_grid([true, true])
            .show(ui, |plot_ui| {
                plot_ui.line(Line::new("sin(2π·0.5·t)", samples));
            });
    }
}
