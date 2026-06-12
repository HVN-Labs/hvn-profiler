//! v0.7.0 Signal-Generators panel — UI + per-frame driver.
//!
//! Lives alongside the Faults panel. Each row holds one [`Generator`]; the
//! `render(...)` entry point lays them out, handles +Add / Delete / Start /
//! Pause, and ticks every running generator at 20 Hz, writing the computed
//! value back into the corresponding Faults slider via [`SLIDER_TARGETS`].
//!
//! Mirrors the matplotlib `signal_generator.py` UX: same five waveforms,
//! same amp / period / centre triple, same lifecycle (Start → Pause →
//! Resume → Delete).

use crate::faults::FaultsPanelState;
use crate::generators::{Generator, Waveform};

/// The slider identifiers a generator can drive — kept here (instead of in
/// `faults.rs`) so the engine doesn't need to import egui-specific state.
/// Order matches the Faults panel section order (GPS → IMU → Mag → Baro).
pub const SLIDER_TARGETS: &[&str] = &[
    // GPS
    "gps.sigma_p",
    "gps.sigma_v",
    "gps.bias_n",
    "gps.bias_e",
    "gps.bias_d",
    // IMU
    "imu.b_a.x",
    "imu.b_a.y",
    "imu.b_a.z",
    "imu.b_g.x",
    "imu.b_g.y",
    "imu.b_g.z",
    "imu.sigma_a_n",
    "imu.sigma_g_n",
    // Mag
    "mag.hi.x",
    "mag.hi.y",
    "mag.hi.z",
    "mag.sigma.x",
    "mag.sigma.y",
    "mag.sigma.z",
    // Baro
    "baro.bias_pa",
    "baro.sigma_pa",
];

/// Mutable state for the signal-generators panel.
#[derive(Debug, Clone, Default)]
pub struct GeneratorPanelState {
    /// Currently visible? Toggled by the toolbar button.
    pub visible: bool,
    /// All generator rows in declaration order.
    pub rows: Vec<Generator>,
}

impl GeneratorPanelState {
    /// Add a new generator row, defaulting to the first available target.
    pub fn add(&mut self) {
        let target = SLIDER_TARGETS.first().copied().unwrap_or("gps.sigma_p");
        self.rows.push(Generator::new(target));
    }

    /// Remove the row at `idx` if it exists.
    pub fn remove(&mut self, idx: usize) {
        if idx < self.rows.len() {
            self.rows.remove(idx);
        }
    }

    /// Tick every running generator and write its current value into the
    /// matching Faults panel slider. Returns the number of generators that
    /// emitted a value this frame (used by the status row + the test
    /// suite).
    ///
    /// `now_ms` is monotonic-time-in-milliseconds; the CLI passes
    /// `started.elapsed().as_millis() as u64`.
    pub fn tick_and_apply(
        &mut self,
        now_ms: u64,
        faults: &mut FaultsPanelState,
    ) -> usize {
        let mut ticked = 0;
        for g in self.rows.iter_mut() {
            if let Some(v) = g.tick(now_ms) {
                apply_to_faults(&g.target, v as f32, faults);
                ticked += 1;
            }
        }
        ticked
    }
}

/// Write `value` into the matching Faults panel slider. Unknown targets
/// are ignored silently (defensive; the dropdown only offers known names).
pub fn apply_to_faults(target: &str, value: f32, s: &mut FaultsPanelState) {
    match target {
        // GPS
        "gps.sigma_p" => s.gps_sigma_p = value,
        "gps.sigma_v" => s.gps_sigma_v = value,
        "gps.bias_n" => s.gps_bias_n = value,
        "gps.bias_e" => s.gps_bias_e = value,
        "gps.bias_d" => s.gps_bias_d = value,
        // IMU
        "imu.b_a.x" => s.imu_b_a_x = value,
        "imu.b_a.y" => s.imu_b_a_y = value,
        "imu.b_a.z" => s.imu_b_a_z = value,
        "imu.b_g.x" => s.imu_b_g_x = value,
        "imu.b_g.y" => s.imu_b_g_y = value,
        "imu.b_g.z" => s.imu_b_g_z = value,
        "imu.sigma_a_n" => s.imu_sigma_a_n = value,
        "imu.sigma_g_n" => s.imu_sigma_g_n = value,
        // Mag
        "mag.hi.x" => s.mag_hi_x = value,
        "mag.hi.y" => s.mag_hi_y = value,
        "mag.hi.z" => s.mag_hi_z = value,
        "mag.sigma.x" => s.mag_sigma_x = value,
        "mag.sigma.y" => s.mag_sigma_y = value,
        "mag.sigma.z" => s.mag_sigma_z = value,
        // Baro
        "baro.bias_pa" => s.baro_bias_pa = value,
        "baro.sigma_pa_rms" => s.baro_sigma_pa_rms = value,
        _ => {
            log::debug!("apply_to_faults: unknown target {target}");
        }
    }
    // Mark the affected section dirty so the existing Faults debounce
    // flushes the new value out to ZMQ on the next render call.
    s.mark_external_change(target);
}

/// Render the Signal Generators panel. Mutates `state.rows` for +Add /
/// Delete / Start / Pause / param edits, then ticks every running row and
/// applies the result to `faults`.
pub fn render_gen_panel(
    ui: &mut egui::Ui,
    state: &mut GeneratorPanelState,
    faults: &mut FaultsPanelState,
    now_ms: u64,
) {
    ui.heading("Signal Generators");
    ui.label(
        egui::RichText::new(
            "Drive any Faults slider with a waveform at 20 Hz. \
             Output is clipped to the slider's safe range by the Faults panel.",
        )
        .small()
        .color(egui::Color32::from_gray(140)),
    );
    ui.separator();

    if ui.button("+ Add Generator").clicked() {
        state.add();
    }
    ui.separator();

    let mut to_remove: Option<usize> = None;
    for (i, gen) in state.rows.iter_mut().enumerate() {
        ui.push_id(i, |ui| {
            ui.horizontal(|ui| {
                // Target.
                egui::ComboBox::from_id_salt("gen_target")
                    .selected_text(&gen.target)
                    .width(140.0)
                    .show_ui(ui, |ui| {
                        for t in SLIDER_TARGETS {
                            ui.selectable_value(&mut gen.target, (*t).into(), *t);
                        }
                    });

                // Waveform.
                egui::ComboBox::from_id_salt("gen_wave")
                    .selected_text(gen.waveform.label())
                    .width(110.0)
                    .show_ui(ui, |ui| {
                        for w in Waveform::all() {
                            ui.selectable_value(&mut gen.waveform, w, w.label());
                        }
                    });

                ui.label("amp");
                ui.add(
                    egui::DragValue::new(&mut gen.amplitude)
                        .speed(0.01)
                        .range(-1e9..=1e9),
                );
                ui.label("period (s)");
                ui.add(
                    egui::DragValue::new(&mut gen.period_s)
                        .speed(0.05)
                        .range(0.01..=1e6),
                );
                ui.label("centre");
                ui.add(
                    egui::DragValue::new(&mut gen.centre)
                        .speed(0.01)
                        .range(-1e9..=1e9),
                );

                // Start / Pause / Resume button cycles by state.
                let (btn_label, want_running) = if gen.running {
                    ("⏸ Pause", false)
                } else if gen.t_start_ms.is_some() {
                    ("▶ Resume", true)
                } else {
                    ("▶ Start", true)
                };
                if ui.button(btn_label).clicked() {
                    if want_running {
                        // Start vs Resume — for Resume we keep the
                        // existing t_start to preserve phase; the engine
                        // ignores the previous run if random_walk_state
                        // grew unbounded somewhere (clamp keeps it sane).
                        if gen.t_start_ms.is_none() {
                            gen.start(now_ms);
                        } else {
                            gen.running = true;
                        }
                    } else {
                        gen.pause();
                    }
                }

                if ui.button("🗑").on_hover_text("Delete generator").clicked() {
                    to_remove = Some(i);
                }
            });
            if let Some(v) = gen.last_value {
                ui.label(
                    egui::RichText::new(format!("→ {v:+.4}"))
                        .small()
                        .color(egui::Color32::from_gray(160)),
                );
            }
            ui.separator();
        });
    }

    if let Some(idx) = to_remove {
        state.remove(idx);
    }

    // Drive every running generator and push its value into the Faults
    // panel state. The existing 50 ms debounce in `faults.rs` will batch
    // the resulting FaultCommand emissions automatically.
    let _ticked = state.tick_and_apply(now_ms, faults);
}
