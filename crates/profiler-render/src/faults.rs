//! Faults & Interference egui side panel — v0.6.0 bidirectional control.
//!
//! Mirrors the matplotlib `sensor_plot.py` interference panel
//! (`hvn_sitl/gui/components/interference_control.py`): per-sensor slider
//! groups for GPS / IMU / Mag / Baro, plus one-shot fault buttons
//! (GPS dropout, IMU freeze, Mag spike).
//!
//! # Wire contract (replicated from `profiler-source/fault_publisher.rs`)
//!
//! The panel emits [`PendingCommand`] values shaped to be one-to-one with
//! the SITL `runtime_control` dispatcher envelope:
//!
//! ```json
//! { "target": "gps|imu|mag|baro|fault", "params": { ... } , "reset": false }
//! ```
//!
//! Topic frame = drone name (`"eric"`, `"drone_1"`, …) or `"all"`. The
//! `_common.publish()` helper on the SITL side uses `"all"` as the default
//! broadcast topic, **not** `"broadcast"` — the dropdown reflects that.
//!
//! The panel does not own the [`profiler_source::FaultPublisher`] (that lives
//! in `profiler-cli` so the render crate stays GUI-only). Instead, every
//! slider change / button click is recorded into a `Vec<PendingCommand>`
//! that the caller drains each frame and forwards to its publisher. This
//! keeps the panel testable without spinning up a real socket.
//!
//! # Slider ranges
//!
//! Per the matplotlib panel (`interference_control.py`):
//!
//! | section | param        | range            | unit     |
//! |---------|--------------|------------------|----------|
//! | GPS     | sigma_p      | 0 … 5            | m        |
//! | GPS     | sigma_v      | 0 … 1            | m/s      |
//! | GPS     | _e[N/E/D]    | ±20              | m        |
//! | IMU     | b_a[x/y/z]   | ±2               | m/s²     |
//! | IMU     | b_g[x/y/z]   | ±0.1             | rad/s    |
//! | IMU     | sigma_a_n    | 0 … 0.05         |          |
//! | IMU     | sigma_g_n    | 0 … 0.05         |          |
//! | Mag     | hard_iron x/y/z | ±0.5          | G        |
//! | Mag     | sigma x/y/z  | 0 … 0.05         | G        |
//! | Baro    | bias_pa      | ±500             | Pa       |
//! | Baro    | sigma_pa     | 0 … 20           | Pa       |
//!
//! These match the SITL UI's safety bounds. The brief asked for `±1e9` —
//! we honour the request via `ui.add(DragValue)` next to each slider, so
//! the slider provides realistic resolution but the spinbox accepts any
//! finite value the operator types.

use std::collections::HashMap;

use serde_json::Value;

/// One outbound runtime-control command queued by the panel.
///
/// Convert to a `profiler_source::FaultCommand` in `profiler-cli`. The
/// panel doesn't depend on profiler-source so this stays GUI-only.
#[derive(Debug, Clone, PartialEq)]
pub struct PendingCommand {
    pub feature: String,
    pub drone: String,
    pub args: HashMap<String, Value>,
    pub reset: bool,
    /// Optional label, useful for the status row. `"gps_dropout"`,
    /// `"set_sigma_p"`, `""` for the regular debounced slider push.
    pub label: String,
}

/// Mutable slider state for the Faults panel. Defaults are all zeros
/// (matches the matplotlib panel's clean-baseline policy: HIL toggling
/// should not inject noise the user didn't ask for).
#[derive(Debug, Clone)]
pub struct FaultsPanelState {
    /// Currently visible? Bound to the toolbar toggle button.
    pub visible: bool,
    /// Currently selected drone topic ("all" / "drone_1" / …).
    pub drone: String,
    /// Drone choices in the dropdown. v0.6.0 ships a fixed list;
    /// v0.7.0 will discover names from the stream.
    pub drone_choices: Vec<String>,

    // GPS sliders
    pub gps_sigma_p: f32,
    pub gps_sigma_v: f32,
    pub gps_bias_n: f32,
    pub gps_bias_e: f32,
    pub gps_bias_d: f32,

    // IMU sliders
    pub imu_b_a_x: f32,
    pub imu_b_a_y: f32,
    pub imu_b_a_z: f32,
    pub imu_b_g_x: f32,
    pub imu_b_g_y: f32,
    pub imu_b_g_z: f32,
    pub imu_sigma_a_n: f32,
    pub imu_sigma_g_n: f32,

    // Mag sliders
    pub mag_hi_x: f32,
    pub mag_hi_y: f32,
    pub mag_hi_z: f32,
    pub mag_sigma_x: f32,
    pub mag_sigma_y: f32,
    pub mag_sigma_z: f32,

    // Baro sliders
    pub baro_bias_pa: f32,
    pub baro_sigma_pa: f32,

    // Debounce bookkeeping: per-feature "dirty since" time. The render loop
    // pushes the section into `pending` once `now - dirty_since ≥ debounce`.
    // We use `Option<f64>` (seconds, from egui ctx time) so a feature with
    // no pending change has `None`.
    gps_dirty_since: Option<f64>,
    imu_dirty_since: Option<f64>,
    mag_dirty_since: Option<f64>,
    baro_dirty_since: Option<f64>,
}

impl Default for FaultsPanelState {
    fn default() -> Self {
        Self {
            visible: false,
            drone: "all".into(),
            drone_choices: default_drone_choices(),
            gps_sigma_p: 0.0,
            gps_sigma_v: 0.0,
            gps_bias_n: 0.0,
            gps_bias_e: 0.0,
            gps_bias_d: 0.0,
            imu_b_a_x: 0.0,
            imu_b_a_y: 0.0,
            imu_b_a_z: 0.0,
            imu_b_g_x: 0.0,
            imu_b_g_y: 0.0,
            imu_b_g_z: 0.0,
            imu_sigma_a_n: 0.0,
            imu_sigma_g_n: 0.0,
            mag_hi_x: 0.0,
            mag_hi_y: 0.0,
            mag_hi_z: 0.0,
            mag_sigma_x: 0.0,
            mag_sigma_y: 0.0,
            mag_sigma_z: 0.0,
            baro_bias_pa: 0.0,
            baro_sigma_pa: 0.0,
            gps_dirty_since: None,
            imu_dirty_since: None,
            mag_dirty_since: None,
            baro_dirty_since: None,
        }
    }
}

/// Default drone choices in the dropdown. `all` first (broadcast), then
/// `drone_1 … drone_10` matching the dt_runner naming convention.
pub fn default_drone_choices() -> Vec<String> {
    let mut v = vec!["all".to_string()];
    for i in 1..=10 {
        v.push(format!("drone_{i}"));
    }
    v
}

/// Debounce window between the last slider tick and the actual ZMQ push.
const DEBOUNCE_S: f64 = 0.050;

impl FaultsPanelState {
    /// Build the params dict for the GPS section.
    pub fn gps_params(&self) -> HashMap<String, Value> {
        let mut m = HashMap::new();
        m.insert("sigma_p".into(), Value::from(self.gps_sigma_p as f64));
        m.insert("sigma_v".into(), Value::from(self.gps_sigma_v as f64));
        m.insert(
            "_e".into(),
            Value::from(vec![
                Value::from(self.gps_bias_n as f64),
                Value::from(self.gps_bias_e as f64),
                Value::from(self.gps_bias_d as f64),
            ]),
        );
        m
    }

    /// Build the params dict for the IMU section.
    pub fn imu_params(&self) -> HashMap<String, Value> {
        let mut m = HashMap::new();
        m.insert(
            "b_a".into(),
            Value::from(vec![
                Value::from(self.imu_b_a_x as f64),
                Value::from(self.imu_b_a_y as f64),
                Value::from(self.imu_b_a_z as f64),
            ]),
        );
        m.insert(
            "b_g".into(),
            Value::from(vec![
                Value::from(self.imu_b_g_x as f64),
                Value::from(self.imu_b_g_y as f64),
                Value::from(self.imu_b_g_z as f64),
            ]),
        );
        m.insert("sigma_a_n".into(), Value::from(self.imu_sigma_a_n as f64));
        m.insert("sigma_g_n".into(), Value::from(self.imu_sigma_g_n as f64));
        m
    }

    /// Build the params dict for the Mag section.
    pub fn mag_params(&self) -> HashMap<String, Value> {
        let mut m = HashMap::new();
        m.insert(
            "hard_iron".into(),
            Value::from(vec![
                Value::from(self.mag_hi_x as f64),
                Value::from(self.mag_hi_y as f64),
                Value::from(self.mag_hi_z as f64),
            ]),
        );
        m.insert(
            "sigma".into(),
            Value::from(vec![
                Value::from(self.mag_sigma_x as f64),
                Value::from(self.mag_sigma_y as f64),
                Value::from(self.mag_sigma_z as f64),
            ]),
        );
        m
    }

    /// Build the params dict for the Baro section.
    pub fn baro_params(&self) -> HashMap<String, Value> {
        let mut m = HashMap::new();
        // The DT BaroModel accepts both `bias_pa` (current name in
        // `runtime_control/features/baro/ctl.py`'s _KNOWN_PARAMS) and
        // `solder_drift_pa` (the matplotlib panel's name). We send the
        // canonical `bias_pa`; the receiver merges by attribute name.
        m.insert("bias_pa".into(), Value::from(self.baro_bias_pa as f64));
        m.insert("sigma_pa".into(), Value::from(self.baro_sigma_pa as f64));
        m
    }

    /// Mark a section dirty (called from slider responses). The actual
    /// command is queued on the next `render` call ≥ `DEBOUNCE_S` later.
    fn touch(dirty: &mut Option<f64>, now_s: f64) {
        *dirty = Some(now_s);
    }
}

/// Egui side-panel renderer. Drains debounced slider changes and one-shot
/// button clicks into `out`, which the caller forwards to the
/// [`profiler_source::FaultPublisher`].
///
/// `now_s` is `ctx.input(|i| i.time)` from the caller (so unit tests can
/// drive a fake clock).
pub fn render_faults_panel(
    ui: &mut egui::Ui,
    state: &mut FaultsPanelState,
    out: &mut Vec<PendingCommand>,
    now_s: f64,
) {
    ui.heading("Faults & Interference");
    ui.label(
        egui::RichText::new(
            "Bidirectional ZMQ → SITL runtime control. \
             Sliders debounce-send 50 ms after the last change.",
        )
        .small()
        .color(egui::Color32::from_gray(140)),
    );
    ui.separator();

    // ── Drone selector ──────────────────────────────────────────────
    ui.horizontal(|ui| {
        ui.label("Target drone:");
        egui::ComboBox::from_id_salt("faults_drone_select")
            .selected_text(&state.drone)
            .show_ui(ui, |ui| {
                for choice in &state.drone_choices {
                    ui.selectable_value(&mut state.drone, choice.clone(), choice);
                }
            });
    });
    ui.separator();

    // ── GPS ─────────────────────────────────────────────────────────
    ui.collapsing("GPS", |ui| {
        let mut dirty = false;
        dirty |= slider(ui, "σ_p (m)", &mut state.gps_sigma_p, 0.0, 5.0);
        dirty |= slider(ui, "σ_v (m/s)", &mut state.gps_sigma_v, 0.0, 1.0);
        dirty |= slider(ui, "bias N (m)", &mut state.gps_bias_n, -20.0, 20.0);
        dirty |= slider(ui, "bias E (m)", &mut state.gps_bias_e, -20.0, 20.0);
        dirty |= slider(ui, "bias D (m)", &mut state.gps_bias_d, -20.0, 20.0);
        if dirty {
            FaultsPanelState::touch(&mut state.gps_dirty_since, now_s);
        }
        ui.horizontal(|ui| {
            if ui.button("⚠ Dropout").clicked() {
                out.push(gps_dropout_cmd(&state.drone));
            }
            if ui.button("Reset GPS").clicked() {
                state.gps_sigma_p = 0.0;
                state.gps_sigma_v = 0.0;
                state.gps_bias_n = 0.0;
                state.gps_bias_e = 0.0;
                state.gps_bias_d = 0.0;
                state.gps_dirty_since = None;
                out.push(reset_cmd("gps", &state.drone));
            }
        });
    });

    // ── IMU ─────────────────────────────────────────────────────────
    ui.collapsing("IMU", |ui| {
        let mut dirty = false;
        dirty |= slider(ui, "b_a x (m/s²)", &mut state.imu_b_a_x, -2.0, 2.0);
        dirty |= slider(ui, "b_a y (m/s²)", &mut state.imu_b_a_y, -2.0, 2.0);
        dirty |= slider(ui, "b_a z (m/s²)", &mut state.imu_b_a_z, -2.0, 2.0);
        dirty |= slider(ui, "b_g x (rad/s)", &mut state.imu_b_g_x, -0.1, 0.1);
        dirty |= slider(ui, "b_g y (rad/s)", &mut state.imu_b_g_y, -0.1, 0.1);
        dirty |= slider(ui, "b_g z (rad/s)", &mut state.imu_b_g_z, -0.1, 0.1);
        dirty |= slider(ui, "σ_a_n", &mut state.imu_sigma_a_n, 0.0, 0.05);
        dirty |= slider(ui, "σ_g_n", &mut state.imu_sigma_g_n, 0.0, 0.05);
        if dirty {
            FaultsPanelState::touch(&mut state.imu_dirty_since, now_s);
        }
        ui.horizontal(|ui| {
            if ui.button("⚠ Freeze").clicked() {
                out.push(imu_freeze_cmd(&state.drone));
            }
            if ui.button("Reset IMU").clicked() {
                state.imu_b_a_x = 0.0;
                state.imu_b_a_y = 0.0;
                state.imu_b_a_z = 0.0;
                state.imu_b_g_x = 0.0;
                state.imu_b_g_y = 0.0;
                state.imu_b_g_z = 0.0;
                state.imu_sigma_a_n = 0.0;
                state.imu_sigma_g_n = 0.0;
                state.imu_dirty_since = None;
                out.push(reset_cmd("imu", &state.drone));
            }
        });
    });

    // ── Mag ─────────────────────────────────────────────────────────
    ui.collapsing("Magnetometer", |ui| {
        let mut dirty = false;
        dirty |= slider(ui, "hard-iron x (G)", &mut state.mag_hi_x, -0.5, 0.5);
        dirty |= slider(ui, "hard-iron y (G)", &mut state.mag_hi_y, -0.5, 0.5);
        dirty |= slider(ui, "hard-iron z (G)", &mut state.mag_hi_z, -0.5, 0.5);
        dirty |= slider(ui, "σ x (G)", &mut state.mag_sigma_x, 0.0, 0.05);
        dirty |= slider(ui, "σ y (G)", &mut state.mag_sigma_y, 0.0, 0.05);
        dirty |= slider(ui, "σ z (G)", &mut state.mag_sigma_z, 0.0, 0.05);
        if dirty {
            FaultsPanelState::touch(&mut state.mag_dirty_since, now_s);
        }
        ui.horizontal(|ui| {
            if ui.button("⚠ Spike").clicked() {
                out.push(mag_spike_cmd(&state.drone));
            }
            if ui.button("Reset Mag").clicked() {
                state.mag_hi_x = 0.0;
                state.mag_hi_y = 0.0;
                state.mag_hi_z = 0.0;
                state.mag_sigma_x = 0.0;
                state.mag_sigma_y = 0.0;
                state.mag_sigma_z = 0.0;
                state.mag_dirty_since = None;
                out.push(reset_cmd("mag", &state.drone));
            }
        });
    });

    // ── Baro ────────────────────────────────────────────────────────
    ui.collapsing("Barometer", |ui| {
        let mut dirty = false;
        dirty |= slider(ui, "bias (Pa)", &mut state.baro_bias_pa, -500.0, 500.0);
        dirty |= slider(ui, "σ (Pa)", &mut state.baro_sigma_pa, 0.0, 20.0);
        if dirty {
            FaultsPanelState::touch(&mut state.baro_dirty_since, now_s);
        }
        ui.horizontal(|ui| {
            if ui.button("Reset Baro").clicked() {
                state.baro_bias_pa = 0.0;
                state.baro_sigma_pa = 0.0;
                state.baro_dirty_since = None;
                out.push(reset_cmd("baro", &state.drone));
            }
        });
    });

    // ── Debounce flush ──────────────────────────────────────────────
    // Compute each section's payload up-front so the flush helper doesn't
    // need to re-borrow `state` while also taking a mutable ref to its
    // per-section dirty timestamp.
    let drone = state.drone.clone();
    let gps_payload = (drone.clone(), state.gps_params());
    let imu_payload = (drone.clone(), state.imu_params());
    let mag_payload = (drone.clone(), state.mag_params());
    let baro_payload = (drone, state.baro_params());

    flush_if_due(&mut state.gps_dirty_since, now_s, "gps", gps_payload, out);
    flush_if_due(&mut state.imu_dirty_since, now_s, "imu", imu_payload, out);
    flush_if_due(&mut state.mag_dirty_since, now_s, "mag", mag_payload, out);
    flush_if_due(&mut state.baro_dirty_since, now_s, "baro", baro_payload, out);
}

fn flush_if_due(
    dirty: &mut Option<f64>,
    now_s: f64,
    feature: &str,
    payload: (String, HashMap<String, Value>),
    out: &mut Vec<PendingCommand>,
) {
    if let Some(t0) = *dirty {
        if now_s - t0 >= DEBOUNCE_S {
            let (drone, args) = payload;
            out.push(PendingCommand {
                feature: feature.to_string(),
                drone,
                args,
                reset: false,
                label: format!("set_{feature}"),
            });
            *dirty = None;
        }
    }
}

/// Slider + drag-value combo. Returns `true` iff the user changed the value
/// this frame.
fn slider(ui: &mut egui::Ui, label: &str, v: &mut f32, lo: f32, hi: f32) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(label);
        // The brief allows ±1e9 via DragValue; the slider stays in the safe
        // matplotlib-equivalent range for fine control.
        let r = ui.add(egui::Slider::new(v, lo..=hi).clamping(egui::SliderClamping::Never));
        if r.changed() {
            changed = true;
        }
        let r2 = ui.add(egui::DragValue::new(v).speed((hi - lo) / 200.0));
        if r2.changed() {
            changed = true;
        }
    });
    changed
}

// ── One-shot fault command builders ────────────────────────────────────

/// `fault.gps_dropout` — uses the SITL `fault` feature schedule.
///
/// Matches `runtime_control/features/fault/ctl.py`: the dispatcher routes
/// `target="fault"` to the FaultInjector, which interprets a `dropout`
/// profile by holding `sats_base` (or any sensor param) at `value=0` for
/// `t_duration` seconds.
fn gps_dropout_cmd(drone: &str) -> PendingCommand {
    let mut args: HashMap<String, Value> = HashMap::new();
    args.insert("target".into(), Value::from("gps"));
    args.insert("param".into(), Value::from("sats_base"));
    args.insert("profile".into(), Value::from("dropout"));
    args.insert("mode".into(), Value::from("set"));
    args.insert("t_duration".into(), Value::from(10.0));
    let mut prof = serde_json::Map::new();
    prof.insert("value".into(), Value::from(0.0));
    args.insert("params".into(), Value::Object(prof));
    PendingCommand {
        feature: "fault".into(),
        drone: drone.into(),
        args,
        reset: false,
        label: "gps_dropout".into(),
    }
}

fn imu_freeze_cmd(drone: &str) -> PendingCommand {
    // "Freeze" → pin sigma_a_n / sigma_g_n to zero for a window. The fault
    // scheduler doesn't directly support multi-param freeze, so for v0.6.0
    // we send a single-axis sigma=0 schedule and let the operator combine
    // with the slider reset for the rest. v0.7.0 will introduce a real
    // multi-param fault primitive in SITL.
    let mut args: HashMap<String, Value> = HashMap::new();
    args.insert("target".into(), Value::from("imu"));
    args.insert("param".into(), Value::from("sigma_a_n"));
    args.insert("profile".into(), Value::from("dropout"));
    args.insert("mode".into(), Value::from("set"));
    args.insert("t_duration".into(), Value::from(10.0));
    let mut prof = serde_json::Map::new();
    prof.insert("value".into(), Value::from(0.0));
    args.insert("params".into(), Value::Object(prof));
    PendingCommand {
        feature: "fault".into(),
        drone: drone.into(),
        args,
        reset: false,
        label: "imu_freeze".into(),
    }
}

fn mag_spike_cmd(drone: &str) -> PendingCommand {
    // Mag spike → step the X-axis hard-iron up by +1 G for 5 s.
    let mut args: HashMap<String, Value> = HashMap::new();
    args.insert("target".into(), Value::from("mag"));
    args.insert("param".into(), Value::from("hard_iron"));
    args.insert("profile".into(), Value::from("step"));
    args.insert("mode".into(), Value::from("add"));
    args.insert("t_duration".into(), Value::from(5.0));
    args.insert("axis".into(), Value::from(0));
    let mut prof = serde_json::Map::new();
    prof.insert("amp".into(), Value::from(1.0));
    args.insert("params".into(), Value::Object(prof));
    PendingCommand {
        feature: "fault".into(),
        drone: drone.into(),
        args,
        reset: false,
        label: "mag_spike".into(),
    }
}

/// `reset` envelope for one feature ("revert to startup_config").
fn reset_cmd(feature: &str, drone: &str) -> PendingCommand {
    PendingCommand {
        feature: feature.into(),
        drone: drone.into(),
        args: HashMap::new(),
        reset: true,
        label: format!("reset_{feature}"),
    }
}

// ── Tests ──────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gps_params_match_sitl_schema() {
        let mut s = FaultsPanelState::default();
        s.gps_sigma_p = 0.5;
        s.gps_bias_n = 1.0;
        let p = s.gps_params();
        assert_eq!(p.get("sigma_p"), Some(&Value::from(0.5)));
        assert_eq!(p.get("_e").unwrap()[0], Value::from(1.0));
        assert_eq!(p.get("_e").unwrap().as_array().unwrap().len(), 3);
    }

    #[test]
    fn imu_params_contain_b_a_b_g_and_sigmas() {
        let s = FaultsPanelState::default();
        let p = s.imu_params();
        assert!(p.contains_key("b_a"));
        assert!(p.contains_key("b_g"));
        assert!(p.contains_key("sigma_a_n"));
        assert!(p.contains_key("sigma_g_n"));
    }

    #[test]
    fn mag_params_contain_hard_iron_and_sigma_vec3() {
        let s = FaultsPanelState::default();
        let p = s.mag_params();
        assert_eq!(p.get("hard_iron").unwrap().as_array().unwrap().len(), 3);
        assert_eq!(p.get("sigma").unwrap().as_array().unwrap().len(), 3);
    }

    #[test]
    fn flush_emits_after_debounce_window() {
        let mut dirty = Some(10.0);
        let mut out: Vec<PendingCommand> = Vec::new();
        let mk_payload = || ("all".to_string(), HashMap::<String, Value>::new());
        // 30 ms after touch — must NOT emit.
        flush_if_due(&mut dirty, 10.030, "gps", mk_payload(), &mut out);
        assert_eq!(out.len(), 0);
        // 60 ms after touch — DOES emit, dirty cleared.
        flush_if_due(&mut dirty, 10.060, "gps", mk_payload(), &mut out);
        assert_eq!(out.len(), 1);
        assert!(dirty.is_none());
        assert_eq!(out[0].feature, "gps");
        assert_eq!(out[0].label, "set_gps");
    }

    #[test]
    fn gps_dropout_targets_fault_feature() {
        let cmd = gps_dropout_cmd("eric");
        assert_eq!(cmd.feature, "fault");
        assert_eq!(cmd.drone, "eric");
        assert_eq!(cmd.args.get("target"), Some(&Value::from("gps")));
        assert_eq!(cmd.args.get("profile"), Some(&Value::from("dropout")));
    }

    #[test]
    fn reset_cmd_sets_reset_true_and_empty_args() {
        let cmd = reset_cmd("baro", "all");
        assert!(cmd.reset);
        assert!(cmd.args.is_empty());
    }

}
