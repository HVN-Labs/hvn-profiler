//! Headless tests for the Faults egui panel: drive an egui [`Context`]
//! without a window, render the panel, and assert that the public
//! [`PendingCommand`] surface matches the SITL runtime-control envelope.
//!
//! Note: egui slider/button responses don't fire in a fully synthetic
//! context (no real pointer events), so this file covers the surfaces
//! the [`profiler_source::FaultPublisher`] actually consumes:
//!
//! - shape of the `gps/imu/mag/baro` params dicts
//! - default drone choices include `"all"` + `drone_1…drone_10`
//! - calling [`render_faults_panel`] in a headless ctx doesn't panic
//!
//! The debounce-flush logic itself is unit-tested in
//! `profiler_render::faults::tests::flush_emits_after_debounce_window`.

use profiler_render::{render_faults_panel, FaultsPanelState, PendingCommand};
use serde_json::Value;

fn run_one_frame(
    ctx: &egui::Context,
    now_s: f64,
    state: &mut FaultsPanelState,
    out: &mut Vec<PendingCommand>,
) {
    let raw_input = egui::RawInput {
        time: Some(now_s),
        ..Default::default()
    };
    let _ = ctx.run_ui(raw_input, |ui| {
        render_faults_panel(ui, state, out, now_s);
    });
}

#[test]
fn gps_params_have_sitl_keys_and_e_is_3vec() {
    let mut s = FaultsPanelState::default();
    // Pick values exactly representable in f32 so the f32 → f64 promotion
    // inside `gps_params()` doesn't introduce float drift (e.g. 0.42 lifts
    // to 0.41999998…). 1.5 and 0.5 are exact in both formats.
    s.gps_sigma_p = 0.5;
    s.gps_bias_n = 1.5;
    let p = s.gps_params();
    assert_eq!(p.get("sigma_p").unwrap(), &Value::from(0.5));
    let e = p.get("_e").expect("GPS _e param").as_array().expect("array");
    assert_eq!(e.len(), 3);
    assert_eq!(e[0], Value::from(1.5));
}

#[test]
fn imu_mag_baro_params_match_runtime_control_known_params() {
    // Cross-reference with runtime_control/features/{imu,mag,baro}/ctl.py
    let s = FaultsPanelState::default();

    let imu = s.imu_params();
    for k in ["b_a", "b_g", "sigma_a_n", "sigma_g_n"] {
        assert!(imu.contains_key(k), "IMU param `{k}` missing");
    }

    let mag = s.mag_params();
    assert_eq!(mag.get("hard_iron").unwrap().as_array().unwrap().len(), 3);
    assert_eq!(mag.get("sigma").unwrap().as_array().unwrap().len(), 3);

    let baro = s.baro_params();
    for k in ["bias_pa", "sigma_pa"] {
        assert!(baro.contains_key(k), "Baro param `{k}` missing");
    }
}

#[test]
fn default_drone_choices_include_all_and_ten_drones() {
    let s = FaultsPanelState::default();
    assert_eq!(s.drone, "all", "default broadcast topic matches SITL CLI default");
    assert!(s.drone_choices.contains(&"all".to_string()));
    for i in 1..=10 {
        assert!(s.drone_choices.contains(&format!("drone_{i}")));
    }
}

#[test]
fn panel_renders_without_panicking() {
    let ctx = egui::Context::default();
    let mut state = FaultsPanelState::default();
    let mut out: Vec<PendingCommand> = Vec::new();
    for i in 0..5 {
        run_one_frame(&ctx, i as f64 * 0.020, &mut state, &mut out);
    }
    // Without synthetic input, no slider response fires → no emit.
    assert!(out.is_empty());
}
