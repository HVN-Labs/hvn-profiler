//! v0.7.0 — integration tests for the Signal Generators panel state
//! (`GeneratorPanelState::tick_and_apply` + `apply_to_faults`). Mirrors
//! the matplotlib `signal_generator.py` behavior: every running row writes
//! its current value into the matching Faults slider on each tick.

use profiler_render::{
    gen_panel::apply_to_faults, FaultsPanelState, GeneratorPanelState, Waveform,
};

#[test]
fn add_three_generators_tick_writes_into_faults_state() {
    let mut panel = GeneratorPanelState::default();
    panel.add();
    panel.add();
    panel.add();
    assert_eq!(panel.rows.len(), 3);

    // Aim each generator at a different slider so we can prove the per-row
    // routing works.
    panel.rows[0].target = "gps.sigma_p".into();
    panel.rows[0].waveform = Waveform::Sine;
    panel.rows[0].amplitude = 1.0;
    panel.rows[0].period_s = 4.0;
    panel.rows[0].centre = 2.0;
    panel.rows[0].start(0);

    panel.rows[1].target = "imu.b_a.x".into();
    panel.rows[1].waveform = Waveform::Sine;
    panel.rows[1].amplitude = 0.5;
    panel.rows[1].period_s = 4.0;
    panel.rows[1].centre = -0.25;
    panel.rows[1].start(0);

    panel.rows[2].target = "baro.bias_pa".into();
    panel.rows[2].waveform = Waveform::Sine;
    panel.rows[2].amplitude = 100.0;
    panel.rows[2].period_s = 4.0;
    panel.rows[2].centre = 0.0;
    panel.rows[2].start(0);

    let mut faults = FaultsPanelState::default();
    // 100 ms after start, ω·t = 2π · 0.1 / 4 = π/20 ≈ 0.157 rad → sin ≈ 0.156.
    let ticked = panel.tick_and_apply(100, &mut faults);
    assert_eq!(ticked, 3);

    // gps.sigma_p: centre 2.0, amp 1.0 → 2 + 1·sin(π/20)
    let expect_gps = 2.0 + 1.0 * (std::f64::consts::TAU * 0.1 / 4.0).sin();
    assert!(
        (faults.gps_sigma_p as f64 - expect_gps).abs() < 1e-4,
        "gps.sigma_p={}, want {}",
        faults.gps_sigma_p,
        expect_gps
    );

    // imu.b_a.x: centre -0.25, amp 0.5
    let expect_imu =
        -0.25 + 0.5 * (std::f64::consts::TAU * 0.1 / 4.0).sin();
    assert!(
        (faults.imu_b_a_x as f64 - expect_imu).abs() < 1e-4,
        "imu.b_a.x={}, want {}",
        faults.imu_b_a_x,
        expect_imu
    );

    // baro.bias_pa: centre 0, amp 100
    let expect_baro = 100.0 * (std::f64::consts::TAU * 0.1 / 4.0).sin();
    assert!(
        (faults.baro_bias_pa as f64 - expect_baro).abs() < 1e-2,
        "baro.bias_pa={}, want {}",
        faults.baro_bias_pa,
        expect_baro
    );
}

#[test]
fn deleting_a_generator_removes_it_and_stops_driving_its_target() {
    let mut panel = GeneratorPanelState::default();
    panel.add();
    panel.add();
    assert_eq!(panel.rows.len(), 2);

    panel.rows[0].target = "gps.sigma_p".into();
    panel.rows[0].waveform = Waveform::Sine;
    panel.rows[0].amplitude = 1.0;
    panel.rows[0].period_s = 1.0;
    panel.rows[0].centre = 5.0;
    panel.rows[0].start(0);

    panel.rows[1].target = "gps.sigma_v".into();
    panel.rows[1].waveform = Waveform::Sine;
    panel.rows[1].amplitude = 0.3;
    panel.rows[1].period_s = 1.0;
    panel.rows[1].centre = 0.1;
    panel.rows[1].start(0);

    let mut faults = FaultsPanelState::default();
    panel.tick_and_apply(100, &mut faults);
    let pre_delete_sigma_v = faults.gps_sigma_v;
    assert!(pre_delete_sigma_v > 0.0);

    panel.remove(1);
    assert_eq!(panel.rows.len(), 1);

    // Manually zero the previously-driven slider; ticking again must NOT
    // restore it (the row that drove it is gone).
    faults.gps_sigma_v = 0.0;
    panel.tick_and_apply(200, &mut faults);
    assert_eq!(
        faults.gps_sigma_v, 0.0,
        "deleted generator should no longer drive its slider"
    );
    // …but gps.sigma_p should still be moving.
    assert!(faults.gps_sigma_p > 4.0);
}

#[test]
fn apply_to_faults_dispatches_to_every_known_slider() {
    let mut f = FaultsPanelState::default();
    // Hit every target name once; assert the corresponding f32 field
    // changed to a known sentinel.
    let cases: &[(&str, f32)] = &[
        ("gps.sigma_p", 1.0),
        ("gps.sigma_v", 0.2),
        ("gps.bias_n", 1.5),
        ("gps.bias_e", -1.5),
        ("gps.bias_d", 0.5),
        ("imu.b_a.x", 0.1),
        ("imu.b_a.y", 0.2),
        ("imu.b_a.z", 0.3),
        ("imu.b_g.x", 0.01),
        ("imu.b_g.y", 0.02),
        ("imu.b_g.z", 0.03),
        ("imu.sigma_a_n", 0.001),
        ("imu.sigma_g_n", 0.002),
        ("mag.hi.x", 0.1),
        ("mag.hi.y", 0.2),
        ("mag.hi.z", 0.3),
        ("mag.sigma.x", 0.001),
        ("mag.sigma.y", 0.002),
        ("mag.sigma.z", 0.003),
        ("baro.bias_pa", 50.0),
        ("baro.sigma_pa", 5.0),
    ];
    for (t, v) in cases {
        apply_to_faults(t, *v, &mut f);
    }
    assert_eq!(f.gps_sigma_p, 1.0);
    assert_eq!(f.gps_bias_d, 0.5);
    assert_eq!(f.imu_b_a_z, 0.3);
    assert_eq!(f.imu_sigma_g_n, 0.002);
    assert_eq!(f.mag_sigma_z, 0.003);
    assert_eq!(f.baro_sigma_pa, 5.0);
}

#[test]
fn unknown_target_is_ignored() {
    let mut f = FaultsPanelState::default();
    let before = f.gps_sigma_p;
    apply_to_faults("nope.does_not_exist", 99.0, &mut f);
    assert_eq!(f.gps_sigma_p, before);
}
