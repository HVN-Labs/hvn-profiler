//! v0.5.0 integration test — dead-reckon trail integrator.
//!
//! The v0.5.0 trail spec for the `dr` trail double-integrates body-frame
//! acceleration through a scalar-first quaternion, no gravity term. This test
//! locks down the identity-attitude / no-gravity case against the matplotlib
//! reference (`a_ned = R @ accel_bb`).

use profiler_render::view3d::{integrate_deadreckon, quat_rotate};

/// Identity-attitude unit-step accel along NED-North: semi-implicit Euler
/// integration with `dt=1` produces `v=0,1,2,3...` and `p(N)=0,1,3,6...`.
/// Crucially, no gravity term: NED-Down stays zero and Up stays zero.
#[test]
fn constant_north_accel_parabolic_north_no_gravity() {
    let q_ident = [1.0_f64, 0.0, 0.0, 0.0];
    let accel: Vec<(f64, [f64; 3])> = (0..4).map(|i| (i as f64, [0.0, 1.0, 0.0])).collect();
    let quats: Vec<(f64, [f64; 4])> = (0..4).map(|i| (i as f64, q_ident)).collect();

    let out = integrate_deadreckon(&accel, &quats, [0.0, 0.0, 0.0]);
    assert_eq!(out.len(), 4);
    let north: Vec<f64> = out.iter().map(|p| p[1]).collect();
    // Semi-implicit Euler with dt=1: v=0,1,2,3 → p=0,1,3,6.
    assert!((north[0] - 0.0).abs() < 1e-9);
    assert!((north[1] - 1.0).abs() < 1e-9);
    assert!((north[2] - 3.0).abs() < 1e-9);
    assert!((north[3] - 6.0).abs() < 1e-9);
    // No gravity term: Up stays exactly zero.
    for p in &out {
        assert!((p[0]).abs() < 1e-12, "E drift on pure-North accel: {}", p[0]);
        assert!((p[2]).abs() < 1e-12, "Up drift (gravity?) on pure-North accel: {}", p[2]);
    }
}

/// Quat that rotates +X (body) to +Y (NED): a +90° yaw about Up. Body-frame
/// accel along x should appear as world-frame accel along North.
#[test]
fn yaw_90_rotates_body_x_to_world_north() {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let q_yaw90 = [s, 0.0, 0.0, s]; // 90° about Up.
    let accel: Vec<(f64, [f64; 3])> = (0..3).map(|i| (i as f64, [1.0, 0.0, 0.0])).collect();
    let quats: Vec<(f64, [f64; 4])> = (0..3).map(|i| (i as f64, q_yaw90)).collect();

    let out = integrate_deadreckon(&accel, &quats, [0.0, 0.0, 0.0]);
    // Rotated accel is +N. North component should grow; East should not.
    assert!(out[2][1] > out[1][1] && out[1][1] > 0.0);
    assert!((out[2][0]).abs() < 1e-9, "spurious East drift: {}", out[2][0]);
}

#[test]
fn seed_position_emitted_as_first_point() {
    let accel = [(0.0_f64, [0.0_f64, 0.0, 0.0])];
    let quats = [(0.0_f64, [1.0_f64, 0.0, 0.0, 0.0])];
    let out = integrate_deadreckon(&accel, &quats, [5.0, 6.0, 7.0]);
    assert_eq!(out.len(), 1);
    // Up = -D = -7.
    assert_eq!(out[0], [5.0, 6.0, -7.0]);
}

#[test]
fn quat_rotate_identity_is_passthrough() {
    let v = [1.0, 2.0, 3.0];
    let r = quat_rotate([1.0, 0.0, 0.0, 0.0], v);
    for k in 0..3 {
        assert!((r[k] - v[k]).abs() < 1e-12);
    }
}
