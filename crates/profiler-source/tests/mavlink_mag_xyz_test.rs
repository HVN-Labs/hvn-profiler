//! v0.16.5 — `RAW_IMU` must expose magnetometer components.
//!
//! Pre-v0.16.5 the RAW_IMU decoder dropped `xmag/ymag/zmag` on the floor, so
//! a fleet fed only by MAVLink (no DT-Python ZMQ) saw an empty
//! `mag_interference` cell because the `hvn-default` template binds
//! `key: "mag_xyz"` and nothing produced it.
//!
//! Contract under test:
//!
//! 1. `ap_raw_imu[6..8]` carry the raw mGauss counts (parity with the other
//!    `ap_raw_imu[i]` indices, which keep the wire units of the corresponding
//!    field).
//! 2. A top-level `mag_xyz` sample of shape `Value::Vector([x, y, z])` is
//!    emitted in **gauss** — the same units DT-Python's `hil_bridge.py`
//!    publishes, so the `hvn-default` template (scale=1000 → mGauss display)
//!    renders identically across sources.
//! 3. The pre-existing acc + gyro fan-out (`ap_raw_imu[0..5]`) is untouched.

#![cfg(feature = "mavlink-source")]

use mavlink::dialects::ardupilotmega::{MavMessage, RAW_IMU_DATA};
use profiler_source::mavlink_source::decode_to_samples;
use profiler_source::Value;

/// Floating-point tolerance for `mGauss / 1000 == gauss`. The cast goes
/// `i16 → f64 / 1000.0` which is exact for the values we use here, but the
/// constant keeps the assertion source-readable.
const EPS: f64 = 1e-12;

#[test]
fn raw_imu_emits_mag_xyz_vector_in_gauss() {
    let d = RAW_IMU_DATA {
        time_usec: 0,
        xacc: 11,
        yacc: 22,
        zacc: 33,
        xgyro: 44,
        ygyro: 55,
        zgyro: 66,
        xmag: 120,
        ymag: -340,
        zmag: 550,
    };
    let msg = MavMessage::RAW_IMU(d);
    let samples = decode_to_samples(&msg, 0.0);

    // (1) Top-level `mag_xyz` Vec[3] in gauss = mGauss / 1000.
    let mag = samples
        .iter()
        .find(|s| s.key == "mag_xyz")
        .expect("RAW_IMU must produce a top-level mag_xyz sample");
    match &mag.value {
        Value::Vector(v) => {
            assert_eq!(v.len(), 3, "mag_xyz must be Vec[3], got {}", v.len());
            assert!(
                (v[0] - 0.120).abs() < EPS,
                "mag_xyz[0]: expected 0.120 g, got {}",
                v[0],
            );
            assert!(
                (v[1] - (-0.340)).abs() < EPS,
                "mag_xyz[1]: expected -0.340 g, got {}",
                v[1],
            );
            assert!(
                (v[2] - 0.550).abs() < EPS,
                "mag_xyz[2]: expected 0.550 g, got {}",
                v[2],
            );
        }
        other => panic!("mag_xyz must be Value::Vector, got {other:?}"),
    }
}

#[test]
fn raw_imu_keeps_per_index_mag_scalars_in_mgauss() {
    let d = RAW_IMU_DATA {
        time_usec: 0,
        xacc: 0,
        yacc: 0,
        zacc: 0,
        xgyro: 0,
        ygyro: 0,
        zgyro: 0,
        xmag: 120,
        ymag: -340,
        zmag: 550,
    };
    let msg = MavMessage::RAW_IMU(d);
    let samples = decode_to_samples(&msg, 0.0);

    // (2) Per-index scalars carry raw mGauss for parity with ap_raw_imu[0..5].
    for (i, expected) in [(6usize, 120.0_f64), (7, -340.0), (8, 550.0)] {
        let key = format!("ap_raw_imu[{i}]");
        let got = samples
            .iter()
            .find(|s| s.key == key)
            .unwrap_or_else(|| panic!("missing {key}"));
        assert_eq!(got.scalar(), expected, "{key} should be raw mGauss");
    }
}

#[test]
fn raw_imu_preserves_pre_v0_16_5_acc_and_gyro_indices() {
    let d = RAW_IMU_DATA {
        time_usec: 0,
        xacc: 11,
        yacc: 22,
        zacc: 33,
        xgyro: 44,
        ygyro: 55,
        zgyro: 66,
        xmag: 0,
        ymag: 0,
        zmag: 0,
    };
    let msg = MavMessage::RAW_IMU(d);
    let samples = decode_to_samples(&msg, 0.0);

    // (3) Acc + gyro indices 0..5 still present and unchanged.
    for (i, expected) in [11.0_f64, 22.0, 33.0, 44.0, 55.0, 66.0].iter().enumerate() {
        let key = format!("ap_raw_imu[{i}]");
        let got = samples
            .iter()
            .find(|s| s.key == key)
            .unwrap_or_else(|| panic!("missing {key}"));
        assert_eq!(got.scalar(), *expected, "{key} should be unchanged");
    }
}
