//! v0.16.3 — `SCALED_IMU2` must emit `scaled_imu2[0..9]` per-component
//! scalars PLUS a Vec[10] sample of the form (ax, ay, az, gx, gy, gz, mx, my,
//! mz, temp). Temp is padded with 0 in mavlink-0.18 (no `temperature` field).

#![cfg(feature = "mavlink-source")]

use mavlink::dialects::ardupilotmega::{MavMessage, SCALED_IMU2_DATA};
use profiler_source::mavlink_source::decode_to_samples;
use profiler_source::Value;

#[test]
fn scaled_imu2_emits_vec10_in_canonical_order() {
    let d = SCALED_IMU2_DATA {
        time_boot_ms: 0,
        xacc: 1,
        yacc: 2,
        zacc: 3,
        xgyro: 4,
        ygyro: 5,
        zgyro: 6,
        xmag: 7,
        ymag: 8,
        zmag: 9,
    };
    let msg = MavMessage::SCALED_IMU2(d);
    let samples = decode_to_samples(&msg, 0.0);

    // Per-component scalars (10 entries, in order).
    for (i, expected) in [1, 2, 3, 4, 5, 6, 7, 8, 9, 0].iter().enumerate() {
        let key = format!("scaled_imu2[{i}]");
        let got = samples
            .iter()
            .find(|s| s.key == key)
            .unwrap_or_else(|| panic!("missing {key}"));
        assert_eq!(got.scalar(), *expected as f64, "{key} value");
    }

    // The bundle sample carries the same Vec[10].
    let bundle = samples
        .iter()
        .find(|s| s.key == "scaled_imu2")
        .expect("scaled_imu2 Vec sample missing");
    match &bundle.value {
        Value::Vector(v) => {
            assert_eq!(v.len(), 10, "scaled_imu2 Vec[10]");
            assert_eq!(v, &vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 0.0]);
        }
        other => panic!("expected Value::Vector, got {other:?}"),
    }
}
