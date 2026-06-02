//! v0.16.3 — `AHRS2` (secondary attitude) decode parity with DT-Python.

#![cfg(feature = "mavlink-source")]

use mavlink::dialects::ardupilotmega::{AHRS2_DATA, MavMessage};
use profiler_source::mavlink_source::decode_to_samples;

#[test]
fn ahrs2_emits_roll_pitch_yaw_alt_lat_lng() {
    let d = AHRS2_DATA {
        roll: 0.1,
        pitch: -0.2,
        yaw: 1.5,
        altitude: 100.0,
        lat: 47_000_000,  // 4.7 degrees * 1e7
        lng: -122_000_000, // -12.2 degrees * 1e7
    };
    let msg = MavMessage::AHRS2(d);
    let samples = decode_to_samples(&msg, 1.0);
    let by_key: std::collections::HashMap<_, _> =
        samples.iter().map(|s| (s.key.as_str(), s.scalar())).collect();

    assert!((by_key["ahrs2_roll"] - 0.1_f32 as f64).abs() < 1e-6);
    assert!((by_key["ahrs2_pitch"] - -0.2_f32 as f64).abs() < 1e-6);
    assert!((by_key["ahrs2_yaw"] - 1.5_f32 as f64).abs() < 1e-6);
    assert!((by_key["ahrs2_alt"] - 100.0_f32 as f64).abs() < 1e-4);
    // lat/lng converted from deg×1e7.
    assert!((by_key["ahrs2_lat"] - 4.7).abs() < 1e-7);
    assert!((by_key["ahrs2_lng"] - -12.2).abs() < 1e-7);
}
