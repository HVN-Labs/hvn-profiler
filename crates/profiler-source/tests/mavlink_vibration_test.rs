//! v0.16.3 — `VIBRATION` decode parity with DT-Python.

#![cfg(feature = "mavlink-source")]

use mavlink::dialects::ardupilotmega::{MavMessage, VIBRATION_DATA};
use profiler_source::mavlink_source::decode_to_samples;

#[test]
fn vibration_emits_xyz_and_three_clip_counters() {
    let d = VIBRATION_DATA {
        time_usec: 0,
        vibration_x: 1.5,
        vibration_y: 2.5,
        vibration_z: 3.5,
        clipping_0: 10,
        clipping_1: 20,
        clipping_2: 30,
    };
    let msg = MavMessage::VIBRATION(d);
    let samples = decode_to_samples(&msg, 0.0);
    let by_key: std::collections::HashMap<_, _> =
        samples.iter().map(|s| (s.key.as_str(), s.scalar())).collect();
    assert_eq!(by_key["vibex"], 1.5_f32 as f64);
    assert_eq!(by_key["vibey"], 2.5_f32 as f64);
    assert_eq!(by_key["vibez"], 3.5_f32 as f64);
    assert_eq!(by_key["vibeclip0"], 10.0);
    assert_eq!(by_key["vibeclip1"], 20.0);
    assert_eq!(by_key["vibeclip2"], 30.0);
}
