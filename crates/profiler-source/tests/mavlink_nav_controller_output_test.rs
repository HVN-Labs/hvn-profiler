//! v0.16.3 — `NAV_CONTROLLER_OUTPUT` decode.

#![cfg(feature = "mavlink-source")]

use mavlink::dialects::ardupilotmega::{MavMessage, NAV_CONTROLLER_OUTPUT_DATA};
use profiler_source::mavlink_source::decode_to_samples;

#[test]
fn nav_controller_output_emits_eight_named_scalars() {
    let d = NAV_CONTROLLER_OUTPUT_DATA {
        nav_roll: 1.0,
        nav_pitch: 2.0,
        alt_error: 3.0,
        aspd_error: 4.0,
        xtrack_error: 5.0,
        nav_bearing: 90,
        target_bearing: 180,
        wp_dist: 250,
    };
    let msg = MavMessage::NAV_CONTROLLER_OUTPUT(d);
    let samples = decode_to_samples(&msg, 0.0);
    let by_key: std::collections::HashMap<_, _> =
        samples.iter().map(|s| (s.key.as_str(), s.scalar())).collect();
    assert_eq!(by_key["nav_roll"], 1.0_f32 as f64);
    assert_eq!(by_key["nav_pitch"], 2.0_f32 as f64);
    assert_eq!(by_key["nav_bearing"], 90.0);
    assert_eq!(by_key["target_bearing"], 180.0);
    assert_eq!(by_key["wp_dist"], 250.0);
    assert_eq!(by_key["alt_error"], 3.0_f32 as f64);
    assert_eq!(by_key["aspd_error"], 4.0_f32 as f64);
    assert_eq!(by_key["xtrack_error"], 5.0_f32 as f64);
}
