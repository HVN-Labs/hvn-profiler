//! v0.10.0 — MAVLink `system_id` → `drone_name` demux.
//!
//! Spinning up real UDP sockets just to feed two sysids would be flaky in
//! parallel-test runs (the existing `mavlink_heartbeat_test` already pays the
//! socket cost). Instead we exercise the pure decoder helper
//! [`decode_to_samples_with_drone`] which is what the recv worker calls per
//! frame after deriving `drone_name = "sysid_<header.system_id>"` (or the
//! `--drone NAME` override).
//!
//! The invariant: two messages with different sysids → two distinct
//! `Sample.drone_name` values. With an explicit override → that override wins
//! regardless of sysid.

#![cfg(feature = "mavlink-source")]

use mavlink::dialects::ardupilotmega::{ATTITUDE_DATA, MavMessage};
use profiler_source::mavlink_source::decode_to_samples_with_drone;

#[test]
fn distinct_sysids_produce_distinct_drone_names() {
    let msg_a = MavMessage::ATTITUDE(ATTITUDE_DATA {
        roll: 0.1, pitch: 0.0, yaw: 0.0, ..Default::default()
    });
    let msg_b = MavMessage::ATTITUDE(ATTITUDE_DATA {
        roll: 0.2, pitch: 0.0, yaw: 0.0, ..Default::default()
    });
    // Mimic what the recv worker does: derive `sysid_<id>` per frame and
    // hand it to the decoder.
    let a = decode_to_samples_with_drone(&msg_a, 0.0, Some("sysid_1".into()));
    let b = decode_to_samples_with_drone(&msg_b, 0.1, Some("sysid_2".into()));

    assert!(!a.is_empty() && !b.is_empty(), "ATTITUDE always emits samples");
    // Every sample from msg_a carries the sys-1 tag.
    for s in &a {
        assert_eq!(s.drone_name.as_deref(), Some("sysid_1"));
    }
    // Every sample from msg_b carries the sys-2 tag.
    for s in &b {
        assert_eq!(s.drone_name.as_deref(), Some("sysid_2"));
    }
    // Both streams emit the same keys (ATTITUDE → ap_attitude[0..2]) — proving
    // the demux happens on `drone_name` ALONE, not by mangling key names.
    let keys_a: Vec<_> = a.iter().map(|s| s.key.as_str()).collect();
    let keys_b: Vec<_> = b.iter().map(|s| s.key.as_str()).collect();
    assert_eq!(keys_a, keys_b, "key shape is identical; only drone_name differs");
}

#[test]
fn explicit_drone_name_override_wins_over_sysid() {
    // Operator launched with `--drone explicit_name`: every frame carries that
    // name regardless of inbound `system_id`. (The worker computes the name
    // up-front and hands it to the decoder; we mirror that here.)
    let msg = MavMessage::ATTITUDE(ATTITUDE_DATA {
        roll: 0.5, pitch: 0.0, yaw: 0.0, ..Default::default()
    });
    let samples = decode_to_samples_with_drone(&msg, 0.0, Some("explicit_name".into()));
    assert!(!samples.is_empty());
    for s in &samples {
        assert_eq!(s.drone_name.as_deref(), Some("explicit_name"));
    }
}

#[test]
fn no_drone_name_falls_back_to_none() {
    // Backwards-compatible call shape (the v0.4.0 `decode_to_samples` is a
    // wrapper that calls `decode_to_samples_with_drone(_, _, None)`).
    let msg = MavMessage::ATTITUDE(ATTITUDE_DATA {
        roll: 0.0, pitch: 0.0, yaw: 0.0, ..Default::default()
    });
    let samples = decode_to_samples_with_drone(&msg, 0.0, None);
    for s in &samples {
        assert!(s.drone_name.is_none(), "no drone_name supplied → None on Sample");
    }
}
