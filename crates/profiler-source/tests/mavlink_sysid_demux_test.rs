//! v0.16.4 — per-sysid demux of MAVLink samples.
//!
//! Verifies the core invariant of fix (1): one `MavlinkSource` carrying
//! multiple sysids fans out into distinct per-drone samples, each carrying
//! the right sysid AND the right drone_name.
//!
//! These exercise the pure pipeline that the recv worker uses: derive
//! `drone_name` via [`resolve_mavlink_drone_name`], then call the decoder
//! and stamp the sample's sysid. We don't open a UDP socket here — the
//! integration cost is paid once by [`mavlink_active_gcs_test`].

#![cfg(feature = "mavlink-source")]

use std::collections::HashMap;
use std::sync::Arc;

use mavlink::dialects::ardupilotmega::{HEARTBEAT_DATA, MavMessage};
use profiler_source::mavlink_source::{
    decode_to_samples_with_drone, resolve_mavlink_drone_name,
};

/// Helper: emulate the recv worker's per-frame pipeline — resolve the
/// drone name then stamp `sysid` on every emitted Sample.
fn worker_pipeline(
    sysid: u8,
    msg: &MavMessage,
    sysid_map: &HashMap<u8, String>,
    drone_name_override: Option<&str>,
) -> Vec<profiler_source::Sample> {
    let drone_name: Arc<str> =
        resolve_mavlink_drone_name(sysid, sysid_map, drone_name_override);
    let mut samples =
        decode_to_samples_with_drone(msg, 0.0, Some(Arc::clone(&drone_name)));
    for s in samples.iter_mut() {
        s.sysid = Some(sysid);
    }
    samples
}

/// Two HEARTBEAT frames with different sysids produce two distinct
/// per-sample drone_names — proving the demux happens on every frame, not
/// once at connection setup.
#[test]
fn two_distinct_sysids_emit_two_drone_names() {
    let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
    let empty: HashMap<u8, String> = HashMap::new();

    let a = worker_pipeline(1, &msg, &empty, None);
    let b = worker_pipeline(2, &msg, &empty, None);

    assert!(!a.is_empty(), "HEARTBEAT emits armed + flight_mode");
    assert!(!b.is_empty());

    for s in &a {
        assert_eq!(s.drone_name.as_deref(), Some("drone_1"));
        assert_eq!(s.sysid, Some(1));
    }
    for s in &b {
        assert_eq!(s.drone_name.as_deref(), Some("drone_2"));
        assert_eq!(s.sysid, Some(2));
    }
}

/// Twenty-five sysids (the shared-:14560 fleet) all decode to distinct
/// drone_names AND retain their sysid → proving the no-clobber invariant
/// holds at scale.
#[test]
fn twenty_five_sysids_on_shared_port_produce_distinct_samples() {
    let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
    let empty: HashMap<u8, String> = HashMap::new();
    let mut seen_drones: std::collections::HashSet<String> =
        std::collections::HashSet::new();
    let mut seen_sysids: std::collections::HashSet<u8> =
        std::collections::HashSet::new();
    for sysid in 1u8..=25 {
        let samples = worker_pipeline(sysid, &msg, &empty, None);
        assert!(!samples.is_empty(), "sysid={sysid} should emit samples");
        for s in &samples {
            // Each sample carries the right sysid + matching drone_name.
            assert_eq!(s.sysid, Some(sysid));
            assert_eq!(
                s.drone_name.as_deref(),
                Some(format!("drone_{sysid}").as_str()),
            );
        }
        seen_drones.insert(samples[0].drone_name.as_deref().unwrap().to_string());
        seen_sysids.insert(samples[0].sysid.unwrap());
    }
    assert_eq!(
        seen_drones.len(),
        25,
        "shared port should produce 25 distinct drone_names",
    );
    assert_eq!(
        seen_sysids.len(),
        25,
        "shared port should produce 25 distinct sysids",
    );
}

/// Without an override or map, demux uses `drone_{sysid}` fallback. The
/// pre-v0.16.4 prefix was `sysid_<id>`; this test pins the new v0.16.4
/// convention so the picker dedupe heuristic (synthetic := `drone_<sysid>`)
/// recognises the right strings.
#[test]
fn fallback_drone_name_format_is_drone_underscore_sysid() {
    let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
    let empty: HashMap<u8, String> = HashMap::new();
    let samples = worker_pipeline(7, &msg, &empty, None);
    for s in &samples {
        assert_eq!(s.drone_name.as_deref(), Some("drone_7"));
    }
}
