//! v0.16.4 — picker identity merge: same sysid from ZMQ + MAVLink collapses
//! to ONE picker entry.
//!
//! This test mirrors `App::drain`'s sysid-based canonicalisation logic. The
//! drain loop lives in `profiler-cli/src/main.rs` which is a bin crate
//! (egui-bound, hard to instantiate from a unit test); we re-implement the
//! merge predicate here and assert it against [`profiler_source::Sample`]
//! values shaped exactly like the two real-world sources emit.
//!
//! Why this matters: in HVN-SITL v0.9.11, drone-1 appeared TWICE in the
//! picker — once via ZMQ envelope (`drone_name = "eric_1"`) and once via
//! MAVLink (`drone_name = "drone_1"` from the synthetic fallback). Both
//! samples carry `sysid = 1`. The picker must use sysid as the primary
//! identity so the picker shows ONE row labelled `"eric_1"`.

use std::collections::HashMap;
use std::sync::Arc;

use profiler_source::{Sample, Value};

/// Mirror of [`profiler_cli::is_synthetic_drone_name`] (not exported from
/// the bin crate). Kept in sync by code review.
fn is_synthetic_drone_name(name: &str, sysid: u8) -> bool {
    name == format!("drone_{sysid}").as_str()
}

/// Mirror of `App::drain`'s sysid canonicalisation step. Given a sample
/// + the current `sysid_to_drone` table, return the drone-key it should
///   route under and update the table (upgrading a synthetic canonical to a
///   meaningful one when ZMQ arrives after MAVLink).
fn route(
    sysid_to_drone: &mut HashMap<u8, String>,
    raw_name: &str,
    sysid: Option<u8>,
) -> String {
    match sysid {
        Some(sid) => {
            let synthetic = is_synthetic_drone_name(raw_name, sid);
            match sysid_to_drone.get(&sid) {
                Some(canonical) => {
                    let canonical_synthetic =
                        is_synthetic_drone_name(canonical, sid);
                    if canonical_synthetic && !synthetic {
                        // Upgrade.
                        sysid_to_drone.insert(sid, raw_name.to_string());
                        raw_name.to_string()
                    } else {
                        canonical.clone()
                    }
                }
                None => {
                    sysid_to_drone.insert(sid, raw_name.to_string());
                    raw_name.to_string()
                }
            }
        }
        None => raw_name.to_string(),
    }
}

fn make_sample(drone_name: &str, sysid: Option<u8>) -> Sample {
    Sample {
        ts: 0.0,
        key: "ap_attitude[0]".into(),
        value: Value::Scalar(0.5),
        drone_name: Some(Arc::from(drone_name)),
        sysid,
    }
}

/// ZMQ envelope arrives first (drone_name = "eric_1"), then MAVLink frame
/// from the same sysid arrives (drone_name = "drone_1" — the synthetic
/// fallback). Both must route into the SAME bucket, keyed by the ZMQ name.
#[test]
fn zmq_then_mavlink_for_same_sysid_merges_into_one_bucket() {
    let mut sysid_to_drone: HashMap<u8, String> = HashMap::new();

    let zmq = make_sample("eric_1", Some(1));
    let mav = make_sample("drone_1", Some(1));

    let z_key = route(&mut sysid_to_drone, "eric_1", zmq.sysid);
    let m_key = route(&mut sysid_to_drone, "drone_1", mav.sysid);

    assert_eq!(z_key, "eric_1", "ZMQ sample registers its own name");
    assert_eq!(
        m_key, "eric_1",
        "MAVLink sample with the synthetic fallback name merges under ZMQ's name",
    );
}

/// MAVLink arrives FIRST with synthetic `drone_1`, then ZMQ arrives with
/// `eric_1`. The canonical name upgrades: the bucket that started as
/// `drone_1` is renamed to `eric_1` so the picker shows the friendly label
/// regardless of arrival order.
#[test]
fn mavlink_then_zmq_for_same_sysid_upgrades_canonical_name() {
    let mut sysid_to_drone: HashMap<u8, String> = HashMap::new();

    let mav_key = route(&mut sysid_to_drone, "drone_1", Some(1));
    assert_eq!(mav_key, "drone_1");
    let zmq_key = route(&mut sysid_to_drone, "eric_1", Some(1));
    assert_eq!(
        zmq_key, "eric_1",
        "ZMQ envelope's meaningful name wins over the synthetic MAVLink fallback",
    );
    assert_eq!(
        sysid_to_drone.get(&1).map(String::as_str),
        Some("eric_1"),
        "the sysid→drone table reflects the upgrade",
    );
}

/// Two MAVLink samples for sysid=2 with the operator-supplied `--drone-map`
/// label `"eric_2"` and a ZMQ sample with `"eric_2"`: they all agree, so no
/// upgrade fires — they simply route into the SAME bucket.
#[test]
fn drone_map_makes_both_transports_agree_immediately() {
    let mut sysid_to_drone: HashMap<u8, String> = HashMap::new();
    let m1 = route(&mut sysid_to_drone, "eric_2", Some(2));
    let z = route(&mut sysid_to_drone, "eric_2", Some(2));
    let m2 = route(&mut sysid_to_drone, "eric_2", Some(2));
    assert_eq!(m1, "eric_2");
    assert_eq!(z, "eric_2");
    assert_eq!(m2, "eric_2");
    assert_eq!(sysid_to_drone.len(), 1, "only one entry; no flip-flop");
}

/// Distinct sysids never merge. The picker must show two rows for two
/// different physical drones — sysid is the primary key, drone_name is
/// just the display label.
#[test]
fn distinct_sysids_route_into_distinct_buckets() {
    let mut sysid_to_drone: HashMap<u8, String> = HashMap::new();
    let a = route(&mut sysid_to_drone, "eric_1", Some(1));
    let b = route(&mut sysid_to_drone, "eric_2", Some(2));
    assert_eq!(a, "eric_1");
    assert_eq!(b, "eric_2");
    assert_ne!(a, b);
    assert_eq!(sysid_to_drone.len(), 2);
}

/// When a sample arrives without a sysid (mock source, older streamer),
/// fall back to drone_name routing exactly like the pre-v0.16.4 behaviour.
#[test]
fn no_sysid_falls_back_to_drone_name_routing() {
    let mut sysid_to_drone: HashMap<u8, String> = HashMap::new();
    let a = route(&mut sysid_to_drone, "mock_a", None);
    let b = route(&mut sysid_to_drone, "mock_b", None);
    assert_eq!(a, "mock_a");
    assert_eq!(b, "mock_b");
    assert!(
        sysid_to_drone.is_empty(),
        "no sysid → no entry in the sysid table",
    );
}

/// End-to-end: replay 4 samples (ZMQ eric_1, MAVLink drone_1, ZMQ eric_1,
/// MAVLink drone_1 again) and verify they all land in the `eric_1` bucket.
/// This is the user-visible "picker shows one entry, not two" assertion.
#[test]
fn replay_yields_single_picker_entry_for_same_sysid() {
    let mut sysid_to_drone: HashMap<u8, String> = HashMap::new();
    let mut stores: HashMap<String, usize> = HashMap::new();

    let stream: Vec<Sample> = vec![
        make_sample("eric_1", Some(1)),   // ZMQ
        make_sample("drone_1", Some(1)),  // MAVLink synthetic
        make_sample("eric_1", Some(1)),   // ZMQ again
        make_sample("drone_1", Some(1)),  // MAVLink again
    ];

    for s in stream {
        let raw = s.drone_name.as_deref().unwrap();
        let key = route(&mut sysid_to_drone, raw, s.sysid);
        *stores.entry(key).or_insert(0) += 1;
    }

    assert_eq!(
        stores.len(),
        1,
        "all 4 samples should collapse to ONE picker entry: {stores:?}",
    );
    assert_eq!(
        stores.get("eric_1"),
        Some(&4),
        "the merged bucket should be labelled with ZMQ's authoritative name",
    );
}
