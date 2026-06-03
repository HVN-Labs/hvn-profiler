#![cfg(feature = "mavlink-source")]

//! v0.10.1 — `Sample.drone_name` is `Option<Arc<str>>` so a single string
//! allocation is shared across every sample fanned out from one MAVLink
//! frame / msgpack envelope.
//!
//! Pre-v0.10.1, the per-sample closure in `decode_to_samples_with_drone`
//! cloned an owned `String` for every sample (originally 6 per `RAW_IMU`
//! message; v0.16.5 raised this to 10 — 9 scalar indices + `mag_xyz`
//! Vec[3]). With the `Arc<str>` migration, the same backing buffer is
//! refcount-bumped instead — verifiable via `Arc::strong_count` since every
//! emitted sample's `drone_name` shares the same allocation as the one we
//! passed in.

use std::sync::Arc;

use mavlink::dialects::ardupilotmega::{MavMessage, RAW_IMU_DATA};

use profiler_source::mavlink_source::decode_to_samples_with_drone;

#[test]
fn decode_emits_arc_str_drone_names_with_shared_backing_buffer() {
    // RAW_IMU expands to 10 samples per frame in v0.16.5 (9 scalar indices
    // + `mag_xyz` Vec[3]) — the worst case for the pre-v0.10.1 String-clone
    // loop. With Arc<str>, every emitted sample's `drone_name` refers to
    // the SAME `Arc` instance we handed in.
    let msg = MavMessage::RAW_IMU(RAW_IMU_DATA::default());
    let dn: Arc<str> = Arc::from("sysid_42");
    let strong_before = Arc::strong_count(&dn);

    let samples = decode_to_samples_with_drone(&msg, 0.0, Some(Arc::clone(&dn)));
    assert_eq!(
        samples.len(),
        10,
        "v0.16.5: RAW_IMU fans out into nine scalar indices + one mag_xyz vector",
    );

    // Every emitted sample's `drone_name` is `Some(arc)` and `Arc::ptr_eq`
    // returns true — i.e. the same backing allocation, not a fresh String.
    for s in &samples {
        let arc = s
            .drone_name
            .as_ref()
            .expect("decoded sample carries a drone_name");
        assert!(
            Arc::ptr_eq(arc, &dn),
            "every sample's drone_name shares the input Arc (no clone of the string buffer)",
        );
    }

    // strong_count: we held one reference (`dn`), plus one per emitted
    // sample (`samples.len()`), plus the `Arc::clone` we passed into the
    // call (the `Some(Arc::clone(&dn))` argument is consumed by the function
    // but each sample borrows from it).
    let strong_after = Arc::strong_count(&dn);
    assert_eq!(
        strong_after,
        strong_before + samples.len(),
        "strong count grows by exactly one per emitted sample (no String allocations)",
    );

    // Dropping the samples brings the count back to the original baseline.
    drop(samples);
    assert_eq!(Arc::strong_count(&dn), strong_before);
}

#[test]
fn decode_without_drone_name_emits_none() {
    // Backwards compat: `None` in → `None` on every sample.
    let msg = MavMessage::RAW_IMU(RAW_IMU_DATA::default());
    let samples = decode_to_samples_with_drone(&msg, 0.0, None);
    // v0.16.5: 9 scalars + mag_xyz Vec[3] = 10 emitted samples.
    assert_eq!(samples.len(), 10);
    for s in &samples {
        assert!(s.drone_name.is_none());
    }
}

#[test]
fn envelope_flatten_shares_arc_across_emitted_samples() {
    // The msgpack flatten path also produces `Arc<str>`-backed names. Build
    // an envelope with a 3-element vector, flatten, and assert every sample
    // shares one Arc.
    use std::collections::BTreeMap;

    #[derive(serde::Serialize)]
    struct Env {
        ts: f64,
        source: String,
        drone_name: Option<String>,
        values: BTreeMap<String, serde_json::Value>,
    }
    let mut values = BTreeMap::new();
    values.insert("accel".into(), serde_json::json!([1.0, 2.0, 3.0]));
    let env = Env {
        ts: 0.0,
        source: "dt".into(),
        drone_name: Some("eric_1".into()),
        values,
    };
    let bytes = rmp_serde::to_vec_named(&env).unwrap();
    let samples = profiler_source::flatten_msgpack(&bytes).unwrap();
    assert!(!samples.is_empty());

    let first = samples[0]
        .drone_name
        .as_ref()
        .expect("envelope drone_name flowed through")
        .clone();
    for s in &samples {
        let arc = s.drone_name.as_ref().unwrap();
        assert!(
            Arc::ptr_eq(arc, &first),
            "all flattened samples share one Arc<str> backing buffer (no per-sample String alloc)",
        );
    }
}
