//! v0.5.0 integration test — msgpack envelope → TraceStore → 3D trail
//!
//! Builds a SITL-shaped msgpack envelope containing `pos_truth_ned` (NED
//! position triplet), flattens it through `profiler_source::flatten_msgpack`,
//! pushes the resulting samples into a `TraceStore`, and resolves a
//! `Trail3d::sources`-bound trail. Asserts:
//! - the right number of points are decoded into the trail history;
//! - coordinate convention is honoured: world `x = NED[1]` (East),
//!   `y = NED[0]` (North), `z = -NED[2]` (Up).

use std::collections::BTreeMap;

use profiler_render::view3d::trail_world_points;
use profiler_render::TraceStore;
use profiler_source::flatten_msgpack;
use profiler_template::{Trail3d, Trail3dSources};
use serde::Serialize;

#[derive(Serialize)]
struct Envelope {
    ts: f64,
    source: String,
    values: BTreeMap<String, serde_json::Value>,
}

/// Encode one msgpack envelope at time `ts` carrying a single `pos_truth_ned`
/// triplet `[N, E, D]`.
fn envelope_with_pos(ts: f64, n: f64, e: f64, d: f64) -> Vec<u8> {
    let mut values = BTreeMap::new();
    values.insert(
        "pos_truth_ned".into(),
        serde_json::json!([n, e, d]),
    );
    let env = Envelope {
        ts,
        source: "test".into(),
        values,
    };
    rmp_serde::to_vec_named(&env).expect("encode envelope")
}

#[test]
fn round_trip_envelope_into_trail() {
    let mut store = TraceStore::new(60.0);

    // Three envelopes along a straight line: NED = [N=1+i, E=2+2i, D=-(3+i)]
    // so the corresponding world points are (E, N, Up) = (2+2i, 1+i, 3+i).
    for i in 0..3 {
        let i = i as f64;
        let bytes = envelope_with_pos(i, 1.0 + i, 2.0 + 2.0 * i, -(3.0 + i));
        for s in flatten_msgpack(&bytes).expect("flatten") {
            // v0.13.0 — only forward scalar leaves into the legacy
            // numeric ring buffer; the new Vector sample is informational
            // here.
            if let Some(v) = s.value.as_scalar() {
                store.push(s.ts, &s.key, v);
            }
        }
    }

    // Sanity: the three NED components are present.
    assert_eq!(store.len("pos_truth_ned[0]"), 3);
    assert_eq!(store.len("pos_truth_ned[1]"), 3);
    assert_eq!(store.len("pos_truth_ned[2]"), 3);

    let trail = Trail3d {
        name: "truth".into(),
        label: "truth".into(),
        color: "#2ca02c".into(),
        sources: Some(Trail3dSources {
            x: "pos_truth_ned[1]".into(),
            y: "pos_truth_ned[0]".into(),
            z_neg: "pos_truth_ned[2]".into(),
        }),
        deadreckon: None,
    };

    let pts = trail_world_points(&trail, &store);
    assert_eq!(pts.len(), 3);

    // Expected at sample i: world (E, N, Up) = (2+2i, 1+i, 3+i).
    for (i, (t, world)) in pts.iter().enumerate() {
        let fi = i as f64;
        assert!((*t - fi).abs() < 1e-9);
        assert!((world[0] - (2.0 + 2.0 * fi)).abs() < 1e-9, "E mismatch at {i}: {world:?}");
        assert!((world[1] - (1.0 + fi)).abs() < 1e-9, "N mismatch at {i}: {world:?}");
        // z_neg = NED[2] = -(3+i). Up = -z_neg = 3+i.
        assert!((world[2] - (3.0 + fi)).abs() < 1e-9, "Up mismatch at {i}: {world:?}");
    }
}

#[test]
fn missing_source_yields_empty_trail() {
    let store = TraceStore::default();
    let trail = Trail3d {
        name: "ghost".into(),
        sources: Some(Trail3dSources {
            x: "nope[0]".into(),
            y: "nope[1]".into(),
            z_neg: "nope[2]".into(),
        }),
        ..Default::default()
    };
    assert!(trail_world_points(&trail, &store).is_empty());
}
