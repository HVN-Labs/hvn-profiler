//! v0.7.0 — feed envelopes through the msgpack flattener and assert each
//! emitted `Sample` carries the `drone_name` field from the wire envelope.
//!
//! The `SeenDrones` set itself lives inside `ZmqSource`'s worker thread (it
//! mutates the set as envelopes arrive over the socket). We don't spin up a
//! real socket here — that path is exercised by the existing
//! `zmq_publisher_publishes_decodable_envelope` test on the SITL side. This
//! file covers the wire-decode side: msgpack bytes in → samples with names
//! out → the set that the worker would populate is exactly what we'd get if
//! we collected the names ourselves.

use std::collections::HashSet;

use profiler_source::flatten_msgpack;
use serde::Serialize;

#[derive(Serialize)]
struct Env {
    ts: f64,
    source: String,
    drone_name: Option<String>,
    values: std::collections::BTreeMap<String, serde_json::Value>,
}

fn envelope_with(ts: f64, drone: Option<&str>) -> Vec<u8> {
    let mut values = std::collections::BTreeMap::new();
    values.insert("accel".into(), serde_json::json!([1.0_f64, 2.0_f64, 3.0_f64]));
    values.insert("ap_vfr_alt".into(), serde_json::json!(4.5_f64));
    let env = Env {
        ts,
        source: "dt".into(),
        drone_name: drone.map(str::to_string),
        values,
    };
    rmp_serde::to_vec_named(&env).expect("encode")
}

#[test]
fn samples_carry_drone_name_from_envelope() {
    let bytes = envelope_with(1.0, Some("eric_1"));
    let samples = flatten_msgpack(&bytes).expect("decode");
    assert!(!samples.is_empty(), "envelope should fan out into samples");
    for s in &samples {
        assert_eq!(s.drone_name.as_deref(), Some("eric_1"));
    }
}

#[test]
fn missing_drone_name_yields_none() {
    let bytes = envelope_with(1.0, None);
    let samples = flatten_msgpack(&bytes).expect("decode");
    assert!(!samples.is_empty());
    for s in &samples {
        assert!(s.drone_name.is_none(), "missing drone_name → None on Sample");
    }
}

#[test]
fn discovery_set_grows_from_multiple_envelopes() {
    // Simulate the worker's seen-drones bookkeeping: feed N envelopes from
    // different drones, collect each Sample's drone_name into a set, and
    // assert the result matches the expected name set.
    let mut seen: HashSet<String> = HashSet::new();
    let inputs = [
        ("eric_1", 0.0_f64),
        ("eric_2", 0.1),
        ("eric_1", 0.2),     // duplicate — should NOT add a new entry
        ("drone_42", 0.3),
        ("eric_2", 0.4),     // duplicate
    ];
    for (name, ts) in inputs {
        let bytes = envelope_with(ts, Some(name));
        let samples = flatten_msgpack(&bytes).expect("decode");
        for s in samples {
            if let Some(n) = s.drone_name {
                seen.insert(n);
            }
        }
    }
    let mut got: Vec<String> = seen.into_iter().collect();
    got.sort();
    assert_eq!(got, vec!["drone_42", "eric_1", "eric_2"]);
}

#[test]
fn discovery_set_ignores_envelopes_with_no_drone_name() {
    // Backward-compat: pre-v0.7.18.4 streamers (or MAVLink CLI without
    // --drone-name) produce envelopes with `drone_name == None`. Those
    // samples must NOT inject a placeholder into the seen set.
    let mut seen: HashSet<String> = HashSet::new();
    let inputs = [
        (None, 0.0_f64),
        (Some("eric_1"), 0.1),
        (None, 0.2),
        (Some("eric_1"), 0.3),
    ];
    for (name, ts) in inputs {
        let bytes = envelope_with(ts, name);
        let samples = flatten_msgpack(&bytes).expect("decode");
        for s in samples {
            if let Some(n) = s.drone_name {
                seen.insert(n);
            }
        }
    }
    let got: Vec<String> = seen.into_iter().collect();
    assert_eq!(got, vec!["eric_1"]);
}
