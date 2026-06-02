//! v0.13.0 — `Sample::Value` end-to-end: the msgpack decoder must preserve
//! String / Bool / IntVector / TextLog leaves through the wire format
//! instead of dropping them like the v0.12.0 decoder did.
//!
//! These tests build msgpack-encoded envelopes by hand (the same path
//! `ZmqSource`'s worker thread takes), run them through
//! [`flatten_msgpack`], and assert each leaf produces the expected
//! [`Value`] variant on the resulting [`Sample`].

use std::collections::BTreeMap;

use profiler_source::{flatten_msgpack, Sample, Value};
use rmp_serde::Serializer;
use serde::Serialize;

/// Helper: serialize an arbitrary map to msgpack-with-string-keys. The
/// `to_vec_named` path matches how `hvn_sitl.streamer` encodes envelopes.
fn encode<T: Serialize>(t: &T) -> Vec<u8> {
    let mut buf = Vec::new();
    t.serialize(&mut Serializer::new(&mut buf).with_struct_map())
        .expect("serialize");
    buf
}

/// Find a sample by exact key.
fn find<'a>(samples: &'a [Sample], key: &str) -> &'a Sample {
    samples
        .iter()
        .find(|s| s.key == key)
        .unwrap_or_else(|| panic!("missing sample with key '{key}' in {:?}", samples_keys(samples)))
}

fn samples_keys(samples: &[Sample]) -> Vec<&str> {
    samples.iter().map(|s| s.key.as_str()).collect()
}

#[test]
fn bool_value_decodes_to_value_bool() {
    #[derive(Serialize)]
    struct Env {
        ts: f64,
        values: BTreeMap<String, serde_json::Value>,
    }
    let mut values = BTreeMap::new();
    values.insert("armed".into(), serde_json::json!(true));
    let bytes = encode(&Env { ts: 1.0, values });

    let samples = flatten_msgpack(&bytes).expect("decode");
    let s = find(&samples, "armed");
    assert_eq!(s.value, Value::Bool(true));
}

#[test]
fn string_value_decodes_to_value_string() {
    #[derive(Serialize)]
    struct Env {
        ts: f64,
        values: BTreeMap<String, serde_json::Value>,
    }
    let mut values = BTreeMap::new();
    values.insert("flight_mode".into(), serde_json::json!("GUIDED"));
    let bytes = encode(&Env { ts: 2.0, values });

    let samples = flatten_msgpack(&bytes).expect("decode");
    let s = find(&samples, "flight_mode");
    match &s.value {
        Value::String(arc) => assert_eq!(arc.as_ref(), "GUIDED"),
        other => panic!("expected Value::String, got {other:?}"),
    }
}

#[test]
fn int_only_array_decodes_to_int_vector_plus_legacy_scalars() {
    #[derive(Serialize)]
    struct Env {
        ts: f64,
        values: BTreeMap<String, serde_json::Value>,
    }
    let mut values = BTreeMap::new();
    // `rc_channels`-shaped: 4-element integer array.
    values.insert(
        "rc_channels".into(),
        serde_json::json!([1500_i64, 1500_i64, 1100_i64, 1500_i64]),
    );
    let bytes = encode(&Env { ts: 3.0, values });

    let samples = flatten_msgpack(&bytes).expect("decode");
    // The base key carries the typed vector.
    let base = find(&samples, "rc_channels");
    match &base.value {
        Value::IntVector(v) => assert_eq!(v, &[1500, 1500, 1100, 1500]),
        other => panic!("expected Value::IntVector, got {other:?}"),
    }
    // And the legacy per-component scalars are still present (so existing
    // templates wired to `rc_channels[0..3]` keep plotting).
    for (i, expected) in [1500.0, 1500.0, 1100.0, 1500.0].iter().enumerate() {
        let s = find(&samples, &format!("rc_channels[{i}]"));
        assert_eq!(s.value, Value::Scalar(*expected));
    }
}

#[test]
fn mixed_numeric_array_decodes_to_vector_plus_legacy_scalars() {
    #[derive(Serialize)]
    struct Env {
        ts: f64,
        values: BTreeMap<String, serde_json::Value>,
    }
    let mut values = BTreeMap::new();
    values.insert(
        "accel".into(),
        serde_json::json!([1.0_f64, 2.0_f64, 3.0_f64]),
    );
    let bytes = encode(&Env { ts: 4.0, values });

    let samples = flatten_msgpack(&bytes).expect("decode");
    let base = find(&samples, "accel");
    match &base.value {
        Value::Vector(v) => assert_eq!(v, &[1.0, 2.0, 3.0]),
        other => panic!("expected Value::Vector, got {other:?}"),
    }
    // Legacy per-component scalars preserved.
    for (i, expected) in [1.0_f64, 2.0, 3.0].iter().enumerate() {
        let s = find(&samples, &format!("accel[{i}]"));
        assert_eq!(s.value, Value::Scalar(*expected));
    }
}

#[test]
fn text_log_array_decodes_to_value_text_log() {
    #[derive(Serialize)]
    struct Env {
        ts: f64,
        values: BTreeMap<String, serde_json::Value>,
    }
    let mut values = BTreeMap::new();
    // Mirror the SITL v0.9.0 `statustexts` schema: list of dicts with
    // `severity`, `text`, `ts` keys.
    values.insert(
        "statustexts".into(),
        serde_json::json!([
            {"severity": 6, "text": "boot complete",    "ts": 1.0},
            {"severity": 4, "text": "PreArm: Battery", "ts": 2.0},
        ]),
    );
    let bytes = encode(&Env { ts: 5.0, values });

    let samples = flatten_msgpack(&bytes).expect("decode");
    let s = find(&samples, "statustexts");
    match &s.value {
        Value::TextLog(entries) => {
            assert_eq!(entries.len(), 2);
            assert_eq!(entries[0].severity, 6);
            assert_eq!(entries[0].text.as_ref(), "boot complete");
            assert_eq!(entries[0].ts, 1.0);
            assert_eq!(entries[1].severity, 4);
            assert_eq!(entries[1].text.as_ref(), "PreArm: Battery");
            assert_eq!(entries[1].ts, 2.0);
        }
        other => panic!("expected Value::TextLog, got {other:?}"),
    }
}

#[test]
fn null_value_does_not_emit_a_scalar_sample() {
    // Null top-level values are surfaced via `flatten_msgpack_with_nulls`
    // — the plain `flatten_msgpack` path simply drops them from the
    // sample stream (the ZMQ worker re-emits them as `Value::Null` for
    // the CLI's schema-only routing).
    #[derive(Serialize)]
    struct Env {
        ts: f64,
        values: BTreeMap<String, serde_json::Value>,
    }
    let mut values = BTreeMap::new();
    values.insert("ap_attitude".into(), serde_json::Value::Null);
    values.insert("flight_mode".into(), serde_json::json!("GUIDED"));
    let bytes = encode(&Env { ts: 6.0, values });

    let samples = flatten_msgpack(&bytes).expect("decode");
    assert!(
        samples.iter().all(|s| s.key != "ap_attitude"),
        "null-valued key '{}' must not produce a sample in samples={:?}",
        "ap_attitude",
        samples_keys(&samples),
    );
    // The non-null sibling still made it through.
    let _ = find(&samples, "flight_mode");
}

#[test]
fn integer_scalar_decodes_to_value_scalar() {
    // `fix_type` is an int but should land as a scalar (not IntVector,
    // because it's not in an array).
    #[derive(Serialize)]
    struct Env {
        ts: f64,
        values: BTreeMap<String, serde_json::Value>,
    }
    let mut values = BTreeMap::new();
    values.insert("fix_type".into(), serde_json::json!(3_i64));
    let bytes = encode(&Env { ts: 7.0, values });
    let samples = flatten_msgpack(&bytes).expect("decode");
    let s = find(&samples, "fix_type");
    assert_eq!(s.value, Value::Scalar(3.0));
}
