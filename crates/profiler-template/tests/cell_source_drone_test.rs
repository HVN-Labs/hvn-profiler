//! v0.16.8 — round-trip `CellSource::source_drone` through JSON to confirm
//! the optional drone-pin field serialises and deserialises cleanly without
//! breaking templates that omit it.
//!
//! The render-side resolver tests live in
//! `crates/profiler-render/tests/per_cell_routing_test.rs` — this test only
//! covers the wire format guarantee.

use profiler_template::CellSource;

#[test]
fn source_drone_round_trips() {
    // Serialise a CellSource with a drone-pin set, then parse it back and
    // assert the field survived.
    let src = CellSource {
        key: "ap_attitude[0]".into(),
        source_drone: Some("eric_1".into()),
        ..Default::default()
    };
    let json = serde_json::to_string(&src).expect("serialise");
    assert!(
        json.contains("\"source_drone\":\"eric_1\""),
        "serialised JSON should carry source_drone: {json}",
    );
    let parsed: CellSource = serde_json::from_str(&json).expect("deserialise");
    assert_eq!(
        parsed.source_drone.as_deref(),
        Some("eric_1"),
        "deserialised source_drone must equal the original",
    );
}

#[test]
fn missing_source_drone_deserialises_as_none() {
    // Templates predating v0.16.8 omit the field entirely. They must load
    // as `source_drone: None` (the `(any)` default) so the renderer falls
    // back to the URI-pin / view-drone resolution chain.
    let json = r#"{"key":"ap_attitude[0]"}"#;
    let parsed: CellSource = serde_json::from_str(json).expect("deserialise legacy");
    assert!(
        parsed.source_drone.is_none(),
        "legacy template (no source_drone) must deserialise to None",
    );
}

#[test]
fn none_source_drone_skipped_on_serialise() {
    // `#[serde(skip_serializing_if = "Option::is_none")]` — a `None` drone-pin
    // must NOT appear in the serialised JSON, so templates remain compact and
    // round-trip-clean for the common case (cell with no drone-pin).
    let src = CellSource {
        key: "ap_attitude[0]".into(),
        source_drone: None,
        ..Default::default()
    };
    let json = serde_json::to_string(&src).expect("serialise");
    assert!(
        !json.contains("source_drone"),
        "None drone-pin must be elided from JSON: {json}",
    );
}
