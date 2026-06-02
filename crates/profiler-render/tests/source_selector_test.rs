//! v0.15.0 — per-cell source selector tests.
//!
//! The Add Panel modal's source dropdown writes the chosen URI into
//! `PanelDraft.source_uri`; on Apply, `apply_panel_draft` stamps that URI
//! onto the cell's primary source. This test exercises that contract
//! headlessly (no egui) against the public editor API.
//!
//! Also covers:
//! - The `(any)` default (`source_uri == ""`) leaves the cell's
//!   `CellSource.source_uri` as `None` so the JSON round-trip omits the
//!   field for templates that don't pin to a source.
//! - Overlay extras inherit the cell's `source_uri` so a single
//!   "Source: zmq://...:9005" pick applies to every line in the overlay.

use profiler_render::{apply_panel_draft, PanelDraft};
use profiler_template::{Primitive, Template};

fn empty_template() -> Template {
    let json = r#"{
        "name": "empty",
        "grid": {"rows": 3, "cols": 3},
        "cells": []
    }"#;
    Template::from_str(json).expect("parse")
}

#[test]
fn apply_panel_draft_stamps_source_uri_when_pinned() {
    // Operator picks `zmq://127.0.0.1:9005` in the source dropdown, then
    // adds a scalar panel. The CellSource's `source_uri` field must
    // round-trip the chosen URI verbatim.
    let mut tpl = empty_template();
    let draft = PanelDraft {
        row: 0,
        col: 0,
        primitive: Primitive::Scalar,
        source_key: "ap_attitude[0]".into(),
        source_uri: "zmq://127.0.0.1:9005".into(),
        ..Default::default()
    };
    apply_panel_draft(&mut tpl, &draft).expect("apply");
    assert_eq!(tpl.cells.len(), 1);
    let cell = &tpl.cells[0];
    assert_eq!(cell.sources.len(), 1);
    assert_eq!(cell.sources[0].key, "ap_attitude[0]");
    assert_eq!(
        cell.sources[0].source_uri.as_deref(),
        Some("zmq://127.0.0.1:9005"),
        "non-empty draft source_uri stamps onto CellSource",
    );
}

#[test]
fn apply_panel_draft_leaves_source_uri_none_for_any() {
    // Operator leaves the source dropdown on `(any)`; `draft.source_uri` is
    // an empty string. The cell's `source_uri` field must be `None` so
    // existing templates that omit the field round-trip byte-for-byte.
    let mut tpl = empty_template();
    let draft = PanelDraft {
        row: 1,
        col: 0,
        primitive: Primitive::Scalar,
        source_key: "ap_vfr_alt".into(),
        source_uri: String::new(), // `(any)` default
        ..Default::default()
    };
    apply_panel_draft(&mut tpl, &draft).expect("apply");
    assert!(
        tpl.cells[0].sources[0].source_uri.is_none(),
        "empty draft.source_uri => CellSource.source_uri = None",
    );
}

#[test]
fn apply_panel_draft_strips_whitespace_from_source_uri() {
    // Trailing spaces in the URI input shouldn't slip into the template.
    let mut tpl = empty_template();
    let draft = PanelDraft {
        row: 0,
        col: 0,
        source_key: "x".into(),
        source_uri: "  zmq://127.0.0.1:9006  ".into(),
        ..Default::default()
    };
    apply_panel_draft(&mut tpl, &draft).expect("apply");
    assert_eq!(
        tpl.cells[0].sources[0].source_uri.as_deref(),
        Some("zmq://127.0.0.1:9006"),
    );
}

#[test]
fn overlay_extras_inherit_source_uri_pin() {
    // Add an overlay with one primary + two extra keys, pinned to a single
    // source. Every CellSource entry should carry the same source_uri so
    // the renderer reads all three lines from the same source.
    let mut tpl = empty_template();
    let draft = PanelDraft {
        row: 0,
        col: 0,
        primitive: Primitive::Overlay,
        source_key: "pos_truth_ned[0]".into(),
        source_uri: "zmq://127.0.0.1:9005".into(),
        overlay_extra_keys: vec!["pos_ekf_ned[0]".into(), "pos_gps_ned[0]".into()],
        ..Default::default()
    };
    apply_panel_draft(&mut tpl, &draft).expect("apply");
    let cell = &tpl.cells[0];
    assert_eq!(cell.sources.len(), 3, "primary + 2 extras");
    for src in &cell.sources {
        assert_eq!(
            src.source_uri.as_deref(),
            Some("zmq://127.0.0.1:9005"),
            "every overlay entry inherits the cell's source_uri",
        );
    }
}

#[test]
fn template_with_source_uri_round_trips_through_json() {
    // Templates saved with explicit `source_uri` values must round-trip
    // through `Template::to_pretty_json` + `Template::from_str` without
    // dropping the field.
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft {
            row: 0,
            col: 0,
            source_key: "ap_attitude[0]".into(),
            source_uri: "zmq://127.0.0.1:9005".into(),
            ..Default::default()
        },
    )
    .expect("apply");
    let json = tpl.to_pretty_json().expect("serialise");
    assert!(
        json.contains("\"source_uri\""),
        "serialised JSON contains the source_uri field",
    );
    assert!(
        json.contains("zmq://127.0.0.1:9005"),
        "serialised JSON preserves the URI value",
    );
    // And the round-trip lands the same field back on the cell.
    let parsed = Template::from_str(&json).expect("re-parse");
    assert_eq!(
        parsed.cells[0].sources[0].source_uri.as_deref(),
        Some("zmq://127.0.0.1:9005"),
    );
}
