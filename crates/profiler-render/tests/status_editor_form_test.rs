//! v0.13.0 — Status-cell editor form: when the operator picks a Status
//! primitive in the Add/Edit Panel modal, the form exposes a kind
//! selector + color_map row editor + default-color field. This file
//! pins the data contract that the egui form commits to on `Apply`:
//! `apply_panel_draft` must thread the Status fields into the resulting
//! `Cell` so the renderer's `status_cell_color` can resolve them.

use profiler_render::{apply_panel_draft, default_status_kind, PanelDraft, ValueShape};
use profiler_template::{Primitive, StatusKind, Template};

fn empty_template() -> Template {
    let json = r#"{
        "name": "empty",
        "grid": {"rows": 3, "cols": 3},
        "cells": []
    }"#;
    Template::from_str(json).expect("parse")
}

#[test]
fn add_panel_status_cell_threads_kind_and_color_map() {
    let mut tpl = empty_template();
    // Simulate the form: programmatically add three color_map rows.
    let mut draft = PanelDraft {
        row: 0,
        col: 0,
        primitive: Primitive::Status,
        title: "Mode".into(),
        source_key: "flight_mode".into(),
        status_kind: StatusKind::Text,
        status_default_color: "#aaaaaa".into(),
        ..Default::default()
    };
    // `+ Add row` clicked three times — append three entries.
    draft
        .status_color_map
        .push(("GUIDED".into(), "#1f77b4".into()));
    draft
        .status_color_map
        .push(("LOITER".into(), "#2ca02c".into()));
    draft
        .status_color_map
        .push(("RTL".into(), "#d62728".into()));

    apply_panel_draft(&mut tpl, &draft).expect("apply");
    assert_eq!(tpl.cells.len(), 1);

    let cell = &tpl.cells[0];
    assert_eq!(cell.primitive, Primitive::Status);
    assert_eq!(cell.source, "flight_mode");
    assert_eq!(cell.kind, Some(StatusKind::Text));
    assert_eq!(cell.default_color.as_deref(), Some("#aaaaaa"));
    assert_eq!(cell.color_map.len(), 3, "three color_map rows survive");
    assert_eq!(cell.color_map.get("GUIDED"), Some(&"#1f77b4".to_string()));
    assert_eq!(cell.color_map.get("LOITER"), Some(&"#2ca02c".to_string()));
    assert_eq!(cell.color_map.get("RTL"), Some(&"#d62728".to_string()));
}

#[test]
fn add_panel_status_cell_drops_blank_color_map_rows() {
    // The `+ Add row` button can leave blank rows on screen; the apply
    // path must drop them rather than serialising `"": "#xxx"` into the
    // template (which would never match a real value).
    let mut tpl = empty_template();
    let draft = PanelDraft {
        row: 0,
        col: 0,
        primitive: Primitive::Status,
        source_key: "flight_mode".into(),
        status_kind: StatusKind::Text,
        status_color_map: vec![
            ("GUIDED".into(), "#1f77b4".into()),
            (String::new(), "#000".into()),
            ("RTL".into(), String::new()),
            ("LOITER".into(), "#2ca02c".into()),
        ],
        ..Default::default()
    };

    apply_panel_draft(&mut tpl, &draft).expect("apply");
    let cell = &tpl.cells[0];
    assert_eq!(
        cell.color_map.len(),
        2,
        "blank value/color rows are dropped: {:?}",
        cell.color_map,
    );
    assert!(cell.color_map.contains_key("GUIDED"));
    assert!(cell.color_map.contains_key("LOITER"));
}

#[test]
fn add_panel_non_status_cell_has_no_status_fields() {
    let mut tpl = empty_template();
    let draft = PanelDraft {
        row: 0,
        col: 0,
        primitive: Primitive::Scalar,
        source_key: "ap_attitude[0]".into(),
        // These should be ignored when primitive != Status.
        status_kind: StatusKind::ArmedBool,
        status_default_color: "#abcdef".into(),
        status_color_map: vec![("k".into(), "v".into())],
        ..Default::default()
    };
    apply_panel_draft(&mut tpl, &draft).expect("apply");
    let cell = &tpl.cells[0];
    assert_eq!(cell.primitive, Primitive::Scalar);
    assert!(
        cell.kind.is_none(),
        "non-Status cells must not carry a StatusKind: {:?}",
        cell.kind,
    );
    assert!(cell.color_map.is_empty());
    assert!(cell.default_color.is_none());
    assert!(cell.source.is_empty());
}

#[test]
fn default_status_kind_picks_name_based_default() {
    // Name-based heuristic wins over the shape fallback.
    assert_eq!(
        default_status_kind("armed", &ValueShape::Bool),
        Some(StatusKind::ArmedBool),
    );
    assert_eq!(
        default_status_kind("fix_type", &ValueShape::Scalar),
        Some(StatusKind::FixType),
    );
    assert_eq!(
        default_status_kind("statustexts", &ValueShape::TextLog),
        Some(StatusKind::TextLog),
    );
    // Pure shape fallback when the key name doesn't match any known
    // pattern.
    assert_eq!(
        default_status_kind("any_string_key", &ValueShape::String),
        Some(StatusKind::Text),
    );
    assert_eq!(
        default_status_kind("any_bool_key", &ValueShape::Bool),
        Some(StatusKind::ArmedBool),
    );
    // Neither name nor shape match → caller keeps the current kind.
    assert_eq!(
        default_status_kind("some_scalar", &ValueShape::Scalar),
        None,
    );
}
