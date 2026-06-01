//! v0.10.0 — in-app cell editor + per-cell context menu operations.
//!
//! Exercises the pure-data apply path used by the "+ Add Panel" modal and the
//! per-cell right-click "Edit panel..." / "Delete panel" menu items. The
//! actual `egui::Window` modal is wired in `profiler-cli`; this file pins the
//! template-mutation contract that the modal commits to on click.

use profiler_render::{
    apply_panel_draft, apply_trail_draft, collect_source_keys, remove_cell_at, replace_cell_at,
    PanelDraft, TraceStore, TrailDraft,
};
use profiler_template::{LabelMode, Primitive, Template};

fn empty_template() -> Template {
    let json = r#"{
        "name": "blank",
        "grid": {"rows": 4, "cols": 3},
        "cells": []
    }"#;
    Template::from_str(json).expect("parse blank template")
}

#[test]
fn add_panel_appends_cell_with_right_shape() {
    let mut tpl = empty_template();
    let draft = PanelDraft {
        row: 2,
        col: 1,
        primitive: Primitive::Scalar,
        title: "Altitude".into(),
        source_key: "gps_alt".into(),
        fallback: "ap_vfr_alt".into(),
        color: "#1f77b4".into(),
        label_mode: LabelMode::Data,
        ..Default::default()
    };

    apply_panel_draft(&mut tpl, &draft).expect("apply");
    assert_eq!(tpl.cells.len(), 1, "one cell appended");

    let cell = &tpl.cells[0];
    assert_eq!((cell.row, cell.col), (2, 1));
    assert_eq!(cell.title, "Altitude");
    assert_eq!(cell.primitive, Primitive::Scalar);
    assert_eq!(cell.label_mode, LabelMode::Data);
    assert!(cell.visible);
    assert_eq!(cell.sources.len(), 1);
    assert_eq!(cell.sources[0].key, "gps_alt");
    assert_eq!(cell.sources[0].fallback.as_deref(), Some("ap_vfr_alt"));
    assert_eq!(cell.sources[0].color, "#1f77b4");
}

#[test]
fn add_panel_overlay_packs_multiple_keys() {
    let mut tpl = empty_template();
    let draft = PanelDraft {
        row: 0,
        col: 0,
        primitive: Primitive::Overlay,
        source_key: "truth_pos[2]".into(),
        overlay_extra_keys: vec!["ekf_pos[2]".into(), "gps_alt".into()],
        ..Default::default()
    };
    apply_panel_draft(&mut tpl, &draft).unwrap();
    let cell = &tpl.cells[0];
    assert_eq!(cell.primitive, Primitive::Overlay);
    assert_eq!(cell.sources.len(), 3);
    let keys: Vec<&str> = cell.sources.iter().map(|s| s.key.as_str()).collect();
    assert_eq!(keys, vec!["truth_pos[2]", "ekf_pos[2]", "gps_alt"]);
}

#[test]
fn edit_panel_replaces_existing_cell_in_place() {
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft {
            row: 0,
            col: 0,
            source_key: "old".into(),
            title: "Old".into(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(tpl.cells.len(), 1);

    // "Edit panel..." → modal pre-fills, operator changes the key.
    replace_cell_at(
        &mut tpl,
        0,
        0,
        &PanelDraft {
            source_key: "new".into(),
            title: "New".into(),
            ..Default::default()
        },
    )
    .unwrap();
    // Still one cell at (0,0) — replaced, not appended.
    assert_eq!(tpl.cells.len(), 1);
    assert_eq!(tpl.cells[0].sources[0].key, "new");
    assert_eq!(tpl.cells[0].title, "New");
}

#[test]
fn delete_panel_removes_cell_from_template() {
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft { row: 1, col: 2, source_key: "x".into(), ..Default::default() },
    )
    .unwrap();
    assert_eq!(tpl.cells.len(), 1);
    remove_cell_at(&mut tpl, 1, 2).unwrap();
    assert!(tpl.cells.is_empty());
}

#[test]
fn add_trail_appends_to_view_3d() {
    let mut tpl = empty_template();
    let draft = TrailDraft {
        name: "mine".into(),
        label: "Mine".into(),
        color: "#9467bd".into(),
        x_key: "pos_truth_ned[1]".into(),
        y_key: "pos_truth_ned[0]".into(),
        z_neg_key: "pos_truth_ned[2]".into(),
        ..Default::default()
    };
    apply_trail_draft(&mut tpl, &draft).unwrap();
    let trails = &tpl.view_3d.as_ref().unwrap().trails;
    assert_eq!(trails.len(), 1);
    assert_eq!(trails[0].name, "mine");
    let src = trails[0].sources.as_ref().unwrap();
    assert_eq!(src.x, "pos_truth_ned[1]");
}

#[test]
fn source_key_dropdown_collects_observed_keys() {
    // Two drones, distinct stores.
    let mut a = TraceStore::new(60.0);
    a.push(0.0, "ap_attitude[0]", 0.0);
    a.push(0.0, "ap_attitude[1]", 0.0);
    let mut b = TraceStore::new(60.0);
    b.push(0.0, "gps_alt", 0.0);
    b.push(0.0, "ap_vfr_alt", 0.0);

    let keys = collect_source_keys(&[a, b]);
    assert!(keys.contains(&"ap_attitude[0]".to_string()));
    assert!(keys.contains(&"ap_attitude[1]".to_string()));
    assert!(keys.contains(&"ap_attitude".to_string()), "base name included for vector primitives");
    assert!(keys.contains(&"gps_alt".to_string()));
    assert!(keys.contains(&"ap_vfr_alt".to_string()));
    // Sorted (BTreeSet contract).
    let mut sorted = keys.clone();
    sorted.sort();
    assert_eq!(keys, sorted);
}

#[test]
fn add_panel_dirties_template_via_save_serialisation() {
    // After Add, `to_pretty_json` must include the new cell's source key.
    // This pairs with the dirty-flag flow in profiler-cli: the toolbar shows
    // `●` until Ctrl+S writes the augmented template back.
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft {
            row: 0,
            col: 0,
            source_key: "fresh_key".into(),
            title: "Fresh".into(),
            ..Default::default()
        },
    )
    .unwrap();
    let json = tpl.to_pretty_json().unwrap();
    assert!(json.contains("fresh_key"));
    assert!(json.contains("Fresh"));
}
