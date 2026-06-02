//! v0.10.0 — in-app cell editor + per-cell context menu operations.
//!
//! Exercises the pure-data apply path used by the "+ Add Panel" modal and the
//! per-cell right-click "Edit panel..." / "Delete panel" menu items. The
//! actual `egui::Window` modal is wired in `profiler-cli`; this file pins the
//! template-mutation contract that the modal commits to on click.

use profiler_render::{
    add_panel_draft, apply_panel_draft, apply_trail_draft, collect_source_keys,
    first_available_slot, remove_cell_at, replace_cell_at, PanelDraft, TraceStore, TrailDraft,
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

// v0.16.0 — fix for "adding a new panel glitches the whole screen because I
// add both at 0,0". The "+ Add Panel" modal now (a) defaults to the first
// unoccupied (row, col) and (b) rejects an Add submit onto an already-
// occupied slot. The Edit/Replace path goes through `replace_cell_at` and
// is intentionally unaffected.

#[test]
fn test_add_panel_defaults_to_first_available_slot() {
    // 2-column grid with cells at (0,0), (0,1), (1,0). Row 0 is fully
    // occupied; row 1 col 0 is taken; first gap in row-major order is
    // (1,1). That's what the Add Panel modal should default to.
    let json = r#"{
        "name": "two_col",
        "grid": {"rows": 3, "cols": 2},
        "cells": []
    }"#;
    let mut tpl = profiler_template::Template::from_str(json).unwrap();
    for (r, c, k) in &[(0usize, 0usize, "a"), (0, 1, "b"), (1, 0, "c")] {
        apply_panel_draft(
            &mut tpl,
            &PanelDraft {
                row: *r,
                col: *c,
                source_key: (*k).into(),
                ..Default::default()
            },
        )
        .unwrap();
    }
    assert_eq!(first_available_slot(&tpl), (1, 1));

    // Sanity: an empty template just returns (0, 0).
    let empty = empty_template();
    assert_eq!(first_available_slot(&empty), (0, 0));
}

#[test]
fn test_add_panel_rejects_occupied_slot() {
    // Template with cell at (0,0); submit Add at (0,0); expect error AND
    // template unchanged (no second cell appended).
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft {
            row: 0,
            col: 0,
            source_key: "a".into(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(tpl.cells.len(), 1);

    let err = add_panel_draft(
        &mut tpl,
        &PanelDraft {
            row: 0,
            col: 0,
            source_key: "b".into(),
            ..Default::default()
        },
    )
    .unwrap_err();
    assert!(
        err.contains("already exists"),
        "error mentions duplicate: {err}"
    );
    assert_eq!(tpl.cells.len(), 1, "template untouched on rejected add");
    assert_eq!(tpl.cells[0].sources[0].key, "a");
}

#[test]
fn test_add_panel_to_full_grid_appends_row() {
    // 2x2 grid fully occupied. first_available_slot returns (rows, 0) =
    // (2, 0); submitting Add at that slot auto-grows the grid to 3 rows
    // and appends the cell there.
    let json = r#"{
        "name": "two_by_two",
        "grid": {"rows": 2, "cols": 2},
        "cells": []
    }"#;
    let mut tpl = profiler_template::Template::from_str(json).unwrap();
    for (r, c) in &[(0usize, 0usize), (0, 1), (1, 0), (1, 1)] {
        apply_panel_draft(
            &mut tpl,
            &PanelDraft {
                row: *r,
                col: *c,
                source_key: format!("k{}{}", r, c),
                ..Default::default()
            },
        )
        .unwrap();
    }
    assert_eq!(tpl.cells.len(), 4);
    assert_eq!(tpl.grid.rows, 2);
    assert_eq!(tpl.grid.cols, 2);

    let (r, c) = first_available_slot(&tpl);
    assert_eq!((r, c), (2, 0), "full grid → points past the last row");

    add_panel_draft(
        &mut tpl,
        &PanelDraft {
            row: r,
            col: c,
            source_key: "appended".into(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(tpl.cells.len(), 5);
    assert_eq!(tpl.grid.rows, 3, "grid auto-grew on submit");
    assert!(tpl
        .cells
        .iter()
        .any(|cell| (cell.row, cell.col) == (2, 0) && cell.sources[0].key == "appended"));
}
