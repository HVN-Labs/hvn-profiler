//! v0.10.1 — Edit modal honours the form's Row/Col fields.
//!
//! Pre-v0.10.1, `replace_cell_at(tpl, row, col, &draft)` overwrote the
//! draft's `row` and `col` with the menu-invocation coordinates, so the
//! form's Row/Col DragValues were silently ignored. v0.10.1 honours the
//! draft's coordinates: if they differ from the menu-invocation slot, the
//! original cell is removed and the new entry is inserted at the destination.
//! Collisions with an unrelated cell at the destination return `Err` and
//! preserve all state so the modal can stay open with a status-bar message.

use profiler_render::{apply_panel_draft, replace_cell_at, PanelDraft};
use profiler_template::Template;

fn empty_template() -> Template {
    let json = r#"{
        "name": "relocate-fixture",
        "grid": {"rows": 4, "cols": 4},
        "cells": []
    }"#;
    Template::from_str(json).unwrap()
}

#[test]
fn edit_relocates_cell_when_form_row_col_changes() {
    let mut tpl = empty_template();
    // Seed a cell at (2, 1) — the menu-invocation slot.
    apply_panel_draft(
        &mut tpl,
        &PanelDraft {
            row: 2,
            col: 1,
            source_key: "alt".into(),
            title: "Altitude".into(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(tpl.cells.len(), 1);

    // Operator right-clicks (2,1) → "Edit panel..." → modal opens pre-filled
    // → operator drags Row to 3 and Col to 2 → clicks Apply.
    let draft = PanelDraft {
        row: 3,
        col: 2,
        source_key: "alt".into(),
        title: "Altitude".into(),
        ..Default::default()
    };
    replace_cell_at(&mut tpl, 2, 1, &draft).expect("relocate succeeds");

    assert_eq!(tpl.cells.len(), 1, "still one cell — moved, not duplicated");
    assert_eq!((tpl.cells[0].row, tpl.cells[0].col), (3, 2));
    assert!(
        !tpl.cells.iter().any(|c| (c.row, c.col) == (2, 1)),
        "original (2,1) slot is empty after relocation"
    );
    assert_eq!(tpl.cells[0].sources[0].key, "alt");
}

#[test]
fn relocation_onto_occupied_slot_errors_and_preserves_state() {
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft { row: 0, col: 0, source_key: "a".into(), ..Default::default() },
    )
    .unwrap();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft { row: 1, col: 1, source_key: "b".into(), ..Default::default() },
    )
    .unwrap();
    assert_eq!(tpl.cells.len(), 2);

    // Operator opens Edit on (0,0), changes Row/Col to (1,1) — collision.
    let draft = PanelDraft {
        row: 1,
        col: 1,
        source_key: "a".into(),
        ..Default::default()
    };
    let err = replace_cell_at(&mut tpl, 0, 0, &draft).unwrap_err();
    assert!(err.contains("occupied"), "error mentions occupancy: {err}");

    // Both original cells preserved — the modal can stay open with the
    // error message and let the operator pick a different slot.
    assert_eq!(tpl.cells.len(), 2);
    assert!(tpl.cells.iter().any(|c| (c.row, c.col) == (0, 0) && c.sources[0].key == "a"));
    assert!(tpl.cells.iter().any(|c| (c.row, c.col) == (1, 1) && c.sources[0].key == "b"));
}

#[test]
fn pure_replace_when_form_row_col_unchanged_keeps_behaviour() {
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft {
            row: 1,
            col: 1,
            source_key: "old".into(),
            title: "Old".into(),
            ..Default::default()
        },
    )
    .unwrap();

    // Edit modal: row/col unchanged, only source_key swapped.
    let draft = PanelDraft {
        row: 1,
        col: 1,
        source_key: "new".into(),
        title: "New".into(),
        ..Default::default()
    };
    replace_cell_at(&mut tpl, 1, 1, &draft).unwrap();
    assert_eq!(tpl.cells.len(), 1);
    assert_eq!((tpl.cells[0].row, tpl.cells[0].col), (1, 1));
    assert_eq!(tpl.cells[0].sources[0].key, "new");
    assert_eq!(tpl.cells[0].title, "New");
}
