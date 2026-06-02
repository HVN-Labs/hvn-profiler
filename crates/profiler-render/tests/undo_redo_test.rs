//! v0.11.0 — undo / redo history for the in-app template editor.
//!
//! Tests the pure-data `EditHistory` struct directly. Every editor mutation
//! in `profiler-cli` calls `history.record(template.clone())` BEFORE applying
//! the change; Ctrl+Z calls `history.undo(current)` to walk back; Ctrl+Y
//! calls `history.redo(current)` to walk forward. Capacity is 64 by default.

use profiler_render::{
    apply_panel_draft, remove_cell_at, replace_cell_at, EditHistory, PanelDraft,
};
use profiler_template::Template;

fn empty_template() -> Template {
    let json = r#"{
        "name": "undo-fixture",
        "grid": {"rows": 3, "cols": 3},
        "cells": []
    }"#;
    Template::from_str(json).unwrap()
}

/// Add panel → record. Undo → cells.len() back to original.
#[test]
fn add_then_undo_drops_the_added_cell() {
    let mut tpl = empty_template();
    let mut h = EditHistory::default();

    h.record(tpl.clone());
    apply_panel_draft(
        &mut tpl,
        &PanelDraft { row: 0, col: 0, source_key: "x".into(), ..Default::default() },
    )
    .unwrap();
    assert_eq!(tpl.cells.len(), 1);

    let prev = h.undo(tpl.clone()).expect("one entry available");
    tpl = prev;
    assert_eq!(tpl.cells.len(), 0, "undo restores pre-add state");
    assert!(h.can_redo());
}

/// Add, edit, delete → 3 records. Undo 3x → original. Redo 3x → restored.
#[test]
fn three_mutations_round_trip_through_history() {
    let mut tpl = empty_template();
    let mut h = EditHistory::default();

    // 1. Add panel at (0, 0).
    h.record(tpl.clone());
    apply_panel_draft(
        &mut tpl,
        &PanelDraft { row: 0, col: 0, source_key: "x".into(), ..Default::default() },
    )
    .unwrap();
    let after_add: Vec<_> = tpl
        .cells
        .iter()
        .map(|c| (c.row, c.col, c.sources[0].key.clone()))
        .collect();

    // 2. Edit panel (replace at (0, 0) with a new key).
    h.record(tpl.clone());
    replace_cell_at(
        &mut tpl,
        0,
        0,
        &PanelDraft { row: 0, col: 0, source_key: "y".into(), ..Default::default() },
    )
    .unwrap();
    let after_edit: Vec<_> = tpl
        .cells
        .iter()
        .map(|c| (c.row, c.col, c.sources[0].key.clone()))
        .collect();
    assert_ne!(after_add, after_edit);

    // 3. Delete panel.
    h.record(tpl.clone());
    remove_cell_at(&mut tpl, 0, 0).unwrap();
    assert!(tpl.cells.is_empty());

    // Undo 3x → back to original.
    tpl = h.undo(tpl).unwrap();
    let undo1: Vec<_> = tpl
        .cells
        .iter()
        .map(|c| (c.row, c.col, c.sources[0].key.clone()))
        .collect();
    assert_eq!(undo1, after_edit, "undo 1 restores the post-edit state");

    tpl = h.undo(tpl).unwrap();
    let undo2: Vec<_> = tpl
        .cells
        .iter()
        .map(|c| (c.row, c.col, c.sources[0].key.clone()))
        .collect();
    assert_eq!(undo2, after_add, "undo 2 restores the post-add state");

    tpl = h.undo(tpl).unwrap();
    assert!(tpl.cells.is_empty(), "undo 3 restores the original empty state");
    assert!(!h.can_undo());
    assert!(h.can_redo());

    // Redo 3x → restored to the final post-delete state.
    tpl = h.redo(tpl).unwrap();
    assert_eq!(tpl.cells.len(), 1);
    assert_eq!(tpl.cells[0].sources[0].key, "x");

    tpl = h.redo(tpl).unwrap();
    assert_eq!(tpl.cells[0].sources[0].key, "y");

    tpl = h.redo(tpl).unwrap();
    assert!(tpl.cells.is_empty(), "redo 3 reaches the final post-delete state");
    assert!(!h.can_redo());
}

/// Capacity overflow: push 70 records into a 64-slot history; assert the
/// oldest 6 are evicted (only the 64 most recent survive).
#[test]
fn capacity_overflow_evicts_oldest() {
    let mut h = EditHistory::new(64);
    for i in 0..70 {
        let mut tpl = empty_template();
        // Stash the index into the template name so we can identify which
        // snapshot survived.
        tpl.name = format!("snapshot-{i}");
        h.record(tpl);
    }
    assert_eq!(h.past_len(), 64, "history clamped to capacity");

    // Walk all the way back. The first undo returns the most recent record
    // (snapshot-69)… so to see the OLDEST surviving record we need to undo
    // 64 times. After 64 undos the past stack is empty.
    let mut tpl = empty_template();
    tpl.name = "current".into();
    let mut last = None;
    for _ in 0..64 {
        tpl = h.undo(tpl).expect("entry available");
        last = Some(tpl.name.clone());
    }
    assert!(!h.can_undo(), "exactly 64 undos consume the past stack");
    // The 64th undo surfaces the OLDEST surviving snapshot. 70 records
    // pushed, capacity 64 → the oldest is index 70 - 64 = 6.
    assert_eq!(last.as_deref(), Some("snapshot-6"));
}

/// New record clears the redo stack (linear history). Branching from an
/// undone state discards the previous future.
#[test]
fn record_after_undo_clears_redo_stack() {
    let mut tpl = empty_template();
    let mut h = EditHistory::default();

    h.record(tpl.clone());
    apply_panel_draft(
        &mut tpl,
        &PanelDraft { row: 0, col: 0, source_key: "x".into(), ..Default::default() },
    )
    .unwrap();
    tpl = h.undo(tpl).unwrap();
    assert!(h.can_redo());

    // Branch from the undone state: a fresh mutation discards the redo.
    h.record(tpl.clone());
    apply_panel_draft(
        &mut tpl,
        &PanelDraft { row: 1, col: 1, source_key: "z".into(), ..Default::default() },
    )
    .unwrap();
    assert!(!h.can_redo(), "branching clears the redo stack");
}

/// Calling `undo` on an empty history returns `None` and leaves the caller's
/// template untouched (the CLI's `apply_undo` puts it back).
#[test]
fn undo_on_empty_history_returns_none() {
    let mut h = EditHistory::default();
    let tpl = empty_template();
    assert!(h.undo(tpl).is_none());
    assert!(!h.can_undo());
    assert!(!h.can_redo());
}
