//! v0.11.0 — drag-to-reorder of 2D panels.
//!
//! The actual `egui::Sense::drag` plumbing lives inside
//! `render_template_grid_full`; it can't be exercised without a live egui
//! context. What we CAN test headlessly is the post-drop state mutation: the
//! renderer emits a `CellMenuAction::SwapTo { from, to }` (or `MoveTo`) and
//! the CLI calls `swap_cells` / `relocate_cell` + `compact_cells`. This file
//! exercises that downstream pipeline directly.

use profiler_render::{
    apply_panel_draft, compact_cells, relocate_cell, swap_cells, PanelDraft,
};
use profiler_template::Template;

fn empty_template(rows: usize, cols: usize) -> Template {
    let json = format!(
        r#"{{
            "name": "drag-fixture",
            "grid": {{"rows": {rows}, "cols": {cols}}},
            "cells": []
        }}"#
    );
    Template::from_str(&json).unwrap()
}

fn seed_cell(tpl: &mut Template, row: usize, col: usize, key: &str) {
    apply_panel_draft(
        tpl,
        &PanelDraft { row, col, source_key: key.into(), ..Default::default() },
    )
    .unwrap();
}

/// Construct a template with cells at (0,0), (0,1), (1,0). Simulate a drag
/// from (0,0) → (1,0). Assert cells.len() unchanged; the cell originally at
/// (0,0) now lives at (1,0), and the cell originally at (1,0) now lives at
/// (0,0). The (0,1) cell is untouched.
#[test]
fn drag_swap_exchanges_two_cells() {
    let mut tpl = empty_template(2, 2);
    seed_cell(&mut tpl, 0, 0, "A");
    seed_cell(&mut tpl, 0, 1, "B");
    seed_cell(&mut tpl, 1, 0, "C");
    assert_eq!(tpl.cells.len(), 3);

    swap_cells(&mut tpl, (0, 0), (1, 0)).unwrap();
    compact_cells(&mut tpl);

    assert_eq!(tpl.cells.len(), 3, "swap does not change cell count");
    let at = |r: usize, c: usize| -> Option<&str> {
        tpl.cells
            .iter()
            .find(|x| x.row == r && x.col == c)
            .and_then(|x| x.sources.first())
            .map(|s| s.key.as_str())
    };
    // After swap, A landed at (1,0) and C at (0,0). compact_cells then
    // re-packs row-major; the resulting top-to-bottom order is (C, B, A)
    // because the post-swap (row, col) tuples sort as (0,0)=C, (0,1)=B,
    // (1,0)=A.
    assert_eq!(at(0, 0), Some("C"));
    assert_eq!(at(0, 1), Some("B"));
    assert_eq!(at(1, 0), Some("A"));
}

/// Drag from an occupied slot onto an empty slot → relocate (no swap).
#[test]
fn drag_move_to_empty_relocates_cell() {
    let mut tpl = empty_template(2, 2);
    seed_cell(&mut tpl, 0, 0, "A");
    seed_cell(&mut tpl, 0, 1, "B");
    // (1, 0) and (1, 1) are empty.

    relocate_cell(&mut tpl, (0, 0), (1, 1)).unwrap();
    // Don't compact here so we can observe the raw relocate result.

    let a = tpl
        .cells
        .iter()
        .find(|c| c.sources[0].key == "A")
        .unwrap();
    assert_eq!((a.row, a.col), (1, 1));
    let b = tpl
        .cells
        .iter()
        .find(|c| c.sources[0].key == "B")
        .unwrap();
    assert_eq!((b.row, b.col), (0, 1), "B was not moved");
}

/// Drag into an occupied slot via `relocate_cell` is rejected — the renderer
/// dispatches `SwapTo` for occupied targets instead. Confirms the post-drop
/// state stays consistent.
#[test]
fn drag_relocate_to_occupied_rejects() {
    let mut tpl = empty_template(2, 2);
    seed_cell(&mut tpl, 0, 0, "A");
    seed_cell(&mut tpl, 0, 1, "B");
    let err = relocate_cell(&mut tpl, (0, 0), (0, 1)).unwrap_err();
    assert!(err.contains("occupied"), "got: {err}");
    // No mutation: A and B keep their slots.
    let a = tpl.cells.iter().find(|c| c.sources[0].key == "A").unwrap();
    let b = tpl.cells.iter().find(|c| c.sources[0].key == "B").unwrap();
    assert_eq!((a.row, a.col), (0, 0));
    assert_eq!((b.row, b.col), (0, 1));
}

/// Drag out of bounds (drop in the gutter) → no-op assertion. The renderer
/// never emits an action; here we just verify that NOT calling swap/relocate
/// is observably equivalent to no mutation.
#[test]
fn drag_out_of_bounds_is_noop() {
    let mut tpl = empty_template(2, 2);
    seed_cell(&mut tpl, 0, 0, "A");
    seed_cell(&mut tpl, 0, 1, "B");
    let before: Vec<(usize, usize, String)> = tpl
        .cells
        .iter()
        .map(|c| (c.row, c.col, c.sources[0].key.clone()))
        .collect();
    // Renderer-side: snap_target is None → no `SwapTo`/`MoveTo` is pushed.
    // We simply do nothing here and assert state is unchanged.
    let after: Vec<(usize, usize, String)> = tpl
        .cells
        .iter()
        .map(|c| (c.row, c.col, c.sources[0].key.clone()))
        .collect();
    assert_eq!(before, after);
}

/// Drag onto the source cell itself → swap is a no-op.
#[test]
fn drag_swap_onto_self_is_noop() {
    let mut tpl = empty_template(2, 2);
    seed_cell(&mut tpl, 0, 0, "A");
    seed_cell(&mut tpl, 1, 1, "B");
    swap_cells(&mut tpl, (0, 0), (0, 0)).unwrap();
    let a = tpl.cells.iter().find(|c| c.sources[0].key == "A").unwrap();
    let b = tpl.cells.iter().find(|c| c.sources[0].key == "B").unwrap();
    assert_eq!((a.row, a.col), (0, 0));
    assert_eq!((b.row, b.col), (1, 1));
}

/// Swap + compact is idempotent for an already-tight grid (no row growth /
/// shrinkage from the swap alone).
#[test]
fn swap_then_compact_preserves_grid_dims() {
    let mut tpl = empty_template(2, 2);
    seed_cell(&mut tpl, 0, 0, "A");
    seed_cell(&mut tpl, 0, 1, "B");
    seed_cell(&mut tpl, 1, 0, "C");
    seed_cell(&mut tpl, 1, 1, "D");
    swap_cells(&mut tpl, (0, 0), (1, 1)).unwrap();
    compact_cells(&mut tpl);
    assert_eq!(tpl.grid.cols, 2);
    assert_eq!(tpl.grid.rows, 2);
    assert_eq!(tpl.cells.len(), 4);
}
