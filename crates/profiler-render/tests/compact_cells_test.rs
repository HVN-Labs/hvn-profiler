//! v0.10.2 — panel auto-arrange after delete + hide-time visible reflow.
//!
//! `compact_cells` re-packs the template's `cells` array into a tightly-
//! packed grid (top-to-bottom, left-to-right) so that deleting a cell from
//! the middle of the grid no longer leaves a visible gap.
//!
//! The "Hide panel" reflow lives in `render_template_grid_full` (driven by
//! `GridRenderOptions::compact_hidden`) — the template is NOT mutated, but
//! the renderer skips hidden cells when laying out, so visible cells fill
//! the gap. We exercise the visible-cell ordering directly here by mirroring
//! the renderer's filter+sort step.

use profiler_render::{apply_panel_draft, compact_cells, remove_cell_at, PanelDraft};
use profiler_template::Template;

fn empty_template(rows: usize, cols: usize) -> Template {
    let json = format!(
        r#"{{
            "name": "compact-fixture",
            "grid": {{"rows": {rows}, "cols": {cols}}},
            "cells": []
        }}"#
    );
    Template::from_str(&json).unwrap()
}

fn seed_cell(tpl: &mut Template, row: usize, col: usize, key: &str) {
    apply_panel_draft(
        tpl,
        &PanelDraft {
            row,
            col,
            source_key: key.into(),
            ..Default::default()
        },
    )
    .unwrap_or_else(|e| panic!("seed ({row}, {col}) failed: {e}"));
}

/// Make a template with cells at `(0,0), (0,2), (1,1), (2,0)`; delete `(0,2)`;
/// assert the remaining 3 cells now occupy `(0,0), (0,1), (0,2)` and
/// `grid.rows == 1`.
#[test]
fn delete_middle_reflows_remaining_three_into_first_row() {
    let mut tpl = empty_template(3, 3);
    seed_cell(&mut tpl, 0, 0, "a");
    seed_cell(&mut tpl, 0, 2, "b"); // to be deleted
    seed_cell(&mut tpl, 1, 1, "c");
    seed_cell(&mut tpl, 2, 0, "d");

    assert_eq!(tpl.cells.len(), 4);

    // Delete (0, 2) — the "b" cell — then compact.
    remove_cell_at(&mut tpl, 0, 2).unwrap();
    compact_cells(&mut tpl);

    // 3 cells survive, and they pack into row 0 since cols == 3.
    assert_eq!(tpl.cells.len(), 3, "delete dropped one cell");
    let mut coords: Vec<(usize, usize)> = tpl.cells.iter().map(|c| (c.row, c.col)).collect();
    coords.sort();
    assert_eq!(coords, vec![(0, 0), (0, 1), (0, 2)]);
    assert_eq!(tpl.grid.rows, 1, "grid shrinks to fit the survivors");
    assert_eq!(tpl.grid.cols, 3, "cols preserved across reflow");

    // Visual ordering is preserved: the survivors keep their pre-delete
    // top-to-bottom / left-to-right sequence (a, c, d).
    let keys: Vec<&str> = tpl
        .cells
        .iter()
        .map(|c| c.sources[0].key.as_str())
        .collect();
    assert_eq!(keys, vec!["a", "c", "d"]);
}

/// Make a template with 5 cells; hide one (via the visibility-override map,
/// matching the CLI's "Hide panel" flow); assert visible cells reflow to fill
/// the gap. This mirrors the filter+sort step inside `render_template_grid_full`
/// when `compact_hidden = true`.
#[test]
fn hide_one_of_five_reflows_visible_cells_to_fill_gap() {
    use std::collections::HashMap;

    let mut tpl = empty_template(2, 3);
    seed_cell(&mut tpl, 0, 0, "a");
    seed_cell(&mut tpl, 0, 1, "b");
    seed_cell(&mut tpl, 0, 2, "c");
    seed_cell(&mut tpl, 1, 0, "d");
    seed_cell(&mut tpl, 1, 1, "e");

    // Hide the middle cell ("b" at (0, 1)).
    let mut visibility: HashMap<(usize, usize), bool> = HashMap::new();
    visibility.insert((0, 1), false);

    // Renderer-equivalent reflow: keep `cell.visible && override.unwrap(true)`
    // and sort by (row, col). Pack into a layout grid with the same `cols`.
    let cols = tpl.grid.cols;
    let mut visible: Vec<_> = tpl
        .cells
        .iter()
        .filter(|c| {
            c.visible && *visibility.get(&(c.row, c.col)).unwrap_or(&true)
        })
        .collect();
    visible.sort_by_key(|c| (c.row, c.col));
    assert_eq!(visible.len(), 4, "hide drops one visible cell");

    // The remaining 4 cells should occupy (0,0), (0,1), (0,2), (1,0).
    let packed: Vec<(usize, usize, &str)> = visible
        .iter()
        .enumerate()
        .map(|(i, c)| (i / cols, i % cols, c.sources[0].key.as_str()))
        .collect();
    assert_eq!(
        packed,
        vec![(0, 0, "a"), (0, 1, "c"), (0, 2, "d"), (1, 0, "e")],
        "visible cells reflow top-to-bottom, left-to-right; the hidden 'b' \
         slot disappears"
    );

    // The template itself is NOT mutated by Hide — hidden cells retain their
    // original coordinates so "Restore" puts them back in place.
    assert_eq!(tpl.cells.len(), 5, "template still owns all 5 cells");
    let b = tpl.cells.iter().find(|c| c.sources[0].key == "b").unwrap();
    assert_eq!((b.row, b.col), (0, 1), "hidden cell keeps its original slot");
}

/// `compact_cells` is idempotent: running it twice equals running it once.
#[test]
fn compact_cells_is_idempotent() {
    let mut tpl = empty_template(4, 2);
    seed_cell(&mut tpl, 0, 0, "a");
    seed_cell(&mut tpl, 2, 1, "b");
    seed_cell(&mut tpl, 3, 0, "c");

    compact_cells(&mut tpl);
    let once: Vec<(usize, usize, String)> = tpl
        .cells
        .iter()
        .map(|c| (c.row, c.col, c.sources[0].key.clone()))
        .collect();
    let rows_once = tpl.grid.rows;

    compact_cells(&mut tpl);
    let twice: Vec<(usize, usize, String)> = tpl
        .cells
        .iter()
        .map(|c| (c.row, c.col, c.sources[0].key.clone()))
        .collect();
    assert_eq!(once, twice);
    assert_eq!(rows_once, tpl.grid.rows);
}

/// Empty templates survive a compact_cells call — guards against div-by-zero
/// when computing `grid.rows`.
#[test]
fn compact_cells_handles_empty_template() {
    let mut tpl = empty_template(5, 3);
    compact_cells(&mut tpl);
    assert!(tpl.cells.is_empty());
    assert_eq!(tpl.grid.rows, 1, "rows clamp to at-least-1 when empty");
    assert_eq!(tpl.grid.cols, 3);
}

/// After delete + compact, `grid.cols` is preserved so the operator's chosen
/// column count survives reflow.
#[test]
fn compact_cells_preserves_cols() {
    let mut tpl = empty_template(3, 4);
    seed_cell(&mut tpl, 0, 0, "a");
    seed_cell(&mut tpl, 1, 2, "b");
    remove_cell_at(&mut tpl, 0, 0).unwrap();
    compact_cells(&mut tpl);
    assert_eq!(tpl.grid.cols, 4);
    assert_eq!(tpl.cells.len(), 1);
    assert_eq!((tpl.cells[0].row, tpl.cells[0].col), (0, 0));
}
