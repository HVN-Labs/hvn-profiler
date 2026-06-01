//! v0.10.0 — per-cell right-click context menu actions.
//!
//! The egui_plot right-click menu emits a typed [`CellMenuAction`] for each
//! item. The CLI applies them between frames: "Hide panel" flips a
//! visibility bit in `UiState`, "Reset zoom" clears the `PanelState.locked`
//! flag, "Edit panel..." opens the editor pre-filled, "Delete panel" drops
//! the cell from the template after a confirm.
//!
//! End-to-end clicking requires a real `egui` event pipeline; these tests
//! lock the ACTION→EFFECT mapping at the data layer, which is what the CLI
//! commits to when it consumes the action queue.

use std::collections::HashMap;

use profiler_render::{
    apply_panel_draft, remove_cell_at, CellMenuAction, PanelDraft, PanelState,
};
use profiler_template::{LabelMode, Template};

fn empty_template() -> Template {
    let json = r#"{
        "name": "ctx-menu-fixture",
        "grid": {"rows": 2, "cols": 2},
        "cells": []
    }"#;
    Template::from_str(json).unwrap()
}

/// Apply a `CellMenuAction` against the CLI's state model — mirror of
/// the App-side handler in profiler-cli. Returns `true` when the action
/// dirtied the template (CLI sets `template_dirty = true` on the same
/// transitions).
fn apply_action(
    action: &CellMenuAction,
    tpl: &mut Template,
    visibility: &mut HashMap<(usize, usize), bool>,
    panel_states: &mut HashMap<(usize, usize), PanelState>,
    label_overrides: &mut HashMap<(usize, usize), LabelMode>,
) -> bool {
    match action {
        CellMenuAction::HideToggle { row, col } => {
            let cur = visibility.get(&(*row, *col)).copied().unwrap_or(true);
            visibility.insert((*row, *col), !cur);
            true
        }
        CellMenuAction::ResetZoom { row, col } => {
            if let Some(st) = panel_states.get_mut(&(*row, *col)) {
                st.locked = false;
            }
            false // pure UI — never dirties the template
        }
        CellMenuAction::SetLabelMode { row, col, mode } => {
            label_overrides.insert((*row, *col), *mode);
            true
        }
        CellMenuAction::Delete { row, col } => remove_cell_at(tpl, *row, *col).is_ok(),
        CellMenuAction::Edit { .. } => {
            // Edit just opens the modal — no state mutation here.
            false
        }
    }
}

#[test]
fn hide_panel_action_flips_visibility_in_state_map() {
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft { row: 0, col: 0, source_key: "x".into(), ..Default::default() },
    )
    .unwrap();

    let mut visibility: HashMap<(usize, usize), bool> = HashMap::new();
    let mut states: HashMap<(usize, usize), PanelState> = HashMap::new();
    let mut labels: HashMap<(usize, usize), LabelMode> = HashMap::new();

    // Hide → visibility[(0,0)] becomes false.
    apply_action(
        &CellMenuAction::HideToggle { row: 0, col: 0 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert_eq!(visibility.get(&(0, 0)).copied(), Some(false));

    // Toggle again → restores to true.
    apply_action(
        &CellMenuAction::HideToggle { row: 0, col: 0 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert_eq!(visibility.get(&(0, 0)).copied(), Some(true));
}

#[test]
fn reset_zoom_clears_locked_flag() {
    let mut tpl = empty_template();
    let mut visibility = HashMap::new();
    let mut states: HashMap<(usize, usize), PanelState> = HashMap::new();
    let mut labels = HashMap::new();

    states.entry((1, 1)).or_default().locked = true;
    assert!(states[&(1, 1)].locked);

    apply_action(
        &CellMenuAction::ResetZoom { row: 1, col: 1 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert!(!states[&(1, 1)].locked, "Reset zoom clears the lock");
}

#[test]
fn set_label_mode_action_stamps_label_override() {
    let mut tpl = empty_template();
    let mut visibility = HashMap::new();
    let mut states = HashMap::new();
    let mut labels: HashMap<(usize, usize), LabelMode> = HashMap::new();

    apply_action(
        &CellMenuAction::SetLabelMode { row: 0, col: 0, mode: LabelMode::Data },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert_eq!(labels.get(&(0, 0)).copied(), Some(LabelMode::Data));

    apply_action(
        &CellMenuAction::SetLabelMode { row: 0, col: 0, mode: LabelMode::Off },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert_eq!(labels.get(&(0, 0)).copied(), Some(LabelMode::Off));
}

#[test]
fn delete_action_removes_cell_from_template() {
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft { row: 1, col: 1, source_key: "x".into(), ..Default::default() },
    )
    .unwrap();
    assert_eq!(tpl.cells.len(), 1);

    let mut visibility = HashMap::new();
    let mut states = HashMap::new();
    let mut labels = HashMap::new();
    apply_action(
        &CellMenuAction::Delete { row: 1, col: 1 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert!(tpl.cells.is_empty(), "delete drops the cell");
}

#[test]
fn edit_action_is_noop_at_state_level() {
    // Edit opens the modal — the CLI side opens the editor, mutates draft
    // state, and only commits via a later "Add"/"Replace" click. The action
    // itself does NOT mutate template state.
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft { row: 0, col: 0, source_key: "x".into(), ..Default::default() },
    )
    .unwrap();
    let before = tpl.cells.clone();
    let mut visibility = HashMap::new();
    let mut states = HashMap::new();
    let mut labels = HashMap::new();
    apply_action(
        &CellMenuAction::Edit { row: 0, col: 0 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert_eq!(tpl.cells.len(), before.len(), "Edit action does not mutate cells");
}

/// v0.10.1 — right-click "Delete panel" round-trip:
///
/// 1. Operator right-clicks cell at (1, 1).
/// 2. Menu emits `CellMenuAction::Delete { row: 1, col: 1 }`.
/// 3. CLI handler removes the cell and sets `template_dirty = true`.
/// 4. The cells array shrinks by exactly 1.
#[test]
fn delete_action_shrinks_cells_and_dirties_template_v010_1() {
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
    assert_eq!(tpl.cells.len(), 2, "two cells seeded");

    let mut visibility = HashMap::new();
    let mut states = HashMap::new();
    let mut labels = HashMap::new();
    let mut template_dirty = false;

    let dirtied = apply_action(
        &CellMenuAction::Delete { row: 1, col: 1 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    template_dirty = template_dirty || dirtied;

    assert_eq!(tpl.cells.len(), 1, "cells array shrank by exactly 1");
    assert_eq!(tpl.cells[0].sources[0].key, "a", "the OTHER cell is preserved");
    assert!(template_dirty, "right-click Delete dirties the template");

    // Deleting a non-existent cell is a no-op: no shrink, no dirty flip
    // (the CLI's actual handler does set dirty, but the data-layer
    // `remove_cell_at` returns Err; this test asserts the underlying
    // contract — if the slot is empty, no cell is removed).
    let dirtied_again = apply_action(
        &CellMenuAction::Delete { row: 9, col: 9 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert_eq!(tpl.cells.len(), 1, "deleting empty slot is a no-op");
    assert!(!dirtied_again, "no dirty flip when nothing was deleted");
}

#[test]
fn all_action_variants_round_trip_through_clone_and_eq() {
    // The CLI buffers actions in a Vec between frames; pin the trait bounds.
    let actions = vec![
        CellMenuAction::Edit { row: 0, col: 0 },
        CellMenuAction::HideToggle { row: 1, col: 0 },
        CellMenuAction::ResetZoom { row: 2, col: 1 },
        CellMenuAction::SetLabelMode { row: 0, col: 0, mode: LabelMode::Data },
        CellMenuAction::Delete { row: 3, col: 2 },
    ];
    let cloned = actions.clone();
    assert_eq!(actions, cloned);
}
