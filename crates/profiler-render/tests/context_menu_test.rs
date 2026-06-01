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
/// the App-side handler in profiler-cli.
fn apply_action(
    action: &CellMenuAction,
    tpl: &mut Template,
    visibility: &mut HashMap<(usize, usize), bool>,
    panel_states: &mut HashMap<(usize, usize), PanelState>,
    label_overrides: &mut HashMap<(usize, usize), LabelMode>,
) {
    match action {
        CellMenuAction::HideToggle { row, col } => {
            let cur = visibility.get(&(*row, *col)).copied().unwrap_or(true);
            visibility.insert((*row, *col), !cur);
        }
        CellMenuAction::ResetZoom { row, col } => {
            if let Some(st) = panel_states.get_mut(&(*row, *col)) {
                st.locked = false;
            }
        }
        CellMenuAction::SetLabelMode { row, col, mode } => {
            label_overrides.insert((*row, *col), *mode);
        }
        CellMenuAction::Delete { row, col } => {
            let _ = remove_cell_at(tpl, *row, *col);
        }
        CellMenuAction::Edit { .. } => {
            // Edit just opens the modal — no state mutation here.
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
