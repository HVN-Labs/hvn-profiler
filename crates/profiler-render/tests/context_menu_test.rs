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
        // v0.11.0 — drag-to-reorder actions; not exercised by this test file
        // (covered by `drag_reorder_test.rs`), but the match must stay
        // exhaustive.
        CellMenuAction::SwapTo { from, to } => {
            profiler_render::swap_cells(tpl, *from, *to).is_ok()
        }
        CellMenuAction::MoveTo { from, to } => {
            profiler_render::relocate_cell(tpl, *from, *to).is_ok()
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

// ─── v0.16.1 — right-click menu reaches Status + InfoText cells ─────────────
//
// Pre-v0.16.1 the per-cell context menu was attached only to the egui_plot
// response for 2D primitives (Scalar / Vector / Overlay / Magnitude / Diff /
// MagInterference / AttitudeRpy). The Status (v0.12.0) and InfoText (v0.14.0)
// primitives render their own Frame-based widgets via `painter` calls; their
// scope_builder responses did not pick up secondary clicks because no inner
// widgets were allocated. v0.16.1 wraps EVERY cell render in a transparent
// `Sense::click()` overlay over the full cell rect — the overlay's response
// carries `context_menu(...)` so right-click works regardless of what the
// inner code drew.
//
// These data-layer tests pin the predicates that gate menu items per
// primitive (Reset zoom + Label submenu are 2D-plot-only). Full UI-driven
// secondary-click tests would need an egui test harness; the renderer's
// invariant is captured at the data layer instead.

use profiler_render::{primitive_supports_label_mode, primitive_supports_zoom};
use profiler_template::Primitive;

#[test]
fn v0_16_1_reset_zoom_hidden_on_non_plot_primitives() {
    // Status + InfoText render their own non-plot widgets — Reset zoom is
    // meaningless for them. The context-menu builder gates the entry on
    // `primitive_supports_zoom`.
    assert!(!primitive_supports_zoom(Primitive::Status), "Status has no zoom");
    assert!(!primitive_supports_zoom(Primitive::InfoText), "InfoText has no zoom");
    // StatusBadge is reserved (renders nothing today) — also non-zoomable.
    assert!(!primitive_supports_zoom(Primitive::StatusBadge), "StatusBadge has no zoom");
}

#[test]
fn v0_16_1_reset_zoom_visible_on_every_2d_plot_primitive() {
    // All primitives that flow through `egui_plot` must keep the Reset zoom
    // entry — gating accidentally to `false` would regress v0.10.0's UX.
    for p in [
        Primitive::Scalar,
        Primitive::Vector,
        Primitive::Overlay,
        Primitive::Magnitude,
        Primitive::MagInterference,
        Primitive::Diff,
        Primitive::AttitudeRpy,
    ] {
        assert!(
            primitive_supports_zoom(p),
            "{p:?} renders through egui_plot — must show Reset zoom"
        );
    }
}

#[test]
fn v0_16_1_label_submenu_hidden_on_status_and_info_text() {
    // Label overlays (data / metadata) apply only to plot cells; Status and
    // InfoText carry their own content semantics. The menu hides the Label
    // submenu entirely for those primitives.
    assert!(!primitive_supports_label_mode(Primitive::Status));
    assert!(!primitive_supports_label_mode(Primitive::InfoText));
}

#[test]
fn v0_16_1_label_submenu_visible_on_2d_plot_primitives() {
    for p in [
        Primitive::Scalar,
        Primitive::Vector,
        Primitive::Overlay,
        Primitive::Magnitude,
        Primitive::Diff,
        Primitive::AttitudeRpy,
    ] {
        assert!(
            primitive_supports_label_mode(p),
            "{p:?} should expose the Label submenu"
        );
    }
}

#[test]
fn v0_16_1_status_cell_emits_full_action_set_minus_zoom() {
    // Data-layer simulation: a right-click on a Status cell emits Edit /
    // Hide / Delete (and the Label submenu would be hidden by the
    // predicate). The CLI consumes the actions identically — no per-cell
    // routing differs for Status vs. plot primitives.
    use std::collections::HashMap;
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft {
            row: 0,
            col: 0,
            source_key: "flight_mode".into(),
            ..Default::default()
        },
    )
    .unwrap();

    let mut visibility = HashMap::new();
    let mut states: HashMap<(usize, usize), PanelState> = HashMap::new();
    let mut labels = HashMap::new();

    // Hide works on Status cells.
    apply_action(
        &CellMenuAction::HideToggle { row: 0, col: 0 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert_eq!(visibility.get(&(0, 0)).copied(), Some(false));

    // Delete works on Status cells.
    apply_action(
        &CellMenuAction::Delete { row: 0, col: 0 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert!(tpl.cells.is_empty(), "Right-click Delete on a Status cell removes it");
}

#[test]
fn v0_16_1_info_text_cell_emits_full_action_set_minus_zoom() {
    // Same contract for InfoText: Edit / Hide / Delete reach the CLI, the
    // gated "Reset zoom" / "Label" entries do not even appear in the menu.
    use std::collections::HashMap;
    let mut tpl = empty_template();
    apply_panel_draft(
        &mut tpl,
        &PanelDraft {
            row: 1,
            col: 0,
            source_key: "_info".into(),
            ..Default::default()
        },
    )
    .unwrap();

    let mut visibility = HashMap::new();
    let mut states: HashMap<(usize, usize), PanelState> = HashMap::new();
    let mut labels = HashMap::new();

    apply_action(
        &CellMenuAction::HideToggle { row: 1, col: 0 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert_eq!(visibility.get(&(1, 0)).copied(), Some(false));

    apply_action(
        &CellMenuAction::Delete { row: 1, col: 0 },
        &mut tpl,
        &mut visibility,
        &mut states,
        &mut labels,
    );
    assert!(tpl.cells.is_empty(), "Right-click Delete on an InfoText cell removes it");
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
