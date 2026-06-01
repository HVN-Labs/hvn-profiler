//! v0.10.0 — in-app template editor state + apply helpers.
//!
//! The actual modal `egui::Window` for "+ Add Panel" / "+ Add Trail" / per-cell
//! "Edit..." is wired in `profiler-cli` (where `App` already owns the toolbar).
//! This module holds the *plain-data* state structs and the pure-function
//! "apply this draft to the loaded template" helpers, so the cell-editor
//! integration tests can run headless against a `Template` mutation.
//!
//! Design notes:
//! - All draft state implements `Default` + `Clone` so the CLI can carry one
//!   instance per editor and reset it on Cancel.
//! - Apply helpers return `Result<(), String>` rather than panicking on bad
//!   input (out-of-bounds row/col, empty key, …). The CLI surfaces the error
//!   string in the toolbar's `last_template_action` slot.
//! - The set of "known source keys" the dropdown offers is gathered by
//!   `collect_source_keys` against the live multi-drone store map — exposed
//!   here so a future per-key autocomplete can swap in without API churn.

use std::collections::BTreeSet;

use profiler_template::{Cell, CellSource, LabelMode, Primitive, Template, Trail3d, Trail3dSources, View3d};

use crate::TraceStore;

/// Draft state for the "+ Add Panel" modal.
///
/// Populated by the egui form widgets in `profiler-cli`; applied to the loaded
/// `Template` via [`apply_panel_draft`] when the operator clicks "Add".
#[derive(Debug, Clone)]
pub struct PanelDraft {
    pub row: usize,
    pub col: usize,
    pub primitive: Primitive,
    pub title: String,
    /// Primary source key. For `Vector`/`Magnitude`/`MagInterference`/
    /// `AttitudeRpy` this is the array BASE (e.g. `"ap_attitude"`, the
    /// renderer reads `base[0..2]`). For scalar/overlay/diff this is a fully-
    /// qualified key (e.g. `"ap_attitude[0]"`).
    pub source_key: String,
    /// Optional fallback (when `source_key` has no data, plot this instead).
    pub fallback: String,
    /// For `Diff` only: the subtrahend key (`source_key − minus`).
    pub minus: String,
    /// `#rrggbb` color string (the matplotlib `C0..C9` shorthand also parses).
    pub color: String,
    pub label_mode: LabelMode,
    /// Extra source keys, used by `Overlay`. Each entry becomes its own line.
    pub overlay_extra_keys: Vec<String>,
}

impl Default for PanelDraft {
    fn default() -> Self {
        Self {
            row: 0,
            col: 0,
            primitive: Primitive::Scalar,
            title: String::new(),
            source_key: String::new(),
            fallback: String::new(),
            minus: String::new(),
            color: "#1f77b4".to_string(),
            label_mode: LabelMode::Off,
            overlay_extra_keys: Vec::new(),
        }
    }
}

/// Apply a [`PanelDraft`] to the loaded template — appending a new `Cell` to
/// the cells vector. Returns `Err(reason)` when the draft is invalid; the
/// caller surfaces the reason via the toolbar status line.
///
/// Validation rules:
/// - `source_key` must be non-empty (no anonymous panels).
/// - `(row, col)` must lie within the template's grid. If a cell already
///   exists at that slot, the new entry is appended anyway — the renderer
///   uses the LAST entry at each `(row, col)`, so this acts like
///   "replace at slot" in practice.
/// - For `Diff`, `minus` must also be non-empty.
pub fn apply_panel_draft(tpl: &mut Template, draft: &PanelDraft) -> Result<(), String> {
    if draft.source_key.trim().is_empty() {
        return Err("source key must not be empty".into());
    }
    if draft.row >= tpl.grid.rows {
        return Err(format!(
            "row {} is out of range (grid has {} rows)",
            draft.row, tpl.grid.rows,
        ));
    }
    if draft.col >= tpl.grid.cols {
        return Err(format!(
            "col {} is out of range (grid has {} cols)",
            draft.col, tpl.grid.cols,
        ));
    }
    if draft.primitive == Primitive::Diff && draft.minus.trim().is_empty() {
        return Err("diff primitive requires a subtrahend key".into());
    }

    // Primary source.
    let mut sources: Vec<CellSource> = vec![CellSource {
        key: draft.source_key.trim().to_string(),
        fallback: non_empty(&draft.fallback),
        minus: if draft.primitive == Primitive::Diff {
            non_empty(&draft.minus)
        } else {
            None
        },
        color: draft.color.clone(),
        ..Default::default()
    }];

    // Overlay: append each extra key as its own source (its own line).
    if draft.primitive == Primitive::Overlay {
        for k in &draft.overlay_extra_keys {
            if !k.trim().is_empty() {
                sources.push(CellSource {
                    key: k.trim().to_string(),
                    ..Default::default()
                });
            }
        }
    }

    let cell = Cell {
        row: draft.row,
        col: draft.col,
        title: draft.title.clone(),
        primitive: draft.primitive,
        sources,
        color: non_empty(&draft.color),
        visible: true,
        label_mode: draft.label_mode,
        ..Default::default()
    };
    tpl.cells.push(cell);
    Ok(())
}

/// Remove a cell from the template by `(row, col)`. Removes ALL entries at
/// that slot (rare, but the spec allows duplicates via `apply_panel_draft`).
/// Returns `Err` if nothing was removed.
pub fn remove_cell_at(tpl: &mut Template, row: usize, col: usize) -> Result<(), String> {
    let before = tpl.cells.len();
    tpl.cells.retain(|c| !(c.row == row && c.col == col));
    if tpl.cells.len() == before {
        Err(format!("no cell at ({row}, {col})"))
    } else {
        Ok(())
    }
}

/// Replace the cell at `(row, col)` with the draft contents — used by the
/// per-cell "Edit panel..." flow. Equivalent to `remove_cell_at` followed by
/// `apply_panel_draft`, but tolerates "no existing cell" (acts as add).
pub fn replace_cell_at(tpl: &mut Template, row: usize, col: usize, draft: &PanelDraft) -> Result<(), String> {
    let mut new_draft = draft.clone();
    new_draft.row = row;
    new_draft.col = col;
    let _ = remove_cell_at(tpl, row, col); // ok if nothing was there
    apply_panel_draft(tpl, &new_draft)
}

/// Draft state for the "+ Add Trail" 3D modal.
#[derive(Debug, Clone, Default)]
pub struct TrailDraft {
    pub name: String,
    pub label: String,
    pub color: String,
    /// `false` → direct (E, N, Up) source bindings; `true` → dead-reckon block.
    pub use_deadreckon: bool,
    pub x_key: String,
    pub y_key: String,
    pub z_neg_key: String,
    pub accel_key: String,
    pub quat_key: String,
    pub seed_key: String,
}

/// Apply a [`TrailDraft`] to the loaded template's `view_3d` block — appending
/// a new `Trail3d` entry. If the template has no `view_3d` yet, one is
/// synthesised with sensible defaults.
pub fn apply_trail_draft(tpl: &mut Template, draft: &TrailDraft) -> Result<(), String> {
    if draft.name.trim().is_empty() {
        return Err("trail name must not be empty".into());
    }
    if draft.use_deadreckon {
        if draft.accel_key.trim().is_empty()
            || draft.quat_key.trim().is_empty()
            || draft.seed_key.trim().is_empty()
        {
            return Err("dead-reckon trail requires accel + quat + seed keys".into());
        }
    } else if draft.x_key.trim().is_empty()
        || draft.y_key.trim().is_empty()
        || draft.z_neg_key.trim().is_empty()
    {
        return Err("direct trail requires x + y + z_neg keys".into());
    }

    let view = tpl
        .view_3d
        .get_or_insert_with(View3d::default);

    let trail = Trail3d {
        name: draft.name.trim().to_string(),
        label: if draft.label.is_empty() {
            draft.name.trim().to_string()
        } else {
            draft.label.clone()
        },
        color: draft.color.clone(),
        sources: if draft.use_deadreckon {
            None
        } else {
            Some(Trail3dSources {
                x: draft.x_key.trim().to_string(),
                y: draft.y_key.trim().to_string(),
                z_neg: draft.z_neg_key.trim().to_string(),
            })
        },
        deadreckon: if draft.use_deadreckon {
            Some(profiler_template::Trail3dDeadreckon {
                accel: draft.accel_key.trim().to_string(),
                quat: draft.quat_key.trim().to_string(),
                seed_from: draft.seed_key.trim().to_string(),
            })
        } else {
            None
        },
    };
    view.trails.push(trail);
    Ok(())
}

/// Collect every store key currently observed across the per-drone store map,
/// sorted alphabetically. Used to populate the editor's source-key dropdown
/// so the operator picks from "things actually in the stream" rather than
/// typing free-form.
///
/// For each store key matching `<base>[i]`, the BASE is also included so
/// vector / magnitude / attitude_rpy primitives can be added without
/// requiring the operator to know the wire format.
pub fn collect_source_keys<'a, I>(stores: I) -> Vec<String>
where
    I: IntoIterator<Item = &'a TraceStore>,
{
    let mut all: BTreeSet<String> = BTreeSet::new();
    for s in stores {
        for k in s.keys() {
            all.insert(k.clone());
            // Add the base for vector primitives (`foo[0]` → `foo`).
            if let Some(idx) = k.rfind('[') {
                let base = &k[..idx];
                if !base.is_empty() {
                    all.insert(base.to_string());
                }
            }
        }
    }
    all.into_iter().collect()
}

fn non_empty(s: &str) -> Option<String> {
    let t = s.trim();
    if t.is_empty() {
        None
    } else {
        Some(t.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn empty_template() -> Template {
        let json = r#"{
            "name": "empty",
            "grid": {"rows": 3, "cols": 3},
            "cells": []
        }"#;
        Template::from_str(json).expect("parse")
    }

    #[test]
    fn apply_panel_draft_appends_cell() {
        let mut tpl = empty_template();
        let draft = PanelDraft {
            row: 1,
            col: 2,
            primitive: Primitive::Scalar,
            title: "Roll".into(),
            source_key: "ap_attitude[0]".into(),
            color: "#1f77b4".into(),
            ..Default::default()
        };
        apply_panel_draft(&mut tpl, &draft).expect("apply");
        assert_eq!(tpl.cells.len(), 1);
        let c = &tpl.cells[0];
        assert_eq!((c.row, c.col), (1, 2));
        assert_eq!(c.primitive, Primitive::Scalar);
        assert_eq!(c.title, "Roll");
        assert_eq!(c.sources.len(), 1);
        assert_eq!(c.sources[0].key, "ap_attitude[0]");
        assert!(c.visible);
    }

    #[test]
    fn apply_panel_draft_rejects_empty_key() {
        let mut tpl = empty_template();
        let draft = PanelDraft::default();
        let err = apply_panel_draft(&mut tpl, &draft).unwrap_err();
        assert!(err.contains("source key"));
        assert!(tpl.cells.is_empty());
    }

    #[test]
    fn apply_panel_draft_rejects_out_of_grid() {
        let mut tpl = empty_template();
        let draft = PanelDraft {
            row: 99,
            col: 0,
            source_key: "foo".into(),
            ..Default::default()
        };
        assert!(apply_panel_draft(&mut tpl, &draft).is_err());
    }

    #[test]
    fn apply_panel_draft_overlay_appends_extra_sources() {
        let mut tpl = empty_template();
        let draft = PanelDraft {
            row: 0,
            col: 0,
            primitive: Primitive::Overlay,
            source_key: "a".into(),
            overlay_extra_keys: vec!["b".into(), "c".into()],
            ..Default::default()
        };
        apply_panel_draft(&mut tpl, &draft).unwrap();
        let cell = &tpl.cells[0];
        assert_eq!(cell.sources.len(), 3);
        assert_eq!(cell.sources[0].key, "a");
        assert_eq!(cell.sources[1].key, "b");
        assert_eq!(cell.sources[2].key, "c");
    }

    #[test]
    fn apply_panel_draft_diff_requires_minus() {
        let mut tpl = empty_template();
        let draft = PanelDraft {
            row: 0,
            col: 0,
            primitive: Primitive::Diff,
            source_key: "a".into(),
            ..Default::default()
        };
        assert!(apply_panel_draft(&mut tpl, &draft).is_err());
        let draft = PanelDraft {
            row: 0,
            col: 0,
            primitive: Primitive::Diff,
            source_key: "a".into(),
            minus: "b".into(),
            ..Default::default()
        };
        apply_panel_draft(&mut tpl, &draft).unwrap();
        assert_eq!(tpl.cells[0].sources[0].minus.as_deref(), Some("b"));
    }

    #[test]
    fn remove_cell_at_drops_entry() {
        let mut tpl = empty_template();
        apply_panel_draft(
            &mut tpl,
            &PanelDraft { row: 0, col: 0, source_key: "x".into(), ..Default::default() },
        )
        .unwrap();
        assert_eq!(tpl.cells.len(), 1);
        remove_cell_at(&mut tpl, 0, 0).unwrap();
        assert!(tpl.cells.is_empty());
        assert!(remove_cell_at(&mut tpl, 0, 0).is_err());
    }

    #[test]
    fn replace_cell_at_swaps_entry() {
        let mut tpl = empty_template();
        apply_panel_draft(
            &mut tpl,
            &PanelDraft { row: 1, col: 1, source_key: "old".into(), ..Default::default() },
        )
        .unwrap();
        replace_cell_at(
            &mut tpl,
            1,
            1,
            &PanelDraft { source_key: "new".into(), ..Default::default() },
        )
        .unwrap();
        assert_eq!(tpl.cells.len(), 1);
        assert_eq!(tpl.cells[0].sources[0].key, "new");
    }

    #[test]
    fn apply_trail_draft_appends_to_view_3d() {
        let mut tpl = empty_template();
        let draft = TrailDraft {
            name: "mine".into(),
            label: "Mine".into(),
            color: "#9467bd".into(),
            use_deadreckon: false,
            x_key: "pos_truth_ned[1]".into(),
            y_key: "pos_truth_ned[0]".into(),
            z_neg_key: "pos_truth_ned[2]".into(),
            ..Default::default()
        };
        apply_trail_draft(&mut tpl, &draft).unwrap();
        let v = tpl.view_3d.as_ref().expect("view_3d created");
        assert_eq!(v.trails.len(), 1);
        let t = &v.trails[0];
        assert_eq!(t.name, "mine");
        let s = t.sources.as_ref().unwrap();
        assert_eq!(s.x, "pos_truth_ned[1]");
        assert!(t.deadreckon.is_none());
    }

    #[test]
    fn apply_trail_draft_deadreckon_branch() {
        let mut tpl = empty_template();
        let draft = TrailDraft {
            name: "dr".into(),
            use_deadreckon: true,
            accel_key: "accel".into(),
            quat_key: "quat_wxyz".into(),
            seed_key: "pos_truth_ned".into(),
            ..Default::default()
        };
        apply_trail_draft(&mut tpl, &draft).unwrap();
        let dr = &tpl.view_3d.as_ref().unwrap().trails[0];
        let dk = dr.deadreckon.as_ref().unwrap();
        assert_eq!(dk.accel, "accel");
        assert!(dr.sources.is_none());
    }

    #[test]
    fn apply_trail_draft_rejects_missing_keys() {
        let mut tpl = empty_template();
        let draft = TrailDraft { name: "x".into(), ..Default::default() };
        assert!(apply_trail_draft(&mut tpl, &draft).is_err());
    }

    #[test]
    fn collect_source_keys_includes_bases() {
        let mut s = TraceStore::new(60.0);
        s.push(0.0, "ap_attitude[0]", 0.0);
        s.push(0.0, "ap_attitude[1]", 0.0);
        s.push(0.0, "gps_alt", 0.0);
        let keys = collect_source_keys(&[s]);
        assert!(keys.contains(&"ap_attitude[0]".to_string()));
        assert!(keys.contains(&"ap_attitude[1]".to_string()));
        assert!(keys.contains(&"ap_attitude".to_string()), "vector base included");
        assert!(keys.contains(&"gps_alt".to_string()));
    }
}
