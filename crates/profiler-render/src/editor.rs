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

use std::collections::{BTreeSet, HashMap};

use profiler_template::{Cell, CellSource, LabelMode, Primitive, Template, Trail3d, Trail3dSources, View3d};

use crate::TraceStore;

/// v0.11.0 — opinionated default schema of source-keys for HVN-SITL fleets.
///
/// The picker dropdown shows this list FIRST so users can author panel
/// templates against AP-fed channels (`ap_attitude`, `ap_raw_imu`, …) BEFORE
/// ArduPilot starts streaming. Without this seed the picker is empty / missing
/// AP entries for the first ~5–20 s after fleet startup because AP-emitted
/// keys flow as `None` from `dt_runner` until the autopilot wakes up, and
/// `TraceStore` only registers a key after a non-`None` value lands.
///
/// The list is project-specific by design: standalone deployments with custom
/// envelopes still get observed-key discovery merged on top via
/// [`collect_source_keys`].
pub const KNOWN_HVN_SITL_KEYS: &[&str] = &[
    // ── DT physics (truth / raw sensor models) ────────────────────────────
    "t",
    "accel[0]", "accel[1]", "accel[2]",
    "gyro[0]", "gyro[1]", "gyro[2]",
    "mag_xyz[0]", "mag_xyz[1]", "mag_xyz[2]",
    "mag_clean_xyz[0]", "mag_clean_xyz[1]", "mag_clean_xyz[2]",
    "wind_ned[0]", "wind_ned[1]", "wind_ned[2]",
    "baro_pressure", "baro_temp", "baro_alt", "state_alt",
    "gps_alt", "gps_vn",
    "quat_wxyz[0]", "quat_wxyz[1]", "quat_wxyz[2]", "quat_wxyz[3]",
    "euler[0]", "euler[1]", "euler[2]",
    // ── AP MAVLink mirrors (what the autopilot sees) ──────────────────────
    "ap_attitude[0]", "ap_attitude[1]", "ap_attitude[2]",
    "ap_raw_imu[0]", "ap_raw_imu[1]", "ap_raw_imu[2]",
    "ap_raw_imu[3]", "ap_raw_imu[4]", "ap_raw_imu[5]",
    "ap_vfr_alt",
    "ap_vel_ned[0]", "ap_vel_ned[1]", "ap_vel_ned[2]",
    // ── Position NED (truth / GPS sensor / EKF / target) ──────────────────
    "pos_truth_ned[0]", "pos_truth_ned[1]", "pos_truth_ned[2]",
    "pos_gps_ned[0]", "pos_gps_ned[1]", "pos_gps_ned[2]",
    "pos_ekf_ned[0]", "pos_ekf_ned[1]", "pos_ekf_ned[2]",
    "pos_target_ned[0]", "pos_target_ned[1]", "pos_target_ned[2]",
];

/// v0.11.0 — per-category collapse state for the grouped source-key dropdown.
///
/// Kept alongside the editor draft (one instance per modal) so each category
/// remembers whether the operator collapsed it independently of any other
/// editor. The default state is "all expanded" — first opens of the picker
/// look exactly like the v0.10.2 behaviour.
///
/// Used by the `source_key_combo` widget to drive a manual ▶/▼ toggle that
/// does NOT close the surrounding `ComboBox` popup (the v0.10.2 bug, where
/// clicking `CollapsingHeader`'s arrow propagated as an "outside" click and
/// dismissed the popup before the operator could pick a key).
#[derive(Debug, Clone, Default)]
pub struct ComboCollapseState {
    collapsed: HashMap<&'static str, bool>,
}

impl ComboCollapseState {
    /// `true` when the category is currently collapsed (hidden).
    pub fn is_collapsed(&self, category: &'static str) -> bool {
        self.collapsed.get(category).copied().unwrap_or(false)
    }

    /// Flip a category's collapsed state. Returns the new state.
    pub fn toggle(&mut self, category: &'static str) -> bool {
        let entry = self.collapsed.entry(category).or_insert(false);
        *entry = !*entry;
        *entry
    }
}

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
    if draft.primitive == Primitive::Diff && draft.minus.trim().is_empty() {
        return Err("diff primitive requires a subtrahend key".into());
    }
    // v0.10.1 — auto-grow the grid so the "+ New blank template…" flow can
    // start at 1×1 and expand as the operator adds panels. Capped to keep a
    // typo (e.g. 9999) from blowing the renderer up; 64×64 is well past what
    // we'd ever ship.
    const MAX_GRID_DIM: usize = 64;
    if draft.row >= MAX_GRID_DIM {
        return Err(format!(
            "row {} exceeds maximum grid dimension ({MAX_GRID_DIM})",
            draft.row,
        ));
    }
    if draft.col >= MAX_GRID_DIM {
        return Err(format!(
            "col {} exceeds maximum grid dimension ({MAX_GRID_DIM})",
            draft.col,
        ));
    }
    if draft.row >= tpl.grid.rows {
        tpl.grid.rows = draft.row + 1;
    }
    if draft.col >= tpl.grid.cols {
        tpl.grid.cols = draft.col + 1;
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

/// v0.10.2 — re-pack the template's `cells` into a tightly-packed grid.
///
/// Iterates the existing cells in display order (`(row, col)`-sorted),
/// re-assigning each to the next free slot top-to-bottom, left-to-right
/// across `tpl.grid.cols`. Visual ordering is preserved while gaps left by
/// `remove_cell_at` (or any other deletion) are removed.
///
/// `tpl.grid.rows` is also shrunk to the minimum needed for the surviving
/// cells (at least 1). `tpl.grid.cols` is left unchanged so the operator's
/// chosen column count persists across reflow.
///
/// Idempotent: calling `compact_cells` twice on the same template is a no-op
/// on the second call.
pub fn compact_cells(tpl: &mut Template) {
    let cols = tpl.grid.cols.max(1);
    let mut cells = std::mem::take(&mut tpl.cells);
    // Stable sort by (row, col) so cells in the same slot keep their relative
    // order (rare, but legal under the v0.10.0 spec).
    cells.sort_by_key(|c| (c.row, c.col));
    for (i, cell) in cells.iter_mut().enumerate() {
        cell.row = i / cols;
        cell.col = i % cols;
    }
    tpl.cells = cells;
    tpl.grid.rows = tpl
        .cells
        .iter()
        .map(|c| c.row + 1)
        .max()
        .unwrap_or(1);
}

/// v0.10.2 — categorize a source key into a section heading for the grouped
/// dropdown. The dropdown sorts groups in this fixed order:
///
/// 1. `"DT physics"` — truth / raw sensor models from the digital twin.
/// 2. `"AP MAVLink"` — autopilot mirrors over MAVLink.
/// 3. `"Position (NED)"` — any `pos_*` channel.
/// 4. `"Timing"` — `t`, `ts`.
/// 5. `"Other"` — everything else.
///
/// Classification is by BASE key (strip array indexing / dotted suffixes), so
/// `accel[0]`, `accel[1]`, and `accel` all land in the same group.
pub fn categorize_key(key: &str) -> &'static str {
    let base = key.split(['[', '.']).next().unwrap_or(key);
    match base {
        // DT physics (truth / raw sensor models)
        "accel" | "gyro" | "mag_xyz" | "mag_clean_xyz" | "wind_ned"
        | "baro_pressure" | "baro_temp" | "baro_alt" | "state_alt"
        | "quat_wxyz" | "euler" | "gps_alt" | "gps_vn"
            => "DT physics",
        // AP MAVLink mirrors (what the autopilot sees)
        "ap_attitude" | "ap_raw_imu" | "ap_vfr_alt" | "ap_vel_ned"
            => "AP MAVLink",
        // Position channels (NED frames)
        k if k.starts_with("pos_") => "Position (NED)",
        // Timing / miscellaneous
        "t" | "ts"
            => "Timing",
        _ => "Other",
    }
}

/// v0.10.2 — fixed group ordering used by the dropdown UI. Returned as a
/// slice so the rendering side can iterate over groups in this exact order
/// regardless of whether a given run observed any keys in some group.
pub const KEY_GROUPS: &[&str] = &[
    "DT physics",
    "AP MAVLink",
    "Position (NED)",
    "Timing",
    "Other",
];

/// v0.10.2 — group a flat list of source keys by category, preserving the
/// fixed `KEY_GROUPS` order. Within each group keys retain their input order
/// (the caller is expected to pass an alphabetically-sorted list, which is
/// what `collect_source_keys` already returns).
///
/// Empty groups are omitted from the returned vector so the UI doesn't draw
/// dead section headers.
pub fn group_source_keys(keys: &[String]) -> Vec<(&'static str, Vec<String>)> {
    let mut buckets: std::collections::BTreeMap<&'static str, Vec<String>> =
        std::collections::BTreeMap::new();
    for k in keys {
        let cat = categorize_key(k);
        buckets.entry(cat).or_default().push(k.clone());
    }
    KEY_GROUPS
        .iter()
        .filter_map(|g| buckets.remove(g).map(|v| (*g, v)))
        .collect()
}

/// Replace the cell originally at `(row, col)` with the draft contents — used
/// by the per-cell "Edit panel..." flow.
///
/// v0.10.1 — the draft's own `(row, col)` is honoured: if the operator drags
/// the panel to a new slot, the original cell at `(row, col)` is removed and
/// the new entry is inserted at `(draft.row, draft.col)`. Pure replacement
/// (when `(draft.row, draft.col) == (row, col)`) is the common case and works
/// as before. Returns `Err` if the destination is already occupied by a
/// DIFFERENT cell — the modal should stay open with a status-bar message.
pub fn replace_cell_at(tpl: &mut Template, row: usize, col: usize, draft: &PanelDraft) -> Result<(), String> {
    let new_row = draft.row;
    let new_col = draft.col;
    let relocating = (new_row, new_col) != (row, col);
    if relocating {
        // Refuse to silently clobber an unrelated cell at the destination.
        let occupied = tpl
            .cells
            .iter()
            .any(|c| c.row == new_row && c.col == new_col);
        if occupied {
            return Err(format!(
                "destination ({new_row}, {new_col}) is already occupied; \
                 delete that cell first"
            ));
        }
    }
    // Remove the cell at the ORIGINAL coordinates (`(row, col)` — the menu-
    // invocation slot). Tolerate "no existing cell" so this also works as a
    // pure add.
    let _ = remove_cell_at(tpl, row, col);
    // `apply_panel_draft` already uses `draft.row` / `draft.col` as the
    // insert location, so the relocation just works.
    apply_panel_draft(tpl, draft)
}

/// v0.11.0 — swap two cells in the template by their `(row, col)` coordinates.
///
/// Both endpoints must currently host a cell; the two cells exchange their
/// `(row, col)` fields in place. Used by drag-to-reorder when the operator
/// drops a panel onto another occupied slot.
///
/// Returns `Err` if either endpoint is empty. Same-slot swap is a no-op and
/// returns `Ok(())`.
pub fn swap_cells(tpl: &mut Template, a: (usize, usize), b: (usize, usize)) -> Result<(), String> {
    if a == b {
        return Ok(());
    }
    let ai = tpl
        .cells
        .iter()
        .position(|c| (c.row, c.col) == a)
        .ok_or_else(|| format!("no cell at ({}, {})", a.0, a.1))?;
    let bi = tpl
        .cells
        .iter()
        .position(|c| (c.row, c.col) == b)
        .ok_or_else(|| format!("no cell at ({}, {})", b.0, b.1))?;
    tpl.cells[ai].row = b.0;
    tpl.cells[ai].col = b.1;
    tpl.cells[bi].row = a.0;
    tpl.cells[bi].col = a.1;
    Ok(())
}

/// v0.11.0 — relocate the cell at `from` to the empty slot `to`. The destination
/// must be empty (use [`swap_cells`] if it isn't). Used by drag-to-reorder when
/// the operator drops a panel onto an empty grid cell.
pub fn relocate_cell(tpl: &mut Template, from: (usize, usize), to: (usize, usize)) -> Result<(), String> {
    if from == to {
        return Ok(());
    }
    if tpl.cells.iter().any(|c| (c.row, c.col) == to) {
        return Err(format!("destination ({}, {}) is occupied", to.0, to.1));
    }
    let idx = tpl
        .cells
        .iter()
        .position(|c| (c.row, c.col) == from)
        .ok_or_else(|| format!("no cell at ({}, {})", from.0, from.1))?;
    tpl.cells[idx].row = to.0;
    tpl.cells[idx].col = to.1;
    if to.0 >= tpl.grid.rows {
        tpl.grid.rows = to.0 + 1;
    }
    if to.1 >= tpl.grid.cols {
        tpl.grid.cols = to.1 + 1;
    }
    Ok(())
}

/// v0.11.0 — undo/redo history of template snapshots.
///
/// Each editor mutation records the template's PRE-change state via
/// [`EditHistory::record`]; Ctrl+Z swaps in the previous snapshot, pushing the
/// current state onto the redo stack. Capacity-bounded (default 64) so the
/// memory footprint stays predictable on long editing sessions; the oldest
/// snapshot is evicted when the past stack fills.
///
/// Redo is consumed (cleared) on the next `record` — i.e. branching from an
/// undone state discards the previously-undone future, matching the standard
/// linear-history model most editors use.
#[derive(Debug, Clone)]
pub struct EditHistory {
    past: Vec<Template>,
    future: Vec<Template>,
    capacity: usize,
}

impl Default for EditHistory {
    fn default() -> Self {
        Self::new(64)
    }
}

impl EditHistory {
    /// Construct an empty history with the given capacity (clamped to ≥ 1).
    pub fn new(capacity: usize) -> Self {
        Self {
            past: Vec::new(),
            future: Vec::new(),
            capacity: capacity.max(1),
        }
    }

    /// Record `snapshot` as a pre-change state. Clears the redo stack
    /// (branching from an undone state). Evicts the oldest entry when the
    /// past stack would exceed `capacity`.
    pub fn record(&mut self, snapshot: Template) {
        self.future.clear();
        self.past.push(snapshot);
        if self.past.len() > self.capacity {
            // Drain the leading overflow (usually exactly 1 entry).
            let excess = self.past.len() - self.capacity;
            self.past.drain(0..excess);
        }
    }

    /// Pop the most recent snapshot from the past stack and return it. The
    /// caller passes its CURRENT template state in `current`; that state is
    /// pushed onto the redo stack so Ctrl+Y can restore it. Returns `None`
    /// (no undo available) when the past stack is empty.
    pub fn undo(&mut self, current: Template) -> Option<Template> {
        let prev = self.past.pop()?;
        self.future.push(current);
        Some(prev)
    }

    /// Inverse of [`Self::undo`]. Pops the most recent entry from the redo
    /// stack and returns it; `current` is pushed back onto the past stack.
    pub fn redo(&mut self, current: Template) -> Option<Template> {
        let next = self.future.pop()?;
        self.past.push(current);
        Some(next)
    }

    /// `true` when Ctrl+Z would have an effect.
    pub fn can_undo(&self) -> bool {
        !self.past.is_empty()
    }

    /// `true` when Ctrl+Y / Ctrl+Shift+Z would have an effect.
    pub fn can_redo(&self) -> bool {
        !self.future.is_empty()
    }

    /// Number of past snapshots currently stored (test helper).
    #[doc(hidden)]
    pub fn past_len(&self) -> usize {
        self.past.len()
    }

    /// Number of redo snapshots currently stored (test helper).
    #[doc(hidden)]
    pub fn future_len(&self) -> usize {
        self.future.len()
    }
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
///
/// v0.11.0 — [`KNOWN_HVN_SITL_KEYS`] is merged in FIRST so the picker shows
/// the standard HVN-SITL vocabulary (including AP MAVLink mirrors that
/// haven't streamed yet) even when the stores are empty or the autopilot is
/// still booting. Observed keys are added on top, so custom dialects /
/// non-HVN sources still surface every channel they emit.
/// Additionally, every observed-key BASE (e.g. `ap_attitude` for
/// `ap_attitude[0]`) is registered even when the value is `None` — see
/// [`TraceStore::null_keys`] for the schema-only registration used by
/// `dt_runner`-emitted-as-None channels.
pub fn collect_source_keys<'a, I>(stores: I) -> Vec<String>
where
    I: IntoIterator<Item = &'a TraceStore>,
{
    let mut all: BTreeSet<String> = BTreeSet::new();
    // v0.11.0 — opinionated HVN-SITL defaults first so the picker is never
    // empty before the first envelope and AP MAVLink keys are addressable
    // immediately at startup.
    for k in KNOWN_HVN_SITL_KEYS {
        all.insert((*k).to_string());
        // Also register the base (`accel` for `accel[0]`) — vector primitives
        // need it.
        if let Some(idx) = k.rfind('[') {
            let base = &k[..idx];
            if !base.is_empty() {
                all.insert(base.to_string());
            }
        }
    }
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
        // v0.11.0 — schema-only "observed but always-None" keys: the store
        // saw an envelope key but the value flowed in as `null` (the
        // dt_runner emits AP MAVLink mirrors this way until AP wakes up).
        // Register the base name so users can pre-build templates against
        // it; the renderer paints "waiting for data..." until a real
        // value lands.
        for k in s.null_keys() {
            all.insert(k.clone());
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
        // Pure replace (same row/col) — common case.
        replace_cell_at(
            &mut tpl,
            1,
            1,
            &PanelDraft { row: 1, col: 1, source_key: "new".into(), ..Default::default() },
        )
        .unwrap();
        assert_eq!(tpl.cells.len(), 1);
        assert_eq!(tpl.cells[0].sources[0].key, "new");
        assert_eq!((tpl.cells[0].row, tpl.cells[0].col), (1, 1));
    }

    /// v0.10.1 — Edit modal honours the form's row/col: moving a panel to a
    /// new slot relocates it. The original coordinates are emptied; the new
    /// coordinates are populated.
    #[test]
    fn replace_cell_at_relocates_when_draft_has_new_row_col() {
        let mut tpl = empty_template();
        apply_panel_draft(
            &mut tpl,
            &PanelDraft { row: 2, col: 1, source_key: "moved".into(), ..Default::default() },
        )
        .unwrap();
        // Operator opens Edit on (2, 1), changes Row/Col to (1, 2) in the
        // form, clicks Apply. Old cell at (2, 1) disappears; new cell at
        // (1, 2) appears.
        replace_cell_at(
            &mut tpl,
            2,
            1,
            &PanelDraft { row: 1, col: 2, source_key: "moved".into(), ..Default::default() },
        )
        .unwrap();
        assert_eq!(tpl.cells.len(), 1);
        assert_eq!((tpl.cells[0].row, tpl.cells[0].col), (1, 2));
        assert!(
            !tpl.cells.iter().any(|c| (c.row, c.col) == (2, 1)),
            "original slot is empty after a relocation"
        );
    }

    /// v0.10.1 — relocating onto an already-occupied slot must error so the
    /// modal can stay open with a status-bar message.
    #[test]
    fn replace_cell_at_rejects_relocation_to_occupied_slot() {
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
        let err = replace_cell_at(
            &mut tpl,
            0,
            0,
            &PanelDraft { row: 1, col: 1, source_key: "a".into(), ..Default::default() },
        )
        .unwrap_err();
        assert!(
            err.contains("occupied"),
            "error message mentions occupancy: {err}"
        );
        // Both cells preserved — modal stays open, no state mutation.
        assert_eq!(tpl.cells.len(), 2);
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
