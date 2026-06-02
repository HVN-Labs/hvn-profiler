//! Multi-panel 2D layout renderer (v0.2.0; v0.9.0 made labels non-reflowing).
//!
//! Given a [`profiler_template::Template`] and a [`crate::TraceStore`], lay the
//! template's `cells` out on a `grid.rows × grid.cols` grid, one
//! [`egui_plot::Plot`] per visible cell. Each panel:
//!
//! - Y auto-scales to the data visible in the rolling window.
//! - X shows the latest `window_s` seconds (matches the `TraceStore`).
//! - Has NO interactive controls (drag / zoom / scroll disabled) — the 2D
//!   panels are static auto-scaling by design. Live controls are exclusive to
//!   the 3D view (see `crate::view3d`); 2D panels stay control-free.
//!
//! Per-panel `label_mode` overlays (`data` / `metadata`) are drawn as a
//! non-reflowing screen-space overlay in the plot's top-right corner: a
//! semi-transparent rounded rect with the label text painted on top.
//!
//! v0.9.0 made the overlay non-reflowing — earlier versions used
//! [`egui_plot::Text`] inside the plot, which kept the layout stable but
//! tied the label to plot coordinates and didn't carry its own background.
//! The new pipeline runs [`egui_plot::Plot::show`] first, captures the
//! response's screen-space rect, then paints the overlay via the parent
//! [`egui::Ui`]'s painter. The plot itself never sees the label, so toggling
//! it on/off cannot resize the plot.
//!
//! The primitive renderers map [`profiler_template::Primitive`] variants onto
//! egui_plot lines; see [`render_cell`].

use std::collections::HashMap;

use egui::{Align2, Color32, CursorIcon, Id, LayerId, Order, Pos2, Rect, RichText, Sense, Stroke, StrokeKind, TextStyle, UiBuilder, Vec2};
use egui_plot::{Corner, HLine, Legend, Line, Plot, PlotPoints};

use profiler_template::{Cell, CellSource, LabelMode, Primitive, Section, Template};

/// v0.10.0 — per-panel runtime state for 2D zoom/pan + auto-scale lock.
///
/// Keyed by `(row, col)` inside the renderer's session-state map. `locked`
/// starts `false` (auto-scale Y on); on the first user interaction
/// (drag / box-zoom / wheel) the renderer sets it `true` and stops
/// recomputing bounds each frame. The right-click "Reset zoom" menu flips
/// it back to `false`.
#[derive(Debug, Clone, Default)]
pub struct PanelState {
    /// `true` once the user has interacted with the plot (drag / wheel /
    /// box-zoom). While locked, the renderer keeps whatever bounds
    /// `egui_plot` is showing and never reapplies its rolling X-window or
    /// auto-bounds-Y reset.
    pub locked: bool,
    /// v0.11.0 — last rendered top-left of the cell rect, in screen pixels.
    /// Snapshotted at the END of each frame so the next frame can detect a
    /// position change (cell moved due to drag-reorder, delete-then-compact,
    /// or hide-then-compact) and animate the transition. `None` until the
    /// cell paints for the first time.
    pub anim_last_rect_min: Option<(f32, f32)>,
    /// v0.11.0 — origin point for the active reflow animation (start of the
    /// lerp). `None` when no animation is in flight.
    pub anim_origin: Option<(f32, f32)>,
    /// v0.11.0 — animation progress, `0.0` (just started) → `1.0` (done).
    /// `None` when no animation is in flight. Driven by frame `dt`.
    pub anim_t: Option<f32>,
}

/// v0.11.0 — in-flight drag-to-reorder state captured during the first pass of
/// the grid renderer. Used by the second pass (cell drawing) to short-circuit
/// the dragged cell's normal paint and substitute a "moving from here" outline,
/// and by the post-loop overlay pass to render the floating cursor copy.
struct DragSession {
    /// Cell's persisted `(row, col)` (used for the emitted action).
    from: (usize, usize),
    /// Layout-grid `(r, c)` the cell was occupying — may differ from `from`
    /// under `compact_hidden = true`.
    layout_rc: (usize, usize),
    /// Original rect of the cell at the start of the frame (before any drag
    /// offset is applied).
    rect_origin: Rect,
    /// Cumulative drag offset (sum of `drag_delta` since drag started).
    offset: Vec2,
    /// Last known pointer position (for hit-testing into snap targets).
    pointer: Pos2,
    /// `true` on the frame where the drag was released (drop event).
    released: bool,
}

/// v0.11.0 — stable egui interaction id for a cell's drag handle. Keying by
/// `(row, col)` keeps the id consistent across frames when the cell doesn't
/// move; on relocation a fresh id is allocated which restarts the per-cell
/// offset memory.
fn drag_id_for_cell(cell: &Cell) -> Id {
    Id::new(("hvn-profiler-cell-drag", cell.row, cell.col))
}

/// v0.10.0 — per-cell context-menu action emitted by the renderer for the CLI
/// to apply between frames. Captured by the parent (`profiler-cli`) so it can
/// open the Edit modal, mutate `UiState`, prompt for delete, etc.
#[derive(Debug, Clone, PartialEq)]
pub enum CellMenuAction {
    /// "Edit panel..." — open the editor modal pre-filled from this cell.
    Edit { row: usize, col: usize },
    /// "Hide panel" toggle — flip the visibility bit in `UiState`.
    HideToggle { row: usize, col: usize },
    /// "Reset zoom" — clear the auto-scale lock for this cell.
    ResetZoom { row: usize, col: usize },
    /// "Label: off/data/metadata" — override the label mode for this cell.
    SetLabelMode { row: usize, col: usize, mode: LabelMode },
    /// "Delete panel" — drop the cell from the template (with confirm).
    Delete { row: usize, col: usize },
    /// v0.11.0 — drag-to-reorder dropped a panel from `from` onto another
    /// occupied slot at `to`. The CLI swaps the two cells' `(row, col)` and
    /// runs `compact_cells` to tidy.
    SwapTo {
        from: (usize, usize),
        to: (usize, usize),
    },
    /// v0.11.0 — drag-to-reorder dropped a panel from `from` onto an empty
    /// grid slot at `to`. The CLI relocates the cell, then compacts.
    MoveTo {
        from: (usize, usize),
        to: (usize, usize),
    },
}

/// Global label-mode override applied to every cell at render time.
///
/// `Respect` honours each cell's own `label_mode` (the default — matches what
/// the JSON template asked for). `Force(mode)` overrides every cell to render
/// in `mode` regardless of the template, used by the v0.5.0 toolbar selector
/// and `--labels` CLI flag.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum LabelOverride {
    /// Honour each cell's own `label_mode` from the template.
    #[default]
    Respect,
    /// Globally force every cell to this label mode.
    Force(LabelMode),
}

impl LabelOverride {
    /// Resolve the effective label mode for a cell.
    pub fn resolve(self, cell_mode: LabelMode) -> LabelMode {
        match self {
            LabelOverride::Respect => cell_mode,
            LabelOverride::Force(m) => m,
        }
    }
}

use crate::TraceStore;

// ─── v0.11.0 — Responsive grid sizing ───────────────────────────────────────
//
// The 2D grid reflows when the available width gets cramped so titles, plot
// data, and label overlays don't overlap. Two break-points:
//
//   width / declared_cols < MIN_CELL_W      → halve the column count
//   width / 1            < SINGLE_COL_W     → fall through to single column
//                                              (one column, N rows)
//
// The fall-through is independent of the declared template grid; once we hit
// single-column mode each visible cell gets its own row. The renderer still
// produces the same `at[]` ordering (top-to-bottom, left-to-right of the
// ORIGINAL grid) so drag-to-reorder's `from`/`to` action coordinates remain
// stable — only the visual layout slot a cell occupies changes.

/// Minimum cell width (in pixels) before the renderer halves the column count.
/// Below this, titles + plot grid + legend collide.
pub const RESPONSIVE_MIN_CELL_W: f32 = 240.0;

/// Below this total available width (in pixels), regardless of cell count, the
/// renderer reflows to a single column. Matches the user's "very narrow window"
/// case where 2-up is still too cramped.
pub const RESPONSIVE_SINGLE_COL_W: f32 = 480.0;

/// Pixel threshold for the 3D side panel collapse: below this WINDOW width,
/// the Split view hides the 3D pane (the user can re-open it via the toolbar
/// toggle / "view: 3D view" radio).
pub const RESPONSIVE_3D_COLLAPSE_W: f32 = 1100.0;

/// Compute the effective `(rows, cols)` for a responsive grid.
///
/// `visible_count` is the number of cells we need to lay out; `declared_cols`
/// is the template's declared column count (used as the upper bound at wide
/// widths). The result is the `(rows, cols)` pair the renderer should pack
/// the visible cells into.
///
/// Behaviour:
/// - width / declared_cols >= MIN_CELL_W      → keep declared_cols
/// - width / (declared_cols/2) >= MIN_CELL_W  → halve, recompute rows
/// - width >= SINGLE_COL_W                    → halve again until each cell ≥ MIN_CELL_W
/// - otherwise                                → single column, one cell per row
///
/// `visible_count == 0` returns `(1, declared_cols.max(1))` so an empty grid
/// still has a valid rect to draw into.
pub fn responsive_grid_dims(
    available_width: f32,
    visible_count: usize,
    declared_cols: usize,
) -> (usize, usize) {
    let declared = declared_cols.max(1);
    let count = visible_count.max(1);

    // Very narrow: fall through to single column regardless of declared.
    if available_width < RESPONSIVE_SINGLE_COL_W {
        return (count, 1);
    }

    // Try the declared column count, then halve until each cell is ≥ MIN_CELL_W.
    let mut cols = declared;
    while cols > 1 && (available_width / cols as f32) < RESPONSIVE_MIN_CELL_W {
        cols = (cols / 2).max(1);
    }
    let rows = count.div_ceil(cols).max(1);
    (rows, cols)
}

/// Per-frame render stats, surfaced to the CLI for the 1 Hz status log.
#[derive(Debug, Clone, Copy, Default)]
pub struct GridStats {
    /// Total visible panels laid out this frame.
    pub panels: usize,
    /// Visible panels that found at least one point for one of their sources.
    pub panels_with_data: usize,
    /// Distinct store keys referenced by the grid that currently hold data.
    pub keys_with_data: usize,
}

/// Render the full template grid into `ui`.
///
/// Returns [`GridStats`] for logging. Panels are laid out row-major using a
/// nested `vertical`/`horizontal` split with equal-sized cells. Invisible
/// cells (`visible: false`) still reserve their grid slot so the layout keeps
/// the template's row/column alignment.
pub fn render_template_grid(ui: &mut egui::Ui, tpl: &Template, store: &TraceStore) -> GridStats {
    render_template_grid_with_override(ui, tpl, store, LabelOverride::default())
}

/// v0.10.0 — knobs that change how the 2D grid renders without changing the
/// data it draws.
///
/// `Default` matches the v0.9.0 behaviour bit-for-bit: no per-cell mutable
/// state, no context menu, no interactivity. The CLI opts in by passing its
/// owned `PanelState` map + a sink for emitted menu actions.
pub struct GridRenderOptions<'a> {
    /// When `Some`, enables egui_plot interactivity (drag / zoom / box-zoom /
    /// double-click reset) and uses the per-cell `PanelState` to remember
    /// the auto-scale lock across frames. Each cell's state lives in this
    /// map keyed by `(row, col)`; the entry is created lazily on first paint.
    pub panel_states: Option<&'a mut HashMap<(usize, usize), PanelState>>,
    /// When `Some`, the renderer will surface a right-click context menu on
    /// each visible 2D panel; user clicks push a [`CellMenuAction`] into
    /// this sink for the CLI to handle next frame.
    pub menu_sink: Option<&'a mut Vec<CellMenuAction>>,
    /// Override visibility per `(row, col)`. Used by the v0.10.0 "Hide panel"
    /// flow; the underlying `tpl.cells[].visible` stays at the template's
    /// default, while this map carries the runtime override.
    pub visibility_override: Option<&'a HashMap<(usize, usize), bool>>,
    /// v0.10.2 — when `true`, hidden cells (either `cell.visible == false` or
    /// flipped off via `visibility_override`) are compacted OUT of the visible
    /// grid: remaining visible cells reflow top-to-bottom, left-to-right to
    /// fill the gap. The template itself is NOT mutated — the hidden cells
    /// keep their original `(row, col)` in `tpl.cells` so "Restore" can put
    /// them back. Defaults to `true` (the v0.10.2 behaviour); set to `false`
    /// to get the v0.10.1 behaviour where hidden cells leave blank slots.
    pub compact_hidden: bool,
    /// v0.11.0 — enable drag-to-reorder: each cell rect becomes a drag handle
    /// that, on drop, emits `CellMenuAction::SwapTo` (onto an occupied slot)
    /// or `CellMenuAction::MoveTo` (onto an empty slot). No-op if `menu_sink`
    /// is `None` (the renderer needs a sink to surface the action). Defaults
    /// to `true`.
    pub drag_to_reorder: bool,
    /// v0.11.0 — enable reflow animation: when a cell's last-rendered top-left
    /// differs from this frame's target, the renderer lerps from the previous
    /// position to the new one over ~150 ms. Pure visual — the underlying
    /// `(row, col)` logic uses the target position immediately. Defaults
    /// to `true`.
    pub animate_reflow: bool,
    /// v0.11.0 — wall-clock delta for the current frame (in seconds). Used to
    /// advance the per-cell animation progress. The CLI passes
    /// `ctx.input(|i| i.stable_dt as f32)`. Defaults to `0.0` (animation
    /// effectively pauses).
    pub frame_dt: f32,
}

impl<'a> Default for GridRenderOptions<'a> {
    fn default() -> Self {
        Self {
            panel_states: None,
            menu_sink: None,
            visibility_override: None,
            compact_hidden: true,
            drag_to_reorder: true,
            animate_reflow: true,
            frame_dt: 0.0,
        }
    }
}

/// Same as [`render_template_grid`], with an explicit [`LabelOverride`] applied
/// uniformly to every cell. The default-arg helper above forwards `Respect`.
///
/// ## Layout (v0.9.0)
///
/// Cells are positioned by computing an absolute pixel `Rect` for each
/// `(row, col)` slot up-front, then drawing each cell into its own rect via
/// `scope_builder(UiBuilder::new().max_rect(rect), …)`. This replaces the
/// v0.8.0 `ui.vertical { ui.horizontal { allocate_ui } }` chain which let the
/// per-cell `Plot` grow past `cell_h` and visually overlap the next row.
///
/// Section labels (the template's `sections` array) are reserved as a thin
/// header strip ABOVE each anchor row, so they never collide with cell titles.
pub fn render_template_grid_with_override(
    ui: &mut egui::Ui,
    tpl: &Template,
    store: &TraceStore,
    label_override: LabelOverride,
) -> GridStats {
    render_template_grid_full(ui, tpl, store, label_override, GridRenderOptions::default())
}

/// v0.10.0 — render the grid with full options: interactivity, context menus,
/// runtime visibility overrides. Default-equivalent for callers that only pass
/// a [`LabelOverride`] is [`render_template_grid_with_override`].
pub fn render_template_grid_full(
    ui: &mut egui::Ui,
    tpl: &Template,
    store: &TraceStore,
    label_override: LabelOverride,
    opts: GridRenderOptions<'_>,
) -> GridStats {
    let declared_cols = tpl.grid.cols.max(1);

    // Compute the absolute rect we get to draw inside FIRST so the responsive
    // breakpoints can drive the column count.
    let avail = ui.available_rect_before_wrap();

    // v0.10.2 — when `compact_hidden` is set, hidden cells (either
    // `cell.visible == false` or flipped off in `visibility_override`) skip
    // the layout entirely; visible cells reflow into a tightly-packed grid
    // top-to-bottom, left-to-right. The template itself is NOT mutated —
    // hidden cells keep their original coords in `tpl.cells` so a future
    // "Restore" restores them in place.
    //
    // When `compact_hidden` is false (v0.10.1 behaviour), each cell stays at
    // its declared `(row, col)` and hidden slots render as gaps.
    //
    // v0.11.0 — responsive reflow: when the available width / declared_cols
    // drops below `RESPONSIVE_MIN_CELL_W`, halve `cols`. Below
    // `RESPONSIVE_SINGLE_COL_W` total width, fall through to a single column
    // regardless of the declared layout. Hidden cells are still elided; the
    // visible-cell order (top-to-bottom, left-to-right of the ORIGINAL grid)
    // is preserved so drag actions stay anchored to the cell's persisted
    // (row, col) — only the visual slot the cell occupies changes.
    let (rows, cols, at) = if opts.compact_hidden {
        let mut visible_cells: Vec<&Cell> = tpl
            .cells
            .iter()
            .filter(|c| {
                if !c.visible {
                    return false;
                }
                opts.visibility_override
                    .as_ref()
                    .and_then(|m| m.get(&(c.row, c.col)).copied())
                    .unwrap_or(true)
            })
            .collect();
        // Preserve top-to-bottom, left-to-right visual order — matches
        // `compact_cells` so the on-disk save and the in-memory layout agree
        // after a "Save".
        visible_cells.sort_by_key(|c| (c.row, c.col));
        let (rows, cols) =
            responsive_grid_dims(avail.width(), visible_cells.len(), declared_cols);
        let mut at: Vec<Option<&Cell>> = vec![None; rows * cols];
        for (i, c) in visible_cells.iter().enumerate() {
            at[i] = Some(*c);
        }
        (rows, cols, at)
    } else {
        // Non-compact mode honours the template's declared grid coordinates,
        // but we still down-shift `cols` when the window is cramped. Cells
        // whose declared `col` no longer fits the responsive `cols` are
        // re-packed into row-major order so they don't get clipped.
        let declared_rows = tpl.grid.rows.max(1);
        let mut placed: Vec<&Cell> = tpl
            .cells
            .iter()
            .filter(|c| c.row < declared_rows && c.col < declared_cols)
            .collect();
        placed.sort_by_key(|c| (c.row, c.col));
        let (rows, cols) = if (avail.width() / declared_cols as f32) >= RESPONSIVE_MIN_CELL_W
            && avail.width() >= RESPONSIVE_SINGLE_COL_W
        {
            // Wide enough → original behaviour: keep declared grid intact.
            let mut at: Vec<Option<&Cell>> =
                vec![None; declared_rows * declared_cols];
            for c in &placed {
                at[c.row * declared_cols + c.col] = Some(*c);
            }
            return finish_render_full(
                ui, tpl, store, label_override, opts, declared_rows, declared_cols, at, avail,
            );
        } else {
            // Cramped → re-pack into responsive dims.
            responsive_grid_dims(avail.width(), placed.len(), declared_cols)
        };
        let mut at: Vec<Option<&Cell>> = vec![None; rows * cols];
        for (i, c) in placed.iter().enumerate() {
            at[i] = Some(*c);
        }
        (rows, cols, at)
    };

    finish_render_full(ui, tpl, store, label_override, opts, rows, cols, at, avail)
}

/// v0.11.0 — extracted body of `render_template_grid_full` once the responsive
/// `(rows, cols)` and packed `at[]` have been decided. Keeps the two branches
/// of the responsive dispatcher (compact-hidden vs. honour-declared) from
/// duplicating the post-layout drag/animate/render pipeline.
#[allow(clippy::too_many_arguments)]
fn finish_render_full(
    ui: &mut egui::Ui,
    tpl: &Template,
    store: &TraceStore,
    label_override: LabelOverride,
    mut opts: GridRenderOptions<'_>,
    rows: usize,
    cols: usize,
    at: Vec<Option<&Cell>>,
    avail: Rect,
) -> GridStats {
    // v0.11.0 — when the responsive layout has collapsed the grid to a single
    // column AND the cell count exceeds what fits vertically, wrap the whole
    // grid in a vertical ScrollArea so the operator can scroll instead of
    // squishing cells below their minimum height.
    let single_col = cols == 1 && rows > 2;
    if single_col {
        let mut stats_out = GridStats::default();
        egui::ScrollArea::vertical()
            .auto_shrink([false; 2])
            .show(ui, |ui| {
                // Re-fetch the rect inside the scroll viewport.
                let avail = ui.available_rect_before_wrap();
                stats_out = render_grid_body(
                    ui, tpl, store, label_override, &mut opts, rows, cols, &at, avail,
                );
            });
        return stats_out;
    }
    render_grid_body(
        ui, tpl, store, label_override, &mut opts, rows, cols, &at, avail,
    )
}

/// v0.11.0 — pure layout + paint, no responsive-mode decisions. Split out so
/// the single-column path can wrap it in a `ScrollArea`.
#[allow(clippy::too_many_arguments)]
fn render_grid_body(
    ui: &mut egui::Ui,
    tpl: &Template,
    store: &TraceStore,
    label_override: LabelOverride,
    opts: &mut GridRenderOptions<'_>,
    rows: usize,
    cols: usize,
    at: &[Option<&Cell>],
    avail: Rect,
) -> GridStats {
    let layout = compute_layout(avail, rows, cols, &tpl.sections);

    let mut stats = GridStats::default();

    // ── Section banners (drawn first so cell titles paint on top) ───────────
    for (sec, rect) in &layout.section_rects {
        draw_section_banner(ui, sec, *rect);
    }

    // ── Cells ───────────────────────────────────────────────────────────────
    let interactive = opts.panel_states.is_some();
    let drag_enabled = opts.drag_to_reorder && opts.menu_sink.is_some() && interactive;
    let animate = opts.animate_reflow && interactive;
    // v0.11.0 — clamp frame_dt so animations don't snap to the end when the
    // app un-minimizes / the window regains focus after a long pause.
    let frame_dt = opts.frame_dt;
    let anim_active = animate && frame_dt > 0.0 && frame_dt < 0.5;

    // v0.11.0 — first pass: discover the cell currently being dragged (so we
    // can render it at its drag-offset position instead of its grid slot).
    // We also collect the per-slot target rects so the drop hit-test in the
    // second pass can detect snap zones.
    let mut dragging: Option<DragSession> = None;
    if drag_enabled {
        for r in 0..rows {
            for c in 0..cols {
                let id = r * cols + c;
                let Some(cell) = at[id] else { continue };
                // v0.12.0 — Status primitive carries its source in
                // `cell.source` (single string) rather than `cell.sources`,
                // so accept it even when `cell.sources` is empty.
                // v0.14.0 — InfoText primitive has no source at all (static
                // literal text), so also accept empty `sources` for it.
                if !cell.visible
                    || (cell.sources.is_empty()
                        && cell.primitive != Primitive::Status
                        && cell.primitive != Primitive::InfoText)
                {
                    continue;
                }
                let rect = layout.cell_rect(r, c);
                // v0.11.0 — drag handle is the cell's title strip (top
                // ~18 px), so the inner plot keeps its own drag/zoom for
                // pan and box-zoom interactions. The strip spans the full
                // cell width and is tall enough to grab comfortably.
                let handle_rect = Rect::from_min_size(
                    rect.min,
                    Vec2::new(rect.width(), 18.0_f32.min(rect.height())),
                );
                let interact_id = drag_id_for_cell(cell);
                let resp = ui.interact(handle_rect, interact_id, Sense::drag());
                if resp.hovered() {
                    ui.ctx().set_cursor_icon(CursorIcon::Grab);
                }
                if resp.dragged() || resp.drag_started() {
                    let delta = resp.drag_delta();
                    // Cumulative offset persisted in a memory id so each
                    // frame of the drag keeps moving relative to the rect.
                    let prev_off: Vec2 = ui
                        .ctx()
                        .memory(|m| m.data.get_temp(interact_id))
                        .unwrap_or_default();
                    let off = if resp.drag_started() {
                        Vec2::ZERO
                    } else {
                        prev_off + delta
                    };
                    ui.ctx()
                        .memory_mut(|m| m.data.insert_temp(interact_id, off));
                    dragging = Some(DragSession {
                        from: (cell.row, cell.col),
                        layout_rc: (r, c),
                        rect_origin: rect,
                        offset: off,
                        pointer: resp.interact_pointer_pos().unwrap_or(rect.center()),
                        released: false,
                    });
                } else if resp.drag_stopped() {
                    let prev_off: Vec2 = ui
                        .ctx()
                        .memory(|m| m.data.get_temp(interact_id))
                        .unwrap_or_default();
                    ui.ctx()
                        .memory_mut(|m| m.data.remove_temp::<Vec2>(interact_id));
                    dragging = Some(DragSession {
                        from: (cell.row, cell.col),
                        layout_rc: (r, c),
                        rect_origin: rect,
                        offset: prev_off,
                        pointer: resp.interact_pointer_pos().unwrap_or(rect.center()),
                        released: true,
                    });
                }
            }
        }
        if dragging.is_some() {
            ui.ctx().set_cursor_icon(CursorIcon::Grabbing);
        }
    }

    // v0.11.0 — second pass: figure out the snap target (slot whose center
    // the dragged panel's center is closest to, within the grid). Used to
    // highlight + to compute the drop action on `released`.
    let snap_target: Option<(usize, usize)> = dragging.as_ref().and_then(|ds| {
        let dragged_center = ds.rect_origin.center() + ds.offset;
        // Only snap when the pointer is inside the grid bounds.
        if !layout.total_used.expand(8.0).contains(ds.pointer) {
            return None;
        }
        let mut best: Option<((usize, usize), f32)> = None;
        for r in 0..rows {
            for c in 0..cols {
                if (r, c) == ds.layout_rc {
                    continue;
                }
                let target_rect = layout.cell_rect(r, c);
                let d = (target_rect.center() - dragged_center).length();
                if best.is_none_or(|b| d < b.1) {
                    best = Some(((r, c), d));
                }
            }
        }
        best.map(|b| b.0)
    });

    for r in 0..rows {
        for c in 0..cols {
            let id = r * cols + c;
            let target_rect = layout.cell_rect(r, c);
            // When `compact_hidden` is true the at-grid only contains visible
            // cells (we filtered above), so the runtime override check is
            // redundant. When false, honour the per-slot override at the
            // SLOT coordinates so the v0.10.1 behaviour is preserved.
            let runtime_visible = if opts.compact_hidden {
                true
            } else {
                opts.visibility_override
                    .as_ref()
                    .and_then(|m| m.get(&(r, c)).copied())
                    .unwrap_or(true)
            };

            // v0.11.0 — if a drag is in flight and this slot owns the dragged
            // cell, paint a dashed "moving from here" outline at the original
            // slot. The dragged cell itself is rendered later as a top-layer
            // overlay (so cancel/no-op snaps it back here automatically).
            let is_drag_source = dragging
                .as_ref()
                .is_some_and(|d| d.layout_rc == (r, c));

            // v0.11.0 — animated reflow: interpolate the rect's top-left from
            // the previously-rendered position toward the target.
            let mut render_rect = target_rect;
            if anim_active {
                if let Some(map) = opts.panel_states.as_deref_mut() {
                    if let Some(cell) = at[id] {
                        let state_key = (cell.row, cell.col);
                        let st = map.entry(state_key).or_default();
                        // Detect a position change vs. the previous frame.
                        if let Some(prev) = st.anim_last_rect_min {
                            let prev_pos = Pos2::new(prev.0, prev.1);
                            if (prev_pos - target_rect.min).length() > 0.5
                                && st.anim_t.is_none()
                            {
                                st.anim_origin = Some(prev);
                                st.anim_t = Some(0.0);
                            }
                        }
                        // Advance + apply animation if in flight.
                        if let (Some(origin), Some(t)) = (st.anim_origin, st.anim_t) {
                            let new_t = (t + frame_dt / 0.150).clamp(0.0, 1.0);
                            let eased = 1.0 - (1.0 - new_t).powi(3);
                            let origin_pos = Pos2::new(origin.0, origin.1);
                            let lerped = origin_pos.lerp(target_rect.min, eased);
                            render_rect = Rect::from_min_size(lerped, target_rect.size());
                            if new_t >= 1.0 {
                                st.anim_origin = None;
                                st.anim_t = None;
                            } else {
                                st.anim_t = Some(new_t);
                                // Request a repaint so the animation keeps
                                // advancing even if nothing else is dirty.
                                ui.ctx().request_repaint();
                            }
                        }
                    }
                }
            }

            // Always claim the rect so the next row's `available_rect`
            // computation downstream of the grid is correct, even when a
            // slot is empty.
            ui.scope_builder(UiBuilder::new().max_rect(target_rect), |ui| {
                ui.set_clip_rect(target_rect);
                match at[id] {
                    Some(cell)
                        if cell.visible
                            && runtime_visible
                            && (!cell.sources.is_empty()
                                || cell.primitive == Primitive::Status
                                || cell.primitive == Primitive::InfoText) => {
                        if is_drag_source {
                            // Draw a dashed outline + dimmed fill where the
                            // panel WAS. The actual content paints on the
                            // top layer below.
                            let painter = ui.painter_at(target_rect);
                            painter.rect_filled(
                                target_rect,
                                4.0,
                                Color32::from_black_alpha(40),
                            );
                            painter.rect_stroke(
                                target_rect,
                                4.0,
                                Stroke::new(1.0, Color32::from_white_alpha(80)),
                                StrokeKind::Inside,
                            );
                            // Still count this panel toward stats so logs
                            // don't blink during a drag.
                            stats.panels += 1;
                            // Snapshot last-rect for animation on next frame.
                            if let Some(map) = opts.panel_states.as_deref_mut() {
                                let st = map.entry((cell.row, cell.col)).or_default();
                                st.anim_last_rect_min =
                                    Some((target_rect.min.x, target_rect.min.y));
                            }
                            return;
                        }
                        stats.panels += 1;
                        // Snap-zone highlight on the target.
                        if snap_target == Some((r, c)) {
                            let painter = ui.painter_at(target_rect);
                            painter.rect_filled(
                                target_rect,
                                4.0,
                                Color32::from_white_alpha(40),
                            );
                        }
                        // Pull the lock state for this cell. Under
                        // `compact_hidden` mode the (r, c) we draw at is a
                        // layout slot, not the cell's persisted coordinates;
                        // we key panel state by the CELL's own (row, col) so
                        // zoom/lock state survives reflow when other cells
                        // are hidden/shown.
                        let state_key = (cell.row, cell.col);
                        let mut local_locked = false;
                        let panel_locked = match &mut opts.panel_states {
                            Some(map) => {
                                let st = map.entry(state_key).or_default();
                                &mut st.locked
                            }
                            None => &mut local_locked,
                        };
                        // v0.16.1 — wrap every cell render in the unified
                        // right-click context-menu overlay so Status (v0.12.0)
                        // and InfoText (v0.14.0) primitives surface the same
                        // menu as 2D plot cells. The wrapper allocates a
                        // transparent `Sense::click()` interaction over the
                        // full cell rect AFTER the inner render — guaranteed
                        // to catch right-click regardless of what the inner
                        // code drew (egui_plot widget, painter-only frame,
                        // status chip, info-text spans).
                        let (had_data, _plot_resp) = if let Some(sink) =
                            opts.menu_sink.as_deref_mut()
                        {
                            wrap_cell_with_context_menu(ui, cell, render_rect, sink, |ui| {
                                render_cell(
                                    ui,
                                    cell,
                                    store,
                                    store.window_s,
                                    label_override,
                                    render_rect,
                                    interactive,
                                    panel_locked,
                                )
                            })
                        } else {
                            render_cell(
                                ui,
                                cell,
                                store,
                                store.window_s,
                                label_override,
                                render_rect,
                                interactive,
                                panel_locked,
                            )
                        };
                        if had_data {
                            stats.panels_with_data += 1;
                        }
                        // Snapshot for next frame's animation detection. Use
                        // the TARGET (logical) rect so the animation chases
                        // the new layout slot, not the lerped intermediate.
                        if let Some(map) = opts.panel_states.as_deref_mut() {
                            let st = map.entry((cell.row, cell.col)).or_default();
                            st.anim_last_rect_min =
                                Some((target_rect.min.x, target_rect.min.y));
                        }
                    }
                    _ => {
                        // Empty slot — paint a snap highlight if a drag is
                        // hovering over it, so the operator sees where it'd
                        // land.
                        if snap_target == Some((r, c)) {
                            let painter = ui.painter_at(target_rect);
                            painter.rect_filled(
                                target_rect,
                                4.0,
                                Color32::from_white_alpha(40),
                            );
                        }
                    }
                }
            });
        }
    }

    // v0.11.0 — drag overlay: render a semi-transparent copy of the dragged
    // panel at the pointer offset, on a top layer so it floats above the
    // grid. Use the cell's content but with reduced opacity via a tint.
    if let Some(ds) = &dragging {
        let cell = at[ds.layout_rc.0 * cols + ds.layout_rc.1];
        if let Some(cell) = cell {
            let overlay_rect = ds.rect_origin.translate(ds.offset);
            let layer = LayerId::new(Order::Tooltip, Id::new(("hvn-profiler-drag", cell.row, cell.col)));
            let painter = ui.ctx().layer_painter(layer);
            painter.rect_filled(overlay_rect, 4.0, Color32::from_rgba_unmultiplied(40, 80, 140, 180));
            painter.rect_stroke(
                overlay_rect,
                4.0,
                Stroke::new(1.5, Color32::from_white_alpha(180)),
                StrokeKind::Inside,
            );
            if !cell.title.is_empty() {
                painter.text(
                    overlay_rect.left_top() + Vec2::new(8.0, 6.0),
                    Align2::LEFT_TOP,
                    &cell.title,
                    egui::FontId::proportional(13.0),
                    Color32::from_gray(240),
                );
            }
            painter.text(
                overlay_rect.center(),
                Align2::CENTER_CENTER,
                cell.sources
                    .first()
                    .map(|s| s.key.as_str())
                    .unwrap_or(""),
                egui::FontId::proportional(12.0),
                Color32::from_white_alpha(200),
            );
        }
    }

    // v0.11.0 — drop resolution: on release, emit SwapTo / MoveTo (or no-op
    // when the snap target is the source / outside the grid).
    if let Some(ds) = &dragging {
        if ds.released {
            if let Some(sink) = opts.menu_sink.as_deref_mut() {
                if let Some(to_rc) = snap_target {
                    // Translate layout (r, c) → cell's persisted coords by
                    // looking at `at[]`. For an occupied target → SwapTo
                    // against that cell's own (row, col). For an empty slot
                    // → MoveTo to the layout slot (which is also the persisted
                    // slot when compact_hidden is false; in compact-hidden
                    // mode dropping onto an empty layout slot is the same as
                    // appending, so we use the layout coords directly).
                    let to_idx = to_rc.0 * cols + to_rc.1;
                    match at[to_idx] {
                        Some(target_cell) => {
                            sink.push(CellMenuAction::SwapTo {
                                from: ds.from,
                                to: (target_cell.row, target_cell.col),
                            });
                        }
                        None => {
                            sink.push(CellMenuAction::MoveTo {
                                from: ds.from,
                                to: to_rc,
                            });
                        }
                    }
                }
                // No snap target → drop in the gutter → no-op (snap back).
            }
        }
    }

    // Consume the full rect on the parent ui so subsequent widgets (status
    // log, etc.) advance past the grid.
    ui.allocate_space(layout.total_used.size());

    stats.keys_with_data = count_keys_with_data(tpl, store);
    stats
}

/// Test helper — compute the per-cell pixel rects a template would lay out
/// into the given window rect. Pure function: no UI side-effects.
///
/// Returns `(rect_for_(row,col), ...)` in row-major order. Indices match
/// `tpl.grid.rows × tpl.grid.cols`. Used by `tests/layout_test.rs` to assert
/// non-overlap on resize.
#[doc(hidden)]
pub fn layout_cell_rects(tpl: &Template, window: Rect) -> Vec<Rect> {
    let rows = tpl.grid.rows.max(1);
    let cols = tpl.grid.cols.max(1);
    let lay = compute_layout(window, rows, cols, &tpl.sections);
    let mut out = Vec::with_capacity(rows * cols);
    for r in 0..rows {
        for c in 0..cols {
            out.push(lay.cell_rect(r, c));
        }
    }
    out
}

/// v0.11.0 — test helper: emit the cell rects the RESPONSIVE renderer would
/// actually use for a given window. Honours the same compact-hidden + cell-count
/// reflow as `render_template_grid_full`. Returned in row-major order of the
/// EFFECTIVE grid (not the declared template grid) — index `i` is the cell at
/// position `(i / cols, i % cols)` of the rendered layout.
///
/// Returns `(rows, cols, rects)` so tests can assert the grid dims directly.
#[doc(hidden)]
pub fn responsive_cell_rects(tpl: &Template, window: Rect) -> (usize, usize, Vec<Rect>) {
    let declared_cols = tpl.grid.cols.max(1);
    let visible_count = tpl
        .cells
        .iter()
        .filter(|c| c.visible)
        .count();
    let (rows, cols) = responsive_grid_dims(window.width(), visible_count, declared_cols);
    let lay = compute_layout(window, rows, cols, &tpl.sections);
    let mut rects = Vec::with_capacity(rows * cols);
    for r in 0..rows {
        for c in 0..cols {
            rects.push(lay.cell_rect(r, c));
        }
    }
    (rows, cols, rects)
}

/// Precomputed pixel rectangles for every grid cell + section banner.
///
/// Built once per frame from `available_rect × grid.rows × grid.cols`. Each
/// cell gets an equal `(cell_w, cell_h)` slot; section banners are skinny
/// strips reserved ABOVE their anchor rows so they don't overlap titles.
struct GridLayout {
    /// Top-left of the cell area (after any leading section banner space).
    origin: egui::Pos2,
    cell_w: f32,
    cell_h: f32,
    /// For each section: (clone of section, banner rect). Drawn FIRST so cell
    /// titles overlay them at the row.
    section_rects: Vec<(Section, Rect)>,
    /// Per-row vertical offset (in pixels) accounting for the banner strips
    /// above that row. `row_y_offset[r]` is added to `origin.y + r * cell_h`.
    row_y_offset: Vec<f32>,
    /// Full bounding rect (used to advance the parent ui after layout).
    total_used: Rect,
}

impl GridLayout {
    fn cell_rect(&self, r: usize, c: usize) -> Rect {
        let x0 = self.origin.x + (c as f32) * self.cell_w;
        let y0 = self.origin.y + (r as f32) * self.cell_h + self.row_y_offset[r];
        // 4 px internal margin keeps adjacent plots from kissing.
        let pad = 3.0;
        Rect::from_min_size(
            egui::pos2(x0 + pad, y0 + pad),
            egui::vec2(self.cell_w - 2.0 * pad, self.cell_h - 2.0 * pad),
        )
    }
}

/// Compute pixel rects from the available rect + grid dims + sections.
fn compute_layout(avail: Rect, rows: usize, cols: usize, sections: &[Section]) -> GridLayout {
    const SECTION_BANNER_H: f32 = 18.0;

    // Per-row banner height contribution (added ABOVE the row).
    let mut row_extra = vec![0.0_f32; rows];
    for sec in sections {
        if sec.label.is_empty() {
            continue;
        }
        if sec.anchor_row < rows {
            row_extra[sec.anchor_row] += SECTION_BANNER_H;
        }
    }

    // Cumulative offsets so cell_rect(r, _) knows how much banner space sits
    // above it.
    let mut row_y_offset = vec![0.0_f32; rows];
    let mut acc = 0.0_f32;
    for r in 0..rows {
        row_y_offset[r] = acc;
        acc += row_extra[r];
    }
    let total_banner_h: f32 = row_extra.iter().sum();

    // Remaining height divided across `rows` cells.
    let usable_h = (avail.height() - total_banner_h).max(rows as f32 * 60.0);
    let cell_h = (usable_h / rows as f32).max(60.0);
    let cell_w = (avail.width() / cols as f32).max(80.0);

    let origin = avail.min;

    // Build banner rects in absolute pixel coords.
    let mut section_rects: Vec<(Section, Rect)> = Vec::new();
    for sec in sections {
        if sec.label.is_empty() || sec.anchor_row >= rows {
            continue;
        }
        let r = sec.anchor_row;
        let y0 = origin.y + (r as f32) * cell_h + row_y_offset[r] - SECTION_BANNER_H;
        let rect = Rect::from_min_size(
            egui::pos2(origin.x, y0.max(origin.y)),
            egui::vec2(cell_w * cols as f32, SECTION_BANNER_H),
        );
        section_rects.push((sec.clone(), rect));
    }

    let total_h = (rows as f32) * cell_h + total_banner_h;
    let total_used =
        Rect::from_min_size(origin, egui::vec2(cell_w * cols as f32, total_h));

    GridLayout {
        origin,
        cell_w,
        cell_h,
        section_rects,
        row_y_offset,
        total_used,
    }
}

/// Paint a section banner: solid colored bar with the label centered-left.
fn draw_section_banner(ui: &mut egui::Ui, sec: &Section, rect: Rect) {
    let bg = parse_color(&sec.color).unwrap_or(Color32::from_rgb(0x33, 0x44, 0x55));
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 2.0, bg.linear_multiply(0.35));
    painter.text(
        rect.left_center() + egui::vec2(6.0, 0.0),
        Align2::LEFT_CENTER,
        &sec.label,
        egui::FontId::proportional(12.0),
        Color32::from_gray(230),
    );
}

/// How many distinct store keys referenced by the template currently have data.
fn count_keys_with_data(tpl: &Template, store: &TraceStore) -> usize {
    use std::collections::BTreeSet;
    let mut keys: BTreeSet<String> = BTreeSet::new();
    for cell in tpl.visible_cells() {
        for src in &cell.sources {
            // Expand vector / array-base keys into their components.
            match cell.primitive {
                Primitive::Vector
                | Primitive::Magnitude
                | Primitive::MagInterference
                | Primitive::AttitudeRpy => {
                    for i in 0..3 {
                        keys.insert(format!("{}[{}]", src.key, i));
                    }
                }
                Primitive::Diff => {
                    keys.insert(src.key.clone());
                    if let Some(m) = &src.minus {
                        keys.insert(m.clone());
                    }
                }
                _ => {
                    keys.insert(src.key.clone());
                    if let Some(f) = &src.fallback {
                        keys.insert(f.clone());
                    }
                }
            }
        }
        // v0.12.0 — status primitive uses `Cell::source` (single key) instead
        // of `Cell::sources`. Include it in the data-key tally so the status
        // log reports the panel as "live".
        if cell.primitive == Primitive::Status && !cell.source.is_empty() {
            keys.insert(cell.source.clone());
        }
    }
    keys.into_iter()
        .filter(|k| {
            store.len(k) > 0
                || store.latest_string(k).is_some()
                || !store.text_log_owned(k).is_empty()
        })
        .count()
}

/// Render a single panel. Returns `(any_data, plot_response)`.
///
/// `cell_rect` is the absolute pixel rect the cell owns; the plot height is
/// forced via `.height(remaining)` so the Plot can never grow past the cell
/// box and bleed into the next row.
///
/// v0.10.0 — `interactive=true` enables zoom/pan/box-zoom/double-click-reset.
/// `locked` is mutated to `true` on the first interaction this frame (so the
/// next frame skips the rolling-window X reset and the auto-bounds-Y reset).
/// Double-click on the plot resets `locked` back to `false`.
#[allow(clippy::too_many_arguments)]
fn render_cell(
    ui: &mut egui::Ui,
    cell: &Cell,
    store: &TraceStore,
    window_s: f64,
    label_override: LabelOverride,
    cell_rect: Rect,
    interactive: bool,
    locked: &mut bool,
) -> (bool, egui::Response) {
    // v0.12.0 — Status primitive bypasses the plot path entirely. The cell
    // renders as a colored Frame with the value text, sized to fit the
    // cell_rect (after the title strip).
    if cell.primitive == Primitive::Status {
        return render_status_cell(ui, cell, store, cell_rect);
    }
    // v0.14.0 — InfoText primitive renders static literal text + an optional
    // icon and the cell's title. No data source is consulted.
    if cell.primitive == Primitive::InfoText {
        return render_info_text_cell(ui, cell, cell_rect);
    }
    let plot_id = format!("cell_{}_{}", cell.row, cell.col);

    // Title above the plot — measured so the plot below knows how much height
    // it has left.
    //
    // v0.11.0 — title font size scales with cell width so narrow cells stay
    // legible without overflowing. Clamped to `[10, 14]` px so we don't go
    // illegible at extreme narrow widths or balloon on a 4K monitor.
    let title_font_size = (cell_rect.width() * 0.030).clamp(10.0, 14.0);
    let title_h: f32 = if cell.title.is_empty() { 0.0 } else { title_font_size + 4.0 };
    if !cell.title.is_empty() {
        // v0.11.0 — ellipsis-truncate when the title would overflow the cell
        // width. We approximate: each char ≈ `title_font_size * 0.55` px.
        let avail_chars = ((cell_rect.width() - 12.0) / (title_font_size * 0.55)) as usize;
        let title_text: String = if cell.title.chars().count() > avail_chars && avail_chars > 3 {
            let mut s: String = cell.title.chars().take(avail_chars.saturating_sub(1)).collect();
            s.push('…');
            s
        } else {
            cell.title.clone()
        };
        ui.label(
            RichText::new(title_text)
                .strong()
                .size(title_font_size),
        );
    }

    let latest_ts = store.latest_ts();
    let x_lo = if latest_ts.is_finite() {
        latest_ts - window_s
    } else {
        0.0
    };
    let x_hi = if latest_ts.is_finite() {
        latest_ts
    } else {
        window_s
    };

    let mut any_data = false;

    let plot_h = (cell_rect.height() - title_h).max(40.0);
    let plot_w = cell_rect.width();

    // v0.11.0 — hide the legend entirely when the cell is narrower than the
    // legend would occupy any usable plot area. Below ~160 px the legend
    // covers the whole left half of the plot and the user can identify
    // traces via the title instead.
    const LEGEND_HIDE_W: f32 = 160.0;
    let show_legend = plot_w >= LEGEND_HIDE_W;

    // v0.10.0 — when interactive, enable the egui_plot zoom/pan/box-zoom
    // suite + double-click reset. The auto-scale-Y default holds UNTIL the
    // user interacts; after that, the per-cell `locked` flag stays `true`
    // until the right-click "Reset zoom" or a double-click flips it off.
    let mut plot = Plot::new(plot_id);
    if show_legend {
        // v0.10.1 — pin the legend to the top-left so it stays clear of the
        // most-recent samples on a rolling X window (which always end at the
        // RIGHT edge of the plot). With the default `RightTop` position the
        // legend overlapped the live trace tip every frame.
        plot = plot.legend(Legend::default().position(Corner::LeftTop));
    }
    let resp = plot
        .show_axes([true, true])
        .show_grid([true, true])
        // Force the plot footprint to the cell rect minus the title strip.
        // Without this the plot grows to its natural size and overlaps the
        // next row on tight layouts (the v0.8.0 bug).
        .height(plot_h)
        .width(plot_w)
        .allow_drag(interactive)
        .allow_zoom(interactive)
        .allow_scroll(interactive)
        .allow_boxed_zoom(interactive)
        .allow_double_click_reset(interactive)
        .show(ui, |plot_ui| {
            // While the plot is unlocked, drive the rolling X window and the
            // auto-scale-Y reset every frame (v0.9.0 behaviour). Once locked,
            // leave the plot's bounds untouched so the user's pan / zoom
            // persists across frames.
            if !*locked {
                if latest_ts.is_finite() {
                    plot_ui.set_plot_bounds_x(x_lo..=x_hi);
                }
                plot_ui.set_auto_bounds([false, true]);
            }

            any_data = draw_primitive(plot_ui, cell, store);

            if cell.zero_reference_line {
                plot_ui.hline(
                    HLine::new("zero", 0.0)
                        .color(Color32::from_gray(110))
                        .width(0.8),
                );
            }
        });

    // v0.10.0 — flip the auto-scale lock on the FIRST interaction (drag /
    // wheel / box-zoom). A double-click clears it so auto-scale resumes.
    if interactive {
        let r = &resp.response;
        if r.double_clicked() {
            *locked = false;
        } else if r.dragged()
            || r.clicked()
            || (r.hovered() && r.ctx.input(|i| i.smooth_scroll_delta.y).abs() > 0.0)
        {
            *locked = true;
        }
    }

    // ── Non-reflowing label overlay (v0.9.0) ────────────────────────────
    // Paint AFTER the plot so the layout was already finalised: the overlay
    // can't push the plot down by toggling on/off. The overlay uses the
    // plot's screen-space response rect as its anchor and the parent ui's
    // painter (NOT plot_ui's coordinate system), so the position is in
    // pixels regardless of what the data is doing.
    draw_label_overlay(ui, resp.response.rect, cell, store, label_override);

    (any_data, resp.response)
}

/// v0.14.0 — render an [`Primitive::InfoText`] cell.
///
/// Layout (top → bottom):
/// 1. Optional `cell.icon` painted as a large emoji glyph, centered.
/// 2. Cell `title`, centered, bold.
/// 3. Body — `cell.text` rendered with a tiny Markdown-ish syntax:
///    - `**span**` → bold span inline,
///    - line starting with `- ` → bullet (`• `) line,
///    - `\n` → hard line break,
///    - long lines wrap to the cell width.
///
/// No data source is consulted; `cell.sources` / `cell.source` are ignored.
/// Returns `(false, response)` — InfoText panels are never counted as
/// `panels_with_data` since they are static literal content.
pub fn render_info_text_cell(
    ui: &mut egui::Ui,
    cell: &Cell,
    cell_rect: Rect,
) -> (bool, egui::Response) {
    // Outer frame: dark slate panel with rounded corners + inner margin.
    let frame_color = Color32::from_rgb(40, 44, 56);
    let title_color = Color32::from_gray(230);
    let body_color = Color32::from_gray(200);

    let resp = ui
        .scope_builder(
            UiBuilder::new().max_rect(cell_rect).sense(Sense::click()),
            |ui| {
                ui.set_clip_rect(cell_rect);
                let painter = ui.painter_at(cell_rect);
                // Background panel.
                painter.rect_filled(cell_rect.shrink(2.0), 6.0, frame_color);

                let inner_margin: f32 = 12.0;
                let inner = cell_rect.shrink(inner_margin);
                let mut y = inner.min.y;

                // 1. Icon (large emoji glyph at the top).
                if let Some(icon) = cell.icon.as_deref().filter(|s| !s.is_empty()) {
                    let icon_size = (inner.height() * 0.20).clamp(20.0, 40.0);
                    painter.text(
                        Pos2::new(inner.center().x, y),
                        Align2::CENTER_TOP,
                        icon,
                        egui::FontId::proportional(icon_size),
                        title_color,
                    );
                    y += icon_size + 4.0;
                }

                // 2. Title (bold, centered).
                if !cell.title.is_empty() {
                    let title_size = (inner.width() * 0.045).clamp(12.0, 18.0);
                    painter.text(
                        Pos2::new(inner.center().x, y),
                        Align2::CENTER_TOP,
                        &cell.title,
                        egui::FontId {
                            size: title_size,
                            family: egui::FontFamily::Proportional,
                        },
                        title_color,
                    );
                    y += title_size + 8.0;
                }

                // 3. Body text. Render line-by-line; each line may have bold
                //    spans split on `**` and may begin with `- ` to indicate a
                //    bullet. Word-wraps to the cell width.
                if let Some(text) = cell.text.as_deref() {
                    let body_size = (inner.width() * 0.035).clamp(10.0, 14.0);
                    let body_font = egui::FontId {
                        size: body_size,
                        family: egui::FontFamily::Proportional,
                    };
                    let body_font_bold = egui::FontId {
                        size: body_size,
                        family: egui::FontFamily::Proportional,
                    };
                    let max_w = inner.width();
                    for raw_line in text.split('\n') {
                        // Bullet detection: `- ` prefix → render with `• `
                        // glyph. The remaining text still goes through the
                        // bold-span parser.
                        let (prefix, rest) = if let Some(r) = raw_line.strip_prefix("- ") {
                            ("• ", r)
                        } else if raw_line == "-" {
                            ("• ", "")
                        } else {
                            ("", raw_line)
                        };
                        if y > inner.max.y {
                            break;
                        }
                        // Build a galley from each span and lay them out
                        // left-to-right, wrapping when we exceed max_w.
                        let spans = parse_info_text_spans(rest);
                        let mut x = inner.min.x;
                        let baseline_y = y;
                        let mut line_y = baseline_y;
                        let mut wrote_prefix = false;
                        for (text_piece, bold) in spans {
                            // For bullet lines, paint the prefix once at the
                            // start of the FIRST span.
                            let mut piece = text_piece.clone();
                            if !wrote_prefix {
                                piece = format!("{prefix}{piece}");
                                wrote_prefix = true;
                            }
                            // Word-wrap this piece by laying out a non-wrap
                            // galley, then if it overflows breaking it word
                            // by word.
                            let font = if bold { &body_font_bold } else { &body_font };
                            // Simple word-wrap: split on spaces.
                            for word in split_keeping_spaces(&piece) {
                                let galley = painter.layout_no_wrap(
                                    word.clone(),
                                    font.clone(),
                                    body_color,
                                );
                                let w = galley.size().x;
                                if x + w > inner.min.x + max_w && x > inner.min.x {
                                    // Wrap.
                                    line_y += body_size + 2.0;
                                    x = inner.min.x;
                                }
                                if line_y + body_size > inner.max.y {
                                    break;
                                }
                                // Bold simulated by painting the text twice
                                // with a 1 px horizontal offset (egui's
                                // default font lacks a bold weight).
                                painter.galley(Pos2::new(x, line_y), galley.clone(), body_color);
                                if bold {
                                    let galley2 = painter.layout_no_wrap(
                                        word.clone(),
                                        font.clone(),
                                        body_color,
                                    );
                                    painter.galley(
                                        Pos2::new(x + 0.6, line_y),
                                        galley2,
                                        body_color,
                                    );
                                }
                                x += w;
                            }
                        }
                        // Advance y for the next line. If no spans were
                        // emitted (e.g. completely blank line), still advance
                        // a line height so blank lines act as paragraph
                        // separators.
                        y = line_y + body_size + 4.0;
                    }
                }
                // Always return false — InfoText cells don't carry "data"
                // in the rolling-window sense.
                false
            },
        );

    (resp.inner, resp.response)
}

/// v0.14.0 — parse a single InfoText line into a sequence of
/// `(text, bold)` spans. `**…**` toggles bold; everything else is regular
/// weight. Unbalanced `**` is treated as literal.
///
/// Exposed for unit testing.
#[doc(hidden)]
pub fn parse_info_text_spans(line: &str) -> Vec<(String, bool)> {
    let mut out: Vec<(String, bool)> = Vec::new();
    let mut bold = false;
    let mut buf = String::new();
    let bytes = line.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        // Look for a `**` toggle.
        if i + 1 < bytes.len() && bytes[i] == b'*' && bytes[i + 1] == b'*' {
            if !buf.is_empty() {
                out.push((std::mem::take(&mut buf), bold));
            }
            bold = !bold;
            i += 2;
            continue;
        }
        // Take one char (handle UTF-8).
        let ch_len = utf8_char_len(bytes[i]);
        let end = (i + ch_len).min(bytes.len());
        buf.push_str(&line[i..end]);
        i = end;
    }
    if !buf.is_empty() {
        out.push((buf, bold));
    }
    // If the line was empty, return a single empty span so the caller still
    // advances the y baseline (preserving blank-line paragraph separators).
    if out.is_empty() {
        out.push((String::new(), false));
    }
    out
}

/// Read the byte-length of a UTF-8 character starting at `first_byte`.
fn utf8_char_len(first_byte: u8) -> usize {
    if first_byte < 0x80 {
        1
    } else if first_byte < 0xC0 {
        // Continuation byte (shouldn't appear as first) — treat as 1 to
        // avoid an infinite loop.
        1
    } else if first_byte < 0xE0 {
        2
    } else if first_byte < 0xF0 {
        3
    } else {
        4
    }
}

/// Split a string into whitespace-preserving tokens for word-wrap:
/// each run of non-whitespace is one token, each run of whitespace is one
/// token (so trailing spaces survive the layout).
fn split_keeping_spaces(s: &str) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    let mut cur = String::new();
    let mut in_ws: Option<bool> = None;
    for ch in s.chars() {
        let is_ws = ch.is_whitespace();
        match in_ws {
            None => {
                cur.push(ch);
                in_ws = Some(is_ws);
            }
            Some(prev) if prev == is_ws => cur.push(ch),
            Some(_) => {
                out.push(std::mem::take(&mut cur));
                cur.push(ch);
                in_ws = Some(is_ws);
            }
        }
    }
    if !cur.is_empty() {
        out.push(cur);
    }
    out
}

/// v0.12.0 — render the [`Primitive::Status`] cell as a colored chip /
/// text-log instead of an egui_plot panel.
///
/// Layout: a `Frame::default().fill(color).rounding(4).inner_margin(8)` with
/// the resolved text inside. For `TextLog` kind, the latest entries are
/// listed newest-first, severity-colored.
///
/// Returns `(any_data, response)`. `any_data` is `true` when the source has
/// any resolvable value; the `response` is the interactive sense rect used
/// for the right-click context menu.
pub fn render_status_cell(
    ui: &mut egui::Ui,
    cell: &Cell,
    store: &TraceStore,
    cell_rect: Rect,
) -> (bool, egui::Response) {
    use profiler_template::StatusKind;
    let title_font_size = (cell_rect.width() * 0.030).clamp(10.0, 14.0);
    let title_h: f32 = if cell.title.is_empty() {
        0.0
    } else {
        title_font_size + 4.0
    };
    if !cell.title.is_empty() {
        ui.label(
            RichText::new(&cell.title)
                .strong()
                .size(title_font_size),
        );
    }

    let body_rect = Rect::from_min_size(
        Pos2::new(cell_rect.min.x, cell_rect.min.y + title_h),
        Vec2::new(cell_rect.width(), (cell_rect.height() - title_h).max(20.0)),
    );

    // Resolve color and content based on kind + source value.
    let kind = cell.kind.unwrap_or(StatusKind::Text);
    let default_color = cell
        .default_color
        .as_deref()
        .and_then(parse_color)
        .unwrap_or(Color32::from_gray(170));
    let key = cell.source.as_str();

    // Build the body painter inside a scope_builder so we honour body_rect.
    let resp = ui
        .scope_builder(UiBuilder::new().max_rect(body_rect).sense(Sense::click()), |ui| {
            ui.set_clip_rect(body_rect);
            match kind {
                StatusKind::Text | StatusKind::Badge => {
                    let value = store
                        .latest_string(key)
                        .map(str::to_string)
                        .or_else(|| store.latest(key).map(format_scalar_status));
                    let txt = value.clone().unwrap_or_else(|| "—".to_string());
                    let bg = cell
                        .color_map
                        .get(&txt)
                        .and_then(|c| parse_color(c))
                        .unwrap_or(default_color);
                    paint_status_chip(ui, body_rect, bg, &txt, matches!(kind, StatusKind::Badge));
                    value.is_some()
                }
                StatusKind::FixType => {
                    let v = store.latest(key);
                    let n = v.map(|f| f.round() as i64).unwrap_or(-1);
                    let (label, bg) = fix_type_chip(n);
                    let bg = if let Some(mapped) = cell.color_map.get(&n.to_string()).and_then(|c| parse_color(c)) {
                        mapped
                    } else if n < 0 {
                        default_color
                    } else {
                        bg
                    };
                    paint_status_chip(ui, body_rect, bg, label, false);
                    v.is_some()
                }
                StatusKind::ArmedBool => {
                    // Bool may arrive as a scalar (0.0/1.0) or as a string ("True"/"False").
                    let armed = if let Some(s) = store.latest_string(key) {
                        matches!(s, "True" | "true" | "ARMED" | "1")
                    } else if let Some(v) = store.latest(key) {
                        v.abs() > 0.5
                    } else {
                        false
                    };
                    let has_data = store.latest_string(key).is_some() || store.latest(key).is_some();
                    let (label, bg) = if !has_data {
                        ("—", default_color)
                    } else if armed {
                        ("ARMED", Color32::from_rgb(0x2c, 0xa0, 0x2c))
                    } else {
                        ("DISARMED", Color32::from_gray(120))
                    };
                    // Honour color_map override on "True" / "False" if user supplied.
                    let key_lookup = if armed { "True" } else { "False" };
                    let bg = cell
                        .color_map
                        .get(key_lookup)
                        .and_then(|c| parse_color(c))
                        .unwrap_or(bg);
                    paint_status_chip(ui, body_rect, bg, label, false);
                    has_data
                }
                StatusKind::TextLog => {
                    let entries = store.text_log_owned(key);
                    paint_text_log(ui, body_rect, &entries, default_color);
                    !entries.is_empty()
                }
                StatusKind::EkfFlags => {
                    let v = store.latest(key);
                    let flags = v
                        .filter(|f| f.is_finite() && *f >= 0.0)
                        .map(|f| f as u32)
                        .unwrap_or(0);
                    paint_ekf_flags(ui, body_rect, flags);
                    v.is_some()
                }
            }
        });

    (resp.inner, resp.response)
}

/// v0.12.0 — paint a single status chip (filled rounded rect with centered
/// text). `badge` shrinks the inner padding for a tighter pill look.
fn paint_status_chip(ui: &egui::Ui, rect: Rect, bg: Color32, text: &str, badge: bool) {
    let painter = ui.painter_at(rect);
    let pad = if badge { 4.0 } else { 8.0 };
    let chip = rect.shrink(pad);
    painter.rect_filled(chip, 4.0, bg);
    let font_size = if badge {
        (chip.height() * 0.55).clamp(11.0, 16.0)
    } else {
        (chip.height() * 0.45).clamp(13.0, 22.0)
    };
    let text_color = if bg.r() as u32 + bg.g() as u32 + bg.b() as u32 > 380 {
        // Light backgrounds — paint dark text.
        Color32::from_gray(20)
    } else {
        Color32::WHITE
    };
    painter.text(
        chip.center(),
        Align2::CENTER_CENTER,
        text,
        egui::FontId::proportional(font_size),
        text_color,
    );
}

/// v0.12.0 — paint a rolling text-log inside `rect`. Newest entry first,
/// severity-colored. Empty buffer → centered "no entries" placeholder.
fn paint_text_log(ui: &egui::Ui, rect: Rect, entries: &[crate::TextLogEntry], default_color: Color32) {
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect.shrink(2.0), 4.0, Color32::from_black_alpha(40));
    if entries.is_empty() {
        painter.text(
            rect.center(),
            Align2::CENTER_CENTER,
            "— no entries —",
            egui::FontId::proportional(12.0),
            default_color,
        );
        return;
    }
    let line_h = 14.0_f32;
    let mut y = rect.min.y + 6.0;
    // Newest first.
    for entry in entries.iter().rev() {
        if y + line_h > rect.max.y {
            break;
        }
        let color = severity_color(entry.severity);
        // Truncate text to fit the cell width (rough: 6 px per char).
        let max_chars = (rect.width() / 7.0) as usize;
        let txt: String = if entry.text.chars().count() > max_chars && max_chars > 3 {
            let mut s: String = entry.text.chars().take(max_chars.saturating_sub(1)).collect();
            s.push('…');
            s
        } else {
            entry.text.clone()
        };
        painter.text(
            Pos2::new(rect.min.x + 8.0, y),
            Align2::LEFT_TOP,
            txt,
            egui::FontId::proportional(11.0),
            color,
        );
        y += line_h;
    }
}

/// v0.14.0 — ArduPilot `EKF_STATUS_REPORT.flags` bitfield labels, ordered by
/// bit index (bit 0 first).
///
/// Matches `mavlink/include/common/common.h` `EKF_*` definitions.
pub const EKF_FLAG_LABELS: [&str; 12] = [
    "ATTITUDE",
    "VELOCITY_HORIZ",
    "VELOCITY_VERT",
    "POS_HORIZ_REL",
    "POS_HORIZ_ABS",
    "POS_VERT_ABS",
    "POS_VERT_AGL",
    "CONST_POS_MODE",
    "PRED_POS_HORIZ_REL",
    "PRED_POS_HORIZ_ABS",
    "GPS_GLITCHING",
    "GPS_QUALITY_GOOD",
];

/// v0.14.0 — decode an `ekf_flags` bitfield into a list of
/// `(label, is_set)` pairs in bit-index order. A negative / non-finite raw
/// value should be passed in as `0` so every bit reads as unset.
///
/// Exposed for unit testing — `paint_ekf_flags` uses the same logic.
#[doc(hidden)]
pub fn decode_ekf_flags(flags: u32) -> Vec<(&'static str, bool)> {
    EKF_FLAG_LABELS
        .iter()
        .enumerate()
        .map(|(i, label)| (*label, (flags & (1u32 << i)) != 0))
        .collect()
}

/// v0.14.0 — paint the EKF-flags multi-row chip: a grid of `● label` rows,
/// green dot for set bits, gray dot for unset bits.
fn paint_ekf_flags(ui: &egui::Ui, rect: Rect, flags: u32) {
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect.shrink(2.0), 4.0, Color32::from_black_alpha(40));
    let rows = decode_ekf_flags(flags);
    // Distribute rows vertically. Cap font size so 12 rows fit comfortably.
    let row_h = ((rect.height() - 8.0) / rows.len() as f32).clamp(10.0, 18.0);
    let font_size = (row_h * 0.7).clamp(9.0, 13.0);
    let set_color = Color32::from_rgb(0x2c, 0xa0, 0x2c);
    let unset_color = Color32::from_gray(90);
    let label_color = Color32::from_gray(220);
    let dot_r = (row_h * 0.25).clamp(2.5, 5.0);
    let mut y = rect.min.y + 6.0;
    for (label, is_set) in rows.iter() {
        if y + row_h > rect.max.y {
            break;
        }
        let dot_center = Pos2::new(rect.min.x + 10.0, y + row_h * 0.5);
        let dot_color = if *is_set { set_color } else { unset_color };
        painter.circle_filled(dot_center, dot_r, dot_color);
        painter.text(
            Pos2::new(rect.min.x + 18.0, y + row_h * 0.5),
            Align2::LEFT_CENTER,
            *label,
            egui::FontId::proportional(font_size),
            label_color,
        );
        y += row_h;
    }
}

/// v0.12.0 — MAVLink statustext severity → display color. Matches the
/// emergency/alert/critical/error → red, warning → yellow, notice → blue,
/// info/debug → gray convention.
fn severity_color(sev: u8) -> Color32 {
    match sev {
        0..=3 => Color32::from_rgb(0xd6, 0x27, 0x28), // red
        4 => Color32::from_rgb(0xe6, 0xa9, 0x00),     // yellow / amber
        5 => Color32::from_rgb(0x1f, 0x77, 0xb4),     // blue
        _ => Color32::from_gray(170),                  // gray (info / debug)
    }
}

/// v0.12.0 — GPS fix-type → (label, color) for [`StatusKind::FixType`].
fn fix_type_chip(n: i64) -> (&'static str, Color32) {
    match n {
        0 => ("No fix", Color32::from_rgb(0xd6, 0x27, 0x28)),
        1 => ("2D", Color32::from_rgb(0xe6, 0xa9, 0x00)),
        2 => ("3D", Color32::from_rgb(0x2c, 0xa0, 0x2c)),
        3 => ("DGPS", Color32::from_rgb(0x1f, 0x77, 0xb4)),
        4 => ("RTK float", Color32::from_rgb(0x94, 0x67, 0xbd)),
        5 => ("RTK fixed", Color32::from_rgb(0x2c, 0xa0, 0x2c)),
        6 => ("RTK fixed", Color32::from_rgb(0x2c, 0xa0, 0x2c)),
        _ => ("—", Color32::from_gray(170)),
    }
}

/// v0.12.0 — render a scalar numeric value as the chip text when no string
/// form is available (e.g. flight_mode came through as a numeric mode id).
fn format_scalar_status(v: f64) -> String {
    if v.fract().abs() < 1e-9 {
        format!("{}", v as i64)
    } else {
        format!("{v:.2}")
    }
}

/// v0.12.0 — public test helper: resolve the background color a status cell
/// would paint, given the cell config + a fresh store snapshot. Pure
/// function — no egui context required.
#[doc(hidden)]
pub fn status_cell_color(cell: &Cell, store: &TraceStore) -> Color32 {
    use profiler_template::StatusKind;
    let kind = cell.kind.unwrap_or(StatusKind::Text);
    let default_color = cell
        .default_color
        .as_deref()
        .and_then(parse_color)
        .unwrap_or(Color32::from_gray(170));
    let key = cell.source.as_str();
    match kind {
        StatusKind::Text | StatusKind::Badge => {
            let value = store
                .latest_string(key)
                .map(str::to_string)
                .or_else(|| store.latest(key).map(format_scalar_status));
            match value {
                Some(v) => cell
                    .color_map
                    .get(&v)
                    .and_then(|c| parse_color(c))
                    .unwrap_or(default_color),
                None => default_color,
            }
        }
        StatusKind::FixType => {
            let v = store.latest(key);
            let n = v.map(|f| f.round() as i64).unwrap_or(-1);
            if let Some(c) = cell.color_map.get(&n.to_string()).and_then(|c| parse_color(c)) {
                c
            } else if n < 0 {
                default_color
            } else {
                fix_type_chip(n).1
            }
        }
        StatusKind::ArmedBool => {
            let armed = if let Some(s) = store.latest_string(key) {
                matches!(s, "True" | "true" | "ARMED" | "1")
            } else if let Some(v) = store.latest(key) {
                v.abs() > 0.5
            } else {
                return default_color;
            };
            let key_lookup = if armed { "True" } else { "False" };
            if let Some(c) = cell.color_map.get(key_lookup).and_then(|c| parse_color(c)) {
                return c;
            }
            if armed {
                Color32::from_rgb(0x2c, 0xa0, 0x2c)
            } else {
                Color32::from_gray(120)
            }
        }
        StatusKind::TextLog => default_color,
        // v0.14.0 — EkfFlags renders a multi-row grid (no single dominant
        // color); for the test-helper we return the default color so callers
        // that probe `status_cell_color` get a sensible answer.
        StatusKind::EkfFlags => default_color,
    }
}

/// v0.12.0 — public test helper: severity color used by [`StatusKind::TextLog`].
#[doc(hidden)]
pub fn status_severity_color(sev: u8) -> Color32 {
    severity_color(sev)
}

/// v0.12.0 — public test helper: GPS fix-type chip label + color.
#[doc(hidden)]
pub fn status_fix_type_chip(n: i64) -> (&'static str, Color32) {
    fix_type_chip(n)
}

/// v0.16.1 — primitives that participate in the egui_plot interactivity
/// suite (drag / zoom / box-zoom / double-click reset). Used to gate the
/// "Reset zoom" and "Label" entries in the per-cell context menu — those
/// items make no sense for non-plot primitives like Status / InfoText.
///
/// Exposed (doc-hidden) for the v0.16.1 context-menu tests.
#[doc(hidden)]
pub fn primitive_supports_zoom(p: Primitive) -> bool {
    matches!(
        p,
        Primitive::Scalar
            | Primitive::Vector
            | Primitive::Overlay
            | Primitive::Magnitude
            | Primitive::MagInterference
            | Primitive::Diff
            | Primitive::AttitudeRpy
    )
}

/// v0.16.1 — primitives that respond to the `Label` overlay (data / metadata).
/// InfoText is itself literal content; Status renders its own chip — neither
/// participates in the label-overlay system, so the menu hides that submenu
/// for those primitives.
///
/// Exposed (doc-hidden) for the v0.16.1 context-menu tests.
#[doc(hidden)]
pub fn primitive_supports_label_mode(p: Primitive) -> bool {
    // Same set as zoom for now — both submenus are 2D-plot-only.
    primitive_supports_zoom(p)
}

/// v0.10.0 — build the per-cell right-click context menu on `resp`. v0.16.1
/// gates "Reset zoom" and the "Label" submenu on plot-typed primitives so
/// Status / InfoText cells get a sensible menu (Edit / Hide / Delete only).
/// Drains user clicks into `sink` for the CLI to apply next frame.
fn attach_context_menu(resp: &egui::Response, cell: &Cell, sink: &mut Vec<CellMenuAction>) {
    let row = cell.row;
    let col = cell.col;
    let supports_zoom = primitive_supports_zoom(cell.primitive);
    let supports_label_mode = primitive_supports_label_mode(cell.primitive);
    resp.clone().context_menu(|ui| {
        if ui.button("Edit panel...").clicked() {
            sink.push(CellMenuAction::Edit { row, col });
            ui.close();
        }
        if ui.button("Hide panel").clicked() {
            sink.push(CellMenuAction::HideToggle { row, col });
            ui.close();
        }
        if supports_zoom && ui.button("Reset zoom").clicked() {
            sink.push(CellMenuAction::ResetZoom { row, col });
            ui.close();
        }
        if supports_label_mode {
            ui.separator();
            ui.label("Label:");
            if ui.button("off").clicked() {
                sink.push(CellMenuAction::SetLabelMode { row, col, mode: LabelMode::Off });
                ui.close();
            }
            if ui.button("data").clicked() {
                sink.push(CellMenuAction::SetLabelMode { row, col, mode: LabelMode::Data });
                ui.close();
            }
            if ui.button("metadata").clicked() {
                sink.push(CellMenuAction::SetLabelMode { row, col, mode: LabelMode::Metadata });
                ui.close();
            }
        }
        ui.separator();
        if ui.button("Delete panel").clicked() {
            sink.push(CellMenuAction::Delete { row, col });
            ui.close();
        }
    });
}

/// v0.16.1 — wrap an arbitrary cell-render closure with a transparent
/// right-click interactive overlay so the per-cell context menu reaches
/// EVERY primitive — including [`Primitive::Status`] (v0.12.0) and
/// [`Primitive::InfoText`] (v0.14.0) which draw their own `Frame`-based
/// content via `painter` calls instead of the `egui_plot` widget tree.
///
/// Pre-v0.16.1 only the plot path attached `context_menu` to `plot_resp`;
/// the Status/InfoText scope-builder responses technically sensed clicks but
/// their inner UIs allocated no widgets so the response never registered
/// secondary clicks. Allocating a dedicated `Sense::click()` interaction
/// over the full `cell_rect` AFTER the inner render guarantees right-click
/// is captured regardless of what the inner code drew. The overlay only
/// uses `Sense::click()`, which does NOT swallow primary clicks aimed at
/// the inner content (status chips, info-text spans — none of which are
/// click-interactive today, but the contract is preserved for future work).
///
/// `cell_rect` is the rect the inner render drew into; we use it (plus a
/// stable id derived from `(row, col)`) to allocate the overlay interaction.
fn wrap_cell_with_context_menu<R>(
    ui: &mut egui::Ui,
    cell: &Cell,
    cell_rect: Rect,
    sink: &mut Vec<CellMenuAction>,
    render_inner: impl FnOnce(&mut egui::Ui) -> R,
) -> R {
    let result = render_inner(ui);
    // Lay a transparent click-sensitive interaction over the entire cell
    // rect so right-click reaches us even when the inner render only did
    // painter calls. `Sense::click()` covers BOTH primary and secondary
    // clicks but `context_menu` only fires on secondary, so primary clicks
    // remain available to any future inner buttons.
    let overlay = ui.interact(
        cell_rect,
        ui.id().with(("hvn-profiler-cell-ctx", cell.row, cell.col)),
        egui::Sense::click(),
    );
    attach_context_menu(&overlay, cell, sink);
    result
}

/// Dispatch on the cell's primitive and draw the appropriate lines.
/// Returns `true` if any line produced ≥1 point.
fn draw_primitive(plot_ui: &mut egui_plot::PlotUi, cell: &Cell, store: &TraceStore) -> bool {
    match cell.primitive {
        Primitive::Scalar | Primitive::Overlay => {
            // Both render every source as its own line; scalar usually has 1,
            // overlay has many. fallback + transform + scale honoured.
            let mut any = false;
            for (i, src) in cell.sources.iter().enumerate() {
                any |= draw_scalar_source(plot_ui, src, store, i);
            }
            any
        }
        Primitive::Vector => draw_vector(plot_ui, cell, store, false),
        Primitive::AttitudeRpy => draw_attitude_rpy(plot_ui, cell, store),
        Primitive::Magnitude => draw_magnitude(plot_ui, cell, store),
        Primitive::MagInterference => draw_vector(plot_ui, cell, store, true),
        Primitive::Diff => draw_diff(plot_ui, cell, store),
        Primitive::StatusBadge => false, // reserved — render nothing
        // Status is handled outside the plot context — never reached here.
        Primitive::Status => false,
        // v0.14.0 — InfoText is handled outside the plot context too; never
        // reached here. Also doesn't count toward "any_data" since the panel
        // is static literal content.
        Primitive::InfoText => false,
    }
}

/// Resolve a source's points: use `key`, else `fallback` if `key` is empty.
/// Applies `transform` and `scale`.
fn resolve_points(src: &CellSource, store: &TraceStore) -> Vec<[f64; 2]> {
    let mut pts = store.points(&src.key);
    if pts.is_empty() {
        if let Some(fb) = &src.fallback {
            pts = store.points(fb);
        }
    }
    apply_transform(&mut pts, src);
    pts
}

/// Apply `transform` (e.g. `rad_to_deg`) and `scale` in place.
fn apply_transform(pts: &mut [[f64; 2]], src: &CellSource) {
    let scale = src.scale.unwrap_or(1.0);
    let rad_to_deg = matches!(src.transform.as_deref(), Some("rad_to_deg"));
    if scale == 1.0 && !rad_to_deg {
        return;
    }
    let factor = if rad_to_deg {
        scale * 180.0 / std::f64::consts::PI
    } else {
        scale
    };
    for p in pts.iter_mut() {
        p[1] *= factor;
    }
}

fn draw_scalar_source(
    plot_ui: &mut egui_plot::PlotUi,
    src: &CellSource,
    store: &TraceStore,
    idx: usize,
) -> bool {
    let pts = resolve_points(src, store);
    if pts.is_empty() {
        return false;
    }
    let label = if src.label.is_empty() {
        src.key.clone()
    } else {
        src.label.clone()
    };
    let color = color_for(&src.color, idx);
    plot_ui.line(Line::new(label, PlotPoints::from(pts)).color(color));
    true
}

/// Draw the 3 components `base[0..2]` of the first source. When `with_mag`,
/// also draw the L2-norm line (for `mag_interference`).
fn draw_vector(
    plot_ui: &mut egui_plot::PlotUi,
    cell: &Cell,
    store: &TraceStore,
    with_mag: bool,
) -> bool {
    let Some(src) = cell.sources.first() else {
        return false;
    };
    let comps = component_points(src, store);
    let mut any = false;
    let axis_labels = ["x", "y", "z"];
    for (i, pts) in comps.iter().enumerate() {
        if pts.is_empty() {
            continue;
        }
        any = true;
        let label = format!("{}.{}", short_key(&src.key), axis_labels[i]);
        plot_ui.line(Line::new(label, PlotPoints::from(pts.clone())).color(color_for("", i)));
    }
    if with_mag {
        let mag = magnitude_points(&comps);
        if !mag.is_empty() {
            any = true;
            plot_ui.line(
                Line::new("|.|", PlotPoints::from(mag))
                    .color(Color32::from_gray(30))
                    .width(1.2),
            );
        }
    }
    any
}

/// `attitude_rpy`: 3 component lines, each converted to degrees.
fn draw_attitude_rpy(plot_ui: &mut egui_plot::PlotUi, cell: &Cell, store: &TraceStore) -> bool {
    let Some(src) = cell.sources.first() else {
        return false;
    };
    let mut comps = component_points(src, store);
    // Convert rad → deg for each component.
    for pts in comps.iter_mut() {
        for p in pts.iter_mut() {
            p[1] = p[1] * 180.0 / std::f64::consts::PI;
        }
    }
    let labels = ["roll", "pitch", "yaw"];
    let mut any = false;
    for (i, pts) in comps.iter().enumerate() {
        if pts.is_empty() {
            continue;
        }
        any = true;
        plot_ui.line(
            Line::new(labels[i], PlotPoints::from(pts.clone())).color(color_for("", i)),
        );
    }
    any
}

/// `magnitude`: single line = L2 norm of the vector source's components.
fn draw_magnitude(plot_ui: &mut egui_plot::PlotUi, cell: &Cell, store: &TraceStore) -> bool {
    let Some(src) = cell.sources.first() else {
        return false;
    };
    let comps = component_points(src, store);
    let mag = magnitude_points(&comps);
    if mag.is_empty() {
        return false;
    }
    let label = if src.label.is_empty() {
        format!("|{}|", short_key(&src.key))
    } else {
        src.label.clone()
    };
    plot_ui.line(Line::new(label, PlotPoints::from(mag)).color(color_for(&src.color, 0)));
    true
}

/// `diff`: one line = `key - minus`, index-aligned in the ring buffers.
fn draw_diff(plot_ui: &mut egui_plot::PlotUi, cell: &Cell, store: &TraceStore) -> bool {
    let Some(src) = cell.sources.first() else {
        return false;
    };
    let Some(minus_key) = &src.minus else {
        // No subtrahend → fall back to plotting the key directly.
        return draw_scalar_source(plot_ui, src, store, 0);
    };
    let a = store.points(&src.key);
    let b = store.points(minus_key);
    if a.is_empty() || b.is_empty() {
        return false;
    }
    // Index-align the two buffers (simplest robust approach per spec). Use the
    // shorter length; take timestamps from `a`.
    let n = a.len().min(b.len());
    let diff: Vec<[f64; 2]> = (0..n).map(|i| [a[i][0], a[i][1] - b[i][1]]).collect();
    if diff.is_empty() {
        return false;
    }
    let label = if src.label.is_empty() {
        format!("{} − {}", short_key(&src.key), short_key(minus_key))
    } else {
        src.label.clone()
    };
    let color = cell
        .color
        .as_deref()
        .map(|c| parse_color(c).unwrap_or_else(|| color_for("", 0)))
        .unwrap_or_else(|| color_for(&src.color, 0));
    plot_ui.line(Line::new(label, PlotPoints::from(diff)).color(color));
    true
}

/// Extract the 3 component point-series `base[0]`, `base[1]`, `base[2]` for a
/// vector source. Empty inner vecs for components with no data.
fn component_points(src: &CellSource, store: &TraceStore) -> [Vec<[f64; 2]>; 3] {
    let base = &src.key;
    let fb = src.fallback.as_deref();
    let mut out: [Vec<[f64; 2]>; 3] = Default::default();
    let scale = src.scale.unwrap_or(1.0);
    for (i, slot) in out.iter_mut().enumerate() {
        let mut pts = store.points(&format!("{base}[{i}]"));
        if pts.is_empty() {
            if let Some(fb) = fb {
                pts = store.points(&format!("{fb}[{i}]"));
            }
        }
        if scale != 1.0 {
            for p in pts.iter_mut() {
                p[1] *= scale;
            }
        }
        *slot = pts;
    }
    out
}

/// L2 norm per index across the 3 component series (index-aligned).
fn magnitude_points(comps: &[Vec<[f64; 2]>; 3]) -> Vec<[f64; 2]> {
    let n = comps.iter().map(|c| c.len()).min().unwrap_or(0);
    (0..n)
        .map(|i| {
            let x = comps[0][i][1];
            let y = comps[1][i][1];
            let z = comps[2][i][1];
            [comps[0][i][0], (x * x + y * y + z * z).sqrt()]
        })
        .collect()
}

/// Drop a trailing `[i]` and keep the base name for compact labels.
fn short_key(k: &str) -> &str {
    k.split('[').next().unwrap_or(k)
}

// ─── Label overlay ───────────────────────────────────────────────────────────

/// Padding (in screen pixels) between the plot rect's edges and the overlay box.
const OVERLAY_PAD: f32 = 6.0;
/// Inner padding between the overlay's background rect and its text.
const OVERLAY_TEXT_INSET: Vec2 = Vec2::new(6.0, 3.0);

/// Build the text that should appear in the overlay for `cell`, or `None`
/// when the resolved label mode is `Off` (or there's nothing to display).
///
/// Returned tuple is `(text, text_color)` so callers can paint with the same
/// shading the v0.5.0–v0.8.0 in-plot overlay used (lighter for data, dimmer
/// for metadata).
///
/// Exposed for testing.
#[doc(hidden)]
pub fn build_label_text(
    cell: &Cell,
    store: &TraceStore,
    label_override: LabelOverride,
) -> Option<(String, Color32)> {
    let mode = label_override.resolve(cell.label_mode);
    match mode {
        LabelMode::Off => None,
        LabelMode::Data => {
            let src = cell.sources.first()?;
            // Latest value of the primary source (honour fallback).
            let v = store
                .latest(&src.key)
                .or_else(|| src.fallback.as_deref().and_then(|f| store.latest(f)))?;
            let fmt = cell
                .label_data
                .as_ref()
                .map(|d| d.format.as_str())
                .unwrap_or("");
            let mut text = format_value(fmt, v);
            if cell.label_data.as_ref().map(|d| d.show_min_max).unwrap_or(false) {
                let key_for_minmax = if store.len(&src.key) > 0 {
                    src.key.clone()
                } else {
                    src.fallback.clone().unwrap_or_else(|| src.key.clone())
                };
                if let Some((lo, hi)) = store.min_max(&key_for_minmax) {
                    text = format!("{text}  [{lo:.2}, {hi:.2}]");
                }
            }
            Some((text, Color32::from_gray(220)))
        }
        LabelMode::Metadata => {
            // Build the metadata block. If the template provided a
            // `label_metadata` struct, use it; otherwise (global override
            // forcing metadata on cells without one) fall back to the
            // primary source key.
            let mut parts: Vec<String> = Vec::new();
            if let Some(md) = &cell.label_metadata {
                if !md.source_path.is_empty() {
                    parts.push(md.source_path.clone());
                }
                if !md.units.is_empty() {
                    parts.push(md.units.clone());
                }
                if let Some(hz) = md.stream_rate_hz {
                    // Trim trailing zero/`.0` for whole-number rates so we get
                    // `4 Hz` and `12.5 Hz`, not `4.00 Hz`.
                    let txt = if (hz - hz.round()).abs() < 1e-9 {
                        format!("{} Hz", hz as i64)
                    } else {
                        format!("{hz} Hz")
                    };
                    parts.push(txt);
                }
            } else if let Some(src) = cell.sources.first() {
                parts.push(src.key.clone());
            }
            if parts.is_empty() {
                None
            } else {
                Some((parts.join("\n"), Color32::from_gray(220)))
            }
        }
    }
}

/// Paint the per-panel `label_mode` overlay onto `plot_rect`'s top-left corner
/// using the parent ui's painter (so it never participates in layout).
///
/// `plot_rect` is the screen-space rect returned by `Plot::show()` —
/// `PlotResponse::response.rect`. The overlay is drawn as a translucent black
/// rounded rectangle with the label text painted on top in [`TextStyle::Small`].
///
/// v0.10.1 — anchor moved from top-right to top-left to match the legend
/// position. On a rolling-window plot the live trace tip is pinned to the
/// RIGHT edge, so right-anchored overlays were continuously occluding fresh
/// data. The left edge holds the oldest samples, which are far more
/// tolerant of an overlay rectangle.
fn draw_label_overlay(
    ui: &mut egui::Ui,
    plot_rect: Rect,
    cell: &Cell,
    store: &TraceStore,
    label_override: LabelOverride,
) {
    let Some((text, text_color)) = build_label_text(cell, store, label_override) else {
        return;
    };

    // Measure the text in screen pixels so we can size the background box.
    let font = TextStyle::Small.resolve(ui.style());
    let galley = ui.painter().layout_no_wrap(text.clone(), font.clone(), text_color);
    let text_size = galley.size();

    // Top-left corner of the plot rect, inset by OVERLAY_PAD on both axes.
    let bg_size = text_size + 2.0 * OVERLAY_TEXT_INSET;
    let bg_min = compute_overlay_pos(plot_rect, text_size);
    let bg_rect = Rect::from_min_size(bg_min, bg_size);

    // Clip to the plot rect so an oversized label can't bleed into adjacent
    // cells if the plot ever gets very small.
    let painter = ui.painter_at(plot_rect);

    // Semi-transparent fill keeps the text readable over busy data; a faint
    // 1 px border helps it pop against light grid lines.
    painter.rect_filled(bg_rect, 3.0, Color32::from_black_alpha(160));

    // Paint the galley at the inset position inside the background box.
    painter.galley(bg_min + OVERLAY_TEXT_INSET, galley, text_color);
}

/// Pure helper: top-left anchor of the label overlay's background rect for a
/// given `plot_rect` and measured `text_size`. Exposed so layout tests can
/// pin the v0.10.1 top-left anchor without spinning up a real egui context.
///
/// `text_size` is unused for the anchor itself but kept in the signature to
/// match `draw_label_overlay`'s callsite and make the intent explicit (the
/// box grows from the anchor to the right + down).
pub fn compute_overlay_pos(plot_rect: Rect, _text_size: Vec2) -> egui::Pos2 {
    egui::pos2(
        plot_rect.left() + OVERLAY_PAD,
        plot_rect.top() + OVERLAY_PAD,
    )
}

/// Pure helper for layout tests: how much screen space the label overlay would
/// claim for `text`. Returns `egui::Vec2::ZERO` for empty text.
///
/// This is here as a sanity check that the overlay's footprint is independent
/// of the plot rect — i.e. that toggling labels can't reshape the plot.
#[doc(hidden)]
pub fn overlay_box_size(ui: &egui::Ui, text: &str) -> Vec2 {
    if text.is_empty() {
        return Vec2::ZERO;
    }
    let font = TextStyle::Small.resolve(ui.style());
    let galley = ui.painter().layout_no_wrap(
        text.to_string(),
        font,
        Color32::WHITE,
    );
    galley.size() + 2.0 * OVERLAY_TEXT_INSET
}


/// Best-effort translation of a Python-style `{:...}` format spec into a
/// Rust-rendered string, plus any literal suffix text outside the braces.
///
/// Exposed for integration testing (see `tests/format_spec_test.rs`).
#[doc(hidden)]
pub fn format_value_pub(spec: &str, v: f64) -> String {
    format_value(spec, v)
}

#[doc(hidden)]
///
/// Supported mini-language (enough for the SITL templates):
/// - sign flag `+`            → forces leading sign
/// - optional width `N`       → minimum integer width (zero-padding ignored)
/// - precision `.N`           → decimal places (default 2)
/// - type `f`/`e`/`g`         → fixed / scientific (lowercase) / default
/// - literal text outside the braces (e.g. `{:.1f}°`, `{:.2f} m/s`) is
///   concatenated verbatim after the numeric body.
///
/// Examples (handled):
/// - `"{:.1f}"`        → `"1.2"`
/// - `"{:+.2f}°"`     → `"+1.23°"`
/// - `"{:.3e} m"`     → `"1.234e0 m"`
/// - `""` or unparseable → `"{v:.2}"` Rust default.
fn format_value(spec: &str, v: f64) -> String {
    if spec.is_empty() {
        return format!("{v:.2}");
    }
    // Split into: prefix-literal { inner } suffix-literal
    let (prefix, rest) = match spec.find('{') {
        Some(i) => (&spec[..i], &spec[i + 1..]),
        None => return spec.to_string(),
    };
    let (inner, suffix) = match rest.find('}') {
        Some(j) => (&rest[..j], &rest[j + 1..]),
        None => return format!("{prefix}{rest}"),
    };
    // inner is `:[flags][width][.prec][type]` — drop the leading colon if any.
    let body = inner.trim_start_matches(':');
    let plus = body.starts_with('+');
    let body = body.trim_start_matches('+');
    // Split body into "width_part" and "prec+type_part" at the dot.
    let (_width_part, prec_type_part) = match body.split_once('.') {
        Some((w, p)) => (w, p),
        None => (body, ""),
    };
    // Detect type suffix (last char of prec_type_part if it is `f`/`e`/`g`).
    let (prec_digits, ty) = match prec_type_part.chars().last() {
        Some(c) if c == 'f' || c == 'e' || c == 'g' => (&prec_type_part[..prec_type_part.len() - c.len_utf8()], c),
        _ => (prec_type_part, 'f'),
    };
    let prec: usize = prec_digits.parse().unwrap_or(2);
    let num = match (ty, plus) {
        ('e', true) => format!("{v:+.prec$e}"),
        ('e', false) => format!("{v:.prec$e}"),
        ('g', true) => format!("{v:+.prec$}"),
        ('g', false) => format!("{v:.prec$}"),
        (_, true) => format!("{v:+.prec$}"),
        (_, false) => format!("{v:.prec$}"),
    };
    format!("{prefix}{num}{suffix}")
}

// ─── Colors ──────────────────────────────────────────────────────────────────

/// matplotlib tab10 palette, used for `C0..C9`.
const TAB10: [Color32; 10] = [
    Color32::from_rgb(0x1f, 0x77, 0xb4), // C0 blue
    Color32::from_rgb(0xff, 0x7f, 0x0e), // C1 orange
    Color32::from_rgb(0x2c, 0xa0, 0x2c), // C2 green
    Color32::from_rgb(0xd6, 0x27, 0x28), // C3 red
    Color32::from_rgb(0x94, 0x67, 0xbd), // C4 purple
    Color32::from_rgb(0x8c, 0x56, 0x4b), // C5 brown
    Color32::from_rgb(0xe3, 0x77, 0xc2), // C6 pink
    Color32::from_rgb(0x7f, 0x7f, 0x7f), // C7 gray
    Color32::from_rgb(0xbc, 0xbd, 0x22), // C8 olive
    Color32::from_rgb(0x17, 0xbe, 0xcf), // C9 cyan
];

/// Resolve a template color string, falling back to the tab10 palette indexed
/// by `idx` when the string is empty or unrecognised.
fn color_for(spec: &str, idx: usize) -> Color32 {
    parse_color(spec).unwrap_or_else(|| TAB10[idx % TAB10.len()])
}

/// Parse a matplotlib-ish color string: `C0..C9`, `#rrggbb`, or a few named
/// single-char colors (`k`,`m`,`r`,`g`,`b`,`c`,`y`,`w`). `None` if unknown.
fn parse_color(spec: &str) -> Option<Color32> {
    let s = spec.trim();
    if s.is_empty() {
        return None;
    }
    // matplotlib cycle color CN.
    if let Some(n) = s.strip_prefix('C').and_then(|r| r.parse::<usize>().ok()) {
        return Some(TAB10[n % TAB10.len()]);
    }
    // Hex #rrggbb.
    if let Some(hex) = s.strip_prefix('#') {
        if hex.len() == 6 {
            let r = u8::from_str_radix(&hex[0..2], 16).ok()?;
            let g = u8::from_str_radix(&hex[2..4], 16).ok()?;
            let b = u8::from_str_radix(&hex[4..6], 16).ok()?;
            return Some(Color32::from_rgb(r, g, b));
        }
    }
    // Single-char named colors.
    Some(match s {
        "k" => Color32::from_gray(20),
        "w" => Color32::WHITE,
        "r" => Color32::from_rgb(0xd6, 0x27, 0x28),
        "g" => Color32::from_rgb(0x2c, 0xa0, 0x2c),
        "b" => Color32::from_rgb(0x1f, 0x77, 0xb4),
        "c" => Color32::from_rgb(0x17, 0xbe, 0xcf),
        "m" => Color32::from_rgb(0xe3, 0x77, 0xc2),
        "y" => Color32::from_rgb(0xbc, 0xbd, 0x22),
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_hex_color() {
        assert_eq!(parse_color("#d62728"), Some(Color32::from_rgb(0xd6, 0x27, 0x28)));
        assert_eq!(parse_color("#1f77b4"), Some(Color32::from_rgb(0x1f, 0x77, 0xb4)));
    }

    #[test]
    fn parse_cycle_color() {
        assert_eq!(parse_color("C0"), Some(TAB10[0]));
        assert_eq!(parse_color("C3"), Some(TAB10[3]));
        // wraps mod 10
        assert_eq!(parse_color("C12"), Some(TAB10[2]));
    }

    #[test]
    fn parse_named_and_unknown() {
        assert_eq!(parse_color("k"), Some(Color32::from_gray(20)));
        assert_eq!(parse_color(""), None);
        assert_eq!(parse_color("chartreuse"), None);
    }

    #[test]
    fn color_for_falls_back_to_palette() {
        assert_eq!(color_for("", 1), TAB10[1]);
        assert_eq!(color_for("bogus", 5), TAB10[5]);
        assert_eq!(color_for("#000000", 0), Color32::from_rgb(0, 0, 0));
    }

    #[test]
    fn format_value_python_specs() {
        assert_eq!(format_value("{:+.2f}", 1.23456), "+1.23");
        assert_eq!(format_value("{:.1f}", 1.23456), "1.2");
        assert_eq!(format_value("{:+.2f}", -2.5), "-2.50");
        // default
        assert_eq!(format_value("", 1.23456), "1.23");
    }

    #[test]
    fn rad_to_deg_transform() {
        let mut pts = [[0.0, std::f64::consts::PI]];
        let src = CellSource {
            transform: Some("rad_to_deg".into()),
            ..Default::default()
        };
        apply_transform(&mut pts, &src);
        assert!((pts[0][1] - 180.0).abs() < 1e-9);
    }

    #[test]
    fn scale_transform() {
        let mut pts = [[0.0, 1.5]];
        let src = CellSource {
            scale: Some(1000.0),
            ..Default::default()
        };
        apply_transform(&mut pts, &src);
        assert!((pts[0][1] - 1500.0).abs() < 1e-9);
    }

    #[test]
    fn magnitude_is_l2_norm() {
        let comps = [
            vec![[0.0, 3.0]],
            vec![[0.0, 4.0]],
            vec![[0.0, 0.0]],
        ];
        let mag = magnitude_points(&comps);
        assert_eq!(mag.len(), 1);
        assert!((mag[0][1] - 5.0).abs() < 1e-9);
    }

    #[test]
    fn diff_render_against_store() {
        // Functional check of the diff math via a tiny store.
        let mut store = TraceStore::new(60.0);
        store.push(0.0, "a", 10.0);
        store.push(0.0, "b", 4.0);
        store.push(1.0, "a", 12.0);
        store.push(1.0, "b", 5.0);
        let a = store.points("a");
        let b = store.points("b");
        let n = a.len().min(b.len());
        let diff: Vec<[f64; 2]> = (0..n).map(|i| [a[i][0], a[i][1] - b[i][1]]).collect();
        assert_eq!(diff, vec![[0.0, 6.0], [1.0, 7.0]]);
    }
}
