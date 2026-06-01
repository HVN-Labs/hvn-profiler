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

use egui::{Align2, Color32, Rect, RichText, TextStyle, UiBuilder, Vec2};
use egui_plot::{HLine, Legend, Line, Plot, PlotPoints};

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
#[derive(Default)]
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
    mut opts: GridRenderOptions<'_>,
) -> GridStats {
    let rows = tpl.grid.rows.max(1);
    let cols = tpl.grid.cols.max(1);

    // Index cells by (row, col) for O(1) lookup during layout.
    let mut at: Vec<Option<&Cell>> = vec![None; rows * cols];
    for c in &tpl.cells {
        if c.row < rows && c.col < cols {
            at[c.row * cols + c.col] = Some(c);
        }
    }

    // Compute the absolute rect we get to draw inside. `available_rect_before_wrap`
    // returns true pixel coords on the parent ui, so child cells can be placed
    // by `scope_builder` with `max_rect`.
    let avail = ui.available_rect_before_wrap();
    let layout = compute_layout(avail, rows, cols, &tpl.sections);

    let mut stats = GridStats::default();

    // ── Section banners (drawn first so cell titles paint on top) ───────────
    for (sec, rect) in &layout.section_rects {
        draw_section_banner(ui, sec, *rect);
    }

    // ── Cells ───────────────────────────────────────────────────────────────
    let interactive = opts.panel_states.is_some();
    for r in 0..rows {
        for c in 0..cols {
            let id = r * cols + c;
            let rect = layout.cell_rect(r, c);
            // Runtime visibility override (v0.10.0 "Hide panel" path). When the
            // override is `Some(false)` we still draw nothing but the rect is
            // reserved — same behaviour as the template's `visible: false`.
            let runtime_visible = opts
                .visibility_override
                .as_ref()
                .and_then(|m| m.get(&(r, c)).copied())
                .unwrap_or(true);
            // Always claim the rect so the next row's `available_rect`
            // computation downstream of the grid is correct, even when a
            // slot is empty.
            ui.scope_builder(UiBuilder::new().max_rect(rect), |ui| {
                ui.set_clip_rect(rect);
                match at[id] {
                    Some(cell) if cell.visible && runtime_visible && !cell.sources.is_empty() => {
                        stats.panels += 1;
                        // Pull the lock state for this cell (None when the
                        // CLI didn't pass a panel_states map — non-interactive
                        // mode, identical to v0.9.0).
                        let mut local_locked = false;
                        let panel_locked = match &mut opts.panel_states {
                            Some(map) => {
                                let st = map.entry((r, c)).or_default();
                                &mut st.locked
                            }
                            None => &mut local_locked,
                        };
                        let (had_data, plot_resp) = render_cell(
                            ui,
                            cell,
                            store,
                            store.window_s,
                            label_override,
                            rect,
                            interactive,
                            panel_locked,
                        );
                        if had_data {
                            stats.panels_with_data += 1;
                        }
                        // Right-click context menu (v0.10.0). Only attached
                        // when the CLI supplied a menu sink.
                        if let Some(sink) = opts.menu_sink.as_deref_mut() {
                            attach_context_menu(&plot_resp, cell, sink);
                        }
                    }
                    _ => {} // empty slot — rect is reserved, nothing drawn.
                }
            });
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
    }
    keys.into_iter().filter(|k| store.len(k) > 0).count()
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
    let plot_id = format!("cell_{}_{}", cell.row, cell.col);

    // Title above the plot — measured so the plot below knows how much height
    // it has left. We use a fixed font size to keep the title row predictable
    // (16 px including baseline).
    const TITLE_H: f32 = 16.0;
    if !cell.title.is_empty() {
        ui.label(RichText::new(&cell.title).small().strong());
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

    let title_h = if cell.title.is_empty() { 0.0 } else { TITLE_H };
    let plot_h = (cell_rect.height() - title_h).max(40.0);
    let plot_w = cell_rect.width();

    // v0.10.0 — when interactive, enable the egui_plot zoom/pan/box-zoom
    // suite + double-click reset. The auto-scale-Y default holds UNTIL the
    // user interacts; after that, the per-cell `locked` flag stays `true`
    // until the right-click "Reset zoom" or a double-click flips it off.
    let resp = Plot::new(plot_id)
        .legend(Legend::default())
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

/// v0.10.0 — attach the per-cell right-click context menu to a plot response.
/// Drains user clicks into `sink` for the CLI to apply next frame.
fn attach_context_menu(resp: &egui::Response, cell: &Cell, sink: &mut Vec<CellMenuAction>) {
    let row = cell.row;
    let col = cell.col;
    resp.clone().context_menu(|ui| {
        if ui.button("Edit panel...").clicked() {
            sink.push(CellMenuAction::Edit { row, col });
            ui.close();
        }
        if ui.button("Hide panel").clicked() {
            sink.push(CellMenuAction::HideToggle { row, col });
            ui.close();
        }
        if ui.button("Reset zoom").clicked() {
            sink.push(CellMenuAction::ResetZoom { row, col });
            ui.close();
        }
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
        ui.separator();
        if ui.button("Delete panel").clicked() {
            sink.push(CellMenuAction::Delete { row, col });
            ui.close();
        }
    });
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

/// Paint the per-panel `label_mode` overlay onto `plot_rect`'s top-right corner
/// using the parent ui's painter (so it never participates in layout).
///
/// `plot_rect` is the screen-space rect returned by `Plot::show()` —
/// `PlotResponse::response.rect`. The overlay is drawn as a translucent black
/// rounded rectangle with the label text painted on top in [`TextStyle::Small`].
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

    // Top-right corner of the plot rect, inset by OVERLAY_PAD on both axes.
    let bg_size = text_size + 2.0 * OVERLAY_TEXT_INSET;
    let bg_min = egui::pos2(
        plot_rect.right() - OVERLAY_PAD - bg_size.x,
        plot_rect.top() + OVERLAY_PAD,
    );
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
