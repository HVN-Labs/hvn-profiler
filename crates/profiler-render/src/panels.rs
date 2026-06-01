//! Multi-panel 2D layout renderer (v0.2.0).
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
//! Per-panel `label_mode` overlays (`data` / `metadata`) are drawn as text in
//! the panel's top-left corner.
//!
//! The primitive renderers map [`profiler_template::Primitive`] variants onto
//! egui_plot lines; see [`render_cell`].

use egui::{Align2, Color32, RichText};
use egui_plot::{HLine, Legend, Line, Plot, PlotPoint, PlotPoints, Text};

use profiler_template::{Cell, CellSource, LabelMode, Primitive, Template};

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

/// Same as [`render_template_grid`], with an explicit [`LabelOverride`] applied
/// uniformly to every cell. The default-arg helper above forwards `Respect`.
pub fn render_template_grid_with_override(
    ui: &mut egui::Ui,
    tpl: &Template,
    store: &TraceStore,
    label_override: LabelOverride,
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

    let avail = ui.available_size();
    // Leave a little vertical headroom so the last row isn't clipped.
    let cell_h = (avail.y / rows as f32).max(60.0);
    let cell_w = (avail.x / cols as f32).max(80.0);

    let mut stats = GridStats::default();

    ui.vertical(|ui| {
        for r in 0..rows {
            ui.horizontal(|ui| {
                for c in 0..cols {
                    let id = r * cols + c;
                    ui.allocate_ui(egui::vec2(cell_w, cell_h), |ui| {
                        match at[id] {
                            Some(cell) if cell.visible && !cell.sources.is_empty() => {
                                stats.panels += 1;
                                let had_data =
                                    render_cell(ui, cell, store, store.window_s, label_override);
                                if had_data {
                                    stats.panels_with_data += 1;
                                }
                            }
                            _ => {
                                // Reserve the slot; draw nothing (keeps alignment).
                                ui.allocate_space(egui::vec2(cell_w, cell_h));
                            }
                        }
                    });
                }
            });
        }
    });

    stats.keys_with_data = count_keys_with_data(tpl, store);
    stats
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

/// Render a single panel. Returns `true` if any line had at least one point.
fn render_cell(
    ui: &mut egui::Ui,
    cell: &Cell,
    store: &TraceStore,
    window_s: f64,
    label_override: LabelOverride,
) -> bool {
    let plot_id = format!("cell_{}_{}", cell.row, cell.col);

    // Title above the plot.
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

    let resp = Plot::new(plot_id)
        .legend(Legend::default())
        .show_axes([true, true])
        .show_grid([true, true])
        // Static panels: no live interaction (3D-only per design).
        .allow_drag(false)
        .allow_zoom(false)
        .allow_scroll(false)
        .allow_boxed_zoom(false)
        .show(ui, |plot_ui| {
            // X = rolling window. Y auto-scales to the visible data.
            if latest_ts.is_finite() {
                plot_ui.set_plot_bounds_x(x_lo..=x_hi);
            }
            plot_ui.set_auto_bounds([false, true]);

            any_data = draw_primitive(plot_ui, cell, store);

            if cell.zero_reference_line {
                plot_ui.hline(
                    HLine::new("zero", 0.0)
                        .color(Color32::from_gray(110))
                        .width(0.8),
                );
            }

            draw_label_overlay(plot_ui, cell, store, x_lo, x_hi, label_override);
        });

    let _ = resp;
    any_data
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

/// Draw the per-panel `label_mode` overlay in the top-left corner.
///
/// `label_override` lets a global toolbar / CLI flag force every cell into a
/// specific mode regardless of what the template asked for.
fn draw_label_overlay(
    plot_ui: &mut egui_plot::PlotUi,
    cell: &Cell,
    store: &TraceStore,
    x_lo: f64,
    x_hi: f64,
    label_override: LabelOverride,
) {
    let mode = label_override.resolve(cell.label_mode);
    match mode {
        LabelMode::Off => {}
        LabelMode::Data => {
            let Some(src) = cell.sources.first() else {
                return;
            };
            // Latest value of the primary source (honour fallback).
            let latest = store
                .latest(&src.key)
                .or_else(|| src.fallback.as_deref().and_then(|f| store.latest(f)));
            let Some(v) = latest else { return };
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
            place_corner_text(plot_ui, &text, x_lo, x_hi, Color32::from_gray(220));
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
                return;
            }
            place_corner_text(
                plot_ui,
                &parts.join("\n"),
                x_lo,
                x_hi,
                Color32::from_gray(170),
            );
        }
    }
}

/// Anchor a small text block to the top-left of the current plot bounds.
fn place_corner_text(
    plot_ui: &mut egui_plot::PlotUi,
    text: &str,
    x_lo: f64,
    x_hi: f64,
    color: Color32,
) {
    let bounds = plot_ui.plot_bounds();
    let y_top = bounds.max()[1];
    // Inset slightly from the left edge of the visible window.
    let x = x_lo + (x_hi - x_lo) * 0.02;
    plot_ui.text(
        Text::new("label", PlotPoint::new(x, y_top), RichText::new(text).small())
            .anchor(Align2::LEFT_TOP)
            .color(color),
    );
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
