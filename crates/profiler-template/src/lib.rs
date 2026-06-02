//! profiler-template — JSON template loader for hvn-profiler.
//!
//! Templates describe a 2D panel grid (`grid.rows` × `grid.cols`), one
//! [`Cell`] per visible panel, plus a [`View3d`] block consumed by the 3D
//! trajectory renderer since v0.3.0 (`profiler_render::view3d`).
//!
//! The schema is shared with HVN-SITL (`templates/hvn-default.json`,
//! `templates/real-drone.json`). Every struct derives `serde(default)`
//! liberally and tolerates unknown fields, so the same binary keeps loading
//! templates that gain new keys in future SITL releases.
//!
//! ## Naming note
//! The per-trace template binding is [`CellSource`], *not* `Source` — the
//! latter name is taken by the runtime `Source` trait in `profiler-source`.

use std::path::Path;

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

pub mod bundled;
pub mod discovery;

pub use discovery::{
    bundled_json, discover, ensure_user_templates_dir, load_entry_json, scan_user_templates,
    user_templates_dir, TemplateEntry, TemplateOrigin,
};

/// Build-time crate version, for logging from the CLI.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Top-level template document.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Template {
    /// Human-readable template name (shown in the title bar).
    #[serde(default)]
    pub name: String,
    /// Free-text description.
    #[serde(default)]
    pub description: String,
    /// Grid dimensions for the 2D panel layout.
    #[serde(default)]
    pub grid: Grid,
    /// Section banners overlaid on the grid (labels / tints). Rendering of
    /// these is best-effort decoration; the layout is driven by `cells`.
    #[serde(default)]
    pub sections: Vec<Section>,
    /// One entry per panel in the grid.
    #[serde(default)]
    pub cells: Vec<Cell>,
    /// 3D trajectory view block. Rendered since v0.3.0.
    #[serde(default)]
    pub view_3d: Option<View3d>,
    /// Top-level view-slider config (`full ◀──▶ live`). The 3D renderer reads
    /// `min_window_s` / `valinit` from here.
    #[serde(default)]
    pub view_slider: Option<ViewSlider>,
    /// v0.8.0 — persisted UI state for Save / Save-as. Optional; templates
    /// authored before v0.8.0 omit this and load unchanged.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ui_state: Option<UiState>,
}

/// v0.8.0 — UI-state snapshot persisted alongside the template by Save / Save-as.
///
/// Pure JSON-compatible additive fields. All sub-fields default to "absent"
/// so an existing template (no `ui_state` block) loads unchanged.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct UiState {
    /// Per-cell visibility overrides, keyed by `"row,col"` (e.g. `"3,1"`).
    /// `false` overrides the JSON `cells[].visible` value at load time.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub cell_visibility: std::collections::BTreeMap<String, bool>,
    /// Per-cell label-mode overrides, keyed by `"row,col"`. When the global
    /// label override is `LabelMode::Off|Data|Metadata` at save time, that
    /// mode is stamped into the per-cell entries so it round-trips.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub cell_label_mode: std::collections::BTreeMap<String, LabelMode>,
    /// 3D view trail visibility, keyed by trail name (e.g. `"truth"`, `"ekf"`).
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub trail_visibility: std::collections::BTreeMap<String, bool>,
    /// 3D view trail-length slider position (0..1).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub trail_frac: Option<f64>,
    /// 3D view "view fraction" slider position (0..1).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub view_frac: Option<f64>,
}

/// 2D grid dimensions.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Grid {
    #[serde(default = "default_rows")]
    pub rows: usize,
    #[serde(default = "default_cols")]
    pub cols: usize,
}

fn default_rows() -> usize {
    1
}
fn default_cols() -> usize {
    1
}

impl Default for Grid {
    fn default() -> Self {
        Self {
            rows: default_rows(),
            cols: default_cols(),
        }
    }
}

/// A section banner anchored at a grid cell. Decorative; ignored if the
/// renderer chooses not to draw section headers.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Section {
    #[serde(default)]
    pub label: String,
    #[serde(default)]
    pub anchor_row: usize,
    #[serde(default)]
    pub anchor_col: usize,
    #[serde(default)]
    pub color: String,
    /// Optional: tint all rows from this index downward.
    #[serde(default)]
    pub tint_rows_from: Option<usize>,
    #[serde(default)]
    pub tint_color: Option<String>,
}

/// How a panel renders the primitive's underlying data.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Primitive {
    /// One `Line` per source (with optional fallback + transform).
    #[default]
    Scalar,
    /// 3 lines from an array base key (`base[0..2]`).
    Vector,
    /// Many sources, each its own line, on one plot.
    Overlay,
    /// One line = L2 norm of a vector source's components.
    Magnitude,
    /// One line = `source.key` minus `source.minus`, index-aligned.
    Diff,
    /// 3 component lines + a magnitude line.
    MagInterference,
    /// 3 lines (roll/pitch/yaw) converted to degrees.
    AttitudeRpy,
    /// Reserved — parsed but not plotted.
    StatusBadge,
    /// v0.12.0 — colored text/chip cell driven by a non-plot data source
    /// (flight mode string, armed bool, GPS fix type, status-text rolling log).
    /// See [`Cell::status`] for the config.
    Status,
    /// v0.14.0 — static literal-text panel. Renders [`Cell::text`] (with simple
    /// Markdown — `**bold**`, `\n` line breaks, `- bullet` lines) under an
    /// optional [`Cell::icon`] emoji and the cell title. No data source — the
    /// primitive ignores `sources` / `source` and reads only `text` / `icon`.
    /// Used to embed instructional / welcome panels in templates.
    InfoText,
}

/// Per-panel label overlay mode.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LabelMode {
    /// Draw nothing.
    #[default]
    Off,
    /// Draw the latest value of the primary source (+ optional min/max).
    Data,
    /// Draw a static metadata block (source path + units).
    Metadata,
}

/// Configuration for `LabelMode::Data`.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LabelData {
    /// Python-style format string, e.g. `"{:+.2f}"`. Best-effort translation;
    /// see `profiler-render`'s formatter. Empty → sensible default.
    #[serde(default)]
    pub format: String,
    /// Also show window min/max alongside the latest value.
    #[serde(default)]
    pub show_min_max: bool,
}

/// Configuration for `LabelMode::Metadata`.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LabelMetadata {
    #[serde(default)]
    pub source_path: String,
    #[serde(default)]
    pub units: String,
    /// Expected per-stream sample rate in Hz. `None` → omitted from the overlay.
    #[serde(default)]
    pub stream_rate_hz: Option<f64>,
}

/// One panel in the grid.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Cell {
    #[serde(default)]
    pub row: usize,
    #[serde(default)]
    pub col: usize,
    #[serde(default)]
    pub title: String,
    #[serde(default)]
    pub primitive: Primitive,
    #[serde(default)]
    pub sources: Vec<CellSource>,
    /// Cell-level line color (used by `diff` and any primitive whose single
    /// line takes its color from the cell rather than the source).
    #[serde(default)]
    pub color: Option<String>,
    /// Draw a horizontal y=0 reference line (used by `diff`).
    #[serde(default)]
    pub zero_reference_line: bool,
    /// `false` → reserve the grid slot but render nothing.
    #[serde(default = "default_true")]
    pub visible: bool,
    #[serde(default)]
    pub label_mode: LabelMode,
    #[serde(default)]
    pub label_data: Option<LabelData>,
    #[serde(default)]
    pub label_metadata: Option<LabelMetadata>,
    /// v0.12.0 — single source key for the [`Primitive::Status`] primitive.
    /// Resolved against the store as either a string-typed key
    /// (`flight_mode`), a bool (`armed`), an integer (`fix_type`), or a
    /// rolling text-log (`statustexts`).
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub source: String,
    /// v0.12.0 — kind discriminant for the [`Primitive::Status`] primitive.
    /// See [`StatusKind`] for the variants.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kind: Option<StatusKind>,
    /// v0.12.0 — color lookup for the [`Primitive::Status`] primitive.
    /// Keys match the resolved string form (e.g. `"GUIDED"`, `"True"`, the
    /// fix-type number `"3"`); the value is a hex `#rrggbb` color.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub color_map: std::collections::BTreeMap<String, String>,
    /// v0.12.0 — fallback color for the [`Primitive::Status`] primitive when
    /// the value is unknown / unmapped. Defaults to `#aaa` (light gray).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub default_color: Option<String>,
    /// v0.14.0 — for [`Primitive::InfoText`] cells: literal Markdown-ish text
    /// to render. Supports `**bold**`, line breaks (`\n`), and bullet lines
    /// (a line starting with `- ` becomes `• `). Optional on every other
    /// primitive; existing templates without this key parse unchanged.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,
    /// v0.14.0 — for [`Primitive::InfoText`] cells: optional emoji / glyph
    /// prefix shown at the top of the panel (e.g. `"👋"`). Optional on every
    /// other primitive; existing templates without this key parse unchanged.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub icon: Option<String>,
}

/// v0.12.0 — kind discriminant for the `status` primitive.
///
/// The renderer dispatches on this to decide what to draw inside the chip:
/// - `Text` / `Badge` — render the source's string value as-is.
/// - `FixType` — map a small integer (0..6) to GPS fix-type colors.
/// - `ArmedBool` — green "ARMED" / gray "DISARMED" chip.
/// - `TextLog` — rolling list of the latest N statustext entries with
///   severity-driven colors.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StatusKind {
    /// Render the raw string value of `source`.
    #[default]
    Text,
    /// Short text in a colored chip (alias of `Text` with smaller styling).
    Badge,
    /// GPS fix type. `0=none, 1=2D, 2=3D, 3=DGPS, 4=RTK_FLOAT,
    /// 5/6=RTK_FIXED`.
    FixType,
    /// `True` → green "ARMED"; `False` → gray "DISARMED".
    ArmedBool,
    /// Render the latest N statustext entries from a rolling buffer
    /// (newest first), colored by severity.
    TextLog,
    /// v0.14.0 — render an `EKF_STATUS_REPORT.flags` bitfield as a vertical
    /// list of `● FLAG_NAME` rows: green dot for set bits, gray dot for
    /// unset bits. The source is the integer-valued `ekf_flags` key.
    EkfFlags,
}

fn default_true() -> bool {
    true
}

/// A trace binding inside a [`Cell`]. Named `CellSource` (not `Source`) so it
/// doesn't collide with the runtime `Source` trait in `profiler-source`.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CellSource {
    /// Primary trace key, e.g. `"ap_raw_imu[0]"` or a vector base `"mag_xyz"`.
    #[serde(default)]
    pub key: String,
    /// Fallback key used when `key` has no data in the store.
    #[serde(default)]
    pub fallback: Option<String>,
    /// For `diff`: the subtrahend key (`key - minus`).
    #[serde(default)]
    pub minus: Option<String>,
    /// Display label (defaults to `key` when empty).
    #[serde(default)]
    pub label: String,
    /// Color string: matplotlib `C0..C9` or `#rrggbb`.
    #[serde(default)]
    pub color: String,
    /// Named value transform, e.g. `"rad_to_deg"`.
    #[serde(default)]
    pub transform: Option<String>,
    /// Multiplicative scale applied before plotting (e.g. mag `1000.0`).
    #[serde(default)]
    pub scale: Option<f64>,
    /// `mag_interference`: a "clean" reference vector base key.
    #[serde(default)]
    pub clean_key: Option<String>,
}

// ─── view_3d (parsed but unrendered) ─────────────────────────────────────────

/// 3D trajectory view block, consumed by the 3D renderer since v0.3.0
/// (`profiler_render::view3d`). `sources` (direct trails) and `deadreckon`
/// (synthesised trails) are typed; unknown keys (rects, etc.) flow into `extra`.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct View3d {
    #[serde(default)]
    pub title: String,
    #[serde(default)]
    pub xlabel: String,
    #[serde(default)]
    pub ylabel: String,
    #[serde(default)]
    pub zlabel: String,
    #[serde(default)]
    pub trails: Vec<Trail3d>,
    /// Initial trail-length fraction (0..1) of the buffer to display.
    #[serde(default = "default_trail_initial")]
    pub trail_slider_initial: f64,
    /// View slider config (maps 0..1 → visible time window). Lives at the
    /// template top level in the SITL schema, but we expose it here for the
    /// 3D renderer's convenience; the `Template` also re-parses it.
    #[serde(default)]
    pub view_slider: Option<ViewSlider>,
    /// Anything else (rects, slider config) is kept opaque for now.
    #[serde(flatten, default)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

fn default_trail_initial() -> f64 {
    0.25
}

/// One 3D trail definition.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Trail3d {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub label: String,
    #[serde(default)]
    pub color: String,
    /// Direct source bindings (`x`/`y`/`z_neg`). Absent for dead-reckon trails.
    #[serde(default)]
    pub sources: Option<Trail3dSources>,
    /// Dead-reckon synthesis config. Absent for direct trails.
    #[serde(default)]
    pub deadreckon: Option<Trail3dDeadreckon>,
}

/// Direct `(E, N, Up)` source bindings for a 3D trail.
///
/// `x` → East, `y` → North, `z_neg` → the NED-down key whose value is negated
/// to obtain Up (`Up = -D`). Each is a fully-qualified scalar store key such
/// as `"pos_truth_ned[1]"`.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Trail3dSources {
    #[serde(default)]
    pub x: String,
    #[serde(default)]
    pub y: String,
    #[serde(default)]
    pub z_neg: String,
}

/// Dead-reckon synthesis config for a 3D trail. The trail position is
/// double-integrated from body-frame `accel` rotated into NED by `quat`
/// (scalar-first `[w,x,y,z]`), seeded at the first `seed_from` position.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Trail3dDeadreckon {
    /// Vector base key for body-frame acceleration (gravity-excluded), e.g.
    /// `"accel"` → reads `accel[0..2]`.
    #[serde(default)]
    pub accel: String,
    /// Vector base key for the orientation quaternion, scalar-first
    /// `[w,x,y,z]`, e.g. `"quat_wxyz"` → reads `quat_wxyz[0..3]`.
    #[serde(default)]
    pub quat: String,
    /// Vector base key for the seed position (NED), e.g. `"pos_truth_ned"`.
    #[serde(default)]
    pub seed_from: String,
}

/// `view_slider` config — maps the 0..1 view fraction onto a visible time
/// window. `0.0` shows the full history; `1.0` shows only `min_window_s`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ViewSlider {
    /// Shortest visible window (seconds) at fraction `1.0`.
    #[serde(default = "default_min_window_s")]
    pub min_window_s: f64,
    /// Initial slider value (0..1).
    #[serde(default = "default_view_valinit")]
    pub valinit: f64,
    #[serde(default)]
    pub label: String,
    /// Anything else (rect, etc.) kept opaque.
    #[serde(flatten, default)]
    pub extra: serde_json::Map<String, serde_json::Value>,
}

fn default_min_window_s() -> f64 {
    2.0
}
fn default_view_valinit() -> f64 {
    0.85
}

impl Default for ViewSlider {
    fn default() -> Self {
        Self {
            min_window_s: default_min_window_s(),
            valinit: default_view_valinit(),
            label: String::new(),
            extra: serde_json::Map::new(),
        }
    }
}

// ─── loaders ─────────────────────────────────────────────────────────────────

impl Template {
    /// Parse a template from a JSON string.
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Result<Template> {
        serde_json::from_str(s).context("parsing template JSON")
    }

    /// Load and parse a template from a JSON file on disk.
    pub fn from_path(path: impl AsRef<Path>) -> Result<Template> {
        let p = path.as_ref();
        let text = std::fs::read_to_string(p)
            .with_context(|| format!("reading template {}", p.display()))?;
        Template::from_str(&text).with_context(|| format!("parsing template {}", p.display()))
    }

    /// Visible cells only (skips `visible: false` placeholders).
    pub fn visible_cells(&self) -> impl Iterator<Item = &Cell> {
        self.cells.iter().filter(|c| c.visible)
    }

    /// v0.8.0 — apply a [`UiState`] snapshot onto the template, mutating in
    /// place. Used after loading a saved user template so the grid + 3D view
    /// boot up in the same state the operator captured.
    pub fn apply_ui_state(&mut self, ui: &UiState) {
        for cell in self.cells.iter_mut() {
            let key = format!("{},{}", cell.row, cell.col);
            if let Some(vis) = ui.cell_visibility.get(&key) {
                cell.visible = *vis;
            }
            if let Some(mode) = ui.cell_label_mode.get(&key) {
                cell.label_mode = *mode;
            }
        }
        if let Some(view) = self.view_3d.as_mut() {
            if let Some(tf) = ui.trail_frac {
                view.trail_slider_initial = tf.clamp(0.01, 1.0);
            }
        }
        if let Some(vf) = ui.view_frac {
            let vs = self
                .view_slider
                .get_or_insert_with(ViewSlider::default);
            vs.valinit = vf.clamp(0.0, 1.0);
        }
    }

    /// Serialise the template (including any `ui_state` block) to a pretty
    /// JSON string. Used by Save / Save-as.
    pub fn to_pretty_json(&self) -> Result<String> {
        serde_json::to_string_pretty(self).context("serialising template to JSON")
    }

    /// v0.10.1 — minimal in-memory template the picker's "+ New blank
    /// template…" entry bootstraps. A 1×1 grid with zero cells; the operator
    /// populates it via "+ Add Panel", and the grid auto-grows when a new
    /// cell is added beyond the current `rows` / `cols` (see
    /// `apply_panel_draft`).
    pub fn blank(name: impl Into<String>) -> Template {
        Template {
            name: name.into(),
            description: String::new(),
            grid: Grid { rows: 1, cols: 1 },
            sections: Vec::new(),
            cells: Vec::new(),
            view_3d: None,
            view_slider: None,
            ui_state: None,
        }
    }
}

/// Backwards-compatible free function (v0.0.1 API).
pub fn load(path: impl AsRef<Path>) -> Result<Template> {
    Template::from_path(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn fixture(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests")
            .join("fixtures")
            .join(name)
    }

    #[test]
    fn parse_minimal() {
        let json = r#"{
            "name": "demo",
            "grid": {"rows": 1, "cols": 1},
            "cells": [
                {"row": 0, "col": 0, "title": "Roll",
                 "primitive": "scalar",
                 "sources": [{"key": "ap_attitude[0]"}]}
            ]
        }"#;
        let t = Template::from_str(json).unwrap();
        assert_eq!(t.name, "demo");
        assert_eq!(t.grid.rows, 1);
        assert_eq!(t.cells.len(), 1);
        assert_eq!(t.cells[0].sources[0].key, "ap_attitude[0]");
        assert_eq!(t.cells[0].primitive, Primitive::Scalar);
    }

    #[test]
    fn unknown_fields_tolerated() {
        // Cell carries `lw`, `marker`, `legend`, `status_text` etc. that we
        // don't model — they must not break deserialization.
        let json = r#"{
            "name": "x",
            "future_field": 42,
            "cells": [
                {"row": 0, "col": 0, "primitive": "scalar",
                 "sources": [{"key": "a", "lw": 0.8, "marker": ".", "markersize": 4}],
                 "legend": {"fontsize": 6}, "status_text": {"foo": "bar"}}
            ]
        }"#;
        let t = Template::from_str(json).unwrap();
        assert_eq!(t.cells.len(), 1);
    }

    #[test]
    fn loads_hvn_default() {
        let t = Template::from_path(fixture("hvn-default.json")).unwrap();
        assert_eq!(t.name, "hvn-default");
        assert_eq!(t.grid.rows, 7);
        assert_eq!(t.grid.cols, 3);
        // 7x3 grid → 21 cells defined (one is visible:false).
        assert_eq!(t.cells.len(), 21);
        assert_eq!(t.sections.len(), 2);
        // The invisible placeholder cell at (2,2).
        let invisible = t.cells.iter().filter(|c| !c.visible).count();
        assert_eq!(invisible, 1);
        assert_eq!(t.visible_cells().count(), 20);
        // view_3d parses with its trails.
        let v = t.view_3d.as_ref().expect("view_3d present");
        assert_eq!(v.trails.len(), 4);
        assert_eq!(v.trails[0].name, "truth");
    }

    #[test]
    fn view_3d_typed_sources_and_deadreckon() {
        let t = Template::from_path(fixture("hvn-default.json")).unwrap();
        let v = t.view_3d.as_ref().expect("view_3d present");
        // truth/gps/ekf carry typed (E,N,Up) source bindings.
        let truth = &v.trails[0];
        let src = truth.sources.as_ref().expect("truth has sources");
        assert_eq!(src.x, "pos_truth_ned[1]");
        assert_eq!(src.y, "pos_truth_ned[0]");
        assert_eq!(src.z_neg, "pos_truth_ned[2]");
        assert!(truth.deadreckon.is_none());
        // dr carries a typed deadreckon block, no direct sources.
        let dr = v.trails.iter().find(|tr| tr.name == "dr").unwrap();
        let dk = dr.deadreckon.as_ref().expect("dr has deadreckon");
        assert_eq!(dk.accel, "accel");
        assert_eq!(dk.quat, "quat_wxyz");
        assert_eq!(dk.seed_from, "pos_truth_ned");
        assert!(dr.sources.is_none());
        // top-level view_slider parses (min_window_s + valinit).
        let vs = t.view_slider.as_ref().expect("view_slider present");
        assert!((vs.min_window_s - 2.0).abs() < 1e-9);
        assert!((vs.valinit - 0.85).abs() < 1e-9);
    }

    #[test]
    fn hvn_default_primitives_parse() {
        let t = Template::from_path(fixture("hvn-default.json")).unwrap();
        // Spot-check each distinct primitive used in the template.
        let prim_at = |row: usize, col: usize| {
            t.cells
                .iter()
                .find(|c| c.row == row && c.col == col)
                .map(|c| c.primitive)
        };
        assert_eq!(prim_at(0, 0), Some(Primitive::Scalar));
        assert_eq!(prim_at(2, 0), Some(Primitive::Overlay));
        assert_eq!(prim_at(4, 0), Some(Primitive::MagInterference));
        assert_eq!(prim_at(6, 0), Some(Primitive::Diff));
        // diff cell carries minus + cell color + zero ref line.
        let diff = t.cells.iter().find(|c| c.row == 6 && c.col == 0).unwrap();
        assert_eq!(diff.sources[0].minus.as_deref(), Some("pos_ekf_ned[0]"));
        assert!(diff.zero_reference_line);
        // The diff line color lives on the source in this template.
        assert_eq!(diff.sources[0].color, "#d62728");
        // scalar attitude cell carries a transform.
        let roll = t.cells.iter().find(|c| c.row == 5 && c.col == 0).unwrap();
        assert_eq!(roll.sources[0].transform.as_deref(), Some("rad_to_deg"));
        // metadata label on accel fx.
        let accel = t.cells.iter().find(|c| c.row == 0 && c.col == 0).unwrap();
        assert_eq!(
            accel.label_metadata.as_ref().map(|m| m.source_path.as_str()),
            Some("RAW_IMU.xacc")
        );
    }

    #[test]
    fn loads_real_drone() {
        let t = Template::from_path(fixture("real-drone.json")).unwrap();
        assert_eq!(t.name, "real-drone");
        assert_eq!(t.grid.rows, 7);
        assert_eq!(t.grid.cols, 3);
        assert_eq!(t.cells.len(), 21);
        // real-drone has more invisible cells (no wind row).
        assert!(t.cells.iter().filter(|c| !c.visible).count() >= 4);
        let v = t.view_3d.as_ref().expect("view_3d present");
        // real-drone keeps only the EKF trail.
        assert_eq!(v.trails.len(), 1);
        assert_eq!(v.trails[0].name, "ekf");
    }
}
