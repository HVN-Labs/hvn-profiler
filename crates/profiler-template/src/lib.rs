//! profiler-template — JSON template loader for hvn-profiler.
//!
//! Templates describe a 2D panel grid (`grid.rows` × `grid.cols`), one
//! [`Cell`] per visible panel, plus a [`View3d`] block that v0.2.0 parses but
//! does **not** render (3D lands in v0.5.0).
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
    /// 3D trajectory view block. Parsed-but-unrendered in v0.2.0.
    #[serde(default)]
    pub view_3d: Option<View3d>,
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

/// 3D trajectory view block. v0.2.0 parses this so deserialization of full
/// SITL templates succeeds, but renders nothing. v0.5.0 will consume it.
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
    /// Anything else (rects, slider config) is kept opaque for now.
    #[serde(flatten, default)]
    pub extra: serde_json::Map<String, serde_json::Value>,
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
    pub sources: Option<serde_json::Value>,
    /// Dead-reckon synthesis config. Absent for direct trails.
    #[serde(default)]
    pub deadreckon: Option<serde_json::Value>,
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
