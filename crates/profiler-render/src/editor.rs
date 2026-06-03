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

use std::collections::{BTreeSet, HashMap, HashSet};

use profiler_template::{Cell, CellSource, LabelMode, Primitive, StatusKind, Template, Trail3d, Trail3dSources, View3d};

use crate::TraceStore;

/// v0.12.0 — observed/declared shape of a source-key's value. Used by the
/// editor's primitive-inference helper ([`infer_primitive`]) and the picker
/// type-filter row to bucket each key into a primitive family.
///
/// Variants:
/// - `Scalar` — a single `f64` per timestamp (most channels).
/// - `Vector(N)` — `N` scalar components emitted as `base[0..N-1]`. Common
///   `N`: 2 (lat/lon), 3 (xyz), 4 (quat), 6 (RAW_IMU), 9–10 (SCALED_IMU2/3).
/// - `String` — a string-typed channel (e.g. `flight_mode`).
/// - `Bool` — `True` / `False` (e.g. `armed`).
/// - `TextLog` — rolling list of dicts (`statustexts`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueShape {
    /// Single `f64` scalar.
    Scalar,
    /// `N`-component vector (emitted as `base[0..N-1]`).
    Vector(usize),
    /// String-typed (e.g. `flight_mode`).
    String,
    /// Bool-typed (e.g. `armed`).
    Bool,
    /// Rolling list of dicts (e.g. `statustexts`).
    TextLog,
}

/// v0.11.0 / v0.12.0 — opinionated default schema of source-keys for HVN-SITL
/// fleets, each tagged with its [`ValueShape`].
///
/// The picker dropdown shows this list FIRST so users can author panel
/// templates against AP-fed channels (`ap_attitude`, `ap_raw_imu`, …) BEFORE
/// ArduPilot starts streaming. Without this seed the picker is empty / missing
/// AP entries for the first ~5–20 s after fleet startup because AP-emitted
/// keys flow as `None` from `dt_runner` until the autopilot wakes up, and
/// `TraceStore` only registers a key after a non-`None` value lands.
///
/// v0.12.0 — the list is now `&[(&str, ValueShape)]` so the editor can pick
/// a sensible default primitive without observing the wire format. Custom
/// (non-HVN) sources still get observed-key discovery merged on top via
/// [`collect_source_keys`].
pub const KNOWN_HVN_SITL_KEYS: &[(&str, ValueShape)] = &[
    // ── Timing ─────────────────────────────────────────────────────────────
    ("t", ValueShape::Scalar),
    // ── DT physics (truth / raw sensor models) ────────────────────────────
    ("accel[0]", ValueShape::Scalar), ("accel[1]", ValueShape::Scalar), ("accel[2]", ValueShape::Scalar),
    ("gyro[0]", ValueShape::Scalar), ("gyro[1]", ValueShape::Scalar), ("gyro[2]", ValueShape::Scalar),
    ("mag_xyz[0]", ValueShape::Scalar), ("mag_xyz[1]", ValueShape::Scalar), ("mag_xyz[2]", ValueShape::Scalar),
    ("mag_clean_xyz[0]", ValueShape::Scalar), ("mag_clean_xyz[1]", ValueShape::Scalar), ("mag_clean_xyz[2]", ValueShape::Scalar),
    ("wind_ned[0]", ValueShape::Scalar), ("wind_ned[1]", ValueShape::Scalar), ("wind_ned[2]", ValueShape::Scalar),
    ("baro_pressure", ValueShape::Scalar), ("baro_temp", ValueShape::Scalar),
    ("baro_alt", ValueShape::Scalar), ("state_alt", ValueShape::Scalar),
    ("gps_alt", ValueShape::Scalar), ("gps_vn", ValueShape::Scalar),
    ("quat_wxyz[0]", ValueShape::Scalar), ("quat_wxyz[1]", ValueShape::Scalar),
    ("quat_wxyz[2]", ValueShape::Scalar), ("quat_wxyz[3]", ValueShape::Scalar),
    ("euler[0]", ValueShape::Scalar), ("euler[1]", ValueShape::Scalar), ("euler[2]", ValueShape::Scalar),
    // ── AP MAVLink mirrors (what the autopilot sees) ──────────────────────
    ("ap_attitude[0]", ValueShape::Scalar), ("ap_attitude[1]", ValueShape::Scalar), ("ap_attitude[2]", ValueShape::Scalar),
    ("ap_raw_imu[0]", ValueShape::Scalar), ("ap_raw_imu[1]", ValueShape::Scalar), ("ap_raw_imu[2]", ValueShape::Scalar),
    ("ap_raw_imu[3]", ValueShape::Scalar), ("ap_raw_imu[4]", ValueShape::Scalar), ("ap_raw_imu[5]", ValueShape::Scalar),
    // v0.16.5 — mag indices 6..8 emit raw mGauss; `mag_xyz` Vec[3] (gauss) is
    // already declared elsewhere in this table.
    ("ap_raw_imu[6]", ValueShape::Scalar), ("ap_raw_imu[7]", ValueShape::Scalar), ("ap_raw_imu[8]", ValueShape::Scalar),
    ("ap_vfr_alt", ValueShape::Scalar),
    ("ap_vel_ned[0]", ValueShape::Scalar), ("ap_vel_ned[1]", ValueShape::Scalar), ("ap_vel_ned[2]", ValueShape::Scalar),
    // ── Position NED (truth / GPS sensor / EKF / target) ──────────────────
    ("pos_truth_ned[0]", ValueShape::Scalar), ("pos_truth_ned[1]", ValueShape::Scalar), ("pos_truth_ned[2]", ValueShape::Scalar),
    ("pos_gps_ned[0]", ValueShape::Scalar), ("pos_gps_ned[1]", ValueShape::Scalar), ("pos_gps_ned[2]", ValueShape::Scalar),
    ("pos_ekf_ned[0]", ValueShape::Scalar), ("pos_ekf_ned[1]", ValueShape::Scalar), ("pos_ekf_ned[2]", ValueShape::Scalar),
    ("pos_target_ned[0]", ValueShape::Scalar), ("pos_target_ned[1]", ValueShape::Scalar), ("pos_target_ned[2]", ValueShape::Scalar),
    // ── EKF status (v0.12.0) ──────────────────────────────────────────────
    ("ekf_flags", ValueShape::Scalar),
    ("ekf_velv", ValueShape::Scalar),
    ("ekf_pos_horiz", ValueShape::Scalar),
    ("ekf_pos_vert", ValueShape::Scalar),
    ("ekf_compv", ValueShape::Scalar),
    ("ekf_terralt", ValueShape::Scalar),
    // ── AHRS2 (secondary attitude, v0.12.0) ───────────────────────────────
    ("ahrs2_roll", ValueShape::Scalar),
    ("ahrs2_pitch", ValueShape::Scalar),
    ("ahrs2_yaw", ValueShape::Scalar),
    ("ahrs2_alt", ValueShape::Scalar),
    ("ahrs2_lat", ValueShape::Scalar),
    ("ahrs2_lng", ValueShape::Scalar),
    // ── Vibration (v0.12.0) ───────────────────────────────────────────────
    ("vibex", ValueShape::Scalar),
    ("vibey", ValueShape::Scalar),
    ("vibez", ValueShape::Scalar),
    ("vibeclip0", ValueShape::Scalar),
    ("vibeclip1", ValueShape::Scalar),
    ("vibeclip2", ValueShape::Scalar),
    // ── Secondary IMUs (v0.12.0) — base + 10-component (ax,ay,az,gx,gy,gz,mx,my,mz,temp)
    ("scaled_imu2[0]", ValueShape::Scalar), ("scaled_imu2[1]", ValueShape::Scalar),
    ("scaled_imu2[2]", ValueShape::Scalar), ("scaled_imu2[3]", ValueShape::Scalar),
    ("scaled_imu2[4]", ValueShape::Scalar), ("scaled_imu2[5]", ValueShape::Scalar),
    ("scaled_imu2[6]", ValueShape::Scalar), ("scaled_imu2[7]", ValueShape::Scalar),
    ("scaled_imu2[8]", ValueShape::Scalar), ("scaled_imu2[9]", ValueShape::Scalar),
    ("scaled_imu3[0]", ValueShape::Scalar), ("scaled_imu3[1]", ValueShape::Scalar),
    ("scaled_imu3[2]", ValueShape::Scalar), ("scaled_imu3[3]", ValueShape::Scalar),
    ("scaled_imu3[4]", ValueShape::Scalar), ("scaled_imu3[5]", ValueShape::Scalar),
    ("scaled_imu3[6]", ValueShape::Scalar), ("scaled_imu3[7]", ValueShape::Scalar),
    ("scaled_imu3[8]", ValueShape::Scalar), ("scaled_imu3[9]", ValueShape::Scalar),
    // ── Pressures (v0.12.0) — abs / diff / temp ───────────────────────────
    ("press_scaled[0]", ValueShape::Scalar), ("press_scaled[1]", ValueShape::Scalar),
    ("press_scaled[2]", ValueShape::Scalar),
    ("press_scaled2[0]", ValueShape::Scalar), ("press_scaled2[1]", ValueShape::Scalar),
    ("press_scaled2[2]", ValueShape::Scalar),
    // ── Battery (v0.12.0) ─────────────────────────────────────────────────
    ("battery_voltage", ValueShape::Scalar),
    ("battery_current", ValueShape::Scalar),
    ("battery_remaining", ValueShape::Scalar),
    // ── ESC, first 4 motors (v0.12.0) ─────────────────────────────────────
    ("esc_rpm[0]", ValueShape::Scalar), ("esc_rpm[1]", ValueShape::Scalar),
    ("esc_rpm[2]", ValueShape::Scalar), ("esc_rpm[3]", ValueShape::Scalar),
    ("esc_voltage[0]", ValueShape::Scalar), ("esc_voltage[1]", ValueShape::Scalar),
    ("esc_voltage[2]", ValueShape::Scalar), ("esc_voltage[3]", ValueShape::Scalar),
    ("esc_current[0]", ValueShape::Scalar), ("esc_current[1]", ValueShape::Scalar),
    ("esc_current[2]", ValueShape::Scalar), ("esc_current[3]", ValueShape::Scalar),
    // ── RC channels + servos (v0.12.0) ────────────────────────────────────
    ("rc_channels[0]", ValueShape::Scalar), ("rc_channels[1]", ValueShape::Scalar),
    ("rc_channels[2]", ValueShape::Scalar), ("rc_channels[3]", ValueShape::Scalar),
    ("rc_channels[4]", ValueShape::Scalar), ("rc_channels[5]", ValueShape::Scalar),
    ("rc_channels[6]", ValueShape::Scalar), ("rc_channels[7]", ValueShape::Scalar),
    ("rc_channels[8]", ValueShape::Scalar), ("rc_channels[9]", ValueShape::Scalar),
    ("rc_channels[10]", ValueShape::Scalar), ("rc_channels[11]", ValueShape::Scalar),
    ("rc_channels[12]", ValueShape::Scalar), ("rc_channels[13]", ValueShape::Scalar),
    ("rc_channels[14]", ValueShape::Scalar), ("rc_channels[15]", ValueShape::Scalar),
    ("rc_rssi", ValueShape::Scalar),
    ("servo_outputs[0]", ValueShape::Scalar), ("servo_outputs[1]", ValueShape::Scalar),
    ("servo_outputs[2]", ValueShape::Scalar), ("servo_outputs[3]", ValueShape::Scalar),
    ("servo_outputs[4]", ValueShape::Scalar), ("servo_outputs[5]", ValueShape::Scalar),
    ("servo_outputs[6]", ValueShape::Scalar), ("servo_outputs[7]", ValueShape::Scalar),
    ("servo_outputs[8]", ValueShape::Scalar), ("servo_outputs[9]", ValueShape::Scalar),
    ("servo_outputs[10]", ValueShape::Scalar), ("servo_outputs[11]", ValueShape::Scalar),
    ("servo_outputs[12]", ValueShape::Scalar), ("servo_outputs[13]", ValueShape::Scalar),
    ("servo_outputs[14]", ValueShape::Scalar), ("servo_outputs[15]", ValueShape::Scalar),
    // ── NAV controller (v0.12.0) ──────────────────────────────────────────
    ("nav_roll", ValueShape::Scalar),
    ("nav_pitch", ValueShape::Scalar),
    ("nav_bearing", ValueShape::Scalar),
    ("target_bearing", ValueShape::Scalar),
    ("wp_dist", ValueShape::Scalar),
    ("alt_error", ValueShape::Scalar),
    ("aspd_error", ValueShape::Scalar),
    ("xtrack_error", ValueShape::Scalar),
    // ── System status (v0.12.0) ───────────────────────────────────────────
    ("sys_load", ValueShape::Scalar),
    ("sys_drop_rate_comm", ValueShape::Scalar),
    ("sys_errors[0]", ValueShape::Scalar), ("sys_errors[1]", ValueShape::Scalar),
    ("sys_errors[2]", ValueShape::Scalar), ("sys_errors[3]", ValueShape::Scalar),
    // ── Status (v0.12.0) — special: string / bool / text-log ──────────────
    ("armed", ValueShape::Bool),
    ("flight_mode", ValueShape::String),
    ("fix_type", ValueShape::Scalar),
    ("statustexts", ValueShape::TextLog),
];

/// v0.12.0 — return the [`ValueShape`] for a known key, or `None` if the key
/// is not in [`KNOWN_HVN_SITL_KEYS`]. Comparison ignores array indexing on
/// known list entries — `accel[0]` and `accel[1]` both resolve to the entry
/// with matching base. Custom keys observed at runtime fall back to `None`
/// and the caller infers from observation.
pub fn known_value_shape(key: &str) -> Option<ValueShape> {
    for (k, s) in KNOWN_HVN_SITL_KEYS {
        if *k == key {
            return Some(*s);
        }
    }
    None
}

/// v0.12.0 — infer the default primitive for a given [`ValueShape`].
///
/// Used by the editor's Add Panel modal to pre-select a sensible primitive
/// when the user picks a source key. The user can still change it via the
/// existing dropdown.
pub fn infer_primitive(value_shape: &ValueShape) -> &'static str {
    match value_shape {
        ValueShape::Scalar => "scalar",
        ValueShape::Vector(2) => "scalar",
        ValueShape::Vector(3) => "vector",
        ValueShape::Vector(6) => "scalar",
        ValueShape::Vector(_) => "scalar",
        ValueShape::String => "status",
        ValueShape::Bool => "status",
        ValueShape::TextLog => "status",
    }
}

/// v0.13.0 — pick a sensible [`StatusKind`] default for a source key
/// when the editor switches the cell into [`Primitive::Status`].
///
/// Name-based heuristic with [`ValueShape`] fallback:
/// - `armed` / `armed_bool` / any `Bool`-shaped key → `ArmedBool`.
/// - `fix_type` → `FixType`.
/// - `statustexts` / `TextLog`-shaped → `TextLog`.
/// - String-shaped (`flight_mode`, etc.) → `Text`.
/// - Otherwise returns `None` — the modal keeps the cell's current kind.
///
/// The form picks this up when the operator chooses a Status-typed source
/// key from the dropdown; manual edits via the kind selector are
/// respected (this helper only fires when the source key changes).
pub fn default_status_kind(key: &str, shape: &ValueShape) -> Option<StatusKind> {
    let base = key.split(['[', '.']).next().unwrap_or(key);
    if base == "armed" || base == "armed_bool" {
        return Some(StatusKind::ArmedBool);
    }
    if base == "fix_type" {
        return Some(StatusKind::FixType);
    }
    if base == "statustexts" {
        return Some(StatusKind::TextLog);
    }
    // v0.14.0 — ArduPilot EKF_STATUS_REPORT.flags bitfield.
    if base == "ekf_flags" {
        return Some(StatusKind::EkfFlags);
    }
    match shape {
        ValueShape::String => Some(StatusKind::Text),
        ValueShape::Bool => Some(StatusKind::ArmedBool),
        ValueShape::TextLog => Some(StatusKind::TextLog),
        _ => None,
    }
}

/// v0.12.0 — freshness classification of a source key in the editor's picker
/// dropdown.
///
/// Drives the color + style applied to the entry:
/// - `Live` — observed in the last [`LIVE_THRESHOLD_S`] seconds (bright text).
/// - `Stale` — observed before that, but not in the last [`STALE_THRESHOLD_S`]
///   seconds (dim text).
/// - `SchemaOnly` — in the static [`KNOWN_HVN_SITL_KEYS`] list but never
///   observed (italic dim text).
/// - `Custom` — observed but not in the static list (warm-tinted text).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyFreshness {
    /// Has fresh, non-null data in the last [`LIVE_THRESHOLD_S`] seconds.
    Live,
    /// Was observed but not recently.
    Stale,
    /// In the static known-key vocabulary but never observed.
    SchemaOnly,
    /// Observed but not in the static known-key vocabulary.
    Custom,
}

/// v0.12.0 — a key is "Live" if observed within this many seconds.
pub const LIVE_THRESHOLD_S: f64 = 3.0;

/// v0.12.0 — a key is "Stale" (not "Live") if last observed before this
/// many seconds ago; before that we treat it as freshly observed.
pub const STALE_THRESHOLD_S: f64 = 30.0;

/// v0.12.0 — classify a key's freshness given a last-seen registry and the
/// current monotonic time.
///
/// `last_seen.get(key)` returns the monotonic seconds at which a non-null
/// value last arrived. Missing entries → either `SchemaOnly` (key is in
/// [`KNOWN_HVN_SITL_KEYS`]) or `Custom` (key is observed somewhere — caller
/// passes `observed = true`).
pub fn classify_key(
    key: &str,
    last_seen: &HashMap<String, f64>,
    now_s: f64,
    observed: bool,
) -> KeyFreshness {
    if let Some(t) = last_seen.get(key) {
        let age = (now_s - t).max(0.0);
        if age <= LIVE_THRESHOLD_S {
            return KeyFreshness::Live;
        }
        if age <= STALE_THRESHOLD_S {
            // Within the stale window but not live.
            return KeyFreshness::Stale;
        }
        // Older than stale threshold — still stale (it WAS observed).
        return KeyFreshness::Stale;
    }
    // Never observed.
    if known_value_shape(key).is_some() {
        KeyFreshness::SchemaOnly
    } else if observed {
        // Observed in a store but never had a real value (null-key).
        KeyFreshness::Custom
    } else {
        KeyFreshness::SchemaOnly
    }
}

/// v0.12.0 — picker type-filter row state (Status / 2D scalar / 2D vector / 3D).
///
/// Defaults all-on; the operator toggles classes off to hide them from the
/// dropdown.
#[derive(Debug, Clone, Copy)]
pub struct PickerTypeFilter {
    /// Show status-typed keys (string / bool / text-log) and keys whose
    /// names match status-typed patterns (`armed`, `flight_mode`,
    /// `statustexts`, `ekf_flags`, `fix_type`, `sys_*`).
    pub status: bool,
    /// Show scalar / 1-vector keys.
    pub scalar_2d: bool,
    /// Show 2..10-vector keys.
    pub vector_2d: bool,
    /// Show 3D-position keys (`pos_*_ned`, `*_lat`, `*_lng`, `*_alt`).
    pub d3: bool,
}

impl Default for PickerTypeFilter {
    fn default() -> Self {
        Self {
            status: true,
            scalar_2d: true,
            vector_2d: true,
            d3: true,
        }
    }
}

impl PickerTypeFilter {
    /// `true` when `key` matches any enabled class.
    ///
    /// A key passes when at least one of its inferred classifications matches
    /// an enabled filter slot. Names without an inferred shape fall through
    /// to the `scalar_2d` slot.
    pub fn allows(&self, key: &str, shape: Option<ValueShape>) -> bool {
        let is_status = match shape {
            Some(ValueShape::String) | Some(ValueShape::Bool) | Some(ValueShape::TextLog) => true,
            _ => key_is_status_name(key),
        };
        let is_3d = key_is_3d_name(key);
        let is_vec = matches!(shape, Some(ValueShape::Vector(n)) if (2..=10).contains(&n));
        let is_scalar = matches!(shape, Some(ValueShape::Scalar) | Some(ValueShape::Vector(1)))
            || (shape.is_none() && !is_status);

        (is_status && self.status)
            || (is_vec && self.vector_2d)
            || (is_3d && self.d3)
            || (is_scalar && self.scalar_2d)
    }
}

/// v0.12.0 — name-based check for "looks like a status key".
fn key_is_status_name(key: &str) -> bool {
    let base = key.split(['[', '.']).next().unwrap_or(key);
    matches!(
        base,
        "armed" | "flight_mode" | "statustexts" | "ekf_flags" | "fix_type"
    ) || base.starts_with("sys_")
}

/// v0.12.0 — name-based check for "looks like a 3D-position key".
fn key_is_3d_name(key: &str) -> bool {
    if key.starts_with("pos_") && key.contains("_ned") {
        return true;
    }
    let base = key.split(['[', '.']).next().unwrap_or(key);
    base.ends_with("_lat") || base.ends_with("_lng") || base.ends_with("_alt")
}

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
    /// v0.15.0 — optional source URI to pin this cell to. Empty string means
    /// `(any)` (the default): the renderer uses the first connected source
    /// at draw time. Non-empty values are written out as
    /// [`CellSource::source_uri`] so the JSON round-trip preserves the
    /// per-cell selection.
    pub source_uri: String,
    /// Optional fallback (when `source_key` has no data, plot this instead).
    pub fallback: String,
    /// For `Diff` only: the subtrahend key (`source_key − minus`).
    pub minus: String,
    /// `#rrggbb` color string (the matplotlib `C0..C9` shorthand also parses).
    pub color: String,
    pub label_mode: LabelMode,
    /// Extra source keys, used by `Overlay`. Each entry becomes its own line.
    pub overlay_extra_keys: Vec<String>,
    /// v0.13.0 — `Status` primitive: kind discriminant (text / armed_bool /
    /// fix_type / text_log). Defaults to `Text` when the modal switches into
    /// Status mode. Ignored for non-Status primitives.
    pub status_kind: StatusKind,
    /// v0.13.0 — `Status` primitive: list of `(string, color)` rows. The
    /// editor renders one row per entry with an `×` removal button and a
    /// `+ Add row` button to append a new blank entry.
    pub status_color_map: Vec<(String, String)>,
    /// v0.13.0 — `Status` primitive: fallback color used when the source
    /// value doesn't match any `status_color_map` row. Defaults to `#aaa`.
    pub status_default_color: String,
}

impl Default for PanelDraft {
    fn default() -> Self {
        Self {
            row: 0,
            col: 0,
            primitive: Primitive::Scalar,
            title: String::new(),
            source_key: String::new(),
            // v0.15.0 — empty == "(any)" — defer source binding to the first
            // connected leg at render time.
            source_uri: String::new(),
            fallback: String::new(),
            minus: String::new(),
            color: "#1f77b4".to_string(),
            label_mode: LabelMode::Off,
            overlay_extra_keys: Vec::new(),
            status_kind: StatusKind::Text,
            status_color_map: Vec::new(),
            status_default_color: "#aaaaaa".to_string(),
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
    // v0.15.0 — `source_uri` is written ONLY when the operator picked a
    // specific source in the form (non-empty string). The `(any)` default
    // leaves the field as `None` so existing templates round-trip unchanged.
    let source_uri = non_empty(&draft.source_uri);
    let mut sources: Vec<CellSource> = vec![CellSource {
        key: draft.source_key.trim().to_string(),
        source_uri: source_uri.clone(),
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
                    // v0.15.0 — overlay extras inherit the cell's
                    // source_uri. Multi-source overlays will get
                    // per-source pins in a later release.
                    source_uri: source_uri.clone(),
                    ..Default::default()
                });
            }
        }
    }

    // v0.13.0 — Status-specific fields: only populated when the primitive
    // is `Primitive::Status`. Keeps the JSON small for the common scalar /
    // vector / overlay / diff cases (the fields all carry
    // `skip_serializing_if`).
    let (source, kind, color_map, default_color) = if draft.primitive == Primitive::Status {
        let mut cm = std::collections::BTreeMap::new();
        for (k, v) in &draft.status_color_map {
            let k = k.trim();
            let v = v.trim();
            if !k.is_empty() && !v.is_empty() {
                cm.insert(k.to_string(), v.to_string());
            }
        }
        (
            draft.source_key.trim().to_string(),
            Some(draft.status_kind),
            cm,
            non_empty(&draft.status_default_color),
        )
    } else {
        (
            String::new(),
            None,
            std::collections::BTreeMap::new(),
            None,
        )
    };

    let cell = Cell {
        row: draft.row,
        col: draft.col,
        title: draft.title.clone(),
        primitive: draft.primitive,
        sources,
        color: non_empty(&draft.color),
        visible: true,
        label_mode: draft.label_mode,
        source,
        kind,
        color_map,
        default_color,
        ..Default::default()
    };
    tpl.cells.push(cell);
    Ok(())
}

/// v0.16.0 — find the first unoccupied `(row, col)` in row-major order for the
/// "+ Add Panel" modal's default slot.
///
/// Returns the first `(r, c)` (scanning rows 0..grid.rows, cols 0..grid.cols)
/// that has no cell. If the grid is completely full, returns `(grid.rows, 0)`
/// — one row past the end — so [`apply_panel_draft`] auto-grows the grid on
/// submit (existing behavior, see the `draft.row >= tpl.grid.rows` branch).
///
/// Background: previously the Add Panel modal always defaulted to `(0, 0)`.
/// Repeated clicks of "+ Add Panel" with an Apply at the defaults stacked
/// multiple cells at `(0, 0)`, which the renderer surfaces as overlapping
/// plots fighting for the same screen rect (visible as flicker/glitch).
pub fn first_available_slot(template: &Template) -> (usize, usize) {
    let rows = template.grid.rows;
    let cols = template.grid.cols;
    let occupied: HashSet<(usize, usize)> = template
        .cells
        .iter()
        .map(|c| (c.row, c.col))
        .collect();
    for r in 0..rows {
        for c in 0..cols {
            if !occupied.contains(&(r, c)) {
                return (r, c);
            }
        }
    }
    // Grid is full — point at the row past the end so `apply_panel_draft`
    // auto-grows the grid (existing behavior added in v0.10.1).
    (rows, 0)
}

/// v0.16.0 — Add-only submit path: reject occupied slots up front so the
/// "+ Add Panel" modal cannot stack two cells on the same `(row, col)`.
///
/// [`apply_panel_draft`] itself still accepts duplicates (the Edit path
/// relies on a remove-then-apply round-trip via [`replace_cell_at`]). This
/// wrapper is what the toolbar "+ Add Panel" commit handler calls.
///
/// On occupied slot, returns
/// `Err("Panel already exists at (r,c); pick a different slot or delete the existing one first")`
/// and leaves the template untouched.
pub fn add_panel_draft(tpl: &mut Template, draft: &PanelDraft) -> Result<(), String> {
    let row = draft.row;
    let col = draft.col;
    let occupied = tpl
        .cells
        .iter()
        .any(|c| c.row == row && c.col == col);
    if occupied {
        return Err(format!(
            "Panel already exists at ({row},{col}); \
             pick a different slot or delete the existing one first"
        ));
    }
    apply_panel_draft(tpl, draft)
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
        // Status-typed (v0.12.0)
        "armed" | "flight_mode" | "statustexts" | "fix_type" | "ekf_flags"
            => "Status",
        // EKF status (v0.12.0)
        "ekf_velv" | "ekf_pos_horiz" | "ekf_pos_vert" | "ekf_compv" | "ekf_terralt"
            => "EKF Status",
        // AHRS2 secondary attitude (v0.12.0)
        "ahrs2_roll" | "ahrs2_pitch" | "ahrs2_yaw"
        | "ahrs2_alt" | "ahrs2_lat" | "ahrs2_lng"
            => "AHRS2 (secondary)",
        // Vibration (v0.12.0)
        "vibex" | "vibey" | "vibez"
        | "vibeclip0" | "vibeclip1" | "vibeclip2"
            => "Vibration",
        // Secondary IMUs (v0.12.0)
        "scaled_imu2" | "scaled_imu3"
            => "AP IMU (secondary)",
        // Pressures (v0.12.0)
        "press_scaled" | "press_scaled2"
            => "AP Pressure",
        // Battery (v0.12.0)
        "battery_voltage" | "battery_current" | "battery_remaining"
            => "Battery",
        // ESC (v0.12.0)
        "esc_rpm" | "esc_voltage" | "esc_current"
            => "ESC",
        // RC + servos (v0.12.0)
        "rc_channels" | "rc_rssi" | "servo_outputs"
            => "Radio / Servos",
        // NAV controller (v0.12.0)
        "nav_roll" | "nav_pitch" | "nav_bearing" | "target_bearing"
        | "wp_dist" | "alt_error" | "aspd_error" | "xtrack_error"
            => "Navigation",
        // System status (v0.12.0)
        k if k.starts_with("sys_") => "System",
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
    "Status",
    "DT physics",
    "AP MAVLink",
    "AHRS2 (secondary)",
    "AP IMU (secondary)",
    "AP Pressure",
    "EKF Status",
    "Vibration",
    "Battery",
    "ESC",
    "Radio / Servos",
    "Navigation",
    "System",
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
    //
    // v0.12.0 — the static list now carries a `ValueShape` per entry; we
    // only need the key here.
    for (k, _shape) in KNOWN_HVN_SITL_KEYS {
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

/// v0.15.0 — resolve a cell's declared `source_uri` against the currently-
/// connected source list.
///
/// Returns the URI the renderer should actually use, plus a flag indicating
/// whether a fallback was applied (i.e. the declared URI was not connected
/// AND a different source was substituted). The toolbar surfaces a status
/// warning when this fires.
///
/// Rules:
/// 1. `declared = None` → use first connected URI (or `None` if no sources).
///    `fallback_applied = false` (operator asked for `(any)`).
/// 2. `declared = Some(uri)` AND `uri` is connected → honour it exactly.
///    `fallback_applied = false`.
/// 3. `declared = Some(uri)` AND `uri` is NOT connected → first connected
///    URI (or `None`); `fallback_applied = true`.
pub fn resolve_source_uri(
    declared: Option<&str>,
    connected: &[String],
) -> ResolvedSource {
    match declared {
        None => ResolvedSource {
            uri: connected.first().cloned(),
            fallback_applied: false,
        },
        Some(d) => {
            if connected.iter().any(|c| c == d) {
                ResolvedSource {
                    uri: Some(d.to_string()),
                    fallback_applied: false,
                }
            } else {
                ResolvedSource {
                    uri: connected.first().cloned(),
                    fallback_applied: true,
                }
            }
        }
    }
}

/// v0.15.0 — return value of [`resolve_source_uri`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedSource {
    /// The URI the renderer should use, or `None` when no sources are
    /// connected at all. Callers paint "waiting for data..." in the latter
    /// case (same as the v0.10.0 no-data path).
    pub uri: Option<String>,
    /// `true` when a non-`None` declared URI was substituted because the
    /// declared one isn't currently connected. Drives the toolbar warning.
    pub fallback_applied: bool,
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
