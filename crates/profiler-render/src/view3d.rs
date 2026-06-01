//! 3D trajectory view (v0.3.0) — orbit-camera painter projection.
//!
//! `egui_plot` is 2D-only, so the 3D trajectory is drawn directly with egui's
//! [`egui::Painter`]: every world point `(E, N, Up)` is projected to a screen
//! pixel through a small orbit camera (azimuth + elevation + scale), then
//! trails are stroked as polylines, axes as arrows, and a faint ground grid is
//! laid down for orientation.
//!
//! All the heavy lifting is pure `f64` math (no `glam`/`nalgebra`, no 3D
//! engine) — see [`OrbitCamera::project`], [`integrate_deadreckon`],
//! [`window_lo`], and [`decimate`], each unit-tested below.
//!
//! ## Conventions
//! - World axes: `x = East`, `y = North`, `z = Up`. A trail's screen point at
//!   frame `k` is `(x_key, y_key, -d_key)` (the template's `z_neg` already
//!   names the NED-down key to negate).
//! - The dead-reckon trail has no direct position source: it is
//!   double-integrated from body-frame `accel` rotated into NED by the
//!   orientation quaternion (`[w, x, y, z]`, scalar-first), seeded at the first
//!   `seed_from` truth position. No gravity term is added — `accel` already
//!   excludes it.
//!
//! Per-frame recompute of the dead-reckon trail is acceptable for v0.3.0 (the
//! window is bounded by the store's retention); incremental integration is a
//! noted perf follow-up.

use std::collections::HashMap;

use egui::{Align2, Color32, FontId, Pos2, Rect, Sense, Stroke, Vec2};

use profiler_template::{LabelMode, Trail3d, View3d};

use crate::panels::LabelOverride;
use crate::TraceStore;

/// A trail's full `(t, [E, N, Up])` history before cropping.
type TrailHistory = Vec<(f64, [f64; 3])>;
/// A named, colored trail with its full history (pre-crop).
type NamedTrail = (String, Color32, TrailHistory);
/// A named, colored trail reduced to drawn `(E, N, Up)` points (post-crop).
type DrawnTrail = (String, Color32, Vec<[f64; 3]>);

/// Per-frame stats for the 1 Hz status log.
#[derive(Debug, Clone, Default)]
pub struct View3dStats {
    /// Number of trails whose visibility checkbox is on AND that produced ≥1
    /// drawn point this frame.
    pub trails_visible: usize,
    /// Drawn (post-window, post-decimation) point count per trail name.
    pub points: Vec<(String, usize)>,
}

impl View3dStats {
    /// Look up a trail's drawn point count by name (0 if absent).
    pub fn pts(&self, name: &str) -> usize {
        self.points
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, c)| *c)
            .unwrap_or(0)
    }
}

/// Orbit camera: azimuth (yaw about Up), elevation (tilt), an orthographic
/// scale (pixels per world-unit), and the world-space point the camera looks
/// at (the data centroid, refreshed by [`OrbitCamera::auto_fit`]).
#[derive(Debug, Clone)]
pub struct OrbitCamera {
    /// Azimuth in radians (rotation about the world Up axis).
    pub azimuth: f64,
    /// Elevation in radians (tilt above the horizon).
    pub elevation: f64,
    /// Orthographic scale — screen pixels per world unit.
    pub scale: f64,
    /// World-space look-at point (data centroid).
    pub center: [f64; 3],
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            // A gentle 3/4 view by default.
            azimuth: 0.6,
            elevation: 0.45,
            scale: 20.0,
            center: [0.0, 0.0, 0.0],
        }
    }
}

impl OrbitCamera {
    /// Project a world point `(E, N, Up)` to camera-space `(right, up, depth)`
    /// before any screen offset/flip. Standard azimuth-about-vertical rotation
    /// followed by an elevation tilt; orthographic in `depth`.
    ///
    /// Returns camera coordinates in *world units* (scale is applied later by
    /// [`Self::project`]). Exposed for testing the rotation in isolation.
    pub fn camera_space(&self, world: [f64; 3]) -> [f64; 3] {
        let (dx, dy, dz) = (
            world[0] - self.center[0],
            world[1] - self.center[1],
            world[2] - self.center[2],
        );
        let (sa, ca) = self.azimuth.sin_cos();
        // Yaw about Up (z): rotate the E/N plane.
        let right = dx * ca - dy * sa;
        let fwd = dx * sa + dy * ca;
        // Tilt by elevation: blend forward into vertical.
        let (se, ce) = self.elevation.sin_cos();
        let up = dz * ce - fwd * se;
        let depth = dz * se + fwd * ce;
        [right, up, depth]
    }

    /// Project a world point `(E, N, Up)` to a screen pixel inside `rect`.
    /// Orthographic: screen `x` grows right, screen `y` grows *down* (so world
    /// Up maps to screen-up via a negation).
    pub fn project(&self, world: [f64; 3], rect: Rect) -> Pos2 {
        let cs = self.camera_space(world);
        let cx = rect.center().x as f64;
        let cy = rect.center().y as f64;
        Pos2::new(
            (cx + cs[0] * self.scale) as f32,
            (cy - cs[1] * self.scale) as f32,
        )
    }

    /// Fit the camera to a set of world points: recenter on their centroid and
    /// choose a scale so the projected spread fills ~80% of `rect`. No-op if
    /// `pts` is empty. Leaves azimuth/elevation untouched.
    pub fn auto_fit(&mut self, pts: &[[f64; 3]], rect: Rect) {
        if pts.is_empty() {
            return;
        }
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in pts {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        self.center = [
            0.5 * (lo[0] + hi[0]),
            0.5 * (lo[1] + hi[1]),
            0.5 * (lo[2] + hi[2]),
        ];
        // Largest extent across the three axes (with a floor to avoid div-by-0).
        let extent = (0..3)
            .map(|k| hi[k] - lo[k])
            .fold(0.0_f64, f64::max)
            .max(1e-3);
        let span_px = (rect.width().min(rect.height()) as f64) * 0.8;
        self.scale = span_px / extent;
    }
}

/// Live-control + camera state for the 3D view, persisted across frames.
#[derive(Debug, Clone)]
pub struct View3dState {
    pub camera: OrbitCamera,
    /// View fraction (0..1): 0 = full history, 1 = only `min_window_s`.
    pub view_frac: f64,
    /// Trail-length fraction (0..1) of the visible buffer to draw.
    pub trail_frac: f64,
    /// Display decimation: draw every `decimate`-th point (1 = all).
    pub decimate: usize,
    /// Auto-follow latest data (auto-fit + window tracking) when `true`.
    pub realtime: bool,
    /// Shortest window (s) at `view_frac == 1.0`, from the template.
    pub min_window_s: f64,
    /// Per-trail visibility, keyed by trail name.
    pub visible: HashMap<String, bool>,
    /// One-shot flag: defaults applied from the template yet?
    initialised: bool,
}

impl Default for View3dState {
    fn default() -> Self {
        Self {
            camera: OrbitCamera::default(),
            view_frac: 0.85,
            trail_frac: 0.25,
            decimate: 1,
            realtime: true,
            min_window_s: 2.0,
            visible: HashMap::new(),
            initialised: false,
        }
    }
}

impl View3dState {
    /// Seed control defaults from the template the first time we see it.
    pub fn init_from(&mut self, view: &View3d, view_slider_min_window_s: Option<f64>, valinit: Option<f64>) {
        if self.initialised {
            return;
        }
        self.trail_frac = view.trail_slider_initial.clamp(0.01, 1.0);
        if let Some(m) = view_slider_min_window_s {
            self.min_window_s = m;
        }
        if let Some(v) = valinit {
            self.view_frac = v.clamp(0.0, 1.0);
        }
        for t in &view.trails {
            self.visible.entry(t.name.clone()).or_insert(true);
        }
        self.initialised = true;
    }

    /// Is trail `name` currently visible? (Defaults to `true` if unseen.)
    pub fn is_visible(&self, name: &str) -> bool {
        self.visible.get(name).copied().unwrap_or(true)
    }
}

/// Compute the lower time bound of the visible window.
///
/// `view_frac == 0.0` → show everything (`t_min`). `view_frac == 1.0` → show
/// only the last `min_window_s`. The window width interpolates linearly
/// between the full span and `min_window_s`.
pub fn window_lo(t_min: f64, t_max: f64, view_frac: f64, min_window_s: f64) -> f64 {
    if !t_max.is_finite() || !t_min.is_finite() || t_max <= t_min {
        return t_min;
    }
    let full = t_max - t_min;
    let frac = view_frac.clamp(0.0, 1.0);
    // Visible width shrinks from `full` (frac=0) to `min_window_s` (frac=1).
    let width = full + (min_window_s - full) * frac;
    let width = width.clamp(min_window_s.min(full), full);
    t_max - width
}

/// Select every `step`-th element (1 = identity). `step == 0` is treated as 1.
/// Always keeps the final element so the latest point survives decimation.
pub fn decimate<T: Copy>(pts: &[T], step: usize) -> Vec<T> {
    let step = step.max(1);
    if step == 1 || pts.len() <= 2 {
        return pts.to_vec();
    }
    let mut out: Vec<T> = pts.iter().copied().step_by(step).collect();
    let last = *pts.last().unwrap();
    // step_by may already include the last element; only append if missing.
    if !(pts.len() - 1).is_multiple_of(step) {
        out.push(last);
    }
    out
}

/// Rotate a body-frame vector into NED by a scalar-first quaternion
/// `q = [w, x, y, z]`: returns `R(q) · v`. Used by the dead-reckon integrator.
pub fn quat_rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    // Rotation matrix rows from the (assumed unit) quaternion.
    let r00 = 1.0 - 2.0 * (y * y + z * z);
    let r01 = 2.0 * (x * y - w * z);
    let r02 = 2.0 * (x * z + w * y);
    let r10 = 2.0 * (x * y + w * z);
    let r11 = 1.0 - 2.0 * (x * x + z * z);
    let r12 = 2.0 * (y * z - w * x);
    let r20 = 2.0 * (x * z - w * y);
    let r21 = 2.0 * (y * z + w * x);
    let r22 = 1.0 - 2.0 * (x * x + y * y);
    [
        r00 * v[0] + r01 * v[1] + r02 * v[2],
        r10 * v[0] + r11 * v[1] + r12 * v[2],
        r20 * v[0] + r21 * v[1] + r22 * v[2],
    ]
}

/// Double-integrate a dead-reckon position trail in NED, then convert to
/// `(E, N, Up)` world points.
///
/// `accel[k]` is body-frame, gravity-excluded; `quat[k]` is the scalar-first
/// orientation at that sample. We rotate `accel` into NED (`a = R·accel`),
/// integrate `v += a·dt; p += v·dt`, seeding `p` at `seed_ned` and `v` at 0.
/// Both inputs must be time-aligned (same length, shared timestamps).
///
/// Returns one `(E, N, Up)` point per input sample where `Up = -D`.
pub fn integrate_deadreckon(
    accel_ned_times: &[(f64, [f64; 3])],
    quats: &[(f64, [f64; 4])],
    seed_ned: [f64; 3],
) -> Vec<[f64; 3]> {
    let n = accel_ned_times.len().min(quats.len());
    if n == 0 {
        return Vec::new();
    }
    let mut pos = seed_ned; // NED
    let mut vel = [0.0_f64; 3];
    let mut out = Vec::with_capacity(n);
    // Emit the seed as the first point (Up = -D).
    out.push([pos[0], pos[1], -pos[2]]);
    for i in 1..n {
        let dt = accel_ned_times[i].0 - accel_ned_times[i - 1].0;
        // Guard against zero / negative dt from out-of-order or duplicate ts.
        let dt = if dt > 0.0 && dt < 10.0 { dt } else { 0.0 };
        let a_ned = quat_rotate(quats[i].1, accel_ned_times[i].1);
        for k in 0..3 {
            vel[k] += a_ned[k] * dt;
            pos[k] += vel[k] * dt;
        }
        out.push([pos[0], pos[1], -pos[2]]);
    }
    out
}

// ─── Trail extraction ────────────────────────────────────────────────────────

/// Resolve a single trail to its full `(t, [E, N, Up])` history from the store
/// (before any window / trail-length / decimation cropping).
///
/// Exposed for the v0.5.0 trail-decode integration test.
pub fn trail_world_points(trail: &Trail3d, store: &TraceStore) -> Vec<(f64, [f64; 3])> {
    if let Some(src) = &trail.sources {
        // Direct trail: E = x, N = y, Up = -(z_neg key).
        let xs = store.points(&src.x);
        let ys = store.points(&src.y);
        let zs = store.points(&src.z_neg);
        let n = xs.len().min(ys.len()).min(zs.len());
        return (0..n)
            .map(|i| (xs[i][0], [xs[i][1], ys[i][1], -zs[i][1]]))
            .collect();
    }
    if let Some(dk) = &trail.deadreckon {
        // Dead-reckon: rebuild accel/quat vectors, seed from truth.
        let accel = store.vec3(
            &format!("{}[0]", dk.accel),
            &format!("{}[1]", dk.accel),
            &format!("{}[2]", dk.accel),
        );
        let quat = store.vec4(
            &format!("{}[0]", dk.quat),
            &format!("{}[1]", dk.quat),
            &format!("{}[2]", dk.quat),
            &format!("{}[3]", dk.quat),
        );
        if accel.is_empty() || quat.is_empty() {
            return Vec::new();
        }
        // Seed from the first available seed_from NED position.
        let seed_pts = store.vec3(
            &format!("{}[0]", dk.seed_from),
            &format!("{}[1]", dk.seed_from),
            &format!("{}[2]", dk.seed_from),
        );
        let seed_ned = seed_pts.first().map(|(_, p)| *p).unwrap_or([0.0; 3]);
        let world = integrate_deadreckon(&accel, &quat, seed_ned);
        let n = world.len().min(accel.len());
        return (0..n).map(|i| (accel[i].0, world[i])).collect();
    }
    Vec::new()
}

/// Crop a trail's `(t, world)` history to the visible window, apply the
/// trail-length fraction, and decimate. Returns just the world points.
fn crop_trail(
    pts: &[(f64, [f64; 3])],
    t_lo: f64,
    trail_frac: f64,
    decimate_step: usize,
) -> Vec<[f64; 3]> {
    // 1. Time window.
    let windowed: Vec<[f64; 3]> = pts
        .iter()
        .filter(|(t, _)| *t >= t_lo)
        .map(|(_, p)| *p)
        .collect();
    // 2. Trail-length fraction: keep the most-recent `frac` of the window.
    let frac = trail_frac.clamp(0.01, 1.0);
    let keep = ((windowed.len() as f64) * frac).ceil() as usize;
    let start = windowed.len().saturating_sub(keep.max(1));
    let trimmed = &windowed[start..];
    // 3. Display decimation.
    decimate(trimmed, decimate_step)
}

// ─── Rendering ─────────────────────────────────────────────────────────────

/// matplotlib-ish hex / `CN` color parse, reusing the panels palette indirectly
/// via a local copy (kept private to avoid cross-module coupling).
fn parse_trail_color(spec: &str) -> Color32 {
    let s = spec.trim();
    if let Some(hex) = s.strip_prefix('#') {
        if hex.len() == 6 {
            if let (Ok(r), Ok(g), Ok(b)) = (
                u8::from_str_radix(&hex[0..2], 16),
                u8::from_str_radix(&hex[2..4], 16),
                u8::from_str_radix(&hex[4..6], 16),
            ) {
                return Color32::from_rgb(r, g, b);
            }
        }
    }
    Color32::from_gray(200)
}

/// Render the 3D view (controls strip + projected trajectory) into `ui`.
/// Returns per-frame stats for the status log.
pub fn render_view3d(
    ui: &mut egui::Ui,
    view: &View3d,
    store: &TraceStore,
    state: &mut View3dState,
) -> View3dStats {
    render_view3d_with_override(ui, view, store, state, LabelOverride::default())
}

/// Same as [`render_view3d`], with a global [`LabelOverride`] applied to the
/// viewport's optional label-mode overlay (drawn in the bottom-left corner).
///
/// The 3D view honours `LabelOverride::Force(LabelMode::Data | Metadata)` and
/// surfaces a small text block summarising the last trail tip (data) or the
/// trail source bindings (metadata). `LabelOverride::Respect` is a no-op —
/// the 3D view has no per-cell `label_mode` of its own.
pub fn render_view3d_with_override(
    ui: &mut egui::Ui,
    view: &View3d,
    store: &TraceStore,
    state: &mut View3dState,
    label_override: LabelOverride,
) -> View3dStats {
    // ── Controls strip ───────────────────────────────────────────────────
    ui.horizontal_wrapped(|ui| {
        ui.label(egui::RichText::new("3D controls").strong());
        ui.separator();
        ui.checkbox(&mut state.realtime, "realtime");
        ui.separator();
        ui.label("view:");
        ui.add(egui::Slider::new(&mut state.view_frac, 0.0..=1.0).text("full◀▶live"));
        ui.separator();
        ui.label("trail:");
        ui.add(egui::Slider::new(&mut state.trail_frac, 0.01..=1.0).text("len"));
        ui.separator();
        ui.label("decimate:");
        let mut dec = state.decimate as f64;
        ui.add(egui::Slider::new(&mut dec, 1.0..=20.0).integer().suffix("×"));
        state.decimate = (dec as usize).max(1);
    });
    ui.horizontal_wrapped(|ui| {
        ui.label("trails:");
        for t in &view.trails {
            let mut on = state.is_visible(&t.name);
            let label = if t.label.is_empty() { &t.name } else { &t.label };
            if ui.checkbox(&mut on, label).changed() {
                state.visible.insert(t.name.clone(), on);
            }
        }
    });
    ui.separator();

    // ── Resolve + crop trails ────────────────────────────────────────────
    let t_max = store.latest_ts();
    // Global earliest timestamp across the trails we'll draw.
    let mut full: Vec<NamedTrail> = Vec::with_capacity(view.trails.len());
    let mut t_min = f64::INFINITY;
    for t in &view.trails {
        let pts = trail_world_points(t, store);
        if let Some((first, _)) = pts.first() {
            t_min = t_min.min(*first);
        }
        full.push((t.name.clone(), parse_trail_color(&t.color), pts));
    }
    let t_lo = window_lo(t_min, t_max, state.view_frac, state.min_window_s);

    let mut drawn: Vec<DrawnTrail> = Vec::with_capacity(full.len());
    let mut all_pts: Vec<[f64; 3]> = Vec::new();
    for (name, color, pts) in &full {
        if !state.is_visible(name) {
            drawn.push((name.clone(), *color, Vec::new()));
            continue;
        }
        let cropped = crop_trail(pts, t_lo, state.trail_frac, state.decimate);
        all_pts.extend_from_slice(&cropped);
        drawn.push((name.clone(), *color, cropped));
    }

    // ── Canvas + interaction ─────────────────────────────────────────────
    let avail = ui.available_size();
    let (resp, painter) = ui.allocate_painter(avail, Sense::click_and_drag());
    let rect = resp.rect;
    painter.rect_filled(rect, 0.0, Color32::from_gray(16));

    // Mouse-drag orbits (azimuth/elevation); wheel zooms.
    if resp.dragged() {
        let d = resp.drag_delta();
        state.camera.azimuth += d.x as f64 * 0.01;
        state.camera.elevation = (state.camera.elevation + d.y as f64 * 0.01)
            .clamp(-1.4, 1.4);
        // A manual orbit implies the user wants to inspect → pause follow.
        state.realtime = false;
    }
    if resp.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y) as f64;
        if scroll.abs() > 0.0 {
            let factor = (scroll * 0.0015).exp();
            state.camera.scale = (state.camera.scale * factor).clamp(0.05, 5000.0);
        }
    }

    // Realtime auto-fit follows the live data; frozen keeps the last camera.
    if state.realtime {
        state.camera.auto_fit(&all_pts, rect);
    }

    // ── Draw ground grid + axes + trails ─────────────────────────────────
    draw_ground_grid(&painter, &state.camera, rect);
    draw_axes(&painter, &state.camera, rect);

    let mut stats = View3dStats::default();
    for (name, color, pts) in &drawn {
        if pts.len() >= 2 {
            let screen: Vec<Pos2> = pts.iter().map(|p| state.camera.project(*p, rect)).collect();
            painter.line(screen, Stroke::new(1.6, *color));
        }
        if let Some(last) = pts.last() {
            let s = state.camera.project(*last, rect);
            painter.circle_filled(s, 3.5, *color);
        }
        if !pts.is_empty() {
            stats.trails_visible += 1;
        }
        stats.points.push((name.clone(), pts.len()));
    }

    // Title overlay.
    if !view.title.is_empty() {
        painter.text(
            rect.left_top() + Vec2::new(8.0, 6.0),
            Align2::LEFT_TOP,
            &view.title,
            FontId::proportional(13.0),
            Color32::from_gray(220),
        );
    }
    let mode = if state.realtime { "live" } else { "frozen" };
    painter.text(
        rect.right_top() + Vec2::new(-8.0, 6.0),
        Align2::RIGHT_TOP,
        format!("[{mode}]  az {:.2}  el {:.2}", state.camera.azimuth, state.camera.elevation),
        FontId::monospace(11.0),
        Color32::from_gray(150),
    );

    // ── Bottom-left label overlay (v0.5.0) ───────────────────────────────
    if let LabelOverride::Force(lm) = label_override {
        let block = build_3d_label_block(lm, view, &drawn);
        if !block.is_empty() {
            painter.text(
                rect.left_bottom() + Vec2::new(8.0, -8.0),
                Align2::LEFT_BOTTOM,
                block,
                FontId::monospace(11.0),
                Color32::from_gray(200),
            );
        }
    }

    stats
}

/// Build the multi-line text for the 3D viewport's bottom-left label overlay.
///
/// `LabelMode::Data` lists each visible trail with its latest `(E, N, Up)` tip.
/// `LabelMode::Metadata` lists each trail's name + source bindings (direct
/// trails) or dead-reckon block (synthesised trails). `LabelMode::Off` is empty.
fn build_3d_label_block(lm: LabelMode, view: &View3d, drawn: &[DrawnTrail]) -> String {
    match lm {
        LabelMode::Off => String::new(),
        LabelMode::Data => {
            let mut lines: Vec<String> = Vec::with_capacity(drawn.len());
            for (name, _color, pts) in drawn {
                if let Some(p) = pts.last() {
                    lines.push(format!(
                        "{name}: E {:+.2}  N {:+.2}  Up {:+.2}",
                        p[0], p[1], p[2]
                    ));
                }
            }
            lines.join("\n")
        }
        LabelMode::Metadata => {
            let mut lines: Vec<String> = Vec::with_capacity(view.trails.len());
            for t in &view.trails {
                if let Some(src) = &t.sources {
                    lines.push(format!(
                        "{}: x={}  y={}  z_neg={}",
                        t.name, src.x, src.y, src.z_neg
                    ));
                } else if let Some(dk) = &t.deadreckon {
                    lines.push(format!(
                        "{}: dead-reckon accel={} quat={} seed={}",
                        t.name, dk.accel, dk.quat, dk.seed_from
                    ));
                }
            }
            lines.join("\n")
        }
    }
}

/// Draw a faint ground grid in the world `z = 0` (Up = 0) plane, spanning a
/// region around the camera center sized to the current scale.
fn draw_ground_grid(painter: &egui::Painter, cam: &OrbitCamera, rect: Rect) {
    let stroke = Stroke::new(0.5, Color32::from_gray(48));
    // Choose a grid extent in world units so ~10 lines fill the canvas.
    let half_world = (rect.width().min(rect.height()) as f64) / cam.scale * 0.5;
    if !half_world.is_finite() || half_world <= 0.0 {
        return;
    }
    let step = nice_step(half_world * 2.0 / 10.0);
    let n = (half_world / step).ceil() as i32;
    let c = cam.center;
    for i in -n..=n {
        let off = i as f64 * step;
        // Lines parallel to East (vary N).
        let a = cam.project([c[0] - half_world, c[1] + off, 0.0], rect);
        let b = cam.project([c[0] + half_world, c[1] + off, 0.0], rect);
        painter.line_segment([a, b], stroke);
        // Lines parallel to North (vary E).
        let a = cam.project([c[0] + off, c[1] - half_world, 0.0], rect);
        let b = cam.project([c[0] + off, c[1] + half_world, 0.0], rect);
        painter.line_segment([a, b], stroke);
    }
}

/// Draw the three world axes (E/N/Up) as colored arrows from the camera center
/// with text labels for orientation.
fn draw_axes(painter: &egui::Painter, cam: &OrbitCamera, rect: Rect) {
    let len = (rect.width().min(rect.height()) as f64) / cam.scale * 0.35;
    if !len.is_finite() || len <= 0.0 {
        return;
    }
    let o = cam.center;
    let origin = cam.project(o, rect);
    let specs: [([f64; 3], Color32, &str); 3] = [
        ([o[0] + len, o[1], o[2]], Color32::from_rgb(0xe0, 0x60, 0x60), "E"),
        ([o[0], o[1] + len, o[2]], Color32::from_rgb(0x60, 0xc0, 0x60), "N"),
        ([o[0], o[1], o[2] + len], Color32::from_rgb(0x60, 0x90, 0xe0), "Up"),
    ];
    for (tip, color, label) in specs {
        let p = cam.project(tip, rect);
        painter.line_segment([origin, p], Stroke::new(1.5, color));
        painter.text(p, Align2::CENTER_CENTER, label, FontId::monospace(12.0), color);
    }
}

/// Round a raw step up to a "nice" 1/2/5 × 10^k value.
fn nice_step(raw: f64) -> f64 {
    if raw <= 0.0 || !raw.is_finite() {
        return 1.0;
    }
    let exp = raw.log10().floor();
    let base = 10f64.powf(exp);
    let frac = raw / base;
    let nice = if frac < 1.5 {
        1.0
    } else if frac < 3.5 {
        2.0
    } else if frac < 7.5 {
        5.0
    } else {
        10.0
    };
    nice * base
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn project_identity_camera_origin_to_center() {
        // Zero azimuth/elevation, unit scale, centered at origin: the world
        // origin projects to rect center; +E goes right; +Up goes up (screen
        // y decreases).
        let cam = OrbitCamera {
            azimuth: 0.0,
            elevation: 0.0,
            scale: 1.0,
            center: [0.0, 0.0, 0.0],
        };
        let rect = Rect::from_min_size(Pos2::new(0.0, 0.0), Vec2::new(200.0, 200.0));
        let c = cam.project([0.0, 0.0, 0.0], rect);
        assert!((c.x - 100.0).abs() < 1e-4);
        assert!((c.y - 100.0).abs() < 1e-4);
        // +10 East → +10 px right, same y.
        let east = cam.project([10.0, 0.0, 0.0], rect);
        assert!((east.x - 110.0).abs() < 1e-4);
        assert!((east.y - 100.0).abs() < 1e-4);
        // +10 Up → screen y decreases by 10.
        let up = cam.project([0.0, 0.0, 10.0], rect);
        assert!((up.x - 100.0).abs() < 1e-4);
        assert!((up.y - 90.0).abs() < 1e-4);
    }

    #[test]
    fn quat_identity_is_noop() {
        let v = [1.0, 2.0, 3.0];
        let r = quat_rotate([1.0, 0.0, 0.0, 0.0], v);
        for k in 0..3 {
            assert!((r[k] - v[k]).abs() < 1e-12);
        }
    }

    #[test]
    fn quat_90deg_about_z_rotates_x_to_y() {
        // q for +90° about Up (z): w=cos45, z=sin45.
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let r = quat_rotate([s, 0.0, 0.0, s], [1.0, 0.0, 0.0]);
        assert!((r[0] - 0.0).abs() < 1e-9);
        assert!((r[1] - 1.0).abs() < 1e-9);
        assert!((r[2] - 0.0).abs() < 1e-9);
    }

    #[test]
    fn deadreckon_constant_accel_is_parabolic() {
        // Identity attitude, constant +1 m/s² along NED-North, dt = 1 s.
        // p(t) = 0.5 a t² ⇒ at samples t=0..3: N = 0, 0.5, 2.0, 4.5 (Euler
        // semi-implicit: v += a*dt then p += v*dt ⇒ p = 1,3,6 ... check shape).
        let q = [1.0, 0.0, 0.0, 0.0];
        let accel: Vec<(f64, [f64; 3])> = (0..4)
            .map(|i| (i as f64, [0.0, 1.0, 0.0]))
            .collect();
        let quats: Vec<(f64, [f64; 4])> = (0..4).map(|i| (i as f64, q)).collect();
        let out = integrate_deadreckon(&accel, &quats, [0.0, 0.0, 0.0]);
        assert_eq!(out.len(), 4);
        // Semi-implicit Euler with dt=1: v=0,1,2,3 ; p(N)=0,1,3,6.
        let n: Vec<f64> = out.iter().map(|p| p[1]).collect();
        assert!((n[0] - 0.0).abs() < 1e-9);
        assert!((n[1] - 1.0).abs() < 1e-9);
        assert!((n[2] - 3.0).abs() < 1e-9);
        assert!((n[3] - 6.0).abs() < 1e-9);
        // Strictly increasing (parabolic-like growth), no E/Up drift.
        assert!(n[3] > n[2] && n[2] > n[1] && n[1] > n[0]);
        for p in &out {
            assert!((p[0]).abs() < 1e-12); // E
            assert!((p[2]).abs() < 1e-12); // Up = -D, D stays 0
        }
    }

    #[test]
    fn deadreckon_seed_is_first_point() {
        let accel = [(0.0, [0.0, 0.0, 0.0])];
        let quats = [(0.0, [1.0, 0.0, 0.0, 0.0])];
        let out = integrate_deadreckon(&accel, &quats, [5.0, 6.0, 7.0]);
        assert_eq!(out.len(), 1);
        // Up = -D = -7.
        assert_eq!(out[0], [5.0, 6.0, -7.0]);
    }

    #[test]
    fn window_lo_full_vs_live() {
        // frac=0 → full history (t_min). frac=1 → last min_window_s.
        let lo_full = window_lo(0.0, 100.0, 0.0, 2.0);
        assert!((lo_full - 0.0).abs() < 1e-9);
        let lo_live = window_lo(0.0, 100.0, 1.0, 2.0);
        assert!((lo_live - 98.0).abs() < 1e-9);
        // Midpoint interpolates the window width linearly.
        let lo_mid = window_lo(0.0, 100.0, 0.5, 2.0);
        // width = 100 + (2-100)*0.5 = 51 ⇒ lo = 49.
        assert!((lo_mid - 49.0).abs() < 1e-9);
    }

    #[test]
    fn window_lo_degenerate_spans() {
        // Non-finite / empty spans collapse to t_min.
        assert_eq!(window_lo(5.0, 5.0, 0.5, 2.0), 5.0);
        assert_eq!(window_lo(f64::NEG_INFINITY, 1.0, 0.5, 2.0), f64::NEG_INFINITY);
    }

    #[test]
    fn decimate_every_nth_keeps_last() {
        let pts: Vec<i32> = (0..10).collect();
        let d = decimate(&pts, 3);
        // indices 0,3,6,9 → 9 is last and already on the stride.
        assert_eq!(d, vec![0, 3, 6, 9]);
        // step=2 over 0..9: 0,2,4,6,8 + last(9) appended (8 not == last).
        let d2 = decimate(&pts, 2);
        assert_eq!(d2, vec![0, 2, 4, 6, 8, 9]);
        // step=1 is identity.
        assert_eq!(decimate(&pts, 1), pts);
        // step=0 treated as 1.
        assert_eq!(decimate(&pts, 0), pts);
    }

    #[test]
    fn crop_trail_applies_window_frac_decimate() {
        // 11 points at t=0..10 along North.
        let pts: Vec<(f64, [f64; 3])> =
            (0..=10).map(|i| (i as f64, [0.0, i as f64, 0.0])).collect();
        // Window from t_lo=5 keeps t=5..10 (6 pts). trail_frac=1, no decimate.
        let c = crop_trail(&pts, 5.0, 1.0, 1);
        assert_eq!(c.len(), 6);
        assert_eq!(c.first().unwrap()[1], 5.0);
        assert_eq!(c.last().unwrap()[1], 10.0);
        // trail_frac=0.5 of 6 → ceil(3) most-recent points: t=8,9,10.
        let c2 = crop_trail(&pts, 5.0, 0.5, 1);
        assert_eq!(c2.len(), 3);
        assert_eq!(c2.first().unwrap()[1], 8.0);
    }

    #[test]
    fn auto_fit_centers_on_centroid() {
        let mut cam = OrbitCamera::default();
        let pts = [[0.0, 0.0, 0.0], [10.0, 20.0, 4.0]];
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(200.0, 200.0));
        cam.auto_fit(&pts, rect);
        assert!((cam.center[0] - 5.0).abs() < 1e-9);
        assert!((cam.center[1] - 10.0).abs() < 1e-9);
        assert!((cam.center[2] - 2.0).abs() < 1e-9);
        assert!(cam.scale > 0.0);
    }
}
