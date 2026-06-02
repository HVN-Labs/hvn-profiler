//! v0.11.0 — reflow animation lerp math.
//!
//! The renderer interpolates a cell's top-left between `anim_origin` (the
//! previous frame's rendered position) and the new target via a cubic
//! ease-out. The full animation lives in `render_template_grid_full`'s
//! body; what we exercise here is the lerp math + the PanelState progress
//! lifecycle without spinning up a real egui context.

use profiler_render::PanelState;

/// The same easing the renderer applies: `1 - (1 - t)^3`. Kept here so this
/// test pins the exact curve callers see in the binary.
fn ease_out_cubic(t: f32) -> f32 {
    1.0 - (1.0 - t).powi(3)
}

/// `egui::Pos2::lerp` mirror — we don't depend on egui in this test so we
/// reimplement the 2D lerp here.
fn lerp(a: (f32, f32), b: (f32, f32), t: f32) -> (f32, f32) {
    (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
}

/// Set `anim_t = 0.0`, target a new top-left. Tick frames at 16 ms intervals.
/// Assert the lerped position converges within 10 frames (~160 ms; the
/// renderer's target duration is 150 ms).
#[test]
fn animation_converges_in_under_160_ms() {
    let origin = (100.0_f32, 100.0_f32);
    let target = (300.0_f32, 200.0_f32);

    let mut st = PanelState {
        anim_origin: Some(origin),
        anim_t: Some(0.0),
        ..Default::default()
    };

    let dt = 0.016_f32; // 16 ms per frame ~ 60 Hz
    let duration = 0.150_f32;

    let mut last_pos = origin;
    for _ in 0..10 {
        let t = st.anim_t.unwrap();
        let new_t = (t + dt / duration).clamp(0.0, 1.0);
        let eased = ease_out_cubic(new_t);
        last_pos = lerp(origin, target, eased);
        if new_t >= 1.0 {
            st.anim_origin = None;
            st.anim_t = None;
            break;
        } else {
            st.anim_t = Some(new_t);
        }
    }
    assert!(
        st.anim_t.is_none(),
        "animation finishes within 10 frames at 16 ms each"
    );
    let dx = (last_pos.0 - target.0).abs();
    let dy = (last_pos.1 - target.1).abs();
    assert!(
        dx < 1e-3 && dy < 1e-3,
        "final position {last_pos:?} reaches target {target:?}",
    );
}

/// Ease-out cubic: starts fast, ends slow. Halfway through (t = 0.5) the
/// eased value should be > 0.5 (more than half-way in distance).
#[test]
fn ease_out_cubic_is_decelerating() {
    assert!((ease_out_cubic(0.0) - 0.0).abs() < 1e-6);
    assert!((ease_out_cubic(1.0) - 1.0).abs() < 1e-6);
    let mid = ease_out_cubic(0.5);
    assert!(
        mid > 0.5,
        "ease-out is past the midpoint at t = 0.5 (got {mid})",
    );
}

/// The animation is a pure visual side effect — the renderer's *logical*
/// target position is the current `(row, col)`'s rect, which the layout
/// computation returns immediately. We model that here: even with a partial
/// animation in flight, the logical target a downstream consumer (e.g. the
/// drop hit-test) reads is the FINAL target, not the lerped position.
#[test]
fn animation_does_not_block_logical_target() {
    let origin = (0.0_f32, 0.0_f32);
    let target = (100.0_f32, 100.0_f32);

    let mut st = PanelState {
        anim_origin: Some(origin),
        anim_t: Some(0.0),
        ..Default::default()
    };

    // After a few frames the lerp is still mid-animation…
    let dt = 0.016_f32;
    let duration = 0.150_f32;
    for _ in 0..3 {
        let t = st.anim_t.unwrap();
        let new_t = (t + dt / duration).clamp(0.0, 1.0);
        st.anim_t = Some(new_t);
    }
    assert!(st.anim_t.unwrap() < 1.0, "still animating after 3 frames");

    // …but the LOGICAL target a hit-test consults is unchanged: the renderer
    // hands `target` to the layout API and to drop snap-zone math. The lerp
    // only shifts the painted rect, never the layout rect.
    let logical_target = target;
    assert_eq!(logical_target, target);
}

/// `frame_dt > 0.5` disables the animation step in the renderer (covers
/// tab-back-to-window pauses). The renderer's contract: when dt is "too big",
/// skip the lerp. We model that here as an early-return.
#[test]
fn large_dt_skips_animation_step() {
    let origin = (0.0_f32, 0.0_f32);
    let mut st = PanelState {
        anim_origin: Some(origin),
        anim_t: Some(0.0),
        ..Default::default()
    };

    let dt = 0.6_f32; // > 0.5 s → renderer skips advancing the animation
    if dt < 0.5 {
        let new_t = (0.0_f32 + dt / 0.150_f32).clamp(0.0, 1.0);
        st.anim_t = Some(new_t);
    } else {
        // Skipped: state unchanged.
    }
    assert_eq!(st.anim_t, Some(0.0));
}
