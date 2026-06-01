//! v0.9.0 integration test — non-reflowing label overlay.
//!
//! Asserts:
//! 1. `build_label_text` returns `None` when the resolved label mode is `Off`
//!    (so the overlay short-circuits before any painter work).
//! 2. The overlay's screen-space box size depends ONLY on the text content,
//!    not on the plot rect — proving that toggling labels on/off cannot
//!    change the plot's allocated rect. This pairs with the v0.9.0 layout
//!    fix in `panels.rs::render_cell`, which paints the overlay AFTER the
//!    plot via the parent ui's painter (so the plot layout is already
//!    committed by the time the label is drawn).
//! 3. End-to-end: rendering the same template twice (once with
//!    `LabelOverride::Force(Off)`, once with `Force(Data)`) produces the
//!    same `ui.min_rect()` for the grid — i.e. labels do NOT reflow.
//!
//! Visual / snapshot tests aren't practical for egui in headless CI; these
//! structural checks are the load-bearing v0.9.0 invariant.

use egui::{Pos2, RawInput, Rect, Vec2};
use profiler_render::{
    build_label_text, compute_overlay_pos, overlay_box_size,
    render_template_grid_with_override, LabelOverride, TraceStore,
};
use profiler_template::{LabelMode, Template};

const TEMPLATE_JSON: &str = r#"{
  "name": "label-overlay-fixture",
  "grid": {"rows": 1, "cols": 1},
  "cells": [
    {
      "row": 0, "col": 0,
      "title": "Roll",
      "primitive": "scalar",
      "sources": [{"key": "roll"}],
      "label_mode": "data",
      "label_data": {"format": "{:+.2f}°", "show_min_max": false}
    }
  ]
}"#;

fn parse_template() -> Template {
    Template::from_str(TEMPLATE_JSON).expect("parse")
}

fn store_with_one_point() -> TraceStore {
    let mut s = TraceStore::new(60.0);
    s.push(0.0, "roll", 0.42);
    s
}

#[test]
fn build_label_text_returns_none_when_resolved_off() {
    let tpl = parse_template();
    let store = store_with_one_point();
    let cell = &tpl.cells[0];

    // Cell's own mode is Data, but the global override forces it Off.
    let force_off = LabelOverride::Force(LabelMode::Off);
    assert!(
        build_label_text(cell, &store, force_off).is_none(),
        "Force(Off) must suppress every cell's overlay",
    );

    // Respect mode honours the cell's `data` setting → returns text.
    let resp = build_label_text(cell, &store, LabelOverride::Respect);
    assert!(resp.is_some(), "Respect honours cell's `data` mode");
    let (text, _color) = resp.unwrap();
    assert!(text.contains("0.42") || text.contains("+0.42"), "got: {text:?}");
}

#[test]
fn overlay_box_size_depends_on_text_only() {
    // Build an egui context so we can resolve text styles.
    let ctx = egui::Context::default();
    let _ = ctx.run_ui(RawInput::default(), |ui| {
        let empty = overlay_box_size(ui, "");
        assert_eq!(empty, Vec2::ZERO, "empty text → zero footprint");

        let short = overlay_box_size(ui, "x");
        let long = overlay_box_size(ui, "this is a much longer label string");
        assert!(short.x > 0.0 && short.y > 0.0, "short label has positive size");
        assert!(long.x > short.x, "long label is wider than short label");
        // Single-line labels have the same height regardless of width.
        assert!(
            (long.y - short.y).abs() < 0.5,
            "single-line height should match: short={} long={}",
            short.y,
            long.y,
        );
    });
}

/// v0.10.1 — pin the label overlay anchor to the top-LEFT of the plot rect.
/// Before v0.10.1 the overlay was top-right, which continuously occluded the
/// live trace tip on a rolling X window. The pure `compute_overlay_pos`
/// helper is the load-bearing primitive — this test guards against regression
/// to a right-anchored position.
#[test]
fn label_overlay_anchors_top_left() {
    let plot_rect = Rect::from_min_max(Pos2::new(100.0, 50.0), Pos2::new(400.0, 250.0));
    let text_size = Vec2::new(80.0, 14.0);
    let pos = compute_overlay_pos(plot_rect, text_size);

    // Top-left: anchor sits in the upper-LEFT quadrant of the plot rect.
    assert!(
        pos.x < plot_rect.center().x,
        "overlay x={} must be left of plot center {}",
        pos.x,
        plot_rect.center().x,
    );
    assert!(
        pos.y < plot_rect.center().y,
        "overlay y={} must be above plot center {}",
        pos.y,
        plot_rect.center().y,
    );

    // And specifically: NOT the old top-right anchor (right - text.x - PAD, ...).
    let old_right_anchor_x = plot_rect.right() - text_size.x - 6.0;
    assert!(
        pos.x < old_right_anchor_x,
        "overlay x={} must NOT be the old right-anchored value {}",
        pos.x,
        old_right_anchor_x,
    );

    // Sanity: a small inset (OVERLAY_PAD = 6.0) keeps it inside the rect.
    assert!(pos.x >= plot_rect.left());
    assert!(pos.y >= plot_rect.top());
    assert!((pos.x - plot_rect.left()).abs() < 16.0);
    assert!((pos.y - plot_rect.top()).abs() < 16.0);
}

/// End-to-end: rendering the grid with labels-off and labels-forced-on must
/// produce IDENTICAL `ui.min_rect()` footprints. This is the v0.9.0 guarantee.
#[test]
fn labels_toggle_does_not_reshape_grid_min_rect() {
    let tpl = parse_template();
    let store = store_with_one_point();

    fn render_once(tpl: &Template, store: &TraceStore, ov: LabelOverride) -> Rect {
        let ctx = egui::Context::default();
        let mut captured = Rect::NOTHING;
        let _ = ctx.run_ui(RawInput::default(), |ui| {
            let before = ui.min_rect();
            let _stats = render_template_grid_with_override(ui, tpl, store, ov);
            let after = ui.min_rect();
            // Capture the delta — the rectangle the grid consumed.
            captured = Rect::from_min_size(
                before.right_bottom(),
                Vec2::new(after.width() - before.width(), after.height() - before.height()),
            );
        });
        captured
    }

    let rect_off = render_once(&tpl, &store, LabelOverride::Force(LabelMode::Off));
    let rect_on = render_once(&tpl, &store, LabelOverride::Force(LabelMode::Data));

    // The grid's allocated footprint must be byte-identical between the two
    // runs: same width, same height. If toggling labels reshaped the layout,
    // these would differ.
    assert!(
        (rect_off.width() - rect_on.width()).abs() < 0.5,
        "labels changed grid width: off={} on={}",
        rect_off.width(),
        rect_on.width(),
    );
    assert!(
        (rect_off.height() - rect_on.height()).abs() < 0.5,
        "labels changed grid height: off={} on={}",
        rect_off.height(),
        rect_on.height(),
    );
}
