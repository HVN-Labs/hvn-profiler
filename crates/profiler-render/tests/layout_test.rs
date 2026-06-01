//! v0.9.0 — assert the grid-layout pure function produces non-overlapping
//! cell rects across a range of window sizes.
//!
//! The v0.8.0 layout bug was visual cell overlap: nested
//! `ui.vertical { ui.horizontal { allocate_ui } }` did NOT clamp the inner
//! `Plot`'s height, so the plot grew past `cell_h` and the next row painted
//! on top of the previous one. The v0.9.0 fix is rect-based: every cell
//! gets a precomputed `Rect` and the Plot is forced to that height.
//!
//! These tests pin the layout MATH (no UI required), so a regression that
//! reintroduces overlap shows up here regardless of which egui version is
//! in use.

use egui::{pos2, vec2, Rect};

use profiler_render::layout_cell_rects;
use profiler_template::Template;

/// Load the bundled HVN-default template (7×3 grid, 2 sections).
fn hvn_default() -> Template {
    let b = profiler_template::bundled::by_name("hvn-default")
        .expect("hvn-default bundled template");
    Template::from_str(b.json).expect("parse hvn-default")
}

/// Brute-force: for every pair of cells (i < j) assert their rects do not
/// intersect. The grid is 7×3 = 21 cells, so this is 210 comparisons —
/// trivial.
fn assert_no_overlap(rects: &[Rect]) {
    for i in 0..rects.len() {
        for j in (i + 1)..rects.len() {
            let a = rects[i];
            let b = rects[j];
            let inter = a.intersect(b);
            assert!(
                !(inter.width() > 0.5 && inter.height() > 0.5),
                "rects {i} ({a:?}) and {j} ({b:?}) overlap by {inter:?}",
            );
        }
    }
}

#[test]
fn hvn_default_cells_do_not_overlap_at_1600x900() {
    let tpl = hvn_default();
    let window = Rect::from_min_size(pos2(0.0, 0.0), vec2(1600.0, 900.0));
    let rects = layout_cell_rects(&tpl, window);
    assert_eq!(
        rects.len(),
        tpl.grid.rows * tpl.grid.cols,
        "rect count matches grid",
    );
    assert_no_overlap(&rects);
}

#[test]
fn hvn_default_cells_do_not_overlap_across_resize_range() {
    // Sweep a range of plausible window sizes — including small windows
    // where the v0.8.0 layout collapsed first.
    let sizes = [
        (800.0, 600.0),
        (1024.0, 768.0),
        (1280.0, 900.0),
        (1600.0, 900.0),
        (1920.0, 1080.0),
        (2560.0, 1440.0),
    ];
    let tpl = hvn_default();
    for (w, h) in sizes {
        let window = Rect::from_min_size(pos2(0.0, 0.0), vec2(w, h));
        let rects = layout_cell_rects(&tpl, window);
        assert_no_overlap(&rects);
    }
}

#[test]
fn cells_are_inside_the_window_rect() {
    let tpl = hvn_default();
    let window = Rect::from_min_size(pos2(0.0, 0.0), vec2(1280.0, 900.0));
    let rects = layout_cell_rects(&tpl, window);
    for (i, r) in rects.iter().enumerate() {
        assert!(
            r.min.x >= window.min.x - 0.5 && r.min.y >= window.min.y - 0.5,
            "cell {i} top-left {:?} escapes window top-left {:?}",
            r.min,
            window.min,
        );
        assert!(
            r.max.x <= window.max.x + 0.5,
            "cell {i} right edge {} > window right {}",
            r.max.x,
            window.max.x,
        );
    }
}

#[test]
fn cell_count_matches_grid_dimensions() {
    let tpl = hvn_default();
    let window = Rect::from_min_size(pos2(0.0, 0.0), vec2(1600.0, 900.0));
    let rects = layout_cell_rects(&tpl, window);
    // 7 rows × 3 cols = 21 cells, even though some are marked
    // `visible: false` (those still reserve their grid slot).
    assert_eq!(rects.len(), 21);
}

#[test]
fn row_major_order_x_then_y() {
    // The pure-function contract: rects are emitted in row-major order so
    // `rects[r * cols + c]` is the (r, c) slot. Verify by walking the
    // first 3 cells (row 0) and the 4th (row 1, col 0).
    let tpl = hvn_default();
    let window = Rect::from_min_size(pos2(0.0, 0.0), vec2(1500.0, 700.0));
    let rects = layout_cell_rects(&tpl, window);
    let cols = tpl.grid.cols;
    // Within a row, x increases.
    assert!(rects[0].min.x < rects[1].min.x);
    assert!(rects[1].min.x < rects[2].min.x);
    // Next row starts back at the left.
    assert!(rects[cols].min.x < rects[1].min.x);
    // …and is lower on the screen (y grows down in egui).
    assert!(rects[cols].min.y > rects[0].min.y);
}
