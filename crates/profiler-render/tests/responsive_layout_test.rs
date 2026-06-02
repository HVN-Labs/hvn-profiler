//! v0.11.0 — responsive 2D grid layout.
//!
//! The 2D grid reflows when the available width gets cramped so titles, plot
//! data, and label overlays don't overlap when the operator shrinks the window
//! horizontally. Three regimes:
//!
//! - **Wide** (`width / declared_cols >= MIN_CELL_W` AND
//!   `width >= SINGLE_COL_W`): keep the template's declared column count.
//! - **Mid** (`width / declared_cols < MIN_CELL_W` BUT
//!   `width >= SINGLE_COL_W`): halve cols until each cell ≥ MIN_CELL_W.
//! - **Narrow** (`width < SINGLE_COL_W`): fall through to single column, one
//!   visible cell per row, wrapped in a vertical scroll area.
//!
//! These tests pin the layout MATH (no UI required), so a regression that
//! breaks the responsive break-points or re-introduces cell overlap shows up
//! here regardless of which egui version is in use.

use egui::{pos2, vec2, Rect};

use profiler_render::{
    responsive_cell_rects, responsive_grid_dims, RESPONSIVE_3D_COLLAPSE_W, RESPONSIVE_MIN_CELL_W,
    RESPONSIVE_SINGLE_COL_W,
};
use profiler_template::Template;

fn hvn_default() -> Template {
    let b = profiler_template::bundled::by_name("hvn-default")
        .expect("hvn-default bundled template");
    Template::from_str(b.json).expect("parse hvn-default")
}

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
fn responsive_constants_are_sane() {
    // The 3D collapse threshold is wider than the single-column threshold —
    // we want 3D to fall away BEFORE the 2D grid starts halving columns.
    const _: () = assert!(RESPONSIVE_3D_COLLAPSE_W > RESPONSIVE_SINGLE_COL_W);
    // MIN_CELL_W must be > 0 or the wide-branch loop would be infinite.
    const _: () = assert!(RESPONSIVE_MIN_CELL_W > 0.0);
}

#[test]
fn wide_window_keeps_declared_columns() {
    // hvn-default has 7 rows × 3 cols. At 1600 px wide each col gets ~533 px
    // (well above MIN_CELL_W = 240), so we keep all 3 columns.
    let (rows, cols) = responsive_grid_dims(1600.0, 21, 3);
    assert_eq!((rows, cols), (7, 3));

    let tpl = hvn_default();
    let window = Rect::from_min_size(pos2(0.0, 0.0), vec2(1600.0, 900.0));
    let (r, c, rects) = responsive_cell_rects(&tpl, window);
    assert_eq!((r, c), (7, 3));
    assert_eq!(rects.len(), 21);
    assert_no_overlap(&rects);
}

#[test]
fn mid_width_halves_columns() {
    // 800 px / 3 cols = 266 px — JUST above MIN_CELL_W (240), so we keep 3.
    let (_, cols) = responsive_grid_dims(800.0, 21, 3);
    assert_eq!(cols, 3, "800 / 3 = 266 px >= 240");

    // 700 px / 3 cols = 233 < 240 → halve to 1 col (since 3/2 = 1).
    let (_, cols) = responsive_grid_dims(700.0, 21, 3);
    assert!(cols <= 2, "narrow window must reduce cols from 3, got {cols}");

    // A 6-col template at 800 px: 800/6 = 133 < 240, halve to 3: 800/3 = 266 >= 240.
    let (rows, cols) = responsive_grid_dims(800.0, 18, 6);
    assert_eq!(cols, 3);
    assert_eq!(rows, 6); // 18 / 3 = 6

    // No cell overlaps at the boundary.
    let tpl = hvn_default();
    for w in [800.0_f32, 720.0, 640.0, 560.0] {
        let window = Rect::from_min_size(pos2(0.0, 0.0), vec2(w, 900.0));
        let (_, _, rects) = responsive_cell_rects(&tpl, window);
        assert_no_overlap(&rects);
    }
}

#[test]
fn very_narrow_collapses_to_single_column() {
    // 400 px < SINGLE_COL_W (480) → single column with one row per cell.
    const _: () = assert!(400.0 < RESPONSIVE_SINGLE_COL_W);
    let (rows, cols) = responsive_grid_dims(400.0, 21, 3);
    assert_eq!(cols, 1);
    assert_eq!(rows, 21);

    let tpl = hvn_default();
    let window = Rect::from_min_size(pos2(0.0, 0.0), vec2(400.0, 900.0));
    let (r, c, rects) = responsive_cell_rects(&tpl, window);
    assert_eq!(c, 1, "narrow → single column");
    // Visible-cell count for hvn-default = number of cells with visible == true.
    let visible_count = tpl.cells.iter().filter(|cell| cell.visible).count();
    assert_eq!(r, visible_count.max(1));
    assert_eq!(rects.len(), r * c);
    assert_no_overlap(&rects);
}

#[test]
fn cells_do_not_overlap_across_resize_sweep() {
    // Sweep a continuous range of plausible window widths, including the
    // SINGLE_COL_W and MIN_CELL_W boundaries.
    let tpl = hvn_default();
    let sweep: Vec<f32> = (300..=2000).step_by(50).map(|w| w as f32).collect();
    for w in sweep {
        let window = Rect::from_min_size(pos2(0.0, 0.0), vec2(w, 900.0));
        let (_, _, rects) = responsive_cell_rects(&tpl, window);
        assert_no_overlap(&rects);
        // Every rect must fit inside the window (a few pixels of float slop).
        for (i, r) in rects.iter().enumerate() {
            assert!(
                r.min.x >= window.min.x - 0.5,
                "cell {i} left {} escapes window at w={w}",
                r.min.x,
            );
            assert!(
                r.max.x <= window.max.x + 0.5,
                "cell {i} right {} > window right {} at w={w}",
                r.max.x,
                window.max.x,
            );
        }
    }
}

#[test]
fn empty_grid_does_not_panic() {
    // An empty template (rare: every cell hidden, or just freshly created).
    let (rows, cols) = responsive_grid_dims(1200.0, 0, 3);
    assert_eq!((rows, cols), (1, 3));
    let (rows, cols) = responsive_grid_dims(300.0, 0, 3);
    assert_eq!((rows, cols), (1, 1)); // narrow + empty still 1×1
}
