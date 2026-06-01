//! v0.10.1 — `+ New blank template…` bootstrap path.
//!
//! The picker's "+ New blank template…" entry calls `Template::blank(name)`
//! to obtain a 1×1, zero-cell template that the operator then populates via
//! "+ Add Panel". These tests lock that contract at the data layer (the
//! actual file-dialog plumbing is in the CLI and exercised manually).

use profiler_template::Template;

#[test]
fn blank_template_is_one_by_one_grid_with_no_cells() {
    let tpl = Template::blank("untitled");
    assert_eq!(tpl.name, "untitled");
    assert!(tpl.cells.is_empty(), "fresh blank template has no cells");
    assert_eq!(tpl.grid.rows, 1, "minimum 1×1 grid");
    assert_eq!(tpl.grid.cols, 1, "minimum 1×1 grid");
    assert!(tpl.view_3d.is_none(), "no 3D block by default");
    assert!(tpl.sections.is_empty());
    assert!(tpl.ui_state.is_none());
}

#[test]
fn blank_template_round_trips_through_json() {
    let tpl = Template::blank("scratch");
    let json = tpl.to_pretty_json().expect("serialise");
    let parsed = Template::from_str(&json).expect("parse");
    assert_eq!(parsed.name, "scratch");
    assert!(parsed.cells.is_empty());
    assert_eq!((parsed.grid.rows, parsed.grid.cols), (1, 1));
}

#[test]
fn blank_template_name_falls_through_from_arg() {
    // Various Into<String> sources should all flow through unchanged.
    let from_string = Template::blank(String::from("from-string"));
    let from_str = Template::blank("from-str");
    assert_eq!(from_string.name, "from-string");
    assert_eq!(from_str.name, "from-str");
}
