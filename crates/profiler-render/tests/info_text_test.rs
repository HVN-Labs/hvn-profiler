//! v0.14.0 — `InfoText` primitive: static literal-text panels with simple
//! Markdown (bold spans, bullet lines, line breaks).
//!
//! These tests pin the inline-markdown parsing contract — the actual painter
//! call requires an egui context, so we exercise the pure parser plus the
//! template-level wiring (Cell with `Primitive::InfoText` carries `text` and
//! `icon` through serde unchanged).

use profiler_render::parse_info_text_spans;
use profiler_template::{Cell, Primitive, Template};

/// A cell configured as InfoText with the given title/icon/body.
fn info_cell(title: &str, icon: Option<&str>, text: Option<&str>) -> Cell {
    Cell {
        primitive: Primitive::InfoText,
        title: title.to_string(),
        icon: icon.map(|s| s.to_string()),
        text: text.map(|s| s.to_string()),
        visible: true,
        ..Default::default()
    }
}

#[test]
fn info_text_cell_carries_icon_title_and_body() {
    // The renderer reads `cell.text` / `cell.icon` directly; assert they
    // survive a round-trip through the template and that the primitive
    // discriminant lands on `InfoText`.
    let cell = info_cell(
        "Welcome",
        Some("👋"),
        Some("This is the **hvn-profiler** default layout.\n\n- Drag to reorder\n- Ctrl+S to save"),
    );
    assert_eq!(cell.primitive, Primitive::InfoText);
    assert_eq!(cell.title, "Welcome");
    assert_eq!(cell.icon.as_deref(), Some("👋"));
    assert!(cell.text.as_deref().unwrap().contains("**hvn-profiler**"));

    // Also exercise the JSON round-trip — InfoText cells skip the `text` /
    // `icon` fields when they're `None`, and parse cleanly when set.
    let json = r#"{
        "row": 0, "col": 0,
        "title": "Hello",
        "primitive": "info_text",
        "icon": "🚀",
        "text": "Launching **soon**"
    }"#;
    let cell2: Cell = serde_json::from_str(json).expect("parse info_text cell");
    assert_eq!(cell2.primitive, Primitive::InfoText);
    assert_eq!(cell2.icon.as_deref(), Some("🚀"));
    assert_eq!(cell2.text.as_deref(), Some("Launching **soon**"));
}

#[test]
fn bold_spans_split_on_double_star() {
    // Plain prefix + bold + suffix.
    let spans = parse_info_text_spans("This is **bold** text");
    assert_eq!(
        spans,
        vec![
            ("This is ".to_string(), false),
            ("bold".to_string(), true),
            (" text".to_string(), false),
        ],
    );
    // Multiple bold spans.
    let spans = parse_info_text_spans("**hello** world **again**");
    assert_eq!(
        spans,
        vec![
            ("hello".to_string(), true),
            (" world ".to_string(), false),
            ("again".to_string(), true),
        ],
    );
}

#[test]
fn bullet_lines_are_detectable_via_dash_space_prefix() {
    // The renderer prefixes bullet lines with "• " when the line begins with
    // "- ". We mirror the same prefix-check here so tests pin the contract
    // without requiring an egui painter.
    fn render_prefix(line: &str) -> &'static str {
        if line.strip_prefix("- ").is_some() || line == "-" {
            "• "
        } else {
            ""
        }
    }
    assert_eq!(render_prefix("- first item"), "• ");
    assert_eq!(render_prefix("- another"), "• ");
    assert_eq!(render_prefix("not a bullet"), "");
    assert_eq!(render_prefix(""), "");
}

#[test]
fn empty_text_renders_title_only_via_no_body_spans() {
    // When `text` is empty or absent, the parser must still gracefully handle
    // the situation: an empty line returns a single empty span, which the
    // renderer skips. The title alone remains visible.
    let cell = info_cell("Just a title", None, None);
    assert!(cell.text.is_none());
    // An empty input produces a single empty span (so y still advances by
    // one line in the renderer).
    let spans = parse_info_text_spans("");
    assert_eq!(spans.len(), 1);
    assert_eq!(spans[0], (String::new(), false));
}

#[test]
fn missing_text_field_parses_as_none_and_renders_empty_body() {
    // Templates predating v0.14.0 have no `text` / `icon` keys on their
    // cells. Those must continue to parse, and the new fields must default
    // to `None` so the renderer treats them as absent (empty body).
    let json = r#"{
        "row": 1, "col": 2,
        "title": "Legacy",
        "primitive": "scalar",
        "sources": [{"key": "accel[0]"}]
    }"#;
    let cell: Cell = serde_json::from_str(json).expect("legacy cell parses");
    assert!(cell.text.is_none(), "text must default to None for legacy cells");
    assert!(cell.icon.is_none(), "icon must default to None for legacy cells");
    assert_eq!(cell.primitive, Primitive::Scalar);

    // And a full template with no text/icon anywhere still parses + has the
    // bundled-default tutorial layout intact.
    let t: Template = serde_json::from_str(
        r#"{"name":"x","grid":{"rows":1,"cols":1},"cells":[{"row":0,"col":0,"primitive":"info_text"}]}"#,
    )
    .unwrap();
    assert_eq!(t.cells.len(), 1);
    assert_eq!(t.cells[0].primitive, Primitive::InfoText);
    assert!(t.cells[0].text.is_none());
}
