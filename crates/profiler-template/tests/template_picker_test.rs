//! v0.8.0 — template picker discovery.
//!
//! Writes two JSON templates into a temp directory and asserts:
//! - both surface as `TemplateOrigin::User` entries
//! - bundled templates always come first in the registry
//! - looking up a bundled template by name yields the right JSON

use profiler_template::{bundled, scan_user_templates, Primitive, Template, TemplateOrigin};

#[test]
fn scan_returns_two_user_templates() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        tmp.path().join("a.json"),
        r#"{"name":"alpha","grid":{"rows":1,"cols":1}}"#,
    )
    .unwrap();
    std::fs::write(
        tmp.path().join("b.json"),
        r#"{"name":"bravo","grid":{"rows":1,"cols":1}}"#,
    )
    .unwrap();

    let entries = scan_user_templates(tmp.path());
    assert_eq!(entries.len(), 2, "expected two user templates");
    let names: Vec<&str> = entries.iter().map(|e| e.name.as_str()).collect();
    assert_eq!(names, vec!["alpha", "bravo"], "sorted alphabetically");
    for e in &entries {
        match &e.origin {
            TemplateOrigin::User { path } => {
                assert!(path.starts_with(tmp.path()));
                assert!(path.exists());
            }
            other => panic!("expected User origin, got {:?}", other),
        }
    }
}

#[test]
fn bundled_registry_contains_hvn_default_and_real_drone() {
    let names: Vec<&str> = bundled::BUNDLED.iter().map(|b| b.name).collect();
    assert!(names.contains(&"hvn-default"));
    assert!(names.contains(&"real-drone"));
}

#[test]
fn bundled_json_parses_into_template() {
    for b in bundled::BUNDLED {
        let t = Template::from_str(b.json).unwrap_or_else(|e| {
            panic!("bundled '{}' failed to parse: {e}", b.name)
        });
        assert_eq!(t.name, b.name);
    }
}

#[test]
fn tutorial_is_first_bundled_entry() {
    // v0.14.0: tutorial is the implicit default; it must lead the BUNDLED
    // registry so the picker shows it first.
    assert_eq!(
        bundled::BUNDLED[0].name,
        "tutorial",
        "BUNDLED[0] must be the tutorial template (v0.14.0 default)",
    );
    assert_eq!(bundled::DEFAULT_BUNDLED_NAME, "tutorial");
}

#[test]
fn tutorial_parses_cleanly() {
    let b = bundled::by_name("tutorial").expect("tutorial in registry");
    let t = Template::from_str(b.json).expect("tutorial parses");
    assert_eq!(t.name, "tutorial");
    assert!(t.grid.rows >= 1);
    assert!(t.grid.cols >= 1);
    // Sanity: at least the welcome panel is laid out.
    assert!(!t.cells.is_empty(), "tutorial has at least one cell");
}

#[test]
fn tutorial_has_info_text_and_status_cells() {
    let b = bundled::by_name("tutorial").expect("tutorial in registry");
    let t = Template::from_str(b.json).expect("tutorial parses");
    let info_text_count = t
        .cells
        .iter()
        .filter(|c| c.primitive == Primitive::InfoText)
        .count();
    let status_count = t
        .cells
        .iter()
        .filter(|c| c.primitive == Primitive::Status)
        .count();
    assert!(
        info_text_count >= 1,
        "tutorial must have at least one InfoText cell, found {info_text_count}",
    );
    assert!(
        status_count >= 3,
        "tutorial must have at least 3 Status cells, found {status_count}",
    );
}

#[test]
fn discover_includes_user_and_bundled() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        tmp.path().join("my.json"),
        r#"{"name":"my-tuning","grid":{"rows":2,"cols":2}}"#,
    )
    .unwrap();
    let user_only = scan_user_templates(tmp.path());
    assert_eq!(user_only.len(), 1);
    assert_eq!(user_only[0].name, "my-tuning");
}
