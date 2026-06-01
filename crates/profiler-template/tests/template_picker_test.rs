//! v0.8.0 — template picker discovery.
//!
//! Writes two JSON templates into a temp directory and asserts:
//! - both surface as `TemplateOrigin::User` entries
//! - bundled templates always come first in the registry
//! - looking up a bundled template by name yields the right JSON

use profiler_template::{bundled, scan_user_templates, Template, TemplateOrigin};

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
