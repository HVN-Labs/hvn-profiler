//! v0.15.0 — template source-URI fallback resolution tests.
//!
//! When a cell pins itself to a `source_uri` that isn't currently
//! connected, the render layer falls back to the first available source
//! (same behaviour as the `(any)` default) and surfaces a toolbar warning
//! so the operator notices.
//!
//! This test pins the resolution function's contract; the toolbar wiring
//! that paints the `⚠` indicator is exercised by the CLI manually (it
//! reads the same `resolve_source_uri` helper from a frame's render path).

use profiler_render::{resolve_source_uri, ResolvedSource};

#[test]
fn declared_none_uses_first_available() {
    // `(any)` default: no warning, pick the first connected source.
    let connected = vec![
        "zmq://127.0.0.1:9005".to_string(),
        "zmq://127.0.0.1:9006".to_string(),
    ];
    let resolved = resolve_source_uri(None, &connected);
    assert_eq!(
        resolved,
        ResolvedSource {
            uri: Some("zmq://127.0.0.1:9005".to_string()),
            fallback_applied: false,
        },
    );
}

#[test]
fn declared_matching_uri_is_honoured() {
    // Operator pinned to a specific URI that IS connected — use it exactly.
    let connected = vec![
        "zmq://127.0.0.1:9005".to_string(),
        "zmq://127.0.0.1:9006".to_string(),
    ];
    let resolved = resolve_source_uri(Some("zmq://127.0.0.1:9006"), &connected);
    assert_eq!(
        resolved,
        ResolvedSource {
            uri: Some("zmq://127.0.0.1:9006".to_string()),
            fallback_applied: false,
        },
    );
}

#[test]
fn declared_unknown_uri_falls_back_with_warning_flag() {
    // The scope's headline test: template references `zmq://nonexistent:9999`
    // which isn't connected. Resolution falls back to the first available
    // source AND flips `fallback_applied = true` so the toolbar can paint
    // the warning chip.
    let connected = vec![
        "zmq://127.0.0.1:9005".to_string(),
    ];
    let resolved = resolve_source_uri(Some("zmq://nonexistent:9999"), &connected);
    assert_eq!(
        resolved,
        ResolvedSource {
            uri: Some("zmq://127.0.0.1:9005".to_string()),
            fallback_applied: true,
        },
        "missing pinned URI falls back to first available and flags the warning",
    );
}

#[test]
fn declared_unknown_uri_no_sources_returns_none() {
    // No sources at all: resolution returns `uri = None`. The renderer will
    // paint the "waiting for data..." placeholder.
    let resolved = resolve_source_uri(Some("zmq://nonexistent:9999"), &[]);
    assert_eq!(
        resolved,
        ResolvedSource {
            uri: None,
            fallback_applied: true,
        },
    );
}

#[test]
fn declared_none_no_sources_returns_none_without_warning() {
    // `(any)` + no connected sources: returns `uri = None`, but the warning
    // is suppressed — the operator didn't ask for a specific source.
    let resolved = resolve_source_uri(None, &[]);
    assert_eq!(
        resolved,
        ResolvedSource {
            uri: None,
            fallback_applied: false,
        },
    );
}

#[test]
fn fallback_picks_first_in_insertion_order() {
    // The fallback uses the first URI in the connected list (insertion
    // order). The CLI feeds `registry.uris()` here, which preserves the
    // order CLI flags / Add Source dialog actions happened in.
    let connected = vec![
        "mavlink://0.0.0.0:14550".to_string(),
        "zmq://127.0.0.1:9005".to_string(),
        "zmq://127.0.0.1:9006".to_string(),
    ];
    let resolved = resolve_source_uri(Some("zmq://nope:0"), &connected);
    assert_eq!(
        resolved.uri.as_deref(),
        Some("mavlink://0.0.0.0:14550"),
        "first connected URI wins the fallback",
    );
    assert!(resolved.fallback_applied);
}
