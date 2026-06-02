//! v0.11.0 — the grouped source-key dropdown stays open when the operator
//! collapses / expands a category.
//!
//! The v0.10.2 implementation used `egui::CollapsingHeader` inside a
//! `ComboBox` popup, which dismissed the entire popup on every ▶/▼ click
//! because the header's click interaction escaped the popup-rect tracking.
//! v0.11.0 replaces it with a manual toggle backed by
//! [`ComboCollapseState`], whose `toggle` does NOT interact with egui at
//! all — only flips a `HashMap<&'static str, bool>` entry. The renderer
//! reads `is_collapsed` next frame and skips the section body.
//!
//! Full popup-still-open verification requires a headless egui context;
//! we instead verify the observable contracts that drive the UI:
//!
//! 1. Toggling a category flips its state (collapsed ↔ expanded) and
//!    leaves OTHER categories untouched (so items in another section keep
//!    rendering — i.e. the popup stays usable).
//! 2. Default state is "all expanded" so first-open looks like v0.10.2.
//! 3. Toggle is independent across instances — one editor's collapse does
//!    not leak into a parallel editor.

use profiler_render::{group_source_keys, ComboCollapseState, KEY_GROUPS};

/// Default state: every category is expanded — matches v0.10.2 behaviour.
#[test]
fn default_state_has_every_category_expanded() {
    let state = ComboCollapseState::default();
    for &group in KEY_GROUPS {
        assert!(
            !state.is_collapsed(group),
            "{group} should default to expanded"
        );
    }
}

/// A toggle flips the named category and ONLY that category. The renderer
/// keeps rendering the other categories' section bodies, so the popup
/// remains usable after the click — i.e. the popup did NOT close.
#[test]
fn toggle_only_affects_named_category_other_sections_still_render() {
    let mut state = ComboCollapseState::default();
    assert!(!state.is_collapsed("DT physics"));
    assert!(!state.is_collapsed("AP MAVLink"));

    // Operator clicks the ▼ next to "DT physics" — section collapses.
    let new_state = state.toggle("DT physics");
    assert!(new_state, "toggle returns the new (collapsed) state");
    assert!(state.is_collapsed("DT physics"));

    // Other categories are untouched. The renderer will still iterate their
    // section bodies on the next frame, so e.g. `ap_attitude[0]` remains
    // visible in the popup — i.e. the popup stayed open.
    for &group in KEY_GROUPS {
        if group == "DT physics" {
            continue;
        }
        assert!(
            !state.is_collapsed(group),
            "{group} must remain expanded after toggling DT physics",
        );
    }
}

/// A second toggle on the same category restores the expanded state.
#[test]
fn double_toggle_restores_expanded() {
    let mut state = ComboCollapseState::default();
    let after_first = state.toggle("AP MAVLink");
    let after_second = state.toggle("AP MAVLink");
    assert!(after_first);
    assert!(!after_second);
    assert!(!state.is_collapsed("AP MAVLink"));
}

/// Sanity check that the section-rendering loop (driven by
/// `group_source_keys` + `is_collapsed`) would skip ONLY the collapsed
/// section. This is the observable invariant for "popup still open".
#[test]
fn rendering_loop_skips_collapsed_section_only() {
    let keys: Vec<String> = vec![
        "accel[0]".into(),
        "ap_attitude[0]".into(),
        "pos_truth_ned[0]".into(),
        "t".into(),
    ];
    let mut state = ComboCollapseState::default();
    state.toggle("AP MAVLink");

    // Simulate the dropdown's per-frame render loop. Only render section
    // bodies for non-collapsed categories.
    let mut rendered_items: Vec<String> = Vec::new();
    for (group, group_keys) in group_source_keys(&keys) {
        if state.is_collapsed(group) {
            continue;
        }
        rendered_items.extend(group_keys);
    }

    // The other categories' items must still be present — proving the
    // popup is still usable (i.e. it didn't close on the toggle click).
    assert!(rendered_items.contains(&"accel[0]".to_string()));
    assert!(rendered_items.contains(&"pos_truth_ned[0]".to_string()));
    assert!(rendered_items.contains(&"t".to_string()));
    // Only the collapsed section's items are absent.
    assert!(!rendered_items.contains(&"ap_attitude[0]".to_string()));
}

/// Independent editor instances keep their own collapse state.
#[test]
fn separate_instances_do_not_share_state() {
    let mut a = ComboCollapseState::default();
    let mut b = ComboCollapseState::default();
    a.toggle("DT physics");
    assert!(a.is_collapsed("DT physics"));
    assert!(
        !b.is_collapsed("DT physics"),
        "the other editor's state is unaffected",
    );
    b.toggle("DT physics");
    a.toggle("DT physics");
    assert!(!a.is_collapsed("DT physics"));
    assert!(b.is_collapsed("DT physics"));
}
