//! v0.10.0 — `view_drone` selection persists across template reload.
//!
//! `App::load_template_at` (profiler-cli) captures `view_drone` before the
//! reload and restores it afterwards if the drone is still in the store map
//! (else it falls back to the first-seen drone). The render crate can't
//! reach into `App`, so we model the invariant against the same primitive
//! data structures the CLI keeps: `HashMap<String, TraceStore>` plus a
//! `Vec<String>` of discovered drones.

use std::collections::HashMap;

use profiler_render::TraceStore;

/// Standalone re-impl of the `view_drone` capture-and-restore contract used
/// inside `App::load_template_at`. Returns the new `view_drone` after the
/// reload: keep the captured value if the drone is still known, else fall
/// back to the first-seen drone (or `None` if no drones are known).
fn restore_view_drone_after_reload(
    captured: Option<String>,
    stores: &HashMap<String, TraceStore>,
    discovered: &[String],
) -> Option<String> {
    match captured {
        Some(d) if stores.contains_key(&d) => Some(d),
        _ => discovered.first().cloned(),
    }
}

#[test]
fn reload_preserves_view_drone_when_still_known() {
    // Two drones discovered, operator picked "eric_1" as the view drone.
    let mut stores: HashMap<String, TraceStore> = HashMap::new();
    stores.entry("eric_1".into()).or_default().push(0.0, "x", 1.0);
    stores.entry("eric_2".into()).or_default().push(0.0, "x", 2.0);
    let discovered = vec!["eric_1".to_string(), "eric_2".into()];
    let captured = Some("eric_1".to_string());

    let after = restore_view_drone_after_reload(captured, &stores, &discovered);
    assert_eq!(after.as_deref(), Some("eric_1"), "view_drone persists across reload");
}

#[test]
fn reload_falls_back_when_view_drone_no_longer_in_stores() {
    // Operator had picked "ghost", but it's no longer in the store map.
    let mut stores: HashMap<String, TraceStore> = HashMap::new();
    stores.entry("eric_1".into()).or_default().push(0.0, "x", 1.0);
    let discovered = vec!["eric_1".to_string()];
    let captured = Some("ghost".to_string());

    let after = restore_view_drone_after_reload(captured, &stores, &discovered);
    assert_eq!(after.as_deref(), Some("eric_1"), "fall back to first-seen drone");
}

#[test]
fn reload_with_no_drones_yields_none() {
    let stores: HashMap<String, TraceStore> = HashMap::new();
    let discovered: Vec<String> = Vec::new();
    let after = restore_view_drone_after_reload(Some("eric_1".into()), &stores, &discovered);
    assert!(after.is_none(), "no drones known → view_drone clears");
}

#[test]
fn reload_with_no_prior_selection_picks_first_seen() {
    // App boot-up: no view_drone yet, several drones in the discovery list.
    let mut stores: HashMap<String, TraceStore> = HashMap::new();
    stores.entry("alpha".into()).or_default().push(0.0, "x", 1.0);
    stores.entry("beta".into()).or_default().push(0.0, "x", 2.0);
    let discovered = vec!["alpha".to_string(), "beta".into()];
    let after = restore_view_drone_after_reload(None, &stores, &discovered);
    assert_eq!(after.as_deref(), Some("alpha"));
}

#[test]
fn reload_keeps_eric_1_across_template_switch() {
    // The flow the spec called out explicitly: load template A with
    // "eric_1" selected, switch to template B, "eric_1" must still be
    // selected. The view-drone map doesn't depend on which template is
    // loaded (templates describe the visualisation, not the data), so the
    // contract is trivial — but pin it as a regression guard.
    let mut stores: HashMap<String, TraceStore> = HashMap::new();
    stores.entry("eric_1".into()).or_default().push(0.0, "x", 1.0);
    stores.entry("eric_2".into()).or_default().push(0.0, "x", 2.0);
    let discovered = vec!["eric_1".to_string(), "eric_2".into()];

    // Operator selects eric_1 on template A.
    let mut view_drone = Some("eric_1".to_string());
    // Switch to template B → run the capture-and-restore step.
    view_drone = restore_view_drone_after_reload(view_drone.clone(), &stores, &discovered);
    assert_eq!(view_drone.as_deref(), Some("eric_1"));
    // And again to template C — still pinned to eric_1.
    view_drone = restore_view_drone_after_reload(view_drone.clone(), &stores, &discovered);
    assert_eq!(view_drone.as_deref(), Some("eric_1"));
}
