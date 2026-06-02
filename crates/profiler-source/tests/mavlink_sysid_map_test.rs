//! v0.16.4 — `--drone-map` resolution.
//!
//! Exercises [`resolve_mavlink_drone_name`] which is what the recv worker
//! uses to label every emitted Sample. The contract:
//!
//! - Map non-empty + sysid present → mapped name wins.
//! - Map non-empty + sysid absent → `drone_{sysid}` fallback (mapped sysids
//!   form an allowlist of labels, not a filter).
//! - Map empty + override set → override wins (pre-v0.16.4 shortcut).
//! - Map empty + no override → `drone_{sysid}` fallback.

#![cfg(feature = "mavlink-source")]

use std::collections::HashMap;

use profiler_source::mavlink_source::resolve_mavlink_drone_name;

#[test]
fn map_hit_overrides_fallback() {
    let mut map = HashMap::new();
    map.insert(1u8, "alpha".to_string());
    map.insert(2u8, "beta".to_string());
    assert_eq!(&*resolve_mavlink_drone_name(1, &map, None), "alpha");
    assert_eq!(&*resolve_mavlink_drone_name(2, &map, None), "beta");
}

#[test]
fn map_miss_falls_back_to_drone_underscore_sysid() {
    let mut map = HashMap::new();
    map.insert(1u8, "alpha".to_string());
    map.insert(2u8, "beta".to_string());
    // sysid 99 isn't in the map → fallback per the v0.16.4 convention.
    assert_eq!(&*resolve_mavlink_drone_name(99, &map, None), "drone_99");
}

#[test]
fn non_empty_map_ignores_single_string_override() {
    let mut map = HashMap::new();
    map.insert(1u8, "alpha".to_string());
    // The single-string `--drone NAME` is the pre-v0.10.0 shortcut; when
    // `--drone-map` is non-empty, the table wins.
    assert_eq!(
        &*resolve_mavlink_drone_name(1, &map, Some("ignored")),
        "alpha",
    );
    // Unmapped sysid → fallback STILL wins over the override (the map's
    // non-emptiness pins the resolution policy as "table-driven").
    assert_eq!(
        &*resolve_mavlink_drone_name(7, &map, Some("ignored")),
        "drone_7",
    );
}

#[test]
fn empty_map_with_override_uses_override() {
    let map: HashMap<u8, String> = HashMap::new();
    // Pre-v0.16.4 shortcut for single-vehicle setups still works.
    assert_eq!(
        &*resolve_mavlink_drone_name(42, &map, Some("eric_solo")),
        "eric_solo",
    );
}

#[test]
fn empty_map_no_override_falls_back() {
    let map: HashMap<u8, String> = HashMap::new();
    assert_eq!(&*resolve_mavlink_drone_name(3, &map, None), "drone_3");
}

#[test]
fn map_handles_sitl_style_eric_naming() {
    // The HVN-SITL convention: sysid N → "eric_N". This test pins the
    // exact string identity so a regression in `--drone-map` parsing
    // doesn't silently produce e.g. "eric1" (no underscore) which would
    // break the picker's source merge.
    let mut map = HashMap::new();
    for sysid in 1u8..=5 {
        map.insert(sysid, format!("eric_{sysid}"));
    }
    for sysid in 1u8..=5 {
        let name = resolve_mavlink_drone_name(sysid, &map, None);
        assert_eq!(&*name, format!("eric_{sysid}").as_str());
    }
}
