//! v0.9.0 — per-drone `TraceStore` isolation tests.
//!
//! The v0.9.0 model: the CLI keeps `HashMap<drone_name, TraceStore>` and
//! routes each incoming `Sample` to `stores[sample.drone_name]`. Each drone
//! ends up with its OWN ring buffer, so cross-drone keys (`accel[0]` on
//! drone-A vs drone-B) don't trample each other.
//!
//! These tests exercise that routing at the data-structure level. The
//! `App.drain()` method lives in `profiler-cli` and isn't easily callable
//! from a render-crate integration test, so we re-implement the small
//! "route by drone_name" loop here against the same `TraceStore` primitive.

use std::collections::HashMap;

use profiler_render::TraceStore;

/// Standalone re-impl of `App::drain`'s routing logic — what we'd test if
/// `App` were callable from here. Pushes `(ts, key, value)` into
/// `stores[drone]`.
fn route(stores: &mut HashMap<String, TraceStore>, drone: &str, ts: f64, key: &str, value: f64) {
    stores
        .entry(drone.to_string())
        .or_default()
        .push(ts, key, value);
}

#[test]
fn samples_route_into_drone_specific_stores() {
    let mut stores: HashMap<String, TraceStore> = HashMap::new();

    // Two drones, same key — each must end up in its own ring.
    route(&mut stores, "eric_1", 0.0, "accel[0]", 1.0);
    route(&mut stores, "eric_2", 0.0, "accel[0]", 100.0);
    route(&mut stores, "eric_1", 0.1, "accel[0]", 2.0);
    route(&mut stores, "eric_2", 0.1, "accel[0]", 200.0);

    let a = stores.get("eric_1").expect("eric_1 store");
    let b = stores.get("eric_2").expect("eric_2 store");

    // Each store sees ONLY its own drone's values.
    assert_eq!(a.points("accel[0]"), vec![[0.0, 1.0], [0.1, 2.0]]);
    assert_eq!(b.points("accel[0]"), vec![[0.0, 100.0], [0.1, 200.0]]);

    // And neither store has been polluted with the other drone's values.
    assert!(
        !a.points("accel[0]").iter().any(|p| p[1] >= 100.0),
        "eric_1 store leaked eric_2 sample",
    );
    assert!(
        !b.points("accel[0]").iter().any(|p| p[1] < 10.0),
        "eric_2 store leaked eric_1 sample",
    );
}

#[test]
fn unnamed_drone_falls_into_dedicated_bucket() {
    // CLI uses `"(unnamed)"` as the key for samples whose envelope had
    // `drone_name == None`. Independent of any named drone.
    let mut stores: HashMap<String, TraceStore> = HashMap::new();
    route(&mut stores, "(unnamed)", 0.0, "x", 1.0);
    route(&mut stores, "eric_1", 0.0, "x", 99.0);
    route(&mut stores, "(unnamed)", 0.5, "x", 2.0);

    assert_eq!(
        stores["(unnamed)"].points("x"),
        vec![[0.0, 1.0], [0.5, 2.0]],
    );
    assert_eq!(stores["eric_1"].points("x"), vec![[0.0, 99.0]]);
}

#[test]
fn switching_view_drone_changes_data_seen_by_renderer() {
    // Simulate the CLI's `view_drone` switch — both stores receive their
    // own data, then the renderer reads one or the other.
    let mut stores: HashMap<String, TraceStore> = HashMap::new();
    for i in 0..10 {
        let t = i as f64 * 0.1;
        route(&mut stores, "alpha", t, "altitude", 50.0 + t);
        route(&mut stores, "beta", t, "altitude", 200.0 + t);
    }
    let view = "alpha";
    let alpha = stores.get(view).unwrap();
    assert!(alpha.points("altitude").iter().all(|p| p[1] < 100.0));
    let view = "beta";
    let beta = stores.get(view).unwrap();
    assert!(beta.points("altitude").iter().all(|p| p[1] > 150.0));
}

#[test]
fn discovered_drones_list_preserves_first_seen_order() {
    // Mirrors the CLI's `App::drain` bookkeeping: the discovered_drones
    // Vec gets a name appended on first arrival, never reordered.
    let mut stores: HashMap<String, TraceStore> = HashMap::new();
    let mut discovered: Vec<String> = Vec::new();
    let order = [
        ("gamma", 0.0_f64),
        ("alpha", 0.1),
        ("gamma", 0.2),  // already seen
        ("beta", 0.3),
        ("alpha", 0.4),  // already seen
    ];
    for (name, ts) in order {
        let is_new = !stores.contains_key(name);
        route(&mut stores, name, ts, "x", 1.0);
        if is_new {
            discovered.push(name.to_string());
        }
    }
    assert_eq!(
        discovered,
        vec!["gamma".to_string(), "alpha".into(), "beta".into()],
        "first-seen order preserved",
    );
    assert_eq!(stores.len(), 3);
}

#[test]
fn empty_store_falls_back_gracefully_for_unknown_drone() {
    // `App::view_store` returns an empty store for an as-yet-unknown
    // drone; the renderers must handle that without crashing.
    let stores: HashMap<String, TraceStore> = HashMap::new();
    let empty = TraceStore::default();
    let s = stores.get("unknown").unwrap_or(&empty);
    assert!(s.is_empty());
    assert_eq!(s.points("accel[0]"), Vec::<[f64; 2]>::new());
    assert_eq!(s.latest_ts(), f64::NEG_INFINITY);
}
