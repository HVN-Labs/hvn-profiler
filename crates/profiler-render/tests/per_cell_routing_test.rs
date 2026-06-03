//! v0.16.2 — per-cell store routing tests.
//!
//! `CellSource::source_uri` (v0.15.0) was being SAVED but the renderer
//! ignored it: every cell read from the single view-drone `TraceStore`. The
//! symptom: a cell pinned to drone-A's `pos_ekf_ned` rendered drone-B's
//! data when drone-B was the active toolbar selection.
//!
//! These tests exercise [`StoresView`]'s resolution rules at the pure-data
//! layer (no `egui` / `eframe` required, so the tests run headlessly in CI).
//! The actual paint pipeline (`render_template_grid_multi`) needs a live
//! `egui::Ui` and is exercised by hand via the binary.
//!
//! Rules (per the v0.16.2 brief):
//! 1. Cell-source `source_uri = Some(uri)` AND `uri_to_drone[uri]` is a
//!    known drone → return that drone's store.
//! 2. Otherwise → return the view-drone's store.
//! 3. Otherwise → return the empty fallback store.

use std::collections::HashMap;

use profiler_render::{StoresView, TraceStore};
use profiler_template::CellSource;

fn make_store(seed: f64) -> TraceStore {
    let mut s = TraceStore::default();
    // Seed with `ap_attitude[0] = seed` so every test can pull a sentinel
    // value through the resolver and confirm WHICH store it came from.
    s.push(0.0, "ap_attitude[0]", seed);
    s
}

/// Build a `StoresView` for the canonical two-drone setup the brief asks for.
/// Drone-A: `ap_attitude[0] = 1.0`. Drone-B: `ap_attitude[0] = 2.0`.
fn two_drone_setup() -> (HashMap<String, TraceStore>, HashMap<String, String>, TraceStore) {
    let mut stores: HashMap<String, TraceStore> = HashMap::new();
    stores.insert("A".to_string(), make_store(1.0));
    stores.insert("B".to_string(), make_store(2.0));
    let mut uri_to_drone: HashMap<String, String> = HashMap::new();
    uri_to_drone.insert("zmq://127.0.0.1:9005".to_string(), "A".to_string());
    uri_to_drone.insert("zmq://127.0.0.1:9006".to_string(), "B".to_string());
    let empty = TraceStore::default();
    (stores, uri_to_drone, empty)
}

#[test]
fn pinned_source_uri_resolves_to_other_drone_store() {
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let view = StoresView::multi(&stores, &uri_to_drone, "A", &empty);

    // Cell pinned to drone-B's URI → should resolve to drone-B's store (2.0),
    // EVEN THOUGH the view-drone is "A". This is the v0.16.2 cross-drone pin.
    let pinned = CellSource {
        key: "ap_attitude[0]".into(),
        source_uri: Some("zmq://127.0.0.1:9006".to_string()),
        ..Default::default()
    };
    let store = view.for_source(&pinned);
    assert_eq!(
        store.latest("ap_attitude[0]"),
        Some(2.0),
        "pinned source_uri must route to drone-B's store (latest=2.0)",
    );
}

#[test]
fn unpinned_source_resolves_to_view_drone_store() {
    let (stores, uri_to_drone, empty) = two_drone_setup();

    // View-drone A → unpinned source reads from A (1.0).
    let view_a = StoresView::multi(&stores, &uri_to_drone, "A", &empty);
    let unpinned = CellSource {
        key: "ap_attitude[0]".into(),
        source_uri: None,
        ..Default::default()
    };
    assert_eq!(
        view_a.for_source(&unpinned).latest("ap_attitude[0]"),
        Some(1.0),
        "view-drone A → unpinned cell reads 1.0",
    );

    // View-drone B → same unpinned source now reads 2.0.
    let view_b = StoresView::multi(&stores, &uri_to_drone, "B", &empty);
    assert_eq!(
        view_b.for_source(&unpinned).latest("ap_attitude[0]"),
        Some(2.0),
        "view-drone B → unpinned cell reads 2.0",
    );
}

#[test]
fn pinned_cell_unaffected_by_view_drone_switch() {
    // The whole point of the v0.16.2 pin: switching the view-drone dropdown
    // must NOT move the pinned cell off drone-B.
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let pinned = CellSource {
        key: "ap_attitude[0]".into(),
        source_uri: Some("zmq://127.0.0.1:9006".to_string()),
        ..Default::default()
    };

    let view_a = StoresView::multi(&stores, &uri_to_drone, "A", &empty);
    let view_b = StoresView::multi(&stores, &uri_to_drone, "B", &empty);
    let view_unknown = StoresView::multi(&stores, &uri_to_drone, "nonexistent", &empty);

    for (label, view) in [("A", &view_a), ("B", &view_b), ("nonexistent", &view_unknown)] {
        assert_eq!(
            view.for_source(&pinned).latest("ap_attitude[0]"),
            Some(2.0),
            "pinned cell stays on drone-B regardless of view-drone={label}",
        );
    }
}

#[test]
fn empty_source_uri_string_treated_as_unpinned() {
    // The editor's "(any)" selection stores `source_uri: Some(String::new())`
    // in the draft and serialises it as `None` via `non_empty(&draft.source_uri)`
    // before writing to the template. The renderer must defensively treat an
    // EMPTY string pin as unpinned too (matches `format_source_combo_label`'s
    // "(any)" display logic).
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let view = StoresView::multi(&stores, &uri_to_drone, "A", &empty);
    let any_pinned = CellSource {
        key: "ap_attitude[0]".into(),
        source_uri: Some(String::new()),
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&any_pinned).latest("ap_attitude[0]"),
        Some(1.0),
        "empty-string source_uri must be treated as `(any)` and fall through to view-drone",
    );
}

#[test]
fn unknown_pinned_uri_falls_back_to_view_drone() {
    // Cell pinned to a URI the registry doesn't know (e.g. saved template
    // loaded against a different source set) → fall back to view-drone.
    // Pre-v0.16.2 this case quietly read from the view-drone store anyway
    // (since URI was ignored entirely); the new behaviour must preserve
    // that — pinning to an unknown URI is a "no-op" from the renderer's
    // perspective, NOT an empty-data state.
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let view = StoresView::multi(&stores, &uri_to_drone, "B", &empty);
    let stale_pin = CellSource {
        key: "ap_attitude[0]".into(),
        source_uri: Some("zmq://10.0.0.1:9999".to_string()),
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&stale_pin).latest("ap_attitude[0]"),
        Some(2.0),
        "stale pin must fall back to view-drone B (2.0), not the empty store",
    );
}

#[test]
fn pin_to_known_uri_with_no_store_yet_falls_back_to_view_drone() {
    // The URI exists in `uri_to_drone` (source connected) but no envelope
    // has arrived yet so `stores` has no entry for that drone-name. The
    // resolver must fall back to the view-drone rather than panic / return
    // empty — the user will see view-drone's data with a (future) toolbar
    // warning chip until samples arrive.
    let (stores, _, empty) = two_drone_setup();
    let mut uri_to_drone: HashMap<String, String> = HashMap::new();
    uri_to_drone.insert("zmq://127.0.0.1:9005".to_string(), "A".to_string());
    uri_to_drone.insert("zmq://127.0.0.1:9006".to_string(), "B".to_string());
    // C is connected but has no samples → not in `stores`.
    uri_to_drone.insert("zmq://127.0.0.1:9007".to_string(), "C".to_string());
    let view = StoresView::multi(&stores, &uri_to_drone, "A", &empty);
    let c_pin = CellSource {
        key: "ap_attitude[0]".into(),
        source_uri: Some("zmq://127.0.0.1:9007".to_string()),
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&c_pin).latest("ap_attitude[0]"),
        Some(1.0),
        "drone-C has no store yet → fall back to view-drone A (1.0)",
    );
}

#[test]
fn legacy_single_view_ignores_source_uri_pin() {
    // Single-store mode (used by `render_template_grid_full`'s wrapper)
    // must NEVER honour a per-cell pin — old single-source callers expect
    // every cell to read from the one wrapped store, and routing through a
    // (non-existent in their world) URI map would surface empty data
    // instead of the wrapped store's samples.
    let store = make_store(7.0);
    let empty_map: HashMap<String, String> = HashMap::new();
    let view = StoresView::single(&store, &empty_map);

    let pinned_to_b = CellSource {
        key: "ap_attitude[0]".into(),
        // A URI no one has heard of — single-store mode IGNORES the pin.
        source_uri: Some("zmq://127.0.0.1:9006".to_string()),
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&pinned_to_b).latest("ap_attitude[0]"),
        Some(7.0),
        "single-store mode must return the wrapped store regardless of pin",
    );
    let unpinned = CellSource {
        key: "ap_attitude[0]".into(),
        source_uri: None,
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&unpinned).latest("ap_attitude[0]"),
        Some(7.0),
        "single-store mode returns the wrapped store for unpinned cells too",
    );
}

#[test]
fn for_uri_resolves_status_pin() {
    // The `Status` primitive carries no `CellSource`s the renderer can pin
    // through — the v0.16.2 `render_cell` hook reads `cell.sources[0]`'s
    // `source_uri` and routes the bare key via `view.for_uri(...)`. Verify
    // that path resolves the same way.
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let view = StoresView::multi(&stores, &uri_to_drone, "A", &empty);

    assert_eq!(
        view.for_uri(Some("zmq://127.0.0.1:9006"))
            .latest("ap_attitude[0]"),
        Some(2.0),
        "Status pinned to drone-B URI reads from drone-B store",
    );
    assert_eq!(
        view.for_uri(None).latest("ap_attitude[0]"),
        Some(1.0),
        "Status with no pin reads from view-drone A",
    );
    assert_eq!(
        view.for_uri(Some("")).latest("ap_attitude[0]"),
        Some(1.0),
        "Status with empty-string pin treated as `(any)` → view-drone A",
    );
}

#[test]
fn view_store_returns_empty_when_view_drone_has_no_store() {
    // First-frame behaviour: drain hasn't run yet, no samples for the
    // configured view-drone. `view_store()` must return the empty
    // fallback (not panic, not pick an arbitrary drone).
    let stores: HashMap<String, TraceStore> = HashMap::new();
    let uri_to_drone: HashMap<String, String> = HashMap::new();
    let empty = TraceStore::default();
    let view = StoresView::multi(&stores, &uri_to_drone, "ghost", &empty);
    assert!(
        view.view_store().is_empty(),
        "view_store() must be the empty store when the view-drone is unknown",
    );
}

#[test]
fn iter_all_stores_covers_every_drone_in_multi_mode() {
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let view = StoresView::multi(&stores, &uri_to_drone, "A", &empty);
    let mut latest_values: Vec<f64> = view
        .iter_all_stores()
        .map(|s| s.latest("ap_attitude[0]").unwrap_or(f64::NAN))
        .collect();
    latest_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(
        latest_values,
        vec![1.0, 2.0],
        "iter_all_stores must enumerate both drones",
    );
}

#[test]
fn iter_all_stores_covers_single_store_in_single_mode() {
    let store = make_store(42.0);
    let empty_map: HashMap<String, String> = HashMap::new();
    let view = StoresView::single(&store, &empty_map);
    let collected: Vec<f64> = view
        .iter_all_stores()
        .map(|s| s.latest("ap_attitude[0]").unwrap_or(f64::NAN))
        .collect();
    assert_eq!(
        collected,
        vec![42.0],
        "iter_all_stores in single mode yields exactly the wrapped store",
    );
}

// ─── v0.16.8 — drone-level pin (CellSource::source_drone) ───────────────────
//
// The v0.15.0 URI pin breaks for the v0.16.4 shared MAVLink demux flow
// where one URI carries N drones (`uri_to_drone` only maps the URI to the
// first drone seen). v0.16.8 adds `CellSource::source_drone` — a stable
// drone-name pin — and reorders `StoresView::for_source` so drone-pin wins
// over URI-pin.

#[test]
fn drone_pin_beats_view_drone() {
    // Cell pinned to drone-B via `source_drone` while the toolbar view is
    // on drone A → resolves to drone-B's store. Mirrors `source_uri` pin's
    // cross-drone semantics but pins by NAME instead of URI.
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let view = StoresView::multi(&stores, &uri_to_drone, "A", &empty);

    let pinned = CellSource {
        key: "ap_attitude[0]".into(),
        source_drone: Some("B".into()),
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&pinned).latest("ap_attitude[0]"),
        Some(2.0),
        "drone-pin must route to drone-B's store regardless of view-drone",
    );
}

#[test]
fn drone_pin_beats_uri_pin() {
    // Cell with BOTH `source_drone = "B"` and `source_uri = <A's URI>`.
    // v0.16.8 precedence: drone-pin always wins. The legacy URI-pin path
    // would have routed to A — the new precedence overrides that so the
    // cell follows the operator's drone-level intent.
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let view = StoresView::multi(&stores, &uri_to_drone, "A", &empty);

    let pinned = CellSource {
        key: "ap_attitude[0]".into(),
        // URI points to A in `uri_to_drone` — confirm URI-pin would route to A
        source_uri: Some("zmq://127.0.0.1:9005".into()),
        // ...but drone-pin says B → drone-pin wins.
        source_drone: Some("B".into()),
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&pinned).latest("ap_attitude[0]"),
        Some(2.0),
        "drone-pin must win over conflicting URI-pin",
    );
}

#[test]
fn shared_mavlink_demux_routes_by_drone() {
    // The v0.16.4 shared MAVLink demux case: TWO drones (A and B) share one
    // URI `mavlink://0.0.0.0:14560`. `uri_to_drone` only holds one entry per
    // URI — it maps to A (the first drone seen on that port). A cell pinned
    // to drone B via `source_drone` MUST still route to B's store. With only
    // URI-pin available (the v0.15.0 path) this would have leaked drone A's
    // data into the pinned cell — the v0.16.8 drone-pin saves us.
    let mut stores: HashMap<String, TraceStore> = HashMap::new();
    stores.insert("A".to_string(), make_store(1.0));
    stores.insert("B".to_string(), make_store(2.0));
    let mut uri_to_drone: HashMap<String, String> = HashMap::new();
    // Only one entry possible: shared URI demuxes to whichever drone was
    // observed first. URI-pin path would always return A.
    uri_to_drone.insert(
        "mavlink://0.0.0.0:14560".to_string(),
        "A".to_string(),
    );
    let empty = TraceStore::default();
    let view = StoresView::multi(&stores, &uri_to_drone, "A", &empty);

    let pinned_to_b = CellSource {
        key: "ap_attitude[0]".into(),
        // Same URI as drone A — but drone-pin names B explicitly.
        source_uri: Some("mavlink://0.0.0.0:14560".into()),
        source_drone: Some("B".into()),
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&pinned_to_b).latest("ap_attitude[0]"),
        Some(2.0),
        "shared MAVLink port: drone-pin must route to B even though URI maps to A",
    );
}

#[test]
fn empty_drone_pin_falls_back_to_uri_pin() {
    // Legacy template: only `source_uri` is set, `source_drone` is `None`.
    // Resolution must walk through to the URI-pin path (drone-pin not
    // applicable). This is the v0.15.0 backward-compat case.
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let view = StoresView::multi(&stores, &uri_to_drone, "A", &empty);

    let legacy_pin = CellSource {
        key: "ap_attitude[0]".into(),
        source_drone: None,
        source_uri: Some("zmq://127.0.0.1:9006".into()),
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&legacy_pin).latest("ap_attitude[0]"),
        Some(2.0),
        "drone-pin None → URI-pin still resolves (v0.15.0 compat)",
    );

    // Same again but with empty-string drone-pin: must still be treated as
    // unpinned and walk to URI-pin (matches the v0.15.0 empty-string contract).
    let legacy_pin_empty = CellSource {
        key: "ap_attitude[0]".into(),
        source_drone: Some(String::new()),
        source_uri: Some("zmq://127.0.0.1:9006".into()),
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&legacy_pin_empty)
            .latest("ap_attitude[0]"),
        Some(2.0),
        "empty-string drone-pin treated as unpinned → URI-pin resolves",
    );
}

#[test]
fn both_empty_falls_back_to_view_drone() {
    // Neither drone-pin nor URI-pin is set → cell follows the view-drone.
    // Preserves the v0.16.1 (and earlier) behaviour exactly.
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let view = StoresView::multi(&stores, &uri_to_drone, "B", &empty);

    let unpinned = CellSource {
        key: "ap_attitude[0]".into(),
        source_drone: None,
        source_uri: None,
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&unpinned).latest("ap_attitude[0]"),
        Some(2.0),
        "no pins → view-drone B's store",
    );
}

#[test]
fn unknown_drone_pin_falls_back_to_uri_pin_then_view() {
    // Cell pinned to a drone NAME that isn't in `stores` yet (e.g. fleet
    // hasn't published a sample for that drone yet, or the template is
    // loaded against a fleet that doesn't include that drone). Should fall
    // back to URI-pin if set, otherwise view-drone — never the empty store.
    let (stores, uri_to_drone, empty) = two_drone_setup();
    let view = StoresView::multi(&stores, &uri_to_drone, "A", &empty);

    // (a) stale drone-pin, valid URI-pin → URI-pin wins
    let stale_drone = CellSource {
        key: "ap_attitude[0]".into(),
        source_drone: Some("nonexistent".into()),
        source_uri: Some("zmq://127.0.0.1:9006".into()),
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&stale_drone).latest("ap_attitude[0]"),
        Some(2.0),
        "stale drone-pin → URI-pin (drone-B) resolves",
    );

    // (b) stale drone-pin, no URI-pin → view-drone (A)
    let stale_drone_no_uri = CellSource {
        key: "ap_attitude[0]".into(),
        source_drone: Some("nonexistent".into()),
        source_uri: None,
        ..Default::default()
    };
    assert_eq!(
        view.for_source(&stale_drone_no_uri).latest("ap_attitude[0]"),
        Some(1.0),
        "stale drone-pin + no URI → view-drone A's store",
    );
}
