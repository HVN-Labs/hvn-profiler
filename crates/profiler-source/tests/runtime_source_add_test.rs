//! v0.15.0 — runtime add/remove tests for [`SourceRegistry`].
//!
//! The spec asks us to add a source at runtime, push samples through it, and
//! observe the new sample stream reaching `try_recv()`. Then remove it and
//! verify no new samples arrive. Because the registry doesn't expose a
//! direct "push" hook (real sources are wire-driven), we exercise the same
//! contract using the `mock://` backend: `MockSource` emits a steady ~60 Hz
//! sine wave, which is enough to prove the registry's drain path picks up a
//! newly-added leg and stops draining once the leg is removed.

use std::time::Instant;

use profiler_source::{MavlinkConfig, Source, SourceRegistry};

/// Drain up to `max` samples from the registry, returning the count actually
/// drained. Stops at the first `None`.
fn drain_n(reg: &mut SourceRegistry, max: usize) -> usize {
    let mut n = 0;
    while n < max {
        match reg.try_recv() {
            Some(_) => n += 1,
            None => break,
        }
    }
    n
}

/// Sleep `MockSource`'s emit period a few times so it's ready to emit again.
fn wait_for_mock_emits() {
    // MockSource ticks at ~60 Hz (16.6 ms). Sleep 30 ms so the next try_recv
    // is guaranteed to yield.
    std::thread::sleep(std::time::Duration::from_millis(30));
}

#[test]
fn add_source_at_runtime_starts_streaming() {
    let mut reg = SourceRegistry::new(MavlinkConfig::default());
    // Empty registry: nothing to drain.
    assert_eq!(reg.len(), 0);
    assert!(reg.is_empty());
    assert_eq!(drain_n(&mut reg, 32), 0);

    // Add a mock source at runtime.
    reg.add("mock://").expect("add mock");
    assert_eq!(reg.len(), 1);
    assert!(!reg.is_empty());

    // Drain — we expect at least one sample from the just-added mock leg
    // within a few try_recv iterations (the first call always yields).
    let start = Instant::now();
    let mut got = 0;
    while got < 3 && start.elapsed().as_secs_f64() < 2.0 {
        got += drain_n(&mut reg, 32);
        if got < 3 {
            wait_for_mock_emits();
        }
    }
    assert!(got >= 1, "registry should drain ≥ 1 sample from runtime-added mock leg (got {got})");
}

#[test]
fn remove_source_stops_streaming() {
    let mut reg = SourceRegistry::new(MavlinkConfig::default());
    reg.add("mock://").expect("add mock");
    // Drain a few samples first so the leg is known-good.
    let mut warmup = 0;
    for _ in 0..5 {
        warmup += drain_n(&mut reg, 8);
        wait_for_mock_emits();
    }
    assert!(warmup >= 1, "warmup drained ≥ 1 sample");

    // Remove the source; registry is empty now.
    let removed = reg.remove("mock://");
    assert!(removed, "remove returns true on hit");
    assert_eq!(reg.len(), 0);
    assert!(reg.is_empty());

    // No new samples arrive — drain_n should return 0 across multiple ticks.
    for _ in 0..5 {
        wait_for_mock_emits();
        assert_eq!(
            drain_n(&mut reg, 32),
            0,
            "removed source must not yield new samples",
        );
    }
}

#[test]
fn add_is_idempotent() {
    // Re-adding the same URI does not open a second leg; the registry stays
    // at size 1 so the operator can spam Connect without leaking resources.
    let mut reg = SourceRegistry::new(MavlinkConfig::default());
    reg.add("mock://").expect("first add");
    reg.add("mock://").expect("second add");
    reg.add("mock://").expect("third add");
    assert_eq!(reg.len(), 1, "duplicate adds are deduplicated");
}

#[test]
fn remove_missing_uri_returns_false() {
    let mut reg = SourceRegistry::new(MavlinkConfig::default());
    assert!(!reg.remove("zmq://not.connected:9999"));
    reg.add("mock://").expect("add mock");
    assert!(!reg.remove("zmq://not.connected:9999"));
    assert_eq!(reg.len(), 1, "remove of non-existent URI is a no-op");
}

#[test]
fn list_reports_uri_and_liveness() {
    // The toolbar Sources dropdown reads `registry.list(live_threshold_s)`
    // each frame; this test pins the basic contract: every connected source
    // shows up in the list with its URI, and `is_live` flips from false to
    // true once samples start flowing.
    let mut reg = SourceRegistry::new(MavlinkConfig::default());
    reg.add("mock://").expect("add mock");
    // Before any sample, the leg is not yet live.
    let before = reg.list(3.0);
    assert_eq!(before.len(), 1);
    assert_eq!(before[0].uri, "mock://");
    assert!(!before[0].is_live, "no samples yet => is_live=false");

    // Drain a sample so last_sample_at advances.
    wait_for_mock_emits();
    let _ = drain_n(&mut reg, 4);
    let after = reg.list(3.0);
    assert_eq!(after.len(), 1);
    assert!(after[0].is_live, "after drain => is_live=true");
}

#[test]
fn multi_uri_construction_via_with_uris() {
    // CLI startup path: a vec of URIs is folded into the registry; the
    // merged seen-drones handle is shared across all legs.
    let uris = vec!["mock://".to_string(), "mock://second/".to_string()];
    let (reg, seen) = SourceRegistry::with_uris(&uris, MavlinkConfig::default())
        .expect("with_uris ok");
    // Both legs registered (mock:// and mock://second/ — different URIs).
    assert_eq!(reg.len(), 2);
    // Merged-seen handle exists and is empty initially.
    let g = seen.read().expect("read lock");
    assert!(g.is_empty(), "no drones seen until samples arrive");
}
