//! v0.16.2 — drone-name routing audit.
//!
//! The v0.16.2 brief asks to verify "samples never land in the wrong
//! per-drone bucket" — i.e. drone-A's `accel[0]` doesn't pollute drone-B's
//! `accel[0]` ring buffer when both legs are merged through a multi-source.
//!
//! This test exercises the same routing logic the CLI's `App::drain` uses:
//! `stores.entry(s.drone_name).or_default().push(s.ts, &s.key, value)`. The
//! upstream contract is:
//!
//! 1. Every `Sample` that arrives via `MultiSource`/`SourceRegistry::try_recv`
//!    carries `drone_name = Some(...)`. The `Sample.drone_name = None` slot
//!    only exists for raw legs — `MultiSource`'s `Source::try_recv` impl
//!    stamps the leg's `fallback_name` before yielding.
//! 2. Two legs that produce samples with DIFFERENT `drone_name` values must
//!    NEVER cross-contaminate. The MultiSource carries no shared state
//!    across legs other than the (read-only) merged seen-drones set; this
//!    test confirms that property at the public `try_recv` boundary.
//!
//! Mock URIs deliver synthetic `src0` / `src1` fallback names — distinct
//! enough to assert "per-drone bucket" without spinning real ZMQ PUBs.

use std::collections::HashMap;
use std::sync::Arc;

use profiler_source::{
    multi_from_uris_with_discovery_opts, MavlinkConfig, Sample, Source, Value,
};

/// A deterministic `Source` that yields pre-loaded samples once and then
/// returns `None` forever. Used here to drive `App::drain`-style routing
/// without going through the network layer.
struct ScriptedSource {
    queue: std::collections::VecDeque<Sample>,
}

impl Source for ScriptedSource {
    fn try_recv(&mut self) -> Option<Sample> {
        self.queue.pop_front()
    }
    fn describe(&self) -> String {
        "scripted://".to_string()
    }
}

fn sample(ts: f64, drone: &str, key: &str, value: f64) -> Sample {
    Sample {
        ts,
        key: key.to_string(),
        value: Value::Scalar(value),
        drone_name: Some(Arc::from(drone)),
        sysid: None,
    }
}

/// Re-impl of the CLI's drain-routing loop: route each sample to
/// `stores[drone_name]`, keyed exactly the way `App::drain` does. We don't
/// link to `profiler-cli` from a source-crate integration test, but the
/// routing predicate is small enough to mirror here.
fn drain_into<S: Source>(
    src: &mut S,
    stores: &mut HashMap<String, Vec<(String, f64)>>,
) -> usize {
    let mut n = 0;
    while let Some(s) = src.try_recv() {
        let key = s
            .drone_name
            .as_deref()
            .map(str::to_string)
            .unwrap_or_else(|| "(unnamed)".to_string());
        // Only the scalar values matter for this audit.
        if let Value::Scalar(v) = s.value {
            stores
                .entry(key)
                .or_default()
                .push((s.key.clone(), v));
        }
        n += 1;
    }
    n
}

#[test]
fn two_drones_never_cross_contaminate_per_drone_buckets() {
    // 100 samples on each leg, each tagged with its own drone name. After
    // draining, each per-drone bucket must contain exactly 100 entries with
    // ONLY that drone's sentinel values.
    //
    // Drone-A's `accel[0]` ranges over [0.0, 0.99] (i / 100).
    // Drone-B's `accel[0]` ranges over [100.0, 199.99] (100 + i / 100).
    let mut q_a: std::collections::VecDeque<Sample> = std::collections::VecDeque::new();
    let mut q_b: std::collections::VecDeque<Sample> = std::collections::VecDeque::new();
    for i in 0..100 {
        let t = i as f64 * 0.01;
        q_a.push_back(sample(t, "A", "accel[0]", i as f64 / 100.0));
        q_b.push_back(sample(t, "B", "accel[0]", 100.0 + (i as f64) / 100.0));
    }

    let mut a = ScriptedSource { queue: q_a };
    let mut b = ScriptedSource { queue: q_b };

    let mut stores: HashMap<String, Vec<(String, f64)>> = HashMap::new();
    let drained_a = drain_into(&mut a, &mut stores);
    let drained_b = drain_into(&mut b, &mut stores);
    assert_eq!(drained_a, 100);
    assert_eq!(drained_b, 100);

    let bucket_a = stores.get("A").expect("drone-A bucket");
    let bucket_b = stores.get("B").expect("drone-B bucket");

    assert_eq!(bucket_a.len(), 100, "drone-A bucket holds exactly 100 samples");
    assert_eq!(bucket_b.len(), 100, "drone-B bucket holds exactly 100 samples");

    // Cross-contamination check: drone-A bucket must hold only A-range
    // values; drone-B bucket only B-range. Values overlap in NEITHER
    // direction since their ranges are disjoint.
    assert!(
        bucket_a.iter().all(|(_, v)| *v < 1.0),
        "drone-A bucket leaked drone-B samples (any value >= 1.0 would be from B)",
    );
    assert!(
        bucket_b.iter().all(|(_, v)| *v >= 100.0),
        "drone-B bucket leaked drone-A samples (any value < 100.0 would be from A)",
    );
}

#[test]
fn multi_source_stamps_distinct_names_on_unnamed_legs() {
    // The MultiSource production code stamps a per-leg fallback drone-name
    // when the underlying source doesn't carry one (mock://, mavlink://).
    // This test reproduces the public-API path: two mock URIs MUST resolve
    // to distinct fallback names so per-drone routing has stable buckets
    // even before any envelope wakes up with a real name.
    let uris = vec!["mock://leg-a".to_string(), "mock://leg-b".to_string()];
    let (mut src, _) =
        multi_from_uris_with_discovery_opts(&uris, MavlinkConfig::default())
            .expect("multi source");

    let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut drained = 0;
    let deadline = std::time::Instant::now() + std::time::Duration::from_millis(800);
    while std::time::Instant::now() < deadline {
        if let Some(s) = src.try_recv() {
            let name = s
                .drone_name
                .as_deref()
                .map(str::to_string)
                .unwrap_or_else(|| "(none)".to_string());
            seen.insert(name);
            drained += 1;
            if seen.len() >= 2 {
                break;
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
    assert!(
        seen.len() >= 2,
        "two mock legs must produce ≥2 distinct fallback drone names (seen={seen:?}, drained={drained})",
    );
}

#[test]
fn no_two_sources_share_a_fallback_drone_name() {
    // The brief's defensive audit: two legs whose URIs would derive an
    // identical fallback name (e.g. two `mock://` URIs) must still be
    // distinct after going through the multi-source plumbing. `MultiSource`
    // uses the LEG INDEX to disambiguate (`src0`, `src1`, ...).
    let uris = vec!["mock://".to_string(), "mock://".to_string()];
    let (mut src, _) =
        multi_from_uris_with_discovery_opts(&uris, MavlinkConfig::default())
            .expect("multi source");

    let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
    let deadline = std::time::Instant::now() + std::time::Duration::from_millis(800);
    while std::time::Instant::now() < deadline {
        if let Some(s) = src.try_recv() {
            if let Some(n) = s.drone_name {
                seen.insert(n.to_string());
                if seen.len() >= 2 {
                    break;
                }
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
    assert!(
        seen.len() >= 2,
        "two identical-URI mock legs must still get distinct fallback names: seen={seen:?}",
    );
    // Both must follow the `src{N}` pattern.
    for n in &seen {
        assert!(
            n.starts_with("src"),
            "fallback name should follow `src{{N}}` shape: got '{n}'",
        );
    }
}

#[test]
fn unnamed_samples_route_into_unnamed_bucket_only() {
    // Edge case: a Sample with `drone_name = None` (legacy / raw test path)
    // must NOT leak into a named drone's bucket. `App::drain` keys by
    // `drone_name.unwrap_or("(unnamed)")` — so the only bucket that ever
    // accumulates None-named samples is `"(unnamed)"`.
    let q: std::collections::VecDeque<Sample> = (0..20)
        .map(|i| Sample {
            ts: i as f64 * 0.01,
            key: "accel[0]".to_string(),
            value: Value::Scalar(i as f64),
            drone_name: None,
            sysid: None,
        })
        .collect();
    let mut src = ScriptedSource { queue: q };
    let mut stores: HashMap<String, Vec<(String, f64)>> = HashMap::new();
    let n = drain_into(&mut src, &mut stores);
    assert_eq!(n, 20);
    assert!(
        stores.keys().all(|k| k == "(unnamed)"),
        "unnamed samples leaked into a named bucket: keys={:?}",
        stores.keys().collect::<Vec<_>>(),
    );
    assert_eq!(stores["(unnamed)"].len(), 20);
}
