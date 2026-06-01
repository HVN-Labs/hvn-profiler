//! v0.10.1 — MultiSource only re-merges native discovery sets when they grow.
//!
//! The v0.10.0 implementation cloned every leg's `inner_seen` set into a Vec
//! and write-locked `merged` on EVERY sample — 300+ write-locks/sec on a
//! 10 Hz × 5-drone fleet. v0.10.1 tracks each leg's `last_seen_len` and
//! skips the re-merge when the leg hasn't grown.
//!
//! We can't easily instrument the internal write-lock count (the field is
//! private), so we pin the observable contract:
//! - After feeding N samples for a stable drone roster, `merged` matches the
//!   union of `drone_name`s.
//! - `merged.len()` stops changing once the roster stabilises (no spurious
//!   re-insertions corrupt the state).

use profiler_source::MavlinkConfig;

/// Two mock legs drained briefly — the merged set should converge to a small
/// stable size (the two fallback URI-derived names) and stay there.
#[test]
fn merged_size_is_stable_after_roster_stabilises() {
    let uris = vec!["mock://".to_string(), "mock://".to_string()];
    let (mut src, seen) = profiler_source::multi_from_uris_with_discovery_opts(
        &uris,
        MavlinkConfig::default(),
    )
    .expect("multi source");
    let seen = seen.expect("multi-URI run returns Some(seen_drones)");

    // Burn a bunch of samples to let both legs stamp their fallback names.
    let mut drained = 0;
    for _ in 0..600 {
        if src.try_recv().is_some() {
            drained += 1;
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
        if drained >= 50 {
            break;
        }
    }

    let len_after_warmup = seen.read().unwrap().len();
    // mock:// has no native discovery, so `inner_seen` is None on every leg
    // — only the inline `Some(name) → insert` path runs. Both fallback names
    // ("src0" / "src1" or similar) should be in `merged` after a brief drain.
    assert!(
        len_after_warmup >= 1,
        "at least one fallback name reaches merged: got len={len_after_warmup}",
    );

    // Drain another batch; the merged set MUST NOT grow further (the roster
    // is stable: only "src0" / "src1" exist).
    for _ in 0..300 {
        let _ = src.try_recv();
    }
    let len_after_more = seen.read().unwrap().len();
    assert_eq!(
        len_after_warmup, len_after_more,
        "merged set must NOT grow on a stable roster (v0.10.1 redundant re-merge fix)",
    );
}

/// Steady-state hot-path sanity: even after thousands of samples the
/// merged-set membership stays bounded. The v0.10.0 bug would NOT actually
/// grow `merged.len()` (the HashSet dedupes), but it WOULD acquire ~300
/// write-locks/sec; this test pins the membership invariant the user-facing
/// behaviour depends on.
#[test]
fn merged_set_membership_does_not_drift_under_sustained_drain() {
    let uris = vec!["mock://".to_string(), "mock://".to_string()];
    let (mut src, seen) = profiler_source::multi_from_uris_with_discovery_opts(
        &uris,
        MavlinkConfig::default(),
    )
    .expect("multi source");
    let seen = seen.expect("multi-URI run returns Some(seen_drones)");

    // Drain ~50 samples in three waves; the merged set captured between
    // waves should be identical (no drift, no spurious mutations).
    fn drain_some(src: &mut Box<dyn profiler_source::Source>, target: usize) {
        let mut got = 0;
        for _ in 0..2_000 {
            if src.try_recv().is_some() {
                got += 1;
                if got >= target {
                    return;
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }

    drain_some(&mut src, 30);
    let snap_a: std::collections::BTreeSet<String> = seen.read().unwrap().iter().cloned().collect();
    drain_some(&mut src, 60);
    let snap_b: std::collections::BTreeSet<String> = seen.read().unwrap().iter().cloned().collect();
    drain_some(&mut src, 90);
    let snap_c: std::collections::BTreeSet<String> = seen.read().unwrap().iter().cloned().collect();

    // Once both legs have emitted at least one sample, the set is steady.
    // It's permitted for the second leg to lag in the first snapshot; once
    // both are in, subsequent snapshots match.
    let stable_size = snap_c.len();
    assert!(snap_a.is_subset(&snap_b));
    assert!(snap_b.is_subset(&snap_c));
    assert!(stable_size <= 2, "at most two fallback names from two mock legs (got {stable_size}: {snap_c:?})");
}
