//! profiler-render — egui_plot wrappers + GPU-friendly trace storage.
//!
//! v0.1.0 introduces [`TraceStore`], a per-key ring buffer of `[t, value]`
//! points. Samples are pushed in via [`TraceStore::push`]; rendering code
//! calls [`TraceStore::points`] each frame to grab a slice for `egui_plot`.
//!
//! Retention is time-based (default 60 s). Old points are pruned lazily on
//! each `push` against the latest observed timestamp, so the store is
//! self-trimming without a background thread.

use std::collections::{HashMap, VecDeque};

pub mod panels;
pub use panels::{render_template_grid, GridStats};

/// Build-time crate version, for logging from the CLI.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Default retention window, in seconds.
pub const DEFAULT_WINDOW_S: f64 = 60.0;

/// Sanity check used by the CLI smoke test.
pub fn hello() -> String {
    format!("profiler-render v{VERSION}")
}

/// Per-key ring buffer of `[t, value]` points with time-based retention.
///
/// Memory grows with input rate × `window_s`; pruning happens on each
/// `push` against the most-recent timestamp seen so far. We track that
/// timestamp separately (rather than relying on `back()`) so an out-of-order
/// sample on one key doesn't drag the retention horizon backward on others.
#[derive(Debug, Clone)]
pub struct TraceStore {
    /// Retention window in seconds. Points older than `latest_ts - window_s`
    /// are pruned on each push.
    pub window_s: f64,
    /// Per-channel rolling buffer of `[t, value]` points.
    traces: HashMap<String, VecDeque<[f64; 2]>>,
    /// Most recent timestamp observed across all channels.
    latest_ts: f64,
}

impl Default for TraceStore {
    fn default() -> Self {
        Self::new(DEFAULT_WINDOW_S)
    }
}

impl TraceStore {
    /// Construct a store with the given retention window (seconds).
    pub fn new(window_s: f64) -> Self {
        Self {
            window_s,
            traces: HashMap::new(),
            latest_ts: f64::NEG_INFINITY,
        }
    }

    /// Push one point. `key` is the trace identifier (e.g. `"accel[0]"`).
    pub fn push(&mut self, t: f64, key: &str, value: f64) {
        if t > self.latest_ts {
            self.latest_ts = t;
        }
        let buf = self
            .traces
            .entry(key.to_string())
            .or_insert_with(|| VecDeque::with_capacity(256));
        buf.push_back([t, value]);

        // Prune old points across every channel against the global horizon.
        // Cheap: front-popping a VecDeque is O(1).
        let cutoff = self.latest_ts - self.window_s;
        for q in self.traces.values_mut() {
            while let Some(front) = q.front() {
                if front[0] < cutoff {
                    q.pop_front();
                } else {
                    break;
                }
            }
        }
    }

    /// Return the points for a key (chronological, oldest → newest).
    /// Empty slice if the key was never seen.
    pub fn points(&self, key: &str) -> Vec<[f64; 2]> {
        self.traces
            .get(key)
            .map(|q| q.iter().copied().collect())
            .unwrap_or_default()
    }

    /// Latest (most recent) value stored for `key`, or `None` if empty.
    pub fn latest(&self, key: &str) -> Option<f64> {
        self.traces.get(key).and_then(|q| q.back()).map(|p| p[1])
    }

    /// `(min, max)` of the values stored for `key` over the current window,
    /// or `None` if the key has no points.
    pub fn min_max(&self, key: &str) -> Option<(f64, f64)> {
        let q = self.traces.get(key)?;
        let mut it = q.iter();
        let first = it.next()?[1];
        let (mut lo, mut hi) = (first, first);
        for p in it {
            let v = p[1];
            if v < lo {
                lo = v;
            }
            if v > hi {
                hi = v;
            }
        }
        Some((lo, hi))
    }

    /// All known keys, alphabetical.
    pub fn keys(&self) -> Vec<String> {
        let mut k: Vec<String> = self.traces.keys().cloned().collect();
        k.sort();
        k
    }

    /// Pick the trace with the most stored points (ties broken alphabetically).
    /// Returns `None` if the store is empty.
    pub fn busiest_key(&self) -> Option<String> {
        self.traces
            .iter()
            .max_by(|a, b| {
                a.1.len()
                    .cmp(&b.1.len())
                    .then_with(|| b.0.cmp(a.0)) // alphabetical-asc tie-break
            })
            .map(|(k, _)| k.clone())
    }

    /// Number of points currently stored for `key`.
    pub fn len(&self, key: &str) -> usize {
        self.traces.get(key).map(VecDeque::len).unwrap_or(0)
    }

    /// `true` if no points have ever been pushed.
    pub fn is_empty(&self) -> bool {
        self.traces.values().all(VecDeque::is_empty)
    }

    /// Latest observed timestamp across all channels (or `-inf` if empty).
    pub fn latest_ts(&self) -> f64 {
        self.latest_ts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_is_versioned() {
        assert!(hello().starts_with("profiler-render v"));
        assert!(hello().contains(VERSION));
    }

    #[test]
    fn push_and_recall() {
        let mut s = TraceStore::new(10.0);
        s.push(0.0, "a", 1.0);
        s.push(0.1, "a", 2.0);
        s.push(0.2, "b", 3.0);
        assert_eq!(s.points("a"), vec![[0.0, 1.0], [0.1, 2.0]]);
        assert_eq!(s.points("b"), vec![[0.2, 3.0]]);
        assert_eq!(s.points("missing"), Vec::<[f64; 2]>::new());
    }

    #[test]
    fn retention_prunes_old() {
        let mut s = TraceStore::new(1.0);
        for i in 0..10 {
            s.push(i as f64 * 0.5, "a", i as f64);
        }
        // latest_ts = 4.5, cutoff = 3.5 → only points with t >= 3.5 survive.
        let pts = s.points("a");
        assert!(pts.iter().all(|p| p[0] >= 3.5));
        assert!(!pts.is_empty());
    }

    #[test]
    fn busiest_key_picks_largest() {
        let mut s = TraceStore::new(60.0);
        for i in 0..5 {
            s.push(i as f64 * 0.01, "rare", i as f64);
        }
        for i in 0..50 {
            s.push(i as f64 * 0.01, "common", i as f64);
        }
        assert_eq!(s.busiest_key().as_deref(), Some("common"));
    }
}
