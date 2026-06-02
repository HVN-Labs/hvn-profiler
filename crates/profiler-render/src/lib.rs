//! profiler-render — egui_plot wrappers + GPU-friendly trace storage.
//!
//! v0.1.0 introduces [`TraceStore`], a per-key ring buffer of `[t, value]`
//! points. Samples are pushed in via [`TraceStore::push`]; rendering code
//! calls [`TraceStore::points`] each frame to grab a slice for `egui_plot`.
//!
//! Retention is time-based (default 60 s). Old points are pruned lazily on
//! each `push` against the latest observed timestamp, so the store is
//! self-trimming without a background thread.

use std::collections::{BTreeSet, HashMap, VecDeque};

pub mod editor;
pub mod faults;
pub mod gen_panel;
pub mod generators;
pub mod panels;
pub mod view3d;
pub use editor::{
    apply_panel_draft, apply_trail_draft, categorize_key, collect_source_keys, compact_cells,
    group_source_keys, relocate_cell, remove_cell_at, replace_cell_at, swap_cells,
    ComboCollapseState, EditHistory, PanelDraft, TrailDraft, KEY_GROUPS, KNOWN_HVN_SITL_KEYS,
};
pub use faults::{
    default_drone_choices, render_faults_panel, FaultsPanelState, PendingCommand, SeenDrones,
};
pub use gen_panel::{render_gen_panel, GeneratorPanelState, SLIDER_TARGETS};
pub use generators::{Generator, Waveform};
pub use panels::{
    build_label_text, compute_overlay_pos, format_value_pub, layout_cell_rects,
    overlay_box_size, render_template_grid, render_template_grid_full,
    render_template_grid_with_override, responsive_cell_rects, responsive_grid_dims,
    CellMenuAction, GridRenderOptions, GridStats, LabelOverride, PanelState,
    RESPONSIVE_3D_COLLAPSE_W, RESPONSIVE_MIN_CELL_W, RESPONSIVE_SINGLE_COL_W,
};
pub use view3d::{render_view3d, render_view3d_with_override, OrbitCamera, View3dState, View3dStats};

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
    /// v0.11.0 — schema-only key set: channel names the envelope advertised
    /// with a `null` value (`dt_runner` does this for AP MAVLink mirrors
    /// until ArduPilot starts streaming). These keys have NO points stored
    /// in `traces`; they exist purely so the editor's source-key picker can
    /// surface them. As soon as a non-null value arrives, [`Self::push`]
    /// pulls the key out of this set and into `traces`.
    null_keys: BTreeSet<String>,
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
            null_keys: BTreeSet::new(),
        }
    }

    /// Push one point. `key` is the trace identifier (e.g. `"accel[0]"`).
    pub fn push(&mut self, t: f64, key: &str, value: f64) {
        if t > self.latest_ts {
            self.latest_ts = t;
        }
        // v0.11.0 — a real value supersedes any schema-only "null"
        // registration for this key.
        self.null_keys.remove(key);
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

    /// v0.11.0 — register a channel name observed with a `null` value.
    ///
    /// No-op if the key already has stored points (real data trumps the
    /// schema-only marker). Used by the App's drain path when the streamer
    /// emits e.g. `ap_attitude: null` so the editor's source-key picker can
    /// surface `ap_attitude` (and `ap_attitude[0..2]` after expansion in
    /// [`crate::collect_source_keys`]) before ArduPilot wakes up.
    pub fn note_null_key(&mut self, key: &str) {
        if self.traces.contains_key(key) {
            return;
        }
        if !self.null_keys.contains(key) {
            self.null_keys.insert(key.to_string());
        }
    }

    /// v0.11.0 — schema-only "null" key set: channels observed but never
    /// (yet) given a real value. The editor's source-key picker merges these
    /// into the dropdown so templates can be authored against
    /// dt_runner-emitted-as-None mirrors before they start streaming.
    pub fn null_keys(&self) -> &BTreeSet<String> {
        &self.null_keys
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

    /// Reconstruct a time-aligned vector trail from three scalar component
    /// keys, returning `(t, [x, y, z])` per sample.
    ///
    /// The three components are emitted from the same envelope, so their ring
    /// buffers are index- and timestamp-aligned. We index-align over the
    /// shortest of the three and take the timestamp from the first key. Used by
    /// the 3D view to rebuild `pos_*_ned` / `accel` / `quat` vectors out of the
    /// per-scalar store.
    pub fn vec3(&self, kx: &str, ky: &str, kz: &str) -> Vec<(f64, [f64; 3])> {
        let (ax, ay, az) = match (
            self.traces.get(kx),
            self.traces.get(ky),
            self.traces.get(kz),
        ) {
            (Some(x), Some(y), Some(z)) => (x, y, z),
            _ => return Vec::new(),
        };
        let n = ax.len().min(ay.len()).min(az.len());
        (0..n)
            .map(|i| (ax[i][0], [ax[i][1], ay[i][1], az[i][1]]))
            .collect()
    }

    /// Reconstruct a time-aligned 4-vector (e.g. a quaternion `w,x,y,z`) from
    /// four scalar component keys: `(t, [a, b, c, d])` per sample.
    pub fn vec4(&self, k0: &str, k1: &str, k2: &str, k3: &str) -> Vec<(f64, [f64; 4])> {
        let (a, b, c, d) = match (
            self.traces.get(k0),
            self.traces.get(k1),
            self.traces.get(k2),
            self.traces.get(k3),
        ) {
            (Some(a), Some(b), Some(c), Some(d)) => (a, b, c, d),
            _ => return Vec::new(),
        };
        let n = a.len().min(b.len()).min(c.len()).min(d.len());
        (0..n)
            .map(|i| (a[i][0], [a[i][1], b[i][1], c[i][1], d[i][1]]))
            .collect()
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
    fn vec3_reconstructs_aligned_vector() {
        let mut s = TraceStore::new(60.0);
        s.push(0.0, "p[0]", 1.0);
        s.push(0.0, "p[1]", 2.0);
        s.push(0.0, "p[2]", 3.0);
        s.push(0.1, "p[0]", 4.0);
        s.push(0.1, "p[1]", 5.0);
        s.push(0.1, "p[2]", 6.0);
        let v = s.vec3("p[0]", "p[1]", "p[2]");
        assert_eq!(v.len(), 2);
        assert_eq!(v[0], (0.0, [1.0, 2.0, 3.0]));
        assert_eq!(v[1], (0.1, [4.0, 5.0, 6.0]));
        // Missing key → empty.
        assert!(s.vec3("p[0]", "p[1]", "missing").is_empty());
    }

    #[test]
    fn vec4_reconstructs_quaternion() {
        let mut s = TraceStore::new(60.0);
        for (i, comp) in ["q[0]", "q[1]", "q[2]", "q[3]"].iter().enumerate() {
            s.push(0.0, comp, i as f64);
        }
        let q = s.vec4("q[0]", "q[1]", "q[2]", "q[3]");
        assert_eq!(q.len(), 1);
        assert_eq!(q[0], (0.0, [0.0, 1.0, 2.0, 3.0]));
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
