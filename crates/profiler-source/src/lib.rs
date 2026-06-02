//! profiler-source — `Source` trait + telemetry backends.
//!
//! v0.1.0 ships two backends:
//! - [`MockSource`] — synthetic sine wave (the v0.0.1 demo, now expressed
//!   as a `Source` impl).
//! - [`ZmqSource`] — subscribes to a ZMQ PUB endpoint, decodes msgpack
//!   envelopes shipped by `hvn_sitl.streamer`, flattens them into [`Sample`]s.
//!
//! ZMQ is implemented on the pure-Rust [`zeromq`] crate (no libzmq C dep,
//! works on Windows out of the box). Because that crate is async-only, we
//! spawn a dedicated `tokio` runtime in a background thread and bridge the
//! decoded samples back to the sync render loop via [`crossbeam_channel`].
//!
//! Later releases:
//! - v0.4.0: direct MAVLink over UDP (gated behind `mavlink-source`)
//! - later:  CSV / log-file replay

use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use serde::Deserialize;

pub mod zmq_source;
pub use zmq_source::{SeenDrones, ZmqSource};

#[cfg(feature = "mavlink-source")]
pub mod mavlink_source;
#[cfg(feature = "mavlink-source")]
pub use mavlink_source::{MavlinkOptions, MavlinkSource};

#[cfg(feature = "fault-channel")]
pub mod fault_publisher;
#[cfg(feature = "fault-channel")]
pub use fault_publisher::{encode_command, FaultCommand, FaultPublisher};

// ─── Sample / trait ────────────────────────────────────────────────────────

/// v0.11.0 — sentinel value carried in legacy scalar samples for schema-only
/// channels: ones the envelope advertised with a `null` payload (e.g. AP
/// MAVLink mirrors before ArduPilot streams). The render loop's drain path
/// recognises this sentinel (via [`Sample::is_schema_only`]) and routes the
/// key to `TraceStore::note_null_key` instead of `push`. NaN was chosen so
/// older consumers that ignore it simply drop the sample (NaN can't plot).
///
/// v0.13.0 — schema-only registrations are now first-class via
/// [`Value::Null`]; this constant is retained for backwards compatibility
/// with the older [`Value::Scalar`] form.
pub const SCHEMA_ONLY_SENTINEL: f64 = f64::NAN;

/// v0.13.0 — one entry in a [`Value::TextLog`] payload. Mirrors the
/// `statustexts` envelope entries from SITL v0.9.0+: a severity, payload
/// text and the timestamp the streamer assigned.
///
/// Decoupled from `profiler_render::TextLogEntry` so the source crate has
/// no render dependency. The CLI's drain loop converts between the two.
#[derive(Debug, Clone, PartialEq)]
pub struct TextLogEntry {
    /// MAVLink-style severity, 0 (emergency) → 7 (debug).
    pub severity: u8,
    /// Plain-text payload, refcounted so a single allocation is shared
    /// across the fanned-out copies of the rolling buffer.
    pub text: Arc<str>,
    /// Producer timestamp (seconds, monotonic).
    pub ts: f64,
}

/// v0.13.0 — rich payload type for a [`Sample`].
///
/// SITL v0.9.0 started shipping non-numeric values (string flight mode,
/// bool armed, list-of-dicts statustexts, integer-vector RC channels) in
/// the streamer envelope. The render layer (`Status` primitive,
/// `TraceStore::push_string` / `push_text_log`) was ready in v0.12.0 but
/// the data path was lossy — every non-scalar collapsed to a numeric
/// `Sample` or got dropped. v0.13.0 makes the wire→`TraceStore` path
/// preserve these types end to end.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// `f64` per timestamp — the common case (most channels are scalar).
    Scalar(f64),
    /// `Vec<f64>` — emitted by the decoder when the envelope's value is
    /// an array of numerics. Each element is fanned out as `<base>[i]`
    /// downstream OR routed to a single multi-line plot if the consumer
    /// prefers.
    Vector(Vec<f64>),
    /// `Vec<i64>` — emitted when EVERY element of an envelope array is
    /// an integer (`rc_channels`, `servo_outputs`, `sys_errors`). The
    /// `TraceStore` widens to `f64` for plotting; preserved as integers
    /// here so the editor can flag them as int-typed for chart-type
    /// inference.
    IntVector(Vec<i64>),
    /// String — refcounted so a single allocation services every clone
    /// of the sample (the `flight_mode` key fires every ~100 ms).
    String(Arc<str>),
    /// `True` / `False` — `armed`, future bool channels.
    Bool(bool),
    /// Rolling list of dicts (e.g. `statustexts`).
    TextLog(Vec<TextLogEntry>),
    /// Schema-only: the envelope advertised the channel name but supplied
    /// `null`. The render layer registers the key in its null-set so the
    /// editor's source-key picker can surface it, without polluting any
    /// trace buffer.
    Null,
}

impl Value {
    /// Convenience: read the inner scalar if this is a [`Value::Scalar`],
    /// or 0/1 if a [`Value::Bool`]. Useful for the few legacy call sites
    /// that still treat samples as opaque `f64`.
    pub fn as_scalar(&self) -> Option<f64> {
        match self {
            Value::Scalar(v) => Some(*v),
            Value::Bool(b) => Some(if *b { 1.0 } else { 0.0 }),
            _ => None,
        }
    }

    /// `true` when this value is the [`Value::Null`] schema-only marker.
    pub fn is_null(&self) -> bool {
        matches!(self, Value::Null)
    }
}

impl Sample {
    /// `true` when this sample is a schema-only registration (the envelope
    /// said the channel exists but had `null` for its value). Such samples
    /// MUST NOT be pushed into a numeric trace buffer — the App's drain
    /// path calls `TraceStore::note_null_key` for them so the editor can
    /// surface the key without plotting noise.
    pub fn is_schema_only(&self) -> bool {
        match &self.value {
            Value::Null => true,
            Value::Scalar(v) => v.is_nan(),
            _ => false,
        }
    }

    /// v0.13.0 — read the underlying scalar (`f64`) for legacy call sites.
    /// Returns `NaN` for non-numeric payloads so consumers that haven't
    /// migrated still get a "drop me" sentinel.
    pub fn scalar(&self) -> f64 {
        self.value.as_scalar().unwrap_or(f64::NAN)
    }

    /// v0.13.0 — convenience constructor for a scalar sample.
    pub fn new_scalar(ts: f64, key: impl Into<String>, value: f64, drone_name: Option<Arc<str>>) -> Self {
        Self {
            ts,
            key: key.into(),
            value: Value::Scalar(value),
            drone_name,
        }
    }
}

/// A single flattened telemetry sample. One envelope from the streamer
/// fans out into many `Sample`s (one per leaf of `values`).
///
/// v0.13.0 — `value` is now a [`Value`] enum: it can carry scalar,
/// vector, integer-vector, string, bool, text-log or null payloads. The
/// pre-v0.13.0 `Sample.value: f64` field migrates to `Value::Scalar(_)`;
/// the [`Sample::scalar`] / [`Sample::new_scalar`] helpers keep migration
/// of legacy call sites cheap.
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    /// Monotonic seconds since stream start (forwarded from the envelope).
    pub ts: f64,
    /// Trace identifier — e.g. `"accel[0]"`, `"ap_vfr_alt"`.
    pub key: String,
    /// v0.13.0 — typed payload. See [`Value`].
    pub value: Value,
    /// Drone name from the envelope (e.g. `"eric_1"`). `None` when the
    /// streamer didn't supply one (older streamer, MAVLink CLI without
    /// `--drone-name`, etc.). The profiler treats missing names as
    /// "unknown" rather than crashing.
    ///
    /// v0.10.1 — backed by `Arc<str>` so a single allocation is shared
    /// across every `Sample` in a flattened envelope / decoded MAVLink
    /// frame. On a 10 Hz × 5-drone × 6-samples/message fleet this
    /// eliminates ~300 short-lived String allocations per second on the
    /// MAVLink decode hot path.
    pub drone_name: Option<Arc<str>>,
}

/// A pull-based telemetry source. The render loop calls `try_recv` in a
/// tight loop each frame until it returns `None`.
pub trait Source: Send {
    /// Pop one sample, or `None` if nothing is buffered. Never blocks.
    fn try_recv(&mut self) -> Option<Sample>;

    /// Human-readable description for the status bar / window title.
    fn describe(&self) -> String;
}

/// Construct a source from a URI.
///
/// Supported schemes:
/// - `mock://`               — synthetic sine wave
/// - `zmq://host:port`       — subscribe to a ZMQ PUB streamer
/// - `mavlink://host:port`   — direct MAVLink UDP, **bind/listen** (`udpin`).
///   The default for real drones / ArduPilot SITL: the vehicle sends to us.
/// - `mavlinkout://host:port`— direct MAVLink UDP, **connect/send-first**
///   (`udpout`), for setups where the profiler must initiate.
///
/// The two `mavlink*` schemes require the `mavlink-source` feature (on by
/// default in the shipped binary).
pub fn from_uri(uri: &str) -> Result<Box<dyn Source>> {
    from_uri_with_options(uri, MavlinkConfig::default())
}

/// Profiler-side options that affect which sources are constructed.
#[derive(Debug, Clone, Default)]
pub struct MavlinkConfig {
    /// When `true`, opens MAVLink sources in passive listen-only mode (no
    /// HEARTBEAT sender, no `REQUEST_DATA_STREAM`). v0.4.0 behaviour.
    pub passive: bool,
    /// v0.10.0 — pin every MAVLink-source sample's `drone_name` to this string,
    /// overriding the default `sysid_<id>` demux. Useful when the operator
    /// knows there's only one vehicle on the link.
    pub drone_name_override: Option<String>,
}

/// Like [`from_uri`] but lets the caller pass [`MavlinkConfig`] (controls
/// the v0.8.0 active-GCS heartbeat / stream-request behaviour).
pub fn from_uri_with_options(uri: &str, cfg: MavlinkConfig) -> Result<Box<dyn Source>> {
    if uri == "mock://" || uri.starts_with("mock://") {
        Ok(Box::new(MockSource::default()))
    } else if let Some(rest) = uri.strip_prefix("mavlinkout://") {
        mavlink_from_addr("udpout", rest, cfg)
    } else if let Some(rest) = uri.strip_prefix("mavlink://") {
        mavlink_from_addr("udpin", rest, cfg)
    } else if let Some(rest) = uri.strip_prefix("zmq://") {
        // `host:port` → `tcp://host:port` for zeromq's connect string.
        let endpoint = format!("tcp://{}", rest.trim_end_matches('/'));
        let zmq = ZmqSource::connect(&endpoint)
            .with_context(|| format!("opening ZMQ source at {endpoint}"))?;
        Ok(Box::new(zmq))
    } else {
        log::warn!("Source '{uri}' not recognised — using mock://");
        Ok(Box::new(MockSource::default()))
    }
}

/// Build a [`MavlinkSource`] from a UDP `scheme` (`"udpin"` / `"udpout"`) and
/// a trailing `host:port`. Gated on the `mavlink-source` feature.
#[cfg(feature = "mavlink-source")]
fn mavlink_from_addr(scheme: &str, rest: &str, cfg: MavlinkConfig) -> Result<Box<dyn Source>> {
    let conn_str = format!("{scheme}:{}", rest.trim_end_matches('/'));
    let opts = MavlinkOptions {
        passive: cfg.passive,
        drone_name_override: cfg.drone_name_override.clone(),
    };
    let src = MavlinkSource::connect_with(&conn_str, opts)
        .with_context(|| format!("opening MAVLink source at {conn_str}"))?;
    Ok(Box::new(src))
}

/// Stub used when the `mavlink-source` feature is compiled out: surface a
/// clear error rather than silently falling back to `mock://`.
#[cfg(not(feature = "mavlink-source"))]
fn mavlink_from_addr(_scheme: &str, _rest: &str, _cfg: MavlinkConfig) -> Result<Box<dyn Source>> {
    anyhow::bail!(
        "this binary was built without the `mavlink-source` feature; \
         rebuild with `--features mavlink-source` to use mavlink:// sources"
    )
}

/// Like [`from_uri`], but also returns an optional [`SeenDrones`] handle
/// when the scheme supports drone-name discovery. Currently only `zmq://`
/// surfaces one; every other scheme returns `None` and the Faults panel
/// falls back to its default / CLI-supplied choices.
pub fn from_uri_with_discovery(uri: &str) -> Result<(Box<dyn Source>, Option<SeenDrones>)> {
    from_uri_with_discovery_opts(uri, MavlinkConfig::default())
}

/// Like [`from_uri_with_discovery`] with MAVLink-specific knobs (v0.8.0).
pub fn from_uri_with_discovery_opts(
    uri: &str,
    cfg: MavlinkConfig,
) -> Result<(Box<dyn Source>, Option<SeenDrones>)> {
    if let Some(rest) = uri.strip_prefix("zmq://") {
        let endpoint = format!("tcp://{}", rest.trim_end_matches('/'));
        let zmq = ZmqSource::connect(&endpoint)
            .with_context(|| format!("opening ZMQ source at {endpoint}"))?;
        let seen = zmq.seen_drones();
        Ok((Box::new(zmq), Some(seen)))
    } else {
        // Mock / MAVLink — no name discovery on the wire.
        let src = from_uri_with_options(uri, cfg)?;
        Ok((src, None))
    }
}

/// v0.9.0 — Multi-URI fan-in.
///
/// Each URI is opened with [`from_uri_with_discovery_opts`] and wrapped into a
/// single [`MultiSource`]. The returned [`SeenDrones`] is a SHARED set: every
/// underlying source whose scheme supports discovery writes into it, so the
/// Faults panel / view-drone dropdown see the union of names across all
/// sources.
///
/// Each sample is tagged with a synthetic `drone_name` derived from the
/// source URI (`"src1:host:port"`) when the underlying source didn't provide
/// one. Real ZMQ envelopes already include their drone name and are passed
/// through untouched.
pub fn multi_from_uris_with_discovery_opts(
    uris: &[String],
    cfg: MavlinkConfig,
) -> Result<(Box<dyn Source>, Option<SeenDrones>)> {
    use std::collections::HashSet;
    use std::sync::{Arc, RwLock};

    anyhow::ensure!(!uris.is_empty(), "at least one --source URI is required");

    // Single-source fast path — preserves v0.8.0 behaviour bit-for-bit. The
    // returned `SeenDrones` is whatever the single source surfaces (Arc-shared
    // with the worker thread, no copy / merging).
    if uris.len() == 1 {
        return from_uri_with_discovery_opts(&uris[0], cfg.clone());
    }

    let merged: SeenDrones = Arc::new(RwLock::new(HashSet::new()));
    let mut subs: Vec<MultiSubSource> = Vec::with_capacity(uris.len());

    for (i, uri) in uris.iter().enumerate() {
        let (src, seen_opt) = from_uri_with_discovery_opts(uri, cfg.clone())?;
        // Fallback drone name when the source has no native discovery (mock /
        // mavlink): derive from URI host:port (strip the scheme).
        let fallback_name: Arc<str> = Arc::from(fallback_drone_name_from_uri(uri, i));
        // Merge: spawn a tiny watcher to copy this sub's seen set into merged.
        // Cheap — only fires when new names appear (read-poll once per push).
        subs.push(MultiSubSource {
            inner: src,
            fallback_name,
            inner_seen: seen_opt,
            last_seen_len: 0,
            merged: Arc::clone(&merged),
        });
    }

    Ok((Box::new(MultiSource { subs, rr_cursor: 0 }), Some(merged)))
}

/// Derive a drone name from a source URI when the source has no native
/// discovery. `idx` is the 0-based position on the command line, used as a
/// last-resort tag (`"src0"`) for `mock://` which has no host/port.
fn fallback_drone_name_from_uri(uri: &str, idx: usize) -> String {
    let trimmed = uri
        .strip_prefix("zmq://")
        .or_else(|| uri.strip_prefix("mavlinkout://"))
        .or_else(|| uri.strip_prefix("mavlink://"))
        .unwrap_or(uri)
        .trim_end_matches('/');
    if trimmed.is_empty() || trimmed.starts_with("mock") {
        format!("src{idx}")
    } else {
        // Replace `:` with `_` for readability ("127.0.0.1_9005").
        trimmed.replace(':', "_")
    }
}

/// One leg of a [`MultiSource`]: the underlying [`Source`], a fallback
/// drone-name to stamp on samples that arrive without one, and an
/// optional handle to the leg's own `SeenDrones` set (re-published into the
/// merged set on every push).
struct MultiSubSource {
    inner: Box<dyn Source>,
    /// Stamped into samples whose `drone_name` is `None` (mock, mavlink).
    /// For ZMQ legs the envelope's own name wins. Held as `Arc<str>` so a
    /// single allocation services every fallback-stamped sample for the
    /// life of the leg.
    fallback_name: Arc<str>,
    /// The leg's own discovery set (`ZmqSource::seen_drones()` clone) when
    /// available. Polled on each push and the new entries unioned into
    /// `merged`.
    inner_seen: Option<SeenDrones>,
    /// v0.10.1 — last observed `inner_seen.len()`. The MultiSource only takes
    /// a write-lock on `merged` and copies entries when the leg's set has
    /// actually grown. Without this we re-cloned + write-locked on every
    /// sample (300+ write-locks/sec on a 10 Hz × 5-drone fleet).
    last_seen_len: usize,
    /// Shared merged set across all legs of the parent [`MultiSource`].
    merged: SeenDrones,
}

/// v0.9.0 — fan-in over multiple [`Source`] backends.
///
/// Round-robin drain: each call to [`Source::try_recv`] starts at a rotating
/// cursor and walks each leg until one yields a sample. Cursor advances by
/// one position per yielded sample (fair-merge). `None` when every leg is
/// empty in this drain cycle.
///
/// Each sample is post-processed:
/// 1. If `drone_name` is `None`, stamp the leg's `fallback_name`.
/// 2. If the leg has a native `SeenDrones`, union new entries into the
///    multi-source's `merged` set so the toolbar dropdown sees them.
/// 3. If the sample carries a name, ensure that name is in `merged`.
pub struct MultiSource {
    subs: Vec<MultiSubSource>,
    /// Round-robin cursor — the leg we try first on the next call.
    rr_cursor: usize,
}

impl Source for MultiSource {
    fn try_recv(&mut self) -> Option<Sample> {
        let n = self.subs.len();
        if n == 0 {
            return None;
        }
        // Walk all legs once, starting at the cursor.
        for offset in 0..n {
            let i = (self.rr_cursor + offset) % n;
            let leg = &mut self.subs[i];
            if let Some(mut s) = leg.inner.try_recv() {
                // Stamp the fallback name when the leg didn't supply one.
                if s.drone_name.is_none() {
                    s.drone_name = Some(Arc::clone(&leg.fallback_name));
                }
                // Union into merged. Fast path: read-lock first.
                if let Some(name) = &s.drone_name {
                    let known = leg
                        .merged
                        .read()
                        .map(|g| g.contains(name.as_ref()))
                        .unwrap_or(true);
                    if !known {
                        if let Ok(mut g) = leg.merged.write() {
                            g.insert(name.to_string());
                        }
                    }
                }
                // Also drain the leg's own native discovery set into merged
                // (covers names that arrived via samples we never saw because
                // a different leg yielded first). v0.10.1 — only re-walk the
                // set when it has actually grown since our last visit; this
                // keeps the hot path lock-free for the steady state where the
                // drone roster is stable.
                if let Some(inner) = &leg.inner_seen {
                    let cur_len = inner.read().map(|g| g.len()).unwrap_or(0);
                    if cur_len > leg.last_seen_len {
                        let to_add: Vec<String> = inner
                            .read()
                            .map(|g| g.iter().cloned().collect())
                            .unwrap_or_default();
                        if !to_add.is_empty() {
                            if let Ok(mut m) = leg.merged.write() {
                                for n in to_add {
                                    m.insert(n);
                                }
                            }
                        }
                        leg.last_seen_len = cur_len;
                    }
                }
                self.rr_cursor = (i + 1) % n;
                return Some(s);
            }
        }
        None
    }

    fn describe(&self) -> String {
        let parts: Vec<String> = self
            .subs
            .iter()
            .map(|s| s.inner.describe())
            .collect();
        format!("multi:[{}]", parts.join(" + "))
    }
}

// ─── MockSource ────────────────────────────────────────────────────────────

/// Synthetic sine-wave source. Used for the v0.0.1 toolchain-proof demo and
/// for tests that don't want a real network dependency.
///
/// Emits points at a steady ~60 Hz (one per `try_recv` call once `next_due`
/// has elapsed), so the render loop's `try_recv → push` cycle exercises the
/// same code path the ZMQ backend uses.
pub struct MockSource {
    started: Instant,
    last_emit: Option<Instant>,
    period: Duration,
}

impl Default for MockSource {
    fn default() -> Self {
        Self {
            started: Instant::now(),
            last_emit: None,
            period: Duration::from_micros(16_666), // ~60 Hz
        }
    }
}

impl Source for MockSource {
    fn try_recv(&mut self) -> Option<Sample> {
        let now = Instant::now();
        if let Some(last) = self.last_emit {
            if now.duration_since(last) < self.period {
                return None;
            }
        }
        self.last_emit = Some(now);
        let t = now.duration_since(self.started).as_secs_f64();
        Some(Sample::new_scalar(
            t,
            "mock.sine",
            (t * std::f64::consts::TAU * 0.5).sin(),
            None,
        ))
    }

    fn describe(&self) -> String {
        "mock:// (synthetic sine, ~60 Hz)".to_string()
    }
}

// ─── Envelope + flatten ────────────────────────────────────────────────────

/// Streamer wire envelope. `values` is intentionally dynamic so we can keep
/// up with the SITL schema without recompiling: every scalar / array leaf is
/// flattened by [`flatten_envelope`].
#[derive(Debug, Clone, Deserialize)]
pub struct Envelope {
    pub ts: f64,
    #[serde(default)]
    pub source: String,
    /// Producing drone (e.g. `"eric_1"`). Added in v0.7.0 / SITL v0.7.18.4
    /// for Faults-panel target discovery. `None` when the streamer didn't
    /// supply one (older SITL versions, MAVLink CLI without `--drone-name`).
    #[serde(default)]
    pub drone_name: Option<String>,
    /// Flat-ish map of channel name → scalar | array | null. Stored as a raw
    /// `rmpv::Value` so we can flatten dynamically without a static schema.
    pub values: rmpv::Value,
}

/// Decode a msgpack-encoded envelope (raw bytes off the ZMQ socket) into
/// a stream of [`Sample`]s.
///
/// Flattening rules — match the streamer's wire schema:
/// - Scalars (`f64`, `i64`, `u64`, `bool`) → one `Sample { key, value }`.
///   `bool` becomes `0.0` / `1.0`.
/// - Arrays of scalars → one `Sample` per element with key `"<base>[i]"`.
/// - `null` / Nil values → dropped silently (the streamer emits `None`
///   for sensors that haven't reported yet).
/// - Nested maps / arrays-of-arrays → currently dropped (no SITL key uses them).
pub fn flatten_msgpack(bytes: &[u8]) -> Result<Vec<Sample>> {
    let env: Envelope = rmp_serde::from_slice(bytes).context("decoding msgpack envelope")?;
    Ok(flatten_envelope(&env))
}

/// v0.11.0 — variant of [`flatten_msgpack`] that ALSO returns the list of
/// channel names whose values came in as `null` (top-level `Nil`).
///
/// Used by the App's drain path to register schema-only keys with
/// `TraceStore::note_null_key`, so the editor's source-key picker surfaces
/// channels (e.g. `ap_attitude`) that the streamer announces but ArduPilot
/// hasn't yet populated.
///
/// Only TOP-LEVEL nulls are reported — null elements inside an array still
/// drop silently (the array's other indices come through as samples).
pub fn flatten_msgpack_with_nulls(bytes: &[u8]) -> Result<(Vec<Sample>, Vec<String>)> {
    let env: Envelope = rmp_serde::from_slice(bytes).context("decoding msgpack envelope")?;
    Ok(flatten_envelope_with_nulls(&env))
}

/// Flatten an already-decoded envelope. Split out from [`flatten_msgpack`]
/// so unit tests can exercise the schema logic without round-tripping bytes.
///
/// Every emitted [`Sample`] inherits `env.drone_name` so downstream consumers
/// (the `ZmqSource` seen-drones set, the Faults panel target dropdown) can
/// trace each scalar back to the producing sim without re-decoding.
pub fn flatten_envelope(env: &Envelope) -> Vec<Sample> {
    flatten_envelope_with_nulls(env).0
}

/// v0.11.0 — like [`flatten_envelope`] but ALSO returns the names of channels
/// whose top-level value was `null`. See [`flatten_msgpack_with_nulls`].
///
/// v0.13.0 — non-numeric leaves (string / bool / list-of-dicts) are now
/// preserved through the decoder via the new [`Value`] variants instead of
/// being silently dropped:
/// - `Boolean` → `Value::Bool`
/// - `String` → `Value::String`
/// - Array of integers → `Value::IntVector` (one sample, not N fanned out)
/// - Array of mixed numerics → `Value::Vector` (one sample, not N fanned out)
/// - Array of dicts shaped like statustext entries → `Value::TextLog`
///
/// Numeric scalars and "legacy" fanned-out `base[i]` arrays still emit
/// per-component scalar samples to preserve the v0.12.0 plot wiring;
/// non-numeric vectors emit a single sample whose `value` is the whole
/// vector so downstream consumers can decide how to render it.
pub fn flatten_envelope_with_nulls(env: &Envelope) -> (Vec<Sample>, Vec<String>) {
    let mut out = Vec::new();
    let mut nulls = Vec::new();
    let ts = env.ts;
    // v0.10.1 — one Arc<str> allocation, cloned (refcount bump) per sample.
    let drone_name: Option<Arc<str>> = env.drone_name.as_deref().map(Arc::from);
    let map = match env.values.as_map() {
        Some(m) => m,
        None => return (out, nulls),
    };
    for (k, v) in map {
        let key = match k.as_str() {
            Some(s) => s,
            None => continue, // streamer always uses string keys; skip otherwise.
        };
        match v {
            rmpv::Value::Nil => {
                // v0.11.0 — surface the channel name for schema-only
                // registration in the editor's source-key picker.
                nulls.push(key.to_string());
                continue;
            }
            rmpv::Value::Boolean(b) => out.push(Sample {
                ts,
                key: key.to_string(),
                value: Value::Bool(*b),
                drone_name: drone_name.clone(),
            }),
            rmpv::Value::Integer(i) => {
                if let Some(f) = i.as_f64() {
                    out.push(Sample::new_scalar(ts, key, f, drone_name.clone()));
                }
            }
            rmpv::Value::F32(f) => {
                out.push(Sample::new_scalar(ts, key, *f as f64, drone_name.clone()));
            }
            rmpv::Value::F64(f) => {
                out.push(Sample::new_scalar(ts, key, *f, drone_name.clone()));
            }
            rmpv::Value::String(s) => {
                // v0.13.0 — preserve the string instead of dropping. The
                // streamer's `String` form is always UTF-8; we still call
                // `as_str()` and skip on the (unreachable) failure rather
                // than panic.
                if let Some(text) = s.as_str() {
                    out.push(Sample {
                        ts,
                        key: key.to_string(),
                        value: Value::String(Arc::from(text)),
                        drone_name: drone_name.clone(),
                    });
                }
            }
            rmpv::Value::Array(arr) => {
                // v0.13.0 — classify the array's shape before fanning out:
                // 1. List-of-dicts shaped like statustext entries → TextLog.
                // 2. All elements integer → IntVector (single sample).
                // 3. All elements numeric, at least one float → Vector (single sample).
                // 4. Mixed / contains non-numeric → fall back to legacy
                //    per-component scalar fan-out (preserves the v0.12.0
                //    `base[i]` plot wiring for `accel`, `ap_attitude`, …).
                if let Some(entries) = try_decode_text_log(arr) {
                    out.push(Sample {
                        ts,
                        key: key.to_string(),
                        value: Value::TextLog(entries),
                        drone_name: drone_name.clone(),
                    });
                    continue;
                }
                if is_int_only_array(arr) {
                    let ints: Vec<i64> = arr
                        .iter()
                        .filter_map(|e| match e {
                            rmpv::Value::Integer(i) => i.as_i64(),
                            _ => None,
                        })
                        .collect();
                    // v0.13.0 — single sample carrying the integer vector,
                    // PLUS the legacy per-index scalar fan-out so existing
                    // templates that bind to `rc_channels[0..15]` etc. keep
                    // plotting without a schema migration.
                    out.push(Sample {
                        ts,
                        key: key.to_string(),
                        value: Value::IntVector(ints.clone()),
                        drone_name: drone_name.clone(),
                    });
                    for (i, v) in ints.iter().enumerate() {
                        out.push(Sample::new_scalar(
                            ts,
                            format!("{key}[{i}]"),
                            *v as f64,
                            drone_name.clone(),
                        ));
                    }
                    continue;
                }
                if is_numeric_array(arr) {
                    let floats: Vec<f64> = arr.iter().filter_map(scalar_to_f64).collect();
                    out.push(Sample {
                        ts,
                        key: key.to_string(),
                        value: Value::Vector(floats),
                        drone_name: drone_name.clone(),
                    });
                    // Legacy per-component scalar fan-out (unchanged).
                    for (i, elt) in arr.iter().enumerate() {
                        if let Some(v) = scalar_to_f64(elt) {
                            out.push(Sample::new_scalar(
                                ts,
                                format!("{key}[{i}]"),
                                v,
                                drone_name.clone(),
                            ));
                        }
                    }
                    continue;
                }
                // Mixed / partly-non-numeric: emit only the salvageable
                // numeric components, preserving prior v0.11.0 behaviour.
                for (i, elt) in arr.iter().enumerate() {
                    if let Some(v) = scalar_to_f64(elt) {
                        out.push(Sample::new_scalar(
                            ts,
                            format!("{key}[{i}]"),
                            v,
                            drone_name.clone(),
                        ));
                    }
                    // non-scalar / null elements drop silently
                }
            }
            // Nested maps, binary blobs etc. aren't plottable.
            _ => continue,
        }
    }
    (out, nulls)
}

/// v0.13.0 — heuristic: `true` when every element of `arr` is an integer.
fn is_int_only_array(arr: &[rmpv::Value]) -> bool {
    !arr.is_empty()
        && arr
            .iter()
            .all(|e| matches!(e, rmpv::Value::Integer(_)))
}

/// v0.13.0 — heuristic: `true` when every element of `arr` is numeric
/// (`Integer` | `F32` | `F64` | `Boolean`).
fn is_numeric_array(arr: &[rmpv::Value]) -> bool {
    !arr.is_empty()
        && arr.iter().all(|e| {
            matches!(
                e,
                rmpv::Value::Integer(_)
                    | rmpv::Value::F32(_)
                    | rmpv::Value::F64(_)
                    | rmpv::Value::Boolean(_)
            )
        })
}

/// v0.13.0 — try to decode `arr` as a list of `{severity, text, ts}` dicts.
///
/// Returns `Some(entries)` when EVERY element is a map containing all three
/// expected keys; otherwise `None` (the array is treated as a numeric or
/// mixed list by the caller). Matches the wire form of SITL v0.9.0's
/// `statustexts` envelope key.
fn try_decode_text_log(arr: &[rmpv::Value]) -> Option<Vec<TextLogEntry>> {
    if arr.is_empty() {
        return None;
    }
    let mut out = Vec::with_capacity(arr.len());
    for elt in arr {
        let map = elt.as_map()?;
        let mut severity: Option<u8> = None;
        let mut text: Option<Arc<str>> = None;
        let mut ts: Option<f64> = None;
        for (k, v) in map {
            match k.as_str()? {
                "severity" => {
                    severity = match v {
                        rmpv::Value::Integer(i) => i.as_u64().map(|n| n as u8),
                        _ => None,
                    };
                }
                "text" => {
                    text = match v {
                        rmpv::Value::String(s) => s.as_str().map(Arc::from),
                        _ => None,
                    };
                }
                "ts" => {
                    ts = scalar_to_f64(v);
                }
                _ => {}
            }
        }
        let severity = severity?;
        let text = text?;
        let ts = ts?;
        out.push(TextLogEntry { severity, text, ts });
    }
    Some(out)
}

fn scalar_to_f64(v: &rmpv::Value) -> Option<f64> {
    match v {
        rmpv::Value::Nil => None,
        rmpv::Value::Boolean(b) => Some(if *b { 1.0 } else { 0.0 }),
        rmpv::Value::Integer(i) => i.as_f64(),
        rmpv::Value::F32(f) => Some(*f as f64),
        rmpv::Value::F64(f) => Some(*f),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Serialize;
    use std::collections::BTreeMap;

    #[test]
    fn mock_emits_samples() {
        let mut m = MockSource::default();
        // First call always yields a sample.
        let s = m.try_recv().expect("first call should yield");
        assert_eq!(s.key, "mock.sine");
        // Second call immediately after returns None (rate-limited).
        assert!(m.try_recv().is_none());
    }

    #[test]
    fn from_uri_mock() {
        let mut s = from_uri("mock://").unwrap();
        assert!(s.try_recv().is_some());
    }

    /// Bench against the streamer schema: `accel` (3-vec) + `ap_vfr_alt`
    /// (scalar) + `skip_me` (null). Expect three `accel[i]` samples plus
    /// the scalar, with the null silently dropped.
    #[test]
    fn flatten_matches_streamer_schema() {
        // Build a msgpack-encoded envelope by hand so the test exercises
        // the same path the ZMQ backend will take.
        #[derive(Serialize)]
        struct Env {
            ts: f64,
            source: String,
            values: BTreeMap<String, serde_json::Value>,
        }
        let mut values = BTreeMap::new();
        values.insert(
            "accel".into(),
            serde_json::json!([1.0_f64, 2.0_f64, 3.0_f64]),
        );
        values.insert("ap_vfr_alt".into(), serde_json::json!(4.5_f64));
        values.insert("skip_me".into(), serde_json::Value::Null);
        let env = Env {
            ts: 12.5,
            source: "dt".into(),
            values,
        };
        let bytes = rmp_serde::to_vec_named(&env).expect("encode");

        let mut samples = flatten_msgpack(&bytes).expect("decode");
        // Sort so the assertion doesn't depend on map iteration order.
        samples.sort_by(|a, b| a.key.cmp(&b.key));

        // v0.13.0 — array values now emit BOTH a single vector sample
        // (`Value::Vector([1.0, 2.0, 3.0])` for `accel`) AND the legacy
        // per-index scalar fan-out (`accel[0..2]`). The test pins the
        // latter contract since downstream templates rely on it.
        let got: Vec<(String, f64)> = samples
            .into_iter()
            .filter_map(|s| {
                assert_eq!(s.ts, 12.5);
                s.value.as_scalar().map(|v| (s.key, v))
            })
            .collect();
        assert_eq!(
            got,
            vec![
                ("accel[0]".into(), 1.0),
                ("accel[1]".into(), 2.0),
                ("accel[2]".into(), 3.0),
                ("ap_vfr_alt".into(), 4.5),
            ]
        );
    }

    #[test]
    fn flatten_drops_null_array_elements() {
        #[derive(Serialize)]
        struct Env {
            ts: f64,
            values: BTreeMap<String, serde_json::Value>,
        }
        let mut values = BTreeMap::new();
        // Mixed array: middle element is null and should silently drop.
        values.insert(
            "mixed".into(),
            serde_json::json!([1.0_f64, serde_json::Value::Null, 3.0_f64]),
        );
        let env = Env { ts: 0.0, values };
        let bytes = rmp_serde::to_vec_named(&env).unwrap();
        // v0.13.0 — only keep scalar samples (the array also produces a
        // vector sample, but we only assert on the per-component contract).
        let mut s: Vec<_> = flatten_msgpack(&bytes)
            .unwrap()
            .into_iter()
            .filter(|s| s.value.as_scalar().is_some())
            .collect();
        s.sort_by(|a, b| a.key.cmp(&b.key));
        assert_eq!(
            s.iter().map(|s| s.key.as_str()).collect::<Vec<_>>(),
            vec!["mixed[0]", "mixed[2]"],
        );
    }
}
