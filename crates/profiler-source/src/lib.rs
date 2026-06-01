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

use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use serde::Deserialize;

pub mod zmq_source;
pub use zmq_source::ZmqSource;

#[cfg(feature = "mavlink-source")]
pub mod mavlink_source;
#[cfg(feature = "mavlink-source")]
pub use mavlink_source::MavlinkSource;

#[cfg(feature = "fault-channel")]
pub mod fault_publisher;
#[cfg(feature = "fault-channel")]
pub use fault_publisher::{encode_command, FaultCommand, FaultPublisher};

// ─── Sample / trait ────────────────────────────────────────────────────────

/// A single flattened telemetry sample. One envelope from the streamer
/// fans out into many `Sample`s (one per scalar leaf of `values`).
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    /// Monotonic seconds since stream start (forwarded from the envelope).
    pub ts: f64,
    /// Trace identifier — e.g. `"accel[0]"`, `"ap_vfr_alt"`.
    pub key: String,
    /// Scalar value.
    pub value: f64,
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
    if uri == "mock://" || uri.starts_with("mock://") {
        Ok(Box::new(MockSource::default()))
    } else if let Some(rest) = uri.strip_prefix("mavlinkout://") {
        mavlink_from_addr("udpout", rest)
    } else if let Some(rest) = uri.strip_prefix("mavlink://") {
        mavlink_from_addr("udpin", rest)
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
fn mavlink_from_addr(scheme: &str, rest: &str) -> Result<Box<dyn Source>> {
    let conn_str = format!("{scheme}:{}", rest.trim_end_matches('/'));
    let src = MavlinkSource::connect(&conn_str)
        .with_context(|| format!("opening MAVLink source at {conn_str}"))?;
    Ok(Box::new(src))
}

/// Stub used when the `mavlink-source` feature is compiled out: surface a
/// clear error rather than silently falling back to `mock://`.
#[cfg(not(feature = "mavlink-source"))]
fn mavlink_from_addr(_scheme: &str, _rest: &str) -> Result<Box<dyn Source>> {
    anyhow::bail!(
        "this binary was built without the `mavlink-source` feature; \
         rebuild with `--features mavlink-source` to use mavlink:// sources"
    )
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
        Some(Sample {
            ts: t,
            key: "mock.sine".to_string(),
            value: (t * std::f64::consts::TAU * 0.5).sin(),
        })
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

/// Flatten an already-decoded envelope. Split out from [`flatten_msgpack`]
/// so unit tests can exercise the schema logic without round-tripping bytes.
pub fn flatten_envelope(env: &Envelope) -> Vec<Sample> {
    let mut out = Vec::new();
    let ts = env.ts;
    let map = match env.values.as_map() {
        Some(m) => m,
        None => return out,
    };
    for (k, v) in map {
        let key = match k.as_str() {
            Some(s) => s,
            None => continue, // streamer always uses string keys; skip otherwise.
        };
        match v {
            rmpv::Value::Nil => continue,
            rmpv::Value::Boolean(b) => out.push(Sample {
                ts,
                key: key.to_string(),
                value: if *b { 1.0 } else { 0.0 },
            }),
            rmpv::Value::Integer(i) => {
                if let Some(f) = i.as_f64() {
                    out.push(Sample {
                        ts,
                        key: key.to_string(),
                        value: f,
                    });
                }
            }
            rmpv::Value::F32(f) => out.push(Sample {
                ts,
                key: key.to_string(),
                value: *f as f64,
            }),
            rmpv::Value::F64(f) => out.push(Sample {
                ts,
                key: key.to_string(),
                value: *f,
            }),
            rmpv::Value::Array(arr) => {
                for (i, elt) in arr.iter().enumerate() {
                    if let Some(v) = scalar_to_f64(elt) {
                        out.push(Sample {
                            ts,
                            key: format!("{key}[{i}]"),
                            value: v,
                        });
                    }
                    // non-scalar / null elements drop silently
                }
            }
            // Strings, nested maps, binary blobs etc. aren't plottable.
            _ => continue,
        }
    }
    out
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

        let got: Vec<(String, f64)> = samples
            .into_iter()
            .map(|s| {
                assert_eq!(s.ts, 12.5);
                (s.key, s.value)
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
        let mut s = flatten_msgpack(&bytes).unwrap();
        s.sort_by(|a, b| a.key.cmp(&b.key));
        assert_eq!(
            s.iter().map(|s| s.key.as_str()).collect::<Vec<_>>(),
            vec!["mixed[0]", "mixed[2]"],
        );
    }
}
