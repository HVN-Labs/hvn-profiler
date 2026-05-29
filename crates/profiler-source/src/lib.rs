//! profiler-source — `Source` trait + telemetry backends.
//!
//! v0.0.1 only ships the `Mock` backend (sine wave). Real backends land in
//! subsequent releases:
//! - v0.1.0: ZMQ msgpack (pure-Rust `zeromq` crate so Windows doesn't need libzmq)
//! - v0.4.0: direct MAVLink over UDP (gated behind the `mavlink-source` feature)
//! - later:  CSV / log-file replay

use anyhow::Result;
use serde::{Deserialize, Serialize};

/// A single telemetry sample. Channel naming follows the SITL streamer schema
/// (e.g. `"ATT.Roll"`, `"BAT.Volt"`); semantics are decided by the template.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Sample {
    /// Wall-clock or vehicle-time timestamp, seconds.
    pub t: f64,
    /// Channel name. Stable identifier the template binds plots against.
    pub channel: String,
    /// Sample value.
    pub value: f64,
}

/// A pull-based telemetry source. Backends drain whatever they have buffered
/// each frame; the renderer handles pacing.
pub trait Source: Send {
    /// Drain any samples that have arrived since the previous call.
    fn poll(&mut self) -> Result<Vec<Sample>>;

    /// Human-readable description for the status bar.
    fn describe(&self) -> String;
}

/// Construct a source from a URI like `mock://`, `zmq://host:port`,
/// `mavlink://host:port`, or `csv://path`. v0.0.1 only handles `mock://`.
pub fn from_uri(uri: &str) -> Result<Box<dyn Source>> {
    if uri == "mock://" || uri.starts_with("mock://") {
        Ok(Box::new(Mock::default()))
    } else {
        // v0.0.1 stub — fall back to the mock backend with a warning.
        log::warn!("Source '{uri}' not yet implemented (v0.0.1) — using mock://");
        Ok(Box::new(Mock::default()))
    }
}

/// Mock backend — emits a synthetic sine wave. Used by the v0.0.1 demo and
/// for headless tests.
#[derive(Debug, Default)]
pub struct Mock {
    t: f64,
}

impl Source for Mock {
    fn poll(&mut self) -> Result<Vec<Sample>> {
        // 50-sample burst per poll, 1 ms apart.
        let mut out = Vec::with_capacity(50);
        for _ in 0..50 {
            self.t += 0.001;
            out.push(Sample {
                t: self.t,
                channel: "MOCK.sine".to_string(),
                value: (self.t * std::f64::consts::TAU * 0.5).sin(),
            });
        }
        Ok(out)
    }

    fn describe(&self) -> String {
        "mock:// (synthetic sine, 1 kHz)".to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_emits_samples() {
        let mut m = Mock::default();
        let s = m.poll().unwrap();
        assert_eq!(s.len(), 50);
        assert_eq!(s[0].channel, "MOCK.sine");
    }

    #[test]
    fn from_uri_mock() {
        let mut s = from_uri("mock://").unwrap();
        assert!(!s.poll().unwrap().is_empty());
    }
}
