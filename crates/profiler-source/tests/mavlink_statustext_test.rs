//! v0.16.3 — `STATUSTEXT` rolling buffer. The decoder keeps the most
//! recent 8 entries in a shared `Arc<Mutex<VecDeque<TextLogEntry>>>` and
//! emits the full snapshot on every push, so cells reading `statustexts`
//! always see the current rolling tail.

#![cfg(feature = "mavlink-source")]

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use mavlink::dialects::ardupilotmega::{MavMessage, MavSeverity, STATUSTEXT_DATA};
use mavlink::types::CharArray;
use profiler_source::mavlink_source::decode_to_samples_with_state;
use profiler_source::{TextLogEntry, Value};

fn make_status(text: &str, severity: MavSeverity) -> MavMessage {
    let mut buf = [0u8; 50];
    let bytes = text.as_bytes();
    let len = bytes.len().min(50);
    buf[..len].copy_from_slice(&bytes[..len]);
    MavMessage::STATUSTEXT(STATUSTEXT_DATA {
        severity,
        text: CharArray::new(buf),
    })
}

#[test]
fn statustexts_buffer_keeps_only_last_eight() {
    let buf: Arc<Mutex<VecDeque<TextLogEntry>>> =
        Arc::new(Mutex::new(VecDeque::with_capacity(8)));

    // Push 12 entries (text00 … text11). Only text04..text11 should remain.
    let mut last_snapshot: Vec<TextLogEntry> = Vec::new();
    for i in 0..12 {
        let msg = make_status(&format!("text{i:02}"), MavSeverity::MAV_SEVERITY_INFO);
        let samples = decode_to_samples_with_state(&msg, i as f64, None, Some(&buf));
        assert_eq!(samples.len(), 1, "STATUSTEXT must produce exactly one sample");
        match &samples[0].value {
            Value::TextLog(entries) => {
                last_snapshot = entries.clone();
            }
            other => panic!("expected Value::TextLog, got {other:?}"),
        }
    }

    assert_eq!(last_snapshot.len(), 8, "rolling buffer should be capped at 8");
    let texts: Vec<&str> = last_snapshot.iter().map(|e| e.text.as_ref()).collect();
    assert_eq!(
        texts,
        vec![
            "text04", "text05", "text06", "text07",
            "text08", "text09", "text10", "text11",
        ],
        "oldest 4 entries should have been dropped"
    );
    // Severity preserved end-to-end.
    assert!(last_snapshot
        .iter()
        .all(|e| e.severity == MavSeverity::MAV_SEVERITY_INFO as u8));
}

#[test]
fn statustexts_without_buffer_emits_nothing() {
    // No state -> the decoder can't maintain history, so no sample.
    let msg = make_status("ignored", MavSeverity::MAV_SEVERITY_WARNING);
    let samples = decode_to_samples_with_state(&msg, 0.0, None, None);
    assert!(samples.is_empty());
}
