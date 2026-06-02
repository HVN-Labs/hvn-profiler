//! v0.14.0 — `StatusKind::EkfFlags` decodes the ArduPilot `EKF_STATUS_REPORT`
//! `flags` bitfield into a 12-row colored-dot list.
//!
//! These tests pin the pure decoder ([`decode_ekf_flags`]) plus the label
//! ordering ([`EKF_FLAG_LABELS`]) — the painter itself needs an egui context
//! but reads the same decoder under the hood.

use profiler_render::{decode_ekf_flags, EKF_FLAG_LABELS};

#[test]
fn decode_0b11111_sets_first_five_flags() {
    let rows = decode_ekf_flags(0b11111);
    assert_eq!(rows.len(), EKF_FLAG_LABELS.len());
    // First 5 bits set.
    for (i, (label, is_set)) in rows.iter().take(5).enumerate() {
        assert_eq!(*label, EKF_FLAG_LABELS[i]);
        assert!(*is_set, "bit {i} must be set (label={label})");
    }
    // Remaining bits unset.
    for (i, (label, is_set)) in rows.iter().enumerate().skip(5) {
        assert!(
            !*is_set,
            "bit {i} must be unset (label={label})",
        );
    }
    // Sanity: labels match ArduPilot order.
    assert_eq!(rows[0].0, "ATTITUDE");
    assert_eq!(rows[1].0, "VELOCITY_HORIZ");
    assert_eq!(rows[2].0, "VELOCITY_VERT");
    assert_eq!(rows[3].0, "POS_HORIZ_REL");
    assert_eq!(rows[4].0, "POS_HORIZ_ABS");
}

#[test]
fn decode_zero_clears_every_flag() {
    let rows = decode_ekf_flags(0);
    assert_eq!(rows.len(), EKF_FLAG_LABELS.len());
    for (label, is_set) in &rows {
        assert!(!*is_set, "{label} must be unset for flags=0");
    }
}

#[test]
fn decode_bad_value_negative_renders_as_all_unset() {
    // The renderer guards `f64 < 0.0` and feeds the decoder `0` in that
    // case. We mirror that contract here: passing the post-guard value `0`
    // produces an all-unset row list — same as the "no data yet" path —
    // rather than panicking or producing garbage labels.
    let rows = decode_ekf_flags(0);
    assert!(rows.iter().all(|(_, set)| !*set));
    // Also: a very large u32 (e.g. all-bits-set) is decoded WITHOUT panicking
    // — we only define labels for the lower 12 bits, and the higher bits
    // are silently ignored (since we iterate the label table).
    let rows = decode_ekf_flags(u32::MAX);
    assert_eq!(rows.len(), EKF_FLAG_LABELS.len());
    for (label, is_set) in &rows {
        assert!(*is_set, "{label} must be set for flags=0xFFFFFFFF");
    }
}
