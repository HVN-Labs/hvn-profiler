//! v0.11.0 — the editor's source-key picker shows the standard HVN-SITL
//! key vocabulary BEFORE any envelope arrives so users can author panel
//! templates against AP-fed channels (`ap_attitude`, `ap_raw_imu`, …)
//! while ArduPilot is still booting.
//!
//! Pre-v0.11.0, `collect_source_keys` only registered a channel once a
//! non-`None` value flowed; the picker was empty (or missing every AP
//! mirror) for the first 5–20 s of every fleet startup.
//!
//! These tests pin the contract:
//! 1. With ZERO observed samples, the picker offers every key in
//!    [`KNOWN_HVN_SITL_KEYS`] — including AP MAVLink mirrors.
//! 2. Vector BASES (e.g. `ap_attitude` for `ap_attitude[0..2]`) are also
//!    in the picker so the operator can pick a Vector primitive without
//!    knowing the indexed form.
//! 3. Schema-only "null" keys observed at runtime are surfaced too —
//!    custom dialects beyond the static list get discovered on the fly.

use profiler_render::{collect_source_keys, TraceStore, KNOWN_HVN_SITL_KEYS};

/// With NO samples ever pushed, `collect_source_keys` returns at minimum
/// every entry in `KNOWN_HVN_SITL_KEYS` — the AP MAVLink mirrors that
/// trigger the user complaint are all present.
#[test]
fn empty_stores_yield_known_hvn_sitl_keys() {
    let stores: Vec<TraceStore> = Vec::new();
    let keys = collect_source_keys(stores.iter());
    let key_set: std::collections::BTreeSet<String> = keys.into_iter().collect();

    // Every static key must be present.
    for &(k, _shape) in KNOWN_HVN_SITL_KEYS {
        assert!(
            key_set.contains(k),
            "picker missing static key {k}; v0.11.0 promised pre-seed",
        );
    }
}

/// The specific keys called out in the user complaint (AP MAVLink mirrors
/// + EKF position) are present at startup with zero samples.
#[test]
fn ap_mavlink_keys_appear_at_startup_with_zero_samples() {
    let keys = collect_source_keys(std::iter::empty::<&TraceStore>());
    let set: std::collections::BTreeSet<String> = keys.into_iter().collect();

    for required in [
        "ap_attitude[0]",
        "ap_attitude[1]",
        "ap_attitude[2]",
        "ap_raw_imu[0]",
        "ap_raw_imu[5]",
        "ap_vfr_alt",
        "ap_vel_ned[0]",
        "pos_ekf_ned[0]",
        "pos_ekf_ned[1]",
        "pos_ekf_ned[2]",
        "pos_truth_ned[0]",
        "pos_target_ned[0]",
    ] {
        assert!(
            set.contains(required),
            "AP MAVLink / position key {required} should be addressable at startup",
        );
    }
}

/// Vector BASES are derived from indexed forms so `Vector` /
/// `Magnitude` / `AttitudeRpy` primitives can be added without knowing the
/// wire indexing.
#[test]
fn known_keys_include_vector_bases() {
    let keys = collect_source_keys(std::iter::empty::<&TraceStore>());
    let set: std::collections::BTreeSet<String> = keys.into_iter().collect();

    for base in [
        "accel",
        "gyro",
        "mag_xyz",
        "ap_mag_xyz",
        "ap_attitude",
        "ap_raw_imu",
        "ap_vel_ned",
        "pos_truth_ned",
        "pos_ekf_ned",
        "quat_wxyz",
        "euler",
    ] {
        assert!(
            set.contains(base),
            "vector base {base} should be derivable from indexed forms",
        );
    }
}

/// Observed keys are merged ON TOP of the static list so custom dialects
/// (non-HVN sources, vendor-specific MAVLink messages) still appear.
#[test]
fn observed_keys_are_merged_on_top_of_known_list() {
    let mut store = TraceStore::new(60.0);
    store.push(0.0, "custom_dialect_key[0]", 1.5);
    let keys = collect_source_keys(&[store]);
    let set: std::collections::BTreeSet<String> = keys.into_iter().collect();

    // Custom key + derived base.
    assert!(set.contains("custom_dialect_key[0]"));
    assert!(set.contains("custom_dialect_key"));
    // Static list still merged in.
    assert!(set.contains("ap_attitude[0]"));
}

/// Schema-only null keys (registered via `TraceStore::note_null_key`)
/// also surface in the picker — the dt_runner path that v0.11.0 was
/// designed to fix.
#[test]
fn null_keys_surface_in_picker() {
    let mut store = TraceStore::new(60.0);
    // Simulate an envelope that announced `weird_vendor_key` with a `null`
    // value (the streamer hasn't filled it yet). No `push`, so the trace
    // buffer stays empty; only `note_null_key` is called.
    store.note_null_key("weird_vendor_key");

    let keys = collect_source_keys(&[store]);
    let set: std::collections::BTreeSet<String> = keys.into_iter().collect();
    assert!(
        set.contains("weird_vendor_key"),
        "schema-only key must appear in the picker before any real value lands",
    );
}

/// A real value supersedes the schema-only registration: once data flows,
/// the key moves out of the null-set and into the regular trace buffer,
/// but stays in the picker.
#[test]
fn real_value_supersedes_schema_only_marker() {
    let mut store = TraceStore::new(60.0);
    store.note_null_key("ap_attitude_late");
    assert!(store.null_keys().contains("ap_attitude_late"));
    store.push(1.0, "ap_attitude_late", 0.5);
    assert!(
        !store.null_keys().contains("ap_attitude_late"),
        "real value should pull the key out of the null-set",
    );

    let keys = collect_source_keys(&[store]);
    assert!(
        keys.iter().any(|k| k == "ap_attitude_late"),
        "key remains visible in the picker after real data lands",
    );
}
