//! v0.10.2 — grouped source-key picker.
//!
//! The "+ Add Panel" / "Edit Panel" modal's source-key dropdown is grouped
//! by category (DT physics, AP MAVLink, Position (NED), Timing, Other). The
//! categorisation is a pure function on the key string; these tests pin both
//! the per-key category and the dropdown's flattened section order.

use profiler_render::{categorize_key, group_source_keys, KEY_GROUPS};

/// Vector base + array-indexed forms land in the same group.
#[test]
fn categorize_handles_array_indexing() {
    assert_eq!(categorize_key("accel"), "DT physics");
    assert_eq!(categorize_key("accel[0]"), "DT physics");
    assert_eq!(categorize_key("accel[2]"), "DT physics");
    assert_eq!(categorize_key("ap_attitude"), "AP MAVLink");
    assert_eq!(categorize_key("ap_attitude[1]"), "AP MAVLink");
}

#[test]
fn categorize_dt_physics_keys() {
    for k in [
        "accel",
        "gyro",
        "mag_xyz",
        "mag_clean_xyz",
        "wind_ned",
        "baro_pressure",
        "baro_temp",
        "baro_alt",
        "state_alt",
        "quat_wxyz",
        "euler",
        "gps_alt",
        "gps_vn",
    ] {
        assert_eq!(categorize_key(k), "DT physics", "{k} should be DT physics");
    }
}

#[test]
fn categorize_ap_mavlink_keys() {
    for k in ["ap_attitude", "ap_raw_imu", "ap_vfr_alt", "ap_vel_ned"] {
        assert_eq!(categorize_key(k), "AP MAVLink", "{k} should be AP MAVLink");
    }
}

#[test]
fn categorize_position_keys_use_prefix() {
    assert_eq!(categorize_key("pos_truth_ned"), "Position (NED)");
    assert_eq!(categorize_key("pos_truth_ned[0]"), "Position (NED)");
    assert_eq!(categorize_key("pos_ekf_ned"), "Position (NED)");
    assert_eq!(categorize_key("pos_gps_ned[2]"), "Position (NED)");
}

#[test]
fn categorize_timing_keys() {
    assert_eq!(categorize_key("t"), "Timing");
    assert_eq!(categorize_key("ts"), "Timing");
}

#[test]
fn categorize_unknown_falls_through_to_other() {
    assert_eq!(categorize_key("some_unknown_key"), "Other");
    assert_eq!(categorize_key("foo[3]"), "Other");
    assert_eq!(categorize_key(""), "Other");
}

#[test]
fn key_groups_constant_is_in_render_order() {
    assert_eq!(
        KEY_GROUPS,
        &["DT physics", "AP MAVLink", "Position (NED)", "Timing", "Other"]
    );
}

/// Feed the categorizer a list of keys; assert correct grouping and that
/// the returned groups follow the fixed `KEY_GROUPS` order.
#[test]
fn group_source_keys_sections_appear_in_fixed_order() {
    let keys: Vec<String> = [
        // Intentionally scrambled across categories.
        "ap_attitude[0]",
        "accel[0]",
        "pos_ekf_ned[0]",
        "t",
        "foo_unknown",
        "gyro[1]",
        "ap_vfr_alt",
        "pos_truth_ned[2]",
        "ts",
    ]
    .into_iter()
    .map(String::from)
    .collect();

    let groups = group_source_keys(&keys);

    // Order: DT physics → AP MAVLink → Position (NED) → Timing → Other.
    let order: Vec<&str> = groups.iter().map(|(g, _)| *g).collect();
    assert_eq!(
        order,
        vec!["DT physics", "AP MAVLink", "Position (NED)", "Timing", "Other"]
    );

    // Per-group contents.
    let by_group: std::collections::HashMap<&str, Vec<String>> =
        groups.into_iter().collect();
    assert_eq!(by_group["DT physics"], vec!["accel[0]", "gyro[1]"]);
    assert_eq!(by_group["AP MAVLink"], vec!["ap_attitude[0]", "ap_vfr_alt"]);
    assert_eq!(
        by_group["Position (NED)"],
        vec!["pos_ekf_ned[0]", "pos_truth_ned[2]"]
    );
    assert_eq!(by_group["Timing"], vec!["t", "ts"]);
    assert_eq!(by_group["Other"], vec!["foo_unknown"]);
}

/// Empty groups are omitted from the returned vector so the UI does not draw
/// dead section headers when a run happens to have no keys in some category.
#[test]
fn empty_groups_are_omitted() {
    let keys: Vec<String> = vec!["accel[0]".into(), "gyro[1]".into()];
    let groups = group_source_keys(&keys);
    assert_eq!(groups.len(), 1, "only DT physics is populated");
    assert_eq!(groups[0].0, "DT physics");
}

/// The flattened entry order matches the dropdown's visual order:
/// section-by-section, alphabetical within each section (the alphabetical
/// invariant is provided by `collect_source_keys` upstream — we just verify
/// the section-major order here).
#[test]
fn flattened_dropdown_entries_in_expected_order() {
    let keys: Vec<String> = vec![
        // Already alphabetical, as `collect_source_keys` returns.
        "accel[0]".into(),
        "ap_attitude[0]".into(),
        "ap_attitude[1]".into(),
        "gyro[0]".into(),
        "pos_truth_ned[0]".into(),
        "t".into(),
        "weird".into(),
    ];
    let flat: Vec<String> = group_source_keys(&keys)
        .into_iter()
        .flat_map(|(_, v)| v)
        .collect();
    assert_eq!(
        flat,
        vec![
            // DT physics first.
            "accel[0]",
            "gyro[0]",
            // Then AP MAVLink.
            "ap_attitude[0]",
            "ap_attitude[1]",
            // Then Position (NED).
            "pos_truth_ned[0]",
            // Timing.
            "t",
            // Other.
            "weird",
        ]
    );
}
