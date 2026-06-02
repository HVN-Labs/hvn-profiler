//! v0.12.0 — `status` primitive: colored chip / text-log cells driven by
//! non-plot data sources (flight mode string, armed bool, GPS fix type,
//! statustext rolling log).
//!
//! These tests pin the renderer's color-resolution contract (the chip's
//! background color is what the operator sees at a glance) without spinning
//! up an egui context: [`status_cell_color`] is a pure function that takes
//! the cell config + a fresh store snapshot.

use std::collections::BTreeMap;

use egui::Color32;
use profiler_render::{
    status_cell_color, status_fix_type_chip, status_severity_color, TextLogEntry, TraceStore,
};
use profiler_template::{Cell, Primitive, StatusKind};

/// Helper: build a minimal `Cell` configured as a status primitive.
fn status_cell(
    source: &str,
    kind: StatusKind,
    color_map: &[(&str, &str)],
    default_color: Option<&str>,
) -> Cell {
    let mut cm = BTreeMap::new();
    for (k, v) in color_map {
        cm.insert((*k).to_string(), (*v).to_string());
    }
    Cell {
        primitive: Primitive::Status,
        source: source.to_string(),
        kind: Some(kind),
        color_map: cm,
        default_color: default_color.map(|s| s.to_string()),
        visible: true,
        ..Default::default()
    }
}

#[test]
fn text_kind_color_map_hit_uses_mapped_color() {
    let mut store = TraceStore::new(60.0);
    store.push_string(1.0, "flight_mode", "GUIDED");
    let cell = status_cell(
        "flight_mode",
        StatusKind::Text,
        &[
            ("GUIDED", "#1f77b4"),
            ("LOITER", "#2ca02c"),
            ("RTL", "#d62728"),
        ],
        Some("#aaaaaa"),
    );
    assert_eq!(
        status_cell_color(&cell, &store),
        Color32::from_rgb(0x1f, 0x77, 0xb4),
    );
}

#[test]
fn text_kind_color_map_miss_falls_back_to_default() {
    let mut store = TraceStore::new(60.0);
    store.push_string(1.0, "flight_mode", "POSHOLD"); // not in the map
    let cell = status_cell(
        "flight_mode",
        StatusKind::Text,
        &[("GUIDED", "#1f77b4")],
        Some("#aaaaaa"),
    );
    assert_eq!(
        status_cell_color(&cell, &store),
        Color32::from_rgb(0xaa, 0xaa, 0xaa),
    );
}

#[test]
fn text_kind_no_data_falls_back_to_default() {
    let store = TraceStore::new(60.0);
    let cell = status_cell(
        "flight_mode",
        StatusKind::Text,
        &[("GUIDED", "#1f77b4")],
        Some("#aaaaaa"),
    );
    assert_eq!(
        status_cell_color(&cell, &store),
        Color32::from_rgb(0xaa, 0xaa, 0xaa),
    );
}

#[test]
fn armed_bool_true_renders_green() {
    let mut store = TraceStore::new(60.0);
    store.push(1.0, "armed", 1.0);
    let cell = status_cell("armed", StatusKind::ArmedBool, &[], None);
    assert_eq!(
        status_cell_color(&cell, &store),
        Color32::from_rgb(0x2c, 0xa0, 0x2c),
    );
}

#[test]
fn armed_bool_false_renders_gray() {
    let mut store = TraceStore::new(60.0);
    store.push(1.0, "armed", 0.0);
    let cell = status_cell("armed", StatusKind::ArmedBool, &[], None);
    assert_eq!(status_cell_color(&cell, &store), Color32::from_gray(120));
}

#[test]
fn armed_bool_string_form_true_renders_green() {
    let mut store = TraceStore::new(60.0);
    store.push_string(1.0, "armed", "True");
    let cell = status_cell("armed", StatusKind::ArmedBool, &[], None);
    assert_eq!(
        status_cell_color(&cell, &store),
        Color32::from_rgb(0x2c, 0xa0, 0x2c),
    );
}

#[test]
fn armed_bool_color_map_override_wins() {
    let mut store = TraceStore::new(60.0);
    store.push(1.0, "armed", 1.0);
    // User supplied a custom red for True.
    let cell = status_cell(
        "armed",
        StatusKind::ArmedBool,
        &[("True", "#ff0000")],
        None,
    );
    assert_eq!(status_cell_color(&cell, &store), Color32::from_rgb(0xff, 0, 0));
}

#[test]
fn fix_type_each_value_has_distinct_color() {
    let expected = [
        (0, "No fix", Color32::from_rgb(0xd6, 0x27, 0x28)),
        (1, "2D", Color32::from_rgb(0xe6, 0xa9, 0x00)),
        (2, "3D", Color32::from_rgb(0x2c, 0xa0, 0x2c)),
        (3, "DGPS", Color32::from_rgb(0x1f, 0x77, 0xb4)),
        (4, "RTK float", Color32::from_rgb(0x94, 0x67, 0xbd)),
        (5, "RTK fixed", Color32::from_rgb(0x2c, 0xa0, 0x2c)),
        (6, "RTK fixed", Color32::from_rgb(0x2c, 0xa0, 0x2c)),
    ];
    for (n, label, color) in expected {
        let (got_label, got_color) = status_fix_type_chip(n);
        assert_eq!(got_label, label, "fix_type {n} label");
        assert_eq!(got_color, color, "fix_type {n} color");
    }
}

#[test]
fn fix_type_cell_resolves_color_from_value() {
    let mut store = TraceStore::new(60.0);
    store.push(1.0, "fix_type", 3.0);
    let cell = status_cell("fix_type", StatusKind::FixType, &[], None);
    // DGPS = blue (#1f77b4).
    assert_eq!(
        status_cell_color(&cell, &store),
        Color32::from_rgb(0x1f, 0x77, 0xb4),
    );
}

#[test]
fn text_log_renders_rolling_buffer_in_newest_first_order() {
    let mut store = TraceStore::new(60.0);
    store.push_text_log(
        1.0,
        "statustexts",
        TextLogEntry { ts: 1.0, text: "boot complete".into(), severity: 6 },
    );
    store.push_text_log(
        2.0,
        "statustexts",
        TextLogEntry { ts: 2.0, text: "EKF alignment".into(), severity: 5 },
    );
    store.push_text_log(
        3.0,
        "statustexts",
        TextLogEntry { ts: 3.0, text: "PreArm: Battery low".into(), severity: 4 },
    );
    let entries = store.text_log_owned("statustexts");
    assert_eq!(entries.len(), 3);
    // Newest is at the back.
    assert_eq!(entries[2].text, "PreArm: Battery low");
    // Severity colors.
    assert_eq!(status_severity_color(3), Color32::from_rgb(0xd6, 0x27, 0x28)); // red
    assert_eq!(status_severity_color(4), Color32::from_rgb(0xe6, 0xa9, 0x00)); // yellow
    assert_eq!(status_severity_color(5), Color32::from_rgb(0x1f, 0x77, 0xb4)); // blue
    assert_eq!(status_severity_color(7), Color32::from_gray(170));             // gray
}

#[test]
fn text_log_buffer_caps_at_capacity() {
    let mut store = TraceStore::new(60.0);
    for i in 0..200 {
        store.push_text_log(
            i as f64,
            "statustexts",
            TextLogEntry {
                ts: i as f64,
                text: format!("msg {i}"),
                severity: 6,
            },
        );
    }
    let entries = store.text_log_owned("statustexts");
    assert_eq!(
        entries.len(),
        profiler_render::TEXT_LOG_CAPACITY,
        "buffer caps at TEXT_LOG_CAPACITY",
    );
    // Oldest evicted: first surviving entry's text is "msg <200-cap>".
    let expected_first = 200 - profiler_render::TEXT_LOG_CAPACITY;
    assert_eq!(entries[0].text, format!("msg {expected_first}"));
}
