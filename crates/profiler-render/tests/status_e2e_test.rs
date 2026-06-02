//! v0.13.0 — Status primitive end-to-end: push samples mirroring the
//! SITL v0.9.0 streamer envelope through a `TraceStore`, then build a
//! template with one cell per `StatusKind` and assert the renderer
//! resolves the correct color + value for each.

use std::collections::BTreeMap;

use egui::Color32;
use profiler_render::{status_cell_color, TextLogEntry, TraceStore};
use profiler_template::{Cell, Primitive, StatusKind};

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
fn status_e2e_three_kinds_resolve_correct_state_and_color() {
    // Drive a single TraceStore with samples representative of one SITL
    // tick: `flight_mode = "GUIDED"`, `armed = True`, plus three
    // statustext entries with varying severity.
    let mut store = TraceStore::new(60.0);
    store.push_string(1.0, "flight_mode", "GUIDED");
    store.push_bool(1.0, "armed", true);
    store.push_text_log(
        1.0,
        "statustexts",
        TextLogEntry { ts: 1.0, text: "boot complete".into(), severity: 6 },
    );
    store.push_text_log(
        1.0,
        "statustexts",
        TextLogEntry { ts: 1.1, text: "EKF alignment".into(), severity: 5 },
    );
    store.push_text_log(
        1.0,
        "statustexts",
        TextLogEntry { ts: 1.2, text: "PreArm: Battery low".into(), severity: 4 },
    );

    // Three cells, one per kind. The template is "logical": we only
    // exercise the pure color-resolution path so an egui context isn't
    // needed.
    let cells = [
        status_cell(
            "flight_mode",
            StatusKind::Text,
            &[
                ("GUIDED", "#1f77b4"),
                ("LOITER", "#2ca02c"),
                ("RTL", "#d62728"),
            ],
            Some("#aaaaaa"),
        ),
        status_cell("armed", StatusKind::ArmedBool, &[], None),
        status_cell("statustexts", StatusKind::TextLog, &[], None),
    ];

    // 1. flight_mode → GUIDED, mapped color = #1f77b4.
    assert_eq!(
        status_cell_color(&cells[0], &store),
        Color32::from_rgb(0x1f, 0x77, 0xb4),
        "flight_mode='GUIDED' should resolve to the mapped color",
    );
    // 2. armed → True, default green chip.
    assert_eq!(
        status_cell_color(&cells[1], &store),
        Color32::from_rgb(0x2c, 0xa0, 0x2c),
        "armed=True should resolve to the green ARMED chip",
    );
    // 3. statustexts → TextLog cells defer to the per-entry severity
    //    colors in the body; the chip background is the default color.
    let entries = store.text_log_owned("statustexts");
    assert_eq!(entries.len(), 3, "all three statustexts arrived");
    assert_eq!(entries[2].text, "PreArm: Battery low");
    assert_eq!(entries[2].severity, 4);

    // 4. Mutate state and verify the resolved color updates without any
    //    rebuild of the cells.
    store.push_bool(2.0, "armed", false);
    assert_eq!(
        status_cell_color(&cells[1], &store),
        Color32::from_gray(120),
        "armed=False should drop to the gray DISARMED chip",
    );
    store.push_string(2.0, "flight_mode", "LOITER");
    assert_eq!(
        status_cell_color(&cells[0], &store),
        Color32::from_rgb(0x2c, 0xa0, 0x2c),
        "flight_mode='LOITER' should pick up the mapped color",
    );
    store.push_string(3.0, "flight_mode", "POSHOLD");
    assert_eq!(
        status_cell_color(&cells[0], &store),
        Color32::from_rgb(0xaa, 0xaa, 0xaa),
        "unmapped flight_mode falls back to the default color",
    );
}

#[test]
fn status_e2e_armed_bool_string_form_works() {
    // SITL v0.9.0 ships `armed` as either a Bool or the literal string
    // "True"/"False" depending on the source path. The store must accept
    // both via push_bool / push_string and the Status renderer must read
    // either.
    let mut store = TraceStore::new(60.0);
    store.push_string(1.0, "armed", "True");
    let cell = status_cell("armed", StatusKind::ArmedBool, &[], None);
    assert_eq!(
        status_cell_color(&cell, &store),
        Color32::from_rgb(0x2c, 0xa0, 0x2c),
    );
}

#[test]
fn status_e2e_fix_type_resolves_per_value_color() {
    let mut store = TraceStore::new(60.0);
    // DGPS (3) → blue.
    store.push(1.0, "fix_type", 3.0);
    let cell = status_cell("fix_type", StatusKind::FixType, &[], None);
    assert_eq!(
        status_cell_color(&cell, &store),
        Color32::from_rgb(0x1f, 0x77, 0xb4),
    );
    // RTK fixed (5) → green.
    store.push(2.0, "fix_type", 5.0);
    assert_eq!(
        status_cell_color(&cell, &store),
        Color32::from_rgb(0x2c, 0xa0, 0x2c),
    );
}
