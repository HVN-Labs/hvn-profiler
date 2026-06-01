//! v0.5.0 integration test — per-panel `label_mode` overlay.
//!
//! Verifies:
//! - JSON templates can declare any `label_mode` (off / data / metadata) and
//!   the per-mode config blocks (`label_data`, `label_metadata`) survive the
//!   round-trip.
//! - The new [`LabelOverride`] global resolver correctly returns either the
//!   cell's own mode (`Respect`) or the forced mode (`Force(...)`).
//! - The `stream_rate_hz` field on `label_metadata` deserialises when present
//!   and is `None` when absent — proving v0.4.0 templates without it still
//!   parse under the v0.5.0 schema.

use profiler_render::LabelOverride;
use profiler_template::{LabelMode, Template};

const TEMPLATE_JSON: &str = r#"{
  "name": "label-mode-fixture",
  "grid": {"rows": 1, "cols": 3},
  "cells": [
    {
      "row": 0, "col": 0,
      "title": "Off cell",
      "primitive": "scalar",
      "sources": [{"key": "a"}],
      "label_mode": "off"
    },
    {
      "row": 0, "col": 1,
      "title": "Data cell",
      "primitive": "scalar",
      "sources": [{"key": "b"}],
      "label_mode": "data",
      "label_data": {"format": "{:+.1f}°", "show_min_max": true}
    },
    {
      "row": 0, "col": 2,
      "title": "Metadata cell",
      "primitive": "scalar",
      "sources": [{"key": "c"}],
      "label_mode": "metadata",
      "label_metadata": {
        "source_path": "ATTITUDE.roll",
        "units": "rad → deg",
        "stream_rate_hz": 4
      }
    }
  ]
}"#;

#[test]
fn parses_all_three_label_modes() {
    let tpl = Template::from_str(TEMPLATE_JSON).expect("parse");
    assert_eq!(tpl.cells.len(), 3);
    assert_eq!(tpl.cells[0].label_mode, LabelMode::Off);
    assert_eq!(tpl.cells[1].label_mode, LabelMode::Data);
    assert_eq!(tpl.cells[2].label_mode, LabelMode::Metadata);

    // Data block survived round-trip.
    let d = tpl.cells[1].label_data.as_ref().expect("label_data");
    assert_eq!(d.format, "{:+.1f}°");
    assert!(d.show_min_max);

    // Metadata block survived round-trip, incl. stream_rate_hz.
    let m = tpl.cells[2].label_metadata.as_ref().expect("label_metadata");
    assert_eq!(m.source_path, "ATTITUDE.roll");
    assert_eq!(m.units, "rad → deg");
    assert_eq!(m.stream_rate_hz, Some(4.0));
}

#[test]
fn override_respects_cell_mode_by_default() {
    let ov = LabelOverride::default();
    assert_eq!(ov.resolve(LabelMode::Off), LabelMode::Off);
    assert_eq!(ov.resolve(LabelMode::Data), LabelMode::Data);
    assert_eq!(ov.resolve(LabelMode::Metadata), LabelMode::Metadata);
}

#[test]
fn override_force_replaces_cell_mode() {
    // Force(Data) overrides every cell, regardless of what the JSON said.
    let force_data = LabelOverride::Force(LabelMode::Data);
    assert_eq!(force_data.resolve(LabelMode::Off), LabelMode::Data);
    assert_eq!(force_data.resolve(LabelMode::Metadata), LabelMode::Data);

    // Force(Off) silences every overlay.
    let force_off = LabelOverride::Force(LabelMode::Off);
    assert_eq!(force_off.resolve(LabelMode::Data), LabelMode::Off);
    assert_eq!(force_off.resolve(LabelMode::Metadata), LabelMode::Off);

    // Force(Metadata) flips data-cells to metadata.
    let force_meta = LabelOverride::Force(LabelMode::Metadata);
    assert_eq!(force_meta.resolve(LabelMode::Data), LabelMode::Metadata);
}

#[test]
fn missing_stream_rate_hz_is_none() {
    // v0.4.0-shape template (no stream_rate_hz on the metadata block) must
    // still load cleanly under the v0.5.0 schema.
    let json = r#"{
        "cells": [{
            "row": 0, "col": 0,
            "primitive": "scalar",
            "sources": [{"key": "a"}],
            "label_mode": "metadata",
            "label_metadata": {"source_path": "X", "units": "u"}
        }]
    }"#;
    let tpl = Template::from_str(json).expect("parse");
    let md = tpl.cells[0].label_metadata.as_ref().unwrap();
    assert!(md.stream_rate_hz.is_none());
}
