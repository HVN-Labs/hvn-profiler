//! v0.8.0 — round-trip a template through capture → save → reload, asserting
//! the per-cell visibility / label-mode / 3D-trail state survives.

use profiler_template::{LabelMode, Template, UiState};

const BASE_JSON: &str = r##"{
  "name": "round-trip",
  "grid": {"rows": 2, "cols": 2},
  "cells": [
    {"row": 0, "col": 0, "primitive": "scalar",
     "sources": [{"key": "a"}], "label_mode": "off"},
    {"row": 0, "col": 1, "primitive": "scalar",
     "sources": [{"key": "b"}], "label_mode": "off"},
    {"row": 1, "col": 0, "primitive": "scalar",
     "sources": [{"key": "c"}], "label_mode": "off"},
    {"row": 1, "col": 1, "primitive": "scalar",
     "sources": [{"key": "d"}], "label_mode": "off"}
  ],
  "view_3d": {
    "trails": [
      {"name": "truth", "color": "#2ca02c",
       "sources": {"x": "x[0]", "y": "x[1]", "z_neg": "x[2]"}}
    ]
  }
}"##;

#[test]
fn save_then_reload_preserves_visibility_and_label_overrides() {
    let mut tpl = Template::from_str(BASE_JSON).unwrap();
    // Simulate user toggling some panels off and forcing "metadata" labels.
    let mut ui = UiState::default();
    ui.cell_visibility.insert("0,0".into(), true);
    ui.cell_visibility.insert("0,1".into(), false);
    ui.cell_visibility.insert("1,0".into(), false);
    ui.cell_visibility.insert("1,1".into(), true);
    for k in ["0,0", "0,1", "1,0", "1,1"] {
        ui.cell_label_mode
            .insert(k.into(), LabelMode::Metadata);
    }
    ui.trail_frac = Some(0.42);
    ui.view_frac = Some(0.13);
    ui.trail_visibility.insert("truth".into(), true);

    // Apply, then snapshot back into the template for serialisation.
    tpl.apply_ui_state(&ui);
    tpl.ui_state = Some(ui.clone());

    // Write to a temp file and read it back.
    let tmp = tempfile::tempdir().unwrap();
    let path = tmp.path().join("saved.json");
    let json = tpl.to_pretty_json().expect("serialise");
    std::fs::write(&path, json.as_bytes()).unwrap();

    let reloaded = Template::from_path(&path).expect("reload");
    let reloaded_ui = reloaded
        .ui_state
        .as_ref()
        .expect("ui_state present after reload");

    // Cell visibility round-tripped.
    assert_eq!(reloaded_ui.cell_visibility.get("0,1"), Some(&false));
    assert_eq!(reloaded_ui.cell_visibility.get("1,0"), Some(&false));
    assert_eq!(reloaded_ui.cell_visibility.get("0,0"), Some(&true));

    // Label-mode override round-tripped.
    for k in ["0,0", "0,1", "1,0", "1,1"] {
        assert_eq!(
            reloaded_ui.cell_label_mode.get(k),
            Some(&LabelMode::Metadata),
            "label mode for {k} should be Metadata"
        );
    }

    // 3D state.
    assert_eq!(reloaded_ui.trail_frac, Some(0.42));
    assert_eq!(reloaded_ui.view_frac, Some(0.13));
    assert_eq!(reloaded_ui.trail_visibility.get("truth"), Some(&true));

    // Applying ui_state on reload should drive the template's actual fields
    // (so a freshly-loaded template boots in the saved state).
    let mut applied = reloaded;
    let captured = applied.ui_state.clone().unwrap();
    applied.apply_ui_state(&captured);
    let invisible_count = applied.cells.iter().filter(|c| !c.visible).count();
    assert_eq!(invisible_count, 2, "(0,1) and (1,0) should be hidden");
    for cell in &applied.cells {
        assert_eq!(cell.label_mode, LabelMode::Metadata);
    }
}

#[test]
fn templates_without_ui_state_round_trip_unchanged() {
    // Spec: "Existing templates without these fields load unchanged."
    let tpl = Template::from_str(BASE_JSON).unwrap();
    assert!(tpl.ui_state.is_none(), "base JSON carries no ui_state");
    // Round-trip through to_pretty_json: no ui_state should be emitted.
    let s = tpl.to_pretty_json().unwrap();
    assert!(
        !s.contains("\"ui_state\""),
        "ui_state must be omitted via skip_serializing_if; got:\n{s}"
    );
}
