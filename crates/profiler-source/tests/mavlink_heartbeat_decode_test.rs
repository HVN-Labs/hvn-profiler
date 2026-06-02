//! v0.16.3 — `HEARTBEAT` decoder pulls the armed bit (`base_mode & 0x80`)
//! and decodes `custom_mode` through the copter mode table into a
//! `flight_mode` string. The render layer's `StatusKind::ArmedBool` and
//! `StatusKind::Text` consume these directly.

#![cfg(feature = "mavlink-source")]

use mavlink::dialects::ardupilotmega::{
    HEARTBEAT_DATA, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType,
};
use profiler_source::mavlink_source::decode_to_samples;
use profiler_source::Value;

#[test]
fn heartbeat_decodes_armed_bit_and_guided_mode() {
    let d = HEARTBEAT_DATA {
        custom_mode: 4, // GUIDED
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED
            | MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    };
    let msg = MavMessage::HEARTBEAT(d);
    let samples = decode_to_samples(&msg, 0.0);

    let armed = samples
        .iter()
        .find(|s| s.key == "armed")
        .expect("armed sample");
    assert_eq!(armed.value, Value::Bool(true));

    let mode = samples
        .iter()
        .find(|s| s.key == "flight_mode")
        .expect("flight_mode sample");
    match &mode.value {
        Value::String(s) => assert_eq!(s.as_ref(), "GUIDED"),
        other => panic!("expected Value::String, got {other:?}"),
    }
}

#[test]
fn heartbeat_disarmed_in_stabilize_decodes_cleanly() {
    let d = HEARTBEAT_DATA {
        custom_mode: 0, // STABILIZE
        base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // armed bit OFF
        ..Default::default()
    };
    let msg = MavMessage::HEARTBEAT(d);
    let samples = decode_to_samples(&msg, 0.0);
    let armed = samples.iter().find(|s| s.key == "armed").unwrap();
    assert_eq!(armed.value, Value::Bool(false));
    let mode = samples.iter().find(|s| s.key == "flight_mode").unwrap();
    if let Value::String(s) = &mode.value {
        assert_eq!(s.as_ref(), "STABILIZE");
    } else {
        panic!("expected Value::String");
    }
}

#[test]
fn heartbeat_unknown_copter_mode_falls_back_to_mode_n() {
    let d = HEARTBEAT_DATA {
        custom_mode: 99, // outside the 0..27 table
        ..Default::default()
    };
    let msg = MavMessage::HEARTBEAT(d);
    let samples = decode_to_samples(&msg, 0.0);
    let mode = samples.iter().find(|s| s.key == "flight_mode").unwrap();
    if let Value::String(s) = &mode.value {
        assert_eq!(s.as_ref(), "MODE_99");
    } else {
        panic!("expected Value::String");
    }
}
