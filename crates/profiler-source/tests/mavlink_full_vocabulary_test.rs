//! v0.16.3 — End-to-end: feed every newly-supported MAVLink message type
//! through the decoder and assert the produced keys cover the DT-Python
//! parity set. This is the "wire-up sanity" pin — if a future refactor
//! drops a message type, the resulting empty key here flags the regression.

#![cfg(feature = "mavlink-source")]

use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex};

use mavlink::dialects::ardupilotmega::{
    AHRS2_DATA, BATTERY_STATUS_DATA, EkfStatusFlags, EKF_STATUS_REPORT_DATA, ESC_STATUS_DATA,
    HEARTBEAT_DATA, MavMessage, MavModeFlag, NAV_CONTROLLER_OUTPUT_DATA, RC_CHANNELS_DATA,
    SCALED_IMU2_DATA, SCALED_IMU3_DATA, SCALED_PRESSURE2_DATA, SCALED_PRESSURE_DATA,
    SERVO_OUTPUT_RAW_DATA, STATUSTEXT_DATA, SYS_STATUS_DATA, VIBRATION_DATA,
};
use mavlink::types::CharArray;
use profiler_source::mavlink_source::decode_to_samples_with_state;
use profiler_source::TextLogEntry;

fn drain(msg: &MavMessage, buf: &Arc<Mutex<VecDeque<TextLogEntry>>>) -> Vec<String> {
    decode_to_samples_with_state(msg, 0.0, None, Some(buf))
        .into_iter()
        .map(|s| s.key)
        .collect()
}

#[test]
fn full_vocabulary_produces_expected_keys() {
    let buf: Arc<Mutex<VecDeque<TextLogEntry>>> =
        Arc::new(Mutex::new(VecDeque::with_capacity(8)));

    let mut keys: HashSet<String> = HashSet::new();

    // EKF_STATUS_REPORT
    keys.extend(drain(
        &MavMessage::EKF_STATUS_REPORT(EKF_STATUS_REPORT_DATA {
            flags: EkfStatusFlags::EKF_ATTITUDE,
            ..Default::default()
        }),
        &buf,
    ));
    // AHRS2
    keys.extend(drain(&MavMessage::AHRS2(AHRS2_DATA::default()), &buf));
    // VIBRATION
    keys.extend(drain(
        &MavMessage::VIBRATION(VIBRATION_DATA {
            time_usec: 0,
            vibration_x: 0.0,
            vibration_y: 0.0,
            vibration_z: 0.0,
            clipping_0: 0,
            clipping_1: 0,
            clipping_2: 0,
        }),
        &buf,
    ));
    // SCALED_IMU2/3
    keys.extend(drain(
        &MavMessage::SCALED_IMU2(SCALED_IMU2_DATA::default()),
        &buf,
    ));
    keys.extend(drain(
        &MavMessage::SCALED_IMU3(SCALED_IMU3_DATA::default()),
        &buf,
    ));
    // SCALED_PRESSURE / 2
    keys.extend(drain(
        &MavMessage::SCALED_PRESSURE(SCALED_PRESSURE_DATA::default()),
        &buf,
    ));
    keys.extend(drain(
        &MavMessage::SCALED_PRESSURE2(SCALED_PRESSURE2_DATA::default()),
        &buf,
    ));
    // BATTERY_STATUS
    keys.extend(drain(
        &MavMessage::BATTERY_STATUS(BATTERY_STATUS_DATA::default()),
        &buf,
    ));
    // ESC_STATUS
    keys.extend(drain(
        &MavMessage::ESC_STATUS(ESC_STATUS_DATA::default()),
        &buf,
    ));
    // RC_CHANNELS
    keys.extend(drain(
        &MavMessage::RC_CHANNELS(RC_CHANNELS_DATA::default()),
        &buf,
    ));
    // SERVO_OUTPUT_RAW
    keys.extend(drain(
        &MavMessage::SERVO_OUTPUT_RAW(SERVO_OUTPUT_RAW_DATA::default()),
        &buf,
    ));
    // NAV_CONTROLLER_OUTPUT
    keys.extend(drain(
        &MavMessage::NAV_CONTROLLER_OUTPUT(NAV_CONTROLLER_OUTPUT_DATA::default()),
        &buf,
    ));
    // SYS_STATUS
    keys.extend(drain(
        &MavMessage::SYS_STATUS(SYS_STATUS_DATA::default()),
        &buf,
    ));
    // STATUSTEXT (needs the buffer)
    keys.extend(drain(
        &MavMessage::STATUSTEXT(STATUSTEXT_DATA {
            severity: mavlink::dialects::ardupilotmega::MavSeverity::MAV_SEVERITY_INFO,
            text: CharArray::new([0u8; 50]),
        }),
        &buf,
    ));
    // HEARTBEAT
    keys.extend(drain(
        &MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 4,
            base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
            ..Default::default()
        }),
        &buf,
    ));

    // Expected: a curated subset of the DT-Python parity vocabulary. Each
    // entry MUST appear at least once. (Not exhaustive — the per-message
    // tests check exact shapes; this test just makes sure no message went
    // missing during a future refactor.)
    let expected = [
        "ekf_flags", "ekf_velv", "ekf_pos_horiz", "ekf_pos_vert", "ekf_compv", "ekf_terralt",
        "ahrs2_roll", "ahrs2_pitch", "ahrs2_yaw", "ahrs2_alt", "ahrs2_lat", "ahrs2_lng",
        "vibex", "vibey", "vibez", "vibeclip0", "vibeclip1", "vibeclip2",
        "scaled_imu2", "scaled_imu2[0]", "scaled_imu2[9]",
        "scaled_imu3", "scaled_imu3[0]", "scaled_imu3[9]",
        "press_scaled", "press_scaled[0]",
        "press_scaled2", "press_scaled2[0]",
        "battery_voltage", "battery_current", "battery_remaining",
        "esc_rpm", "esc_voltage", "esc_current",
        "esc_rpm[0]", "esc_rpm[3]",
        "rc_channels", "rc_channels[0]", "rc_channels[15]", "rc_rssi",
        "servo_outputs", "servo_outputs[0]", "servo_outputs[15]",
        "nav_roll", "nav_pitch", "nav_bearing", "target_bearing",
        "wp_dist", "alt_error", "aspd_error", "xtrack_error",
        "sys_load", "sys_drop_rate_comm", "sys_errors", "sys_errors[0]", "sys_errors[3]",
        "statustexts",
        "armed", "flight_mode",
    ];
    let missing: Vec<&str> = expected
        .iter()
        .copied()
        .filter(|k| !keys.contains(*k))
        .collect();
    assert!(
        missing.is_empty(),
        "missing expected keys after full-vocabulary decode: {missing:?}\n\n\
         got keys: {keys:?}"
    );
}
