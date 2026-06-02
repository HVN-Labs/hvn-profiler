//! v0.16.3 — `MavlinkSource` must decode `EKF_STATUS_REPORT` into the 6
//! `ekf_*` keys the streamer-side bridge ships. Without this, a cell
//! binding `ekf_compv` stays empty even though AP is sending the data.

#![cfg(feature = "mavlink-source")]

use mavlink::dialects::ardupilotmega::{
    EkfStatusFlags, EKF_STATUS_REPORT_DATA, MavMessage,
};
use profiler_source::mavlink_source::decode_to_samples;
use profiler_source::Sample;

/// Helper: collect (key, scalar) pairs at the asserted timestamp.
fn pairs(samples: Vec<Sample>, ts: f64) -> Vec<(String, f64)> {
    samples
        .into_iter()
        .map(|s| {
            assert_eq!(s.ts, ts);
            let v = s.scalar();
            (s.key, v)
        })
        .collect()
}

#[test]
fn ekf_status_report_emits_six_scalar_keys() {
    let d = EKF_STATUS_REPORT_DATA {
        velocity_variance: 0.5,
        pos_horiz_variance: 0.25,
        pos_vert_variance: 0.125,
        compass_variance: 1.0,
        terrain_alt_variance: 2.0,
        flags: EkfStatusFlags::EKF_ATTITUDE
            | EkfStatusFlags::EKF_VELOCITY_HORIZ
            | EkfStatusFlags::EKF_POS_HORIZ_REL,
    };
    let msg = MavMessage::EKF_STATUS_REPORT(d);
    let got = pairs(decode_to_samples(&msg, 7.0), 7.0);

    // `flags` should be 1 | 2 | 8 = 11.
    let expected_flags = (EkfStatusFlags::EKF_ATTITUDE
        | EkfStatusFlags::EKF_VELOCITY_HORIZ
        | EkfStatusFlags::EKF_POS_HORIZ_REL)
        .bits() as f64;

    assert_eq!(
        got,
        vec![
            ("ekf_flags".into(), expected_flags),
            ("ekf_velv".into(), 0.5_f32 as f64),
            ("ekf_pos_horiz".into(), 0.25),
            ("ekf_pos_vert".into(), 0.125),
            ("ekf_compv".into(), 1.0),
            ("ekf_terralt".into(), 2.0),
        ]
    );
}

#[test]
fn ekf_compv_key_is_produced_when_ekf_status_arrives() {
    // The bug report: "ekf_compv panel stays empty even though the data
    // was on the wire". This pins the contract that decoding an
    // EKF_STATUS_REPORT frame produces a sample with key `ekf_compv`.
    let d = EKF_STATUS_REPORT_DATA {
        compass_variance: 0.42,
        ..Default::default()
    };
    let msg = MavMessage::EKF_STATUS_REPORT(d);
    let samples = decode_to_samples(&msg, 0.0);
    let compv = samples
        .iter()
        .find(|s| s.key == "ekf_compv")
        .expect("ekf_compv must be in the emitted samples");
    assert!((compv.scalar() - 0.42_f64).abs() < 1e-5);
}
