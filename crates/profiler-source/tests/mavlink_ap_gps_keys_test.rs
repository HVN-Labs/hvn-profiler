//! v0.16.7 — regression test for the `gps_alt` / `gps_vn` rename on the
//! MAVLink decoder side.
//!
//! Before v0.16.7 the MAVLink decoder emitted `gps_alt` (from
//! GLOBAL_POSITION_INT and GPS_RAW_INT) and `gps_vn` (from GPS_RAW_INT,
//! which is actually ground-speed magnitude — semantically wrong). Both keys
//! collided with DT-Python's envelope which emits `gps_alt = state.lla[2]`
//! (truth altitude) and `gps_vn = v_ned[0]` (truth north velocity). In HIL
//! mode whichever source arrived last overwrote the other.
//!
//! v0.16.7 renames the MAVLink emissions to `ap_gps_alt` (matching the
//! `ap_*` prefix convention) and `ap_gps_speed` (the second rename is also
//! semantic — `GPS_RAW_INT.vel` is a magnitude, not a north component).
//! This test pins both renames so future refactors don't re-introduce the
//! collision.

#![cfg(feature = "mavlink-source")]

use mavlink::dialects::ardupilotmega::{
    GLOBAL_POSITION_INT_DATA, GPS_RAW_INT_DATA, GpsFixType, MavMessage,
};
use profiler_source::mavlink_source::decode_to_samples;
use profiler_source::Value;

const EPS: f64 = 1e-9;

#[test]
fn global_position_int_emits_ap_gps_alt_not_bare_gps_alt() {
    let d = GLOBAL_POSITION_INT_DATA {
        time_boot_ms: 12_345,
        lat: 515_034_700,
        lon: -25_316_430,
        alt: 45_700, // 45.7 m on the wire (mm)
        relative_alt: 0,
        vx: 100,
        vy: 200,
        vz: 50,
        hdg: 0,
    };
    let msg = MavMessage::GLOBAL_POSITION_INT(d);
    let samples = decode_to_samples(&msg, 0.0);

    let alt = samples
        .iter()
        .find(|s| s.key == "ap_gps_alt")
        .expect("GLOBAL_POSITION_INT must produce ap_gps_alt");
    match &alt.value {
        Value::Scalar(v) => assert!(
            (v - 45.7).abs() < EPS,
            "ap_gps_alt should be 45.7 m (alt mm /1000), got {v}",
        ),
        other => panic!("ap_gps_alt must be Value::Scalar, got {other:?}"),
    }

    assert!(
        samples.iter().all(|s| s.key != "gps_alt"),
        "v0.16.7: MAVLink must NOT emit bare `gps_alt` (collides with DT truth)",
    );
}

#[test]
fn gps_raw_int_emits_ap_gps_alt_and_ap_gps_speed() {
    let d = GPS_RAW_INT_DATA {
        time_usec: 1_000_000,
        lat: 515_034_700,
        lon: -25_316_430,
        alt: 45_700,
        eph: 100,
        epv: 100,
        vel: 350, // 3.50 m/s ground-speed magnitude (cm/s on the wire)
        cog: 0,
        fix_type: GpsFixType::GPS_FIX_TYPE_RTK_FIXED,
        satellites_visible: 14,
    };
    let msg = MavMessage::GPS_RAW_INT(d);
    let samples = decode_to_samples(&msg, 0.0);

    let alt = samples
        .iter()
        .find(|s| s.key == "ap_gps_alt")
        .expect("GPS_RAW_INT must produce ap_gps_alt");
    if let Value::Scalar(v) = &alt.value {
        assert!((v - 45.7).abs() < EPS);
    } else {
        panic!("ap_gps_alt must be Value::Scalar");
    }

    let spd = samples
        .iter()
        .find(|s| s.key == "ap_gps_speed")
        .expect("GPS_RAW_INT must produce ap_gps_speed");
    if let Value::Scalar(v) = &spd.value {
        assert!(
            (v - 3.50).abs() < EPS,
            "ap_gps_speed should be 3.50 m/s (vel cm/s /100), got {v}",
        );
    } else {
        panic!("ap_gps_speed must be Value::Scalar");
    }
}

#[test]
fn gps_raw_int_does_not_emit_bare_gps_alt_or_gps_vn() {
    // Collision-guard: DT-Python emits `gps_alt` (truth altitude) and
    // `gps_vn` (truth north velocity). MAVLink must NOT emit either bare
    // name — they are reserved for the DT side.
    let d = GPS_RAW_INT_DATA {
        time_usec: 0,
        lat: 0,
        lon: 0,
        alt: 0,
        eph: 0,
        epv: 0,
        vel: 0,
        cog: 0,
        fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
        satellites_visible: 0,
    };
    let msg = MavMessage::GPS_RAW_INT(d);
    let samples = decode_to_samples(&msg, 0.0);

    assert!(
        samples.iter().all(|s| s.key != "gps_alt"),
        "v0.16.7: MAVLink must NOT emit bare `gps_alt`",
    );
    assert!(
        samples.iter().all(|s| s.key != "gps_vn"),
        "v0.16.7: MAVLink must NOT emit bare `gps_vn` (was semantically wrong — \
         GPS_RAW_INT.vel is ground-speed magnitude, not a N velocity component; \
         renamed to `ap_gps_speed`)",
    );
}
