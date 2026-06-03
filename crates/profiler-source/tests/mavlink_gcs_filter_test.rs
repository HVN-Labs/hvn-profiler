//! v0.16.5 — non-drone MAVLink peers (Mission Planner, MAVProxy, Skybrush,
//! our own outbound GCS heartbeat at sysid 255) must NOT show up in the
//! profiler's drone picker.
//!
//! Pre-v0.16.5 the recv worker fanned every observed sysid into a sample with
//! `drone_name = drone_<sysid>`, so a shared 14550/14560 port carrying a
//! Mission Planner client would surface as `drone_255` alongside real
//! vehicles. v0.16.5 fixes this by classifying the *first* HEARTBEAT from
//! each sysid by `mavtype` and dropping all subsequent frames from non-drone
//! types (`MAV_TYPE_GCS`, `MAV_TYPE_ONBOARD_CONTROLLER`,
//! `MAV_TYPE_ANTENNA_TRACKER`).
//!
//! Contracts under test:
//!
//! 1. A HEARTBEAT from `sysid = 255` with `mavtype = MAV_TYPE_GCS` produces
//!    ZERO samples at the profiler's `try_recv` boundary, even though the
//!    payload itself decodes cleanly via `decode_to_samples`.
//! 2. A HEARTBEAT from `sysid = 1` with `mavtype = MAV_TYPE_QUADROTOR` DOES
//!    emit samples (drone_1 confirmed).
//! 3. The recv worker never aims a `REQUEST_DATA_STREAM` at the GCS sysid —
//!    stream-requests are reserved for actual vehicles. (The vehicle-side
//!    behaviour is already pinned by `mavlink_active_gcs_test`; we just
//!    confirm the *negative* here for a GCS peer.)

#![cfg(feature = "mavlink-source")]

use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{MavAutopilot, MavMessage, MavType, HEARTBEAT_DATA};
use mavlink::{MavConnection, MavHeader};

use profiler_source::{MavlinkOptions, MavlinkSource, Source};

fn pick_free_udp_port() -> u16 {
    let s = std::net::UdpSocket::bind("127.0.0.1:0").expect("bind 0");
    s.local_addr().expect("local_addr").port()
}

fn open_fake_peer(port: u16) -> mavlink::Connection<MavMessage> {
    let conn_str = format!("udpin:127.0.0.1:{port}");
    let mut conn = mavlink::connect::<MavMessage>(&conn_str).expect("peer bind");
    conn.set_allow_recv_any_version(true);
    conn
}

fn recv_one_with_timeout(
    conn: &mavlink::Connection<MavMessage>,
    timeout: Duration,
) -> Option<(MavHeader, MavMessage)> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match conn.try_recv() {
            Ok(p) => return Some(p),
            Err(_) => std::thread::sleep(Duration::from_millis(20)),
        }
    }
    None
}

fn send_heartbeat(
    conn: &mavlink::Connection<MavMessage>,
    sysid: u8,
    mavtype: MavType,
) {
    let header = MavHeader {
        system_id: sysid,
        component_id: 1,
        sequence: 0,
    };
    let hb = HEARTBEAT_DATA {
        mavtype,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        ..HEARTBEAT_DATA::default()
    };
    conn.send(&header, &MavMessage::HEARTBEAT(hb))
        .expect("send heartbeat");
}

/// Drain every sample the profiler has queued up to `budget` and return them.
fn drain_samples(src: &mut MavlinkSource, budget: Duration) -> Vec<profiler_source::Sample> {
    let deadline = Instant::now() + budget;
    let mut out = Vec::new();
    while Instant::now() < deadline {
        if let Some(s) = src.try_recv() {
            out.push(s);
        } else {
            std::thread::sleep(Duration::from_millis(10));
        }
    }
    out
}

/// (1) + (2): GCS heartbeats are silently dropped; a real-drone heartbeat on
/// the same source is still surfaced.
#[test]
fn gcs_heartbeat_is_dropped_drone_heartbeat_is_kept() {
    let port = pick_free_udp_port();
    let peer = open_fake_peer(port);

    let mut src = MavlinkSource::connect_with(
        &format!("udpout:127.0.0.1:{port}"),
        MavlinkOptions {
            passive: false,
            active_gcs: true,
            ..Default::default()
        },
    )
    .expect("source connect");

    // Wait for the profiler's first outbound HEARTBEAT (worker is alive).
    let _ = recv_one_with_timeout(&peer, Duration::from_secs(3))
        .expect("expected profiler HEARTBEAT");

    // GCS peer says hello — sysid=255, mavtype=GCS. Must be ignored.
    send_heartbeat(&peer, 255, MavType::MAV_TYPE_GCS);
    // Send a follow-up GCS HEARTBEAT (in case the first one races the
    // first drone one we send below).
    send_heartbeat(&peer, 255, MavType::MAV_TYPE_GCS);

    // Real drone says hello — sysid=1, mavtype=QUADROTOR.
    send_heartbeat(&peer, 1, MavType::MAV_TYPE_QUADROTOR);

    // Give the worker ~1.5 s to drain inbound frames and produce samples.
    let samples = drain_samples(&mut src, Duration::from_millis(1500));

    // Bucket samples by sysid.
    let mut from_255 = 0usize;
    let mut from_1 = 0usize;
    for s in &samples {
        match s.sysid {
            Some(255) => from_255 += 1,
            Some(1) => from_1 += 1,
            _ => {}
        }
    }

    assert_eq!(
        from_255, 0,
        "v0.16.5: MAV_TYPE_GCS heartbeat from sysid=255 must emit ZERO samples, got {from_255}",
    );
    assert!(
        from_1 > 0,
        "v0.16.5: MAV_TYPE_QUADROTOR heartbeat from sysid=1 must still emit samples, got 0",
    );

    // Also verify drone_name is never `drone_255` — the picker keys on
    // `drone_name` AND `sysid`, so both projections must be clean.
    let any_drone_255 = samples
        .iter()
        .any(|s| s.drone_name.as_deref() == Some("drone_255"));
    assert!(
        !any_drone_255,
        "v0.16.5: no sample may carry drone_name=\"drone_255\" (GCS sysid leaked into picker)",
    );
}

/// (3): The recv worker must NOT aim a `REQUEST_DATA_STREAM` at a sysid that
/// only ever announced itself as a GCS. The vehicle-side counterpart of this
/// test (drone heartbeat → stream request fires once) already lives in
/// `mavlink_active_gcs_test::one_heartbeat_from_sysid_5_triggers_one_request`.
#[test]
fn gcs_heartbeat_does_not_trigger_request_data_stream() {
    let port = pick_free_udp_port();
    let peer = open_fake_peer(port);

    let _src = MavlinkSource::connect_with(
        &format!("udpout:127.0.0.1:{port}"),
        MavlinkOptions {
            passive: false,
            active_gcs: true,
            ..Default::default()
        },
    )
    .expect("source connect");

    // Wait for the profiler's first outbound HEARTBEAT.
    let _ = recv_one_with_timeout(&peer, Duration::from_secs(3))
        .expect("expected profiler HEARTBEAT");

    // Drive multiple GCS heartbeats — every one MUST be ignored by the
    // stream-request latch.
    for _ in 0..3 {
        send_heartbeat(&peer, 254, MavType::MAV_TYPE_GCS);
        std::thread::sleep(Duration::from_millis(80));
    }

    // Watch the outbound stream for ~2 s. Any REQUEST_DATA_STREAM aimed at
    // sysid=254 is a regression (we'd be asking the GCS to "stream data" at
    // us). HEARTBEATs are fine — those are the 1 Hz keepalive.
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        #[allow(deprecated)]
        if let Some((_h, MavMessage::REQUEST_DATA_STREAM(d))) =
            recv_one_with_timeout(&peer, Duration::from_millis(300))
        {
            if d.target_system == 254 {
                panic!(
                    "v0.16.5: must not send REQUEST_DATA_STREAM to a GCS sysid \
                     (saw target_system=254)",
                );
            }
        }
    }
}
