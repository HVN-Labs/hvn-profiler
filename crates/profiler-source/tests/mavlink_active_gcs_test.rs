//! v0.16.4 — per-sysid active-GCS stream-request behaviour.
//!
//! Contract under test:
//!
//! 1. Open a `udpout://` profiler source with active-GCS ON. The fake
//!    "vehicle" sends one HEARTBEAT with `sysid = 5` and the profiler must
//!    respond with exactly one `REQUEST_DATA_STREAM(start=1, ALL)` aimed at
//!    `target_system = 5`.
//! 2. A SECOND HEARTBEAT from the SAME sysid must NOT trigger a duplicate
//!    request — the per-sysid latch fires once per sysid.
//! 3. With `--no-mavlink-active-gcs` (`active_gcs: false`), zero
//!    `REQUEST_DATA_STREAM` frames appear regardless of inbound HEARTBEATs.
//!
//! Why `udpout` instead of `udpin`: the test's "vehicle" needs a known
//! destination port the source can target. The existing v0.8.0 active-GCS
//! tests use the same pattern; we follow the convention rather than open
//! two sockets in opposite directions.

#![cfg(feature = "mavlink-source")]

use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{MavAutopilot, MavMessage, MavType, HEARTBEAT_DATA};
use mavlink::{MavConnection, MavHeader};

use profiler_source::{MavlinkOptions, MavlinkSource};

fn pick_free_udp_port() -> u16 {
    let s = std::net::UdpSocket::bind("127.0.0.1:0").expect("bind 0");
    s.local_addr().expect("local_addr").port()
}

fn open_fake_vehicle(port: u16) -> mavlink::Connection<MavMessage> {
    let conn_str = format!("udpin:127.0.0.1:{port}");
    let mut conn = mavlink::connect::<MavMessage>(&conn_str).expect("vehicle bind");
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

fn send_vehicle_heartbeat(
    conn: &mavlink::Connection<MavMessage>,
    sysid: u8,
) {
    let header = MavHeader {
        system_id: sysid,
        component_id: 1,
        sequence: 0,
    };
    let hb = HEARTBEAT_DATA {
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        ..HEARTBEAT_DATA::default()
    };
    conn.send(&header, &MavMessage::HEARTBEAT(hb))
        .expect("vehicle send heartbeat");
}

#[test]
fn one_heartbeat_from_sysid_5_triggers_one_request() {
    let port = pick_free_udp_port();
    let vehicle = open_fake_vehicle(port);

    let _src = MavlinkSource::connect_with(
        &format!("udpout:127.0.0.1:{port}"),
        MavlinkOptions {
            passive: false,
            active_gcs: true,
            ..Default::default()
        },
    )
    .expect("source connect");

    // Wait for the profiler's first outbound HEARTBEAT (so worker is running).
    let _ = recv_one_with_timeout(&vehicle, Duration::from_secs(3))
        .expect("expected profiler HEARTBEAT");

    // Vehicle says hello, sysid = 5.
    send_vehicle_heartbeat(&vehicle, 5);

    // Hunt for the REQUEST_DATA_STREAM, ignoring intervening HEARTBEATs.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut found = false;
    while Instant::now() < deadline {
        #[allow(deprecated, clippy::collapsible_match)]
        if let Some((_h, MavMessage::REQUEST_DATA_STREAM(d))) =
            recv_one_with_timeout(&vehicle, Duration::from_secs(2))
        {
            assert_eq!(d.target_system, 5, "should target vehicle sysid=5");
            assert_eq!(d.start_stop, 1, "start_stop=1 = request streams to begin");
            // `req_stream_id = 0` is MAV_DATA_STREAM_ALL — the value the
            // profiler uses for the one-shot request.
            assert_eq!(d.req_stream_id, 0, "should request MAV_DATA_STREAM_ALL");
            assert!(d.req_message_rate > 0, "rate must be positive");
            found = true;
            break;
        }
    }
    assert!(
        found,
        "expected a REQUEST_DATA_STREAM aimed at sysid=5 within 5 s",
    );
}

#[test]
fn second_heartbeat_from_same_sysid_does_not_resend() {
    let port = pick_free_udp_port();
    let vehicle = open_fake_vehicle(port);

    let _src = MavlinkSource::connect_with(
        &format!("udpout:127.0.0.1:{port}"),
        MavlinkOptions {
            passive: false,
            active_gcs: true,
            ..Default::default()
        },
    )
    .expect("source connect");

    let _ = recv_one_with_timeout(&vehicle, Duration::from_secs(3))
        .expect("profiler HEARTBEAT");

    // First HEARTBEAT — request fires.
    send_vehicle_heartbeat(&vehicle, 7);
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut first_seen = false;
    while Instant::now() < deadline {
        #[allow(deprecated, clippy::collapsible_match)]
        if let Some((_h, MavMessage::REQUEST_DATA_STREAM(d))) =
            recv_one_with_timeout(&vehicle, Duration::from_secs(2))
        {
            assert_eq!(d.target_system, 7);
            first_seen = true;
            break;
        }
    }
    assert!(first_seen, "first request for sysid=7 should arrive");

    // Second HEARTBEAT from sysid=7 — the latch must suppress a duplicate.
    send_vehicle_heartbeat(&vehicle, 7);
    // Window of 2.5 s — generous enough to catch a duplicate without
    // dragging the test suite. Skip past any HEARTBEAT noise.
    let watch_deadline = Instant::now() + Duration::from_millis(2500);
    while Instant::now() < watch_deadline {
        match recv_one_with_timeout(&vehicle, Duration::from_millis(500)) {
            #[allow(deprecated)]
            Some((_h, MavMessage::REQUEST_DATA_STREAM(_))) => {
                panic!(
                    "second HEARTBEAT from sysid=7 should NOT trigger a \
                     duplicate REQUEST_DATA_STREAM (per-sysid latch broken)",
                );
            }
            _ => continue,
        }
    }
}

#[test]
fn active_gcs_off_suppresses_request_entirely() {
    let port = pick_free_udp_port();
    let vehicle = open_fake_vehicle(port);

    let _src = MavlinkSource::connect_with(
        &format!("udpout:127.0.0.1:{port}"),
        MavlinkOptions {
            passive: false,         // heartbeat sender still on…
            active_gcs: false,      // …but no stream-request.
            ..Default::default()
        },
    )
    .expect("source connect");

    let _ = recv_one_with_timeout(&vehicle, Duration::from_secs(3))
        .expect("profiler HEARTBEAT (active_gcs=false still emits heartbeats)");

    // Drive several inbound HEARTBEATs across distinct sysids — we expect
    // ZERO `REQUEST_DATA_STREAM` frames in response.
    for sysid in [1u8, 2, 3] {
        send_vehicle_heartbeat(&vehicle, sysid);
    }

    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        match recv_one_with_timeout(&vehicle, Duration::from_millis(300)) {
            #[allow(deprecated)]
            Some((_h, MavMessage::REQUEST_DATA_STREAM(_))) => {
                panic!(
                    "--no-mavlink-active-gcs must suppress every \
                     REQUEST_DATA_STREAM, even across multiple sysids",
                );
            }
            _ => continue,
        }
    }
}
