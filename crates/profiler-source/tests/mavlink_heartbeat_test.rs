//! v0.8.0 — `MavlinkSource` active-GCS behaviour:
//!
//! 1. After construction, the source must emit a HEARTBEAT to its peer
//!    within one cadence (~1.2 s in practice).
//! 2. After the peer sends a HEARTBEAT back, the source must follow up with a
//!    one-shot `REQUEST_DATA_STREAM(start=1)` aimed at that peer's
//!    system/component ids.
//! 3. With `MavlinkOptions { passive: true }`, neither outgoing message
//!    appears — the v0.4.0 listen-only contract is preserved.
//!
//! We act as a "fake vehicle" by binding a UDP socket and exchanging raw
//! MAVLink frames with the source via `MavConnection`. The source is
//! constructed with `udpout:127.0.0.1:<our-port>` so it knows our address
//! up front (no need to wait for an inbound packet to learn it).

#![cfg(feature = "mavlink-source")]

use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{MavAutopilot, MavMessage, MavType, HEARTBEAT_DATA};
use mavlink::{MavConnection, MavHeader};

use profiler_source::{MavlinkOptions, MavlinkSource};

fn pick_free_udp_port() -> u16 {
    // Bind to ephemeral port, drop the socket, return its number. There's a
    // small TOCTOU window but Windows is generally happy with it for tests.
    let s = std::net::UdpSocket::bind("127.0.0.1:0").expect("bind 0");
    s.local_addr().expect("local_addr").port()
}

fn open_fake_vehicle(port: u16) -> mavlink::Connection<MavMessage> {
    // We are the "vehicle": bind/listen on `port`. The profiler-side source
    // will connect to us with `udpout:127.0.0.1:<port>`.
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
        // mavlink-core's `recv()` is blocking — but our UDP socket is set up
        // with no read timeout, so we spin a sub-deadline by attempting
        // try_recv (non-blocking) and sleeping.
        match conn.try_recv() {
            Ok(p) => return Some(p),
            Err(_) => std::thread::sleep(Duration::from_millis(20)),
        }
    }
    None
}

#[test]
fn active_source_emits_heartbeat_to_peer() {
    let port = pick_free_udp_port();
    let vehicle = open_fake_vehicle(port);

    // Profiler-side source: udpout → connect/send to us.
    let _src = MavlinkSource::connect_with(
        &format!("udpout:127.0.0.1:{port}"),
        MavlinkOptions { passive: false, ..Default::default() },
    )
    .expect("source connect");

    // The source's heartbeat worker fires every ~1 s; allow generous slack
    // for slow CI machines (also need ~200 ms for the first sleep slice).
    let pair = recv_one_with_timeout(&vehicle, Duration::from_secs(3))
        .expect("expected a HEARTBEAT from the profiler within 3 s");

    let (header, msg) = pair;
    assert!(
        matches!(msg, MavMessage::HEARTBEAT(_)),
        "expected HEARTBEAT, got something else: {msg:?}"
    );
    // GCS identity matches the constants the source uses.
    assert_eq!(header.system_id, 255, "GCS sysid");
    assert_eq!(header.component_id, 190, "GCS component id");
    if let MavMessage::HEARTBEAT(hb) = msg {
        assert_eq!(hb.mavtype, MavType::MAV_TYPE_GCS);
        assert_eq!(hb.autopilot, MavAutopilot::MAV_AUTOPILOT_INVALID);
    }
}

#[test]
fn active_source_sends_request_data_stream_after_first_inbound_heartbeat() {
    let port = pick_free_udp_port();
    let vehicle = open_fake_vehicle(port);

    let _src = MavlinkSource::connect_with(
        &format!("udpout:127.0.0.1:{port}"),
        MavlinkOptions { passive: false, ..Default::default() },
    )
    .expect("source connect");

    // Wait for the profiler's first outbound HEARTBEAT (so its peer learning
    // is primed — udpout already knows the peer, but this also confirms the
    // worker threads are running).
    let _first = recv_one_with_timeout(&vehicle, Duration::from_secs(3))
        .expect("source HEARTBEAT");

    // Now we (the "vehicle") send a HEARTBEAT to the source. The udpin
    // socket on the source's side learns our address from the prior outbound
    // frame's source-address tracking.
    let veh_header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };
    let hb = HEARTBEAT_DATA {
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        ..HEARTBEAT_DATA::default()
    };
    vehicle
        .send(&veh_header, &MavMessage::HEARTBEAT(hb))
        .expect("vehicle send heartbeat");

    // Listen for the REQUEST_DATA_STREAM (it should arrive within one frame).
    // We may receive more HEARTBEATs along the way — skip past them.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut found = false;
    while Instant::now() < deadline {
        #[allow(deprecated, clippy::collapsible_match)]
        if let Some((_h, MavMessage::REQUEST_DATA_STREAM(d))) =
            recv_one_with_timeout(&vehicle, Duration::from_secs(2))
        {
            assert_eq!(d.target_system, 1, "should target the vehicle's sysid");
            assert_eq!(d.target_component, 1, "should target the vehicle's compid");
            assert_eq!(d.start_stop, 1, "should request streams to start");
            assert!(d.req_message_rate > 0, "should request a positive rate");
            found = true;
            break;
        }
    }
    assert!(
        found,
        "expected a REQUEST_DATA_STREAM after the vehicle's HEARTBEAT"
    );
}

#[test]
fn passive_source_emits_no_heartbeat() {
    let port = pick_free_udp_port();
    let vehicle = open_fake_vehicle(port);

    let _src = MavlinkSource::connect_with(
        &format!("udpout:127.0.0.1:{port}"),
        MavlinkOptions { passive: true, ..Default::default() },
    )
    .expect("source connect");

    // Listen for a generous 1.5 s — strictly longer than the active-mode
    // cadence — and assert NOTHING arrives.
    let got = recv_one_with_timeout(&vehicle, Duration::from_millis(1500));
    assert!(
        got.is_none(),
        "passive mode must not emit any MAVLink traffic, got {got:?}"
    );
}
