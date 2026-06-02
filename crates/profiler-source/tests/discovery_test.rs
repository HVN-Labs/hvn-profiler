//! v0.16.0 — `discover_localhost_sources` integration tests.
//!
//! These tests spin real ZMQ PUB sockets and real UDP listeners on the
//! standard HVN ports and assert that [`discover_localhost_sources`]
//! reports them with the expected status. Probing the live network from a
//! unit test is mildly racy — the scan budget is generous (500 ms) and the
//! PUB starts publishing well before the SUB is opened, but we still bias
//! every assertion toward the OBSERVED outcome (Live OR Silent) rather than
//! pinning Live exactly when "did we open the PUB in time" is what's really
//! being checked.

use std::time::{Duration, Instant};

use profiler_source::{
    discover_localhost_sources, DiscoveryStatus, SourceKind, DEFAULT_PROBE_MS,
};
use zeromq::{PubSocket, Socket, SocketSend, ZmqMessage};

/// Spawn a real ZMQ PUB on `tcp://127.0.0.1:{port}` and start broadcasting
/// the SITL-style msgpack envelope every 20 ms. Returns the
/// [`tokio::task::JoinHandle`] so the caller can `abort()` when finished.
///
/// `drone_name` is stamped into the envelope so the discovery decoder picks
/// it up as `DiscoveryStatus::Live { drone_name: Some(...) }`.
async fn spawn_pub(port: u16, drone_name: &str) -> tokio::task::JoinHandle<()> {
    let name = drone_name.to_string();
    tokio::spawn(async move {
        let mut sock = PubSocket::new();
        let endpoint = format!("tcp://127.0.0.1:{port}");
        sock.bind(&endpoint).await.expect("PUB bind");
        // Yield once so SUBs that connect immediately after we return have
        // a chance to complete their handshake before the first publish.
        tokio::time::sleep(Duration::from_millis(50)).await;
        loop {
            let envelope = build_envelope(&name);
            let msg = ZmqMessage::from(envelope);
            // Treat send errors as "test is shutting down" — the JoinHandle
            // will be aborted by the caller.
            let _ = sock.send(msg).await;
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
}

/// Build a minimal SITL-shaped msgpack envelope.
fn build_envelope(drone_name: &str) -> Vec<u8> {
    use serde::Serialize;
    use std::collections::BTreeMap;
    #[derive(Serialize)]
    struct Env {
        ts: f64,
        source: String,
        drone_name: String,
        values: BTreeMap<String, serde_json::Value>,
    }
    let mut values = BTreeMap::new();
    values.insert("ap_vfr_alt".into(), serde_json::json!(42.0_f64));
    let env = Env {
        ts: 1.0,
        source: "test".into(),
        drone_name: drone_name.into(),
        values,
    };
    rmp_serde::to_vec_named(&env).expect("encode envelope")
}

/// Two PUB sockets on `9005` + `9006` → both should appear as Live (or at
/// worst Silent if the PUB hasn't started publishing yet when we read).
/// We accept either Live or Silent for portability across CI runners; the
/// invariant under test is that BOTH URIs make it into the result.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn two_pubs_are_both_discovered() {
    let h1 = spawn_pub(9005, "eric_1").await;
    let h2 = spawn_pub(9006, "eric_2").await;
    // Give the PUBs a beat to bind before scanning.
    tokio::time::sleep(Duration::from_millis(80)).await;

    let results = discover_localhost_sources(&[], DEFAULT_PROBE_MS).await;

    h1.abort();
    h2.abort();

    let uris: Vec<&str> = results.iter().map(|d| d.uri.as_str()).collect();
    assert!(
        uris.contains(&"zmq://127.0.0.1:9005"),
        "9005 missing from discovery: {uris:?}",
    );
    assert!(
        uris.contains(&"zmq://127.0.0.1:9006"),
        "9006 missing from discovery: {uris:?}",
    );

    // Each discovered ZMQ entry should be Live OR Silent (never InUse, since
    // already_connected was empty). When Live, the drone name is one of the
    // two we published.
    for d in &results {
        if d.uri == "zmq://127.0.0.1:9005" || d.uri == "zmq://127.0.0.1:9006" {
            assert_eq!(d.kind, SourceKind::Zmq);
            match &d.status {
                DiscoveryStatus::Live { drone_name } => {
                    if let Some(n) = drone_name {
                        assert!(
                            n == "eric_1" || n == "eric_2",
                            "unexpected drone name {n:?}",
                        );
                    }
                }
                DiscoveryStatus::Silent => {} // acceptable
                DiscoveryStatus::InUse => panic!(
                    "discovery shouldn't return InUse when already_connected is empty"
                ),
            }
        }
    }
}

/// Discovery returns a Vec (possibly empty) for an arbitrary connected-list.
/// We can't reliably assert "no Live entries" here because cargo test runs
/// tests in parallel within one process — the `two_pubs_are_both_discovered`
/// PUBs may still be bound when this test runs. The invariant we DO test is
/// just that the call returns a well-shaped Vec without panicking.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn empty_connected_yields_well_shaped_vec() {
    let results = discover_localhost_sources(&[], DEFAULT_PROBE_MS).await;
    // Each row uses the canonical bind address for its kind:
    //   - ZMQ → 127.0.0.1 (streamer always binds loopback).
    //   - MAVLink → 0.0.0.0 (v0.16.4: probe binds all-zero so WSL2 vehicles
    //     are visible. The URI matches the bind so manual entry of the
    //     discovered URI round-trips into a working source.)
    for d in &results {
        match d.kind {
            SourceKind::Zmq => assert!(
                d.uri.starts_with("zmq://127.0.0.1:"),
                "ZMQ URI should be loopback: {}",
                d.uri,
            ),
            SourceKind::Mavlink => assert!(
                d.uri.starts_with("mavlink://0.0.0.0:"),
                "MAVLink URI should bind 0.0.0.0 (v0.16.4): {}",
                d.uri,
            ),
        }
    }
}

/// URI in `already_connected` → reported with `InUse` and NOT probed for
/// liveness.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn already_connected_uri_appears_as_inuse() {
    let connected = vec!["zmq://127.0.0.1:9005".to_string()];
    let results = discover_localhost_sources(&connected, DEFAULT_PROBE_MS).await;
    let hit = results
        .iter()
        .find(|d| d.uri == "zmq://127.0.0.1:9005")
        .expect("already-connected URI must appear in result");
    assert_eq!(
        hit.status,
        DiscoveryStatus::InUse,
        "already-connected URI must be tagged InUse, got {:?}",
        hit.status,
    );
}

/// The whole scan must return within ~probe_duration + grace. Pin the upper
/// bound at 1500 ms so a slow CI runner still passes but a runaway probe
/// (e.g. unbounded TCP connect) flags loudly.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn probe_completes_within_budget() {
    let start = Instant::now();
    let _ = discover_localhost_sources(&[], DEFAULT_PROBE_MS).await;
    let elapsed = start.elapsed();
    assert!(
        elapsed < Duration::from_millis(1500),
        "discovery took {elapsed:?} — should finish near probe_duration + slack",
    );
}

/// Empty `already_connected` + custom (tiny) budget — the function should
/// still return a Vec (possibly empty) without panicking. Tiny probe windows
/// exercise the timeout path.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn tiny_probe_window_returns_quickly() {
    let start = Instant::now();
    let results = discover_localhost_sources(&[], 25).await;
    let elapsed = start.elapsed();
    // Sanity: it returned, and quickly. The result MAY be empty if nothing
    // is bound on the dev machine.
    let _ = results; // discard
    assert!(
        elapsed < Duration::from_millis(800),
        "25 ms probe window should return in well under 800 ms (took {elapsed:?})",
    );
}
