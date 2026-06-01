//! Integration test: round-trip a [`FaultCommand`] through a paired ZMQ
//! PUB → SUB loopback and assert the decoded envelope matches the SITL
//! receiver schema documented in `fault_publisher.rs`.

#![cfg(feature = "fault-channel")]

use std::collections::HashMap;
use std::time::Duration;

use profiler_source::{FaultCommand, FaultPublisher};
use serde_json::{json, Value};
use zeromq::{Socket, SocketOptions, SocketRecv, SubSocket, ZmqMessage};

/// Pick a port the OS is unlikely to be using already. The test creates a
/// loopback PUB→SUB pair, so the only requirement is that nothing else is
/// listening here on the same Windows session at test time.
const TEST_ENDPOINT: &str = "tcp://127.0.0.1:59921";

/// Drain one SUB message into `(topic, payload)`. Times out after `wait` to
/// keep the test bounded even if PUB → SUB never connects.
async fn recv_one(sub: &mut SubSocket, wait: Duration) -> Option<(Vec<u8>, Vec<u8>)> {
    match tokio::time::timeout(wait, sub.recv()).await {
        Ok(Ok(msg)) => Some(zmsg_to_pair(msg)),
        _ => None,
    }
}

fn zmsg_to_pair(msg: ZmqMessage) -> (Vec<u8>, Vec<u8>) {
    let frames: Vec<_> = msg.into_vec();
    assert!(
        frames.len() >= 2,
        "expected multipart [topic, payload]; got {} frames",
        frames.len()
    );
    (frames[0].to_vec(), frames[1].to_vec())
}

#[test]
fn fault_publisher_round_trips_gps_command() {
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();

    rt.block_on(async move {
        // SUB side: bind first so the PUB connect handshake completes before
        // we send. (The PUB→SUB direction in zeromq 0.6 doesn't care which
        // side binds; binding the SUB keeps the test deterministic.)
        let mut sub = SubSocket::with_options(SocketOptions::default());
        sub.bind(TEST_ENDPOINT).await.expect("SUB bind");
        sub.subscribe("").await.expect("SUB subscribe(\"\")");

        // PUB side: profiler's FaultPublisher in connect mode.
        let pub_ = FaultPublisher::new(TEST_ENDPOINT).expect("FaultPublisher::new");

        // Give the slow-joiner sleep inside the worker time to elapse, plus
        // a margin for the handshake.
        tokio::time::sleep(Duration::from_millis(400)).await;

        let mut args: HashMap<String, Value> = HashMap::new();
        args.insert("sigma_p".into(), json!(0.42));
        args.insert("_e".into(), json!([1.5, -0.5, 0.0]));
        let cmd = FaultCommand::set("gps", "eric", args);
        pub_.send(&cmd).expect("queue send");

        let (topic, payload) = recv_one(&mut sub, Duration::from_secs(2))
            .await
            .expect("expected one multipart frame on the SUB side");

        assert_eq!(topic, b"eric", "topic frame must equal the drone name");

        let env: Value = serde_json::from_slice(&payload).expect("payload is JSON");
        assert_eq!(env["target"], "gps", "feature → target");
        assert_eq!(env["params"]["sigma_p"], 0.42);
        assert_eq!(env["params"]["_e"], json!([1.5, -0.5, 0.0]));
        assert!(
            env.get("reset").is_none(),
            "reset key must be omitted when not resetting (matches SITL _common.publish)"
        );

        // And a reset envelope rounds-trips with reset:true and target=mag.
        pub_.send(&FaultCommand::reset("mag", "all")).expect("queue reset");
        let (topic2, payload2) = recv_one(&mut sub, Duration::from_secs(2))
            .await
            .expect("expected reset frame");
        assert_eq!(topic2, b"all");
        let env2: Value = serde_json::from_slice(&payload2).unwrap();
        assert_eq!(env2["target"], "mag");
        assert_eq!(env2["reset"], true);

        pub_.close();
    });
}
