//! `FaultPublisher` — outbound ZMQ PUB for SITL runtime-control commands.
//!
//! # SITL wire format (verified against `HVN-Labs/SITL`)
//!
//! Source of truth: `external/digital-twin-python/runtime_control/`
//! (`dispatcher.py`, `command_subscriber.py`, `command_router.py`,
//! `features/_common.py`).
//!
//! - **Transport**: ZMQ `PUB → XSUB` proxy. The dispatcher binds
//!   `tcp://*:9003` (XSUB, frontend) for controllers and `tcp://*:9004`
//!   (XPUB, backend) for sims. Profiler is a controller, so it **PUBs**
//!   to `tcp://127.0.0.1:9003`.
//! - **Multipart**: `[topic_bytes, json_payload_bytes]`. ZMQ filters
//!   subscribers by the topic frame.
//! - **Topic**: drone name (e.g. `"eric"`, `"drone_1"`), a group name,
//!   or the literal `"all"` for broadcast. **Not** `"broadcast"` — the
//!   SITL CLI default is `--drone all`.
//! - **Payload**: UTF-8 JSON envelope:
//!
//!   ```json
//!   {"target": "gps|imu|mag|baro|fault", "params": {...}, "reset": false}
//!   ```
//!
//!   `params` keys come from each feature's `_KNOWN_PARAMS`:
//!
//!   | feature | params (subset)                                              |
//!   |---------|--------------------------------------------------------------|
//!   | `gps`   | `sigma_p`, `sigma_v`, `_e: [N,E,D]`, `tau`, `ref_lat/lon/alt`|
//!   | `imu`   | `b_a: [x,y,z]`, `b_g: [x,y,z]`, `sigma_a_n`, `sigma_g_n`,    |
//!   |         | `sigma_bi_a`, `sigma_bi_g`, `tau_b_a`, `tau_b_g`             |
//!   | `mag`   | `hard_iron: [x,y,z]`, `sigma: [x,y,z]`, `soft_iron`, `k_emi`,|
//!   |         | `B_ned`                                                      |
//!   | `baro`  | `sigma_pa`, `sigma_bias_rw`, `bias_pa` (DT MagModel/BaroModel|
//!   |         | also accepts `solder_drift_pa`, `sigma_pa_rms` from the      |
//!   |         | matplotlib panel — both names work via partial-update merge) |
//!   | `fault` | full schedule envelope: `{target, param, profile, mode,       |
//!   |         | t_duration, params:{...}, axis?}`                            |
//!
//!   `"reset": true` reverts the whole sensor to its startup_config values
//!   (params then ignored).
//!
//! # Representative wire bytes
//!
//! Multipart message sent over the wire for a GPS noise tweak to drone `eric`:
//!
//! ```text
//! frame 0: b"eric"
//! frame 1: b"{\"target\":\"gps\",\"params\":{\"sigma_p\":0.5}}"
//! ```
//!
//! For broadcast it's just `b"all"` as the topic.
//!
//! # Why JSON, not msgpack
//!
//! Confirmed in `command_subscriber.py` (`json.loads(payload.decode())`).
//! The streamer's *outbound* envelopes (the ones profiler-source already
//! decodes via `ZmqSource`) are msgpack — but the **inbound** runtime-control
//! path is JSON. Different schema, different encoding.
//!
//! # Threading
//!
//! `zeromq` is async-only, so we own a dedicated single-thread tokio runtime
//! identical in shape to `ZmqSource`. Commands queue through a sync
//! [`crossbeam_channel`] and the async worker pumps them out on the PUB
//! socket. This way the egui main loop never blocks on the network.
//!
//! On `close()` (or drop) we send a sentinel and the worker exits cleanly.

use std::collections::HashMap;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use crossbeam_channel::{Receiver, Sender};
use serde::Serialize;
use serde_json::Value;
use zeromq::{PubSocket, Socket, SocketSend, ZmqMessage};

/// One runtime-control command bound for SITL.
///
/// Field names mirror the SITL receiver envelope, with `drone` standing in
/// for the multipart ZMQ topic frame.
#[derive(Debug, Clone, PartialEq)]
pub struct FaultCommand {
    /// SITL feature key: `"gps"`, `"imu"`, `"mag"`, `"baro"`, `"fault"`.
    pub feature: String,
    /// ZMQ topic — drone name, group, or `"all"` for broadcast.
    pub drone: String,
    /// Logical command name. **Today this is a passthrough hint only** — the
    /// SITL dispatcher routes by `feature` (= `target`) and merges `args`
    /// into the receiver via partial update. We keep `command` on the type
    /// so the panel can express intent (e.g. `"set_sigma_p"`, `"reset"`,
    /// `"gps_dropout"`) and so a future v0.7.x can switch wire formats
    /// without breaking the panel API. An empty string means "merge args
    /// directly".
    pub command: String,
    /// Key/value pairs for the `params` field of the envelope. Arrays are
    /// `Value::Array(...)` (e.g. 3-vec biases like `b_a`, `hard_iron`).
    pub args: HashMap<String, Value>,
    /// When `true`, sends `"reset": true` and ignores `args` server-side.
    /// Matches the matplotlib panel's per-sensor Reset button.
    pub reset: bool,
}

impl FaultCommand {
    /// Convenience constructor for "set these params, no reset, no special
    /// command name". The typical sliders-changed path.
    pub fn set(feature: &str, drone: &str, args: HashMap<String, Value>) -> Self {
        Self {
            feature: feature.into(),
            drone: drone.into(),
            command: String::new(),
            args,
            reset: false,
        }
    }

    /// Convenience constructor for the per-sensor Reset button.
    pub fn reset(feature: &str, drone: &str) -> Self {
        Self {
            feature: feature.into(),
            drone: drone.into(),
            command: "reset".into(),
            args: HashMap::new(),
            reset: true,
        }
    }
}

/// JSON payload envelope serialised to frame 1 of the multipart message.
///
/// Matches `runtime_control/features/_common.publish()` exactly:
/// `{"target": ..., "params": {...}, "reset": true?}`.
#[derive(Debug, Serialize)]
struct Envelope<'a> {
    target: &'a str,
    params: &'a HashMap<String, Value>,
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    reset: bool,
}

/// Encode a [`FaultCommand`] into the `(topic, json_payload)` pair that goes
/// on the wire. Public so the tests in `fault_publisher_test.rs` (and the
/// faults panel's unit tests) can exercise the encoder without spinning up
/// a real socket.
pub fn encode_command(cmd: &FaultCommand) -> Result<(Vec<u8>, Vec<u8>)> {
    let env = Envelope {
        target: &cmd.feature,
        params: &cmd.args,
        reset: cmd.reset,
    };
    let json = serde_json::to_vec(&env).context("encoding FaultCommand to JSON")?;
    Ok((cmd.drone.as_bytes().to_vec(), json))
}

/// Sentinel message that asks the worker to exit. We model this as a private
/// enum on the channel rather than dropping the sender, so an explicit
/// `close()` returns only after the socket is shut down.
enum WorkerMsg {
    Send(FaultCommand),
    Shutdown,
}

/// Outbound runtime-control publisher. Cheaply cloneable (handle pattern —
/// the channel sender is the actual cheap part; the worker thread is owned
/// by the original `FaultPublisher`).
pub struct FaultPublisher {
    tx: Sender<WorkerMsg>,
    endpoint: String,
    /// `Option` so `close()` can `join` the worker. `None` after close/drop.
    worker: Option<thread::JoinHandle<()>>,
}

impl FaultPublisher {
    /// Spawn the publisher worker and start connecting to `endpoint`
    /// (e.g. `tcp://127.0.0.1:9003`). Returns immediately — the connection
    /// completes asynchronously, and `send()` queues are buffered until then
    /// (with a brief ZMQ slow-joiner sleep inside the worker, matching the
    /// SITL Python `make_pub` helper).
    pub fn new(endpoint: &str) -> Result<Self> {
        let endpoint = endpoint.to_string();
        let (tx, rx) = crossbeam_channel::unbounded::<WorkerMsg>();

        let ep_for_thread = endpoint.clone();
        let worker = thread::Builder::new()
            .name("profiler-fault-pub".into())
            .spawn(move || worker_main(ep_for_thread, rx))
            .context("spawning FaultPublisher worker thread")?;

        log::info!("FaultPublisher: spawned worker, connecting to {endpoint}");
        Ok(Self {
            tx,
            endpoint,
            worker: Some(worker),
        })
    }

    /// Queue a command for transmission. Non-blocking; the worker drains
    /// the channel as fast as ZMQ permits. Returns `Err` only if the
    /// worker has already exited.
    pub fn send(&self, cmd: &FaultCommand) -> Result<()> {
        self.tx
            .send(WorkerMsg::Send(cmd.clone()))
            .context("FaultPublisher worker is gone")
    }

    /// Endpoint this publisher was configured with.
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }

    /// Drop the publisher, waiting up to ~250 ms for the worker to flush
    /// in-flight commands and close the socket cleanly.
    pub fn close(mut self) {
        let _ = self.tx.send(WorkerMsg::Shutdown);
        if let Some(w) = self.worker.take() {
            let _ = w.join();
        }
    }
}

impl Drop for FaultPublisher {
    fn drop(&mut self) {
        // Best-effort: signal the worker to stop. We don't join — drop must
        // not block indefinitely if the worker is wedged. `close()` is the
        // explicit-graceful path.
        let _ = self.tx.send(WorkerMsg::Shutdown);
    }
}

fn worker_main(endpoint: String, rx: Receiver<WorkerMsg>) {
    let rt = match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => {
            log::error!("FaultPublisher worker: tokio runtime build failed: {e}");
            return;
        }
    };
    rt.block_on(async move {
        if let Err(e) = run_loop(&endpoint, rx).await {
            log::error!("FaultPublisher worker exited with error: {e}");
        }
    });
}

async fn run_loop(endpoint: &str, rx: Receiver<WorkerMsg>) -> Result<()> {
    let mut sock = PubSocket::new();
    sock.connect(endpoint)
        .await
        .with_context(|| format!("PubSocket::connect({endpoint})"))?;
    log::info!("FaultPublisher worker: connected to {endpoint}");

    // ZMQ slow-joiner: the SITL `make_pub` helper sleeps 200 ms; mirror that
    // so the first send after construction isn't silently dropped against
    // an XSUB that's still completing its handshake.
    tokio::time::sleep(Duration::from_millis(200)).await;

    let mut sent: u64 = 0;
    let mut errors: u64 = 0;
    loop {
        // Bridge the sync crossbeam channel into the async loop without
        // `block_in_place` (which requires a multi-thread runtime; we run
        // a current-thread runtime here, same as `ZmqSource`). A short
        // periodic poll is fine — this is a low-rate channel (slider
        // debounce ≥50 ms) so we don't need lower latency than ~10 ms.
        let msg = loop {
            match rx.try_recv() {
                Ok(m) => break m,
                Err(crossbeam_channel::TryRecvError::Empty) => {
                    tokio::time::sleep(Duration::from_millis(10)).await;
                    continue;
                }
                Err(crossbeam_channel::TryRecvError::Disconnected) => {
                    log::info!(
                        "FaultPublisher worker: channel closed; exiting \
                         (sent={sent} errors={errors})"
                    );
                    return Ok(());
                }
            }
        };

        let cmd = match msg {
            WorkerMsg::Send(c) => c,
            WorkerMsg::Shutdown => {
                log::info!(
                    "FaultPublisher worker: shutdown signal (sent={sent} errors={errors})"
                );
                return Ok(());
            }
        };

        let (topic, json) = match encode_command(&cmd) {
            Ok(p) => p,
            Err(e) => {
                log::warn!("FaultPublisher: encode failed for {cmd:?}: {e}");
                errors += 1;
                continue;
            }
        };

        // Multipart: frame 0 = topic, frame 1 = JSON payload.
        let mut zmsg = ZmqMessage::from(topic);
        zmsg.push_back(json.into());

        match sock.send(zmsg).await {
            Ok(()) => sent += 1,
            Err(e) => {
                log::warn!("FaultPublisher: send failed for feature={}: {e}", cmd.feature);
                errors += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn encode_command_matches_sitl_envelope() {
        let mut args = HashMap::new();
        args.insert("sigma_p".into(), json!(0.5));
        args.insert("_e".into(), json!([1.0, 0.0, -2.0]));
        let cmd = FaultCommand::set("gps", "eric", args);
        let (topic, payload) = encode_command(&cmd).unwrap();
        assert_eq!(topic, b"eric");
        let v: serde_json::Value = serde_json::from_slice(&payload).unwrap();
        assert_eq!(v["target"], "gps");
        assert_eq!(v["params"]["sigma_p"], 0.5);
        assert_eq!(v["params"]["_e"], json!([1.0, 0.0, -2.0]));
        // No reset field on a non-reset envelope (matches SITL _common.publish).
        assert!(v.get("reset").is_none());
    }

    #[test]
    fn encode_reset_includes_reset_true() {
        let cmd = FaultCommand::reset("mag", "all");
        let (topic, payload) = encode_command(&cmd).unwrap();
        assert_eq!(topic, b"all");
        let v: serde_json::Value = serde_json::from_slice(&payload).unwrap();
        assert_eq!(v["target"], "mag");
        assert_eq!(v["reset"], true);
    }

    #[test]
    fn publisher_constructs_and_queues_send_without_panic() {
        // No subscriber present — send must still succeed (PUB drops on the
        // floor if nobody is listening, by design).
        let pub_ = FaultPublisher::new("tcp://127.0.0.1:59933").expect("construct");
        let cmd = FaultCommand::set("gps", "all", HashMap::new());
        pub_.send(&cmd).expect("queue");
        pub_.close();
    }
}
