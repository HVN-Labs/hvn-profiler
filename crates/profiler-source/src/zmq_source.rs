//! `ZmqSource` — pure-Rust SUB backend for the SITL msgpack streamer.
//!
//! The [`zeromq`] crate is async-only, so we spin up a single-threaded
//! [`tokio`] runtime in a dedicated worker thread, connect a `SubSocket`
//! there, decode each msgpack frame into samples, and forward those samples
//! through a [`crossbeam_channel`] back to the synchronous render loop.
//!
//! Bridging async → sync this way (rather than the other direction) keeps
//! the eframe/egui mainloop free of any async machinery — `try_recv` is a
//! plain non-blocking channel pop.

use std::collections::HashSet;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use crossbeam_channel::{Receiver, Sender, TrySendError};
use zeromq::{Socket, SocketRecv, SubSocket};

use crate::{flatten_msgpack, Sample, Source};

/// Shared set of drone names this source has seen on the wire so far. Cloned
/// into the Faults panel state so the Target dropdown can populate from real
/// telemetry rather than a hardcoded `drone_1..drone_10` list.
pub type SeenDrones = Arc<RwLock<HashSet<String>>>;

/// Channel capacity. ~5 s of headroom even at 1 kHz envelopes × ~20 keys.
const CHANNEL_CAPACITY: usize = 100_000;

/// A SUB-side subscriber for the SITL streamer's msgpack-over-ZMQ wire format.
pub struct ZmqSource {
    rx: Receiver<Sample>,
    endpoint: String,
    /// Names of drones whose envelopes have arrived on this socket. Shared
    /// (Arc<RwLock<_>>) with the Faults panel so its target dropdown can
    /// reflect what's actually streaming.
    seen_drones: SeenDrones,
    /// Kept alive so the worker thread (and tokio runtime) shut down when
    /// `ZmqSource` is dropped. We don't actually `join` — letting the
    /// thread drop is fine because the runtime owns no shared state.
    _worker: thread::JoinHandle<()>,
}

impl ZmqSource {
    /// Spawn the worker, connect to `endpoint` (e.g. `tcp://127.0.0.1:9005`).
    ///
    /// Connection is async; this function returns as soon as the worker is
    /// up — it does NOT wait for the publisher to be reachable. Samples
    /// start arriving once the SUB socket completes its handshake.
    pub fn connect(endpoint: &str) -> Result<Self> {
        let endpoint = endpoint.to_string();
        let (tx, rx) = crossbeam_channel::bounded::<Sample>(CHANNEL_CAPACITY);
        let seen_drones: SeenDrones = Arc::new(RwLock::new(HashSet::new()));

        let ep_for_thread = endpoint.clone();
        let seen_for_thread = Arc::clone(&seen_drones);
        let worker = thread::Builder::new()
            .name("profiler-zmq".into())
            .spawn(move || worker_main(ep_for_thread, tx, seen_for_thread))
            .context("spawning ZMQ worker thread")?;

        log::info!("ZmqSource: spawned worker, connecting to {endpoint}");
        Ok(Self {
            rx,
            endpoint,
            seen_drones,
            _worker: worker,
        })
    }

    /// Shared handle to the seen-drones set. Cloned into UI state so the
    /// Faults panel can read it each frame; the worker thread writes new
    /// names into it as envelopes arrive.
    pub fn seen_drones(&self) -> SeenDrones {
        Arc::clone(&self.seen_drones)
    }
}

impl Source for ZmqSource {
    fn try_recv(&mut self) -> Option<Sample> {
        self.rx.try_recv().ok()
    }

    fn describe(&self) -> String {
        format!("zmq:// {} (msgpack)", self.endpoint)
    }
}

fn worker_main(endpoint: String, tx: Sender<Sample>, seen: SeenDrones) {
    // Single-threaded runtime is plenty — one socket, one decode loop.
    let rt = match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => {
            log::error!("ZmqSource worker: tokio runtime build failed: {e}");
            return;
        }
    };
    rt.block_on(async move {
        if let Err(e) = run_loop(&endpoint, &tx, &seen).await {
            log::error!("ZmqSource worker exited: {e}");
        }
    });
}

async fn run_loop(endpoint: &str, tx: &Sender<Sample>, seen: &SeenDrones) -> Result<()> {
    let mut sock = SubSocket::new();
    // Subscribe to ALL messages — the streamer doesn't topic-prefix today.
    sock.subscribe("")
        .await
        .context("SubSocket::subscribe(\"\")")?;
    sock.connect(endpoint)
        .await
        .with_context(|| format!("SubSocket::connect({endpoint})"))?;
    log::info!("ZmqSource worker: connected to {endpoint}");

    let mut decoded = 0u64;
    let mut dropped_full = 0u64;
    loop {
        let msg = match sock.recv().await {
            Ok(m) => m,
            Err(zeromq::ZmqError::NoMessage) => {
                tokio::time::sleep(Duration::from_millis(5)).await;
                continue;
            }
            Err(e) => {
                log::error!("ZmqSource recv error: {e}");
                return Err(anyhow::anyhow!("ZMQ recv failed: {e}"));
            }
        };

        // PUB → SUB typically sends one frame per envelope. If we ever see
        // a multi-frame envelope, concatenate.
        let payload: Vec<u8> = if msg.len() == 1 {
            msg.get(0).map(|b| b.to_vec()).unwrap_or_default()
        } else {
            let mut buf = Vec::new();
            for frame in msg.iter() {
                buf.extend_from_slice(frame);
            }
            buf
        };

        let samples = match flatten_msgpack(&payload) {
            Ok(s) => s,
            Err(e) => {
                log::warn!("ZmqSource: failed to decode envelope: {e}");
                continue;
            }
        };

        for s in samples {
            // Drone-name discovery (v0.7.0). Take the read lock first to
            // avoid the write lock on the hot path when the name is
            // already known.
            if let Some(name) = &s.drone_name {
                let known = seen.read().map(|g| g.contains(name)).unwrap_or(true);
                if !known {
                    if let Ok(mut g) = seen.write() {
                        if g.insert(name.clone()) {
                            log::info!("ZmqSource: discovered drone '{name}'");
                        }
                    }
                }
            }
            match tx.try_send(s) {
                Ok(()) => decoded += 1,
                Err(TrySendError::Full(_)) => {
                    dropped_full += 1;
                    if dropped_full.is_power_of_two() {
                        log::warn!(
                            "ZmqSource: channel full, dropped {dropped_full} samples \
                             (render loop is falling behind)"
                        );
                    }
                }
                Err(TrySendError::Disconnected(_)) => {
                    log::info!(
                        "ZmqSource: receiver dropped, exiting worker \
                         (decoded={decoded}, dropped={dropped_full})"
                    );
                    return Ok(());
                }
            }
        }
    }
}
