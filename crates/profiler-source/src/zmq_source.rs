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
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use crossbeam_channel::{Receiver, Sender, TrySendError};
use zeromq::{Socket, SocketRecv, SubSocket};

use crate::{flatten_msgpack_with_nulls, Sample, Source, Value};

/// Shared set of drone names this source has seen on the wire so far. Cloned
/// into the Faults panel state so the Target dropdown can populate from real
/// telemetry rather than a hardcoded `drone_1..drone_10` list.
pub type SeenDrones = Arc<RwLock<HashSet<String>>>;

/// v0.15.0 — shared most-recently-seen drone name for a source. Updated by
/// the ZMQ worker as envelopes arrive; read by the toolbar Sources dropdown
/// so the operator can tell "this is eric_1" without remembering the port.
pub type LastDroneName = Arc<RwLock<Option<Arc<str>>>>;

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
    /// v0.15.0 — most recently observed drone name on this socket. Updated by
    /// the worker each envelope; read by the Sources toolbar dropdown.
    last_drone_name: LastDroneName,
    /// v0.15.0 — stop flag the worker polls each iteration. Flipping it to
    /// `true` causes the run-loop to exit on the next recv timeout, which
    /// closes the SUB socket and lets the runtime drop. Used by the in-app
    /// `[×]` Sources button.
    stop_flag: Arc<AtomicBool>,
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
        let last_drone_name: LastDroneName = Arc::new(RwLock::new(None));
        let stop_flag = Arc::new(AtomicBool::new(false));

        let ep_for_thread = endpoint.clone();
        let seen_for_thread = Arc::clone(&seen_drones);
        let last_for_thread = Arc::clone(&last_drone_name);
        let stop_for_thread = Arc::clone(&stop_flag);
        let worker = thread::Builder::new()
            .name("profiler-zmq".into())
            .spawn(move || worker_main(
                ep_for_thread,
                tx,
                seen_for_thread,
                last_for_thread,
                stop_for_thread,
            ))
            .context("spawning ZMQ worker thread")?;

        log::info!("ZmqSource: spawned worker, connecting to {endpoint}");
        Ok(Self {
            rx,
            endpoint,
            seen_drones,
            last_drone_name,
            stop_flag,
            _worker: worker,
        })
    }

    /// Shared handle to the seen-drones set. Cloned into UI state so the
    /// Faults panel can read it each frame; the worker thread writes new
    /// names into it as envelopes arrive.
    pub fn seen_drones(&self) -> SeenDrones {
        Arc::clone(&self.seen_drones)
    }

    /// v0.15.0 — shared handle to the most-recently-seen drone name for this
    /// source. Returns `None` until at least one envelope has arrived; after
    /// that, returns the latest envelope's `drone_name`. Cloned into the
    /// Sources toolbar dropdown state so the UI can show "this is eric_1".
    pub fn last_drone_name(&self) -> LastDroneName {
        Arc::clone(&self.last_drone_name)
    }

    /// v0.15.0 — shared stop flag the worker polls each loop iteration.
    /// Setting it to `true` (via `request_stop`) causes the recv loop to
    /// exit on its next iteration so the SUB socket closes and the runtime
    /// drops. Used by the in-app `[×]` Sources button so removed sources
    /// don't leak file descriptors.
    pub fn stop_flag(&self) -> Arc<AtomicBool> {
        Arc::clone(&self.stop_flag)
    }

    /// v0.15.0 — request graceful shutdown of the worker thread. The actual
    /// shutdown happens on the next recv timeout (≤ 5 ms in the steady
    /// state); this returns immediately. Idempotent.
    pub fn request_stop(&self) {
        self.stop_flag.store(true, Ordering::Release);
    }
}

impl Drop for ZmqSource {
    fn drop(&mut self) {
        // v0.15.0 — signal the worker to shut down on drop so removing a
        // source from the registry actually closes its socket without
        // waiting for the channel to detect a disconnect.
        self.stop_flag.store(true, Ordering::Release);
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

fn worker_main(
    endpoint: String,
    tx: Sender<Sample>,
    seen: SeenDrones,
    last: LastDroneName,
    stop: Arc<AtomicBool>,
) {
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
        if let Err(e) = run_loop(&endpoint, &tx, &seen, &last, &stop).await {
            log::error!("ZmqSource worker exited: {e}");
        }
    });
}

async fn run_loop(
    endpoint: &str,
    tx: &Sender<Sample>,
    seen: &SeenDrones,
    last_drone: &LastDroneName,
    stop: &Arc<AtomicBool>,
) -> Result<()> {
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
        // v0.15.0 — check the stop flag at the top of every loop iteration so
        // removed sources release their socket within ~5 ms of the request.
        if stop.load(Ordering::Acquire) {
            log::info!(
                "ZmqSource worker: stop requested for {endpoint} \
                 (decoded={decoded}, dropped={dropped_full})"
            );
            return Ok(());
        }
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

        let (samples, null_keys) = match flatten_msgpack_with_nulls(&payload) {
            Ok(s) => s,
            Err(e) => {
                log::warn!("ZmqSource: failed to decode envelope: {e}");
                continue;
            }
        };

        // v0.11.0 — surface schema-only channels (envelope key + `null`
        // value) as sentinel-valued samples. The App's drain path detects
        // `Sample::is_schema_only` and routes them to
        // `TraceStore::note_null_key` so the editor's source-key picker
        // shows AP MAVLink mirrors before AP starts streaming.
        let drone_name_hint = samples
            .first()
            .and_then(|s| s.drone_name.clone());
        // v0.16.4 — also extract the envelope's sysid (if any) to stamp on
        // schema-only null samples below, so the picker's sysid-keyed
        // identity model stays consistent across real and null channels.
        let sysid_hint: Option<u8> = samples.first().and_then(|s| s.sysid);

        // v0.15.0 — publish the latest envelope's drone name to the shared
        // `last_drone_name` slot for the Sources toolbar dropdown. Skip the
        // write when the name hasn't changed to keep the hot path lock-free
        // in the steady state.
        if let Some(name) = &drone_name_hint {
            let already = last_drone
                .read()
                .map(|g| g.as_ref().map(|s| s.as_ref() == name.as_ref()).unwrap_or(false))
                .unwrap_or(false);
            if !already {
                if let Ok(mut g) = last_drone.write() {
                    *g = Some(Arc::clone(name));
                }
            }
        }

        let null_samples: Vec<Sample> = null_keys
            .into_iter()
            .map(|key| Sample {
                ts: samples.first().map(|s| s.ts).unwrap_or(0.0),
                key,
                value: Value::Null,
                drone_name: drone_name_hint.clone(),
                sysid: sysid_hint,
            })
            .collect();

        for s in samples.into_iter().chain(null_samples) {
            // Drone-name discovery (v0.7.0). Take the read lock first to
            // avoid the write lock on the hot path when the name is
            // already known.
            if let Some(name) = &s.drone_name {
                let known = seen.read().map(|g| g.contains(name.as_ref())).unwrap_or(true);
                if !known {
                    if let Ok(mut g) = seen.write() {
                        if g.insert(name.to_string()) {
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
