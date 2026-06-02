//! `MavlinkSource` — direct MAVLink-over-UDP backend for real drones / SITL.
//!
//! v0.4.0 lets the profiler read an ArduPilot vehicle (or a real flight
//! controller) straight off the wire, with no Python streamer in between.
//!
//! The [`mavlink`] crate's I/O is *synchronous*, so — unlike [`ZmqSource`],
//! which needs a tokio runtime — we just spawn a plain [`std::thread`] that
//! owns the [`mavlink::MavConnection`], loops on `recv()`, decodes each
//! message into flattened [`Sample`]s, and forwards them through a
//! [`crossbeam_channel`] back to the synchronous render loop. `try_recv` on
//! the main thread is a non-blocking channel pop, identical to the ZMQ path.
//!
//! ## Connection strings
//! - `mavlink://HOST:PORT`    → `udpin:HOST:PORT`  — bind/listen (passive).
//!   This is the default and matches how ArduPilot's `udpclient` output and
//!   the Python streamer's `MavlinkSourceBackend` pair up: the vehicle sends
//!   to us, we listen.
//! - `mavlinkout://HOST:PORT` → `udpout:HOST:PORT` — connect/send-first, for
//!   setups where the profiler must initiate the conversation.
//!
//! ## Dialect
//! We decode with [`mavlink::ardupilotmega::MavMessage`], which supersets
//! `common` — so every `common` message (ATTITUDE, LOCAL_POSITION_NED, …)
//! still decodes, plus any ArduPilot-specific ones we might want later.
//!
//! ## Heartbeat / peer-learning
//! `udpin` is passive: we never have to send a heartbeat to "wake" the link
//! at the *socket* level — the kernel accepts whatever lands on the bound
//! port. But many ArduPilot vehicles only **start streaming the rich
//! messages** (ATTITUDE / LOCAL_POSITION_NED / RAW_IMU / VFR_HUD) after
//! a GCS sends them a HEARTBEAT and/or `REQUEST_DATA_STREAM`. Stock
//! ArduPilot serial output only sends `GLOBAL_POSITION_INT`, `GPS_RAW_INT`,
//! `SYS_STATUS`, and `HEARTBEAT` by default. (See the
//! `profiler-mavlink-stream-gap` learning.)
//!
//! v0.8.0 closes that gap by default:
//! - The worker sends a 1 Hz GCS HEARTBEAT (system 255, component 190,
//!   `MAV_TYPE_GCS`, `MAV_AUTOPILOT_INVALID`) on the same socket as long as
//!   it is running.
//! - After the **first inbound HEARTBEAT** we send a one-shot
//!   `REQUEST_DATA_STREAM(stream=ALL, rate=10 Hz, start_stop=1)` aimed at the
//!   peer's system / component IDs. This wakes the rich-message stream on
//!   vehicles that otherwise stay quiet.
//! - The decoder also handles `GLOBAL_POSITION_INT` and `GPS_RAW_INT` so a
//!   stock-stream vehicle has at least position + GPS plotted.
//!
//! Both behaviours can be disabled by passing `--mavlink-passive on`
//! (`MavlinkOptions { passive: true }`) — useful when sharing a port with
//! another GCS that's already issuing the stream requests, or when listening
//! to a `mavlinkrouter` fan-out that mustn't see profiler traffic.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;

use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use crossbeam_channel::{Receiver, Sender, TrySendError};
use mavlink::dialects::ardupilotmega::{MavAutopilot, MavMessage, MavType, HEARTBEAT_DATA};
// `REQUEST_DATA_STREAM` is marked deprecated upstream in favour of
// `MAV_CMD_SET_MESSAGE_INTERVAL`, but ArduPilot still honours it and that's
// what every working real-drone GCS we've seen sends. Allow the warning.
#[allow(deprecated)]
use mavlink::dialects::ardupilotmega::REQUEST_DATA_STREAM_DATA;
use mavlink::{MavConnection, MavHeader};

use crate::{Sample, Source};

/// Channel capacity. A real vehicle streams ~10–50 Hz across a handful of
/// message types (≈ tens of keys per second), so this is ~hours of headroom —
/// but we keep it large for parity with [`ZmqSource`] and burst tolerance.
const CHANNEL_CAPACITY: usize = 100_000;

/// MAVLink GCS identity used when v0.8.0 sends a 1 Hz HEARTBEAT / one-shot
/// `REQUEST_DATA_STREAM`. `255` is the historical GCS sysid; `190` is the
/// reserved "GCS component" id.
pub const GCS_SYSTEM_ID: u8 = 255;
pub const GCS_COMPONENT_ID: u8 = 190;

/// Heartbeat cadence. ArduPilot considers a GCS lost after ~3 s of silence.
const HEARTBEAT_PERIOD: Duration = Duration::from_secs(1);

/// `REQUEST_DATA_STREAM` payload defaults used after the first inbound
/// HEARTBEAT in non-passive mode. Stream id `0` = `MAV_DATA_STREAM_ALL`.
const REQUEST_STREAM_ALL: u8 = 0;
const REQUEST_STREAM_RATE_HZ: u16 = 10;

/// v0.8.0 — runtime knobs for [`MavlinkSource`]. `passive=true` falls back to
/// the v0.4.0 listen-only behaviour (no heartbeat sender, no stream request).
///
/// `Default` is `passive=false`: the v0.8.0 active-GCS behaviour. Toggle to
/// `passive=true` via `--mavlink-passive on` on the CLI when sharing a
/// socket with another GCS that already drives stream requests.
///
/// v0.10.0 — `drone_name_override` pins the per-sample `drone_name` to a fixed
/// string (the operator's `--drone NAME`), suppressing the default
/// `system_id`-derived `sysid_<id>` naming. Useful when the operator knows
/// there's only one vehicle on the link and wants a friendly label.
#[derive(Debug, Clone, Default)]
pub struct MavlinkOptions {
    /// When `true`, skip the 1 Hz HEARTBEAT and the `REQUEST_DATA_STREAM` —
    /// behave exactly like v0.4.0.
    pub passive: bool,
    /// v0.10.0 — when `Some(name)`, every emitted `Sample.drone_name` is set
    /// to `name` regardless of the inbound MAVLink frame's `system_id`. When
    /// `None`, samples carry `sysid_<id>` derived from the frame header.
    pub drone_name_override: Option<String>,
}

/// A direct MAVLink-over-UDP source. Spawns a worker thread that owns the
/// connection and decodes messages into [`Sample`]s.
pub struct MavlinkSource {
    rx: Receiver<Sample>,
    /// The `mavlink`-crate connection string actually used (e.g.
    /// `udpin:127.0.0.1:14560`), for the status bar / window title.
    conn_str: String,
    /// Flips to `false` on drop so the receiver thread + optional heartbeat
    /// thread can shut down promptly.
    stop_flag: Arc<AtomicBool>,
    /// Receiver worker — exits when `stop_flag` flips or the channel closes.
    _recv_worker: thread::JoinHandle<()>,
    /// Heartbeat sender worker — `None` in passive mode.
    _hb_worker: Option<thread::JoinHandle<()>>,
}

impl MavlinkSource {
    /// Spawn the worker and open `conn_str` (a `mavlink`-crate address such
    /// as `udpin:127.0.0.1:14560`). Uses v0.8.0 defaults
    /// ([`MavlinkOptions::default`] — active GCS: heartbeat sender on,
    /// stream-request on first inbound heartbeat).
    pub fn connect(conn_str: &str) -> Result<Self> {
        Self::connect_with(conn_str, MavlinkOptions::default())
    }

    /// Like [`Self::connect`] with explicit options. `passive: true` restores
    /// the v0.4.0 listen-only behaviour (no heartbeat sender, no stream
    /// request) — handy when sharing a port with another GCS.
    ///
    /// We bind/open the socket *up front* (on the calling thread) so that a
    /// bad address or an already-bound port surfaces as an error from
    /// `from_uri` rather than silently dying inside the worker.
    pub fn connect_with(conn_str: &str, opts: MavlinkOptions) -> Result<Self> {
        // Capture the small bool up front: the rest of `opts` (which now holds
        // an owned `String` for the v0.10.0 drone-name override) is moved into
        // the recv worker below.
        let passive = opts.passive;
        let mut conn = mavlink::connect::<MavMessage>(conn_str)
            .with_context(|| format!("opening MAVLink connection at {conn_str}"))?;

        // Accept BOTH MAVLink v1 (0xFE) and v2 (0xFD) frames. The crate
        // defaults its read state to V2-only and silently discards frames of
        // the other version — but pymavlink and many ground stations still
        // emit v1 by default, and a real vehicle may send either. Without
        // this, `recv()` blocks forever on a pure-v1 stream. (This bit us in
        // the v0.4.0 smoke test: the synthetic pymavlink publisher sends v1.)
        conn.set_allow_recv_any_version(true);

        // Share the connection across the recv + heartbeat threads. Sending
        // takes `&self`, so an Arc is sufficient — no Mutex needed for the
        // serialiser side, only for the shared "first heartbeat seen" peer
        // state.
        let conn: Arc<dyn MavConnection<MavMessage> + Send + Sync> = Arc::new(conn);

        let (tx, rx) = crossbeam_channel::bounded::<Sample>(CHANNEL_CAPACITY);
        let conn_str_owned = conn_str.to_string();
        let stop_flag = Arc::new(AtomicBool::new(false));

        // Shared "peer learned" cell: set by the recv thread on the first
        // inbound HEARTBEAT so the recv thread itself can fire the one-shot
        // REQUEST_DATA_STREAM aimed at the peer. The heartbeat thread reads
        // it (informational) so the log shows the learned peer once.
        let peer = Arc::new(std::sync::Mutex::new(None::<(u8, u8)>));

        let conn_recv = Arc::clone(&conn);
        let stop_recv = Arc::clone(&stop_flag);
        let peer_recv = Arc::clone(&peer);
        let recv_worker = thread::Builder::new()
            .name("profiler-mavlink-rx".into())
            .spawn(move || recv_worker_main(conn_recv, tx, opts, stop_recv, peer_recv))
            .context("spawning MAVLink recv worker thread")?;

        let hb_worker = if passive {
            None
        } else {
            let conn_hb = Arc::clone(&conn);
            let stop_hb = Arc::clone(&stop_flag);
            let handle = thread::Builder::new()
                .name("profiler-mavlink-hb".into())
                .spawn(move || heartbeat_worker_main(conn_hb, stop_hb))
                .context("spawning MAVLink heartbeat worker thread")?;
            Some(handle)
        };

        log::info!(
            "MavlinkSource: spawned worker on {conn_str} (passive={passive})",
        );
        Ok(Self {
            rx,
            conn_str: conn_str_owned,
            stop_flag,
            _recv_worker: recv_worker,
            _hb_worker: hb_worker,
        })
    }
}

impl Drop for MavlinkSource {
    fn drop(&mut self) {
        // Signal both worker threads. The recv thread also exits when the
        // channel disconnects, but the heartbeat thread polls the flag.
        self.stop_flag.store(true, Ordering::Relaxed);
    }
}

impl Source for MavlinkSource {
    fn try_recv(&mut self) -> Option<Sample> {
        self.rx.try_recv().ok()
    }

    fn describe(&self) -> String {
        format!("mavlink:// {} (ardupilotmega)", self.conn_str)
    }
}

fn recv_worker_main(
    conn: Arc<dyn MavConnection<MavMessage> + Send + Sync>,
    tx: Sender<Sample>,
    opts: MavlinkOptions,
    stop: Arc<AtomicBool>,
    peer: Arc<std::sync::Mutex<Option<(u8, u8)>>>,
) {
    // `Instant` captured at thread start gives us a monotonic-ish stream
    // clock: `ts` is seconds since the first byte the worker was ready for.
    // (Workflow scripts forbid wall-clock nondeterminism; plain Rust timing
    // with `Instant` is fine and is what `MockSource`/the render loop use.)
    let started = Instant::now();
    let mut decoded = 0u64;
    let mut dropped_full = 0u64;
    let mut stream_requested = false;

    loop {
        if stop.load(Ordering::Relaxed) {
            log::info!(
                "MavlinkSource recv worker: stop flag set, exiting \
                 (decoded={decoded}, dropped={dropped_full})"
            );
            return;
        }
        let (header, msg) = match conn.recv() {
            Ok(pair) => pair,
            Err(e) => {
                // A parse error on a single frame is transient (bad CRC, an
                // unknown message id, a truncated UDP datagram) — log and keep
                // looping. Only an unrecoverable I/O error should stop us.
                if is_fatal(&e) {
                    log::error!(
                        "MavlinkSource recv worker exiting on fatal recv error: {e} \
                         (decoded={decoded}, dropped={dropped_full})"
                    );
                    return;
                }
                log::trace!("MavlinkSource: skipping undecodable frame: {e}");
                continue;
            }
        };

        // v0.8.0 — on the first inbound HEARTBEAT (in active-GCS mode), learn
        // the peer's system/component id and fire one REQUEST_DATA_STREAM so
        // stock-stream vehicles wake their rich-message output.
        if !opts.passive
            && !stream_requested
            && matches!(msg, MavMessage::HEARTBEAT(_))
        {
            let learned = (header.system_id, header.component_id);
            *peer.lock().expect("peer mutex poisoned") = Some(learned);
            if let Err(e) = send_request_data_stream(&*conn, learned.0, learned.1) {
                log::warn!(
                    "MavlinkSource: REQUEST_DATA_STREAM to sys={} comp={} failed: {e}",
                    learned.0, learned.1
                );
            } else {
                log::info!(
                    "MavlinkSource: requested ALL streams @ {} Hz from sys={} comp={}",
                    REQUEST_STREAM_RATE_HZ, learned.0, learned.1
                );
            }
            stream_requested = true;
        }

        // v0.10.0 — demux by `system_id` so a single MAVLink leg carrying
        // multiple vehicles fans out into distinct per-drone samples. The
        // operator-supplied `--drone NAME` override (carried via
        // `MavlinkOptions::drone_name_override`) wins when set.
        //
        // v0.10.1 — held as `Arc<str>` so every emitted `Sample` clones a
        // refcount instead of allocating a fresh `String`.
        let drone_name: Arc<str> = match opts.drone_name_override.as_deref() {
            Some(name) => Arc::from(name),
            None => Arc::from(format!("sysid_{}", header.system_id).as_str()),
        };

        let ts = started.elapsed().as_secs_f64();
        for s in decode_to_samples_with_drone(&msg, ts, Some(Arc::clone(&drone_name))) {
            match tx.try_send(s) {
                Ok(()) => decoded += 1,
                Err(TrySendError::Full(_)) => {
                    dropped_full += 1;
                    if dropped_full.is_power_of_two() {
                        log::warn!(
                            "MavlinkSource: channel full, dropped {dropped_full} samples \
                             (render loop is falling behind)"
                        );
                    }
                }
                Err(TrySendError::Disconnected(_)) => {
                    log::info!(
                        "MavlinkSource recv worker: receiver dropped, exiting \
                         (decoded={decoded}, dropped={dropped_full})"
                    );
                    return;
                }
            }
        }
    }
}

/// Heartbeat sender — emits one GCS HEARTBEAT per [`HEARTBEAT_PERIOD`] until
/// `stop` flips. Errors are logged at debug level (a transient `WouldBlock`
/// on `udpin` before a peer is known is normal).
fn heartbeat_worker_main(
    conn: Arc<dyn MavConnection<MavMessage> + Send + Sync>,
    stop: Arc<AtomicBool>,
) {
    let header = MavHeader {
        system_id: GCS_SYSTEM_ID,
        component_id: GCS_COMPONENT_ID,
        sequence: 0,
    };
    let msg = MavMessage::HEARTBEAT(gcs_heartbeat_payload());
    let mut sent = 0u64;
    let mut errors = 0u64;
    while !stop.load(Ordering::Relaxed) {
        match conn.send(&header, &msg) {
            Ok(_) => sent += 1,
            Err(e) => {
                errors += 1;
                if errors.is_power_of_two() {
                    log::debug!(
                        "MavlinkSource heartbeat: send failed ({errors} so far): {e}"
                    );
                }
            }
        }
        // Sleep in 100 ms slices so a shutdown signal can interrupt us within
        // ~100 ms instead of waiting a full second.
        let mut slept = Duration::ZERO;
        while slept < HEARTBEAT_PERIOD && !stop.load(Ordering::Relaxed) {
            thread::sleep(Duration::from_millis(100));
            slept += Duration::from_millis(100);
        }
    }
    log::info!(
        "MavlinkSource heartbeat worker exiting (sent={sent} errors={errors})"
    );
}

/// Build the constant payload used by [`heartbeat_worker_main`].
pub fn gcs_heartbeat_payload() -> HEARTBEAT_DATA {
    HEARTBEAT_DATA {
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        ..HEARTBEAT_DATA::default()
    }
}

/// Send one `REQUEST_DATA_STREAM(stream_id=ALL, rate=10 Hz, start_stop=1)`
/// aimed at the freshly-learned peer. Used at most once per session.
///
/// MAVLink upstream marks this message deprecated in favour of
/// `MAV_CMD_SET_MESSAGE_INTERVAL`, but real ArduPilot firmware still honours
/// `REQUEST_DATA_STREAM` (and many GCSes — Mission Planner, MAVProxy — still
/// send it on connect). We pick the de-facto-working message, not the spec
/// purest one.
#[allow(deprecated)]
fn send_request_data_stream(
    conn: &dyn MavConnection<MavMessage>,
    target_system: u8,
    target_component: u8,
) -> std::result::Result<usize, mavlink::error::MessageWriteError> {
    let header = MavHeader {
        system_id: GCS_SYSTEM_ID,
        component_id: GCS_COMPONENT_ID,
        sequence: 0,
    };
    let data = REQUEST_DATA_STREAM_DATA {
        target_system,
        target_component,
        req_stream_id: REQUEST_STREAM_ALL,
        req_message_rate: REQUEST_STREAM_RATE_HZ,
        start_stop: 1,
    };
    conn.send(&header, &MavMessage::REQUEST_DATA_STREAM(data))
}

/// Classify a [`mavlink::error::MessageReadError`]. A `Parse` error is per-frame
/// and recoverable (skip and keep reading); an `Io` error means the socket is
/// gone, so the worker should stop.
fn is_fatal(e: &mavlink::error::MessageReadError) -> bool {
    matches!(e, mavlink::error::MessageReadError::Io(_))
}

/// Decode one MAVLink message into flattened [`Sample`]s, mirroring the ZMQ
/// flattener's `"<base>[i]"` array convention so the 2D grid / 3D view consume
/// the identical keys.
///
/// Split out from [`worker_main`] so unit tests can construct messages
/// in-memory and assert the emitted keys/values without any socket.
///
/// Message → key mapping:
/// - `ATTITUDE` → `ap_attitude[0..2]` = roll, pitch, yaw (radians).
/// - `LOCAL_POSITION_NED` → `pos_ekf_ned[0..2]` = x, y, z and `ap_vel_ned[0..2]` = vx, vy, vz.
/// - `RAW_IMU` → `ap_raw_imu[0..5]` = xacc, yacc, zacc, xgyro, ygyro, zgyro. **Raw sensor units** (accel int16 counts, gyro mrad/s) — passed through as-is; the consumer rescales.
/// - `VFR_HUD` → `ap_vfr_alt` = alt (m, MSL).
/// - `POSITION_TARGET_LOCAL_NED` → `pos_target_ned[0..2]` = x, y, z.
/// - `GLOBAL_POSITION_INT` → `gps_alt` (m), `ap_vel_ned[0..2]` (m/s, cm/s → m/s),
///   plus `gps_lat` / `gps_lon` (degrees) for completeness. **v0.8.0** —
///   added so a stock-stream vehicle has at least altitude + velocity even
///   before the rich streams wake.
/// - `GPS_RAW_INT` → `gps_alt`, `gps_lat`, `gps_lon`, plus `gps_vn` (cm/s →
///   m/s) when the cog/vel scalars are valid. **v0.8.0** — used as the
///   altitude source on stock-stream vehicles.
/// - Everything else (SCALED_IMU2/3, HEARTBEAT, …) is ignored for now.
pub fn decode_to_samples(msg: &MavMessage, ts: f64) -> Vec<Sample> {
    decode_to_samples_with_drone(msg, ts, None)
}

/// v0.10.0 — like [`decode_to_samples`] but stamps each emitted [`Sample`]
/// with the supplied `drone_name`. The recv worker derives the name from each
/// MAVLink frame's `system_id` (or from `MavlinkOptions::drone_name_override`)
/// so a single leg carrying multiple vehicles fans out into distinct per-drone
/// streams downstream.
pub fn decode_to_samples_with_drone(
    msg: &MavMessage,
    ts: f64,
    drone_name: Option<Arc<str>>,
) -> Vec<Sample> {
    // v0.10.1 — one shared `Arc<str>` across every emitted sample; the
    // closure just bumps the refcount instead of allocating a `String`.
    let s = |key: &str, value: f64| Sample::new_scalar(
        ts,
        key,
        value,
        drone_name.as_ref().map(Arc::clone),
    );
    match msg {
        MavMessage::ATTITUDE(d) => vec![
            s("ap_attitude[0]", d.roll as f64),
            s("ap_attitude[1]", d.pitch as f64),
            s("ap_attitude[2]", d.yaw as f64),
        ],
        MavMessage::LOCAL_POSITION_NED(d) => vec![
            s("pos_ekf_ned[0]", d.x as f64),
            s("pos_ekf_ned[1]", d.y as f64),
            s("pos_ekf_ned[2]", d.z as f64),
            s("ap_vel_ned[0]", d.vx as f64),
            s("ap_vel_ned[1]", d.vy as f64),
            s("ap_vel_ned[2]", d.vz as f64),
        ],
        MavMessage::RAW_IMU(d) => vec![
            // Raw int16 sensor counts, passed through unscaled (see doc above).
            s("ap_raw_imu[0]", d.xacc as f64),
            s("ap_raw_imu[1]", d.yacc as f64),
            s("ap_raw_imu[2]", d.zacc as f64),
            s("ap_raw_imu[3]", d.xgyro as f64),
            s("ap_raw_imu[4]", d.ygyro as f64),
            s("ap_raw_imu[5]", d.zgyro as f64),
        ],
        MavMessage::VFR_HUD(d) => vec![s("ap_vfr_alt", d.alt as f64)],
        MavMessage::POSITION_TARGET_LOCAL_NED(d) => vec![
            s("pos_target_ned[0]", d.x as f64),
            s("pos_target_ned[1]", d.y as f64),
            s("pos_target_ned[2]", d.z as f64),
        ],
        // v0.8.0 — stock-stream messages.
        // GLOBAL_POSITION_INT: alt in mm, vx/vy/vz in cm/s, lat/lon * 1e7.
        MavMessage::GLOBAL_POSITION_INT(d) => vec![
            s("gps_alt", d.alt as f64 / 1_000.0),
            s("ap_vel_ned[0]", d.vx as f64 / 100.0),
            s("ap_vel_ned[1]", d.vy as f64 / 100.0),
            s("ap_vel_ned[2]", d.vz as f64 / 100.0),
            s("gps_lat", d.lat as f64 / 1e7),
            s("gps_lon", d.lon as f64 / 1e7),
        ],
        // GPS_RAW_INT: alt in mm, vel in cm/s, lat/lon * 1e7.
        MavMessage::GPS_RAW_INT(d) => vec![
            s("gps_alt", d.alt as f64 / 1_000.0),
            s("gps_lat", d.lat as f64 / 1e7),
            s("gps_lon", d.lon as f64 / 1e7),
            s("gps_vn", d.vel as f64 / 100.0),
        ],
        // SCALED_IMU2/3, HEARTBEAT, SYS_STATUS, … — not plotted.
        _ => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::dialects::ardupilotmega::{
        ATTITUDE_DATA, LOCAL_POSITION_NED_DATA, POSITION_TARGET_LOCAL_NED_DATA, RAW_IMU_DATA,
        VFR_HUD_DATA,
    };

    /// Helper: collect (key, value) pairs, asserting the shared `ts`.
    fn pairs(samples: Vec<Sample>, ts: f64) -> Vec<(String, f64)> {
        samples
            .into_iter()
            .map(|s| {
                assert_eq!(s.ts, ts);
                let value = s.scalar();
                (s.key, value)
            })
            .collect()
    }

    #[test]
    fn attitude_maps_to_ap_attitude() {
        let d = ATTITUDE_DATA {
            roll: 0.1,
            pitch: -0.2,
            yaw: 1.5,
            ..Default::default()
        };
        let msg = MavMessage::ATTITUDE(d);
        assert_eq!(
            pairs(decode_to_samples(&msg, 3.0), 3.0),
            vec![
                ("ap_attitude[0]".into(), 0.1_f32 as f64),
                ("ap_attitude[1]".into(), -0.2_f32 as f64),
                ("ap_attitude[2]".into(), 1.5_f32 as f64),
            ]
        );
    }

    #[test]
    fn local_position_ned_maps_to_pos_and_vel() {
        let d = LOCAL_POSITION_NED_DATA {
            x: 1.0,
            y: 2.0,
            z: -3.0,
            vx: 0.5,
            vy: -0.5,
            vz: 0.25,
            ..Default::default()
        };
        let msg = MavMessage::LOCAL_POSITION_NED(d);
        assert_eq!(
            pairs(decode_to_samples(&msg, 0.0), 0.0),
            vec![
                ("pos_ekf_ned[0]".into(), 1.0),
                ("pos_ekf_ned[1]".into(), 2.0),
                ("pos_ekf_ned[2]".into(), -3.0),
                ("ap_vel_ned[0]".into(), 0.5_f32 as f64),
                ("ap_vel_ned[1]".into(), -0.5_f32 as f64),
                ("ap_vel_ned[2]".into(), 0.25),
            ]
        );
    }

    #[test]
    fn raw_imu_maps_to_six_indexed_keys_raw_units() {
        let d = RAW_IMU_DATA {
            xacc: 10,
            yacc: 20,
            zacc: -1000,
            xgyro: 1,
            ygyro: -2,
            zgyro: 3,
            // mag fields should be ignored (we only emit indices 0..5).
            xmag: 999,
            ..Default::default()
        };
        let msg = MavMessage::RAW_IMU(d);
        assert_eq!(
            pairs(decode_to_samples(&msg, 1.0), 1.0),
            vec![
                ("ap_raw_imu[0]".into(), 10.0),
                ("ap_raw_imu[1]".into(), 20.0),
                ("ap_raw_imu[2]".into(), -1000.0),
                ("ap_raw_imu[3]".into(), 1.0),
                ("ap_raw_imu[4]".into(), -2.0),
                ("ap_raw_imu[5]".into(), 3.0),
            ]
        );
    }

    #[test]
    fn vfr_hud_maps_to_alt_scalar() {
        let d = VFR_HUD_DATA {
            alt: 42.5,
            ..Default::default()
        };
        let msg = MavMessage::VFR_HUD(d);
        assert_eq!(
            pairs(decode_to_samples(&msg, 2.0), 2.0),
            vec![("ap_vfr_alt".into(), 42.5)]
        );
    }

    #[test]
    fn position_target_local_ned_maps_to_three_keys() {
        let d = POSITION_TARGET_LOCAL_NED_DATA {
            x: 5.0,
            y: 6.0,
            z: -7.0,
            ..Default::default()
        };
        let msg = MavMessage::POSITION_TARGET_LOCAL_NED(d);
        assert_eq!(
            pairs(decode_to_samples(&msg, 4.0), 4.0),
            vec![
                ("pos_target_ned[0]".into(), 5.0),
                ("pos_target_ned[1]".into(), 6.0),
                ("pos_target_ned[2]".into(), -7.0),
            ]
        );
    }

    #[test]
    fn unmapped_message_yields_no_samples() {
        // HEARTBEAT is decoded but not plotted → no samples.
        let msg = MavMessage::HEARTBEAT(Default::default());
        assert!(decode_to_samples(&msg, 0.0).is_empty());
    }
}
