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

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
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

use crate::{Sample, TextLogEntry, Value, Source};

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

/// v0.16.3 — `STATUSTEXT` rolling-buffer depth. Mission Planner shows the last
/// ~5 lines in its MAVLink Inspector → MESSAGES tab; 8 gives us a little
/// headroom and matches the streamer-side `_STATUSTEXT_MAX` in DT-Python's
/// `hil_bridge.py`.
const STATUSTEXT_MAX: usize = 8;

/// v0.16.3 — `HEARTBEAT.base_mode` bit that flags vehicle as armed. The
/// mavlink crate exposes it as [`MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED`]
/// (`0x80`); we keep the raw constant here for the bit-test fast path.
const MAV_MODE_FLAG_SAFETY_ARMED: u8 = 0x80;

/// v0.16.3 — ArduCopter `HEARTBEAT.custom_mode` → flight-mode-name lookup.
///
/// Copied verbatim from DT-Python's `hil_bridge.py:_COPTER_MODE_NAMES`. The
/// streamer-side parity matters: when a profiler cell binds `flight_mode` to
/// a colour map (e.g. `GUIDED → blue`), the same human-readable strings must
/// arrive whether the data came through ZMQ or direct MAVLink.
///
/// Plane / Rover / Sub get a `MODE_<n>` fallback — out of scope for v0.16.3.
fn copter_mode_name(custom_mode: u32) -> String {
    match custom_mode {
        0 => "STABILIZE".into(),
        1 => "ACRO".into(),
        2 => "ALT_HOLD".into(),
        3 => "AUTO".into(),
        4 => "GUIDED".into(),
        5 => "LOITER".into(),
        6 => "RTL".into(),
        7 => "CIRCLE".into(),
        9 => "LAND".into(),
        11 => "DRIFT".into(),
        13 => "SPORT".into(),
        14 => "FLIP".into(),
        15 => "AUTOTUNE".into(),
        16 => "POSHOLD".into(),
        17 => "BRAKE".into(),
        18 => "THROW".into(),
        19 => "AVOID_ADSB".into(),
        20 => "GUIDED_NOGPS".into(),
        21 => "SMART_RTL".into(),
        22 => "FLOWHOLD".into(),
        23 => "FOLLOW".into(),
        24 => "ZIGZAG".into(),
        25 => "SYSTEMID".into(),
        26 => "AUTOROTATE".into(),
        27 => "AUTO_RTL".into(),
        n => format!("MODE_{n}"),
    }
}

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

        // v0.16.3 — STATUSTEXT rolling buffer. Every inbound STATUSTEXT frame
        // appends to this deque (capacity STATUSTEXT_MAX, oldest dropped on
        // overflow); each emitted `statustexts` sample carries the full
        // snapshot so downstream `TextLog` consumers receive the
        // most-recent N lines on every push, just like the DT-Python bridge.
        let statustext_buf: Arc<Mutex<VecDeque<TextLogEntry>>> =
            Arc::new(Mutex::new(VecDeque::with_capacity(STATUSTEXT_MAX)));

        let conn_recv = Arc::clone(&conn);
        let stop_recv = Arc::clone(&stop_flag);
        let peer_recv = Arc::clone(&peer);
        let statustext_recv = Arc::clone(&statustext_buf);
        let recv_worker = thread::Builder::new()
            .name("profiler-mavlink-rx".into())
            .spawn(move || recv_worker_main(conn_recv, tx, opts, stop_recv, peer_recv, statustext_recv))
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
    statustext_buf: Arc<Mutex<VecDeque<TextLogEntry>>>,
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
        for s in decode_to_samples_with_state(
            &msg,
            ts,
            Some(Arc::clone(&drone_name)),
            Some(&statustext_buf),
        ) {
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
/// v0.16.3 — expanded from 4 → 16+ message types to reach parity with
/// DT-Python's `hil_bridge.py`. The full key set is documented in
/// [`decode_to_samples_with_state`]; the no-state variant here cannot
/// surface `statustexts` (it has no rolling buffer to share across frames)
/// but every other key is produced identically.
pub fn decode_to_samples(msg: &MavMessage, ts: f64) -> Vec<Sample> {
    decode_to_samples_with_state(msg, ts, None, None)
}

/// v0.10.0 — like [`decode_to_samples`] but stamps each emitted [`Sample`]
/// with the supplied `drone_name`. Retained for backwards compatibility with
/// pre-v0.16.3 call sites; new code should prefer [`decode_to_samples_with_state`]
/// which also accepts the shared `STATUSTEXT` rolling buffer.
pub fn decode_to_samples_with_drone(
    msg: &MavMessage,
    ts: f64,
    drone_name: Option<Arc<str>>,
) -> Vec<Sample> {
    decode_to_samples_with_state(msg, ts, drone_name, None)
}

/// v0.16.3 — full-vocabulary decoder mirroring DT-Python's `hil_bridge.py`.
///
/// Key conventions (matching `KNOWN_HVN_SITL_KEYS` in `editor.rs` and the
/// streamer envelope shape):
///
/// | MAVLink | Key(s) |
/// |---|---|
/// | `ATTITUDE` | `ap_attitude[0..2]` |
/// | `RAW_IMU` | `ap_raw_imu[0..5]` (raw counts) |
/// | `LOCAL_POSITION_NED` | `pos_ekf_ned[0..2]`, `ap_vel_ned[0..2]` |
/// | `POSITION_TARGET_LOCAL_NED` | `pos_target_ned[0..2]` |
/// | `VFR_HUD` | `ap_vfr_alt` |
/// | `GLOBAL_POSITION_INT` | `gps_alt`, `ap_vel_ned[0..2]`, `gps_lat`, `gps_lon` |
/// | `GPS_RAW_INT` | `gps_alt`, `gps_lat`, `gps_lon`, `gps_vn`, `fix_type` |
/// | `EKF_STATUS_REPORT` | `ekf_flags`, `ekf_velv`, `ekf_pos_horiz`, `ekf_pos_vert`, `ekf_compv`, `ekf_terralt` |
/// | `AHRS2` | `ahrs2_roll`, `ahrs2_pitch`, `ahrs2_yaw`, `ahrs2_alt`, `ahrs2_lat`, `ahrs2_lng` |
/// | `VIBRATION` | `vibex`, `vibey`, `vibez`, `vibeclip0..2` |
/// | `SCALED_IMU2/3` | `scaled_imu2[0..9]` / `scaled_imu3[0..9]` (Vec[10]) + Vector sample |
/// | `SCALED_PRESSURE` | `press_scaled[0..2]` (abs / diff / temp) + Vector sample |
/// | `SCALED_PRESSURE2` | `press_scaled2[0..2]` + Vector sample |
/// | `BATTERY_STATUS` | `battery_voltage`, `battery_current`, `battery_remaining` |
/// | `ESC_STATUS` | `esc_rpm[0..3]`, `esc_voltage[0..3]`, `esc_current[0..3]` (first 4 ESCs) + Vector samples |
/// | `RC_CHANNELS` | `rc_channels[0..15]`, `rc_rssi`, plus IntVector sample |
/// | `SERVO_OUTPUT_RAW` | `servo_outputs[0..15]` (padded to 16), plus IntVector sample |
/// | `NAV_CONTROLLER_OUTPUT` | `nav_roll`, `nav_pitch`, `nav_bearing`, `target_bearing`, `wp_dist`, `alt_error`, `aspd_error`, `xtrack_error` |
/// | `SYS_STATUS` | `sys_load` (load‰), `sys_drop_rate_comm`, `sys_errors[0..3]` + IntVector sample |
/// | `STATUSTEXT` | `statustexts` `TextLog` (rolling buffer, capacity 8) |
/// | `HEARTBEAT` | `armed` `Bool`, `flight_mode` `String` (copter mode table) |
///
/// Units mirror the streamer-side wire format: floats kept as-is, integer
/// scaling normalised once (mm→m, cm→A, deg×1e-7→deg). For `BATTERY_STATUS`
/// the voltage is summed across all valid cell entries in `voltages[]`
/// (mV per cell, `0xFFFF` = unused), then divided by 1000 — same convention
/// `hil_bridge.py` uses.
pub fn decode_to_samples_with_state(
    msg: &MavMessage,
    ts: f64,
    drone_name: Option<Arc<str>>,
    statustext_buf: Option<&Arc<Mutex<VecDeque<TextLogEntry>>>>,
) -> Vec<Sample> {
    // v0.10.1 — one shared `Arc<str>` across every emitted sample; the
    // closure just bumps the refcount instead of allocating a `String`.
    let s = |key: &str, value: f64| Sample::new_scalar(
        ts,
        key,
        value,
        drone_name.as_ref().map(Arc::clone),
    );
    let make = |key: &str, value: Value| Sample {
        ts,
        key: key.to_string(),
        value,
        drone_name: drone_name.as_ref().map(Arc::clone),
    };
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
        // GPS_RAW_INT: alt in mm, vel in cm/s, lat/lon * 1e7. v0.16.3 — also
        // surface `fix_type` so the status primitive can colour by lock
        // quality (3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed).
        MavMessage::GPS_RAW_INT(d) => vec![
            s("gps_alt", d.alt as f64 / 1_000.0),
            s("gps_lat", d.lat as f64 / 1e7),
            s("gps_lon", d.lon as f64 / 1e7),
            s("gps_vn", d.vel as f64 / 100.0),
            // `fix_type` is an enum on the wire; cast through u8 via the
            // bitflags-style `From` conversion the mavlink crate exposes for
            // every C-enum (`as u8` works because the discriminants are
            // defined in spec order — but the safe route is the explicit
            // primitive cast on the FromPrimitive-derived enum).
            s("fix_type", d.fix_type as u8 as f64),
        ],
        // v0.16.3 — EKF health (Mission Planner "EKF bars" in the HUD).
        MavMessage::EKF_STATUS_REPORT(d) => vec![
            // `flags` is a bitflags `EkfStatusFlags : u16`; bits() yields the
            // raw u16 so a profiler cell with `key: "ekf_flags"` and a chip
            // renderer can decompose them downstream.
            s("ekf_flags", d.flags.bits() as f64),
            s("ekf_velv", d.velocity_variance as f64),
            s("ekf_pos_horiz", d.pos_horiz_variance as f64),
            s("ekf_pos_vert", d.pos_vert_variance as f64),
            s("ekf_compv", d.compass_variance as f64),
            s("ekf_terralt", d.terrain_alt_variance as f64),
        ],
        // v0.16.3 — Secondary AHRS estimate (DCM-based). Useful for sanity
        // checking the primary EKF roll/pitch/yaw.
        MavMessage::AHRS2(d) => vec![
            s("ahrs2_roll", d.roll as f64),
            s("ahrs2_pitch", d.pitch as f64),
            s("ahrs2_yaw", d.yaw as f64),
            s("ahrs2_alt", d.altitude as f64),
            s("ahrs2_lat", d.lat as f64 / 1e7),
            s("ahrs2_lng", d.lng as f64 / 1e7),
        ],
        // v0.16.3 — Vibration. `vibration_{x,y,z}` are m/s² standard deviation;
        // `clipping_{0,1,2}` are uint32 counts of accel saturations on the
        // first 3 IMUs (cumulative since boot).
        MavMessage::VIBRATION(d) => vec![
            s("vibex", d.vibration_x as f64),
            s("vibey", d.vibration_y as f64),
            s("vibez", d.vibration_z as f64),
            s("vibeclip0", d.clipping_0 as f64),
            s("vibeclip1", d.clipping_1 as f64),
            s("vibeclip2", d.clipping_2 as f64),
        ],
        // v0.16.3 — secondary / tertiary IMUs. Mirror DT-Python's SI
        // conversion on capture (accel mG→m/s², gyro mrad/s→rad/s, temp
        // cdegC→degC) so cells reading `scaled_imu2[6]` (mx) see mgauss as
        // emitted by AP and cells reading `scaled_imu2[9]` (temp) see degC.
        // The mavlink-0.18 dialect omits the `temperature` extension on
        // SCALED_IMU2/3 — we pad the 10th component with 0 to keep the Vec[10]
        // shape DT-Python's bridge ships (consumers index by position).
        MavMessage::SCALED_IMU2(d) => scaled_imu_samples("scaled_imu2", d.xacc, d.yacc, d.zacc, d.xgyro, d.ygyro, d.zgyro, d.xmag, d.ymag, d.zmag, 0, ts, &drone_name),
        MavMessage::SCALED_IMU3(d) => scaled_imu_samples("scaled_imu3", d.xacc, d.yacc, d.zacc, d.xgyro, d.ygyro, d.zgyro, d.xmag, d.ymag, d.zmag, 0, ts, &drone_name),
        // v0.16.3 — barometric pressure sensors. press_abs hPa, press_diff
        // hPa (pitot), temperature cdegC (passed through as the raw 0.01°C
        // count — consumers can divide by 100).
        MavMessage::SCALED_PRESSURE(d) => press_scaled_samples("press_scaled", d.press_abs, d.press_diff, d.temperature, ts, &drone_name),
        MavMessage::SCALED_PRESSURE2(d) => press_scaled_samples("press_scaled2", d.press_abs, d.press_diff, d.temperature, ts, &drone_name),
        // v0.16.3 — battery: cell-voltage sum (mV per cell, 0xFFFF=unused),
        // current_battery in cA → A, battery_remaining in %.
        MavMessage::BATTERY_STATUS(d) => {
            let mut v_mv: u32 = 0;
            for &cell_mv in d.voltages.iter() {
                if cell_mv != 0 && cell_mv != u16::MAX {
                    v_mv = v_mv.saturating_add(cell_mv as u32);
                }
            }
            let voltage_v = v_mv as f64 / 1000.0;
            let current_a = d.current_battery as f64 / 100.0;
            vec![
                s("battery_voltage", voltage_v),
                s("battery_current", current_a),
                s("battery_remaining", d.battery_remaining as f64),
            ]
        }
        // v0.16.3 — ESC status, first 4 motors only. Emit per-index scalars
        // + a Vec[4] sample so cells can bind either style.
        MavMessage::ESC_STATUS(d) => {
            let rpm: Vec<f64> = d.rpm.iter().map(|&v| v as f64).collect();
            let voltage: Vec<f64> = d.voltage.iter().map(|&v| v as f64).collect();
            let current: Vec<f64> = d.current.iter().map(|&v| v as f64).collect();
            let mut out = Vec::with_capacity(15);
            for (i, &v) in rpm.iter().enumerate() {
                out.push(s(&format!("esc_rpm[{i}]"), v));
            }
            for (i, &v) in voltage.iter().enumerate() {
                out.push(s(&format!("esc_voltage[{i}]"), v));
            }
            for (i, &v) in current.iter().enumerate() {
                out.push(s(&format!("esc_current[{i}]"), v));
            }
            out.push(make("esc_rpm", Value::Vector(rpm)));
            out.push(make("esc_voltage", Value::Vector(voltage)));
            out.push(make("esc_current", Value::Vector(current)));
            out
        }
        // v0.16.3 — RC channels. 16 entries (the mavlink-0.18 dialect exposes
        // chan1..chan18; we cap at 16 for parity with `KNOWN_HVN_SITL_KEYS`).
        MavMessage::RC_CHANNELS(d) => {
            let channels: [u16; 16] = [
                d.chan1_raw, d.chan2_raw, d.chan3_raw, d.chan4_raw,
                d.chan5_raw, d.chan6_raw, d.chan7_raw, d.chan8_raw,
                d.chan9_raw, d.chan10_raw, d.chan11_raw, d.chan12_raw,
                d.chan13_raw, d.chan14_raw, d.chan15_raw, d.chan16_raw,
            ];
            let mut out = Vec::with_capacity(18);
            for (i, &v) in channels.iter().enumerate() {
                out.push(s(&format!("rc_channels[{i}]"), v as f64));
            }
            out.push(make(
                "rc_channels",
                Value::IntVector(channels.iter().map(|&v| v as i64).collect()),
            ));
            out.push(s("rc_rssi", d.rssi as f64));
            out
        }
        // v0.16.3 — servo outputs. mavlink-0.18 exposes only servo1..8; pad
        // to 16 with 0 so the wire shape matches DT-Python's emitter (which
        // sees 16 from pymavlink's extended dialect).
        MavMessage::SERVO_OUTPUT_RAW(d) => {
            let servos: [u16; 16] = [
                d.servo1_raw, d.servo2_raw, d.servo3_raw, d.servo4_raw,
                d.servo5_raw, d.servo6_raw, d.servo7_raw, d.servo8_raw,
                0, 0, 0, 0, 0, 0, 0, 0,
            ];
            let mut out = Vec::with_capacity(17);
            for (i, &v) in servos.iter().enumerate() {
                out.push(s(&format!("servo_outputs[{i}]"), v as f64));
            }
            out.push(make(
                "servo_outputs",
                Value::IntVector(servos.iter().map(|&v| v as i64).collect()),
            ));
            out
        }
        // v0.16.3 — Position controller targets. nav_roll/pitch deg,
        // *_bearing deg, wp_dist m, alt_error m, aspd_error m/s, xtrack_error m.
        MavMessage::NAV_CONTROLLER_OUTPUT(d) => vec![
            s("nav_roll", d.nav_roll as f64),
            s("nav_pitch", d.nav_pitch as f64),
            s("nav_bearing", d.nav_bearing as f64),
            s("target_bearing", d.target_bearing as f64),
            s("wp_dist", d.wp_dist as f64),
            s("alt_error", d.alt_error as f64),
            s("aspd_error", d.aspd_error as f64),
            s("xtrack_error", d.xtrack_error as f64),
        ],
        // v0.16.3 — System status. load is permille (0–1000); drop_rate_comm
        // is permille; errors_count* are uint16 per-link error counters.
        MavMessage::SYS_STATUS(d) => {
            let errs: [u16; 4] = [d.errors_count1, d.errors_count2, d.errors_count3, d.errors_count4];
            let mut out = Vec::with_capacity(7);
            out.push(s("sys_load", d.load as f64));
            out.push(s("sys_drop_rate_comm", d.drop_rate_comm as f64));
            for (i, &v) in errs.iter().enumerate() {
                out.push(s(&format!("sys_errors[{i}]"), v as f64));
            }
            out.push(make(
                "sys_errors",
                Value::IntVector(errs.iter().map(|&v| v as i64).collect()),
            ));
            out
        }
        // v0.16.3 — STATUSTEXT rolling buffer. We need a shared
        // `Arc<Mutex<VecDeque<TextLogEntry>>>` carried by the source so the
        // most-recent N entries accumulate across frames. Without it the
        // decoder still extracts the single entry but emits no sample —
        // the no-state caller has nowhere to keep the history.
        MavMessage::STATUSTEXT(d) => {
            let text_str = d.text.to_str().unwrap_or("").trim_end_matches('\0').to_string();
            // `severity` is an enum (`MavSeverity`) discriminant 0..7; raw cast.
            let severity = d.severity as u8;
            let entry = TextLogEntry {
                severity,
                text: Arc::from(text_str.as_str()),
                ts,
            };
            match statustext_buf {
                Some(buf) => {
                    let snapshot = {
                        let mut g = match buf.lock() {
                            Ok(g) => g,
                            Err(p) => p.into_inner(),
                        };
                        if g.len() >= STATUSTEXT_MAX {
                            g.pop_front();
                        }
                        g.push_back(entry);
                        // Snapshot the full deque oldest→newest. Cheap (≤ 8 entries).
                        g.iter().cloned().collect::<Vec<TextLogEntry>>()
                    };
                    vec![make("statustexts", Value::TextLog(snapshot))]
                }
                None => Vec::new(),
            }
        }
        // v0.16.3 — HEARTBEAT: surface armed bit + decoded copter mode.
        // `base_mode` is a `MavModeFlag` bitflags; the SAFETY_ARMED bit is
        // 0x80. `custom_mode` is a raw u32 the copter mode table decodes.
        MavMessage::HEARTBEAT(d) => {
            let armed = (d.base_mode.bits() & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            let mode_str = copter_mode_name(d.custom_mode);
            vec![
                make("armed", Value::Bool(armed)),
                make("flight_mode", Value::String(Arc::from(mode_str.as_str()))),
            ]
        }
        // Other messages — not (yet) plotted. ACTUATOR_OUTPUT_STATUS,
        // ESC_INFO (all 12 ESCs), MISSION_CURRENT, … live here.
        _ => Vec::new(),
    }
}

/// v0.16.3 — helper: build a SCALED_IMU2/3 fan-out + Vec[10] sample.
///
/// The 10-component vector layout — `(ax, ay, az, gx, gy, gz, mx, my, mz, temp)`
/// — matches DT-Python's bridge so cells binding `scaled_imu2[6]` (X mag) or
/// the whole vector get identical semantics whether the data came through ZMQ
/// or direct MAVLink. Raw on-wire units are preserved (accel mG, gyro mrad/s,
/// mag mgauss, temp cdegC) so the same downstream rescaling rules apply.
#[allow(clippy::too_many_arguments)]
fn scaled_imu_samples(
    base: &str,
    xacc: i16,
    yacc: i16,
    zacc: i16,
    xgyro: i16,
    ygyro: i16,
    zgyro: i16,
    xmag: i16,
    ymag: i16,
    zmag: i16,
    temperature: i16,
    ts: f64,
    drone_name: &Option<Arc<str>>,
) -> Vec<Sample> {
    let comps: [f64; 10] = [
        xacc as f64, yacc as f64, zacc as f64,
        xgyro as f64, ygyro as f64, zgyro as f64,
        xmag as f64, ymag as f64, zmag as f64,
        temperature as f64,
    ];
    let mut out = Vec::with_capacity(11);
    for (i, &v) in comps.iter().enumerate() {
        out.push(Sample::new_scalar(
            ts,
            format!("{base}[{i}]"),
            v,
            drone_name.as_ref().map(Arc::clone),
        ));
    }
    out.push(Sample {
        ts,
        key: base.to_string(),
        value: Value::Vector(comps.to_vec()),
        drone_name: drone_name.as_ref().map(Arc::clone),
    });
    out
}

/// v0.16.3 — helper: build a SCALED_PRESSURE / SCALED_PRESSURE2 Vec[3] sample.
///
/// Layout: `(press_abs hPa, press_diff hPa, temperature cdegC)`. The
/// `temperature` field stays in centidegrees on the wire — the consumer
/// divides by 100 if it wants degC. This matches the streamer-side wire
/// format (cells bind the same index → same meaning across sources).
fn press_scaled_samples(
    base: &str,
    press_abs: f32,
    press_diff: f32,
    temperature: i16,
    ts: f64,
    drone_name: &Option<Arc<str>>,
) -> Vec<Sample> {
    let comps: [f64; 3] = [press_abs as f64, press_diff as f64, temperature as f64];
    let mut out = Vec::with_capacity(4);
    for (i, &v) in comps.iter().enumerate() {
        out.push(Sample::new_scalar(
            ts,
            format!("{base}[{i}]"),
            v,
            drone_name.as_ref().map(Arc::clone),
        ));
    }
    out.push(Sample {
        ts,
        key: base.to_string(),
        value: Value::Vector(comps.to_vec()),
        drone_name: drone_name.as_ref().map(Arc::clone),
    });
    out
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
    fn truly_unmapped_message_yields_no_samples() {
        // PING is decoded but not plotted (no panel cell binds it) → no
        // samples. v0.16.3 expanded HEARTBEAT / STATUSTEXT / EKF_STATUS_REPORT
        // / VIBRATION / … to be emitted, so this assertion now targets a
        // genuinely-unmapped message instead. PING is deprecated upstream
        // but still a valid construction target for this test.
        #[allow(deprecated)]
        {
            use mavlink::dialects::ardupilotmega::PING_DATA;
            let msg = MavMessage::PING(PING_DATA::default());
            assert!(decode_to_samples(&msg, 0.0).is_empty());
        }
    }
}
