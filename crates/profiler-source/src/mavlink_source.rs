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
//! `udpin` is passive: we never have to send a heartbeat to "wake" the link.
//! We simply loop on `recv()` and decode whatever arrives. (For `udpout` the
//! crate handles the initial send-to-peer itself on first I/O.)

use std::thread;
use std::time::Instant;

use anyhow::{Context, Result};
use crossbeam_channel::{Receiver, Sender, TrySendError};
use mavlink::dialects::ardupilotmega::MavMessage;
use mavlink::MavConnection;

use crate::{Sample, Source};

/// Channel capacity. A real vehicle streams ~10–50 Hz across a handful of
/// message types (≈ tens of keys per second), so this is ~hours of headroom —
/// but we keep it large for parity with [`ZmqSource`] and burst tolerance.
const CHANNEL_CAPACITY: usize = 100_000;

/// A direct MAVLink-over-UDP source. Spawns a worker thread that owns the
/// connection and decodes messages into [`Sample`]s.
pub struct MavlinkSource {
    rx: Receiver<Sample>,
    /// The `mavlink`-crate connection string actually used (e.g.
    /// `udpin:127.0.0.1:14560`), for the status bar / window title.
    conn_str: String,
    /// Kept alive so the worker thread shuts down when `MavlinkSource` is
    /// dropped (the worker exits once the channel receiver disconnects).
    _worker: thread::JoinHandle<()>,
}

impl MavlinkSource {
    /// Spawn the worker and open `conn_str` (a `mavlink`-crate address such
    /// as `udpin:127.0.0.1:14560`).
    ///
    /// We bind/open the socket *up front* (on the calling thread) so that a
    /// bad address or an already-bound port surfaces as an error from
    /// `from_uri` rather than silently dying inside the worker. The blocking
    /// `recv()` loop then runs on the worker thread.
    pub fn connect(conn_str: &str) -> Result<Self> {
        let mut conn = mavlink::connect::<MavMessage>(conn_str)
            .with_context(|| format!("opening MAVLink connection at {conn_str}"))?;

        // Accept BOTH MAVLink v1 (0xFE) and v2 (0xFD) frames. The crate
        // defaults its read state to V2-only and silently discards frames of
        // the other version — but pymavlink and many ground stations still
        // emit v1 by default, and a real vehicle may send either. Without
        // this, `recv()` blocks forever on a pure-v1 stream. (This bit us in
        // the v0.4.0 smoke test: the synthetic pymavlink publisher sends v1.)
        conn.set_allow_recv_any_version(true);

        let (tx, rx) = crossbeam_channel::bounded::<Sample>(CHANNEL_CAPACITY);
        let conn_str_owned = conn_str.to_string();

        let worker = thread::Builder::new()
            .name("profiler-mavlink".into())
            .spawn(move || worker_main(conn, tx))
            .context("spawning MAVLink worker thread")?;

        log::info!("MavlinkSource: spawned worker, listening/connecting on {conn_str}");
        Ok(Self {
            rx,
            conn_str: conn_str_owned,
            _worker: worker,
        })
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

fn worker_main(conn: mavlink::Connection<MavMessage>, tx: Sender<Sample>) {
    // `Instant` captured at thread start gives us a monotonic-ish stream
    // clock: `ts` is seconds since the first byte the worker was ready for.
    // (Workflow scripts forbid wall-clock nondeterminism; plain Rust timing
    // with `Instant` is fine and is what `MockSource`/the render loop use.)
    let started = Instant::now();
    let mut decoded = 0u64;
    let mut dropped_full = 0u64;

    loop {
        let (_header, msg) = match conn.recv() {
            Ok(pair) => pair,
            Err(e) => {
                // A parse error on a single frame is transient (bad CRC, an
                // unknown message id, a truncated UDP datagram) — log and keep
                // looping. Only an unrecoverable I/O error should stop us.
                if is_fatal(&e) {
                    log::error!(
                        "MavlinkSource worker exiting on fatal recv error: {e} \
                         (decoded={decoded}, dropped={dropped_full})"
                    );
                    return;
                }
                log::trace!("MavlinkSource: skipping undecodable frame: {e}");
                continue;
            }
        };

        let ts = started.elapsed().as_secs_f64();
        for s in decode_to_samples(&msg, ts) {
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
                        "MavlinkSource: receiver dropped, exiting worker \
                         (decoded={decoded}, dropped={dropped_full})"
                    );
                    return;
                }
            }
        }
    }
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
/// - Everything else (SCALED_IMU2/3, HEARTBEAT, …) is ignored for now.
pub fn decode_to_samples(msg: &MavMessage, ts: f64) -> Vec<Sample> {
    let s = |key: &str, value: f64| Sample {
        ts,
        key: key.to_string(),
        value,
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
        // SCALED_IMU2/3, HEARTBEAT, SYS_STATUS, … — not plotted in v0.4.0.
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
                (s.key, s.value)
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
