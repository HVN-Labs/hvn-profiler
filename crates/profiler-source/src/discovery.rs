//! v0.16.0 — auto-discovery of telemetry sources on localhost.
//!
//! Powers the `+ Add Source...` dialog's "Detected on localhost:" list. The
//! profiler probes:
//!
//! - **ZMQ** PUB endpoints on the HVN-SITL streamer port range
//!   (`9005..=9020` — drone 1 through drone 16).
//! - **MAVLink** UDP endpoints on the well-known GCS ports
//!   (`14550`, `14551`, `14555`, `14560`, `14570` — Mission Planner, QGC,
//!   SkyBrush, real-drone-direct, HIL bridge).
//!
//! All probes run in parallel via a [`tokio::task::JoinSet`] with a single
//! shared `probe_duration_ms` budget, so the whole scan returns in roughly
//! one probe window (typically 500 ms).
//!
//! ## Probe strategy
//!
//! - **ZMQ**: open a `tokio::net::TcpStream` to the port with a short
//!   `connect_timeout`. If the port is closed (connection refused), drop the
//!   port from the result. If TCP succeeds, drop the TCP socket immediately
//!   and open a real `zeromq::SubSocket`; wait for one frame up to the
//!   remaining probe budget. If a frame arrives, try to msgpack-decode it
//!   for the `drone_name`. If no frame arrives, report `Silent` (port bound,
//!   no traffic yet — the operator may want to connect and wait).
//!
//! - **MAVLink**: try to bind a UDP socket on the port. If the bind succeeds,
//!   wait for one datagram up to the probe budget. Got a frame → `Live`;
//!   bindable but no traffic → `Silent`. If the bind fails (`EADDRINUSE`)
//!   the port is unusable by the profiler anyway, so we drop it from the
//!   result.
//!
//! - **Already-connected dedup**: any URI in `already_connected` is replaced
//!   by `DiscoveryStatus::InUse` so the UI can grey it out without losing
//!   the entry.
//!
//! All discovered URIs use `127.0.0.1` (not `0.0.0.0` / `localhost`) so the
//! string the operator sees in the dialog round-trips exactly into the
//! existing `+ Add Source...` Connect flow.

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::Duration;

use tokio::net::{TcpStream, UdpSocket};
use tokio::task::JoinSet;
use tokio::time::timeout;

use crate::{flatten_msgpack, Sample};

/// HVN-SITL streamer port range: drone 1 = 9005 through drone 16 = 9020.
/// Inclusive on both ends.
pub const ZMQ_PORT_RANGE: std::ops::RangeInclusive<u16> = 9005..=9020;

/// Well-known MAVLink GCS / endpoint ports we probe by default.
///
/// - `14550` — Mission Planner, QGroundControl default
/// - `14551` — secondary GCS port (often used by SkyBrush / second GCS leg)
/// - `14555` — SkyBrush
/// - `14560` — real-drone-direct (HVN convention)
/// - `14570` — HIL bridge
pub const MAVLINK_PORTS: &[u16] = &[14550, 14551, 14555, 14560, 14570];

/// Default probe window — long enough for one envelope at 1 Hz, short enough
/// that the dialog feels instant. Used when callers don't override.
pub const DEFAULT_PROBE_MS: u64 = 500;

/// One backend kind for a discovered source. Maps 1:1 to the URI scheme the
/// operator would type ("zmq://" / "mavlink://"). Future schemes
/// (mavlinkout, mock) are deliberately omitted — discovery is a localhost
/// helper, not a URI catalogue.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum SourceKind {
    /// `zmq://host:port` — HVN-SITL msgpack streamer.
    Zmq,
    /// `mavlink://host:port` — direct MAVLink UDP (udpin / listen).
    Mavlink,
}

/// Whether the port had traffic during the probe window.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum DiscoveryStatus {
    /// Got data within the probe window. Optionally with a drone name from
    /// the msgpack envelope (ZMQ only — MAVLink's name comes from sysid
    /// demux and isn't surfaced here).
    Live { drone_name: Option<String> },
    /// Bound port present but no data arrived in the probe window. The
    /// operator may still want to connect and wait for traffic to start.
    Silent,
    /// Already a connected source in the App's registry. The dialog should
    /// grey out the `[+ Connect]` button.
    InUse,
}

/// One row in the dialog's "Detected on localhost:" list.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct DiscoveredSource {
    /// URI the operator would connect to (`zmq://127.0.0.1:9005`, etc.).
    pub uri: String,
    /// Which backend serves this port.
    pub kind: SourceKind,
    /// Live / Silent / InUse — drives the visual indicator + button enable.
    pub status: DiscoveryStatus,
    /// Wall-clock milliseconds (since UNIX_EPOCH) when the probe last saw a
    /// frame on this URI. `None` when the port was silent or in-use. Used by
    /// the dialog for staleness sorting; the actual freshness display is left
    /// to the caller.
    pub last_seen_ms: Option<u64>,
}

/// Run the full localhost scan and return everything we found.
///
/// `already_connected` is a slice of URIs from [`crate::SourceRegistry::uris`]
/// (or whatever the caller's connected-source list looks like) — entries that
/// match exactly get `DiscoveryStatus::InUse` instead of being probed, so the
/// dialog can grey them out without losing the entry. URIs in
/// `already_connected` that fall OUTSIDE the probed port ranges aren't
/// included in the result (the dialog only shows discoverable URIs).
///
/// `probe_duration_ms` is the per-probe budget — typically [`DEFAULT_PROBE_MS`]
/// (500 ms). All probes run in parallel via [`tokio::task::JoinSet`] so the
/// whole scan returns in roughly one probe window.
///
/// The result is sorted: live first, then silent, then in-use; within each
/// bucket, ZMQ before MAVLink, then by port ascending. Stable ordering means
/// the dialog row positions don't flicker between re-scans.
pub async fn discover_localhost_sources(
    already_connected: &[String],
    probe_duration_ms: u64,
) -> Vec<DiscoveredSource> {
    let budget = Duration::from_millis(probe_duration_ms);
    let mut join: JoinSet<Option<DiscoveredSource>> = JoinSet::new();
    let connected: std::collections::HashSet<String> =
        already_connected.iter().cloned().collect();

    // ZMQ scan.
    for port in ZMQ_PORT_RANGE {
        let uri = format!("zmq://127.0.0.1:{port}");
        if connected.contains(&uri) {
            join.spawn(async move {
                Some(DiscoveredSource {
                    uri,
                    kind: SourceKind::Zmq,
                    status: DiscoveryStatus::InUse,
                    last_seen_ms: None,
                })
            });
            continue;
        }
        join.spawn(async move { probe_zmq(port, budget).await });
    }

    // MAVLink scan.
    for &port in MAVLINK_PORTS {
        let uri = format!("mavlink://127.0.0.1:{port}");
        if connected.contains(&uri) {
            join.spawn(async move {
                Some(DiscoveredSource {
                    uri,
                    kind: SourceKind::Mavlink,
                    status: DiscoveryStatus::InUse,
                    last_seen_ms: None,
                })
            });
            continue;
        }
        join.spawn(async move { probe_mavlink(port, budget).await });
    }

    let mut out: Vec<DiscoveredSource> = Vec::new();
    while let Some(res) = join.join_next().await {
        if let Ok(Some(d)) = res {
            out.push(d);
        }
    }

    sort_discovered(&mut out);
    out
}

/// Sort order for the dialog rows. Live first (most-actionable), then
/// silent, then in-use; within each bucket ZMQ before MAVLink, ascending
/// port. Pulled out so the unit tests can pin the contract.
fn sort_discovered(v: &mut [DiscoveredSource]) {
    fn status_rank(s: &DiscoveryStatus) -> u8 {
        match s {
            DiscoveryStatus::Live { .. } => 0,
            DiscoveryStatus::Silent => 1,
            DiscoveryStatus::InUse => 2,
        }
    }
    fn kind_rank(k: &SourceKind) -> u8 {
        match k {
            SourceKind::Zmq => 0,
            SourceKind::Mavlink => 1,
        }
    }
    v.sort_by(|a, b| {
        status_rank(&a.status)
            .cmp(&status_rank(&b.status))
            .then(kind_rank(&a.kind).cmp(&kind_rank(&b.kind)))
            .then(a.uri.cmp(&b.uri))
    });
}

/// Probe one ZMQ port on `127.0.0.1`. Returns `None` when the port is closed
/// (TCP connect refused) so the caller drops it from the dialog list entirely.
async fn probe_zmq(port: u16, budget: Duration) -> Option<DiscoveredSource> {
    let addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), port);
    let uri = format!("zmq://127.0.0.1:{port}");

    // Step 1 — fast TCP check. `zeromq::SubSocket::connect` doesn't fail on
    // a closed port (it retries in the background), so we use a real TCP
    // connect with a 100 ms timeout to skip closed ports cheaply.
    //
    // The probe budget covers BOTH this TCP step AND the SUB recv that
    // follows; the TCP timeout is capped at the lesser of 150 ms and the
    // full budget so a 500 ms budget still leaves ~350 ms for the recv.
    let tcp_budget = budget.min(Duration::from_millis(150));
    match timeout(tcp_budget, TcpStream::connect(addr)).await {
        Ok(Ok(stream)) => {
            // Drop the TCP probe socket *before* opening the ZMQ subscriber.
            // The zeromq crate will open its own connection underneath.
            drop(stream);
        }
        _ => {
            // Closed port or TCP-level timeout — nothing useful here.
            return None;
        }
    }

    // Step 2 — open a real SUB and wait for one envelope. Use whatever's
    // left of the budget for the recv attempt.
    let recv_budget = budget.saturating_sub(tcp_budget);
    let endpoint = format!("tcp://127.0.0.1:{port}");

    use zeromq::{Socket, SocketRecv, SubSocket};
    let mut sock = SubSocket::new();
    if sock.subscribe("").await.is_err() {
        return Some(DiscoveredSource {
            uri,
            kind: SourceKind::Zmq,
            status: DiscoveryStatus::Silent,
            last_seen_ms: None,
        });
    }
    if sock.connect(&endpoint).await.is_err() {
        return Some(DiscoveredSource {
            uri,
            kind: SourceKind::Zmq,
            status: DiscoveryStatus::Silent,
            last_seen_ms: None,
        });
    }

    let status = match timeout(recv_budget, sock.recv()).await {
        Ok(Ok(msg)) => {
            // Try to decode the first frame to grab the drone name. If it
            // doesn't decode we still report Live — the operator can decide
            // whether to connect a non-HVN ZMQ stream.
            let payload: Vec<u8> = if msg.len() == 1 {
                msg.get(0).map(|b| b.to_vec()).unwrap_or_default()
            } else {
                let mut buf = Vec::new();
                for f in msg.iter() {
                    buf.extend_from_slice(f);
                }
                buf
            };
            let drone_name = decode_drone_name(&payload);
            DiscoveryStatus::Live { drone_name }
        }
        // recv error OR timeout: port was bindable + we connected, but no
        // frame arrived in the window. Report Silent so the operator sees
        // the port and can connect anyway.
        _ => DiscoveryStatus::Silent,
    };

    // Explicit drop so the socket closes before we return. (Drop-on-scope-end
    // would do the same, but being explicit makes the intent clear.)
    drop(sock);

    Some(DiscoveredSource {
        uri,
        kind: SourceKind::Zmq,
        status,
        last_seen_ms: now_ms(),
    })
}

/// Decode `payload` as a msgpack envelope and pull out the `drone_name`. We
/// use the existing [`flatten_msgpack`] helper so wire-schema parity is
/// guaranteed (no second decoder to keep in sync).
fn decode_drone_name(payload: &[u8]) -> Option<String> {
    let samples: Vec<Sample> = flatten_msgpack(payload).ok()?;
    samples
        .first()
        .and_then(|s| s.drone_name.as_ref().map(|a| a.to_string()))
}

/// Probe one MAVLink UDP port on `127.0.0.1`. Returns `None` when the bind
/// fails (port in use by another process) — the profiler can't open it
/// anyway, so showing it in the dialog would be misleading.
async fn probe_mavlink(port: u16, budget: Duration) -> Option<DiscoveredSource> {
    let bind_addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), port);
    let uri = format!("mavlink://127.0.0.1:{port}");

    let sock = match UdpSocket::bind(bind_addr).await {
        Ok(s) => s,
        Err(_) => {
            // EADDRINUSE — another process owns the port. The profiler can't
            // listen on it either, so omit it from the result.
            return None;
        }
    };

    let mut buf = [0u8; 280]; // MAVLink v2 max frame ≈ 280 bytes
    let status = match timeout(budget, sock.recv_from(&mut buf)).await {
        Ok(Ok((n, _peer))) if n > 0 => {
            // Real MAVLink frames start with 0xFD (v2) or 0xFE (v1). Anything
            // else is junk — still report Live (the operator might be running
            // something exotic).
            let _looks_mavlink = matches!(buf[0], 0xFD | 0xFE);
            DiscoveryStatus::Live { drone_name: None }
        }
        // Timeout or recv error: bindable but no traffic.
        _ => DiscoveryStatus::Silent,
    };

    // Drop the socket so the port is released immediately and the profiler
    // can re-bind it on Connect.
    drop(sock);

    Some(DiscoveredSource {
        uri,
        kind: SourceKind::Mavlink,
        status,
        last_seen_ms: now_ms(),
    })
}

/// Wall-clock milliseconds since UNIX_EPOCH. Used for `last_seen_ms`.
/// Falls back to `None` if the system clock is somehow before the epoch.
fn now_ms() -> Option<u64> {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .ok()
        .map(|d| d.as_millis() as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `sort_discovered` orders rows: Live → Silent → InUse; ZMQ before
    /// MAVLink within each bucket; then ascending URI. The dialog relies on
    /// this contract for stable row positions.
    #[test]
    fn sort_orders_live_silent_inuse() {
        let mut v = vec![
            DiscoveredSource {
                uri: "mavlink://127.0.0.1:14550".into(),
                kind: SourceKind::Mavlink,
                status: DiscoveryStatus::Silent,
                last_seen_ms: None,
            },
            DiscoveredSource {
                uri: "zmq://127.0.0.1:9008".into(),
                kind: SourceKind::Zmq,
                status: DiscoveryStatus::InUse,
                last_seen_ms: None,
            },
            DiscoveredSource {
                uri: "zmq://127.0.0.1:9006".into(),
                kind: SourceKind::Zmq,
                status: DiscoveryStatus::Live { drone_name: Some("eric_2".into()) },
                last_seen_ms: Some(2),
            },
            DiscoveredSource {
                uri: "zmq://127.0.0.1:9005".into(),
                kind: SourceKind::Zmq,
                status: DiscoveryStatus::Live { drone_name: Some("eric_1".into()) },
                last_seen_ms: Some(1),
            },
            DiscoveredSource {
                uri: "zmq://127.0.0.1:9007".into(),
                kind: SourceKind::Zmq,
                status: DiscoveryStatus::Silent,
                last_seen_ms: None,
            },
        ];
        sort_discovered(&mut v);
        let order: Vec<&str> = v.iter().map(|d| d.uri.as_str()).collect();
        assert_eq!(
            order,
            vec![
                "zmq://127.0.0.1:9005",         // Live, ZMQ
                "zmq://127.0.0.1:9006",         // Live, ZMQ
                "zmq://127.0.0.1:9007",         // Silent, ZMQ
                "mavlink://127.0.0.1:14550",    // Silent, MAVLink
                "zmq://127.0.0.1:9008",         // InUse, ZMQ
            ],
        );
    }

    #[test]
    fn zmq_port_range_matches_sitl_streamer() {
        // Pinning: SITL drone 1 → port 9005, drone 16 → port 9020.
        let ports: Vec<u16> = ZMQ_PORT_RANGE.collect();
        assert_eq!(ports.first().copied(), Some(9005));
        assert_eq!(ports.last().copied(), Some(9020));
        assert_eq!(ports.len(), 16);
    }

    #[test]
    fn mavlink_ports_include_canonical_gcs_ports() {
        // Pinning: Mission Planner + QGC default port is 14550. Don't drop
        // it accidentally during refactors.
        assert!(MAVLINK_PORTS.contains(&14550));
        assert!(MAVLINK_PORTS.contains(&14551));
    }
}
