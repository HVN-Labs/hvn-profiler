//! v0.16.4 — discovery probes bind `0.0.0.0`, not `127.0.0.1`.
//!
//! WSL2 vehicles send to the Windows host NIC (source IP `172.x.x.x`); a
//! loopback bind silently drops those packets so the auto-discovery scan
//! reported every `:14560` MAVLink port as "Silent" even when 25 drones
//! were flooding it.
//!
//! The contract:
//!
//! - `discover_localhost_sources()` returns every MAVLink URI with the
//!   `mavlink://0.0.0.0:PORT` form (not the old `mavlink://127.0.0.1:PORT`).
//! - The ZMQ URI scheme stays `127.0.0.1` — the streamer always binds
//!   loopback, so we don't widen its probe.
//! - When the operator's `already_connected` list contains the legacy
//!   loopback form, it still dedupes against the new `0.0.0.0` form so a
//!   manually-typed `mavlink://127.0.0.1:14560` collapses with the
//!   auto-discovered `mavlink://0.0.0.0:14560`.

use profiler_source::{
    canonicalise_source_uri, discover_localhost_sources, DiscoveryStatus,
    SourceKind,
};

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn every_mavlink_probe_advertises_zero_zero_zero_zero() {
    // No publishers / vehicles — the probe just needs to enumerate ports.
    // Use the smallest sensible probe window so the test is fast.
    let results = discover_localhost_sources(&[], 50).await;

    let mavlink_uris: Vec<&str> = results
        .iter()
        .filter(|d| d.kind == SourceKind::Mavlink)
        .map(|d| d.uri.as_str())
        .collect();

    assert!(
        !mavlink_uris.is_empty(),
        "no MAVLink ports were enumerated at all",
    );
    for uri in &mavlink_uris {
        assert!(
            uri.starts_with("mavlink://0.0.0.0:"),
            "MAVLink probe URI should bind 0.0.0.0, got: {uri}",
        );
        assert!(
            !uri.starts_with("mavlink://127.0.0.1:"),
            "pre-v0.16.4 loopback URI leaked: {uri}",
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_loopback_uri_still_dedupes_via_inuse() {
    // Operator added the source with the legacy form. Discovery should
    // still surface the port as `InUse` (not Live/Silent — the dialog
    // mustn't propose to re-add it).
    let already_connected = vec!["mavlink://127.0.0.1:14550".to_string()];
    let results = discover_localhost_sources(&already_connected, 50).await;
    let entry = results
        .iter()
        .find(|d| {
            d.kind == SourceKind::Mavlink
                && d.uri.ends_with(":14550")
        })
        .expect("14550 should appear in the result");
    assert!(
        matches!(entry.status, DiscoveryStatus::InUse),
        "legacy-form connected URI should be marked InUse; got {:?}",
        entry.status,
    );
    // The advertised URI is the canonical 0.0.0.0 form.
    assert_eq!(
        entry.uri, "mavlink://0.0.0.0:14550",
        "advertised URI should be canonical 0.0.0.0",
    );
}

#[test]
fn canonicalise_collapses_loopback_to_zero_zero_zero_zero() {
    assert_eq!(
        canonicalise_source_uri("mavlink://127.0.0.1:14560"),
        "mavlink://0.0.0.0:14560",
    );
    // Already canonical → identity.
    assert_eq!(
        canonicalise_source_uri("mavlink://0.0.0.0:14560"),
        "mavlink://0.0.0.0:14560",
    );
    // ZMQ stays untouched: the streamer always binds loopback.
    assert_eq!(
        canonicalise_source_uri("zmq://127.0.0.1:9005"),
        "zmq://127.0.0.1:9005",
    );
    // mavlinkout target stays untouched: the operator's destination matters.
    assert_eq!(
        canonicalise_source_uri("mavlinkout://127.0.0.1:14550"),
        "mavlinkout://127.0.0.1:14550",
    );
    // Non-loopback hosts pass through (e.g. an explicit interface).
    assert_eq!(
        canonicalise_source_uri("mavlink://192.168.1.10:14550"),
        "mavlink://192.168.1.10:14550",
    );
}
