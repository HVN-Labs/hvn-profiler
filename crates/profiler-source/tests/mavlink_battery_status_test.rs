//! v0.16.3 — `BATTERY_STATUS` decode: cell-voltage sum (mV per cell,
//! `0xFFFF` = unused) → volts; current cA → A; remaining %.

#![cfg(feature = "mavlink-source")]

use mavlink::dialects::ardupilotmega::{BATTERY_STATUS_DATA, MavMessage};
use profiler_source::mavlink_source::decode_to_samples;

#[test]
fn battery_status_sums_cell_voltages_and_scales_current() {
    let mut voltages = [u16::MAX; 10];
    // 4S pack: 3700 mV × 4 cells = 14800 mV = 14.8 V.
    voltages[0] = 3700;
    voltages[1] = 3700;
    voltages[2] = 3700;
    voltages[3] = 3700;

    let d = BATTERY_STATUS_DATA {
        voltages,
        current_battery: 1250, // cA → 12.5 A
        battery_remaining: 87,
        id: 0,
        ..Default::default()
    };

    let msg = MavMessage::BATTERY_STATUS(d);
    let samples = decode_to_samples(&msg, 0.0);
    let by_key: std::collections::HashMap<_, _> =
        samples.iter().map(|s| (s.key.as_str(), s.scalar())).collect();
    assert!((by_key["battery_voltage"] - 14.8).abs() < 1e-9);
    assert!((by_key["battery_current"] - 12.5).abs() < 1e-9);
    assert_eq!(by_key["battery_remaining"], 87.0);
}
