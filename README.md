# hvn-profiler

GPU-accelerated telemetry profiler for [HVN-SITL](https://github.com/HVN-Labs/SITL)
and ArduPilot real drones. Built in Rust on top of `egui` + `wgpu` so it can
sustain 60 FPS while ingesting 200+ Hz telemetry streams across dozens of traces.

Companion to the HVN-SITL project. Eventually replaces the matplotlib-based
`sensor_plot.py` that ships with HVN-SITL today.

## Architecture

```
                                  +---------------------+
  Real drone --- MAVLink (UDP) -->|                     |
                                  |                     |
  HVN-SITL --- streamer (ZMQ) --->|   hvn-profiler      |---> egui + wgpu
                                  |   (Rust workspace)  |     window @ 60 FPS
  CSV / log file ---------------->|                     |
                                  +---------------------+
```

Workspace crates:

| crate | purpose |
| --- | --- |
| `profiler-cli` | binary entry point — opens the egui window |
| `profiler-render` | egui_plot wrappers + GPU-friendly trace storage |
| `profiler-source` | `Source` trait + mock / MAVLink / ZMQ / CSV backends |
| `profiler-template` | JSON template loader (layout, traces, units) |

## Build

```bash
cargo build --release
```

## Run (today)

```bash
# Mock sine-wave demo — proves egui + wgpu work on this machine.
cargo run --release --bin hvn-profiler

# Live ZMQ data from the HVN-SITL streamer (v0.7.18+):
#   python -m hvn_sitl.streamer --source mavlink://127.0.0.1:14560 \
#                               --pub tcp://127.0.0.1:9005
cargo run --release --bin hvn-profiler -- --source zmq://127.0.0.1:9005
```

The ZMQ backend uses the pure-Rust [`zeromq`](https://crates.io/crates/zeromq)
crate — no libzmq C dependency needed on Windows.

## Roadmap

| version | scope |
| --- | --- |
| v0.0.1 | Mock sine-wave demo (toolchain proof) |
| v0.1.0 | ZMQ msgpack source; render one live trace <- **shipping now** |
| v0.2.0 | JSON template loading; multi-panel layout |
| v0.3.0 | Live controls (view slider, trail, zoom, decimation) |
| v0.4.0 | Direct MAVLink source (real-drone support) |
| v0.5.0 | 3D trails + per-panel desc/label overlay |
| v0.6.0 | Feature parity with HVN-SITL's `sensor_plot.py` |

## License

MIT
