# Memristor Simulator (prototype)

This is a minimal prototype of a memristor simulator written in Rust. It demonstrates:

- A simple memristor model (linear drift, bounded state 0..1)
- Composable components: `Series` and `Parallel` networks
- Time-stepping simulation with state updates and measurement recording
- A CLI demo that prints CSV measurements

Usage

1. Build and run the demo (requires Rust toolchain):

```powershell
cd C:\Users\capta\Desktop\MEMRISTORS
cargo run --bin cli
```

2. The demo prints CSV to stdout: `time,path,voltage,current,state`.

Extending

- Add more realistic memristor dynamics in `src/lib.rs`.
- Implement a richer driver (AC, pulses, sweep) in `src/bin/cli.rs`.
- Add export to files or a simple web UI served by a small web server or WebAssembly frontend for visualization.

Notes

This is a prototype intended as a starting point — it supports series/parallel topologies with a single source across the whole network. Extending to arbitrary netlists, multiple nodes, and full nodal analysis is the natural next step.
