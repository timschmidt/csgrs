# Hyperpath fuzzing

The suite covers line, arc, conic, and Bezier arrangements; PCB geometry and
fabrication rules; Specctra import/export; CAM; routing; keepouts; and feed
schedules. `hyperreal_representations` crosses all eight public Hyperreal
structural kinds against each other through translated exact path topology.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```
