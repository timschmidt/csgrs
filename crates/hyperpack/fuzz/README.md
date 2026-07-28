# Hyperpack fuzzing

`packing_invariants` crosses every pair of Hyperreal structural
representations through exact dimensions, replay, capacity and incompatibility
bounds, model export, analysis, objectives, snapshots, and every 3D heuristic.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run packing_invariants --fuzz-dir fuzz -- -max_total_time=30
```
