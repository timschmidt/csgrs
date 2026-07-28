# Hyperbrep fuzzing

The suite covers planar pcurves, BREP-to-voxel handoff, and exact spatial
curves/surface frames. `hyperreal_representations` crosses all eight public
Hyperreal structural kinds against each other in the curve and surface APIs.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```
