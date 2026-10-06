# Hypercurve fuzzing

The suite covers Bezier and B-spline evaluation, splitting, arrangements,
regions and Boolean operations, curve-string editing, retained import,
straight skeletons, and SVG input. `hyperreal_representations` additionally
crosses every pair of the eight public Hyperreal structural kinds through
exact similarity transformations and native curve evaluation.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```
