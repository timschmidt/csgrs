# Hypersdf fuzzing

The suite covers exact-aware dual and gradient contouring.
`hyperreal_representations` crosses every pair of the eight public Hyperreal
structural kinds through arithmetic, CSG, translation, point, interval, batch,
and preview expression paths.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```
