# Hypergraphics fuzzing

`scene_invariants` crosses every Hyperreal structural representation through
scene construction, exact triangle predicates, cache reuse, mutation, and the
explicit lossy render boundary.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run scene_invariants --fuzz-dir fuzz -- -max_total_time=30
```
