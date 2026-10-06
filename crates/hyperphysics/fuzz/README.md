# Hyperphysics fuzzing

`physics_invariants` crosses every pair of Hyperreal structural
representations through exact boxes, support maps, contact, GJK, planes, rays,
segments, triangles, and mesh handoff.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run physics_invariants --fuzz-dir fuzz -- -max_total_time=30
```
