# Hypergraphics fuzzing

`scene_invariants` derives structured inputs that select Hyperreal structural
representations, predicate policies, valid and invalid triangle indices, cache
reuse and mutation, checked colors, bounded grids, camera projection and
conditioned approximate unprojection, and exact polygon triangulation. This
keeps input mutations coverage-relevant instead of repeating the same full
cross-product for every byte string.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run scene_invariants --fuzz-dir fuzz -- -max_total_time=30
```
