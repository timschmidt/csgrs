# Hypermesh fuzzing

The targets construct only bounded exact-integer geometry. Primitive floats are
not used to choose topology, and every successful Boolean result is replayed
through the public closure and certified-triangulation checks.

Compile every target:

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
```

Run bounded smoke campaigns from the repository root:

```sh
cargo +nightly fuzz run polygon_predicates --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run bvh_queries --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run mesh_and_hull --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run boolean_pipeline --fuzz-dir fuzz -- -max_total_time=30 -timeout=10
```

The Boolean target is intentionally separate because certified arrangement
construction is much more expensive than predicate, BVH, and input-validation
work. Minimize any crash and promote it to a deterministic regression test.
