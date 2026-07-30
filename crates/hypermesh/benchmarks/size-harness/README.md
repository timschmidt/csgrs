# Hypermesh size harness

These dependency-only binaries exercise overlapping exact Booleans without
Hypermesh's dev dependencies, so native and `wasm32-unknown-unknown` artifacts
measure the linked Hyper stack rather than Criterion, fuzzing, UI, or
competitor code.

`hypermesh-size-harness` retains polygon selection plus certified
triangulation. `immediate` retains direct certified triangle materialization,
including analytic fast paths. Each selects the operation from a runtime
argument; `immediate` also accepts `strict` as its second argument:

```sh
cargo run --release -- union
cargo run --release -- intersection
cargo run --release -- difference
cargo run --release -- symmetric-difference
cargo run --release --bin immediate -- union strict
```

Measure the default or all-feature dependency graph:

```sh
./measure.sh default
./measure.sh all
```

Set `HYPERMESH_SIZE_TARGET_DIR` to keep build artifacts elsewhere. The script
reports both binaries' raw and compressed bytes, native sections,
`wasm-opt -Oz` output, and artifact hashes for the speed-oriented `release`
profile and size-oriented `size` profile.
