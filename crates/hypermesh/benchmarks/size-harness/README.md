# Hypermesh size harness

This dependency-only binary exercises one overlapping exact Boolean through
certified triangle materialization. It avoids Hypermesh's dev dependencies, so
native and `wasm32-unknown-unknown` artifacts measure the linked Hyper stack
rather than Criterion, fuzzing, UI, or competitor code.

The operation is selected from a runtime argument so the size consumer retains
the complete Boolean selection path:

```sh
cargo run --release -- union
cargo run --release -- intersection
cargo run --release -- difference
cargo run --release -- symmetric-difference
```

Measure the default or all-feature dependency graph:

```sh
./measure.sh default
./measure.sh all
```

Set `HYPERMESH_SIZE_TARGET_DIR` to keep build artifacts elsewhere. The script
reports raw and compressed bytes, native sections, `wasm-opt -Oz` output, and
artifact hashes for both the speed-oriented `release` profile and the
size-oriented `size` profile.
