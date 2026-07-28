# Hyperlattice fuzz targets

This unpublished `cargo-fuzz` package exercises Hyperlattice scalar, complex,
vector, and matrix operations with bounded generated values. It also crosses
all public Hyperreal structural representations through lattice construction
and algebra.

## Run

From the Hyperlattice repository:

```sh
cargo install cargo-fuzz
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run vector_ops --fuzz-dir fuzz -- -max_total_time=30
```

Targets are:

- `scalar_ops`
- `complex_ops`
- `vector_ops`
- `matrix_ops`
- `hyperreal_representations`

Minimize failures and promote license-clean inputs to the closest deterministic
unit or regression test. This package is `publish = false` and follows
Hyperlattice's Apache-2.0 license.
