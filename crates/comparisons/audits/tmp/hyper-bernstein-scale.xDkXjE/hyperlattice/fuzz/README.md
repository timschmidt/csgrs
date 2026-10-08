# Hyperlattice fuzz targets

This unpublished `cargo-fuzz` package exercises Hyperlattice scalar, complex,
vector, point, projective, and matrix operations with bounded generated values.
The `hyperreal_representations` target constructs all 22 finite optimized
Hyperreal certificate classes on every execution, asserts coverage of all 8
public structural kinds, and grows additional input-directed opaque expression
DAGs. A rotating stride exposes every ordered cross-representation pairing to
the fuzzer without making each execution quadratic.

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

## Retained performance offenders

Run `cargo bench --bench retained_fuzz` from the repository root to replay
deterministic inputs shaped by every target above. The run merges the latest
timings into `slow_performers.txt`, retaining each target/seed pair's worst-ever
observation. It also rotates the worst eligible case into the 100-case
`promoted_slow_offenders.txt` lexicase set, emits a dedicated Criterion row for
every promoted case, and refreshes its average, delta, and delta derivative in
`benchmarks.md`.
