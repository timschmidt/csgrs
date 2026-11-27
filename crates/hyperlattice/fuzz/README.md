# Hyperlattice fuzz targets

This unpublished standalone `cargo-fuzz` package exercises selected Hyperlattice
scalar, complex, vector, point, projective, and matrix operations. The four
`*_ops` targets seed coordinates with exact finite-binary64 values or i128
integers. NaN/infinity bit patterns are replaced by zero before scalar import.
These seeds are dyadic rationals; magnitudes span the finite float range.

`hyperreal_representations` builds 22 fixed scalar recipes, checks their ordered
public structural kinds (eight categories), and adds four opaque graph recipes.
It then applies a nonzero integer scale and visits one rotated partner per value.
Varying the stride makes every ordered recipe pair reachable; the target does
not assert campaign coverage or enumerate every private class/composition.

Most operation calls discard their results and test panic resistance. Identity
checks share the implementation under test. Several cancellation assertions
accept `Unknown`. Representation equality helpers accept certified equality or,
failing that, overlapping rational enclosures; overlap alone does not prove
exact equality. Abort-aware vector/matrix calls use an inactive flag.

Finite input encodings, bounded exponents, and one-to-eight graph updates do not
bound total evaluation work, expression growth, memory, or cancellation latency.
Use the fuzzing engine's execution/resource controls when running campaigns.

## Run

From the monorepo root, with `cargo-fuzz` and a suitable nightly toolchain installed:

```sh
cargo check --manifest-path crates/hyperlattice/fuzz/Cargo.toml --bins
cd crates/hyperlattice
cargo +nightly fuzz run vector_ops --fuzz-dir fuzz -- -max_total_time=30
```

Targets are `scalar_ops`, `complex_ops`, `vector_ops`, `matrix_ops`, and
`hyperreal_representations`. The manifest excludes them from ordinary test/doc
selection. To inspect their private source documentation explicitly, use, for
example, from the monorepo root:

```sh
cargo rustdoc --manifest-path crates/hyperlattice/fuzz/Cargo.toml \
  --bin scalar_ops -- --document-private-items
```

The manual Input decoders keep Arbitrary's default zero lower size hint and can
synthesize zero operands from short/empty input with the current decoder. The
representation target derives an eight-byte lower hint, which the typed macro
checks before entering the callback. Neither rule implies uniform sampling.

Minimize failures and promote suitable inputs to deterministic regressions while
preserving the original hard cases. This package is `publish = false`; see
Hyperlattice's [MIT license](../LICENSE). Its separate lockfile and generated
corpus/artifacts are ignored by Git.

## Retained performance workloads

From the monorepo root, `cargo bench -p hyperlattice --bench retained_fuzz` runs
the separate deterministic workloads in [retained_fuzz.rs](../benches/retained_fuzz.rs).
They use named target/seed recipes rather than replaying the full libFuzzer
callbacks or raw corpus. Benchmark reports are separate evidence from fuzz
coverage and correctness checks.
