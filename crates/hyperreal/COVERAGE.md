# Hyperreal Coverage Contract

Hyperreal's representation space has finite discriminants and unbounded
parameters. Coverage is exhaustive over every finite representation tag and
uses matrices, property tests, and fuzzing for the unbounded values and graph
topologies carried by those tags. “Every representation” does not mean
enumerating every rational coefficient, precision, or expression DAG, which
would be infinite.

## Representation inventory

| Layer | Complete finite inventory | Regression guard |
| --- | ---: | --- |
| Public `StructuralKind` | 8 | Exhaustive Rust `match` plus observed-kind assertion |
| Private optimized `Real::Class` | 20 | Named construction recipe, serialized-tag assertion, and serde unknown-variant drift probe |
| Private `Computable::Approximation` node | 57 | One valid serialized construction per tag, evaluation at two precisions, round trip, opaque-`Real` embedding, and serde drift probe |
| Private shared constant | 18 | One construction/evaluation per tag plus serde drift probe |
| `RationalStorageClass` | 4 | Zero, word-sized, multi-limb, and very-large representatives |
| `PrimitiveFloatStatus` | 5 per primitive type | Zero, normal, subnormal/underflow, overflow, and unknown representatives for both binary32 and binary64 |
| Primitive approximation cache | Every compiled state | Empty, finite, and overflow values under no cache, binary32-only, binary64-only, and dual-cache builds |
| Cancellation state | 2 | Clear and already-signaled abort handles on exact, symbolic, and opaque values |

The 20 optimized `Real` certificates are:

`One`, `Pi`, `PiPow`, `PiInv`, `PiExp`, `PiInvExp`, `PiSqrt`,
`ConstProduct`, `ConstOffset`, `ConstProductSqrt`, `Sqrt`, `Exp`,
`Ln`, `LnAffine`, `LnProduct`, `Log10`, `Log2`, `SinPi`,
`TanPi`, and `Irrational`.

`tests/real_representations.rs` checks that every recipe really serializes
to its named private tag rather than merely landing in the expected public
category. It then crosses all 400 ordered certificate pairs through owned and
borrowed addition, subtraction, multiplication, and division; checks
commutativity or antisymmetry as appropriate; exercises inversion, rational
scales, positive/negative/zero signs, assignments, primitive operands,
iterator sums, bounded equality/order/sign certification, and JSON/CBOR cache
omission.

The finite serde inventory intentionally fails when a new private enum variant
is added without a representative.
`fuzz/fuzz_targets/structural_representations.rs` covers the infinite
remainder: parameter mutations and shared `Computable` DAGs up to 32
generated nodes. Its 20 seed strides guarantee that the initial corpus
contains every ordered offset through the 20x20 binary-dispatch matrix.

## Feature matrix

Run the focused representation contract in every primitive-cache state and in
the all-feature serde state:

```sh
scripts/representation_coverage.sh
```

Run instrumented coverage across no features, each primitive cache
independently, both caches, serde, the Simple expression language, all
features, examples, binaries, and all benchmark fixtures:

```sh
scripts/coverage.sh
```

The coverage script uses `target/coverage`, so ordinary build artifacts and
profiles are not mixed with instrumented objects. Benchmark report writers
honor `HYPERREAL_SKIP_BENCHMARK_REPORTS`; executing fixtures for coverage
therefore cannot rewrite tracked timing ledgers.

The report presents two views:

- LLVM's instrumented Rust-source summary, including inline unit-test modules.
- A physical executable-line view that excludes dedicated test-only files and
  trailing inline `#[cfg(test)] mod tests` blocks while retaining the
  production lines those tests execute.

The 2026-08-29 all-feature/all-target run measured 36,086 of 38,979 raw source
lines (92.58%) and 23,299 of 25,878 production executable lines (90.03%).
Current Rust coverage objects emit no branch counters, so the report does not
mislabel region coverage as branch coverage. The browsable report is written
to `target/coverage/html/index.html`.

## Fuzzing

Compile all six targets:

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
```

The targets divide the unbounded input space into rational arithmetic, exact
`Real` operations, elementary functions, direct `Computable`
approximation, structural representations, and numeric-string parsing. See
`fuzz/README.md` for bounded smoke and long-running campaign commands.

## Performance and memory

Every optimized certificate has a Criterion construction/export row and
prepared clone/export/certification rows:

```sh
cargo bench --bench real_representations
```

The construction, clone, and binary64-export boundaries have paired 192-bit
Rug/MPFR rows for the same mathematical values. MPFR supplies a
fixed-precision approximation while Hyperreal retains exact symbolic meaning;
the distinction is part of the benchmark interpretation. The separate
`gmp_api` suite compares or explicitly classifies every public numeric API,
and `tests/gmp_api_coverage.rs` prevents that competitive inventory from
silently drifting.

Run the counting-allocator profile with a chosen lifecycle count:

```sh
scripts/memory_profile.sh 64
```

The CSV output records allocation, deallocation, and reallocation events;
allocated/deallocated bytes; peak live-byte growth; and retained bytes for all
20 certificate forms after shared process-cache warmup.
