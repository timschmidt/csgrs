# Policy-aware cache checkpoint — 2026-07-30

This checkpoint measures Hypermesh `911f9caf` against its direct parent
`0973f776` with the same current Hyperreal, Hyperlattice, and Hyperlimit
dependencies.

## Correctness and ownership

- Reusable PWN and convexity facts are one-byte monotonic atomics. A certified
  fact works under either policy; an approximate fact is consumed only by
  `APPROXIMATE_512` and marks the operation outcome. Later strict certification
  upgrades, rather than being blocked by, an earlier approximate fact.
- Retained Boolean polygons and construction planes carry their producing
  certainty. `STRICT` rebuilds approximate provenance from native exact
  triangles instead of reusing it.
- Empty-passthrough, pointer-identical, and certified-disjoint carrier
  shortcuts validate every nonempty operand before returning. Open,
  degenerate, and non-PWN input can no longer bypass the canonical input
  contract through an algebraic terminal.
- Certified PWN facts are retained across transformations that preserve the
  proof and across exact triangle subdivision. Reversed winding retains PWN
  validity but deliberately does not retain outward convexity.
- The rational linear-form filter cache is lazy and operation-local. Exact
  operations that never request a filter allocate nothing for it, and rational
  owners are released when the operation ends instead of being retained in
  thread-local state.

Policy-order, invalid-shortcut, terminal-policy, and cache-collision
regressions pass under both feature modes.

## Dispatch

One general cube union through `boolean_operation` used 42 unique linear-form
filters and 3,792 cache hits: a 98.90% hit rate with no capacity clear. The
operation-local lifetime therefore preserves reuse while bounding retained
owners to the current operation.

## Long-lived memory

The checked-in `benchmarks/size-harness/src/bin/retention.rs` consumer performs
512 sequential general cube unions through raw mesh views. Heaptrack includes
process setup and all output destruction.

| Evidence | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| Peak heap | 1,063,232 B | 335,024 B | -68.49% |
| Allocation calls | 3,392,817 | 3,397,405 | +0.135% |
| Temporary allocations | 212,477 | 212,477 | 0 |
| Heaptrack runtime | 2.179 s | 2.156 s | -1.06% |
| Peak RSS including Heaptrack | 9.23 MiB | 8.44 MiB | -8.56% |

## Runtime

CPU-0 Criterion confidence intervals for one general cube union were
2.2442–2.2632 ms at the parent and 2.2483–2.3325 ms at the checkpoint; they
overlap. Five CPU-0 `perf stat` repetitions over 512 unions reported:

| Evidence | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| Instructions | 9,323,990,377 ±0.02% | 9,322,836,137 ±0.01% | -0.012% |
| Cycles | 4,542,252,729 ±1.08% | 4,614,024,720 ±1.56% | +1.58% |
| Task clock | 1,073.75 ms ±1.09% | 1,090.47 ms ±1.59% | +1.56% |

Instruction counts are unchanged and the noisier cycle/time intervals overlap,
so no runtime regression is resolved.

## Linked artifact size

Percentages are checkpoint growth over the parent. `native code` is `.text`;
`WASM code` is `wasm-opt -Oz`.

| Consumer | Profile | Target | Parent raw | Current raw | Raw change | Parent code | Current code | Code change |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| General | Release | Native | 4,318,312 | 4,317,416 | -0.0207% | 3,703,653 | 3,703,453 | -0.0054% |
| General | Release | WASM | 3,336,958 | 3,337,021 | +0.0019% | 2,593,657 | 2,594,043 | +0.0149% |
| General | Size | Native | 1,909,072 | 1,909,408 | +0.0176% | 1,672,963 | 1,673,291 | +0.0196% |
| General | Size | WASM | 1,277,944 | 1,277,919 | -0.0020% | 1,085,379 | 1,085,402 | +0.0021% |
| Immediate | Release | Native | 4,353,864 | 4,353,248 | -0.0141% | 3,737,509 | 3,737,741 | +0.0062% |
| Immediate | Release | WASM | 3,353,967 | 3,354,177 | +0.0063% | 2,608,785 | 2,609,374 | +0.0226% |
| Immediate | Size | Native | 1,921,712 | 1,922,016 | +0.0158% | 1,685,063 | 1,685,375 | +0.0185% |
| Immediate | Size | WASM | 1,288,414 | 1,288,544 | +0.0101% | 1,096,003 | 1,096,173 | +0.0155% |

The maximum linked-code movement is +0.0226%.

## Verification

- `cargo test --all-features`: 1,153 executed tests passed; 7 ignored.
- `cargo test --no-default-features`: 1,151 executed tests passed; 7 ignored.
- all-target checks passed with all features and without default features.
- every fuzz target compiled.
- `cargo clippy --all-targets --all-features -- -D warnings` passed.
- native/WASM release and size consumers compiled and were measured.
- formatting, shell syntax, TOML parsing, and `git diff --check` passed.

Machine-readable values are in `policy-cache-2026-07-30.toml`.
