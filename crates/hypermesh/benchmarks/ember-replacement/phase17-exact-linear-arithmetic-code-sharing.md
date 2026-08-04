# Phase 17 exact linear-arithmetic code-sharing checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol at
Hyperreal `c2217bae132b725e29faeb89495b8cd51012e1bc` and Hypermesh
`410f9372038207bfc2ce208fd8d370464568c091`.

This checkpoint recovers linked size after the retained exact-construction
work without changing a Boolean algorithm, scalar algorithm, public operator,
retained fact, or Hyperlimit decision. Rust had emitted the complete generic
`Rational` addition and subtraction bodies for several downstream `AsRef`
monomorphizations. Hyperreal now keeps one fixed-signature exact core for each
operation; the existing generic operator adapters only obtain the borrowed
operand and call that core.

The arithmetic order is unchanged. Zero/unit shortcuts, retained sum and
directed-difference lookups, word-sized dispatch, denominator GCD, scaled
numerator construction, exact cancellation, possible-divisor reduction, and
retained-result publication occur in the same order as before. The fixed
boundary is deliberately non-inlined so downstream crates cannot clone the
large body again. It contains no mesh, fixture, bit-width, policy, topology,
operation-result, or expected-output condition.

This is a general Hyperreal cleanup that plays to retained facts: callers still
reach the same cached fast paths and retain the same results, but the cache
logic has one instruction body and better locality. Multiplication and division
remain inlinable after direct scalar measurements showed that centralizing them
would trade too much runtime for size.

## Policy and exactness

Both cores return the same exact `Rational` values and populate the same bounded
linear caches. No approximation, comparison, or terminal was added. Hypermesh
continues to route every uncertain equality through Hyperlimit:

- `STRICT` cannot terminate by approximation;
- `APPROXIMATE_512` may terminate only through Hyperlimit's 512-bit equality
  policy; and
- every large fixture measured here remains exactly policy-equal and
  `Certified`, so neither consumes the approximate terminal.

## Monomorphization and linked size

Before the change, `cargo bloat` found four 3.1--3.2 KiB addition bodies and
multiple approximately 2.7 KiB subtraction bodies in the canonical release
consumer, in addition to small owned/operator adapters. After the change, the
heavy implementation is one 3.2 KiB `add_ref` and one 2.8 KiB `subtract_ref`;
the remaining filtered generic adapters total 556 bytes for addition and 258
bytes for subtraction.

The parent is the affine-edge-plane checkpoint at Hyperreal `f59a6ce7` and
Hypermesh `5f9e45d2`. Native values are executable text. WASM values are after
`wasm-opt -Oz`.

| Profile / consumer | Parent native text | Current native text | Movement | Parent optimized WASM | Current optimized WASM | Movement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| release / general | 2,044,326 B | 2,020,214 B | -24,112 B (-1.179%) | 1,450,878 B | 1,437,333 B | -13,545 B (-0.934%) |
| release / immediate | 2,047,470 B | 2,023,358 B | -24,112 B (-1.178%) | 1,452,733 B | 1,439,188 B | -13,545 B (-0.932%) |
| size / general | 1,084,559 B | 1,084,423 B | -136 B (-0.0125%) | 679,339 B | 679,361 B | +22 B (+0.0032%) |
| size / immediate | 1,085,499 B | 1,085,371 B | -128 B (-0.0118%) | 679,746 B | 679,768 B | +22 B (+0.0032%) |

The two implementation commits add a net 14 source lines. The 22-byte
size-profile WASM movement is an explicit Pareto loss; it is retained because
both speed-profile consumers shrink materially, native size-profile consumers
also shrink, scalar runtime improves, and no production algorithm is duplicated.

## Direct exact-scalar performance

The fixed inputs are the permanent `borrowed_ops` exact-rational pair. Parent
and current Criterion binaries were run serially and pinned to CPU 11, with 100
samples after the standard three-second warmup. Values below are independent
means and reported 95% intervals.

| Operation | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| owned addition | 9.9962 ns [9.9810, 10.016] | 9.7434 ns [9.7327, 9.7556] | -2.53% |
| borrowed addition | 8.5611 ns [8.5403, 8.5875] | 8.0325 ns [8.0247, 8.0413] | -6.17% |
| owned subtraction | 10.433 ns [10.410, 10.462] | 9.8744 ns [9.8361, 9.9282] | -5.35% |
| borrowed subtraction | 8.6349 ns [8.5893, 8.6874] | 8.3844 ns [8.3614, 8.4106] | -2.90% |

The result is not a size-for-speed exchange: one shared exact body improves
instruction locality on these retained-result workloads.

## Large-mesh controls

Fresh-process counters include fixture construction, exact import, PWN priming,
and one Boolean. The full YeahRight baseline was rebuilt from the exact parent
source before the current binary was rebuilt. The wide parent is the recorded
affine-edge-plane checkpoint. Task-clock samples vary more than the retired
work, so no wall-time win is claimed here.

| Fixture | Counter | Parent | Current | Movement |
| --- | --- | ---: | ---: | ---: |
| 23,788-triangle full rotated YeahRight intersection | instructions | 35,245,810,810 | 35,242,076,813 | -0.0106% |
| 23,788-triangle full rotated YeahRight intersection | branches | 5,965,419,063 | 5,965,405,877 | -0.0002% |
| 6,144-triangle 2,049-bit wide union | instructions | 4,195,402,389 | 4,195,442,185 | +0.0009% |
| 6,144-triangle 2,049-bit wide union | branches | 780,805,097 | 780,762,383 | -0.0055% |

The full case remains an exact empty certified result. The wide case remains a
certified 2,410-vertex, 4,816-triangle union. The movements are effectively
neutral and do not hide a benchmark-only shortcut.

## Direct large-mesh heap

The final allocator-instrumented runs are byte-for-byte and count-for-count
identical to the affine-edge-plane checkpoint under both policies:

| Fixture | Incremental peak | Allocation calls | Reallocations | Allocated-byte churn | Retained input facts | Output payload |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 2,049-bit wide union | 31,038,074 B | 2,109,647 | 228,217 | 463,318,390 B | 26,568 B | 520,472 B |
| full rotated YeahRight intersection | 158,258,204 B | 39,175,099 | 4,027,656 | 2,445,957,944 B | 24,389,848 B | 56 B |

Fixture preparation remains outside the kernel boundary. Output and input are
dropped separately. Both `STRICT` and `APPROXIMATE_512` produce these exact
values and `Certified` certainty.

## Rejected cleanups and measurement discipline

Three correct alternatives were measured and fully removed:

- routing the affine scheduling query through the broader fused dyadic
  product-sum helper preserved heap but increased 2,049-bit instructions
  0.177%, branches 0.164%, and native text about 480 bytes;
- centralizing generic multiplication saved linked code but slowed direct
  owned/borrowed exact multiplication about 6.11%/6.39%; and
- centralizing generic division slowed direct owned/borrowed exact division
  about 3.29%/6.77%.

An early addition comparison used a stale release test executable and was
discarded. Rebuilding both sides from their exact source produced the paired
full-mesh and scalar results above. No conclusion in this checkpoint depends
on the discarded comparison.

No multiplication/division wrapper, alternate arithmetic route, diagnostic
counter, fixture dispatch, Boolean special case, compatibility shim, or second
engine remains.

## Graph and validation

The regenerated five-crate graph contains 14,683 function nodes and 24,419
edges at
`/tmp/hypermesh-linear-operator-code-sharing-callgraph-2026-08-03`. It includes
the single Hyperreal addition and subtraction cores and excludes Hypercurve and
HyperSolve.

Hyperreal passes 649 all-feature unit tests plus every integration and
documentation target, the GMP public-API audit, no-default checking,
warning-denied all-target/all-feature Clippy, warning-denied rustdoc, formatting,
and diff checks.

Hypermesh passes 122 unit, 8 Boolean, 8 executed competitive, 11 manifest,
2 intersection, 9 policy, and 2 README tests: 162 executed tests with six
documented competitive ignores. It also passes no-default checking,
warning-denied all-target/all-feature Clippy, warning-denied rustdoc, every fuzz
target check, benchmark compilation, formatting, the full size matrix, the
ignored full YeahRight oracle, and both-policy large heap probes.

## Open work

This closes the specific linked growth introduced by the affine edge-plane
checkpoint in release consumers, but it does not close Phase 17 or Phase 18.
The 2,049-bit wide row remains about 4.44x pinned CGAL EPECK, the full fixture
still has a large historical CGAL runtime/RSS deficit, the corpus still needs
legally distributable external and deeper-symbolic pathologies, and every
remaining per-case performance, heap, source, size, and final path audit gate
remains open.

## Reproduction

```sh
cargo test --all-features
cargo check --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
cargo check --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --no-run --all-features

taskset -c 11 cargo bench --bench borrowed_ops -- '^rational_ops/(add|sub)'

taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/examples/large_mesh_heap_probe wide-rational-2048 strict

YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 5 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/deps/competitive-92c64513605410c9 \
  full_resolution_yeahright_rotated_intersection_certifies_empty \
  --ignored --exact --nocapture

taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 strict
taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 approximate-512
YEAHRIGHT_BENCH=1 taskset -c 11 \
  target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated strict
YEAHRIGHT_BENCH=1 taskset -c 11 \
  target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated approximate-512

benchmarks/size-harness/measure.sh

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-linear-operator-code-sharing-callgraph-2026-08-03 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library
```
