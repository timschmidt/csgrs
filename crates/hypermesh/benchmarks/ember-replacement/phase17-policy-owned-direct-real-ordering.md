# Phase 17: policy-owned direct Real ordering

Captured 2026-08-04. This checkpoint follows the accepted first-proof and
wide-dyadic scheduling evidence at Hyperreal `3582c1e6` and Hypermesh
`c74b77da`. Its production change is Hypertri `de12f48e`; Hyperreal,
Hyperlattice, Hyperlimit, and Hypermesh production code are unchanged.

## Result

Hypertri's exact kernel now delegates scalar ordering directly to Hyperlimit's
canonical `compare_reals` predicate. The former private implementation first
constructed `left - right`, queried local structural sign facts, and then
entered Hyperlimit's sign classifier. That was exact, but it discarded the
fact that both input scalars were already exact rationals and forced the
rational arithmetic core to allocate and reduce a difference solely to learn
its sign.

The canonical Hyperlimit ordering predicate already implements the complete
policy-aware schedule:

1. compare borrowed exact rationals directly;
2. use Hyperreal's certified structural and bounded-refinement ordering;
3. construct a difference only if those proof routes remain unknown; and
4. enter the existing `STRICT` or `APPROXIMATE_512` terminal policy cascade.

Hypertri now consumes that result through its existing operation-local
certainty ledger. Its redundant private Real-sign method, structural-fact
branch, sign mapper, and imports are deleted. Production source loses 32 net
lines. Thirty-eight focused test lines make the scheduling and policy contract
permanent, for a six-line net source increase including tests.

This is one general scalar-predicate route. It does not inspect a mesh,
fixture, coordinate width, point count, triangle count, topology, Boolean
operation, expected result, policy name, benchmark, or competitor. There is no
compatibility shim, alternate ordering engine, cache, retained field,
allocation, dependency, or work limit. All controlled callers moved directly.
Hypercurve and HyperSolve are excluded and untouched.

## Exactness and policy coverage

Exact-rational `Rational::partial_cmp` is a total exact ordering. It uses
pointer/sign/equal-denominator, dyadic, word, magnitude, leading-interval, and
arbitrary-precision cross-product schedules without constructing or reducing
the difference. A permanent dispatch regression proves that an exact
rational Hypertri comparison reaches `hyperlimit/compare_reals/exact-rational`
once and performs zero rational subtractions and zero rational GCDs.

The existing representation-distinct `pi + e` versus `e + pi` regression now
also checks both terminals through Hypertri's kernel. `APPROXIMATE_512`
decides equality only at Hyperlimit's 512-bit terminal and marks the operation
`Approximate512Consumed`. `STRICT` returns the typed `PredicateUndecided`
error and does not contaminate aggregate certainty. Exact-rational work stays
`Certified` under both policies. Full and wide large-fixture results, retained
facts, and heap counters are policy-identical.

## Paired deterministic work

Independent release targets compare Hypertri `41631f65` with `de12f48e`.
Both use Hyperreal `3582c1e6`, Hyperlimit `76281a34`, Hypermesh `c74b77da`, the
same compiler and lockfiles, CPU 11, and three repetitions. Instructions and
branches are the acceptance metrics; variable wall time is not.

| Workload | Parent instructions | Current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: | ---: |
| Full rotated YeahRight, 23,788 triangles | 14,312,067,954 | 13,723,276,881 | -4.11395% | -3.57778% |
| 2,049-bit rational boxes, five unions | 14,120,873,731 | 14,119,169,725 | -0.01207% | -0.01031% |
| Clipped voxel torus 33, three all-result arrangements | 3,133,376,683 | 3,126,387,839 | -0.22305% | -0.26250% |
| Ordinary overlapping boxes, 1,000 all-result arrangements | 5,680,886,628 | 5,474,831,546 | -3.62716% | -4.12295% |

Every output is identical. Unlike a mesh shortcut, the movement follows
general ordering pressure: the largest improvements occur where planar point
ordering repeatedly compares retained exact rationals, while the fixed-
topology wide row remains slightly positive.

## Large-fixture heap

The direct global-allocator probe excludes fixture construction from the
incremental Boolean boundary. Parent and current rows are exact across both
`STRICT` and `APPROXIMATE_512`.

| Fixture | Metric | Parent | Current | Movement |
| --- | --- | ---: | ---: | ---: |
| Full rotated YeahRight | incremental peak | 158,258,204 B | 158,258,204 B | equal |
| Full rotated YeahRight | allocations | 16,146,116 | 15,558,650 | -587,466 (-3.6384%) |
| Full rotated YeahRight | reallocations | 2,294,770 | 2,255,915 | -38,855 (-1.6932%) |
| Full rotated YeahRight | added bytes | 893,400,104 B | 863,068,648 B | -30,331,456 B (-3.3951%) |
| 2,049-bit rational boxes | incremental peak | 31,234,658 B | 31,234,658 B | equal |
| 2,049-bit rational boxes | allocations | 1,947,527 | 1,947,027 | -500 (-0.0257%) |
| 2,049-bit rational boxes | reallocations | 223,172 | 223,172 | equal |
| 2,049-bit rational boxes | added bytes | 438,862,678 B | 438,743,478 B | -119,200 B (-0.0272%) |

Full input payload, 24,389,848-byte input-fact growth, output payload,
post-drop residual, and peak are unchanged. Wide input payload, 26,568-byte
input-fact growth, output payload, post-drop residual, and peak are unchanged.
The savings are temporary exact difference objects and their reductions, not
retained storage.

## Source and linked size

The implementation deletes 32 net production lines and adds no field,
dependency, or API. Paired canonical size-harness values use the same compiler
and lockfiles; native values are `.text`, and WASM values are `wasm-opt -Oz`
bytes.

| Profile / consumer | Parent native | Current native | Parent WASM | Current WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 1,969,526 | 1,970,782 (+1,256; +0.0638%) | 1,394,092 | 1,394,510 (+418; +0.0300%) |
| release / immediate | 1,972,670 | 1,973,926 (+1,256; +0.0637%) | 1,395,935 | 1,396,354 (+419; +0.0300%) |
| size / general | 1,072,255 | 1,072,367 (+112; +0.0104%) | 668,390 | 668,271 (-119; -0.0178%) |
| size / immediate | 1,073,219 | 1,073,331 (+112; +0.0104%) | 668,796 | 668,678 (-118; -0.0176%) |

The small release growth is the linked `PredicateOutcome<Ordering>` consumer;
the old private source body is gone. It is accepted because performance has
priority, size-profile native growth is 112 bytes, optimized size-WASM
shrinks, and the broad work and allocation reductions are material.

## Dispatch and call graph

The current control YeahRight arrangement reaches
`hyperlimit/compare_reals/exact-rational` 10,252 times. The focused Hypertri
test isolates one such decision and proves zero `rational/sub` events and zero
GCDs. Other algorithms legitimately retain their own subtraction events; no
global counter is mistaken for the isolated invariant.

The regenerated five-crate source graph at
`/tmp/hypermesh-direct-real-order-callgraph-2026-08-04` contains 14,846
function nodes and 24,749 edges:

| Crate | Nodes | Edges |
| --- | ---: | ---: |
| Hyperreal | 7,256 | 12,496 |
| Hyperlattice | 1,370 | 2,560 |
| Hyperlimit | 1,938 | 2,982 |
| Hypertri | 1,376 | 2,010 |
| Hypermesh | 2,913 | 4,668 |

The direct edge is `hypertri::kernel::ExactKernel::cmp` to
`hyperlimit::compare_reals`; the deleted private `ExactKernel::real_sign`
node is absent. Hypercurve and HyperSolve are excluded. Production searches
still find no EMBER route or alternate Boolean engine.

## Rejected alternatives

Two clean general experiments preceded this selection and were removed fully:

- Expanding Hyperlimit's exact affine `orient2d` into the standard six
  original-coordinate products preserved every test but raised full
  instructions about 20.9% and branches about 18.6%.
- A Hyperreal arbitrary-width shared-denominator add/sub route improved the
  full row only about 0.003%, regressed the wide row 0.0016% instructions and
  0.0041% branches, and added 1,712 native text bytes.

Neither algorithm, assertion, counter, or branch remains. The accepted route
was selected from the post-wide-dyadic frame-pointer profile, where exact point
ordering was a repeated subtraction/GCD owner.

## Validation and competitive status

- Hypertri passes 75 all-feature unit tests, all 48 integration executions,
  four doctests, the default suite, warning-denied all-target Clippy and
  rustdoc, and no-default checking.
- Hypermesh passes 179 all-feature and 178 default executions; six documented
  manual/external tests remain ignored.
- Hypermesh warning-denied all-target Clippy and rustdoc, no-default checking,
  every fuzz binary, every benchmark target, formatting, and diff checks pass.
- Both large fixtures pass both policies with exactly equal outputs, certainty,
  peaks, retained facts, and post-drop residuals.

This checkpoint is gauged against the permanent historical/competitive ledger
without relabeling old measurements as current. The last pinned CGAL 6.0.3
EPECK comparison reports a 19.00x full-row runtime loss and 12.25x
fresh-process RSS loss, with ordinary-box losses of 4.81x under `STRICT` and
4.53x under `APPROXIMATE_512`. CGAL was not rerun for this predicate-routing
checkpoint, so no runtime or RSS ratio is inferred from deterministic
instruction work. Current CGAL confidence runs and every still-losing per-case
gate remain open.

## Reproduction

```sh
(cd ../hypertri && cargo test --locked --all-features && cargo test --locked)
cargo test --locked --all-features
cargo test --locked
(cd ../hypertri && cargo clippy --locked --all-targets --all-features -- -D warnings)
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run

YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u \
  target/release/examples/large_mesh_heap_probe \
  yeahright-full-rotated strict
taskset -c 11 perf stat -r 3 -x, -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  wide_rational_boxes_2048 union strict 5
target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 approximate-512
benchmarks/size-harness/measure.sh default

(cd .. && tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-direct-real-order-callgraph-2026-08-04 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library)
```
