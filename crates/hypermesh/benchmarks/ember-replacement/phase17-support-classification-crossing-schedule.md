# Phase 17 support classification and crossing schedule

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11
single-thread protocol. The implementation is Hypermesh `42903e3e`,
`06d51a47`, and `7129b524` over the retained-crossing checkpoint
`e2bdff30`. The other participating heads remain Hyperreal `c2217bae`,
Hyperlattice `a475bb75`, Hyperlimit `6e4d68c8`, and Hypertri `3b436192`.

This checkpoint removes three forms of work from the one path-complete exact
surface-arrangement engine:

1. A nonparallel closed triangle pair is rejected as soon as either triangle
   is certified to lie in one strict open halfspace of the other support
   plane. All six classifications are retained when both triangles reach the
   opposite support, so the successful path makes exactly the decisions it
   previously made.
2. Support-plane parallelism classifies the three cross-product components
   lazily. Exact-rational components use Hyperreal's sign-only two-product
   difference scheduler; symbolic/general components retain the complete
   `Real` construction and Hyperlimit policy route.
3. An off-plane triangle with mixed signs has exactly two proper crossing
   edges. Its three exact support values are constructed once on the stack and
   borrowed by those two crossings instead of reconstructing their shared
   endpoint value.

These are geometric schedules, not benchmark cases. No rule examines a
fixture name, coordinate width, triangle count, Boolean operation, expected
output, certainty, or policy. The triangle schedule applies to the canonical
mesh primitive; the public arbitrary-convex-polygon route retains its complete
generic edge walk. No heap cache, retry, work limit, compatibility shim, or
second engine was introduced. Hypercurve and HyperSolve were not touched.

## Exactness and policy

The separating-support proof is exhaustive for a closed triangle: it reaches
a plane exactly when at least one vertex is on the plane or vertices occur on
both strict sides. A 27-pattern test checks every three-vertex classification
tuple. A second 27-pattern test proves that values are shared exactly when the
cycle contains two proper sign-changing edges.

`supports_are_parallel` still visits x, y, then z. A certified nonzero
component proves nonparallelism immediately. An undecided component is
remembered while later components may still prove nonparallelism; parallelism
is returned only when all three are certified zero. Generated exact-rational
differential tests compare 512 inputs with the former materialized value.
Symbolic tests prove that `STRICT` returns `PredicateUndecided`, while
`APPROXIMATE_512` resolves the same equality only through Hyperlimit's terminal
512-bit policy and aggregates `Approximate512Consumed`.

All measured mesh workloads are byte-identical between `STRICT` and
`APPROXIMATE_512`, remain `Certified`, and do not consume the approximate
terminal. Dispatch tracing reports zero unknown-fact events and zero
fallback-or-abort events.

## Full-resolution performance

Five fresh-process repetitions include fixture preparation, exact import,
input certification, one 23,788-triangle rotated YeahRight intersection,
result validation, and destruction. Runs are serialized and pinned to CPU 11.
The parent is the retained-crossing checkpoint `e2bdff30`.

| Metric | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Median test time | 2.18 s | 1.93 s | -11.47% |
| Cycles | 8,528,856,984 | 7,497,540,892 | -12.09% |
| Instructions | 22,237,885,130 | 19,222,742,051 | -13.56% |
| Branches | 3,988,513,749 | 3,433,388,544 | -13.92% |
| Cache misses | 20,080,741 | 19,858,900 | -1.10% |

The final stack sharing alone moves the immediately preceding sign-only head
from 19,358,787,386 to 19,222,742,051 instructions (-0.70%), and from a
1.96-second to a 1.93-second median. A candidate that materialized exact
rational cross-product components instead of returning only their signs raised
the full row to approximately 20.565 billion instructions (+6.23%); it was
fully removed. This is why the retained implementation plays to Hyperreal's
sign scheduler instead of flattening the algorithm into eagerly normalized
rationals.

The exact result remains empty and `Certified`. The current row is about
1,716.4x faster than historical EMBER's 3,312.66 seconds, but is still 21.44x
slower than the established 0.09-second CGAL EPECK row. The full competitive
runtime gate remains open.

## Large-fixture heap and RSS

The direct global-allocator boundary excludes fixture preparation, keeps both
input meshes alive through the Boolean, and drops output and inputs
separately. Both policies have identical bytes, calls, topology, and
certainty.

| Metric | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Incremental kernel peak | 158,258,204 B | 158,258,204 B | unchanged |
| Allocation calls | 26,006,589 | 19,290,062 | -25.83% |
| Reallocation calls | 2,525,282 | 2,133,982 | -15.50% |
| Allocated-byte churn | 1,279,258,296 B | 1,017,109,024 B | -20.49% |
| Input-attached fact growth | 24,389,848 B | 24,389,848 B | unchanged |
| Output-live payload | 56 B | 56 B | unchanged |
| Post-input residual | 10,792 B | 10,792 B | unchanged |

The current deallocation count is 18,937,421 and removed bytes are
992,719,120. The immediately preceding sign-only head already removed most
temporary construction; stack sharing then removes another 1.32% of allocation
calls, 3.90% of reallocations, and 0.90% of allocated bytes. The unchanged
158.26 MB peak makes the live-memory owner, rather than transient call count,
an explicit remaining target.

A fresh `/usr/bin/time -v` process reached 189,656 KiB RSS with zero swaps,
still 12.22x the established 15,516 KiB CGAL row. The 2,049-bit,
6,144-triangle wide-rational control remains byte-identical under both
policies: 31,038,074 peak bytes, 2,107,571 allocations, 227,677
reallocations, 462,626,870 allocated bytes, 26,568 input-fact bytes, 520,472
output bytes, and 96,192 post-input residual bytes.

## Independent controls and CGAL

The independent controls compare with `e2bdff30` and show that the schedules
are general rather than a one-fixture trade:

| Workload | Parent instructions | Current instructions | Movement |
| --- | ---: | ---: | ---: |
| 2,049-bit wide rational, 6,144 triangles | 4,190,199,389 | 4,160,089,259 | -0.72% |
| Clipped voxel torus, 6,412 triangles | 1,246,596,747 | 1,216,283,938 | -2.43% |
| Clipped voxel torus, 25,100 triangles | 5,334,327,874 | 5,220,285,535 | -2.14% |
| Dense coplanar, 6,144 triangles | 3,268,905,691 | 3,205,690,332 | -1.93% |

The final exact-box Criterion control estimates 627.85 microseconds for
`STRICT` and 631.93 microseconds for `APPROXIMATE_512`. Against pinned CGAL's
119.5965/128.976-microsecond copy-outside/copy-inside rows, the directional
ratios remain approximately 5.25x/4.90x, so the small-case gate stays open.

Three independent 31-call medium dense-coplanar aggregates at the sign-only
head measured 14.639013 ms for `STRICT` and 14.667867 ms for
`APPROXIMATE_512`, versus pinned CGAL's 16.036 ms. The final stack-sharing
change does not execute on coplanar support pairs; this member therefore
retains its approximately 8.7% runtime win.

## Linked size

The sign-only exact-rational ordering instantiates a small additional generic
body. The performance-first decision costs less than one half percent in every
native artifact and less than 0.58% in optimized WASM relative to `e2bdff30`.
Stack sharing then adds only 120 native release text bytes while removing 433
optimized release-WASM bytes.

| Profile / consumer | Parent native | Current native | Parent WASM | Current WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 2,019,526 B | 2,028,558 B | 1,437,843 B | 1,446,143 B |
| release / immediate | 2,022,670 B | 2,031,702 B | 1,439,697 B | 1,447,997 B |
| size / general | 1,082,351 B | 1,086,399 B | 679,625 B | 682,891 B |
| size / immediate | 1,083,315 B | 1,087,355 B | 680,032 B | 683,301 B |

Maximum growth is 0.58%. It remains a recovery target, but the 13.56%
full-case instruction and 20.49% allocation-churn reductions govern because
performance has priority over size.

## Profile and call graph

The frame-pointer profile at
`/tmp/hypermesh-support-scheduling-fp.data` contains 1,981 samples with none
lost. Inclusive ownership is led by surface construction (84.08%),
corefinement (41.11%), pairwise intersection (36.19%), BVH self-pair traversal
(35.76%), pair append (30.43%), polygon intersection (28.06%), constrained
triangulation (20.10%), edge-plane crossing collection (19.32%), and one edge
crossing (16.28%). Hyperreal rational GCD scheduling is 11.98% inclusive and
the exact 8-by-3 ordering body is 0.67% self. The next work must therefore
consider complete corefinement/CDT and scalar retained-fact ownership, not add
a benchmark-shaped rejection rule.

The regenerated five-crate call graph has 14,685 function nodes and 24,433
edges at
`/tmp/hypermesh-support-separators-sign-only-shared-values-callgraph-2026-08-03`.
Hypercurve and HyperSolve are excluded.

## Validation and open work

Hypermesh passes 167 executed tests with six documented ignores, all-feature
and no-default builds, warning-denied Clippy and rustdoc, fuzz-target checks,
benchmark compilation, the full dispatch trace, the ignored full-resolution
oracle, both-policy full and wide direct heap probes, and all eight canonical
native/WASM size rows. The corpus retains exact lower-dimensional,
coplanar/noncoplanar, symbolic-policy, arbitrary-rational-width, dense,
high-genus, and full-resolution paths.

Phase 17 and Phase 18 remain open. In particular, full and small-case CGAL
runtime, full RSS and live peak heap, exact wide-rational absolute losses,
external real-world/deeper-symbolic fixture expansion, stage-specific lifetime
attribution, broad current CGAL confidence runs, and the final path/removal
audit are not closed by this checkpoint.

## Reproduction

```sh
cargo test --all-features
cargo check --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
cargo check --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --no-run --all-features

YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 5 -x, \
  -e cycles,instructions,branches,cache-misses \
  target/release/deps/competitive-92c64513605410c9 \
  --ignored --exact full_resolution_yeahright_rotated_intersection_certifies_empty

YEAHRIGHT_BENCH=1 target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated strict
YEAHRIGHT_BENCH=1 target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated approximate-512
target/release/examples/large_mesh_kernel_heap_probe wide-rational-2048 strict
target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 approximate-512

YEAHRIGHT_BENCH=1 cargo bench --bench dispatch_trace --features dispatch-trace
benchmarks/size-harness/measure.sh

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir \
  /tmp/hypermesh-support-separators-sign-only-shared-values-callgraph-2026-08-03 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library
```
