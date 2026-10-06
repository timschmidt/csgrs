# Phase 17 checkpoint: retained exact triangle orientations

Date: 2026-08-05

Implementation parent: Hypertri `210a66c`

Implementation commit: Hypertri `66d449d`

Hypermesh implementation: `fe13ddee` (evidence parent `a4f188c3`)

Companion data: `phase17-retained-triangle-orientations.toml`

Status: clean retained-topology performance and size checkpoint; Phases 17
and 18 remain open

## Outcome

Hypertri now carries an orientation already decided by its operation-local
exact kernel through two places that previously rebuilt the same determinant:

1. every active point-set triangle has positive winding by construction, so
   point insertion classifies a query against that retained positive fact;
2. cavity ear selection has just decided the candidate turn, so all contained
   vertex tests and the accepted ear reuse that same sign.

The general classifier still decides the triangle orientation when no fact is
available. Every query-edge orientation still goes through Hyperlimit's
complete certified/exact/policy cascade. The last cavity triangle, whose turn
has not just been evaluated, still takes the ordinary `make_oriented` route.

```text
unknown triangle
  -> Hyperlimit orient2d(a, b, c)
  -> orientation-aware containment

retained active triangle / just-proved ear
  -> retained exact-kernel sign
  -> three policy-aware edge orientations
  -> exact zero-incidence location
```

The common classifier now derives `OnVertex` from two zero edge incidences,
the same exact geometric rule used by Hyperlimit, instead of performing up to
three separate coordinate-equality queries before the edge predicates. This
removes duplicated work without changing the supported geometric domain.

There is no fixture, coordinate, mesh width, operation, result, policy,
competitor, or benchmark dispatch. No threshold, cache, allocation, alternate
triangulator, fallback engine, compatibility shim, or public API was added.

## Exactness and policy contract

The reused sign was already consumed by the same `ExactKernel`. If
`APPROXIMATE_512` terminated that decision, the operation's aggregate
`Approximate512Consumed` state was already recorded; reuse cannot relabel the
operation as certified. Under `STRICT`, only an exact decided sign reaches the
topology. All subsequent edge predicates preserve their existing scheduling,
exact fallbacks, terminal owner, and decision order.

Focused tests cover both windings, inside, outside, edge, vertex, and
degenerate locations under both policies. A dispatch-trace test proves that an
immediate classification performs four `orient2d` calls while the retained
route performs three. Hypertri's symbolic policy suite still proves typed
`STRICT` indeterminacy and `APPROXIMATE_512` aggregate consumption.

## Deterministic retained-work controls

The fixed parent is `target/phase17-coplanar-matrix-predicates`; the current
binary is `target/phase17-retained-triangle-orientations`. Each row is a fresh
adjacent CPU-11 three-run mean. Small rows execute 1,000 complete arrangements,
dense coplanar executes ten, and dense crossing/full execute one. Every
arrangement materializes union, intersection, difference, and reverse
difference together.

| Fixture | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| overlapping boxes | 3,543,729,180 | 3,517,699,570 | -0.735% | 602,404,687 | 596,775,129 | -0.935% |
| identical boxes | 3,533,526,546 | 3,532,991,007 | -0.015% | 598,547,262 | 598,411,168 | -0.023% |
| face-touching boxes | 2,147,156,249 | 2,145,614,944 | -0.072% | 354,013,684 | 353,630,585 | -0.108% |
| crossing octahedra | 2,834,352,511 | 2,807,013,483 | -0.965% | 480,125,090 | 474,561,626 | -1.159% |
| affine boxes | 6,820,884,590 | 6,784,443,639 | -0.534% | 1,161,371,028 | 1,153,394,148 | -0.687% |
| dense coplanar boxes 16 | 22,190,974,956 | 22,188,238,028 | -0.012% | 3,688,927,332 | 3,688,241,534 | -0.019% |
| dense crossing grid 17 | 13,240,461,285 | 12,802,307,270 | -3.309% | 1,841,668,495 | 1,777,400,464 | -3.490% |
| full rotated YeahRight | 9,750,751,336 | 9,690,099,658 | -0.622% | 1,645,481,283 | 1,634,640,063 | -0.659% |

The accepted-ear sign reuse is independently favorable on dense crossing:
relative to the orientation-aware containment prototype it removes another
0.059% instructions and 0.085% branches. The broad table has no losing
retained-work row.

## Historical and competitive boundary

Historical EMBER remains 3,312.66 seconds and 329,352 KiB on full YeahRight.
The fresh strict all-four-output median is 0.907417 seconds, about 3,651x
faster than EMBER.

Fresh paired CPU-11 runs use pinned CGAL 6.0.3 EPECK, exact OFF inputs, copies
outside the CGAL timer, and the same four Boolean expressions. Small CGAL rows
report the median of 1,000 iterations; dense/full rows execute one complete
sample per trial. Both engines return structurally valid closed outputs and
the same exact-volume results. Different valid output triangulations are not
treated as equal work.

| Fixture | Trial ratios, Hypermesh / CGAL | Median boundary |
| --- | --- | ---: |
| overlapping boxes | 3.086x / 3.006x / 2.988x | Hypermesh 3.006x slower |
| crossing octahedra | 2.464x / 2.426x / 2.358x | Hypermesh 2.426x slower |
| affine boxes | 1.953x / 1.959x / 1.923x | Hypermesh 1.953x slower |
| dense coplanar boxes 16 | 0.668x / 0.668x / 0.640x | Hypermesh 1.498x faster |
| full rotated YeahRight | 21.355x / 21.432x / 22.012x | Hypermesh 21.432x slower |

The full row is a newly refreshed all-four-output comparison, not the older
single-result 0.09-second boundary. CGAL retains the two authored shells for
union/differences (23,788/11,894 faces), while Hypermesh emits its exact
corefined arrangement (33,512/16,756 faces). This difference is exposed rather
than normalized away; output simplification and avoided unnecessary retained
work remain general algorithmic opportunities. Dense coplanar remains a real
win of the same production algorithm. Every losing row remains open.

## Large-fixture heap

The requested-payload probe uses the user's large fixtures and measures the
Boolean peak after subtracting retained input. Both policies are
counter-identical and `Certified`. Every byte, allocation, reallocation, and
cumulative-traffic counter is identical to the preceding checkpoint because
the change retains a stack sign rather than allocating evidence.

| Fixture | Incremental peak | Allocations | Reallocations | Added bytes | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| full rotated YeahRight, 23,788 triangles | 53,217,804 | 9,571,455 | 1,466,413 | 556,743,274 | exact-empty selected result |
| dense crossing 65, 1,572 triangles | 197,318,844 | 33,193,335 | 345,512 | 6,852,763,630 | 73,844 vertices / 164,068 triangles |
| dense coplanar 32, 24,576 triangles | 55,758,866 | 926,777 | 123,014 | 220,619,383 | 12,290 vertices / 24,576 triangles |
| clipped voxel torus 65, 25,100 triangles | 47,367,618 | 98,761 | 1,800 | 64,530,652 | 6,532 vertices / 13,060 triangles |

## Source and linked size

The Hypertri commit is 148 additions and 36 deletions including focused tests;
the full Hypertri source is 9,886 Rust code lines. Despite the explicit tests
and internal orientation-aware entry point, all eight canonical linked
native/WASM cells shrink relative to the preceding checkpoint.

| Profile/features | Parent native text | Current native text | Change | Parent `wasm-opt -Oz` | Current `wasm-opt -Oz` | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| release/default | 2,036,098 | 2,035,546 | -552 (-0.027%) | 1,456,724 | 1,456,241 | -483 (-0.033%) |
| release/all | 2,173,067 | 2,172,515 | -552 (-0.025%) | 1,532,991 | 1,532,516 | -475 (-0.031%) |
| size/default | 1,122,151 | 1,121,511 | -640 (-0.057%) | 706,193 | 705,597 | -596 (-0.084%) |
| size/all | 1,124,743 | 1,124,095 | -648 (-0.058%) | 705,597 | 705,001 | -596 (-0.084%) |

## Validation and sanitizer

The checkpoint passes:

- 139 executed Hypertri tests including four doctests;
- 217 executed all-feature Hypermesh tests with seven documented ignores;
- all 59 manifest records and the complete policy, intersection, Boolean,
  lower-dimensional, symbolic, dense, and pathology suites;
- default and no-default builds, warning-denied all-target/all-feature Clippy,
  warning-denied rustdoc, formatting, diff, fuzz-workspace, and benchmark
  builds;
- AddressSanitizer/libFuzzer `boolean_pipeline`: all 2,182 source seeds copied
  to an isolated corpus, 3,713 executions in 31 seconds, 491 MiB fuzzer RSS,
  and no failure.

LeakSanitizer remained disabled with `ASAN_OPTIONS=detect_leaks=0` because the
managed environment prevents its final ptrace scan. `/usr/bin/time` observed
724,904 KiB including compilation. The isolated corpus and artifact directory
were removed; the source corpus remains exactly 2,182 files.

## Call graph and removal audit

The workspace call-graph utility scanned exactly Hyperreal, Hyperlattice,
Hyperlimit, Hypertri, and Hypermesh; Hypercurve and HyperSolve were excluded:

- production: 15,568 nodes / 26,023 edges;
- tests, examples, benches, and fuzz included: 21,971 nodes / 35,538 edges;
- 49 direct Hypermesh/Hypertri-to-Hyperlimit predicate boundaries;
- one `build_surface_arrangement -> assemble_surface_cells` edge;
- direct production edges from point insertion to the retained-orientation
  classifier and from cavity triangulation to retained containment and
  `oriented_triangle`;
- zero exact EMBER, `segment_trace`, `local_bsp`, or `SurfaceSheet` namespace
  nodes.

The generated graphs were temporary and removed after the route audit.

## Open work

Phases 17 and 18 remain open. Every losing CGAL row, particularly the fresh
full all-output gap, output/corefinement work reduction, external real-world
and generated pathology expansion, fuzz mutation-source audit, the 480-byte
coplanar scratch cost, deeper lifetime/allocation recovery, deferred callers,
and the final requirement audit remain open.

## Reproduction

```sh
cargo test --locked --all-features
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --locked --no-run --all-features

taskset -c 11 perf stat -r 3 -x, -e instructions:u,branches:u -- \
  target/phase17-retained-triangle-orientations \
  dense_crossing_grid_17 all strict 1
YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u -- \
  target/phase17-retained-triangle-orientations \
  yeahright_full_resolution_rotated_intersection all strict 1

target/phase17-retained-triangle-orientations-heap dense-coplanar-32 strict
target/phase17-retained-triangle-orientations-heap dense-crossing-65 strict
YEAHRIGHT_BENCH=1 target/phase17-retained-triangle-orientations-heap \
  yeahright-full-rotated strict

benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```
