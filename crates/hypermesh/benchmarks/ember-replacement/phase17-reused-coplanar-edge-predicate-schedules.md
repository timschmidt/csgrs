# Phase 17 checkpoint: reused coplanar edge predicate schedules

Date: 2026-08-05

Implementation parent: Hypermesh `bf327f46`

Implementation commit: Hypermesh `fe13ddee`

Companion data: `phase17-reused-coplanar-edge-predicate-schedules.toml`

Status: exact retained-fact scheduling, performance, heap, and size
checkpoint; Phases 17 and 18 remain open

## Outcome

Coplanar separating-axis and lower-dimensional-contact classification now
prepare each demanded edge-plane predicate once per polygon pair. The existing
classification matrix owns one lazy slot per edge. Its first uncached matrix
cell retains the borrowed plane, four borrowed exact-rational coefficients,
and Hyperreal's certified rational linear-form filter; later cells on that row
reuse the same complete schedule.

```text
first demanded edge/vertex cell
  -> operation-local filter cache lookup
  -> borrowed exact coefficient/filter schedule
  -> later vertices on the same edge reuse it
  -> unchanged exact-rational or policy-owned Real fallback
```

Slots are initialized to `None`, so a separating-axis short circuit does not
prepare an edge that the former walk would not have reached. The slots share
the existing pairwise scratch lifetime and capacity across the complete BVH
candidate stream. They are cleared between pairs and borrow planes from the
immutable polygon arena; no raw pointer, global face table, invalid index,
fixture threshold, or second Boolean path is introduced.

This is deliberately pair-local. A previously measured global per-face index
schedule added retained state and lost ordinary/full controls, while a two-slot
recent-predicate cache thrashed across triangle edge cycles; neither is
present. A compact 48-byte predicate carrier was also measured here and fully
removed: repeating exact coefficient extraction cost 0.423% instructions and
0.716% branches on dense-coplanar-16 relative to the retained 80-byte carrier.
Performance remains more important than the 192 bytes that carrier would have
saved for the common six-edge pair.

There is no fixture, coordinate, triangle-count, operation, result, policy
name, competitor, or benchmark dispatch. There remains one exact surface
arrangement engine and no compatibility shim.

## Exactness and policy proof

The matrix stores a predicate schedule, never a geometric classification. Its
classification cache, vertex queries, traversal order, and short-circuit order
are unchanged. `PointPlanePredicate` continues to use this cascade:

1. a certified retained rational filter, when available and conclusive;
2. the complete exact-rational signed product sum;
3. the existing general `Real` expression and Hyperlimit decision.

An unavailable filter or non-rational plane therefore reaches exactly the
same complete fallback. Retaining a certified filter cannot consume a policy
terminal. `STRICT` remains exact-only; only Hyperlimit may terminate
`APPROXIMATE_512`, and `DecisionContext` still aggregates
`Approximate512Consumed`. Direct unit coverage proves that all six schedules
are reused for coincident triangles, that a separated pair leaves unvisited
slots empty after scratch reset, and that approximate cached exclusion still
cannot outrank retained vertex identity.

Every measured large output is byte-identical across policies and remains
`Certified`.

## Deterministic performance

The saved `bf327f46` parent and final `fe13ddee` release binaries ran on CPU 11
with identical retained inputs. Small rows are three-run means over 1,000
strict four-output arrangements. Dense rows and full YeahRight use three runs
of the stated iteration count. Retired instructions and branches are the
authority; exact outputs and certainty are unchanged.

| Fixture | Iterations | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| overlapping boxes | 1,000 | 3,548,794,296 | 3,543,684,549 | -0.144% | 602,794,836 | 602,396,741 | -0.066% |
| identical boxes | 1,000 | 3,550,305,807 | 3,532,477,612 | -0.502% | 600,505,923 | 598,284,985 | -0.370% |
| face-touching boxes | 1,000 | 2,163,035,546 | 2,146,565,988 | -0.761% | 356,036,370 | 353,864,711 | -0.610% |
| partial-face touching boxes | 1,000 | 3,123,059,969 | 3,112,477,806 | -0.339% | 524,299,732 | 523,176,340 | -0.214% |
| crossing octahedra | 1,000 | 2,835,950,707 | 2,835,394,781 | -0.020% | 480,131,023 | 480,397,181 | +0.055% |
| affine boxes | 1,000 | 6,832,033,424 | 6,820,501,076 | -0.169% | 1,161,522,080 | 1,161,276,616 | -0.021% |
| dense coplanar boxes 16 | 10 | 22,659,464,167 | 22,187,390,474 | -2.083% | 3,740,724,544 | 3,688,029,651 | -1.409% |
| dense crossing grid 17 | 1 | 13,242,559,599 | 13,240,447,428 | -0.016% | 1,842,317,289 | 1,841,664,574 | -0.035% |
| full rotated YeahRight | 1 | 9,760,593,124 | 9,750,726,825 | -0.101% | 1,647,109,958 | 1,645,475,244 | -0.099% |

Eight of nine rows improve both counters. Crossing octahedra improves
instructions and pays a bounded +0.055% branch movement. No favorable clock
sample or workload subset is used to retain the change.

On one dispatch-traced dense-coplanar-16 arrangement, exact classifications,
filter successes, point-query facts, and the 474 first filter constructions
are identical. Repeated filter-cache hits fall 1,211,490 to 760,176, removing
451,314 lookups (37.253%). The schedule changes repeated fact acquisition, not
predicate work or topology.

## Historical and competitive boundary

Historical EMBER remains 3,312.66 seconds and 329,352 KiB on full YeahRight.
The current full timed scope has a 0.916-second median, about 3,618x faster,
while the inherited approximately 10.1x runtime and 4.55x RSS gaps to the
pinned full CGAL EPECK run remain open.

Fresh retained-input CPU-11 trials compare the same exact inputs and four
Boolean outputs with pinned CGAL 6.0.3 EPECK. Each CGAL trial reports the
median of 1,000 iterations with copies outside its timer; Hypermesh reports
one 1,000-iteration aggregate. All outputs are closed, structurally valid, and
agree in topology and exact volume.

| Fixture | Trial ratios, Hypermesh / CGAL | Median boundary |
| --- | --- | ---: |
| overlapping boxes | 2.943x / 2.977x / 3.011x | Hypermesh 2.977x slower |
| crossing octahedra | 2.503x / 2.483x / 2.480x | Hypermesh 2.483x slower |
| affine boxes | 1.943x / 1.942x / 1.962x | Hypermesh 1.943x slower |
| dense coplanar boxes 16 | 0.666x / 0.680x / 0.657x | Hypermesh 1.502x faster |

The dense coplanar family is a genuine current win for the same general
algorithm, not a parity claim for other cases. All three ordinary losses and
the full-case CGAL boundary remain open.

## Large-fixture heap

Fresh requested-payload probes use large fixtures and isolate prepared input,
incremental Boolean peak, output ownership, allocation calls, reallocations,
and cumulative bytes. The one new allocation is the reusable six-edge
predicate-slot vector; it is 480 bytes and is allocated only when a coplanar
pair reaches the matrix.

| Fixture | Policies | Parent/current incremental peak | Parent/current allocations | Reallocations | Parent/current added bytes | Result |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| full rotated YeahRight, 23,788 triangles | both | 53,217,804 / 53,217,804 | 9,571,455 / 9,571,455 | 1,466,413 | 556,743,274 / 556,743,274 | exact empty, `Certified` |
| dense crossing 65, 1,572 triangles | both | 197,318,844 / 197,318,844 | 33,193,334 / 33,193,335 | 345,512 | 6,852,763,150 / 6,852,763,630 | 73,844 vertices / 164,068 triangles, `Certified` |
| dense coplanar 32, 24,576 triangles | both | 55,758,386 / 55,758,866 | 926,776 / 926,777 | 123,014 | 220,618,903 / 220,619,383 | 12,290 vertices / 24,576 triangles, `Certified` |
| clipped voxel torus 65, 25,100 triangles | both | 47,367,138 / 47,367,618 | 98,760 / 98,761 | 1,800 | 64,530,172 / 64,530,652 | 6,532 vertices / 13,060 triangles, `Certified` |

The full path never enters coplanar matrix classification and stays
counter-identical. Dense crossing's output dominates peak, so only calls and
traffic expose the slot vector. The two coplanar/pathological rows expose the
exact +480-byte peak. No per-pair allocation occurs after the first reserve.

## Source and linked size

The implementation changes one production file by 73 insertions and 23
deletions, including tests, bringing Hypermesh production source to 20,822
Tokei code lines. The eight canonical native/WASM rows grow by 416--696 bytes
(at most 0.067%).

| Features/profile | Parent native text | Current native text | Change | Parent optimized WASM | Current optimized WASM | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| default release | 2,035,442 | 2,036,098 | +0.032% | 1,456,174 | 1,456,724 | +0.038% |
| all-feature release | 2,172,395 | 2,173,067 | +0.031% | 1,532,484 | 1,532,991 | +0.033% |
| default size | 1,121,695 | 1,122,151 | +0.041% | 705,718 | 706,193 | +0.067% |
| all-feature size | 1,124,047 | 1,124,743 | +0.062% | 705,181 | 705,597 | +0.059% |

The bounded footprint increase is retained because the general hot path wins
all instruction controls, materially improves the intended coplanar family,
and uses the larger carrier to avoid repeated exact-fact extraction.

## Validation and sanitizer

The implementation passes:

- all-feature Hypermesh tests: 163 library tests plus every integration and
  documentation test, for 217 executed tests and seven documented ignores;
- the permanent 59-record corpus manifest and every policy, Boolean,
  lower-dimensional contact, exact intersection, dense, symbolic, and
  pathology gate;
- no-default check, warning-denied all-target/all-feature Clippy,
  warning-denied rustdoc, formatting, diff, fuzz-workspace, and all-feature
  benchmark builds;
- AddressSanitizer/libFuzzer `boolean_pipeline`: all 2,182 source seeds copied
  to an isolated corpus, 3,727 executions in 31 seconds, 493 MiB fuzzer RSS,
  and no failure.

LeakSanitizer remained disabled with `ASAN_OPTIONS=detect_leaks=0` because the
managed environment prevents its final ptrace scan. `/usr/bin/time` observed
724,108 KiB including compilation. The temporary corpus was removed and the
source corpus remains exactly 2,182 files.

## Call graph and removal audit

The workspace utility scanned exactly Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, and Hypermesh; Hypercurve and HyperSolve were excluded:

- production: 15,558 nodes / 26,005 edges;
- tests, examples, benches, and fuzz included: 21,961 nodes / 35,520 edges;
- 49 direct Hypermesh/Hypertri-to-Hyperlimit predicate boundaries;
- one `build_surface_arrangement -> assemble_surface_cells` edge;
- direct matrix edges to lazy `PointPlanePredicate::new` and reused
  `plane_predicate.classify` inside the one coplanar relation path;
- zero exact EMBER, `segment_trace`, `local_bsp`, or `SurfaceSheet` namespace
  nodes.

Static resolution remains navigation and removal evidence, not a substitute
for runtime, policy, corpus, or sanitizer gates.

## Open work

Phases 17 and 18 remain open. Every losing CGAL row, external real-world and
generated pathology expansion, fuzz mutation-source audit, broader
arrangement/corefinement lifetime reduction, the 480-byte coplanar scratch
cost, source/native/WASM recovery, deferred caller audit, and final release and
removal matrix remain open.

## Reproduction

```sh
cargo test --locked --all-features
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --locked --no-run --all-features

taskset -c 11 perf stat -r 3 -x, -e instructions:u,branches:u -- \
  target/phase17-coplanar-matrix-predicates \
  dense_coplanar_boxes_16 all strict 10
YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u -- \
  target/phase17-coplanar-matrix-predicates \
  yeahright_full_resolution_rotated_intersection all strict 1

target/phase17-coplanar-matrix-predicates-heap dense-coplanar-32 strict
target/phase17-coplanar-matrix-predicates-heap voxel-torus-65 strict
target/phase17-coplanar-matrix-predicates-heap dense-crossing-65 strict
YEAHRIGHT_BENCH=1 target/phase17-coplanar-matrix-predicates-heap \
  yeahright-full-rotated strict

benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```
