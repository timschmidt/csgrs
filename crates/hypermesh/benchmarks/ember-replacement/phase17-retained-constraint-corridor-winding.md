# Phase 17 checkpoint: retained constraint-corridor winding

Date: 2026-08-05

Implementation parent: Hypertri `66d449d`

Implementation commit: Hypertri `86189ff`

Hypermesh implementation: `fe13ddee` (evidence parent `df541ce9`)

Companion data: `phase17-retained-constraint-corridor-winding.toml`

Status: clean retained-topology performance, heap, and size checkpoint; Phases
17 and 18 remain open

## Outcome

Hypertri's exact constraint-corridor walk already classifies every advancing
vertex against the directed protected segment and appends it to exactly one
ordered half-hole:

- the left chain contains the strictly positive-side vertices and runs from
  the constraint start to its end; closing it along the constraint therefore
  has negative winding;
- the right chain contains the strictly negative-side vertices in the same
  endpoint order; closing it has positive winding.

The chains are exact topology facts, not coordinate heuristics. Protected-edge
backtracking can make a chain weakly simple, but each such spike contributes
zero signed area. Hole absorption removes only boundary detours and preserves
the remaining vertices' decided side and order. Hypertri now carries these two
winding signs into cavity ear selection instead of cloning every point into a
second ring and evaluating one global exact area polynomial.

Every local turn and every candidate containment test still enters the normal
exact-kernel predicate path. The last triangle still uses `make_oriented`, so a
degenerate terminal triangle is not accepted from the retained sign alone.
Straight-chain pruning still occurs before any ear is emitted, and an invalid
sub-three-vertex side still returns the same typed `NoEarFound` failure.

There is no fixture, coordinate, triangle-count, operation, result, policy,
competitor, or benchmark dispatch. No threshold, cache, allocation, public
API, compatibility shim, alternate triangulator, repair retry, or second mesh
engine was added. The change is one private proof handoff inside the standard
constraint-recovery algorithm.

## Exactness and policy contract

The side classifications that establish each chain are decided by the same
operation-local `ExactKernel`. Under `STRICT`, only exact classifications can
reach the corridor topology. If `APPROXIMATE_512` consumes its Hyperlimit-owned
terminal while deciding a side, the kernel has already recorded aggregate
`Approximate512Consumed` before the winding is reused. Removing a logically
redundant area query cannot conceal a consumed terminal.

All later ear turns, point/triangle relations, and the terminal triangle check
retain their existing policy-aware paths. Hyperlimit remains the only owner of
the approximate-512 terminal. All measured exact-rational fixtures remain
byte-identical between policies and report `Certified`.

## Profile-led selection

A five-iteration CPU-11 frame-pointer profile of the fixed parent attributed
43.54% of cycles to face corefinement and 21.10% to bounded Hypertri CDT. The
whole-ring cavity-area decision alone accounted for 5.52% of the full process.
Pairwise intersection was the other major stage at 38.23%.

The post-change profile contains no cavity-to-ring-area route. Face
corefinement falls to 40.54%, bounded CDT to 16.33%, and cavity retriangulation
to 2.18%. Pairwise intersection becomes the next largest general target at
40.52%; exact face line-intersection construction is 9.20%. Percentages are
sampling attribution, while the deterministic counters below are the
acceptance authority.

## Deterministic retained-work controls

The fixed parent is `target/phase17-retained-triangle-orientations`; the
current binary is `target/phase17-retained-constraint-corridor-winding`. Each
row is a fresh adjacent CPU-11 three-run mean. Small rows execute 1,000 complete
four-output arrangements, dense coplanar executes ten, and dense crossing/full
execute one.

| Fixture | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| overlapping boxes | 3,517,455,078 | 3,519,281,450 | +0.052% | 596,708,457 | 597,153,903 | +0.075% |
| identical boxes | 3,532,434,079 | 3,534,005,748 | +0.044% | 598,275,232 | 598,652,861 | +0.063% |
| face-touching boxes | 2,148,040,901 | 2,146,455,277 | -0.074% | 354,228,276 | 353,835,664 | -0.111% |
| crossing octahedra | 2,806,339,945 | 2,798,458,156 | -0.281% | 474,394,192 | 472,996,985 | -0.295% |
| affine boxes | 6,784,178,235 | 6,785,564,627 | +0.020% | 1,153,323,218 | 1,153,674,327 | +0.030% |
| dense coplanar boxes 16 | 22,199,013,599 | 22,187,066,532 | -0.054% | 3,690,935,287 | 3,687,946,631 | -0.081% |
| dense crossing grid 17 | 12,802,297,753 | 12,719,800,223 | -0.644% | 1,777,398,393 | 1,763,085,912 | -0.805% |
| full rotated YeahRight | 9,690,120,638 | 9,038,830,121 | -6.721% | 1,634,645,555 | 1,533,971,657 | -6.159% |

The full-case three-run wall median improves from 0.913422 to 0.847787 seconds
(-7.186%). The three +0.020--0.075% movements are retained as explicit
layout-scale controls: they are far smaller than the general dense/full wins,
and no production branch is allowed to select around them.

## Historical and competitive boundary

Historical EMBER remains 3,312.66 seconds on full YeahRight. The fresh current
all-four-output median is 0.848828 seconds, about 3,902.6x faster.

Fresh paired CPU-11 trials use pinned CGAL 6.0.3 EPECK, exact rational OFF
inputs, copies outside the CGAL timer, and the same four Boolean expressions.
Small CGAL trials report the median of 1,000 internal iterations; dense/full
trials execute one complete operation. Both engines validate the same exact
volumes and closed outputs. Different valid triangulations remain visible.

| Fixture | Trial ratios, Hypermesh / CGAL | Median boundary |
| --- | --- | ---: |
| overlapping boxes | 2.987x / 3.092x / 3.008x | Hypermesh 3.008x slower |
| crossing octahedra | 2.406x / 2.407x / 2.408x | Hypermesh 2.407x slower |
| affine boxes | 1.957x / 1.974x / 1.927x | Hypermesh 1.957x slower |
| dense coplanar boxes 16 | 0.645x / 0.658x / 0.669x | Hypermesh 1.521x faster |
| full rotated YeahRight | 20.096x / 20.697x / 20.374x | Hypermesh 20.374x slower |

The full row improves from the preceding 21.432x loss but remains the largest
competitive deficit. CGAL retains 23,788/11,894 faces for union/differences;
Hypermesh retains 33,512/16,756 exact corefined faces. No simplifier or
benchmark-specific bypass hides that general output-work difference.

## Large-fixture heap

The requested-payload probe uses the permanent large fixtures and subtracts
retained input from the Boolean peak. Both policies are counter-identical and
`Certified`. Peak bytes stay unchanged because the output topology still owns
the maximum, while removal of the temporary predicate ring materially reduces
allocation traffic on cavity-heavy cases.

| Fixture | Incremental peak | Parent -> current allocations | Parent -> current reallocations | Parent -> current added bytes | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| full rotated YeahRight | 53,217,804 | 9,571,455 -> 8,681,722 (-9.296%) | 1,466,413 -> 1,296,791 (-11.567%) | 556,743,274 -> 500,833,490 (-10.042%) | exact-empty selected result |
| dense crossing 65 | 197,318,844 | 33,193,335 -> 31,166,606 (-6.106%) | 345,512 -> 344,218 (-0.375%) | 6,852,763,630 -> 6,631,418,894 (-3.230%) | 73,844 vertices / 164,068 triangles |
| dense coplanar 32 | 55,758,866 | 926,777 -> 926,777 | 123,014 -> 123,014 | 220,619,383 -> 220,619,383 | 12,290 vertices / 24,576 triangles |
| clipped voxel torus 65 | 47,367,618 | 98,761 -> 98,161 (-0.608%) | 1,800 -> 1,800 | 64,530,652 -> 64,455,376 (-0.117%) | 6,532 vertices / 13,060 triangles |

## Source and linked size

The Hypertri implementation commit changes one private file by 25 insertions
and 17 deletions; Hypertri contains 9,888 Tokei Rust code lines. Every canonical
consumer shrinks substantially because the cavity-only ring construction and
its monomorphized exact-area path disappear.

| Profile/features | Parent native text | Current native text | Change | Parent `wasm-opt -Oz` | Current `wasm-opt -Oz` | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| release/default | 2,035,546 | 2,024,754 | -10,792 (-0.530%) | 1,456,241 | 1,447,132 | -9,109 (-0.626%) |
| release/all | 2,172,515 | 2,159,827 | -12,688 (-0.584%) | 1,532,516 | 1,522,220 | -10,296 (-0.672%) |
| size/default | 1,121,511 | 1,113,951 | -7,560 (-0.674%) | 705,597 | 698,695 | -6,902 (-0.978%) |
| size/all | 1,124,095 | 1,116,527 | -7,568 (-0.673%) | 705,001 | 698,089 | -6,912 (-0.980%) |

## Validation and sanitizer

The checkpoint passes:

- 139 executed Hypertri tests, including four doctests, under all features;
- 217 executed all-feature Hypermesh tests with seven documented ignores, and
  162 default-feature library tests;
- the 59-record manifest plus every exact intersection, policy, Boolean,
  lower-dimensional, symbolic, dense, and pathology gate;
- no-default checks, warning-denied all-target/all-feature Clippy,
  warning-denied rustdoc, formatting, diff, fuzz-workspace, and all-feature
  benchmark builds;
- AddressSanitizer/libFuzzer `boolean_pipeline`: all 2,182 source seeds copied
  to an isolated corpus, 3,641 executions in 31 seconds, 497 MiB fuzzer RSS,
  and no failure or artifact.

LeakSanitizer remained disabled with `ASAN_OPTIONS=detect_leaks=0` because the
managed environment prevents its final ptrace scan. `/usr/bin/time` observed
714,644 KiB including compilation. The permanent source corpus remains exactly
2,182 files. The shared `/tmp` quota caused one rust-lld `SIGBUS`; the identical
four doctests passed immediately with `TMPDIR` under the repository target.

## Call graph and removal audit

The workspace utility scanned exactly Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, and Hypermesh; Hypercurve and HyperSolve were excluded:

- production: 15,565 nodes / 26,017 edges;
- tests, examples, benches, and fuzz included: 21,968 nodes / 35,532 edges;
- 47 direct Hypermesh/Hypertri-to-Hyperlimit predicate boundaries, down from
  49 because the duplicate cavity ring construction and area query are gone;
- one `build_surface_arrangement -> assemble_surface_cells` edge;
- a direct `triangulate_cavity_region -> triangulate_cavity_side` route, then
  only local exact turns, retained-orientation containment, `oriented_triangle`,
  and the terminal `make_oriented` check;
- no Hypertri cavity-to-ring-area edge and zero exact EMBER, `segment_trace`,
  `local_bsp`, or `SurfaceSheet` namespace nodes.

Static resolution remains navigation and removal evidence, not a substitute
for the runtime, policy, corpus, heap, or sanitizer gates.

## Open work

Phases 17 and 18 remain open. Pairwise intersection and exact face crossing
construction are the next measured general hot paths. Every losing CGAL row,
the larger full corefined output, external real-world/generated corpus work,
fuzz mutation-source coverage, deeper arrangement lifetime recovery, deferred
callers, and the final requirement audit remain open.

## Reproduction

```sh
cargo test --locked --all-features
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --locked --no-run --all-features

taskset -c 11 perf stat -r 3 -x, -e instructions:u,branches:u -- \
  target/phase17-retained-constraint-corridor-winding \
  dense_crossing_grid_17 all strict 1
YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u -- \
  target/phase17-retained-constraint-corridor-winding \
  yeahright_full_resolution_rotated_intersection all strict 1

target/phase17-retained-constraint-corridor-winding-heap \
  dense-crossing-65 strict
YEAHRIGHT_BENCH=1 target/phase17-retained-constraint-corridor-winding-heap \
  yeahright-full-rotated strict

benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```
