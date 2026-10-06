# Phase 17 checkpoint: retained shared-vertex support slices

Date: 2026-08-07

Implementation parent: Hypermesh `43a2d815`

Implementation commit: Hypermesh `c2d4c17e`

Companion data: `phase17-retained-shared-vertex-support-slices.toml`

Status: clean exact pairwise-intersection scheduling checkpoint; Phases 17 and
18 remain open

## Outcome

Two authored source triangles from one operand can have exactly one shared
canonical source vertex. That retained identity, together with the identical
retained `Point3`, proves that the vertex lies on both authored support planes.
Hypermesh now uses that topology fact instead of asking the scalar layer to
rediscover both incidences.

For a noncoplanar pair, one pair-local optional schedule classifies only the
other four vertices against the opposite supports. A probe may return only a
`STRICT`-certified result. If either triangle has its shared vertex on the
opposite plane and its other two vertices on the same strict side, convexity
proves that the complete triangle/plane slice is exactly that vertex. Every
triangle/triangle intersection lies on both supports, so the complete closed
intersection is that already-authored vertex. The arrangement graph omits the
redundant intrinsic point event just as it already omits other retained source
features; the public closed-set intersection API does not omit the point.

When all four strict probes certify but neither slice is a singleton, their
classifications are passed into the ordinary nonparallel intersection rather
than recomputed. If any optional probe is unavailable, if the supports are
coplanar, if the triangles do not have exactly one shared source identity, or
if the retained geometry cannot certify the identity, the unchanged complete
path decides the pair. Contradictory exact-rational coordinates for one source
identity remain a typed `SurfaceArrangementFailed` error.

There is no fixture, coordinate, face-count, operation, result, policy-name,
competitor, or benchmark dispatch. No threshold, output simplifier, public API,
compatibility shim, alternate engine, repair path, or benchmark-specific case
was added. The small torus/coplanar costs below remain visible rather than
being selected around.

## Exactness and policy contract

The optional schedule calls Hyperlimit with `STRICT`, even when the enclosing
operation selected `APPROXIMATE_512`. It can therefore use an exact rational,
certified filter, retained scalar fact, or completed exact refinement, but it
cannot consume or imitate the approximate terminal. An unresolved probe
returns `Unavailable` without changing aggregate certainty. The ordinary
required predicate path then runs under the caller's selected policy.

The symbolic reordered-expression regression uses `pi + e` and `e + pi` to
exercise that distinction directly:

- the optional schedule declines under both policies and certainty remains
  `Certified`;
- the ordinary `STRICT` path returns `PredicateUndecided`;
- the ordinary `APPROXIMATE_512` path completes and records
  `Approximate512Consumed`.

An exhaustive 27-state classification test covers every three-vertex
negative/on/positive combination. Both policies separately verify that a true
shared-point intersection is omitted only from the internal arrangement
event graph, while a segment continuing away from the shared vertex follows
the complete construction path. Every measured rational fixture remains
policy-identical and `Certified`.

## Runtime profile and deterministic controls

The fixed parent is a fresh clone at `43a2d815`; current is `c2d4c17e`. Every
row is an adjacent CPU-11 three-run `perf stat` mean with the same release
toolchain. Small rows execute 1,000 complete four-result arrangements, dense
coplanar executes ten, and the other large rows execute one. Topology and
certainty are identical in every parent/current pair.

| Fixture | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| overlapping boxes | 3,516,959,338 | 3,271,763,858 | -6.972% | 596,221,672 | 555,727,505 | -6.792% |
| identical boxes | 3,532,865,848 | 3,310,554,376 | -6.293% | 598,201,126 | 562,383,446 | -5.988% |
| face-touching boxes | 2,147,316,240 | 1,918,449,652 | -10.658% | 354,043,965 | 316,755,888 | -10.532% |
| crossing octahedra | 2,799,110,443 | 2,682,175,152 | -4.178% | 473,255,977 | 454,319,279 | -4.001% |
| affine boxes | 6,782,242,119 | 6,553,597,999 | -3.371% | 1,153,021,976 | 1,115,132,073 | -3.286% |
| dense crossing grid 17 | 12,719,828,595 | 12,718,742,889 | -0.009% | 1,763,670,774 | 1,762,970,256 | -0.040% |
| dense coplanar boxes 16 | 22,186,691,558 | 22,232,318,037 | +0.206% | 3,686,365,324 | 3,697,046,804 | +0.290% |
| clipped voxel torus 33 | 777,266,221 | 778,046,216 | +0.100% | 133,448,183 | 133,682,342 | +0.176% |
| full rotated YeahRight | 9,038,986,337 | 7,304,899,113 | -19.185% | 1,533,823,982 | 1,220,651,290 | -20.418% |

The pinned counter-run wall median improves from 893,596,574 ns to
722,273,118 ns (-19.172%). Dense crossing is neutral. The +0.100--0.290%
torus/coplanar movements are retained controls for the structural equal-support
gate and the optional general schedule; no production branch selects around
them.

A temporary dispatch profile, removed before the size build, explains the full
gain. Full YeahRight reaches 194,888 eligible same-mesh shared-vertex pairs:
113,297 prove point-only (86,261 in the first direction and 27,036 in the
second), 9,559 retain all four non-point classifications, and 72,032 decline
to the ordinary route. Only 8,867 shared-input-feature checks remain after full
intersection construction. Torus 33 instead has 39,792 structurally equal
supports, 2,330 point proofs, 912 fully classified continuations, 1,834
declines, and 26,524 later shared-feature checks. The checked-in diagnostic
surface retains only the aggregate `pairwise-intersection/known-shared-vertex`
event.

## Historical and CGAL boundary

Historical EMBER required 3,312.66 seconds on full YeahRight. The fresh final
Hypermesh median is 0.681646973 seconds, about 4,859.8x faster.

Fresh paired CPU-11 trials use pinned CGAL 6.0.3 EPECK, exact-rational OFF
input, copies outside the CGAL timer, and the same four Boolean expressions.
Small trials use 1,000 internal iterations; large trials use one. Each median
below is the median of three fresh processes. Outputs are valid and closed.

| Fixture | Hypermesh median | CGAL median | Hypermesh / CGAL | Boundary |
| --- | ---: | ---: | ---: | --- |
| overlapping boxes | 330,834 ns | 120,642 ns | 2.742x | loss |
| crossing octahedra | 270,594 ns | 119,791 ns | 2.259x | loss |
| affine boxes | 667,682 ns | 356,295 ns | 1.874x | loss |
| dense coplanar boxes 16 | 192,447,506 ns | 295,256,853 ns | 0.652x | Hypermesh 1.534x faster |
| clipped voxel torus 33 | 54,642,431 ns | 6,303,078 ns | 8.669x | loss |
| full rotated YeahRight | 681,646,973 ns | 42,633,375 ns | 15.989x | loss |

The full row is deliberately not an equal-output-work shortcut. Hypermesh
path-completely corefines same-operand intersections and emits
33,512/0/16,756/16,756 triangles. The ordinary CGAL `Surface_mesh` operation
retains 23,788/0/11,894/11,894. Both report the same valid Boolean volumes, but
the larger Hypermesh result remains an open algorithm/output-work target. A
fresh process uses 70,236 KiB RSS for Hypermesh and 22,092 KiB for CGAL
(3.179x); the different corefinement work is part of that open boundary.

## Large-fixture heap

The requested-payload probe subtracts retained inputs and runs the permanent
large fixtures under both policies. Every policy pair is counter-identical and
`Certified`. The new schedule is allocation-free. Full benefits because the
point proof prevents the complete construction path; other large controls are
exactly unchanged.

| Fixture | Incremental peak | Parent -> current allocations | Parent -> current reallocations | Parent -> current added bytes | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| full rotated YeahRight | 53,217,804 | 8,681,722 -> 6,558,967 (-24.451%) | 1,296,791 -> 910,496 (-29.789%) | 500,833,490 -> 424,413,658 (-15.259%) | exact-empty selected result |
| dense crossing 65 | 197,318,844 | 31,166,606 -> 31,166,606 | 344,218 -> 344,218 | 6,631,418,894 -> 6,631,418,894 | 73,844 vertices / 164,068 triangles |
| dense coplanar 32 | 55,758,866 | 926,777 -> 926,777 | 123,014 -> 123,014 | 220,619,383 -> 220,619,383 | 12,290 vertices / 24,576 triangles |
| clipped voxel torus 65 | 47,367,618 | 98,161 -> 98,161 | 1,800 -> 1,800 | 64,455,376 -> 64,455,376 | 6,532 vertices / 13,060 triangles |
| transverse self-PWN 512 | 11,451,636 | 197,040 -> 197,040 | 13,655 -> 13,655 | 30,927,366 -> 30,927,366 | 5,124 vertices / 8,196 triangles |

## Source and linked size

The implementation changes three private production files by 451 insertions
and 24 deletions, including the exhaustive and policy regressions. Tokei Rust
code grows from 20,822 to 21,217 lines (+395, +1.90%). Runtime remains the
priority; canonical linked growth is bounded below 0.19%.

| Profile/features | Parent native text | Current native text | Change | Parent `wasm-opt -Oz` | Current `wasm-opt -Oz` | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| release/default | 2,024,754 | 2,027,794 | +3,040 (+0.150%) | 1,447,132 | 1,449,486 | +2,354 (+0.163%) |
| release/all | 2,159,827 | 2,162,955 | +3,128 (+0.145%) | 1,522,220 | 1,524,574 | +2,354 (+0.155%) |
| size/default | 1,113,951 | 1,115,391 | +1,440 (+0.129%) | 698,695 | 699,996 | +1,301 (+0.186%) |
| size/all | 1,116,527 | 1,118,183 | +1,656 (+0.148%) | 698,089 | 699,384 | +1,295 (+0.186%) |

## Validation and sanitizer

The final implementation passes:

- 220 executed all-feature Hypermesh tests with seven documented ignores;
- the 59-record fixture manifest and every exact intersection, policy,
  Boolean, lower-dimensional, symbolic, dense, and pathology gate;
- no-default checks, warning-denied all-target/all-feature Clippy,
  warning-denied rustdoc, formatting, diff, fuzz-workspace, and all-feature
  benchmark builds;
- AddressSanitizer/libFuzzer `boolean_pipeline`: all 2,182 permanent seeds
  copied to an isolated corpus, 3,294 executions in 31 seconds, 484 MiB fuzzer
  RSS, zero failure, and zero artifact.

The isolated mutating corpus grew to 2,214 files; the permanent source corpus
remained unchanged. LeakSanitizer stayed disabled with
`ASAN_OPTIONS=detect_leaks=0` because the managed environment prevents its
final ptrace scan. `/usr/bin/time` observed 799,252 KiB including compilation;
that is sanitizer process evidence, not the release heap boundary above.

## Call graph and removal audit

The workspace utility scanned exactly Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, and Hypermesh; Hypercurve and HyperSolve were excluded:

- production: 15,591 nodes / 26,096 edges;
- tests, examples, benches, and fuzz included: 21,994 / 35,611;
- 47 direct Hypermesh/Hypertri-to-Hyperlimit predicate boundaries, unchanged;
- one `build_surface_arrangement -> assemble_surface_cells` edge;
- direct canonical routes from pairwise construction to the retained
  shared-vertex schedule and from `probe_strict` to Hyperlimit's strict sign
  predicate;
- zero exact EMBER, `segment_trace`, `local_bsp`, or `SurfaceSheet` namespace
  nodes or production source matches.

Static resolution remains navigation and removal evidence, not a substitute
for the policy, corpus, heap, sanitizer, or runtime gates.

## Open work

Phases 17 and 18 remain open. The torus/equal-support overhead, exact face
crossing construction, output/corefinement work, every losing CGAL row, full
CGAL RSS gap, external real-world/generated corpus growth, fuzz mutation-source
coverage, deeper arrangement lifetime recovery, deferred callers, and the
final requirement audit all remain general targets.

## Reproduction

```sh
cargo test --locked --all-features
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --locked --no-run --all-features

taskset -c 11 perf stat -r 3 -x, -e instructions:u,branches:u -- \
  target/release/examples/competitive_arrangement_probe \
  dense_crossing_grid_17 all strict 1
YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u -- \
  target/release/examples/competitive_arrangement_probe \
  yeahright_full_resolution_rotated_intersection all strict 1

target/release/examples/large_mesh_kernel_heap_probe dense-crossing-65 strict
YEAHRIGHT_BENCH=1 target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated strict

benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```
