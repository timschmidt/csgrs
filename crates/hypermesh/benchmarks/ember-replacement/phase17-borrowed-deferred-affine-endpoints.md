# Phase 17 checkpoint: borrowed deferred affine endpoints

Date: 2026-08-05

Implementation parent: Hypermesh `fe7f143c`

Implementation commit: Hypermesh `ba524a4f`

Companion data: `phase17-borrowed-deferred-affine-endpoints.toml`

Status: exact ownership/lifetime, performance, heap, and size checkpoint;
Phases 17 and 18 remain open

## Outcome

The nonparallel polygon walker no longer clones every authored on-plane vertex
into its short-lived support-line interval. A deferred affine endpoint now
borrows the immutable `Point3` already owned by the pairwise polygon-vertex
arena. Exact interval comparison reads the borrowed coordinate, and only an
endpoint proved to survive the intersection is cloned into the constructed
pairwise graph.

This is the ownership model already used by deferred proper segment/plane
crossings. The same lifetime now covers both endpoint kinds:

```text
polygon vertex arena
  -> borrowed deferred affine endpoint
  -> exact/certified closed-interval clipping
  -> clone only surviving constructed endpoint
```

No predicate, interval rule, construction identity, discovery order, topology,
or output changes. There is no fixture, coordinate, count, operation, result,
policy, competitor, or benchmark dispatch, and no compatibility shim or second
engine.

## Lifetime and exactness proof

`PolygonVertexArena` owns every vertex slice for the complete pairwise BVH
walk. Each `DeferredIntersectionSpan` is created, compared, and materialized
inside one polygon-pair call, so its affine references cannot outlive the
arena. Rust expresses and checks that relationship through the existing
deferred-point lifetime.

The exact endpoint value is unchanged:

- interval enclosures are computed from the same `Real` coordinate;
- exact affine comparisons borrow that same coordinate;
- canonical construction identity and discovery-order merging are unchanged;
- a surviving endpoint executes the same `Point3::clone` at materialization;
- discarded endpoints execute no clone or refcount traffic.

`STRICT` remains exact-only. `APPROXIMATE_512` can terminate only in
Hyperlimit's existing 512-bit terminal, and aggregate `MeshCertainty` is
unchanged. Both large policy pairs below are exactly equal and `Certified`.

## Deterministic performance

The saved `fe7f143c` parent and final `ba524a4f` release binaries were run with
identical inputs on CPU 11. Small rows are three-run means over 1,000 strict
four-output arrangements. The dense and full rows are three- and five-run
means respectively. Retired instructions and branches are the authority;
concurrent desktop load made clocks and cycles unsuitable for a claim in this
checkpoint.

| Fixture | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 2,850,595,931 | 2,838,290,683 | -0.432% | 482,833,653 | 480,712,628 | -0.439% |
| overlapping boxes | 3,605,333,188 | 3,551,701,944 | -1.488% | 613,261,134 | 603,522,801 | -1.588% |
| affine boxes | 6,940,133,815 | 6,838,453,847 | -1.465% | 1,181,946,816 | 1,163,743,107 | -1.540% |
| identical boxes | 3,720,753,198 | 3,572,883,941 | -3.974% | 632,935,727 | 604,551,119 | -4.485% |
| dense crossing grid 17 | 13,249,299,882 | 13,242,321,048 | -0.053% | 1,843,590,244 | 1,841,987,544 | -0.087% |
| full rotated YeahRight | 9,844,090,158 | 9,763,075,952 | -0.823% | 1,662,998,078 | 1,647,310,473 | -0.943% |

Every row produces the same exact vertices, triangle lists, and `Certified`
certainty as its parent. The full four-output counts remain 14,626 vertices and
`[33,512, 0, 16,756, 16,756]` triangles; the exact intersection remains empty.

Historical EMBER's 3,312.66-second / 329,352-KiB result and the pinned CGAL
6.0.3 EPECK boundaries are unchanged. The preceding approximately 10.1x full
runtime / 4.55x RSS gap and roughly 2.39x crossing, 2.73x overlapping, and
1.93x affine runtime gaps all remain open. This checkpoint does not relabel a
counter win as CGAL parity.

## Large-fixture heap

Borrowing eliminates clone/refcount work but introduces no heap owner, so the
allocator boundary is expected to remain unchanged. Fresh probes confirm every
counter on the full input and output-dominated dense row:

| Fixture | Policy | Output | Incremental peak | Allocations | Reallocations | Added bytes | Certainty |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| full rotated YeahRight, 23,788 input triangles | STRICT | 0 triangles | 53,217,804 | 9,571,455 | 1,466,413 | 556,743,274 | Certified |
| full rotated YeahRight, 23,788 input triangles | APPROXIMATE_512 | 0 triangles | 53,217,804 | 9,571,455 | 1,466,413 | 556,743,274 | Certified |
| dense crossing grid 65, 1,572 input triangles | STRICT | 73,844 vertices / 164,068 triangles | 197,318,844 | 33,193,334 | 345,512 | 6,852,763,150 | Certified |
| dense crossing grid 65, 1,572 input triangles | APPROXIMATE_512 | 73,844 vertices / 164,068 triangles | 197,318,844 | 33,193,334 | 345,512 | 6,852,763,150 | Certified |

All rows are byte-identical to `fe7f143c`. The change reduces CPU work and
temporary `Real` reference traffic rather than allocator payload.

## Source and linked size

The implementation changes one source file by 31 insertions and 27 deletions,
including lifetime-adapted tests; current Hypermesh production source is 20,756
Tokei code lines. Every canonical linked row shrinks:

| Features/profile | Parent native text | Current native text | Change | Parent optimized WASM | Current optimized WASM | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| default release | 2,038,666 | 2,037,234 | -0.070% | 1,455,415 | 1,455,385 | -0.002% |
| all-feature release | 2,175,867 | 2,174,507 | -0.063% | 1,531,580 | 1,531,562 | -0.001% |
| default size | 1,123,807 | 1,123,703 | -0.009% | 706,837 | 706,637 | -0.028% |
| all-feature size | 1,125,999 | 1,125,895 | -0.009% | 706,120 | 705,911 | -0.030% |

## Validation and sanitizer

The final source passes:

- all-feature Hypermesh tests: 163 library tests plus every integration and
  documentation test, for 217 executed tests and seven documented ignores;
- the 59-record permanent corpus manifest and every exact intersection,
  policy, Boolean expression, lower-dimensional contact, and dense public gate;
- no-default check, warning-denied all-target/all-feature Clippy, warning-denied
  rustdoc, formatting, diff, fuzz-workspace, and all-feature benchmark builds;
- AddressSanitizer/libFuzzer `boolean_pipeline`: 2,182 source seeds copied to a
  temporary directory, 4,855 executions in 39 seconds, 480 MiB maximum RSS,
  and no failure.

`cargo-fuzz` also read its default ignored corpus, which duplicated those
seeds; neither source corpus count changed. LeakSanitizer remained disabled
with `ASAN_OPTIONS=detect_leaks=0` because the managed environment prevents its
final ptrace scan. The temporary corpus was removed after the run.

## Call graph and removal audit

The workspace utility scanned exactly Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, and Hypermesh; Hypercurve and HyperSolve were excluded:

- production: 15,547 nodes / 25,997 edges;
- tests, examples, benches, and fuzz included: 21,950 nodes / 35,512 edges;
- 49 direct Hypermesh/Hypertri-to-Hyperlimit predicate boundaries;
- one `build_surface_arrangement -> assemble_surface_cells` edge;
- direct production route
  `extend_polygon_plane_slice_edge -> affine_deferred_geometry`, followed by
  interval clipping and `materialize_deferred_point -> Point3::clone` only for
  survivors;
- zero exact EMBER, `segment_trace`, `local_bsp`, or `SurfaceSheet` namespace
  nodes.

Compact JSON artifacts were generated under `/tmp`. Static call-graph
resolution remains navigation/removal evidence, not a substitute for runtime,
policy, corpus, or sanitizer gates.

## Open work

Phases 17 and 18 remain open. Every current CGAL loss, external real-world and
generated pathology expansion, fuzz mutation-source coverage, deeper
stage-lifetime/allocation reduction, source/native/WASM recovery, deferred
callers, and the final path/removal audit remain open.

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
  overlapping_boxes all strict 1000
YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 5 -x, \
  -e instructions:u,branches:u -- \
  target/release/examples/competitive_arrangement_probe \
  yeahright_full_resolution_rotated_intersection all strict 1

YEAHRIGHT_BENCH=1 target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated strict
target/release/examples/large_mesh_kernel_heap_probe dense-crossing-65 strict
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```
