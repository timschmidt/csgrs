# Phase 17 checkpoint: borrowed coplanar contact candidates

Date: 2026-08-05

Implementation parent: Hypermesh `ba524a4f`

Implementation commit: Hypermesh `bf327f46`

Companion data: `phase17-borrowed-coplanar-contact-candidates.toml`

Status: exact ownership/lifetime, performance, heap, and size checkpoint;
Phases 17 and 18 remain open

## Outcome

The coplanar lower-dimensional-contact path no longer clones every contained
input `Point3` into pairwise scratch storage. Each candidate now borrows the
point already owned by the immutable polygon-vertex arena and carries only its
optional construction identity. Exact numeric deduplication, canonical recipe
merging, lexicographic extrema, and support-plane collinearity operate on those
references. Only the point or two endpoints that survive as the final contact
are materialized.

```text
polygon vertex arena
  -> borrowed coplanar candidates
  -> exact/policy-aware deduplication
  -> exact extrema and collinearity proof
  -> clone only the surviving point or segment endpoints
```

The scratch allocation remains reusable across the whole BVH candidate walk.
Rust ties its candidate lifetime to the owning `PolygonVertexArena`, so there
is no raw pointer, copied reference vector, index encoding, invalid-index
state, or new heap owner. A unit invariant requires the borrowed carrier to
remain smaller than the former owned constructed point.

There is no fixture, coordinate, size, operation, result, policy, competitor,
or benchmark dispatch. There is still one Boolean engine, no compatibility
shim, and no second shipped path.

## Exactness and policy proof

The geometric decision sequence is unchanged:

- containment uses the same cached point/edge classifications;
- deduplication first checks the same `Point3` equality and then calls the same
  policy-aware exact `points_equal` predicate;
- a duplicate keeps the first-discovered coordinate representation while the
  minimum construction identity is merged exactly as before;
- contacts with more than two candidates select the same exact
  lexicographic extrema and prove every candidate collinear on the support;
- materialization performs the same `Point3::clone`, but only after a point or
  endpoint has survived.

`STRICT` remains exact-only. `APPROXIMATE_512` can terminate only in
Hyperlimit's existing 512-bit terminal, and aggregate `MeshCertainty` is
unchanged. Every large policy pair below is byte-identical and `Certified`.

## Deterministic performance

The saved `ba524a4f` parent and final `bf327f46` release binaries ran on CPU 11
with identical retained inputs. Small rows are three-run means over 1,000
strict four-output arrangements; dense rows use three runs of the stated
iteration count. Retired instructions and branches are the authority because
desktop frequency and scheduling made clocks noisy.

| Fixture | Iterations | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| overlapping boxes | 1,000 | 3,549,850,102 | 3,548,941,767 | -0.026% | 602,721,979 | 603,167,665 | +0.074% |
| identical boxes | 1,000 | 3,573,466,501 | 3,551,115,357 | -0.625% | 604,479,880 | 600,821,160 | -0.605% |
| face-touching boxes | 1,000 | 2,182,339,226 | 2,163,285,018 | -0.873% | 359,331,270 | 356,053,794 | -0.912% |
| partial-face touching boxes | 1,000 | 3,131,659,913 | 3,122,910,996 | -0.279% | 525,721,029 | 524,349,329 | -0.261% |
| crossing octahedra | 1,000 | 2,837,050,911 | 2,838,112,392 | +0.037% | 480,242,877 | 480,832,187 | +0.123% |
| affine boxes | 1,000 | 6,835,092,228 | 6,833,928,979 | -0.017% | 1,161,648,155 | 1,163,165,749 | +0.131% |
| dense coplanar boxes 16 | 10 | 23,208,320,989 | 22,663,897,801 | -2.346% | 3,843,322,689 | 3,740,775,528 | -2.668% |
| dense crossing grid 17 | 1 | 13,242,826,876 | 13,241,932,958 | -0.007% | 1,842,337,833 | 1,841,935,970 | -0.022% |
| full rotated YeahRight | 1 | 9,763,090,884 | 9,760,497,475 | -0.027% | 1,647,207,214 | 1,647,193,053 | -0.001% |

The targeted dense-coplanar path improves materially. The full and dense
transverse controls also improve slightly. Crossing and affine show only
layout-scale branch/instruction movement (at most 0.131%); those small losses
are recorded rather than hidden. No clock sample, favorable subset, or corpus
identity is used to dispatch production work.

All exact outputs and certainty match the parent. The full row remains 14,626
vertices and `[33,512, 0, 16,756, 16,756]` triangles. Dense-coplanar-16 remains
3,074 vertices and `[6,144, 6,144, 0, 0]` triangles.

## Large-fixture heap

Fresh final probes cover three distinct large families under both policies.
The established full and output-dominated dense-crossing rows remain exactly
equal to the parent. An isolated build of adjacent parent `ba524a4f` against
the same current Hyper dependencies measures the ownership boundary directly
on dense-coplanar-32:

| Fixture | Policy | Output | Parent/current incremental peak | Parent/current allocations | Parent/current reallocations | Parent/current added bytes | Certainty |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| full rotated YeahRight, 23,788 triangles | STRICT | 0 triangles | 53,217,804 / 53,217,804 | 9,571,455 / 9,571,455 | 1,466,413 / 1,466,413 | 556,743,274 / 556,743,274 | Certified |
| full rotated YeahRight, 23,788 triangles | APPROXIMATE_512 | 0 triangles | 53,217,804 / 53,217,804 | 9,571,455 / 9,571,455 | 1,466,413 / 1,466,413 | 556,743,274 / 556,743,274 | Certified |
| dense crossing grid 65, 1,572 triangles | STRICT | 73,844 vertices / 164,068 triangles | 197,318,844 / 197,318,844 | 33,193,334 / 33,193,334 | 345,512 / 345,512 | 6,852,763,150 / 6,852,763,150 | Certified |
| dense crossing grid 65, 1,572 triangles | APPROXIMATE_512 | 73,844 vertices / 164,068 triangles | 197,318,844 / 197,318,844 | 33,193,334 / 33,193,334 | 345,512 / 345,512 | 6,852,763,150 / 6,852,763,150 | Certified |
| dense coplanar boxes 32, 24,576 triangles | STRICT | 12,290 vertices / 24,576 triangles | 55,759,202 / 55,758,386 | 926,776 / 926,776 | 123,014 / 123,014 | 220,619,719 / 220,618,903 | Certified |
| dense coplanar boxes 32, 24,576 triangles | APPROXIMATE_512 | 12,290 vertices / 24,576 triangles | 55,759,202 / 55,758,386 | 926,776 / 926,776 | 123,014 / 123,014 | 220,619,719 / 220,618,903 | Certified |

The diagnostic coplanar row removes exactly 816 bytes from both peak and
cumulative allocator payload without changing call counts. This is the one
reusable scratch allocation becoming smaller; it is deliberately not relabeled
as a broad 816-byte-per-pair saving.

## Source and linked size

The implementation changes one source file by 100 insertions and 83 deletions,
including lifetime and ownership tests. Hypermesh production source is 20,772
Tokei code lines. Six of eight canonical linked rows shrink; the two
release-profile optimized-WASM rows grow by less than one KiB.

| Features/profile | Parent native text | Current native text | Change | Parent optimized WASM | Current optimized WASM | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| default release | 2,037,234 | 2,035,442 | -0.088% | 1,455,385 | 1,456,174 | +0.054% |
| all-feature release | 2,174,507 | 2,172,395 | -0.097% | 1,531,562 | 1,532,484 | +0.060% |
| default size | 1,123,703 | 1,121,695 | -0.179% | 706,637 | 705,718 | -0.130% |
| all-feature size | 1,125,895 | 1,124,047 | -0.164% | 705,911 | 705,181 | -0.103% |

The performance-first decision retains the clean borrowed algorithm despite
the two sub-kilobyte release-WASM movements; native text and both size-profile
targets improve.

## Historical and competitive boundary

Historical EMBER remains 3,312.66 seconds / 329,352 KiB, and the pinned CGAL
6.0.3 EPECK boundaries are unchanged. The inherited approximately 10.1x full
runtime / 4.55x RSS gap and roughly 2.39x crossing, 2.73x overlapping, and
1.93x affine runtime gaps all remain open. This checkpoint improves a general
coplanar ownership boundary; it does not claim CGAL parity from a counter win.

## Validation and sanitizer

The implementation passes:

- all-feature Hypermesh tests: 163 library tests plus every integration and
  documentation test, for 217 executed tests and seven documented ignores;
- the permanent 59-record corpus manifest and every policy, Boolean,
  lower-dimensional contact, exact intersection, and dense public gate;
- no-default check, warning-denied all-target/all-feature Clippy,
  warning-denied rustdoc, formatting, diff, fuzz-workspace, and all-feature
  benchmark builds;
- AddressSanitizer/libFuzzer `boolean_pipeline`: 2,182 source seeds, 3,504
  executions in 31 seconds, 487 MiB fuzzer RSS, and no failure.

LeakSanitizer remained disabled with `ASAN_OPTIONS=detect_leaks=0` because the
managed environment prevents its final ptrace scan. The fuzzer used an
isolated corpus copy; it was removed, and the source corpus remains 2,182
files.

## Call graph and removal audit

The workspace utility scanned exactly Hyperreal, Hyperlattice, Hyperlimit,
Hypertri, and Hypermesh; Hypercurve and HyperSolve were excluded:

- production: 15,551 nodes / 25,997 edges;
- tests, examples, benches, and fuzz included: 21,954 nodes / 35,512 edges;
- 49 direct Hypermesh/Hypertri-to-Hyperlimit predicate boundaries;
- one `build_surface_arrangement -> assemble_surface_cells` edge;
- direct production edges from `intersect_coplanar_constructed` through
  borrowed deduplication to `materialize_coplanar_intersection`, with clone
  edges only from the materialization carrier;
- zero exact EMBER, `segment_trace`, `local_bsp`, or `SurfaceSheet` namespace
  nodes.

The production graph and main all-target graph completed. The optional last
per-library copy of the all-target graph hit the environment's temporary-file
quota after the main JSON had been written; the audited main graphs were then
removed after metrics were extracted. Static resolution remains navigation
and removal evidence, not a substitute for runtime, policy, corpus, or
sanitizer gates.

## Open work

Phases 17 and 18 remain open. Every current CGAL loss, external real-world and
generated pathology expansion, fuzz mutation-source audit, deeper
stage-lifetime/allocation reduction, source/native/WASM recovery, deferred
caller audit, and the final path/removal audit remain open.

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
  dense_coplanar_boxes_16 all strict 10
YEAHRIGHT_BENCH=1 taskset -c 11 perf stat -r 3 -x, \
  -e instructions:u,branches:u -- \
  target/release/examples/competitive_arrangement_probe \
  yeahright_full_resolution_rotated_intersection all strict 1

YEAHRIGHT_BENCH=1 target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated strict
target/release/examples/large_mesh_kernel_heap_probe dense-coplanar-32 strict
target/release/examples/large_mesh_kernel_heap_probe dense-crossing-65 strict
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```
