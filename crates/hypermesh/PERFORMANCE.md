# Paper-driven performance audit

This file records optimization hypotheses derived from the papers in the
README, including rejected ideas and the evidence required to retain a change.
All three papers in the README reference list are mapped to the relevant
production paths below.

The guiding safety boundary comes from EMBER's exact plane representation and
adaptive spatial subdivision, together with Mesh Arrangements' separation of
arrangement construction from winding-number extraction: approximate data may
organize work, but it must never decide topology, classification, or output.

## Public-path trace coverage

Coverage is audited by executable public family. Constructors, enums, report
accessors, and borrowed views are validated through the public operation that
produces or consumes them; they do not receive misleading standalone timing
rows. Every exact-computation family below has semantic assertions, a release
timing, and a recording window that fails on an empty dispatch trace.

| Public executable family | Semantic tests | Release benchmark | Exact path trace |
| --- | --- | --- | --- |
| Input meshes, polygon soups, and preparation | `core`, `regression` | `end_to_end/prepare_input` | `mesh_prepare_input` plus the dispatch-trace integration test |
| Primitives, clipping, intersections, BVHs, and local BSP | `core`, `regression` | exercised inside `end_to_end` Boolean and hull workloads | `polygon_clip_intersection_bvh_bsp` |
| Boolean arrangement construction, scoped/certified-convex preparation, extraction, and all operations | `core`, `regression` | `end_to_end/boolean_operation` and arrangement/crossover groups | every operation over overlapping, nested, variadic, and subdivided inputs plus `prepared_certified_convex_and_output_views` |
| Subdivision, leaf processing, segment tracing, and winding propagation | `core`, `regression` | exercised inside `end_to_end` Boolean workloads | Boolean recordings plus `segment_and_winding` |
| Certified output extraction, triangulation, and closure reports | `core`, `regression` | `end_to_end/output` | Boolean recordings include output-closure certification |
| Convex hull, coplanar groups, and retained construction facts | `core`, `regression` | all `end_to_end/convex_hull` cases, including both retained variants | `convex_hull/grid_4913` and `convex_hull_public_variants` |

`cargo bench --features dispatch-trace --bench dispatch_trace` records the
exact-computation paths for every Boolean operation across overlapping,
contained, variadic, and subdivided inputs, plus convex hull construction. It
also contains direct public-module workloads for mesh preparation; polygon
construction, clipping, and intersection; exact BVH queries; local BSP
splitting; axis/general segment tracing; and winding propagation and output
classification. Every recording window fails if it emits neither dispatch nor
rational-reducer evidence.

These trace workloads complement the Criterion timings in `end_to_end` and the
unit/integration tests: the Criterion suite measures retained end-to-end costs,
the dispatch benchmark identifies the selected exact paths, and the tests lock
their semantic results and failure behavior.

## 2026-07-15: cache approximate BVH partition keys

Status: **kept**

References considered:

- Trettner, Nehring-Wirxel, and Kobbelt, *EMBER: Exact Mesh Booleans via
  Efficient & Robust Local Arrangements*.
- Zhou, Grinspun, Zorin, and Jacobson, *Mesh Arrangements for Solid Geometry*.

Hypothesis: `BoundsBvh` repeatedly converted exact coordinates to `f64` inside
median-partition comparators. The conversions only choose BVH partitions; exact
AABB comparisons and exact point/plane predicates certify every rejection and
reported candidate. Precomputing the three approximate keys per item should
therefore remove repeated work without weakening the exact computation model.

Implementation:

- Precompute polygon-bound centers once before recursive BVH construction.
- Precompute point coordinates once before recursive point-BVH construction.
- Share the convex hull's existing approximate point table with its point BVH.
- Keep exact bounds, exact longest-axis selection, and exact query predicates
  unchanged.

Benchmark evidence was collected with Criterion on an AMD Ryzen 7 5800X3D,
Rust 1.97.0, using the `bench` profile:

| Workload | Before | Cached BVH keys | Shared hull/BVH keys |
| --- | ---: | ---: | ---: |
| `convex_hull/grid_4913` | 13.383 ms | 7.747 ms | 7.399 ms |

The final result is 44.7% faster than the same-turn baseline. Criterion also
reported improvements against its stored baseline for
`convex_hull/moment_curve_64` (53.8%) and `convex_hull/curved_shell_684`
(26.9%). `boolean_operation/cubes/Union`, the Boolean workload with a current
comparable baseline, was statistically unchanged.

Trace evidence from `cargo bench --bench dispatch_trace --features
dispatch-trace` for `convex_hull/grid_4913`:

- 11 output vertices and 18 output triangles, matching the benchmark workload.
- 327,498 exact-rational `compare-real` dispatches.
- 385 certified predicate events.
- zero unknown-fact events and zero fallback/abort events.

Validation:

- `cargo test --all-targets`: 950 unit, 52 core, and 48 regression tests passed;
  one benchmark-style regression test remained intentionally ignored.
- `cargo clippy --all-targets -- -D warnings` passed.
- `RUSTDOCFLAGS="-D warnings" cargo doc --no-deps` passed.
- `cargo check --examples --benches` passed.

## 2026-07-15: bound leaf-cache lookup to completed passes

Status: **kept**

Reference considered:

- Trettner, Nehring-Wirxel, and Kobbelt, *EMBER: Exact Mesh Booleans via
  Efficient & Robust Local Arrangements*, especially its profile showing leaf
  classification as the largest cost center and its emphasis on cheap local
  fast paths.

Hypothesis: the leaf classification caches preserve useful results across
failed subdivision attempts and certified child reuse, but tracing showed no
reuse among classifications appended during the same leaf-processing pass.
Every new polygon therefore scanned an increasingly long suffix that could not
produce a valid hit. Snapshotting the cache lengths at the beginning of a pass
should preserve cross-pass reuse while avoiding the quadratic same-pass scan.

Implementation:

- Snapshot both leaf-cache lengths before processing a leaf.
- Search only the completed-pass prefix while continuing to append current-pass
  results for later retries or child reuse.
- Trace hits, misses, and skipped same-pass scans separately.
- Add focused tests proving that same-pass entries are skipped and completed-
  pass entries remain reusable.

Benchmark evidence was collected with Criterion on the same machine and
profile as the BVH experiment:

| Workload | Before | Bounded lookup | Result |
| --- | ---: | ---: | ---: |
| `boolean_operation/nested_tools_5/Difference` | 20.818 ms | 20.493 ms | 1.56% faster |
| `boolean_operation/cubes/Union` | about 8.04 ms | 8.050 ms | statistically unchanged |
| `boolean_operation/octahedra/Union` | 10.146 ms stored baseline | 10.060 ms | within noise threshold |

The new variadic benchmark subtracts five disjoint nested cubes from one host
cube, producing 72 classified polygons. Its trace recorded 72 misses and 71
skipped same-pass scans in each of the polygon and interior-point caches, with
zero hits. The existing cube, nested-cube, and octahedron traces showed the same
all-miss pattern. The optimization does not remove the cache: entries from
earlier passes are still searched and the focused tests cover that reuse.

## 2026-07-15: EMBER subdivision experiments rejected or architecture-inapplicable

Status: **no production changes retained**

The following hypotheses were implemented and measured, then removed:

- Moving the complete winding-reachability early-out ahead of root split-basis
  preparation regressed `boolean_operation/nested_cubes/Difference` from
  5.626 ms to 5.759 ms (2.37% slower). The more expensive reachability check is
  correctly left after the cheaper preparation and contraction path.
- A constant-time absent-transition-component early-out never fired in any
  representative trace and added measurable overhead, so it was removed.
- A dedicated upper-triangular BVH self-pair traversal changed the variadic
  workload from 20.818 ms to 20.906 ms and cube union from about 8.04 ms to
  about 8.15 ms. It was removed.
- Returning unsorted internal BVH intersection candidates was statistically
  neutral and was removed to retain deterministic ordering.

Instrumentation retained from this audit records subdivision tasks, bound
contractions, winding-reachability discards, completed leaves, and split
searches. Every current Boolean trace performs two task entries, one bound
contraction, and one completed leaf, with no split search or reachability
discard. Consequently, EMBER's split heuristics and parallel work scheduling
were additionally probed with exact-rational closed tetrahedra using certified
face-, edge-, and vertex-reference normalization. Each again completed after a
bound contraction and one leaf, with zero split searches. Closed-cube boundary
references that lacked the required adjacent-cell winding certificate failed
explicitly during reference propagation before subdivision, as required by the
public correctness contract.

This differs from EMBER's cost model: the paper subdivides until a local
polygon threshold is reached, whereas this implementation first attempts its
complete local BSP and exact segment-trace classifier even for the 384-polygon
stress leaf. Recursive splitting is therefore a certification fallback, not a
size-driven production hot path. Split ranking, branch work stealing, and
parallel cache sharing have no correctness-certified workload under the
current API on which they can be meaningfully timed. They are classified as
architecture-inapplicable until the leaf policy or supported input contract
changes, rather than left as an unbounded optimization task.

Validation after the first two retained optimizations:

- `cargo test --all-targets`: 952 unit and 52 core tests passed; 48 regression
  tests passed and one benchmark-style regression remained intentionally
  ignored; all benchmark smoke executions passed.
- `cargo clippy --all-targets -- -D warnings` passed.
- `RUSTDOCFLAGS="-D warnings" cargo doc --no-deps` passed.
- `cargo check --examples --benches` passed.
- `cargo fmt --all -- --check` and `git diff --check` passed.
- Dispatch traces remained certified with zero unknown-fact events. The
  convex-hull trace had zero fallback/abort events; Boolean traces had only the
  expected guarded division-by-zero probes and completed successfully.

## 2026-07-15: reuse triangle support classifications during intersection

Status: **kept**

Reference considered:

- Zhou, Grinspun, Zorin, and Jacobson, *Mesh Arrangements for Solid
  Geometry*, especially its performance profile identifying exact
  triangle-triangle intersection detection and construction as the dominant
  cost of arrangement construction.

The existing implementation already follows two central rules from the paper:
it constructs each unordered candidate pair once and reverses that certified
result for the other polygon, and it retains exact pairwise results in the
subdivision runtime cache. The audit therefore focused inside the remaining
exact construction.

Hypothesis: each edge of a triangular polygon classified both endpoints
against the opposing support plane, so each vertex was classified twice. A
specialized triangle path can classify all three vertices once and reuse those
results for the three edge tests without changing any predicate, construction,
or topology decision. The generic convex-polygon path remains unchanged.

Same-turn Criterion evidence:

| Workload | Before | Triangle classification reuse | Result |
| --- | ---: | ---: | ---: |
| `boolean_operation/cubes/Union` | 8.1043 ms | 7.9284 ms | 2.17% faster |
| `boolean_operation/octahedra/Union` | 10.101 ms | 9.8733 ms | 2.25% faster |
| `boolean_operation/nested_tools_5/Difference` | 20.780 ms | 20.776 ms | unchanged |

Dispatch traces explain the improvement while preserving certified behavior:

- Each operation records one pairwise-cache miss followed by one exact-order
  hit, confirming that root analysis and leaf processing share the constructed
  arrangement intersections.
- Cube union affine exact point/plane classifications fell from 14,170 to
  12,838 across 132 BVH candidate pairs and 30 non-empty cuts.
- Octahedron union classifications fell from 17,345 to 15,509 across 83
  candidates and 36 non-empty cuts.
- The variadic difference classifications fell from 31,742 to 29,150 across
  324 candidates and 72 non-empty cuts, though the end-to-end time was
  unchanged because intersection predicates are a smaller part of that case.
- All traces retained zero unknown-fact events and the same output polygon
  counts.

Two broader variants were rejected before retaining the narrow triangle path:

- Precomputing classifications into temporary vectors regressed the variadic
  workload by 1.36%; allocation outweighed predicate reuse.
- A rolling classification loop avoided allocation but regressed the variadic
  workload from 20.780 ms to 21.398 ms in a repeat run. The generic loop was
  restored, confining reuse to the common fixed-size triangle case where it is
  measurably beneficial.

## 2026-07-15: Generalized Winding Numbers applicability audit

Status: **no production optimization applicable to the current contract**

Reference considered:

- Jacobson, Kavan, and Sorkine-Hornung, *Robust Inside-Outside Segmentation
  Using Generalized Winding Numbers*.

The paper evaluates a real-valued solid-angle field as an inside/outside
confidence measure for open, self-intersecting, non-manifold, or otherwise
imperfect oriented meshes. It accelerates repeated field evaluations with an
AABB hierarchy whose nodes close local open patches; outside a node's convex
hull, the winding contribution of the patch is the negative contribution of
its usually smaller closure. Construction stops at about 100 facets or when a
closure is no smaller than its source. The paper then uses graph-cut
segmentation because a simple threshold is not reliable for imperfect input.

`hypermesh` has a stricter and materially different contract:

- Inputs must certify as PWN meshes; open or ambiguous inputs are rejected.
- Classification propagates exact integer winding transitions along certified
  paths. It never thresholds a floating solid-angle confidence value.
- Boundary contacts produce an explicit unknown result and trigger alternate
  probes or subdivision rather than being assigned by a numerical threshold.
- Exact per-polygon bounds already cull segment-trace crossings, and the
resulting front/back winding vectors are retained with output polygons.

Replacing that path with generalized solid-angle evaluation or graph-cut
labels would weaken exact Boolean topology guarantees, so it was not attempted.
The paper's exact closure hierarchy also does not fit the measured workload:
representative traces contain only 24–72 leaf polygons, below its approximate
100-facet stopping threshold. Its applicability was tested separately on a
much larger certified leaf below; any compatible design must accelerate the
same exact transition semantics rather than introduce confidence thresholds.

Final validation after the intersection change repeated the full matrix: 952
unit tests and 52 core tests passed; 48 regression tests passed with the one
benchmark-style regression intentionally ignored; benchmark smoke execution,
Clippy with warnings denied, rustdoc with warnings denied, and example/bench
compilation all passed.

## 2026-07-15: large-leaf exact trace hierarchy rejected

Status: **benchmark retained; production experiment removed**

The earlier hierarchy audit lacked a representative leaf above the generalized
winding-number paper's roughly 100-facet stopping scale. A new certified
fixture recursively tessellates each face of two overlapping closed cubes,
producing 192 triangles per mesh without changing their geometry. Its union
completes as one exact leaf with 384 classification misses and 276 output
polygons. The trace records 4,032 pairwise BVH candidates, 168 nonempty cuts,
and 2,783,428 exact comparisons, so it is a genuine large-leaf stress case.

An experiment built one shared `ExactBvh` in the leaf probe cache and queried
each axis segment's exact AABB before running the unchanged exact tracer on the
reported candidates. The hierarchy remained scheduling-only: exact node bounds
certified every rejection and every candidate still entered the existing exact
plane/edge predicates. Nevertheless, BVH construction, exact node traversal,
candidate collection, and temporary polygon cloning outweighed the avoided
linear checks:

| workload | linear scan | exact trace BVH | result |
| --- | ---: | ---: | ---: |
| `boolean_operation/subdivided_cubes_192/Union` | 312.36 ms | 445.86 ms | 42.7% slower |

All production hierarchy plumbing was removed. The benchmark and dispatch
fixture remain so a future zero-copy query integration can prove a different
cost model rather than extrapolating from the smaller cube cases.

## 2026-07-15: unified scoped arrangement extraction

Status: **kept**

Reference considered:

- Zhou, Grinspun, Zorin, and Jacobson, *Mesh Arrangements for Solid
  Geometry*, especially its two-stage separation between operation-independent
  arrangement construction and winding-vector extraction.

Hypothesis: clients requesting several Boolean results for the same inputs
should not repeat input preparation, pairwise intersections, local BSP
construction, and exact winding classification for every operator.

Implementation:

- Add `prepare_boolean_operations`, which accepts the exact operation set to
  retain. A singleton set preserves operation-specific winding-reachability
  pruning and selected-transition emission; a multi-operation set retains the
  winding transitions required for every requested extraction.
- Make `boolean_operation` a compatibility wrapper over singleton preparation
  and extraction, eliminating its separate general-operation pipeline.
- Keep `build_boolean_arrangement` as the all-four-operation convenience API.
- Add `BooleanArrangement::extract`, which applies union, intersection,
  difference, or symmetric-difference indicators to the retained winding
  evidence and closure-certifies each result.
- Preserve coincident fragments with distinct winding evidence inside the
  arrangement; ordinary operation output keeps its existing geometric
  deduplication.
- Reject extraction of operations outside the prepared set instead of silently
  claiming evidence that was not retained.

Criterion crossover evidence for two overlapping cubes:

| requested operations | scoped direct calls | scoped preparation plus extraction | extraction from prebuilt all-operation arrangement |
| ---: | ---: | ---: | ---: |
| 1 | 8.061 ms | 8.373 ms | 0.190 ms |
| 2 | 16.129 ms | 8.333 ms | 0.273 ms |
| 3 | 24.442 ms | 8.425 ms | 0.424 ms |
| 4 | 32.445 ms | 8.711 ms | 0.751 ms |

Singleton preparation and `boolean_operation` execute the same scoped path;
their small measured difference is benchmark-order noise. Preparation crosses
over decisively at two requested results, while a prebuilt arrangement makes
every retained extraction sub-millisecond on this fixture.

Focused tests compare every extracted operation byte-for-byte with the direct
result for both overlapping and exactly coincident cube pairs. The coincident
case specifically guards retention of distinct winding evidence.

Final validation passed 952 unit tests, 58 core integration tests, 48
regressions (with one benchmark-style smoke test intentionally ignored), and
all doctests. Formatting, all-target/all-feature checking, Clippy with warnings
denied, rustdoc with warnings denied, and the no-default-feature test matrix
also passed.

## 2026-07-18: retain rational point filter intervals

Status: **kept**

Sampled call stacks for the 192-triangle-per-mesh subdivided-cube union showed
that exact point/plane classification was the largest named function and that
rational magnitude detection plus conversion to conservative `f64` intervals
accounted for another substantial share. Dispatch tracing recorded 211,430
prepared affine points feeding 265,416 exact-rational classifications: the
same point was commonly tested against a support plane and several edge
planes, but its three coordinates and homogeneous weight were converted again
for every filter attempt.

`hyperreal` now exposes a hidden prepared rational linear-form query carrying
four approximate values and their certified error radii. `PreparedPoint3`
constructs the affine form once, uses an exact `1.0` homogeneous weight with
zero error, and reuses it across every plane classification. Inconclusive
filters still execute the same exact rational signed-product-sum ordering.

Matched release A/B runs used identical code and settings except for retaining
the prepared query:

| workload | repeated conversion | retained query | result |
| --- | ---: | ---: | ---: |
| `boolean_operation/subdivided_cubes_192/Union` | 168.49 ms | 163.39 ms | 3.03% faster |
| `boolean_operation/cubes/Union` | 3.8631 ms | 3.7679 ms | 2.46% faster |
| `convex_hull/grid_4913` | 7.6063 ms | 7.4052 ms | 2.64% faster |

The subdivided case used 30 samples and a ten-second measurement window for
each side of the A/B. Criterion reported a significant 3.12% regression when
the retained query was disabled (`p < 0.01`), corroborating the direct
point-estimate comparison. Cube union and the grid hull likewise regressed
significantly when disabled (`p < 0.01`). The complete dispatch workload
retained the same output polygon/triangle counts with zero unknown-fact and
fallback/abort events.

The full default and all-feature test matrices passed, as did the
no-default-feature build, warning-denied Clippy, rustdoc, benchmark and fuzz
target builds, the release WASM UI build, and a 15-second sanitizer campaign
over `polygon_predicates` (35,735 executions with no failure).

## 2026-07-19: consume single-operation orientation during triangulation

Status: **kept for difference and intersection**

The certified two-convex path already prunes fragments for one requested
operation and retains their exact front/back winding evidence. Difference and
intersection nevertheless cloned both winding vectors onto every generated
triangle, then immediately classified those copies, allocated a second
triangle/source list, and cloned the merged exact vertex pool. Their operation
orientation is now classified once per retained polygon and consumed directly
while triangulating. The arrangement keeps its original winding pairs for
public retained extraction, closure is still certified on the oriented soup,
and failure still enters the existing precomputed-f64 exact fallback.

Union and symmetric difference retain the prior construction-plus-winding
selection path after the direct path did not improve both workloads. A
31-fresh-process CSGRS/CGAL/OpenCascade sphere/box matrix compared the selective
version with the clean implementation on the same host:

| operation | clean cold | selective cold | cold result | clean warm | selective warm |
| --- | ---: | ---: | ---: | ---: | ---: |
| difference | 2.651918 ms | 2.582393 ms | 2.62% faster | 111.867 us | 88.964 us |
| intersection | 2.005512 ms | 1.981404 ms | 1.20% faster | 33.222 us | 33.023 us |
| union (control) | 4.010995 ms | 3.912562 ms | run-order variation | 129.991 us | 102.673 us |
| symmetric difference (control) | 3.431765 ms | 3.439824 ms | 0.23% noise | 135.436 us | 132.836 us |

Every operation retained the same output size and checksum. Difference and
intersection remained much faster than OpenCascade; intersection also beat
CGAL in the selective run, while the remaining cold competitor gaps stay open.

## 2026-07-19: retain source-point filter queries across support planes

Status: **kept**

The two-convex classifier already caches the certified sign of each exact
source point against each opposing support plane. It still rebuilt the same
four-value floating filter query separately for every previously unseen
point/plane pair. The cache now retains one prepared query per unique source
point alongside its plane-indexed signs. Only conservative approximate values
and their certified error radii are retained; uncertain filters continue to
use the unchanged exact rational signed-product-sum ordering.

A 500-operation alternating-input profile forced a fresh arrangement on every
call. Rational-to-`f64` conversion fell from 10.97% to 8.20% of sampled cycles,
a 25.3% reduction in the targeted hotspot's share. Total sampled cycles also
fell from 4.775 billion to 4.713 billion.

Because sequential release builds showed thermal drift, the end-to-end check
preserved both binaries and alternated their execution for 101 fresh processes
per side. Each process ran the same four CSGRS sphere/box operations; output
sizes and checksums matched throughout.

| operation | repeated query | retained query | cold result |
| --- | ---: | ---: | ---: |
| difference | 2.617380 ms | 2.558905 ms | 2.23% faster |
| intersection | 1.978735 ms | 1.919859 ms | 2.98% faster |
| union | 4.474474 ms | 4.355922 ms | 2.65% faster |
| symmetric difference | 3.500130 ms | 3.393198 ms | 3.06% faster |

Warm measurements used 31 similarly interleaved processes. Union and
symmetric difference were unchanged; difference and intersection moved by
about 1--2.5% even though prepared-arrangement reuse bypasses the modified
code, identifying that residual as binary-layout and measurement variation.

Validation passed the default and all-feature matrices (954 unit tests, 59/60
core integration tests, and 48 regressions with one benchmark smoke test
ignored), the no-default-feature check, warning-denied Clippy and rustdoc,
benchmark and fuzz-target builds, and the release WASM demo. A 20-second ASAN
campaign completed 371 `boolean_pipeline` executions without failure.

## 2026-07-19: prepare projective points once for plane recertification

Status: **kept**

The two-convex path first uses an inexact clip only to propose a smaller set of
active opposing planes, then certifies the resulting homogeneous cycle against
every exact candidate plane. That certification previously converted the same
four exact projective coordinates into conservative floating intervals once
per point/plane pair. `PreparedProjectivePoint3` now retains those intervals
once per cycle point while visiting the candidate planes. The traversal remains
fully certified: inconclusive filters evaluate the unchanged exact rational
four-term signed product sum, and non-rational coordinates retain the general
`Real` predicate fallback.

Both release binaries were preserved and alternated for 101 fresh processes
per side. Output sizes and checksums matched for every sample.

| operation | repeated projective query | prepared projective query | cold result |
| --- | ---: | ---: | ---: |
| difference | 2.540766 ms | 2.415564 ms | 4.93% faster |
| intersection | 1.914299 ms | 1.786318 ms | 6.69% faster |
| union | 4.300215 ms | 4.199283 ms | 2.35% faster |
| symmetric difference | 3.384948 ms | 3.257197 ms | 3.77% faster |

Thirty-one interleaved warm processes also improved every operation by
1.2--1.8%. The focused predicate test checks repeated negative, on-plane, and
positive results, including the non-rational coefficient fallback.

Validation passed the default and all-feature matrices (955 unit tests, 59/60
core integration tests, and 48 regressions with one benchmark smoke test
ignored), the no-default-feature check, warning-denied Clippy and rustdoc,
benchmark and fuzz-target builds, and the release WASM demo. A 20-second ASAN
campaign completed 371 `boolean_pipeline` executions without failure.

## 2026-07-19: keep unselected union fragments recovery-only

Status: **kept for union**

Union needs vertices from unselected arrangement fragments to split selected
boundaries at exact T-junctions, but it does not need to triangulate those
fragments, clone their winding vectors onto every triangle, and then copy the
selected triangle list. The single-union path now classifies each fragment
once. Zero-classification fragments remain in the merged vertex pool and
construction-candidate index but skip boundary triangulation; selected
fragments emit triangles in their final exact orientation. Closure is still
certified, and any failure retains the existing classified-triangle fallback.
XOR remains on its prior path because the broader experiment did not improve
it.

A 301-pair fresh-process release A/B isolated the CSGRS sphere/box union and
alternated the preserved binaries. Every output size and checksum matched:

| operation | classified triangle selection | recovery-only fragments | result |
| --- | ---: | ---: | ---: |
| union | 4.272108 ms | 4.248489 ms | 0.55% faster |

Validation passed the default and all-feature matrices (955 unit tests, 59/60
core integration tests, and 48 regressions with one benchmark smoke test
ignored), the no-default-feature check, warning-denied Clippy and rustdoc,
benchmark and fuzz-target builds, and the release WASM demo. A 20-second ASAN
campaign completed 358 `boolean_pipeline` executions without failure.

## 2026-07-19: defer certified-convex source-edge placeholders

Status: **kept**

Certified two-convex input preparation rebuilds exact edge planes only for
selected output fragments. Its initial triangle carriers nevertheless cloned
the four-coefficient support plane into all three edge slots before knowing
whether a source triangle would reach projective clipping. The carrier now
retains one placeholder support plane. `ProjectiveCycle` expands that value to
the source vertex count only when the triangle actually needs clipping; direct
inside/outside transitions avoid the two unused clones entirely. Exact support
classification, crossing construction, final edge-plane rebuilding, and output
closure certification are unchanged.

Five interleaved release runs per side forced a fresh arrangement for each of
500 alternating CSGRS sphere/box operations:

| operation | eager placeholders | deferred placeholders | result |
| --- | ---: | ---: | ---: |
| union | 1,238.474 ms | 1,219.832 ms | 1.51% faster |
| difference | 1,009.196 ms | 999.312 ms | 0.98% faster |

An exact algebraic alternative expanded each `(p1 - p0) x (p2 - p0)` normal
component into one six-term product sum. It was rejected: the additional
products outweighed delayed rational normalization, regressing union by 6.05%
and difference by 8.83% against the retained deferred-placeholder version.

Validation passed the default and all-feature matrices (956 unit tests, 59/60
core integration tests, and 48 regressions with one benchmark smoke test
ignored), the no-default-feature check, warning-denied Clippy and rustdoc,
benchmark and fuzz-target builds, and the release WASM demo. A 20-second ASAN
campaign completed 362 `boolean_pipeline` executions without failure, and the
downstream CSGRS library suite passed all 304 tests.

## 2026-07-19: borrow classified polygons during triangulation

Status: **kept**

The arrangement already owns every classified polygon through final triangle
extraction. The construction-candidate triangulator nevertheless cloned the
whole polygon set into a temporary vector, including exact support coefficients
and winding vectors, before immediately borrowing those clones. Its internal
polygon consumers now accept either owned or borrowed carriers and the two
classified-arrangement entry points pass references. Vertex materialization,
exact duplicate merging, construction-edge T-junction recovery, orientation,
and closure certification are unchanged.

Five interleaved release runs per side forced a fresh arrangement for each of
500 alternating CSGRS sphere/box operations:

| operation | cloned polygons | borrowed polygons | result |
| --- | ---: | ---: | ---: |
| union | 1,235.264 ms | 1,224.814 ms | 0.85% faster |
| difference | 995.178 ms | 987.234 ms | 0.80% faster |

A follow-up that also borrowed each polygon's retained vertex cycle was
rejected. Avoiding its temporary `Vec<Point3>` changed union by -0.10% and
difference by +0.06%, which is noise, while adding a second materialization
branch.

Validation passed the default and all-feature matrices (956 unit tests, 59/60
core integration tests, and 48 regressions with one benchmark smoke test
ignored), the no-default-feature check, warning-denied Clippy and rustdoc,
benchmark and fuzz-target builds, and the release WASM demo. A 20-second ASAN
campaign completed 359 `boolean_pipeline` executions without failure, and the
downstream CSGRS library suite passed all 304 tests.

## 2026-07-19: borrow convex support planes during classification

Status: **kept**

Certified two-convex classification used a second owned support-plane table
even though the input polygon soup remains alive for the whole preparation.
For the sphere/box workload this cloned every exact normal and offset before
performing read-only equality, classification, and clipping queries. The table
now borrows each source polygon's support plane. Emitted fragments still own
their source or derived polygon geometry, so arrangement lifetime and public
ownership are unchanged.

Five interleaved release runs per side forced a fresh arrangement for each of
500 alternating CSGRS sphere/box operations:

| operation | cloned support planes | borrowed support planes | result |
| --- | ---: | ---: | ---: |
| union | 1,234.734 ms | 1,216.364 ms | 1.49% faster |
| difference | 1,004.544 ms | 995.970 ms | 0.85% faster |

Validation passed the default and all-feature matrices (956 unit tests, 59/60
core integration tests, and 48 regressions with one benchmark smoke test
ignored), the no-default-feature check, warning-denied Clippy and rustdoc,
benchmark and fuzz-target builds, and the release WASM demo. A 20-second ASAN
campaign completed 359 `boolean_pipeline` executions without failure, and the
downstream CSGRS library suite passed all 304 tests.

## 2026-07-19: stop source-plane classification at exact crossing

Status: **kept**

Once one source-triangle vertex is exactly positive and another is exactly
negative against an opposing support plane, the triangle's `Crossing` relation
cannot change. The classifier previously continued through the remaining
vertex. It now returns immediately, avoiding an unnecessary prepared predicate
query and allowing a decided crossing even when a later irrelevant query would
be undecidable.

Wall-clock movement was below scheduler noise, so five interleaved release runs
per side measured retired instructions for 500 alternating fresh arrangements:

| operation | visit every vertex | stop at crossing | result |
| --- | ---: | ---: | ---: |
| union | 11,289,466,082 | 11,277,251,607 | 0.108% fewer instructions |
| difference | 9,012,280,707 | 8,999,990,649 | 0.136% fewer instructions |

Every paired run retired fewer instructions. The focused test also verifies
that only the two decisive source points enter the exact point/plane cache.

Validation passed the default and all-feature matrices (957 unit tests, 59/60
core tests, and 48 regressions plus one intentional ignore), the no-default-
feature check, warning-denied Clippy and rustdoc, benchmark and fuzz-target
builds, and the release WASM demo. A 20-second ASAN campaign completed 358
`boolean_pipeline` executions without failure, and the downstream CSGRS library
suite passed all 304 tests.

## 2026-07-19: reduce exact output centroids once

Status: **kept**

Expanded polygon boundaries require a centroid fan to preserve the certified
closed output. Each coordinate previously accumulated canonical `Real` sums
and then performed another exact division. Exact-rational coordinates now use
Hyperreal's borrowed mean reducer, which chooses one common-denominator
schedule and canonicalizes only the final mean. Non-rational coordinates keep
the original exact `Real` sum and division path, and triangulation topology is
unchanged.

Preserved release binaries each prepared 500 fresh sphere/box arrangements.
Seven counter runs measured:

| operation | repeated reduction | one final reduction | result |
| --- | ---: | ---: | ---: |
| union | 11,171,163,355 | 10,665,871,420 | 4.52% fewer instructions |
| difference | 8,915,422,180 | 8,599,082,893 | 3.55% fewer instructions |

Cycles fell 4.89% for union and 3.60% for difference. In the matched
five-sample cross-kernel run, cold exact difference reached 1.966 ms versus
CGAL EPECK's 1.882 ms, while cold union measured 3.097 ms versus 2.496 ms.
Retained CSGRS remained 22.33x faster than CGAL for difference and 17.75x for
union, and exceeded the tight OpenCascade rows at both temperatures.

Validation passed the default and all-feature matrices (957 unit tests, all 60
core tests, and 48 regressions plus one intentional ignore), the no-default-
feature check, warning-denied Clippy and rustdoc, benchmark and fuzz-target
builds, and the release WASM demo. A 20-second ASAN campaign completed 376
`boolean_pipeline` executions without failure, and the downstream CSGRS library
suite passed all 304 tests.

## 2026-07-19: center expanded fans from source geometry

Status: **kept**

An exact T-junction expands a convex polygon's emitted boundary but does not
change the polygon interior. The closure-preserving triangle fan therefore now
chooses its center from the original polygon vertex cycle instead of averaging
every inserted boundary point. The center remains a strict convex combination
inside the same support plane, while the exact fan boundary, triangle count,
source provenance, and represented surface remain unchanged. A focused
regression inserts a midpoint into a source triangle edge and verifies that the
fan retains the source triangle's exact `(2/3, 2/3, 0)` centroid.

Preserved release binaries each prepared 500 fresh sphere/box arrangements.
Seven counter runs measured:

| operation | expanded-boundary mean | source-cycle mean | result |
| --- | ---: | ---: | ---: |
| union | 10,664,618,650 | 9,926,385,863 | 6.92% fewer instructions |
| difference | 8,599,114,266 | 8,294,119,288 | 3.55% fewer instructions |

Cycles fell 4.91% for union and 2.79% for difference. A two-midpoint dyadic
interior point was rejected after increasing union instructions by 0.81%.

Certified convex arrangements now also expose the already-constructed exact
source support normal by global source triangle and output orientation. The
lookup validates retained source identity and returns no value for general
arrangements that consumed their source polygon carrier. This lets downstream
adapters reuse exact support work without adding normals to `TriangleSoup` or
changing its public storage contract.

Validation passed the default and all-feature matrices (958 unit tests, 60/61
core integration tests, and 48 regressions plus one intentional benchmark
ignore), the no-default-feature check, warning-denied Clippy and rustdoc,
benchmark and fuzz-target builds, and the release WASM demo. A 20-second ASAN
campaign completed 352 `boolean_pipeline` executions without failure. The
downstream CSGRS default and all-feature suites passed all 305/369 library tests
and their integration tests.

## 2026-07-19: emit construction fans directly

Status: **kept**

Closed-arrangement triangulation formerly allocated one boundary vector and
one triangle vector for every emitted polygon, then copied each temporary fan
into the final triangle soup. It now reserves each boundary from the known
polygon size, preallocates the final triangle/source rows, and appends
construction-identity fans directly. A single pass skips consecutive segments
with the same construction-edge identity while retaining the prior weakly
convex fallback when fewer than three strict corners remain. Exact vertices,
triangle order, source provenance, winding rows, and closure certification are
unchanged.

Preserved release binaries each performed 500 fresh sphere/box operations.
Seven counter runs measured:

| operation | temporary fans | direct fans | instruction result | cycle result |
| --- | ---: | ---: | ---: | ---: |
| union | 18,792,107,702 | 18,761,534,911 | 0.16% fewer | 0.98% fewer |
| difference | 13,681,648,051 | 13,653,035,759 | 0.21% fewer | 0.73% fewer |

Heap profiles of 25 unions fell from 1,045,599 to 1,038,724 allocations: 275
allocations removed per operation, or 0.66% of the complete construction,
Boolean, and materialization workload. Direct regressions cover an ordinary
fan, a repeated collinear construction identity, and an uncertified fan that
must leave output untouched for the exact fallback.

Validation passed the default and all-feature matrices (961 unit tests, all 61
core integration tests, and 48 regressions plus one intentional benchmark
ignore), the no-default-feature check, warning-denied Clippy and rustdoc,
benchmark and fuzz-target builds, and the release WASM demo. A 20-second ASAN
campaign completed 370 `boolean_pipeline` executions without failure. The
downstream CSGRS all-feature suite passed all 369 library tests and every
integration test.

## 2026-07-19: index certified source-point classifications

Status: **kept**

Certified-convex polygon edges retain the source mesh and their original endpoint
indices. The vertex shared by the incoming and outgoing source edges therefore
identifies the original source point without hashing its three arbitrary-precision
coordinates. Source-plane classification now stores prepared point queries and
per-plane results directly by that certified source index. Split, derived, or
otherwise unlabeled points continue to use the exact coordinate-identity map, so
the optimization changes neither the certified floating filter nor its exact
fallback.

Thirty deterministic counter runs over 500 alternating fresh sphere/box
operations measured:

| operation | coordinate-map instructions | source-index instructions | result | cycle result |
| --- | ---: | ---: | ---: | ---: |
| union | 9,936,194,568 | 9,861,417,347 | 0.75% fewer | 1.14% fewer |
| difference | 7,860,766,245 | 7,787,262,464 | 0.94% fewer | 0.88% fewer |

In the union profile, `prepare_two_convex_inputs_projectively` fell from 5.20%
to 3.29% self time. Heap profiles of 50 unions fell only from 1,060,050 to
1,059,904 allocations (2.92 allocations per operation), confirming that the
measured CPU reduction comes primarily from avoiding arbitrary-precision
coordinate hashing. A focused regression verifies that certified source
vertices populate the indexed cache without touching the coordinate map, while
the existing early-crossing regression continues to exercise the fallback map.

Validation passed the default, all-feature, and all-target matrices (962 unit
tests, all 61 core integration tests, and 48 regressions plus one intentional
benchmark ignore), the no-default-feature check, warning-denied Clippy and
rustdoc, benchmark and fuzz-target builds, and the release WASM demo. A
20-second ASAN campaign completed 368 `boolean_pipeline` executions without
failure. The downstream CSGRS all-feature suite passed all 370 library tests and
every integration test.

## 2026-07-19: borrow retained output vertex cycles

Status: **kept**

Output extraction and triangulation previously called `ConvexPolygon::vertices`
even when the polygon already retained its exact affine vertex cycle. That call
cloned the complete cycle into a temporary vector before the output path moved
or cloned the same coordinates again. Public polygon extraction, fallback fan
triangulation, and duplicate-vertex merging now borrow retained cycles directly;
polygons without retained vertices preserve the computed affine fallback. The
merge path also reserves its exact total input vertex count before collection.

Eight alternating counter runs each performed 500 fresh, globally shifted
sphere/box operations:

| operation | temporary-cycle instructions | borrowed-cycle instructions | result | cycle result |
| --- | ---: | ---: | ---: | ---: |
| union | 9,988,951,430 | 9,955,446,701 | 0.34% fewer | 0.84% fewer |
| difference | 8,520,274,492 | 8,489,569,986 | 0.36% fewer | 0.82% fewer |

Heap profiles of 50 unions fell from 1,191,406 to 1,183,880 allocations: 7,526
allocations removed in total, or 150.52 per operation and 0.63% of the complete
construction, Boolean, and materialization workload. Exact `Real` coordinates,
polygon order, source provenance, and the non-retained intersection fallback are
unchanged.

Validation passed the default, all-feature, and all-target matrices (962 unit
tests, all 61 core integration tests, and 48 regressions plus one intentional
benchmark ignore), the no-default-feature check, warning-denied Clippy and
rustdoc, benchmark and fuzz-target builds, and the release WASM demo. A
20-second ASAN campaign completed 371 `boolean_pipeline` executions without
failure. The downstream CSGRS all-feature suite passed all 370 library tests and
every integration test.

## 2026-07-19: prepare projected rational output vertices once

Status: **kept**

Construction-aware T-junction filtering projected the same exact output vertex
into many certified 2D line predicates, converting its arbitrary-precision
coordinates and rebuilding conservative error radii each time. The output merge
now prepares every rational 3D vertex once when recovery filtering is enabled.
Line endpoints and candidate queries select their two axes from those retained
intervals; failed conversions, invalid projections, and uncertain filters still
fall through to the unchanged exact point-on-segment predicate.

Eight alternating counter runs each performed 500 fresh, globally shifted
sphere/box operations:

| operation | repeated-conversion instructions | prepared-point instructions | result | cycle result |
| --- | ---: | ---: | ---: | ---: |
| union | 9,955,432,140 | 9,516,772,993 | 4.41% fewer | 4.04% fewer |
| difference | 8,488,857,196 | 8,487,528,295 | 0.02% fewer | neutral |

`Rational::to_f64_lossy` fell from 4.91% to 2.09% self time in the union
profile. Heap profiles rose from 1,183,904 to 1,183,949 allocations over 50
unions—only 0.9 allocation per operation for the prepared-query vector. The
Hyperreal regression suite locks positive, negative, uncertain, and invalid-axis
filter behavior while Hypermesh's exact split-edge and output suites retain the
same topology.

Validation passed 962 unit tests, 60 default integration tests, 48 regression
tests with the benchmark smoke test ignored, the complete all-target/all-feature
suite, the no-default-feature build, Clippy with warnings denied, warning-clean
documentation, benchmark compilation, every fuzz-target build, and the locked
release WASM application build. A 20-second ASAN campaign completed 365
`boolean_pipeline` executions without failure. The downstream CSGRS all-feature
suite passed all 370 library tests and every integration test.

## 2026-07-19: pack certified source classification rows

Status: **kept**

Certified source vertices previously stored one independently allocated plane-
classification vector per vertex. The indexed source path now retains prepared
queries in one vector and classifications in one contiguous vertex-by-plane
matrix. Coordinate-keyed fallback points keep their existing independent cache,
and a changed plane count fails closed instead of indexing an incompatible row.

Eight alternating counter runs each performed 500 fresh, globally shifted 8x4
sphere/box operations:

| operation | per-vertex rows instructions | packed rows instructions | result | cycle result |
| --- | ---: | ---: | ---: | ---: |
| union | 8,048,764,677 | 8,037,695,540 | 0.14% fewer | 0.29% fewer |
| difference | 6,796,668,533 | 6,787,164,684 | 0.14% fewer | 0.17% fewer |

Heap profiles over 100 unions fell from 1,912,386 to 1,909,782 allocations,
removing 2,604 allocations, or 26.04 per operation. The source-relation
regression now verifies both packed query and classification occupancy.

Validation passed the complete 962-test all-target/all-feature suite, the no-
default-feature build, Clippy with warnings denied, warning-clean documentation,
benchmark compilation, every fuzz-target build, and the locked release WASM
application build. A 20-second ASAN campaign completed 373 `boolean_pipeline`
executions without failure. The downstream CSGRS all-feature suite passed all
370 library tests and every integration test.

## 2026-07-20: store retained triangle facts in shared slices

Status: **kept**

Input triangles retained their three exact affine vertices and three source-edge
identities as `Arc<Vec<_>>`. Each fixed-size cache therefore allocated both an
`Arc` owner and a separate `Vec` buffer. These internal caches now use
`Arc<[_]>`; triangle construction places each three-element array directly in
one shared allocation, while variable-size derived cycles convert their owned
vectors at the same boundary. Exact coordinates, edge identities, clone
sharing, polygon order, and every public type remain unchanged.

An isolated cold CSGRS sphere/box union under Callgrind fell from 55,517,396 to
55,466,235 retired instructions, 51,161 fewer instructions (0.092%). A complete
47-workload CSGRS/CGAL/OpenCascade sweep kept all warm CSGRS rows ahead of both
comparison kernels. The cold sphere/box union measured 2.083 ms versus CGAL's
2.024 ms (0.972x), while the related difference, intersection, and symmetric
difference rows remained CSGRS wins.

Two nearby ideas were rejected by deterministic measurement. Carrying selected
fragment orientation through operation-scoped triangulation changed 55,517,396
instructions to 55,518,574, and a per-arrangement prepared-plane filter cache
increased the count to 55,531,805. The existing winding and global predicate
filter paths are already cheaper for this workload.

Validation passed all 962 unit tests, 60 core integration tests, and 48
regressions plus one intentional benchmark ignore; all-target benchmark
harnesses, warning-denied Clippy, rustdoc, and every fuzz-target build were
clean. A 20-second ASAN `boolean_pipeline` campaign completed 369 executions
without failure (LeakSanitizer was disabled because the runner uses ptrace).
The downstream CSGRS all-feature suite passed all 373 library tests and the
nine focused exact adapter/differential tests.

## Completed reference disposition

All reference-derived ideas are mapped as follows:

- **EMBER:** exact plane predicates, local BSP construction, segment tracing,
  early-outs, caches, split ranking, and reference propagation were audited.
  Measured workloads—including a 384-polygon exact leaf—complete in the local
  classifier. Recursive split and work-stealing ideas are architecture-
  inapplicable until that execution policy changes.
- **Mesh Arrangements:** intersection reuse, arrangement cell construction,
  exact winding-vector propagation, output extraction, and repair avoidance.
  Pair symmetry, BVH culling, exact-result caching, triangle predicate reuse,
  and explicit build-once/extract-many arrangement reuse are implemented and
  benchmarked.
- **Generalized Winding Numbers:** the hierarchy, retained winding evidence,
  boundary behavior, and repeated-query model have been audited. A 384-polygon
  certified leaf now covers the paper's size scale, but a shared exact segment
  hierarchy was 42.7% slower. Its approximate solid-angle threshold and graph
  cut are incompatible with the exact closed-PWN contract; a different result
  would require a zero-copy hierarchy design or an intentional contract change.
