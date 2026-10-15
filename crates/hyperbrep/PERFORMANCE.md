# HyperBREP Performance Contract

## 2026-10-15: selected archive replay

[Evidence and commands](docs/evidence/2026-10-15-selected-archive/README.md)
cover format 9 graph persistence and independent sheet validation. Three
alternating pinned process pairs preserve all 66 native benchmark receipts
against `3426a3a43`. The median of case median timing ratios is **1.00163**;
the largest sampled ratio is **1.03629**. The benchmark executable grows
**205,040 bytes**; median process peak RSS changes from **21,636 to 22,020 KiB**.
This is a correctness/completeness extension, not a claimed optimization.
Format 9 intentionally changes archive bytes; its own round trips are checked
for byte stability. Existing geometric benchmark outputs retain their checksums.

The new `selected_archive` benchmark constructs sources outside timing and
checks exact area, selected-coordinate retention and topology counts during
replay. Medians across three processes, each with 100 iterations:

| Operation | Median per iteration | Archive bytes |
| --- | ---: | ---: |
| Native sheet encode | 27.472 µs | 8,163 |
| Native sheet parse, validate and area | 109.994 µs | 8,163 |
| Selected sheet encode | 44.462 µs | 9,681 |
| Selected sheet parse, validate and area | 218.026 µs | 9,681 |
| Selected sheet plus 16 cuts encode | 133.211 µs | 26,923 |
| Selected sheet plus 16 cuts replay and area | 587.420 µs | 26,923 |
| Selected replay, another split and area | 258.493 µs | 9,681 input |

The 5-edge and 21-edge archives both retain one selected root and one authored
pcurve source. This fixed composition fixture checks sharing; it is not an
asymptotic time/memory bound. RSS is not an allocation count. Selected solid
revalidation and bounded hostile-input decoding remain open.


Correctness is exact. Performance determines which certified route reaches the
answer, not what answer is accepted.

## Architecture

- Typed integer IDs provide O(1) arena lookup.
- `Model` retains vertex/edge, edge/use, use/wire, wire/face, face/shell, and
  shell/solid ownership indexes.
- Geometry is immutable and clone-shared through `Arc`.
- Expensive immutable geometry facts use thread-safe once caches.
- Ordinary model queries do not rebuild identifier maps.
- Broad phases may eliminate work only with conservative certified bounds.
- Numerical solvers may propose candidates only; exact replay decides topology.

## Required benchmark cohorts

Every established operation will be measured for:

- small integers and rationals;
- large rationals;
- dyadic values lifted from binary interchange;
- algebraic values;
- lazy/computable values;
- close but unequal values;
- exact degeneracies;
- mixed scalar representations;
- cold and warm caches;
- increasing topology and candidate-pair counts.

Operations include geometry construction/evaluation, model commit, adjacency,
classification, intersections, editing, Booleans, area, volume, and derived
tessellation as those capabilities land.

## Instrumentation

Benchmarks should retain counts for:

- broad-phase candidates and certified rejections;
- exact predicate calls and unresolved outcomes;
- refinement and solver replay;
- cache hits and misses;
- allocations;
- expression-size or exact-arithmetic growth where observable without changing
  semantics.

The default regression budget is 10% for an established cohort. A deliberate
regression requires a recorded semantic or architectural reason and a new
baseline.

## Selected topology — 2026-10-15

Kept for exact selected vertices, trims, shared-edge subdivision and pcurve
composition. The [qualification](docs/evidence/2026-10-15-selected-topology/README.md)
records ten public regressions, four rejected mutants and all native/WASM-build
checks. The first native-mapping prototype exceeded the default budget in
three editing cohorts and was replaced with direct stored-scalar formulas.
Selected charts continue to use Hypercurve's existing algebraic authority.

All 66 existing receipts agree across three alternating process pairs. The
median of case median ratios is **1.011410**, with a maximum of **1.058066**;
all cohorts stay within 10%. Binary size grows **67,040 bytes**; median process
peak RSS is **21,472 → 21,808 KiB**. The
[comparison](docs/evidence/2026-10-15-selected-topology/comparison.json) and
[rejected prototype](docs/evidence/2026-10-15-selected-topology/prototype/comparison.json)
retain every sample. No allocation-count or statistical significance is claimed.

The new four-row topology harness measures native/selected splits, sixteen
composed splits and reflection/inverse with exact checks. Prepared roots and
source construction are outside timing. Median costs are 148.213,
161.454, 1931.441 and 337.269 µs respectively; these are
feature costs, not speedups. Selected archive graphs, complete work bounds,
large-chain growth and external comparisons remain open.

## Retained spatial points — 2026-10-15

Kept for exact evaluation and composition of selected line, Bézier and NURBS
images. A point shares immutable source geometry and one selected parameter.
XY/XZ carrier and image preparation are independently lazy; identity proofs
avoid projection work. Affine composition transforms the native controls and
keeps the selector, without adding point-wrapper depth. The tests cover source
lifetime, concurrent readers and retry after stronger scalar evidence. This
is a capability extension, not a claimed optimization.

The [qualification record](docs/evidence/2026-10-15-retained-spatial-points/manifest.json)
binds sources, commands, four rejected mutants, executables and raw logs.
The kernel harness has a direct public-API migration at six call sites;
[the audit](docs/evidence/2026-10-15-retained-spatial-points/migration-audit.json)
verifies unchanged fixtures, iterations and assertions. It is not byte-identical
source. All **66 receipts** agree across three alternating process pairs.
The median of case median ratios is **1.00352**; the largest is **1.07127** for
`boolean_kernel/disjoint_sphere_union_remap_validate`, within the 10% cohort
budget. Binary size increases by **25,232 bytes**. Whole-process median peak
RSS is **21,468 → 21,736 KiB**. These descriptive samples establish neither
allocation counts nor statistical significance. All samples and rows remain in
[comparison.json](docs/evidence/2026-10-15-retained-spatial-points/comparison.json).

The new `retained_points` harness runs 1,000 exact-assertion iterations per row.
Three-run median costs are:

| Scenario | Median per iteration |
| --- | ---: |
| Authored scalar line evaluation and stored-coordinate check | 735 ns |
| Fresh selected line carrier/image and six coordinate order checks | 76.804 µs |
| Shared point identity | 17 ns |
| Independent weighted selected images, warm coincidence | 989 ns |
| Fresh NURBS image on warm carrier, authored knot chart and origin check | 75.174 µs |
| Warm bounds plus six enclosure checks | 115.144 µs |
| Affine transform/inverse pair plus exact coincidence | 30.750 µs |

Root isolation is outside these timings; even the cold-carrier row shares a
prepared selector. These are feature costs, not baseline speedups. The four
retained-parameter and four migrated work-budget receipts also agree in three
runs each. See
[feature-cost.json](docs/evidence/2026-10-15-retained-spatial-points/feature-cost.json).

## Retained inverse parameters — 2026-10-15

Kept for exact inverse-query completeness. `Parameter` retains admitted roots
without requiring a stored scalar; NURBS chart transport reuses Hypercurve's
existing implementation. Represented Bézier queries keep the previous 3D
evaluation check. A second projection is built only when a selected candidate
needs a nonconstant third-coordinate proof, and is shared across that query's
candidates. This is a capability extension, not a claimed optimization.

The [qualification record](docs/evidence/2026-10-15-retained-inverse-parameters/manifest.json)
binds sources, commands, three rejected mutants, executable hashes and logs.
All 66 unchanged kernel receipts agree across three alternating process pairs.
The median of case median ratios is **0.97391**; the largest is **1.05369** for
`spline_kernel/nurbs_planar_face_build_exact_area`, within the 10% cohort budget.
Binary size increases by **18,096 bytes**. Whole-process median peak RSS is
**21,712 → 21,784 KiB**. These are descriptive samples, without statistical or
allocation-count claims. Every row and sample remains in
[comparison.json](docs/evidence/2026-10-15-retained-inverse-parameters/comparison.json).

The new `retained_parameters` release harness includes exact assertions and
runs 250 iterations per row. Three-run median costs are **44.280 µs** for a
quintic root query, **126.885 µs** for a weighted spatial quintic,
**139.514 µs** for rejection of a planar-only candidate, and **137.711 µs** for
two authored NURBS spans. These capabilities were unavailable in the baseline;
the measurements are feature costs. The migrated work-budget harness preserves
its four receipts as well. See
[feature-cost.json](docs/evidence/2026-10-15-retained-inverse-parameters/feature-cost.json).

## Initial kernel baseline

`benches/kernel.rs` measures the first stable final-API cohorts: exact cuboid,
native-cylinder, boundaryless-face sphere, truncated-cone, periodic-patch
torus, polygonal-revolution, affine linear-sweep, fixed-frame rational
Bézier curved-sweep, and homothetic-loft
construction; retained-certificate
solid-volume evaluation; and exact point classification.
Every cohort includes semantic assertions so a faster but incorrect path
cannot become the baseline.
The editing cohorts separately expose canonical edge attachment, repeated
whole-model revalidation, deterministic multi-trace partitioning, and retained
intersection-graph partition dispatch.

The 2026-07-29 optimized baseline on the development workstation is:

The tensor-plane cohort is deliberately rebased from `4.455 us` to
`5.203 us`: it now constructs and retains exact pcurves on both surface
operands rather than returning only a spatial curve. That semantic expansion
is the recorded reason for exceeding the ordinary 10% repeat budget.

Tensor patch validation now derives and compares exact homogeneous iso-curves
rather than recognizing only stored boundary rows and columns. That stronger
certificate deliberately rebases rational Bézier patch construction from
`66.082 us` to `74.365 us` and NURBS patch construction from `286.598 us` to
`323.509 us`.

- cuboid build + exact volume + classification: `248.478 us/iteration`;
- cylinder build + exact volume + classification: `341.097 us/iteration`;
- sphere build + exact volume + classification: `3.610 us/iteration`;
- cone-frustum build + exact volume + classification: `4.153020 ms/iteration`;
- torus build + exact volume + classification: `1.208950 ms/iteration`;
- revolved polygon build + exact volume + classification:
  `889.698 us/iteration`;
- native two-arc circular-profile revolution with eight periodic faces,
  exact curved face carriers, Pappus volume, and radial/profile
  classification: `711.462 us/iteration`;
- affine linear sweep build + exact volume + classification:
  `307.438 us/iteration`;
- fixed-frame nonuniform-weight rational Bézier curved sweep with four native
  tensor translation sides, Bernstein affine-progress certification, exact
  volume, and exact section classification: `306.620 us/iteration`;
- explicit moving-frame polynomial taper with native tensor sides, exact
  Bernstein area-law integration, volume, and section classification:
  `387.317 us/iteration`;
- homothetic loft build + exact volume + classification:
  `343.839 us/iteration`;
- convex corresponding non-homothetic loft with four native bilinear sides,
  exact quadratic section-area volume, and interpolated-section
  classification: `230.517 us/iteration`;
- three-section C⁰ loft with one homothetic span, one convex bilinear span,
  topology-derived layer recertification, exact piecewise volume, and seam
  classification: `485.146 us/iteration`;
- cuboid edge split + full revalidation: `214.314 us/iteration`;
- cuboid face split + full revalidation: `203.518 us/iteration`;
- cuboid curve-driven face split, including two exact boundary-edge splits and
  three full revalidations: `708.639 us/iteration`;
- cuboid two-trace face partition, including four exact boundary-edge splits,
  two face splits, and their current full revalidations:
  `1.596921 ms/iteration`;
- cuboid two-diagonal arrangement, including one exact shared-chord split,
  three face splits, and their current full revalidations:
  `947.797 us/iteration`;
- unit prism cap replacement by an affine-image rational Bézier tensor,
  full shell recertification, and exact volume: `242.584 us/iteration`;
- rational Bézier patch build + validation: `74.365 us/iteration`;
- NURBS patch build + validation: `323.509 us/iteration`;
- native-domain NURBS extrusion-patch build, complete profile-image
  validation, and certified monotone-planar exact face area:
  `176.412 us/iteration`;
- paired complete rational Bézier and native-domain NURBS affine-image area
  queries under separable positive projective weights:
  `18.645 us/paired iteration`;
- two-face rational Bézier patch shell construction with projective boundary
  matching, identity stitching, and full validation: `76.707 us/iteration`;
- explicit tensor-face chordal derivation with four exact samples per boundary
  use, three shared-midpoint refinement levels, 896 output triangles, and
  exact source-surface evaluation at every retained vertex:
  `1.361081 ms/iteration`;
- linear tensor patch / parallel-plane exact native iso-curve and two-pcurve
  intersection: `4.820 us/iteration`;
- curved u-linear translation tensor / oblique-plane exact native
  non-isoparametric curve, rational graph pcurve materialization, and midpoint
  evaluation: `11.368 us/iteration`;
- non-separable positive-weight rational bilinear tensor / oblique-plane exact
  rational-quartic spatial section, rational-quadratic graph construction, and
  midpoint evaluation: `32.671 us/iteration`;
- equivalent degree-one NURBS tensor over authored `[2, 5] × [-3, 7]` active
  knot domains, including exact native-coordinate pcurve remapping and
  midpoint evaluation: `34.393 us/iteration`;
- non-separable positive-weight rational bilinear tensor / plane intersection
  with one retained denominator pole, two bounded rational-quartic branches,
  and spatial plus pcurve midpoint evaluation on both components:
  `69.205 us/iteration`;
- non-separable positive-weight planar rational bilinear tensor / containing
  plane exact complete-surface relation: `395 ns/iteration`;
- non-separable positive-weight planar rational bilinear face / partially
  overlapping bounded plane face exact `CurveRegion2` trim:
  `54.358 us/iteration`, including the exact contained-face coverage
  difference proof;
- complete non-isoparametric rational tensor section, graph-control
  recertification, curved-loop validation, identity-shared face split, and full
  model revalidation: `180.175 us/iteration`;
- complete single-span non-isoparametric NURBS tensor section with a native
  `[2,5]` parameter domain, rational graph recertification, identity-shared
  face split, and full model revalidation: `407.129 us/iteration` for the
  u-linear layout and `404.990 us/iteration` for the v-linear layout;
- complete two-span non-isoparametric NURBS tensor section with exact
  per-span graph elevation, native-domain NURBS pcurve assembly, knot/control
  recertification, identity-shared split, and full revalidation:
  `500.329 us/iteration`;
- two represented partial NURBS graph fragments, exact cross-span carrier
  merging, deterministic all-fragment descendant partitioning, two
  identity-shared same-boundary splits, exact Bezier-loop orientation,
  partial-profile recertification, and full model revalidation:
  `3.641689 ms/iteration`;
- two transverse retained tensor iso-curves, certified pcurve intersection,
  exact crossing atomization, one identity-shared crossing vertex, four
  descendant faces, and full model revalidation: `1.322209 ms/iteration`;
- tensor face intersection, exact two-pcurve clipping, canonical boundary
  attachment, identity-shared iso-curve split, and full revalidation:
  `340.603 us/iteration`;
- affine rational-tensor auxiliary-plane section, authoritative inverse
  pcurve, two-boundary endpoint attachment, identity-shared face partition,
  and full model revalidation: `380.718 us/iteration`;
- constant-weight affine-tensor parameter-plane certification, exact curved
  rational inverse pcurve, identity-shared face partition, and full model
  revalidation: `553.064 us/iteration`;
- rank-one projective affine-bilinear certification, two exact Möbius inverse
  coordinates, homogeneous Bernstein denominator product, identity-shared
  face partition, and full model revalidation: `1.556065 ms/iteration`;
- degree-one projective affine NURBS certification over authored
  `[2, 5] × [-3, 7]` active knot domains, native-coordinate Möbius inverse,
  identity-shared face partition, and full model revalidation:
  `2.076926 ms/iteration`;
- circular rational-span normalization, one closed NURBS inverse pcurve,
  identity-shared inner/outer topology, and full model revalidation:
  `1.082481 ms/iteration`;
- mixed-degree rational Bézier/NURBS span normalization, homogeneous degree
  elevation, one closed NURBS inverse pcurve, identity-shared inner/outer
  topology, and full model revalidation: `1.548178 ms/iteration`;
- two disjoint affine-tensor inverse-pcurve fragments crossing one face hole,
  four exact boundary attachments, paired outer/inner bridge authorship,
  identity-shared descendants, exact volume, and full revalidation:
  `3.992495 ms/iteration`;
- three affine-tensor inverse-pcurve fragments threading two face holes, six
  exact boundary attachments, generalized bridge-cycle authorship, explicit
  wire-arena compaction, exact volume, and full revalidation:
  `8.555741 ms/iteration`;
- disjoint cuboid 6×6 face intersection graph with 36 certified broad-phase
  rejections: `11.423 us/iteration`;
- overlapping cuboid intersection graph with exact transverse trim clipping
  and coincident-plane support evidence: `319.782 us/iteration`;
- retained overlapping-cuboid graph to deterministic transverse and
  coplanar-support face partitions: `6.389954 ms/iteration`;
- retained graph partition plus exact planar interior-witness classification
  and intersection selection: `6.638052 ms/iteration`;
- retained graph partition, selection, identity transfer, union shell
  stitching, validation, and exact volume: `15.505052 ms/iteration`;
- skew-cuboid transverse arrangement, selection, convex-shell stitching,
  validation, and exact volume: `25.483189 ms/iteration`;
- skew-cuboid difference through general concave planar-shell pair
  certification, validation, and exact volume: `31.308049 ms/iteration`;
- contained skew-cuboid difference through general planar void assignment,
  strict shell nesting, validation, and exact volume: `3.141293 ms/iteration`;
- sphere/planar-face graph with exact rational-conic region clipping:
  `3.091935 ms/iteration`;
- disjoint sphere union + arena remap + validation: `3.569 us/iteration`;
- partial sphere intersection + periodic-cap build + exact volume:
  `152.334 us/iteration`;
- oriented coaxial cylinder interior cut + two-solid rebuild + exact volume:
  `1.537104 ms/iteration`;
- coincident cone-frustum interior cut + two-solid rebuild + exact volume:
  `2.274817 ms/iteration`.
- axial plane/cone ray clipping, longitudinal half-frustum standard
  intersection, exact volume, compact JSON encoding, untrusted parse, and full
  revalidation: `460.130112 ms/iteration`. The result JSON is required to stay
  below 100 KB so planar arrangement-expression history cannot silently
  re-enter mixed-shell pcurves.
- coaxial revolution contained cut + inward-shell rebuild + exact volume:
  `2.415905 ms/iteration`.

The cuboid, cylinder, and sphere figures are wall-clock means over 1,000
semantic-checked iterations. The larger torus, cone-frustum, editing, spline,
and Boolean cohorts use the iteration counts printed by the benchmark.

Run the core cohorts with:

```sh
cargo bench --bench kernel
```

Run the derived-output cohort as well with:

```sh
cargo bench --all-features --bench kernel
```

The deleted report-oriented benchmark measured the displaced API and is not a
compatibility target.
