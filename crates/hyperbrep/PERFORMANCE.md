# HyperBREP Performance Contract

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

## Current baseline

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
