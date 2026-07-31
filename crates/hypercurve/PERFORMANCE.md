# Performance and Reference Audit

Cross-crate measurements live in
[`COMPARATIVE_BENCHMARKS.md`](COMPARATIVE_BENCHMARKS.md). They are kept separate from
this exact-path audit because peer crates use different numeric and topology contracts.

This document records how every source in the README reference list maps to
`hypercurve`, which ideas are already embodied by the implementation, and which
optimization experiments were retained or rejected. The governing constraint is
that a speedup may not weaken exact topology, erase retained evidence, or move a
finite approximation across a predicate boundary.

## Rational-image batch API gate

HyperSolve's shared-denominator transform is now consumed through the immediate
`transform_algebraic_root_rational_images` operation. Its
`AlgebraicRootRationalMap` remains a mathematical map reused across separately
discovered roots. HyperCurve's private conic carrier is correspondingly a
`ConicParameterCandidate2`; no preparation lifecycle remains at this boundary.

Three serialized 20,000-iteration runs gated the immediate batch migration.
The rational Bezier algebraic point-and-tangent image median moved from
6.132 us to 6.123 us per iteration (-0.15%), and every run retained all 40,000
expected transformed coordinate images.

Three serialized optimized runs compared the live code with archived
pre-change HyperSolve and HyperCurve sources. The 20,000-iteration rational
Bezier algebraic point-and-tangent image row moved from a 5.317 us median to
5.315 us per iteration (-0.04%). The 100-iteration irrational-weight
conic/cubic contact row improved from a 736.837 us median to 732.518 us per
query (-0.59%). Every run retained its expected transformed-image and contact
checksums.

## Runtime path tracing

Coverage is audited by executable public family, not by assigning artificial
timings to every enum variant, evidence accessor, or zero-cost data carrier. A
family is covered only when a public workload has semantic test assertions, a
release benchmark, and—when it enters exact computation—a nonempty dispatch
recording. Finite-only adapters are benchmarked and tested but correctly have
no exact-dispatch requirement.

| Public executable family | Semantic tests | Release benchmark | Exact path trace |
| --- | --- | --- | --- |
| Line, arc, bounds, transforms, and primitive evaluation | `hypercurve_arc_bezier`, `hypercurve_bbox`, `hypercurve_curve`, `hypercurve_bezier_evaluation` | `arc`, `bezier_evaluation`, `api_surface` | line/line, arc/arc, quadratic evaluation, similarity transform |
| Curve intersections and curve paths | `hypercurve_curve_intersection`, `hypercurve_curve_string`, `hypercurve_self_contacts` | `intersection`, `intersection_sweep`, `curve_path` | line/line and arc/arc intersections |
| Exact straight-skeleton trajectories, event predictors, construction, and native-family dispatch | straight-skeleton unit fixtures plus `hypercurve_dispatch_trace` and the `straight_skeleton` fuzzer | `straight_skeleton` | general-position concave construction |
| Polynomial/rational Bezier algebra, splitting, arrangement, and retained evidence | the `hypercurve_bezier_*` tests and `hypercurve_rational_bezier` | the `bezier_*` benches and `rational_bezier` | quadratic evaluation and region Boolean |
| B-spline, polynomial spline, and NURBS construction/evaluation | `hypercurve_bspline`, `hypercurve_polynomial_spline`, `hypercurve_nurbs`, `hypercurve_nurbs_interpolation` | `bspline`, `rational_bezier`, `api_surface` | global NURBS interpolation |
| Editing, offsets, fitting, and reconstruction | `hypercurve_contour`, `hypercurve_offset`, `hypercurve_bezier_fit_offset`, `hypercurve_reconstruct` | `editing`, `offset`, `reconstruction` | checked curve-string offset |
| Contours, regions, Boolean topology, and batched point queries | `hypercurve_boolean`, `hypercurve_region*`, `hypercurve_curve_region_boolean` | `containment`, `bezier_region` | region Boolean and batched containment |
| Pathological retained-region memory, transforms, intersections, and all Boolean operations | benchmark fixture smoke paths plus the ordinary family/Boolean suites | `pathological_regions`; feature-gated pathological lanes in `comparative` | every curve family and `Real` representation class across calibrated 100 MiB, 500 MiB, and 1 GiB native inputs |
| Finite projection, retained import, and triangulation boundary | `hypercurve_region`, `hypercurve_triangulation` | `api_surface` | not applicable to the finite-only adapter work; exact reconstruction/topology is traced by the rows above |

The `dispatch-trace` feature enables the shared `hyperreal`/`hyperlimit`
exact-computation trace recorder. The `dispatch_trace` benchmark exercises
public line and arc intersections, polynomial Bezier evaluation, curve-string
offsetting, exact similarity transforms, global NURBS interpolation, region
Boolean construction, and batched region containment. Each workload is
isolated in its own recording window and fails if it produces no dispatch or
rational-reducer evidence.

```bash
cargo test --features dispatch-trace --test hypercurve_dispatch_trace
cargo bench --features dispatch-trace --bench dispatch_trace
HYPERCURVE_PATHOLOGICAL_MODE=boolean \
HYPERCURVE_PATHOLOGICAL_CELL_LIMIT=1 \
HYPERCURVE_PATHOLOGICAL_TIERS=100mb \
HYPERCURVE_PATHOLOGICAL_DISPATCH_TRACE=1 \
cargo bench --features dispatch-trace --bench pathological_regions
```

The integration test protects the trace contract itself; the benchmark prints
the per-operation summaries and cross-stack correlation counters used to relate
performance observations to exact predicate, structural-fact, reducer, cache,
refinement, and approximation paths. The pathological opt-in records one
complete mixed-family retained Boolean run in a shared window and prints every
raw dispatch count plus the rational reducer summary.

The complementary `api_surface` benchmark covers public adapter and authoring
families that are not hot paths inside the topology workloads: checked and
unchecked finite measurements, exact similarity transforms, global NURBS
interpolation, retained import records, and feature-gated finite-ring
triangulation. With `dispatch-trace` enabled, every row records an explicit
`hypercurve-benchmark/api-surface/recorded-workload` entry marker and fails on an empty recording,
including finite-only rows that correctly emit no exact-arithmetic events.

```bash
cargo bench --features triangulation,dispatch-trace --bench api_surface
```

The `straight_skeleton` benchmark isolates public trajectory and event predictors,
full contour construction, native-family dispatch, and exact convex scaling. Its
fuzzer applies topology-preserving orientation, scale, and translation changes to
complete convex, split, and non-general-position fixtures, then compares contour and
curve-path dispatch while validating every emitted graph index.

```bash
cargo bench --bench straight_skeleton
ASAN_OPTIONS=detect_leaks=0 cargo +nightly fuzz run straight_skeleton -- -runs=1000
# Isolate one workload and iteration count under a profiler.
HYPERCURVE_STRAIGHT_SKELETON_GROUP=concave/contour \
HYPERCURVE_STRAIGHT_SKELETON_ITERATIONS=1 cargo bench --bench straight_skeleton
```

The retained-overlap arrangement sentinel defaults to 100 repetitions because
each repetition traverses a 64-curve graph through every retained overlap
view. Set `HYPERCURVE_BENCH_ARRANGEMENT_ITERATIONS` for longer statistical
runs without changing the benchmark workload.

## Reference-by-reference findings

| Reference | Applied finding and disposition |
| --- | --- |
| CGAL, *2D Regularized Boolean Set Operations* and *2D Arrangements* | The arrangement model supports the existing split, classify, resolve, and traverse pipeline in `bezier_arrangement`, `boolean`, and the region Boolean modules. Its aggregate-sweep idea motivated the retained adaptive x-interval scheduler for large curve-string pair batches. A full x-monotone arrangement sweep would require a different exact event/status and provenance architecture, rather than a local optimization of the current pipeline. |
| Bentley and Ottmann, intersection enumeration | Retained as an adaptive one-axis event sweep for large curve-string pair batches. It is deliberately a conservative AABB candidate scheduler, not a full Bentley--Ottmann intersection-status implementation: exact line/arc predicates and source ordering remain unchanged. The crossover and dense fallback are measured below. |
| de Casteljau, affine Bézier evaluation | Directly underlies polynomial Bézier evaluation, exact splitting, flattening, metric prefixes, and moment prefixes. Reusing common affine weights throughout subdivision triangles produced the retained optimization measured below; evaluation preserves that expression graph for non-rational parameters. |
| de Berg et al., *Computational Geometry* | Plane-sweep, arrangement, point-location, and robust subdivision principles match the crate's broad-phase filtering and explicit topology stages. The retained conservative x sweep applies the scheduling portion while leaving exact intersection predicates and ownership unchanged. |
| Boehm, knot insertion | `bspline` performs exact homogeneous Boehm insertion and retains the resulting Bézier spans and source provenance. This is already the appropriate local transformation; no lossy span approximation was introduced. |
| de Boor, splines | Local B-spline evaluation and knot-domain rules are reflected in exact evaluation, sided behavior at discontinuous knots, refinement, and extraction. Cached decomposition and native topology already avoid repeatedly rebuilding that work. |
| Farouki and Neff, plane offsets | The curvature/evolute analysis supplies the exact distance-dependent cusp equation. `bezier_offset` retains the analytic parallel, isolates source and offset cusps, materializes line-image and Pythagorean-hodograph offsets exactly, and keeps all other products behind conservative certification. |
| Farouki and Rajan, Bernstein-form algorithms | Bernstein arithmetic, sign variation, subdivision, substitution, and elimination support the rational-Bézier sign tests, monotonicity certificates, resultants, and root isolation. It also reinforces retaining Bernstein/de Casteljau form instead of eagerly converting every operation to expanded power basis. |
| Farin, CAGD | Bézier/B-spline evaluation, subdivision, rational homogeneous form, derivatives, and variation-diminishing bounds are pervasive throughout the curve carriers. The retained shared-weight change preserves these exact affine identities. |
| Foster, Hormann, and Popa, degenerate polygon clipping | The key lesson is to classify and label degenerate intersections explicitly instead of perturbing them. Curve arrangements retain contact multiplicity, tangent/crossing status, overlap ranges, vertex identities, and operation-aware ownership before traversal. |
| Greiner and Hormann, arbitrary polygon clipping | Intersection insertion followed by entry/exit traversal is reflected in split/classify/traverse Boolean structure. Hypercurve extends the carrier and evidence model for curves and exact degeneracies rather than copying a floating-point polygon-only traversal. |
| Hobby, finite-precision segment output | Finite output can create or erase incidences, so certified flattening, SVG import/export, and reconstruction stay explicit boundaries with evidence. Snap rounding is not silently applied inside exact topology. |
| Hormann and Agathos, point in polygon | Boundary classification precedes winding decisions, and contours expose both nonzero and even-odd fill rules. Conservative boxes and internal batch indexes accelerate repeated classification without changing the winding result. |
| Kasa, algebraic circle fitting | The fit is fast but is a multi-sample algebraic approximation with known bias. It was rejected for deterministic streaming reconstruction, where `reconstruct` instead uses the exact three-point circumcircle/Menger witness and records the finite source provenance. |
| Martinez, Rueda, and Feito, polygon Boolean operations | The sweep overlay provides the asymptotic design and explicit event classification used to motivate the retained large-batch x scheduler. The current curved-region events still retain contour/curve candidates and provenance; replacing them with the paper's polygon-only status structure would be a new carrier architecture and would not preserve curved overlap evidence by construction. |
| Menger, metric geometry | Three-point Menger curvature/circumcircle geometry is used by polyline-to-line/arc reconstruction. It gives the deterministic local witness needed by the streaming reconstruction policy. |
| Patrikalakis, Maekawa, and Cho, *Shape Interrogation* | Bernstein/B-spline interrogation, subdivision solvers, intersections, and offset singularities provide the broad model for native curve carriers and certified candidate isolation. Existing exact interval/resultant stages follow that model while returning uncertainty when completeness is not proved. |
| Schneider and Eberly, geometric tools | Primitive distance, intersection, and construction formulas support the line, arc, box, transform, and offset building blocks. Hypercurve evaluates their branch predicates through the exact policy layer instead of treating approximate formulas as topology decisions. |
| Sederberg and Nishita, Bézier clipping | Convex-hull/Bernstein sign exclusion and recursive parameter contraction are used throughout rational-Bézier candidate and root isolation. Exact parameter intervals are retained when a represented scalar cannot yet be recovered; tolerance is not substituted for proof. |
| Shewchuk, adaptive robust predicates | Adaptive evaluation with exact fallback is supplied through `hyperlimit` and used before topology branches. Hypercurve preserves the separation between a fast certificate and the exact result rather than using epsilon signs. |
| Tiller and Hanson, profile offsets | Exact line/arc joins, caps, and primitive profile offsets are implemented in `offset`. Free-form Bézier offset fitting remains staged behind exact hazard analysis because trimming and singular joins need separate certificates. |
| Raph Levien, parallel Béziers and path simplification | Endpoint-tangent cubic fitting with positive arm solving through the exact midpoint is tried before subdivision. It is only a candidate: exact parallel verification controls acceptance, while a deterministic Blend2D lane remains the completion fallback. |
| Blend2D, simplification and offsetting | Exact same-parameter cubic-to-two-quadratic reduction and the quadratic endpoint-normal construction provide deterministic candidates and radial diagnostics. Hypercurve does not treat Blend2D's radial heuristic as a Hausdorff proof; its independent verifier certifies every emitted span. |
| Vatti, generic polygon clipping | Scanbeam clipping demonstrates a general event/ownership formulation that handles holes and complex polygons. Hypercurve's region pipeline keeps those roles explicit, and its retained x scheduler supplies the compatible broad-phase benefit. A second polygon-only scanbeam carrier would duplicate rather than optimize the curved-arrangement representation. |
| Yap, exact geometric computation | The exact-object discipline is the crate-wide rule: structural filters may accelerate a decision, but a topology branch needs certified evidence. Homogeneous carriers, algebraic parameter intervals, retained blockers, and internally retained construction evidence preserve the information needed for replay. |

## Certified Bézier offset completion and baseline

The retained exact parallel evaluates `P + d J(P') / |P'|` without pretending
that a general cubic parallel is another finite polynomial Bézier. Exact Sturm
isolation schedules source singularities and distance-dependent parallel cusps.
Polynomial Pythagorean hodographs are replayed as exact arbitrary-degree
rational Béziers. Regular non-PH spans try a Levien-style cubic, then Blend2D
quadratic construction, and are accepted only by an exact-scalar conservative
same-parameter bound.

`CurveRegion2::offset_with_certified_bezier_parallel` uses those paths for
smooth joins, separately chord-certifies the produced path, and regularizes the
line arrangement. Its evidence limits the summed bound to the raw
pre-regularization boundary; branch removal is not mislabeled as a Hausdorff
certificate for final topology. Corners and unsupported families use the
existing source-chord fallback, whose weaker guarantee remains explicit.

A release run on the same development machine measured:

| Workload | Result |
| --- | ---: |
| Exact cubic parallel point evaluation | 5.98 us/iter |
| Exact offset-cusp isolation | 122.9 us/iter |
| Exact cubic PH rational materialization | 15.7 us/iter |
| Tight certified cubic construction, 3 spans / 32 verifier leaves | 313.2 ms/iter |
| Smooth four-quadratic `CurveRegion2` certified offset | 4.00 ms/iter |
| Same region through the older source-chord fallback | 9.91 ms/iter |

The feature-gated cross-suite lane uses an identical open cubic and a `0.05`
tolerance where applicable. A three-sample, fixed-one-iteration release smoke
run measured 331.0 ms for Hypercurve's certified construction, 1.28 ms for its
weaker chord fallback, and 10.6 us for Curvo's floating heuristic. These are
intentionally labeled by guarantee: the speed difference is not an
interchangeable-correctness comparison. A focused regression also proves a
case where the accepted Levien cubic uses one span instead of Blend2D's initial
two-quadratic reduction.

```bash
cargo bench --bench offset
HYPERCURVE_COMPARE_GROUP=bezier_offset/open_cubic \
  cargo bench --features comparative-benchmarks --bench comparative
```

## Retained affine de Casteljau optimization

Every interpolation node at a fixed parameter uses the same pair of affine
weights, `1 - t` and `t`. Previously, each `Point2::lerp` invocation rebuilt the
complement. Polynomial splitting now computes the complement once and passes
both weights through the complete de Casteljau triangle. The generic subdivision
kernels used by prefix length and area moments do the same. Certified flattening
now delegates its midpoint split to the canonical optimized split implementation,
removing a second hand-written triangle.

The following timings are paired runs of the focused release benchmarks on the
same machine. They are wall-clock sentinels, not portable absolute performance
claims.

| Workload | Before | After | Change |
| --- | ---: | ---: | ---: |
| Exact cubic split materialization, 25,000 iterations | 29.248 us/iter | 27.027 us/iter | 7.6% faster |
| Linear-algebraic split promotion, 25,000 iterations | 30.354 us/iter | 27.709 us/iter | 8.7% faster |
| Certified cubic flattening, 10,000 iterations | 98.307 us/iter | 88.890 us/iter | 9.6% faster |
| Cubic prefix length bounds | 7.067 us/iter | 6.481 us/iter | 8.3% faster |

Cubic prefix area moments moved by about 0.9%, below a useful independent
signal because integration dominates that workload. It still uses the same
shared subdivision kernel whose benefit is visible in the length measurement.
The benchmark rows remain in `bezier_split_materialization` and `bezier_region`
as focused regression sentinels.

The change is algebraically identical: the same exact `Real` subtraction is
performed once and its result is reused. It neither introduces a finite scalar
nor changes the order or grouping of coordinate multiplications and additions
within an interpolation node.

## Retained exact-rational polynomial evaluation

Farouki and Rajan's Bernstein arithmetic suggests reducing repeated polynomial
work without moving a topology decision into floating point. A focused A/B test
found two useful schedules when the parameter is structurally an exact rational:
quadratics use exact power-basis Horner evaluation, while cubics compute the four
Bernstein weights once and share them across both coordinates. Non-rational and
symbolic parameters still use the original de Casteljau triangle because its
expression structure is observable to downstream `Real` zero-status reasoning.

The focused release benchmark cycles through `1/4`, `1/2`, and `3/4` for 500,000
evaluations. The table evidence medians of three same-machine runs.

| Workload | de Casteljau baseline | Rational fast path | Change |
| --- | ---: | ---: | ---: |
| Exact-rational quadratic evaluation | 1.309 us/iter | 0.853 us/iter | 34.8% faster |
| Exact-rational cubic evaluation | 2.833 us/iter | 2.113 us/iter | 25.4% faster |

An initially unconditional version was rejected after an irrational
`sqrt(2) / 2` cross-check showed that differently grouped but mathematically
equal expressions were not always proved equal by `Point2`'s zero-status query.
The retained structural guard therefore preserves the prior expression graph for
that entire input class. The regression test compares both optimized rational
evaluation and the guarded irrational path against explicit de Casteljau replay,
including extrapolation outside `[0, 1]`.

## Rejected experiment

The same complement-sharing idea was tested in homogeneous rational de
Casteljau subdivision. A short run suggested a small improvement, but a longer
250,000-iteration A/B sequence measured 35.122 us/iter for the provisional
version, 35.686 us/iter for the baseline, and 35.690 us/iter when the provisional
version was repeated. That spread does not distinguish the implementation from
system noise, so the rational production change was reverted. The rational
quadratic split benchmark remains as a sentinel for a future, independently
measurable optimization.

## Retained adaptive x-interval sweep

Bentley--Ottmann and the polygon-overlay references motivated testing an ordered
broad phase rather than continuing to defer the idea without a crossover
workload. Curve-string pair queries now choose an x-interval event sweep when
the flat product contains at least 4,096 pairs and an 8-by-8 exact-rational
sample is not dense. Certified dyadic enclosures provide conservative event
coordinates; starts precede ends at equal coordinates, so endpoint contact is
never pruned. An event-only counting pass abandons the schedule when more than
half the flat pairs overlap in x. Surviving rows are restored to source-index
order before the existing exact AABB and line/arc predicates run.

The benchmark uses globally overlapping curve-string boxes with remote tails,
so a whole-object AABB rejection cannot manufacture the result. Each retained
number is the median of five release-profile runs; every run preserved the flat
candidate count, skip/test accounting, an empty exact intersection set, and the
same checksum.

| Workload | Flat scan median | Adaptive sweep median | Change |
| --- | ---: | ---: | ---: |
| 64 x 66 segments, direct | 970.400 us | 107.487 us | 88.9% faster |
| 64 x 66 segments, prepared (historical, API retired) | 911.943 us | 72.506 us | 92.0% faster |
| 128 x 130 segments, direct | 3.841 ms | 212.683 us | 94.5% faster |
| 128 x 130 segments, prepared (historical, API retired) | 3.797 ms | 142.745 us | 96.2% faster |
| 512 x 514 segments, direct | 69.282 ms | 1.001 ms | 98.6% faster |
| 512 x 514 segments, prepared (historical, API retired) | 68.029 ms | 632.556 us | 99.1% faster |

The adversarial x-dense/global-overlap sentinel selected the flat scan. Its
64-, 128-, and 512-segment medians stayed within 1.3% of baseline; the largest
movement was a 1.1% slowdown and is below the retention threshold. A 64-by-65
equal-x endpoint-contact regression exercised the active sweep and proved that
the immediate and now-retired retained-query lanes produced the same event and
source indices.

## Retained exact AABB separation short-circuit

`Aabb2::overlaps` formerly certified all four directed axis separations before
combining them. A single strict separation is already a complete exact proof
that two closed boxes are disjoint, so later comparisons were redundant. They
could also turn a proved disjoint result into `Uncertain` if an unrelated later
axis exhausted its ordering budget. The classifier now visits the four
directions in the same order and returns `Decided(false)` on the first certified
separation; only a still-required undecidable comparison returns uncertainty.
Edge and corner contact remain inclusive because equality is not separation.

On the csgrs Reuleaux region-Boolean workload, the paired 30-sample median fell
from 14.266 to 12.523 ms/op (12.22%). The optimized 12.405--12.598 ms/op
interquartile range did not overlap the 14.199--14.441 ms/op control range.
Dispatch tracing showed the mechanism directly: Real comparisons fell from
16,081 to 12,307 (23.5%), comparison refinements from 7,163 to 5,830 (18.6%),
and total dispatch events from 141,208 to 116,040. The all-target/all-feature
gate exercised every integration, benchmark, and example target without a
failure.

## Retained shared-coordinate NURBS solve

Global NURBS interpolation solves the same exact basis matrix for the x and y
control coordinates. The former path constructed one coefficient determinant,
then one replaced-column determinant per control and coordinate. The rational
branch now uses Hypersolve's shared multi-right-hand-side Bareiss elimination:
the matrix is eliminated once, both coordinate columns follow the same
certified row operations, and each coordinate retains its Cramer numerators and
independent exact residual replay. Symbolic chord/centripetal inputs keep the
existing determinant-identity path because their mathematically zero residuals
are not always normalizable by the current scalar package.

Five same-machine 10,000-iteration release runs reduced the uniform quadratic
interpolation median from 19.276 us/iter to 13.098 us/iter (32.1%), with the
same checksum. Dispatch events fell from 11,520,001 to 6,400,001 per 10,000
constructions (44.4%) with no refinement events on either path. The exact test
suite retains determinant and numerator evidence, exercises rational weighted,
nonuniform, and symbolic parameterizations, and property-checks generated cubic
control recovery. An intermediate integration of two independent augmented
solves was rejected at 20.356 us/iter because it repeated the matrix
elimination.

## Retained certified triangulation-conformity fast path

Finite regions with holes delegate exact triangulation to HyperTri. HyperTri formerly
followed every successful earcut with a global exact vertex-on-triangle-edge scan to
repair source boundary vertices that normalization may have skipped. The retained
implementation first certifies the emitted mesh structurally: every authored exterior
or hole edge must occur exactly once and every other triangle edge exactly twice. Only
a certified mesh skips the scan; any missing authored edge runs the unchanged exact
geometric repair. A collinear-boundary regression proves both certificate rejection
and repaired conformance.

For `finite_ring_triangulation`, five same-source release control runs had a
62.656 us/iter median; five retained runs had a 32.671 us/iter median (47.9% faster),
with the same eight-triangle checksum. Per 10,000 calls, trace events fell from
8,460,001 to 7,260,001 (14.2%) and exact predicates from 5,460,000 to 4,260,000
(22.0%); neither path refined. Hypercurve's six triangulation integration tests,
strict all-target/all-feature Clippy, and warning-denied rustdoc passed after the
dependency change.

The retained dependency then adds an exact rectangular-annulus dispatch for the common
four-corner material plus one four-corner hole case. It structurally identifies both
axis-aligned rectangles, proves strict containment through exact scalar ordering,
constructs eight consistently wound triangles, and requires the authored boundary
certificate before returning. Rotated starts and reversed winding are supported;
nonrectangular, touching, multi-hole, or collinear-authored inputs use general earcut
and its exact conformity repair.

Five additional release runs reduced the median from 32.671 to 6.254 us/iter (80.9%),
or 90.0% from the original 62.656 us/iter control, at the same checksum. The retained
one-call trace has 136 events and 52 predicates, down from 726 and 426 respectively;
scalar comparisons fell from 141 to 8 and orientations from 87 to 8, with no
refinements. Hypertri's full unit/adversarial/differential/property and all-target gate
plus Hypercurve's public triangulation integration tests remain green.

## Retained finite-ring boundary deduplication

`Contour2::import_finite_ring` previously promoted every source point before
using exact `Point2`/`LineSeg2` construction to count cyclic duplicate edges,
then exact-compared adjacent points again while filtering those duplicates.
Finite input equality is already authoritative at this explicit adapter
boundary: equality of finite binary64 coordinates is identical to equality of
their promoted exact dyadics, including signed zero. The importer now validates
all coordinates for finiteness, counts and filters source duplicates in one
finite pass, and promotes each retained point once. Because finite dyadic
promotion preserves that equality proof, and the cyclic native segments share
cloned endpoints, the adapter now constructs the already-proven nonzero,
connected, closed exact ring directly instead of repeating two exact squared-
distance validation passes.

The direct 384-point retained-import sentinel fell from the original 751.199
to 127.175 us/iter over 10,000 iterations (83.1%), or 76.2% from the preceding
533.464 us/iter retained version, with the same 384 emitted segments and
retained-record accounting. CSGRS's involute-gear constructor, which crosses
this boundary with its finite analytic tessellation, fell from 0.637 to 0.173
ms/op across matched 30-sample runs (72.8%); cycloidal gear fell from 0.391 to
0.118 ms/op (69.8%). Their one-call traces fell from 24,619 to 943 and from
12,839 to 799 dispatch events respectively, with zero approximations,
refinements, fallbacks, or unknown facts. Signed-zero, adjacent/closing duplicate,
all-duplicate, and nonfinite regressions protect the adapter semantics; an
exact unified-region comparison proves retained point order and coordinates.
The dedicated retained-import target completed 1,000
AddressSanitizer-instrumented executions (651 coverage points and 1,686 feature
edges). Its isolated 64-point dispatch sentinel records 128 events and zero
approximations, refinements, or unknown facts.

## Retained Boolean query work

The evidence-bearing direct region Boolean path already collected one exact
boundary-intersection set, but its contact checks recomputed that set and its
fragment classifier rebuilt both operands' contour and region boxes for every
representative point. The retained path now consumes the original intersection
set throughout contact and overlap decisions, constructs transient prepared
views once for fragment classification, and reuses their conservative segment,
contour, and region boxes. These remain filters only: a box can discard only a
certified miss, while every surviving candidate reaches the same exact boundary
and winding predicates.

Prepared winding classification now also uses each retained segment box's
maximum x. Winding casts a ray toward positive x, so a segment whose certified
maximum is strictly left of the query cannot contribute. Equality remains on
the exact path, preserving boundary behavior, and an undecidable comparison
retains the candidate.

The comparative 64-vertex star intersection fell from 37.065 ms/iter to
18.742 ms/iter (49.4% faster), with the same 100-segment checksum. Rectangle
union fell from 163.476 us/iter to 139.375 us/iter (14.7% faster), with the same
eight-segment checksum. These are end-to-end medians from the release
comparative runner; the final run used 15 samples with a 50 ms calibration
target. A one-iteration Callgrind sweep of all comparative lanes fell from
1,413,942,582 to 725,893,515 instructions (48.7%). The exact implementation
still trails finite-only competitors in these lanes, so these numbers are a
checkpoint rather than a parity claim.

The dedicated `region_boolean` fuzz target compares direct and prepared results
for all four Boolean operations and point classification over generated exact
rectangles. It completed 1,000 AddressSanitizer-instrumented executions with
4,829 coverage points and 10,609 feature edges. The full nightly fuzz build also
now handles reversed algebraic-endpoint split fragments instead of failing its
exhaustiveness check.

The next profile showed that graph traversal repeatedly revalidated the same
directed-fragment geometry while transferring an already-validated private
carrier through chain and loop stages. Public chain, chain-set, loop, and
loop-set constructors still perform their complete ownership, nonzero,
connectivity, and closure checks. Internal transitions now preserve the proof
established by fragment emission and adjacency traversal instead of replaying
it at every carrier boundary. The star intersection fell again from 18.742 to
16.925 ms/iter (9.7%), and the complete comparative Callgrind sweep fell from
725,893,515 to 631,370,861 instructions (13.0%). Cumulatively, the star lane is
54.3% faster and the sweep executes 55.3% fewer instructions than the original
checkpoint.

Split materialization then accounted for almost a quarter of the remaining
Boolean work. Every adjacent marker pair proved its endpoints distinct with an
exact squared-distance expression, and retained line fragments repeated the
same proof in their support-range constructor. Line segments now retain whether
their construction certified distinct endpoints. After split-marker incidence
and strict parameter order are established, affine injectivity carries that
fact to each line fragment; arcs keep the geometric check because a full-circle
sweep may revisit its start point. The private support-range constructor also
accepts the immediately preceding endpoint proof instead of recomputing it.

Prepared contour point classification now uses its retained line/arc predicate
handles for both boundary membership and winding orientation. The canonical
boundary-first order, half-open winding convention, fill rule, and uncertainty
behavior remain unchanged; only fixed-endpoint predicate preparation is reused.
Together these changes reduced the star intersection from 16.925 to 13.453
ms/iter (20.5%) and rectangle union from 135.149 to 127.385 us/iter (5.7%) in
the same 15-sample, 50 ms comparative configuration. The complete one-iteration
Callgrind sweep fell from 631,370,861 to 517,904,109 instructions (18.0%). From
the original checkpoint, the star lane is now 63.7% faster and the sweep uses
63.4% fewer instructions, with unchanged output checksums.

The next winding profile showed 3,306 exact line-orientation calls despite the
retained segment boxes. Once a line is certified to straddle the query y, a box
whose minimum x is strictly greater than the query x proves that the positive
horizontal ray crosses the line. The crossing direction already determines the
winding delta, so both prepared and ordinary cached-box classifiers now avoid
orientation in that case. Equality and uncertain x order still execute the
exact orientation predicate. This reduced prepared winding orientations to
1,098 in the comparative sweep, the star intersection to 11.292 ms/iter (16.1%
below the preceding checkpoint), and the full sweep from 517,904,109 to
427,921,184 instructions (17.4%). Cumulatively, the star lane and instruction
sweep are both about 69.5% below the original checkpoint.

Boundary emission then spent 46.2 million instructions re-proving that every
selected directed fragment had nonzero geometry. Public fragment emission still
performs that proof because callers can supply their own `ContourFragmentSet`.
The private region Boolean pipelines now preserve the stronger provenance from
`ContourFragmentSet::from_split_markers`: strict adjacent marker order on an
injective source segment certifies nonzero output, and reversal preserves endpoint
inequality. Ownership uniqueness and unresolved-boundary validation still run at
the emission boundary. This reduced star intersection from 11.292 to 10.237
ms/iter (9.3%) and the comparative sweep from 427,921,184 to 383,736,469
instructions (10.3%).

The sealed `BooleanBoundaryLoop` carrier similarly already guarantees nonzero,
connected, closed geometry: its public constructor proves those invariants, and
its private extraction constructor receives an exactly assembled closed chain.
Contour transfer now constructs the closed carrier directly instead of asking
`CurveString2` and `Contour2` to replay the same endpoint proofs. Public loop and
loop-set constructors retain their full validation. Star intersection fell again
to 9.220 ms/iter (9.9%), rectangle union measured 121.996 us/iter, and the full
sweep fell to 336,036,245 instructions (12.4%). From the original checkpoint,
the star lane is 75.1% faster and the instruction sweep is 76.2% smaller, with
unchanged output checksums. A direct midpoint/lazy fallback experiment was
rejected after increasing the sweep by 7.8 million instructions.

The exact Boolean differential fuzz target completed another 1,000
AddressSanitizer-instrumented runs after these carrier changes, reaching 4,969
coverage points and 13,625 feature edges without a failure.

The dominant remaining fragment-classification scan still asked every prepared
contour to prove boundary exclusion before computing winding. For an internally
produced split set, successful marker construction has already rejected unresolved
segment relations. When the complete retained intersection set contains no overlap,
every opposite-boundary contact is therefore a marker endpoint, so a strict
one-half/one-third/two-thirds fragment sample cannot lie on that boundary. The direct
and prepared Boolean pipelines now preserve this proof and use winding-only prepared
classification in that case. Any overlap or unresolved event keeps the canonical
boundary-first classifier, as do all public point-classification APIs.

This reduced prepared line classifications from 1,098 to 549 in the comparative
sweep. Star64 intersection fell from 9.220 to 7.822 ms/iter (15.2%), while rectangle
union remained in its recent noisy range at 126.046 us/iter. The full Callgrind sweep
fell from 336,036,245 to 280,749,710 instructions (16.5%). Cumulatively from the
original checkpoint, star64 is 78.9% faster and the sweep executes 80.1% fewer
instructions, with unchanged output checksums. The post-change exact Boolean
differential target now checks decided output membership against the Boolean truth
table in addition to comparing direct and prepared paths. It completed 1,000
AddressSanitizer-instrumented runs at 5,067 coverage points and 14,297 feature edges
without a failure.

Boundary-contour role assignment then accounted for almost the entire next
output-materialization hotspot. After pairwise exact contour intersection has
accepted every contour pair, the start point already stored on each candidate
contour is a valid nesting sample against every other boundary. Nevertheless,
the implementation eagerly constructed three exact interpolants on every
segment before trying that start point. Nesting now evaluates those interior
samples lazily: the existing endpoint is classified first, while the unchanged
one-half/one-third/two-thirds sequence remains available if an exact
point-containment predicate is undecided. Intersection validation, containment
classification, uncertainty propagation, and public role evidence are unchanged.

In the same 15-sample, 50 ms comparative configuration, star64 intersection
fell from 7.822 to 6.379 ms/iter (18.4%) and rectangle union measured 105.207
us/iter, with unchanged checksums. The complete one-iteration Callgrind sweep
fell from 280,749,710 to 227,341,565 instructions (19.0%), removing the prior
53.4-million-instruction eager interpolation cost rather than replacing it with
an approximate sample. From the original checkpoint, star64 is now 82.8% faster
and the instruction sweep is 83.9% smaller. The exact Boolean differential fuzz
target completed 1,000 AddressSanitizer-instrumented runs at 5,054 coverage
points and 14,451 feature edges without a failure.

Structural evidence construction and prepared predicates then recomputed the same
`LineSeg2Facts` across immutable line carriers. Those facts depend only on the
endpoint coordinate set and the structural zero status of `(dx, dy)`, so lines
now retain them lazily. Clones carry an already-computed cell, reversal preserves
it because coordinate-set and zero-mask facts are direction invariant, and any
operation that changes endpoint geometry starts with an empty cell. The public
facts API and its conservative scheduling semantics are unchanged.

This reduced line-fact computations in the comparative sweep from 5,790 to 730
(87.4%). Star64 intersection fell from 6.379 to 5.309 ms/iter (16.8%), and
rectangle union fell from 105.207 to 62.840 us/iter (40.3%), with unchanged
checksums. The complete one-iteration Callgrind sweep fell from 227,341,565 to
185,994,304 instructions (18.2%). From the original checkpoint, star64 is now
85.7% faster and the sweep executes 86.8% fewer instructions. An alternative
heap-shared cache representation was rejected because it increased the full
sweep and added about 0.8 million instructions to the offset lane. The exact
Boolean differential fuzz target completed 1,000 AddressSanitizer-instrumented
runs at 5,071 coverage points and 14,417 feature edges without a failure.

Prepared winding classification then retained two exact candidate schedules
instead of re-proving every axis test for every fragment sample. All prepared
contours with decidable exact boxes retain segment order by maximum x, allowing
queries to skip the prefix strictly left of the sample. All-line contours with
at least eight segments additionally retain exact ranks by minimum y and maximum
y, plus each segment's vertical direction, so the half-open crossing condition
`min_y <= y < max_y` is intersected with the x candidates using integer ranks.
Mixed contours, smaller line contours, `EdgePreview` data, and any undecidable
ordering or query comparison keep the prior conservative scan. Boundary-first
public classification and exact orientation tests are unchanged. Boolean
fragment classification also constructs its unchanged one-half, one-third, and
two-thirds fractions once per selection rather than once per fragment.

In the same 15-sample comparative configuration, star64 intersection fell from
5.309 to 4.621 ms/iter (13.0%), while rectangle union measured 62.692 us/iter
and remained within baseline noise. The complete one-iteration Callgrind sweep
fell from 185,995,006 to 161,994,281 instructions (12.9%). From the original
checkpoint, star64 is now 87.5% faster and the sweep executes 88.5% fewer
instructions. Specialized midpoint and affine-step interpolation experiments
were rejected after increasing either the instruction sweep or multiple wall-time
lanes. A half-open vertex-level differential regression now covers the retained
line index, and the exact Boolean differential fuzz target completed 1,000
AddressSanitizer-instrumented runs at 5,113 coverage points and 14,772 feature
edges without a failure.

Nonparallel line intersection then spent four shared-endpoint comparisons plus
four endpoint-incidence predicates before solving parameters even when every
endpoint coordinate was exact rational. For that structural class, the exact
rational parameters already decide finite-range membership and endpoint status,
so the kernel now solves first and reuses a source endpoint whenever either
parameter is zero or one. Symbolic and mixed
lines retain the endpoint-first path because their source incidence can remain
decidable when a derived parameter is not; a dedicated symbolic shared-endpoint
regression preserves that distinction. Ordinary lines also reuse the already
certified support determinant as their fragment determinant, borrow their
ordinary deltas instead of cloning support deltas, and construct the support
origin delta only in the parallel branch. Cross and dot kernels now use
`Real::diff_of_products` and `Real::dot2_refs`, preserving the same exact
polynomials while delaying rational normalization.

Star64 intersection fell from 4.621 to 4.233 ms/iter (8.4%) and rectangle union
from 62.692 to 59.657 us/iter (4.8%) in the final 15-sample comparative run, with
unchanged checksums and effectively unchanged offset/NURBS lanes. The complete
one-iteration Callgrind sweep fell from 161,994,281 to 148,433,531 instructions
(8.4%). From the original checkpoint, star64 is now 88.6% faster and the sweep
executes 89.5% fewer instructions. A whole-polynomial point-interpolation route
was rejected after increasing the sweep to 162,499,975 instructions and
regressing offset/NURBS timing. The exact Boolean differential fuzz target
completed 1,000 AddressSanitizer-instrumented runs at 5,156 coverage points and
14,798 feature edges without a failure.

Fragment classification still constructed at least one new exact interior point
for every emitted split fragment, even when an existing source vertex had already
been proved off the opposite boundary. Region event collection now retains a
point-contact endpoint index keyed by contour and source segment. Endpoint queries
check both incident segments around the closed contour, so event normalization at
a shared vertex cannot make the other segment appear contact-free. A fragment may
classify an existing start or end point only when the complete event set contains
no overlap or unresolved event and that endpoint has no retained contact. Unknown
parameter comparisons disable the shortcut, strict-interior split parameters remain
contacts, and an uncertain endpoint classification falls through to the unchanged
one-half/one-third/two-thirds sequence. Public and custom fragment classifiers keep
their canonical sampling path.

In the final 15-sample comparative run, star64 intersection fell from 4.233 to
2.858 ms/iter (32.5%) and rectangle union fell from 59.657 to 49.612 us/iter
(16.8%), with unchanged checksums. The complete one-iteration Callgrind sweep fell
from 148,433,531 to 95,828,985 instructions (35.4%). From the original checkpoint,
star64 is now 92.3% faster and the sweep executes 93.2% fewer instructions. Direct
prepared exact-rational orientation and retained-range `point_at` experiments were
rejected after increasing the sweep to 149,737,564 and 187,988,683 instructions,
respectively. A closed-contour adjacency regression covers contact ownership by
either incident segment. The exact Boolean differential fuzz target completed
1,000 AddressSanitizer-instrumented runs at 5,229 coverage points and 15,007 feature
edges without a failure.

The remaining fragment-classification cost was proportional to the number of
fragments even when the normalized event set already proved a repeated sequence
of transverse line crossings. A new retained crossing-winding index records the
exact integer winding change at each strict proper crossing from the sign of the
two oriented line directions. The Boolean classifier computes one seed winding
per source contour, then visits its fragments once and applies those retained
transitions. This tracks the full integer winding rather than toggling an inside
bit, so both non-zero and even-odd fill rules remain valid even when a contour is
self-intersecting.

The path is intentionally narrow. It requires one material contour and no holes
on each side, at least one retained point event, line primitives on both sides,
strict-interior `Crossing` events, unique and comparable source parameters, zero
net winding change around both closed contours, and a one-to-one match between
events and materialized split boundaries. Endpoint contacts, tangencies, arcs,
overlaps, duplicate parameters, uncertain predicates, and any
proof mismatch retain the per-fragment classifier. A differential regression
compares the new selection and evidence with the canonical classifier and confirms
that a qualifying two-contour case issues only two seed winding queries.

In the latest 15-sample comparative run, star64 intersection fell from 2.858 to
2.428 ms/iter (15.0%). Rectangle union stayed on the existing small-event path
and measured 53.079 us/iter with a 50.727 us minimum; offset and NURBS lanes were
effectively unchanged. The complete one-iteration Callgrind sweep fell from
95,828,985 to 80,471,959 instructions (16.0%), with unchanged checksums. From the
original checkpoint, star64 is now 93.5% faster and the sweep executes 94.3% fewer
instructions. The exact Boolean differential fuzz target completed 1,000
AddressSanitizer-instrumented runs at 5,253 coverage points and 15,082 feature
edges without a failure.

Exact line-intersection witnesses then used the general point interpolation path,
which cloned the source parameter and normalized two independent multiply-add
expressions per coordinate. The nonparallel line kernel now constructs the same
`(1 - t) * start + t * end` polynomial with the fused exact two-product sum. This
is deliberately local to line intersection: applying the representation change to
all point interpolation increased the offset lane's Callgrind subtree from 742,275
to 756,286 instructions, while the specialized kernel improved the complete sweep.

The retained crossing-winding path also eagerly built two prepared region views
before proving that it needed only two seed winding numbers. It now scans each
already-certified all-line source contour directly for those seeds and constructs
prepared predicates only on fallback. Because that changes the crossover economics,
every nonempty event set may attempt the conservative retained-crossing proof; any
ineligible event still falls through unchanged. Finally, fragment evidence recover
the primitive family directly from the `Segment2` variant instead of recomputing
full structural facts for each source and materialized fragment. Splitting preserves
the primitive family, so kind counts and provenance evidence remain identical.

In the final 15-sample comparative run, star64 intersection fell from 2.428 to
1.734 ms/iter (28.6%) and rectangle union from 53.079 to 35.740 us/iter (32.7%).
Offset measured 20.194 us/iter and NURBS evaluation 1.167 us/iter, with unchanged
checksums. The complete stripped one-iteration Callgrind sweep fell from 80,471,959
to 61,253,225 instructions (23.9%). From the original checkpoint, star64 is now
95.3% faster and the sweep executes 95.7% fewer instructions. The full all-feature
and no-default-feature test matrices, warnings-as-errors Clippy and rustdoc gates,
and the exact Boolean differential fuzz target all passed; the latter completed
1,000 AddressSanitizer-instrumented runs at 5,587 coverage points and 15,636 feature
edges without a failure.

Exact AABB construction then remained the largest Hypercurve-owned source of
coordinate cloning. A connected all-line curve string previously boxed both ends
of every edge and repeatedly unioned shared vertices; closed contours also revisited
their final endpoint even though it is the first vertex. Line strings now visit each
authored vertex once, and line contours visit each segment start once. Mixed and arc
carriers retain the existing certified sweep-extrema construction. Region event
collection also retains each contour box and lazily retains each segment-box set once
across all material/hole role combinations instead of rebuilding the same immutable
broad-phase evidence for each tested contour pair. Contours rejected by their outer
boxes never pay for segment boxes.

For contour pairs with at least 256 Cartesian segment pairs, event collection now
orders decided second-operand boxes by exact minimum x. Each first-operand scan stops
only after a certified strict x separation; earlier candidates are rejected only by
the same exact x/y interval comparisons as the flat filter. Missing or uncertain
boxes remain candidates, an unprovable retained ordering falls back to the flat scan,
edge contact remains inclusive, and candidate pairs are restored to source segment
scan order before exact intersection. A differential unit regression compares the
retained candidates with the original flat AABB filter and includes an unknown box.
`Segment2::kind` also exposes the authoritative enum family directly so event and
fragment evidence no longer request unrelated structural facts merely to name a
primitive.

In the final 15-sample comparative run, star64 intersection fell from 1.734 to
1.565 ms/iter (9.7%) and rectangle union from 35.740 to 30.344 us/iter (15.1%),
with unchanged checksums. Offset measured 20.854 us/iter and NURBS evaluation
1.139 us/iter. The complete stripped one-iteration Callgrind sweep fell from
61,253,225 to 55,566,500 instructions (9.3%). From the original checkpoint,
star64 is now 95.8% faster and the sweep executes 96.1% fewer instructions. The
full all-feature and no-default-feature test matrices, warnings-as-errors Clippy
and rustdoc gates, and the exact Boolean differential fuzz target all passed; the
latter completed 1,000 AddressSanitizer-instrumented runs at 5,615 coverage points
and 15,661 feature edges without a failure.

The retained x sweep originally collected every surviving segment pair in one
global vector before restoring source order. It now visits one first-operand
segment at a time, restores only that segment's second-operand order, and then
reuses the buffer. Exact AABB rejection, uncertain-box retention, event order,
and the flat-scan fallback are unchanged, while temporary candidate storage is
bounded by the second contour's segment count instead of the total surviving
pair count.

The evidence-bearing region Boolean path also kept borrowed intermediate geometry
alive through final materialization, cloned closed chains into loops, cloned
loops into contours, and cloned every completed stage evidence into the aggregate
evidence. Successful stages now consume chains and loops and move their evidence;
the public evidence remains identical without the redundant exact carriers.
Against the preceding checkpoint, the stripped one-iteration comparative
Callgrind sweep fell from 55,566,500 to 53,745,141 instructions (3.28%). The
one-cell all-operation pathological comparison fell from 165,920,306 to
159,422,324 instructions (3.92%). In the matching 15-sample, 50 ms run,
star64 intersection measured 1.431 ms/iter, rectangle union 24.758 us/iter,
offset 20.081 us/iter, and NURBS evaluation 1.113 us/iter, with unchanged
checksums.

The new native pathological fixture calibrates each all-family shard at roughly
1.5 MiB. Its 100 MiB construction tier selected 67 shards and observed a
102.8 MiB resident-set increase in about 135 ms; an exact deep-copy rotation of
the built tier took 20.5 ms. The curved Boolean lane prepared all 67 pairs and
retained 9,288 candidate carrier pairs, then reported 268 exact predicate
blockers rather than approximating them. The common finite projection decided
all 268 operation/cell combinations in about 277 ms. These lanes are diagnostic
capacity tests, not claims that unlike numeric and topology models have equal
semantics.

Ordinary Boolean and boundary-role APIs then still constructed the complete
audit trail used by their `*_with_evidence` counterparts and immediately dropped
it. Fragment emission and chain assembly now share one topology core with
separate lean and evidence-retaining materialization. The ordinary consuming
path moves chains into loops and loops into contours without allocating
per-fragment provenance, and ordinary boundary nesting assigns material/hole
roles without retaining per-contour sample copies. Evidence-bearing methods keep
the existing evidence and blockers unchanged. The Boolean fuzz target now
differentially compares ordinary, evidence-bearing, and prepared results for all
four operations.

Exact scalar ordering also returns directly from two exposed rational carriers
instead of routing them through general symbolic predicate refinement. The
curve-string x scheduler likewise reuses exact rational endpoints when present,
falling back to certified outward dyadic intervals for other representations.
Sparse schedules are capped at 1,048,576 materialized pairs; larger or dense
cases retain the authoritative flat scan rather than risking quadratic schedule
storage. In paired local runs, the rational endpoint path reduced the 64-, 128-,
and 512-segment prepared sparse intersection lanes by approximately 7.7%, 5.8%,
and 3.3%, respectively.

Against the preceding checkpoint, the stripped comparative Callgrind sweep fell
from 53,745,141 to 48,341,067 instructions (10.05%), while the one-cell
all-operation pathological comparison fell from 159,422,324 to 142,864,302
instructions (10.39%). In the matching 15-sample run, star64 intersection fell
from 1.431 to 1.170 ms/iter (18.2%) and rectangle union from 24.758 to
17.717 us/iter (28.4%); offset and NURBS lanes remained within run-to-run noise.
The 67-cell, 100 MiB finite exact Boolean tier fell from about 277 to 181 ms
(34.6%) with all 268 operation/cell results still decided. The updated
AddressSanitizer fuzz target completed 1,000 runs at 5,693 coverage points and
15,929 feature edges without a failure.

Large sparse contour pairs still rescanned the complete minimum-x prefix for
every first-operand segment. Expired intervals therefore accumulated into a
quadratic exact-comparison workload even when each segment overlapped only a
constant number of neighbors. Pairs with at least 16,384 Cartesian segment
pairs now use a balanced interval index when an eight-by-eight exact sample has
at most one-eighth x overlap. Each node stores only the source position whose
box has the greatest certified maximum x in that subtree. Queries prune a
subtree only when its exact minimum or retained maximum proves separation;
unknown boxes remain candidates, unorderable maxima retain the previous scan,
and candidate indices are restored to source order before intersection.

A diagonal-ribbon benchmark keeps whole-contour boxes overlapping while local
segment boxes stay disjoint. Against the prior ordered-prefix scan, the 64-,
128-, and 512-rung lanes (130, 258, and 1,026 closed-contour segments) improved
by approximately 35%, 59%, and 87%, respectively. Temporary index and candidate
storage remain linear in the second contour rather than in the Cartesian pair
count. Prepared contours with at least 128 retained segment boxes cache the
same index for repeated queries.
The ordinary star64 comparative lane remains below the indexed crossover and
its complete Callgrind sweep stayed effectively flat (48,341,067 versus
48,384,838 instructions).

Ordinary region splitting still called the evidence-bearing builder and cloned
every exact source endpoint, parameter range, and output fragment into
provenance that only `*_with_evidence` callers observe. Splitting now shares one
four-role contour traversal with optional evidence retention. Lean callers keep
only the fragment inventory; evidence-bearing calls retain the same successful
and partial-blocker evidence. The ordinary Boolean pipeline also consumes its
completed selection when no shared boundary needs resolution instead of
cloning it out of a evidence wrapper.

The same profile found two repeated exact-proof walks. `ContourSplitMarkers`
already guarantees endpoint coverage, strict parameter order, unique inserted
events, and source incidence, so fragment materialization now trusts that
private invariant instead of re-comparing every adjacent parameter and point.
The proper-line crossing proof likewise certifies and applies each transition
in one traversal. Its deliberately narrow two-contour index now uses two
segment-indexed vectors instead of a general `BTreeMap`; missing, duplicate,
non-identical, or non-closing transitions still reject the proof and fall back
to canonical per-fragment classification.

Against the preceding checkpoint, the stripped one-iteration comparative
Callgrind sweep fell from 48,384,819 to 45,721,981 instructions (5.50%). The
corresponding allocator-path instruction cost fell from 2,177,445 to 1,863,759
(14.4%), and `Real` clone instructions from 958,985 to 829,529 (13.5%). In the
final 15-sample wall-time run, rectangle union measured 16.127 us/iter versus
18.014 us/iter (10.5% lower), and star64 intersection measured 1.132 ms/iter
versus 1.235 ms/iter (8.4% lower). Offset and NURBS evaluation remained within
run-to-run noise, and all comparative checksums were unchanged. The
`HYPERCURVE_COMPARE_GROUP` filter now isolates named lanes without changing the
default complete peer sweep. The AddressSanitizer differential Boolean fuzzer
completed 1,000 runs at 5,646 coverage points and 15,821 feature edges without
a failure.

The retained minimum-x order previously ran an exact comparison throughout its
sort and repeated all four exact interval comparisons while scanning every
prefix candidate. It now sorts one finite `f64` preview per source box, then
certifies every adjacent ordering exactly. A rounded collision or misordering
retries an allocation-free exact unstable sort; an order that still cannot be
certified retains the flat scan. The preview therefore schedules work but never
decides topology. A regression places two exact rationals below one binary64 ulp
in reverse source order and proves that exact certification recovers their true
order.

Dense certified contour pairs with at least 4,096 Cartesian pairs also retain
exact ranks for second-operand maximum x, minimum y, and maximum y. Four exact
binary-search cuts per first segment replace the repeated interval predicates
inside the minimum-x prefix. Equal interval boundaries remain candidates,
unknown boxes bypass the ranks, any uncertain sort or partition falls back to
the previous scan, and source pair order is restored before intersection. The
schedule stores four `u32` cuts per first segment and three `u32` ranks per
second segment (16 and 12 bytes respectively); each coordinate sort reuses only
16 transient bytes per second segment. A dense differential regression includes
x/y edge contacts and an unknown box and matches the authoritative flat filter.

Against the preceding checkpoint, the isolated debug-info Callgrind star64 run
fell from 43,543,161 to 42,351,387 instructions (2.74%), while exact
`compare_reals` calls fell from 30,165 to 18,264 (39.5%). In the paired
11-sample, 500-iteration release run, star64 intersection fell from 1.074 to
1.049 ms/iter (2.32%), with unchanged checksums. The all-feature and
no-default-feature test matrices, warnings-as-errors Clippy and rustdoc gates,
and both candidate-schedule differential regressions passed. The
AddressSanitizer differential Boolean fuzzer completed 1,000 runs at 5,647
coverage points and 15,809 feature edges without a failure; LeakSanitizer alone
was disabled because the sandbox executes under ptrace.

The exact-rational line kernel now classifies each parametric quotient against
the closed unit interval from its borrowed numerator and signed denominator.
An exact miss returns before constructing either quotient, and retained
interior/endpoint evidence avoids repeating the same comparisons after a hit.
Symbolic and mixed carriers keep the existing quotient, ordering, and
uncertainty path. Intersection points reuse the already-computed line delta in
the affine evaluation, and a direct borrowed-carrier check replaces the full
four-coordinate structural-fact cache when selecting this rational fast path.

The dense candidate schedule also passes its certified AABB-overlap proof into
the private segment-pair kernel. Decided ranked candidates therefore avoid two
redundant endpoint-box tests; candidates involving an unknown box still use the
complete public intersection path. Finally, module-private event collectors now
construct their result directly after normalizing every appended relation,
while the public constructor continues to validate arbitrary event vectors.

Against the preceding checkpoint, the isolated debug-info Callgrind star64 run
fell from 42,351,387 to 28,656,205 instructions (32.34%). Exact
`compare_reals` calls fell from 18,264 to 10,527 (42.36%), allocator calls from
27,290 to 22,742 (16.67%), and deallocator calls from 28,101 to 23,079
(17.87%). The exact divisions in the line-pair profile fell from 1,074 to 276.
In paired release measurements, the 11-sample star64 median fell from 1.049 to
0.700 ms/iter (33.31%), and rectangle union fell from 16.127 to 14.563
us/iter (9.70%), with unchanged checksums. The all-feature and
no-default-feature test matrices, format, warnings-as-errors Clippy and rustdoc
gates, and candidate-proof differential regression passed. The
AddressSanitizer differential Boolean fuzzer completed 1,000 runs at 5,732
coverage points and 16,121 feature edges without a failure; LeakSanitizer alone
was disabled under ptrace.

Dense decided line candidates now reuse the ranked AABB crossover to run a
bounded exact-sign orientation filter before arbitrary-precision intersection
algebra. Exact dyadic coordinates use retained certified `f64` determinant
signs; inconclusive rational coordinates retry a checked homogeneous `i128`
filter. A strict same-side proof rejects the finite segment pair, two opposite-
side proofs certify a proper crossing, and every zero, overflow, symbolic, or
otherwise inconclusive case falls through to the existing exact kernel. The
filter is disabled for edge-preview policy and for small/public line pairs, so
it neither changes tolerance semantics nor taxes the measured rectangle path.

On star64, all 537 dense candidates were decided by the bounded filter: 399
strictly separated pairs returned without an exact determinant and 138 proper
crossings carried their nonzero/interior proof into parameter construction.
Exact line-pair determinant materializations fell from 1,272 to 414 (67.45%).
Against the preceding checkpoint, the isolated debug-info Callgrind run fell
from 28,656,205 to 25,735,959 instructions (10.19%), allocator calls from
22,742 to 16,941 (25.51%), and deallocator calls from 23,079 to 17,278
(25.14%). A paired 15-sample, 1,000-iteration release run fell from 0.707 to
0.615 ms/iter (13.05%); the rectangle Callgrind lane also fell slightly from
1,275,560 to 1,271,514 instructions, and all checksums were unchanged. Both
feature-mode test matrices, format, warnings-as-errors Clippy and rustdoc, and
the certified/fallback differential regressions passed. The AddressSanitizer
Boolean differential fuzzer completed 1,000 runs at 5,743 coverage points and
16,123 feature edges without a failure; LeakSanitizer alone remained disabled
under ptrace.

Exact contour-multiset identity now uses pointer identity first and cached exact
signed-area magnitude second. Unequal cached rational magnitudes or denominators
prove that two contours cannot have the same boundary, while equal areas and
oppositely oriented boundaries still fall through to the complete cyclic exact
boundary comparison. Split line fragments also share one retained source-support
allocation per source segment instead of independently rebuilding an equivalent
support for every child. On star64 this prepared 234 supports for 510 fragments,
eliminating 276 allocations while preserving each composed source range.

Against the preceding checkpoint, the isolated debug-info Callgrind star64 run
fell from 25,735,959 to 25,085,971 instructions (2.53%), allocator calls from
16,941 to 16,101 (4.96%), and deallocator calls from 17,278 to 16,438 (4.86%).
The rectangle control lane fell from 1,271,514 to 1,255,584 instructions (1.25%).
Interleaved 11-sample, 1,000-iteration release runs remained centered near 0.60
ms/iter amid scheduler noise, with unchanged checksums. Both feature-mode test
matrices, format, warnings-as-errors Clippy and rustdoc, and focused cached-area
and shared-support regressions passed. The AddressSanitizer Boolean differential
fuzzer completed 1,000 runs at 5,771 coverage points and 16,223 feature edges
without a failure; LeakSanitizer alone remained disabled under ptrace.

All-line contour area now accumulates the doubled shoelace sum with the existing
exact multiply/subtract kernel and performs the common division by two once
after the complete edge walk. Mixed line/arc contours retain their existing
per-segment contribution path. A 1,024-edge subdivided exact rectangle
regression matches its 65,536-unit closed-form area.

On star64 this removes 126 exact divisions across the two 64-edge input
contours. Two interleaved 11-sample, 1,000-iteration release comparisons lowered
the combined median from 0.589 to 0.582 ms/iter (1.29%); allocator calls fell
from 16,101 to 15,843 (1.60%) and deallocator calls from 16,438 to 16,180
(1.57%), with unchanged checksums. Rectangle release throughput remained
neutral near 14.17 us/iter, while its Callgrind lane fell from 1,255,584 to
1,252,683 instructions (0.23%). The complete star64 Callgrind lane increased
from 25,085,971 to 25,268,873 instructions (0.73%): retaining the fast dyadic
multiply/subtract schedule costs more simple instructions than aggregate
determinant variants, but avoids their measured 7--10% throughput regression.
Both feature-mode test matrices, format, warnings-as-errors Clippy and rustdoc
passed. The AddressSanitizer Boolean differential fuzzer completed 1,000 runs
at 5,775 coverage points and 16,241 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Prepared region Boolean convenience queries now enter the same authoritative
lean arrangement pipeline as ordinary region queries while supplying their
already-built prepared point and winding classifiers. Evidence-bearing prepared
queries select the same shared pipeline with evidence retention enabled. This
removes a separate 266-line prepared orchestration copy whose convenience path
always paid evidence construction, and prevents the two implementations from
drifting apart again. The comparative polygon lanes now time both ordinary and
prepared Hypercurve and verify that their boundary sizes match before sampling;
preparation remains outside the timed operation.

In paired 11-sample, 1,000-iteration release runs, prepared star64 intersection
fell from 1.275 to 0.565 ms/iter (55.7%) and became 3.7% faster than the 0.587
ms ordinary lane in the same run. Prepared rectangle union fell from 25.83 to
12.43 us/iter (51.9%). The complete one-iteration comparative Callgrind process,
including both Hypercurve lanes and all competitors, fell from 51,897,089 to
40,095,332 instructions (22.7%); allocator calls fell from 30,978 to 22,784
(26.5%) and deallocator calls from 31,805 to 23,345 (26.6%). Whole-process DHAT
traffic fell from 9,108,171 bytes in 35,871 blocks to 7,528,699 bytes in 26,165
blocks, while peak heap fell from 1,782,938 to 1,144,334 bytes (35.8%). The
production and benchmark diff removes a net 157 lines before this note.

Both feature-mode test matrices, format, warnings-as-errors Clippy and rustdoc
passed, including prepared/direct adversarial polygon parity and evidence
evidence regressions. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,304 coverage points and 15,624 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace. The prepared star64 lane is
still about 20.5 times slower than `cavalier_contours`, so the large-curve goal
remains open despite this API-wide correction.

Ordinary and prepared boundary-loop and boundary-contour convenience queries
now reuse that same once-visiting arrangement pipeline. They previously rebuilt
contact evidence and then classified fragments individually, making adjacent
public APIs 9--19 times slower than region materialization. One shared output
selector now preserves extracted loop provenance when loops are requested and
unwraps contours directly for contour or region callers. The comparative suite
times all six Hypercurve result shapes and verifies their boundary sizes before
sampling. The production and benchmark diff removes a net 551 lines before this
note.

On star64, ordinary contours fell from 10.033 to 0.579 ms/iter (94.2%),
ordinary loops from 9.632 to 0.586 ms/iter (93.9%), prepared contours from
5.516 to 0.551 ms/iter (90.0%), and prepared loops from 5.212 to 0.548 ms/iter
(89.5%). Rectangle controls improved by 57--67%, while ordinary and prepared
region throughput remained neutral within 0.7%. Across the complete expanded
one-iteration comparative process, Callgrind instructions fell from
1,203,208,210 to 107,825,800 (91.0%), allocator calls from 540,759 to 64,523
(88.1%), and deallocator calls from 560,784 to 65,883 (88.3%). DHAT traffic
fell from 67,974,402 bytes in 753,945 blocks to 23,670,444 bytes in 76,208
blocks; peak live heap fell from 1,321,910 to 1,144,334 bytes (13.4%).
Both feature-mode test matrices, format, warnings-as-errors Clippy and rustdoc
passed. The AddressSanitizer Boolean differential fuzzer completed 1,000 runs
at 5,307 coverage points and 15,628 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Lean Boolean boundary materialization now consumes its certified split output,
selection, emitted fragments, and assembled chains. Provenance, source ranges,
segments, and reversed segments therefore move through the once-visiting path
instead of being cloned at each representation boundary. Evidence-bearing and
public borrowed APIs retain their previous evidence-preserving behavior, while
both assembly modes share one endpoint-index traversal. The comparative runner
also accepts `HYPERCURVE_COMPARE_IMPL` so individual implementations can be
profiled without timing the other validated lanes.

Against the preceding checkpoint, paired five-sample, 500-iteration star64
release medians improved across all six Hypercurve result shapes: ordinary
regions by 13.88%, contours by 12.15%, loops by 12.59%, prepared regions by
9.25%, prepared contours by 13.66%, and prepared loops by 11.51%. A paired
seven-sample, 10,000-iteration rectangle control improved from 16.116 to 13.466
us/iter (16.44%), with unchanged checksums. Across 100 selected ordinary
star64 operations plus fixture validation, Callgrind instructions fell from
587,900,022 to 568,715,069 (3.26%). DHAT allocation traffic fell from
139,890,257 to 123,804,705 bytes (11.50%), peak live heap from 1,144,344 to
991,924 bytes (13.32%), reads by 4.02%, and writes by 11.06%.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,429 coverage points and 15,904 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Ranked and indexed AABB traversal now compares wide exact dyadic coordinates
through Hyperreal's lossless binary64 view before falling back to arbitrary-
precision rational ordering. This is a certified representation fast path, not
an approximation: only exactly representable dyadics commit an ordering. The
dispatch stays inside the large-contour broad phase and keeps small native-word
rationals on their cheaper existing comparator.

Against the preceding checkpoint, a paired 11-sample, 1,000-iteration ordinary
star64 run fell from 0.532 to 0.513 ms/iter (3.67%). In seven-sample runs,
ordinary contours, prepared regions, prepared contours, and prepared loops
improved by 3.73--4.84%; the ordinary loop median was neutral within 0.17%.
The 10,000-iteration rectangle control remained neutral at 12.922 versus 12.902
us/iter. Across 100 selected ordinary star64 operations plus fixture
validation, Callgrind instructions fell from 568,715,069 to 558,453,381
(1.81%). DHAT allocation bytes, block counts, and peak heap were identical;
heap reads fell by 6.46% while writes differed by 0.17%.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,520 coverage points and 16,089 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Contour fragment materialization now reserves the exact adjacent-marker upper
bound before rebuilding geometry, and Boolean emission reserves the exact
number of already-selected directed fragments. Both counts come from retained
work that the corresponding passes must consume anyway; no predicate,
classification, or output ordering changes. A second experiment that
pre-counted events for every per-segment marker bin regressed star64 by 3.1%
and was removed.

Against the preceding checkpoint, a paired 15-sample, 1,000-iteration ordinary
star64 run fell from 0.537 to 0.511 ms/iter (4.70%). In seven-sample runs, the
other five ordinary and prepared result-shape lanes improved by 4.88--8.70%.
The 10,000-iteration rectangle control remained neutral at 12.597 versus
12.601 us/iter. Across 100 selected ordinary star64 operations plus fixture
validation, Callgrind instructions fell from 558,453,381 to 553,363,425
(0.91%). DHAT allocation traffic fell from 123,804,705 bytes to 88,575,169
bytes (28.46%), heap reads by 10.55%, and writes by 20.68%; block count fell by
0.38% and peak heap remained neutral within 0.12%.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,524 coverage points and 16,097 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Lean region traversal now consumes two invariants already certified by its
retained carriers. Internally constructed split markers no longer replay source
incidence validation, while normalized point-event kinds route only endpoint
events into the endpoint-contact index and carry strict-interior proof directly
into line-crossing winding propagation. Proper crossings therefore avoid four
redundant exact comparisons against zero and one. A marker-bin pre-count and an
in-place single-chain permutation were also profiled; their extra event walk
and large-record memory traffic regressed release throughput, so both were
removed. The retained production diff removes a net three lines before this
note.

Against the preceding checkpoint, a paired 21-sample, 1,000-iteration ordinary
star64 run fell from 0.501 to 0.496 ms/iter (1.06%). The other five ordinary and
prepared result-shape lanes improved by 0.80--1.87% in their stable paired
runs. A 21-sample, 10,000-iteration rectangle control fell from 12.703 to
12.239 us/iter (3.65%). Across 100 selected ordinary star64 operations plus
fixture validation, Callgrind instructions fell from 553,363,425 to 544,236,152
(1.65%). DHAT allocation bytes, block counts, and peak heap were identical;
heap reads fell by 2.49% and writes by 0.55%.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,517 coverage points and 16,109 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Contour intersection storage now boxes the uncommon overlap payload instead of
inflating every event to the size of its largest variant. On the measured
64-bit target this reduces `ContourIntersection` from 648 to 224 bytes, the
size of its common point payload. Exact overlap geometry, parameter ranges, and
source evidence are unchanged; only the enum representation differs.

Across 100 selected ordinary star64 operations plus fixture validation,
Callgrind instructions fell from 543,939,023 to 540,965,420 (0.55%). DHAT
allocation traffic fell from 88,575,169 to 82,949,537 bytes (6.35%), peak live
heap from 993,076 to 965,940 bytes (2.73%), reads by 1.11%, and writes by 4.20%,
with the same block count. Twenty-iteration Callgrind controls improved all six
ordinary/prepared region, contour, and loop lanes by 0.19--0.59%; a 1,000-
iteration rectangle-union control improved by 0.68%.

Focused line- and arc-overlap tests preserve their exact event evidence. The
single line-overlap control added 516 instructions and 512 total allocated
bytes, while the arc-overlap control added 1,002 instructions but allocated
21,558 fewer bytes. Both complete feature-mode test matrices, format,
warnings-as-errors Clippy and rustdoc passed. The AddressSanitizer Boolean
differential fuzzer completed 1,000 runs at 4,728 coverage points and 8,665
feature edges without a failure; LeakSanitizer alone remained disabled under
ptrace.

Lean contour and region results now consume endpoint-chain indices directly
into their final `Segment2` contour vectors. The previous shared path first
materialized a 776-byte `DirectedBooleanFragment` for every ordered chain
position, only to discard its provenance fields while converting the chain to
the requested contour. Loop-returning APIs retain that full carrier, and the
ambiguous tangent-order fallback retains the established chain traversal.

Against the compact-event checkpoint, the 100-operation ordinary star64
Callgrind lane fell from 540,965,420 to 539,699,496 instructions (0.23%). DHAT
allocation traffic fell from 82,949,537 to 71,190,377 bytes (14.18%), reads by
8.15%, and writes by 14.85%; allocation blocks fell by 210 and peak live heap
was unchanged. The four ordinary/prepared contour and region lanes improved by
0.22--0.24% in focused Callgrind runs, while the two loop lanes stayed neutral
within 0.03%. A 1,000-iteration rectangle-union control improved by 1.58%.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,145 coverage points and 12,378 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Lean region splitting now keeps endpoint-only marker bins implicit. The first
retained event touching a segment materializes that segment's endpoint markers
and reserves room for the event in the same required merge visit; untouched
segments move directly into the fragment output with the implicit full source
range. Public marker constructors still expose explicit endpoint evidence, and
the lean path adds neither an event pre-count nor a second predicate pass.

Against the direct-contour checkpoint, the 100-operation ordinary star64
Callgrind lane fell from 539,699,496 to 528,231,186 instructions (2.13%). DHAT
allocation traffic fell from 71,190,377 to 66,807,657 bytes (6.16%), allocation
blocks from 417,533 to 403,837 (3.28%), reads by 1.93%, and writes by 5.10%; peak
live heap was unchanged. Focused Callgrind runs improved all six ordinary and
prepared result lanes by 2.00--2.15%, and a 1,000-iteration rectangle-union
control improved by 5.33%.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,269 coverage points and 13,981 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Lean contour output now builds its endpoint schedule from borrowed selected
fragments before consuming the certified split. When endpoint adjacency is
unambiguous, it moves only the oriented `Segment2` values into the final
contours instead of first allocating the 776-byte directed provenance carrier.
Loop-returning APIs retain the full carrier, and duplicate starts or other
branch topology still fall through to the existing tangent-ordered traversal.

Against the implicit-marker checkpoint, the 100-operation ordinary star64
Callgrind lane fell from 528,231,186 to 524,245,052 instructions (0.75%). DHAT
allocation traffic fell from 66,807,657 to 63,111,657 bytes (5.53%), reads by
4.05%, and writes by 4.15%; allocation blocks were unchanged and peak live heap
fell by eight bytes. Focused Callgrind controls improved ordinary/prepared
region and contour lanes by 0.69--0.78%, the two loop lanes by 0.07--0.13%, and
a 1,000-iteration rectangle-union control by 2.66%.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,366 coverage points and 14,475 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Both Boolean classifiers now reserve their already-computed exact source
fragment count. This replaces geometric growth with one classification-vector
allocation while leaving classification order, evidence, and fallback behavior
unchanged.

Against the borrowed-endpoint checkpoint, the 100-operation ordinary star64
Callgrind lane fell from 524,245,052 to 523,766,373 instructions (0.09%). DHAT
allocation traffic fell from 63,111,657 to 62,125,545 bytes (1.56%), allocation
blocks from 403,837 to 403,195, reads by 0.41%, writes by 1.01%, and peak live
heap by 1,152 bytes. Twenty-iteration Callgrind controls improved all six
ordinary/prepared region, contour, and loop lanes by 0.06--0.12%; a 1,000-
iteration rectangle-union control improved by 0.72%.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,412 coverage points and 15,070 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Internally classified Boolean fragments now retain the proof supplied by their
complete keyed source traversal. The public constructor still validates and
sorts arbitrary owner evidence, while the two internal classifiers no longer
allocate and sort a duplicate owner vector after visiting every certified
fragment exactly once. A debug assertion guards the complete-count invariant.

Against the preallocation checkpoint, the 100-operation ordinary star64
Callgrind lane fell from 523,766,373 to 522,336,104 instructions (0.27%). DHAT
allocation traffic fell from 62,125,545 to 61,560,585 bytes (0.91%), reads by
1.08%, and writes by 0.67%; allocation blocks fell by 107 and peak live heap was
unchanged. Twenty-iteration Callgrind controls improved all six ordinary and
prepared result lanes by 0.26--0.28%, and a 1,000-iteration rectangle-union
control improved by 0.67%.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,391 coverage points and 15,235 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Certified all-line contours whose endpoint coordinates have lossless binary64
views now use transient 32-byte boxes for their contour-pair sweep. Binary64 is
used only to order and reject candidates after exact representability has been
proved; every retained pair still reaches the exact line relation kernel.
Arcs, non-representable coordinates, preview mode, and exact-symbolic mode keep
the general exact-box path. Keeping the compact boxes operation-local avoids
increasing retained or peak heap as source complexity grows.

Against the certified-traversal checkpoint, the 100-operation ordinary star64
Callgrind lane fell from 522,336,104 to 454,215,855 instructions (13.04%). DHAT
allocation traffic fell from 61,560,585 to 58,966,409 bytes (4.21%), reads by
21.05%, and writes by 7.61%; allocation blocks fell by 624 and peak live heap
was unchanged. Twenty-iteration Callgrind controls improved the three ordinary
region, contour, and loop lanes by 11.28--11.35% and the three prepared lanes
by 1.50--1.53%. A 1,000-iteration rectangle-union control improved by 10.47%.

In an 11-sample, 500-iteration release comparison, ordinary star64 intersection
measured 0.401 ms/iter, versus 0.028 ms for `cavalier_contours`, 0.035 ms for
`i_overlay`, and 0.036 ms for `geo`. The exact Hypercurve path therefore remains
behind approximate competitors, but has fallen from the previously recorded
0.496 ms/iter before the four latest memory and traversal checkpoints.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,512 coverage points and 15,911 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Strict line crossings now reuse the sign carried by their certified support
relation when building the winding-propagation index. The fallback still forms
the exact determinant for symbolic or inconclusive inputs, while the common
lossless-binary64 path avoids reconstructing and normalizing that determinant.
Validated crossing and tangent events also insert split markers only among the
strictly interior slots that their event kind certifies, omitting impossible
comparisons against parameters zero and one.

Against the compact exact-dyadic sweep checkpoint, paired twenty-iteration
Callgrind runs improved all six ordinary and prepared region, contour, and loop
lanes by 2.89--3.07%. DHAT allocation traffic fell from 58,966,409 to
58,029,945 bytes (1.59%), allocation blocks from 402,464 to 383,739 (4.65%),
reads by 3.75%, and writes by 1.62%; peak live heap remained 964,780 bytes. In
an 11-sample, 500-iteration release comparison, ordinary star64 intersection
measured 0.393 ms/iter, versus 0.027 ms for `cavalier_contours`, 0.034 ms for
`i_overlay`, and 0.036 ms for `geo`.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,575 coverage points and 16,103 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Prepared contour intersections now select the same compact, lossless-binary64
line sweep as ordinary certified contours before falling back to their cached
general `Aabb2` path. The compact bounds remain operation-local: a rejected
prototype retained them in every prepared contour, increasing live heap even
when no intersection query followed preparation.

Against the certified-crossing-reuse checkpoint, twenty-iteration Callgrind
runs reduced the prepared region, contour, and loop lanes by 5.25--5.28%. The
three ordinary controls shifted by 0.70--0.71% despite having no changed call
path. Prepared-lane DHAT reads fell from 46,826,598 to 43,727,644 bytes (6.62%).
Building the transient bounds per query increased total allocation from
15,284,369 to 15,609,641 bytes (2.13%), allocation blocks from 101,701 to
103,987, and peak live heap from 964,789 to 971,829 bytes; unlike the rejected
retained cache, that storage is released with the query and does not enlarge
every prepared object. In an 11-sample, 500-iteration release comparison,
prepared star64 region intersection measured 0.394 ms/iter, versus 0.028 ms for
`cavalier_contours`, 0.035 ms for `i_overlay`, and 0.036 ms for `geo`.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,543 coverage points and 15,891 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Strict-crossing winding propagation now asks only which side of the first
oriented support contains the second segment's start. The normalized event has
already certified an interior crossing, so the previous four-orientation
segment relation repeated three unnecessary side tests. The narrow binary64 or
checked-word predicate retains the exact direction-determinant fallback when
its filter is inconclusive.

Against the prepared compact-sweep checkpoint, paired twenty-iteration
Callgrind runs improved all six ordinary and prepared region, contour, and loop
lanes by 0.887--0.899%. Prepared-lane DHAT allocation bytes, blocks, and peak
live heap were unchanged; reads fell from 43,727,644 to 43,503,578 bytes
(0.51%). An 11-sample, 500-iteration release comparison measured ordinary
star64 intersection at 0.397 ms/iter, versus 0.028 ms for
`cavalier_contours`, 0.035 ms for `i_overlay`, and 0.036 ms for `geo`.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,553 coverage points and 15,950 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

The strict-crossing winding index now borrows exact parameters from its
retained region event set instead of cloning a `Real` into every transition.
Its lifetime makes that dependency explicit. Per-segment bins reserve the two
records that cover the common case before retaining geometric growth for denser
crossing distributions, cutting unused first-allocation capacity in half.

Against the narrow-crossing-predicate checkpoint, twenty-iteration Callgrind
runs improved all six ordinary and prepared result lanes by 0.19--0.23%.
Prepared-lane DHAT allocation fell from 15,609,641 to 15,208,745 bytes (2.57%),
peak live heap from 971,829 to 956,917 bytes (1.53%), reads by 0.32%, and writes
by 0.71%. The smaller initial bins added 54 growth blocks across twenty
operations (0.05%) where a few star edges exceeded two crossings, without an
instruction regression.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,544 coverage points and 15,946 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

The strict-crossing winding index now stores each contour's transitions in one
segment-sorted flat vector instead of allocating an empty vector for every
source segment and a second allocation for every crossing-bearing segment.
Exact duplicate-parameter validation remains grouped by source segment, and
fragment lookup uses the retained segment order without cloning any `Real`.
The refactor also removes five lines from the implementation.

Against the borrowed-parameter checkpoint, twenty-iteration Callgrind runs
improved all six ordinary and prepared region, contour, and loop lanes by
0.426--0.455%. Prepared-lane DHAT allocation fell from 15,208,745 to
15,114,582 bytes (0.62%), allocation blocks from 104,041 to 101,881 (2.08%),
and peak live heap from 956,917 to 953,506 bytes (0.36%). The flat construction
and lookup raised reads by 0.41% and writes by 0.08%, while still reducing
executed instructions in every measured path.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,552 coverage points and 16,010 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Internal region splitting now counts each event's per-segment marker upper
bound before materializing markers. Every touched bin reserves that exact bound
once and installs its source endpoints once, replacing repeated source-segment
lookups and geometric marker-vector growth. Multiple contour-pair contributions
retain the same incremental merge and exact duplicate suppression behavior.

Against the flat-crossing-index checkpoint, twenty-iteration Callgrind runs
improved all six ordinary and prepared region, contour, and loop lanes by
0.239--0.257%. Prepared-lane DHAT allocation fell from 15,114,582 to
14,796,630 bytes (2.10%), reads by 0.11%, and writes by 0.08%. Allocation blocks
remained 101,881 and peak live heap remained 953,506 bytes because the temporary
per-segment count arrays replace the eliminated marker slack during that phase.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,567 coverage points and 16,090 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Non-evidence strict-crossing Booleans now split into a private compact fragment
carrier that retains only source index, exact parameter range, and native
segment geometry. The public and evidence-bearing `ContourFragment` continues to
carry both source endpoints. Direct contour output moves compact segments into
the result, while loop output clones original source endpoints only for selected
fragments. Overlaps, endpoint contacts, curved inputs, incomplete winding
proofs, and evidence requests retain the general provenance-rich pipeline.

Against the exact-marker-preallocation checkpoint, twenty-iteration Callgrind
runs improved all six ordinary and prepared large-star lanes. Region and
contour outputs fell by 2.006--2.036%; provenance-retaining loop outputs fell by
1.470--1.478%. Prepared-region DHAT allocation fell from 14,796,630 to
13,558,086 bytes (8.37%), peak live heap from 953,506 to 907,666 bytes (4.81%),
reads by 5.78%, and writes by 7.86%. Allocation blocks fell by 27. The prepared
loop lane reduced allocation from 16,244,460 to 15,005,916 bytes (7.62%), peak
live heap by 4.81%, reads by 4.53%, and writes by 6.72%.

In an 11-sample, 500-iteration release comparison, ordinary star64 intersection
measured 0.407 ms/iter, versus 0.028 ms for `cavalier_contours`, 0.035 ms for
`i_overlay`, and 0.037 ms for `geo`. Exact point construction remains the
dominant performance boundary despite the smaller Boolean carrier.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,787 coverage points and 16,413 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

Medium dense exact-line contour pairs now retain their certified proper-
crossing candidates before constructing exact points. The retained pair list
provides the exact final event capacity, and its orientation certificate goes
straight to the nonparallel line kernel instead of repeating the support test.
Any inconclusive, collinear, endpoint, or otherwise non-point relation falls
back to the original exact sweep. The schedule is limited to 256--16,384 raw
segment pairs so its compact scratch carrier stays bounded and large sparse or
degenerate contours retain the one-pass path.

Against the compact-fragment checkpoint, two-point Callgrind slopes improved
all six ordinary and prepared star64 region, contour, and loop lanes by
8.20--8.36%. The 1,000/2,000-operation prepared rectangle control increased by
0.44%; the complete sparse-intersection benchmark shifted by 0.08%, including
fixed process startup. Prepared-region DHAT allocation fell from 13,558,086 to
13,132,998 bytes (3.14%), allocation blocks from 101,854 to 101,503 (0.34%),
peak live heap from 907,666 to 903,634 bytes (0.44%), reads by 8.85%, and writes
by 7.15%.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,795 coverage points and 16,410 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

The compact strict-crossing Boolean path now carries line endpoints, exact
source range, source index, and shared support directly instead of embedding a
full mixed `Segment2` in every candidate fragment. Interior classification and
endpoint chaining consume that representation in place; full line geometry is
materialized only for fragments retained in the result. Unsplit lines continue
to clone their authored geometry, while curved or otherwise unsupported inputs
fall through to the general provenance-rich pipeline.

Against the retained-crossing checkpoint, two-point Callgrind slopes improved
all six ordinary and prepared star64 region, contour, and loop lanes by
4.30--4.34%. The 1,000/2,000-operation prepared rectangle control improved by
1.23%, while the complete sparse-intersection benchmark differed by only 37
instructions including fixed process startup. Prepared-region DHAT allocation
fell from 13,132,998 to 11,849,958 bytes (9.77%), peak live heap from 903,634
to 857,346 bytes (5.12%), reads by 3.76%, and writes by 7.03%; allocation block
count remained 101,503.

In an 11-sample, 500-iteration release comparison, ordinary star64 intersection
measured 0.349 ms/iter, versus 0.027 ms for `cavalier_contours`, 0.035 ms for
`i_overlay`, and 0.036 ms for `geo`. Exact point and event construction remain
the dominant gap to approximate competitors after fragment materialization was
moved behind selection.

Both complete feature-mode test matrices, format, warnings-as-errors Clippy and
rustdoc passed. The AddressSanitizer Boolean differential fuzzer completed
1,000 runs at 5,803 coverage points and 16,425 feature edges without a failure;
LeakSanitizer alone remained disabled under ptrace.

The public straight-skeleton surface now has a dedicated release/scaling benchmark,
an exact-dispatch trace, and an AddressSanitizer differential fuzzer covering the
trajectory and event predictors, contour construction, native `CurvePath2` dispatch,
orientation reversal, and topology-preserving scale and translation. The first
1,000-run post-change fuzz pass reached 3,243 coverage points and 9,054 feature edges
without a failure; LeakSanitizer remains disabled under ptrace.

General line-event scheduling now retains only the current exact minimum-time group
while visiting candidates, instead of materializing every edge, split, and vertex
candidate and scanning the collection twice. Output nodes and arcs reserve their
known linear topology allowance. Uncommon conic geometry and generated-support
provenance moved behind separately named payloads, so ordinary line/source-bisector
arcs do not carry their maximum enum storage. On the benchmark target,
`StraightSkeletonArc2` fell from 520 to 48 bytes (90.8%), its geometry carrier from
440 to 8 bytes, and its kind carrier from 64 to 24 bytes. Two-point DHAT slopes for
the completed eight-edge concave construction fell from 71,776 bytes in 579 blocks
to 44,960 bytes in 570 blocks per operation: 37.4% less allocation traffic and 1.6%
fewer blocks. The isolated Callgrind lane fell from 1,956,957 to 1,938,966
instructions (0.92%), and a 1,000-iteration release run measured 93.906 us/iter.

The dense exact-dyadic line path now carries its existing input certificate into
the three determinant constructions needed for each retained proper crossing.
Those determinants enter Hyperreal's shift-aligned dyadic reducer directly instead
of repeating generic denominator-shape discovery. Immutable contour clones also
share one lazy compact binary64 AABB array, so ordinary and prepared repeated
queries stop rescanning the same exact coordinates and reallocating identical
broad-phase storage. The cached array remains only a rejection filter; all
topology and output coordinates remain exact.

The end-to-end dispatch harness now includes the actual star64 intersection and
prints rational operand-width statistics. One traced operation reported 465 GCDs:
80 mixed 64/128-bit calls, 80 balanced 128-bit calls, and 305 calls with an
operand wider than 128 bits, peaking at 359 bits. That evidence exposed an
unconditional no-op remainder when balanced two-limb inputs reached Hyperreal in
ascending order; ordering them before the Euclidean tail removes it without
changing the algorithm or canonical result.

Across identical twenty-iteration Callgrind runs, the ordinary star64 region path
fell from 102,991,860 to 99,745,892 instructions (3.15%). Prepared-region DHAT
allocation fell from the preceding 11,849,958 bytes in 101,503 blocks to
11,640,961 bytes in 100,732 blocks (1.76% fewer bytes and 0.76% fewer blocks).
Retaining both 64-segment box arrays raised peak live heap from 857,346 to 861,653
bytes (0.50%); this bounded 4.3 KiB cost replaces repeated allocation and scanning.
A 31-sample, 500-iteration comparison measured ordinary star64 intersection at
327.611 us/iter and prepared boundary-contour output at 318.558 us/iter. The same
run measured 27.959 us for `cavalier_contours`, 36.093 us for `i_overlay`, and
35.929 us for `geo`, so the exact path remains about 11.7 times slower than the
fastest approximate competitor.

The complete all-feature and no-default Hypercurve suites, warnings-as-errors
Clippy, and the all-feature test suites of CSGRS, Hypermesh, Hyperlattice,
Hyperlimit, Hypersolve, and Hyperreal passed. Nightly AddressSanitizer campaigns
completed 1,000 region-Boolean runs at 5,886 coverage points and 16,891 feature
edges and 1,000 straight-skeleton runs at 3,252 coverage points and 9,172 feature
edges without failure; LeakSanitizer alone remained disabled under ptrace.

Retained exact-dyadic proper crossings now construct both parameters and the point
without expanding a general affine expression. Hyperreal first cross-cancels the
stored magnitudes of each dyadic quotient and applies the net power-of-two shift. For
the point, two fused dyadic numerators represent
`origin * denominator + parameter_numerator * delta`; dividing those directly by
the original determinant avoids canonicalizing `parameter * delta` before adding
the origin. The public and fallback paths still return identical canonical `Real`
parameters and coordinates, as checked against the unreserved dense sweep.

Against the retained-box and ordered-GCD checkpoint, twenty-operation Callgrind
fell from 99,745,892 to 81,698,829 instructions (18.09%). Prepared-region DHAT
allocation fell from 11,640,961 to 10,608,905 bytes (8.87%), allocation blocks from
100,732 to 83,760 (16.85%), peak live heap from 861,653 to 776,589 bytes (9.87%),
reads by 13.25%, and writes by 13.02%. The exact trace eliminated all 104 expanded
cross-numerator and cross-denominator division events seen in the first aggregate
prototype.

A 31-sample, 500-iteration release comparison measured ordinary, contour, and loop
star64 outputs at 256.917, 255.464, and 297.433 us/iter. Prepared variants measured
253.557, 254.123, and 298.960 us/iter. The same run measured 27.695 us for
`cavalier_contours`, 38.628 us for `i_overlay`, and 35.682 us for `geo`. The exact
region path is 21.6% faster than the previous 327.611 us checkpoint, but remains
9.28 times slower than the fastest approximate competitor.

Compact exact-line Boolean output now retains lightweight selected-fragment
references through chain assembly and materializes each `Segment2` directly into
its final contour order. The former schedule first allocated every selected segment
in source order, then allocated the same large geometry again while following chain
indices. The once-visiting schedule preserves the same exact endpoints, source
ranges, retained supports, direction, and validated closed-contour constructor.

The accompanying native Hyperreal two-limb GCD tail handles Euclidean quotients one
through four by subtraction before invoking full-width remainder. Together, the
ordinary twenty-operation star64 Callgrind lane fell from 81,698,829 to 77,709,243
instructions (4.88%). Prepared-region DHAT allocation fell from 10,608,905 to
9,608,934 bytes (9.43%); reads fell 6.24% and writes 7.34%, with the same 83,760
blocks and effectively unchanged peak live heap.

A matched 31-sample, 500-iteration comparison measured ordinary region, contour,
and loop outputs at 232.400, 227.763, and 281.681 us/iter. Prepared variants measured
230.817, 225.869, and 276.898 us/iter. `cavalier_contours`, `i_overlay`, and `geo`
measured 28.345, 37.839, and 36.074 us/iter. Ordinary exact region output is 9.54%
faster than the preceding checkpoint and remains 8.20 times slower than the fastest
approximate competitor.

Hyperreal's native two-limb tail now replaces most remaining quotient-five-and-up
compiler-runtime remainders with a non-overshooting high-limb quotient estimate and
an exact residual correction. The twenty-operation ordinary star64 Callgrind lane
fell from 77,709,243 to 76,985,290 instructions (0.93%). Prepared-region DHAT stayed
at 9,608,934 bytes in 83,760 blocks with a 776,618-byte peak; reads fell slightly
from 28,209,578 to 28,207,882 bytes and writes remained 12,912,843 bytes.

In a matched 31-sample, 500-iteration release comparison, ordinary region, contour,
and loop outputs measured 231.336, 224.095, and 272.283 us/iter. Prepared variants
measured 224.789, 224.390, and 271.700 us/iter. `cavalier_contours`, `i_overlay`, and
`geo` measured 28.282, 36.484, and 37.278 us/iter. The ordinary exact path remains
8.18 times slower than the fastest approximate competitor. The optimization and all
production arithmetic are implemented by Hyperreal itself; GMP remains limited to
development-only competitive benchmarks and test oracles.

Line-pair support classification now prepares the four endpoints' certified dyadic
views once and evaluates the reverse orientation only after the first same-side exit
fails. Each inconclusive floating determinant retains the same homogeneous word filter
and arbitrary-precision fallback. In a symbolized twenty-operation star64 trace, calls
to Hyperreal's cached exact-dyadic conversion fell from 62,990 to 47,222 (25.0%). The
matching stripped Callgrind lane fell from 76,985,290 to 76,081,746 instructions
(1.17%). Prepared-region DHAT allocation remained at 83,760 blocks; allocation traffic
was effectively unchanged at 9,608,905 bytes, while reads fell from 28,207,882 to
27,925,250 bytes (1.00%) and writes remained effectively flat.

The matched 31-sample, 500-iteration release comparison measured ordinary region,
contour, and loop output at 225.848, 221.581, and 272.583 us/iter. Prepared variants
measured 222.742, 228.006, and 271.049 us/iter. `cavalier_contours`, `i_overlay`, and
`geo` measured 28.092, 36.044, and 36.004 us/iter. The ordinary exact path is 2.37%
faster than the preceding median and remains 8.04 times slower than the fastest
approximate competitor.

The shared exact-dyadic contour-box cache now also retains its minimum-x segment
order. Repeated Boolean calls previously allocated and sorted the identical index
vector before every dense candidate scan. The cached order uses four-byte indices;
contours beyond that representable range retain their exact boxes and take the
unreserved exact fallback. Against the paired-filter checkpoint, the ordinary
twenty-operation star64 Callgrind lane fell from 76,081,746 to 75,700,486
instructions (0.50%). Prepared-region DHAT allocation fell from 9,608,905 bytes in
83,760 blocks to 9,595,641 bytes in 83,735 blocks. Reads fell 1.94% and writes 1.17%;
retaining the two fixture orders raised peak live heap by 560 bytes (0.07%).

The corresponding 31-sample, 500-iteration run measured ordinary region, contour,
and loop output at 227.693, 226.728, and 289.004 us/iter. Prepared variants measured
224.163, 220.254, and 271.473 us/iter. `cavalier_contours`, `i_overlay`, and `geo`
measured 28.230, 36.503, and 37.340 us/iter. The mixed wall-clock movement remains
within the recent run-to-run range; the deterministic instruction and allocation
reductions justify retaining the shared order. The ordinary exact path remains 8.07
times slower than the fastest approximate competitor.

Hyperreal's certified dyadic quotient now keeps word-sized cross-cancellation and
net binary scaling in its native `u128` reducer, falling back before any overflowing
shift or for an already-wide magnitude. The ordinary twenty-operation star64
Callgrind lane fell from 75,700,486 to 74,297,181 instructions (1.85%). Prepared-
region DHAT allocation fell from 83,735 to 81,575 blocks (2.58%); reads fell 1.39%
and writes 0.52%, with effectively flat allocated bytes. Peak live heap rose by a
bounded 2,560 bytes as more small quotient results entered Hyperreal's canonical
storage cache.

The matched 31-sample, 500-iteration run measured ordinary region, contour, and loop
output at 223.696, 217.832, and 272.180 us/iter. Prepared variants measured 217.497,
216.799, and 267.500 us/iter. `cavalier_contours`, `i_overlay`, and `geo` measured
29.410, 36.466, and 35.752 us/iter. Ordinary exact region output is 1.76% faster than
the preceding median and remains 7.61 times slower than the fastest approximate
competitor in this run. The production quotient branch is implemented entirely by
Hyperreal; GMP/MPFR remains confined to development-only competitive benchmarks and
test oracles and is absent from release dependencies.

Certified proper line crossings now pass their four exact dyadic endpoints to one
bounded Hyperreal kernel. It retains coordinate differences and all three
determinants as signed native words or fixed stack accumulators until constructing
the final two parameters and point. Inputs outside those bounds fall through to the
unchanged arbitrary-precision construction. Against the preceding fused-affine
checkpoint, the twenty-operation star64 Callgrind lane fell from 63,819,545 to
58,160,378 instructions (8.87%). DHAT allocation fell from 8,852,333 bytes in
55,238 blocks to 7,958,405 bytes in 39,275 blocks; reads and writes fell 13.91% and
9.01%, while peak live heap remained effectively flat.

The corresponding 31-sample, 500-iteration release run measured ordinary region,
contour, and loop output at 160.947, 158.426, and 205.317 us/iter. Prepared variants
measured 159.871, 155.453, and 200.508 us/iter. `cavalier_contours`, `i_overlay`, and
`geo` measured 28.084, 36.449, and 37.192 us/iter. Ordinary exact region output is
10.67% faster than the preceding 180.17 us checkpoint and remains 5.73 times slower
than the fastest approximate competitor. The Curvo cubic-offset comparison fixture
also now avoids an inflection at its randomized tessellation probe, eliminating a
nondeterministic too-few-control-points failure without changing either library's
timed implementation.

The retained exact-line candidate scan now carries each already-certified crossing
orientation into region winding propagation instead of recomputing the determinant
after event materialization. Up to 63 signs occupy one optional nonzero machine word
inside the otherwise opaque event set; larger sets take the unchanged exact fallback
without allocating a sidecar. Candidate indices also use the scan's existing
16,384-pair bound, shrinking each temporary record from two machine words to six
bytes. Event equality, debug output, public construction, and exact fallback behavior
remain unchanged.

Against the fused-crossing checkpoint, the symbolized twenty-operation star64
Callgrind lane fell from 58,145,388 to 57,721,161 instructions (0.73%). DHAT allocation
fell from 7,958,405 to 7,925,789 bytes with the same 39,275 blocks; peak live heap
fell slightly to 779,825 bytes. Two order-reversed, CPU-pinned 41-sample runs measured
the new path at 160.047 and 160.254 us/iter versus 161.534 and 163.055 us/iter for the
committed baseline, a 0.9--1.7% wall-time improvement. The exact path remains roughly
5.7 times slower than the latest matched `cavalier_contours` checkpoint.

Compact strict-crossing fragments now borrow exact split points and parameters from
one shared marker owner per contour. Previously every fragment cloned both exact
endpoints and its parameter range out of the already-retained marker bins, then
dropped those duplicates after selection. Unsplit lines now retain only their source
segment index; split fragments retain that index, their marker position, and the
shared source support. Endpoint chaining and winding propagation borrow the original
exact values, while only selected output fragments clone the values required by
`LineSeg2`'s retained support range. This preserves exact coordinates, parameter
ranges, source-support identity, and the existing general fallback.

Against the certified-orientation checkpoint, the 5,000-operation star64 instruction
run fell from 8,758,225,603 to 8,274,445,706 retired instructions (5.52%). Twenty-
operation DHAT allocation fell from 7,925,789 to 6,217,661 bytes (21.55%), reads from
21,541,922 to 19,967,289 bytes (7.31%), and writes from 10,454,460 to 8,257,928 bytes
(21.01%). Allocation blocks rose by 54 (0.14%) because each contour now has one shared
marker-owner allocation; peak live heap remained exactly 779,825 bytes. A final
CPU-pinned 11-sample paired release run measured ordinary exact region output at
144.352 us/iter, 7.87% faster than the 156.674 us post-provenance-removal baseline.
`cavalier_contours`, `i_overlay`, and `geo` measured 28.274, 34.958, and 36.666
us/iter in that run, leaving the exact path 5.11 times behind the fastest approximate
competitor.

The shared exact-dyadic contour cache now retains two endpoint-direction bits per
line. Together with its existing exact binary64 AABB, those bits reconstruct the
directed endpoints without storing four duplicate coordinates. The dense candidate
scan passes these already-proved dyadic views to Hyperreal's paired determinant
filter, avoiding eight repeated scalar-cache loads per candidate. Every floating
sign still uses the conservative determinant error bound; an inconclusive result
continues through the exact homogeneous-word or arbitrary-precision fallback.

Against the shared-marker checkpoint, the 5,000-operation star64 hardware-counter
run fell from 8,274,445,706 to 8,038,736,058 retired instructions (2.85%). The new
twenty-operation DHAT run allocated 6,213,997 bytes in 39,323 blocks, down 0.06% and
six blocks; peak live heap rose by only 176 bytes to 780,001 bytes. Reads rose 0.17%
and writes fell 0.04%. In the post-change profile,
`Real::exact_dyadic_f64_cached` disappeared from the functions above 0.2% self-time.

A 15-sample, 1,000-iteration comparison measured ordinary exact region, contour,
and loop output at 143.792, 136.661, and 176.869 us/iter. Prepared variants measured
138.004, 135.744, and 179.380 us/iter. `cavalier_contours`, `i_overlay`, and `geo`
measured 27.348, 34.722, and 36.379 us/iter. The exact region path therefore remains
5.26 times slower than the fastest approximate competitor.

Both complete feature-mode test matrices, all-target/all-feature warnings-as-errors
Clippy, and warnings-as-errors rustdoc passed. The AddressSanitizer region-Boolean
differential fuzzer completed 1,138 runs at 5,472 coverage points and 16,159 feature
edges without a failure; LeakSanitizer alone remained disabled under ptrace.

Large exact-dyadic line pairs now retain the certified binary64 candidate filter and
fused exact crossing kernel through a bounded 4,194,304-pair Cartesian product,
provided both contour indices fit the compact event record. Previously the dispatcher
returned to the generic exact segment kernel above 16,384 pairs even though both
paths perform the same conservative x sweep. A binary64 decision still only rejects
or certifies a proper crossing under its error bound; any inconclusive relation
restarts the unchanged exact fallback. The bound caps worst-case temporary candidate
storage while admitting the 256- and 1,024-edge scaling fixtures.

Prepared contour boundary queries now collect only x-interval candidates. Prepared
winding schedules share the event module's binary64-sort/exact-adjacency
certification, and their binary partitions may use a lossy position only after the
exact predecessor and successor certify it. Boolean role assignment reuses the
validated off-boundary winding path instead of rescanning every container boundary.
For non-overlap all-line Boolean output, the exact turn at the lexicographically
minimum vertex recovers the already-directed material/hole role; mixed line/arc,
zero-turn, unresolved, and overlap cases retain exact nesting. Strict-crossing
winding propagation also keeps direct per-segment offsets rather than repeating a
binary search for every adjacent fragment.

In the final seven-sample release comparison, star256 exact region and contour output
measured 1.435 and 1.216 ms/iter, down from 3.296 and 2.239 ms at the preceding
large-curve checkpoint. Star1024 exact region, contour, and loop output measured
31.582, 22.219, and 23.059 ms/iter; the preceding checkpoint measured 183.2, 38.26,
and 38.55 ms. Prepared region and contour output measured 31.462 and 22.219 ms.
`cavalier_contours`, `i_overlay`, and `geo` measured 19.218, 9.922, and 10.027 ms.
Thus exact boundary output is now 1.16 times the Cavalier time instead of 1.94
times, while complete role-assigned output is 5.80 times faster than the preceding
exact checkpoint. A one-operation star1024 region run peaked at 31,388 KiB process
RSS (30.7 MiB), versus 38.1 MiB at the preceding checkpoint.

The complete all-feature test matrix, all-target/all-feature warnings-as-errors
Clippy, and warnings-as-errors rustdoc passed on the integrated change. The final
AddressSanitizer region-Boolean differential fuzz pass completed 1,295 executions at
5,626 coverage points and 16,867 feature edges without a failure; LeakSanitizer alone
remained disabled under ptrace.

Strict line-crossing Boolean assembly now orders each contour's retained crossings
once and consumes that order throughout the compact pipeline. A finite binary64
parameter preview supplies an inexpensive candidate order, but exact adjacent
comparisons certify every same-segment relation; rounded ties, nonfinite previews,
and ambiguous preview orders fall back to the all-exact sort, while an undecidable
exact order rejects the narrow proof. The previous uniqueness check compared every
crossing pair on a source segment. Fragment classification now advances through the
certified crossing deltas ordinally instead of searching the same segment by exact
parameter for every adjacent fragment. Finally, compact
splitting borrows the already-ordered parameters and points from that crossing
index, avoiding a second marker insertion and sort. These shortcuts remain confined
to the complete, unique, proper all-line crossing proof; all other event sets retain
the general exact pipeline.

In the clean seven-sample release comparison, exact star64 contour output measured
136.070 us/iter, down from 149.8 us at the preceding checkpoint. Star256 region and
contour output measured 1.240 and 1.018 ms/iter, down from 1.435 and 1.216 ms.
Star1024 exact region, contour, and loop output measured 22.754, 13.615, and 14.162
ms/iter, down from 31.582, 22.219, and 23.059 ms. Prepared region and contour output
measured 22.491 and 13.613 ms. `cavalier_contours`, `i_overlay`, and `geo` measured
19.189, 10.013, and 10.123 ms. Exact star1024 boundary contours are therefore 38.7%
faster than the preceding exact checkpoint and now complete 1.41 times faster than
Cavalier; role assignment remains the next scaling target for complete region
output. A standalone one-operation star1024 region run peaked at 31,200 KiB process
RSS, slightly below the preceding 31,388 KiB checkpoint.

Both default and all-feature test matrices, all-target/all-feature warnings-as-errors
Clippy, and warnings-as-errors rustdoc passed. A rounded-preview-tie regression
directly exercises exact order recovery and duplicate rejection. The final
AddressSanitizer region-Boolean differential fuzz pass completed 1,299 executions at
5,638 coverage points and 16,904 feature edges without a failure; LeakSanitizer alone
remained disabled under ptrace.

Directed all-line Boolean contours now recover material/hole orientation from the
exact winding of their tangent directions around the origin. For a simple closed
polyline this tangent-map rotation index is exactly the contour orientation.
Previously the role shortcut found a lexicographically extreme output vertex by
comparing every wide rational intersection coordinate, then tested its local turn.
Split lines already retain their source support, so the new path performs the same
once-visiting proof over the much smaller exact source directions. One reversal bit
keeps shared supports oriented with emitted fragments; it occupies existing
`LineSeg2` padding, leaving the type at 328 bytes. Mixed curves, undecidable signs,
exact half turns, and winding counts other than positive or negative one retain the
extreme-vertex proof and general nesting fallbacks.

Stage timing on star1024 reduced exact role assignment and result construction from
about 8.6 to 1.1 ms. In the clean seven-sample release comparison, ordinary and
prepared exact region output measured 15.206 and 14.818 ms/iter, down from 22.754
and 22.491 ms. Exact contour and loop output measured 13.772 and 14.359 ms.
`cavalier_contours`, `i_overlay`, and `geo` measured 19.205, 9.879, and 9.954 ms.
Complete exact star1024 region output is therefore 33.2% faster than the preceding
checkpoint and now completes 1.26 times faster than Cavalier. Star64 exact region
output measured 135.817 us and star256 measured 1.195 ms, both within the recent
small- and medium-fixture run range. A standalone star1024 run peaked at 31,336 KiB
RSS, effectively unchanged from the preceding 31,200 KiB checkpoint.

Default and all-feature tests, all-target/all-feature warnings-as-errors Clippy, and
warnings-as-errors rustdoc passed. Direction-winding regressions cover both
orientations of simple concave contours, reversed retained fragments, equivalence
with the prior exact extreme-vertex proof, and exact half-turn rejection. The final
AddressSanitizer region-Boolean differential fuzz pass completed 1,305 executions at
5,674 coverage points and 16,991 feature edges without a failure; LeakSanitizer alone
remained disabled under ptrace.

The next clean star1024 contour checkpoint reproduced at 13.44--13.55 ms/iter,
versus about 9.9--10.0 ms for `i_overlay` and `geo`. A 999 Hz profile assigned
15.24% self time to Hyperreal's exact `u128` GCD, 5.46% to fixed-stack dyadic
products, 4.88% to retained candidate/event collection, 4.42% to memory moves,
and 2.72% to the fused exact line-intersection wrapper.

Several bounded alternatives were measured and removed. Choosing the smaller of
minimum-x and maximum-x candidate prefixes reduced candidate samples but was
wall-time neutral. A cached exact-AABB hierarchy preserved the public event
order but increased contour time about 4%. Enlarging the compact-candidate
reservation regressed about 0.5%. Compacting `LineSeg2`'s retained parameter
range reduced that type from 328 to 256 bytes, but `Segment2` remained fixed by
the 376-byte arc variant and the extra conversion increased contour latency.
Prepared primitive-direction certificates removed wide coordinate reductions
inside Hyperreal but left star1024 unchanged after certificate plumbing.

The remaining scaling target is therefore the event representation, not another
candidate filter: proper crossings need a compact shared-determinant form that
can serve split ordering, point identity, and winding propagation before public
APIs materialize four independent canonical rationals. This is also the
architecture required to reduce the large line-only `Segment2` memory traffic
without boxing public arc variants or weakening exact degeneracy behavior.

The first shared-determinant representation step now lives at the Hyperreal
intersection boundary. Both segment parameters remain eagerly reduced because
split order observes them. The two exact affine point coordinates retain their
unreduced quotient internally, so the compact all-line pipeline can clone,
compare, and emit them without performing two odd GCDs per proper crossing.
Observable rational access, exact extraction, hashing, formatting,
serialization, representation-sensitive dyadic kernels, and lossy IO all
canonicalize through one thread-safe retained value. Sign and cross-product
comparisons remain exact on the stored ratio. This uses the existing primary
cache slot and leaves `RationalData` at 88 bytes.

The star64 dispatch trace recorded 40 fused line intersections and zero
lazy-coordinate canonicalizations during Boolean assembly. In the matched
21-sample, 50-iteration star1024 contour run, the clean 13.517 ms checkpoint
fell to 12.761 ms/iteration (5.6%). The wider nine-sample comparison measured
star64 region/contours at 129.8/137.7 us, star256 at 1.080/0.958 ms, and
star1024 at 13.742/12.681 ms; prepared star1024 contours measured 12.497 ms.
At star1024 this is faster than `cavalier_contours` at 19.340 ms, but remains
behind `i_overlay` and `geo` at 10.000 and 10.095 ms. At 64 and 256 vertices
the finite competitors also remain faster, so the broad completion target is
not met: candidate/event storage and exact parameter canonicalization remain
the next scaling work, and the general mixed-curve pipeline and LineArc
accelerator remain in place.

Three standalone star1024 runs peaked at 31,288--31,504 KiB RSS (31,416 KiB
median), effectively flat against the 31,336 KiB checkpoint. Hyperreal's full
557-test unit suite plus every integration, property, serde, oracle, and doc
test passed. Its extended AddressSanitizer `real_exact` fuzz target completed
85,423 executions in 31 seconds without failure; LeakSanitizer alone was
disabled because the managed ptrace environment cannot attach it.

The next parameter experiment established a narrower boundary. Deferring the
two parameter GCDs reduced the star64 trace from 116 to 55 total GCDs, but
repeated ordering on the larger unreduced cross-products raised the matched
star1024 contour row from 12.621 to 13.548 ms. That experiment was removed:
canonical parameters are cheaper for the current repeatedly compared event
model. A future shared determinant event must compare its native words
directly rather than store ordinary unreduced `Rational` nodes.

The retained event-storage step instead consumes the already sorted crossing
index directly. It no longer rebuilds a contour-sized
`Vec<Vec<SegmentSplitMarker>>`, repeats the segment index on every marker, or
clones source endpoints and zero/one parameters into every crossed segment.
Compact marker arrays now contain only true intersection points and parameters;
source endpoint geometry is implicit, and one zero/one parameter pair is shared
by all sibling fragments. One shared split-data node owns those values and the
source support, reducing `CompactLineContourFragment` from 40 to 32 bytes and
its interior marker from 56 to 48 bytes on 64-bit targets. Removing the
superseded marker builder and certified constructor also leaves this change 60
source lines smaller.

The conservative matched 31-sample, 50-iteration star1024 contour median is now
12.548 ms against the immediate 12.621 ms checkpoint and the original 13.517
ms clean anchor. The nine-sample matrix measured star64 region/contours at
125.8/127.5 us, star256 at 1.086/0.966 ms, and star1024 at 13.981/12.643 ms;
prepared star1024 contours measured 12.815 ms. Cavalier measured 19.333 ms at
star1024, while `i_overlay` and `geo` remain ahead at 10.034 and 10.097 ms. The
compact representation is retained for its measured latency, static memory
reduction, and larger-curve capacity. The full all-feature suite passed, and
the AddressSanitizer `region_boolean` target completed 8,431 executions in 31
seconds without failure after seeding from 1,039 retained cases. It does not
satisfy the completion gate.

A native determinant-event prototype established the next representation
boundary. It retained crossing numerators and the shared denominator in fixed
words, compared parameters without constructing `Real` values, sorted the
crossing index in that form, and materialized the public event values on
demand. Complete Boolean output still has to put the selected parameters into
each emitted `LineSeg2::support_range`, however, so every retained crossing
parameter was eventually materialized while the determinant carrier remained
live. The prototype therefore added storage instead of replacing it.

On the star64 dispatch trace, the prototype increased total dispatch events
from 3,272 to 3,461, rational temporaries from 177 to 194, reductions from 24
to 47, GCDs from 116 to 140, and `Real` constructions from 903 to 915; the 40
fused line intersections were unchanged. The fixed 11-sample, 50-iteration
star1024 exact-contour comparison measured 12.680 ms/iteration versus 12.548 ms
for the retained checkpoint, a 1.05% regression. The prototype was removed.
Another determinant event should be attempted only after retained line source
ranges can hold a shared compact determinant-backed form, or another
representation that replaces the two final `Real` parameters rather than
coexisting with them.

The retained-line prerequisite is now smaller than a new parameter
representation: `LineSeg2::support_range` was duplicate state. Its only
consumer asked whether two fragments sharing the same source support were
strictly disjoint. Ordering the fragment endpoints on a certified nonconstant
support coordinate proves the same interval relation exactly, including
vertical and reversed fragments, without retaining or composing two source
parameters. Recursively split and similarity-transformed fragments keep the
same support and endpoint proof. Removing the field, its composition
arithmetic, and its reversal plumbing reduces `LineSeg2` from 328 to 232 bytes
on 64-bit targets and removes 19 net implementation lines before the new
regression coverage. The public mixed `Segment2` remains 376 bytes because the
arc variant still fixes its inline size.

The star64 dispatch trace is unchanged at 3,272 events, 177 rational
temporaries, 24 reductions, 116 GCDs, 903 `Real` constructions, and 40 fused
line intersections: the improvement removes duplicate retained state rather
than moving arithmetic elsewhere. The immediate matched 11-sample,
50-iteration star1024 exact-contour median fell from 12.787 to 12.340 ms. A
more conservative 31-sample run measured 12.291 ms, 2.0% below the preceding
12.548 ms checkpoint. The nine-sample matrix measured ordinary and prepared
exact contours at 12.324 and 12.168 ms, exact regions at 13.596 and 13.407 ms,
and exact loops at 12.997 and 12.798 ms. Cavalier measured 19.669 ms, while
`i_overlay` and `geo` remain ahead at 10.041 and 10.196 ms.

All-feature tests, warnings-as-errors all-target Clippy, and warnings-as-errors
rustdoc passed. The AddressSanitizer `region_boolean` differential fuzz target
completed 8,257 executions at 5,844 coverage points and 18,140 feature edges
without failure; LeakSanitizer alone remained disabled under ptrace. Direct
contour emission no longer materializes retained line parameters, so a compact
determinant event can now replace ordinary parameter `Real`s on that path.
Evidence-bearing boundary-fragment APIs still require exact parameters on demand
and remain the materialization boundary. The broad competitor gate is not met.

The next retained-crossing experiments further narrowed that boundary and were
removed. A fixed-word determinant carrier could not represent every star1024
crossing. Falling back without repeating the broad phase restored the retained
star64 arithmetic trace exactly (3,272 dispatches, 177 temporaries, 24
reductions, 116 GCDs, 903 `Real` constructions, and 40 fused intersections),
but the 11-sample, 50-iteration star1024 median was 12.800 ms, about 4% slower
than the 12.291 ms retained checkpoint. Widening or retaining that carrier
would therefore add live state without removing the eventual parameter
materialization.

An exact point plus each source line's nonconstant dyadic coordinate was a
smaller carrier and ordered crossings without parameters. Reconstructing a
parameter as `(point_axis - start_axis) / (end_axis - start_axis)` is exact,
but complete Boolean output repeated costly rational divisions after the fused
intersection kernel had already computed those parameters. It measured 54.371
ms/iteration. Retaining the eager parameters while lazily constructing public
events reduced that to 14.406 ms, and borrowing those parameters directly
through winding and compact markers reached 14.012 ms. The clean checkpoint
reproduced at 13.473 ms on the same run sequence, so the lazy event sidecar and
indirection were also removed. The existing contiguous normalized event vector
is the faster carrier while selected output still observes ordinary `Real`
parameters.

Finally, the retained profile attributed 15.25% of instructions to Hyperreal's
balanced `u128` GCD. Replacing its tuned quotient/subtraction hybrid with a
remainder-free Stein loop passed all 483 Hyperreal unit tests but raised the
same star1024 median from 13.473 to 14.059 ms. Lower retired instruction count
does not compensate for the longer dependency chain on these determinant
operands. Future work should avoid creating or repeatedly observing parameter
GCD inputs at the event/output boundary rather than replacing the established
native reducer.

A contour-only determinant lane now crosses that boundary without changing the
public intersection APIs or evidence-bearing provenance path. Hyperreal retains
only the two determinant numerators and their shared denominator in a 96-byte
fixed-word carrier. It compares either source parameter by aligned native
cross-products, with an exact arbitrary-precision comparison fallback for
extreme exponent gaps, and reduces a parameter only if an observable API asks
for it. Hypercurve pairs that carrier with the exact point in a lazy event
sidecar. The all-line Boolean crossing index consumes the carrier directly;
compact fragment classification and endpoint-chain assembly consume only exact
points. Crossings outside the fixed kernel retain the eager parameters, and
mixed, curved, overlap, loop-provenance, and evidence-bearing queries retain the
existing exact pipeline. A static layout regression requires the lazy crossing
carrier to be no larger than the eager point event.

The star64 trace fell from 3,272 to 3,116 dispatch events, from 177 to 175
rational temporaries, from 116 to 40 GCD observations, and from 903 to 825
`Real` constructions; reductions remained 24 and all 40 intersections stayed
on exact line kernels. Unlike the rejected point-coordinate ordering prototype,
the trace contains no coordinate canonicalization during crossing sort. A
randomized Hyperreal regression compares both retained parameter orders with
their materialized exact values, and the Hypercurve dense-event regression
materializes the lazy events and proves equality with both eager retained and
arbitrary-precision collectors.

The conservative 31-sample, 50-iteration star1024 region median is 9.819 ms,
20.1% below the retained 12.291 ms checkpoint. In fixed 11-sample runs,
ordinary and prepared contour output measured 8.668 and 8.653 ms, while
prepared region output measured 9.675 ms. The contemporary finite competitors
measured 19.439 ms for Cavalier, 9.954 ms for `i_overlay`, and 10.230 ms for
`geo`, so exact Hypercurve region and contour output lead this large fixture.
The crossover remains incomplete: star64 region/contour output measured about
99 us versus 28--36 us for the finite competitors, and star256 contour output
measured 0.692 ms versus 0.478--0.512 ms. Provenance-bearing loop output remains
eager by design.

Three standalone one-operation star1024 runs peaked at 32,848--32,868 KiB RSS,
about 1.4 MiB above the preceding 31,416 KiB median despite the lazy carrier
not exceeding the eager event's static size. The large-curve latency and exact
arithmetic reductions justify retaining the lane, but lowering process peak
memory remains an explicit follow-up rather than a completed gate. Both full
all-feature suites, warnings-as-errors Clippy, and warnings-as-errors rustdoc
passed. The AddressSanitizer `region_boolean` differential target completed
7,864 executions at 5,863 coverage points and 18,235 feature edges without a
failure; LeakSanitizer alone remained disabled under ptrace.

The next checkpoint extends the retained determinant lane instead of
materializing parameters when a crossing exceeds the native carrier. Hyperreal
now retains up to four fixed limbs per determinant component, compares
wide/wide and wide/native parameters with exact fixed-limb cross-products, and
constructs the exact point with the existing checked stack accumulator. The
uncommon 128-byte wide carrier is boxed; packing the dominant native carrier
reduced it from 96 to 80 bytes. Boxing the already-uncommon eager parameter pair
at the same enum boundary reduced the complete lazy crossing event from 224 to
192 bytes, versus 224 bytes for the eager point event. Randomized wide dyadic
crossings check both reconstructed coordinates and retained parameter ordering
against expanded arbitrary-precision arithmetic.

The star64 trace is now 2,823 dispatch events, 135 rational temporaries, 12
reductions, 12 GCD observations, and 755 `Real` constructions. In a 21-sample,
500-iteration release run, region/contour output measured 78.044/76.726
us/iteration and prepared region/contour output measured 75.347/74.972
us/iteration. The finite competitors measured 27.433 us for Cavalier, 34.420 us
for `i_overlay`, and 35.872 us for `geo`; the small-fixture crossover therefore
remains open. Across 100 selected ordinary star64 operations plus fixture
validation, heaptrack allocations fell from 108,153 at the preceding checkpoint
to 71,271, while temporary allocations fell from 4,513 to 1,153. In addition to
the wide carrier, compact split data now stores its common single-marker case
inline rather than allocating a separate marker vector.

In fixed 11-sample, 500-iteration runs, star256 region/contour output measured
0.723/0.593 ms, with prepared contour output at 0.588 ms; Cavalier,
`i_overlay`, and `geo` measured 0.480, 0.507, and 0.516 ms. In a 21-sample,
50-iteration star1024 run, region/contour output measured 9.667/8.475 ms and
prepared region/contour output measured 9.686/8.523 ms. The corresponding
finite competitors measured 19.280, 10.047, and 10.161 ms, so exact Hypercurve
still leads all three on both large exact output lanes. Three standalone
one-operation star1024 runs peaked at
32,972--33,076 KiB RSS (33,048 KiB median), 188 KiB above the preceding 32,860
KiB median. Both all-feature suites, warnings-as-errors Clippy, and
warnings-as-errors rustdoc passed. With LeakSanitizer disabled because ptrace
prevents it from starting, the AddressSanitizer `region_boolean` differential
target completed 7,991 executions at 5,881 coverage points and 18,384 feature
edges without a failure.

The next checkpoint compacts the immutable native geometry rather than changing
its arithmetic. `Point2` now owns exact coordinates through one shared `Arc`;
clones and endpoint-chain identities reuse that allocation while preserving
`Send + Sync`. `CircularArc2` moves its geometry into the `Rc` allocation that
already held its lazy exact facts, so an arc clone shares both geometry and
certificates. A source `LineSeg2` lazily retains one fragment support for all of
its children. Consuming reversal drops that oriented cache, and fragments which
already carry a source support continue to reuse it directly. On 64-bit targets,
`Point2`, `LineSeg2`, `CircularArc2`, `Segment2`, and the compact split marker
fell from 104, 232, 376, 376, and 152 bytes to 8, 48, 8, 48, and 56 bytes.
Static layout regressions, shared-coordinate/cache tests, and a reversal
orientation regression protect those boundaries.

The star64 arithmetic trace is exactly unchanged at 2,823 dispatch events, 135
rational temporaries, 12 reductions, 12 GCD observations, 755 `Real`
constructions, and 40 fused line intersections. The current CPU profile reduces
`memmove` from 10.65% at the preceding inline-geometry checkpoint to 1.33%;
the previously visible per-source fragment-support allocator is no longer a
top-level sample. Across 100 selected ordinary star64 operations plus fixture
validation, heaptrack records 68,828 allocations and 1,154 temporaries, versus
71,271 and 1,153 at the preceding checkpoint. The line-support cache removes
8,267 allocations from the otherwise-identical compact-point candidate. Peak
tracked heap fell from 838.92 to 703.89 KiB.

In the fixed 21-sample, 500-iteration star64 matrix, ordinary region/contour
output measured 65.328/65.431 us per iteration and prepared region/contour
output measured 64.289/63.286 us, versus 78.044/76.726 and 75.347/74.972 us at
the preceding checkpoint. Provenance-bearing ordinary/prepared loop output
measured 119.299/118.905 us. The finite competitors measured 27.760 us for
Cavalier, 34.915 us for `i_overlay`, and 36.078 us for `geo`, so the small
crossover remains open. Arc-only star-loop isolation improved by 9--11%, and a
mixed capsule inward-offset lane improved by 6--10%, showing that the compact
arc carrier is not merely a line-Boolean specialization.

In fixed 11-sample, 500-iteration star256 runs, region/contour output measured
0.655/0.532 ms and prepared contour output measured 0.523 ms. Cavalier,
`i_overlay`, and `geo` measured 0.480, 0.507, and 0.513 ms, leaving exact contour
output close to but still behind the finite implementations at this size. In
the 21-sample, 50-iteration star1024 matrix, region/contour output measured
9.379/8.161 ms and prepared region/contour output measured 9.196/8.055 ms.
Cavalier, `i_overlay`, and `geo` measured 19.607, 10.078, and 10.278 ms, so exact
Hypercurve retains its large-fixture lead. Five standalone one-operation
star1024 runs peaked at 24,016--24,356 KiB RSS (24,284 KiB median), 8,764 KiB
or 26.5% below the preceding 33,048 KiB median.

All-feature tests, warnings-as-errors all-target Clippy, and
warnings-as-errors rustdoc passed. With LeakSanitizer disabled because ptrace
prevents it from starting, the AddressSanitizer `region_boolean` differential
target completed 7,730 executions at 5,940 coverage points and 18,614 feature
edges without failure. Stable Rust cannot const-dereference `Arc` or `Rc`, so
the public `Point2`, `CircularArc2`, native-segment, prepared-segment, and AABB
accessors that traverse these handles are no longer `const fn`; their ordinary
signatures, immutability, exact values, and runtime behavior are unchanged.

The following checkpoint removes duplicated compact-split ownership. The
retained line-crossing index already stores each exact point and optional
materialized source parameter in certified per-segment order, and it remains
alive through fragment classification and output emission. A compact fragment
is now an 8-byte view containing only a `u32` source-segment index and a `u32`
boundary-marker index; a sentinel represents an unsplit source segment. The
previous per-source split allocation, then its contour-local data and marker
arenas, are unnecessary. Selected fragments still materialize ordinary exact
line geometry at the same output boundary.

Endpoint-chain assembly also uses a short-lived hash table keyed only by
process-local shared-point allocation identities. A SplitMix finalizer
scrambles the aligned pointer values without the keyed general-purpose hash
overhead; these keys are internal identities rather than user-supplied
geometry, and `HashMap` still resolves collisions normally. Two alternating
41-sample, 1,000-iteration star64 contour runs measured 54.1--54.2 us with the
identity hasher and 56.3--57.5 us with the standard hasher.

The paired release matrices below compare the preceding compact-native-geometry
checkpoint with the retained crossing-index view. Star64 used 21 samples of
500 iterations, star256 used 21 samples of 200 iterations, and star1024 used
11 samples of 20 iterations.

| Workload | Before | After | Change |
| --- | ---: | ---: | ---: |
| star64 region / contours | 67.161 / 65.261 us | 55.696 / 54.590 us | 17.1% / 16.4% faster |
| star64 prepared region / contours | 63.118 / 62.805 us | 52.010 / 51.546 us | 17.6% / 17.9% faster |
| star64 ordinary / prepared loops | 116.824 / 114.927 us | 108.701 / 104.826 us | 7.0% / 8.8% faster |
| star256 region / contours | 0.653 / 0.529 ms | 0.582 / 0.457 ms | 11.0% / 13.5% faster |
| star1024 region / contours | 9.163 / 8.030 ms | 8.770 / 7.501 ms | 4.3% / 6.6% faster |
| star1024 ordinary / prepared loops | 13.158 / 13.316 ms | 11.997 / 11.773 ms | 8.8% / 11.6% faster |

Exact Hypercurve contour output now measures 0.457 ms at star256, ahead of
Cavalier at 0.482 ms, `geo` at 0.524 ms, and `i_overlay` at 0.540 ms in this
run. At star1024, exact region/contour output measures 8.770/7.501 ms, versus
19.691 ms for Cavalier, 10.058 ms for `i_overlay`, and 10.448 ms for `geo`.
The star64 crossover remains open: exact contour output is 54.590 us, versus
27.414, 34.123, and 35.904 us respectively.

Across 100 selected ordinary star64 contour operations plus fixture validation,
heaptrack records 59,096 allocations and 1,052 temporaries, versus 68,828 and
1,154 at the preceding checkpoint. Peak tracked heap is effectively unchanged
at 697.47 KiB versus 703.89 KiB. Eleven paired standalone star1024 runs measured
24,080 KiB median RSS versus 24,132 KiB for the preceding binary. The exact
trace also reflects the eliminated parameter copies: star64 falls from 2,823
to 2,819 dispatch events, 135 to 131 rational temporaries, and 755 to 751
`Real` constructions; reductions and GCD observations remain 12 each.
The full all-feature test suite, warnings-as-errors all-target Clippy,
warnings-as-errors rustdoc, and all-feature bench compilation pass. With
LeakSanitizer disabled under ptrace, the AddressSanitizer `region_boolean`
differential target completed 10,000 executions at 5,901 coverage points and
18,616 feature edges without failure.

The next event-collector checkpoint packs each retained proper-crossing
candidate's two `u16` segment indices into one `u32`. Crossing orientation was
already representable in the existing sub-64-event bitset, so the temporary
record falls from 6 to 4 bytes without moving or weakening a predicate. A
static layout assertion protects that bound. Two paired 31-sample star256
contour runs measured 0.456 versus 0.475 ms and 0.475 versus 0.489 ms, a
2.8--4.0% improvement. A 21-sample star1024 run measured 7.557 versus
7.605 ms, while star64 remained within roughly 1% of the preceding checkpoint.
Heaptrack still records 59,096 allocations, 1,052 temporaries, and 697.47 KiB
peak heap because the change shrinks an existing staging allocation.

Three broader variants were measured and removed. Emitting crossings during
the broad-phase scan avoided one allocation and improved star1024 by about 3%,
but geometric vector growth raised the eleven-run RSS median from 24,040 to
24,432 KiB; the two-stage collector intentionally keeps its exactly sized
retained output. A lazy exact-f64 interval tree raised star1024 to 8.060 ms,
6.7% slower than the packed linear sweep. Replacing endpoint hashing with a
sorted flat index raised star64 by about 2% and star1024 by about 2.5%.

The next cross-stack checkpoint removes a full-width temporary from
Hyperreal's exact dyadic product accumulator. Fixed-stack determinant,
parameter-ordering, and affine-point products are now shifted directly into
their destination limbs, with carry propagation stopping as soon as the carry
does. The six-limb admission boundary and arbitrary-precision fallback are
unchanged.

Matched contour-only trials measured star64 at 53.849 versus 55.065 us,
star256 at 448.589 versus 467.143 us, and star1024 at 7.203 versus 7.550 ms:
2.2%, 4.0%, and 4.6% faster. Follow-up full matrices showed gains across
ordinary and prepared region, contour, and loop outputs after isolated noisy
rows were rerun: dedicated star64 prepared-loop trials measured 102.027 versus
103.476 us, and star256 ordinary-region trials measured 568.358 versus 580.775
us. In the final absolute comparison, exact contours measured 52.623 us at
star64, 441.384 us at star256, and 7.039 ms at star1024. The corresponding
competitor rows were 27.729/34.924/36.712 us, 492.026/514.954/530.687 us, and
20.281/10.135/10.351 ms for Cavalier, `i_overlay`, and `geo`. Exact contour
output therefore retains the star256 and star1024 lead; the star64 crossover
remains open.

The 500-operation star1024 profile reduced exact accumulator self-time from
17.46% to 12.64%. Heaptrack remains exactly 59,096 allocations and 697.47 KiB
peak heap in the 100-operation star64 contour workload. Eleven standalone
star1024 processes measured 23,928 KiB median RSS versus 24,172 KiB for the
preceding binary, with normal process-layout variance. Hyperreal's 561-test
all-feature suite and Hypercurve's complete all-feature suite, strict Clippy,
warning-denied rustdoc, and a 10,000-run AddressSanitizer region-Boolean
campaign (5,902 coverage points and 18,712 feature edges) pass.

The next exact-ordering checkpoint specializes retained compact line parameters.
Each quotient comparison now multiplies its two `u128` magnitudes into four
fixed limbs, compares the product bit lengths plus their dyadic exponents, and
only when those totals tie compares lazily normalized limbs from most to least
significant. This is the exact cross-product order: it neither converts through
binary64 nor materializes a `BigUint`. Wide and mixed events retain their
existing arbitrary-precision paths.

Applying the larger comparator to every sort improved star1024 but regressed
star64 by 1--3% through instruction-layout pressure. The crossing index
therefore selects it only for certified sets containing at least 1,024 events;
smaller inputs keep the preceding compact accumulator. Two alternating paired
41/31/21-sample contour runs left star64 and star256 neutral and moved star1024
to 6.765--6.948 ms from 7.219--7.316 ms, a 5--6% gain. In the final full
matrix, exact contour output measured 52.727 us at star64, 446.860 us at
star256, and 6.932 ms at star1024. The corresponding finite competitor rows
were 27.603/34.323/36.519 us, 485.112/511.131/519.842 us, and
20.359/10.193/10.312 ms for Cavalier, `i_overlay`, and `geo`. Exact Hypercurve
therefore extends its star256 and star1024 lead; the star64 crossover remains
open. Dedicated reruns also found ordinary/prepared full-region output faster
and provenance-bearing loop output neutral within run noise.

The 500-operation star1024 profile moves exact accumulator self-time from
12.60% to 6.07%; the new fixed-product comparator accounts for 6.63%, and the
remaining accumulator work constructs final exact points. Heaptrack remains
exactly 59,096 allocations with 697.48 KiB peak heap for 100 selected star64
contour operations. Eleven paired star1024 processes measured 24,024 KiB median
RSS versus 23,912 KiB for the preceding binary, within normal layout variance
and with no retained allocation added.

Three broader variants were removed. A product-bit-bound prefilter made star64
about 3% slower and did not improve larger cases. Eagerly normalizing two
`u128` halves reduced generated code but made star1024 about 1--2% slower
because most comparisons decide after the first lazy `u64` limb. Selecting the
fixed-product path at all sizes preserved the large gain but paid its
instruction-locality cost on small contours. A 20,000-case `BigUint` oracle
covers both parameter coordinates, signs, and shifts through 2,047. Both
complete all-feature suites, warnings-as-errors Clippy and rustdoc, and the
10,000-run AddressSanitizer differential Boolean campaign pass; the fuzzer
finished at 5,906 coverage points and 18,772 feature edges.

The following crossing-index checkpoint separates the dense integer key from
the exact local key. Large certified crossing arrays are first grouped by
source-segment index using the allocation-free unstable integer sort. The
already-required segment-offset vector then identifies independent parameter
slices. Groups of at most 16 crossings use one exact insertion pass that both
locates each value and rejects equality; larger groups retain `sort_unstable`
and a separate adjacent certificate, preserving \(O(k \log k)\) behavior for a
pathological segment crossed many times.

Two alternating 21-sample, 30-iteration star1024 contour trials measured
6.391--6.427 ms after grouping versus 6.815--6.842 ms at the normalized-product
checkpoint, another 5.7--6.6% improvement. Repeated star64 and star256 trials
remain neutral because they stay below the 1,024-event crossover. In the
complete star1024 matrix, ordinary/prepared exact contours measured 6.813/6.705
ms, versus 20.301 ms for Cavalier, 10.188 ms for `i_overlay`, and 10.086 ms for
`geo`.

The 500-operation profile reduces normalized-comparator self-time from 6.63%
at the preceding checkpoint to 3.72%; final exact point construction leaves
5.27% in the shared dyadic accumulator. Heaptrack records exactly 1,104,312
allocations, 2,192 temporaries, and 16.58 MiB peak heap for both binaries across
ten star1024 contour operations. A corrected in-place counting-bucket prototype
improved the preceding checkpoint by only 1--2% and required an additional
segment-sized cursor allocation, so the faster zero-allocation grouped sort was
retained instead.

Direct tests cover shuffled segment groups, exact local order, duplicates, and
the bounded dense-group fallback. The complete all-feature suite, strict
Clippy, and warning-denied rustdoc pass. The 10,000-run AddressSanitizer
differential Boolean campaign completed without failure at 5,912 coverage
points and 18,797 feature edges.

The following broad-phase checkpoint eliminates repeated expired-prefix visits
without retaining another index. Each existing four-byte minimum-x order entry
now packs its segment index in the low 16 bits and the segment supplying the
greatest maximum x in its prefix in the high 16 bits. A binary search over
those nondecreasing prefix maxima skips every box whose maximum x is strictly
left of the current source box. Touching boundaries remain candidates, and all
coordinates in this cache remain lossless binary64 dyadics; exact segment
predicates still decide every surviving pair.

For the star64, star256, and star1024 fixtures, the skipped expired prefix
removes 1,330 of 2,116, 22,110 of 33,804, and 356,634 of 540,592 ordered box
visits respectively. Alternating fixed-iteration star1024 contour trials moved
from 6.46--6.50 ms medians to 6.28 ms, a 2.8--3.3% improvement; star64 improved
about 2% and star256 remained neutral within run noise. In the follow-up full
star1024 matrix, ordinary/prepared exact contours measured 6.169/6.099 ms,
versus 19.344 ms for Cavalier, 10.131 ms for `i_overlay`, and 10.044 ms for
`geo`.

The collector's self-time in the frame-pointer profile fell from 11.47% to
7.97%. In-place packing preserves the preceding trace exactly: ten star1024
contour operations still record 1,104,312 allocations, 2,192 temporaries, and
16.58 MiB peak heap. Direct tests cover a changing prefix maximum, strict
left-of rejection, boundary retention, packed order, and equality with the
unreserved exact collector. The complete all-feature suite, strict Clippy, and
warning-denied rustdoc pass. The 10,000-run AddressSanitizer differential
Boolean campaign completed without failure at 5,924 coverage points and 18,872
feature edges.

The next cross-stack predicate checkpoint reuses the fixed affine line through
each source segment's entire candidate suffix. Hyperreal's prepared filter now
retains its already-certified direction and evaluates both retained exact-dyadic
binary64 query points together. The second line direction remains lazy, so a
first-line same-side separation still exits before doing that work. Its
aggregate determinant bound also makes separate product and result
classifications redundant: a normal scaled bound dominates absolute subnormal
rounding, while non-normal magnitudes still fall through to the unchanged exact
filters.

Two alternating 21-sample star1024 contour comparisons measured
5.996--6.121 ms versus 6.115--6.251 ms at the preceding prefix-sweep
checkpoint, about 2% faster. Star64 remained neutral within run noise and
star256 improved about 2%. In the complete star1024 matrix, ordinary/prepared
exact contours measured 5.878/5.856 ms, versus 19.807 ms for Cavalier,
10.233 ms for `i_overlay`, and 10.604 ms for `geo`. The ordinary four-segment
rectangle-union contour path also improved from 5.471 to 5.367 us.

The reuse is stack-only. Heaptrack remains exactly 1,104,312 allocations,
2,192 temporaries, and 16.58 MiB peak heap across ten star1024 contour
operations. Direct tests compare retained pair queries with one-shot
certificates, cover direction overflow and subnormal-boundary fallback, and
retain equality with the unreserved exact event collector. Both complete
all-feature suites, strict Clippy, and warning-denied rustdoc pass. The
10,000-run AddressSanitizer differential Boolean campaign completed without
failure at 5,892 coverage points and 18,786 feature edges.

The next exact-point checkpoint prepares each source segment's compact dyadic
endpoint words and exact delta once for its A-major candidate group. Compact
and wide determinant plans reuse that first line while still deriving the
second line, all three exact determinants, and both rational coordinates per
crossing. A source line outside the fixed-word envelope immediately returns to
the unchanged exact collector, so the optimization neither widens an
approximate predicate nor narrows supported geometry.

Two alternating 21-sample star1024 contour trials measured
5.733--5.737 ms versus 5.865--5.880 ms at the preceding predicate-reuse
checkpoint, another 2.2--2.5% improvement. Star64 measured 53.155 us versus
53.560 us, and a reversed 41-sample star256 rerun measured 0.423 ms versus
0.434 ms. Ordinary rectangle contours measured 5.339 us versus 5.487 us.

The complete star1024 comparison measured ordinary exact contours at
5.816 ms; a dedicated prepared-contour rerun measured 5.863 ms. The competitor
rows were 19.590 ms for Cavalier, 10.316 ms for `i_overlay`, and 10.275 ms for
`geo`. Heaptrack remains exactly 1,104,312 allocations, 2,192 temporaries, and
16.58 MiB peak heap across ten contour operations.

Hyperreal's compact and wide randomized exact-arithmetic oracles now compare
one-shot and prepared parameters and points for every admitted case. The
retained collector equality test covers the downstream grouping and fallback.
Both complete all-feature suites, strict Clippy, and warning-denied rustdoc
pass. The 10,000-run AddressSanitizer differential Boolean campaign completed
without failure at 5,892 coverage points and 18,825 feature edges.

The next point-construction checkpoint consumes the exact finite binary64
endpoints already reconstructed by the line AABB cache. Hyperreal decodes
their IEEE-754 sign, significand, and exponent directly into normalized dyadic
words, avoiding four retained-rational canonicalization and magnitude probes
per crossing. The compact/wide determinant plans and every exact fallback are
otherwise unchanged.

Two alternating 21-sample star1024 contour comparisons measured
5.634 and 5.542 ms versus 5.769 and 5.686 ms at the prepared-rational
checkpoint, a 2.3--2.5% improvement. Seven-run counters over 320 fixed
iterations reduced cycles from 8.247 to 8.080 billion, instructions from
26.274 to 25.969 billion, and branches from 4.443 to 4.345 billion. Star64
measured 51.242 us versus 52.424 us; star256 measured 0.416 ms versus
0.435 ms.

The complete star1024 matrix measured ordinary/prepared exact contours at
5.656/5.445 ms, versus 19.809 ms for Cavalier, 10.139 ms for `i_overlay`, and
10.074 ms for `geo`. The 16.58 MiB peak and per-operation allocations are
unchanged. Heaptrack records one extra 240-byte process-startup temporary, so
the ten-operation process totals are 1,104,313 allocations and 2,193
temporaries instead of 1,104,312 and 2,192.

The retained collector still matches the unreserved exact sweep. Hyperreal
also compares direct and canonical dyadic words over 20,000 random IEEE-754
patterns, runs the direct compact path through the 512-case crossing oracle,
and covers a wide direct determinant plus non-finite and oversized fallback.
Both complete all-feature suites, strict Clippy, and warning-denied rustdoc
pass. The 10,000-run AddressSanitizer differential Boolean campaign completed
without failure at 5,891 coverage points and 18,881 feature edges.

The next exact-arithmetic checkpoint specializes the two-product sums used by
compact line determinants and affine point numerators. Checked `u128`
multiplication, alignment, signed summation, and normalization handle the
common binary64 envelope directly; any overflow or shift miss reruns the
unchanged 384-bit stack accumulator.

Two same-layout alternating 21-sample star1024 contour comparisons measured
5.575 and 5.653 ms versus 5.732 and 5.736 ms, improving 2.7% and 1.4%.
Seven-run counters over 320 iterations reduced instructions from 25.969 to
25.392 billion, branches from 4.345 to 4.152 billion, and branch misses from
23.52 to 21.04 million; cycles were neutral within system noise. Reversed
star64 trials improved 2.7--3.0%, and ordinary rectangle contours improved
from 5.768 to 5.223 us.

Replacing the const-generic loop with the explicit fixed two-term shape then
reduced a 31-sample star1024 contour trial from 5.550 to 5.403 ms. Seven-run
counters fell from 8.119 to 7.792 billion cycles and from 25.392 to
25.190 billion instructions. Final star64 and star256 trials measured
49.806 us and 0.392 ms; rectangle contours measured 5.306 us.

The complete final star1024 matrix measured ordinary/prepared exact contours
at 5.661/5.376 ms, with the ordinary row noisier than its dedicated trial.
Competitors measured 19.857 ms for Cavalier, 10.119 ms for `i_overlay`, and
10.117 ms for `geo`. Heaptrack remains 1,104,313 allocations, 2,193
temporaries, and 16.58 MiB peak heap across ten operations.

Hyperreal's 20,000-case native-versus-stack oracle covers admitted results and
checked overflow deferrals; the existing compact/wide crossing oracles and
retained collector equality test cover integration. Both complete all-feature
suites, strict Clippy, and warning-denied rustdoc pass. The 10,000-run
AddressSanitizer differential Boolean campaign completed without failure at
5,891 coverage points and 18,933 feature edges.

The next point-storage checkpoint removes eager rational materialization from
proper line crossings that only need a retained point. Hyperreal now returns
the two fixed-stack affine numerators and their shared denominator as a compact
exact carrier. `Point2` retains that carrier behind the same one-word shared
handle used by ordinary points and materializes its two public `Real`
coordinates once, on demand. Low pointer bits distinguish the ordinary,
compact, and wide shared payloads; compile-time alignment checks, matching
`Arc` clone/drop dispatch, stable `OnceLock` storage, and cross-thread tests
protect the ownership boundary. Ordinary point payloads remain exactly two
`Real` values (96 bytes), so the broad public point API does not pay for the
line-only optimization.

This changes storage, not geometry: exact crossing parameters, ordering,
fallbacks, point equality, and public `x`/`y` results are unchanged. Randomized
compact and wide arithmetic oracles compare deferred materialization with the
eager path, while retained-event tests cover downstream grouping and fallback.

The fixed star workloads show the scaling effect. Star64 exact contours fell
from 49.806 to 35.327 us, star256 from 0.392 to 0.308 ms, and the complete
star1024 matrix measured ordinary/prepared exact contours at 3.948/3.886 ms.
The same star1024 run measured 19.455 ms for Cavalier, 10.033 ms for
`i_overlay`, and 10.075 ms for `geo`, making the exact contour row about 2.5
times faster than the nearest finite competitor on this polygon workload.
Ordinary exact regions measured 5.027 ms and prepared regions 5.281 ms;
complete loop-producing rows measured 11.129/11.049 ms. Rectangle contours
remained effectively flat at 5.390--5.404 us versus 5.306 us.

Across ten star1024 contour operations, heaptrack allocations fell from
1,104,313 to 464,773 (57.9%) while the 2,193 temporary peak and 16.58 MiB peak
heap were unchanged. Seven counter runs over 320 iterations fell from 7.792 to
5.854 billion cycles, 25.190 to 19.254 billion instructions, and 4.152 to
2.994 billion branches. The remaining polygon profile is led by event
collection, support relations, and normalized parameter comparison rather than
point rational construction.

The complete default and all-feature suites, strict default/all-feature
Clippy, warning-denied rustdoc, fuzz-target builds, and the native demo suite
pass. The post-change AddressSanitizer `region_boolean` differential campaign
completed 10,000 executions without failure at 6,025 coverage points and
19,382 feature edges.

This is a line-overlay scaling checkpoint, not the endpoint of the Hypercurve
audit. The next selection is based on large complex polynomial/rational
Bézier, arc, B-spline/NURBS, offset, arrangement, and pathological-region
profiles across the whole public API. The general `CurveRegion2` pipeline and
the `LineArc` accelerator remain until those mixed-curve workloads meet the
same exactness, capacity, and competitive performance gates.

## Large complex-curve scaling checkpoint

The next whole-API pass added parameterized 64/256/1024-scale lanes for
arbitrary-degree rational Béziers, B-spline/NURBS decomposition, rational and
polynomial arrangement, arc paths, and curved-region containment. These lanes
are isolated with `HYPERCURVE_BENCH_*` environment variables so one operation
can be profiled without timing unrelated benchmark setup.

Three shared algorithm changes dominate the result:

- Boehm insertion now grows one control slot and updates only its affected
  window in reverse. Exact knot multiplicity and insertion-span searches use
  binary partitions. A randomized repeated-knot oracle compares both searches
  with complete scans.
- Retained Bézier overlap preparation caches conservative control hulls and
  uses an exact-certified x sweep for sparse pairs. Control-hull separation is
  attempted before degree-aligned same-image algebra, and tangent traversal
  builds adjacency and predecessor counts in one pass.
- Bernstein/power conversion now uses in-place forward differences and
  arbitrary-precision binomials, removing the former `u64` degree ceiling and
  quadratic Pascal-triangle storage. High-degree point evaluation stays in
  Bernstein form instead of retaining exponentially large power coefficients.
  Homogeneous subdivision uses Hyperreal's exact two-lane aggregate, and a
  linear control-net certificate avoids constructing a doubled-degree
  derivative merely to prove obvious monotonicity.

Same-machine release measurements were:

| Exact workload | Before | After |
| --- | ---: | ---: |
| 1024-control NURBS cold Bézier decomposition | 800.3 ms | 13.1 ms |
| 256-curve retained overlap workflow | 721 ms | 82.5 ms |
| 1024-curve retained overlap workflow, before/after exact x sweep | 2.054 s | 711 ms |
| 256-control rational Bézier midpoint split | 3.215 s | 13.5 ms |
| 1024-control rational Bézier midpoint split | 1.205 s / 567 MiB isolated peak at the preceding in-place checkpoint | 325.7 ms / 7.0 MiB |
| 1024-control rational Bézier point evaluation after removing the degree blocker | 430.4 ms / 1.44 GiB aggregate peak with retained power basis | 10.3 ms / 9.0 MiB isolated peak |
| 1024-arc `CurveRegion2` containment without `LineArcRegion2` | `Uncertain(Ordering)`; then 300.9 ms after the first completeness fix | decided `Inside` in 4.15 ms |

The rational point evaluator still reuses a power basis if another algebraic
API already retained one. Otherwise degree-above-256 evaluation uses an exact
linear Bernstein recurrence with constant auxiliary state. An independent
varying-weight test compares it coordinate-for-coordinate with homogeneous de
Casteljau replay. Arbitrary-degree offset conversion has the matching
power-to-Bernstein fix.

The arc result does not call the native line/arc accelerator. Irrational-weight
quadratic contacts first use the represented exact quadratic solver, while
exact-rational polynomials retain their algebraic-parameter carrier. Ray replay
checks every contact, including tangencies, for zero projection before winding;
an inconclusive optional point-incidence precheck therefore falls through
without weakening boundary detection. Conservative control-hull tests reject
irrelevant fragments and cast in both directions, which is especially
important for a query near one end of a thousand-arc path.

```bash
HYPERCURVE_BENCH_NURBS_ONLY=1 HYPERCURVE_BENCH_NURBS_CONTROLS=1024 \
  HYPERCURVE_BENCH_NURBS_ITERATIONS=1 cargo bench --bench bspline
HYPERCURVE_BENCH_RATIONAL_ONLY=1 HYPERCURVE_BENCH_RATIONAL_CONTROLS=1024 \
  HYPERCURVE_BENCH_RATIONAL_ITERATIONS=1 cargo bench --bench rational_bezier
HYPERCURVE_BENCH_ARC_ONLY=1 HYPERCURVE_BENCH_ARC_COUNT=1024 \
  HYPERCURVE_BENCH_ARC_ITERATIONS=1 cargo bench --bench arc
HYPERCURVE_BENCH_ARRANGEMENT_CURVES=1024 \
  HYPERCURVE_BENCH_ARRANGEMENT_ITERATIONS=1 cargo bench --bench bezier_arrangement
```

At this checkpoint the all-family pathological cell remained an explicit next
target. It prepared 84 candidate pairs in about 659 ms and blocked Boolean
selection on a `RationalQuadraticBezier` `RealSign` classification, while its
exact line/arc projection decided all four operations in about 1.4 ms. The arc
containment fix removed one instance of that blocker class without yet closing
the mixed-region gap.

### Exact endpoint adjacency indexing

The next arrangement profile showed that traversal still compared every
fragment end with every fragment start, even though exact-rational endpoints
provide a canonical hash key. Materialized, branch-free, tangent-ordered, and
retained tangent traversal now build one start index once the graph reaches 16
fragments. Retained traversal combines exact coordinate buckets with topology
vertex buckets: two carried vertex identifiers remain authoritative, while a
missing identifier still falls through to exact coordinate equality.

This is a candidate scheduler only. Every selected pair still passes through
the existing exact equality predicate. Symbolic or algebraic endpoints are
kept in an unkeyed bucket and compared wherever they could equal a keyed
endpoint; an entirely unkeyed query retains the complete scan. The
large-exact-rational case therefore changes from a quadratic candidate scan to
linear indexing plus actual endpoint multiplicity, without approximating a
coordinate or rejecting a supported representation. End keys are created only
for the current query instead of being retained in a second graph-sized array.
Graphs below the measured crossover retain the allocation-free complete scan.

Same-machine release A/B measurements against commit `6065ecd` were:

| Exact arrangement workload | Before | After | Change |
| --- | ---: | ---: | ---: |
| 256-curve materialized tangent traversal | 4.229 ms | 0.728 ms | 82.8% faster |
| 256-curve complete retained-overlap workflow | 79.781 ms | 66.814 ms | 16.3% faster |
| 1024-curve materialized tangent traversal | 113.818 ms | 4.928 ms | 95.7% faster |
| 1024-curve complete retained-overlap workflow | 682.674 ms | 283.142 ms | 58.5% faster |

The 256-curve after values are medians of three 30-iteration runs. The
1024-curve rows are sequential three-iteration runs of the baseline and
optimized binaries. Tiny three-curve tangent-order sentinels remain at their
previous complete-scan timings. The arrangement integration suite includes a
16-fragment symbolic-endpoint case so the indexed crossover cannot silently
drop unkeyed equality candidates.

### Retained exact overlap classification

The next retained-workflow profile showed the same exact overlap scan being
rebuilt by evidence inspection, duplicate consumption, linear refinement, and
the refined traversal. A graph now retains the complete certified-policy
classification after its first scan. The retained value includes uncertainty
as well as decided evidence, so reuse cannot turn an undecided predicate into a
decision. Other policies continue to scan independently. Graph equality and
debug output remain functions of fragments alone, while clones preserve the
cache because their fragment indices and exact geometry are unchanged.
Overlap-evidence clones share their immutable record vector.

When an exact overlap scan produces no split boundaries, linear and rational
refinement now preserve the graph directly and emit one exact unit-range
provenance record per fragment. This avoids rebuilding every fragment,
resorting `[0, 1]` boundaries, and then rescanning the identical refined graph.
Nonempty overlap split sets still run the complete existing construction and
validation path.

Same-machine release A/B measurements against commit `95e10c5`, with 1,024
curves and 20 iterations, were:

| Exact arrangement workload | Before | After | Change |
| --- | ---: | ---: | ---: |
| Materialized tangent traversal | 4.251 ms/iter | 3.848 ms/iter | 9.5% faster |
| Complete retained-overlap workflow | 280.502 ms/iter | 17.750 ms/iter | 93.7% faster |
| Reversed overlap cancellation | 217.594 us/iter | 20.244 us/iter | 90.7% faster |

A separate one-iteration run, which includes the cold classification, reduced
the complete workflow from 302.255 to 95.639 ms. In the isolated scan lane the
first 1,024-curve classification took 55.115 ms and immediate retained access
took 60 ns. The tangent row confirms that retaining overlap evidence does not
regress the endpoint index. Relative to the original 682.674 ms arrangement
checkpoint, the repeated complete workflow is cumulatively 97.4% faster.

The benchmark supports isolated profiling through
`HYPERCURVE_BENCH_ARRANGEMENT_GROUP`: `overlap-scan`,
`cold-splitting-overlap`, `tangent-order`, and `full-overlap`. With no group it
retains the complete original benchmark sequence.

### Retained all-family Boolean topology

The all-family pathological cell now decides every exact Boolean operation,
but its rational-quadratic algebraic split endpoints exposed another retained-
proof bottleneck. One endpoint image was constructed independently for both
adjacent fragments, and the public split-materialization validator rebuilt it
again. Each of the four Boolean operations then rebuilt the same events,
splits, representative-point locations, and endpoint images. Generated graph
and region carriers replayed the public endpoint-provenance validators yet
again.

Algebraic endpoint images are now one-word clone-shared immutable handles.
Generated split boundaries construct each image once, while the public split,
arrangement, loop, and region constructors retain their complete forged-
evidence validation. The private Boolean pipeline transfers the proof from
sorted split construction through arrangement traversal into its output region.
Its operation-independent contact topology, split fragments, and exact
inside/outside/boundary classifications are retained once and shared by union,
intersection, difference, and XOR.

The one-cell release benchmark against commit `dc0e884` changed as follows:

| Exact all-family workload | Before | After | Change |
| --- | ---: | ---: | ---: |
| Prepare and materialize all four `CurveRegion2` Booleans | 4.315 s | 551.721 ms | 87.2% faster |
| Candidate pairs / decided operations | 9 / 4 | 9 / 4 | unchanged |

The after value is the median of three complete process runs; all runs produced
the same six-boundary checksum with no blocker. One instrumented run separated
42.7 ms of pair preparation from operation times of 341.3 ms, 0.315 ms,
0.340 ms, and 155.1 ms. The first union includes construction of the shared
topology; intersection and difference then consume it directly, while XOR
retains more boundary and still performs the larger tangent traversal. The
exact polyline projection measured 0.646 ms in the matched median run, so
native all-family parity remains an optimization target rather than a completed
claim.

### Loop-wide exact Boolean classification

The retained topology profile after commit `a19a50a` showed three kinds of
operation-independent proof being recomputed eagerly:

- tangent ordering constructed algebraic squared norms and cross products
  before checking whether certified coordinate enclosures already decided
  their signs;
- every rational algebraic split endpoint constructed second- and
  third-derivative images even though only a same-tangent branch needs them;
- every split fragment independently ran a complete exact point-in-region ray
  classification, despite the retained contact topology already proving where
  a regular boundary crossing changes inside/outside state.

Arrangement traversal now has a sign-only exact enclosure route while the
public evidence API still constructs and returns represented scalar roots.
Private refined rational splits retain point and first-derivative evidence and
reconstruct higher derivatives from their retained source only at an actual
same-tangent tie. Public endpoint constructors remain eager. Exact fragment
classification starts once per connected boundary loop, remains unchanged
across ordinary authored vertices, and toggles only across a unique strict-
interior contact whose algebraic tangent cross is certified nonzero. Tangencies,
multi-contact vertices, and overlap endpoints conservatively run the full
classifier.

The algebraic parameter certificate is also a one-word clone-shared immutable
handle. Decided rational-root reconstruction, including a decided irrational
result, is retained across parameter clones and certified interval refinement;
uncertain policy-dependent results are not cached.

The same one-cell all-family release workload changed as follows:

| Exact all-family workload | `a19a50a` | Current | Change |
| --- | ---: | ---: | ---: |
| Prepare and materialize all four `CurveRegion2` Booleans | 551.721 ms | 126.015 ms | 77.2% faster |
| Exact representative-point classifications / split fragments | 48 / 48 | 2 / 48 | 95.8% fewer |
| Candidate pairs / decided operations / checksum | 9 / 4 / 6 | 9 / 4 / 6 | unchanged |

The current value is the median of three complete process runs. Its median
pair preparation was 43.486 ms; operation times were 80.907 ms, 0.218 ms,
0.212 ms, and 0.879 ms. The matched exact polyline projection median was
0.525 ms. This checkpoint is 97.1% faster than the earlier 4.315 s all-family
baseline, but the remaining roughly 240x native gap still gates removal of the
embedded line/arc accelerator. The next profile frontier is split endpoint
point/tangent rational-image construction and initial retained-pair
preparation.

### Lazy retained algebraic endpoint evidence

The next frame-pointer profile attributed 59.7% of the complete workload to
refined split construction. Rational point and derivative image resultants
accounted for 43.7% of the process even though most endpoint coordinates were
used only to reconnect fragments whose exact topology vertex identities were
already retained. Refined splitting also spent about one third of the complete
run trying to reconstruct nonlinear algebraic roots as represented rationals
before putting the irrational roots back into algebraic carriers.

Private rational split endpoints now retain the source curve, algebraic
parameter, and policy, and construct point or first-derivative images on first
observation. Public endpoint constructors and public `is_transformed`
observation remain eager and validate the same exact evidence. Retained
arrangement traversal treats its internal topology vertex identity as the
primary connectivity certificate; exact coordinate keys remain the fallback
for public or untagged fragments. Tangents are reconstructed from the retained
source only at a genuine multi-successor branch or at a unique contact whose
transversality proof is needed for loop-wide classification. Reversal carries
the derivative source and applies the existing odd-derivative sign rule.

The private Boolean split path also leaves nonlinear algebraic roots
algebraic. It no longer pays an exhaustive rational-root reconstruction merely
to discover that an intersection parameter is irrational. Public split APIs
retain their existing promotion behavior, including exact materialization of
represented nonlinear rational roots. Rational point and derivative image
results are clone-shared one-word handles and are cached with their algebraic
parameter. General rational and rational-quadratic caches remain distinct
because the two public curve families intentionally expose different
coefficient certificates for the same affine derivative.

The same one-cell all-family release workload changed as follows:

| Exact all-family workload | Previous | Current | Change |
| --- | ---: | ---: | ---: |
| Prepare and materialize all four `CurveRegion2` Booleans | 126.015 ms | 85.632 ms | 32.0% faster |
| First complete union, including shared-topology construction | 80.907 ms | 37.256 ms | 53.9% faster |
| Exact representative-point classifications / split fragments | 2 / 48 | 2 / 48 | unchanged |
| Candidate pairs / decided operations / checksum | 9 / 4 / 6 | 9 / 4 / 6 | unchanged |

The current value is the median of three sequential complete process runs
(88.625, 85.632, and 81.441 ms). Pair preparation remained around 43--47 ms;
the other three operations were each below 1 ms. The matched exact polyline
projection median was 0.525 ms, leaving a roughly 163x complete native gap.
Relative to the earlier 4.315 s all-family baseline, the cumulative reduction
is 98.0%.

The post-change profile puts initial retained-pair preparation at 50.4% and
the two required algebraic contact-tangent constructions at 34.0%; refined
split construction is down to 4.9%. These are the next exact-work frontiers,
and the native line/arc accelerator therefore remains justified. All feature
and default test matrices, strict all-feature Clippy, warning-denied rustdoc,
library and UI WASM builds, and UI tests/Clippy passed. Six AddressSanitizer
fuzz targets covering algebraic parameters, algebraic images, split
materialization, arrangement, retained regions, and region Booleans completed
at least 1,000 executions without a library failure. The image campaign found
and corrected an overstrong fuzz assertion: a positive rational denominator
does not imply that an irrational coordinate image is monotone enough to
transform, so `XImageFailed` or `YImageFailed` remains valid exact evidence.

### Retained simple-crossing certificates

The implicit-conic intersection route already isolates the roots of the
cleared exact substitution polynomial. A simple root of that polynomial,
together with the route's nonsingular conic frame and certified nonzero
rational denominators, proves that the two affine curve images are transverse.
That proof is now retained on rational-Bezier and top-level curve contacts.
Multiple roots and undecided multiplicity remain on the existing exact tangent
fallback; a degree-elevated tangent-line regression verifies that they are not
misclassified.

Curved-region topology consumes the retained proof in two places. It toggles
the loop-wide inside/outside classification without constructing algebraic
tangent coordinate images. For Boolean arrangement traversal, the already
retained before/after region classifications and filled-side orientation
recover the sign of the tangent cross product. Fragment reversal then supplies
the remaining direction signs needed by the existing half-turn ordering.
Only a unique strict-interior two-carrier crossing uses this shortcut; all
tangencies, overlaps, multi-contact vertices, and incomplete local
classifications retain tangent-order fallback. Start vertices are indexed once,
so the shortcut adds linear rather than quadratic graph work.

The same one-cell all-family release workload changed as follows:

| Exact all-family workload | Previous | Current | Change |
| --- | ---: | ---: | ---: |
| Prepare and materialize all four `CurveRegion2` Booleans | 85.632 ms | 55.178 ms | 35.6% faster |
| First complete union, including shared-topology construction | 37.256 ms | 9.657 ms | 74.1% faster |
| Exact representative-point classifications / split fragments | 2 / 48 | 2 / 48 | unchanged |
| Candidate pairs / decided operations / checksum | 9 / 4 / 6 | 9 / 4 / 6 | unchanged |

The current values are medians of five sequential complete process runs. The
complete runs were 55.178, 58.821, 55.089, 57.498, and 54.230 ms; median pair
preparation was 44.420 ms. Median operation times were 9.657, 0.226, 0.228,
and 0.460 ms for union, intersection, difference, and XOR. The matched exact
polyline projection median was 0.484 ms, leaving a roughly 114x complete native
gap. Relative to the earlier 4.315 s all-family baseline, the cumulative
reduction is 98.7%.

The fresh frame-pointer profile no longer shows algebraic contact-tangent
construction as a material frontier. About four fifths of measured wall time
is now pair preparation, with exact rational normalization dominating self
cycles; initial topology construction accounts for most of the remainder.
The native line/arc accelerator therefore remains justified. Complete default
and all-feature tests, strict all-feature Clippy, warning-denied rustdoc,
library and UI WASM builds, and the 32-test UI suite passed. Seven
AddressSanitizer campaigns covering algebraic parameters and images, split
materialization, tangent ordering, arrangements, retained regions, and region
Booleans completed at least 1,000 executions without a finding.

### Batched square-free contact certification

The retained crossing proof initially classified every isolated contact root
independently. Algebraic roots of one implicit substitution share the same
defining polynomial, so this repeated the exact derivative construction and
`gcd(P, P')` calculation for every root. Multiplicity classification now
builds that square-free evidence once per polynomial. A constant GCD certifies
the whole batch as simple; if repeated roots exist, one Sturm sequence for the
repeated factor is reused across the disjoint root isolators. Represented roots
still validate the polynomial and derivative directly, and any undecidable
GCD, sign, or interval count remains uncertified.

A controlled release comparison rebuilt the committed and batched sources
against the same dependency state. On the one-cell all-family workload, ten
`perf stat` process runs reduced exact benchmark instructions from 653,107,010
to 650,584,843 (0.39%). Median complete Boolean time fell from 52.778 ms to
52.030 ms (1.42%), while median pair preparation fell from 42.168 ms to
41.783 ms (0.91%). All runs retained 9 candidate pairs, 48 fragments, 2 exact
point classifications, 4 decided operations, and checksum 6. The optimization
does not change the roughly two-orders-of-magnitude gap to the exact polyline
projection, so the native line/arc accelerator remains justified. Focused
AddressSanitizer campaigns completed 1,000 algebraic-parameter executions and
2,509 region-Boolean corpus executions without a library finding.

### Modular rejection of irrational parameter roots

Exact root isolation promotes represented rational roots before returning its
ordered parameter set. For an irrational root, the former path refined its
Sturm isolator until the rational-root-theorem denominator bound made continued
fraction reconstruction unique, only to replay and reject the candidate. This
was useful work for rational roots but dominated implicit conic intersection
preparation when every contact parameter was irrational.

The rational-coefficient polynomial is already cleared to a primitive integer
polynomial to derive that denominator bound. Root isolation now reduces those
integer coefficients modulo a fixed set of small primes. For any prime that
does not divide the primitive leading coefficient, a reduced rational root
would have an invertible denominator and therefore appear as a finite-field
root. A single rootless reduction is consequently an exact proof that the
polynomial has no rational root. Polynomials that survive every tested prime
run the unchanged bounded reconstruction and exact replay. The resulting
denominator decision is also shared by all isolated roots of one polynomial.

A controlled ten-run `perf stat` comparison against commit `abc2169`, using
the same dependency build, reduced instructions from 650,584,843 to
409,730,483 (37.0%). Median instrumented complete Boolean time fell from
52.030 ms to 33.939 ms (34.8%), and median pair preparation fell from
41.783 ms to 23.361 ms (44.1%). Five ordinary release runs had a 32.513 ms
complete median and a 22.445 ms preparation median; the exact polyline
projection median was 0.440 ms. Every run retained 9 candidate pairs,
48 fragments, 2 exact point classifications, 4 decided operations, and
checksum 6. The remaining roughly 74x native gap still justifies the line/arc
accelerator. Complete default and all-feature tests, strict all-feature
Clippy, warning-denied rustdoc, and the library WASM build passed. Focused
AddressSanitizer campaigns completed 1,000 algebraic-parameter executions and
2,509 region-Boolean corpus executions without a finding.

### Lazy alternate contact-point evidence

Implicit-conic contact replay asks the conic operand for exact point evidence
first and uses the general rational operand only when that construction cannot
complete. The implementation expressed this priority with `Option::or`, whose
argument is eager, so both algebraic point images and their resultant
determinants were constructed even when the first was retained. Replay now
evaluates the alternate operand only after a missing first result. The selected
public evidence and fallback order are unchanged.

A controlled ten-run `perf stat` comparison against commit `fa10383` reduced
instructions from 409,730,483 to 383,444,180 (6.42%). Median instrumented
complete Boolean time fell from 33.939 ms to 30.804 ms (9.24%), and median
pair preparation fell from 23.361 ms to 20.410 ms (12.6%). Five ordinary
release runs had a 30.524 ms complete median, a 20.414 ms preparation median,
and a 0.428 ms exact-polyline projection median. All topology counts and the
six-boundary checksum remained unchanged; the resulting roughly 71x native
gap still retains the line/arc accelerator. The complete all-feature test
matrix, strict all-feature Clippy, warning-denied rustdoc, and a 2,509-run
AddressSanitizer region-Boolean campaign passed.

### Adaptive conic-parameter isolation

Mapping an implicit conic contact back to the conic's parameter requires a
monotone rational image of the other curve's algebraic parameter. The former
path bisected every source isolator eight times before attempting that proof.
Those extra bisections can help difficult maps, but most contacts need much
less separation. The mapping now tries two certified bisections first and
escalates through four and the original eight only when the exact rational
image remains undecided. Exact parameters still take the single direct path.
No approximate parameter or sampled fallback is introduced.

A controlled ten-run `perf stat` comparison against commit `6cb7ed9` reduced
instructions from 383,444,180 to 320,660,631 (16.4%). Median instrumented
complete Boolean time fell from 30.804 ms to 28.137 ms (8.66%), and median
pair preparation fell from 20.410 ms to 18.095 ms (11.3%). Every run retained
9 candidate pairs, 48 fragments, 2 exact point classifications, 4 decided
operations, and checksum 6. A zero-refinement control correctly remained
undecided on the same workload, while the escalating path preserves the former
eight-step certification budget for such inputs. Five ordinary release runs
had a 26.628 ms complete median, a 16.830 ms preparation median, and a
0.442 ms exact-polyline projection median. The remaining roughly 60x native
gap still justifies the line/arc accelerator. Complete default and all-feature
tests, strict all-feature Clippy, warning-denied rustdoc, the library WASM
build, and a 2,509-run AddressSanitizer region-Boolean campaign passed.

### Primitive integer rational-image elimination

Algebraic rational images were feeding exact fractional coefficients directly
into every sampled Sylvester determinant. A resultant's roots are invariant
when either input polynomial is multiplied by a nonzero constant, so
Hypersolve now clears the source polynomial to primitive integers and clears
the rational map's numerator and denominator with one shared primitive integer
scale. The original rational map still supplies its denominator-domain proof
and exact interval endpoints. Hyperreal also provides a checked exactly
divisible integer quotient for Bareiss' fraction-free recurrence, with the
former general exact division retained as fallback.

The combined cross-crate change reduced ten-run instructions from 320,660,631
to 189,533,986 (40.9%). Denominator clearing accounted for the first reduction
to 229,530,874, checked integer Bareiss division reached 192,745,230, and
primitive content removal supplied the remainder. Five ordinary release runs
had an 18.154 ms complete median, a 9.016 ms preparation median, and a
0.429 ms exact-polyline projection median. Every run retained 9 candidate
pairs, 48 fragments, 2 exact point classifications, 4 decided operations, and
checksum 6. The remaining roughly 42x native gap still justifies the line/arc
accelerator. Complete all-feature tests, strict all-feature Clippy,
warning-denied rustdoc, the release library WASM build, and a 2,509-run
AddressSanitizer region-Boolean campaign passed.

### Integer-scaled images and adaptive split refinement

The sibling polynomial-image path still formed fractional Sylvester matrices,
and every sampled resultant path reconstructed its power basis with rational
Lagrange divisions. Hypersolve now normalizes polynomial, rational, and binary
algebraic inputs to primitive integers. It interpolates their integer
resultant samples through forward differences while retaining one harmless
common factorial scale, then removes content once. Polynomial images scale the
symbolic image variable together with the mapped coefficients, while rational
images retain one shared numerator/denominator scale, so neither transformation
changes the represented algebraic value.

Split-carrier construction also no longer refines every algebraic boundary
eight times before attempting exact materialization. It first tries one
certified refinement step, then escalates through two, four, and the original
eight whenever the complete split, endpoint-image, or topology-vertex replay
does not succeed. Successful evidence is still exact, and difficult inputs
retain the former proof budget and final error behavior.

Primitive polynomial-image relations reduced ten-run instructions from
189,533,986 to 176,631,590, and shared integer-scaled interpolation reached
165,927,095. Adaptive carrier refinement then reached 151,620,313: 20.0% below
the preceding committed checkpoint and 52.7% below the 320,660,631 adaptive
conic-image baseline. Eleven ordinary release runs had a 14.994 ms complete
median, an 8.682 ms preparation median, and a 0.427 ms exact-polyline
projection median. Every run retained 9 candidate pairs, 48 fragments, 2 exact
point classifications, 4 decided operations, and checksum 6. The remaining
roughly 35x native gap still justifies the line/arc accelerator. Complete
all-feature tests, strict all-feature Clippy, warning-denied rustdoc, the
release library WASM build, and a 2,509-run AddressSanitizer region-Boolean
campaign passed.

### Fused integer Bareiss updates

Primitive integer Sylvester matrices still passed every Bareiss recurrence
through three separate rational operations: two products, a subtraction, and
then the checked quotient. Hyperreal now provides one checked integer
cross-difference quotient, and Hypersolve uses it across determinant, dense,
multi-right-hand-side, and sparse elimination. The operation constructs only
the signed integer cross difference and its exactly divisible quotient.
Fractional operands and any failed divisibility proof retain the former general
exact `Real` fallback.

Ten-run instructions fell from 151,620,313 to 131,393,603 (13.3%), or 59.0%
below the 320,660,631 adaptive conic-image baseline. Eleven ordinary release
runs had a 12.985 ms complete median, a 7.844 ms preparation median, and a
0.407 ms exact-polyline projection median. Every run retained 9 candidate
pairs, 48 fragments, 2 exact point classifications, 4 decided operations, and
checksum 6. The remaining roughly 32x native gap still retains the line/arc
accelerator.
The complete all-feature suite, strict all-target Clippy, warning-denied
rustdoc, and release WASM library build passed. A 2,509-run AddressSanitizer
region-Boolean campaign completed without failure at 5,894 coverage points and
19,170 feature edges; LeakSanitizer alone remained disabled under ptrace.

### Retained algebraic-root refinement

Each curved split boundary previously re-entered Hypersolve's general isolated-
root refinement API. That correctly rebuilt a square-free polynomial and Sturm
sequence, but Hypercurve had already certified the defining polynomial and
singleton interval. Algebraic parameters now retain one successful Sturm
sequence across clones, carry the isolation-time sequence directly into every
root produced by that pass, and bisect through Hypercurve's existing certified
singleton refinement. A represented midpoint root is still promoted exactly;
uncertain work retains the original isolator.

Refined intervals share the source root's private identity. `same_value` uses
that identity as an exact certificate when topology maps a refined split back
to its original event. Structural `PartialEq` deliberately remains unchanged,
preserving its transitivity for independently constructed equal isolators.
Square-free and repeated-root regressions compare three local refinement steps
with the former Hypersolve reference, prove the interval changed, and prove
clone-shared value identity and Sturm retention.

The same-clean-tree Callgrind A/B fell from 126,924,875 to 91,848,172
instructions (27.6%). The final ten-run median was 91,844,448 instructions,
30.1% below the preceding 131,393,603 checkpoint and 71.4% below the
320,660,631 adaptive conic-image baseline. Eleven ordinary runs had a 9.517 ms
complete median, a 6.354 ms preparation median, and a 0.392 ms exact-polyline
projection median. Every run retained 9 candidate pairs, 48 fragments, 2 exact
point classifications, 4 decided operations, and checksum 6.

Clean matched Heaptrack runs fell from 155,590 to 138,889 allocations (10.7%)
and from 11,438 to 9,916 temporary allocations (13.3%); peak heap fell from
2.06 to 2.00 MiB. In the final repeated frame-pointer profile, pair
intersection preparation accounts for 41.6%, resultant construction 20.3%,
split carriers 15.6%, and remaining Sturm construction 6.4%. The roughly 24x
native projection gap still retains the line/arc accelerator.
The complete all-feature and no-default suites, strict all-target Clippy,
warning-denied rustdoc, and release WASM library build passed. The requested
2,509-run AddressSanitizer region-Boolean campaign completed without failure
at 5,897 coverage points and 19,162 feature edges; LeakSanitizer alone remained
disabled under ptrace.

### Sign-certified odd-root refinement

A singleton algebraic isolator whose defining polynomial has opposite nonzero
signs at its endpoints contains an odd-multiplicity root. Exact midpoint
evaluation therefore selects the unique sign-changing child without a Sturm
count. Hypercurve now tries that continuous-polynomial certificate before its
retained Sturm fallback. Midpoint roots still promote to represented exact
parameters, while equal endpoint signs, undecided signs, and even-multiplicity
roots retain the former path.

The repeated-root reference regression now exercises both branches: the simple
quadratic refines three times without constructing a Sturm sequence, while its
squared companion reuses one retained sequence and matches Hypersolve's
square-free reference interval exactly.

The final ten-run instruction median fell from 91,844,448 to 85,201,993 (7.2%),
73.4% below the 320,660,631 adaptive conic-image baseline. Eleven ordinary
runs had an 8.606 ms complete median, a 6.084 ms preparation median, and a
0.390 ms exact-polyline projection median. Every run retained 9 candidate
pairs, 48 fragments, 2 exact point classifications, 4 decided operations, and
checksum 6. Heaptrack fell from 138,889 to 133,767 allocations, from 9,916 to
9,677 temporary allocations, and from 2.00 to 1.92 MiB peak heap. The remaining
roughly 22x projection gap still retains the line/arc accelerator.

The complete all-feature and no-default-feature suites, warning-denied Clippy
and rustdoc, formatting, and the release WASM library build passed. The
requested AddressSanitizer region-Boolean fuzz replay completed all 2,509 runs
at 5,893 coverage points and 19,169 feature edges; LeakSanitizer alone remained
disabled under ptrace.

### Quotient-ring algebraic image resultants

Hypersolve now handles affine and linear-fractional algebraic images through
its exact Mobius substitution. General polynomial and rational maps construct
the numerator and denominator multiplication matrices once in `Q[x] / (P)`,
clear them with one shared scale, and interpolate exact relation norms from
`deg(P)`-dimensional Bareiss determinants. This replaces repeated six- or
seven-dimensional Sylvester determinants with four-dimensional determinants
for the sentinel's quartic parameter evidence. The defining polynomial changes
only by a nonzero global scale; a generated test compares the new samples with
the retained Sylvester fallback for cubic sources and quadratic-over-linear
maps.

The ten-run instruction median fell from 85,201,993 to 79,151,572 (7.1%),
75.3% below the original 320,660,631 baseline. Eleven ordinary runs had an
8.625 ms complete median, a 6.272 ms preparation median, and a 0.393 ms
exact-polyline projection median. All runs retained 9 candidate pairs,
48 fragments, 2 exact point classifications, 4 decided operations, no
blockers, and checksum 6. Heaptrack fell from 133,767 to 119,861 allocations
and from 9,677 to 7,949 temporary allocations; measured peak heap moved from
1.92 to 1.97 MiB. The roughly 22x projection gap still retains the line/arc
accelerator.

The complete Hypersolve and Hypercurve all-feature and no-default-feature
suites, formatting, warning-denied Clippy and rustdoc, and release WASM library
builds passed. The requested AddressSanitizer region-Boolean fuzz replay
completed at 2,512 executions with 5,897 coverage points and 19,158 feature
edges; LeakSanitizer alone remained disabled under ptrace.

### Fraction-free quotient reduction

Hypersolve's quotient-basis construction now pseudo-reduces primitive integer
source and map polynomials without rational division. Scaling every quotient
column by the same fixed power of the source leading coefficient preserves the
numerator/denominator norm samples up to one common nonzero factor. The
nonmonic and interpolation-degree-cancellation regressions exercise that
invariant; unsupported coefficient forms retain the Sylvester fallback.

The ten-run instruction median fell from 79,151,572 to 78,335,067 (1.0%),
75.6% below the original baseline. Eleven ordinary runs had an 8.458 ms
complete median, a 6.073 ms preparation median, and a 0.430 ms exact-polyline
projection median. Every run retained the same 9 candidate pairs, 48
fragments, 2 classifications, 4 decided operations, no blockers, and checksum
6. Heaptrack fell from 119,861 to 116,469 allocations and from 7,949 to 6,834
temporary allocations while peak heap remained 1.97 MiB.

The complete Hypersolve and Hypercurve feature matrices, warning-denied Clippy
and rustdoc, formatting, and release WASM library builds passed. The requested
AddressSanitizer region-Boolean fuzz replay completed all 2,509 executions at
5,895 coverage points and 19,157 feature edges; LeakSanitizer alone remained
disabled under ptrace.

### Homogeneous Horner Mobius images

Hypersolve now constructs exact-rational Mobius image polynomials with a
homogeneous Horner recurrence. For inverse linear forms `A(y) = d*y - b` and
`B(y) = a - c*y`, it tracks the growing power of `B` while repeatedly forming
`A*H + p_k*B^j`. This produces the same
`B(y)^n P(A(y) / B(y))` as the retained power-sum construction without
rebuilding every pair of powers. A dedicated two-diagonal convolution handles
the known-linear multipliers. Non-exact-rational coefficients keep the former
general `Real` path.

A fixed degree-five regression and generated degree-zero-through-five
exact-rational cases compare both construction schedules directly. On the
one-cell all-family exact Boolean sentinel, the ten-run instruction median
fell from 78,335,067 to 77,532,932 (1.0%), 75.8% below the original
320,660,631 baseline. The specialized linear convolution contributed a further
0.10% reduction from the generic Horner version. Eleven ordinary runs had an
8.593 ms complete median, a 6.437 ms preparation median, and a 0.409 ms
exact-polyline projection median. Heaptrack recorded 115,778 allocations,
6,834 temporary allocations, 1.96 MiB peak heap, and 13.01 MiB peak RSS.
Every run retained 9 candidate pairs, 48 fragments, 2 classifications,
4 decided operations, no blockers, and checksum 6.

The complete Hypersolve and Hypercurve all-feature and no-default-feature
suites, formatting, warning-denied all-target Clippy and rustdoc, and release
WASM library builds passed. The requested AddressSanitizer region-Boolean fuzz
replay completed all 2,509 executions at 5,895 coverage points and 19,165
feature edges; LeakSanitizer alone remained disabled under ptrace.

### Two-observation rational linear caches

Hyperreal no longer treats an ownership clone by itself as proof that the same
exact sum or directed difference will recur. A first arithmetic observation
records the existing compact reuse fact, a second admits a bounded cache
entry, and later calls reuse it. Existing product or linear caches remain
immediate evidence. The known-repeat dense self-dot path explicitly primes its
sum intermediates, preserving its retained-result behavior.

This policy fits Hypercurve's immutable carriers: coefficient and endpoint
values are often cloned into several owners but paired only once during
elimination, root isolation, area construction, or topology replay. Hyperreal's
matched fresh-but-cloned scalar sentinels improved add from 111.87 to 93.85 ns
(16.1%) and subtract from 116.40 to 97.04 ns (17.1%); retained and fresh
unshared controls did not regress.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 77,532,932 to 76,301,712 (1.6%), 76.2% below the original
320,660,631 baseline. Heaptrack allocations fell from 115,778 to 114,193,
allocations beneath `retain_linear` from 7,971 to 3,718, and peak heap from
1.96 to 1.53 MiB. Temporary allocations measured 6,833 and peak RSS was
12.60 MiB. Eleven ordinary runs had an 8.840 ms complete median, a 6.630 ms
preparation median, and a 0.376 ms exact-polyline projection median. All runs
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete Hyperreal, Hypersolve, and Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The requested AddressSanitizer
region-Boolean fuzz replay completed all 2,509 executions at 5,903 coverage
points and 19,183 feature edges; LeakSanitizer alone remained disabled under
ptrace.

### Flat integer quotient determinants

Hypersolve's quotient-ring image path now evaluates each already-flat
exact-integer multiplication matrix with a private flat Bareiss kernel. It
avoids rebuilding nested `Real` rows and constructing the public determinant
report for every interpolation sample. The recurrence retains Hyperreal's
checked exact integer cross-difference quotient. A failed shape check or exact
division returns `None`, preserving the established Sylvester-resultant
fallback.

Fixed zero-, one-, two-, and three-dimensional cases plus generated
four-by-four integer matrices compare the flat result with the public
report-bearing determinant. The existing generated quotient-ring/Sylvester
comparison continues to cover the caller. On the one-cell all-family exact
Boolean sentinel, the ten-run instruction median fell from 76,301,712 to
74,732,427 (2.1%), 76.7% below the original 320,660,631 baseline.

Heaptrack allocations fell from 114,193 to 112,178; temporary allocations
remained 6,833, peak heap remained 1.53 MiB, and peak RSS was 12.51 MiB.
Eleven ordinary runs had an 8.004 ms complete median, a 5.749 ms preparation
median, and a 0.379 ms exact-polyline projection median. Every run retained
9 candidate pairs, 48 fragments, 2 classifications, 4 decided operations,
no blockers, and checksum 6.

The complete Hypersolve and Hypercurve all-feature and no-default-feature
suites, formatting, warning-denied all-target Clippy and rustdoc, and release
WASM library builds passed. The requested AddressSanitizer region-Boolean fuzz
replay completed all 2,509 executions at 5,890 coverage points and 19,156
feature edges; LeakSanitizer alone remained disabled under ptrace.

### Fused integer quotient samples

Hyperreal now provides a checked exact integer scaled difference that computes
`a - b*k` directly from integer magnitudes and a signed machine-word scale.
Hypersolve uses it for every flat quotient-ring interpolation entry `N - y*D`,
avoiding a general rational product, reduction, cache admission, and
subtraction. Fractional inputs return `None` and preserve the established
Sylvester-resultant fallback.

Exhaustive small signed cases, fractional rejection, and a 192-bit scalar case
cover the primitive. Fixed and generated determinant/resultant equivalence
continues to cover the caller. Matched fresh 192-bit Criterion sentinels
measured the composed multiply then subtract at 310.13 ns and the fused
operation at 102.50 ns, a 67.0% reduction.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 74,732,427 to 72,782,675 (2.6%), 77.3% below the original
320,660,631 baseline. Heaptrack allocations fell from 112,178 to 108,842 and
temporary allocations from 6,833 to 6,236; peak heap remained 1.53 MiB and
peak RSS was 12.55 MiB. Eleven ordinary runs had a 7.672 ms complete median,
a 5.501 ms preparation median, and a 0.339 ms exact-polyline projection
median. Every run retained 9 candidate pairs, 48 fragments, 2 classifications,
4 decided operations, no blockers, and checksum 6.

The complete Hyperreal, Hypersolve, and Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The requested AddressSanitizer
region-Boolean fuzz replay completed all 2,509 executions at 5,895 coverage
points and 19,144 feature edges; LeakSanitizer alone remained disabled under
ptrace.

### Unit-divisor Bareiss updates

Hyperreal's checked integer cross-difference quotient now applies the sign and
returns directly when the divisor magnitude is one. All exact-integer and
nonzero-divisor guards remain in force, but the first Bareiss stage no longer
invokes big-integer division. Positive and negative unit-divisor tests
supplement the existing exhaustive signed/divisibility coverage.

A matched fresh 192-bit Criterion sentinel measured the composed
multiply/subtract/divide at 625.27 ns and the fused unit-divisor path at
190.15 ns, a 69.6% reduction. On the one-cell all-family exact Boolean
sentinel, the ten-run instruction median fell from 72,782,675 to 72,479,577
(0.4%), 77.4% below the original 320,660,631 baseline.

Heaptrack allocations fell from 108,842 to 107,461; temporary allocations
remained 6,236, peak heap remained 1.53 MiB, and peak RSS was 12.65 MiB.
Eleven ordinary runs had a 7.552 ms complete median, a 5.446 ms preparation
median, and a 0.353 ms exact-polyline projection median. Every run retained
9 candidate pairs, 48 fragments, 2 classifications, 4 decided operations,
no blockers, and checksum 6.

The complete Hyperreal, Hypersolve, and Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The requested AddressSanitizer
region-Boolean replay completed its 2,509-run budget after 2,513 executions at
5,900 coverage points and 19,165 feature edges; LeakSanitizer alone remained
disabled under ptrace.

### Fused quotient-basis pseudo-reduction

Hypersolve now evaluates every affected pseudo-reduction coefficient as one
checked exact integer cross difference,
`leading*value - source*eliminand`. Only the untouched low-degree prefix is
scaled separately, and a unit source leading coefficient leaves that prefix
unchanged. Shifted relation coefficients are cloned directly instead of being
added to zero. A failed integer check still returns `None` and preserves the
Sylvester-resultant fallback.

The generated quotient-ring/Sylvester property covers the complete schedule.
On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 72,479,577 to 71,182,596 (1.8%), 77.8% below the original
320,660,631 baseline. Heaptrack allocations fell from 107,461 to 105,140;
temporary allocations rose from 6,236 to 6,409, peak heap remained 1.53 MiB,
and peak RSS fell from 12.65 to 12.37 MiB.

Eleven ordinary runs had a 7.446 ms complete median, a 5.430 ms preparation
median, and a 0.343 ms exact-polyline projection median. Every run retained
9 candidate pairs, 48 fragments, 2 classifications, 4 decided operations,
no blockers, and checksum 6.

The complete Hypersolve and Hypercurve all-feature and no-default-feature
suites, formatting, warning-denied all-target Clippy and rustdoc, and release
WASM library builds passed. The requested AddressSanitizer region-Boolean
replay completed all 2,509 executions at 5,893 coverage points and 19,125
feature edges; LeakSanitizer alone remained disabled under ptrace.

### Single-pass Sturm endpoint validation

Each interval root count formerly evaluated the defining polynomial at both
endpoints to reject endpoint roots, then evaluated it again as the first member
of the Sturm variation scan. The first sequence member is the defining
polynomial in every caller, so the scan now rejects a zero first-member sign
itself and reuses that evaluation for variation counting. Later zero sequence
members retain the standard Sturm skip rule.

Fixed endpoint-root rejection and generated interval-count tests preserve the
carrier boundary. On the one-cell all-family exact Boolean sentinel, the
ten-run instruction median fell from 71,182,596 to 69,867,161 (1.8%), 78.2%
below the original 320,660,631 baseline. Heaptrack allocations fell from
105,140 to 103,004; temporary allocations remained 6,409, peak heap remained
1.53 MiB, and peak RSS measured 12.66 MiB.

Eleven ordinary runs had a 7.038 ms complete median, a 5.052 ms preparation
median, and a 0.334 ms exact-polyline projection median. Every run retained
9 candidate pairs, 48 fragments, 2 classifications, 4 decided operations,
no blockers, and checksum 6.

The complete Hypercurve all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy and rustdoc, and the release WASM library
build passed. The requested AddressSanitizer region-Boolean replay completed
its 2,509-run budget after 2,514 executions at 5,893 coverage points and
19,173 feature edges; LeakSanitizer alone remained disabled under ptrace.

### Carried Sturm boundary variations

Unit-interval root isolation now evaluates each initial boundary's complete
Sturm evidence once and carries the resulting variation counts with every
pending interval. A non-root midpoint is likewise scanned once, then its
variation count is shared by both child intervals. The interval root count is
therefore the carried endpoint difference instead of two repeated sequence
evaluations. A midpoint that is itself a root remains distinguished and is
still returned as an exactly represented parameter.

A fixed partition test compares carried variation differences with independent
interval root counts and checks the exact-midpoint-root case. On the one-cell
all-family exact Boolean sentinel, the ten-run instruction median fell from
69,867,161 to 66,871,601 (4.3%), 79.1% below the original 320,660,631
baseline. Heaptrack allocations fell from 103,004 to 98,658, temporary
allocations from 6,409 to 6,389, peak heap from 1.53 to 1.50 MiB, and peak RSS
from 12.66 to 12.41 MiB.

Eleven ordinary runs had a 6.781 ms complete median, a 4.797 ms preparation
median, and a 0.332 ms exact-polyline projection median. Every measured run
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete Hypercurve all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy, all-feature and no-default-feature rustdoc,
and default and no-default release WASM library builds passed. The
AddressSanitizer region-Boolean replay completed all 2,509 executions at 5,894
coverage points and 19,157 feature edges with no finding; LeakSanitizer alone
remained disabled under ptrace.

### Deferred singleton midpoint scans

Root isolation now consumes a certified singleton interval before evaluating
its midpoint's complete Sturm sequence. The retained carrier's rational-root
reconstruction still replays any represented root exactly, including a root at
the midpoint, while an irrational singleton avoids a scan that cannot affect
its already-proved root count. Midpoint Sturm evidence remains mandatory for
intervals that touch represented or unit-domain boundaries and therefore
still require bisection.

The mixed represented/algebraic isolation regression covers the exact
midpoint reconstruction path. On the one-cell all-family exact Boolean
sentinel, the ten-run instruction median fell from 66,871,601 to 66,350,069
(0.78%), 79.3% below the original 320,660,631 baseline. Heaptrack allocations
fell from 98,658 to 97,906 and temporary allocations from 6,389 to 6,367;
peak heap remained 1.50 MiB and peak RSS remained 12.41 MiB.

Eleven ordinary runs had a 6.776 ms complete median, a 4.755 ms preparation
median, and a 0.328 ms exact-polyline projection median. Every measured run
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete Hypercurve all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy, all-feature and no-default-feature rustdoc,
and default and no-default release WASM library builds passed. The
AddressSanitizer region-Boolean replay completed all 2,509 executions at 5,899
coverage points and 19,170 feature edges with no finding; LeakSanitizer alone
remained disabled under ptrace.

### Direct polynomial-remainder leading cancellation

Each long-division step chooses its factor from the current and divisor leading
coefficients, so updating the aligned leading slot can only compute exact zero.
Polynomial remainder now updates the strictly lower divisor coefficients and
pops that algebraically canceled slot directly. The next normalization pass
still classifies and removes any additional lower-order cancellation, so
symbolic uncertainty behavior is unchanged.

The fixed and generated Sturm, GCD, root-count, and algebraic-parameter suites
cover the shared remainder kernel. On the one-cell all-family exact Boolean
sentinel, the ten-run instruction median fell from 66,350,069 to 64,966,544
(2.1%), 79.7% below the original 320,660,631 baseline. Heaptrack allocations
fell from 97,906 to 97,195, temporary allocations from 6,367 to 6,331, peak
heap from 1.50 to 1.49 MiB, and peak RSS from 12.41 to 12.39 MiB.

The clean eleven-run ordinary replacement series had a 6.755 ms complete
median, a 4.691 ms preparation median, and a 0.349 ms exact-polyline projection
median. Every measured run retained 9 candidate pairs, 48 fragments, 2
classifications, 4 decided operations, no blockers, and checksum 6.

The complete Hypercurve all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy, all-feature and no-default-feature rustdoc,
and default and no-default release WASM library builds passed. The requested
AddressSanitizer region-Boolean replay completed after 2,523 executions at
5,896 coverage points and 19,157 feature edges with no finding; LeakSanitizer
alone remained disabled under ptrace.

### Borrowed polynomial divisors and quotient leading cancellation

The private polynomial-remainder kernel now borrows its divisor from the
parameter polynomial, GCD loop, or Sturm sequence. It scans trailing
coefficients in that slice with the same exact zero classifier instead of
cloning and normalizing an owned vector, and divides borrowed leading
coefficients. The coupled Hypersolve quotient-ring multiplication likewise
omits its highest pseudo-reduction update: the chosen quotient coefficient
makes that term identically `leading*eliminand - leading*eliminand`, so it is
assigned exact zero directly. Lower coefficients, integer guards, and the
Sylvester fallback remain unchanged.

The fixed and generated Sturm, GCD, root-count, algebraic-parameter, and
quotient-ring/Sylvester comparisons cover both changes. On the one-cell
all-family exact Boolean sentinel, the ten-run instruction median fell from
64,966,544 to 64,678,125 (0.44%), 79.8% below the original 320,660,631
baseline. Heaptrack allocations fell from 97,195 to 96,817 and temporary
allocations from 6,331 to 6,165; peak heap remained 1.49 MiB and peak RSS
remained 12.39 MiB.

Eleven ordinary runs had a 6.967 ms complete median, a 4.841 ms preparation
median, and a 0.337 ms exact-polyline projection median. Every measured run
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete Hypersolve and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy,
all-feature and no-default-feature rustdoc, and default and no-default release
WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 executions at 5,892 coverage points and 19,170 feature
edges with no finding; LeakSanitizer alone remained disabled under ptrace.

### Primitive integer Sturm pseudo-remainders

Rational GCD and Sturm construction now clear each coefficient ratio directly
to primitive `BigInt` values and use division-free pseudo-remainders. Each
elimination step cancels its leading term in place. Because a negative divisor
leading coefficient contributes one sign per step, the kernel corrects that
parity before removing positive integer content; every retained Sturm member
is therefore a positive multiple of the ordinary field remainder and has the
same sign variations. The existing field-remainder kernel remains in place
for value-preserving power-basis reduction and for any symbolic coefficient.

Exact-rational polynomial evaluation now feeds Horner's two terms through
Hyperreal's retained fixed product-sum accumulator. A direct regression
compares the primitive result with field division for fractional inputs,
negative leading coefficients, both step parities, and an exact zero
remainder. The fixed and generated GCD, Sturm, root-count, root-isolation,
algebraic-parameter, and downstream Boolean suites cover the complete
dispatch.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 64,678,125 to 61,647,633 (4.7%), 80.8% below the original
320,660,631 baseline. Root isolation's inclusive profile fell from about
4.52 million to 2.22 million instructions. Heaptrack allocation events rose
from 96,817 to 98,024 and temporary events from 6,165 to 6,461, while peak
heap fell from 1.49 to 1.41 MiB; peak RSS measured 12.45 MiB.

Eleven ordinary runs had a 6.229 ms complete median, a 4.193 ms preparation
median, and a 0.332 ms exact-polyline projection median. Every measured run
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete Hyperreal, Hypersolve, and Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy,
all-feature and no-default-feature rustdoc, and default and no-default release
WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 executions at 5,896 coverage points and 19,157 feature
edges with no finding; LeakSanitizer alone remained disabled under ptrace.

### Direct integer quotient resultants and interpolation

Hypersolve's quotient-ring resultant path now converts exact coefficients to
`BigInt` before pseudo-reduction and retains that representation through the
sampled Bareiss determinants. One determinant matrix is overwritten for every
sample, and one quotient-product buffer is reused across columns with reduced
coefficients moved into the multiplication matrix. Exact division remainders
are still checked, and inputs outside the exact integer boundary still select
the existing Sylvester fallback.

Newton interpolation now updates integer forward differences and its
falling-factorial basis in place, uses an arbitrary-size factorial scale, and
removes coefficient content once before constructing output `Real` values.
Algebraic-image callers therefore avoid an immediate duplicate rational
normalization pass. The arbitrary-size scale also removes the former `i64`
factorial ceiling; a degree-22 regression covers that extended exact range.
Fixed and generated determinant, quotient/Sylvester, polynomial-image,
rational-image, and binary algebraic-image suites cover the complete dispatch.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 61,647,633 to 57,499,110 (6.73%), 82.07% below the original
320,660,631 baseline. Heaptrack allocation events fell from 98,024 to 87,084;
temporary events rose from 6,461 to 7,847 as direct integer products are
released promptly, peak heap remained 1.41 MiB, and peak RSS measured
12.51 MiB.

Eleven ordinary runs had a 5.736 ms complete median, a 3.794 ms preparation
median, and a 0.331 ms exact-polyline projection median. Every measured run
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete Hypersolve and Hypercurve all-feature and no-default-feature
suites, formatting, warning-denied all-target Clippy and rustdoc, and
supported default and no-default release WASM library builds passed. The
AddressSanitizer region-Boolean replay completed all 2,509 executions at 5,899
coverage points and 19,166 feature edges with no finding; LeakSanitizer alone
remained disabled under ptrace.

### Specialized rational endpoint derivative jets

General rational Bezier endpoint topology formerly evaluated homogeneous
numerator and denominator derivative jets through the full power-basis Horner
recurrence at parameters exactly zero and one. Start jets now select each
factorial-scaled power coefficient directly. End jets retain the same reverse
recurrence and exact addition order while omitting multiplications by one.
Interior derivative evaluation remains on the general Horner path.

A direct regression compares third-order start and end results against the
general evaluator for an unequal-weight cubic. The existing rational tangent
ordering, retained traversal, conic, and higher-derivative suites cover the
downstream use.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 57,499,110 to 56,559,931 (1.63%), 82.36% below the original
320,660,631 baseline. Heaptrack retained 87,084 allocation events and 7,847
temporary events, peak heap remained 1.41 MiB, and peak RSS fell from 12.51 to
12.43 MiB.

Eleven ordinary runs had a 5.732 ms complete median, a 3.910 ms preparation
median, and a 0.312 ms exact-polyline projection median. Every measured run
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
strict Clippy and rustdoc checks, and supported default/no-default WASM library
builds passed. AddressSanitizer region-Boolean replay completed all 2,509
executions at 5,892 coverage points and 19,098 feature edges with no finding;
LeakSanitizer alone remained disabled under ptrace.

### Area-only polynomial Bezier Green integrals

Polynomial quadratic and cubic signed-area queries formerly delegated to the
full area-and-first-moments evaluator, constructing and integrating the
unused `x^2 dy` and `y^2 dx` products. Area-only queries now evaluate the
quadratic and cubic Green integrals directly from grouped control-point cross
products. Prefix area queries reuse the same path after exact subdivision;
the complete moment API and its general power-basis evaluator remain
unchanged.

A direct regression compares both specialized formulas with the complete
moment evaluator. Existing exact area, moment, prefix, fitting, region-role,
and generated Boolean suites cover their public and downstream use.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 56,559,931 to 52,759,597 (6.72%), 83.55% below the original
320,660,631 baseline. Heaptrack allocation events fell from 87,084 to 82,283
and temporary events from 7,847 to 7,829; peak heap fell from 1.41 to 1.37
MiB and peak RSS from 12.43 to 12.21 MiB.

Across twenty-two ordinary runs, the complete median was 5.671 ms,
preparation was 3.768 ms, and exact-polyline projection was 0.368 ms. Every
measured run retained 9 candidate pairs, 48 fragments, 2 classifications, 4
decided operations, no blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
strict Clippy and rustdoc checks, and supported default/no-default WASM
library builds passed. AddressSanitizer region-Boolean replay completed all
2,509 executions at 5,897 coverage points and 19,144 feature edges with no
finding; LeakSanitizer alone remained disabled under ptrace.

### Source-root-first implicit-conic contact evidence

The implicit-conic intersection route maps each retained source root into the
quadratic conic's parameter space. It formerly constructed the exact contact
point through that newly mapped parameter first, even though the original
root and source curve already define the same certified incidence. Contact
replay now constructs and retains the source-parameter point image first and
keeps the mapped-conic construction as an exact fallback. This also warms the
image cache consumed by later source splitting.

An irrational parabola/line regression verifies that preparation retains the
source point image without constructing the mapped conic image. Existing
implicit-conic order, tangency, algebraic image, split, and Boolean suites
cover the contact and topology semantics.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 52,759,597 to 51,224,737 (2.91%), 84.03% below the original
320,660,631 baseline. Heaptrack allocation events fell from 82,283 to 80,043
and temporary events from 7,829 to 7,562; peak heap fell from 1.37 to 1.35
MiB and peak RSS from 12.21 to 12.17 MiB. Every measured run retained 9
candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
strict Clippy and rustdoc checks, and supported default/no-default WASM
library builds passed. AddressSanitizer region-Boolean replay completed all
2,509 executions at 5,897 coverage points and 19,156 feature edges with no
finding; LeakSanitizer alone remained disabled under ptrace.

### Closed-form rational-quadratic endpoint jets

Retained arrangement traversal formerly promoted every rational quadratic to
a general rational Bezier before asking the homogeneous power-basis quotient
recurrence for its endpoint derivatives. Rational quadratics now evaluate
their first three affine derivatives directly from the two endpoint-relative
weight ratios and control-point differences. End traversal uses the same
identity in the reversed parameter and restores the odd-derivative signs. The
zero-denominator and exact-sign decisions remain routed through the active
curve policy.

A direct regression compares first-, second-, and third-order derivatives at
both ends with the general rational evaluator for polynomial, unequal
positive-weight, and unequal negative-weight cases. Existing irrational-conic,
rational same-tangent, retained traversal, and region-Boolean suites cover the
downstream topology use.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 51,224,737 to 49,357,412 (3.65%), 84.61% below the original
320,660,631 baseline. Heaptrack allocation events fell from 80,043 to 76,840
and temporary events from 7,562 to 7,334; peak heap fell from 1.35 to 1.32
MiB and peak RSS from 12.17 to 12.07 MiB. Every measured run retained 9
candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, all-feature and no-default-feature rustdoc,
and supported default/no-default release WASM library builds passed.
AddressSanitizer region-Boolean replay completed all 2,516 executions at 5,897
coverage points and 19,105 feature edges with no finding; LeakSanitizer alone
remained disabled under ptrace.

### Single-loop roles and unique conic extrema

A retained region with one boundary loop has containment depth zero by
construction. When no authored role is attached, filled-side classification
now uses that loop's exact nonzero signed-area orientation directly instead of
constructing sample-point nesting evidence. Multi-loop regions retain the full
curved containment classifier, and unsupported or zero-area single loops keep
their existing explicit uncertainty.

Rational-quadratic bounds split one sorted parameter sequence into consecutive
monotone windows. The bounds evaluator formerly sampled each interior split
twice, once as a window end and again as the next window start. It now samples
only non-unit window ends, which visits every interior extremum exactly once.
A focused single-loop regression covers both orientations and verifies that
filled-side classification does not populate nesting bounds; the complete
rational evaluation/bounds and region suites cover the monotone-window change.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 49,357,412 to 48,952,778 (0.82%), 84.73% below the original
320,660,631 baseline. Heaptrack allocation events fell from 76,840 to 76,363
and temporary events from 7,334 to 7,332; peak heap remained 1.32 MiB and peak
RSS fell from 12.07 to 12.02 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, all-feature and no-default-feature rustdoc,
and supported default/no-default release WASM library builds passed.
AddressSanitizer region-Boolean replay completed all 2,509 executions at 5,899
coverage points and 19,138 feature edges with no finding; LeakSanitizer alone
remained disabled under ptrace.

### Representation-aware rational-quadratic point evaluation

Unit-end-weight rational quadratics use a conjugate expression to eliminate a
non-rational middle weight from the affine quotient. That expression formerly
ran for rational middle weights too, where it added a second denominator
product without eliminating anything. Conjugation is now limited to genuinely
non-rational weights. For an exact-rational parameter, the remaining path
evaluates the weighted coordinate and denominator quadratics directly in
power-basis Horner form; non-rational parameters retain the homogeneous
Bernstein evaluator.

A direct unequal-weight regression checks the exact rational midpoint produced
by the power quotient. Existing irrational circular-arc, rational evaluation,
subdivision, bounds, contact, and generated intersection suites cover both the
conjugate and Bernstein fallbacks.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 48,952,778 to 48,246,452 (1.44%), 84.95% below the original
320,660,631 baseline. Heaptrack allocation events fell from 76,363 to 75,087
and temporary events from 7,332 to 7,330; peak heap fell from 1.32 to 1.31 MiB
and peak RSS from 12.02 to 11.96 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, all-feature and no-default-feature rustdoc,
and supported default/no-default release WASM library builds passed.
AddressSanitizer region-Boolean replay completed all 2,515 executions at 5,893
coverage points and 19,125 feature edges with no finding; LeakSanitizer alone
remained disabled under ptrace.

### Direct rational-conic quotient derivatives

Rational-quadratic extrema formerly expanded each weighted coordinate
numerator and the homogeneous denominator into power form, multiplied out
`N'D - ND'`, and relied on the cubic terms cancelling. The quotient derivative
is now formed directly from its three quadratic Bernstein controls:
`w0*w1*(p1-p0)`, `w0*w2*(p2-p0)`, and `w1*w2*(p2-p1)`. The x and y solves
share their three weight products and denominator power basis, and finite-root
validation evaluates that denominator with Horner's rule. The general
quadratic root helper also classifies its leading coefficient once instead of
repeating the same exact-zero predicate.

An unequal-weight regression constructs a conic whose x quotient derivative
has the exact root `1/2`. Existing rational bounds, subdivision, conic
intersection, topology, and Boolean suites cover algebraic roots, projective
denominator boundaries, negative weights, and downstream extrema use.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 48,246,452 to 48,079,691 (0.35%), 85.01% below the original
320,660,631 baseline. Heaptrack allocation events fell from 75,087 to 74,759
while temporary events moved from 7,330 to 7,350; peak heap moved from 1.31 to
1.33 MiB and peak RSS from 11.96 to 12.11 MiB. Every measured run retained 9
candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, all-feature and no-default-feature rustdoc,
and supported default/no-default release WASM library builds passed.
AddressSanitizer region-Boolean replay completed all 2,509 executions at 5,898
coverage points and 19,154 feature edges with no finding; LeakSanitizer alone
remained disabled under ptrace.

### Retained polynomial endpoint-tangent evidence

Quadratic and cubic endpoint-tangent construction already computes a
structural zero/nonzero classification from the exact squared norm.
Arrangement endpoint extraction formerly cloned the two `Real` components,
discarded that classification, and immediately rebuilt the same squared norm
to reject zero tangents. Extraction now moves the owned components into the
arrangement vector and carries the structural classification through the
validation boundary. Only structurally unknown and rational-quotient tangents
invoke the policy-driven exact fallback. That fallback also retains its first
squared-norm expression instead of constructing it twice.

A focused regression checks that exact zero and nonzero endpoint tangents keep
their structural evidence through arrangement conversion. Existing
quadratic/cubic branch traversal, zero-tangent rejection, equal-tangent
higher-derivative ordering, rational tangent, and Boolean suites cover the
downstream behavior.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 48,079,691 to 47,834,643 (0.51%), 85.08% below the original
320,660,631 baseline. Heaptrack allocation events fell from 74,759 to 74,515
while temporary events remained 7,350; peak heap remained 1.33 MiB and peak
RSS fell from 12.11 to 11.92 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, all-feature and no-default-feature rustdoc,
and supported default/no-default release WASM library builds passed.
AddressSanitizer region-Boolean replay completed all 2,514 executions at 5,895
coverage points and 19,124 feature edges with no finding; LeakSanitizer alone
remained disabled under ptrace.

### Demand-driven arrangement higher-order jets

Retained traversal formerly constructed second- and third-order endpoint
derivatives for every materialized fragment before it knew whether a tangent
tie existed. Boolean topology already supplies certified successors for
crossing branches, so most of those exact quotient and polynomial jets were
never observed. Traversal now builds endpoint points and first tangents first,
constructs adjacency, and checks whether every multi-successor vertex has
usable certified evidence. Higher derivatives are omitted when that evidence
fully determines traversal; if any branch is uncovered, endpoint data is
rebuilt with the original exact higher-order path before comparison. The
ordinary public tangent-order traversal still requests complete jets
immediately.

The deferral applies equally to native polynomial, rational-quadratic, general
rational, and retained algebraic endpoint images. A focused partial-evidence
regression forces the fallback at an equal-tangent quadratic branch and
verifies that it exactly matches ordinary retained tangent traversal. Existing
second-/third-order, rational/algebraic, overlap, and Boolean suites cover the
certified and fallback paths.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 47,834,643 to 45,626,071 (4.62%), 85.77% below the original
320,660,631 baseline. Heaptrack allocation events fell from 74,515 to 71,365
while temporary events remained 7,350; peak heap fell from 1.33 to 1.30 MiB
and peak RSS from 11.92 to 11.69 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, all-feature and no-default-feature rustdoc,
and supported default/no-default release WASM library builds passed.
AddressSanitizer region-Boolean replay completed all 2,509 executions at 5,894
coverage points and 19,160 feature edges with no finding; LeakSanitizer alone
remained disabled under ptrace.

### Direct rational-conic Green numerators

Rational-quadratic signed area formerly expanded the weighted x and y
coordinates into power form, differentiated both, multiplied the two
coordinate/derivative pairs, and subtracted them before their cubic terms
cancelled. The homogeneous Green numerator is now formed directly from its
three quadratic Bernstein controls:
`w0*w1*(P0 cross P1)`, `w0*w2*(P0 cross P2)`, and
`w1*w2*(P1 cross P2)`. Its common factor two cancels the Green integral's
one-half factor before integration. The denominator, affine-boundary
certification, and exact inverse-quadratic integration branches are unchanged.
The now-unused generic polynomial-difference layer was removed.

A focused unequal-weight regression verifies invariance under a uniformly
negative projective weight scale. Existing exact polynomial-equivalence,
irrational quarter-circle, split/reversal, same-sign conic-region, and
projective-boundary tests cover polynomial, elliptic, subdivision, orientation,
and unsupported-denominator behavior.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 45,626,071 to 45,347,329 (0.61%), 85.86% below the original
320,660,631 baseline. Heaptrack allocation events fell from 71,365 to 71,190
and temporary events from 7,350 to 7,342; peak heap fell from 1.30 to 1.28 MiB
while peak RSS moved from 11.69 to 11.78 MiB. Every measured run retained 9
candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, all-feature and no-default-feature rustdoc,
and supported default/no-default release WASM library builds passed.
AddressSanitizer region-Boolean replay completed all 2,509 executions at 5,898
coverage points and 19,135 feature edges with no finding; LeakSanitizer alone
remained disabled under ptrace.

### Target-bounded algebraic rational images

Mapping an algebraic curve-intersection parameter back to a rational-conic
parameter formerly constructed a full exact resultant before checking whether
the mapped root lay in the Bezier parameter domain. The rational-expression
domain check had already produced a conservative exact value enclosure, but
that evidence was only used to certify the denominator. Hypersolve now offers
a target-bounded rational-image transform that returns retained disjointness
evidence before elimination when the enclosure is wholly outside a requested
closed interval. Hypercurve requests `[0, 1]` for conic parameters and keeps
the existing exact construction for overlapping, boundary-touching, or
inconclusive enclosures. Nine of the sentinel's sixteen conic parameter maps
now avoid their unused resultants.

Focused hypersolve regressions cover certified disjointness and ensure that a
boundary-touching enclosure still constructs the represented image. Existing
implicit-conic contact, parameterization-invariance, curved-intersection, and
Boolean suites cover the consuming path and preserve its exact topology.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 45,347,329 to 42,877,040 (5.45%), 86.63% below the original
320,660,631 baseline. Heaptrack allocation events fell from 71,190 to 65,679
and temporary events from 7,342 to 7,008; peak heap remained 1.28 MiB and peak
RSS remained 11.78 MiB. Every measured run retained 9 candidate pairs, 48
fragments, 2 classifications, 4 decided operations, no blockers, and checksum
6.

The complete all-feature and no-default-feature test suites passed in both
hypersolve and hypercurve, together with formatting, warning-denied all-target
Clippy, warning-denied all-feature and no-default-feature rustdoc, and
supported default/no-default release WASM library builds. AddressSanitizer
region-Boolean replay completed all 2,509 executions at 5,897 coverage points
and 19,176 feature edges with no finding; LeakSanitizer alone remained disabled
under ptrace.

### Direct quotient-ring determinant polynomials

Algebraic polynomial and rational images formerly evaluated their quotient-ring
matrix pencil at `degree + 1` integer image values, ran a separate
fraction-free Bareiss determinant for every sample, and interpolated the
resultant polynomial afterward. Hypersolve now expands the determinant of the
linear polynomial matrix directly with an exact subset dynamic program. Each
partial assigns one source-basis row to a distinct column, retains its
permutation sign, and accumulates integer coefficients in ascending image
power. Structurally zero constant or linear matrix entries are skipped. The
result remains the same elimination polynomial up to the shared nonzero scale
introduced by the quotient-ring basis; the existing Sylvester path remains the
fallback when integer matrix construction is unavailable.

A fixed nonmonic cubic-source regression and generated monic cubic cases
evaluate the new coefficient form at every old sample point and compare it
both to the retained flat-Bareiss reference and to independently constructed
Sylvester resultants up to one scale. Existing polynomial-image,
rational-image, algebraic-parameter, curve-intersection, and Boolean suites
cover the two consuming construction paths.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 42,877,040 to 40,252,053 (6.12%), 87.45% below the original
320,660,631 baseline. Heaptrack allocation events fell from 65,679 to 61,157
and temporary events from 7,008 to 5,376; peak heap remained 1.28 MiB and peak
RSS fell from 11.78 to 11.76 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete all-feature and no-default-feature test suites passed in both
hypersolve and hypercurve, together with formatting, warning-denied all-target
Clippy, warning-denied all-feature and no-default-feature rustdoc, and
supported default/no-default release WASM library builds. AddressSanitizer
region-Boolean replay completed all 2,509 executions at 5,898 coverage points
and 19,139 feature edges with no finding; LeakSanitizer alone remained disabled
under ptrace.

### Lazy polynomial algebraic split endpoints

Refined Boolean splitting already retained rational endpoint images as lazy
first-order source evidence, but polynomial quadratic and cubic boundaries
still constructed algebraic point, tangent, second-derivative, and
third-derivative images immediately. Certified topology usually supplies both
the endpoint vertex and branch successor, leaving every one of those
polynomial images unused. The lazy endpoint carrier now supports all four
Bezier source families. When a topology vertex is present, traversal retains
only the source curve and algebraic parameter. Point, tangent, and
higher-order polynomial images are constructed from that source only if
coordinate fallback or an uncovered tangent branch observes them. Public split
construction and validation retain their eager exact-evidence behavior.

A focused irrational-parameter cubic regression compares lazy point and
tangent observation with the eager endpoint image, then constructs first-,
second-, and third-order derivatives through the retained source and verifies
that every represented vector matches its eager counterpart. Existing partial
certified-successor fallback, algebraic tangent-order, split validation, and
Boolean suites cover demand-driven traversal and source validation.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 40,252,053 to 37,185,392 (7.62%), 88.40% below the original
320,660,631 baseline. Heaptrack allocation events fell from 61,157 to 55,571
and temporary events from 5,376 to 4,764; peak heap fell from 1.28 to 1.15 MiB
and peak RSS from 11.76 to 11.44 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. AddressSanitizer region-Boolean replay completed all
2,509 executions at 5,895 coverage points and 19,155 feature edges with no
finding; LeakSanitizer alone remained disabled under ptrace.

### Loop-local rational-conic area kernels

Exact loop orientation formerly reconstructed the same weight-only
inverse-quadratic integral for every rational-quadratic fragment. Circular-arc
decomposition in particular produces several conics with identical weight
denominators but different geometric cross-product numerators, so each segment
repeated the same exact square root and arctangent construction. Native and
retained Bezier loop accumulation now keep a small local cache keyed by the
exact Bernstein-to-power denominator polynomial. A successfully constructed
inverse-quadratic definite integral is reused only for an exactly equal
denominator; the remaining numerator-dependent rational terms are still
evaluated independently. Standalone segment queries retain the uncached path,
and unsupported or degenerate integral branches are not cached.

A focused regression evaluates two geometrically different conics with equal
weights through both paths, verifies exact equality with their independently
constructed uncached results, and confirms that only one denominator integral
is retained. Existing polynomial/conic area, negative projective scale,
quarter-circle sector, split/reversal, region-orientation, and Boolean suites
cover the consuming loop paths.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 37,185,392 to 36,366,179 (2.20%), 88.66% below the original
320,660,631 baseline. Heaptrack allocation events fell from 55,571 to 53,735
and temporary events from 4,764 to 4,159; peak heap remained 1.15 MiB and peak
RSS moved from 11.44 to 11.45 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. AddressSanitizer region-Boolean replay completed all
2,509 executions at 5,892 coverage points and 19,133 feature edges with no
finding; LeakSanitizer remained disabled under ptrace.

### Cross-operand conic area evidence

Retained Boolean preparation asks both input regions for their exact filled
side before constructing carrier pairs. Similarity-transformed or independently
authored operands commonly preserve rational-conic weights, but each region
formerly owned a separate loop-local area cache. Boolean carrier preparation
now passes one short-lived exact area-integral cache through both orientation
queries. The regions continue to retain only their decided filled-side result;
the shared cache does not become persistent region state and standalone region
queries retain their existing local lifetime.

A focused regression constructs two independent rational-quadratic regions
with equal denominator weights, decides both orientations through one cache,
and verifies that the second region does not add another inverse-quadratic
integral. Existing clone-sharing, transformed-region, filled-side, conic-area,
and Boolean suites cover cached and already-decided operands.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 36,366,179 to 35,910,812 (1.25%), 88.80% below the original
320,660,631 baseline. Together, the two area-kernel checkpoints reduce the
preceding 37,185,392 median by 3.43%. Heaptrack allocation events fell from
53,735 to 52,711 and temporary events from 4,159 to 3,825; peak heap remained
1.15 MiB and peak RSS fell from 11.45 to 11.35 MiB. Every measured run retained
9 candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. AddressSanitizer region-Boolean replay completed all
2,509 executions at 5,895 coverage points and 19,144 feature edges with no
finding; LeakSanitizer remained disabled under ptrace.

### Single-conversion rational Sturm chains

Exact Bezier parameter isolation uses scale-invariant Sturm sequences. For
rational defining polynomials, every pseudo-remainder step formerly converted
both of its already-rational operands back to primitive integer coefficient
vectors. The rational path now clears denominators once, derives and
content-reduces the integer derivative, and builds the complete signed
pseudo-remainder chain in the integer domain. Only the completed chain is
converted back to `Real` coefficients for the existing retained certificate
and point-variation APIs. Nonrational polynomials continue through the
unchanged field-arithmetic fallback.

A focused equivalence regression compares variation and exact-root evidence
from the integer chain with an ordinary field-division Sturm construction for
rational quadratic through quartic polynomials, including sparse, mixed-sign,
fractional, and repeated-root inputs. Existing rational reconstruction,
multiplicity, refinement, algebraic ordering, and five-root isolation suites
cover the downstream certificate consumers.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 35,910,812 to 35,779,537 (0.37%), 88.84% below the original
320,660,631 baseline. Heaptrack allocation events fell from 52,711 to 52,252
and temporary events from 3,825 to 3,759; peak heap remained 1.15 MiB and peak
RSS fell from 11.35 to 11.32 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. AddressSanitizer region-Boolean replay completed all
2,509 executions at 5,899 coverage points and 19,192 feature edges with no
finding; LeakSanitizer remained disabled under ptrace.

### Conservative native-region query hulls

Repeated point classification retains one exact outer box for each native
Bezier boundary loop. These boxes are used only to reject points that are
provably outside a loop; no consumer requires a tight extrema envelope.
Polynomial fragments therefore now contribute their exact control hulls
instead of isolating derivative roots and evaluating interior extrema.
Rational-quadratic fragments take the same route only after certifying that
all homogeneous weights have one nonzero sign, which supplies the rational
Bezier convex-hull guarantee. Mixed-sign conics keep the existing extrema
fallback, and general rational fragments retain their existing certified
same-sign control-hull classifier.

A focused regression uses a cubic whose control hull is strictly wider than
its tight extrema box, verifies that the query bound equals that hull, and
checks exact containment of points across the parameter domain. Existing
native/retained point classification, boundary, nesting, Boolean, rational
projective-denominator, and unified-region bounds suites cover the consuming
queries and the fallback boundary.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 35,779,537 to 33,781,716 (5.58%), 89.47% below the original
320,660,631 baseline. Heaptrack allocation events fell from 52,252 to 49,025
and temporary events from 3,759 to 3,540; peak heap fell from 1.15 to 1.14 MiB
and peak RSS from 11.32 to 11.18 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. The requested 2,509-case AddressSanitizer region-Boolean
replay completed at 5,897 coverage points and 19,147 feature edges with no
finding; LeakSanitizer remained disabled under ptrace.

### Shared quotient-ring source conversion

Algebraic rational images construct numerator and denominator multiplication
matrices in the same quotient ring. Each matrix formerly converted the common
source polynomial from exact rational `Real` coefficients to integer
coefficients independently. The paired construction now performs that source
conversion once, converts each relation separately, and reuses the integer
source for both existing pseudo-reduction passes.

The pseudo-reduction scaling itself remains unchanged. In particular, even a
constant relation must retain the zero high-degree reduction steps because
they apply the source-leading scale shared by the numerator and denominator
matrices when the source polynomial is nonmonic. The algebraic tangent-order
regression exercises this invariant, while the algebraic rational-image and
resultant suites cover the paired construction and downstream root evidence.

On the one-cell all-family exact Boolean sentinel, the ten-run instruction
median fell from 33,781,716 to 33,700,629 (0.24%), 89.49% below the original
320,660,631 baseline. Heaptrack allocation events fell from 49,025 to 48,920;
temporary events remained 3,540, peak heap remained 1.14 MiB, and peak RSS
fell from 11.18 to 11.10 MiB. Every measured run retained 9 candidate pairs,
48 fragments, 2 classifications, 4 decided operations, no blockers, and
checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. The requested `-runs=2509` AddressSanitizer
region-Boolean replay completed with libFuzzer reporting 2,512 executions at
5,896 coverage points and 19,164 feature edges with no finding; LeakSanitizer
remained disabled under ptrace.

### Scale-preserving constant quotient operators

The quotient-ring resultant pads numerator and denominator relations to one
shared degree so that their multiplication matrices carry the same
fraction-free source-leading scale. When one relation is constant, its padded
pseudo-reduction is exactly a diagonal operator: the relation constant times
the source-leading coefficient raised to the shared relation degree. The
constant path now constructs that diagonal directly instead of allocating a
product workspace and visiting zero high-degree coefficients for every
column.

A focused nonmonic regression verifies the retained source-leading power in
the diagonal. This guards the scale that must be shared with the nonconstant
matrix; omitting it changes the represented resultant and breaks downstream
algebraic tangent ordering. The quotient/Sylvester comparison, rational-image
suite, and that tangent-order regression cover the direct operator and its
consumers.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 33,700,629 to 33,658,227 (0.13%), 89.50% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
48,920 to 48,884 and temporary events from 3,540 to 3,238; peak heap remained
1.14 MiB, while peak RSS moved from 11.10 to 11.20 MiB. Every measured run
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. AddressSanitizer region-Boolean replay completed all
2,509 executions at 5,898 coverage points and 19,166 feature edges with no
finding; LeakSanitizer remained disabled under ptrace.

### Pair-scoped implicit-conic parameter maps

Implicit conic/general-rational intersection first isolates every parameter
root on the general curve and then transports each root into the conic's
parameterization. The two rational transport maps depend only on the curve
pair, but were formerly rebuilt for every isolated root. Their homogeneous
conic frame, curve power basis, dual coordinates, and source-range
localization are now prepared once per pair and borrowed by every root
transformation and refinement retry.

The retained transport contains only exact `Real` polynomial coefficients.
Root-specific isolating-interval refinement, denominator certification,
resultant construction, and represented-root validation remain unchanged.
The sentinel profile now constructs 9 pair maps for 13 contact roots and
reuses their localized coefficients across 16 exact transformation attempts.
Focused implicit-conic source-image, degree-elevated line, tangent,
transversality, and algebraic tangent-order regressions cover the prepared
maps and both parameter formulas.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 33,658,227 to 33,192,889 (1.38%), 89.65% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
48,884 to 48,406 and temporary events from 3,238 to 3,226; peak heap remained
1.14 MiB and peak RSS moved from 11.20 to 11.19 MiB. Every measured run
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. AddressSanitizer region-Boolean replay completed all
2,509 executions at 5,900 coverage points and 19,131 feature edges with no
finding; LeakSanitizer remained disabled under ptrace.

### Pole-free conic parameter transport

The retained conic transport formerly used two dual-coordinate ratios. One
ratio has a pole at the conic's final parameter and the other at its initial
parameter, so endpoint and wide-interval contacts can force a second exact
algebraic image construction. For dual coordinates
`lambda_0, lambda_1, lambda_2`, the same parameter is
`(lambda_1 + 2 lambda_2) / (2 (lambda_0 + lambda_1 + lambda_2))`. At a conic
point the denominator is the certified nonzero frame determinant times the
nonzero projective scale, so this primary map has no conic-parameter endpoint
pole.

The pair-scoped transport now prepares only that localized primary ratio.
It retains the three coordinate polynomials and constructs the two former
endpoint ratios lazily only if all existing primary-map refinement attempts
remain uncertain. Thus adversarial decisiveness is preserved without paying
for fallback maps on ordinary contacts. The sentinel's 13 roots now need 14
image attempts instead of 16. A focused unequal-weight rational-quadratic
regression proves the dual-coordinate identity at both endpoints and an
interior parameter; the implicit-conic, tangent, transversality, and algebraic
ordering suites cover the lazy fallback boundary and downstream evidence.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 33,192,889 to 32,852,557 (1.03%), 89.75% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
48,406 to 47,321 and temporary events from 3,226 to 2,988; peak heap remained
1.14 MiB and peak RSS fell from 11.19 to 11.13 MiB. Every measured run retained
9 candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. The requested `-runs=2509` AddressSanitizer
region-Boolean replay completed with libFuzzer reporting 2,515 executions at
5,896 coverage points and 19,148 feature edges with no finding; LeakSanitizer
remained disabled under ptrace.

### Retained spline span-boundary classification

Polynomial-spline and NURBS evaluation first scan retained Bezier span
intervals to select every span containing the authored parameter. That scan
already proves whether the parameter equals the selected span's start or end,
but the result formerly retained only span indices. Point and derivative
evaluation then repeated exact subtraction and division to reconstruct local
parameter zero or one and evaluated the full rational or polynomial Bezier
span.

Both selectors now retain a typed start/interior/end location beside each
selected index. Point queries at certified boundaries clone the exact retained
span endpoint, while derivative queries reuse exact local zero or one before
applying the existing authored-knot chain scale. Interior parameters follow
the unchanged normalization and evaluator paths. At discontinuous knots the
first and last selected spans retain their own end and start locations, so
automatic, left, and right side behavior is unchanged. The polynomial-spline
and NURBS endpoint, interior-knot, discontinuity, periodic-seam, derivative,
higher-derivative, reversal, and editing suites cover these paths.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 32,852,557 to 32,813,991 (0.12%), 89.77% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
47,321 to 47,301 and temporary events from 2,988 to 2,982; peak heap remained
1.14 MiB and peak RSS fell from 11.13 to 11.12 MiB. Every measured run retained
9 candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. The requested `-runs=2509` AddressSanitizer
region-Boolean replay completed with libFuzzer reporting 2,511 executions at
5,898 coverage points and 19,152 feature edges with no finding; LeakSanitizer
remained disabled under ptrace.

### Retained native endpoint evaluation

Top-level native-curve evaluation formerly validated a parameter against both
ends of the unit interval and then discarded whether either comparison proved
equality. Exact endpoint queries consequently evaluated a full polynomial
Bezier, built and projected a rational power basis, or promoted and searched a
circular-arc decomposition even though every geometry already retains its
authored endpoints.

Closed-unit-interval classification now returns a typed
start/interior/end/outside location while preserving the former Boolean helper
for its other consumers. `Curve2` reuses that classification to clone retained
native endpoints directly. Rational quadratic and general rational curves take
the shortcut only after the selected endpoint weight is certified nonzero;
unknown or zero projective weights retain the former evaluator and typed
boundary behavior. Interior queries and authored spline-domain evaluation are
unchanged. A focused all-family regression covers both endpoints and verifies
that rational endpoint queries do not construct the homogeneous power basis.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 32,813,991 to 32,588,978 (0.69%), 89.84% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
47,301 to 47,057 and temporary events from 2,982 to 2,960; peak heap fell from
1.14 to 1.13 MiB and peak RSS remained 11.12 MiB. Every measured run retained
9 candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete all-feature and no-default-feature test suites, formatting,
warning-denied all-target Clippy, warning-denied all-feature and
no-default-feature rustdoc, and supported default/no-default release WASM
library builds passed. The requested `-runs=2509` AddressSanitizer
region-Boolean replay completed at 5,898 coverage points and 19,147 feature
edges with no finding; LeakSanitizer remained disabled under ptrace.

### Shared projective coordinate transforms

Exact algebraic rational-point and derivative images transform two coordinate
numerators at one represented parameter over the same homogeneous denominator.
The coordinate wrapper formerly invoked the complete rational-image package
independently for x and y, repeating denominator evaluation and source
polynomial conversion before each coordinate-specific resultant.

Point and tangent construction now prepare one shared-denominator Hypersolve
transform and consume it sequentially. This retains the original x-first
failure boundary: y is still skipped when x cannot be represented. Successful
coordinates preserve their complete independent evaluation, resultant,
isolating-interval, validation, and fallback reports; only the denominator
certificate and lazily converted source polynomial are shared. Rational
quadratic and general rational point, tangent, higher-derivative, intersection,
algebraic split, and typed projective-boundary suites cover both the successful
and blocked paths.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 32,588,978 to 32,508,278 (0.25%), 89.86% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
47,057 to 46,869 and temporary events from 2,960 to 2,930; peak heap remained
1.13 MiB and peak RSS moved from 11.12 to 11.17 MiB. Every measured run
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete Hypersolve and Hypercurve all-feature and no-default-feature
suites, formatting, warning-denied all-target Clippy and rustdoc, and supported
default/no-default release WASM library builds passed. The requested
AddressSanitizer region-Boolean replay completed with libFuzzer reporting 2,515
executions at 5,893 coverage points and 19,142 feature edges with no finding;
LeakSanitizer remained disabled under ptrace.

### Shared conic-map elimination across contact roots

One implicit conic substitution produces a single exact polynomial whose
isolated roots are all transported through the same conic-parameter map. The
map's direct rational-image resultant depends on that source polynomial and
the map coefficients, not on the individual isolating interval. The former
contact loop nevertheless normalized the same map and rebuilt its identical
resultant for each represented root and refinement attempt.

The primary conic-parameter candidate is now normalized and prepared once per
substitution polynomial. Its exact primitive source and direct resultant are
retained lazily across every contact root and adaptive refinement. Root-local
denominator, monotonicity, image-interval, selection, and validation evidence
is still reconstructed independently. The two endpoint-oriented fallback maps
remain lazy: they are normalized and prepared only if the pole-free primary
map exhausts its refinement budget.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 32,508,278 to 32,120,772 (1.19%), 89.98% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
46,869 to 46,054; recorder-level temporary allocation events fell from 2,930
to 2,714, while the postprocessor's broader temporary count was 2,962. Peak
heap remained 1.13 MiB and peak RSS measured 11.25 MiB. Every measured run
retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete Hypersolve and Hypercurve all-feature and no-default-feature
suites, formatting, warning-denied all-target Clippy and rustdoc, and supported
default/no-default release WASM library builds passed. The requested
AddressSanitizer region-Boolean replay completed with libFuzzer reporting 2,510
executions at 5,900 coverage points and 19,164 feature edges with no finding;
LeakSanitizer remained disabled under ptrace.

### Retained conic-map algebra

The prepared Hypersolve map now retains the conic transport's normalized
numerator and denominator, constant-map classification, quotient-rule
derivative polynomial, and common integer coefficient scaling as separately
lazy stages. These values depend only on the pair-scoped transport, not on a
particular implicit-conic contact root. Root-local interval evaluation,
monotonicity certification, image bounds, selection, and exact validation are
unchanged.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 32,120,772 to 32,005,223 (0.36%), 90.02% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
46,054 to 45,821; recorder-level temporary events fell from 2,714 to 2,702 and
the postprocessor count fell from 2,962 to 2,950. Peak heap remained 1.13 MiB
and peak RSS fell from 11.25 to 11.15 MiB. Every measured run retained 9
candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete Hypersolve and Hypercurve all-feature and no-default-feature
suites, formatting, warning-denied all-target Clippy and rustdoc, and supported
default/no-default release WASM library builds passed. The AddressSanitizer
region-Boolean replay completed all 2,509 requested executions at 5,900
coverage points and 19,225 feature edges with no finding; LeakSanitizer
remained disabled under ptrace.

### Allocation-free small-prime root sieve

The exact rational-root rejection sieve reduces primitive `BigInt`
coefficients modulo eleven fixed primes no larger than 31. It formerly created
a `BigInt` modulus, one `BigInt` remainder per coefficient, and a fresh residue
vector for each tested prime. Root isolation invokes this proof once for every
implicit substitution polynomial even when the first useful prime rejects all
rational roots.

Small-prime residues now fold the existing magnitude limbs directly in native
`u64` arithmetic and apply the signed floor-remainder adjustment explicitly.
One residue buffer is reused across primes. This changes only how the exact
finite-field coefficients are represented; candidate evaluation and the
one-sided rejection rule are unchanged. A signed zero, boundary integer, and
positive/negative 200-bit regression compares every native residue against
`BigInt::mod_floor`, while the existing generated rational-root suite proves
that roots with denominators through eight remain admitted.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 32,005,223 to 31,937,120 (0.21%), 90.04% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
45,821 to 45,504; recorder-level temporary events fell from 2,702 to 2,663 and
the postprocessor count fell from 2,950 to 2,911. Peak heap remained 1.13 MiB
and peak RSS measured 11.16 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy and rustdoc, and supported default/no-default
release WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 requested executions at 5,896 coverage points and 19,168
feature edges with no finding; LeakSanitizer remained disabled under ptrace.

### Native integer Sturm sign evaluation

Sturm point evidence needs only the sign of each sequence member at a rational
boundary. The primitive Sturm chain already has exact integer coefficients, but
the generic evaluator formerly constructed and normalized a `Rational` value
at every Horner step before discarding everything except its sign.

The sign scan now evaluates the homogeneous integer numerator directly. A
checked `i128` path covers the common small-coefficient chain without allocating;
overflow or wider exact integers fall back to the same recurrence in `BigInt`.
Noninteger coefficients and parameters outside the exact rational tower retain
the generic `Real` evaluation and predicate path. Root isolation also transfers
its coefficient vector into the parameter polynomial and recovers it only when
a represented rational root requires another division pass, removing the
unconditional clone. Ordinary rational samples, zero, and signed 200-bit
coefficients are checked against exact rational evaluation, while a noninteger
coefficient verifies the generic fallback remains selected.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 31,937,120 to 31,366,779 (1.79%), 90.22% below
the original 320,660,631 baseline. Inclusive instruction cost beneath
`sturm_point_evidence` fell from 769,824 to 193,338 (74.9%). Heaptrack
allocation events fell from 45,504 to 43,984; recorder-level temporary events
rose from 2,663 to 2,685 and the postprocessor count rose from 2,911 to 2,933.
Peak heap remained 1.13 MiB and peak RSS measured 11.18 MiB. Every measured
run retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy and rustdoc, and supported default/no-default
release WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 requested executions at 5,897 coverage points and 19,144
feature edges with no finding; LeakSanitizer remained disabled under ptrace.

### Fixed exact cubic Bernstein sums

Cubic Bezier evaluation at an exact rational parameter constructs four
Bernstein weights and formerly evaluated each coordinate as four independent
products joined by three pairwise additions. For exact rational control
coordinates, that expression repeatedly normalized intermediate rationals even
though the complete fixed product sum is already known to stay in the exact
rational tower.

The rational-parameter branch now sends each four-term coordinate through
Hyperreal's fixed exact rational product-sum reducer. The fast path is selected
only after all eight control coordinates prove exact rational. A symbolic
control coordinate preserves the former Bernstein expression and operation
order verbatim, while parameters outside the exact rational tower retain the
former de Casteljau evaluation. A focused regression checks the fused result
against exact de Casteljau evaluation across endpoints, interior and exterior
rationals, and an algebraic parameter; it separately verifies that symbolic
controls retain the established Bernstein representation at rational
parameters and the established de Casteljau representation otherwise.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from the current dependency-qualified 31,154,077
baseline to 30,417,147 (2.37%), 90.51% below the original 320,660,631
baseline. Inclusive instruction cost beneath `CubicBezier2::point_at` fell
from 1,175,419 to 590,316 (49.8%). Heaptrack allocation events fell from
43,975 to 42,571; recorder-level temporary events remained 2,685 and the
postprocessor count remained 2,933. Peak heap moved from 1.13 to 1.14 MiB,
peak RSS measured 11.09 MiB, and retained memory moved from 96.57 to 100.03
KiB. Every measured run retained 9 candidate pairs, 48 fragments, 2
classifications, 4 decided operations, no blockers, and checksum 6.

The complete all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy and rustdoc, and supported default/no-default
release WASM library builds passed. The requested `-runs=2509`
AddressSanitizer region-Boolean replay completed with libFuzzer reporting 2,513
executions at 5,897 coverage points and 19,125 feature edges with no finding;
LeakSanitizer remained disabled under ptrace.

### Fixed exact squared distances

Squared point distance first constructs the two coordinate differences. It
formerly squared each difference independently and added the two normalized
products, creating three more rational normalization boundaries for the common
exact-coordinate case.

When both constructed differences prove exact rational, `Point2` now sends the
two squares through Hyperreal's fixed exact rational product-sum reducer.
Symbolic differences retain the former two-products-and-add expression
verbatim. A focused rational regression compares the fused result against the
former exact expression, while a radical-coordinate case verifies that the
symbolic expression and representation stay unchanged.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 30,417,147 to 29,809,524 (2.00%), 90.70% below
the original 320,660,631 baseline. Inclusive instruction cost beneath
`Point2::distance_squared` fell from 1,710,171 to 1,231,286 (28.0%).
Heaptrack allocation events fell from 42,571 to 41,714; recorder-level
temporary events remained 2,685 and the postprocessor count remained 2,933.
Peak heap fell from 1.14 to 1.04 MiB, peak RSS remained 11.09 MiB, and retained
memory moved from 100.03 to 103.18 KiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers, and
checksum 6.

The complete all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy and rustdoc, and supported default/no-default
release WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 requested executions at 5,935 coverage points and 19,233
feature edges with no finding; LeakSanitizer remained disabled under ptrace.

### Shared exact point determinants

Line-contour area accumulation and polynomial Bezier moment formulas both
evaluate the same point determinant `x0*y1 - y0*x1`. Each private helper
formerly constructed and normalized the two products separately before
subtracting them.

Both consumers now route through Hyperreal's exact-aware
difference-of-products reducer; the Bezier formula shares an internal
`Point2` cross-product kernel, while the contour formula retains its historical
negative-term operand order explicitly. Exact rational coordinates are reduced
as one signed product sum; symbolic coordinates retain each caller's former
two-products-and-subtraction expression. Focused exact and radical-coordinate
regressions verify both operand-order contracts, while the contour and Bezier
area suites exercise the downstream consumers.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 29,809,524 to 28,805,252 (3.37%), 91.02% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
41,714 to 39,952; recorder-level temporary events moved from 2,685 to 2,686
and the postprocessor count from 2,933 to 2,934. Peak heap fell from 1.04 to
1.03 MiB, peak RSS from 11.09 to 10.98 MiB, and retained memory from 103.18 to
101.49 KiB. Every measured run retained 9 candidate pairs, 48 fragments, 2
classifications, 4 decided operations, no blockers, and checksum 6.

The complete all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy and rustdoc, and supported default/no-default
release WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 requested executions at 5,936 coverage points and 19,068
feature edges with no finding; LeakSanitizer remained disabled under ptrace.

### Fixed exact similarity point transforms

Similarity point transformation evaluates two affine coordinates, each as two
products plus a translation. The former implementation normalized both
products and their first sum before adding the translation.

When the point and the three coefficients for a coordinate are all exact
rational, `Similarity2::transform_point` now reduces the complete affine
coordinate as one fixed signed product sum, representing the translation as
`offset * 1`. If any participating value is symbolic, it retains the former
two-products-and-two-additions expression verbatim. A focused exact regression
compares both fused coordinates against the former formula, while radical
point coordinates verify that the symbolic expression stays unchanged.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 28,805,252 to 28,431,564 (1.30%), 91.13% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
39,952 to 39,349; recorder-level temporary events moved from 2,686 to 2,687
and the postprocessor count from 2,934 to 2,935. Peak heap fell from 1.03 to
1.02 MiB, peak RSS moved from 10.98 to 11.00 MiB, and retained memory fell
from 101.49 to 96.06 KiB. Every measured run retained 9 candidate pairs, 48
fragments, 2 classifications, 4 decided operations, no blockers, and checksum
6.

The complete all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy and rustdoc, and supported default/no-default
release WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 requested executions at 5,927 coverage points and 19,037
feature edges with no finding; LeakSanitizer remained disabled under ptrace.

### Fixed exact rational-quadratic power evaluation

Rational quadratic evaluation at a rational parameter uses a quadratic power
basis for the denominator and both coordinate numerators. Its former Horner
schedule normalized two products and two additions independently for each of
those three polynomials.

When the parameter and all three power coefficients are exact rational, the
evaluator now reduces the constant, linear, and quadratic terms as one fixed
signed product sum. A symbolic coefficient retains the former Horner
expression verbatim. Focused regressions compare the fused exact result and
the guarded radical-coefficient expression against explicit Horner replay.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 28,431,564 to 28,001,118 (1.51%), 91.27% below
the original 320,660,631 baseline. Inclusive cost beneath
`RationalQuadraticBezier2::point_at` fell from 1,328,719 to 960,107
instructions (27.7%). Heaptrack allocation events fell from 39,349 to 38,467;
recorder-level temporary events remained 2,687 and the postprocessor count
remained 2,935. Peak heap fell from 1.02 MiB to 987.31 KiB, peak RSS remained
11.00 MiB, and retained memory fell from 96.06 to 89.49 KiB. Every measured
run retained 9 candidate pairs, 48 fragments, 2 classifications, 4 decided
operations, no blockers, and checksum 6.

The complete all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy and rustdoc, and supported default/no-default
release WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 requested executions at 5,932 coverage points and 19,036
feature edges with no finding; LeakSanitizer remained disabled under ptrace.

### Batched exact line-contour area

The all-line contour area path formerly reduced every edge determinant
separately and normalized the running rational sum after every edge. Large
exact contours therefore crossed two rational normalization boundaries per
edge even though all operands were known before accumulation began.

When every line coordinate is exact rational, contour area now reduces eight
edge determinants at a time as one fixed 16-term signed product sum. A short
remainder uses the existing determinant helper, and any symbolic coordinate
retains the former per-edge fold verbatim. The 1,024-edge exact polygon
regression exercises batching across many blocks; a focused eight-edge radical
case compares the guarded path against explicit replay of the former
accumulation.

On the one-cell all-family exact Boolean sentinel, the rounded ten-run
instruction median fell from 28,001,118 to 27,757,309 (0.87%), 91.34% below
the original 320,660,631 baseline. Inclusive cost beneath
`compute_contour_signed_area` fell from 512,425 to 261,884 instructions
(48.9%). Heaptrack allocation events fell from 38,467 to 37,651;
recorder-level temporary events remained 2,687 and the postprocessor count
remained 2,935. Peak heap remained 987.31 KiB, peak RSS fell from 11.00 to
10.96 MiB, and retained memory remained 89.49 KiB. Every measured run retained
9 candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete all-feature and no-default-feature suites, formatting,
warning-denied all-target Clippy and rustdoc, and supported default/no-default
release WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 requested executions at 5,959 coverage points and 19,056
feature edges with no finding; LeakSanitizer remained disabled under ptrace.

### Reused Sturm multiplicity certificates

Implicit conic intersection isolates every root of the substituted polynomial
with a retained Sturm chain, then classifies root multiplicity to certify
transverse contacts. Multiplicity classification formerly rebuilt
`gcd(P, P')` even though the final nonzero polynomial in that retained
polynomial-remainder chain is already the same gcd up to a nonzero scalar.

The classifier now reads the retained final remainder. A constant remainder
certifies that the source polynomial is square-free immediately. A
nonconstant remainder is reused as the repeated-root polynomial, and only that
smaller polynomial receives the additional Sturm chain needed to locate its
roots. A nonconstant chain that reaches the 64-polynomial construction bound
retains the former unbounded gcd path because its last remainder is not yet a
completion certificate. Exact-parameter derivative evaluation is unchanged.
The mixed simple/repeated-root regression explicitly verifies that root
isolation retained the certificate before multiplicity classification consumes
it.

On a same-build A/B of the one-cell all-family exact Boolean sentinel,
Callgrind instruction references fell from 26,627,740 to 26,021,056 (2.28%).
Inclusive cost beneath `simple_root_classifications` fell from 673,447 to
48,369 instructions (92.8%). Every run retained 9 candidate pairs, 48
fragments, 2 point classifications, 4 decided operations, no blockers, and
checksum 6. The optimized heaptrack run recorded 33,232 allocation events,
2,712 temporary allocations, 935.00 KiB peak heap, and 10.93 MiB peak RSS.
The release benchmark executable contains 4,639,985 bytes of text; the
release curve-intersection test binary attributes 1.4 MiB of text to
Hypercurve, 808.4 KiB to Hyperreal, 223.1 KiB to Hypersolve, 87.5 KiB to
Hyperlattice, and 14.5 KiB to Hyperlimit.

The blocker-resolution history audit and its named regression gate are
recorded in `BLOCKER_REGRESSIONS.md`. In particular, the formerly blocked
captured demo geometry now runs through core `CurveRegion2` preparation,
requires an empty blocker list, decides union, intersection, difference, and
XOR, and projects every result through the finite family-preserving boundary.

The complete all-feature/all-target test and benchmark suite, the no-default
library and integration suite, strict Clippy in both feature configurations,
and warnings-as-errors rustdoc passed. Default and no-default release WASM
library builds passed. The Hypercurve UI passed all 37 native tests and strict
Clippy, then Trunk 0.21.14 produced the release Pages bundle for
`/hypercurve/`. The requested `-runs=2509` AddressSanitizer region-Boolean
replay completed 2,718 executions at 5,985 coverage points and 19,306 feature
edges with no finding; LeakSanitizer remained disabled under ptrace.

### Exact Real-coefficient conic parameter images

The all-representation pathological schedule first became blocked when its
rational-quadratic weights reached `pi`. The specialized implicit-conic route
isolated the correct roots, but its rational parameter map then tried to force
those certified Real-coefficient roots through Hypersolve's intentionally
rational-coefficient algebraic-number representation. That conversion
correctly rejected the evidence, and the generic resultant fallback eventually
reported `RealSign`.

Prepared conic candidates now retain the exact numerator and denominator of
their rational parameter map. They also construct its exact image polynomial:
the resultant is sampled at enough integer image parameters to cover its
degree, then reconstructed by exact Lagrange interpolation. When the existing
rational-coefficient fast path cannot represent the source root, Hypercurve
isolates the Real-coefficient image roots once and refines the source isolator
until conservative exact interval arithmetic certifies that the mapped
interval lies inside exactly one image-root isolator. No primitive floating
point conversion or approximate topology decision is involved. Rational
Bezier point images likewise retain the certified source root and exact
rational-coordinate expressions when only the narrower Hypersolve
representation is unavailable.

On the 67-cell, 100.5 MiB all-family benchmark, the exact native Boolean path
moved from 40 decided and 228 blocked operations to all 268 operations decided
with no blocker. Preparation fell from approximately 312.397 seconds to
1.077 seconds (about 290x), and total native Boolean time fell from
approximately 312.5 seconds to 1.663 seconds (about 188x). The completed path
now materializes 3,248 fragments and performs 134 point classifications,
compared with 488 fragments and 20 classifications when most operations
stopped early, so the timing improvement includes substantially more exact
topology work. The family-flattened exact control decided the same 268
operations in 17.289 milliseconds.

Heaptrack measured 36.00 MiB peak heap for the complete 67-cell run; its
largest consumers remain Hyperreal rational storage and bigint growth rather
than the retained image objects. A three-cell Callgrind run, which includes the
first `pi` cell, completed all 12 operations and recorded 326,746,257
instruction references. Hyperreal computable structural equality was the
largest exclusive symbol at 13.48%, identifying expression comparison as the
next local optimization target. The release benchmark executable contains
4,663,333 bytes of text, 23,348 bytes (0.50%) more than the preceding
checkpoint.

The blocker matrix names both the end-to-end
`pathological_pi_weight_conic_decides_native_booleans_without_projection`
regression and the focused
`rational_point_image_retains_real_coefficient_root_expression` evidence test.
The complete all-feature suite and no-default library/integration suite,
strict all-target Clippy in both feature configurations, formatting,
warnings-as-errors rustdoc, and default/no-default release WASM library builds
passed. The Hypercurve UI passed all 37 native tests and strict Clippy; its
release WASM build and Trunk 0.21.14 Pages bundle for `/hypercurve/` both
completed successfully. The requested `-runs=2509` AddressSanitizer
region-Boolean replay completed 2,708 executions at 5,985 coverage points and
19,289 feature edges with no finding; LeakSanitizer remained disabled under
ptrace.

### Clone-shared exact expression identity

The preceding three-cell profile attributed 13.48% of all exclusive
instructions to Hyperreal's structural expression equality. Retained
Hypercurve geometry frequently compares two handles cloned from the same
immutable `Computable` node, but the scalar layer formerly traversed the
complete expression graph before accepting those handles as structurally
equal.

Hyperreal now accepts shared node identity first while preserving recursive
comparison for independently constructed graphs. On the identical three-cell
Callgrind workload, instruction references fell from 326,746,257 to
273,691,629 (16.24%). The complete 67-cell, 100.5 MiB native Boolean
benchmark's matched ten-run median fell from 1.6343 seconds to 1.0095 seconds
(38.2%). Preparation fell 10.7%, from 1.0640 seconds to 949.8 milliseconds;
union fell from 539.3 to 26.75 milliseconds (95.0%, or 20.2x). All 268
operations remained decided with no blockers, 2,883 candidate pairs, 3,248
fragments, 134 point classifications, and checksum 6.

Heaptrack remained at 14,614,979 allocation events and 36.00 MiB peak heap.
The release benchmark gained 160 bytes of text, while its total loadable size
remained unchanged. Hyperreal retains a named shared-composite regression and
Criterion row for the optimized comparison.

Validation passed across the complete Hyperreal all-target suite and
fuzz-target build, all-feature suites in Hyperlattice, Hyperlimit, Hypersolve,
and Hypercurve, strict Hypercurve Clippy, all 37 UI tests, UI Clippy, and the
release WASM demo build. The `computable_approximation` AddressSanitizer replay
completed 2,509 executions at 5,271 coverage points and 19,200 feature edges;
the downstream `region_boolean` replay completed 2,704 executions at 5,984
coverage points and 19,292 feature edges. Neither found an error; LeakSanitizer
alone remained disabled under ptrace.

### Bounded exact GCD reduction

The next three-cell profile attributed 9.34% of the complete workload to
`num_bigint` right shifts inside balanced rational-operation GCDs. Hyperreal
now extends its existing allocation-free fixed-limb binary reducer from 256 to
512 bits while preserving the backend for larger operands. Its focused cold
reduction improved 21.8%, from 6.396 to 5.000 microseconds.

On the identical three-cell workload, instruction references fell from
273,691,629 to 266,961,560 (2.46%). The complete 67-cell native Boolean
ten-run median fell from 1.0095 seconds to 965.8 milliseconds (4.33%);
preparation fell 3.95% and union fell 8.80%. All 268 operations remained
decided with no blockers and unchanged candidate, fragment, classification,
and checksum counts.

Peak heap remained 36.00 MiB and recorder-inclusive peak RSS was 96.81 MiB.
Allocation events increased 0.36%, while postprocessed temporary allocations
fell by 67. The release benchmark gained 6,080 bytes of text (0.13%) and 4,096
bytes of total loadable size.

Validation passed across all-feature Hyperreal, Hyperlattice, Hyperlimit,
Hypersolve, and Hypercurve suites, strict Hyperreal/Hypercurve/UI Clippy, all
37 UI tests, and the release WASM demo build. The `rational_arithmetic`
AddressSanitizer replay completed 2,509 executions at 1,830 coverage points
and 4,630 feature edges; `region_boolean` completed 2,706 executions at 5,989
coverage points and 19,320 feature edges. Neither found an error;
LeakSanitizer alone remained disabled under ptrace.

### Exact bound caching and shared cancellation

Hyperreal approximation-bound publication now derives the magnitude bit length
without allocating a temporary absolute-value bigint. The concurrent cache
upgrade only replaces absent or unknown bounds, so an approximation cannot
weaken an exact structural MSD certificate already published by another
analysis.

The same audit found a supportable `RealSign` blocker in rational Bézier
weights shaped as `(pi + epsilon) - pi`, where `epsilon` was smaller than the
sign-refinement floor. Exact nested-addition cancellation now recognizes the
shared or structurally equivalent term and returns `epsilon` directly.
Rational Bézier monotonicity, evaluation, derivative, bounds, and disjoint
contact queries that previously preserved this blocker now complete with exact
curve evidence. `BLOCKER_REGRESSIONS.md` names the focused Hyperreal sign and
atan2 tests and all three downstream Hypercurve blocker-resolution tests.

On the identical three-cell exact native Boolean workload, instruction
references fell from 266,961,560 to 261,496,219 (2.05%). The matched ten-run
median for the complete 67-cell workload fell from 965.8 to 949.3
milliseconds (1.7%), and preparation fell from 912.3 to 895.6 milliseconds
(1.8%). Heaptrack allocation events fell from 14,668,175 to 13,458,420
(8.25%); peak heap remained 36.00 MiB.

The final current-build native run prepared all 67 regions and decided all 268
union, intersection, difference, and XOR operations with zero blockers. It
retained 3,248 exact native curve fragments from 2,883 candidate pairs,
performed 134 exact point classifications, and produced checksum 6. The
family-flattened exact-polyline lane remains a comparison only and is never
substituted for a native curve result.

All-feature Hyperreal and Hypercurve library/integration suites and strict
all-target Clippy passed. The `computable_approximation` AddressSanitizer
replay completed 2,509 executions without a finding; the downstream
`region_boolean` replay completed 2,717 executions at 5,987 coverage points
and 19,309 feature edges without a finding. LeakSanitizer alone remained
disabled under ptrace.

### Exact quadratic-surd collapse certification

A convex right-triangle erosion exposed a remaining supportable blocker at its
exact collapse distance, `4 - 2 sqrt(2)`. The native offset constructed exact
curves, but its final degeneracy predicate expanded to a quadratic-surd
identity that ordinary structural cancellation did not recognize. The
operation therefore returned `RealSign` uncertainty instead of the exact empty
region.

Hyperreal now has a bounded exact reducer for one quadratic field
`a + b sqrt(d)`. It recognizes rational arithmetic, shared square-root
constants, exact rational square roots, powers of two, products, squares,
inverses, and fixed three-term linear combinations. Sign comparison squares
the opposing rational magnitudes exactly; it never uses an approximation as a
predicate. Parsing is capped at 256 expression nodes and memoizes shared DAG
nodes in a lazy compact vector, so unrelated or larger expression families
continue through the existing conservative path.

The named Hypercurve regression now requires the inradius erosion to return a
decided empty `CurveRegion2`. Hyperreal regressions independently cover the
expanded collapse identity, conjugate inversion, nonzero surd ordering, and a
previously unresolved opposite-sign sum. `BLOCKER_REGRESSIONS.md` links the
downstream resolution to every scalar regression.

On the identical three-cell Callgrind workload, instruction references moved
from 261,496,219 to 262,166,772, a 0.26% correctness cost. In the complete
67-cell native workload, the stronger exact signs eliminated 120 conservative
candidate pairs, from 2,883 to 2,763. Every one of the 268 union,
intersection, difference, and XOR operations still completed with exact native
curve results, zero blockers, 3,248 fragments, 134 point classifications, and
checksum 6.

The current ten-run native median was 994.7 milliseconds, compared with 949.3
milliseconds before the additional proof capability. The family-flattened
exact-polyline comparison median was 17.43 milliseconds, so the native path
remains about 57 times slower even though both decide all 268 operations.
Consequently, the `LineArc` accelerator remains: its correctness replacement
gate is satisfied for this fixture, but its performance-removal gate is not.

Heaptrack recorded 13,717,229 allocation events, 2,495,405 temporary
allocations, and 36.05 MiB peak heap. Relative to the prior exact-bound/shared-
cancellation baseline, allocations increased 1.92%, temporary allocations
fell 0.78%, and peak heap was effectively unchanged. The release benchmark
grew by 9,560 bytes of text (0.20%), 8,168 bytes of total loadable size
(0.17%), and 10,608 bytes on disk (0.17%).

All 584 all-feature Hyperreal library tests passed. The complete all-feature
and no-default-feature Hypercurve library/integration suites passed, and the
final compact-memo representation was rechecked through the focused collapse
test in both feature configurations. Strict all-target Clippy passed in both
repositories. The `computable_approximation` AddressSanitizer replay completed
2,509 executions at 5,410 coverage points and 19,539 feature edges; the
downstream `region_boolean` replay completed 2,721 executions at 5,980 coverage
points and 19,259 feature edges. Neither found an error; LeakSanitizer alone
remained disabled under ptrace.

### Once-only certified bounds before implicit conic solving

Retained rational-Bezier preparation formerly tried the implicit-conic solver
before consulting the same-sign control-hull certificate already used by the
generic resultant path. A disjoint conic/cubic pair therefore constructed the
cubic homogeneous power basis, substituted it into the conic implicit form,
and isolated an empty polynomial root set before reaching an exact bounds test
that could have rejected the pair immediately.

Preparation now performs that exact rejection first. When both rational curves
have one certified weight sign and their exact control-hull boxes are disjoint,
it retains an already-complete `NoIntersection` result without constructing
implicit or resultant algebra. Overlapping and uncertified boxes continue
unchanged. The generic fallback reuses the first bounds outcome rather than
boxing and comparing the pair a second time. This is a once-visiting schedule
change only: lossy coordinates do not participate in the decision.

The new cold benchmark improved from 10.945 microseconds to 413 nanoseconds per
disjoint conic/cubic preparation, a 96.2% reduction (26.5x). Its regression
requires complete cached no-contact evidence and verifies that the cubic
homogeneous power basis remains unconstructed, proving that the implicit solver
was skipped rather than merely producing the same answer later.

On the identical three-cell exact-native Callgrind workload, instruction
references fell from 262,166,772 to 229,883,683 (12.31%). The final ten-run
median for the complete 67-cell workload fell from 994.7 to 861.1
milliseconds (13.4%), while preparation fell from 936.2 to 802.5 milliseconds
(14.3%). All 268 native union, intersection, difference, and XOR operations
remained exact and decided with zero blockers, 2,763 carrier candidate pairs,
3,248 fragments, 134 point classifications, and checksum 6.

Heaptrack allocation events fell from 13,717,229 to 11,563,420 (15.70%) and
temporary allocations from 2,495,405 to 2,093,056 (16.12%). Peak heap moved
from 36.05 to 35.97 MiB. The release benchmark added 4,192 bytes of text
(0.09%), 4,096 bytes of total loadable size (0.08%), and 4,856 bytes on disk
(0.08%).

The latest family-flattened exact-polyline comparison median was 18.27
milliseconds, leaving the native exact-curve path about 47 times slower.
`LineArc` removal therefore remains gated on native performance even though
the workload's correctness and feature-completion gate remains satisfied.

Complete all-feature and no-default-feature library/integration suites passed,
as did strict all-target Clippy. The targeted `bezier_region`
AddressSanitizer replay completed 2,509 executions at 10,140 coverage points
and 34,775 feature edges without a finding; LeakSanitizer alone remained
disabled under ptrace.

### Direct exact contacts reject certified disjoint bounds first

The public `RationalBezier2::intersection_contacts` path still performed the
implicit-conic substitution before the certified control-hull rejection used
by retained preparation and generic resultant candidates. It also called the
generic candidate entry point after the special cases, repeating both rational
bounds and their exact overlap comparison when the boxes were not disjoint.

Direct contacts now use the shared certified-bounds rejection before any
implicit or resultant algebra and continue directly to the post-bounds
candidate path after the special cases. A missing weight-sign or box-ordering
certificate still falls through to the complete algebraic solver, while
touching boxes remain inclusive. The optimization therefore changes only the
once-visiting schedule; every rejection still comes from same-sign rational
control hulls and exact `Real` comparisons.

The new direct-path regression checks a disjoint conic/cubic pair, requires a
complete `NoIntersection` result, and verifies that the cubic homogeneous power
basis remains unconstructed. Before the source change, the new 2,000-iteration
cold benchmark took 10.947 microseconds per call. Its five-run optimized median
is now 396 nanoseconds per call, a 96.4% reduction (27.6x).

The complete 67-cell exact-native workload still prepared every region and
decided all 268 Boolean operations with zero blockers, 2,763 carrier pairs,
3,248 fragments, 134 point classifications, and checksum 6. The measured run
took 836.0 milliseconds, including 779.3 milliseconds of retained preparation;
the direct-only change is intentionally outside that retained call graph. The
pathological benchmark's text, data, BSS, and total loadable sizes were
unchanged at 4,682,077, 256,360, 2,960, and 4,941,397 bytes respectively; its
file grew by 32 bytes.

Complete all-feature and no-default-feature library/integration suites passed,
including both retained and direct cache-state regressions, as did strict
all-target Clippy. The `bezier_region` AddressSanitizer replay completed 2,509
executions at 10,241 coverage points and 37,743 feature edges without a finding;
LeakSanitizer alone remained disabled under ptrace.

### Lazy exact conic image fallback

Implicit conic contact analysis built the real-coefficient algebraic image
polynomial for every parameter map. Constructing that fallback
evaluates one resultant per source-polynomial sample, interpolates those exact
`Real` values, certifies the resulting degree, and later isolates its roots.
The primary `AlgebraicRootRationalMap` transform usually decides the
same parameter directly, so most of this symbolic algebra was never read.

The parameter candidate now retains an empty `OnceCell` for the fallback image
polynomial. Only an uncertain primary transform constructs it, deriving the
source polynomial from the already-retained algebraic parameter; its exact
polynomial and isolated image roots remain cached for subsequent refinement.
This changes neither representation nor acceptance: primary and fallback maps
use the same exact numerator and denominator, and no finite projection is
introduced.

The focused regression isolates a quadratic algebraic parameter, applies an
exact identity rational map, and verifies both that the mapped parameter is
decided and that the fallback `OnceCell` remains empty. Existing conic/cubic
contact and pathological pi-weight Boolean regressions continue to exercise the
fallback and complete exactly. A new 100-iteration pi-weight conic/cubic
benchmark moved from 259.630 to a five-run median of 234.401 microseconds per
contact query (9.72%).

On the identical three-cell exact-native Callgrind workload, instruction
references fell from 229,883,683 to 162,196,807 (29.44%). Dispatch tracing
retained the same 310 certified sign refinements while reducing exact `Real`
rational constructions from 74,832 to 55,335, zero constructions from 20,816
to 17,884, rational reductions from 1,820 to 352, and GCDs from 2,693 to 716.
The gain therefore comes from deferred unused algebra rather than weaker
predicate work.

The ten-run median for the complete 67-cell workload fell from 861.1 to 790.5
milliseconds (8.20%), with retained preparation falling from 802.5 to 733.2
milliseconds (8.63%). Every run still prepared all 67 regions and decided all
268 native Boolean operations with zero blockers, 2,763 carrier pairs, 3,248
fragments, 134 point classifications, and checksum 6. The corresponding
exact-polyline median was 17.96 milliseconds, so native exact curves remain
about 44 times slower and `LineArc` removal remains performance-gated.

Heaptrack allocation events fell from 11,563,420 to 10,928,391 (5.49%),
temporary allocation events from 2,093,056 to 2,083,289 (0.47%), and peak heap
from 35.97 to 35.69 MiB. The release pathological benchmark lost 3,336 bytes
of text, 4,072 bytes of total loadable size, and 2,144 bytes on disk.

Complete all-feature and no-default-feature library/integration suites passed,
including the slow all-operation exact Boolean regression, as did strict
all-target Clippy. The `bezier_region` AddressSanitizer replay completed 2,509
executions at 10,312 coverage points and 39,522 feature edges without a finding;
LeakSanitizer alone remained disabled under ptrace.

### Exact Bernstein endpoint roots avoid radical cancellation

The pi-weight rational quadratic in the pathological fixture has a transformed
axis derivative whose endpoint Bernstein value is exactly zero. The generic
power-basis quadratic solver nevertheless constructed that endpoint through
the quadratic formula. Comparing the resulting radical expression with one
could not prove equality even after refinement to precision -4096, so certified
bounds discarded the monotone split and retained a much wider control hull.

Quadratic Bernstein callers now pass their retained endpoint values into the
shared root solver. Structurally zero endpoints are emitted directly as exact
zero or one parameters, and the other root is recovered by Vieta's formula
before the generic radical path is considered. Line contacts, point incidence,
matching-weight graph roots, and rational-quadratic monotonicity all use the
same endpoint-preserving path. No approximate parameter is introduced.

The focused pi-weight regression failed with `Ordering` before the change and
now returns the exact monotone parameter `[1]`. The complete 268-operation
regression continues to pass, and its debug runtime fell from 9.05 to 6.26
seconds. Dispatch tracing on the three-cell workload removed every unresolved
sign refinement: 303 sign queries remain, with 224 decided at precision zero
and 79 at the first -16 step. Previously 310 queries included two comparisons
that exhausted -512 and then retried the same expression through -4096.

On the identical three-cell exact-native Callgrind workload, instruction
references fell from 162,196,807 to 147,498,172 (9.06%). Exact endpoint bounds
also reduced the complete 67-cell workload from 2,763 to 603 carrier pairs
(78.2%) while retaining 3,248 fragments, 134 point classifications, all 268
exact Boolean results, zero blockers, and checksum 6.

The final five-run complete-workload median fell from 790.5 to 574.3
milliseconds (27.4%), with retained preparation falling from 733.2 to 519.4
milliseconds (29.2%). The matched exact-polyline median was 17.06 milliseconds,
narrowing the native exact-curve gap from roughly 44x to 34x; `LineArc` removal
therefore remains performance-gated. The release pathological executable now
contains 4,683,825 bytes of text and 4,945,497 total loadable bytes.

### Refined quotient-ring conic parameter images

The lazy conic parameter fallback still started too early. A coarse algebraic
source interval can leave a rational-map denominator apparently containing
zero even though the represented root is outside that zero set. The candidate
loop immediately constructed the fallback image polynomial on that first
answer, so its later exact interval refinements never had an opportunity to
certify the already-prepared primary map.

Primary rational maps now exhaust the retained `[0, 2, 4, 8]` refinement
schedule before any real-coefficient image fallback is considered. The focused
regression starts with a denominator interval containing zero, proves that
refinement decides the exact image, and verifies that the fallback `OnceCell`
remains empty. Full-workload dispatch records 67 initial
`denominator-may-contain-zero` reports, followed by exact primary decisions
without constructing a fallback for those coarse reports.

The 114 genuinely non-rational defining equations that still require a
fallback formerly evaluated and interpolated one Bareiss resultant per image
degree. The replacement reduces the rational-map numerator and denominator
once in the quotient ring of the source polynomial, then computes
`det(M_n - y M_d)` directly over exact `Real` coefficients. Its subset
determinant is bounded to source degree 12; higher degrees retain the existing
sampled Bareiss path. The new non-rational regression eliminates
`x² - pi` through `y = x / (x + 1)` and checks every exact coefficient of
`y² - pi(1 - y)²`. No primitive approximation or projected curve participates.
Dispatch confirms that all 114 workload fallbacks use the quotient-ring path
and none use sampled Bareiss interpolation.

On the identical three-cell exact-native Callgrind workload, instruction
references fell from 147,498,172 to 122,760,480 (16.77%). Heaptrack allocation
events fell from 220,100 to 173,956 (20.97%), and temporary allocations fell
from 34,085 to 19,534 (42.69%). The complete debug blocker regression now
finishes in 4.29--4.43 seconds while still returning all 268 exact results.

The final seven-run complete-workload median fell from 574.3 to 432.0
milliseconds (24.8%), with retained preparation falling from 519.4 to 379.9
milliseconds (26.9%). Every run retained 603 carrier pairs, 3,248 fragments,
134 point classifications, all 268 exact Boolean results, zero blockers, and
checksum 6. The matched exact-polyline median was 17.88 milliseconds, narrowing
the native exact-curve gap from about 34x to 24.2x; `LineArc` removal therefore
remains performance-gated.

The release pathological executable contains 4,696,565 bytes of text,
4,957,789 total loadable bytes, and 6,234,776 bytes on disk. Relative to the
preceding endpoint-root build, those are increases of 12,740, 12,292, and
18,112 bytes respectively. Complete all-feature and no-default-feature
library/integration suites passed, including the full 268-operation blocker
sentinel and the new fallback regressions, as did strict all-target Clippy in
both feature configurations.

### Shared nonrational quotient-ring source inverse

The numerator and denominator multiplication matrices for an algebraic
rational image reduce in the same quotient ring. Every column reduction
formerly divided by the unchanged source leading coefficient independently,
and the two matrices repeated that work again. For a symbolic leading
coefficient each division constructs a fresh exact reciprocal expression.
The paired construction now creates that reciprocal once and borrows it for
both matrices and every column. Exact-rational leaders retain the direct
division path, whose structural quotient simplifications are cheaper.

The regression scales `x² - pi` by the nonrational factor `pi`, maps both
defining polynomials through `y = x / (x + 1)`, and verifies exact coefficient
equality of the two quotient-ring images. This exercises the shared symbolic
inverse while proving that a nonzero source scale neither changes the image
polynomial nor introduces a projection.

On the identical three-cell exact-native Callgrind workload, instruction
references fell from 122,760,480 to 121,965,154 (0.65%). Heaptrack allocation
events fell from 173,956 to 171,683 (1.31%), and temporary events from 19,534
to 19,069 (2.38%). Dispatch tracing reduced generic exact reciprocal
construction from 124 to 114 events. The complete nine-run 67-cell workload
had a 427.8 millisecond median, including 376.4 milliseconds of retained
preparation, and continued to decide all 268 exact Boolean operations with
zero blockers. Its matched exact-polyline median was 18.03 milliseconds, so
the native exact-curve gap is now about 23.7x and `LineArc` removal remains
performance-gated.

The release pathological executable contains 4,697,361 bytes of text,
4,957,801 total loadable bytes, and 6,235,664 bytes on disk. Complete
all-feature and no-default-feature suites passed, including the full
268-operation blocker regression and the new nonrational-scale regression, as
did strict all-target Clippy in both feature configurations.

### Immediate top-level curve intersections

The public top-level curve workflow no longer exposes a prepared pair handle.
`Curve2::intersect_curve` immediately returns complete contact, overlap, and
blocker evidence, while `Curve2::intersection_topology` returns that evidence
with its exact split topology. `span_pair_count` moved onto the evidence result.
Path and region Boolean batches retain a private curve-pair context only for
the duration of one call, so their evidence passes still share expensive
native or resultant replay without exposing a long-lived cache API.

The private context no longer needs the prepared handle's outer `Rc` allocation
or its unused cached-topology slot. Clone-shared result and arrangement data
remain unchanged. Exact line, line/arc, coincident-arc, endpoint, retained
lineage, and general rational dispatch all feed the same immediate result
constructors, with no projection or predicate weakening.

The immediate curve/path regressions cover every native dispatch, partial
nonlinear and algebraic overlap splitting, spline-knot deduplication, operand
order, and the complete four-operation path matrix. The pathological blocker
sentinel still decides all 268 native exact Boolean operations with zero
blockers.

On the identical three-cell Callgrind workload, instruction references fell
from 97,554,466 after the immediate path-Boolean conversion to 96,661,943
(0.91%). Heaptrack allocation events fell from 130,794 to 130,767, exactly one
removed context allocation for each of the 27 candidate carrier pairs;
temporary allocations remained 11,973. A five-run complete 67-cell release
workload had a 271.2 millisecond native exact-Boolean median and a 16.83
millisecond exact-polyline median while preserving 603 candidate pairs, 3,248
fragments, 134 point classifications, checksum 6, and all 268 exact results.
The release pathological executable contains 4,752,425 bytes of text,
5,011,041 total loadable bytes, and 6,282,984 bytes on disk.
Complete all-feature and no-default-feature validation passed, including the
slow all-mode algebraic blocker regression, as did strict all-target Clippy in
both feature configurations.

### Immediate rational-Bezier intersections

The final public retained intersection handle has been removed.
`RationalBezier2::intersection_candidates`, `intersection_contacts`, and the
new `intersection_topology` now form an immediate result surface. A private
one-call context still shares candidates and paired contact replay while a
top-level curve/path/region operation is in progress, but it cannot escape or
be cloned into a long-lived prepared API. Its cached topology slot and outer
`Rc` allocation are gone; immediate topology is built exactly once for its
call, and returned topology continues to lazily share only its arrangement.

The disjoint conic/cubic regression proves that both immediate candidates and
contacts terminate at certified bounds without constructing the implicit
power basis. The algebraic-resultant regression now obtains immediate topology
and still verifies its exact algebraic contact, four source fragments, and
four-vertex arrangement. Exact full-image overlap continues to return the
typed arrangement blocker rather than projecting or manufacturing a split.
The blocker-history gate now explicitly names the complete 268-operation
mixed-family regression.

The release rational-Bezier benchmark measured disjoint candidates at
0.715 microseconds, disjoint contacts at 0.739 microseconds, algebraic
immediate contacts at 100.8 microseconds, and algebraic immediate topology
including arrangement access at 119.0 microseconds per call. On the identical
three-cell Callgrind workload, instruction references were 96,697,042 versus
96,661,943 at the preceding immediate-curve checkpoint (a 0.036% difference).
Heaptrack allocation events fell from 130,767 to 130,740, exactly one removed
allocation for each of the 27 candidate carrier pairs; temporary allocations
remained 11,973.

Five complete 67-cell release runs had a 272.8 millisecond native exact-Boolean
median and a 16.66 millisecond exact-polyline median. Every run preserved 603
candidate pairs, 3,248 fragments, 134 point classifications, checksum 6, all
268 exact results, and zero blockers. The release pathological executable
contains 4,749,141 bytes of text, 5,011,037 total loadable bytes, and
6,278,912 bytes on disk, reductions of 3,284, 4, and 4,072 bytes respectively.

### Persistent exact curve-region Boolean fuzzing

The exact curved-region property test now generates paired closed regions over
all eight retained carrier families, requires blocker-free intersections and
all four Boolean results, and compares each immediate result with the shared
four-operation batch. Proptest persists minimized failures beside the test and
replays them before fresh generation. A separate named corpus maps every
retired failure category to at least one authored geometry and compares that
coverage with the exhaustive `RetiredFailure::ALL` list.

The corpus also retains the shared-endpoint XOR failure whose filled-face
traversal must pair each incoming half-edge with a distinct outgoing boundary
at a four-carrier vertex.

The first campaign exposed a uniform-weight general rational region whose
orientation was previously unsupported. Equal nonzero weights now cancel
exactly into the arbitrary-degree polynomial Green integral. Nonuniform
rational loops can instead carry authored left/right interior-side topology;
this is exact caller evidence and does not project or sample the carrier.

The next persisted seed exposed an algebraic point-image interval accelerator
that discarded a real rational/cubic contact. Those intervals were only an
optimization and were not sound enough to decide pair inequality, so exact
candidate replay no longer uses them. The resulting complete replay then
exposed a degree-twelve rational/rational intersection parameter whose cubic
coordinate map exceeded Hypersolve's dimension-12 rational-image ceiling. The
bounded exact package now admits Sylvester dimension 16 and has a direct
degree-twelve/cubic regression.

The extended campaign then found finite rational/line contacts hidden by
degree-elevated line parameterization factors. Exact line images are now
recognized before the generic resultant: supporting-line roots are solved
directly, mapped back to the finite line parameter through an exact rational
image, and retained as exact or algebraic point evidence. Conic parameter
charts also skip an exact pole and try their remaining charts instead of
turning a projective base point into global `Predicate` uncertainty.

The extended generated campaign found three further exact replay defects.
Conic charts now retain certified out-of-domain image results, polynomial-graph
replay no longer assumes the two unpaired resultant projections have equal
cardinality, and Hypersolve skips retained-parameter samples where
specialization lowers a Sylvester polynomial's eliminated degree. The last fix
restored the correct roots for nonuniform rational/rational contacts and made
the full pathological native workload complete all 268 exact Booleans.
Independently degree-elevated line images also replay their certified partial
overlap before entering the generic resultant.

The next minimized seed combined a shared conic endpoint with a second
nonrational intersection. Unit-interval isolation had correctly deflated the
represented endpoint, so the remaining algebraic root retained a lower-degree
carrier. Shared multiplicity classification now uses that certified carrier
for algebraic roots while continuing to classify represented endpoints
against the original polynomial.

Removing the unsafe pruning increases difficult debug-profile campaigns:
32 generated cases plus the persisted seeds complete in about 9.6 seconds on
the current host. This is the deliberate exactness/performance boundary until
a replacement accelerator carries independently validated conservative image
bounds.

### Immediate unified-region classification

`CurveRegion2` no longer exposes `CurveRegionQuery2`. The facade only borrowed
the authoritative region, populated caches already owned clone-shared by that
region, and conditionally constructed a native `RegionQuery2`. Immediate
classification already selected the cached line-image region directly. Immediate
signed depth now follows the same route after populating the region-owned caches,
instead of allocating segment boxes, sweep indexes, winding indexes, and prepared
predicate handles for a transient query object on every call.

The retained `curve_region_immediate_native_signed_depth` benchmark guards this
one-shot path. On the current host, 2,000 exact classifications of a point inside
an exact native rectangle measured 1.219 microseconds per call. Replaying the
removed facade's native-query construction schedule over the same source,
point, policy, and checksum measured 10.542 microseconds per call, so the
immediate path is 88.4% faster. Higher-order and signed-loop paths still reuse
the same native-bound and rational-evaluator caches; no projection, finite
predicate, or topology contract changed.

The complete all-feature and no-default-feature library/test matrices, strict
all-target Clippy, formatting, and warning-denied rustdoc passed. Regenerated
nightly-rustdoc API coverage and the five-crate static call graph contain no
`CurveRegionQuery2` item or call edge. The no-default matrix also exposed and
fixed a feature-gated `Ordering` import left by the preceding exact conic replay.

### Immediate native query surface

The remaining public `CurveStringQuery2`, `ContourQuery2`, and `RegionQuery2`
handles duplicated immediate intersections, trimming, Boolean operations, and
point classification while making cache lifetime a caller concern. They are no
longer exported. Conservative boxes, winding indexes, and prepared predicates
remain internal implementation details. Immediate `structural_facts` methods
return scheduling evidence directly, while `Contour2::classify_points` and
`LineArcRegion2::classify_points` reuse one internal index across an explicit
point batch. `hyperbrep` follows the same model through
its transient `LineArcRegion2::classify_point` face-region query rather than a
retained public query facade.

The removal deleted 1,060 net production lines from `hypercurve` and 90 from
`hyperbrep`; tests, fuzzers, benchmarks, and documentation were migrated away
from retained public handles. A same-compiler build of commit `64933fd` and the
new source measured the default-feature pathological benchmark executable:

| Release artifact component | `64933fd` | Immediate-only surface | Change |
| --- | ---: | ---: | ---: |
| Text | 4,877,637 bytes | 4,870,301 bytes | 7,336 bytes smaller (0.150%) |
| Total loadable | 5,138,013 bytes | 5,133,925 bytes | 4,088 bytes smaller (0.080%) |
| File size | 6,441,104 bytes | 6,427,312 bytes | 13,792 bytes smaller (0.214%) |

The release containment sentinel classified 64 decided contour misses in
8.207 microseconds per batch (128 nanoseconds per point), versus 461
nanoseconds for scalar immediate calls, a 72.2% per-point reduction. A mixed
64-point sparse-region batch measured 1.215 milliseconds (18.99 microseconds
per point), versus a 45.56-microsecond scalar average for its alternating
outside and single-hit points, a 58.3% reduction. These are complete-call
measurements including internal index construction and output allocation; they
do not claim the lifecycle semantics of the retired already-built handle.

The first replacement routed one-shot region trimming through a transient
retained index and regressed the release sentinel from 14.1 to 29.4
microseconds, so that experiment was reverted. The final immediate region trim
measured 14.056 microseconds versus 14.138 at `64933fd`. Reusing source boxes
inside immediate two-cutter curve trimming was retained: that lane improved
from 5.333 to 5.051 microseconds (5.3%) without changing exact intersection or
materialization behavior.

### Immediate symbolic-conic contacts

The immediate rational-Bezier contact path formerly treated an uncertain
direct line-image shortcut as terminal. A quadratic with a symbolic weight
such as pi therefore returned a `RealSign` blocker before reaching the exact
implicit-conic replay that already supports coefficients in that field.
Immediate contacts now treat only decided shortcut results as terminal and
schedule symbolic quadratic conics through implicit replay first. Candidate,
contact, and topology regressions cover the exact pi-weight conic against a
degree-elevated cubic line in both operand orders.

Five alternating release runs of the complete `rational_bezier` benchmark
measured a median 1.024462 milliseconds per pi-conic contact call before route
selection and 775.225 microseconds after it, a 24.3% reduction. Both variants
included the correctness fallback; the comparison isolates the scheduling
change. Callgrind over the full benchmark measured 4,513,909,722 versus
4,300,545,239 instructions (4.73% fewer). Inclusive work below
`exact_line_image_intersection_contacts` fell from 459,522,283 to 247,279,812
instructions (46.2%), removing that unsuccessful layer from each symbolic
conic call.

Heaptrack over the same complete benchmark recorded 4,983,018 versus 4,722,544
allocations (5.23% fewer), 535,909 versus 453,880 postprocessed temporary
allocations (15.3% fewer), and 23.85 versus 23.84 MiB peak heap. A same-compiler
symbolized executable changed as follows:

| Release artifact component | Fallback only | Symbolic implicit-first | Change |
| --- | ---: | ---: | ---: |
| Text | 3,261,112 bytes | 3,262,724 bytes | 1,612 bytes larger |
| Total loadable | 3,515,050 bytes | 3,515,046 bytes | 4 bytes smaller |
| Debug file size | 122,954,768 bytes | 122,985,160 bytes | 30,392 bytes larger |

The full rational, curve-path, curve-region, containment, editing, and
all-feature API-surface benchmark gates completed. The all-feature test suite,
the 268-decision pathological Boolean workload, strict all-target Clippy, and
10,000 nightly libFuzzer algebraic-image runs under AddressSanitizer also
completed without an exactness or safety failure.

### Deferred implicit-conic contact coordinates

Implicit-conic replay already proves an exact pair of source parameters maps to
the same contact point. It formerly evaluated both algebraic coordinate images
immediately even though region Boolean topology only observes those coordinates
when it must deduplicate a contact vertex. Algebraic contacts now retain the
exact rational-Bezier source and algebraic parameter behind the existing
`RationalBezierIntersectionPointEvidence2::Algebraic` surface. The parameter
remains immediately available; x and y images are materialized once, through a
clone-shared cache, only if topology comparison requests them. Exact-parameter
contacts retain their previous eager path.

The one-cell pathological Boolean sentinel retained identical work: four
decided operations, nine candidate pairs, 48 fragments, two point
classifications, no blockers, and checksum 6. Callgrind measured 28,520,330
instructions before and 28,089,032 after, a reduction of 431,298 instructions
(1.51%). Heaptrack recorded 37,793 versus 36,802 allocations (2.62% fewer) and
2,942 versus 2,723 postprocessed temporary allocations (7.44% fewer). Peak RSS
fell from 11.63 to 11.47 MiB. Peak heap rose from 985.42 to 993.77 KiB and
reported retained memory rose from 210.97 to 213.02 KiB; these costs are
recorded rather than hidden by the allocation-count improvement.

Fifteen CPU-pinned, alternating executions of the complete 67-cell native
Boolean lane measured medians of 582.504 milliseconds before and 574.434
milliseconds after, a 1.39% reduction. Every execution completed 268 decided
operations over 603 candidate pairs and 3,248 fragments with no blocker and
checksum 6.

A same-compiler symbolized pathological executable changed as follows:

| Release artifact component | Eager coordinates | Deferred coordinates | Change |
| --- | ---: | ---: | ---: |
| Text | 4,866,059 bytes | 4,866,715 bytes | 656 bytes larger |
| Total loadable | 5,125,731 bytes | 5,129,827 bytes | 4,096 bytes larger |
| Debug file size | 124,856,728 bytes | 124,906,632 bytes | 49,904 bytes larger |

The workspace call-graph utility regenerated all source, test, benchmark,
example, and fuzz nodes for `hypercurve`, `hyperlattice`, `hyperlimit`,
`hyperreal`, and `hypersolve`. SCC-condensed depth stayed 4 for
`CurveRegion2::boolean_regions`, 9 for
`RationalBezier2::implicit_conic_intersection_contacts`, and 6 for the
contact-comparison helper; every reachable SCC remained a single node.

The final source passed the complete all-feature/all-target test run, strict
all-target Clippy, warning-denied rustdoc, the no-default-feature library/test
matrix, and `cargo bench --workspace --all-features`. The benchmark command
executed every benchmark target, including every immediate API lane and the
complete comparative target; no representative subset was substituted.

### Algebraic conic primary-absence short circuit

The implicit-conic replay maps each exact source parameter through a primary
conic chart before considering two equivalent fallback charts. For an
algebraic source parameter, the prepared rational-image route returns
`Decided(None)` only after proving that the chart image is disjoint from the
target parameter interval. The fallback charts represent the same nonsingular
conic parameter and therefore cannot recover an in-range value. Replay now
accepts that exact negative result immediately. An exact source parameter keeps
the old fallback behavior because its direct quotient-ring image also uses
`None` when the primary chart has a pole.

The one-cell pathological Boolean sentinel retained the same four decisions,
nine candidate pairs, 48 fragments, two point classifications, zero blockers,
and checksum 6. Callgrind fell from 28,089,644 to 24,958,182 instructions, a
reduction of 3,131,462 instructions (11.15%). Fifteen CPU-pinned alternating
runs of the complete 67-cell native Boolean lane measured medians of 546.624
milliseconds before and 467.197 milliseconds after, a 14.53% reduction. Every
run completed all 268 decisions over 603 candidate pairs and 3,248 fragments
with zero blockers and checksum 6. The final unpinned release all-benchmark run
completed the same lane in 464.411 milliseconds.

Heaptrack over matched one-cell executables changed as follows:

| Heap metric | Fallback charts after exact absence | Short circuit | Change |
| --- | ---: | ---: | ---: |
| Allocation events | 36,801 | 31,702 | 5,099 fewer (13.86%) |
| Postprocessed temporary allocations | 2,722 | 2,083 | 639 fewer (23.48%) |
| Peak heap | 993.77 KiB | 960.37 KiB | 33.40 KiB lower |
| Peak RSS | 11.45 MiB | 11.48 MiB | 0.03 MiB higher |
| Reported retained memory | 213.02 KiB | 179.62 KiB | 33.40 KiB lower |

A same-compiler symbolized pathological executable retained the same
5,129,827-byte total loadable size. Text grew by 144 bytes, BSS fell by 144
bytes, and the debug file grew by 2,616 bytes; the matched stripped file grew by
144 bytes. The workspace call-graph utility regenerated all source, test,
benchmark, example, and fuzz nodes across `hypercurve`, `hyperlattice`,
`hyperlimit`, `hyperreal`, and `hypersolve`. Its graph contains 41,973 nodes
and 71,153 edges. SCC-condensed reachable counts and depths were unchanged:
68/depth 4 for `CurveRegion2::boolean_regions`, 224/depth 9 for implicit-conic
contacts, 155/depth 8 for conic parameter recovery, and 48/depth 6 for contact
comparison; each reachable SCC remained a single node.

The final source passed the complete all-feature/all-target workspace test run,
the no-default-feature library/test matrix, strict all-target Clippy,
warning-denied rustdoc, and `cargo bench --workspace --all-features`. That
benchmark command executed every benchmark target, including all immediate API
and comparative lanes. AddressSanitizer fuzzing completed 10,000
algebraic-image runs and 10,000 region-Boolean runs. The heavier Bézier-region
fuzzer completed 10,381 target executions across fresh processes; its first
process stopped only at libFuzzer's cumulative 2 GiB RSS guard after 9,381
successful inputs, and the emitted input replayed alone in 38 milliseconds
without reproducing a target OOM.

No prepared API or carrier is removed by this change. It uses the existing
prepared algebraic-image proof to avoid redundant fallback work, so any future
prepared-surface removal remains subject to the complete gate below.

### Certified contact-source separation

Region Boolean topology deduplicates algebraic contact vertices by comparing
their exact affine coordinate images. Deferred implicit-conic contacts retain
the source rational Bezier and isolated source parameter, but the comparison
formerly resolved both coordinate images before proving that contacts from
widely separated source curves were unequal. Contact comparison now asks each
lazy point image for its source curve's certified bounds first. When both
rational Bezier weight sequences have a decided common sign, their exact
control hulls enclose the full affine images. Decided disjoint hulls therefore
prove point inequality without constructing either coordinate image. Overlap,
an uncertain weight sign, or nonparametric evidence falls through to the
existing exact represented-root comparison unchanged.

Regressions exercise both proof outcomes. Disjoint lazy sources remain
unresolved after comparison, while overlapping sources still resolve their
coordinate evidence and compare equal. The one-cell dispatch trace recorded 12
`source-bounds-disjoint` decisions and 40
`lazy-topology-deferred` endpoint images. It retained four decided operations,
nine candidate pairs, 48 fragments, two point classifications, no blockers,
and checksum 6. Rational temporaries fell from 16,267 to 15,701 (3.48%) and
GCDs from 186 to 156 (16.1%).

Callgrind over the matched one-cell exact Boolean workload fell from 24,958,182
to 23,457,930 instructions, a reduction of 1,500,252 instructions (6.01%).
Heaptrack changed as follows:

| Heap metric | Coordinate comparison | Source-hull rejection | Change |
| --- | ---: | ---: | ---: |
| Allocation events | 31,702 | 29,305 | 2,397 fewer (7.56%) |
| Postprocessed temporary allocations | 2,083 | 1,711 | 372 fewer (17.86%) |
| Peak heap | 960.37 KiB | 918.80 KiB | 41.57 KiB lower |
| Peak RSS | 11.48 MiB | 11.25 MiB | 0.23 MiB lower |
| Reported retained memory | 179.62 KiB | 169.54 KiB | 10.08 KiB lower |

The complete 67-cell release Boolean lane completed all 268 decisions over 603
candidate pairs and 3,248 fragments with zero blockers and checksum 6 in
450.579 milliseconds. The preceding checkpoint's final unpinned complete run
measured 464.411 milliseconds, so the observed reduction was 2.98%.

The exact prefilter adds code in exchange for removing substantially more
dynamic algebraic work. A same-compiler stripped pathological executable
changed as follows:

| Release artifact component | Coordinate comparison | Source-hull rejection | Change |
| --- | ---: | ---: | ---: |
| Text | 4,866,859 bytes | 4,877,009 bytes | 10,150 bytes larger |
| Total loadable | 5,129,827 bytes | 5,138,025 bytes | 8,198 bytes larger (0.160%) |
| Stripped file size | 5,129,360 bytes | 5,139,792 bytes | 10,432 bytes larger (0.203%) |

The workspace call-graph utility regenerated source, tests, benchmarks,
examples, and fuzz targets for `hypercurve`, `hyperlattice`, `hyperlimit`,
`hyperreal`, and `hypersolve`: 41,996 nodes and 71,186 edges. SCC-condensed
reachable counts/depths were 68/depth 4 for
`CurveRegion2::boolean_regions`, 224/depth 9 for implicit-conic contacts, and
60/depth 6 for contact comparison. Every reachable SCC remained a single node;
the Boolean and contact-comparison depths are unchanged.

The final source passed all-feature/all-target tests, the no-default-feature
library/test matrix, strict all-target Clippy, warning-denied rustdoc, and
`cargo bench --workspace --all-features`. That optimized command executed
every benchmark target, including every immediate API and comparative lane; no
representative subset was used. AddressSanitizer completed 10,000 region
Boolean runs and 10,000 algebraic-image runs without a target failure. Leak
detection alone was disabled because LeakSanitizer cannot run under the
managed ptrace environment.

No prepared API or implementation carrier is removed by this change.

### Retained-output-only Boolean fragment cloning

The immediate four-operation region Boolean path shares one classified split
topology across union, intersection, difference, and XOR. Result construction
formerly cloned every classified split fragment before applying the
operation-specific keep/discard action. The one-cell pathological workload
therefore cloned all 48 topology fragments four times even though discarded
fragments never entered an arrangement graph. Result construction now borrows
the shared classification through action selection and clones or reverses only
a retained output fragment. Every exact action, topology vertex, source index,
and traversal branch is unchanged.

Symbolized Callgrind attribution also ruled out an apparent adjacent hotspot:
all 182 `Curve2::point_at` calls and their 2,392,596 inclusive instructions
belong to benchmark fixture flattening, not Boolean result construction. On the
matched one-cell exact Boolean workload, total instructions fell from
23,525,066 to 23,446,849, a reduction of 78,217 instructions (0.33%).
Inclusive work in `build_boolean_region_from_topology` fell from 2,485,913 to
2,415,025 instructions (2.85%), and `CurveRegion2::boolean_regions` fell from
14,210,772 to 14,140,026 instructions (0.50%). The workload retained four
decisions, nine candidate pairs, 48 fragments, two point classifications, zero
blockers, and checksum 6. Its dispatch counts and 15,701 rational temporaries,
54 reductions, and 156 GCDs were unchanged.

The removed clones share their backing evidence and therefore were reference
count traffic rather than heap allocations. Heaptrack accordingly remained at
29,305 allocation events, 1,711 postprocessed temporary allocations, 918.80
KiB peak heap, and 169.54 KiB reported retained memory. Peak RSS including
Heaptrack overhead measured 11.27 MiB versus a matched 11.26 MiB baseline, a
0.01 MiB increase. A same-compiler default-feature release artifact lost 244
text bytes and four total loadable bytes; the stripped file shrank by 240
bytes, from 5,139,792 to 5,139,552 bytes.

The workspace call-graph utility regenerated source, tests, benchmarks,
examples, and fuzz targets for `hypercurve`, `hyperlattice`, `hyperlimit`,
`hyperreal`, and `hypersolve`: 41,997 nodes and 71,186 edges. Replacing the
parser's heuristic `.cloned()` target with an explicit fragment `clone` adds
one named leaf but no edge or depth. SCC-condensed reachable counts/depths were
68/depth 4 for `CurveRegion2::boolean_regions`, 75/depth 7 for Boolean result
construction, 224/depth 9 for implicit-conic contacts, and 60/depth 6 for
contact comparison. Every reachable SCC remained a single node.

The final source passed all-feature/all-target tests, the no-default-feature
library/test matrix, strict all-target Clippy, warning-denied rustdoc, and the
unfiltered `cargo bench --workspace --all-features` command. That optimized
command executed every benchmark target and lane. The complete 67-cell
pathological workload decided all 268 exact Booleans over 603 candidate pairs
and 3,248 fragments with no blocker and checksum 6. AddressSanitizer completed
10,000 region-Boolean runs and 10,000 algebraic-image runs without a target
failure. Leak detection alone remained disabled for the managed ptrace
environment.

No prepared API or implementation carrier is removed by this change.

### Connectivity-first retained Boolean endpoints

Retained arrangement traversal formerly built tangent evidence for every
materialized endpoint before it knew whether tangent ordering was needed.
Region Boolean construction already carries certified successor choices from
the classified split topology. When that evidence completely determines every
multi-successor vertex, endpoint tangents and their higher derivatives are
unobserved work.

Traversal with certified successors now builds a connectivity-only endpoint
view first. It retains exact endpoint coordinates and topology vertices but
does not construct first-, second-, or third-derivative evidence. If any
multi-successor vertex lacks a valid certified successor, traversal discards
that partial view and rebuilds the complete tangent-order view before choosing
a branch. The public tangent-ordered traversal still requests the complete
view immediately. A zero-tangent regression proves that certified branch-free
traversal no longer observes an unused tangent, while the existing partial
successor regression proves that fallback still rebuilds and applies all
higher-order exact evidence.

The matched one-cell pathological Boolean workload retained four decided
operations, nine candidate pairs, 48 fragments, two point classifications,
zero blockers, and checksum 6. Its trace used the connectivity-first path four
times and never rebuilt tangent order. It retained 40 lazy topology-keyed
endpoint images and 12 contact-source bound rejections. Rational reductions
and GCDs stayed at 54 and 156; rational temporaries fell from 15,701 to 14,252,
a 9.23% reduction.

Symbolized Callgrind attribution changed as follows:

| Inclusive instruction scope | Complete endpoint view | Connectivity first | Change |
| --- | ---: | ---: | ---: |
| Whole one-cell workload | 23,446,849 | 22,284,706 | 1,162,143 fewer (4.96%) |
| `CurveRegion2::boolean_regions` | 14,140,026 | 12,989,614 | 1,150,412 fewer (8.14%) |
| Shared Boolean result construction | 3,716,114 | 2,565,842 | 1,150,272 fewer (30.95%) |
| One result-region build | 2,415,025 | 1,264,692 | 1,150,333 fewer (47.63%) |
| Retained arrangement traversal | 2,104,569 | 962,607 | 1,141,962 fewer (54.26%) |

The old path made 78 native
`endpoint_data_with_higher_derivatives` calls costing 1,038,787 inclusive
instructions. The connectivity-only workload makes none of those calls.
Fixture flattening remains outside the changed Boolean path.

Heaptrack over same-compiler, one-cell executables changed as follows:

| Heap metric | Complete endpoint view | Connectivity first | Change |
| --- | ---: | ---: | ---: |
| Allocation events | 29,305 | 28,215 | 1,090 fewer (3.72%) |
| Postprocessed temporary allocations | 1,711 | 1,663 | 48 fewer (2.81%) |
| Peak heap | 918.80 KiB | 881.22 KiB | 37.58 KiB lower (4.09%) |
| Peak RSS | 11.27 MiB | 11.14 MiB | 0.13 MiB lower |
| Reported retained memory | 169.54 KiB | 155.82 KiB | 13.72 KiB lower (8.09%) |

A matched default-feature release pathological executable also shrank:

| Release artifact component | Complete endpoint view | Connectivity first | Change |
| --- | ---: | ---: | ---: |
| Text | 4,876,765 bytes | 4,873,209 bytes | 3,556 bytes smaller |
| Total loadable | 5,138,021 bytes | 5,133,921 bytes | 4,100 bytes smaller (0.080%) |
| Stripped file size | 5,139,552 bytes | 5,135,944 bytes | 3,608 bytes smaller (0.070%) |

The unfiltered optimized `cargo bench --workspace --all-features` command
executed every benchmark target and lane. The affected immediate API lanes
completed at 16.173 microseconds per union, 94.248 microseconds per four-op
region Boolean batch, and 111.578 microseconds per immediate curve-path Boolean
batch, with their exact checksums unchanged. The complete comparative matrix
ran rectangle and 64-, 256-, and 1,024-vertex polygon Booleans across all
implementations, as well as every offset and NURBS comparison; no
representative subset was substituted. The release 100 MiB pathological lane
completed all 268 exact Booleans over 603 candidate pairs and 3,248 fragments
with no blocker and checksum 6 in 440.582 milliseconds.

The workspace call-graph utility regenerated source, test, benchmark, example,
and fuzz nodes for `hypercurve`, `hyperlattice`, `hyperlimit`, `hyperreal`, and
`hypersolve`: 41,999 nodes and 71,199 edges. SCC-condensed reachable
counts/depths stayed 68/depth 4 for `CurveRegion2::boolean_regions` and
75/depth 7 for Boolean result construction. The private retained traversal
gained two reachable leaf nodes, from 228 to 230, while staying depth 11.
Implicit-conic contacts remained 224/depth 9 and contact comparison remained
60/depth 6. Every reachable SCC remained a single node.

The final source passed the complete all-feature/all-target workspace test run,
the no-default-feature library/test matrix, strict all-target Clippy,
warning-denied rustdoc, and the full optimized benchmark command above.
AddressSanitizer completed 10,000 region-Boolean runs and 10,000
algebraic-image runs without a target failure; leak detection alone remained
disabled for the managed ptrace environment.

No prepared API or implementation carrier is removed by this change.

### Topology-only retained Boolean connectivity

The connectivity-first traversal still cloned exact endpoint coordinates for
materialized fragments. Those coordinates are necessary when a graph mixes
topology-keyed and unkeyed endpoints, because exact coordinates provide the
fallback join. Region Boolean output graphs are stronger: every start and end
already carries a topology vertex from the shared classified split topology.
When that graph-wide invariant holds, endpoint equality is decided entirely by
vertex identity.

Certified traversal now detects complete topology coverage once and constructs
endpoint records containing only the two topology vertices. Mixed or unkeyed
graphs keep the previous exact-coordinate connectivity path, and incomplete
successor evidence still rebuilds full first-, second-, and third-derivative
tangent evidence. A regression deliberately gives two joined topology
fragments different endpoint coordinates and proves that authoritative
topology connects them without observing either coordinate. Existing mixed-key
and tangent-rebuild regressions continue to cover both fallbacks.

The matched one-cell workload retained four exact decisions, nine candidate
pairs, 48 fragments, two point classifications, zero blockers, and checksum 6.
Its trace used `topology-connectivity-first` four times and never rebuilt
tangent order. Rational reductions and GCDs stayed at 54 and 156; temporaries
fell from 14,252 to 13,577, a 4.74% reduction.

| Inclusive instruction scope | Coordinate connectivity | Topology only | Change |
| --- | ---: | ---: | ---: |
| Whole one-cell workload | 22,284,706 | 21,550,160 | 734,546 fewer (3.30%) |
| `CurveRegion2::boolean_regions` | 12,989,614 | 12,256,949 | 732,665 fewer (5.64%) |
| Shared Boolean result construction | 2,565,842 | 1,832,951 | 732,891 fewer (28.56%) |
| One result-region build | 1,264,692 | 531,005 | 733,687 fewer (58.01%) |
| Retained arrangement traversal | 962,607 | 262,699 | 699,908 fewer (72.71%) |

Heaptrack recorded 28,215 versus 27,599 allocations, 616 fewer (2.18%).
Postprocessed temporary allocations stayed at 1,663. Peak heap fell from
881.22 to 873.91 KiB, while reported retained memory stayed at 155.82 KiB and
peak RSS including Heaptrack overhead rose from 11.14 to 11.26 MiB. The
same-compiler stripped pathological executable grew from 5,135,944 to
5,140,448 bytes, a recorded 4,504-byte (0.088%) tradeoff.

The unfiltered optimized `cargo bench --workspace --all-features` command again
executed every benchmark target and lane. The affected release API target
completed at 15.987 microseconds per immediate union and 96.865 microseconds
per immediate four-operation batch with exact checksums unchanged. The complete
67-cell pathological lane decided all 268 exact Booleans over 603 candidate
pairs and 3,248 fragments with zero blockers and checksum 6 in 433.937
milliseconds, versus 440.582 milliseconds at the preceding checkpoint. Every
competitor and 64-, 256-, and 1,024-vertex comparative lane ran; no
representative subset was substituted.

The workspace call-graph utility regenerated all source, test, benchmark,
example, and fuzz targets across the five hyper crates: 42,010 nodes and 71,225
edges. Public SCC-condensed counts/depths stayed 68/depth 4 for
`CurveRegion2::boolean_regions` and 75/depth 7 for result construction. The
private retained traversal grew from 230 to 236 reachable singleton SCCs while
remaining depth 11; every reachable SCC remains a single node.

The final source passed all-feature/all-target tests, the no-default-feature
library/test matrix, strict all-target Clippy, warning-denied rustdoc, and the
full optimized benchmark command. AddressSanitizer completed 10,000
region-Boolean and 10,000 algebraic-image runs without a target failure. Leak
detection alone remained disabled for the managed ptrace environment.

No prepared API or implementation carrier is removed by this change.

### Cross-stack prepared-removal gate

Prepared implementation surfaces below Hypercurve are removed only after two
ordered gates. The first gate covers every immediately affected public API:
exact-result and failure-boundary regressions, executed release benchmarks,
Callgrind instructions, Heaptrack allocations, loadable and file size, and
static callgraph depth. A failure at this tier is reverted before broader
validation. A passing candidate must then run correctness and the complete
executed benchmark suite, including every benchmark target, for all workspace
hyper crates:
`hyperbrep`, `hypercircuit`, `hypercurve`, `hyperdrc`, `hyperevolution`,
`hypergraphics`, `hyperlattice`, `hyperlimit`, `hypermesh`, `hyperpack`,
`hyperparts`, `hyperpath`, `hyperphysics`, `hyperreal`, `hypersdf`,
`hypersolve`, `hypertri`, and `hypervoxel`. A representative subset cannot
substitute for this all-benchmark gate. Relevant sanitizer fuzzers remain the
final cross-stack correctness tier.

The second tier is executable through
`scripts/prepared-removal-all-hyper-gate.sh`. It refuses to start without a
nonempty first-tier evidence report, records the exact source state of all 18
repositories, runs all-feature/all-target correctness tests, and executes one
complete all-feature workspace benchmark suite per crate, including every
benchmark target. Its logs are review evidence:
successful benchmark processes do not by themselves establish that timings
passed, so every candidate-versus-baseline result must still be accepted
explicitly before removal.

The attempted immediate replacement for Hypersolve's shared-denominator
rational-image carrier stopped at the first gate. Although denominator and
source-polynomial reuse and x-before-y short circuiting were preserved, the
best pair-return form raised the complete algebraic-parameter Callgrind count
from 9,570,229,121 to 9,586,812,181 instructions (0.17%); moving the two large
evidence reports accounted for 7,880,394 `memcpy` instructions. An output-slot
variant reached 9,598,735,012 instructions and added 4,092 loadable bytes.
Both were reverted. A separate local experiment replacing the conic chart's
small absence vector saved 18 allocations but was instruction-neutral and
added 712 text bytes plus one 4 KiB loadable page, so it was also reverted.
These rejected results prevent API simplification from silently weakening the
current performance envelope.

### Closed curve-operation policy

`CurvePolicy` is now the only public numeric-policy type. Its private state
admits two valid constructions: certified topology, or an explicitly
toleranced edge preview. The redundant `ExactSymbolic` label, separately
public `NumericMode` and `Tolerance` types, mutable predicate policy, and public
fields are removed. Exact decisions retain the strict Hyperlimit predicate
policy; the old exact-symbolic cache bypasses were removed because those
caches contain lossless exact evidence.

The release containment and intersection gates were run serially before and
after the change. Because repeated sub-microsecond wall-clock passes moved by
more than the candidate/control deltas, the committed pre-change tree was also
built in a local sibling clone and interleaved with the candidate. Wins and
losses reversed between adjacent passes, and every candidate timing band
overlapped its unchanged control band. The final private representation and
crate-internal reads preserve the pre-change field order, discriminants, and
strict predicate-policy storage, so the public simplification does not insert
an accessor boundary or alter a topology algorithm.

Representative final candidate and immediately adjacent unchanged-control
passes were:

| Containment case | Candidate | Unchanged control |
| --- | ---: | ---: |
| Contour bounding-box miss | 439 ns | 420 ns |
| 64-point batched bounding-box miss | 6.262 us | 6.329 us |
| Sparse-region outside | 32.440 us | 33.051 us |
| Sparse-region single hit | 54.266 us | 52.808 us |
| 64-point batched sparse region | 831.960 us | 821.772 us |
| Sparse-region filled area | 14.166 us | 14.192 us |

| Intersection case | Candidate | Unchanged control |
| --- | ---: | ---: |
| Line/circle secant relation | 1.936 us | 1.974 us |
| Circle/circle secant relation | 568 ns | 512 ns |
| Same-circle arc overlap | 6.710 us | 6.649 us |
| Same-circle endpoint pair | 4.334 us | 4.346 us |
| Two-point arc crossing | 1.849 us | 1.780 us |
| Sparse 160-segment self contacts | 120.873 us | 124.567 us |
| Sparse 160-segment curve intersections | 21.732 us | 21.582 us |
| Sparse 120-contour region events | 2.475 us | 2.426 us |

Earlier candidate passes reached 421 ns for the contour miss, 31.424 us for
sparse outside classification, 52.346 us for the single-hit case, 121.277 us
for sparse self contacts, and 1.872 us for the line/circle relation. The
unchanged controls varied comparably, including 420--445 ns for the contour
miss, 13.409--14.435 us for filled area, 1.974--2.086 us for the line/circle
relation, and 6.512--6.798 us for same-circle overlap. There is no repeatable
performance regression within the observed machine-noise envelope.

Correctness validation covered the all-feature library and integration suite,
the no-default-feature library and integration suite, the UI example, and all
in-workspace direct consumers (`csgrs`, `hyperbrep`, `hypercircuit`,
`hyperdrc`, `hyperpack`, and `synaps-cad`).

### Retain immediate line-orientation evidence

Hyperlimit retired `PreparedLine2` in favor of explicit endpoints, owned
`Line2Orientation` evidence, and immediate classification functions.
Hypercurve's retained line and arc query structures now store that evidence
directly. This preserves certified dyadic and exact-word filters across
repeated containment queries; the previous facts-only cache reconstructed
both filters for every classified point.

Two serialized release runs before and after the migration showed no
regression. The affected single-hit and batched sparse-region paths improved
slightly, while unrelated controls remained within their observed run-to-run
variation:

| Containment case | Before range | After range |
| --- | ---: | ---: |
| Contour bounding-box miss | 442--451 ns | 430--440 ns |
| 64-point batched bounding-box miss | 6.205--6.471 us | 6.392--6.712 us |
| Sparse-region outside | 32.521--32.948 us | 32.471--32.623 us |
| Sparse-region single hit | 52.099--56.555 us | 53.415--54.387 us |
| 64-point batched sparse region | 830.291--845.751 us | 825.186--830.554 us |
| Sparse-region filled area | 13.538--14.050 us | 14.054--14.240 us |

### Immediate arrangement reports

The unordered line/arc arrangement API now returns an immediate
`RegionArrangement2` with a semantic `RegionArrangementReport2`. Public
`ExactCurveArrangement*Cache2`, bucket, reference, and fact carriers, along
with cache-returning accessors, are gone from the crate root and generated
documentation. The result and report expose output, blocker, count, and
provenance facts directly; private caches remain an implementation detail.
`CurveRegionArrangement2` follows the same `report()` vocabulary.

The first pre-change serialized editing run measured 21.046 us for the line
arrangement, 21.473 us for the native line/arc arrangement, and 3 ns for report
replay. Candidate wall-clock samples varied enough to make a comparison
against that one run ambiguous, so the committed pre-change tree was built in
an isolated sibling checkout. Five pre-change runs and then five candidate
runs used their already-built release executables serially in the same
session. Medians were:

| Editing case | Pre-change | Immediate report | Change |
| --- | ---: | ---: | ---: |
| Boundary-contour control | 15.446 us | 15.257 us | -1.2% |
| Unordered line arrangement | 21.697 us | 21.428 us | -1.2% |
| Unordered native line/arc arrangement | 22.015 us | 22.385 us | +1.7% |
| Arrangement report replay | 274.292 us / 100k | 279.281 us / 100k | +1.8% |

Checksums and observed fact totals were unchanged. The immediate surface
therefore introduces no repeatable performance regression: both affected
increases remain below 2%, and every five-run candidate range overlaps its
pre-change range.

Correctness validation covered all all-feature Hypercurve tests, including
the retired curved-region Boolean corpus and pathological workload, strict
all-target Clippy, warning-free rustdoc, the focused immediate arrangement
tests, and compilation of every fuzz target.

### Single immediate arrangement result

`RegionArrangementReport2` and the `report()` / `into_report()` transitions
have now been retired. `RegionArrangement2` owns its shared immutable facts,
summary, and optional region directly, so callers inspect the completed
operation without entering a second report lifecycle. `CurveRegionArrangement2`
uses the same ownership model after immediately promoting any native output and
exposes its summary, status, blocker, and source count directly. Three duplicate
source-cache fields that became unreachable with the report wrapper were also
removed.

Three serialized editing runs of the committed pre-change tree in an isolated
checkout and three candidate runs produced the following ranges and medians:

| Editing case | Pre-change range (median) | Immediate-only range (median) | Median change |
| --- | ---: | ---: | ---: |
| Boundary-contour control | 15.252--15.669 us (15.414 us) | 15.528--15.673 us (15.632 us) | +1.4% |
| Unordered line arrangement | 22.265--22.596 us (22.534 us) | 21.901--22.928 us (22.620 us) | +0.4% |
| Unordered native line/arc arrangement | 22.691--22.958 us (22.830 us) | 22.613--23.128 us (22.994 us) | +0.7% |
| Immediate evidence replay | 274.141--282.661 us / 100k (277.530 us) | 129.212--130.491 us / 100k (129.861 us) | -53.2% |

All construction ranges overlap, and their sub-percent affected median changes
are smaller than the control movement. The replay case intentionally measures
the new direct borrow instead of cloning the retired report wrapper; checksums
remain identical.

### Completed overlap-split terminology

Linear and rational overlap-refinement results now expose
`overlap_splits()` instead of `split_plan()`. The refinement call has already
derived, applied, resolved, and validated every returned split, so the new
name describes completed immutable evidence rather than implying a deferred
operation.

The serialized 64-curve full-overlap workflow measured 636.574 us/iteration
before the rename. Two post-change runs measured 647.987 and
590.214 us/iteration with the same 25,800 checksum. The post-change range
straddles the baseline and the confirmation is 7.3% faster, so this
terminology-only API change introduces no measurable regression.

### General-rational cache-state boundary

`RationalBezier2` now exposes exact evaluation, derivative, containment,
intersection, and elevation results without exposing whether its private
homogeneous control or power-basis cells are populated. Clone-sharing and lazy
retention are unchanged; tests compare the repeated exact results instead of
observing representation state.

The full serialized rational-Bezier benchmark showed stable or improved core
paths. Because its default 64-control sentinel runs only ten iterations, the
committed pre-change tree and candidate were also built in isolated sibling
directories and run alternately for 1,000 iterations. Pre-change/candidate
times were 484.376/477.364 us for cold evaluation, 8.115/8.009 us for retained
evaluation, and 628.701/619.791 us for exact splitting. Checksums were
unchanged, and each candidate result is about 1.3--1.4% faster.

### Algebraic-parameter cache-state boundary

`BezierAlgebraicParameter2` now exposes represented rational roots directly,
without exposing whether the clone-shared reconstruction cell is populated.
The immediate accessor keeps a small inline retained-result path and delegates
first-use reconstruction to a private helper. Its focused replay sentinel was
lengthened from 5,000 to 500,000 calls so sub-microsecond comparisons are
meaningful.

Three serialized runs of isolated pre-change and final binaries showed no
regression. Replaying 500,000 represented roots took 17.248--18.773 ms before
and 16.966--17.371 ms after. Refined ordering overlapped at
11.768--12.326 us versus 11.975--12.178 us, while polynomial point/tangent
images improved from 2.559--2.772 us to 2.518--2.634 us. Every checksum was
unchanged.

### Rational-operation cache-state boundary

Completed rational-Bezier topology and degree-elevation carriers now expose
their arrangement and elevated curves directly, without public
`is_*_cached` probes. Repeated views, owned arrangements, elevated curves, and
retained failure results remain clone-shared and are tested by value.

Serialized isolated pre-change/candidate binaries measured the immediate
topology workflow at 99.628/95.708 us, cold degree elevation at
25.614/25.340 us per curve, and 100,000 retained elevations at
652.916/513.835 us. The full rational benchmark kept every checksum unchanged;
the two additional candidate runs also placed topology and cold elevation
within their observed pre-change variation.

### Intersection-topology cache-state boundary

Intersection topology, path Boolean selections, and retained overlap graphs
now expose their immediate arrangement, traversal, region, and overlap
evidence results without public cache-state probes. Clone-sharing remains
private; tests verify repeated borrowed-result identity or exact value
equality.

Serialized curve-path runs measured immediate topology at 21.563 us before
and 20.638--21.174 us after. The four retained line, arc, nonlinear, and
circle-region replays remained 2--6 ns. The Boolean batch baseline of
115.729 us was bracketed by 112.772 and 123.095 us candidate runs. The
64-curve full-overlap workflow similarly moved from 600.337 us to a candidate
range of 587.604--614.277 us with the same 25,800 checksum.

### NURBS editing cache-state boundary

NURBS knot insertion, exact knot removal, span elevation, and
continuity-preserving carrier elevation now return their immediate results
without public cache-state probes. Equal requests, failures, and blockers
remain privately retained across clones; tests verify repeated exact results
and shared retained data directly.

Serialized `bspline` runs measured cold batch insertion at 5.268 us before and
4.775--5.008 us after, exact knot removal at 7.040 us before and
6.331--6.566 us after, and exact span elevation at 37.907 us before and
36.633--37.500 us after. Continuity-preserving elevation measured 79.279 us
before and 78.746 us in the confirming candidate run. Retained operations
remained 3--34 ns.

### Spline representation cache-state boundary

Polynomial splines and NURBS now expose their immediate Bezier decomposition
and exact evaluation results without public decomposition or rational-span
cache-state probes. Decompositions remain clone-shared borrowed values, and
tests verify identity or repeated exact evaluation directly.

The serialized `bspline` baseline/candidate measured polynomial retained
decomposition at 2/2 ns, general rational evaluation at 962/936 ns, its first
derivative at 2.950/2.866 us, and derivatives one through three at
10.648/9.997 us. The 256-control NURBS cold decomposition measured
1.912/1.907 ms and retained evaluation measured 9.840/9.790 us.

### Curve and path result cache-state boundary

Top-level curves and paths now expose bounds, native fragments, closed Bezier
boundaries, and exact derivatives directly without public cache-state probes.
Borrowed result lifetimes and private clone sharing are unchanged; tests
verify repeated exact derivatives instead of cache transitions.

Serialized `arc` baseline/candidate runs measured retained native promotion at
2/2 ns, top-level evaluation at 7.398/7.380 us, 256-arc cold native promotion
at 1.506/1.401 ms, and its retained replay at 8/7 ns. Serialized `curve_path`
runs kept retained path promotion at 2/2 ns and immediate topology at
21.073/20.836 us.

### Curve-region result cache-state boundary

Curve regions now expose classification, native-contour eligibility, boundary
roles, exact rational evaluation, and signed area through their result APIs
without public cache-state probes. Clone-shared retention remains private;
tests verify repeated classification and area results across clones.

Serialized `bezier_region` runs measured algebraic ray classification at
76.196 us before and 75.368--77.510 us after, clone-retained curve-region
classification at 35.781 us before and 35.932--37.578 us after, native signed
depth at 1.197 us before and 1.192--1.334 us after, and retained algebraic
classification at 27.837 us before and 27.703--27.884 us after. The serialized
`editing` signed-area gate improved cold computation from 11.010 to 10.249 us
and 100,000 clone replays from 32 to 31 ns each.

### Immediate curve-region classification path

Curve-region classification no longer runs a separate eager preparation phase
that populated native boundaries, bounds, rational evaluators, and line-image
state together. Immediate native line/arc queries now retain only their result;
unsupported carriers fall through to the exact boundary representation, which
initializes only the data that representation consumes.

Against the immediately preceding serialized run, two candidate
`bezier_region` runs measured retained curve-region classification at
35.674--36.017 us versus 35.932 us, native signed depth at 1.185--1.196 us
versus 1.192 us, algebraic classification at 27.333--28.293 us versus
27.703 us, and algebraic line-role evidence at 58.883--60.406 us versus
59.425 us.

### Retained structural-fact collapse

Circular arcs now retain `CircularArc2Facts` lazily in their existing shared
geometry allocation. The one-word `CircularArc2` handle and 48-byte
`Segment2` ceiling are unchanged, and the fact packet allocates only after a
caller requests it. A permanent replay row improved from 470 ns to 3--5 ns per
query, about 117x.

Prepared line and arc query objects no longer duplicate
`LineSeg2Facts`/`CircularArc2Facts` solely for equality. Their Hyperlimit
orientation evidence already retains the fixed-input facts and filters used by
classification. Curve-string and exact-region evidence paths that only need a
segment family now call the constant-time `Segment2::kind()` discriminator
instead of scanning every scalar into a complete fact packet.

The serialized neighboring rows remained stable after the collapse: major arc
containment 9.35 us, top-level evaluation 7.19 us, inverse-witness replay
1.56 us, 256-arc sparse path intersection 1.11 ms, and 256-arc region
containment 0.992 ms.

Point and line fact caches were rejected because they would violate the
one-pointer point layout or consume the 48-byte line/segment budget. Inline
Bezier caches were also rejected because the workspace call graph found no
production repeated-fact consumer; general rational Bezier, spline, NURBS,
top-level curve/path, and curved-region owners already retain the expensive
derived evidence their algorithms use.

### One-byte curve context and explicit preview boundary

The predicate-only `CurveContext` replaces `CurvePolicy` throughout the
crate, tests, benchmarks, fuzz targets, examples, and standalone UI. There is
no compatibility alias. `CurveContext` is a one-byte transparent value whose
public states map exactly to Hyperlimit `STRICT` and `APPROXIMATE_512`.
Lossy display tolerances moved into validated `CurvePreviewOptions`, which
scopes preview-only evidence to one synchronous closure and cannot be used as
certified topology or construction provenance.

An initial implementation consulted thread-local preview state at every
predicate site. Repeated pathological runs exposed a roughly 3% strict-path
regression, so that design was rejected. The final representation carries a
private preview tag in the otherwise unused context bits and only enters the
cold, non-inlined thread-local tolerance lookup when the tag is present.

Nine final-source pre-change/candidate pairs were run interleaved, reversing
execution order on alternate pairs. The complete mixed-family, mixed-`Real`
100 MiB Boolean workload produced identical topology in every run: 67
completed cells, 603 candidate carrier pairs, 3,248 fragments, 134 point
classifications, 268 decided operations, no blockers, and checksum 6.

| Pathological all-operation Boolean | Pre-change | One-byte context | Change |
| --- | ---: | ---: | ---: |
| Range | 470.718--493.192 ms | 469.766--494.102 ms | overlapping |
| Median | 480.842 ms | 481.742 ms | +0.2% |

The standalone comparative benchmark also showed no repeatable small-rectangle
regression: 32,768-iteration interleaved controls measured 4,910.7 and
4,865.5 ns, while candidates measured 4,875.5 and 4,693.3 ns. The release
pathological executable grew from 5,523,105 to 5,535,425 bytes (+12,320,
0.22%) relative to the immediately preceding checkpoint, and by 4,132 bytes
(0.075%) relative to the frozen consolidation baseline. This temporary Phase
1 size debt is recorded for repayment when the superseded policy and legacy
engines are deleted.

Validation covered the complete all-feature suite (including the 170-second
exact CurveRegion2 fuzz corpus and all 268 pathological operations), all-target
Clippy with warnings denied, no-default-feature compilation, every fuzz
target, the standalone UI, formatting, and diff whitespace checks.

### Compact `CurveRegion2` shared carrier

`CurveRegion2` is now a one-word shared handle. Immutable boundary and loop
semantics plus their lazy exact caches live in one `Arc` allocation instead of
copying the boundary vector while independently sharing six cache allocations.
All empty values reuse one process-wide data allocation. Clones therefore
share geometry and computed exact area, consuming a uniquely owned value still
moves out its loop vector, and consuming a shared clone returns independent
owned loops without affecting the source.

The handle shrank from 112 to 8 bytes. The boxed four-region payload inside
`CurveRegionBooleanResults2` consequently shrank from 448 to 32 bytes while
the result wrapper remains 40 bytes. A permanent carrier-only mode in the
`bezier_region` benchmark times one million nonempty clones and one million
empty constructions. Fifteen parent/candidate process pairs were interleaved
with reversed order:

| Carrier operation | Parent median | Shared-carrier median | Change |
| --- | ---: | ---: | ---: |
| 1,000,000 nonempty clones | 314.899 ms | 3.728 ms | -98.8%; 84.5x |
| 1,000,000 empty constructions | 119.232 ms | 3.128 ms | -97.4%; 38.1x |

An isolated Heaptrack run recorded 12,000,166 allocation calls for the parent
and 151 process/fixture allocations for the candidate. The candidate's two
million timed carrier operations added no heap allocations; the change
eliminated 12,000,015 calls from this workload.

Nine interleaved pathological runs retained exactly 67 cells, 603 candidate
pairs, 3,248 fragments, 134 point classifications, 268 decided operations, no
blockers, and checksum 6. The all-four Boolean median moved from 470.503 to
473.041 ms (+0.54%) with overlapping ranges. Eleven interleaved construction
runs moved from 79.137 to 76.650 ms (-3.1%) with overlapping ranges. Full-run
Heaptrack allocation calls fell from 8,250,415 to 8,248,405 and peak heap from
36.22 to 36.20 MiB.

The release pathological executable fell 4,080 bytes, from 5,535,425 to
5,531,345 bytes. It is now only 52 bytes larger than the frozen consolidation
baseline, repaying essentially all native size debt from the one-byte context
checkpoint.

Validation passed the complete all-feature suite, including the 173.86-second
exact Boolean corpus and all 268 pathological operations; all-feature
all-target and no-default library/test Clippy with warnings denied;
no-default all-target compilation; every fuzz-target build; and all 37
standalone UI tests, including its 333.28-second shared Boolean scene. The
warning-denied all-feature rustdoc and standalone release WASM build also pass.
The no-default test runner still exposes predicate-dependent rational-moment
and arc assertions that also fail at the paired parent revision; this
checkpoint does not hide or claim to repair that pre-existing feature-contract
defect.
Machine-readable samples and provenance are in
[`2026-07-31-compact-curve-region-carrier.json`](benchmarks/checkpoints/2026-07-31-compact-curve-region-carrier.json).

### Policy-explicit `CurveRegion2` construction

Boundary-path promotion, native/retained loop validation, retained arrangement
materialization, and signed-area/nesting evidence now receive the caller's
`CurveContext`. The authoritative `CurveRegion2` boundary-path constructor no
longer enters the policy-free cached Bezier boundary path. A symbolic closure
whose endpoints differ structurally as `pi + e` and `e + pi` is therefore a
typed `Construction/RealSign` blocker under `STRICT`; `APPROXIMATE_512`
decides the same terminal equality and the operation observation records
`Approximate512Consumed`. No compatibility overload preserves the old
policy-free surface.

The complete all-feature suite passed with 251 unit tests, the 167.01-second
exact CurveRegion2 Boolean corpus, and all 268 pathological operations. The
all-feature/no-default warning-denied matrices, fuzz targets, rustdoc, and the
affected standalone UI paths also pass.

Five release pathological processes retained exactly 67 cells, 603 candidate
pairs, 3,248 fragments, 134 point classifications, 268 decided operations, no
blockers, and checksum 6. The all-four Boolean median was 470.731 ms versus
473.041 ms at the immediately preceding checkpoint (-0.49%, overlapping
ranges). The release executable fell from 5,531,345 to 5,523,149 bytes
(-8,196 bytes, -0.15%).

Two production `STRICT` references remain in `bezier_region.rs`: both are
conservative rational-line symbolic-area capability probes. They do not admit
topology or construct `CurveRegion2`; they belong to the forthcoming
policy-explicit measurement/evaluation API cutover.

Machine-readable samples and provenance are in
[`2026-07-31-policy-explicit-curve-region-construction.json`](benchmarks/checkpoints/2026-07-31-policy-explicit-curve-region-construction.json).

### Explicit construction outcomes

Every predicate-bearing public `CurveRegion2` constructor and arrangement
entry point now returns `CurveOutcome<T>` across `d7bc1ab` and the retained
traversal completion `cd11bb4`. Construction therefore cannot silently discard
an Approximate-512 terminal decision. Internal composition uses private raw
builders under one outer observation frame, so the geometry is built once and
certainty is aggregated once. The same contract applies when exact line-role
evidence is promoted back into the authoritative region. No dereference or
implicit-value compatibility shim was added.

The complete all-feature suite passed with 251 unit tests, the 173.92-second
exact CurveRegion2 Boolean corpus, and all 268 pathological operations. Both
warning-denied feature matrices, fuzz targets, rustdoc, and all 37 standalone
UI tests also pass.

Seven uncontended release processes retained exactly 67 cells, 603 candidate
pairs, 3,248 fragments, 134 point classifications, 268 decided operations, no
blockers, and checksum 6. The all-four Boolean median was 475.984 ms, 1.12%
above the preceding 470.731 ms checkpoint and 0.96% above the frozen
471.450 ms baseline, with overlapping ranges and one retained scheduler
outlier. The separately timed construction median was 74.537 ms, 2.76% below
the compact-carrier checkpoint. The release executable fell another 16,380
bytes to 5,506,769 bytes, 0.44% below the frozen baseline.

The seven-sample competitive lane remains stable against the frozen
Hypercurve medians (all movements within 5.2%). Hypercurve measured 4.765 us
for a tiny rectangle union, 36.380 us at 64-star intersection, 425.674 us at
256 vertices, and 5.170 ms at 1,024 vertices. The finite engines retain their
small-input advantage; Hypercurve is 1.50--1.61 times faster than iOverlay and
Cavalier Contours at 256 vertices, and 1.94--3.80 times faster at 1,024
vertices. The exact certified cubic-offset and general NURBS lanes remain much
slower than heuristic/finite competitors and are explicit optimization
targets, not like-for-like exactness comparisons.

Machine-readable samples, competitive medians, validation, binary size, and
call-graph evidence are in
[`2026-07-31-curve-region-construction-outcomes.json`](benchmarks/checkpoints/2026-07-31-curve-region-construction-outcomes.json).

### Explicit query outcomes

Every predicate-bearing public `CurveRegion2` read/query operation now returns
`CurveOutcome<T>`: point classification and signed depth, filled area and
bounds, loop roles/profiles/filled sides, native-contour eligibility, and the
three retained role-evidence routes. Internal Boolean, projection, transform,
offset, and SVG composition call private raw kernels under their existing
outer observation frame, so no query is recomputed and no nested public
outcome is discarded. The one-word region carrier and its persistent storage
are unchanged.

A symbolic right boundary authored with `pi + e` and queried with `e + pi`
remains a certified outer outcome containing explicit uncertainty under
`STRICT`. `APPROXIMATE_512` identifies the same point as boundary and reports
`Approximate512Consumed`; the returned point and region geometry remain exact.
No implicit value or dereference compatibility shim was added.

Seven parent/candidate `bezier_region` process pairs were interleaved with
execution order reversed on alternate pairs. The timing ranges overlap:

| Public query | Parent median | Outcome median | Change |
| --- | ---: | ---: | ---: |
| Retained point classification | 34.457 us | 33.896 us | -1.63% |
| Native signed depth | 0.846 us | 0.821 us | -2.96% |
| Retained algebraic classification | 26.104 us | 25.696 us | -1.56% |
| Algebraic line-role evidence | 56.436 us | 56.537 us | +0.18% |

Seven pathological processes retained 67 cells, 603 candidate pairs, 3,248
fragments, 134 point classifications, all 268 decided operations, zero
blockers, and checksum 6. Their median was 470.508 ms, 1.15% below the
preceding checkpoint and 0.20% below the frozen baseline. The build median was
74.008 ms and median observed RSS delta was 34.6 MiB. The stripped release
executable grew 1,591 bytes to 5,508,360 bytes, remaining 22,933 bytes below
the frozen consolidation baseline.

The seven-sample competitive lane remained stable: the largest movement from
the frozen Hypercurve medians was a 7.0% improvement in the source-chord cubic
fallback, while every other exact Hypercurve lane remained within 4.0%.
Hypercurve retains its larger-input polygon advantage and its explicit
exactness/performance gap on certified general offsets and NURBS evaluation.

Validation passed the complete all-feature suite, including the 161.61-second
exact CurveRegion2 corpus and all pathological operations; warning-denied
all-target and no-default matrices; fuzz builds; rustdoc; the release WASM UI;
and all 37 standalone UI tests in 323.96 seconds.

Machine-readable samples and call-graph evidence are in
[`2026-07-31-curve-region-query-outcomes.json`](benchmarks/checkpoints/2026-07-31-curve-region-query-outcomes.json).

## Optimization boundary

The retained x sweep addresses broad-phase pair scheduling only. A full
Bentley--Ottmann status ordered by exact curve y-at-x, or the Martinez/Vatti
overlay ownership machinery, remains architecture-inapplicable to the current
mixed line/arc pair API unless it can preserve degeneracy evidence, overlaps,
and authored provenance. The new sparse and dense sentinels are the crossover
evidence for the portion that can be adopted independently.
