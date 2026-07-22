# Performance and Reference Audit

Cross-crate measurements live in
[`COMPARATIVE_BENCHMARKS.md`](COMPARATIVE_BENCHMARKS.md). They are kept separate from
this exact-path audit because peer crates use different numeric and topology contracts.

This document records how every source in the README reference list maps to
`hypercurve`, which ideas are already embodied by the implementation, and which
optimization experiments were retained or rejected. The governing constraint is
that a speedup may not weaken exact topology, erase retained evidence, or move a
finite approximation across a predicate boundary.

## Runtime path tracing

Coverage is audited by executable public family, not by assigning artificial
timings to every enum variant, report accessor, or zero-cost data carrier. A
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
| Contours, regions, Boolean topology, and prepared queries | `hypercurve_boolean`, `hypercurve_region*`, `hypercurve_curve_region_boolean` | `containment`, `bezier_region` | region Boolean and prepared containment |
| Pathological retained-region memory, transforms, intersections, and all Boolean operations | benchmark fixture smoke paths plus the ordinary family/Boolean suites | `pathological_regions`; feature-gated pathological lanes in `comparative` | every curve family and `Real` representation class across calibrated 100 MiB, 500 MiB, and 1 GiB native inputs |
| Finite projection, retained import, triangulation, and SVG boundary | `hypercurve_region`, `hypercurve_triangulation`, `hypercurve_svg_io` | `api_surface`, `svg_io` | not applicable to the finite-only adapter work; exact reconstruction/topology is traced by the rows above |

The `dispatch-trace` feature enables the shared `hyperreal`/`hyperlimit`
exact-computation trace recorder. The `dispatch_trace` benchmark exercises
public line and arc intersections, polynomial Bezier evaluation, curve-string
offsetting, exact similarity transforms, global NURBS interpolation, region
Boolean construction, and prepared region containment. Each workload is
isolated in its own recording window and fails if it produces no dispatch or
rational-reducer evidence.

```bash
cargo test --features dispatch-trace --test hypercurve_dispatch_trace
cargo bench --features dispatch-trace --bench dispatch_trace
```

The integration test protects the trace contract itself; the benchmark prints
the per-operation summaries and cross-stack correlation counters used to relate
performance observations to exact predicate, structural-fact, reducer, cache,
refinement, and approximation paths.

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
| Bentley and Ottmann, intersection reporting | Retained as an adaptive one-axis event sweep for large curve-string pair batches. It is deliberately a conservative AABB candidate scheduler, not a full Bentley--Ottmann intersection-status implementation: exact line/arc predicates and source ordering remain unchanged. The crossover and dense fallback are measured below. |
| de Casteljau, affine Bézier evaluation | Directly underlies polynomial Bézier evaluation, exact splitting, flattening, metric prefixes, and moment prefixes. Reusing common affine weights throughout subdivision triangles produced the retained optimization measured below; evaluation preserves that expression graph for non-rational parameters. |
| de Berg et al., *Computational Geometry* | Plane-sweep, arrangement, point-location, and robust subdivision principles match the crate's broad-phase filtering and explicit topology stages. The retained conservative x sweep applies the scheduling portion while leaving exact intersection predicates and ownership unchanged. |
| Boehm, knot insertion | `bspline` performs exact homogeneous Boehm insertion and retains the resulting Bézier spans and source provenance. This is already the appropriate local transformation; no lossy span approximation was introduced. |
| de Boor, splines | Local B-spline evaluation and knot-domain rules are reflected in exact evaluation, sided behavior at discontinuous knots, refinement, and extraction. Cached decomposition and native topology already avoid repeatedly rebuilding that work. |
| Farouki and Neff, plane offsets | The curvature/evolute analysis supplies the exact distance-dependent cusp equation. `bezier_offset` retains the analytic parallel, isolates source and offset cusps, materializes line-image and Pythagorean-hodograph offsets exactly, and keeps all other products behind conservative certification. |
| Farouki and Rajan, Bernstein-form algorithms | Bernstein arithmetic, sign variation, subdivision, substitution, and elimination support the rational-Bézier sign tests, monotonicity certificates, resultants, and root isolation. It also reinforces retaining Bernstein/de Casteljau form instead of eagerly converting every operation to expanded power basis. |
| Farin, CAGD | Bézier/B-spline evaluation, subdivision, rational homogeneous form, derivatives, and variation-diminishing bounds are pervasive throughout the curve carriers. The retained shared-weight change preserves these exact affine identities. |
| Foster, Hormann, and Popa, degenerate polygon clipping | The key lesson is to classify and label degenerate intersections explicitly instead of perturbing them. Curve arrangements retain contact multiplicity, tangent/crossing status, overlap ranges, vertex identities, and operation-aware ownership before traversal. |
| Greiner and Hormann, arbitrary polygon clipping | Intersection insertion followed by entry/exit traversal is reflected in split/classify/traverse Boolean structure. Hypercurve extends the carrier and evidence model for curves and exact degeneracies rather than copying a floating-point polygon-only traversal. |
| Hobby, finite-precision segment output | Finite output can create or erase incidences, so certified flattening, SVG import/export, and reconstruction stay explicit boundaries with reports. Snap rounding is not silently applied inside exact topology. |
| Hormann and Agathos, point in polygon | Boundary classification precedes winding decisions, and contours expose both nonzero and even-odd fill rules. Conservative boxes and prepared views accelerate repeated classification without changing the winding result. |
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
| Vatti, generic polygon clipping | Scanbeam clipping demonstrates a general event/ownership formulation that handles holes and complex polygons. Hypercurve's region pipeline keeps those roles explicit, and its retained x scheduler supplies the compatible broad-phase benefit. A second polygon-only scanbeam carrier would duplicate rather than optimize the prepared curved-arrangement representation. |
| Yap, exact geometric computation | The exact-object discipline is the crate-wide rule: structural filters may accelerate a decision, but a topology branch needs certified evidence. Homogeneous carriers, algebraic parameter intervals, retained blockers, and report-bearing prepared objects preserve the information needed for replay. |

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
line arrangement. Its report limits the summed bound to the raw
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
evaluations. The table reports medians of three same-machine runs.

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
| 64 x 66 segments, prepared | 911.943 us | 72.506 us | 92.0% faster |
| 128 x 130 segments, direct | 3.841 ms | 212.683 us | 94.5% faster |
| 128 x 130 segments, prepared | 3.797 ms | 142.745 us | 96.2% faster |
| 512 x 514 segments, direct | 69.282 ms | 1.001 ms | 98.6% faster |
| 512 x 514 segments, prepared | 68.029 ms | 632.556 us | 99.1% faster |

The adversarial x-dense/global-overlap sentinel selected the flat scan. Its
64-, 128-, and 512-segment medians stayed within 1.3% of baseline; the largest
movement was a 1.1% slowdown and is below the retention threshold. A 64-by-65
equal-x endpoint-contact regression exercises the active sweep and proves that
direct and prepared queries retain the same event and source indices.

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

The report-bearing direct region Boolean path already collected one exact
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
classification, uncertainty propagation, and public role reports are unchanged.

In the same 15-sample, 50 ms comparative configuration, star64 intersection
fell from 7.822 to 6.379 ms/iter (18.4%) and rectangle union measured 105.207
us/iter, with unchanged checksums. The complete one-iteration Callgrind sweep
fell from 280,749,710 to 227,341,565 instructions (19.0%), removing the prior
53.4-million-instruction eager interpolation cost rather than replacing it with
an approximate sample. From the original checkpoint, star64 is now 82.8% faster
and the instruction sweep is 83.9% smaller. The exact Boolean differential fuzz
target completed 1,000 AddressSanitizer-instrumented runs at 5,054 coverage
points and 14,451 feature edges without a failure.

Structural report construction and prepared predicates then recomputed the same
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
compares the new selection and report with the canonical classifier and confirms
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
ineligible event still falls through unchanged. Finally, fragment reports recover
the primitive family directly from the `Segment2` variant instead of recomputing
full structural facts for each source and materialized fragment. Splitting preserves
the primitive family, so kind counts and provenance reports remain identical.

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
fragment reports no longer request unrelated structural facts merely to name a
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

The report-bearing region Boolean path also kept borrowed intermediate geometry
alive through final materialization, cloned closed chains into loops, cloned
loops into contours, and cloned every completed stage report into the aggregate
report. Successful stages now consume chains and loops and move their reports;
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
audit trail used by their `*_with_report` counterparts and immediately dropped
it. Fragment emission and chain assembly now share one topology core with
separate lean and evidence-retaining materialization. The ordinary consuming
path moves chains into loops and loops into contours without allocating
per-fragment provenance, and ordinary boundary nesting assigns material/hole
roles without retaining per-contour sample copies. Report-bearing methods keep
the existing reports and blockers unchanged. The Boolean fuzz target now
differentially compares ordinary, report-bearing, and prepared results for all
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

Ordinary region splitting still called the report-bearing builder and cloned
every exact source endpoint, parameter range, and output fragment into
provenance that only `*_with_report` callers observe. Splitting now shares one
four-role contour traversal with optional report retention. Lean callers keep
only the fragment inventory; report-bearing calls retain the same successful
and partial-blocker evidence. The ordinary Boolean pipeline also consumes its
completed selection when no shared boundary needs resolution instead of
cloning it out of a report wrapper.

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
algebra. Exact dyadic coordinates use prepared certified `f64` determinant
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
already-built prepared point and winding classifiers. Report-bearing prepared
queries select the same shared pipeline with evidence retention enabled. This
removes a separate 266-line prepared orchestration copy whose convenience path
always paid report construction, and prevents the two implementations from
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
passed, including prepared/direct adversarial polygon parity and report
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
instead of being cloned at each representation boundary. Report-bearing and
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

Non-report strict-crossing Booleans now split into a private compact fragment
carrier that retains only source index, exact parameter range, and native
segment geometry. The public and report-bearing `ContourFragment` continues to
carry both source endpoints. Direct contour output moves compact segments into
the result, while loop output clones original source endpoints only for selected
fragments. Overlaps, endpoint contacts, curved inputs, incomplete winding
proofs, and report requests retain the general provenance-rich pipeline.

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

## Optimization boundary

The retained x sweep addresses broad-phase pair scheduling only. A full
Bentley--Ottmann status ordered by exact curve y-at-x, or the Martinez/Vatti
overlay ownership machinery, remains architecture-inapplicable to the current
mixed line/arc pair API unless it can preserve degeneracy reports, overlaps,
and authored provenance. The new sparse and dense sentinels are the crossover
evidence for the portion that can be adopted independently.
