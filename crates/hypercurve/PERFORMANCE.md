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
| Polynomial/rational Bezier algebra, splitting, arrangement, and retained evidence | the `hypercurve_bezier_*` tests and `hypercurve_rational_bezier` | the `bezier_*` benches and `rational_bezier` | quadratic evaluation and region Boolean |
| B-spline, polynomial spline, and NURBS construction/evaluation | `hypercurve_bspline`, `hypercurve_polynomial_spline`, `hypercurve_nurbs`, `hypercurve_nurbs_interpolation` | `bspline`, `rational_bezier`, `api_surface` | global NURBS interpolation |
| Editing, offsets, fitting, and reconstruction | `hypercurve_contour`, `hypercurve_offset`, `hypercurve_bezier_fit_offset`, `hypercurve_reconstruct` | `editing`, `offset`, `reconstruction` | checked curve-string offset |
| Contours, regions, Boolean topology, and prepared queries | `hypercurve_boolean`, `hypercurve_region*`, `hypercurve_curve_region_boolean` | `containment`, `bezier_region` | region Boolean and prepared containment |
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
| Farouki and Neff, plane offsets | The curvature/evolute analysis explains cusp and singular-offset hazards. `bezier_offset` detects cusp, inflection, and denominator risks and only constructs a proven line-image offset; unsupported free-form offsets remain explicit unresolved candidates rather than unchecked approximations. |
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
| Vatti, generic polygon clipping | Scanbeam clipping demonstrates a general event/ownership formulation that handles holes and complex polygons. Hypercurve's region pipeline keeps those roles explicit, and its retained x scheduler supplies the compatible broad-phase benefit. A second polygon-only scanbeam carrier would duplicate rather than optimize the prepared curved-arrangement representation. |
| Yap, exact geometric computation | The exact-object discipline is the crate-wide rule: structural filters may accelerate a decision, but a topology branch needs certified evidence. Homogeneous carriers, algebraic parameter intervals, retained blockers, and report-bearing prepared objects preserve the information needed for replay. |

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
exact `Region2` comparison proves retained point order and coordinates.
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

## Optimization boundary

The retained x sweep addresses broad-phase pair scheduling only. A full
Bentley--Ottmann status ordered by exact curve y-at-x, or the Martinez/Vatti
overlay ownership machinery, remains architecture-inapplicable to the current
mixed line/arc pair API unless it can preserve degeneracy reports, overlaps,
and authored provenance. The new sparse and dense sentinels are the crossover
evidence for the portion that can be adopted independently.
