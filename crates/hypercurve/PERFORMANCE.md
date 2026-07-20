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

## Optimization boundary

The retained x sweep addresses broad-phase pair scheduling only. A full
Bentley--Ottmann status ordered by exact curve y-at-x, or the Martinez/Vatti
overlay ownership machinery, remains architecture-inapplicable to the current
mixed line/arc pair API unless it can preserve degeneracy reports, overlaps,
and authored provenance. The new sparse and dense sentinels are the crossover
evidence for the portion that can be adopted independently.
