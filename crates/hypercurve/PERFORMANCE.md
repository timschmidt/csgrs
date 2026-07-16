# Performance and Reference Audit

This document records how every source in the README reference list maps to
`hypercurve`, which ideas are already embodied by the implementation, and which
optimization experiments were retained or rejected. The governing constraint is
that a speedup may not weaken exact topology, erase retained evidence, or move a
finite approximation across a predicate boundary.

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

## Optimization boundary

The retained x sweep addresses broad-phase pair scheduling only. A full
Bentley--Ottmann status ordered by exact curve y-at-x, or the Martinez/Vatti
overlay ownership machinery, remains architecture-inapplicable to the current
mixed line/arc pair API unless it can preserve degeneracy reports, overlaps,
and authored provenance. The new sparse and dense sentinels are the crossover
evidence for the portion that can be adopted independently.
