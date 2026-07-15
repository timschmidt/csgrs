<h1>
  hypercurve
  <img src="./doc/hypercurve.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hypercurve` is the planar curved-topology crate for the Hyper geometry stack. It owns
line, circular-arc, polynomial Bezier, rational-conic, contour, region,
boolean-boundary, offset, fitting, flattening, and prepared-query surfaces over
`hyperreal::Real` values. Its geometry is strictly 2D; spatial curves and their
3D parameterizations belong to `hyperbrep`.

The crate provides exact mixed-family curve/path intersection and regularized region
Booleans for its supported topology. Unsupported implicit branch correspondence and
free-form offset trimming remain typed blockers instead of being hidden by display
polylines.

## Core API

`CurveRegion2` is the exact mixed-family region type. It accepts closed `CurvePath2`
boundaries containing any public curve family and preserves exact source, path, curve,
promoted-span, split-range, operand, and reversal provenance on every emitted fragment.
It classifies points directly against native polynomial/rational boundaries with
even-odd fill semantics, retaining native loop, decided AABB, and exact signed-area
facts across clones. Exact line-image algebraic carriers are lowered once to a
clone-shared native line region; nonlinear algebraic carriers filter exact source-curve
incidence to their represented parameter ranges. Only non-line carriers lacking retained
source-curve provenance remain explicit classification blockers.
`CurvePath2::boolean_region` computes a one-shot union, intersection, difference, or
XOR. `PreparedCurvePathIntersection2::boolean_region_view` caches and borrows each
operation/side-policy result when the same path pair is queried repeatedly.
`CurveRegion2::boolean_region` accepts those retained results directly, including
nonlinear algebraic endpoint carriers, nested holes, and prior Boolean output.
`CurveRegion2::try_prepare_boolean` returns a `PreparedCurveRegionBoolean2` that
computes each cross-region carrier intersection once and clone-shares the four lazy
regularized results. Region preparation retains certified loop-junction vertex
identity, so independently represented algebraic endpoint images are not reclassified
when a result feeds another operation.

`Region2` remains the native line/circular-arc region type. `RegionView2<'_>` and
prepared borrowed views serve repeated classification and Boolean queries without
copying contours. `Contour2` owns a closed line/arc boundary, while `CurveString2`
owns an open or closed ordered line/arc path.

`Curve2` is the immutable shared carrier for lines, circular arcs, quadratic and
cubic Beziers, arbitrary-degree rational Beziers, polynomial B-splines, and NURBS.
`CurveView2<'_>` and `CurvePathView2<'_>` provide borrowed traversal without
copying geometry. `CurveParameterDomain2` gives every family a uniform exact
public domain: native curves use `[0, 1]`, while splines retain `[U[p], U[n+1]]`.
`PolynomialSplineCurve2` and `NurbsCurve2` accept exact controls, weights, and
knots without a runtime policy, support clamped and non-clamped finite active
domains plus explicit periodic one-period carriers, retain optional `CurveSource2`
provenance, and share exact Bezier
decompositions across clones. Their borrowed span views preserve source identity,
source span index, and knot interval through topology promotion. Exact point and
derivative evaluation accepts explicit left/right knot-side selection; automatic
selection certifies that both sides agree. `ExactCurveError` reports the operation,
curve family, source version, and blocking predicate or invariant. Curves and
connected paths reverse or undergo exact planar similarity transforms without
changing their public parameter domains, families, or provenance. `Curve2::split_at`
and `Curve2::subcurve` dispatch exact trimming across every family while retaining
family and provenance; spline results also retain their selected authored domains.
`CurvePath2::chamfer_vertex_by_parameters` and
`CurvePath2::fillet_vertex_by_parameters` apply that exact trimming uniformly to
mixed-family vertices. They accept each curve's native `Real` parameter domain,
support the start/end seam of closed paths, preserve source family and provenance,
and certify fillet radius, tangency, and traversal direction before inserting a
native line or circular arc. Edited closed paths remain valid inputs to
`CurveRegion2::try_from_boundary_paths`.
Circular arcs promote exactly to one or more rational quadratic spans, including
minor, semicircular, major, and full-circle sweeps, and share that promotion
across top-level curve clones.

Pipeline reports, finite projection, reconstruction, and IO are secondary APIs.
`Region2::arrange_unordered_segments` and its line-specialized and borrowed
variants return a retained `RegionArrangement2`; arrangement machinery and
caches stay behind that domain result so callers do not manage workspaces or
repeat exact geometric calculations.

## WASM Demo

The deployed WASM app is available at <https://timschmidt.github.io/hypercurve/>.

## Typical Curve Problems

Curved planar geometry combines robust-predicate failures with representation failures:
tangent contacts, overlapping arcs, nearly coincident boundaries, Bezier roots, offset
self-intersections, and lossy chordization. Fixed epsilons can make one fixture pass and
the next fail; eager exact algebra can also expand before broad-phase filters eliminate
obvious misses.

`hypercurve` stages the work. Native curve objects keep exact control structure,
prepared views and boxes reduce candidate sets, low-degree exact cases are promoted
when available, and unresolved tangent, overlap, root-isolation, trimming, or topology
cases remain explicit uncertainty instead of being hidden behind display polylines.

## Main Types

- `CurveRegion2` is the exact mixed-family region type; `Region2` and `RegionView2`
  are the native line/arc owned and borrowed region types.
- `Curve2`, `CurveView2`, `CurvePath2`, and `CurvePathView2` are the primary
  mixed-family curve and connected-path types.
- `PreparedCurveIntersection2` dispatches top-level curve pairs through retained
  native spans, uses exact circle predicates before generic rational resultants,
  deduplicates spline-knot contacts, and reports exact source/span/parameter provenance
  plus unresolved pair evidence. Trimmed and reversed curves distinguish their current
  public parameter ranges from retained root-source ranges, and represented contacts
  expose both values. Complete reports lazily retain contact-derived span
  splits and arrangements.
- `PreparedCurvePathIntersection2` computes authored curve-pair facts once and lazily
  retains aggregate contacts, overlaps, split topology, regularized Boolean selection,
  arrangement traversal, and `CurveRegion2` output for each operation and declared
  boundary-interior side. `CurvePath2::boolean_region` is the direct one-shot entry.
- `PreparedCurveRegionBoolean2` performs the corresponding operation on retained
  `CurveRegion2` values, reuses exact carrier-pair reports across all four operations,
  and preserves algebraic fragment intervals and parent source provenance in chained
  results.
- `Contour2`, `CurveString2`, `Point2`, `LineSeg2`, `CircularArc2`, and `Segment2`
  provide boundary and primitive geometry. `CircularArc2::sweep_fraction` and
  `point_at_sweep_fraction` are exact inverse directed-angular operations for minor,
  major, semicircular, and full-circle arcs; `rational_bezier_decomposition` exposes
  the separate retained piecewise-conic parameterization. `CurveString2` parameter
  trims use affine line fractions and directed-angular arc fractions and retain the
  exact evaluated endpoint witnesses. `CurveString2::chamfer_vertex_by_parameters`
  and its point-bearing counterpart use those same conventions for line-line,
  line-arc, arc-line, and arc-arc vertices; source segments retain their native
  families and exact source-range reports around the inserted line bevel.
  `CurveString2::fillet_vertex_by_parameters` and its point-bearing counterpart
  likewise certify exact source/fillet tangency for every native line/arc pairing,
  preserve trimmed source families, and retain inverse arc-parameter witnesses so
  parameter-driven edits replay the original exact points across clones.
  The corresponding `CurvePath2` parameter APIs cover line, circular-arc,
  quadratic/cubic Bezier, rational quadratic/general Bezier, polynomial B-spline,
  and NURBS pairings, including closed-path seam vertices. Point-bearing APIs stay
  on the native line/arc carrier because general algebraic point inversion is not
  silently approximated.
- `QuadraticBezier2`, `CubicBezier2`, `RationalQuadraticBezier2`, and their fact,
  relation, metric, zero-error fitting, staged offset, and flattening APIs represent polynomial and
  rational curve work.
- `BezierParameterPolynomial` isolates represented and algebraic roots in `[0, 1]`;
  `BezierRootIsolationResult2` and `BezierRootIsolationTrace2` expose the ordered
  exact carriers and certificate-work counts for profiling root-heavy operations.
- `PolynomialSplineCurve2` and `NurbsCurve2` retain exact spline geometry, source
  provenance, exact active-domain point and one-sided/certified arbitrary-order
  derivative evaluation, exact knot insertion, splitting, subcurve extraction,
  traversal reversal, shared Bezier decomposition, and zero-allocation
  provenance-bearing span iteration. Periodic constructors accept one period of
  controls and knot breaks, retain the exact period, and provide certified wrapped
  point and derivative evaluation without repeated period stepping.
- `CurveParameterDomain2` and `CurveParameterSide2` provide family-independent
  parameter-domain and knot-side semantics. `Similarity2` transforms every
  top-level curve family and connected path without changing its family or source;
  top-level splitting and subcurve extraction dispatch through the same exact native
  and spline algorithms.
- `Aabb2`, prepared line/arc/curve-string/contour/region views, and segment/region fact
  types preserve repeated-query structure.
- Boolean, event, fragment, split, and boundary-loop types describe staged region
  boolean assembly.
- `ExactCurveError`, `ExactCurveBlocker`, `CurveFamily2`, and `CurveSource2` provide
  contextual exact-operation errors and provenance.
- Finite projection, reconstruction, bulge, and display/certified polyline types define
  IO and display boundaries.

## Precision Model

Native geometry uses `Real` coordinates. Primitive floats appear only in named finite
projection, reconstruction, test, benchmark, rendering, or IO helpers. Bulge imports,
flattened polylines, display offsets, and finite projections use explicit types so
callers can tell whether they are using topology evidence or display geometry.

The crate promotes exact low-degree evidence where it can: line/arc relations, selected
Bezier roots, retained intersection-region shape/refinement/isolation facts, monotone
graph ordering, exact area and moment contributions, length intervals, zero-error
line/arc and Bezier/conic primitive-fit evidence, exact Bezier area/moment
prefix sums for repeated path-range queries, exact-parameter Bezier/conic
split materialization, and retained branch-free Bezier/conic arrangement
traversal with exact tangent-ordered successor selection for simple branch
vertices. Closed retained traversals can materialize native Bezier/conic
boundary regions; polynomial Bezier loops and supported rational conics expose
exact Green-integral signed area. Algebraic split boundaries retain exact point,
tangent, and higher-order endpoint images when construction is certified instead
of being rounded into native coordinates. Algebraic carriers reverse without
demoting their source evidence: endpoint order is swapped, odd derivatives are
negated, even derivatives are preserved, and exact source boundaries are replayed
once and reused. Exact partial collinear, same-circle circular-arc, and
injectively parameterized nonlinear Bezier overlaps retain both source parameter
ranges, split only at overlap endpoints, and apply operation-aware ownership
only to the shared fragments. Exact subdivisions of the same certified-injective
rational Bezier retain their common source interval, so partial overlaps recover
represented local endpoints such as `1/3` directly and bypass resultants even
when point-incidence isolation would otherwise return an algebraic carrier.
For independently constructed curves, exact-rational polynomial coefficients use
the rational-root denominator bound plus isolator refinement and continued-fraction
reconstruction; candidates such as `1/3` are promoted only after exact replay.
Certified injective line images additionally retain irrational algebraic overlap
endpoints in `BezierParameterRange2`; those boundaries flow through top-level
splitting, path ownership, retained traversal, and region materialization without
scalar demotion. Exact polynomial graph certificates extend the same behavior to
curved nonlinear overlaps with independently parameterized irrational endpoints.
Cases that require multivalued implicit branch correspondence, bounded higher-order
fit, or offset trimming return unresolved regions, exact bisection or target-width
isolation results, or explicit uncertainty.

## Performance Model

`hypercurve` avoids numerical explosion by keeping curve objects native and using
structure before generic algebra. Bounding boxes, segment kind, endpoint equality,
monotone spans, material/hole role, prepared views, low-degree dispatch, dyadic
candidate promotion, and source metadata all reduce the number of exact predicates and
root checks.

Prepared curve-string, contour, and region views cache conservative boxes and predicate
handles for repeated containment, self-contact, event, and Boolean-boundary queries.
Prepared mixed-family path pairs use clone-shared exact curve bounds as a conservative
broad phase: only certified AABB misses are removed, and authored versus retained
candidate-pair counts remain observable. When two candidate boxes intersect at one
certified point and that point is an exact endpoint of both curves, endpoint topology
is retained directly without constructing a generic resultant.
Top-level spline clones share successful decompositions, promoted rational evaluators,
and contextual blockers. Top-level derivative dispatch reuses those spline-owned facts
instead of constructing a duplicate `Curve2` evaluator cache.
Single-span polynomial `Curve2` trims retain a positive source-injectivity certificate
and exact root parameter ranges. Related trimmed pairs reuse that fact for partial
overlap dispatch, including reversed ranges, instead of rebuilding resultants.
General rational Bezier clones also share lazily constructed homogeneous control-net,
power-basis, coordinate-derivative numerator, and decided axis-monotonicity facts
across evaluation, subdivision, derivative, incidence, and overlap predicates. Exact
point, derivative-batch, bounds, monotonicity, incidence, candidate, contact, and
overlap queries expose `ExactCurveResult`, preserving operation, curve-family, and
predicate-blocker context instead of leaking an ambiguous public classification.
Exact
subdivisions additionally retain
their source-parameter lineage and a positive source-injectivity certificate, allowing
related partial-overlap pairs to skip resultant construction. Prepared rational-Bezier pairs retain resultant projections, exact paired
contact replay, contact-derived split topology, and lazily assembled arrangements across
clones. Top-level prepared curve pairs retain every promoted span-pair dispatch and share
their provenance-bearing report. Native arc dispatch computes exact circle witnesses,
recovers represented rational-span parameters by projective inversion, and avoids
irrelevant span-pair resultants. Curved Boolean arrangements retain certified contact
vertex identities, so traversal reuses proven connectivity rather than comparing
independently expanded radical coordinates. Borrowed fact views avoid copying algebraic
evidence on repeated queries.
Prepared curved-region pairs additionally seed each retained loop junction as a known
topology vertex, merge new contacts into those identities, and split only the carriers
whose source intervals contain a contact. Each operation result is retained separately,
while source-curve intersection reports remain shared by the pair.
Degree-aligned shared-component replay elevates only the lower-degree homogeneous
Bernstein control net and reuses retained elevations, so independently rebuilt exact
degree elevations do not fall through to an unresolved resultant. Benchmarks track raw
and retained spline decomposition, cached general-rational evaluation, path-pair
preparation/candidate filtering, and ordinary, prepared, and mixed-prepared paths.

## Current Status

Implemented today:

- exact point, line-segment, circular-arc, bulge, curve-string, contour, region, and
  bounding-box APIs, including exact rational quadratic decomposition and
  top-level evaluation for minor, semicircular, major, and full-circle arcs;
- polynomial quadratic/cubic Bezier, rational quadratic/conic, and arbitrary-degree
  rational Bezier objects with structural facts, exact homogeneous evaluation and
  subdivision, certified bounds, one-sign monotonicity fast paths, exact
  mixed-sign derivative-root monotonicity, complete exact point
  incidence with represented or isolated algebraic parameters, exact line contacts
  retaining represented or isolated roots with multiplicity-correct crossing/tangent
  classification, two-axis homogeneous resultant candidate projections, exact represented/algebraic
  candidate replay with explicit incomplete evidence, retained exact parameter ranges for
  full and strict partial rational overlaps, exact algebraic-parameter point and
  first/second/third derivative images, clone-shared prepared pair facts, contact-derived algebraic split topology,
  projective overlap recognition, graph-order
  predicates, retained intersection-region refinement/isolation helpers, and exact
  low-degree relation fast paths;
- exact arbitrary-positive-degree polynomial B-spline and rational B-spline/NURBS
  carriers over clamped or non-clamped finite active knot domains, with homogeneous
  Boehm knot insertion, retained rational Bezier spans, exact parameter evaluation,
  source/version provenance, shared decomposition/native-topology caches, and exact
  homogeneous degree elevation for linear rational spans, native arbitrary-degree
  rational Bezier promotion, per-span parameter provenance, exact authored-knot-domain
  derivatives of arbitrary order, explicit left/right handling for points and
  derivatives at discontinuous knots, exact subdivision and subcurve extraction,
  exact traversal reversal, one-pass clone-shared batch knot refinement, exact
  proof-bearing knot removal by homogeneous inverse insertion, bounded retention of
  positive and negative editing results, and uniform top-level parameter domains;
- exact global NURBS interpolation at authored, uniform, chord-length, or centripetal
  parameters, including fixed rational control weights, averaged clamped knot
  construction, fraction-free Bareiss/Cramer solve evidence, exact matrix residual
  and curve-point replay when scalar normalization certifies it, a retained nonzero
  determinant identity for otherwise unresolved symbolic residuals, stable source
  provenance, and typed singular, projective, and ordering failure context;
- exact arbitrary-target rational Bezier degree elevation in homogeneous Bernstein
  coordinates, retaining root source-parameter lineage and bounded clone-shared
  intermediate elevations and blockers, composed into source-interval-bearing NURBS
  span elevation and exact elevated NURBS reconstruction for finite and periodic
  carriers; elevated carriers align homogeneous span scales and remove extraction
  knots by certified inverse insertion to preserve the source continuity order;
- explicit nonuniform periodic polynomial B-spline and NURBS construction from one
  period of controls and knot breaks, exact cyclic carrier expansion, certified seam
  closure, arbitrary exact parameter wrapping, side-aware seam derivatives, and
  retained periodicity through knot insertion, exact knot removal, reversal, and similarities;
- exact planar similarity transforms for every top-level curve family and connected
  path, preserving family, parameter domain, connectivity, and source provenance;
- exact SVG path-boundary materialization for absolute/relative line, quadratic,
  smooth-quadratic, cubic, smooth-cubic, and circular-arc commands into `CurvePath2`,
  including radical circular centers, major-sweep selection, SVG radius correction,
  retained source/version reports, exact polynomial path export, and explicit refusal
  to approximate SVG-inexpressible rational or higher-order families;
- intersection surfaces for line/line, line/arc, arc/arc, Bezier contact cases, curve
  strings, contours, regions, and prepared repeated-query views, including top-level
  native line/line, line/arc, and arc/arc dispatch with exact contact provenance
  and retained blockers, certified same/reversed full-image overlap orientation
  plus partial collinear and same-circle arc overlap ranges, algebraic parameter
  ranges for certified injective line-image and polynomial-graph overlaps, represented exact
  strict rational-Bezier overlap refinement with same/reversed ownership and retained source
  provenance, clone-shared path promotion, native-family topology materialization,
  prepared all-curve path-pair replay, aggregate path splitting, lazy arrangement assembly,
  operation-aware shared-span ownership from explicit boundary interior sides, exact
  representative-point classification, retained contact-vertex traversal, and
  union/intersection/difference/XOR region materialization for supported split topology,
  including exact algebraic-fragment reversal, endpoint-touching root-isolator
  refinement, composable retained-region Booleans, nested-hole ownership, and cached
  repeated region-pair operations; chained retained regions sharing one source
  parameterization clip full-component overlap reports to their exact algebraic carrier
  intervals before ownership classification;
- signed area, area moment, length interval, flattening, and zero-error primitive
  fitting for Bezier/conic line and point images;
- staged Bezier/conic offset candidates that construct exact line-image offsets and
  leave free-form offsets unresolved until certified trimming/fitting exists;
- primitive line/arc offsets, checked offsets, cap styles, region event/fragment
  extraction, boolean-boundary assembly, retained exact unordered line/arc
  arrangement construction with source/split/endpoint/ring/role/output caches,
  and conservative unresolved states.

Known limits: shared components requiring multivalued implicit branch correspondence,
source curves with neither certified source lineage nor an injective graph axis, generic
resultants whose scalar signs remain uncertified, and offset self-intersection trimming
remain explicit blockers. Algebraically isolated split parameters are retained through
top-level path Booleans when a certified line image or polynomial graph supplies the
branch correspondence. Chained regions consume shared components whose overlap range is
contained by both retained carriers; clipping a shared component at a carrier boundary
is exact when retained identity-parameter evidence certifies the correspondence. A
carrier boundary under an independently parameterized shared component whose parameter
correspondence is not yet certified remains an `Unsupported` blocker.

## Installation

```toml
[dependencies]
hypercurve = "0.3.0"
```

For sibling checkouts:

```toml
[dependencies]
hypercurve = { path = "../hypercurve" }
```

Feature summary:

- `predicates`: default feature enabling `hyperlimit` predicate integration.
- `serde`: enables serialization for public records that support it.

## Usage

The native API uses `hyperreal::Real` coordinates:

```rust
use hypercurve::{CurvePolicy, LineSeg2, Point2, Real};

fn main() -> hypercurve::CurveResult<()> {
    let segment = LineSeg2::try_new(
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(1), Real::from(0)),
    )?;

    let side = segment.classify_point(
        &Point2::new(Real::from(0), Real::from(1)),
        &CurvePolicy::certified(),
    );

    assert!(matches!(side, hypercurve::Classification::Decided(_)));
    Ok(())
}
```

Use native curve objects for Bezier facts, contours, regions, and downstream
geometry work:

```rust
use hypercurve::{
    Contour2, CurvePolicy, LineSeg2, Point2, QuadraticBezier2, Region2, Segment2,
};
use hyperreal::Real;

fn main() -> hypercurve::CurveResult<()> {
    let p = |x, y| Point2::new(Real::from(x), Real::from(y));
    let bezier = QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0));
    let facts = bezier.structural_facts();
    assert_eq!(facts.degree, hypercurve::BezierDegree::Quadratic);

    let bottom = Segment2::Line(LineSeg2::try_new(p(0, 0), p(2, 0))?);
    let right = Segment2::Line(LineSeg2::try_new(p(2, 0), p(2, 2))?);
    let top = Segment2::Line(LineSeg2::try_new(p(2, 2), p(0, 2))?);
    let left = Segment2::Line(LineSeg2::try_new(p(0, 2), p(0, 0))?);
    let contour = Contour2::try_new(vec![bottom, right, top, left])?;
    let region = Region2::from_material_contours(vec![contour]);

    let location = region.classify_point(&p(1, 1), &CurvePolicy::certified());
    assert!(matches!(location, hypercurve::Classification::Decided(_)));
    Ok(())
}
```

For unordered exact line/arc input, arrange through `Region2` and read output and
blockers from the retained result. The runnable
[`arrangement_report`](examples/arrangement_report.rs) example demonstrates the
arrangement, classification, and reporting workflow.

`RegionArrangement2` retains one canonical evaluation and optional materialized
region without duplicating report-shaped state.

## Development

Useful local checks:

```sh
cargo fmt --all -- --check
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
cargo check --benches --all-features
cargo run --example basic
cargo run --example arrangement_report
cargo run --manifest-path examples/hypercurve_ui/Cargo.toml
cargo check --manifest-path examples/hypercurve_ui/Cargo.toml --target wasm32-unknown-unknown
```

## References

CGAL Project. "2D Regularized Boolean Set Operations" and
"2D Arrangements" user manuals. https://doc.cgal.org/latest/.

Bentley, Jon Louis, and Thomas A. Ottmann. "Algorithms for Reporting and
Counting Geometric Intersections." *IEEE Transactions on Computers*, vol. C-28,
no. 9, 1979, pp. 643-647.
https://doi.org/10.1109/TC.1979.1675432.

de Casteljau, Paul. "Outillage methodes calcul." Andre Citroen Automobiles SA,
1959.

de Berg, Mark, et al. *Computational Geometry: Algorithms and Applications*. 3rd
ed., Springer, 2008. https://doi.org/10.1007/978-3-540-77974-2.

Boehm, Wolfgang. "Inserting New Knots into B-Spline Curves." *Computer-Aided
Design*, vol. 12, no. 4, 1980, pp. 199-201.
https://doi.org/10.1016/0010-4485(80)90154-2.

de Boor, Carl. *A Practical Guide to Splines*. Springer, 1978.
https://doi.org/10.1007/978-1-4612-6333-3.

Farouki, Rida T., and C. Andrew Neff. "Analytic Properties of Plane Offset
Curves." *Computer Aided Geometric Design*, vol. 7, nos. 1-4, 1990, pp. 83-99.
https://doi.org/10.1016/0167-8396(90)90002-N.

Farouki, Rida T., and V. T. Rajan. "Algorithms for Polynomials in Bernstein
Form." *Computer Aided Geometric Design*, vol. 5, no. 1, 1988, pp. 1-26.
https://doi.org/10.1016/0167-8396(88)90016-7.

Farin, Gerald. *Curves and Surfaces for Computer-Aided Geometric Design: A
Practical Guide*. 5th ed., Morgan Kaufmann, 2002.
https://doi.org/10.1016/B978-1-55860-737-8.X5000-5.

Foster, Erich L., Kai Hormann, and Romeo Traian Popa. "Clipping Simple Polygons
with Degenerate Intersections." *Computers & Graphics: X*, vol. 2, 2019,
article 100007. https://doi.org/10.1016/j.cagx.2019.100007.

Greiner, Gunther, and Kai Hormann. "Efficient Clipping of Arbitrary Polygons."
*ACM Transactions on Graphics*, vol. 17, no. 2, 1998, pp. 71-83.
https://doi.org/10.1145/274363.274364.

Hobby, John D. "Practical Segment Intersection with Finite Precision Output."
*Computational Geometry*, vol. 13, no. 4, 1999, pp. 199-214.
https://doi.org/10.1016/S0925-7721(99)00021-8.

Hormann, Kai, and Alexander Agathos. "The Point in Polygon Problem for
Arbitrary Polygons." *Computational Geometry*, vol. 20, no. 3, 2001, pp.
131-144. https://doi.org/10.1016/S0925-7721(01)00012-8.

Kåsa, I. "A Circle Fitting Procedure and Its Error Analysis." *IEEE
Transactions on Instrumentation and Measurement*, vol. IM-25, no. 1, Mar.
1976, pp. 8-14. https://doi.org/10.1109/TIM.1976.6312298.

Martinez, Francisco, Antonio J. Rueda, and Francisco R. Feito. "A New Algorithm
for Computing Boolean Operations on Polygons." *Computers & Geosciences*,
vol. 35, no. 6, 2009, pp. 1177-1185.
https://doi.org/10.1016/j.cageo.2008.08.009.

Menger, K. "Untersuchungen über allgemeine Metrik." *Mathematische Annalen*,
vol. 100, 1928, pp. 75-163. https://eudml.org/doc/159284.

Patrikalakis, Nicholas M., Takashi Maekawa, and Woojin Cho. *Shape
Interrogation for Computer Aided Design and Manufacturing*. MIT Hyperbook,
2009. https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/.

Schneider, Philip J., and David H. Eberly. *Geometric Tools for Computer
Graphics*. Morgan Kaufmann, 2002.

Sederberg, Thomas W., and Tomoyuki Nishita. "Curve Intersection Using Bezier
Clipping." *Computer-Aided Design*, vol. 22, no. 9, 1990, pp. 538-549.
https://doi.org/10.1016/0010-4485(90)90039-F.

Shewchuk, Jonathan Richard. "Adaptive Precision Floating-Point Arithmetic and
Fast Robust Geometric Predicates." *Discrete & Computational Geometry*, vol.
18, no. 3, 1997, pp. 305-363. https://doi.org/10.1007/PL00009321.

Tiller, Wayne, and Eric G. Hanson. "Offsets of Two-Dimensional Profiles." *IEEE
Computer Graphics and Applications*, vol. 4, no. 9, 1984, pp. 36-46.
https://doi.org/10.1109/MCG.1984.275995.

Vatti, Bala R. "A Generic Solution to Polygon Clipping." *Communications of the
ACM*, vol. 35, no. 7, 1992, pp. 56-63.
https://doi.org/10.1145/129902.129906.

Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*,
vol. 7, nos. 1-2, 1997, pp. 3-23.
https://doi.org/10.1016/0925-7721(95)00040-2.

## Hyper Ecosystem

`hypercurve` uses [hyperreal](https://github.com/timschmidt/hyperreal),
[hyperlimit](https://github.com/timschmidt/hyperlimit),
[hypersolve](https://github.com/timschmidt/hypersolve), and optionally
[hypertri](https://github.com/timschmidt/hypertri). It provides planar curve and
region topology to the other [Hyper geometry and engineering
crates](https://github.com/timschmidt?tab=repositories&q=hyper&type=source).
