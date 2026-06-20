<h1>
  hypercurve
  <img src="./doc/hypercurve.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hypercurve` is the planar curved-topology crate for the Hyper geometry stack. It owns
line, circular-arc, polynomial Bezier, rational-conic, contour, region,
boolean-boundary, offset, fitting, flattening, and prepared-query surfaces over
`hyperreal::Real` values.

The crate is not yet a complete general boolean or offset engine. It is already useful
as a native exact-aware curve model and as the place where unresolved tangent, overlap,
root-isolation, trimming, and boundary cases stay explicit instead of being hidden by
display polylines.

## WASM Demo

The deployed WASM app is available at <https://timschmidt.github.io/hypercurve/>.

## Hyper Ecosystem

`hypercurve` is the 2D curved-geometry layer.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact scalar values for curve
  coordinates and parameters.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy for
  sidedness, incidence, and sign decisions.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): vector and transform
  facts used by higher geometry and domain crates.
- [hypertri](https://github.com/timschmidt/hypertri): line-only triangulation target for
  polygonalized or straight-edge regions.
- [hypermesh](https://github.com/timschmidt/hypermesh): 3D mesh topology that consumes
  planar arrangements and triangulated face regions.
- [hyperbrep](https://github.com/timschmidt/hyperbrep): BREP trims and planar face
  boundaries that should preserve curve evidence before tessellation.
- [hypersdf](https://github.com/timschmidt/hypersdf): implicit-field consumers and
  preview extraction boundaries.
- [hypersolve](https://github.com/timschmidt/hypersolve): Bernstein, interval, and root
  isolation support for algebraic subproblems.
- [hyperpath](https://github.com/timschmidt/hyperpath): routing, CAM, offset, tangent,
  and swept-path carriers.
- [hypervoxel](https://github.com/timschmidt/hypervoxel): voxel/process consumers for
  flattened or certified region evidence.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): physical shape and
  support consumers.
- [hypercircuit](https://github.com/timschmidt/hypercircuit): circuit context for PCB
  paths and geometry-derived fixtures.
- [hyperparts](https://github.com/timschmidt/hyperparts): part, package, and footprint
  geometry handles.
- [hyperpack](https://github.com/timschmidt/hyperpack): exact packing and clearance
  consumers.
- [hyperevolution](https://github.com/timschmidt/hyperevolution): proposal/search layer
  for curve or layout candidates that still require exact validation.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB checks that use native curve,
  contour, and region objects instead of resolving topology with local float tolerances.

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

- `Point2`, `LineSeg2`, `CircularArc2`, `Segment2`, `CurveString2`, `Contour2`,
  `Region2`, and `RegionView2` are the main planar geometry objects.
- `QuadraticBezier2`, `CubicBezier2`, `RationalQuadraticBezier2`, and their fact,
  relation, metric, zero-error fitting, staged offset, and flattening APIs represent polynomial and
  rational curve work.
- `Aabb2`, prepared line/arc/curve-string/contour/region views, and segment/region fact
  types preserve repeated-query structure.
- `ExactCurveArrangementRequest2`, `ExactCurveWorkspace2`,
  `ExactCurveArrangementEvaluation2`, `ExactCurveArrangementResult2`, and
  `ExactCurveArrangementReport2` are the canonical retained unordered
  line/arc arrangement surface; compatibility region-build reports are derived
  from retained caches when needed.
- Boolean, event, fragment, split, and boundary-loop types describe staged region
  boolean assembly.
- `CurvePolicy`, `Classification<T>`, and uncertainty enums make exact, unresolved, and
  adapter-facing decisions explicit.
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
boundary regions; polynomial Bezier loops expose exact signed area while
rational conic area remains explicit unsupported. Algebraic split boundaries
are carried as certified unresolved fragments until the scalar layer can
materialize true algebraic root endpoints. Cases that need overlap or
same-tangent degeneracy traversal, retained-region role assignment, a more
complete root solver, bounded higher-order fit, or offset trimming return
unresolved regions, exact bisection or target-width isolation results, or
explicit uncertainty.

## Performance Model

`hypercurve` avoids numerical explosion by keeping curve objects native and using
structure before generic algebra. Bounding boxes, segment kind, endpoint equality,
monotone spans, material/hole role, prepared views, low-degree dispatch, dyadic
candidate promotion, and source metadata all reduce the number of exact predicates and
root checks.

Prepared curve-string, contour, and region views cache conservative boxes and predicate
handles for repeated containment, self-contact, event, and boolean-boundary queries.
Benchmarks track ordinary, prepared, and mixed-prepared paths so exactness work can be
optimized where it matters.

## Current Status

Implemented today:

- exact point, line-segment, circular-arc, bulge, curve-string, contour, region, and
  bounding-box APIs;
- polynomial quadratic/cubic Bezier and rational quadratic/conic objects with structural
  facts, evaluation, subdivision, certified bounds, monotone spans, graph-order
  predicates, retained intersection-region refinement/isolation helpers, and exact
  low-degree relation fast paths;
- intersection surfaces for line/line, line/arc, arc/arc, Bezier contact cases, curve
  strings, contours, regions, and prepared repeated-query views;
- signed area, area moment, length interval, flattening, and zero-error primitive
  fitting for Bezier/conic line and point images;
- staged Bezier/conic offset candidates that construct exact line-image offsets and
  leave free-form offsets unresolved until certified trimming/fitting exists;
- primitive line/arc offsets, checked offsets, cap styles, region event/fragment
  extraction, boolean-boundary assembly, retained exact unordered line/arc
  arrangement construction with source/split/endpoint/ring/role/output caches,
  and conservative unresolved states.

Known limits: shared-boundary overlap beyond certified fast paths, full Bezier/rational
root isolation, NURBS, and offset self-intersection trimming remain future work.

## Installation

```toml
[dependencies]
hypercurve = "0.2.0"
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

```rust,ignore
use hypercurve::{
    Contour2, CurvePolicy, LineSeg2, Point2, QuadraticBezier2, Region2, Segment2,
};
use hyperreal::Real;

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

let location = region.classify_point(&p(1, 1), &CurvePolicy::certified())?;
assert!(matches!(location, hypercurve::Classification::Decided(_)));
```

For unordered exact line/arc input, evaluate the retained arrangement attempt and
read output and blockers from the result. Derive a canonical arrangement report
only when a reporting view is needed; use `arrangement_report().into_region_build_report()`,
`derived_region_build_report()`, or `derived_region_build_result()` only for
legacy-shaped compatibility output:

```rust,ignore
use hypercurve::{
    Classification, CurvePolicy, ExactCurveArrangementAttempt2,
    ExactCurveArrangementRequest2, FillRule, LineSeg2, Point2,
};
use hyperreal::Real;

let p = |x, y| Point2::new(Real::from(x), Real::from(y));
let lines = vec![
    LineSeg2::try_new(p(0, 0), p(4, 0))?,
    LineSeg2::try_new(p(4, 0), p(4, 4))?,
    LineSeg2::try_new(p(4, 4), p(0, 4))?,
    LineSeg2::try_new(p(0, 4), p(0, 0))?,
];

let request =
    ExactCurveArrangementRequest2::from_unordered_line_segments(lines, FillRule::NonZero);
let result = ExactCurveArrangementAttempt2::new(request).evaluate(&CurvePolicy::certified())?;

assert!(result.status().unwrap().is_native_exact());
let region = match result.region_classification() {
    Classification::Decided(region) => region,
    Classification::Uncertain(reason) => panic!("arrangement blocked: {reason:?}"),
};
assert!(matches!(
    region.classify_point(&p(2, 2), &CurvePolicy::certified())?,
    Classification::Decided(_)
));

let report = result.arrangement_report();
assert_eq!(report.summary_cache(), result.summary_cache());
```

For convenience callers that want a region classification plus canonical retained
evidence without managing the request object directly, use
`Region2::from_unordered_line_segments_with_arrangement_report`,
`Region2::from_unordered_line_segments_borrowed_with_arrangement_report`,
`Region2::from_unordered_segments_with_arrangement_report`, or
`Region2::from_unordered_segments_borrowed_with_arrangement_report`. These
return `ExactCurveArrangementReport2`; the older `*_with_report` constructors
remain only for legacy-shaped compatibility reports.

## Development

Useful local checks:

```text
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps
cargo run --example arrangement_report
cargo run --manifest-path examples/hypercurve_ui/Cargo.toml
cargo check --manifest-path examples/hypercurve_ui/Cargo.toml --target wasm32-unknown-unknown
cargo bench --bench containment
cargo bench --bench intersection
cargo bench --bench offset
cargo bench --bench reconstruction
cargo bench --bench svg_io --features svg
```

## References

Bentley, Jon Louis, and Thomas A. Ottmann. "Algorithms for Reporting and
Counting Geometric Intersections." *IEEE Transactions on Computers*, vol. C-28,
no. 9, 1979, pp. 643-647.

de Casteljau, Paul. "Outillage methodes calcul." Andre Citroen Automobiles SA,
1959.

de Berg, Mark, et al. *Computational Geometry: Algorithms and Applications*. 3rd
ed., Springer, 2008. https://doi.org/10.1007/978-3-540-77974-2.

Farouki, Rida T., and C. Andrew Neff. "Analytic Properties of Plane Offset
Curves." *Computer Aided Geometric Design*, vol. 7, nos. 1-4, 1990, pp. 83-99.

Farouki, Rida T., and V. T. Rajan. "Algorithms for Polynomials in Bernstein
Form." *Computer Aided Geometric Design*, vol. 5, no. 1, 1988, pp. 1-26.

Farin, Gerald. *Curves and Surfaces for Computer-Aided Geometric Design: A
Practical Guide*. 5th ed., Morgan Kaufmann, 2002.

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
vol. 100, 1928, pp. 75-163. http://eudml.org/doc/159284.

Schneider, Philip J., and David H. Eberly. *Geometric Tools for Computer
Graphics*. Morgan Kaufmann, 2002.

Sederberg, Thomas W., and Tomoyuki Nishita. "Curve Intersection Using Bezier
Clipping." *Computer-Aided Design*, vol. 22, no. 9, 1990, pp. 538-549.

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
