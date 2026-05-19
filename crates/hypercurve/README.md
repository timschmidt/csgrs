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
  coordinates, parameters, and certificates.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy for
  sidedness, incidence, and sign decisions.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): vector and transform
  facts used by higher geometry and domain crates.
- [hypertri](https://github.com/timschmidt/hypertri): line-only triangulation target for
  polygonalized or straight-edge regions.
- [hyperdrc](https://github.com/timschmidt/hyperdrc),
  [hyperpath](https://github.com/timschmidt/hyperpath), and
  [hyperparts](https://github.com/timschmidt/hyperparts): domain crates that should
  hand curve, contour, and region evidence here instead of resolving topology with local
  float tolerances.

## Typical Curve Problems

Curved planar geometry combines robust-predicate failures with representation failures:
tangent contacts, overlapping arcs, nearly coincident boundaries, Bezier roots, offset
self-intersections, and lossy chordization. Fixed epsilons can make one fixture pass and
the next fail; eager exact algebra can also expand before broad-phase filters eliminate
obvious misses.

`hypercurve` stages the work. Native curve objects keep exact control structure,
prepared views and boxes reduce candidate sets, low-degree certificates are promoted
when available, and unresolved tangent, overlap, root-isolation, trimming, or topology
cases are reported explicitly instead of hidden behind display polylines.

## Main Types

- `Point2`, `LineSeg2`, `CircularArc2`, `Segment2`, `CurveString2`, `Contour2`,
  `Region2`, and `RegionView2` are the main planar geometry objects.
- `QuadraticBezier2`, `CubicBezier2`, `RationalQuadraticBezier2`, and their fact,
  relation, metric, fitting, flattening, and offset reports represent polynomial and
  rational curve work.
- `BezierOffsetAdapterReport2` classifies staged offset outputs as exact primitive,
  zero-distance identity, ready for a certified adapter, or blocked by preflight risks.
- `BezierFitReadinessReport2` classifies certified flattened polylines as exact point
  fits, exact line fits, or future higher-order fitting candidates while retaining
  flattening and simplification certificates.
- `BezierFitSourceReport2` gathers exact polynomial Bezier source facts for future
  higher-order fitting: control facts, monotone spans, cusp/inflection status,
  endpoint tangents, moments, length bounds, and primitive-image fit attempts.
- `BezierFitSourceBatchReport2` aggregates those source reports over path ranges so
  future simplifiers can inspect exact primitive counts, higher-order-fit counts,
  length intervals, and moment totals without resampling.
- `BezierFitSourcePrefixSums2` stores prefix aggregates for those source reports so
  half-open path ranges can be queried by exact subtraction instead of rewalking
  sources.
- `BezierIntersectionRegionFacts`, `BezierIntersectionRegionSummary`,
  `BezierIntersectionRegionRefinement`, and
  `BezierIntersectionRegionIsolationReport` classify retained Bezier curve/curve
  parameter regions and run bounded or target-width exact bisection with explicit stop
  reasons. `BezierIntersectionRegionIsolationCertificate` compacts a retained frontier
  into auditable shape counts and certified maximum parameter widths for later solvers.
- `BezierBooleanHandoffReport2` converts Bezier curve/curve relation results into a
  boolean-topology readiness report: split-ready parameterized events, point witnesses
  that still need parameter recovery, overlap obligations, retained-region isolation
  blockers, unresolved cases, or primitive predicate uncertainty.
- `Aabb2`, prepared line/arc/curve-string/contour/region views, and segment/region fact
  types preserve repeated-query structure.
- Boolean, event, fragment, split, and boundary-loop types describe staged region
  boolean assembly.
- `CurvePolicy`, `Classification<T>`, and uncertainty enums make exact, unresolved, and
  adapter-facing decisions explicit.
- Finite projection, reconstruction, bulge, and display/certified polyline types define
  IO and display boundaries.

## Precision Model

Native geometry uses `Real` coordinates. Primitive floats appear only in named finite
projection, reconstruction, test, benchmark, rendering, or IO helpers. Bulge imports,
flattened polylines, display offsets, and finite projections carry certificates or
adapter status so callers can tell whether they are using topology evidence or display
geometry.

The crate promotes exact low-degree certificates where it can: line/arc relations,
selected Bezier roots, retained intersection-region shape/refinement/isolation facts and
frontier certificates, Bezier boolean handoff reports, monotone graph ordering, exact
area and moment contributions, length intervals, zero-error simplification, zero-error line/point fitting,
fitting-readiness reports, exact source-fact, source-batch, and source-prefix reports
for higher-order fitting, and exact staged-offset readiness reports. Cases that need a
more complete root solver, bounded higher-order fit, or offset trimmer return unresolved
regions, exact bisection or target-width isolation reports, boolean handoff blockers,
adapter-readiness reports, or explicit uncertainty.

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
  predicates, retained intersection-region summaries/refinement/isolation reports, and
  exact low-degree relation fast paths;
- intersection surfaces for line/line, line/arc, arc/arc, Bezier contact cases, curve
  strings, contours, regions, and prepared repeated-query views;
- signed area, area moment, length interval, flattening, simplification, fitting, and
  display/certified polyline offset adapters;
- primitive line/arc offsets, checked offsets, cap styles, region event/fragment
  extraction, boolean-boundary assembly, and conservative unresolved states.

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

## Development

Useful local checks:

```text
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps
cargo run --manifest-path examples/hypercurve_ui/Cargo.toml
cargo check --manifest-path examples/hypercurve_ui/Cargo.toml --target wasm32-unknown-unknown
cargo bench --bench boolean_pipeline
cargo bench --bench containment
cargo bench --bench intersection
cargo bench --bench offset
cargo bench --bench reconstruction
cargo bench --bench bezier_facts
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
