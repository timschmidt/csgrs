<h1>
  hypercurve
  <img src="./doc/hypercurve.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hypercurve` is a planar curve kernel for line, circular-arc, and first
polynomial Bezier geometry in the hyperreal geometry stack. The current
implementation focuses on
exactness-aware topology: segment intersections, closed-contour containment,
signed region views, region-pair event extraction, fragment splitting, prepared
query views, and the first region boolean / offset pipeline.

The crate keeps Cavalier-compatible bulge semantics where they are useful, but
the native APIs expose explicit uncertainty for tangent, overlap, boundary, and
unsupported cases instead of silently resolving them through global epsilon
rules.

In the Hyper ecosystem it is the 2D curved-topology layer: `hyperreal`,
`hyperlattice`, and `hyperlimit` provide scalar/algebra/predicate support;
`hypercurve` owns contours, regions, offsets, and curve relations; `hypertri`
handles line-only triangulation; and `hyperdrc` consumes this work for
manufacturing-readiness checks.

## WASM Demo

The deployed WASM app is available at
<https://timschmidt.github.io/hypercurve/>.

## Hyper Stack Links

- [hyperreal](../hyperreal/README.md): exact rational, symbolic, and computable
  real arithmetic.
- [hyperlimit](../hyperlimit/README.md): exact predicate policy and certified
  geometric decisions.
- [hyperlattice](../hyperlattice/README.md): small exact vector, matrix, and
  transform algebra.
- [hypercurve](../hypercurve/README.md): planar curve, contour, region, and
  boolean geometry.
- [hypertri](../hypertri/README.md): exact polygon triangulation and constrained
  Delaunay topology.
- [hypermesh](../hypermesh/README.md): 3D mesh boolean experiments and the
  future exact-aware mesh-topology layer.
- [hypersolve](../hypersolve/README.md): experimental exact-aware solver layer.
- [hyperdrc](../hyperdrc/README.md): PCB design-readiness checks over exact-aware
  geometry adapters.
- [hyperphysics](../hyperphysics/README.md): placeholder physics-domain crate
  for the exact geometry stack.
- [csgrs](../csgrs/readme.md): constructive solid geometry and polygon boolean
  engine used by HyperDRC and available as an interop target.

## Numeric Model

`hypercurve` stores and decides core geometry with `hyperreal::Real`
values. Primitive `f64` appears only in named edge
conversions, tests, benchmarks, rendering/IO helpers, and compatibility
harnesses. There is no approximate numeric mode feature.

## Traditional Curve Problems

Curved planar geometry combines all the usual robust-predicate failures with
extra representation problems: tangent contacts, overlapping arcs, nearly
coincident boundaries, Bezier roots, offset self-intersections, and lossy
chordization. A fixed epsilon can make a boolean pass one fixture and fail the
next; eager exact algebra can also explode before broad-phase filters have
removed obvious misses.

`hypercurve` therefore stages work. It keeps native curve objects and exact
control structure, uses boxes and prepared views to cut candidate sets, promotes
low-degree exact certificates when they are available, and returns explicit
uncertainty or parameter regions for cases needing a complete root solver.
Flattening, fitting, simplification, and display offsets are represented as
certified or display-only adapters rather than hidden replacements for the
topology model.

## Semantic Boundary and Structural Facts

`hypercurve` owns planar curve and region semantics: line and circular-arc
segments, curve strings, closed contours, material/hole bins, prepared
broad-phase views, boolean boundary traversal, offset policy, and uncertainty
for curve-topology cases that are not yet resolved.

It does not own Real representation, small linear algebra kernels, generic
predicate policy, triangulation topology, solver active sets, or PCB/CAM domain
metadata. Real facts come from `hyperreal`, optional vector and transform
facts come from `hyperlattice`, exact sidedness/incidence decisions from
`hyperlimit`, and line-only triangulation should be delegated to `hypertri`.

Prepared curve and region objects should continue to retain structural metadata
where it is discovered cheaply: bounding boxes, segment kind, endpoint equality
classes, exact ring area/winding, material-versus-hole role, monotone arc spans,
parameter ranges, tangent/contact hints, and source ids. Those facts are
valuable for selecting exact line-line, line-arc, and arc-arc kernels and for
reducing broad-phase work. They should not collapse into primitive-float
topology decisions; lossy chordization or export must stay behind explicit
policies.

## Current Status

Core geometry:

- Line and circular-arc segments with explicit intersection result types,
  including finite same-circle arc overlap intervals.
- Polynomial quadratic and cubic Bezier object types that store exact `Real`
  control points, evaluate with de Casteljau subdivision, expose certified
  control-hull boxes, and retain structural facts for derivative edges, second
  differences, curvature witnesses, exact-rational schedules, and symbolic
  dependencies.
- First polynomial Bezier topology predicates: exact derivative-root monotone
  decomposition, certified endpoint/extrema bounds, quadratic line-root
  relation, cubic supporting-line root isolation, coarse curve/curve
  disjoint/shared-endpoint relations, exact cusp checks from common derivative
  roots, and cubic inflection parameters.
- Rational quadratic Bezier/conic object type with exact homogeneous
  evaluation, structural coordinate/weight facts, and certified conic-family
  classification from the weight discriminant.
- First rational-conic predicates for parameterized point membership and
  exact point-on-conic parameter recovery through homogeneous coordinate
  equations, supporting-line relation through the exact weighted Bernstein
  numerator, plus quotient-derivative monotone decomposition and certified
  bounds, with projective denominator boundaries reported as uncertainty.
- Coarse rational-conic curve/curve relation for exact homogeneous identity,
  projective homogeneous weight-scaling identity, positive-weight convex-hull
  disjointness, exact native line-line intersection dispatch for certified
  endpoint line-segment images, equal-weight collapse into polynomial Bezier
  predicates, and exact shared endpoints.
- Mixed polynomial quadratic/cubic curve relation through exact shared
  endpoints and certified Bezier bounds, with overlapping boxes left explicit
  for a later root-isolation solver.
- Polynomial Bezier curve/curve subdivision regions that cover all remaining
  possible intersections after exact control-hull box pruning and narrower
  algebraic dispatches.
- Exact same-parameter polynomial quadratic/quadratic intersection dispatch,
  returning certified interior crossing points before generic subdivision, and
  certifying no-intersection for degree-normalized shared-axis strictly
  monotone quadratic/cubic graph pairs whose remaining coordinate is
  sign-separated in Bernstein form.
- Exact degree-elevated quadratic/cubic polynomial image equality, reported
  before endpoint-only overlap checks.
- Exact same-parameter dyadic candidate promotion through thirty-seconds for mixed
  quadratic/cubic and native cubic/cubic polynomial Bezier pairs after degree
  normalization.
- Certified dyadic point-parameter probes for cubic Beziers, reused to promote
  endpoint-on-cubic intersections before generic subdivision while keeping the
  complete point-on-cubic solver explicit future work.
- Exact native line-line intersection dispatch when polynomial Bezier control
  polygons are certified to trace their endpoint line segments.
- Exact supporting-line root dispatch when one polynomial Bezier is a certified
  endpoint line image and the other curve has certified line roots, yielding
  exact intersection points or certified no-intersection after finite-segment
  containment.
- Exact endpoint-on-quadratic dispatch for polynomial Bezier curve relations,
  certifying endpoint hits that occur at interior quadratic parameters before
  falling back to subdivision regions.
- Exact polynomial quadratic point-on-curve parameter recovery, exposed as a
  certified API and reused by endpoint-on-quadratic relation dispatch.
- Certified polynomial Bezier length intervals using exact endpoint-chord lower
  bounds and exact control-polygon upper bounds, with exact de Casteljau
  subdivision refinement and prefix-interval bounds for tighter metric
  enclosures plus inverse arc-length parameter regions.
- Exact polynomial Bezier signed-area and first area-moment contributions,
  prefix contributions through Green's-theorem boundary integrals, plus exact
  signed-area and area-moment prefix-sum tables for fitting and simplification
  ranges.
- Mixed rational-conic/polynomial Bezier curve relation in both directions,
  certifying equal-weight quadratic identity, positive-weight hull
  disjointness, exact native line-line intersection dispatch for certified
  endpoint line-segment images, exact supporting-line root dispatch when one
  side is a certified endpoint line image, exact shared endpoints,
  endpoint-on-conic hits, and homogeneous subdivision regions for overlapping
  positive-weight cases.
- Certified polynomial Bezier flattening adapter that emits a polyline only
  when exact hull-to-chord flatness is within the requested error budget.
- Zero-error certified polyline simplification for flattened Beziers that only
  removes exactly collinear interior vertices.
- Zero-error exact-line fitting for certified flattened Bezier polylines, with
  the source flattening certificate retained on successful fits.
- Direct zero-error exact-line-image fitting for polynomial Bezier control
  polygons and positive-weight rational quadratic conics, with true
  line-primitive offsets for certified line images.
- True line-primitive offsets for certified zero-error Bezier line fits.
- Display-only offset previews for certified flattened Bezier polylines, with
  no topology claim beyond the source flattening certificate.
- A supporting-line/full-circle predicate surface that classifies disjoint,
  tangent, secant, and uncertain outcomes before finite segment and arc-sweep
  filters are applied.
- A full-circle/full-circle predicate surface that classifies coincident,
  disjoint, tangent, secant, and uncertain outcomes before finite arc-sweep
  filters are applied.
- Cavalier-compatible bulge vertex helpers and bulge import/export for
  supported line and circular-arc sweeps.
- Axis-aligned bounding boxes for points, segments, curve strings, contours, and
  regions, used as conservative broad-phase filters before exact curve
  intersections, contour/region event collection, point classification, and
  self-contact checks.
- Closed contours with winding and boundary classification.
- Owned and borrowed regions with explicit material and hole contour bins.
- Polyline reconstruction can collapse sampled point runs into native line and
  circular-arc curve strings or closed contours, using finite-difference
  curvature witnesses and conservative tolerances at the IO boundary.

Offsets:

- Primitive left offsets for line and circular-arc segments, with explicit
  uncertainty when an arc offset would collapse or reverse radius.
- Raw open curve-string and closed-contour left offsets with line joins:
  line-line corners are mitered, while parallel, arc, and mixed joins use
  circular join arcs until the full offset trim/rebuild pipeline is implemented.
- Curve-string/contour self-contact detection and checked offset entry points
  that reject raw joined offsets requiring self-intersection trimming.
- Checked closed outlines for open curve strings using left/right offsets and a
  selectable `OffsetCap` style: circular, straight butt, or square end caps.

Prepared and repeated-query paths:

- Prepared borrowed curve-string, contour, and region views cache segment,
  contour, and whole-region boxes for repeated self-contact, curve-string
  intersection, contour/region point classification, and contour/region event
  queries. They also retain per-segment prepared predicate handles for future
  line/arc batches, route prepared curve-string pairs through those segment
  handles, plus prepared region boolean-boundary loop,
  checked-contour, and region-result traversal. Mixed prepared-vs-region event
  and boolean wrappers reuse either prepared operand while transiently
  preparing the ordinary side, without changing exact boundary semantics.
- Prepared line-segment views cache exact predicate endpoint conversion and
  structural facts for repeated supporting-line classifications while keeping
  finite-segment containment semantics on the native segment type.
- Prepared circular-arc views cache radial sweep predicates and structural arc
  facts for repeated sweep and point-on-arc classification.
- Region point classification skips contours whose bounding boxes are decidably
  missed before exact boundary and winding tests.

Boolean and region pipeline:

- Region-pair event collection and point-bearing fragment splitting.
- Boolean fragment classification for union, intersection, difference, and xor.
- Directed boundary-fragment emission, endpoint-connected chain assembly, and
  closed-loop reconstruction into checked contours.
- Region-level boolean boundary pipeline for producing closed result loops or
  checked boundary contours when no unresolved topology remains.
- Resolved boolean boundary contours can be nested into material/hole bins to
  produce a `Region2` result.
- Exact contour-bin boolean identities, including reordered bins, rotated start
  vertices, and reversed traversal, are handled before general traversal so
  coincident boundaries do not force uncertainty for `A op A`.
- Empty-operand boolean identities are handled before traversal, preserving
  material/hole roles for the nonempty region.
- Region-result identity and empty-set fast paths clone explicit material/hole
  bins directly, so valid touching-bin regions are not reinterpreted by
  boundary-only nesting.
- Boolean fragment emission is role-aware: selected fragments from hole
  contours are oriented as negative-fill edges before chain assembly.
- Region-level xor is assembled from the two one-sided differences and merges
  their explicit signed bins, preserving boundary-touching components.
- Checked boundary-contour xor uses the same symmetric-difference region path
  before exposing unassigned boundary loops.
- Boundary-only contacts are certified before traversal: point contacts use
  regularized set identities, and external shared-edge contacts drop coincident
  zero-area edges for union/xor output.
- Boundary-touching containment identities are handled before traversal for
  union, intersection, and contained-minus-container difference.
- Container-minus-boundary-touching-subset difference is rebuilt for certified
  shared-edge containment by dropping the coincident zero-area boundary and
  assembling the remaining directed fragments into the notched result.
- Imported Cavalier deterministic and fuzz suites are present as compatibility
  references.

Known limits:

- This crate is not yet a complete boolean or offset engine.
- Shared-boundary fragments with positive-area overlap beyond the certified
  containment/contact fast paths still report unresolved topology.
- Some point-touching containment branches and otherwise ambiguous topology are
  intentionally unresolved until the general overlap resolver is implemented.
- Joined offsets are not yet self-intersection trimmed; checked offset entry
  points reject cases that need that trimming.
- General implicit conics, full rational-conic and Bezier curve/curve root
  isolation, and NURBS are not part of the current implementation.
- Primitive-float reconstruction and tessellation are IO/display helpers, not
  the internal topology model.

## API Shape

The native API uses `hyperreal::Real` coordinates:

```rust
use hypercurve::{CurvePolicy, LineSeg2, Point2, Real};

fn main() -> hypercurve::CurveResult<()> {
    let a = Point2::new(Real::from(0), Real::from(0));
    let b = Point2::new(Real::from(1), Real::from(0));
    let segment = LineSeg2::try_new(a, b)?;

    let policy = CurvePolicy::certified();
    let query = Point2::new(Real::from(0), Real::from(1));
    let side = segment.classify_point(&query, &policy);

    assert!(matches!(side, hypercurve::Classification::Decided(_)));
    Ok(())
}
```

For Cavalier-style polyline data, `BulgeVertex2` and reconstruction helpers sit
at the adapter boundary. They are useful for compatibility and UI workflows,
but topology decisions should still flow through native `Segment2`, `Contour2`,
`Region2`, and prepared query views.

## Documentation

Public APIs are written to build cleanly on docs.rs. Local verification uses:

```text
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps
```

## UI Test Article

The `examples/hypercurve_ui` package recreates the Cavalier UI test article
around `hypercurve` operations. It keeps a bulge-vertex editor for convenient
interactive testing, then converts that data into `hypercurve` contours and
regions for booleans, intersections, slices, and offsets.

```text
cargo run --manifest-path examples/hypercurve_ui/Cargo.toml
cargo check --manifest-path examples/hypercurve_ui/Cargo.toml --target wasm32-unknown-unknown
trunk serve examples/hypercurve_ui/index.html
```

If Trunk 0.21 rejects `NO_COLOR=1` in the local environment, run the Trunk
command as `env -u NO_COLOR trunk serve examples/hypercurve_ui/index.html`.
The `.github/workflows/deploy-ui-pages.yml` workflow builds this same Trunk app
on pushes to `main` and deploys the release artifact with GitHub Pages Actions;
the repository's Pages source must be set to GitHub Actions for the first
deployment.

## Benchmarks

Small no-dependency benchmark targets exercise the current ordinary, prepared,
and mixed-prepared boolean boundary pipeline, containment hot paths including
prepared contour and region repeated-query classifiers, segment-intersection
hot paths, ordinary and prepared curve-string intersections, ordinary and
prepared bounding-box filtered self-contact scans, ordinary and prepared
region-event scans, and primitive plus open-outline offsets:

```text
cargo bench --bench boolean_pipeline
cargo bench --bench containment
cargo bench --bench intersection
cargo bench --bench offset
cargo bench --bench reconstruction
```

## References

Bentley, Jon Louis, and Thomas A. Ottmann. "Algorithms for Reporting and
Counting Geometric Intersections." *IEEE Transactions on Computers*, vol. C-28,
no. 9, 1979, pp. 643-647.

de Casteljau, Paul. "Outillage methodes calcul." Andre Citroen Automobiles SA,
1959.

cavalier_contours. "2D Polyline/Shape Library for Offsetting, Combining, etc."
Rust crate and repository. https://github.com/jbuckmccready/cavalier_contours.

CavalierContours. C++ polyline offsetting and combining library.
https://github.com/jbuckmccready/CavalierContours.

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
