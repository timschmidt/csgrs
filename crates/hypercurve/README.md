# Hypercurve

Exact, evidence-carrying planar curves, paths, contours, and regions for CAD
topology.

Hypercurve is the two-dimensional curve kernel in the Hyper geometry stack. It
models lines, circular arcs, Bézier curves, B-splines, and NURBS with
[`hyperreal::Real`](https://docs.rs/hyperreal) coordinates, then builds
intersection, classification, regularized Boolean, offset, reconstruction, and
finite-projection operations on those carriers.

The crate owns planar curve geometry and topology. It deliberately does not own
solid modeling or mesh topology: CSG grammar and operations such as extrusion,
revolve, sweep, and loft belong in
[CSGRS](https://github.com/timschmidt/csgrs), while triangle-mesh Boolean work
belongs in [Hypermesh](https://github.com/timschmidt/hypermesh).

This README describes crate version `0.3.1`.

## Primary types

| Type | Role |
| --- | --- |
| `Point2`, `Aabb2`, `Similarity2` | Exact planar coordinates, bounds, and similarity transforms |
| `LineSeg2`, `CircularArc2`, `Segment2` | Native line/arc primitives and their common enum |
| `QuadraticBezier2`, `CubicBezier2`, `RationalQuadraticBezier2`, `RationalBezier2` | Polynomial and rational Bézier carriers |
| `PolynomialSplineCurve2`, `NurbsCurve2` | Validated B-spline and NURBS curves |
| `Curve2`, `CurveView2` | Unified owned and borrowed curve carriers |
| `CurveString2`, `CurvePath2`, `Contour2` | Connected open strings, general paths, and closed line/arc contours |
| `CurveRegion2` | Native mixed-family filled planar region |
| `CurveContext`, `CurvePreviewOptions`, `Classification<T>` | One-byte predicate context, explicit lossy preview adapter, and decided/uncertain result |
| `CurveError`, `ExactCurveError` | Construction and exact-topology failure information |

`LineArcRegion2` remains available for compatibility, but new mixed-curve code
should use `CurveRegion2`.

## Install

```toml
[dependencies]
hypercurve = "0.3.1"
```

The default `predicates` feature enables Hyperlimit-backed certified predicate
policy. See [Feature flags](#feature-flags) before disabling defaults or
enabling adapters.

## Quick start

This builds a quadratic Bézier, constructs a square region from exact line
segments, and classifies an interior point.

<!-- quickstart:start -->
```rust
use hypercurve::{
    BezierDegree, Classification, Contour2, CurveContext, CurveRegion2, LineSeg2, Point2,
    QuadraticBezier2, Segment2,
};
use hyperreal::Real;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let p = |x, y| Point2::new(Real::from(x), Real::from(y));
    let bezier = QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0));
    assert_eq!(bezier.structural_facts().degree, BezierDegree::Quadratic);

    let boundary = [
        ((0, 0), (2, 0)),
        ((2, 0), (2, 2)),
        ((2, 2), (0, 2)),
        ((0, 2), (0, 0)),
    ]
    .into_iter()
    .map(|(start, end)| LineSeg2::try_new(p(start.0, start.1), p(end.0, end.1)).map(Segment2::Line))
    .collect::<hypercurve::CurveResult<Vec<_>>>()?;

    let policy = CurveContext::STRICT;
    let contour = Contour2::try_new(boundary)?;
    let region =
        CurveRegion2::try_from_native_material_contours(vec![contour], &policy)?.into_value();
    let location = region.classify_point(&p(1, 1), &policy)?;
    assert!(matches!(location, Classification::Decided(_)));
    Ok(())
}
```
<!-- quickstart:end -->

Run the checked copy:

```sh
cargo run --example basic
```

## How the model fits together

```text
Point2
  ├─ LineSeg2 / CircularArc2 ── Segment2 ── CurveString2 / Contour2
  └─ Bézier / B-spline / NURBS ── Curve2 ── CurvePath2
                                      │
                     arrange / classify / regularize
                                      │
                                CurveRegion2
                                      │
                  Boolean / offset / project / triangulate
```

A `Contour2` is a closed, connected line/arc boundary with a fill rule.
`CurvePath2` generalizes connected paths to every supported curve family.
`CurveRegion2` stores filled topology as oriented native Bézier boundary
fragments and is the main input to mixed-family region operations.

## API guide

The following list covers the useful public front doors. Result and evidence
types have accessors for inspecting counts, sources, blockers, and retained
topology; consult [docs.rs](https://docs.rs/hypercurve) for those fields and
exact signatures.

### Coordinates, primitives, and transforms

- `Point2::{new, from_values, x, y, delta_from, distance_squared, lerp,
  translated, structural_facts}` creates and inspects exact points.
- `LineSeg2::{try_new, point_at, reversed, classify_point, contains_point,
  structural_facts}` covers checked segments and point predicates.
- `CircularArc2::{try_from_center, from_bulge, contains_point,
  contains_sweep_point, point_at_sweep_fraction, reversed, structural_facts}`
  covers directed circular arcs.
- `Segment2::{from_bulge, kind, point_at, contains_point, reversed,
  structural_facts}` dispatches over lines and arcs.
- `Similarity2::{try_from_real_affine, try_from_f64_affine, transform_point,
  reverses_orientation}` validates exact translation, rotation, reflection,
  and uniform scale transforms. Curve and region types expose
  `transform_similarity`; `CurveRegion2` also exposes `transform_affine`.

### Bézier curves

- `QuadraticBezier2::{new, from_line_segment,
  interpolate_point_at_parameter, interpolate_midpoint, point_at,
  control_hull_box, endpoint_tangent, structural_facts}`.
- `CubicBezier2::{new, interpolate_hermite, point_at, control_hull_box,
  endpoint_tangent, structural_facts}`.
- `RationalQuadraticBezier2` and `RationalBezier2` provide checked rational
  construction, evaluation, derivatives, splitting, reversal, transforms, and
  topology/intersection evidence.
- `BezierParameter2`, `BezierParameterRange2`, and
  `BezierRootIsolationResult2` retain exact algebraic parameter information.
- Bézier analysis includes cusp and inflection classification, monotone spans,
  line/curve contacts, curve/curve intersection, length bounds, exact
  polynomial and rational area moments for conics, exact homogeneous degree
  elevations, arbitrary-degree carriers with at-most-quadratic weight
  polynomials, cubic-weight carriers with exactly classified discriminants,
  and arbitrary-degree weight carriers whose rational-root deflation leaves
  either a power of one irreducible quadratic or a quartic product of two,
  plus certified flattening, fitting, and split materialization.
- Parallel and offset entry points include `parallel_left`, `parallel_right`,
  `offset_preflight`, `offset_left_staged`, `offset_right_staged`, and
  `approximate_parallel_blend2d_certified`. Their result types retain error and
  singularity evidence.

### Splines and unified curves

- `PolynomialSplineCurve2::{try_new, try_new_periodic, point_at,
  derivative_at, insert_knot, split_at, subcurve, clamped_subcurve, reversed,
  transform_similarity, bezier_decomposition, bezier_spans}`.
- `NurbsCurve2::{try_new, try_new_periodic, point_at, derivative_at,
  insert_knot, insert_knots, remove_knot, degree_elevation,
  elevated_to_degree, split_at, subcurve, clamped_subcurve, reversed,
  transform_similarity, bezier_decomposition, bezier_spans,
  native_subcurves}`.
- Wrapped evaluation and one-sided evaluation are available on periodic spline
  carriers through the `*_wrapped` and `*_side` method families.
- `Curve2::{new, try_polynomial_bspline, try_nurbs,
  try_periodic_polynomial_bspline, try_periodic_nurbs, family, point_at,
  derivative_at, bounds, split_at, subcurve, clamped_subcurve, reversed,
  transform_similarity, native_bezier_fragments, trim_inside_region,
  trim_inside_region_with_parameters}` is the common owned carrier. Exact
  region trimming returns retained promoted Bézier fragments and keeps
  algebraic/shared-boundary blockers explicit; the parameter-retaining form
  also reports each fragment's promoted span and top-level public parameter
  range.
  `CurveView2` supplies the borrowed equivalents.

### Strings, paths, contours, and regions

- `CurveString2::{try_new, from_bulge_vertices, link_connected_endpoints,
  connect_endpoints_with_line, merge_adjacent_collinear_lines,
  remove_adjacent_reversed_duplicates, trim_between_parameters,
  trim_between_points, chamfer_vertex_by_parameters,
  fillet_vertex_by_parameters}` edits connected line/arc strings.
- `CurvePath2::{try_new, reversed, transform_similarity,
  chamfer_vertex_by_parameters, fillet_vertex_by_parameters, bounds,
  classify_point, native_bezier_fragments, bezier_boundary_loop}` handles
  general connected curves.
- `Contour2::{try_new, try_new_with_fill_rule, from_bulge_vertices,
  signed_area, winding_number, classify_point, point_on_boundary,
  intersect_contour, intersect_self, split_at_intersections,
  split_at_self_intersections}` handles closed line/arc boundaries.
- `CurveRegion2::{empty, arrange_unordered_segments,
  arrange_unordered_line_segments, try_from_native_contours,
  try_from_native_material_contours, try_from_native_boundary_contours,
  try_from_boundary_paths, classify_point, signed_depth, signed_area,
  filled_area, boundary_profiles, materialized_boundary_paths,
  segment_certified, offset, offset_with_certified_segmentation,
  offset_with_certified_bezier_parallel}` is the mixed-family region API.
- `CurveRegion2::{intersect_region, boolean_region, boolean_regions}` returns
  intersection topology or regularized union, intersection, difference, and
  xor results. `BooleanOp` selects an operation; batched
  `CurveRegionBooleanResults2` exposes all four from one evaluation.
- `CurveRegion2::straight_skeleton` and the
  `straight_skeleton_*_events` methods expose staged skeleton construction and
  blockers. `translation_obstacle_convex` constructs the exact translational
  configuration-space obstacle for supported convex contours.

### Conversion, finite output, and adapters

- `CurveString2::{from_real_line_string, from_finite_line_string,
  reconstruct_from_polyline}` and
  `Contour2::{from_real_ring, from_finite_ring,
  reconstruct_from_closed_polyline}` import or reconstruct line/arc geometry.
- `CurveRegion2::recover_from_finite_profiles` reconstructs a region from
  finite material/hole profiles. `PolylineReconstructionOptions` controls the
  distance tolerance.
- `project_to_finite_polyline`, `project_to_finite_curve_paths`,
  `project_to_finite_profiles`, and `project_to_finite_region` provide explicit
  finite approximations. `FiniteProjectionOptions` makes arc chord error
  visible at the boundary.
- With `triangulation`, `FiniteRegionProjection2::triangulate` and
  `triangulate_finite_rings` produce finite triangles through Hypertri.
- With `svg`, `SvgGeometry2::{from_svg, from_svg_with_options, to_svg,
  to_svg_with_options}`, `parse_svg_path_data`, `import_svg_document`, and
  `export_svg_document` provide SVG exchange. Native `L`, `A`, `Q`, and `C`
  commands are used where possible; a versioned `data-hypercurve-path`
  attribute preserves curve families and exact values for Hypercurve
  round-trips.

## Precision, guarantees, and boundaries

Hypercurve separates exact values from decisions about them:

- Coordinates are `Real` values, not an implicit `f64` tolerance model.
- Checked constructors reject malformed or structurally invalid input.
- Topological branches use an explicit `CurveContext`.
  `CurveContext::STRICT` accepts only certified decisions, while
  `CurveContext::APPROXIMATE_512` may consume Hyperlimit's terminal 512-bit
  interpretation.
- `CurvePreviewOptions` owns finite display tolerances separately. Its scoped
  preview results are never exact topology or construction provenance.
- `Classification::Decided(value)` is a supported conclusion.
  `Classification::Uncertain(reason)` preserves an undecidable or unsupported
  predicate instead of silently choosing a side.
- `CurveResult<T>` reports ordinary construction/operation failures.
  `ExactCurveResult<T>` can additionally report the precise exact-topology
  blocker.
- Native output remains exact where the implementation has complete evidence.
  Projection to `f64`, polyline segmentation, SVG rendering, and triangulation
  are explicit conversion boundaries with caller-visible options or evidence.
- Boolean and arrangement result types retain contacts, overlaps, blockers,
  source provenance, and completeness rather than exposing private caches or
  sweep internals.

Support is deliberately operation-specific. A curve family being representable
does not imply that every topology operation is decidable for every symbolic
input. Inspect returned status and blocker evidence instead of treating
uncertainty as empty geometry.

## Feature flags

| Feature | Default | Purpose |
| --- | --- | --- |
| `predicates` | yes | Hyperlimit-backed certified predicate policy |
| `dispatch-trace` | no | Hyperreal/Hyperlimit dispatch instrumentation |
| `triangulation` | no | Finite-region triangulation through Hypertri |
| `svg` | no | SVG import/export and exact round-trip extension |
| `hershey` | no | Compiled Hershey stroke fonts and native curve-string text |
| `comparative-benchmarks` | no | Third-party benchmark adapters only |

Minimal and common configurations:

```sh
cargo check --no-default-features
cargo test --all-features
cargo run --example arrangement
cargo check --features svg
```

The browser demo lives in `examples/hypercurve_ui` and is built separately with
Trunk. It is not part of the library API.

## Validation and performance

The quick start is compiled as `examples/basic.rs` and checked byte-for-byte
against this README. The test suite also covers adversarial exact predicates,
mixed-family region Booleans, regression corpora, and finite adapters.

Detailed benchmark definitions and interpretation live in
[PERFORMANCE.md](PERFORMANCE.md) and
[COMPARATIVE_BENCHMARKS.md](COMPARATIVE_BENCHMARKS.md). Fuzz target ownership
and replay instructions live in [fuzz/README.md](fuzz/README.md). These are
maintainer validation resources, not API guarantees.

## References

These sources describe algorithms or numerical principles used by the crate;
they are not claims of source-code derivation.

- Aichholzer, O., Aurenhammer, F., Alberts, D., and Gärtner, B. “A Novel Type
  of Skeleton for Polygons.” *Journal of Universal Computer Science* 1(12),
  1995, 752–761. [DOI: 10.3217/jucs-001-12-0752](https://doi.org/10.3217/jucs-001-12-0752).
- Bentley, J. L., and Ottmann, T. A. “Algorithms for Reporting and Counting
  Geometric Intersections.” *IEEE Transactions on Computers* C-28(9), 1979,
  643–647. [DOI: 10.1109/TC.1979.1675432](https://doi.org/10.1109/TC.1979.1675432).
- Boehm, W. “Inserting New Knots into B-Spline Curves.”
  *Computer-Aided Design* 12(4), 1980, 199–201.
  [DOI: 10.1016/0010-4485(80)90154-2](https://doi.org/10.1016/0010-4485(80)90154-2).
- de Boor, C. *A Practical Guide to Splines*. Springer, 1978.
  [DOI: 10.1007/978-1-4612-6333-3](https://doi.org/10.1007/978-1-4612-6333-3).
- de Berg, M., Cheong, O., van Kreveld, M., and Overmars, M.
  *Computational Geometry: Algorithms and Applications*, 3rd ed. Springer,
  2008. [DOI: 10.1007/978-3-540-77974-2](https://doi.org/10.1007/978-3-540-77974-2).
- Farouki, R. T., and Neff, C. A. “Analytic Properties of Plane Offset
  Curves.” *Computer Aided Geometric Design* 7(1–4), 1990, 83–99.
  [DOI: 10.1016/0167-8396(90)90002-N](https://doi.org/10.1016/0167-8396(90)90002-N).
- Farouki, R. T., and Rajan, V. T. “Algorithms for Polynomials in Bernstein
  Form.” *Computer Aided Geometric Design* 5(1), 1988, 1–26.
  [DOI: 10.1016/0167-8396(88)90016-7](https://doi.org/10.1016/0167-8396(88)90016-7).
- Foster, E. L., Hormann, K., and Popa, R. T. “Clipping Simple Polygons with
  Degenerate Intersections.” *Computers & Graphics: X* 2, 2019, 100007.
  [DOI: 10.1016/j.cagx.2019.100007](https://doi.org/10.1016/j.cagx.2019.100007).
- Greiner, G., and Hormann, K. “Efficient Clipping of Arbitrary Polygons.”
  *ACM Transactions on Graphics* 17(2), 1998, 71–83.
  [DOI: 10.1145/274363.274364](https://doi.org/10.1145/274363.274364).
- Hormann, K., and Agathos, A. “The Point in Polygon Problem for Arbitrary
  Polygons.” *Computational Geometry* 20(3), 2001, 131–144.
  [DOI: 10.1016/S0925-7721(01)00012-8](https://doi.org/10.1016/S0925-7721(01)00012-8).
- Martinez, F., Rueda, A. J., and Feito, F. R. “A New Algorithm for Computing
  Boolean Operations on Polygons.” *Computers & Geosciences* 35(6), 2009,
  1177–1185. [DOI: 10.1016/j.cageo.2008.08.009](https://doi.org/10.1016/j.cageo.2008.08.009).
- Patrikalakis, N. M., Maekawa, T., and Cho, W. *Shape Interrogation for
  Computer Aided Design and Manufacturing*. MIT Hyperbook, 2009.
  [MIT](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/).
- Sederberg, T. W., and Nishita, T. “Curve Intersection Using Bézier
  Clipping.” *Computer-Aided Design* 22(9), 1990, 538–549.
  [DOI: 10.1016/0010-4485(90)90039-F](https://doi.org/10.1016/0010-4485(90)90039-F).
- Shewchuk, J. R. “Adaptive Precision Floating-Point Arithmetic and Fast
  Robust Geometric Predicates.” *Discrete & Computational Geometry* 18(3),
  1997, 305–363. [DOI: 10.1007/PL00009321](https://doi.org/10.1007/PL00009321).
- Tiller, W., and Hanson, E. G. “Offsets of Two-Dimensional Profiles.”
  *IEEE Computer Graphics and Applications* 4(9), 1984, 36–46.
  [DOI: 10.1109/MCG.1984.275995](https://doi.org/10.1109/MCG.1984.275995).
- Vatti, B. R. “A Generic Solution to Polygon Clipping.”
  *Communications of the ACM* 35(7), 1992, 56–63.
  [DOI: 10.1145/129902.129906](https://doi.org/10.1145/129902.129906).
- Weiss, M., Jüttler, B., and Aurenhammer, F. “Mitered Offsets and Skeletons
  for Circular Arc Polygons.” *Mathematics of Computation* 90, 2021,
  251–283. [DOI: 10.1090/mcom/3551](https://doi.org/10.1090/mcom/3551).
- Yap, C. K. “Towards Exact Geometric Computation.” *Computational Geometry*
  7(1–2), 1997, 3–23.
  [DOI: 10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).

## Acknowledgements

Hypercurve builds on
[Hyperreal](https://github.com/timschmidt/hyperreal),
[Hyperlimit](https://github.com/timschmidt/hyperlimit), and
[Hypersolve](https://github.com/timschmidt/hypersolve), with optional
[Hypertri](https://github.com/timschmidt/hypertri) integration. The wider
[Hyper ecosystem](https://github.com/timschmidt?tab=repositories&q=hyper&type=source)
provides the three-dimensional and engineering layers.

The bibliography above acknowledges the research traditions that inform the
implementation. Optional comparison dependencies are benchmark or validation
peers and do not provide Hypercurve’s native topology.

The optional compiled single-stroke font catalog was created by Dr. A. V.
Hershey at the U.S. National Bureau of Standards. Its source distribution
format was created by James Hurt of Cognition, Inc.; the integrated Rust
representation is not the U.S. NTIS distribution format. The complete
required acknowledgement is available as
`hypercurve::hershey::FONT_DATA_NOTICE`.

## License and contributing

Licensed under the [Apache License 2.0](LICENSE).

Bug reports should include the smallest exact input, selected features, policy,
operation, and returned blocker or uncertainty evidence. Before proposing a
change, run `cargo fmt --all -- --check`, the relevant focused test, and
`cargo test --all-features`.
