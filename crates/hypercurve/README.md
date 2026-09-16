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
| `CurvePoint2` | Exact curve point retaining coordinates or selected geometric evidence |
| `LineSeg2`, `CircularArc2`, `Segment2` | Native line/arc primitives and their common enum |
| `QuadraticBezier2`, `CubicBezier2`, `RationalQuadraticBezier2`, `RationalBezier2` | Polynomial and rational Bézier carriers |
| `PolynomialSplineCurve2`, `NurbsCurve2` | Validated B-spline and NURBS curves |
| `Curve2` | Shared exact curve carrier with borrowed operations |
| `CurveString2`, `CurvePath2`, `Contour2` | Connected open strings, general paths, and closed line/arc contours |
| `CurveRegion2` | Native mixed-family filled planar region |
| `CurveContext`, `CurvePreviewOptions`, `Classification<T>` | One-byte predicate context, explicit lossy preview adapter, and decided/uncertain result |
| `CurveError`, `ExactCurveError` | Construction and exact-topology failure information |

`CurveRegion2` is the sole public filled-region carrier. Native line/arc
specializations remain private fast paths inside the unified kernel.

## Install

```toml
[dependencies]
hypercurve = "0.3.1"
```

Hyperreal, Hypersolve, and Hyperlimit are mandatory parts of the exact kernel;
feature flags select only optional adapters and instrumentation.

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
    let location = region.classify_point(&p(1, 1), &policy)?.into_value();
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
- `CurvePoint2::{from, coordinates, coincides_with, compare_coordinate, bounds}`
  handles exact curve contacts through one opaque value. `coordinates()` is an
  optional view of stored `Real` coordinates; selected points retain their
  exact meaning and support geometric queries without that view. Predicates
  return their certainty under the requested `CurveContext`.
- `LineSeg2::{try_new, point_at, reversed, classify_point, contains_point,
  structural_facts}` covers checked segments and point predicates.
- `CircularArc2::{try_from_center, from_bulge, contains_point,
  contains_sweep_point, point_at_sweep_fraction, reversed, structural_facts}`
  covers directed circular arcs.
- `Segment2::{from_bulge, kind, point_at, contains_point, reversed,
  structural_facts}` dispatches over lines and arcs.
- `Similarity2::{try_from_real_affine, try_from_f64_affine, transform_point,
  scale, reverses_orientation}` validates exact translation, rotation, reflection,
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
- `RationalBezier2::from_homogeneous_controls` retains exact Bernstein
  `(X, Y, W)` coefficients, including zero-weight intermediate controls.
  `try_new` accepts affine authoring points and weights; `affine_control_points`
  returns an optional finite view. Splitting and degree elevation preserve the
  homogeneous representation without forcing every control into affine space.
  Domain finiteness and local convex-hull certificates are separate proofs.
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
- Parallel entry points include `parallel_left`, `parallel_right`, and
  `approximate_parallel_blend2d_certified`. Their result types retain error and
  singularity evidence; exact topology-producing offsets are owned by the
  unified `CurveRegion2` engine below.
- `BezierParallel2::{from_source, source, distance, point_at, derivative_at,
  reversed, split_at_exact, subcurve_between_exact, conservative_bounds,
  transform_similarity, point_incidence, supporting_line_incidence,
  intersections, parallel_intersection_candidates, parallel_intersections}`
  is the compact exact procedural parallel carrier. General parallel pairs use
  complete polynomial projections, exact common-component saturation,
  selected normal and tangent-degeneracy predicates, refined tensor-Bernstein
  rejection, and preconditioned Poincare-Miranda box replay. Exact source
  overlap transport handles partial and reversed reparameterizations;
  materializable and structural overlap lanes remain cheaper. Its
  `BezierParallelSource2` plus signed distance is the lossless structural
  export boundary.

### Splines and unified curves

- `PolynomialSplineCurve2::{try_new, try_new_periodic, point_at,
  derivative_at, insert_knot, split_at, subcurve, clamped_subcurve, reversed,
  transform_similarity, bezier_decomposition, bezier_spans}`.
- `NurbsCurve2::{try_new, try_new_periodic, from_homogeneous_controls,
  point_at, derivative_at,
  insert_knot, insert_knots, remove_knot, degree_elevation,
  elevated_to_degree, split_at, subcurve, clamped_subcurve, reversed,
  transform_similarity, bezier_decomposition, bezier_spans,
  native_subcurves}`.
- NURBS and rational Bézier spans share `HomogeneousControl2` coefficients.
  Extraction, knot insertion/removal, and degree recomposition preserve zero
  and mixed control weights without affine projection. `homogeneous_controls`
  is authoritative; `affine_control_points` is an optional finite view.
  Homogeneous NURBS construction takes an expanded knot vector and explicit
  `SplinePeriodicity2`; span evaluators retain their exact source knot intervals.
  Conic and polynomial specializations are optional, and linear rational spans
  retain degree one. Bounds and monotonicity use the actual curve denominator.
- Wrapped evaluation and one-sided evaluation are available on periodic spline
  carriers through the `*_wrapped` and `*_side` method families.
- `Curve2::{new, try_polynomial_bspline, try_nurbs,
  try_periodic_polynomial_bspline, try_periodic_nurbs, family, point_at,
  derivative_at, bounds, split_at, subcurve, reversed,
  transform_similarity, native_bezier_fragments, trim_inside_region,
  trim_inside_region_with_parameters}` is the common owned carrier. Exact
  region trimming returns reusable `Curve2` pieces from authored and generated
  supports. The parameter-retaining form reports their oriented source
  locations as `CurveLocation2`, exact parameter ranges and boundary contacts.
  Positive-length boundary overlaps are retained; isolated tangencies add no
  curve. Path trimming keeps disconnected spline spans in separate chunks.
  These operations borrow `&Curve2` directly and reuse its retained calculations.
  Borrow paths as `&CurvePath2`; iterate their curves with `path.curves().iter()`.
  `Curve2` also retains generated analytic parallels, selected circles, chords,
  and algebraic cuts. `start` and `end` return `CurvePoint2`; `parameter_domain`
  returns `CurveParameterRange2`, preserving selected endpoint evidence.
  `geometry()` is an optional native definition, and `coordinates()` is an
  optional scalar view of a point. Neither view is required for lossless
  `CurveRegion2::boundary_paths` export or subsequent region construction.
  `point_at` and `point_at_side` accept `CurveParameter2` and return `CurvePoint2`,
  retaining selected roots and local fibers without requiring coordinate images.
  A `Real` or `BezierParameter2` converts directly into the common parameter.
  `parameter.scalar()` and `range.scalar_endpoints()` expose stored `Real`
  views. Selected parameters remain exact when these views are absent.
  Intersections return `CurveLocation2` contacts and `CurveParameterRange2`
  overlap ranges. A location retains its support parameter and span chart;
  `location.parameter(&policy)` maps it into the authored curve domain on
  demand, preserving selected roots for evaluation and subdivision.
  `CurveParameter2::compare` compares parameters in a shared support chart
  without requiring scalar payloads and reports predicate certainty.
  Curve and open-path intersections consume retained rational-source cuts
  and exact chords directly, clipping contacts and overlaps to both active domains.
  Overlap boundaries retain their certified correspondence, including a shared
  endpoint when clipping leaves no positive-length span. Chord pairs and
  chord/rational-source pairs share the region intersection kernels while
  retaining endpoint contacts for open paths. Overlap ranges pair corresponding
  endpoints in the first curve's traversal order, including reversed overlaps.
  Selected overlap cuts reuse their source-interval certificates, and evaluation
  reuses finite-chord parameter certificates without reconstructing incidence.
  Self-crossings of projectively corresponding supports retain their
  off-diagonal contacts.
  Curve and path `intersection_topology` results expose reusable `Curve2` pieces
  in traversal order. They preserve selected source parameters and one-sided
  spline endpoints without requiring native Bézier materialization. A path's
  `CurvePathSplit2::curves()` groups pieces by authored curve. The borrowed
  `arrangement_graph()` shares the topology's retained graph; its source indices
  identify authored curves, followed by fragment indices in traversal order.
  Graph preparation participates in the topology operation's certainty result.
  Retraced components, exterior source domains, and generated circle
  and parallel pair kernels still report explicit blockers where their
  common dispatch is unfinished.
  Generated curves keep their source chart when traversal is reversed.
  Subdivision accepts the same common parameters. Selected ranges retain their
  source chart and endpoint evidence, share one authored source through repeated
  cuts, and cover every intersected arc or spline span. Splitting returns pieces
  in traversal order and preserves both one-sided endpoints at spline jumps.
  Reversal, similarity transforms, bounds, and path corner edits consume these
  retained carriers. General differential and pair-intersection APIs are still
  being unified; operations report an explicit blocker for carriers they cannot
  yet consume.

### Strings, paths, contours, and regions

- `CurveString2::{try_new, from_bulge_vertices, link_connected_endpoints,
  connect_endpoints_with_line, merge_adjacent_collinear_lines,
  remove_adjacent_reversed_duplicates, trim_between_parameters,
  trim_between_points}` edits connected line/arc strings without owning corner
  construction semantics.
- `CurvePath2::{try_new, reversed, transform_similarity,
  chamfer_vertex_by_setbacks, fillet_vertex_by_radius, bounds, classify_point,
  native_bezier_fragments, bezier_boundary_loop}` is the sole connected-curve
  corner-edit authority and handles general connected curves. Corner edits
  enumerate exact solutions from design setbacks or radius; callers do not
  supply a preselected trim/contact answer.
- `Contour2::{try_new, try_new_with_fill_rule, from_bulge_vertices,
  signed_area, winding_number, classify_point, point_on_boundary,
  intersect_contour, intersect_self, split_at_intersections,
  split_at_self_intersections}` handles closed line/arc boundaries.
- `CurveRegion2::{empty, arrange_unordered_segments,
  try_from_native_contours,
  try_from_native_material_contours, try_from_native_boundary_contours,
  try_from_boundary_paths, classify_point, signed_depth, signed_area,
  filled_area, boundary_profiles, boundary_paths,
  segment_certified, offset}` is the mixed-family region API. `offset` is the
  sole region offset operation and takes an explicit `OffsetCornerStyle2`;
  unsupported exact carriers remain explicit blockers. `segment_certified`
  is a separate lossy output adapter and never participates in offset
  topology.
- `CurveRegion2::{intersect_region, boolean_region, boolean_regions}` returns
  intersection topology between regularized boundaries, or regularized union,
  intersection, difference, and xor results. Authored winding and canceled
  seams are resolved first. `BooleanOp` selects an operation; batched
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
  finite approximations. `FiniteProjectionOptions` makes the curve chord-error
  budget visible at the boundary.
- With `triangulation`, `FiniteRegionProfile2::triangulate` and
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
| `dispatch-trace` | no | Hyperreal/Hyperlimit dispatch instrumentation |
| `triangulation` | no | Finite-region triangulation through Hypertri |
| `svg` | no | SVG import/export and exact round-trip extension |
| `hershey` | no | Compiled Hershey stroke fonts and native curve-string text |
| `comparative-benchmarks` | no | Third-party benchmark adapters only |

Common configurations:

```sh
cargo check
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

The rank-independent public-Boolean matrix, recursive radial/projective
regressions (including fourth-generation offsets), and release-scale PCB
corpora run in the normal suite. Longer selected-fiber and source-cusp stroke
corpora can also be run explicitly:

```sh
cargo test --release --locked --all-features --lib selected_fiber_genuinely_analytic_contacts_complete_region_booleans -- --ignored
cargo test --release --locked --all-features --test hypercurve_curve_region_stroke -- --ignored
```

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
