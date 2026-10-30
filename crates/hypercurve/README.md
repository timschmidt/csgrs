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
belongs in [Hypermesh](https://github.com/timschmidt/csgrs/tree/main/crates/hypermesh).

This README describes crate version `0.3.1`.

## Primary types

| Type | Role |
| --- | --- |
| `Point2`, `Aabb2`, `Similarity2` | Exact planar coordinates, bounds, and similarity transforms |
| `CurvePoint2` | Exact curve point retaining coordinates or selected geometric evidence |
| `CurvePointDefinition2` | Replayable point sources and shared selected roots, with strict validation on reconstruction |
| `CurvePointAffineForm2` | Source-bound query of `a*x + b*y + c`, preserving selected coordinate correlation |
| `LineSeg2`, `CircularArc2`, `Segment2` | Native line/arc primitives and their common enum |
| `QuadraticBezier2`, `CubicBezier2`, `RationalQuadraticBezier2`, `RationalBezier2` | Polynomial and rational Bézier carriers |
| `PolynomialSplineCurve2`, `NurbsCurve2` | Validated B-spline and NURBS curves |
| `PolynomialCurveSet2`, `PolynomialCurveBranch2` | Complete solver-certified polynomial zero sets as native graph branches and isolated points |
| `Curve2` | Shared exact curve carrier with borrowed operations |
| `CurveNativeSegment2` | Proof of a complete oriented line/arc image, borrowing the original curve |
| `CurveNativeImage2` | Complete native curve image, retaining its source and selected-root authority |
| `CurveAnalyticParallel2` | Borrowed procedural parallel definition, including selected range and traversal |
| `CurveString2`, `CurvePath2`, `Contour2` | Connected open strings, general paths, and closed line/arc contours |
| `CurveRegion2` | Native mixed-family filled planar region |
| `provisional`, `evaluate_under`, `Provisional<T>`, `CurvePreviewOptions`, `Classification<T>` | Explicitly qualified approximate evaluation, its certainty-carrying result, lossy preview adapter, and decided/uncertain result |
| `evaluate_with_budget`, `WorkBudget`, `BudgetOutcome<T>`, `BudgetExceeded` | Shared operation work limits, admitted usage, and typed resource exhaustion |
| `CurveError`, `ExactCurveError` | Construction and exact-topology failure information |

`CurveRegion2` is the sole public filled-region carrier. Native line/arc
specializations remain private fast paths inside the unified kernel.

`PolynomialCurveSet2::from_topology()` consumes a complete
`hypersolve::bivariate_topology::BivariateCurveTopology`. Its `Curve2` branches
retain the equation, projection slab, ordered fiber root and selected event
endpoints. Evaluation, subdivision, reversal, conservative bounds and similarity
transforms keep the original first-coordinate parameter chart and shared source.
`CurvePointDefinition2::PolynomialFiber` replays the selected base and fiber
together; untrusted singleton claims are checked. Isolated points remain separate
from branches. This collection does not establish winding or a filled region.
A branch in its polynomial's own plane intersects an authored line with rational
endpoints, and the pieces either split returns, through `Curve2::intersect_curve`
and `intersection_topology`, including tangencies. The line parameter replays the
branch's fiber point, except on a horizontal line, where it is the abscissa's own
root. A line contained in the zero set and other branch intersections report
typed blockers.
General derivatives, rational Bézier promotion, filled arrangements, and branch
archive replay remain unsupported. Mixed affine forms can be signed in the local
fiber; constructing their scalar parameter is restricted when both coefficients
are nonzero and the base parameter is algebraic. Fiber subdivision/refinement
limits and instrumented work budgets are not total-work or heap-byte bounds.

`CurvePoint2::definition()` retains supported rational/parallel source images,
correlated coordinate expressions, similarities and intersections of exact
supporting lines. `SupportingLineIntersection` stores two endpoint pairs;
`into_point()` rechecks their strict definitions, nondegeneracy and unique
intersection. The intersection may lie outside either finite witness chord.
Parallel or coincident supports return `CurveError::ParallelLineSupports`;
degenerate supports and unresolved proofs keep their own typed outcomes.
NURBS coordinate images reuse their original selected root for replay and affine
queries. Independent coordinate roots and stored certainty flags are not used
to replace that identity. This source view is not complete authoring history.

`Curve2::native_segment()` proves eligible complete line/arc images under a
strict policy, including represented retained chords and supported single-span
restrictions. The returned view retains the source curve; its native segment
may have a different parameter chart. A support fit alone does not prove a
trim's traversal. Missing range, injectivity or representation proofs return
typed blockers. This query neither samples geometry nor certifies an earlier
approximate construction's fidelity.

`Curve2::native_image()` also admits single retained Bézier spans whose endpoints
have exact scalar views. Selected quadratic roots replay their defining
polynomial and isolator without changing their original field or caches. Exact
subcurve substitution preserves the complete nonlinear or nonmonotone curve;
the returned geometry uses its own native chart. The same shared proof enables
retained boundary measurements when the native area integral is available. General selected
fields, multiple retained spans and nonlinear parallels can still return typed
blockers. Simplicity, filled material and source qualification remain separate
claims.

`Curve2::analytic_parallel()` borrows the procedural source and distance,
ascending parameter range, traversal and original curve. The view also covers
analytic selected-fiber subdivisions. It preserves the current definition
without fitting or rebuilding selected coordinates; it is not a certificate
to trust on untrusted replay or a complete authoring-history receipt.

Region and closed-path point queries accept `CurvePoint2`, preserving selected
roots and generated contact evidence. Convert authored coordinates with
`Point2::into()`; `coordinates()` is an optional stored-coordinate view. Region
batches accept `&[CurvePoint2]` and share query preparation across scalar and
generated points. Closed paths retain their trace and even-odd interior; filled
regions classify their regularized boundary, including holes and nested islands.

`CurveRegion2::try_from_boundary_paths(paths, fill_rule)` applies one
fill rule to the total signed winding of all closed paths, then publishes the
regularized set. `NonZero` preserves equally oriented overlaps and cancels
opposite winding; `EvenOdd` selects odd winding. SVG compound fills use this
same admission path. Explicit material/hole constructors instead combine each
loop's filled membership by its supplied role. Interior sides are certified by the
arrangement from winding; authoring does not require orientation hints.

## Install

Add hypercurve 0.4.0 to a project with `cargo add hypercurve`; it requires Rust
1.95 or newer. The examples below are checked in; from the root of a
[CSGRS source checkout](../../readme.md), run:

```sh
cargo run -p hypercurve --example basic
```

Hyperreal, Hypersolve, and Hyperlimit are mandatory parts of the exact kernel;
feature flags select only optional adapters and instrumentation.

## Quick start

This builds a quadratic Bézier, constructs a square region from exact line
segments, and classifies an interior point.

<!-- quickstart:start -->
```rust
use hypercurve::{
    BezierDegree, Contour2, CurveRegion2, LineSeg2, Point2, QuadraticBezier2, RegionPointLocation,
    Segment2,
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

    let contour = Contour2::try_new(boundary)?;
    let region = CurveRegion2::try_from_native_material_contours(vec![contour])?;
    assert_eq!(
        region.classify_point(&p(1, 1).into())?,
        RegionPointLocation::Inside
    );
    Ok(())
}
```
<!-- quickstart:end -->

Run the checked copy:

```sh
cargo run -p hypercurve --example basic
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
`CurvePath2::simple_loop_orientation` proves clockwise/counterclockwise traversal
from complete arrangement topology and winding without requiring an area
integral. Its domain is one unpinched closed chain of certified injective spans,
with no overlaps or internal splits. Open paths are invalid; an unavailable
simplicity or winding proof is a typed blocker. A single periodic carrier may
need subdivision. Endpoint chords never supply this proof.
`CurveRegion2` stores filled topology as oriented exact boundary curves and
is the main input to mixed-family region operations. Curve operations emit
boundary provenance for inspection; authored input supplies geometry and fill
semantics.

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
  exact meaning and support geometric queries without that view. Undecided
  predicates are reported as blockers.
- `CurvePoint2::affine_form(a, b, c)` retains an exact affine query of the point.
  `compare_to_real`, `compare`, and `bounds` reuse its roots and normal-sheet
  evidence; they do not reconstruct independent scalar coordinates. Bounds are
  conservative intervals, and unavailable proofs remain typed blockers.
- `LineSeg2::{try_new, point_at, reversed, classify_point, contains_point,
  structural_facts}` covers checked segments and point predicates.
- `CircularArc2::{try_from_center, from_bulge, contains_point,
  contains_sweep_point, point_at_sweep_fraction, reversed, structural_facts}`
  covers directed circular arcs.
- `Segment2::{from_bulge, kind, point_at, contains_point, reversed,
  structural_facts, distance_squared, point_distance_squared}` dispatches over
  lines and arcs. Minimum squared distances use exact endpoints and stationary
  candidates on the native spans; unresolved candidates return typed blockers.
  These primitive queries do not measure general Bézier/NURBS boundaries or
  replace filled-region regularization.
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
  Domain finiteness and local convex-hull certificates are separate proofs. Retained
  bounds and rational region point queries use the active finite interval,
  including exterior charts and finite restrictions excluding source poles.
- `BezierParameter2`, `BezierParameterRange2`, and
  `BezierRootIsolationResult2` retain exact algebraic parameter information.
- `Curve2::has_certified_injective_image` supplies a strict sufficient proof
  for complete images, including connected native spans with one common
  monotone coordinate. A missing certificate does not imply self-intersection.
  [Qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-spline-replay/README.md)
  covers its use in independent BREP replay and its explicit limits.
- Bézier analysis includes cusp and inflection classification, monotone spans,
  line/curve contacts, curve/curve intersection, length bounds, exact
  polynomial and rational area moments for conics, exact homogeneous degree
  elevations, arbitrary-degree carriers with at-most-quadratic weight
  polynomials, cubic-weight carriers with exactly classified discriminants,
  and arbitrary-degree weight carriers whose rational-root deflation leaves
  either a power of one irreducible quadratic or a quartic product of two,
  plus certified flattening and fitting. Exact cuts go through `Curve2::split_at`.
- Parallel entry points include `parallel_left`, `parallel_right`, and
  `approximate_parallel_blend2d_certified`. Their result types retain error and
  singularity evidence; exact topology-producing offsets are owned by the
  unified `CurveRegion2` engine below.
- `BezierParallel2::{from_source, source, distance, point_at, derivative_at,
  reversed, split_at_exact, subcurve_between_exact, conservative_bounds,
  transform_similarity}` is the compact exact procedural parallel carrier.
  Incidence and intersection go through `Curve2::try_analytic_parallel`,
  `Curve2::point_locations`, and `Curve2::intersect_curve`. General parallel pairs use
  complete polynomial projections, exact common-component saturation,
  selected normal and tangent-degeneracy predicates, refined tensor-Bernstein
  rejection, and preconditioned Poincare-Miranda box replay. Exact source
  overlap transport handles partial and reversed reparameterizations;
  materializable and structural overlap lanes remain cheaper. Its
  `BezierParallelSource2` plus signed distance is the lossless structural
  export boundary. `Curve2::try_analytic_parallel` restricts a parallel to an
  exact source-parameter range as a general `Curve2`, without fitting.

### Splines and unified curves

- `PolynomialSplineCurve2::{try_new, try_new_periodic, point_at,
  derivative_at, insert_knot, split_at, subcurve, clamped_subcurve, reversed,
  transform_similarity}`.
- `NurbsCurve2::{try_new, try_new_periodic, from_homogeneous_controls,
  point_at, derivative_at, insert_knot, insert_knots, remove_knot,
  elevated_to_degree, split_at, subcurve, clamped_subcurve, reversed,
  transform_similarity, native_subcurves}`.
- Exact Bézier spans of any spline come from the unified curve:
  `Curve2::from(spline).native_bezier_fragments()` returns each span as
  `CurveGeometry2`, the same authored-family vocabulary used by `Curve2::new`,
  with its exact source parameter interval.
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
- `Curve2::{new, try_line, try_analytic_parallel, try_from_bezier_range,
  point_locations,
  try_polynomial_bspline, try_nurbs,
  try_periodic_polynomial_bspline, try_periodic_nurbs, family, point_at,
  derivative_at, bounds, box_containment, split_at, subcurve, reversed,
  transform_similarity, native_bezier_fragments, trim_inside_region,
  trim_inside_region_with_parameters}` is the common owned carrier.
  `bounds` is a cached conservative enclosure, the control hull for polynomial
  Bézier spans; `box_containment` decides whether the curve itself lies in a
  closed box, from that enclosure and the exact coordinate extrema, and
  returns an exact curve point outside the box when it does not.
  Rational spans and retained rational trims use quotient-derivative roots on
  a certified pole-free active range within the unit source chart, including
  NURBS span charts. Root isolation or comparison may still return a typed
  blocker; the witness remains a point rather than a published curve location.
  `try_line` joins two general exact points, which may be selected algebraic
  intersection or corner locations, without rounding either endpoint.
  `point_locations` reports every exact parameter visit of a represented
  point as `CurveLocation2` values, one per visit and one per continuous
  spline seam, or `EntireCurve` for a constant curve. Authored spans reuse
  their cached rational evaluators; generated parallels, chords, and selected
  circles use their own incidence authorities on their retained ranges.
  `point_at` and `derivative_at` take general `CurveParameter2` values.
  Derivatives are `CurveVector2`: represented coordinates when the parameter
  is a plain real, otherwise a selected-field vector with an exact chart
  factor whose coordinate signs are decided without rounding. Selected
  parameters are supported on authored spans, retained rational pieces and,
  to first order, analytic parallels: the source velocity scaled by the
  exact speed ratio `1 - d cross(v, a) / |v|^3`, whose sign (including the
  exact zero at a parallel cusp) is decided in the selected field. Other
  generated carriers and orders report an explicit unsupported blocker. Exact
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
  `location.parameter()` maps it into the authored curve domain on
  demand, preserving selected roots for evaluation and subdivision.
  `curve.parameter_from_span(index, local)` transports overlap and collapsed
  component parameters through the same authored chart; it does not itself
  certify incidence or membership. `restrict_source` preserves that chart for
  authored curves and generated retained supports, including reversed traversal.
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
  `CurvePathSplit2::curves()` groups pieces by authored curve. Arrangement
  graphs are internal: regions are admitted from `CurvePath2` boundaries.
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
  chamfer_vertex_by_setbacks, fillet_vertex, bounds, box_containment,
  classify_point, native_bezier_fragments}` is the sole connected-curve
  corner-edit authority and handles general connected curves. Chamfers
  enumerate exact solutions from design setbacks. Fillets take `CurveFillet2`
  with an exact radius and optional center or contact constraints. Contact
  parameters use the incident input curve's chart; contact points retain every
  matching source location. Both edits return finite `CurveCornerSolutions2`.
  A continuous fillet family requires an additional exact constraint and returns
  `CurveError::FilletConstraintRequired`; it never publishes an arbitrary sample.
- `Contour2::{try_new, try_new_with_fill_rule, from_bulge_vertices,
  signed_area, winding_number, classify_point, point_on_boundary,
  intersect_contour, intersect_self, split_at_intersections,
  split_at_self_intersections}` handles closed line/arc boundaries.
- `CurveRegion2::{empty, arrange_unordered_segments,
  try_from_native_contours,
  try_from_native_material_contours, try_from_native_boundary_contours,
  try_from_boundary_paths, classify_point, classify_points, signed_area,
  filled_area, boundary_profiles, boundary_paths,
  segment_certified, offset, stroke_path}` is the mixed-family region API. `offset` is the
  sole region offset operation and takes an explicit `OffsetCornerStyle2`;
  unsupported exact carriers remain explicit blockers. `segment_certified`
  is a separate lossy output adapter and never participates in offset
  topology. Point queries return membership of the regularized set; authored
  winding multiplicity is resolved during construction.
  Boundary-path and native-boundary constructors take one explicit global fill
  rule and return a regularized region directly. Native contours may cross,
  overlap, or touch; their individual contour fill rules do not override the
  chosen compound fill. Explicit material/hole constructors retain per-loop
  fill semantics.
  `stroke_path` takes a positive half-width, a corner style and an endpoint
  `OffsetCap`. A certified distinct single line supports exact butt and round
  strokes, including diagonal directions, symbolic radii and exact coordinates
  beyond binary64 range. Retained parallels and semicircular caps keep analytic
  area, bounds and rebuilt boundary paths consumable under `STRICT`. Boundary
  subdivision and ordering can differ from earlier arrangement output; compare
  represented sets separately from serialized artifacts. Unresolved predicates
  remain typed blockers. This does not add a total-work or heap-byte bound.
  Unordered segment admission returns the region through the same exact-result
  contract. Unresolved endpoint assembly or topology remains an exact blocker;
  native views and structural counts are computed when requested.
- `CurveRegion2::{intersect_region, boolean_region, boolean_regions}` returns
  intersection topology between regularized boundaries, or regularized union,
  intersection, difference, and xor results. Authored winding and canceled
  seams are resolved first. `BooleanOp` selects an operation; batched
  `CurveRegionBooleanResults2` exposes all four from one evaluation.
- `Contour2::straight_skeleton`, `CurvePath2::straight_skeleton`, and the
  `Contour2::straight_skeleton_*_events` methods expose staged skeleton construction and
  blockers. `translation_obstacle_convex` constructs the exact translational
  configuration-space obstacle for supported convex contours.

### Conversion, finite output, and adapters

- `CurveString2::{from_real_line_string, from_finite_line_string,
  reconstruct_from_polyline}` and
  `Contour2::{from_real_ring, from_finite_ring,
  reconstruct_from_closed_polyline}` import or reconstruct line/arc geometry.
- `CurveRegion2::recover_from_finite_profiles` reconstructs and regularizes a
  region from finite material/hole profiles. `PolylineReconstructionOptions` controls reconstruction
  tolerance; the import remains lossy relative to the original source curves.
- `project_to_finite_polyline`, `project_to_finite_curve_paths`,
  `project_to_finite_profiles`, and `project_to_finite_ring` provide explicit
  finite approximations. `FiniteProjectionOptions` makes the curve chord-error
  budget visible at the boundary.
- `CurveRegionProfile2::linear_contours` retains exact certified line images,
  including authored lines stored as Bézier curves, with material/hole ownership.
  Missing line support returns `None`; unresolved evidence remains a typed error.
  With `triangulation`, `triangulate_linear_contours` delegates to Hypertri and
  returns exact native points with predicate certainty, without finite projection.
- With `triangulation`, `FiniteRegionProfile2::triangulate` and
  `triangulate_finite_rings` produce finite triangles through Hypertri.
- `marching_squares` consumes `MarchingSquaresGrid` with increasing exact axes,
  x-fast scalar samples, and a strictly exterior boundary. Edge crossings remain
  exact; ambiguous cells use the center-sample-sum sign rule, treating zero as
  inside. Chords define a polygonal proposal, not the bilinear level curve or a
  continuous-field fidelity bound. Existing native arrangement supplies material,
  holes and disconnected components. Invalid grids, unknown decisions, allocation
  failures and unsupported topology return `MarchingSquaresError`. Use
  `evaluate_under` to retain certainty under an explicit policy. Primary buffers
  are checked; secondary allocations, total work and cancellation remain unbounded.
- With `svg`, `SvgGeometry2::{from_svg, from_svg_with_options, to_svg,
  to_svg_with_options}`, `parse_svg_path_data`, `import_svg_document`, and
  `export_svg_document` provide SVG exchange. Native `L`, `A`, `Q`, and `C`
  commands are used where possible; a versioned `data-hypercurve-path`
  attribute preserves curve families and exact values for Hypercurve
  path and filled-region round-trips. Version 3 remains readable and is still emitted for
  authored paths. Version 4 retains authored-source restrictions and procedural
  quadratic, cubic and rational Bezier parallels, including signed distance,
  selected Bezier roots, projective parameter charts and traversal in a shared
  definition graph per path. Import rechecks roots, charts, domains, parallel
  regularity and connectivity under strict predicates; `SvgError::Exact` retains
  failures from this replay. Parallel ranges keep their original source chart,
  including finite ranges outside the authored unit interval. Cusp boundaries
  must remain explicitly split.
  Selected straight chords preserve their supporting line, validated monotone
  coordinate chart and directed endpoint selections. Replayable endpoints include
  exact coordinates, rational source images, rational expressions and native
  parallel points at retained Bezier parameters. Shared curve-endpoint records
  preserve their incidence relation. Import proves point definitions, support
  incidence and chart order, including for unused records. Auxiliary chord
  contacts and general joint-parameter charts remain open.
  Parallel endpoint projection uses the requested refinement independently of
  stronger cached predicate bounds, keeping repeated display output stable after
  exact incidence queries.
  The standard SVG `d` attribute is only a finite display projection.
  Selected rational-curve points accept shared rational images and repeated
  maps. Predicates pull queries into their original parameter field and retain
  original denominator guards; independent coordinate projection is optional.
  Checked definitions, similarities and native line replay consume these images.
  Point dependencies also retain polynomial fibers, rational maps, exact
  similarities and unique supporting-line intersections. Replay validates fiber
  selectors and optional exact-root witnesses, original denominator guards and
  line uniqueness. These points need no independently materialized scalar
  coordinates. Chord displacements retain the actual selected source, normal or
  tangent direction, signed distance, optional origin and exact translation;
  paired endpoints share one replayed recipe.
  `SvgOptions::max_point_definition_depth` defaults to 256 and
  counts nested map, similarity, displacement and supporting-line point definitions; leaves
  have depth zero. Endpoint and chord references carry their dependency depth.
  Reached depth limits return `SvgError::SizeOverflow` before publishing a path;
  callers may retry with a larger limit. This limit and extension bytes do not
  bound total arithmetic, coefficient growth, retained memory or cancellation.
  Chord-normal selected circles retain their center, directed frame chord,
  signed nonzero radius, scalar or selected normal-contact parameters and finite
  oriented range. Independently supplied incident points retain their own
  definition and re-prove incidence and the selected half on replay. A transported
  oblique chord direction is checked for unit length, parallelism and orientation;
  a stored certification flag is insufficient. Construction-time tangent flags
  are not serialized. The retained relations support fresh-process splitting,
  reversal, algebraic similarities and the named region reconstruction witnesses.
  Correlated point queries reuse the original rational-map and fiber fields for
  oblique linear comparison and circle incidence. When a certified point's
  displacement from a represented circle frame reduces to exact constants, the
  existing half chart inverts those displacements without solving another
  line-circle intersection. The degree-seven/five center witness continues
  through exact point location, subdivision, reversal and replay. Nonconstant
  incident points in a shared rational/algebraic field now retain the point as
  their angular authority. A represented circle frame orders those points
  without reconstructing an independent angular scalar. Exact cuts, reversal,
  similarities and native replay preserve the shared field. Circle derivatives
  in a represented unit frame reuse the rational chart evaluator. Represented
  parameters and incident points in a shared polynomial-point field keep exact
  derivative magnitude and all requested orders; restriction and reversal retain
  the source parameter. Shared-field circle boundaries now enter `CurveRegion2`
  through the existing line/minor-arc winding and arrangement. Named witnesses
  cover angular trimming and path replay, holes, disconnected components,
  winding cancellation, mixed circle/diameter loops and a round +1/4 then -1/4
  offset chain. Generated line representatives preserve their common point field.
  Circle/line endpoint contacts retain selected-half and transverse evidence.
  One certified incident endpoint can also determine the second contact in the
  same selected field. The original circle equation, selected half, finite chord
  and crossing sign are checked separately; tangency retains one contact.
  Generated chord comparisons reuse the shared-field determinant through
  similarities, and retained line intersections keep their two-support identity
  proof. The named radius-one disk erosion by two now produces canonical empty
  geometry through the arrangement and continues through offset and Boolean
  identities. Arbitrary independent fields, general circle/line roots, other
  frame and point-image carriers, and general region admission still have
  implementation gaps.
  Transported unit directions are omitted only when reconstruction retains the
  same serialized scalar definitions; value equality alone is insufficient for
  deterministic display projection.
  A circle frame adds one dependency level to the point-depth limit.
  Filled regions use version 5 compound path roots over one shared definition
  graph. Export regularizes the existing region, retains its exact boundaries,
  and emits a single even-odd fill with a separate display projection. Import
  checks every definition and closed path before reconstructing the region
  through the existing arrangement; stored roles or success flags are unnecessary.
  The named fresh-process annulus retains its joint selected center through
  further round offsets and exact set identities. Metadata is independent of
  display tolerance. Reached byte/depth limits permit larger retries. A zero-root
  compound imports as canonical empty. Entirely empty documents export with that
  exact graph and a unit display viewport that adds no geometry. Other generated supports and arbitrary
  implicit-region closure remain open. Sharing covers the compound graph, not
  unrelated SVG elements or scalar expression DAGs. Byte and segment limits do
  not bound all scalar decoding, arithmetic, expression growth, memory or
  cancellation.

## Precision, guarantees, and boundaries

Extensions follow the stack-wide [admission protocol](../../docs/admission.md),
including semantic, representational, evidentiary, and computational closure.

Hypercurve separates exact values from decisions about them:

- Coordinates are `Real` values, not an implicit `f64` tolerance model.
- Checked constructors reject malformed or structurally invalid input.
- Every public operation is exact: it takes no policy argument and returns
  `ExactCurveResult<T>`. A decision that exact predicates cannot settle
  is reported as an `ExactCurveError::Blocked` with its operation and reason,
  never guessed. Optional results are `Option`s: the empty region has no
  bounds, and a region without a line/arc boundary has no native view.
- `hypercurve::provisional(|| ...)` evaluates those operations under the
  `APPROXIMATE_512` policy, which may consume Hyperlimit's terminal 512-bit
  interpretation. Its `Provisional<T>` result yields the value through
  `certified()` only when no such terminal was consumed, and otherwise only
  through the explicitly unverified accessors.
- `hypercurve::evaluate_under(policy, || ...)` takes a Hyperlimit
  `PredicatePolicy` for callers whose policy is configuration: `STRICT`
  evaluates exactly (certified), and `APPROXIMATE_512` behaves as
  `provisional`. It suspends enclosing display-preview tolerances and restores
  them afterward; explicit topology policy does not inherit a preview scope.
- `hypercurve::evaluate_with_budget(policy, budget, || ...)` adds an explicit
  `WorkBudget { refinements, promotions }`. Nested synchronous calls share
  actual admitted work; a child cannot replenish its parent. The outer
  `BudgetOutcome::result` returns `BudgetExceeded` on exhaustion, and successful
  results retain their normal certainty. Budget failures are retryable and do
  not become permanent cached capability blockers. Existing exact proofs can
  succeed with zero new work. These counters currently cover instrumented root
  subdivisions and selected algebraic promotions, not every curve algorithm,
  scalar refinement, coefficient growth, allocation, or wall-clock time. The
  [accounting module](../hypersolve/src/work_budget.rs) lists the coverage and
  remaining gaps; the full per-operation bound remains an open target.
- `CurvePreviewOptions` owns finite display tolerances separately. Operations
  called inside `evaluate` use its edge-preview context; its scoped preview
  results are never exact topology or construction provenance.
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
| `triangulation` | no | Exact linear and finite-region triangulation through Hypertri |
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

The browser demo is deployed at <https://timschmidt.github.io/csgrs/hypercurve/>.
It lives in `examples/hypercurve_ui` and is built separately with Trunk. It is
not part of the library API.

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
[Hyperreal](https://github.com/timschmidt/csgrs/tree/main/crates/hyperreal),
[Hyperlimit](https://github.com/timschmidt/csgrs/tree/main/crates/hyperlimit), and
[Hypersolve](https://github.com/timschmidt/csgrs/tree/main/crates/hypersolve), with optional
[Hypertri](https://github.com/timschmidt/csgrs/tree/main/crates/hypertri) integration. The wider
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

`hershey::strings(text, font, size)` returns `Result<Vec<CurveString2>,
TextError>`. Missing glyphs, malformed records and failed strokes retain their
typed causes and never publish partial text. Empty text and blank glyphs succeed
after validating the font's record structure and nonzero size. Negative sizes
remain supported. Whitespace advances horizontally by six times the size;
other unmapped characters are errors. Use `evaluate_under` for an explicit
predicate policy and a receipt for consumed certainty. Checked reservations
diagnose buffer capacity/allocation failures; they are not a total-work,
heap-byte or cancellation bound.

## License and contributing

Licensed under the [MIT License](LICENSE) or the
[Apache License 2.0](LICENSE-APACHE), at your option.

Bug reports should include the smallest exact input, selected features, policy,
operation, and returned blocker or uncertainty evidence. Before proposing a
change, run `cargo fmt --all -- --check`, the relevant focused test, and
`cargo test --all-features`.
