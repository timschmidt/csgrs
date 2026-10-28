# Changelog

## [0.4.0] - Unreleased

- Export and replay exact empty SVG documents through the native zero-root
  graph, using a unit display viewport. Empty topology retains consumed policy
  through Boolean identities, affine transforms and finite projection; strict
  consumers continue to reject inherited approximate topology.

- Add independent selected-fiber predicate tests for source identity, repeated
  roots, degree drops, tiny nonzero residues, nonrational coefficients and
  certainty-preserving bounded retries.

- Clarify that `BezierLineImageFitRelation::NotLine` includes collapsed images
  and does not certify nonlinearity. Independent circle, point, retracing,
  singular-chart and policy-receipt tests preserve the existing fit behavior.

- Retained formula roots compare locally with polynomial roots over independently
  rebuilt equivalent selected-source fields and radical towers. Exact nearby
  roots can be ordered without global projection; unsupported field combinations
  retain the existing typed fallback.

- Exact box containment admits nonmonotone rational Bézier/NURBS spans and
  selected rational trims within their unit source charts. It proves the
  denominator on the active range and compares stationary selected points
  without forcing Cartesian reconstruction. Conservative `bounds()` is unchanged.

- Finite sampling refines retained Bézier parameter intervals in source
  coordinates before selecting their midpoints, avoiding displacement from
  coarse root isolators. Sampling remains an explicit fidelity boundary.

- Affine transforms of terminal-qualified loops retain their boundary evidence
  without requiring a strictly closed native contour accelerator. Translation,
  reflection and nonsingular scaling preserve the consumed-policy receipt.

- `CurveRegionProfile2::sample_to_linear_contours` retains native line members,
  shared native endpoints and classified hole ownership while sampling curved
  interiors. It is an explicit sampled modeling boundary; finite export APIs
  continue to project every coordinate to `f64`.

### Breaking

- `CurveError` gains `IncompatiblePredicatePolicy`; exhaustive matches must
  handle it. Retained chord and selected-fiber replay validators use this cause
  instead of a generic topology string. Principal operations report it as
  `ExactCurveError::Blocked(Predicate)`, preserving operation/family context.
- Policy-taking operations, such as `Contour2::{winding_number,
  classify_point, intersect_contour}`, `LineSeg2::classify_point`,
  `Aabb2::from_*` and the Bézier flattening, length and moment methods, no
  longer take a `&CurvePolicy` argument and evaluate under the strict
  predicate policy. Most of their decision results changed from
  `Classification<T>` to `ExactCurveResult<T>`, which reports an undecided
  predicate as `ExactCurveError::Blocked`. `CurvePolicy`, `NumericMode` and
  `Tolerance` are removed. Use `provisional` or `evaluate_under` to select a
  predicate policy, and `CurvePreviewOptions` for display tolerances.
- `Region2`, `RegionView2`, `BezierRegion2`, `BezierRetainedRegion2` and
  their boundary-loop types are replaced by `CurveRegion2`, the single
  filled-region type for every curve family. `Region2::from_material_contours`
  becomes the fallible `CurveRegion2::try_from_native_material_contours`.
  `Region2::boolean_region(other, op, fill_rule, policy)` becomes
  `CurveRegion2::boolean_region(other, op)`, with the fill rule chosen at
  construction. The staged Boolean pipeline types (`RegionIntersectionSet`,
  `RegionFragmentSet`, `BooleanFragmentClassification`,
  `BooleanFragmentSelection`, `BooleanBoundaryLoopSet` and related) are no
  longer public.
- `PolynomialBSplineCurve2` is replaced by `PolynomialSplineCurve2`, and
  `RationalQuadraticBSplineCurve2` and `RationalBSplineCurve2` by
  `NurbsCurve2`. Bézier span extraction moves to
  `Curve2::native_bezier_fragments`.
- The staged Bézier offset API (`offset_preflight`, `offset_left_staged`,
  `offset_right_staged`, `BezierOffsetCandidate2`, `BezierOffsetPreflight2`,
  `BezierOffsetRisk`) and the curve-string and contour offset methods
  (`CurveString2::offset_outline*`, `offset_left_with_line_joins`,
  `offset_left_checked`) are replaced by `parallel_left` and `parallel_right`
  on the Bézier types, `CurveRegion2::offset` and `CurveRegion2::stroke_path`.
- `Classification` and `UncertaintyReason` are now re-exports of
  `hypersolve::classification`, and `UncertaintyReason` has a new
  `BudgetExhausted` variant. The `ZeroStatus` alias is removed; use
  `ZeroKnowledge`.
- Exhaustive public types changed. `CurveError` has new variants, including
  NURBS interpolation, arc sweep, curve path, offset and corner option, and
  periodic spline errors. `ContourPointIntersection`,
  `ContourOverlapIntersection`, `ContourUncertainIntersection` and
  `CurveStringIntersection` gained `a_segment_kind` and `b_segment_kind`;
  `CurveStringIntersection` and `ContourFragment` gained source segment
  endpoints; `LineArcIntersectionPoint` gained `arc_param` and
  `ArcArcIntersectionPoint` gained `a_param` and `b_param`.
- `Point2::{new, x, y}`, the `Aabb2` extent accessors, several `CircularArc2`
  and `Segment2` accessors, and `LineSeg2::new_unchecked` and
  `Contour2::new_unchecked` are no longer `const fn`.
  `BezierArcLengthParameterRegion2::parameter_span` now returns the exact
  `(start, end)` pair instead of a `BezierMonotoneSpan`.

### Added

- `Curve2`, an exact curve type covering lines, arcs, polynomial and rational
  Bézier curves, B-splines, NURBS and generated curves such as analytic
  parallels. It supports evaluation, derivatives, splitting, subcurves,
  reversal, similarity transforms, bounds, point location, curve/curve
  intersection (`intersect_curve`, `intersection_topology`) and trimming
  against a region. `CurvePath2` represents connected paths, and
  `CurvePoint2` and `CurveParameter2` keep exact algebraic points and
  parameters without rounding.
- Graph branches of a `PolynomialCurveSet2` intersect authored lines with
  rational endpoints through `Curve2::intersect_curve` and
  `Curve2::intersection_topology`, in either operand order and after
  restriction or reversal. Every crossing and tangency is reported, with its
  exact tangent cross sign where certified. The line parameter retains the
  branch's correlated fiber point, and both operands split in their own
  parameter charts. A line that is a component of the zero set, other
  polynomial pairs, mapped branches and filled regions with polynomial
  boundaries still report typed unsupported blockers.
- `Curve2::box_containment` and `CurvePath2::box_containment` decide whether
  a curve lies in a closed axis-aligned box. They return
  `CurveBoxContainment2::Contained`, or `Escapes` with a `CurveBoxEscape2`
  naming the curve, axis, `BoxBoundary2` and an exact curve point outside the
  box. `bounds()` is a conservative enclosure, the control hull for
  polynomial Bézier spans, and remains the fast path; sides it does not settle
  are decided from exact coordinate extrema. The cubic with controls
  `(8, 0), (9, 3), (11, 3), (12, 0)` has hull height 3 and curve height 9/4,
  so the box `y <= 5/2` contains it however the curve is split, and `y <= 2`
  is left at `(10, 9/4)`. Curves without exact extremum candidates report a
  typed blocker when the enclosure is not enough.
- `PolynomialSplineCurve2` and `NurbsCurve2` support periodic construction,
  knot insertion, splitting and subcurves; `NurbsCurve2` also supports knot
  removal and degree elevation. `NurbsCurve2::interpolate_global`,
  `interpolate_uniform`, `interpolate_chord_length`, `interpolate_centripetal`
  and `interpolate_with_parameters_and_knots` provide exact global
  interpolation. `RationalBezier2` is a rational Bézier curve of any positive
  degree stored as homogeneous controls (`HomogeneousControl2`), including
  zero-weight controls.
- `CurveRegion2` construction from closed boundary paths under an explicit
  `FillRule` (`try_from_boundary_paths`), from native contours or from
  unordered segments, with point classification (also batched), signed and
  filled area, and similarity and affine transforms.
- Regularized region Booleans: `CurveRegion2::boolean_region` with
  `BooleanOp::{Union, Intersection, Difference, Xor}`, `boolean_regions`
  for all four results from one evaluation, and `intersect_region` for the
  contacts and overlaps between region boundaries.
- Region offsets and strokes: `CurveRegion2::offset` with
  `OffsetCornerStyle2`, `CurveRegion2::stroke_path` with `OffsetCap`, and
  exact Bézier parallel curves (`BezierParallel2`) with certified
  approximations (`approximate_parallel_blend2d_certified`).
- Corner and path editing: `CurvePath2::fillet_vertex` (exact radius with
  optional center or contact constraints) and
  `CurvePath2::chamfer_vertex_by_setbacks` return finite sets of exact
  solutions (`CurveCornerSolutions2`), and `CurveString2` gains
  `link_connected_endpoints`, `connect_endpoints_with_line`,
  `merge_adjacent_collinear_lines`, `remove_adjacent_reversed_duplicates`,
  `trim_between_parameters` and `trim_between_points`.
- `Contour2::straight_skeleton` and `CurvePath2::straight_skeleton` for exact
  straight skeletons of supported line/arc boundaries, and
  `translation_obstacle_convex` for the translational configuration-space
  obstacle of supported convex contours. Unsupported cases return typed
  blockers.
- `evaluate_with_budget` with `WorkBudget`, `BudgetOutcome` and
  `BudgetExceeded` limits instrumented refinement and promotion work and
  reports exhaustion as a typed error. It does not bound wall-clock time or
  memory.
- Optional `svg` and `hershey` features. `svg` adds SVG import and export
  (`import_svg_document`, `export_svg_document`, `parse_svg_path_data`,
  `SvgGeometry2`) using native line, arc and Bézier path commands where
  possible, with a `data-hypercurve-path` attribute that preserves exact
  values for round-trips. `hershey` adds compiled Hershey single-stroke
  fonts rendered to `CurveString2` text by `hershey::strings`.
- `marching_squares` builds the filled `CurveRegion2` of a marching-squares
  polygonal approximation of a sampled scalar grid, and
  `triangulate_linear_contours` (`triangulation` feature) triangulates
  line-only region profiles through Hypertri while keeping exact vertices.

### Changed

- Depends on hyperreal 0.14, hyperlimit 0.5 and hypersolve 0.4. Re-exported
  types such as `Real` and `PredicatePolicy` come from those releases.
- License is now `MIT OR Apache-2.0` (was `Apache-2.0`).
- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.3.0 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names.
- Region Booleans and regularization prepare a Bézier carrier's curve for
  the pair kernels only when a carrier pair that survives the broad phase,
  or the carrier's own self-intersection query, reads it. A Boolean of a
  large region with a small one no longer prepares every carrier of the
  large region. On the Easyduino boards this removes 99% of those
  preparations, 0.7–2.7% of all allocations and up to 5.6% of the peak of
  live bytes, and 6% of the instructions of nano's CAM round-trip audit;
  results are unchanged.

### Fixed

- Two retained local polynomial roots whose isolating brackets still
  overlap after refinement are now ordered by one sign of either defining
  polynomial at the other root, in their own coefficient fields. Equal roots
  no longer go through 512 bisection steps and a global algebraic projection
  of both, and two distinct roots 2^-603 apart, which that route left
  undecided, are now ordered. A local root also keeps the native algebraic
  roots it has been proved equal to. Rebuilding the bevel stroke of a
  chamfered quadratic path from its exported boundary now completes in
  about a minute; it did not finish within forty minutes. With faster
  remainders in Hypersolve it now takes about 15 s.
- Filleting a vertex that an earlier fillet left as a tangent junction now
  completes, in three to five minutes; it did not finish within forty. Signs of
  wide polynomials at a selected parameter, once certified nonzero, are
  refined with interval Horner and Bernstein enclosures over integer common
  denominators instead of normalized rationals. Every enclosure decides as
  before.
- Region construction and Booleans no longer fail with a `RealSign` blocker
  when two contacts lie on one circle and one supporting line but carry
  independent square roots. The line meets the circle in at most two points,
  and the side of the circle center's projection on the line decides whether
  the contacts are the same one. The Easyduino STM32F103 copper aggregate,
  where a route edge and a zero-clearance zone's fragment of that edge cross
  a pad's corner arc at one point, now completes.

### Removed

- Prepared query views (`PreparedLineSeg2`, `PreparedCircularArc2`,
  `PreparedSegment2`, `PreparedCurveStringView2`, `PreparedContourView2`,
  `PreparedRegionView2`) and the `prepare_topology_queries` methods.
- The public Bézier topology, tangent-order, split, arrangement and
  retained-overlap API: `relation_to_*`, `graph_order_*`, `monotone_spans`,
  `cusp_classification`, `inflection_classification`, the
  `compare_algebraic_*` functions, and the `BezierArrangement*`,
  `BezierSplitFragment2`, `BezierRetainedOverlap*` and algebraic-image
  carrier types. Curve/curve and line/curve contacts are available through
  `Curve2::intersect_curve`.
- Retained planar face and p-curve types (`RetainedPlanarFace2`,
  `RetainedPlanarPcurve2` and related), `CurveError::InvalidPlanarFace`, and
  the import provenance records (`RetainedImportRecord2`,
  `FiniteCurveStringImport2`, `FiniteContourImport2`) with the
  `import_finite_*` methods. `CurveString2::from_finite_line_string` and
  `Contour2::from_finite_ring` remain.
- The `predicates` feature, whose functionality is now always enabled (the
  default feature set is now empty), and the `serde` feature, which had no
  effect. Remove them from dependency declarations.

For 0.3.0 and earlier, see [crates.io](https://crates.io/crates/hypercurve/versions).
