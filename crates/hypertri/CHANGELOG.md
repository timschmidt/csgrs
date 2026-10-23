# Changelog

## [0.5.0] - Unreleased

### Breaking

- Every triangulation and validation entry point now takes a
  `&TriangulationContext` that selects the predicate policy
  (`PredicatePolicy::STRICT` or `PredicatePolicy::APPROXIMATE_512`), and
  returns `TriangulationOutcome<T>`, which pairs the value with the weakest
  `TriangulationCertainty` consumed. This covers `earcut`, `earcut_report`,
  `cdt::delaunay`, `cdt::constrained_delaunay`, `f64::earcut`,
  `f64::delaunay`, `f64::constrained_delaunay`, `triangulate_polygon`,
  `runtime::triangulate_polygon_points`, the D-dimensional
  `delaunay_complex`, and the `validate`,
  `validate_unconstrained_edges_are_delaunay`, `validate_geometric`,
  `validate_geometric_report`, `insert_point_oracle`,
  `validate_bistellar_flip` and `flip_oracle` methods. The fixed internal
  refinement policy of 0.4.0 is gone; under `STRICT`, a decision that would
  need the 512-bit approximation returns `Error::PredicateUndecided`.
- `Point2` and `ExactPoint` are now the shared `hyperlimit::Point2` instead of
  a Hypertri-owned struct.
- `cdt::ConstrainedDelaunayTriangulation` is renamed
  `cdt::ConstrainedTriangulation`.
- The `earcut` module is private. Use the crate-root `earcut`,
  `earcut_report`, `EarcutReport` and `EarcutDiagnostics` instead of
  `earcut::triangulate`, `earcut::triangulate_report` and the module paths.
- `plan_polygon_triangulation` and `PolygonTriangulationPlan` are replaced by
  `triangulate_polygon_with_report`, which triangulates and returns a
  `PolygonTriangulationReport` with public `algorithm`, `quality` and `facts`
  fields.
- Requires `hyperreal` 0.14 and `hyperlimit` 0.5, whose `Real`, `Rational`,
  `Point2` and `PredicatePolicy` appear in this crate's public API.

### Added

- `triangulate_simple_ring` validates one simple, nonzero-area ring without
  holes and returns a `SimpleRingTriangulation` whose triangles keep the input
  winding. Crossings, self-touching, backtracking and repeated points return
  `Error::InvalidInput`.
- `triangulate_simple_ring_around` validates the same rings and fans the
  triangulation around a chosen ring vertex wherever that vertex sees the
  polygon, clipping other ears only in the pockets it cannot see. The result
  does not depend on where the ring starts. Hypermesh uses it to fan the
  polygons its Boolean output reduction retriangulates around their oldest
  vertex.
- `cdt::delaunay_spatial` and `f64::delaunay_spatial` insert points in a
  deterministic BRIO-style order while keeping caller indices. On cocircular
  input they can choose different valid diagonals than `cdt::delaunay`.
- `cdt::constrained_delaunay_convex_hull` triangulates a planar
  straight-line graph over its whole convex hull, keeping both sides of
  closed constraint cycles. It rejects crossing or overlapping constraints
  instead of planarizing them.
- `cdt::constrained_triangulation_convex_hull` returns topology-only
  constrained triangles without Delaunay legalization. Identical protected
  cells with the same points and relative vertex order get identical
  triangle sets.
- `cdt::constrained_triangulation_convex_hull_with_collinear_facts` takes
  `cdt::CollinearPointFacts`, sets of point indices that the caller has
  proved collinear, and uses orientation zero for any three points of one
  set instead of evaluating the determinant; order along the line is still
  decided exactly. The facts carry the certainty of their evidence: facts that
  consumed the approximate terminal are ignored under `STRICT` and make an
  `APPROXIMATE_512` outcome approximate when used. A fact naming an absent
  point is `Error::InvalidInput`.
- `BistellarFlipReportD::undecided_predicate` and
  `TdsGeometricViolationD::undecided_predicate` name the predicate that was
  left undecided.
- The `dispatch-trace` feature forwards to Hyperreal and Hyperlimit dispatch
  instrumentation.

### Changed

- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.4.0 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names.

### Fixed

- `earcut` output now conforms at every input vertex: a triangle edge that
  passes through another input vertex is split there. This restores collinear
  boundary vertices that normalization drops, with or without holes, and
  repairs non-conforming edges between aligned holes, removing T-junctions
  with adjacent geometry that shares those boundary edges. Affected inputs
  produce more triangles.

### Removed

- The public `kernel` module (`Kernel`, `ExactKernel`), the
  `earcut::triangulate_with_kernel` and
  `earcut::triangulate_report_with_kernel` functions, and the public
  `predicates` module. Predicates go through Hyperlimit under the selected
  `TriangulationContext`.
- `RingConvexity`, the `RingInputFacts::signed_area` and
  `RingInputFacts::convexity` fields, and the
  `PolygonInputFacts::all_ring_orientations_certified` and
  `PolygonInputFacts::unknown_convexity_ring_count` methods. Retained input
  facts no longer depend on a predicate policy.

For 0.4.0 and earlier, see [crates.io](https://crates.io/crates/hypertri/versions).
