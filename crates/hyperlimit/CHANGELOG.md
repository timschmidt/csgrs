# Changelog

## [0.5.0] - Unreleased

### Breaking

- Predicates and batch functions now take an explicit `PredicatePolicy` as
  their last argument, and the separate `*_with_policy` functions are folded
  into the unsuffixed names, for example `compare_reals(a, b, policy)`.
  Several report `validate*` methods and the `SupportDop3` and
  `WitnessedSupportDop3` constructors and classifiers also take the policy,
  replacing their `*_with_policy` methods.
- `PredicatePolicy` is now an opaque value with the constants `STRICT` and
  `APPROXIMATE_512`. Its public fields, `Default` implementation and
  `api_semantics` method are removed. `APPROXIMATE_512` allows one terminal
  512-bit approximation after the certified stages are exhausted; such
  results carry the new `Certainty::Approximate` variant, which `STRICT`
  never produces.
- Dimension suffixes are normalized: `orient2d`, `orient3d`, `incircle2d`
  and `insphere3d` and their batch and parallel batch forms become `orient2`,
  `orient3`, `incircle2`, `insphere3`, `orient2_batch` and so on. The
  `Orient2dCase`, `Orient3dCase`, `Incircle2dCase` and `Insphere3dCase`
  aliases become `Orient2Case`, `Orient3Case`, `Incircle2Case` and
  `Insphere3Case`, and the `ExactPredicateKernel` variants are renamed to
  match (`Orient2RationalDet2`, `Orient3RationalDet3`,
  `Incircle2RationalLiftedDet3`, `Insphere3RationalLiftedDet4`).
- Prepared predicate objects are replaced by immediate functions and
  reusable evidence values:
  - `PreparedLine2` by `line2_orientation` with
    `classify_point_line_with_orientation`;
  - `PreparedSegment2` by `segment2_facts` with
    `classify_point_segment_with_facts`, `point_on_segment_with_facts` and
    `classify_segment_intersection_with_facts`;
  - `PreparedTriangle2` by passing a retained `orient2` outcome to
    `classify_point_triangle_with_orientation`, and `PreparedTriangle3` by
    `triangle3_orientation` with `classify_point_triangle3_with_orientation`;
  - `PreparedIncircle2` and `PreparedInsphere3` by `incircle2_evidence` with
    `incircle2_with_evidence` and `insphere3_evidence` with
    `insphere3_with_evidence`;
  - `PreparedPlane3`, `PreparedOrientedPlane3` and `Plane3::prepare` by
    `plane3_evidence`, `oriented_plane3_evidence`,
    `classify_point_plane_with_evidence` and
    `classify_point_oriented_plane_with_evidence`.

  `PreparedSegment3`, `PreparedAabb2`, `PreparedAabb3`,
  `PreparedExplicitSphere3` and `PreparedHalfspaceSystem3` are removed in
  favour of the immediate functions.
- `PreparedPredicateFacts`, `PreparedLiftedPolynomialFacts`,
  `PreparedCircle2Polynomial` and `PreparedSphere3Polynomial` are renamed
  `PredicateFacts`, `LiftedPolynomialFacts`, `Circle2Polynomial` and
  `Sphere3Polynomial`, and `TrianglePlaneClassification` and
  `TrianglePlaneValidationError` are renamed `TrianglePlaneReport` and
  `TrianglePlaneReportValidationError`.
- The `batch`, `classify`, `geometry`, `orient`, `plane`, `predicate`,
  `predicates` and `real` modules are private; import their items from the
  crate root. Only `error` remains a public module.
- Re-exported Hyperreal types use Hyperreal's names: `RealDomainFacts`,
  `RealDomainStatus`, `RealExpressionDegree`, `RealSymbolicDependencyMask`
  and `RealZeroOneMinusOneStatus` become `DomainFacts`, `DomainStatus`,
  `ExpressionDegree`, `SymbolicDependencyMask` and `ZeroOneMinusOneStatus`.
  The `RealFacts` and `RealZeroKnowledge` aliases and the
  `RealPredicateExt::real_facts` and `RealPredicateExt::zero_knowledge`
  methods are removed; use `Real::structural_facts()` and its `zero` field.
- `classify_triangle3_degeneracy` returns
  `PredicateOutcome<TriangleDegeneracy>` instead of `TrianglePredicateReport`,
  and `TriangleDegeneracy::Unknown` is removed.
  `CoplanarTriangleClassification`, `RayTriangleIntersectionReport` and
  `SegmentPlaneIntersection` no longer have a `predicates` field or the
  `all_proof_producing`/`projection_proof_producing` methods.
- `proper_segment_intersection_point` is replaced by
  `construct_line_intersection_point`, which only constructs the intersection
  of the supporting lines and returns `Option<Point2>`. Classify the segments
  with `classify_segment_intersection` first.
- `Real`, `Point2`, `Point3` and the other re-exported types come from
  Hyperreal 0.14 and Hyperlattice 0.7.

### Added

- `classify_real_sign` and `classify_real_sign_pair` decide the sign of one
  or two `Real` values, and `reciprocal_real` constructs a reciprocal after
  deciding that the denominator is nonzero, keeping that decision's
  certainty and stage.
- `compare_point_segment2_distance_squared` and
  `compare_point_triangle3_distance_squared` compare exact squared distances
  to a 2D segment and a 3D triangle.
- Predicates for boxes whose bounds are already ordered:
  `ordered_aabb2s_intersect_coordinates`, `point_in_ordered_aabb2_coordinates`,
  `ordered_aabb3s_intersect`, `ordered_aabb3s_intersect_coordinates`,
  `ordered_aabb3_contains` and `point_in_ordered_aabb3_relative_interior`.
- `classify_ball_sign`, which, unlike `certified_ball_sign`, reports an
  invalid radius (`RefinementNeed::Unsupported`) separately from an
  inconclusive enclosure.
- A `serde` feature that enables serialization of the re-exported `Point2`
  through `hyperreal/serde` and `hyperlattice/serde`.

### Changed

- `point2_equal` and `point3_equal` decide `false` from any certified unequal
  coordinate, even when another coordinate is undecided. 0.4.0 compared
  lexicographically and could return `Unknown` in that case.
- Sign resolution tries Hyperreal's exact sign certificates for nested
  quadratic and square-root tower expressions before refinement or
  approximation.
- Predicate dispatch paths were reworked for speed. Each retained change and
  its measurement is recorded in [`PERFORMANCE.md`](PERFORMANCE.md).
- Declares a minimum supported Rust version (MSRV) of 1.95.0; 0.4.0 declared
  none.
- Development moved to the
  [csgrs repository](https://github.com/timschmidt/csgrs), which the
  `repository` field now names.

### Fixed

- The package includes the Apache-2.0 license text (`LICENSE-APACHE`) for its
  declared `MIT OR Apache-2.0` license; 0.4.0 shipped only the MIT text.

### Removed

- The session and construction-provenance APIs: `ExactGeometrySession`,
  `ConstructionCertificate`, `ConstructionDependencies`,
  `ConstructionFreshness`, `ConstructionVersion`, `VersionedFacts`,
  `VersionedPredicateReport`, `VersionedPrepared`, `CachePayoff`,
  `CachedApproximateView`, `ConstructionProvenance`, `SourceProvenance`,
  `MeshSource`, `ApproximationPolicy` and
  `ConstructionProvenanceValidationError`.
- Per-call provenance reports: `PredicateReport`, `PredicateUse`,
  `PredicateCertificate`, `PredicatePrecisionStage`, `PredicateApiSemantics`,
  and the `*_report` forms of orientation, in-circle, in-sphere, scalar
  comparison, point ordering and equality, and the certified ball and
  interval sign filters. `PredicateOutcome` still records the certainty and
  stage of each decision.
- The binary64 orientation functions `orient2d_f64` and
  `orient2d_f64_report` and their policy variants.

For 0.4.0 and earlier, see [crates.io](https://crates.io/crates/hyperlimit/versions).
