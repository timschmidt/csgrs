//! Hyperreal-backed exact predicates with structural Real awareness.
//!
//! `hyperlimit` is intentionally positioned between Real semantics and
//! application geometry code. It asks `hyperreal::Real` for facts such as known
//! sign, exact zero, rational structure, and refinement capability before
//! escalating a predicate.
//!
//! Predicate exactness means the reported classification has an exact or
//! certified decision path, not that all input expressions were eagerly
//! canonicalized first. This follows Yap's exact geometric computation model:
//! filters may exploit preserved structure, but unresolved cases return
//! explicit uncertainty or escalate through exact hyperreal refinement instead
//! of falling back to primitive-float tolerances. See Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry*, 1997, pp. 3-23.

mod trace;
pub(crate) use trace::trace_dispatch;

pub mod batch;
pub mod classify;
pub mod error;
pub mod geometry;
pub mod orient;
pub mod plane;
pub mod predicate;
pub mod predicates;
pub mod provenance;
pub mod real;
mod resolve;
pub mod session;

pub use hyperreal::{
    CertifiedRealSign, DomainFacts as RealDomainFacts, DomainStatus as RealDomainStatus,
    ExpressionDegree as RealExpressionDegree, RationalStorageClass, Real,
    RealExactSetDenominatorKind, RealExactSetDyadicExponentClass, RealExactSetSignPattern,
    RealSignCertificate, SymbolicDependencyMask as RealSymbolicDependencyMask,
    ZeroOneMinusOneStatus as RealZeroOneMinusOneStatus,
};

pub use batch::{
    CircleLine2Case, CircleSegment2Case, Incircle2dCase, Insphere3dCase, Orient2dCase,
    Orient3dCase, PointPlaneCase, RayTriangle3IntersectionCase, Segment3IntersectionCase,
    SegmentTriangle3IntersectionCase, classify_circle_line2_batch,
    classify_circle_line2_batch_with_policy, classify_circle_segment2_batch,
    classify_circle_segment2_batch_with_policy, classify_point_line_batch,
    classify_point_line_batch_with_policy, classify_point_oriented_plane_batch,
    classify_point_oriented_plane_batch_with_policy, classify_point_plane_batch,
    classify_point_plane_batch_with_policy, classify_ray_triangle3_intersection_batch,
    classify_ray_triangle3_intersection_batch_with_policy,
    classify_segment_triangle3_intersection_batch,
    classify_segment_triangle3_intersection_batch_with_policy,
    classify_segment3_intersection_batch, classify_segment3_intersection_batch_with_policy,
    incircle2d_batch, incircle2d_batch_with_policy, insphere3d_batch, insphere3d_batch_with_policy,
    orient2d_batch, orient2d_batch_with_policy, orient3d_batch, orient3d_batch_with_policy,
};
#[cfg(feature = "parallel")]
pub use batch::{
    classify_circle_line2_batch_parallel, classify_circle_line2_batch_parallel_with_policy,
    classify_circle_segment2_batch_parallel, classify_circle_segment2_batch_parallel_with_policy,
    classify_point_line_batch_parallel, classify_point_line_batch_parallel_with_policy,
    classify_point_oriented_plane_batch_parallel,
    classify_point_oriented_plane_batch_parallel_with_policy, classify_point_plane_batch_parallel,
    classify_point_plane_batch_parallel_with_policy,
    classify_ray_triangle3_intersection_batch_parallel,
    classify_ray_triangle3_intersection_batch_parallel_with_policy,
    classify_segment_triangle3_intersection_batch_parallel,
    classify_segment_triangle3_intersection_batch_parallel_with_policy,
    classify_segment3_intersection_batch_parallel,
    classify_segment3_intersection_batch_parallel_with_policy, incircle2d_batch_parallel,
    incircle2d_batch_parallel_with_policy, insphere3d_batch_parallel,
    insphere3d_batch_parallel_with_policy, orient2d_batch_parallel,
    orient2d_batch_parallel_with_policy, orient3d_batch_parallel,
    orient3d_batch_parallel_with_policy,
};
pub use classify::{
    Aabb2Intersection, Aabb2PointLocation, Aabb3Intersection, Aabb3PointLocation,
    AabbSphereIntersection, CircleLineRelation, CircleSegmentRelation, ClosedIntervalIntersection,
    ConvexPointLocation, HalfspaceFeasibility, LineSide, PlaneAabbRelation, PlaneSegmentRelation,
    PlaneSide, PlaneTriangleRelation, PointSegmentLocation, RayTriangleIntersection,
    RealIntervalLocation, RingConvexity, RingPointLocation, Segment3Intersection,
    SegmentIntersection, SegmentTriangleIntersection, SphereIntersection, SpherePointLocation,
    SupportDopRelation, TetrahedronLocation, Triangle3Location, TriangleLocation,
    TriangleTriangleIntersection,
};
pub use geometry::{
    Aabb2Facts, CoordinateAxis2, HomogeneousLine3, HomogeneousPoint3, Plane3Facts,
    Point2DisplacementFacts, Point2Facts, Point3Facts, PointSharedScaleView, Segment2Facts,
    Triangle2Facts, TriangleEdge2, aabb2_facts, classify_homogeneous_point_plane,
    classify_homogeneous_point_plane_with_policy, intersect_homogeneous_line_plane,
    intersect_three_planes, intersect_two_planes, point2_displacement_facts, segment2_facts,
    triangle2_facts,
};
pub use orient::{
    Point2, Point3, PreparedCircle2Polynomial, PreparedIncircle2, PreparedInsphere3,
    PreparedLiftedPolynomialFacts, PreparedLine2, PreparedPredicateFacts,
    PreparedSphere3Polynomial, classify_point_line, classify_point_line_with_policy, incircle2d,
    incircle2d_report, incircle2d_report_with_policy, incircle2d_with_policy, insphere3d,
    insphere3d_report, insphere3d_report_with_policy, insphere3d_with_policy, orient2d,
    orient2d_f64, orient2d_f64_report, orient2d_f64_report_with_policy, orient2d_f64_with_policy,
    orient2d_report, orient2d_report_with_policy, orient2d_with_policy, orient3d, orient3d_report,
    orient3d_report_with_policy, orient3d_with_policy,
};
pub use plane::{
    Plane3, PreparedOrientedPlane3, PreparedPlane3, classify_plane_aabb3,
    classify_plane_aabb3_with_policy, classify_plane_segment, classify_plane_segment_with_policy,
    classify_plane_triangle, classify_plane_triangle_with_policy, classify_point_oriented_plane,
    classify_point_plane,
};
pub use predicate::{
    Certainty, DeterminantScheduleHint, Escalation, ExactPredicateKernel, PredicateApiSemantics,
    PredicateCertificate, PredicateOutcome, PredicatePolicy, PredicatePrecisionStage,
    PredicateReport, PredicateUse, RefinementNeed, Sign, SignKnowledge,
};
pub use predicates::aabb::{
    PreparedAabb2, PreparedAabb3, aabb2s_intersect, aabb2s_intersect_with_policy, aabb3s_intersect,
    aabb3s_intersect_with_policy, classify_aabb2_intersection,
    classify_aabb2_intersection_with_facts, classify_aabb2_intersection_with_policy,
    classify_aabb2_intersection_with_policy_and_facts, classify_aabb3_intersection,
    classify_aabb3_intersection_with_policy, classify_point_aabb2,
    classify_point_aabb2_with_policy, classify_point_aabb3, classify_point_aabb3_with_policy,
    point_in_aabb2, point_in_aabb2_with_policy, point_in_aabb3, point_in_aabb3_with_policy,
    point_in_triangle2_aabb, point_in_triangle2_aabb_with_policy,
};
pub use predicates::convex::{
    classify_point_convex_planes3, classify_point_convex_planes3_with_policy,
    classify_point_convex_polygon2, classify_point_convex_polygon2_with_policy,
};
pub use predicates::coplanar::{
    CoplanarProjection, CoplanarTriangleClassification, CoplanarTriangleRelation,
    CoplanarTriangleValidationError, TriangleDegeneracy, TrianglePredicateReport,
    choose_coplanar_projection, classify_coplanar_triangle_points, classify_coplanar_triangles,
    classify_triangle3_degeneracy, derive_coplanar_triangle_relation, orient2d_value,
    project_point3, project_triangle3, projected_line_parameter3, projected_polygon_area2_sign,
    projected_polygon_area2_value, projected_segment_parameter3,
};
pub use predicates::distance::{
    PreparedExplicitSphere3, classify_aabb3_sphere_intersection,
    classify_aabb3_sphere_intersection_with_policy, classify_circle_line2,
    classify_circle_line2_with_policy, classify_circle_segment2,
    classify_circle_segment2_with_policy, classify_point_sphere3,
    classify_point_sphere3_with_policy, classify_sphere3_intersection,
    classify_sphere3_intersection_with_policy, compare_point_line3_distance_squared,
    compare_point_line3_distance_squared_with_policy, compare_point_plane_distance_squared,
    compare_point_plane_distance_squared_with_policy, compare_point_segment3_distance_squared,
    compare_point_segment3_distance_squared_with_policy, compare_point2_distance_squared,
    compare_point2_distance_squared_with_policy, compare_point3_distance_squared,
    compare_point3_distance_squared_with_policy,
};
pub use predicates::dop::{
    SupportDop3, SupportSlab3, support_dop3_from_points, support_dop3_from_points_with_policy,
};
pub use predicates::filters::{
    certified_ball_sign, certified_ball_sign_report, certified_ball_sign_report_with_policy,
    certified_ball_sign_with_policy, certified_interval_sign, certified_interval_sign_report,
    certified_interval_sign_report_with_policy, certified_interval_sign_with_policy,
};
pub use predicates::halfspace::{
    HalfspaceFeasibilityReport, HalfspaceInfeasibilityCertificate, PreparedHalfspaceSystem3,
    classify_halfspace_feasibility3, classify_halfspace_feasibility3_with_policy,
};
pub use predicates::interval::{
    classify_closed_interval_intersection, classify_closed_interval_intersection_with_policy,
    classify_real_closed_interval, classify_real_closed_interval_with_policy,
    closed_intervals_intersect, closed_intervals_intersect_with_policy, real_in_closed_interval,
    real_in_closed_interval_with_policy,
};
pub use predicates::nd::{
    affine_independent_d, affine_independent_d_with_policy, insphere_d, insphere_d_with_policy,
    orient_d, orient_d_with_policy,
};
pub use predicates::order::{
    compare_point2_lexicographic, compare_point2_lexicographic_report,
    compare_point2_lexicographic_report_with_policy, compare_point2_lexicographic_with_policy,
    compare_reals, compare_reals_report, compare_reals_report_with_policy,
    compare_reals_with_policy, point2_equal, point2_equal_report, point2_equal_report_with_policy,
    point2_equal_with_policy, real_clamp, real_clamp_with_policy, real_ge, real_ge_with_policy,
    real_le, real_le_with_policy, real_max, real_max_with_policy, real_min, real_min_with_policy,
};
pub use predicates::ring::{
    Ring2Facts, classify_point_indexed_ring_even_odd,
    classify_point_indexed_ring_even_odd_with_policy, classify_point_ring_even_odd,
    classify_point_ring_even_odd_with_policy, indexed_ring_area_sign,
    indexed_ring_area_sign_with_policy, indexed_ring_convexity, indexed_ring_convexity_with_policy,
    indexed_ring2_facts, indexed_ring2_facts_with_policy, point_in_indexed_ring_even_odd,
    point_in_indexed_ring_even_odd_with_policy, point_in_ring_even_odd,
    point_in_ring_even_odd_with_policy, ring_area_sign, ring_area_sign_with_policy, ring_convexity,
    ring_convexity_with_policy, ring2_facts, ring2_facts_with_policy,
};
pub use predicates::segment::{
    PreparedSegment2, PreparedSegment3, classify_point_segment, classify_point_segment_with_facts,
    classify_point_segment_with_policy, classify_point_segment_with_policy_and_facts,
    classify_point_segment3, classify_point_segment3_with_policy, classify_segment_intersection,
    classify_segment_intersection_with_facts, classify_segment_intersection_with_policy,
    classify_segment_intersection_with_policy_and_facts, classify_segment3_intersection,
    classify_segment3_intersection_with_policy, point_on_segment, point_on_segment_with_facts,
    point_on_segment_with_policy, point_on_segment_with_policy_and_facts, point_on_segment3,
    point_on_segment3_with_policy, proper_segment_intersection_point,
    proper_segment_intersection_point_with_policy,
};
pub use predicates::segment_plane::{
    SegmentPlaneConstructionFailure, SegmentPlaneIntersection, SegmentPlaneParameterRatio,
    SegmentPlaneRelation, SegmentPlaneValidationError,
    construct_segment_plane_crossing_from_values, interpolate_point3,
    intersect_segment_with_oriented_plane, intersect_segment_with_plane,
    intersect_segment_with_plane_values, point_plane_value, segment_parameter_from_axis,
};
pub use predicates::triangle::{
    PreparedTriangle2, PreparedTriangle3, RayTriangleIntersectionReport, RayTriangleParameterRatio,
    RayTriangleValidationError, SegmentTriangleIntersectionReport, SegmentTriangleValidationError,
    classify_point_tetrahedron, classify_point_tetrahedron_with_policy, classify_point_triangle,
    classify_point_triangle_with_facts, classify_point_triangle_with_policy,
    classify_point_triangle_with_policy_and_facts, classify_point_triangle3,
    classify_point_triangle3_with_policy, classify_ray_triangle3_intersection,
    classify_ray_triangle3_intersection_report,
    classify_ray_triangle3_intersection_report_with_policy,
    classify_ray_triangle3_intersection_with_policy, classify_segment_triangle3_intersection,
    classify_segment_triangle3_intersection_report,
    classify_segment_triangle3_intersection_report_with_policy,
    classify_segment_triangle3_intersection_with_policy, triangle3_winding_normal_sign,
    triangle3_winding_normal_sign_with_policy,
};
pub use predicates::triangle_triangle::{
    TriangleTriangleClassification, TriangleTriangleValidationError, classify_triangle_triangle3,
    classify_triangle_triangle3_points_with_policy, classify_triangle_triangle3_with_policy,
};
pub use provenance::{
    ApproximationPolicy, ConstructionProvenance, ConstructionProvenanceValidationError, MeshSource,
    SourceProvenance,
};
pub use real::{RealFacts, RealPredicateExt, RealZeroKnowledge};
pub use session::{
    CachePayoff, CachedApproximateView, ConstructionCertificate, ConstructionDependencies,
    ConstructionFreshness, ConstructionVersion, ExactGeometrySession, VersionedFacts,
    VersionedPredicateReport, VersionedPrepared,
};
