//! Hyperreal-backed exact predicates with structural Real awareness.
//!
//! `hyperlimit` is intentionally positioned between Real semantics and
//! application geometry code. It asks `hyperreal::Real` for facts such as known
//! sign, exact zero, rational structure, and refinement capability before
//! escalating a predicate.
//!
//! Predicate exactness means the reported classification has an exact or
//! certified decision path, not that all input expressions were eagerly
//! canonicalized first. Following the exact geometric computation model,
//! filters may exploit preserved structure, but unresolved cases return
//! explicit uncertainty or escalate through exact hyperreal refinement instead
//! of falling back to primitive-float tolerances.

mod trace;
pub(crate) use trace::trace_dispatch;

mod batch;
mod classify;
pub mod error;
mod geometry;
mod orient;
mod plane;
mod predicate;
mod predicates;
mod provenance;
mod real;
mod resolve;
mod session;

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
    SegmentTriangle3IntersectionCase, classify_circle_line2_batch, classify_circle_segment2_batch,
    classify_point_line_batch, classify_point_oriented_plane_batch, classify_point_plane_batch,
    classify_ray_triangle3_intersection_batch, classify_segment_triangle3_intersection_batch,
    classify_segment3_intersection_batch, incircle2d_batch, insphere3d_batch, orient2d_batch,
    orient3d_batch,
};
#[cfg(feature = "parallel")]
pub use batch::{
    classify_circle_line2_batch_parallel, classify_circle_segment2_batch_parallel,
    classify_point_line_batch_parallel, classify_point_oriented_plane_batch_parallel,
    classify_point_plane_batch_parallel, classify_ray_triangle3_intersection_batch_parallel,
    classify_segment_triangle3_intersection_batch_parallel,
    classify_segment3_intersection_batch_parallel, incircle2d_batch_parallel,
    insphere3d_batch_parallel, orient2d_batch_parallel, orient3d_batch_parallel,
};
pub use classify::{
    Aabb2Intersection, Aabb2PointLocation, Aabb3Intersection, Aabb3PointLocation,
    AabbSphereIntersection, CircleLineRelation, CircleSegmentRelation, ClosedIntervalIntersection,
    ConvexPointLocation, HalfspaceFeasibility, LineSide, PlaneAabbRelation, PlaneSegmentRelation,
    PlaneSide, PlaneTriangleRelation, PointSegmentLocation, RayTriangleIntersection,
    RealIntervalLocation, RingConvexity, RingPointLocation, Segment3Intersection,
    SegmentIntersection, SegmentTriangleIntersection, SphereIntersection, SpherePointLocation,
    SupportDopPlaneRelation, SupportDopRelation, TetrahedronLocation, Triangle3Location,
    TriangleLocation, TriangleTriangleIntersection,
};
pub use geometry::{
    Aabb2Facts, CoordinateAxis2, HomogeneousLine3, HomogeneousPoint3, Plane3Facts,
    Point2DisplacementFacts, Point2Facts, Point3Facts, PointSharedScaleView, Segment2Facts,
    Triangle2Facts, TriangleEdge2, aabb2_facts, classify_homogeneous_point_plane,
    intersect_homogeneous_line_plane, intersect_three_planes, intersect_two_planes,
    point2_displacement_facts, segment2_facts, triangle2_facts,
};
pub use orient::{
    Point2, Point3, PreparedCircle2Polynomial, PreparedIncircle2, PreparedInsphere3,
    PreparedLiftedPolynomialFacts, PreparedLine2, PreparedPredicateFacts,
    PreparedSphere3Polynomial, classify_point_line, incircle2d, incircle2d_report, insphere3d,
    insphere3d_report, orient2d, orient2d_report, orient2d_with_policy, orient3d, orient3d_report,
};
pub use plane::{
    Plane3, PlaneAabbReport, PlaneAabbReportValidationError, PreparedOrientedPlane3,
    PreparedPlane3, TrianglePlaneClassification, TrianglePlaneRelation,
    TrianglePlaneValidationError, classify_plane_aabb3, classify_plane_aabb3_report,
    classify_plane_segment, classify_plane_triangle, classify_point_oriented_plane,
    classify_point_plane, classify_triangle_against_oriented_plane,
    triangle_plane_relation_from_sides,
};
pub use predicate::{
    Certainty, DeterminantScheduleHint, Escalation, ExactPredicateKernel, PredicateApiSemantics,
    PredicateCertificate, PredicateOutcome, PredicatePolicy, PredicatePrecisionStage,
    PredicateReport, PredicateUse, RefinementNeed, Sign, SignKnowledge,
};
pub use predicates::aabb::{
    PreparedAabb2, PreparedAabb3, aabb2s_intersect, aabb3s_intersect, classify_aabb2_intersection,
    classify_aabb2_intersection_with_facts, classify_aabb3_intersection, classify_point_aabb2,
    classify_point_aabb3, point_in_aabb2, point_in_aabb3, point_in_triangle2_aabb,
};
pub use predicates::convex::{classify_point_convex_planes3, classify_point_convex_polygon2};
pub use predicates::coplanar::{
    CoplanarProjection, CoplanarTriangleClassification, CoplanarTriangleRelation,
    CoplanarTriangleValidationError, TriangleDegeneracy, TrianglePredicateReport,
    ccw_projected_turn_less, choose_coplanar_projection, classify_coplanar_triangle_points,
    classify_coplanar_triangles, classify_point_projected_triangle3, classify_triangle3_degeneracy,
    derive_coplanar_triangle_relation, intersect_segment_with_projected_line3, midpoint3,
    orient2d_value, project_point3, project_triangle3, projected_line_parameter3,
    projected_polygon_area2_abs_value, projected_polygon_area2_sign, projected_polygon_area2_value,
    projected_segment_parameter3, projected_vector3,
};
pub use predicates::distance::{
    PreparedExplicitSphere3, classify_aabb3_sphere_intersection, classify_circle_line2,
    classify_circle_segment2, classify_point_sphere3, classify_sphere3_intersection,
    compare_point_line3_distance_squared, compare_point_plane_distance_squared,
    compare_point_segment3_distance_squared, compare_point2_distance_squared,
    compare_point3_distance_squared,
};
pub use predicates::dop::{
    SupportDop3, SupportDopAabb3Report, SupportDopAabb3SlabReport, SupportDopAabb3ValidationError,
    SupportDopAxis3, SupportDopExpansionKind, SupportDopExpansionReport, SupportDopPlane3Report,
    SupportDopPlane3ValidationError, SupportDopRefreshReport, SupportDopValidationError,
    SupportSlab3, SupportWitness3, WitnessedSupportDop3, WitnessedSupportSlab3,
    support_dop3_from_points, witnessed_support_dop3_from_points,
};
pub use predicates::filters::{
    certified_ball_sign, certified_ball_sign_report, certified_ball_sign_report_with_policy,
    certified_interval_sign, certified_interval_sign_report,
};
pub use predicates::halfspace::{
    HalfspaceFeasibilityReport, HalfspaceInfeasibilityCertificate, PreparedHalfspaceSystem3,
    classify_halfspace_feasibility3,
};
pub use predicates::interval::{
    classify_closed_interval_intersection, classify_real_closed_interval,
    closed_intervals_intersect, real_in_closed_interval,
};
pub use predicates::nd::{affine_independent_d, insphere_d, orient_d};
pub use predicates::order::{
    classify_real_sign, classify_real_sign_report, compare_point2_lexicographic,
    compare_point2_lexicographic_report, compare_point3_lexicographic,
    compare_point3_lexicographic_report, compare_reals, compare_reals_report,
    compare_reals_with_policy, point2_equal, point2_equal_report, point3_equal,
    point3_equal_report, real_clamp, real_ge, real_le, real_max, real_min,
};
pub use predicates::ring::{
    Ring2Facts, RingEvenOddEdgeReport, RingEvenOddReport, RingEvenOddValidationError,
    classify_point_indexed_ring_even_odd, classify_point_indexed_ring_even_odd_report,
    classify_point_ring_even_odd, classify_point_ring_even_odd_report, indexed_ring_area_sign,
    indexed_ring_convexity, indexed_ring2_facts, point_in_indexed_ring_even_odd,
    point_in_ring_even_odd, ring_area_sign, ring_convexity, ring2_facts,
};
pub use predicates::segment::{
    PreparedSegment2, PreparedSegment3, classify_point_segment, classify_point_segment_with_facts,
    classify_point_segment3, classify_segment_intersection,
    classify_segment_intersection_with_facts, classify_segment3_intersection, point_on_segment,
    point_on_segment_with_facts, point_on_segment3, proper_segment_intersection_point,
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
    classify_point_tetrahedron, classify_point_triangle, classify_point_triangle_with_facts,
    classify_point_triangle3, classify_ray_triangle3_intersection,
    classify_ray_triangle3_intersection_report, classify_segment_triangle3_intersection,
    classify_segment_triangle3_intersection_report, triangle3_winding_normal_sign,
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
