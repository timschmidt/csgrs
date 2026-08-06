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
mod real;
mod resolve;

pub use hyperreal::{
    CertifiedRealSign, DomainFacts as RealDomainFacts, DomainStatus as RealDomainStatus,
    ExpressionDegree as RealExpressionDegree, RationalStorageClass, Real,
    RealExactSetDenominatorKind, RealExactSetDyadicExponentClass, RealExactSetSignPattern,
    RealSignCertificate, SymbolicDependencyMask as RealSymbolicDependencyMask,
    ZeroOneMinusOneStatus as RealZeroOneMinusOneStatus,
};

pub use batch::{
    CircleLine2Case, CircleSegment2Case, Incircle2dCase as Incircle2Case,
    Insphere3dCase as Insphere3Case, Orient2dCase as Orient2Case, Orient3dCase as Orient3Case,
    PointPlaneCase, RayTriangle3IntersectionCase, Segment3IntersectionCase,
    SegmentTriangle3IntersectionCase,
    classify_circle_line2_batch_with_policy as classify_circle_line2_batch,
    classify_circle_segment2_batch_with_policy as classify_circle_segment2_batch,
    classify_point_line_batch_with_policy as classify_point_line_batch,
    classify_point_oriented_plane_batch_with_policy as classify_point_oriented_plane_batch,
    classify_point_plane_batch_with_policy as classify_point_plane_batch,
    classify_ray_triangle3_intersection_batch_with_policy as classify_ray_triangle3_intersection_batch,
    classify_segment_triangle3_intersection_batch_with_policy as classify_segment_triangle3_intersection_batch,
    classify_segment3_intersection_batch_with_policy as classify_segment3_intersection_batch,
    incircle2d_batch_with_policy as incircle2_batch,
    insphere3d_batch_with_policy as insphere3_batch, orient2d_batch_with_policy as orient2_batch,
    orient3d_batch_with_policy as orient3_batch,
};
#[cfg(feature = "parallel")]
pub use batch::{
    classify_circle_line2_batch_parallel_with_policy as classify_circle_line2_batch_parallel,
    classify_circle_segment2_batch_parallel_with_policy as classify_circle_segment2_batch_parallel,
    classify_point_line_batch_parallel_with_policy as classify_point_line_batch_parallel,
    classify_point_oriented_plane_batch_parallel_with_policy as classify_point_oriented_plane_batch_parallel,
    classify_point_plane_batch_parallel_with_policy as classify_point_plane_batch_parallel,
    classify_ray_triangle3_intersection_batch_parallel_with_policy as classify_ray_triangle3_intersection_batch_parallel,
    classify_segment_triangle3_intersection_batch_parallel_with_policy as classify_segment_triangle3_intersection_batch_parallel,
    classify_segment3_intersection_batch_parallel_with_policy as classify_segment3_intersection_batch_parallel,
    incircle2d_batch_parallel_with_policy as incircle2_batch_parallel,
    insphere3d_batch_parallel_with_policy as insphere3_batch_parallel,
    orient2d_batch_parallel_with_policy as orient2_batch_parallel,
    orient3d_batch_parallel_with_policy as orient3_batch_parallel,
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
    Triangle2Facts, TriangleEdge2, aabb2_facts,
    classify_homogeneous_point_plane_with_policy as classify_homogeneous_point_plane,
    intersect_homogeneous_line_plane, intersect_three_planes, intersect_two_planes,
    point2_displacement_facts, segment2_facts, triangle2_facts,
};
pub use orient::{
    Circle2Polynomial, Incircle2Evidence, Insphere3Evidence, LiftedPolynomialFacts,
    Line2Orientation, Point2, Point3, PredicateFacts, Sphere3Polynomial, classify_point_line,
    classify_point_line_with_orientation, incircle2, incircle2_evidence, incircle2_with_evidence,
    insphere3, insphere3_evidence, insphere3_with_evidence, line2_orientation,
    line2_orientation_with_facts, orient2, orient3,
};
pub use plane::{
    OrientedPlane3Evidence, Plane3, Plane3Evidence, PlaneAabbReport,
    PlaneAabbReportValidationError, TrianglePlaneRelation, TrianglePlaneReport,
    TrianglePlaneReportValidationError, classify_plane_aabb3, classify_plane_aabb3_report,
    classify_plane_segment, classify_plane_triangle, classify_point_oriented_plane,
    classify_point_oriented_plane_with_evidence, classify_point_plane,
    classify_point_plane_with_evidence, classify_triangle_against_oriented_plane,
    oriented_plane3_evidence, plane3_evidence, triangle_plane_relation_from_sides,
};
pub use predicate::{
    Certainty, DeterminantScheduleHint, Escalation, ExactPredicateKernel, PredicateOutcome,
    PredicatePolicy, RefinementNeed, Sign, SignKnowledge,
};
pub use predicates::aabb::{
    aabb2s_intersect_with_policy as aabb2s_intersect,
    aabb3s_intersect_with_policy as aabb3s_intersect,
    classify_aabb2_intersection_with_policy as classify_aabb2_intersection,
    classify_aabb2_intersection_with_policy_and_facts as classify_aabb2_intersection_with_facts,
    classify_aabb3_intersection_with_policy as classify_aabb3_intersection,
    classify_point_aabb2_with_policy as classify_point_aabb2,
    classify_point_aabb3_with_policy as classify_point_aabb3,
    ordered_aabb2s_intersect_coordinates_with_policy as ordered_aabb2s_intersect_coordinates,
    ordered_aabb3_contains_with_policy as ordered_aabb3_contains,
    ordered_aabb3s_intersect_coordinates_with_policy as ordered_aabb3s_intersect_coordinates,
    ordered_aabb3s_intersect_with_policy as ordered_aabb3s_intersect,
    point_in_aabb2_with_policy as point_in_aabb2, point_in_aabb3_with_policy as point_in_aabb3,
    point_in_ordered_aabb2_coordinates_with_policy as point_in_ordered_aabb2_coordinates,
    point_in_ordered_aabb3_relative_interior_with_policy as point_in_ordered_aabb3_relative_interior,
    point_in_triangle2_aabb_with_policy as point_in_triangle2_aabb,
};
pub use predicates::convex::{
    classify_point_convex_planes3_with_policy as classify_point_convex_planes3,
    classify_point_convex_polygon2_with_policy as classify_point_convex_polygon2,
};
pub use predicates::coplanar::{
    CoplanarProjection, CoplanarTriangleClassification, CoplanarTriangleRelation,
    CoplanarTriangleValidationError, TriangleDegeneracy,
    ccw_projected_turn_less_with_policy as ccw_projected_turn_less,
    choose_coplanar_projection_with_policy as choose_coplanar_projection,
    classify_coplanar_triangle_points_with_policy as classify_coplanar_triangle_points,
    classify_coplanar_triangles_with_policy as classify_coplanar_triangles,
    classify_point_projected_triangle3_with_policy as classify_point_projected_triangle3,
    classify_triangle3_degeneracy_with_policy as classify_triangle3_degeneracy,
    derive_coplanar_triangle_relation,
    intersect_segment_with_projected_line3_with_policy as intersect_segment_with_projected_line3,
    midpoint3, orient2d_value, project_point3, project_triangle3,
    projected_line_parameter3_with_policy as projected_line_parameter3,
    projected_polygon_area2_abs_value_with_policy as projected_polygon_area2_abs_value,
    projected_polygon_area2_sign_with_policy as projected_polygon_area2_sign,
    projected_polygon_area2_value,
    projected_segment_parameter3_with_policy as projected_segment_parameter3, projected_vector3,
};
pub use predicates::distance::{
    classify_aabb3_sphere_intersection_with_policy as classify_aabb3_sphere_intersection,
    classify_circle_line2_with_policy as classify_circle_line2,
    classify_circle_segment2_with_policy as classify_circle_segment2,
    classify_point_sphere3_with_policy as classify_point_sphere3,
    classify_sphere3_intersection_with_policy as classify_sphere3_intersection,
    compare_point_line3_distance_squared_with_policy as compare_point_line3_distance_squared,
    compare_point_plane_distance_squared_with_policy as compare_point_plane_distance_squared,
    compare_point_segment3_distance_squared_with_policy as compare_point_segment3_distance_squared,
    compare_point2_distance_squared_with_policy as compare_point2_distance_squared,
    compare_point3_distance_squared_with_policy as compare_point3_distance_squared,
};
pub use predicates::dop::{
    SupportDop3, SupportDopAabb3Report, SupportDopAabb3SlabReport, SupportDopAabb3ValidationError,
    SupportDopAxis3, SupportDopExpansionKind, SupportDopExpansionReport, SupportDopPlane3Report,
    SupportDopPlane3ValidationError, SupportDopRefreshReport, SupportDopValidationError,
    SupportSlab3, SupportWitness3, WitnessedSupportDop3, WitnessedSupportSlab3,
    support_dop3_from_points_with_policy as support_dop3_from_points,
    witnessed_support_dop3_from_points_with_policy as witnessed_support_dop3_from_points,
};
pub use predicates::filters::{
    certified_ball_sign_with_policy as certified_ball_sign,
    certified_interval_sign_with_policy as certified_interval_sign,
    classify_ball_sign_with_policy as classify_ball_sign,
};
pub use predicates::halfspace::{
    HalfspaceFeasibilityReport, HalfspaceInfeasibilityCertificate,
    classify_halfspace_feasibility3_with_policy as classify_halfspace_feasibility3,
};
pub use predicates::interval::{
    classify_closed_interval_intersection_with_policy as classify_closed_interval_intersection,
    classify_real_closed_interval_with_policy as classify_real_closed_interval,
    closed_intervals_intersect_with_policy as closed_intervals_intersect,
    real_in_closed_interval_with_policy as real_in_closed_interval,
};
pub use predicates::nd::{
    affine_independent_d_with_policy as affine_independent_d, insphere_d_with_policy as insphere_d,
    orient_d_with_policy as orient_d,
};
pub use predicates::order::{
    classify_real_sign_pair_with_policy as classify_real_sign_pair,
    classify_real_sign_with_policy as classify_real_sign,
    compare_point2_lexicographic_with_policy as compare_point2_lexicographic,
    compare_point3_lexicographic_with_policy as compare_point3_lexicographic,
    compare_reals_with_policy as compare_reals, point2_equal_with_policy as point2_equal,
    point3_equal_with_policy as point3_equal, real_clamp_with_policy as real_clamp,
    real_ge_with_policy as real_ge, real_le_with_policy as real_le,
    real_max_with_policy as real_max, real_min_with_policy as real_min,
    reciprocal_real_with_policy as reciprocal_real,
};
pub use predicates::ring::{
    Ring2Facts, RingEvenOddEdgeReport, RingEvenOddReport, RingEvenOddValidationError,
    classify_point_indexed_ring_even_odd_report_with_policy as classify_point_indexed_ring_even_odd_report,
    classify_point_indexed_ring_even_odd_with_policy as classify_point_indexed_ring_even_odd,
    classify_point_ring_even_odd_report_with_policy as classify_point_ring_even_odd_report,
    classify_point_ring_even_odd_with_policy as classify_point_ring_even_odd,
    indexed_ring_area_sign_with_policy as indexed_ring_area_sign,
    indexed_ring_convexity_with_policy as indexed_ring_convexity,
    indexed_ring2_facts_with_policy as indexed_ring2_facts,
    point_in_indexed_ring_even_odd_with_policy as point_in_indexed_ring_even_odd,
    point_in_ring_even_odd_with_policy as point_in_ring_even_odd,
    ring_area_sign_with_policy as ring_area_sign, ring_convexity_with_policy as ring_convexity,
    ring2_facts_with_policy as ring2_facts,
};
pub use predicates::segment::{
    classify_point_segment_with_orientation_and_policy as classify_point_segment_with_orientation,
    classify_point_segment_with_policy as classify_point_segment,
    classify_point_segment_with_policy_and_facts as classify_point_segment_with_facts,
    classify_point_segment3_with_policy as classify_point_segment3,
    classify_segment_intersection_with_policy as classify_segment_intersection,
    classify_segment_intersection_with_policy_and_facts as classify_segment_intersection_with_facts,
    classify_segment3_intersection_with_policy as classify_segment3_intersection,
    construct_line_intersection_point,
    point_on_segment_with_orientation_and_policy as point_on_segment_with_orientation,
    point_on_segment_with_policy as point_on_segment,
    point_on_segment_with_policy_and_facts as point_on_segment_with_facts,
    point_on_segment3_with_policy as point_on_segment3,
};
pub use predicates::segment_plane::{
    SegmentPlaneConstructionFailure, SegmentPlaneIntersection, SegmentPlaneParameterRatio,
    SegmentPlaneRelation, SegmentPlaneValidationError,
    construct_segment_plane_crossing_from_values_with_policy as construct_segment_plane_crossing_from_values,
    interpolate_point3,
    intersect_segment_with_oriented_plane_with_policy as intersect_segment_with_oriented_plane,
    intersect_segment_with_plane_values_with_policy as intersect_segment_with_plane_values,
    intersect_segment_with_plane_with_policy as intersect_segment_with_plane, point_plane_value,
    segment_parameter_from_axis_with_policy as segment_parameter_from_axis,
};
pub use predicates::triangle::{
    RayTriangleIntersectionReport, RayTriangleParameterRatio, RayTriangleValidationError,
    SegmentTriangleIntersectionReport, SegmentTriangleValidationError, Triangle3Orientation,
    classify_point_tetrahedron_with_policy as classify_point_tetrahedron,
    classify_point_triangle_with_orientation,
    classify_point_triangle_with_policy as classify_point_triangle,
    classify_point_triangle_with_policy_and_facts as classify_point_triangle_with_facts,
    classify_point_triangle3_with_orientation,
    classify_point_triangle3_with_policy as classify_point_triangle3,
    classify_ray_triangle3_intersection_report_with_policy as classify_ray_triangle3_intersection_report,
    classify_ray_triangle3_intersection_with_policy as classify_ray_triangle3_intersection,
    classify_segment_triangle3_intersection_report_with_policy as classify_segment_triangle3_intersection_report,
    classify_segment_triangle3_intersection_with_policy as classify_segment_triangle3_intersection,
    triangle3_orientation,
    triangle3_winding_normal_sign_with_policy as triangle3_winding_normal_sign,
};
pub use predicates::triangle_triangle::{
    TriangleTriangleClassification, TriangleTriangleValidationError,
    classify_triangle_triangle3_points_with_policy as classify_triangle_triangle3_points,
    classify_triangle_triangle3_with_policy as classify_triangle_triangle3,
};
pub use real::RealPredicateExt;
