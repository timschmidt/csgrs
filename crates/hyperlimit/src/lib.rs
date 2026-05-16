//! Hyperreal-backed exact predicates with structural Real awareness.
//!
//! `hyperlimit` is intentionally positioned between Real semantics and
//! application geometry code. It asks `hyperreal::Real` for facts such as known
//! sign, exact zero, rational structure, and refinement capability before
//! escalating a predicate.

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
    Incircle2dCase, Insphere3dCase, Orient2dCase, Orient3dCase, PointPlaneCase,
    classify_point_line_batch, classify_point_line_batch_with_policy,
    classify_point_oriented_plane_batch, classify_point_oriented_plane_batch_with_policy,
    classify_point_plane_batch, classify_point_plane_batch_with_policy, incircle2d_batch,
    incircle2d_batch_with_policy, insphere3d_batch, insphere3d_batch_with_policy, orient2d_batch,
    orient2d_batch_with_policy, orient3d_batch, orient3d_batch_with_policy,
};
#[cfg(feature = "parallel")]
pub use batch::{
    classify_point_line_batch_parallel, classify_point_line_batch_parallel_with_policy,
    classify_point_oriented_plane_batch_parallel,
    classify_point_oriented_plane_batch_parallel_with_policy, classify_point_plane_batch_parallel,
    classify_point_plane_batch_parallel_with_policy, incircle2d_batch_parallel,
    incircle2d_batch_parallel_with_policy, insphere3d_batch_parallel,
    insphere3d_batch_parallel_with_policy, orient2d_batch_parallel,
    orient2d_batch_parallel_with_policy, orient3d_batch_parallel,
    orient3d_batch_parallel_with_policy,
};
pub use classify::{
    Aabb2Intersection, Aabb2PointLocation, ClosedIntervalIntersection, LineSide, PlaneSide,
    PointSegmentLocation, RealIntervalLocation, RingPointLocation, SegmentIntersection,
    TriangleLocation,
};
pub use geometry::{
    Aabb2Facts, CoordinateAxis2, Plane3Facts, Point2DisplacementFacts, Point2Facts, Point3Facts,
    PointSharedScaleView, Segment2Facts, Triangle2Facts, TriangleEdge2, aabb2_facts,
    point2_displacement_facts, segment2_facts, triangle2_facts,
};
pub use orient::{
    Point2, Point3, PreparedCircle2Polynomial, PreparedIncircle2, PreparedInsphere3,
    PreparedLiftedPolynomialFacts, PreparedLine2, PreparedPredicateFacts,
    PreparedSphere3Polynomial, classify_point_line, classify_point_line_with_policy, incircle2d,
    incircle2d_report, incircle2d_report_with_policy, incircle2d_with_policy, insphere3d,
    insphere3d_report, insphere3d_report_with_policy, insphere3d_with_policy, orient2d,
    orient2d_report, orient2d_report_with_policy, orient2d_with_policy, orient3d, orient3d_report,
    orient3d_report_with_policy, orient3d_with_policy,
};
pub use plane::{
    Plane3, PreparedOrientedPlane3, PreparedPlane3, classify_point_oriented_plane,
    classify_point_plane,
};
pub use predicate::{
    Certainty, DeterminantScheduleHint, Escalation, ExactPredicateKernel, PredicateApiSemantics,
    PredicateCertificate, PredicateOutcome, PredicatePolicy, PredicatePrecisionStage,
    PredicateReport, Sign, SignKnowledge,
};
pub use predicates::aabb::{
    PreparedAabb2, aabb2s_intersect, aabb2s_intersect_with_policy, classify_aabb2_intersection,
    classify_aabb2_intersection_with_facts, classify_aabb2_intersection_with_policy,
    classify_aabb2_intersection_with_policy_and_facts, classify_point_aabb2,
    classify_point_aabb2_with_policy, point_in_aabb2, point_in_aabb2_with_policy,
};
pub use predicates::distance::{
    compare_point2_distance_squared, compare_point2_distance_squared_with_policy,
};
pub use predicates::filters::{
    certified_ball_sign, certified_ball_sign_report, certified_ball_sign_report_with_policy,
    certified_ball_sign_with_policy, certified_interval_sign, certified_interval_sign_report,
    certified_interval_sign_report_with_policy, certified_interval_sign_with_policy,
};
pub use predicates::interval::{
    classify_closed_interval_intersection, classify_closed_interval_intersection_with_policy,
    classify_real_closed_interval, classify_real_closed_interval_with_policy,
    closed_intervals_intersect, closed_intervals_intersect_with_policy, real_in_closed_interval,
    real_in_closed_interval_with_policy,
};
pub use predicates::order::{
    compare_point2_lexicographic, compare_point2_lexicographic_report,
    compare_point2_lexicographic_report_with_policy, compare_point2_lexicographic_with_policy,
    compare_reals, compare_reals_report, compare_reals_report_with_policy,
    compare_reals_with_policy, point2_equal, point2_equal_report, point2_equal_report_with_policy,
    point2_equal_with_policy,
};
pub use predicates::ring::{
    classify_point_ring_even_odd, classify_point_ring_even_odd_with_policy, point_in_ring_even_odd,
    point_in_ring_even_odd_with_policy, ring_area_sign, ring_area_sign_with_policy,
};
pub use predicates::segment::{
    PreparedSegment2, classify_point_segment, classify_point_segment_with_facts,
    classify_point_segment_with_policy, classify_point_segment_with_policy_and_facts,
    classify_segment_intersection, classify_segment_intersection_with_facts,
    classify_segment_intersection_with_policy, classify_segment_intersection_with_policy_and_facts,
    point_on_segment, point_on_segment_with_facts, point_on_segment_with_policy,
    point_on_segment_with_policy_and_facts,
};
pub use predicates::triangle::{
    PreparedTriangle2, classify_point_triangle, classify_point_triangle_with_facts,
    classify_point_triangle_with_policy, classify_point_triangle_with_policy_and_facts,
};
pub use real::{RealFacts, RealPredicateExt, RealZeroKnowledge};
pub use session::{
    CachePayoff, CachedApproximateView, ConstructionCertificate, ConstructionDependencies,
    ConstructionFreshness, ConstructionVersion, ExactGeometrySession, VersionedFacts,
    VersionedPredicateReport,
};
