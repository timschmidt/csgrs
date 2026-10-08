//! Batch predicate helpers for Real-backed geometry.

use crate::classify::{
    CircleLineRelation, CircleSegmentRelation, LineSide, PlaneSide, RayTriangleIntersection,
    Segment3Intersection, SegmentTriangleIntersection,
};
use crate::orient::{
    Point2, Point3, classify_point_line_with_policy, incircle2d_with_policy,
    insphere3d_with_policy, orient2d_with_policy, orient3d_with_policy,
};
use crate::plane::{
    Plane3, classify_point_oriented_plane_with_policy, classify_point_plane_with_policy,
};
use crate::predicate::{PredicateOutcome, PredicatePolicy, Sign};
use crate::predicates::distance::{
    classify_circle_line2_with_policy, classify_circle_segment2_with_policy,
};
use crate::predicates::segment::classify_segment3_intersection_with_policy;
use crate::predicates::triangle::{
    classify_ray_triangle3_intersection_with_policy,
    classify_segment_triangle3_intersection_with_policy,
};
use hyperreal::Real;

/// Case tuple exported as [`crate::Orient2Case`] and accepted by
/// [`crate::orient2_batch`] and, with the `parallel` feature,
/// `crate::orient2_batch_parallel`.
pub type Orient2dCase = (Point2, Point2, Point2);
/// Case tuple exported as [`crate::Orient3Case`] and accepted by
/// [`crate::orient3_batch`] and, with the `parallel` feature,
/// `crate::orient3_batch_parallel`.
pub type Orient3dCase = (Point3, Point3, Point3, Point3);
/// Case tuple exported as [`crate::Incircle2Case`] and accepted by
/// [`crate::incircle2_batch`] and, with the `parallel` feature,
/// `crate::incircle2_batch_parallel`.
pub type Incircle2dCase = (Point2, Point2, Point2, Point2);
/// Case tuple exported as [`crate::Insphere3Case`] and accepted by
/// [`crate::insphere3_batch`] and, with the `parallel` feature,
/// `crate::insphere3_batch_parallel`.
pub type Insphere3dCase = (Point3, Point3, Point3, Point3, Point3);
/// Case tuple accepted by [`crate::classify_point_plane_batch`] and
/// `crate::classify_point_plane_batch_parallel` when the `parallel` feature is
/// enabled.
pub type PointPlaneCase = (Point3, Plane3);
/// Case tuple accepted by [`crate::classify_segment3_intersection_batch`].
pub type Segment3IntersectionCase = (Point3, Point3, Point3, Point3);
/// Case tuple accepted by [`crate::classify_segment_triangle3_intersection_batch`].
pub type SegmentTriangle3IntersectionCase = (Point3, Point3, Point3, Point3, Point3);
/// Case tuple accepted by [`crate::classify_ray_triangle3_intersection_batch`].
pub type RayTriangle3IntersectionCase = (Point3, Point3, Point3, Point3, Point3);
/// Case tuple accepted by [`crate::classify_circle_line2_batch`].
pub type CircleLine2Case = (Point2, Real, Point2, Point2);
/// Case tuple accepted by [`crate::classify_circle_segment2_batch`].
pub type CircleSegment2Case = (Point2, Real, Point2, Point2);

/// Evaluate a batch of 2D orientation predicates with an explicit policy.
pub fn orient2d_batch_with_policy(
    cases: &[Orient2dCase],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("hyperlimit", "batch", "orient2d-sequential");
    cases
        .iter()
        .map(|(a, b, c)| orient2d_with_policy(a, b, c, policy))
        .collect()
}

/// Evaluate a batch of line-side classifications with an explicit policy.
pub fn classify_point_line_batch_with_policy(
    cases: &[Orient2dCase],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<LineSide>> {
    crate::trace_dispatch!("hyperlimit", "batch", "classify-point-line-sequential");
    cases
        .iter()
        .map(|(from, to, point)| classify_point_line_with_policy(from, to, point, policy))
        .collect()
}

/// Evaluate a batch of 3D orientation predicates with an explicit policy.
pub fn orient3d_batch_with_policy(
    cases: &[Orient3dCase],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("hyperlimit", "batch", "orient3d-sequential");
    cases
        .iter()
        .map(|(a, b, c, d)| orient3d_with_policy(a, b, c, d, policy))
        .collect()
}

/// Evaluate a batch of explicit point-plane classifications with an explicit policy.
pub fn classify_point_plane_batch_with_policy(
    cases: &[PointPlaneCase],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<PlaneSide>> {
    crate::trace_dispatch!("hyperlimit", "batch", "classify-point-plane-sequential");
    cases
        .iter()
        .map(|(point, plane)| classify_point_plane_with_policy(point, plane, policy))
        .collect()
}

/// Evaluate a batch of oriented-plane classifications with an explicit policy.
pub fn classify_point_oriented_plane_batch_with_policy(
    cases: &[Orient3dCase],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<PlaneSide>> {
    crate::trace_dispatch!(
        "hyperlimit",
        "batch",
        "classify-point-oriented-plane-sequential"
    );
    cases
        .iter()
        .map(|(a, b, c, point)| classify_point_oriented_plane_with_policy(a, b, c, point, policy))
        .collect()
}

/// Evaluate a batch of 2D in-circle predicates with an explicit policy.
pub fn incircle2d_batch_with_policy(
    cases: &[Incircle2dCase],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("hyperlimit", "batch", "incircle2d-sequential");
    cases
        .iter()
        .map(|(a, b, c, d)| incircle2d_with_policy(a, b, c, d, policy))
        .collect()
}

/// Evaluate a batch of 3D in-sphere predicates with an explicit policy.
pub fn insphere3d_batch_with_policy(
    cases: &[Insphere3dCase],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("hyperlimit", "batch", "insphere3d-sequential");
    cases
        .iter()
        .map(|(a, b, c, d, e)| insphere3d_with_policy(a, b, c, d, e, policy))
        .collect()
}

/// Evaluate a batch of closed 3D segment/segment relation predicates with an
/// explicit policy.
///
/// Batch APIs are scheduling helpers only: every item still returns its own
/// exact predicate outcome and provenance. Batching can reuse object structure
/// but cannot turn unknown or lossy answers into topology.
pub fn classify_segment3_intersection_batch_with_policy(
    cases: &[Segment3IntersectionCase],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<Segment3Intersection>> {
    crate::trace_dispatch!("hyperlimit", "batch", "segment3-intersection-sequential");
    cases
        .iter()
        .map(|(a, b, c, d)| classify_segment3_intersection_with_policy(a, b, c, d, policy))
        .collect()
}

/// Evaluate a batch of closed segment/triangle relation predicates with an
/// explicit policy.
pub fn classify_segment_triangle3_intersection_batch_with_policy(
    cases: &[SegmentTriangle3IntersectionCase],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<SegmentTriangleIntersection>> {
    crate::trace_dispatch!(
        "hyperlimit",
        "batch",
        "segment-triangle3-intersection-sequential"
    );
    cases
        .iter()
        .map(|(p, q, a, b, c)| {
            classify_segment_triangle3_intersection_with_policy(p, q, a, b, c, policy)
        })
        .collect()
}

/// Evaluate a batch of ray/triangle relation predicates with an explicit
/// policy.
pub fn classify_ray_triangle3_intersection_batch_with_policy(
    cases: &[RayTriangle3IntersectionCase],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<RayTriangleIntersection>> {
    crate::trace_dispatch!(
        "hyperlimit",
        "batch",
        "ray-triangle3-intersection-sequential"
    );
    cases
        .iter()
        .map(|(origin, direction, a, b, c)| {
            classify_ray_triangle3_intersection_with_policy(origin, direction, a, b, c, policy)
        })
        .collect()
}

/// Evaluate a batch of circle/line relation predicates with an explicit policy.
pub fn classify_circle_line2_batch_with_policy(
    cases: &[CircleLine2Case],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<CircleLineRelation>> {
    crate::trace_dispatch!("hyperlimit", "batch", "circle-line2-sequential");
    cases
        .iter()
        .map(|(center, radius_squared, a, b)| {
            classify_circle_line2_with_policy(center, radius_squared, a, b, policy)
        })
        .collect()
}

/// Evaluate a batch of circle/segment relation predicates with an explicit
/// policy.
pub fn classify_circle_segment2_batch_with_policy(
    cases: &[CircleSegment2Case],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<CircleSegmentRelation>> {
    crate::trace_dispatch!("hyperlimit", "batch", "circle-segment2-sequential");
    cases
        .iter()
        .map(|(center, radius_squared, a, b)| {
            classify_circle_segment2_with_policy(center, radius_squared, a, b, policy)
        })
        .collect()
}

#[cfg(feature = "parallel")]
mod parallel {
    use super::*;
    use rayon::prelude::*;

    /// Evaluate a batch of 2D orientation predicates in parallel with an explicit policy.
    pub fn orient2d_batch_parallel_with_policy(
        cases: &[Orient2dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>> {
        crate::trace_dispatch!("hyperlimit", "batch", "orient2d-parallel");
        cases
            .par_iter()
            .map(|(a, b, c)| orient2d_with_policy(a, b, c, policy))
            .collect()
    }

    /// Evaluate a batch of line-side classifications in parallel with an explicit policy.
    pub fn classify_point_line_batch_parallel_with_policy(
        cases: &[Orient2dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<LineSide>> {
        crate::trace_dispatch!("hyperlimit", "batch", "classify-point-line-parallel");
        cases
            .par_iter()
            .map(|(from, to, point)| classify_point_line_with_policy(from, to, point, policy))
            .collect()
    }

    /// Evaluate a batch of 3D orientation predicates in parallel with an explicit policy.
    pub fn orient3d_batch_parallel_with_policy(
        cases: &[Orient3dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>> {
        crate::trace_dispatch!("hyperlimit", "batch", "orient3d-parallel");
        cases
            .par_iter()
            .map(|(a, b, c, d)| orient3d_with_policy(a, b, c, d, policy))
            .collect()
    }

    /// Evaluate a batch of explicit point-plane classifications in parallel with an explicit policy.
    pub fn classify_point_plane_batch_parallel_with_policy(
        cases: &[PointPlaneCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<PlaneSide>> {
        crate::trace_dispatch!("hyperlimit", "batch", "classify-point-plane-parallel");
        cases
            .par_iter()
            .map(|(point, plane)| classify_point_plane_with_policy(point, plane, policy))
            .collect()
    }

    /// Evaluate a batch of oriented-plane classifications in parallel with an explicit policy.
    pub fn classify_point_oriented_plane_batch_parallel_with_policy(
        cases: &[Orient3dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<PlaneSide>> {
        crate::trace_dispatch!(
            "hyperlimit",
            "batch",
            "classify-point-oriented-plane-parallel"
        );
        cases
            .par_iter()
            .map(|(a, b, c, point)| {
                classify_point_oriented_plane_with_policy(a, b, c, point, policy)
            })
            .collect()
    }

    /// Evaluate a batch of 2D in-circle predicates in parallel with an explicit policy.
    pub fn incircle2d_batch_parallel_with_policy(
        cases: &[Incircle2dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>> {
        crate::trace_dispatch!("hyperlimit", "batch", "incircle2d-parallel");
        cases
            .par_iter()
            .map(|(a, b, c, d)| incircle2d_with_policy(a, b, c, d, policy))
            .collect()
    }

    /// Evaluate a batch of 3D in-sphere predicates in parallel with an explicit policy.
    pub fn insphere3d_batch_parallel_with_policy(
        cases: &[Insphere3dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>> {
        crate::trace_dispatch!("hyperlimit", "batch", "insphere3d-parallel");
        cases
            .par_iter()
            .map(|(a, b, c, d, e)| insphere3d_with_policy(a, b, c, d, e, policy))
            .collect()
    }

    /// Evaluate a batch of closed 3D segment/segment relation predicates in parallel with an explicit policy.
    pub fn classify_segment3_intersection_batch_parallel_with_policy(
        cases: &[Segment3IntersectionCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Segment3Intersection>> {
        crate::trace_dispatch!("hyperlimit", "batch", "segment3-intersection-parallel");
        cases
            .par_iter()
            .map(|(a, b, c, d)| classify_segment3_intersection_with_policy(a, b, c, d, policy))
            .collect()
    }

    /// Evaluate a batch of closed segment/triangle relation predicates in parallel with an explicit policy.
    pub fn classify_segment_triangle3_intersection_batch_parallel_with_policy(
        cases: &[SegmentTriangle3IntersectionCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<SegmentTriangleIntersection>> {
        crate::trace_dispatch!(
            "hyperlimit",
            "batch",
            "segment-triangle3-intersection-parallel"
        );
        cases
            .par_iter()
            .map(|(p, q, a, b, c)| {
                classify_segment_triangle3_intersection_with_policy(p, q, a, b, c, policy)
            })
            .collect()
    }

    /// Evaluate a batch of ray/triangle relation predicates in parallel with an explicit policy.
    pub fn classify_ray_triangle3_intersection_batch_parallel_with_policy(
        cases: &[RayTriangle3IntersectionCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<RayTriangleIntersection>> {
        crate::trace_dispatch!("hyperlimit", "batch", "ray-triangle3-intersection-parallel");
        cases
            .par_iter()
            .map(|(origin, direction, a, b, c)| {
                classify_ray_triangle3_intersection_with_policy(origin, direction, a, b, c, policy)
            })
            .collect()
    }

    /// Evaluate a batch of circle/line relation predicates in parallel with an explicit policy.
    pub fn classify_circle_line2_batch_parallel_with_policy(
        cases: &[CircleLine2Case],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<CircleLineRelation>> {
        crate::trace_dispatch!("hyperlimit", "batch", "circle-line2-parallel");
        cases
            .par_iter()
            .map(|(center, radius_squared, a, b)| {
                classify_circle_line2_with_policy(center, radius_squared, a, b, policy)
            })
            .collect()
    }

    /// Evaluate a batch of circle/segment relation predicates in parallel with an explicit policy.
    pub fn classify_circle_segment2_batch_parallel_with_policy(
        cases: &[CircleSegment2Case],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<CircleSegmentRelation>> {
        crate::trace_dispatch!("hyperlimit", "batch", "circle-segment2-parallel");
        cases
            .par_iter()
            .map(|(center, radius_squared, a, b)| {
                classify_circle_segment2_with_policy(center, radius_squared, a, b, policy)
            })
            .collect()
    }
}

#[cfg(feature = "parallel")]
pub use parallel::*;
