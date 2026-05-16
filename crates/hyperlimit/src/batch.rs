//! Batch predicate helpers for Real-backed geometry.

use crate::classify::{LineSide, PlaneSide};
use crate::orient::{
    Point2, Point3, classify_point_line_with_policy, incircle2d_with_policy,
    insphere3d_with_policy, orient2d_with_policy, orient3d_with_policy,
};
use crate::plane::{
    Plane3, classify_point_oriented_plane_with_policy, classify_point_plane_with_policy,
};
use crate::predicate::{PredicateOutcome, PredicatePolicy, Sign};

/// Case tuple accepted by [`orient2d_batch`] and [`orient2d_batch_parallel`].
pub type Orient2dCase = (Point2, Point2, Point2);
/// Case tuple accepted by [`orient3d_batch`] and [`orient3d_batch_parallel`].
pub type Orient3dCase = (Point3, Point3, Point3, Point3);
/// Case tuple accepted by [`incircle2d_batch`] and [`incircle2d_batch_parallel`].
pub type Incircle2dCase = (Point2, Point2, Point2, Point2);
/// Case tuple accepted by [`insphere3d_batch`] and [`insphere3d_batch_parallel`].
pub type Insphere3dCase = (Point3, Point3, Point3, Point3, Point3);
/// Case tuple accepted by [`classify_point_plane_batch`] and
/// [`classify_point_plane_batch_parallel`].
pub type PointPlaneCase = (Point3, Plane3);

/// Evaluate a batch of 2D orientation predicates.
pub fn orient2d_batch(cases: &[Orient2dCase]) -> Vec<PredicateOutcome<Sign>> {
    orient2d_batch_with_policy(cases, PredicatePolicy::default())
}

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

/// Evaluate a batch of line-side classifications.
pub fn classify_point_line_batch(cases: &[Orient2dCase]) -> Vec<PredicateOutcome<LineSide>> {
    classify_point_line_batch_with_policy(cases, PredicatePolicy::default())
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

/// Evaluate a batch of 3D orientation predicates.
pub fn orient3d_batch(cases: &[Orient3dCase]) -> Vec<PredicateOutcome<Sign>> {
    orient3d_batch_with_policy(cases, PredicatePolicy::default())
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

/// Evaluate a batch of explicit point-plane classifications.
pub fn classify_point_plane_batch(cases: &[PointPlaneCase]) -> Vec<PredicateOutcome<PlaneSide>> {
    classify_point_plane_batch_with_policy(cases, PredicatePolicy::default())
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

/// Evaluate a batch of oriented-plane classifications.
pub fn classify_point_oriented_plane_batch(
    cases: &[Orient3dCase],
) -> Vec<PredicateOutcome<PlaneSide>> {
    classify_point_oriented_plane_batch_with_policy(cases, PredicatePolicy::default())
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

/// Evaluate a batch of 2D in-circle predicates.
pub fn incircle2d_batch(cases: &[Incircle2dCase]) -> Vec<PredicateOutcome<Sign>> {
    incircle2d_batch_with_policy(cases, PredicatePolicy::default())
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

/// Evaluate a batch of 3D in-sphere predicates.
pub fn insphere3d_batch(cases: &[Insphere3dCase]) -> Vec<PredicateOutcome<Sign>> {
    insphere3d_batch_with_policy(cases, PredicatePolicy::default())
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

#[cfg(feature = "parallel")]
mod parallel {
    use super::*;

    /// Evaluate a batch of 2D orientation predicates in parallel.
    pub fn orient2d_batch_parallel(cases: &[Orient2dCase]) -> Vec<PredicateOutcome<Sign>> {
        orient2d_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of 2D orientation predicates in parallel with an explicit policy.
    pub fn orient2d_batch_parallel_with_policy(
        cases: &[Orient2dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>> {
        crate::trace_dispatch!("hyperlimit", "batch", "orient2d-parallel");
        cases
            .iter()
            .map(|(a, b, c)| orient2d_with_policy(a, b, c, policy))
            .collect()
    }

    /// Evaluate a batch of line-side classifications in parallel.
    pub fn classify_point_line_batch_parallel(
        cases: &[Orient2dCase],
    ) -> Vec<PredicateOutcome<LineSide>> {
        classify_point_line_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of line-side classifications in parallel with an explicit policy.
    pub fn classify_point_line_batch_parallel_with_policy(
        cases: &[Orient2dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<LineSide>> {
        crate::trace_dispatch!("hyperlimit", "batch", "classify-point-line-parallel");
        cases
            .iter()
            .map(|(from, to, point)| classify_point_line_with_policy(from, to, point, policy))
            .collect()
    }

    /// Evaluate a batch of 3D orientation predicates in parallel.
    pub fn orient3d_batch_parallel(cases: &[Orient3dCase]) -> Vec<PredicateOutcome<Sign>> {
        orient3d_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of 3D orientation predicates in parallel with an explicit policy.
    pub fn orient3d_batch_parallel_with_policy(
        cases: &[Orient3dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>> {
        crate::trace_dispatch!("hyperlimit", "batch", "orient3d-parallel");
        cases
            .iter()
            .map(|(a, b, c, d)| orient3d_with_policy(a, b, c, d, policy))
            .collect()
    }

    /// Evaluate a batch of explicit point-plane classifications in parallel.
    pub fn classify_point_plane_batch_parallel(
        cases: &[PointPlaneCase],
    ) -> Vec<PredicateOutcome<PlaneSide>> {
        classify_point_plane_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of explicit point-plane classifications in parallel with an explicit policy.
    pub fn classify_point_plane_batch_parallel_with_policy(
        cases: &[PointPlaneCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<PlaneSide>> {
        crate::trace_dispatch!("hyperlimit", "batch", "classify-point-plane-parallel");
        cases
            .iter()
            .map(|(point, plane)| classify_point_plane_with_policy(point, plane, policy))
            .collect()
    }

    /// Evaluate a batch of oriented-plane classifications in parallel.
    pub fn classify_point_oriented_plane_batch_parallel(
        cases: &[Orient3dCase],
    ) -> Vec<PredicateOutcome<PlaneSide>> {
        classify_point_oriented_plane_batch_parallel_with_policy(cases, PredicatePolicy::default())
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
            .iter()
            .map(|(a, b, c, point)| {
                classify_point_oriented_plane_with_policy(a, b, c, point, policy)
            })
            .collect()
    }

    /// Evaluate a batch of 2D in-circle predicates in parallel.
    pub fn incircle2d_batch_parallel(cases: &[Incircle2dCase]) -> Vec<PredicateOutcome<Sign>> {
        incircle2d_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of 2D in-circle predicates in parallel with an explicit policy.
    pub fn incircle2d_batch_parallel_with_policy(
        cases: &[Incircle2dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>> {
        crate::trace_dispatch!("hyperlimit", "batch", "incircle2d-parallel");
        cases
            .iter()
            .map(|(a, b, c, d)| incircle2d_with_policy(a, b, c, d, policy))
            .collect()
    }

    /// Evaluate a batch of 3D in-sphere predicates in parallel.
    pub fn insphere3d_batch_parallel(cases: &[Insphere3dCase]) -> Vec<PredicateOutcome<Sign>> {
        insphere3d_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of 3D in-sphere predicates in parallel with an explicit policy.
    pub fn insphere3d_batch_parallel_with_policy(
        cases: &[Insphere3dCase],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>> {
        crate::trace_dispatch!("hyperlimit", "batch", "insphere3d-parallel");
        cases
            .iter()
            .map(|(a, b, c, d, e)| insphere3d_with_policy(a, b, c, d, e, policy))
            .collect()
    }
}

#[cfg(feature = "parallel")]
pub use parallel::*;
