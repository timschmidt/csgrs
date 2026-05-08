//! Batch predicate helpers.

use crate::classify::{LineSide, PlaneSide};
use crate::orient::{
    Point2, Point3, classify_point_line_with_policy, incircle2d_with_policy,
    insphere3d_with_policy, orient2d_with_policy, orient3d_with_policy,
};
use crate::plane::{
    Plane3, classify_point_oriented_plane_with_policy, classify_point_plane_with_policy,
};
use crate::predicate::{PredicateOutcome, PredicatePolicy, Sign};
use crate::scalar::BorrowedPredicateScalar;

/// Case tuple accepted by [`orient2d_batch`] and [`orient2d_batch_parallel`].
pub type Orient2dCase<S> = (Point2<S>, Point2<S>, Point2<S>);
/// Case tuple accepted by [`orient3d_batch`] and [`orient3d_batch_parallel`].
pub type Orient3dCase<S> = (Point3<S>, Point3<S>, Point3<S>, Point3<S>);
/// Case tuple accepted by [`incircle2d_batch`] and [`incircle2d_batch_parallel`].
pub type Incircle2dCase<S> = (Point2<S>, Point2<S>, Point2<S>, Point2<S>);
/// Case tuple accepted by [`insphere3d_batch`] and [`insphere3d_batch_parallel`].
pub type Insphere3dCase<S> = (Point3<S>, Point3<S>, Point3<S>, Point3<S>, Point3<S>);
/// Case tuple accepted by [`classify_point_plane_batch`] and
/// [`classify_point_plane_batch_parallel`].
pub type PointPlaneCase<S> = (Point3<S>, Plane3<S>);

/// Evaluate a batch of 2D orientation predicates.
pub fn orient2d_batch<S: BorrowedPredicateScalar>(
    cases: &[Orient2dCase<S>],
) -> Vec<PredicateOutcome<Sign>> {
    orient2d_batch_with_policy(cases, PredicatePolicy::default())
}

/// Evaluate a batch of 2D orientation predicates with an explicit policy.
pub fn orient2d_batch_with_policy<S: BorrowedPredicateScalar>(
    cases: &[Orient2dCase<S>],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("predicated", "batch", "orient2d-sequential");
    cases
        .iter()
        .map(|(a, b, c)| orient2d_with_policy(a, b, c, policy))
        .collect()
}

/// Evaluate a batch of line-side classifications.
pub fn classify_point_line_batch<S: BorrowedPredicateScalar>(
    cases: &[Orient2dCase<S>],
) -> Vec<PredicateOutcome<LineSide>> {
    classify_point_line_batch_with_policy(cases, PredicatePolicy::default())
}

/// Evaluate a batch of line-side classifications with an explicit policy.
pub fn classify_point_line_batch_with_policy<S: BorrowedPredicateScalar>(
    cases: &[Orient2dCase<S>],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<LineSide>> {
    crate::trace_dispatch!("predicated", "batch", "classify-point-line-sequential");
    cases
        .iter()
        .map(|(from, to, point)| classify_point_line_with_policy(from, to, point, policy))
        .collect()
}

/// Evaluate a batch of 3D orientation predicates.
pub fn orient3d_batch<S: BorrowedPredicateScalar>(
    cases: &[Orient3dCase<S>],
) -> Vec<PredicateOutcome<Sign>> {
    orient3d_batch_with_policy(cases, PredicatePolicy::default())
}

/// Evaluate a batch of 3D orientation predicates with an explicit policy.
pub fn orient3d_batch_with_policy<S: BorrowedPredicateScalar>(
    cases: &[Orient3dCase<S>],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("predicated", "batch", "orient3d-sequential");
    cases
        .iter()
        .map(|(a, b, c, d)| orient3d_with_policy(a, b, c, d, policy))
        .collect()
}

/// Evaluate a batch of explicit point-plane classifications.
pub fn classify_point_plane_batch<S: BorrowedPredicateScalar>(
    cases: &[PointPlaneCase<S>],
) -> Vec<PredicateOutcome<PlaneSide>> {
    classify_point_plane_batch_with_policy(cases, PredicatePolicy::default())
}

/// Evaluate a batch of explicit point-plane classifications with an explicit policy.
pub fn classify_point_plane_batch_with_policy<S: BorrowedPredicateScalar>(
    cases: &[PointPlaneCase<S>],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<PlaneSide>> {
    crate::trace_dispatch!("predicated", "batch", "classify-point-plane-sequential");
    cases
        .iter()
        .map(|(point, plane)| classify_point_plane_with_policy(point, plane, policy))
        .collect()
}

/// Evaluate a batch of oriented-plane classifications.
pub fn classify_point_oriented_plane_batch<S: BorrowedPredicateScalar>(
    cases: &[Orient3dCase<S>],
) -> Vec<PredicateOutcome<PlaneSide>> {
    classify_point_oriented_plane_batch_with_policy(cases, PredicatePolicy::default())
}

/// Evaluate a batch of oriented-plane classifications with an explicit policy.
pub fn classify_point_oriented_plane_batch_with_policy<S: BorrowedPredicateScalar>(
    cases: &[Orient3dCase<S>],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<PlaneSide>> {
    crate::trace_dispatch!(
        "predicated",
        "batch",
        "classify-point-oriented-plane-sequential"
    );
    cases
        .iter()
        .map(|(a, b, c, point)| classify_point_oriented_plane_with_policy(a, b, c, point, policy))
        .collect()
}

/// Evaluate a batch of 2D in-circle predicates.
pub fn incircle2d_batch<S: BorrowedPredicateScalar>(
    cases: &[Incircle2dCase<S>],
) -> Vec<PredicateOutcome<Sign>> {
    incircle2d_batch_with_policy(cases, PredicatePolicy::default())
}

/// Evaluate a batch of 2D in-circle predicates with an explicit policy.
pub fn incircle2d_batch_with_policy<S: BorrowedPredicateScalar>(
    cases: &[Incircle2dCase<S>],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("predicated", "batch", "incircle2d-sequential");
    cases
        .iter()
        .map(|(a, b, c, d)| incircle2d_with_policy(a, b, c, d, policy))
        .collect()
}

/// Evaluate a batch of 3D in-sphere predicates.
pub fn insphere3d_batch<S: BorrowedPredicateScalar>(
    cases: &[Insphere3dCase<S>],
) -> Vec<PredicateOutcome<Sign>> {
    insphere3d_batch_with_policy(cases, PredicatePolicy::default())
}

/// Evaluate a batch of 3D in-sphere predicates with an explicit policy.
pub fn insphere3d_batch_with_policy<S: BorrowedPredicateScalar>(
    cases: &[Insphere3dCase<S>],
    policy: PredicatePolicy,
) -> Vec<PredicateOutcome<Sign>> {
    crate::trace_dispatch!("predicated", "batch", "insphere3d-sequential");
    cases
        .iter()
        .map(|(a, b, c, d, e)| insphere3d_with_policy(a, b, c, d, e, policy))
        .collect()
}

#[cfg(feature = "parallel")]
mod parallel {
    use super::*;
    use rayon::prelude::*;

    /// Evaluate a batch of 2D orientation predicates in parallel.
    pub fn orient2d_batch_parallel<S>(cases: &[Orient2dCase<S>]) -> Vec<PredicateOutcome<Sign>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        orient2d_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of 2D orientation predicates in parallel with an explicit policy.
    pub fn orient2d_batch_parallel_with_policy<S>(
        cases: &[Orient2dCase<S>],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        crate::trace_dispatch!("predicated", "batch", "orient2d-parallel");
        cases
            .par_iter()
            .map(|(a, b, c)| orient2d_with_policy(a, b, c, policy))
            .collect()
    }

    /// Evaluate a batch of line-side classifications in parallel.
    pub fn classify_point_line_batch_parallel<S>(
        cases: &[Orient2dCase<S>],
    ) -> Vec<PredicateOutcome<LineSide>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        classify_point_line_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of line-side classifications in parallel with an explicit policy.
    pub fn classify_point_line_batch_parallel_with_policy<S>(
        cases: &[Orient2dCase<S>],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<LineSide>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        crate::trace_dispatch!("predicated", "batch", "classify-point-line-parallel");
        cases
            .par_iter()
            .map(|(from, to, point)| classify_point_line_with_policy(from, to, point, policy))
            .collect()
    }

    /// Evaluate a batch of 3D orientation predicates in parallel.
    pub fn orient3d_batch_parallel<S>(cases: &[Orient3dCase<S>]) -> Vec<PredicateOutcome<Sign>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        orient3d_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of 3D orientation predicates in parallel with an explicit policy.
    pub fn orient3d_batch_parallel_with_policy<S>(
        cases: &[Orient3dCase<S>],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        crate::trace_dispatch!("predicated", "batch", "orient3d-parallel");
        cases
            .par_iter()
            .map(|(a, b, c, d)| orient3d_with_policy(a, b, c, d, policy))
            .collect()
    }

    /// Evaluate a batch of explicit point-plane classifications in parallel.
    pub fn classify_point_plane_batch_parallel<S>(
        cases: &[PointPlaneCase<S>],
    ) -> Vec<PredicateOutcome<PlaneSide>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        classify_point_plane_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of explicit point-plane classifications in parallel with an explicit policy.
    pub fn classify_point_plane_batch_parallel_with_policy<S>(
        cases: &[PointPlaneCase<S>],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<PlaneSide>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        crate::trace_dispatch!("predicated", "batch", "classify-point-plane-parallel");
        cases
            .par_iter()
            .map(|(point, plane)| classify_point_plane_with_policy(point, plane, policy))
            .collect()
    }

    /// Evaluate a batch of oriented-plane classifications in parallel.
    pub fn classify_point_oriented_plane_batch_parallel<S>(
        cases: &[Orient3dCase<S>],
    ) -> Vec<PredicateOutcome<PlaneSide>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        classify_point_oriented_plane_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of oriented-plane classifications in parallel with an explicit policy.
    pub fn classify_point_oriented_plane_batch_parallel_with_policy<S>(
        cases: &[Orient3dCase<S>],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<PlaneSide>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        crate::trace_dispatch!(
            "predicated",
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

    /// Evaluate a batch of 2D in-circle predicates in parallel.
    pub fn incircle2d_batch_parallel<S>(cases: &[Incircle2dCase<S>]) -> Vec<PredicateOutcome<Sign>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        incircle2d_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of 2D in-circle predicates in parallel with an explicit policy.
    pub fn incircle2d_batch_parallel_with_policy<S>(
        cases: &[Incircle2dCase<S>],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        crate::trace_dispatch!("predicated", "batch", "incircle2d-parallel");
        cases
            .par_iter()
            .map(|(a, b, c, d)| incircle2d_with_policy(a, b, c, d, policy))
            .collect()
    }

    /// Evaluate a batch of 3D in-sphere predicates in parallel.
    pub fn insphere3d_batch_parallel<S>(cases: &[Insphere3dCase<S>]) -> Vec<PredicateOutcome<Sign>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        insphere3d_batch_parallel_with_policy(cases, PredicatePolicy::default())
    }

    /// Evaluate a batch of 3D in-sphere predicates in parallel with an explicit policy.
    pub fn insphere3d_batch_parallel_with_policy<S>(
        cases: &[Insphere3dCase<S>],
        policy: PredicatePolicy,
    ) -> Vec<PredicateOutcome<Sign>>
    where
        S: BorrowedPredicateScalar + Sync,
    {
        crate::trace_dispatch!("predicated", "batch", "insphere3d-parallel");
        cases
            .par_iter()
            .map(|(a, b, c, d, e)| insphere3d_with_policy(a, b, c, d, e, policy))
            .collect()
    }
}

#[cfg(feature = "parallel")]
pub use parallel::*;
