//! Plane classification helpers.

use crate::classify::PlaneSide;
use crate::orient::{Point3, orient3d_with_policy};
use crate::predicate::{
    Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign, SignKnowledge,
};
use crate::resolve::{map_outcome, resolve_scalar_sign};
use crate::scalar::{BorrowedPredicateScalar, PredicateScalar};

pub use crate::batch::{
    Orient3dCase, PointPlaneCase, classify_point_oriented_plane_batch,
    classify_point_oriented_plane_batch_with_policy, classify_point_plane_batch,
    classify_point_plane_batch_with_policy,
};
#[cfg(feature = "parallel")]
pub use crate::batch::{
    classify_point_oriented_plane_batch_parallel,
    classify_point_oriented_plane_batch_parallel_with_policy, classify_point_plane_batch_parallel,
    classify_point_plane_batch_parallel_with_policy,
};

/// Plane represented by `normal . point + offset = 0`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane3<S> {
    /// Plane normal vector.
    pub normal: Point3<S>,
    /// Constant offset in `normal . point + offset = 0`.
    pub offset: S,
}

impl<S> Plane3<S> {
    /// Construct a plane from a normal vector and offset.
    pub const fn new(normal: Point3<S>, offset: S) -> Self {
        Self { normal, offset }
    }
}

/// Classify a point relative to a plane.
pub fn classify_point_plane<S: BorrowedPredicateScalar>(
    point: &Point3<S>,
    plane: &Plane3<S>,
) -> PredicateOutcome<PlaneSide> {
    classify_point_plane_with_policy(point, plane, PredicatePolicy::default())
}

/// Classify a point relative to a plane with an explicit escalation policy.
pub fn classify_point_plane_with_policy<S: BorrowedPredicateScalar>(
    point: &Point3<S>,
    plane: &Plane3<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    // Exact symbolic backends can spend more constructing the plane-side scalar than the
    // conservative f64 filter costs, so try the filter before scalar arithmetic when the
    // backend opts in.
    if S::prefer_f64_filter_before_arithmetic() {
        if let Some(outcome) = classify_point_plane_filter(point, plane) {
            crate::trace_dispatch!("liminal", "classify_point_plane", "f64-point-filter-hit");
            return outcome;
        }
        crate::trace_dispatch!(
            "liminal",
            "classify_point_plane",
            "f64-point-filter-miss"
        );
    }

    crate::trace_dispatch!("liminal", "classify_point_plane", "scalar-dot");
    let x_term = mul(&plane.normal.x, &point.x);
    let y_term = mul(&plane.normal.y, &point.y);
    let z_term = mul(&plane.normal.z, &point.z);
    let xy = add(&x_term, &y_term);
    let xyz = add(&xy, &z_term);
    let value = add(&xyz, &plane.offset);

    map_outcome(
        resolve_scalar_sign(
            &value,
            policy,
            || {
                classify_point_plane_filter(point, plane)
                    .map(|outcome| map_outcome(outcome, sign_from_plane_side))
            },
            || None,
            || None,
            RefinementNeed::ScalarRefinement,
        ),
        PlaneSide::from,
    )
}

/// Classify a point relative to the oriented plane through `a`, `b`, and `c`.
pub fn classify_point_oriented_plane<S: BorrowedPredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    point: &Point3<S>,
) -> PredicateOutcome<PlaneSide> {
    classify_point_oriented_plane_with_policy(a, b, c, point, PredicatePolicy::default())
}

/// Classify a point relative to the oriented plane through `a`, `b`, and `c`
/// with an explicit escalation policy.
pub fn classify_point_oriented_plane_with_policy<S: BorrowedPredicateScalar>(
    a: &Point3<S>,
    b: &Point3<S>,
    c: &Point3<S>,
    point: &Point3<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    crate::trace_dispatch!("liminal", "classify_point_oriented_plane", "orient3d");
    map_outcome(
        orient3d_with_policy(a, b, c, point, policy),
        PlaneSide::from,
    )
}

fn sign_from_plane_side(side: PlaneSide) -> Sign {
    match side {
        PlaneSide::Below => Sign::Negative,
        PlaneSide::On => Sign::Zero,
        PlaneSide::Above => Sign::Positive,
    }
}

fn add<S: BorrowedPredicateScalar>(left: &S, right: &S) -> S {
    left.add_ref(right)
}

fn mul<S: BorrowedPredicateScalar>(left: &S, right: &S) -> S {
    left.mul_ref(right)
}

fn classify_point_plane_filter<S: PredicateScalar>(
    point: &Point3<S>,
    plane: &Plane3<S>,
) -> Option<PredicateOutcome<PlaneSide>> {
    // This mirrors the orientation filters: return only when the floating error bound
    // proves the side, otherwise let structural/exact scalar paths decide.
    let nx = plane.normal.x.to_f64()?;
    let ny = plane.normal.y.to_f64()?;
    let nz = plane.normal.z.to_f64()?;
    let d = plane.offset.to_f64()?;
    let x = point.x.to_f64()?;
    let y = point.y.to_f64()?;
    let z = point.z.to_f64()?;

    let value = nx.mul_add(x, ny.mul_add(y, nz.mul_add(z, d)));
    let scale = nx.abs() * x.abs() + ny.abs() * y.abs() + nz.abs() * z.abs() + d.abs();

    match crate::filter::det_sign_filter(value, scale, 16.0) {
        SignKnowledge::Known { sign, certainty } => {
            crate::trace_dispatch!("liminal", "classify_point_plane_filter", "decided");
            Some(PredicateOutcome::decided(
                PlaneSide::from(sign),
                certainty,
                Escalation::Filter,
            ))
        }
        SignKnowledge::NonZero => {
            crate::trace_dispatch!(
                "liminal",
                "classify_point_plane_filter",
                "nonzero-no-sign"
            );
            None
        }
        SignKnowledge::Unknown => {
            crate::trace_dispatch!("liminal", "classify_point_plane_filter", "unknown");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classifies_point_plane() {
        let plane = Plane3::new(Point3::new(0.0, 0.0, 1.0), -2.0);

        assert_eq!(
            classify_point_plane(&Point3::new(0.0, 0.0, 3.0), &plane).value(),
            Some(PlaneSide::Above)
        );
        assert_eq!(
            classify_point_plane(&Point3::new(0.0, 0.0, 1.0), &plane).value(),
            Some(PlaneSide::Below)
        );
    }

    #[test]
    fn classifies_point_oriented_plane_from_points() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(0.0, 1.0, 0.0);

        assert_eq!(
            classify_point_oriented_plane(&a, &b, &c, &Point3::new(0.0, 0.0, 1.0)).value(),
            Some(PlaneSide::Below)
        );
    }

    #[cfg(feature = "hyperreal")]
    #[test]
    fn classifies_point_plane_with_hyperreal_filter_before_refinement() {
        use crate::predicate::{Certainty, Escalation};

        let plane = Plane3::new(
            Point3::new(hyperreal::Real::from(1), 0.into(), 0.into()),
            (-4).into(),
        );
        let point = Point3::new(hyperreal::Real::pi(), 0.into(), 0.into());

        assert_eq!(
            classify_point_plane(&point, &plane),
            PredicateOutcome::decided(PlaneSide::Below, Certainty::Filtered, Escalation::Filter)
        );
    }
}
