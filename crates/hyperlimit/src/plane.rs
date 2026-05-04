//! Plane classification helpers.

use crate::classify::PlaneSide;
use crate::orient::Point3;
use crate::predicate::{
    Certainty, Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign, SignKnowledge,
};
use crate::scalar::PredicateScalar;

/// Plane represented by `normal . point + offset = 0`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane3<S> {
    pub normal: Point3<S>,
    pub offset: S,
}

impl<S> Plane3<S> {
    pub const fn new(normal: Point3<S>, offset: S) -> Self {
        Self { normal, offset }
    }
}

/// Classify a point relative to a plane.
pub fn classify_point_plane<S: PredicateScalar>(
    point: &Point3<S>,
    plane: &Plane3<S>,
) -> PredicateOutcome<PlaneSide> {
    classify_point_plane_with_policy(point, plane, PredicatePolicy::default())
}

/// Classify a point relative to a plane with an explicit escalation policy.
pub fn classify_point_plane_with_policy<S: PredicateScalar>(
    point: &Point3<S>,
    plane: &Plane3<S>,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    let value = plane.normal.x.clone() * point.x.clone()
        + plane.normal.y.clone() * point.y.clone()
        + plane.normal.z.clone() * point.z.clone()
        + plane.offset.clone();

    match value.known_sign() {
        SignKnowledge::Known { sign, certainty } => {
            PredicateOutcome::decided(PlaneSide::from(sign), certainty, Escalation::Structural)
        }
        SignKnowledge::NonZero | SignKnowledge::Unknown => {
            classify_point_plane_filter(point, plane)
                .or_else(|| {
                    if policy.allow_approximate {
                        let sign = Sign::from_f64(value.to_f64()?)?;
                        Some(PredicateOutcome::decided(
                            PlaneSide::from(sign),
                            Certainty::Approximate,
                            Escalation::Undecided,
                        ))
                    } else {
                        None
                    }
                })
                .unwrap_or_else(|| {
                    PredicateOutcome::unknown(RefinementNeed::RobustFallback, Escalation::Undecided)
                })
        }
    }
}

fn classify_point_plane_filter<S: PredicateScalar>(
    point: &Point3<S>,
    plane: &Plane3<S>,
) -> Option<PredicateOutcome<PlaneSide>> {
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
        SignKnowledge::Known { sign, certainty } => Some(PredicateOutcome::decided(
            PlaneSide::from(sign),
            certainty,
            Escalation::Filter,
        )),
        SignKnowledge::NonZero | SignKnowledge::Unknown => None,
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
}
