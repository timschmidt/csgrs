//! Exact point-distance comparison predicates.
//!
//! Distance comparisons use squared Euclidean distance so predicate callers do
//! not force square-root construction or lossy approximations.

use core::cmp::Ordering;

use crate::classify::SpherePointLocation;
use crate::geometry::{Point2, Point3};
use crate::predicate::{PredicateOutcome, PredicatePolicy};
use crate::predicates::order::compare_reals_with_policy;
use crate::real::{add_ref, mul_ref, sub_ref};
use hyperreal::Real;

/// Reusable explicit 3D sphere classifier.
#[derive(Clone, Copy, Debug)]
pub struct PreparedExplicitSphere3<'a> {
    center: &'a Point3,
    radius_squared: &'a Real,
}

impl<'a> PreparedExplicitSphere3<'a> {
    /// Prepare an explicit sphere from a center and squared radius.
    pub fn new(center: &'a Point3, radius_squared: &'a Real) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_explicit_sphere3", "new");
        Self {
            center,
            radius_squared,
        }
    }

    /// Return the borrowed sphere center.
    pub const fn center(&self) -> &'a Point3 {
        self.center
    }

    /// Return the borrowed squared radius.
    pub const fn radius_squared(&self) -> &'a Real {
        self.radius_squared
    }

    /// Classify a point using the default predicate policy.
    pub fn classify_point(&self, point: &Point3) -> PredicateOutcome<SpherePointLocation> {
        self.classify_point_with_policy(point, PredicatePolicy::default())
    }

    /// Classify a point using an explicit predicate policy.
    pub fn classify_point_with_policy(
        &self,
        point: &Point3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<SpherePointLocation> {
        classify_point_sphere3_with_policy(self.center, self.radius_squared, point, policy)
    }
}

/// Compare squared distances from `anchor` to `left` and `right`.
pub fn compare_point2_distance_squared(
    anchor: &Point2,
    left: &Point2,
    right: &Point2,
) -> PredicateOutcome<Ordering> {
    compare_point2_distance_squared_with_policy(anchor, left, right, PredicatePolicy::default())
}

/// Compare squared distances from `anchor` to `left` and `right` with an
/// explicit predicate escalation policy.
///
/// Squared-distance comparison is the exact form needed by nearest-candidate
/// selection in bridge construction, snapping, and broad-phase refinement. It
/// avoids constructing a square root and asks the Real sign resolver to
/// certify `|anchor-left|^2 - |anchor-right|^2`. This is the standard
/// distance-ordering reduction used throughout computational geometry texts
/// such as de Berg, Cheong, van Kreveld, and Overmars, *Computational Geometry:
/// Algorithms and Applications*, 3rd ed., Springer, 2008, and it keeps the
/// final sign decision in the exact-geometric-computation model of Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
pub fn compare_point2_distance_squared_with_policy(
    anchor: &Point2,
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let left_distance = squared_distance2(anchor, left);
    let right_distance = squared_distance2(anchor, right);
    compare_reals_with_policy(&left_distance, &right_distance, policy)
}

/// Compare squared 3D distances from `anchor` to `left` and `right`.
pub fn compare_point3_distance_squared(
    anchor: &Point3,
    left: &Point3,
    right: &Point3,
) -> PredicateOutcome<Ordering> {
    compare_point3_distance_squared_with_policy(anchor, left, right, PredicatePolicy::default())
}

/// Compare squared 3D distances from `anchor` to `left` and `right` with an
/// explicit predicate escalation policy.
///
/// This is the 3D lift of [`compare_point2_distance_squared`]. It compares
/// `|anchor-left|^2` and `|anchor-right|^2` through exact `Real` predicates,
/// avoiding square-root construction and primitive-float tie decisions.
pub fn compare_point3_distance_squared_with_policy(
    anchor: &Point3,
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let left_distance = squared_distance3(anchor, left);
    let right_distance = squared_distance3(anchor, right);
    compare_reals_with_policy(&left_distance, &right_distance, policy)
}

/// Classify a point relative to an explicit 3D sphere with squared radius.
pub fn classify_point_sphere3(
    center: &Point3,
    radius_squared: &Real,
    point: &Point3,
) -> PredicateOutcome<SpherePointLocation> {
    classify_point_sphere3_with_policy(center, radius_squared, point, PredicatePolicy::default())
}

/// Classify a point relative to an explicit 3D sphere with squared radius and
/// an explicit predicate escalation policy.
///
/// The API accepts squared radius so callers do not need to construct square
/// roots. Domain validation for nonnegative radius remains with the caller that
/// owns the sphere object; this predicate only certifies the distance relation.
pub fn classify_point_sphere3_with_policy(
    center: &Point3,
    radius_squared: &Real,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<SpherePointLocation> {
    let distance_squared = squared_distance3(center, point);
    match compare_reals_with_policy(&distance_squared, radius_squared, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let location = match value {
                Ordering::Less => SpherePointLocation::Inside,
                Ordering::Equal => SpherePointLocation::On,
                Ordering::Greater => SpherePointLocation::Outside,
            };
            PredicateOutcome::decided(location, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn squared_distance2(left: &Point2, right: &Point2) -> Real {
    let dx = sub_ref(&right.x, &left.x);
    let dy = sub_ref(&right.y, &left.y);
    add_ref(&mul_ref(&dx, &dx), &mul_ref(&dy, &dy))
}

fn squared_distance3(left: &Point3, right: &Point3) -> Real {
    let dx = sub_ref(&right.x, &left.x);
    let dy = sub_ref(&right.y, &left.y);
    let dz = sub_ref(&right.z, &left.z);
    let xy = add_ref(&mul_ref(&dx, &dx), &mul_ref(&dy, &dy));
    add_ref(&xy, &mul_ref(&dz, &dz))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(hyperreal::Real::from(x), hyperreal::Real::from(y))
    }

    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(
            hyperreal::Real::from(x),
            hyperreal::Real::from(y),
            hyperreal::Real::from(z),
        )
    }

    #[test]
    fn squared_distance_comparison_avoids_square_roots() {
        let anchor = p2(0, 0);
        let near = p2(3, 4);
        let far = p2(6, 8);
        let also_near = p2(-3, -4);

        assert_eq!(
            compare_point2_distance_squared(&anchor, &near, &far).value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            compare_point2_distance_squared(&anchor, &near, &also_near).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_point2_distance_squared(&anchor, &far, &near).value(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn squared_distance3_comparison_avoids_square_roots() {
        let anchor = p3(0, 0, 0);
        let near = p3(1, 2, 2);
        let far = p3(2, 4, 4);
        let also_near = p3(-1, -2, -2);

        assert_eq!(
            compare_point3_distance_squared(&anchor, &near, &far).value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            compare_point3_distance_squared(&anchor, &near, &also_near).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_point3_distance_squared(&anchor, &far, &near).value(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn point_sphere3_classifier_uses_squared_radius() {
        let center = p3(0, 0, 0);
        let radius_squared = hyperreal::Real::from(25);
        let sphere = PreparedExplicitSphere3::new(&center, &radius_squared);

        assert_eq!(sphere.center(), &center);
        assert_eq!(sphere.radius_squared(), &radius_squared);
        assert_eq!(
            classify_point_sphere3(&center, &radius_squared, &p3(1, 2, 2)).value(),
            Some(SpherePointLocation::Inside)
        );
        assert_eq!(
            sphere.classify_point(&p3(3, 4, 0)).value(),
            Some(SpherePointLocation::On)
        );
        assert_eq!(
            sphere.classify_point(&p3(6, 0, 0)).value(),
            Some(SpherePointLocation::Outside)
        );
    }
}
