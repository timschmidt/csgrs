//! Exact point-distance comparison predicates.
//!
//! Distance comparisons use squared Euclidean distance so predicate callers do
//! not force square-root construction or lossy approximations.

use core::cmp::Ordering;

use crate::geometry::Point2;
use crate::predicate::{PredicateOutcome, PredicatePolicy};
use crate::predicates::order::compare_reals_with_policy;
use crate::real::{add_ref, mul_ref, sub_ref};
use hyperreal::Real;

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

fn squared_distance2(left: &Point2, right: &Point2) -> Real {
    let dx = sub_ref(&right.x, &left.x);
    let dy = sub_ref(&right.y, &left.y);
    add_ref(&mul_ref(&dx, &dx), &mul_ref(&dy, &dy))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(hyperreal::Real::from(x), hyperreal::Real::from(y))
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
}
