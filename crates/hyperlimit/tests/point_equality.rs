//! A different coordinate proves inequality even when another is undecided.
use hyperlimit::{Certainty, Point2, Point3, PredicateOutcome, PredicatePolicy, Real};

fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
fn point_inequality_uses_any_certified_coordinate_witness() {
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let unknown = unknown_zero();
        let left = Point2::new(unknown.clone(), Real::one());
        let right = Point2::new(Real::zero(), Real::zero());
        assert!(matches!(
            hyperlimit::point2_equal(&left, &right, policy),
            PredicateOutcome::Decided {
                value: false,
                certainty: Certainty::Exact | Certainty::Filtered,
                ..
            }
        ));
        let left = Point3::new(unknown.clone(), unknown, Real::one());
        let right = Point3::new(Real::zero(), Real::zero(), Real::zero());
        assert!(matches!(
            hyperlimit::point3_equal(&left, &right, policy),
            PredicateOutcome::Decided {
                value: false,
                certainty: Certainty::Exact | Certainty::Filtered,
                ..
            }
        ));
    }
}

#[test]
fn equality_still_needs_all_coordinates_and_ordering_keeps_its_contract() {
    let left = Point2::new(unknown_zero(), Real::zero());
    let right = Point2::new(Real::zero(), Real::zero());
    assert!(matches!(
        hyperlimit::point2_equal(&left, &right, PredicatePolicy::STRICT),
        PredicateOutcome::Unknown { .. }
    ));
    let different_y = Point2::new(left.x.clone(), Real::one());
    assert!(matches!(
        hyperlimit::compare_point2_lexicographic(&different_y, &right, PredicatePolicy::STRICT),
        PredicateOutcome::Unknown { .. }
    ));
}
