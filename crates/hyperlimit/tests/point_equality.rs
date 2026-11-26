//! A different coordinate proves inequality even when another is undecided.
//!
//! These controls separate the coordinate-equality witness search from the
//! ordered dependencies of lexicographic comparison, and retain a symbolic
//! identity whose equality is now certified.
use hyperlimit::{Certainty, Point2, Point3, PredicateOutcome, PredicatePolicy, Real};

/// Builds sin²(e)+cos²(e)-1, retained as the original exact-equality fixture.
fn original_unit_circle_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

/// Builds an angle-addition identity used by the strict Unknown control below.
/// Its mathematical value is zero; the fixture tests the current proof frontier.
fn unknown_zero() -> Real {
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}

/// Checks a later certified unequal coordinate under strict/approximate/strict
/// policies, rebuilding inputs each iteration rather than testing shared-cache replay.
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

/// Checks that unresolved equality needs all coordinates, while unresolved x
/// prevents lexicographic order even when y is provably different.
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

/// Reuses the original identity through strict/approximate/strict queries and
/// requires certified point equality, lexicographic order and spatial inequality.
#[test]
fn original_unit_circle_points_now_have_certified_equality_and_order() {
    let left = Point2::new(original_unit_circle_zero(), Real::zero());
    let right = Point2::new(Real::zero(), Real::zero());
    let different_y = Point2::new(left.x.clone(), Real::one());
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        assert!(matches!(
            hyperlimit::point2_equal(&left, &right, policy),
            PredicateOutcome::Decided {
                value: true,
                certainty: Certainty::Exact | Certainty::Filtered,
                ..
            }
        ));
        assert!(matches!(
            hyperlimit::compare_point2_lexicographic(&different_y, &right, policy),
            PredicateOutcome::Decided {
                value: core::cmp::Ordering::Greater,
                certainty: Certainty::Exact | Certainty::Filtered,
                ..
            }
        ));
        let left3 = Point3::new(left.x.clone(), left.x.clone(), Real::one());
        let right3 = Point3::new(Real::zero(), Real::zero(), Real::zero());
        assert!(matches!(
            hyperlimit::point3_equal(&left3, &right3, policy),
            PredicateOutcome::Decided {
                value: false,
                certainty: Certainty::Exact | Certainty::Filtered,
                ..
            }
        ));
    }
}
