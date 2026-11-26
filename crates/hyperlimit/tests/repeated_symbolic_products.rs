//! Symbolic product identities through direct and retained geometric predicates.
//!
//! Reusing the same points and retained constructions across STRICT,
//! APPROXIMATE_512, and STRICT checks policy-sensitive outcomes in one process.
//! These fixtures do not measure cache hits or perform serialized replay.

use hyperlimit::{
    Certainty, LineSide, Point2, Point3, PredicateOutcome, PredicatePolicy, Real, Sign,
    Triangle3Location,
};

/// Build three positive-coordinate fixtures from constants, radicals, and a chord normal.
///
/// The last point is the radius-15 circle point at `pi/20`, shifted by the unit
/// left normal of the chord from `(15, 0)`. Its normalization requires successful
/// square root and division by the nonzero chord length.
fn symbolic_points() -> [Point2; 3] {
    let angle = (Real::pi() / Real::from(20)).unwrap();
    let cosine = angle.clone().cos();
    let sine = angle.sin();
    let dx = Real::from(15) * (&cosine - Real::one());
    let dy = Real::from(15) * &sine;
    let length = (&dx * &dx + &dy * &dy).sqrt().unwrap();
    [
        Point2::new(Real::e(), Real::pi()),
        Point2::new(Real::from(2).sqrt().unwrap(), Real::from(3).sqrt().unwrap()),
        Point2::new(
            Real::from(15) * cosine - (&dy / &length).unwrap(),
            Real::from(15) * sine + (&dx / &length).unwrap(),
        ),
    ]
}

/// Require a decided expected value with Exact certainty, leaving stage unconstrained.
fn exact<T: PartialEq + std::fmt::Debug>(outcome: PredicateOutcome<T>, expected: T) {
    assert!(
        matches!(outcome, PredicateOutcome::Decided { value, certainty: Certainty::Exact, .. } if value == expected)
    );
}

/// Require exact zero/On results for repeated points through direct and retained paths.
///
/// Four point arrangements run through both policies and back to STRICT. A
/// separate x shift of `2^-80` gives determinant `-y*2^-80`; reversing it must
/// reverse the exact nonzero sign.
#[test]
fn repeated_symbolic_points_have_zero_orientation_under_replay() {
    let origin = Point2::new(Real::zero(), Real::zero());
    for point in symbolic_points() {
        let clone = point.clone();
        for [a, b, c] in [
            [&origin, &point, &clone],
            [&point, &origin, &clone],
            [&point, &clone, &origin],
            [&point, &clone, &point],
        ] {
            let line = hyperlimit::line2_orientation(a, b);
            let query = hyperlimit::line2_orientation_query(c);
            for policy in [
                PredicatePolicy::STRICT,
                PredicatePolicy::APPROXIMATE_512,
                PredicatePolicy::STRICT,
            ] {
                exact(hyperlimit::orient2(a, b, c, policy), Sign::Zero);
                exact(
                    hyperlimit::classify_point_line_with_orientation(a, b, c, &line, policy),
                    LineSide::On,
                );
                exact(
                    hyperlimit::classify_point_line_with_orientation_and_query(
                        a, b, c, &line, &query, policy,
                    ),
                    LineSide::On,
                );
            }
        }
        // y > 0 for each independently constructed point. Moving only x by
        // 2^-80 gives determinant -y*2^-80, so distinct points stay distinct.
        let near = Point2::new(
            &point.x + Real::from(2).powi_i64(-80).unwrap(),
            point.y.clone(),
        );
        exact(
            hyperlimit::orient2(&origin, &point, &near, PredicatePolicy::STRICT),
            Sign::Negative,
        );
        exact(
            hyperlimit::orient2(&origin, &near, &point, PredicatePolicy::STRICT),
            Sign::Positive,
        );
    }
}

/// Check exact normal signs and off-plane/degenerate classifications across policies.
///
/// The normal is `(20*y, -20*x, 0)` and its dot product with the query is
/// `20*(x*x+y*y) > 0`. Direct and retained paths must agree on OffPlane;
/// repeating a triangle vertex must yield Degenerate.
#[test]
fn vertical_symbolic_triangles_retain_certified_normal_components() {
    let origin = Point3::new(Real::zero(), Real::zero(), Real::zero());
    for point in symbolic_points() {
        // x,y are positive, hence the normal is (20*y, -20*x, 0).
        let b = Point3::new(point.x.clone(), point.y.clone(), Real::zero());
        let c = Point3::new(point.x.clone(), point.y.clone(), Real::from(20));
        let query = Point3::new(point.y.clone(), -point.x.clone(), Real::one());
        let retained = hyperlimit::triangle3_orientation(&origin, &b, &c, PredicatePolicy::STRICT);
        exact(
            retained.normal_signs(),
            [Sign::Positive, Sign::Negative, Sign::Zero],
        );
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            // Its normal dotted with query is 20*(x*x+y*y), strictly positive.
            exact(
                hyperlimit::classify_point_triangle3(&origin, &b, &c, &query, policy),
                Triangle3Location::OffPlane,
            );
            exact(
                hyperlimit::classify_point_triangle3_with_orientation(
                    &origin, &b, &c, &query, &retained, policy,
                ),
                Triangle3Location::OffPlane,
            );
            exact(
                hyperlimit::classify_point_triangle3(&b, &b.clone(), &c, &query, policy),
                Triangle3Location::Degenerate,
            );
        }
    }
}

/// Keep the angle-addition determinant unresolved under STRICT after approximate queries.
///
/// Two rounds require strict uncertainty followed by an approximate Zero, then
/// a final strict query must remain unknown. The same source points are reused;
/// no particular uncertainty reason or stage is asserted.
#[test]
fn unrelated_unresolved_determinant_keeps_its_policy_receipt() {
    let z = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    let a = Point2::new(z, Real::zero());
    let b = Point2::new(Real::zero(), Real::one());
    let c = Point2::new(Real::zero(), Real::zero());
    for _ in 0..2 {
        assert!(matches!(
            hyperlimit::orient2(&a, &b, &c, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            hyperlimit::orient2(&a, &b, &c, PredicatePolicy::APPROXIMATE_512),
            PredicateOutcome::Decided {
                value: Sign::Zero,
                certainty: Certainty::Approximate,
                ..
            }
        ));
    }
    assert!(matches!(
        hyperlimit::orient2(&a, &b, &c, PredicatePolicy::STRICT),
        PredicateOutcome::Unknown { .. }
    ));
}
