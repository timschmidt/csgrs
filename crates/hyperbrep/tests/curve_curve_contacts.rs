//! Complete incidence between finite lines and lines, rational Bézier curves
//! and NURBS curves in space.
//!
//! Expectations come from the defining geometry stated beside each witness.

use hyperbrep::*;
use std::cmp::Ordering;

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn p(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn pi(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

fn line(start: Point3, end: Point3) -> Curve3 {
    Curve3::line(start, end).unwrap()
}

fn equal(parameter: &Parameter, expected: Real) -> bool {
    parameter.compare(&Parameter::from_real(expected)).unwrap() == Ordering::Equal
}

fn points(first: &Curve3, second: &Curve3) -> Vec<CurveCurvePoint> {
    match first.intersect_curve(second).unwrap() {
        CurveCurveIntersection::Points(points) => {
            for point in &points {
                assert!(
                    point
                        .point
                        .coincides_with(&first.point_at(&point.first).unwrap())
                        .unwrap()
                );
                assert!(
                    point
                        .point
                        .coincides_with(&second.point_at(&point.second).unwrap())
                        .unwrap()
                );
            }
            points
        }
        other => panic!("isolated incidences required, found {other:?}"),
    }
}

/// Cubic with x = 4t - 2, y = 3t(1 - t)/2, z = 3t: points of equal y lie on
/// one line of direction (4, 0, 3).
fn bump() -> Curve3 {
    Curve3::rational_bezier(
        vec![
            p((-2).into(), 0.into(), 0.into()),
            p(q(-2, 3), q(1, 2), 1.into()),
            p(q(2, 3), q(1, 2), 2.into()),
            p(2.into(), 0.into(), 3.into()),
        ],
        vec![Real::one(); 4],
    )
    .unwrap()
}

#[test]
fn line_pairs_cross_miss_overlap_and_touch() {
    let crossing = points(
        &line(pi(0, 0, 0), pi(2, 2, 0)),
        &line(pi(0, 2, 0), pi(2, 0, 0)),
    );
    assert_eq!(crossing.len(), 1);
    assert!(equal(&crossing[0].first, q(1, 2)));
    assert!(equal(&crossing[0].second, q(1, 2)));
    assert!(
        crossing[0]
            .point
            .coincides_with(&pi(1, 1, 0).into())
            .unwrap()
    );
    // Skew lines and parallel offset lines miss.
    for second in [
        line(pi(0, 2, 1), pi(2, 0, 1)),
        line(pi(0, 1, 0), pi(2, 3, 0)),
    ] {
        assert!(matches!(
            line(pi(0, 0, 0), pi(2, 2, 0))
                .intersect_curve(&second)
                .unwrap(),
            CurveCurveIntersection::None
        ));
    }
    // Collinear segments [0, 2] and [1, 3] on the x axis share [1, 2].
    let CurveCurveIntersection::Overlap { first, second } = line(pi(0, 0, 0), pi(2, 0, 0))
        .intersect_curve(&line(pi(1, 0, 0), pi(3, 0, 0)))
        .unwrap()
    else {
        panic!("one coincident range required");
    };
    assert!(equal(first.start(), q(1, 2)) && equal(first.end(), Real::one()));
    assert!(equal(second.start(), Real::zero()) && equal(second.end(), q(1, 2)));
    // Collinear segments touching at (2, 0, 0) meet at one point.
    let touching = points(
        &line(pi(0, 0, 0), pi(2, 0, 0)),
        &line(pi(2, 0, 0), pi(3, 0, 0)),
    );
    assert_eq!(touching.len(), 1);
    assert!(equal(&touching[0].first, Real::one()));
    assert!(equal(&touching[0].second, Real::zero()));
}

/// y = 9/32 at t = 1/4 and 3/4, at (-1, 9/32, 3/4) and (1, 9/32, 9/4), both on
/// the line x = -3 + 6τ, z = -3/4 + 9τ/2 at τ = 1/3 and 2/3.
#[test]
fn line_meets_a_cubic_at_rational_points_in_both_orders() {
    let chord = line(
        p((-3).into(), q(9, 32), q(-3, 4)),
        p(3.into(), q(9, 32), q(15, 4)),
    );
    let found = points(&chord, &bump());
    assert_eq!(found.len(), 2);
    for (point, (tau, t)) in found.iter().zip([(q(1, 3), q(1, 4)), (q(2, 3), q(3, 4))]) {
        assert!(equal(&point.first, tau));
        assert!(equal(&point.second, t));
    }
    let swapped = points(&bump(), &chord);
    assert_eq!(swapped.len(), 2);
    assert!(equal(&swapped[0].first, q(1, 4)));
    assert!(equal(&swapped[0].second, q(1, 3)));
}

/// y = 1/4 at t = 1/2 ∓ √3/6: the points (4k, 1/4, 3/2 + 3k), k = ∓√3/6, lie on
/// the line x = -4 + 8τ at τ = 1/2 ∓ √3/12. Both parameters are selected.
#[test]
fn line_meets_a_cubic_at_selected_parameters() {
    let chord = line(
        p((-4).into(), q(1, 4), q(-3, 2)),
        p(4.into(), q(1, 4), q(9, 2)),
    );
    let found = points(&chord, &bump());
    assert_eq!(found.len(), 2);
    let root = Real::from(3).sqrt().unwrap();
    for (point, sign) in found.iter().zip([-1, 1]) {
        let k = (Real::from(sign) * &root / Real::from(6)).unwrap();
        assert!(equal(&point.second, q(1, 2) + &k));
        assert!(equal(
            &point.first,
            q(1, 2) + (k.clone() / Real::from(2)).unwrap()
        ));
        assert!(
            point
                .point
                .coincides_with(
                    &p(Real::from(4) * &k, q(1, 4), q(3, 2) + Real::from(3) * &k).into()
                )
                .unwrap()
        );
    }
}

/// Two quadratic spans on the x axis (knots [2,2,2,3,4,4,4], x = 2u - 6) meet
/// the segment x = 0, y in [-1, 1], at the interior knot u = 3, τ = 1/2.
#[test]
fn line_meets_a_nurbs_curve_at_its_knot() {
    let spline = Curve3::nurbs(
        2,
        vec![pi(-2, 0, 0), pi(-1, 0, 0), pi(1, 0, 0), pi(2, 0, 0)],
        vec![Real::one(); 4],
        [2, 2, 2, 3, 4, 4, 4].map(Real::from).to_vec(),
    )
    .unwrap();
    let found = points(&line(pi(0, -1, 0), pi(0, 1, 0)), &spline);
    assert_eq!(found.len(), 1);
    assert!(equal(&found[0].first, q(1, 2)));
    assert!(equal(&found[0].second, 3.into()));
    // A segment that stops short misses.
    assert!(matches!(
        line(pi(0, 1, 0), pi(0, 2, 0))
            .intersect_curve(&spline)
            .unwrap(),
        CurveCurveIntersection::None
    ));
}

#[test]
fn unsupported_pairs_remain_typed() {
    // A rational curve along the line.
    let along = Curve3::rational_bezier(
        vec![pi(0, 0, 0), pi(1, 0, 0), pi(2, 0, 0)],
        vec![Real::one(); 3],
    )
    .unwrap();
    assert!(matches!(
        line(pi(0, 0, 0), pi(2, 0, 0)).intersect_curve(&along),
        Err(GeometryError::UnsupportedIntersection)
    ));
    // Two non-line families.
    assert!(matches!(
        bump().intersect_curve(&along),
        Err(GeometryError::UnsupportedIntersection)
    ));
}
