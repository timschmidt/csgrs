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
    // A conic arc against a rational curve has no admitted route yet.
    let circle = Curve3::circle_arc(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Real::one(),
        Real::zero(),
        Real::one(),
    )
    .unwrap();
    assert!(matches!(
        circle.intersect_curve(&bump()),
        Err(GeometryError::UnsupportedIntersection)
    ));
}

/// y = 1 - x^2 (controls (-1,0), (0,2), (1,0)) and y = x^2 (controls (-1,1),
/// (0,-1), (1,1)) in z = 0 meet where x = ±√2/2, at t = s = 1/2 ± √2/4.
#[test]
fn coplanar_rational_curves_meet_at_selected_parameters() {
    let cap = Curve3::rational_bezier(
        vec![pi(-1, 0, 0), pi(0, 2, 0), pi(1, 0, 0)],
        vec![Real::one(); 3],
    )
    .unwrap();
    let cup = Curve3::rational_bezier(
        vec![pi(-1, 1, 0), pi(0, -1, 0), pi(1, 1, 0)],
        vec![Real::one(); 3],
    )
    .unwrap();
    let found = points(&cap, &cup);
    assert_eq!(found.len(), 2);
    let root = Real::from(2).sqrt().unwrap();
    for (point, sign) in found.iter().zip([-1, 1]) {
        let offset = (Real::from(sign) * &root / Real::from(4)).unwrap();
        assert!(equal(&point.first, q(1, 2) + &offset));
        assert!(equal(&point.second, q(1, 2) + &offset));
        assert!(
            point
                .point
                .coincides_with(&p(Real::from(2) * &offset, q(1, 2), Real::zero()).into())
                .unwrap()
        );
    }
    // A sub-arc of the cap overlaps it on [0, 1/2] of the cap's chart.
    let half = cap.subcurve(&Real::zero(), &q(1, 2)).unwrap();
    let CurveCurveIntersection::Overlap { first, second } = cap.intersect_curve(&half).unwrap()
    else {
        panic!("one coincident range required");
    };
    assert!(equal(first.start(), Real::zero()) && equal(first.end(), q(1, 2)));
    assert!(equal(second.start(), Real::zero()) && equal(second.end(), Real::one()));
}

/// The radius-5 quarter circle from (5, 0) to (0, 5) as a planar image in
/// z = 0: the coplanar line x = y meets it at (5√2/2, 5√2/2, 0), and the
/// vertical line through (3, 4) pierces it there at t = 1/2.
#[test]
fn lines_meet_planar_images_in_and_across_their_plane() {
    let arc = hypercurve::CircularArc2::try_from_center(
        hypercurve::Point2::from_values(5, 0),
        hypercurve::Point2::from_values(0, 5),
        hypercurve::Point2::from_values(0, 0),
        false,
    )
    .unwrap();
    let image = Curve3::planar_image(
        hypercurve::Curve2::from(arc),
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
    )
    .unwrap();
    let diagonal = line(pi(0, 0, 0), pi(6, 6, 0));
    let found = points(&diagonal, &image);
    assert_eq!(found.len(), 1);
    let coordinate = (Real::from(5) * Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
    assert!(
        found[0]
            .point
            .coincides_with(&p(coordinate.clone(), coordinate, Real::zero()).into())
            .unwrap()
    );
    let vertical = line(pi(3, 4, -1), pi(3, 4, 1));
    let found = points(&vertical, &image);
    assert_eq!(found.len(), 1);
    assert!(equal(&found[0].first, q(1, 2)));
    assert!(found[0].point.coincides_with(&pi(3, 4, 0).into()).unwrap());
    let swapped = points(&image, &vertical);
    assert!(equal(&swapped[0].second, q(1, 2)));
    // A vertical line off the arc misses it.
    assert!(matches!(
        line(pi(4, 4, -1), pi(4, 4, 1))
            .intersect_curve(&image)
            .unwrap(),
        CurveCurveIntersection::None
    ));
}

/// The quadratic with controls (1, 1, 0), (0, 3/4, 3/2), (-1, -1, 3) has
/// x = 1 - 2s, z = 3s and y = 1 - s/2 - 3s^2/2. The cubic lies in the plane
/// 4z - 3x = 6, which the quadratic crosses only at s = 1/2, at (0, 3/8, 3/2)
/// = cubic(1/2). The curves are not coplanar.
#[test]
fn noncoplanar_rational_curves_meet_at_their_single_common_point() {
    let quadratic = |shift: i64| {
        Curve3::rational_bezier(
            vec![
                p(1.into(), Real::from(1 + shift), 0.into()),
                p(0.into(), q(3, 4) + Real::from(shift), q(3, 2)),
                p((-1).into(), Real::from(shift - 1), 3.into()),
            ],
            vec![Real::one(); 3],
        )
        .unwrap()
    };
    let found = points(&bump(), &quadratic(0));
    assert_eq!(found.len(), 1);
    assert!(equal(&found[0].first, q(1, 2)));
    assert!(equal(&found[0].second, q(1, 2)));
    assert!(
        found[0]
            .point
            .coincides_with(&p(0.into(), q(3, 8), q(3, 2)).into())
            .unwrap()
    );
    let swapped = points(&quadratic(0), &bump());
    assert_eq!(swapped.len(), 1);
    // Shifted by (0, 1, 0), the plane crossing (0, 11/8, 3/2) is off the cubic.
    assert!(matches!(
        bump().intersect_curve(&quadratic(1)).unwrap(),
        CurveCurveIntersection::None
    ));
    // Shifted by (0, 0, 1), the xy projections still cross at the old pair, but
    // the heights differ there (3/2 versus 5/2); the only plane crossing,
    // s = 5/18, is off the cubic. Exact verification rejects the candidate.
    let lifted = Curve3::rational_bezier(
        vec![pi(1, 1, 1), p(0.into(), q(3, 4), q(5, 2)), pi(-1, -1, 4)],
        vec![Real::one(); 3],
    )
    .unwrap();
    assert!(matches!(
        bump().intersect_curve(&lifted).unwrap(),
        CurveCurveIntersection::None
    ));
    // The collinear x-axis segment never meets the cubic, whose only points
    // with y = 0 are (-2, 0, 0) and (2, 0, 3).
    let along = Curve3::rational_bezier(
        vec![pi(0, 0, 0), pi(1, 0, 0), pi(2, 0, 0)],
        vec![Real::one(); 3],
    )
    .unwrap();
    assert!(matches!(
        bump().intersect_curve(&along).unwrap(),
        CurveCurveIntersection::None
    ));
}

/// The x-axis spline x = 2u - 6 and the quadratic in x = 0 with
/// y = 2s(2s - 1), z = 2s - 1 meet only at the origin: u = 3, s = 1/2.
#[test]
fn nurbs_meets_a_transverse_quadratic_at_its_knot() {
    let spline = Curve3::nurbs(
        2,
        vec![pi(-2, 0, 0), pi(-1, 0, 0), pi(1, 0, 0), pi(2, 0, 0)],
        vec![Real::one(); 4],
        [2, 2, 2, 3, 4, 4, 4].map(Real::from).to_vec(),
    )
    .unwrap();
    let quadratic = Curve3::rational_bezier(
        vec![pi(0, 0, -1), pi(0, -1, 0), pi(0, 2, 1)],
        vec![Real::one(); 3],
    )
    .unwrap();
    let found = points(&spline, &quadratic);
    assert_eq!(found.len(), 1);
    assert!(equal(&found[0].first, 3.into()));
    assert!(equal(&found[0].second, q(1, 2)));
    assert!(
        found[0]
            .point
            .coincides_with(&Point3::origin().into())
            .unwrap()
    );
}
