use hypercurve::{
    BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CubicBezier2, CurvePolicy, Point2, QuadraticBezier2,
    Real,
};
use hypersolve::AlgebraicRootKind;
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

fn polynomial(coefficients: Vec<Real>) -> BezierParameterPolynomial {
    decided(BezierParameterPolynomial::try_new_power_basis(coefficients, &policy()).unwrap())
}

fn interval(start: Real, end: Real) -> BezierParameterInterval {
    decided(BezierParameterInterval::try_new(start, end, &policy()).unwrap())
}

fn isolate(
    polynomial: BezierParameterPolynomial,
    interval: BezierParameterInterval,
) -> BezierAlgebraicParameter2 {
    decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap())
}

fn sqrt_half_parameter() -> BezierAlgebraicParameter2 {
    isolate(polynomial(vec![r(-1), r(0), r(2)]), interval(q(1, 2), r(1)))
}

#[test]
fn quadratic_point_and_tangent_images_retain_algebraic_coordinate_evidence() {
    let curve = QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2));
    let parameter = sqrt_half_parameter();

    let point = curve
        .point_at_algebraic_parameter(&parameter, &policy())
        .unwrap();
    let tangent = curve
        .tangent_at_algebraic_parameter(&parameter, &policy())
        .unwrap();

    assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
    assert_eq!(point.x().unwrap().coefficients(), &[r(0), r(0), r(1)]);
    assert!(point.x().unwrap().representation().unwrap().is_valid());
    assert_eq!(point.y().unwrap().coefficients(), &[r(0), r(2), r(0)]);
    assert_eq!(
        point.y().unwrap().representation().unwrap().kind,
        AlgebraicRootKind::IsolatingInterval
    );

    assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
    assert_eq!(tangent.dx().unwrap().coefficients(), &[r(0), r(2)]);
    assert_eq!(tangent.dy().unwrap().coefficients(), &[r(2), r(0)]);
    assert_eq!(
        tangent
            .dy()
            .unwrap()
            .representation()
            .unwrap()
            .exact_rational_witness(),
        Some(&r(2))
    );
}

#[test]
fn cubic_point_and_tangent_images_use_power_basis_resultants() {
    let curve = CubicBezier2::new(p(0, 0), p(0, 1), p(0, 2), p(1, 3));
    let parameter = sqrt_half_parameter();

    let point = curve
        .point_at_algebraic_parameter(&parameter, &policy())
        .unwrap();
    let tangent = curve
        .tangent_at_algebraic_parameter(&parameter, &policy())
        .unwrap();

    assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
    assert_eq!(point.x().unwrap().coefficients(), &[r(0), r(0), r(0), r(1)]);
    assert_eq!(point.y().unwrap().coefficients(), &[r(0), r(3), r(0), r(0)]);
    assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
    assert_eq!(tangent.dx().unwrap().coefficients(), &[r(0), r(0), r(3)]);
    assert!(tangent.dx().unwrap().representation().unwrap().is_valid());
    assert_eq!(tangent.dy().unwrap().coefficients(), &[r(3), r(0), r(0)]);
}

#[test]
fn nonmonotone_coordinate_image_is_reported_instead_of_sampled() {
    let curve = QuadraticBezier2::new(
        Point2::new(q(9, 16), r(0)),
        Point2::new(q(-3, 16), r(1)),
        Point2::new(q(1, 16), r(2)),
    );
    let parameter = sqrt_half_parameter();

    let point = curve
        .point_at_algebraic_parameter(&parameter, &policy())
        .unwrap();

    assert_eq!(point.status(), BezierAlgebraicImageStatus::XImageFailed);
    assert!(point.x().is_none());
    assert!(point.y().is_none());
    assert!(point.message().unwrap().contains("x coordinate"));
}

proptest! {
    #[test]
    fn linear_coordinate_images_match_exact_midpoint_values(
        x0 in -8_i32..=8,
        x1 in -8_i32..=8,
        x2 in -8_i32..=8,
        y0 in -8_i32..=8,
        y1 in -8_i32..=8,
        y2 in -8_i32..=8,
    ) {
        let curve = QuadraticBezier2::new(
            Point2::from_values(x0, y0),
            Point2::from_values(x1, y1),
            Point2::from_values(x2, y2),
        );
        let parameter = isolate(
            polynomial(vec![r(-1), r(2)]),
            interval(q(2, 5), q(3, 5)),
        );

        let point = curve.point_at_algebraic_parameter(&parameter, &policy()).unwrap();
        let tangent = curve.tangent_at_algebraic_parameter(&parameter, &policy()).unwrap();
        let exact_point = curve.point_at(q(1, 2));

        prop_assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
        prop_assert_eq!(
            point.x().unwrap().representation().unwrap().exact_rational_witness(),
            Some(exact_point.x())
        );
        prop_assert_eq!(
            point.y().unwrap().representation().unwrap().exact_rational_witness(),
            Some(exact_point.y())
        );
        prop_assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
    }
}
