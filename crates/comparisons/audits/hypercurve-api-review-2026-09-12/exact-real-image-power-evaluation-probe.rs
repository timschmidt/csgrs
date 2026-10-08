#![allow(dead_code, unused_imports)]
use hypercurve::{
    BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurveContext, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, Real,
};
use hypercurve::{CubicBezier2, RationalBezier2};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn policy() -> CurveContext {
    CurveContext::STRICT
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

fn rational_point_image_transforms_exact_real_linear_root() {
    let conic =
        RationalQuadraticBezier2::try_new(p(0, 0), p(2, 4), p(6, 0), r(1), r(2), r(3)).unwrap();
    let parameter = isolate(
        polynomial(vec![r(-1), Real::pi()]),
        interval(q(1, 4), q(1, 2)),
    );

    let point = conic
        .point_at_algebraic_parameter(&parameter, &policy())
        .unwrap();

    assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
    assert!(point.parameter().is_valid());
    let exact_parameter = point
        .parameter()
        .exact_point_witness()
        .expect("a linear exact-Real polynomial has an exact point witness");
    assert_eq!(exact_parameter, &(Real::one() / Real::pi()).unwrap());
    assert!(exact_parameter.exact_rational_ref().is_none());
    let t = exact_parameter;
    let denominator = r(2) * t + r(1);
    let x = ((r(10) * t + r(8)) * t / &denominator).unwrap();
    let y = ((r(-16) * t + r(16)) * t / denominator).unwrap();
    let exact_point = Point2::new(x, y);
    let x = point.x().unwrap().representation().unwrap();
    let y = point.y().unwrap().representation().unwrap();
    assert!(x.exact_point_witness().is_some());
    assert!(y.exact_point_witness().is_some());
    assert_eq!(x.interval.lower, x.interval.upper);
    assert_eq!(y.interval.lower, y.interval.upper);
    assert_eq!(
        point
            .x()
            .unwrap()
            .compare_to_real(exact_point.x(), &CurveContext::STRICT),
        Classification::Decided(std::cmp::Ordering::Equal),
    );
    assert_eq!(
        point
            .y()
            .unwrap()
            .compare_to_real(exact_point.y(), &CurveContext::STRICT),
        Classification::Decided(std::cmp::Ordering::Equal),
    );
    assert!(point.retained_parameter().is_none());
    assert!(point.message().is_none());
}

fn main() { rational_point_image_transforms_exact_real_linear_root(); }
