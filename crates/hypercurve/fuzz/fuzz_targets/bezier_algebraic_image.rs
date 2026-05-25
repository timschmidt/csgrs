#![no_main]

use hypercurve::{
    BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurveError, CurvePolicy, Point2, QuadraticBezier2,
    Real,
};
use libfuzzer_sys::fuzz_target;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn control(byte: u8) -> i32 {
    i32::from(byte % 17) - 8
}

fn decided<T>(classification: Classification<T>) -> Option<T> {
    match classification {
        Classification::Decided(value) => Some(value),
        Classification::Uncertain(_) => None,
    }
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 7 {
        return;
    }
    let policy = CurvePolicy::certified();
    let curve = if data[6] & 1 == 0 {
        QuadraticBezier2::new(
            Point2::from_values(control(data[0]), control(data[1])),
            Point2::from_values(control(data[2]), control(data[3])),
            Point2::from_values(control(data[4]), control(data[5])),
        )
    } else {
        // x(t) = (t - 3/4)^2 is deliberately non-monotone over the
        // sqrt(1/2) isolator; this should report failure instead of sampling.
        QuadraticBezier2::new(
            Point2::new(q(9, 16), r(0)),
            Point2::new(q(-3, 16), r(1)),
            Point2::new(q(1, 16), r(2)),
        )
    };

    let (polynomial_coefficients, start, end) = if data[6] & 1 == 0 {
        (vec![r(-1), r(2)], q(2, 5), q(3, 5))
    } else {
        (vec![r(-1), r(0), r(2)], q(1, 2), r(1))
    };
    let polynomial =
        match BezierParameterPolynomial::try_new_power_basis(polynomial_coefficients, &policy) {
            Ok(Classification::Decided(polynomial)) => polynomial,
            Ok(Classification::Uncertain(_)) | Err(CurveError::InvalidBezierPolynomial) => return,
            Err(_) => return,
        };
    let interval = match BezierParameterInterval::try_new(start, end, &policy) {
        Ok(classification) => match decided(classification) {
            Some(interval) => interval,
            None => return,
        },
        Err(_) => return,
    };
    let parameter = match BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy) {
        Ok(classification) => match decided(classification) {
            Some(parameter) => parameter,
            None => return,
        },
        Err(_) => return,
    };

    let point = curve
        .point_at_algebraic_parameter(&parameter, &policy)
        .expect("valid algebraic parameter should produce a report");
    let tangent = curve
        .tangent_at_algebraic_parameter(&parameter, &policy)
        .expect("valid algebraic parameter should produce a tangent report");

    if data[6] & 1 == 0 {
        assert_eq!(point.status(), BezierAlgebraicImageStatus::Transformed);
        assert_eq!(tangent.status(), BezierAlgebraicImageStatus::Transformed);
        assert!(point.x().unwrap().representation().is_some());
        assert!(point.y().unwrap().representation().is_some());
    } else {
        assert_eq!(point.status(), BezierAlgebraicImageStatus::XImageFailed);
    }
});
