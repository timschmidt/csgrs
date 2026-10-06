#![no_main]

mod support;

use hypercurve::{
    Axis2, BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, Curve2, PredicatePolicy, CurveParameter2, CurvePoint2,
    Point2, QuadraticBezier2, RationalQuadraticBezier2, Real,
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
    let policy = PredicatePolicy::STRICT;
    let mode = data[6] % 3;
    let curve = if mode == 0 {
        QuadraticBezier2::new(
            Point2::from_values(control(data[0]), control(data[1])),
            Point2::from_values(control(data[2]), control(data[3])),
            Point2::from_values(control(data[4]), control(data[5])),
        )
    } else {
        // x(t) = (t - 3/4)^2 is deliberately non-monotone over the
        // sqrt(1/2) isolator; its image must still be certified without sampling.
        QuadraticBezier2::new(
            Point2::new(q(9, 16), r(0)),
            Point2::new(q(-3, 16), r(1)),
            Point2::new(q(1, 16), r(2)),
        )
    };
    let conic = RationalQuadraticBezier2::try_new(
        Point2::from_values(control(data[0]), control(data[1])),
        Point2::from_values(control(data[2]), control(data[3])),
        Point2::from_values(control(data[4]), control(data[5])),
        r(1),
        if mode == 2 { r(-1) } else { r(2) },
        r(1),
    )
    .ok();

    let (polynomial_coefficients, start, end) = if mode == 0 || mode == 2 {
        (vec![r(-1), r(2)], q(2, 5), q(3, 5))
    } else {
        (vec![r(-1), r(0), r(2)], q(1, 2), r(1))
    };
    let polynomial = match support::under_classified_result(&policy, || {
        BezierParameterPolynomial::try_new_power_basis(polynomial_coefficients)
    }) {
        Ok(Classification::Decided(polynomial)) => polynomial,
        Ok(Classification::Uncertain(_)) | Err(_) => return,
    };
    let interval = match support::under_classified_result(&policy, || {
        BezierParameterInterval::try_new(start, end)
    }) {
        Ok(classification) => match decided(classification) {
            Some(interval) => interval,
            None => return,
        },
        Err(_) => return,
    };
    let parameter = match support::under_classified_result(&policy, || {
        BezierAlgebraicParameter2::try_isolate(polynomial, interval)
    }) {
        Ok(classification) => match decided(classification) {
            Some(parameter) => parameter,
            None => return,
        },
        Err(_) => return,
    };

    let selected = CurveParameter2::from(BezierParameter2::Algebraic(parameter));
    let general = Curve2::from(curve.clone());
    let point = support::under(&policy, || general.point_at(&selected))
        .expect("a finite polynomial point at a selected parameter must complete");
    let tangent = support::under(&policy, || general.derivative_at(&selected))
        .expect("a finite polynomial tangent at a selected parameter must complete");

    if mode == 0 {
        // 2t - 1 selects t = 1/2 exactly: the selected point and tangent must
        // agree with the represented evaluation.
        let half = CurveParameter2::from(q(1, 2));
        let represented = support::under(&policy, || general.point_at(&half)).unwrap();
        assert_eq!(
            support::under_outcome_classification(&policy, || point.coincides_with(&represented)),
            Classification::Decided(true)
        );
        let represented_tangent = support::under(&policy, || general.derivative_at(&half)).unwrap();
        for axis in [Axis2::X, Axis2::Y] {
            assert_eq!(
                support::under_classified_result(&policy, || tangent.coordinate_sign(axis))
                    .unwrap(),
                support::under_classified_result(&policy, || represented_tangent
                    .coordinate_sign(axis))
                .unwrap()
            );
        }
    } else if mode == 1 {
        // x(t) = (t - 3/4)^2 at t = sqrt(1/2) lies strictly in (0, 1/16).
        for (bound, ordering) in [
            (Real::zero(), std::cmp::Ordering::Greater),
            (q(1, 16), std::cmp::Ordering::Less),
        ] {
            let bound = CurvePoint2::from(Point2::new(bound, Real::zero()));
            assert_eq!(
                support::under_outcome_classified(&policy, || point
                    .compare_coordinate(&bound, Axis2::X))
                .unwrap(),
                Classification::Decided(ordering)
            );
        }
    }

    if let Some(conic) = conic {
        let general = Curve2::from(conic.clone());
        let rational_point = support::under(&policy, || general.point_at(&selected));
        if mode == 2 {
            // Weights (1, -1, 1) put a projective pole at t = 1/2.
            assert!(rational_point.is_err());
        } else if mode == 0 {
            let rational_point = rational_point.expect("a finite rational point must complete");
            let represented = support::under(&policy, || {
                general.point_at(&CurveParameter2::from(q(1, 2)))
            })
            .unwrap();
            assert_eq!(
                support::under_outcome_classification(&policy, || rational_point
                    .coincides_with(&represented)),
                Classification::Decided(true)
            );
        }
    }
});
