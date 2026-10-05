#![no_main]

mod support;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, Curve2, CurveContext, CurvePoint2, Point2,
    QuadraticBezier2, Real,
};
use libfuzzer_sys::fuzz_target;

fn real_from_byte(byte: u8) -> Real {
    Real::from(byte as i32 - 128)
}

fn unit_from_byte(byte: u8) -> Real {
    (Real::from((byte % 17) as i32) / Real::from(16_i32)).unwrap()
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(real_from_byte(x), real_from_byte(y))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }

    let policy = CurveContext::STRICT;
    let curve = QuadraticBezier2::new(
        point(data[0], data[1]),
        point(data[2], data[3]),
        point(data[4], data[5]),
    );

    let mut parameters = Vec::new();
    // Public cuts require a strict interior parameter.
    if !matches!(data[6] % 17, 0 | 16) {
        parameters.push(BezierParameter2::Exact(unit_from_byte(data[6])));
    }

    if data[9] & 1 == 1 {
        let start = unit_from_byte(data[7].min(data[8]));
        let end = unit_from_byte(data[7].max(data[8]));
        if let Ok(Classification::Decided(polynomial)) =
            BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(-1_i32), Real::from(2_i32)],
                &policy,
            )
            && let Ok(Classification::Decided(interval)) =
                BezierParameterInterval::try_new(start, end, &policy)
            && let Ok(Classification::Decided(algebraic)) =
                BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
        {
            parameters.push(BezierParameter2::Algebraic(algebraic));
        }
    }
    if data[9] & 2 == 2
        && let Ok(Classification::Decided(polynomial)) =
            BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(-1_i32), Real::from(0_i32), Real::from(2_i32)],
                &policy,
            )
        && let Ok(Classification::Decided(interval)) = BezierParameterInterval::try_new(
            (Real::from(2_i32) / Real::from(3_i32)).unwrap(),
            (Real::from(3_i32) / Real::from(4_i32)).unwrap(),
            &policy,
        )
        && let Ok(Classification::Decided(algebraic)) =
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
    {
        parameters.push(BezierParameter2::Algebraic(algebraic));
    }

    let curve = Curve2::from(curve);
    let coincide = |first: &CurvePoint2, second: &CurvePoint2| {
        assert_eq!(
            first.coincides_with(second, &policy).value,
            Classification::Decided(true)
        );
    };
    for parameter in parameters {
        // Exact and selected interior cuts of an authored quadratic always
        // complete; each piece keeps exact, connected endpoint evidence.
        let (first, second) = support::under(&policy, || curve.split_at(parameter.into()))
            .expect("an interior quadratic cut must complete");
        coincide(&first.start(), &curve.start());
        coincide(&first.end(), &second.start());
        coincide(&second.end(), &curve.end());
    }
});
