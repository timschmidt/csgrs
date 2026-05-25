#![no_main]

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierSplitFragment2, Classification, CurvePolicy, Point2,
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

    let policy = CurvePolicy::certified();
    let curve = QuadraticBezier2::new(
        point(data[0], data[1]),
        point(data[2], data[3]),
        point(data[4], data[5]),
    );

    let mut parameters = Vec::new();
    if let Ok(Classification::Decided(parameter)) =
        BezierParameter2::exact(unit_from_byte(data[6]), &policy)
    {
        parameters.push(parameter);
    }

    if data[9] & 1 == 1 {
        let start = unit_from_byte(data[7].min(data[8]));
        let end = unit_from_byte(data[7].max(data[8]));
        if let Ok(Classification::Decided(polynomial)) =
            BezierParameterPolynomial::try_new_power_basis(vec![Real::from(-1_i32), Real::from(2_i32)], &policy)
            && let Ok(Classification::Decided(interval)) =
                BezierParameterInterval::try_new(start, end, &policy)
            && let Ok(Classification::Decided(algebraic)) =
                BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
        {
            parameters.push(BezierParameter2::algebraic(algebraic));
        }
    }

    if let Ok(Classification::Decided(materialization)) =
        curve.split_at_parameters(&parameters, &policy)
    {
        for fragment in materialization.fragments() {
            match fragment {
                BezierSplitFragment2::Materialized { start, end, .. } => {
                    assert!(start.is_exact());
                    assert!(end.is_exact());
                }
                BezierSplitFragment2::AlgebraicEndpointImages {
                    start,
                    end,
                    start_image,
                    end_image,
                } => {
                    assert!(start_image.is_some() || end_image.is_some());
                    if !start.is_exact() {
                        assert!(start_image.as_ref().is_some_and(|image| image.is_transformed()));
                    }
                    if !end.is_exact() {
                        assert!(end_image.as_ref().is_some_and(|image| image.is_transformed()));
                    }
                }
                BezierSplitFragment2::Unresolved { start, end } => {
                    assert!(!start.is_exact() || !end.is_exact());
                }
            }
        }
    }
});
