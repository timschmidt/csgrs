#![no_main]

use hypercurve::{
    BezierAlgebraicParameter2, BezierAlgebraicTangentOrderStatus,
    BezierAlgebraicTangentVector2, BezierEndpointTangentImage2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurvePolicy, Point2, QuadraticBezier2, Real,
    compare_algebraic_same_tangent_second_order, compare_algebraic_same_tangent_third_order,
    compare_algebraic_tangent_turn_from_base,
};
use libfuzzer_sys::fuzz_target;

fn real(byte: u8) -> Real {
    Real::from(byte as i32 - 128)
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(real(x), real(y))
}

fn vector_from_curve(
    curve: &QuadraticBezier2,
    parameter: &BezierAlgebraicParameter2,
    policy: &CurvePolicy,
) -> Option<BezierAlgebraicTangentVector2> {
    let tangent = curve
        .tangent_at_algebraic_parameter(parameter, policy)
        .ok()?;
    BezierAlgebraicTangentVector2::from_endpoint_image(&BezierEndpointTangentImage2::Polynomial(
        tangent,
    ))
    .vector
}

fn second_vector_from_curve(
    curve: &QuadraticBezier2,
    parameter: &BezierAlgebraicParameter2,
    policy: &CurvePolicy,
) -> Option<BezierAlgebraicTangentVector2> {
    let tangent = curve
        .second_derivative_at_algebraic_parameter(parameter, policy)
        .ok()?;
    BezierAlgebraicTangentVector2::from_endpoint_image(&BezierEndpointTangentImage2::Polynomial(
        tangent,
    ))
    .vector
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 18 {
        return;
    }

    let policy = CurvePolicy::certified();
    let Ok(Classification::Decided(polynomial)) =
        BezierParameterPolynomial::try_new_power_basis(vec![Real::from(-1), Real::from(2)], &policy)
    else {
        return;
    };
    let Ok(Classification::Decided(interval)) =
        BezierParameterInterval::try_new((Real::from(2) / Real::from(5)).unwrap(), (Real::from(3) / Real::from(5)).unwrap(), &policy)
    else {
        return;
    };
    let Ok(Classification::Decided(parameter)) =
        BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy)
    else {
        return;
    };

    let curves = [
        QuadraticBezier2::new(point(data[0], data[1]), point(data[2], data[3]), point(data[4], data[5])),
        QuadraticBezier2::new(point(data[6], data[7]), point(data[8], data[9]), point(data[10], data[11])),
        QuadraticBezier2::new(point(data[12], data[13]), point(data[14], data[15]), point(data[16], data[17])),
    ];
    let Some(base) = vector_from_curve(&curves[0], &parameter, &policy) else {
        return;
    };
    let Some(first) = vector_from_curve(&curves[1], &parameter, &policy) else {
        return;
    };
    let Some(second) = vector_from_curve(&curves[2], &parameter, &policy) else {
        return;
    };

    if let Classification::Decided(report) =
        compare_algebraic_tangent_turn_from_base(&base, &first, &second, &policy)
    {
        if report.status == BezierAlgebraicTangentOrderStatus::Ordered {
            assert!(report.ordering.is_some());
        }
    }

    let Some(first_second) = second_vector_from_curve(&curves[1], &parameter, &policy) else {
        return;
    };
    let Some(second_second) = second_vector_from_curve(&curves[2], &parameter, &policy) else {
        return;
    };
    let _ = compare_algebraic_same_tangent_second_order(
        &first,
        &first_second,
        &second,
        &second_second,
        &policy,
    );
    let _ = compare_algebraic_same_tangent_third_order(
        &first,
        &first_second,
        &second,
        &second_second,
        &policy,
    );
});
