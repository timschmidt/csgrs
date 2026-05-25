use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicParameter2, BezierAlgebraicTangentVector2, BezierEndpointTangentImage2,
    BezierParameterInterval, BezierParameterPolynomial, Classification, CurvePolicy, CurveResult,
    Point2, QuadraticBezier2, Real, compare_algebraic_same_tangent_second_order,
    compare_algebraic_tangent_turn_from_base,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: Real, y: Real) -> Point2 {
    Point2::new(x, y)
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark unexpectedly uncertain: {reason:?}"),
    }
}

fn vector(
    curve: &QuadraticBezier2,
    parameter: &BezierAlgebraicParameter2,
    policy: &CurvePolicy,
) -> BezierAlgebraicTangentVector2 {
    let tangent = curve
        .tangent_at_algebraic_parameter(parameter, policy)
        .unwrap();
    BezierAlgebraicTangentVector2::from_endpoint_image(&BezierEndpointTangentImage2::Polynomial(
        tangent,
    ))
    .vector
    .unwrap()
}

fn second_vector(
    curve: &QuadraticBezier2,
    parameter: &BezierAlgebraicParameter2,
    policy: &CurvePolicy,
) -> BezierAlgebraicTangentVector2 {
    let tangent = curve
        .second_derivative_at_algebraic_parameter(parameter, policy)
        .unwrap();
    BezierAlgebraicTangentVector2::from_endpoint_image(&BezierEndpointTangentImage2::Polynomial(
        tangent,
    ))
    .vector
    .unwrap()
}

fn main() -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let parameter = decided(BezierAlgebraicParameter2::try_isolate(
        decided(BezierParameterPolynomial::try_new_power_basis(
            vec![r(-1), r(0), r(2)],
            &policy,
        )?),
        decided(BezierParameterInterval::try_new(q(1, 2), r(1), &policy)?),
        &policy,
    )?);

    let base_curve = QuadraticBezier2::new(p(r(0), r(0)), p(q(1, 2), r(0)), p(r(1), r(0)));
    let first_curve = QuadraticBezier2::new(p(r(0), r(0)), p(r(0), r(0)), p(q(1, 2), r(1)));
    let second_curve = QuadraticBezier2::new(p(r(0), r(0)), p(q(1, 2), r(0)), p(r(1), q(1, 2)));
    let base = vector(&base_curve, &parameter, &policy);
    let first = vector(&first_curve, &parameter, &policy);
    let second = vector(&second_curve, &parameter, &policy);

    let iterations = 10_000_u32;
    let started = Instant::now();
    let mut ordered = 0_usize;
    for _ in 0..iterations {
        let report = decided(compare_algebraic_tangent_turn_from_base(
            &base, &first, &second, &policy,
        ));
        ordered += black_box(usize::from(report.ordering.is_some()));
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_algebraic_tangent_order: {iterations} iterations in {elapsed:?} ({:?}/iter), ordered={ordered}",
        elapsed / iterations
    );

    let same_tangent = vector(&base_curve, &parameter, &policy);
    let upward_curve = QuadraticBezier2::new(p(r(-1), r(1)), p(r(0), r(-1)), p(r(1), r(1)));
    let downward_curve = QuadraticBezier2::new(p(r(-1), r(-1)), p(r(0), r(1)), p(r(1), r(-1)));
    let upward_second = second_vector(&upward_curve, &parameter, &policy);
    let downward_second = second_vector(&downward_curve, &parameter, &policy);
    let started = Instant::now();
    let mut same_tangent_ordered = 0_usize;
    for _ in 0..iterations {
        let report = decided(compare_algebraic_same_tangent_second_order(
            &same_tangent,
            &upward_second,
            &same_tangent,
            &downward_second,
            &policy,
        ));
        same_tangent_ordered += black_box(usize::from(report.ordering.is_some()));
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_algebraic_same_tangent_second_order: {iterations} iterations in {elapsed:?} ({:?}/iter), ordered={same_tangent_ordered}",
        elapsed / iterations
    );

    Ok(())
}
