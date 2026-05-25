use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicParameter2, BezierAlgebraicTangentVector2, BezierEndpointTangentImage2,
    BezierParameterInterval, BezierParameterPolynomial, Classification, CurvePolicy, CurveResult,
    Point2, QuadraticBezier2, Real, compare_algebraic_tangent_turn_from_base,
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

    Ok(())
}
