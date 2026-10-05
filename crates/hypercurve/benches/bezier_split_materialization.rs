#[path = "../tests/support/mod.rs"]
mod support;
use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicParameter2, BezierFlatteningOptions, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CubicBezier2, Curve2, CurveContext, CurveParameter2,
    Point2, RationalQuadraticBezier2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark unexpectedly uncertain: {reason:?}"),
    }
}

/// Times one exact public cut, returning the number of pieces produced.
fn time_cuts(
    label: &str,
    curve: &Curve2,
    parameter: &CurveParameter2,
    iterations: u32,
    policy: &CurveContext,
) -> Result<(), Box<dyn std::error::Error>> {
    let started = Instant::now();
    let mut total = 0_usize;
    for _ in 0..iterations {
        let (first, second) =
            crate::support::under(policy, || curve.split_at(parameter.clone()))?.into_value();
        total += black_box(usize::from(first.family() == second.family()) + 1);
    }
    let elapsed = started.elapsed();
    println!(
        "{label}: {iterations} iterations in {elapsed:?} ({:?}/iter), total={total}",
        elapsed / iterations
    );
    Ok(())
}

fn algebraic(
    coefficients: Vec<Real>,
    lower: Real,
    upper: Real,
    policy: &CurveContext,
) -> Result<CurveParameter2, Box<dyn std::error::Error>> {
    let polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        coefficients,
        policy,
    )?);
    let interval = decided(BezierParameterInterval::try_new(lower, upper, policy)?);
    Ok(
        BezierParameter2::Algebraic(decided(BezierAlgebraicParameter2::try_isolate(
            polynomial, interval, policy,
        )?))
        .into(),
    )
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let policy = CurveContext::STRICT;
    let cubic = CubicBezier2::new(p(0, 0), p(2, 6), p(6, -2), p(8, 0));
    let curve = Curve2::from(cubic.clone());
    let iterations = 25_000_u32;
    let half: CurveParameter2 = BezierParameter2::Exact(q(1, 2)).into();
    time_cuts(
        "bezier_split_materialization_cubic",
        &curve,
        &half,
        iterations,
        &policy,
    )?;

    let flattening_options = BezierFlatteningOptions::try_new(q(1, 10), 16, &policy)?;
    let flatten_iterations = 10_000_u32;
    let started = Instant::now();
    let mut flattened_total = 0_usize;
    for _ in 0..flatten_iterations {
        let flattened = decided(cubic.flatten_certified(&flattening_options, &policy));
        flattened_total += black_box(flattened.points().len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_flatten_materialization_cubic: {flatten_iterations} iterations in {elapsed:?} ({:?}/iter), total={flattened_total}",
        elapsed / flatten_iterations
    );

    let rational_curve = Curve2::from(RationalQuadraticBezier2::try_new(
        p(0, 0),
        p(4, 8),
        p(8, 0),
        r(1),
        r(2),
        r(1),
    )?);
    time_cuts(
        "bezier_split_materialization_rational_quadratic",
        &rational_curve,
        &half,
        iterations,
        &policy,
    )?;

    // 2t - 1 has the represented root 1/2 and promotes to a native cut.
    let linear = algebraic(vec![r(-1), r(2)], q(2, 5), q(3, 5), &policy)?;
    time_cuts(
        "bezier_split_linear_algebraic_promotion_cubic",
        &curve,
        &linear,
        iterations,
        &policy,
    )?;

    // 2t^2 - 1 retains the selected root sqrt(1/2) and its endpoint images.
    let quadratic = algebraic(vec![r(-1), r(0), r(2)], q(2, 3), q(3, 4), &policy)?;
    time_cuts(
        "bezier_split_algebraic_endpoint_images_cubic",
        &curve,
        &quadratic,
        iterations,
        &policy,
    )?;

    Ok(())
}
