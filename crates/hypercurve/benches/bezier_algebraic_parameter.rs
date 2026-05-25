use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial, Classification,
    CurvePolicy, CurveResult, Point2, QuadraticBezier2, RationalQuadraticBezier2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark setup became uncertain: {reason:?}"),
    }
}

fn main() -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        vec![q(1, 16), r(-1), r(1)],
        &policy,
    )?);
    let left = decided(BezierParameterInterval::try_new(r(0), q(1, 4), &policy)?);
    let right = decided(BezierParameterInterval::try_new(q(3, 4), r(1), &policy)?);

    let iterations = 20_000_u32;
    let started = Instant::now();
    let mut total = 0_usize;

    for _ in 0..iterations {
        let first = decided(BezierAlgebraicParameter2::try_isolate(
            polynomial.clone(),
            left.clone(),
            &policy,
        )?);
        let second = decided(BezierAlgebraicParameter2::try_isolate(
            polynomial.clone(),
            right.clone(),
            &policy,
        )?);
        total += black_box(first.root_count() + second.root_count());
    }

    let elapsed = started.elapsed();
    println!(
        "bezier_algebraic_parameter_sturm: {iterations} iterations in {elapsed:?} ({:?}/iter), total={total}",
        elapsed / iterations
    );

    let midpoint_polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        vec![r(-1), r(2)],
        &policy,
    )?);
    let midpoint_interval = decided(BezierParameterInterval::try_new(q(2, 5), q(3, 5), &policy)?);
    let midpoint = decided(BezierAlgebraicParameter2::try_isolate(
        midpoint_polynomial,
        midpoint_interval,
        &policy,
    )?);
    let curve = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(1, 3),
        Point2::from_values(4, 0),
    );

    let started = Instant::now();
    let mut transformed = 0_usize;
    for _ in 0..iterations {
        let point = curve.point_at_algebraic_parameter(&midpoint, &policy)?;
        let tangent = curve.tangent_at_algebraic_parameter(&midpoint, &policy)?;
        transformed += black_box(point.x().is_some() as usize + tangent.dx().is_some() as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_algebraic_point_tangent_image: {iterations} iterations in {elapsed:?} ({:?}/iter), transformed={transformed}",
        elapsed / iterations
    );

    let conic = RationalQuadraticBezier2::try_new(
        Point2::from_values(0, 0),
        Point2::from_values(2, 4),
        Point2::from_values(6, 0),
        r(1),
        r(2),
        r(3),
    )?;
    let started = Instant::now();
    let mut rational_transformed = 0_usize;
    for _ in 0..iterations {
        let point = conic.point_at_algebraic_parameter(&midpoint, &policy)?;
        let tangent = conic.tangent_at_algebraic_parameter(&midpoint, &policy)?;
        rational_transformed +=
            black_box(point.x().is_some() as usize + tangent.dx().is_some() as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "rational_bezier_algebraic_point_tangent_image: {iterations} iterations in {elapsed:?} ({:?}/iter), transformed={rational_transformed}",
        elapsed / iterations
    );

    Ok(())
}
