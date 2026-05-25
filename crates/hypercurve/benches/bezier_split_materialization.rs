use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CubicBezier2, CurvePolicy, CurveResult, Point2,
    Real,
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

fn main() -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let curve = CubicBezier2::new(p(0, 0), p(2, 6), p(6, -2), p(8, 0));
    let parameters = [
        decided(BezierParameter2::exact(q(1, 4), &policy)?),
        decided(BezierParameter2::exact(q(1, 2), &policy)?),
        decided(BezierParameter2::exact(q(3, 4), &policy)?),
    ];

    let iterations = 25_000_u32;
    let started = Instant::now();
    let mut total = 0_usize;
    for _ in 0..iterations {
        let materialization = decided(curve.split_at_parameters(&parameters, &policy)?);
        total += black_box(materialization.fragments().len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_split_materialization_cubic: {iterations} iterations in {elapsed:?} ({:?}/iter), total={total}",
        elapsed / iterations
    );

    let linear_algebraic_polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        vec![r(-1), r(2)],
        &policy,
    )?);
    let linear_algebraic_interval =
        decided(BezierParameterInterval::try_new(q(2, 5), q(3, 5), &policy)?);
    let linear_algebraic =
        BezierParameter2::algebraic(decided(BezierAlgebraicParameter2::try_isolate(
            linear_algebraic_polynomial,
            linear_algebraic_interval,
            &policy,
        )?));
    let linear_algebraic_parameters = [
        decided(BezierParameter2::exact(q(1, 4), &policy)?),
        linear_algebraic,
        decided(BezierParameter2::exact(q(3, 4), &policy)?),
    ];

    let started = Instant::now();
    let mut promoted = 0_usize;
    for _ in 0..iterations {
        let materialization =
            decided(curve.split_at_parameters(&linear_algebraic_parameters, &policy)?);
        promoted += black_box(usize::from(materialization.is_fully_materialized()));
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_split_linear_algebraic_promotion_cubic: {iterations} iterations in {elapsed:?} ({:?}/iter), promoted={promoted}",
        elapsed / iterations
    );

    let algebraic_polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        vec![r(-1), r(0), r(2)],
        &policy,
    )?);
    let algebraic_interval = decided(BezierParameterInterval::try_new(q(2, 3), q(3, 4), &policy)?);
    let algebraic = BezierParameter2::algebraic(decided(BezierAlgebraicParameter2::try_isolate(
        algebraic_polynomial,
        algebraic_interval,
        &policy,
    )?));
    let algebraic_parameters = [
        decided(BezierParameter2::exact(q(1, 4), &policy)?),
        algebraic,
        decided(BezierParameter2::exact(q(3, 4), &policy)?),
    ];

    let started = Instant::now();
    let mut retained = 0_usize;
    for _ in 0..iterations {
        let materialization = decided(curve.split_at_parameters(&algebraic_parameters, &policy)?);
        retained += black_box(usize::from(materialization.has_algebraic_endpoint_images()));
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_split_algebraic_endpoint_images_cubic: {iterations} iterations in {elapsed:?} ({:?}/iter), retained={retained}",
        elapsed / iterations
    );

    Ok(())
}
