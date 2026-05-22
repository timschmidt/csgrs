use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial, Classification,
    CurvePolicy, CurveResult, Real,
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

    Ok(())
}
