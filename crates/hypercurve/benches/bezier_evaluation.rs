use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierParameterPolynomial, Classification, CubicBezier2, CurveContext, Point2,
    QuadraticBezier2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).expect("nonzero denominator")
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn main() {
    let quadratic = QuadraticBezier2::new(p(-3, 2), p(5, 11), p(13, -7));
    let cubic = CubicBezier2::new(p(-3, 2), p(2, 13), p(9, -8), p(15, 4));
    let parameters = [q(1, 4), q(1, 2), q(3, 4)];
    let iterations = 500_000_u32;

    let started = Instant::now();
    for index in 0..iterations {
        let parameter = parameters[index as usize % parameters.len()].clone();
        black_box(quadratic.point_at(black_box(parameter)));
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_evaluation_quadratic: {iterations} iterations in {elapsed:?} ({:?}/iter)",
        elapsed / iterations
    );

    let started = Instant::now();
    for index in 0..iterations {
        let parameter = parameters[index as usize % parameters.len()].clone();
        black_box(cubic.point_at(black_box(parameter)));
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_evaluation_cubic: {iterations} iterations in {elapsed:?} ({:?}/iter)",
        elapsed / iterations
    );

    let coefficients = (0..=32).map(|index| r(index % 7 - 3)).collect::<Vec<_>>();
    let symbolic = coefficients
        .iter()
        .enumerate()
        .map(|(index, coefficient)| {
            coefficient
                * if index % 2 == 0 {
                    Real::pi()
                } else {
                    Real::e()
                }
        })
        .collect::<Vec<_>>();
    let alpha = q(1, 2).sqrt().unwrap();
    let root = -q(1, 2) + (&alpha + q(1, 4)).sqrt().unwrap();
    for (label, coefficients, parameter, iterations) in [
        ("rational_32", coefficients, q(1, 3), 20_000_u32),
        ("symbolic_32", symbolic, q(1, 3), 2_000),
        ("nested_quadratic", vec![-alpha, r(1), r(1)], root, 10_000),
        (
            "balanced_128",
            (0..=128).map(|index| r(index % 7 - 3)).collect(),
            r(2).sqrt().unwrap(),
            2_000,
        ),
    ] {
        let mut power = Real::one();
        let mut terms = Vec::with_capacity(coefficients.len());
        for coefficient in &coefficients {
            terms.push(coefficient * &power);
            power = &power * &parameter;
        }
        let expected = Real::sum_refs(&terms)
            .certified_dyadic_interval(-160)
            .unwrap();
        let Classification::Decided(polynomial) =
            BezierParameterPolynomial::try_new_power_basis(coefficients, &CurveContext::STRICT)
                .unwrap()
        else {
            panic!("benchmark polynomial degree is uncertified");
        };
        let started = Instant::now();
        let mut certified = 0_usize;
        for _ in 0..iterations {
            let value = polynomial.evaluate(black_box(&parameter));
            let interval = black_box(value.certified_dyadic_interval(-128).unwrap());
            assert!(
                interval[0] <= expected[1] && expected[0] <= interval[1],
                "{label}"
            );
            certified += 1;
        }
        let elapsed = started.elapsed();
        println!(
            "bezier_polynomial_evaluation_{label}: {iterations} iterations in {elapsed:?} ({:?}/iter), certified={certified}",
            elapsed / iterations
        );
    }
}
