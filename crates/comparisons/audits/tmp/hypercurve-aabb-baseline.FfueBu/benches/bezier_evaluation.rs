use std::hint::black_box;
use std::time::Instant;

use hypercurve::{CubicBezier2, Point2, QuadraticBezier2, Real};

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
}
