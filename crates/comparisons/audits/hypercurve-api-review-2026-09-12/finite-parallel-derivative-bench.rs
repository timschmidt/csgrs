use hypercurve::*;
use std::{hint::black_box, time::Instant};

fn main() {
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    let quadratic = QuadraticBezier2::new(
        Point2::from_values(0, 0), Point2::new(q(1, 2), Real::zero()), Point2::from_values(1, 1),
    );
    let cubic = CubicBezier2::new(
        Point2::from_values(-1, 0), Point2::new((-1).into(), q(-1, 3)),
        Point2::new(q(-2, 3), q(-2, 3)), Point2::from_values(0, 0),
    );
    let rational = RationalBezier2::try_new(
        vec![Point2::from_values(0, 0), Point2::from_values(1, 3), Point2::from_values(3, -2), Point2::from_values(5, 1)],
        vec![1.into(), 2.into(), 3.into(), 1.into()],
    ).unwrap();
    let parameters = [Real::zero(), q(1, 4), q(1, 2), q(3, 4), Real::one()];
    for (name, parallel) in [
        ("quadratic", quadratic.parallel_left(q(1, 10)).unwrap()),
        ("cubic", cubic.parallel_left(q(1, 10)).unwrap()),
        ("rational", rational.parallel_left(q(1, 10)).unwrap()),
        ("zero_distance_rational", rational.parallel_left(Real::zero()).unwrap()),
    ] {
        let evaluate = || {
            for parameter in &parameters {
                let Classification::Decided(derivative) = parallel.derivative_at(black_box(parameter), &CurveContext::STRICT).unwrap() else {
                    panic!("benchmark requires exact decided derivatives");
                };
                black_box(derivative);
            }
        };
        evaluate();
        let start = Instant::now();
        for _ in 0..20_000 { evaluate(); }
        println!("{{\"case\":\"{name}\",\"calls\":100000,\"elapsed_ns\":{}}}", start.elapsed().as_nanos());
    }
}
