use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let numerator: i64 = args[1].parse().unwrap();
    let denominator: u64 = args[2].parse().unwrap();
    let exponent: i64 = args[3].parse().unwrap();
    let iterations: u32 = args[4].parse().unwrap();
    let evaluate: bool = args[5].parse().unwrap();
    let samples: u32 = args[6].parse().unwrap();
    for _ in 0..samples {
        let start = Instant::now();
        for _ in 0..iterations {
            let base = Real::new(Rational::fraction(black_box(numerator), black_box(denominator)).unwrap());
            let power = base.powi_i64(black_box(exponent)).unwrap();
            if evaluate {
                let value = power.to_f64_lossy().expect("finite power");
                assert!(value.is_finite());
                black_box(value);
            } else {
                black_box(power);
            }
        }
        println!("{:.3}", start.elapsed().as_nanos() as f64 / f64::from(iterations));
    }
}
