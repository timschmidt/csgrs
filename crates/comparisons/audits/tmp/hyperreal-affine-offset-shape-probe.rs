use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};

fn main() {
    let iterations = 100000;
    let half = Real::new(Rational::fraction(1, 2).unwrap());
    let eighth = Real::new(Rational::fraction(1, 8).unwrap());
    let nested = (&half.clone().sqrt().unwrap() + &half * &half).sqrt().unwrap() - &half;
    let lower = &nested - &eighth;
    let upper = &nested + &eighth;
    let expected = nested.to_f64_lossy().unwrap();
    let mean = Real::average_pair(&lower, &upper);
    println!("mean: {mean:?}");
    let difference = &mean - &nested;
    println!("difference: {:?}", difference.certified_sign_until(0).sign());
    for mode in ["construct", "sign", "convert"] {
        let start = Instant::now();
        for _ in 0..iterations {
            let value = Real::average_pair(black_box(&lower), black_box(&upper));
            match mode {
                "sign" => { black_box(value.certified_sign_until(128)); },
                "convert" => { assert_eq!(black_box(value.to_f64_lossy().unwrap()), expected); },
                _ => { black_box(value); },
            }
        }
        println!("{mode}: {}", start.elapsed().as_nanos() as f64 / f64::from(iterations));
    }
}
