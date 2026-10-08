use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};

fn main() {
    let iterations: u64 = std::env::args().nth(1).unwrap_or_else(|| "10000".into()).parse().unwrap();
    let half = Real::new(Rational::fraction(1, 2).unwrap());
    let eighth = Real::new(Rational::fraction(1, 8).unwrap());
    let nested = (&half.clone().sqrt().unwrap() + &half * &half).sqrt().unwrap() - &half;
    let opaque = Real::one().sin();
    for (name, center) in [("nested_midpoint_conversion", nested), ("opaque_midpoint_conversion", opaque.clone())] {
        let lower = &center - &eighth;
        let upper = &center + &eighth;
        let expected = center.to_f64_lossy().unwrap();
        let start = Instant::now();
        let mut checksum = 0;
        for _ in 0..iterations {
            let mean = Real::average_pair(black_box(&lower), black_box(&upper));
            let actual = black_box(mean.to_f64_lossy().unwrap());
            assert_eq!(actual, expected);
            checksum = u64::wrapping_add(checksum, actual.to_bits());
        }
        println!("{name},{iterations},{},{checksum}", start.elapsed().as_nanos() as f64 / iterations as f64);
    }
    for (name, left, right) in [
        ("unrelated_symbols", Real::pi(), Real::e()),
        ("unrelated_affine_bases", &opaque + &eighth, Real::from(2).sin() + &eighth),
    ] {
        let expected = (&left + &right).to_f64_lossy().unwrap();
        let start = Instant::now();
        let mut checksum = 0u64;
        for _ in 0..iterations {
            let value = black_box(&left) + black_box(&right);
            let actual = black_box(value.to_f64_lossy().unwrap());
            assert_eq!(actual, expected);
            checksum = checksum.wrapping_add(actual.to_bits());
        }
        println!("{name},{iterations},{},{checksum}", start.elapsed().as_nanos() as f64 / iterations as f64);
    }
}
