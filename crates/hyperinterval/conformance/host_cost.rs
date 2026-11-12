//! Paired rational product probe; both paths materialize owned exact endpoints.
use hyperinterval::RationalInterval;
use hyperreal::Rational;
use std::{hint::black_box, time::Instant};

#[path = "../tests/support/legacy_rational_product.rs"]
mod legacy;

#[inline(never)]
fn old(a: &RationalInterval, b: &RationalInterval) -> (Rational, Rational) {
    let (a0, a1) = a.bounds().unwrap();
    let (b0, b1) = b.bounds().unwrap();
    legacy::rational_interval_product(a0, a1, b0, b1)
}

#[inline(never)]
fn shared(a: &RationalInterval, b: &RationalInterval) -> (Rational, Rational) {
    a.mul(b).into_bounds().unwrap()
}

type Product = fn(&RationalInterval, &RationalInterval) -> (Rational, Rational);

fn main() {
    let cases = [
        ("positive", "1/7", "3/5", "2/9", "7/11"),
        ("mixed", "-3/7", "5/11", "-7/13", "9/17"),
        (
            "wide",
            "-12345678901234567890123456789/17",
            "98765432109876543210987654321/19",
            "-9/7",
            "13/17",
        ),
    ];
    println!("case,sample,backend,elapsed_ns,completed_operations,checksum");
    for (name, a, b, c, d) in cases {
        let a = RationalInterval::new(a.parse().unwrap(), b.parse().unwrap()).unwrap();
        let b = RationalInterval::new(c.parse().unwrap(), d.parse().unwrap()).unwrap();
        assert_eq!(old(&a, &b), shared(&a, &b));
        for _ in 0..4096 {
            black_box(old(&a, &b));
            black_box(shared(&a, &b));
        }
        for sample in 0..30 {
            let pair: [(&str, Product); 2] = [("legacy", old), ("shared", shared)];
            for index in 0..2 {
                let (backend, operation) = pair[(index + sample) % 2];
                let start = Instant::now();
                let mut checksum = 0u64;
                for _ in 0..16384 {
                    let (lo, hi) = black_box(operation(black_box(&a), black_box(&b)));
                    checksum = checksum.wrapping_add(lo.numerator().bits() + hi.numerator().bits());
                }
                let elapsed = start.elapsed().as_nanos();
                println!("{name},{sample},{backend},{elapsed},16384,{checksum}");
            }
        }
    }
}
