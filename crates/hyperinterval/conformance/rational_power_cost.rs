//! Materialized exact rational power chains; independent expected endpoints.
use hyperinterval::{IntervalError, RationalInterval as I};
use hyperreal::Rational as R;
use std::{hint::black_box, time::Instant};

fn main() {
    eprintln!(
        "layout_bytes,error={},interval={},outcome={}",
        size_of::<IntervalError>(),
        size_of::<I>(),
        size_of::<Result<I, IntervalError>>()
    );
    println!("case,sample,elapsed_ns,completed_chains");
    let r = |n, d| R::fraction(n, d).unwrap();
    let cases = [
        (
            "positive",
            I::new(r(3, 4), r(5, 4)).unwrap(),
            I::new(r(4096, 15625), r(4096, 729)).unwrap(),
        ),
        (
            "negative",
            I::new(r(-5, 4), r(-3, 4)).unwrap(),
            I::new(r(4096, 15625), r(4096, 729)).unwrap(),
        ),
        ("point", I::point(r(4, 3)), I::point(r(729, 4096))),
    ];
    for (name, input, expected) in cases {
        let work = || {
            black_box(&input)
                .checked_pow(black_box(3))
                .unwrap()
                .checked_pow(black_box(-2))
                .unwrap()
        };
        assert_eq!(work(), expected);
        for _ in 0..4096 {
            black_box(work().into_bounds());
        }
        for sample in 0..30 {
            let start = Instant::now();
            for _ in 0..8192 {
                black_box(work().into_bounds());
            }
            println!("{name},{sample},{},8192", start.elapsed().as_nanos());
        }
    }
}
