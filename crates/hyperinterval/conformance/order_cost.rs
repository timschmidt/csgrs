//! Completed absolute/min/max images. Host observations, not speedup claims.
use hyperinterval::{
    DyadicBudget, DyadicInterval, F32Enclosure, F64Enclosure, FixedInterval, IntegerRange,
    RationalInterval,
};
use std::{hint::black_box, time::Instant};

fn measure<I: PartialEq + core::fmt::Debug>(name: &str, expected: I, mut work: impl FnMut() -> I) {
    assert_eq!(work(), expected);
    // Explicit warmup; timed calls own and drop their completed result.
    for _ in 0..4096 {
        black_box(work());
    }
    for sample in 0..30 {
        let start = Instant::now();
        for _ in 0..16384 {
            black_box(work());
        }
        println!("{name},{sample},{},16384", start.elapsed().as_nanos());
    }
}

fn main() {
    println!("family,sample,elapsed_ns,completed_chains");
    // abs([-3,5]) -> min([2,4]) -> max([1,3]) = [1,4].
    let x = IntegerRange::<i64>::new(-3, 5).unwrap();
    let cap = IntegerRange::<i64>::new(2, 4).unwrap();
    let floor = IntegerRange::<i64>::new(1, 3).unwrap();
    measure("integer-i64", IntegerRange::new(1, 4).unwrap(), || {
        black_box(&x)
            .abs()
            .unwrap()
            .min_image(black_box(&cap))
            .max_image(black_box(&floor))
    });

    type Fixed = FixedInterval<i64, 16>;
    let x = Fixed::new(-3, 5).unwrap();
    let cap = Fixed::new(2, 4).unwrap();
    let floor = Fixed::new(1, 3).unwrap();
    measure("fixed-i64-16", Fixed::new(1, 4).unwrap(), || {
        black_box(&x)
            .abs()
            .unwrap()
            .min_image(black_box(&cap))
            .max_image(black_box(&floor))
    });

    let make = |a: i64, b: i64| RationalInterval::new(a.into(), b.into()).unwrap();
    let (x, cap, floor) = (make(-3, 5), make(2, 4), make(1, 3));
    measure("rational", make(1, 4), || {
        black_box(&x)
            .abs()
            .min_image(black_box(&cap))
            .max_image(black_box(&floor))
    });

    let make = |a: i64, b: i64, e| DyadicInterval::new(a.into(), b.into(), e).unwrap();
    // Different authored scales describe the same workload in common grid 2^-3.
    let (x, cap, floor) = (make(-3, 5, -3), make(1, 2, -2), make(1, 3, -3));
    measure("dyadic", make(1, 4, -3), || {
        let mut budget = DyadicBudget::new(4096, 1_000_000);
        black_box(&x)
            .abs(&mut budget)
            .unwrap()
            .min_image(black_box(&cap), &mut budget)
            .unwrap()
            .max_image(black_box(&floor), &mut budget)
            .unwrap()
    });

    if F64Enclosure::backend_supported() {
        macro_rules! width {
            ($i:ty,$name:literal) => {{
                let (x, cap, floor) = (
                    <$i>::new(-3.0, 5.0).unwrap(),
                    <$i>::new(2.0, 4.0).unwrap(),
                    <$i>::new(1.0, 3.0).unwrap(),
                );
                measure($name, <$i>::new(1.0, 4.0).unwrap(), || {
                    black_box(&x)
                        .abs()
                        .min_image(black_box(&cap))
                        .max_image(black_box(&floor))
                });
            }};
        }
        width!(F32Enclosure, "f32");
        width!(F64Enclosure, "f64");
    }
}
