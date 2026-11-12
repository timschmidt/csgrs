//! Completed direct division images with explicit pole/coverage semantics.
use hyperinterval::{DyadicBudget, DyadicInterval, ExtendedInterval as E, IntervalSet2};
use std::{hint::black_box, time::Instant};
#[cfg(not(target_env = "msvc"))]
#[path = "../tests/support/division_oracle.rs"]
mod oracle;
fn d(a: i64, b: i64) -> E<DyadicInterval> {
    E::from_finite(DyadicInterval::new(a.into(), b.into(), 0).unwrap())
}
fn main() {
    eprintln!(
        "layout_bytes,dyadic={},extended={},split={}",
        size_of::<DyadicInterval>(),
        size_of::<E<DyadicInterval>>(),
        size_of::<IntervalSet2<DyadicInterval>>()
    );
    println!("backend,case,precision,sample,elapsed_ns,completed_images,work_units_per_image");
    for (case, a, b) in [
        ("positive", d(1, 2), d(3, 4)),
        ("negative", d(-2, -1), d(-4, -3)),
        ("split", d(1, 2), d(-1, 1)),
        ("touching", d(1, 2), d(0, 3)),
        ("entire_denominator", d(1, 2), E::entire()),
        ("zero", d(0, 0), d(-1, 1)),
        (
            "unbounded_numerator",
            E::above(DyadicInterval::point(1.into(), 0).unwrap()).unwrap(),
            d(2, 3),
        ),
        (
            "unbounded_denominator",
            d(1, 2),
            E::above(DyadicInterval::point(1.into(), 0).unwrap()).unwrap(),
        ),
        ("entire_numerator", E::entire(), d(1, 2)),
    ] {
        for precision in [-16, -64, -256] {
            let work = || {
                let mut budget = DyadicBudget::new(4096, 100_000_000_000);
                let image = black_box(&a)
                    .div(black_box(&b), black_box(precision), &mut budget)
                    .unwrap();
                (image, 100_000_000_000 - budget.remaining_work())
            };
            let (image, units) = work();
            #[cfg(not(target_env = "msvc"))]
            {
                // The oracle reads represented source bounds, then evaluates every
                // endpoint/limit corner independently of monotone dispatch.
                let qa = oracle::actual(&IntervalSet2::from_interval(a.clone()))
                    .into_iter()
                    .next();
                let qb = oracle::actual(&IntervalSet2::from_interval(b.clone()))
                    .into_iter()
                    .next();
                oracle::verify(Ok(image.clone()), &qa, &qb, precision);
            }
            black_box(image);
            for _ in 0..128 {
                black_box(work());
            }
            for sample in 0..20 {
                let start = Instant::now();
                for _ in 0..1024 {
                    black_box(work());
                }
                println!(
                    "dyadic,{case},{precision},{sample},{},1024,{units}",
                    start.elapsed().as_nanos()
                );
            }
        }
    }
}
