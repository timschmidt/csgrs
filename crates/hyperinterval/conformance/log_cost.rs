//! Completed finite and lower-unbounded logarithm enclosures.
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{DyadicBudget, DyadicInterval, ExtendedInterval, RationalInterval};
use hyperreal::Rational;
use num_bigint::{BigInt, BigUint};
use std::{hint::black_box, time::Instant};
#[cfg(not(target_env = "msvc"))]
#[path = "../tests/support/log_oracle.rs"]
mod oracle;
fn main() {
    eprintln!(
        "layout_bytes,dyadic={},extended={}",
        size_of::<DyadicInterval>(),
        size_of::<ExtendedInterval<DyadicInterval>>()
    );
    println!("backend,case,precision,sample,elapsed_ns,completed_images,work_units_per_image");
    let mut cases: Vec<_> = [
        ("point", 384i64, 384i64),
        ("narrow", 384, 385),
        ("wide", 64, 1024),
        ("partial", -256, 1024),
    ]
    .into_iter()
    .map(|(name, a, b)| (name, BigInt::from(a), BigInt::from(b), -8))
    .collect();
    let near: BigInt = (BigInt::from(1) << 1024usize) + 1;
    cases.push(("near_one", near.clone(), near, -1024));
    for (case, a, b, e) in cases {
        let dyadic = DyadicInterval::new(a.clone(), b.clone(), e).unwrap();
        let rational = RationalInterval::new(
            Rational::from_bigint_fraction(a.clone(), BigUint::from(1u8) << e.unsigned_abs())
                .unwrap(),
            Rational::from_bigint_fraction(b.clone(), BigUint::from(1u8) << e.unsigned_abs())
                .unwrap(),
        )
        .unwrap();
        for precision in [-16, -64, -256] {
            for backend in ["dyadic", "rational"] {
                let work = || {
                    let mut budget = DyadicBudget::new(16384, 1_000_000_000_000);
                    let image = if backend == "dyadic" {
                        black_box(&dyadic).log(black_box(precision), &mut budget)
                    } else {
                        black_box(&rational).log(black_box(precision), &mut budget)
                    }
                    .unwrap();
                    (image, 1_000_000_000_000 - budget.remaining_work())
                };
                let (image, units) = work();
                #[cfg(not(target_env = "msvc"))]
                {
                    oracle::verify(
                        &image.range,
                        &oracle::q(&a, e),
                        &oracle::q(&b, e),
                        precision,
                    );
                    // Exercise the independent power-of-two oracle helper too.
                    let scale = DyadicInterval::point(1.into(), e)
                        .unwrap()
                        .log(precision, &mut DyadicBudget::new(16384, 1_000_000_000_000))
                        .unwrap();
                    oracle::log_two_multiple(&scale.range, i64::from(e), precision);
                }
                black_box(image);
                for _ in 0..128 {
                    black_box(work());
                }
                for sample in 0..20 {
                    let start = Instant::now();
                    for _ in 0..128 {
                        black_box(work());
                    }
                    println!(
                        "{backend},{case},{precision},{sample},{},128,{units}",
                        start.elapsed().as_nanos()
                    );
                }
            }
        }
    }
}
