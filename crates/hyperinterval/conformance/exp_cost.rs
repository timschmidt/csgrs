//! Materialized tight exponential images, including all refinement work.
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{DyadicBudget, DyadicInterval, RationalInterval};
use hyperreal::Rational;
use std::{hint::black_box, time::Instant};
#[cfg(not(target_env = "msvc"))]
#[path = "../tests/support/exp_oracle.rs"]
mod oracle;
fn main() {
    println!("backend,case,precision,sample,elapsed_ns,completed_images,work_units_per_image");
    for (case, a, b) in [
        ("point", 256i64, 256i64),
        ("narrow", 256, 257),
        ("wide", -1024, 1024),
        ("negative", -4096, -2048),
    ] {
        let dyadic = DyadicInterval::new(a.into(), b.into(), -8).unwrap();
        let rational = RationalInterval::new(
            Rational::fraction(a, 256).unwrap(),
            Rational::fraction(b, 256).unwrap(),
        )
        .unwrap();
        for precision in [-16, -64, -256] {
            for backend in ["dyadic", "rational"] {
                let work = || {
                    let mut budget = DyadicBudget::new(16384, 1_000_000_000_000);
                    let result = if backend == "dyadic" {
                        black_box(&dyadic).exp(black_box(precision), &mut budget)
                    } else {
                        black_box(&rational).exp(black_box(precision), &mut budget)
                    }
                    .unwrap();
                    (result, 1_000_000_000_000 - budget.remaining_work())
                };
                let (result, units) = work();
                #[cfg(not(target_env = "msvc"))]
                oracle::verify(
                    &result.range,
                    &oracle::q(&a, -8),
                    &oracle::q(&b, -8),
                    precision,
                );
                black_box(result);
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
