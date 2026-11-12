//! Completed rational-power images, including exact roots and pole branches.
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
#[cfg(not(target_env = "msvc"))]
use hyperinterval::IntervalSet2;
use hyperinterval::{DyadicBudget, DyadicInterval, ExtendedInterval as E};
use num_bigint::BigInt;
use std::{hint::black_box, time::Instant};
#[cfg(not(target_env = "msvc"))]
#[path = "../tests/support/power_oracle.rs"]
mod oracle;
fn d(a: i64, b: i64, e: i32) -> E<DyadicInterval> {
    E::from_finite(DyadicInterval::new(a.into(), b.into(), e).unwrap())
}
fn main() {
    println!("backend,case,precision,sample,elapsed_ns,completed_images,work_units_per_image");
    for (case, source, n, q) in [
        ("point", d(2, 2, 0), 1, 3),
        ("narrow", d(2, 3, 0), 3, 5),
        ("negative", d(-8, -1, 0), 2, 3),
        ("split", d(-3, 5, 0), -1, 3),
        ("partial", d(-3, 5, 0), -3, 2),
        ("perfect", d(8, 27, 0), 1, 3),
        ("huge", d(1, 1, 512), 1, 3),
        ("tiny", d(1, 1, -512), 1, 3),
        ("entire", E::entire(), -1, 3),
    ] {
        let numerator: BigInt = n.into();
        let denominator: BigInt = q.into();
        for precision in [-16, -64, -256] {
            let work = || {
                let mut budget = DyadicBudget::new(32768, 10_000_000_000_000);
                let result = black_box(&source)
                    .pow_rational(
                        black_box(&numerator),
                        black_box(&denominator),
                        black_box(precision),
                        &mut budget,
                    )
                    .unwrap();
                (result, 10_000_000_000_000 - budget.remaining_work())
            };
            let (result, units) = work();
            #[cfg(not(target_env = "msvc"))]
            {
                let exact = oracle::actual(&IntervalSet2::from_interval(source.clone()))
                    .into_iter()
                    .next();
                oracle::verify(Ok(result.clone()), &exact, n, q, precision);
                if case == "point" {
                    oracle::mpfr_point(&result.range, &2.into(), n, q as u32);
                }
            }
            black_box(result);
            for _ in 0..32 {
                black_box(work());
            }
            for sample in 0..20 {
                let start = Instant::now();
                for _ in 0..128 {
                    black_box(work());
                }
                println!(
                    "dyadic,{case},{precision},{sample},{},128,{units}",
                    start.elapsed().as_nanos()
                );
            }
        }
    }
}
