//! Completed finite and unbounded arctangent enclosures.
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{DyadicBudget, DyadicInterval, ExtendedInterval};
use std::{hint::black_box, time::Instant};
#[cfg(not(target_env = "msvc"))]
#[path = "../tests/support/atan_oracle.rs"]
mod oracle;
fn main() {
    println!("backend,case,precision,sample,elapsed_ns,completed_images,work_units_per_image");
    let cases = [
        ("point", 3i64, 3i64, -1),
        ("narrow", 384, 385, -8),
        ("wide", -4, 4, 0),
        ("huge", 1, 1, 4096),
        ("tiny", 1, 1, -1074),
    ];
    for (case, a, b, e) in cases {
        let dyadic = DyadicInterval::new(a.into(), b.into(), e).unwrap();
        let rational = dyadic
            .to_rational(&mut DyadicBudget::new(16384, 1_000_000_000_000))
            .unwrap();
        for precision in [-16, -64, -256] {
            for backend in ["dyadic", "rational"] {
                let work = || {
                    let mut budget = DyadicBudget::new(16384, 1_000_000_000_000);
                    let image = if backend == "dyadic" {
                        black_box(&dyadic).atan(black_box(precision), &mut budget)
                    } else {
                        black_box(&rational).atan(black_box(precision), &mut budget)
                    }
                    .unwrap();
                    (image, 1_000_000_000_000 - budget.remaining_work())
                };
                let (image, units) = work();
                #[cfg(not(target_env = "msvc"))]
                oracle::verify(
                    &image.range,
                    &oracle::q(&a, e),
                    &oracle::q(&b, e),
                    precision,
                );
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
    // Consume unbounded outputs directly; no finite surrogate replaces infinity.
    for precision in [-16, -64, -256] {
        let source = ExtendedInterval::<DyadicInterval>::entire();
        let work = || {
            let mut budget = DyadicBudget::new(16384, 1_000_000_000_000);
            let image = black_box(&source)
                .atan(black_box(precision), &mut budget)
                .unwrap();
            (image, 1_000_000_000_000 - budget.remaining_work())
        };
        let (image, units) = work();
        #[cfg(not(target_env = "msvc"))]
        {
            let (_, hi) = oracle::half_pi();
            oracle::finite(&image.range, -hi.clone(), hi, precision);
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
                "extended,entire,{precision},{sample},{},128,{units}",
                start.elapsed().as_nanos()
            );
        }
    }
}
