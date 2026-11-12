//! Completed periodic images by width, scale, precision and source backend.
use hyperinterval::{DyadicBudget, DyadicInterval, ExtendedInterval};
use std::{hint::black_box, time::Instant};
#[cfg(not(target_env = "msvc"))]
#[path = "../tests/support/trig_oracle.rs"]
mod oracle;
fn main() {
    println!("backend,case,precision,sample,elapsed_ns,completed_images,work_units_per_image");
    for (case, a, b, e) in [
        ("point", 1i64, 1i64, 0),
        ("narrow", 256, 257, -8),
        ("extrema", 1, 2, 0),
        ("wide", -4, 4, 0),
        ("huge", 1, 1, 128),
        ("tiny", 1, 1, -1074),
    ] {
        let dyadic = DyadicInterval::new(a.into(), b.into(), e).unwrap();
        let rational = dyadic
            .to_rational(&mut DyadicBudget::new(32768, 10_000_000_000_000))
            .unwrap();
        for precision in [-16, -64, -256] {
            for cosine in [false, true] {
                for backend in ["dyadic", "rational"] {
                    let work = || {
                        let mut budget = DyadicBudget::new(32768, 10_000_000_000_000);
                        let image = match (backend, cosine) {
                            ("dyadic", false) => {
                                black_box(&dyadic).sin(black_box(precision), &mut budget)
                            }
                            ("dyadic", true) => {
                                black_box(&dyadic).cos(black_box(precision), &mut budget)
                            }
                            (_, false) => {
                                black_box(&rational).sin(black_box(precision), &mut budget)
                            }
                            (_, true) => {
                                black_box(&rational).cos(black_box(precision), &mut budget)
                            }
                        }
                        .unwrap();
                        (image, 10_000_000_000_000 - budget.remaining_work())
                    };
                    let (image, units) = work();
                    #[cfg(not(target_env = "msvc"))]
                    oracle::verify(
                        &image.range,
                        &oracle::q(&a, e),
                        &oracle::q(&b, e),
                        precision,
                        cosine,
                    );
                    black_box(image);
                    for _ in 0..128 {
                        black_box(work());
                    }
                    let function = if cosine { "cos" } else { "sin" };
                    for sample in 0..20 {
                        let start = Instant::now();
                        for _ in 0..128 {
                            black_box(work());
                        }
                        println!(
                            "{backend},{function}_{case},{precision},{sample},{},128,{units}",
                            start.elapsed().as_nanos()
                        );
                    }
                }
            }
        }
    }
    for precision in [-16, -64, -256] {
        for cosine in [false, true] {
            let source = ExtendedInterval::<DyadicInterval>::entire();
            let work = || {
                let mut budget = DyadicBudget::new(32768, 10_000_000_000_000);
                let image = if cosine {
                    black_box(&source).cos(black_box(precision), &mut budget)
                } else {
                    black_box(&source).sin(black_box(precision), &mut budget)
                }
                .unwrap();
                (image, 10_000_000_000_000 - budget.remaining_work())
            };
            let (image, units) = work();
            assert_eq!(
                image.range,
                DyadicInterval::new((-1).into(), 1.into(), 0).unwrap()
            );
            black_box(image);
            for _ in 0..128 {
                black_box(work());
            }
            let function = if cosine { "cos" } else { "sin" };
            for sample in 0..20 {
                let start = Instant::now();
                for _ in 0..128 {
                    black_box(work());
                }
                println!(
                    "extended,{function}_entire,{precision},{sample},{},128,{units}",
                    start.elapsed().as_nanos()
                );
            }
        }
    }
}
