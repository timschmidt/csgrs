//! Completed tangent/cotangent images by width, scale, precision and backend.
use hyperinterval::{DyadicBudget, DyadicInterval, ExtendedInterval};
use std::{hint::black_box, time::Instant};
#[cfg(not(target_env = "msvc"))]
#[path = "../tests/support/tangent_oracle.rs"]
mod oracle;
fn main() {
    println!("backend,case,precision,sample,elapsed_ns,completed_images,work_units_per_image");
    for (case, a, b, e) in [
        ("point", 1i64, 1i64, 0),
        ("narrow", 256, 257, -8),
        ("pole", 1, 2, 0),
        ("wide", -4, 4, 0),
        ("huge", 1, 1, 128),
        ("tiny", 1, 1, -1074),
    ] {
        let dyadic = DyadicInterval::new(a.into(), b.into(), e).unwrap();
        let rational = dyadic
            .to_rational(&mut DyadicBudget::new(32768, 10_000_000_000_000))
            .unwrap();
        for precision in [-16, -64, -256] {
            for cotangent in [false, true] {
                for backend in ["dyadic", "rational"] {
                    let work = || {
                        let mut budget = DyadicBudget::new(32768, 10_000_000_000_000);
                        let image = match (backend, cotangent) {
                            ("dyadic", false) => {
                                black_box(&dyadic).tan(black_box(precision), &mut budget)
                            }
                            ("dyadic", true) => {
                                black_box(&dyadic).cot(black_box(precision), &mut budget)
                            }
                            (_, false) => {
                                black_box(&rational).tan(black_box(precision), &mut budget)
                            }
                            (_, true) => {
                                black_box(&rational).cot(black_box(precision), &mut budget)
                            }
                        }
                        .unwrap();
                        (image, 10_000_000_000_000 - budget.remaining_work())
                    };
                    let (image, units) = work();
                    #[cfg(not(target_env = "msvc"))]
                    oracle::verify(
                        Ok(image.clone()),
                        &Some((
                            oracle::End::Value(oracle::q(&a, e)),
                            oracle::End::Value(oracle::q(&b, e)),
                        )),
                        precision,
                        cotangent,
                    );
                    black_box(image);
                    for _ in 0..16 {
                        black_box(work());
                    }
                    let function = if cotangent { "cot" } else { "tan" };
                    for sample in 0..20 {
                        let start = Instant::now();
                        for _ in 0..16 {
                            black_box(work());
                        }
                        println!(
                            "{backend},{function}_{case},{precision},{sample},{},16,{units}",
                            start.elapsed().as_nanos()
                        );
                    }
                }
            }
        }
    }
    for precision in [-16, -64, -256] {
        for cotangent in [false, true] {
            let source = ExtendedInterval::<DyadicInterval>::entire();
            let work = || {
                let mut budget = DyadicBudget::new(32768, 10_000_000_000_000);
                let image = if cotangent {
                    black_box(&source).cot(black_box(precision), &mut budget)
                } else {
                    black_box(&source).tan(black_box(precision), &mut budget)
                }
                .unwrap();
                (image, 10_000_000_000_000 - budget.remaining_work())
            };
            let (image, units) = work();
            assert_eq!(image.range.components(), &[ExtendedInterval::entire()]);
            black_box(image);
            for _ in 0..16 {
                black_box(work());
            }
            let function = if cotangent { "cot" } else { "tan" };
            for sample in 0..20 {
                let start = Instant::now();
                for _ in 0..16 {
                    black_box(work());
                }
                println!(
                    "extended,{function}_entire,{precision},{sample},{},16,{units}",
                    start.elapsed().as_nanos()
                );
            }
        }
    }
}
