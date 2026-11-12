//! Materialized square-root enclosures by requested grid and source width.
use hyperinterval::{DyadicBudget, DyadicInterval, RationalInterval};
use hyperreal::Rational;
use std::{hint::black_box, time::Instant};
#[cfg(not(target_env = "msvc"))]
#[path = "../tests/support/sqrt_oracle.rs"]
mod sqrt_oracle;

fn main() {
    println!("backend,case,precision,sample,elapsed_ns,completed_images");
    for (name, a, b) in [
        ("point", 131072i64, 131072i64),
        ("narrow", 131072, 131073),
        ("wide", 0, 1048576),
        ("partial", -65536, 262144),
    ] {
        let dyadic = DyadicInterval::new(a.into(), b.into(), -16).unwrap();
        let rational = RationalInterval::new(
            Rational::fraction(a, 65536).unwrap(),
            Rational::fraction(b, 65536).unwrap(),
        )
        .unwrap();
        for precision in [-16, -64, -256, -1024] {
            for backend in ["dyadic", "rational"] {
                let work = || {
                    let mut budget = DyadicBudget::new(8192, 10_000_000_000);
                    if backend == "dyadic" {
                        black_box(&dyadic)
                            .sqrt(black_box(precision), &mut budget)
                            .unwrap()
                    } else {
                        black_box(&rational)
                            .sqrt(black_box(precision), &mut budget)
                            .unwrap()
                    }
                };
                #[cfg(not(target_env = "msvc"))]
                sqrt_oracle::verify(
                    &work().range,
                    rug::Rational::from((a.max(0), 65536)),
                    rug::Rational::from((b, 65536)),
                    precision,
                );
                for _ in 0..1024 {
                    black_box(work());
                }
                for sample in 0..20 {
                    let start = Instant::now();
                    for _ in 0..1024 {
                        black_box(work());
                    }
                    println!(
                        "{backend},{name},{precision},{sample},{},1024",
                        start.elapsed().as_nanos()
                    );
                }
            }
        }
    }
}
