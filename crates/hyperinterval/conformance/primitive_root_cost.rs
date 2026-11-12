//! Owned primitive square-root outputs across widths and endpoint lattices.
use hyperinterval::{F32Enclosure, F64Enclosure, FixedInterval};
use std::{hint::black_box, time::Instant};
#[cfg(not(target_env = "msvc"))]
#[path = "../tests/support/fixed_sqrt_oracle.rs"]
mod fixed_oracle;
#[cfg(not(target_env = "msvc"))]
#[path = "../tests/support/primitive_sqrt_oracle.rs"]
mod oracle;

fn measure<T>(backend: &str, case: &str, work: impl Fn() -> T) {
    for _ in 0..4096 {
        black_box(work());
    }
    for sample in 0..30 {
        let start = Instant::now();
        for _ in 0..8192 {
            black_box(work());
        }
        println!(
            "{backend},{case},{sample},{},8192",
            start.elapsed().as_nanos()
        );
    }
}
fn main() {
    println!("backend,case,sample,elapsed_ns,completed_images");
    for (case, a, b) in [
        ("point", 1, 1),
        ("narrow", 1, 2),
        ("wide", 0, 3),
        ("partial", -1, 2),
    ] {
        macro_rules! fixed {
            ($ty:ty,$p:literal) => {{
                let input = FixedInterval::<$ty, $p>::new(
                    (a as $ty) * (1 << ($p - 2)),
                    (b as $ty) * (1 << ($p - 2)),
                )
                .unwrap();
                let work = || black_box(input).sqrt().unwrap();
                #[cfg(not(target_env = "msvc"))]
                {
                    let (lo, hi) = input.bounds().unwrap();
                    let result = work();
                    let (l, u) = result.range.bounds().unwrap();
                    fixed_oracle::fixed(
                        i64::from(lo),
                        i64::from(hi),
                        $p,
                        i64::from(<$ty>::MAX),
                        Ok(((i64::from(l), i64::from(u)), result.domain)),
                    );
                }
                measure(concat!(stringify!($ty), "/", stringify!($p)), case, work);
            }};
        }
        fixed!(i32, 8);
        fixed!(i32, 16);
        fixed!(i32, 30);
        fixed!(i64, 16);
        fixed!(i64, 30);
        fixed!(i64, 62);
        macro_rules! ieee {
            ($interval:ident,$float:ty,$root:ident) => {{
                let input = $interval::new(a as $float / 4.0, b as $float / 4.0).unwrap();
                let work = || black_box(input).sqrt().unwrap();
                #[cfg(not(target_env = "msvc"))]
                {
                    let result = work();
                    let (lo, hi) = result.range.bounds().unwrap();
                    assert_eq!(
                        (lo.to_bits(), hi.to_bits()),
                        (
                            oracle::$root(a.max(0) as $float / 4.0).unwrap().0,
                            oracle::$root(b as $float / 4.0).unwrap().1
                        )
                    );
                }
                measure(stringify!($float), case, work);
            }};
        }
        ieee!(F32Enclosure, f32, sqrt_f32);
        ieee!(F64Enclosure, f64, sqrt_f64);
    }
}
