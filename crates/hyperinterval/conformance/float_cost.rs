//! Completed strict-float workload and integer-code inspection entry points.
use hyperinterval::{F32Enclosure, F64Enclosure, IntervalError};
use std::{hint::black_box, time::Instant};

// Unique, stable symbols for recursive object-code inspection. These example
// exports confer no public-library authority or ABI guarantee.
#[unsafe(no_mangle)]
#[inline(never)]
pub fn hyperinterval_strict_chain64(a: f64, b: f64) -> Result<F64Enclosure, IntervalError> {
    F64Enclosure::new(a, b)?
        .affine(-0.5, 0.25)?
        .square()?
        .add(&F64Enclosure::point(1.0)?)?
        .reciprocal()
}

#[unsafe(no_mangle)]
#[inline(never)]
pub fn hyperinterval_strict_chain32(a: f32, b: f32) -> Result<F32Enclosure, IntervalError> {
    F32Enclosure::new(a, b)?
        .affine(-0.5, 0.25)?
        .square()?
        .add(&F32Enclosure::point(1.0)?)?
        .reciprocal()
}

macro_rules! inspect {
    ($name:ident, $interval:ident, $float:ty) => {
        #[unsafe(no_mangle)]
        #[inline(never)]
        pub fn $name(a: $float, b: $float, exponent: u32) -> Result<$interval, IntervalError> {
            let x = $interval::new(a, b)?;
            let wider = $interval::new($interval::next_down(a)?, $interval::next_up(b)?)?;
            let image = x
                .checked_pow(exponent)?
                .expanded()?
                .intersection(&wider)
                .hull(&x);
            black_box((
                image.abs(),
                image.min_image(&wider),
                image.max_image(&wider),
            ));
            black_box((
                image.signs(),
                image.is_empty(),
                image.is_singleton(),
                image.lower(),
                image.upper(),
            ));
            black_box((
                image.subset_of(&wider),
                image.interior_subset_of(&wider),
                image.disjoint(&x),
                image.overlaps(&x),
                image.contains_point(a)?,
            ));
            image.sub(&x.neg())?.div(&$interval::point(2.0)?)
        }
    };
}
inspect!(hyperinterval_strict_surface32, F32Enclosure, f32);
inspect!(hyperinterval_strict_surface64, F64Enclosure, f64);

fn main() {
    if !F64Enclosure::backend_supported() {
        return;
    }
    eprintln!(
        "layout_bytes,f32={},f64={}",
        size_of::<F32Enclosure>(),
        size_of::<F64Enclosure>()
    );
    // The exact intermediate square is [0, 1/4]; reciprocal gives [4/5,1].
    let check64 = hyperinterval_strict_chain64(-0.5, 1.25).unwrap();
    let check32 = hyperinterval_strict_chain32(-0.5, 1.25).unwrap();
    assert_eq!(check64.bounds().unwrap().0.to_bits(), 0x3fe9_9999_9999_9999);
    assert_eq!(check32.bounds().unwrap().0.to_bits(), 0x3f4c_cccc);
    assert!(check64.contains_point(1.0).unwrap());
    assert!(check32.contains_point(1.0).unwrap());
    black_box(
        hyperinterval_strict_surface32(black_box(0.75), black_box(1.25), black_box(3)).unwrap(),
    );
    black_box(
        hyperinterval_strict_surface64(black_box(0.75), black_box(1.25), black_box(3)).unwrap(),
    );
    println!("width,sample,elapsed_ns,completed_chains,checksum");
    macro_rules! measure {
        ($name:ident, $width:literal) => {
            for sample in 0..30 {
                let start = Instant::now();
                let mut checksum = 0u64;
                for _ in 0..16384 {
                    let result = black_box($name(black_box(-0.5), black_box(1.25)).unwrap());
                    let (a, b) = result.bounds().unwrap();
                    checksum =
                        checksum.wrapping_add(u64::from(a.to_bits()) ^ u64::from(b.to_bits()));
                }
                println!(
                    "{},{sample},{},16384,{checksum}",
                    $width,
                    start.elapsed().as_nanos()
                );
            }
        };
    }
    measure!(hyperinterval_strict_chain32, 32);
    measure!(hyperinterval_strict_chain64, 64);
}
