#![cfg(feature = "float-enclosure")]

use hyperinterval::{F32Enclosure, F64Enclosure, IntervalError};

#[cfg(all(target_arch = "x86_64", not(target_env = "msvc")))]
include!("support/mxcsr.rs");

#[test]
fn direct_format_conversion_preserves_empty_and_directs_each_endpoint() {
    assert_eq!(
        F32Enclosure::empty().to_f64_enclosure(),
        Ok(F64Enclosure::empty())
    );
    assert_eq!(
        F64Enclosure::empty().to_f32_enclosure(),
        Ok(F32Enclosure::empty())
    );
    if !F32Enclosure::backend_supported() {
        return;
    }
    let x = F64Enclosure::new(-f64::from_bits(1), 1.0 + f64::EPSILON).unwrap();
    let y = x.to_f32_enclosure().unwrap();
    assert_eq!(
        y,
        F32Enclosure::new(-f32::from_bits(1), f32::from_bits(0x3f80_0001)).unwrap()
    );
    assert!(x.subset_of(&y.to_f64_enclosure().unwrap()));
    // All finite exponent fields, with representative fraction boundaries of
    // both signs. An interior binary64 point between consecutive binary32
    // values has exactly those neighbours as its tight binary32 enclosure.
    for exponent in 0..255 {
        for fraction in [0, 1, 0x3f_ffff, 0x7f_fffe, 0x7f_ffff] {
            let bits = exponent << 23 | fraction;
            let a = f32::from_bits(bits);
            let source = F32Enclosure::new(-a, a).unwrap();
            let wide = F64Enclosure::new(-f64::from(a), f64::from(a)).unwrap();
            assert_eq!(source.to_f64_enclosure(), Ok(wide));
            assert_eq!(wide.to_f32_enclosure(), Ok(source));
            let upper = f32::from_bits(bits + 1);
            let interior = f64::from_bits(f64::from(a).to_bits() + 1);
            if upper.is_finite() {
                for (x, lo, hi) in [(interior, a, upper), (-interior, -upper, -a)] {
                    assert_eq!(
                        F64Enclosure::point(x).unwrap().to_f32_enclosure(),
                        F32Enclosure::new(lo, hi)
                    );
                }
            } else {
                for (lo, hi) in [(-interior, 0.), (0., interior), (-f64::MAX, f64::MAX)] {
                    assert_eq!(
                        F64Enclosure::new(lo, hi).unwrap().to_f32_enclosure(),
                        Err(IntervalError::Overflow)
                    );
                }
            }
        }
    }
}

#[cfg(all(target_arch = "x86_64", not(target_env = "msvc")))]
#[test]
fn format_conversions_preserve_subnormals_in_all_native_modes() {
    for mode in 0..16 {
        with_mxcsr(mode, || {
            let source = F32Enclosure::point(std::hint::black_box(f32::from_bits(1))).unwrap();
            let widened = source.to_f64_enclosure().unwrap();
            let (lo, hi) = widened.bounds().unwrap();
            assert_eq!((lo.to_bits(), hi.to_bits()), (874_u64 << 52, 874_u64 << 52));
            assert_eq!(widened.to_f32_enclosure(), Ok(source));

            let tiny = F64Enclosure::point(std::hint::black_box(f64::from_bits(1))).unwrap();
            let (lo, hi) = tiny.to_f32_enclosure().unwrap().bounds().unwrap();
            assert_eq!((lo.to_bits(), hi.to_bits()), (0, 1));
        });
    }
}
