#![cfg(feature = "integer")]

use hyperinterval::{IntegerRange, IntervalError};

#[test]
fn affine_cancellation_and_overflow_across_primitive_widths() {
    macro_rules! signed {
        ($($ty:ty),+) => {$({
            let min = <$ty>::MIN;
            let max = <$ty>::MAX;
            let source = IntegerRange::new(max - 1, max).unwrap();
            assert_eq!(source.scale(2), Err(IntervalError::Overflow));
            let doubled = source.affine(2, min).unwrap();
            assert_eq!(doubled.bounds(), Some((max - 3, max - 1)));
            assert_eq!(doubled.affine(-1, max).unwrap().bounds(), Some((1, 3)));
            assert_eq!(IntegerRange::point(min).affine(-1, min), Ok(IntegerRange::point(0)));
            assert_eq!(IntegerRange::point(min).affine(2, max), Err(IntervalError::Overflow));
            assert_eq!(IntegerRange::point(min).affine(-2, min), Err(IntervalError::Overflow));
            assert_eq!(IntegerRange::new(min, max).unwrap().affine(0, min), Ok(IntegerRange::point(min)));
            assert_eq!(IntegerRange::<$ty>::empty().affine(min, max), Ok(IntegerRange::empty()));
        })+};
    }
    macro_rules! unsigned {
        ($($ty:ty),+) => {$({
            let max = <$ty>::MAX;
            let source = IntegerRange::new(0, max / 2).unwrap();
            assert_eq!(source.affine(2, 1).unwrap().bounds(), Some((1, max)));
            assert_eq!(source.affine(2, 2), Err(IntervalError::Overflow));
            assert_eq!(IntegerRange::point(max).affine(2, 0), Err(IntervalError::Overflow));
            assert_eq!(IntegerRange::new(0, max).unwrap().affine(0, max), Ok(IntegerRange::point(max)));
            assert_eq!(IntegerRange::<$ty>::empty().affine(max, max), Ok(IntegerRange::empty()));
        })+};
    }
    signed!(i8, i16, i32, i64, i128, isize);
    unsigned!(u8, u16, u32, u64, u128, usize);
}

#[test]
fn signed_byte_hulls_match_enumerated_images() {
    for a in -8_i8..=8 {
        for b in a..=8 {
            let source = IntegerRange::new(a, b).unwrap();
            for factor in [i8::MIN, -43, -17, -3, -1, 0, 1, 3, 17, 43, i8::MAX] {
                for offset in [i8::MIN, -127, -63, -1, 0, 1, 63, 126, i8::MAX] {
                    let image =
                        (a..=b).map(|x| i16::from(x) * i16::from(factor) + i16::from(offset));
                    let lo = i8::try_from(image.clone().min().unwrap());
                    let hi = i8::try_from(image.max().unwrap());
                    let expected = match (lo, hi) {
                        (Ok(lo), Ok(hi)) => Ok(Some((lo, hi))),
                        _ => Err(IntervalError::Overflow),
                    };
                    assert_eq!(source.affine(factor, offset).map(|v| v.bounds()), expected);
                }
            }
        }
    }
}

#[cfg(not(target_env = "msvc"))]
#[path = "support/integer_affine_oracle.rs"]
mod oracle;

#[cfg(not(target_env = "msvc"))]
#[test]
fn full_width_final_bounds_match_gmp() {
    fn matrix<T: Copy>(values: &[T], verify: impl Fn(T, T, T, T)) {
        for (index, &a) in values.iter().enumerate() {
            for &b in &values[index..] {
                for &factor in values {
                    for &offset in values {
                        verify(a, b, factor, offset);
                    }
                }
            }
        }
    }
    matrix(
        &[
            i128::MIN,
            i128::MIN + 1,
            -(1 << 64),
            -2,
            -1,
            0,
            1,
            2,
            1 << 64,
            i128::MAX - 1,
            i128::MAX,
        ],
        oracle::signed,
    );
    matrix(
        &[0_u128, 1, 2, 1 << 64, 1 << 127, u128::MAX - 1, u128::MAX],
        oracle::unsigned,
    );
}
