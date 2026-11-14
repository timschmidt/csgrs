#![cfg(feature = "integer")]
use hyperinterval::{IntegerRange, IntervalError};

#[test]
fn signed_byte_ranges_match_enumerated_integer_images() {
    for a in -5_i8..=5 {
        for b in a..=5 {
            for factor in i8::MIN..=i8::MAX {
                let range = IntegerRange::new(a, b).unwrap();
                let actual = range.scale(factor);
                let image: Vec<_> = (a..=b).map(|x| i32::from(x) * i32::from(factor)).collect();
                let lo = *image.iter().min().unwrap();
                let hi = *image.iter().max().unwrap();
                match (i8::try_from(lo), i8::try_from(hi)) {
                    (Ok(lo), Ok(hi)) => assert_eq!(actual.unwrap().bounds(), Some((lo, hi))),
                    _ => assert_eq!(actual, Err(IntervalError::Overflow)),
                }
                assert_eq!(actual, range.checked_mul(&IntegerRange::point(factor)));
            }
        }
    }
}

#[test]
fn all_endpoint_families_preserve_identity_zero_empty_and_overflow() {
    macro_rules! check {
        ($($ty:ty),*) => {$(
            let all = IntegerRange::<$ty>::new(<$ty>::MIN, <$ty>::MAX).unwrap();
            assert_eq!(all.scale(1), Ok(all));
            assert_eq!(all.scale(0), Ok(IntegerRange::point(0)));
            assert_eq!(all.scale(2), Err(IntervalError::Overflow));
            assert_eq!(IntegerRange::<$ty>::empty().scale(<$ty>::MAX), Ok(IntegerRange::empty()));
            assert_eq!(IntegerRange::<$ty>::point(0).scale(<$ty>::MAX), Ok(IntegerRange::point(0)));
            assert_eq!(IntegerRange::<$ty>::new(1, 3).unwrap().scale(2).unwrap().checked_add(&IntegerRange::point(1)).unwrap().bounds(), Some((3, 7)));
        )*};
    }
    check!(
        i8, i16, i32, i64, i128, isize, u8, u16, u32, u64, u128, usize
    );
}

#[cfg(not(target_env = "msvc"))]
#[test]
fn full_width_signed_and_unsigned_images_match_gmp() {
    use rug::Integer;
    let signed = [
        i128::MIN,
        i128::MIN + 1,
        -(1_i128 << 64),
        -3,
        -1,
        0,
        1,
        3,
        1_i128 << 64,
        i128::MAX,
    ];
    for &a in &signed {
        for &b in &signed {
            if a > b {
                continue;
            }
            for &factor in &signed {
                let aa = Integer::from(a) * Integer::from(factor);
                let bb = Integer::from(b) * Integer::from(factor);
                let (lo, hi) = if aa <= bb { (aa, bb) } else { (bb, aa) };
                let actual = IntegerRange::new(a, b).unwrap().scale(factor);
                match (lo.to_i128(), hi.to_i128()) {
                    (Some(lo), Some(hi)) => assert_eq!(actual.unwrap().bounds(), Some((lo, hi))),
                    _ => assert_eq!(actual, Err(IntervalError::Overflow)),
                }
            }
        }
    }
    let unsigned = [0, 1, 2, 3, 1_u128 << 64, (1_u128 << 127) + 1, u128::MAX];
    for &a in &unsigned {
        for &b in &unsigned {
            if a > b {
                continue;
            }
            for &factor in &unsigned {
                let lo = Integer::from(a) * Integer::from(factor);
                let hi = Integer::from(b) * Integer::from(factor);
                let actual = IntegerRange::new(a, b).unwrap().scale(factor);
                match (lo.to_u128(), hi.to_u128()) {
                    (Some(lo), Some(hi)) => assert_eq!(actual.unwrap().bounds(), Some((lo, hi))),
                    _ => assert_eq!(actual, Err(IntervalError::Overflow)),
                }
            }
        }
    }
}

#[cfg(not(target_env = "msvc"))]
#[test]
fn ordered_sum_preserves_full_width_bounds_and_overflow() {
    use rug::Integer;
    let endpoints = [
        i128::MIN,
        i128::MIN + 1,
        -3,
        -1,
        0,
        1,
        3,
        i128::MAX - 1,
        i128::MAX,
    ];
    for &a in &endpoints {
        for &b in &endpoints {
            if a > b {
                continue;
            }
            let left = IntegerRange::new(a, b).unwrap();
            assert_eq!(
                left.checked_add(&IntegerRange::empty()),
                Ok(IntegerRange::empty())
            );
            for &c in &endpoints {
                for &d in &endpoints {
                    if c > d {
                        continue;
                    }
                    let right = IntegerRange::new(c, d).unwrap();
                    let lower = Integer::from(a) + Integer::from(c);
                    let upper = Integer::from(b) + Integer::from(d);
                    let actual = left.checked_add(&right);
                    match (lower.to_i128(), upper.to_i128()) {
                        (Some(lower), Some(upper)) => {
                            assert_eq!(actual.unwrap().bounds(), Some((lower, upper)))
                        }
                        _ => assert_eq!(actual, Err(IntervalError::Overflow)),
                    }
                }
            }
        }
    }
}
