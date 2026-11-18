#![cfg(feature = "dyadic")]

use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};

fn interval(lo: i64, hi: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lo.into(), hi.into(), exponent).unwrap()
}

#[test]
fn finite_order_distinguishes_contact_empty_and_exhausted_queries() {
    let a = interval(-3, 8, -2);
    let b = interval(1, 2, 1);
    let edge = interval(1, 1, 1);
    let empty = DyadicInterval::empty();
    let mut work = DyadicBudget::new(16, 128);
    assert_eq!(a.strictly_left_of(&b, &mut work), Ok(false));
    assert_eq!(edge.subset_of(&a, &mut work), Ok(true));
    assert_eq!(edge.interior_subset_of(&a, &mut work), Ok(false));
    let mut none = DyadicBudget::new(0, 0);
    assert_eq!(empty.interior_subset_of(&empty, &mut none), Ok(true));
    assert_eq!(a.subset_of(&empty, &mut none), Ok(false));
    assert_eq!(empty.strictly_left_of(&a, &mut none), Ok(false));
    assert_eq!(
        edge.subset_of(&a, &mut DyadicBudget::new(8, 0)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(edge.subset_of(&a, &mut work), Ok(true));
    let zero = interval(0, 0, 0);
    assert_eq!(
        zero.subset_of(&zero, &mut DyadicBudget::new(0, 4)),
        Ok(true)
    );
}

#[test]
fn mixed_grid_queries_match_independently_scaled_signed_integers() {
    for e in -2..=2 {
        for f in -2..=2 {
            for a in -3..=3 {
                for b in a..=3 {
                    let source = interval(a, b, e);
                    for c in -3..=3 {
                        for d in c..=3 {
                            let target = interval(c, d, f);
                            let (a, b, c, d) =
                                (a << (e + 2), b << (e + 2), c << (f + 2), d << (f + 2));
                            let mut budget = DyadicBudget::new(64, 1024);
                            assert_eq!(
                                source.subset_of(&target, &mut budget),
                                Ok(c <= a && b <= d)
                            );
                            assert_eq!(
                                source.interior_subset_of(&target, &mut budget),
                                Ok(c < a && b < d)
                            );
                            assert_eq!(source.strictly_left_of(&target, &mut budget), Ok(b < c));
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn equal_leading_powers_keep_the_lowest_significant_limb() {
    use num_bigint::BigInt;
    let unit = BigInt::from(1_u8);
    let pivot = &unit << 4096usize;
    for sign in [-1, 1] {
        let center = &pivot * sign;
        let wide = DyadicInterval::new(&center - &unit, &center + &unit, -4096).unwrap();
        let point = interval(sign, sign, 0);
        let right = DyadicInterval::point(&center + &unit, -4096).unwrap();
        let mut budget = DyadicBudget::new(4100, 100_000);
        assert_eq!(point.interior_subset_of(&wide, &mut budget), Ok(true));
        assert_eq!(point.strictly_left_of(&right, &mut budget), Ok(true));
        assert_eq!(right.interior_subset_of(&wide, &mut budget), Ok(false));
    }
}
