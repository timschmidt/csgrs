#![cfg(feature = "dyadic")]

use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use num_bigint::BigInt;

fn d(lo: i64, hi: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lo.into(), hi.into(), exponent).unwrap()
}

fn budget() -> DyadicBudget {
    DyadicBudget::new(4096, 1_000_000_000)
}

#[cfg(not(target_env = "msvc"))]
#[path = "support/dyadic_oracle.rs"]
mod dyadic_oracle;

#[cfg(not(target_env = "msvc"))]
#[test]
fn affine_images_match_gmp_endpoint_images_and_compose() {
    use dyadic_oracle::q;
    for lo in -4..=4 {
        for hi in lo..=4 {
            for (input_grid, factor_grid, offset_grid) in [(-7, 2, -11), (4, -5, 3), (0, 0, 0)] {
                let input = d(lo, hi, input_grid);
                for factor in -3..=3 {
                    for offset in [-3, 0, 6] {
                        let (scale, shift) = (BigInt::from(factor), BigInt::from(offset));
                        let mut work = budget();
                        let scaled = input.scale((&scale, factor_grid), &mut work).unwrap();
                        let (l, h, e) = scaled.bounds().unwrap();
                        let left = q(&lo, input_grid) * q(&factor, factor_grid);
                        let right = q(&hi, input_grid) * q(&factor, factor_grid);
                        assert_eq!(q(l, e), left.clone().min(right.clone()));
                        assert_eq!(q(h, e), left.max(right));
                        let image = input
                            .affine((&scale, factor_grid), (&shift, offset_grid), &mut work)
                            .unwrap();
                        let value = |x| {
                            q(&x, input_grid) * q(&factor, factor_grid) + q(&offset, offset_grid)
                        };
                        let (a, b) = (value(lo), value(hi));
                        let (l, h, e) = image.bounds().unwrap();
                        assert_eq!(q(l, e), a.clone().min(b.clone()));
                        assert_eq!(q(h, e), a.clone().max(b.clone()));
                        let reflection = image
                            .affine((&(-BigInt::from(1)), 0), (&shift, offset_grid), &mut work)
                            .unwrap();
                        let (l, h, e) = reflection.bounds().unwrap();
                        assert_eq!(q(l, e), q(&offset, offset_grid) - a.clone().max(b.clone()));
                        assert_eq!(q(h, e), q(&offset, offset_grid) - a.min(b));
                    }
                }
            }
        }
    }
}

#[test]
fn empty_constants_and_extreme_grids() {
    let (zero, one, two, minus_two) = (0.into(), 1.into(), 2.into(), (-2).into());
    let huge = BigInt::from(1) << 8192;
    assert_eq!(
        DyadicInterval::empty().affine(
            (&huge, i32::MAX),
            (&huge, i32::MIN),
            &mut DyadicBudget::new(0, 0)
        ),
        Ok(DyadicInterval::empty())
    );
    for input in [d(0, 0, 0), d(1, 7, i32::MAX)] {
        let scale = if input.is_singleton() { &huge } else { &zero };
        assert_eq!(
            input.affine((scale, i32::MAX), (&minus_two, i32::MIN), &mut budget()),
            Ok(d(-1, -1, i32::MIN + 1))
        );
    }
    assert_eq!(
        d(-2, 3, i32::MAX).affine((&minus_two, 1), (&one, i32::MAX), &mut budget()),
        Ok(d(-11, 9, i32::MAX))
    );
    assert_eq!(
        d(-1, 3, i32::MIN).affine((&two, -1), (&one, i32::MIN), &mut budget()),
        Ok(d(0, 4, i32::MIN))
    );
    assert_eq!(
        d(1, 1, i32::MIN).affine((&one, -1), (&one, i32::MIN), &mut budget()),
        Err(IntervalError::Overflow)
    );
    assert_eq!(
        d(1, 1, i32::MAX).affine((&one, i32::MAX), (&zero, 0), &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        d(1, 1, i32::MIN).affine((&one, 0), (&one, i32::MAX), &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
}

#[test]
fn signed_scale_reserves_two_endpoint_products_and_retries() {
    let scale = BigInt::from(-3);
    let zero = BigInt::from(0);
    let source = d(-3, 5, 0);
    let run = |b: &mut DyadicBudget| source.affine((&scale, 0), (&zero, i32::MIN), b);
    let mut work = DyadicBudget::new(5, 50);
    assert_eq!(run(&mut work), Ok(d(-15, 9, 0)));
    assert_eq!(work.remaining_work(), 0);
    assert_eq!(
        run(&mut DyadicBudget::new(5, 49)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        run(&mut DyadicBudget::new(4, 50)),
        Err(IntervalError::SizeExhausted)
    );
    let generic = DyadicInterval::point(scale.clone(), 0).unwrap();
    assert_eq!(
        source.mul(&generic, &mut DyadicBudget::new(5, 50)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(run(&mut DyadicBudget::new(5, 50)), Ok(d(-15, 9, 0)));
}

#[test]
fn translating_a_wide_product_can_cancel_at_either_endpoint() {
    let a = (BigInt::from(1) << 130usize) + 1u32;
    let b = &a + 4u32;
    let scale = (BigInt::from(1) << 80usize) - 1u32;
    let source = DyadicInterval::new(a.clone(), b.clone(), -70).unwrap();
    for m in [scale.clone(), -scale] {
        for pivot in [&a, &b] {
            let offset = -(pivot * &m);
            let got = source
                .affine((&m, 90), (&offset, 20), &mut budget())
                .unwrap();
            let l = (&a - pivot) * &m;
            let h = (&b - pivot) * &m;
            let expected = DyadicInterval::new(l.clone().min(h.clone()), l.max(h), 20).unwrap();
            assert_eq!(got, expected);
        }
    }
}

#[test]
fn translation_uses_the_remaining_product_budget() {
    let source = d(-3, 5, 0);
    let (factor, offset) = (BigInt::from(-3), BigInt::from(1));
    let run = |work: &mut DyadicBudget| source.affine((&factor, 0), (&offset, 0), work);
    let mut only_products = DyadicBudget::new(5, 50);
    assert_eq!(run(&mut only_products), Err(IntervalError::WorkExhausted));
    assert_eq!(only_products.remaining_work(), 0);
    // Two five-bit products and two five-bit sums complete this equal-grid map.
    let mut both_stages = DyadicBudget::new(5, 60);
    assert_eq!(run(&mut both_stages), Ok(d(-14, 10, 0)));
    assert_eq!(both_stages.remaining_work(), 0);
}

#[test]
fn repairs_the_scalar_before_reserving_products_at_the_minimum_grid() {
    let source = d(-3, 5, i32::MIN);
    let factor = -(BigInt::from(3) << 120usize);
    // Inspect and shift the 122-bit factor once, then form two five-bit products.
    let mut work = DyadicBudget::new(122, 122 + 2 * 5 * 5);
    assert_eq!(
        source.scale((&factor, -120), &mut work),
        Ok(d(-15, 9, i32::MIN))
    );
    assert_eq!(work.remaining_work(), 0);
    let odd = factor + 1;
    assert_eq!(
        source.scale((&odd, -120), &mut budget()),
        Err(IntervalError::Overflow)
    );
}

#[test]
fn upper_grid_shift_is_reserved_before_multiplication() {
    let unit = BigInt::from(-1);
    let range = d(0, 3, i32::MAX);
    assert_eq!(
        range.scale((&unit, 128), &mut DyadicBudget::new(128, u64::MAX)),
        Err(IntervalError::SizeExhausted)
    );
    let expected = DyadicInterval::new(BigInt::from(-3) << 128usize, 0.into(), i32::MAX).unwrap();
    assert_eq!(range.scale((&unit, 128), &mut budget()), Ok(expected));
}
