#![cfg(feature = "dyadic")]
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError, SignSet};
use num_bigint::BigInt;

fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn budget() -> DyadicBudget {
    DyadicBudget::new(4096, 1_000_000_000)
}

#[test]
fn nonnegative_powers_match_independent_integer_images() {
    for a in -6i64..=6 {
        for b in a..=6 {
            for scale in -3..=3 {
                for exponent in 0..=9 {
                    let x = d(a, b, scale);
                    // Integer endpoint and zero critical values give extrema.
                    let values: Vec<_> = (a..=b).map(|n| n.pow(exponent)).collect();
                    let expected = d(
                        *values.iter().min().unwrap(),
                        *values.iter().max().unwrap(),
                        scale * exponent as i32,
                    );
                    let mut work = budget();
                    let actual = x.checked_pow(exponent, &mut work).unwrap();
                    assert_eq!(actual, expected);
                    assert!(
                        actual
                            .subset_of(&actual.regrid(0, &mut work).unwrap(), &mut work)
                            .unwrap()
                    );
                    if !actual.signs().contains_zero() {
                        // Reciprocal composition contains the multiplicative
                        // identity, without asserting that the interval is a point.
                        let inverse = actual.reciprocal(-24, &mut work).unwrap();
                        let product = actual.mul(&inverse, &mut work).unwrap();
                        assert!(product.contains_point(&1.into(), 0, &mut work).unwrap());
                    }
                }
            }
        }
    }
    // Interval multiplication loses dependence for even powers across zero.
    assert_eq!(
        d(-2, 3, 0).checked_pow(4, &mut budget()).unwrap(),
        d(0, 81, 0)
    );
}

#[test]
fn powers_bound_growth_and_preserve_zero_units_and_exponent_extremes() {
    let mut no_budget = DyadicBudget::new(0, 0);
    for exponent in [0, 1, 2, 3, u32::MAX] {
        assert_eq!(
            DyadicInterval::empty()
                .checked_pow(exponent, &mut no_budget)
                .unwrap(),
            DyadicInterval::empty()
        );
    }
    assert_eq!(
        d(0, 0, 0).checked_pow(0, &mut budget()).unwrap(),
        d(1, 1, 0)
    );
    for (input, exponent, expected) in [
        (d(-1, 1, 0), u32::MAX, d(-1, 1, 0)),
        (d(-1, 1, 0), u32::MAX - 1, d(0, 1, 0)),
        (d(-1, -1, 0), u32::MAX - 1, d(1, 1, 0)),
        (d(0, 0, 0), u32::MAX, d(0, 0, 0)),
        (d(0, 1, 0), u32::MAX, d(0, 1, 0)),
    ] {
        assert_eq!(
            input
                .checked_pow(exponent, &mut DyadicBudget::new(1, 2))
                .unwrap(),
            expected
        );
    }
    assert_eq!(
        d(3, 4, 0).checked_pow(u32::MAX, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        d(1, 1, i32::MAX).checked_pow(u32::MAX, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        d(1, 1, i32::MIN).checked_pow(u32::MAX, &mut budget()),
        Err(IntervalError::Overflow)
    );
    // [-1,2]^4 produces [0,16], allowing a four-bit exact scale correction.
    assert_eq!(
        d(-1, 2, -536_870_913)
            .checked_pow(4, &mut budget())
            .unwrap(),
        d(0, 1, i32::MIN)
    );
    assert_eq!(
        d(1, 2, 715_827_883).checked_pow(3, &mut budget()).unwrap(),
        d(4, 32, i32::MAX)
    );
    let x = d(-3, 5, -2);
    assert_eq!(
        x.checked_pow(7, &mut DyadicBudget::new(4096, 0)),
        Err(IntervalError::WorkExhausted)
    );
    // The first unit endpoint succeeds; the second endpoint exceeds size.
    let mut small = DyadicBudget::new(10, 1000);
    assert_eq!(
        d(1, 3, 0).checked_pow(7, &mut small),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(small.remaining_work(), 999);
    assert_eq!(
        x.checked_pow(7, &mut budget()).unwrap(),
        d(-2187, 78125, -14)
    );
    assert_eq!(
        x.checked_pow(0, &mut no_budget),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        x.checked_pow(1, &mut no_budget),
        Err(IntervalError::SizeExhausted)
    );
}

#[test]
fn exact_measures_and_square_match_integer_oracles_and_reconstruct_input() {
    for a in -9i64..=9 {
        for b in a..=9 {
            for e in -4..=4 {
                let x = d(a, b, e);
                let mut work = budget();
                assert_eq!(x.width(&mut work).unwrap(), d(b - a, b - a, e));
                let midpoint = x.midpoint(&mut work).unwrap();
                let radius = x.radius(&mut work).unwrap();
                assert_eq!(midpoint, d(a + b, a + b, e - 1));
                assert_eq!(radius, d(b - a, b - a, e - 1));
                // Consume exact measurements through the existing set algebra.
                let lower = midpoint.sub(&radius, &mut work).unwrap();
                let upper = midpoint.add(&radius, &mut work).unwrap();
                assert_eq!(lower.hull(&upper, &mut work).unwrap(), x);
                // Integer endpoints and the stationary point 0 suffice for x².
                let min_square = (a..=b).map(|n| n * n).min().unwrap();
                let max_square = (a..=b).map(|n| n * n).max().unwrap();
                let squared = x.square(&mut work).unwrap();
                assert_eq!(squared, d(min_square, max_square, 2 * e));
                assert!(
                    squared
                        .subset_of(&x.mul(&x, &mut work).unwrap(), &mut work)
                        .unwrap()
                );
                for p in -10i64..=10 {
                    // Point uses a finer grid than x; no floating-point oracle.
                    assert_eq!(
                        x.contains_point(&p.into(), e - 1, &mut work).unwrap(),
                        2 * a <= p && p <= 2 * b
                    );
                }
                let (lo, hi, scale) = x.bounds().unwrap();
                assert_eq!(x.lower(), Some((lo, scale)));
                assert_eq!(x.upper(), Some((hi, scale)));
            }
        }
    }
}

#[test]
fn measures_handle_empty_exponent_limits_and_retry() {
    let empty = DyadicInterval::empty();
    let mut no_work = DyadicBudget::new(0, 0);
    assert_eq!(empty.lower(), None);
    assert_eq!(empty.upper(), None);
    assert!(
        !empty
            .contains_point(&1.into(), i32::MIN, &mut no_work)
            .unwrap()
    );
    for result in [
        empty.width(&mut no_work),
        empty.midpoint(&mut no_work),
        empty.radius(&mut no_work),
        empty.square(&mut no_work),
        empty.reciprocal(i32::MIN, &mut no_work),
    ] {
        assert_eq!(result.unwrap(), empty);
    }
    let minimum = d(0, 1, i32::MIN);
    assert_eq!(
        minimum.midpoint(&mut budget()),
        Err(IntervalError::Overflow)
    );
    assert_eq!(minimum.radius(&mut budget()), Err(IntervalError::Overflow));
    assert_eq!(minimum.width(&mut budget()).unwrap(), d(1, 1, i32::MIN));
    assert_eq!(
        d(1, 3, i32::MIN).midpoint(&mut budget()).unwrap(),
        d(1, 1, i32::MIN + 1)
    );
    assert_eq!(
        d(1, 3, i32::MIN).radius(&mut budget()).unwrap(),
        d(1, 1, i32::MIN)
    );
    assert_eq!(
        d(-1, 1, i32::MIN).midpoint(&mut budget()).unwrap(),
        d(0, 0, 0)
    );
    assert_eq!(d(1, 1, i32::MIN).radius(&mut budget()).unwrap(), d(0, 0, 0));
    assert_eq!(
        d(-1, 1, i32::MAX).width(&mut budget()).unwrap(),
        d(2, 2, i32::MAX)
    );
    // A zero-crossing square can remove common factors even when its input
    // cannot. Conversely a positive square may be genuinely unrepresentable.
    assert_eq!(
        d(-1, 2, -1_073_741_825).square(&mut budget()).unwrap(),
        d(0, 1, i32::MIN)
    );
    assert_eq!(
        d(1, 2, -1_073_741_825).square(&mut budget()),
        Err(IntervalError::Overflow)
    );
    assert_eq!(
        d(-2, 3, -1_073_741_825).square(&mut budget()),
        Err(IntervalError::Overflow)
    );
    for exponent in [-1_073_741_825, 1_073_741_824] {
        // Products fit the budget, but correcting the temporary scale does not.
        assert_eq!(
            d(-1, 2, exponent).square(&mut DyadicBudget::new(64, 32)),
            Err(IntervalError::WorkExhausted)
        );
        assert!(d(-1, 2, exponent).square(&mut budget()).is_ok());
    }
    assert_eq!(
        d(1, 2, 1_073_741_824).square(&mut budget()).unwrap(),
        d(2, 8, i32::MAX)
    );
    assert_eq!(
        d(1, 2, i32::MAX).square(&mut budget()),
        Err(IntervalError::SizeExhausted)
    );

    let x = d(-7, 9, -3);
    for result in [
        x.width(&mut no_work),
        x.midpoint(&mut no_work),
        x.radius(&mut no_work),
        x.square(&mut no_work),
        d(1, 2, 0).reciprocal(-3, &mut no_work),
    ] {
        assert_eq!(result, Err(IntervalError::SizeExhausted));
    }
    let mut no_work = DyadicBudget::new(4096, 0);
    assert_eq!(x.width(&mut no_work), Err(IntervalError::WorkExhausted));
    assert_eq!(
        x.contains_point(&1.into(), 0, &mut no_work),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(x.square(&mut no_work), Err(IntervalError::WorkExhausted));
    assert_eq!(x.square(&mut budget()).unwrap(), d(0, 81, -6));
    assert_eq!(x.midpoint(&mut budget()).unwrap(), d(1, 1, -3));
    assert_eq!(
        x.reciprocal(0, &mut no_work),
        Err(IntervalError::ContainsZero)
    );
    assert_eq!(
        d(1, 2, 0).reciprocal(i32::MIN, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
}

#[test]
fn wide_mantissas_retain_exact_measures_and_square() {
    let n: BigInt = (BigInt::from(1) << 512usize) - 1;
    let x = DyadicInterval::new(-&n, n.clone(), -37).unwrap();
    let mut work = budget();
    assert_eq!(x.midpoint(&mut work).unwrap(), d(0, 0, 0));
    assert_eq!(
        x.radius(&mut work).unwrap(),
        DyadicInterval::point(n.clone(), -37).unwrap()
    );
    assert_eq!(
        x.width(&mut work).unwrap(),
        DyadicInterval::point(n, -36).unwrap()
    );
    // Independent binomial identity (2^512 - 1)^2 = 2^1024 - 2^513 + 1.
    let squared = (BigInt::from(1) << 1024usize) - (BigInt::from(1) << 513usize) + 1;
    assert_eq!(
        x.square(&mut work).unwrap(),
        DyadicInterval::new(0.into(), squared, -74).unwrap()
    );
    assert_eq!(
        x.square(&mut DyadicBudget::new(1023, u64::MAX)),
        Err(IntervalError::SizeExhausted)
    );
    assert!(x.square(&mut DyadicBudget::new(1024, 3_000_000)).is_ok());
}

#[test]
fn reciprocal_matches_independent_integer_floor_and_ceiling() {
    for a in -9i128..=9 {
        for b in a..=9 {
            if a <= 0 && b >= 0 {
                continue;
            }
            for e in -3..=3 {
                for p in -4..=4 {
                    let shift: i32 = -e - p;
                    let numerator = 1i128 << shift.max(0);
                    let floor = |den: i128| {
                        let den = den << (-shift).max(0);
                        let q = numerator / den;
                        q - i128::from(numerator % den != 0 && den < 0)
                    };
                    let ceil = |den: i128| {
                        let den = den << (-shift).max(0);
                        let q = numerator / den;
                        q + i128::from(numerator % den != 0 && den > 0)
                    };
                    let actual = d(a as i64, b as i64, e)
                        .reciprocal(p, &mut budget())
                        .unwrap();
                    assert_eq!(
                        actual,
                        DyadicInterval::new(floor(b).into(), ceil(a).into(), p).unwrap()
                    );
                }
            }
        }
    }
}

#[test]
fn canonical_common_scale_extremes_and_exact_relations() {
    assert_eq!(d(-8, 12, -3), d(-2, 3, -1));
    assert_eq!(d(0, 0, i32::MIN), d(0, 0, 0));
    assert_eq!(d(0, 0, i32::MAX).signs(), SignSet::Zero);
    assert_eq!(d(0, 8, -2), d(0, 1, 1));
    assert_eq!(d(4, 8, i32::MAX - 1), d(2, 4, i32::MAX));
    assert_eq!(
        DyadicInterval::new(1.into(), 0.into(), 0),
        Err(IntervalError::ReversedEndpoints)
    );
    let a = d(-2, 3, -1);
    let b = d(3, 8, -2);
    assert_eq!(a.add(&b, &mut budget()).unwrap(), d(-1, 14, -2));
    assert_eq!(a.sub(&b, &mut budget()).unwrap(), d(-12, 3, -2));
    assert_eq!(a.neg(&mut budget()).unwrap(), d(-3, 2, -1));
    assert_eq!(a.mul(&b, &mut budget()).unwrap(), d(-16, 24, -3));
    assert_eq!(a.hull(&b, &mut budget()).unwrap(), d(-1, 2, 0));
    assert_eq!(a.intersection(&b, &mut budget()).unwrap(), d(3, 6, -2));
    assert!(d(4, 5, -2).interior_subset_of(&a, &mut budget()).unwrap());
    assert!(!a.interior_subset_of(&a, &mut budget()).unwrap());
    assert!(d(-7, -5, -2).strictly_left_of(&b, &mut budget()).unwrap());
    assert!(d(-4, 6, -2).subset_of(&a, &mut budget()).unwrap());
    assert!(
        d(99, 100, 1)
            .intersection(&a, &mut budget())
            .unwrap()
            .is_empty()
    );
    let empty = DyadicInterval::empty();
    assert!(empty.subset_of(&a, &mut budget()).unwrap());
    assert!(!a.subset_of(&empty, &mut budget()).unwrap());
    assert!(!empty.strictly_left_of(&a, &mut budget()).unwrap());
    assert!(empty.interior_subset_of(&empty, &mut budget()).unwrap());
    assert_eq!(empty.hull(&a, &mut budget()).unwrap(), a);
    assert_eq!(a.hull(&empty, &mut budget()).unwrap(), a);
    assert!(empty.hull(&empty, &mut budget()).unwrap().is_empty());
    assert_eq!(empty.bounds(), None);
    assert!(!empty.is_singleton());
    assert_eq!(empty.signs(), SignSet::Empty);
    for result in [
        a.add(&empty, &mut budget()),
        a.sub(&empty, &mut budget()),
        a.mul(&empty, &mut budget()),
        a.div(&empty, 0, &mut budget()),
        empty.neg(&mut budget()),
        empty.regrid(i32::MIN, &mut budget()),
        a.intersection(&empty, &mut budget()),
    ] {
        assert!(result.unwrap().is_empty());
    }
    assert_eq!(
        empty.div(&a, 0, &mut budget()),
        Err(IntervalError::ContainsZero)
    );
}

#[test]
fn coarsening_is_outward_and_huge_scales_fail_before_allocation() {
    let a = d(-7, 9, -3);
    assert_eq!(a.regrid(-1, &mut budget()).unwrap(), d(-2, 3, -1));
    assert_eq!(a.regrid(-20, &mut budget()).unwrap(), a);
    assert_eq!(
        a.regrid(i32::MAX, &mut budget()).unwrap(),
        d(-1, 1, i32::MAX)
    );
    let mut tiny = DyadicBudget::new(64, 100_000);
    assert_eq!(
        a.regrid(i32::MIN, &mut tiny),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(tiny.remaining_work(), 100_000);
    assert_eq!(tiny.max_bits(), 64);
    assert_eq!(
        d(1, 1, i32::MAX).add(&d(1, 1, i32::MIN), &mut tiny),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        d(1, 1, i32::MAX).mul(&d(1, 1, 1), &mut budget()).unwrap(),
        d(2, 2, i32::MAX)
    );
    assert_eq!(
        d(1, 1, i32::MIN).mul(&d(1, 1, -1), &mut budget()),
        Err(IntervalError::Overflow)
    );
    assert_eq!(
        d(0, 0, 0).mul(&d(1, 1, i32::MAX), &mut budget()).unwrap(),
        d(0, 0, 0)
    );
    assert_eq!(
        d(1, 1, i32::MAX).div(&d(1, 1, i32::MIN), i32::MIN, &mut tiny),
        Err(IntervalError::SizeExhausted)
    );
    let mut work = DyadicBudget::new(4096, 0);
    assert_eq!(a.add(&a, &mut work), Err(IntervalError::WorkExhausted));
    assert_eq!(a.add(&a, &mut budget()).unwrap(), d(-7, 9, -2));
    let mut saturated = DyadicBudget::new(u64::MAX, u64::MAX);
    assert_eq!(
        d(1, 1, i32::MAX).div(&d(1, 1, i32::MIN), i32::MIN, &mut saturated),
        Err(IntervalError::WorkExhausted)
    );
}

#[test]
fn dyadic_division_matches_independent_small_integer_floor_and_ceiling() {
    // All scalar endpoint quotients on this reduced domain, including negative
    // denominators and off-grid results. Oracle uses only i128 arithmetic.
    for a in -9i128..=9 {
        for b in -9i128..=9 {
            if b == 0 {
                continue;
            }
            for e in -3..=3 {
                for f in -3..=3 {
                    for p in [-4, 0, 4] {
                        let shift = e - f - p;
                        let n = a << shift.max(0);
                        let den = b << (-shift).max(0);
                        let q = n / den;
                        let rem = n % den;
                        let lo = q - i128::from(rem != 0 && (n < 0) != (den < 0));
                        let hi = q + i128::from(rem != 0 && (n < 0) == (den < 0));
                        let actual = d(a as i64, a as i64, e)
                            .div(&d(b as i64, b as i64, f), p, &mut budget())
                            .unwrap();
                        assert_eq!(
                            actual,
                            DyadicInterval::new(BigInt::from(lo), BigInt::from(hi), p).unwrap()
                        );
                    }
                }
            }
        }
    }
}

#[cfg(feature = "rational")]
#[test]
fn rational_bridges_preserve_exact_sets_and_round_only_outward() {
    use hyperinterval::RationalInterval;
    use hyperreal::Rational;
    for a in -8..=8 {
        for b in a..=8 {
            for e in -4..=4 {
                let original = d(a, b, e);
                let rational = original.to_rational(&mut budget()).unwrap();
                let (lo, hi) = rational.bounds().unwrap();
                assert_eq!(
                    DyadicInterval::from_grid_bounds(lo, hi, e, &mut budget()).unwrap(),
                    original
                );
                assert_eq!(rational.to_dyadic(e, &mut budget()).unwrap(), original);
                assert_eq!(
                    original
                        .square(&mut budget())
                        .unwrap()
                        .to_rational(&mut budget())
                        .unwrap(),
                    rational.square()
                );
                for (measured, expected) in [
                    (original.width(&mut budget()), rational.width()),
                    (original.midpoint(&mut budget()), rational.midpoint()),
                    (original.radius(&mut budget()), rational.radius()),
                ] {
                    assert_eq!(
                        measured.unwrap().to_rational(&mut budget()).unwrap(),
                        RationalInterval::point(expected.unwrap())
                    );
                }
                let coarse = original
                    .regrid(2, &mut budget())
                    .unwrap()
                    .to_rational(&mut budget())
                    .unwrap();
                assert!(rational.subset_of(&coarse));
            }
        }
    }
    let third = Rational::fraction(1, 3).unwrap();
    assert_eq!(
        DyadicInterval::from_grid_bounds(&third, &third, -8, &mut budget()),
        Err(IntervalError::NonDyadicEndpoint)
    );
    assert_eq!(
        DyadicInterval::from_grid_bounds(&Rational::one(), &Rational::zero(), 0, &mut budget()),
        Err(IntervalError::ReversedEndpoints)
    );
    let x = RationalInterval::point(third);
    assert_eq!(x.to_dyadic(-3, &mut budget()).unwrap(), d(2, 3, -3));
    assert!(
        DyadicInterval::empty()
            .to_rational(&mut budget())
            .unwrap()
            .is_empty()
    );
    assert!(
        RationalInterval::empty()
            .to_dyadic(0, &mut budget())
            .unwrap()
            .is_empty()
    );
    assert_eq!(
        d(1, 1, i32::MIN).to_rational(&mut budget()),
        Err(IntervalError::SizeExhausted)
    );
}

#[cfg(feature = "serde")]
#[test]
fn checked_dyadic_replay_normalizes_and_rejects_reversed_input() {
    for value in [
        DyadicInterval::empty(),
        d(-7, 8, -9),
        d(0, 0, 0),
        d(2, 4, i32::MAX),
        d(-7, 8, -9).square(&mut budget()).unwrap(),
        d(-7, 8, -9).midpoint(&mut budget()).unwrap(),
        d(-7, 8, -9).radius(&mut budget()).unwrap(),
        d(3, 7, -9).reciprocal(-16, &mut budget()).unwrap(),
        d(-3, 7, -9).checked_pow(7, &mut budget()).unwrap(),
    ] {
        let bytes = serde_json::to_vec(&value).unwrap();
        assert_eq!(
            serde_json::from_slice::<DyadicInterval>(&bytes).unwrap(),
            value
        );
    }
    let reversed = serde_json::to_vec(&Some((BigInt::from(2), BigInt::from(-1), 0))).unwrap();
    assert!(serde_json::from_slice::<DyadicInterval>(&reversed).is_err());
    let unnormalized = serde_json::to_vec(&Some((BigInt::from(4), BigInt::from(8), -1))).unwrap();
    assert_eq!(
        serde_json::from_slice::<DyadicInterval>(&unnormalized).unwrap(),
        d(1, 2, 1)
    );
}
