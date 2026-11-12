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
        d(1, 1, i32::MAX).mul(&d(1, 1, 1), &mut budget()),
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
