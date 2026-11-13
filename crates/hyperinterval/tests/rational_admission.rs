#![cfg(all(feature = "rational", feature = "dyadic"))]
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperreal::Rational;
use num_bigint::{BigInt, BigUint};
fn b() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
#[test]
fn admitted_operations_match_independent_integer_formulas_and_compose() {
    for a in -8_i64..=8 {
        for ad in 1_i64..=7 {
            for c in -8_i64..=8 {
                for cd in 1_i64..=7 {
                    let x = Rational::fraction(a, ad as u64).unwrap();
                    let y = Rational::fraction(c, cd as u64).unwrap();
                    let mut work = b();
                    assert_eq!(
                        work.rational().compare(&x, &y).unwrap(),
                        (a * cd).cmp(&(c * ad))
                    );
                    let sum = work.rational().add(&x, &y).unwrap();
                    let difference = work.rational().sub(&x, &y).unwrap();
                    let product = work.rational().mul(&x, &y).unwrap();
                    assert_eq!(
                        sum,
                        Rational::fraction(a * cd + c * ad, (ad * cd) as u64).unwrap()
                    );
                    assert_eq!(
                        difference,
                        Rational::fraction(a * cd - c * ad, (ad * cd) as u64).unwrap()
                    );
                    assert_eq!(
                        product,
                        Rational::fraction(a * c, (ad * cd) as u64).unwrap()
                    );
                    let image = DyadicInterval::from_rational(&sum, -32, &mut work).unwrap();
                    assert!(image.to_rational(&mut work).unwrap().contains_point(&sum));
                    if a * cd <= c * ad {
                        let span = work.rational().interval(x.clone(), y.clone()).unwrap();
                        assert_eq!(span.lower(), Some(&x));
                        assert_eq!(span.upper(), Some(&y));
                    } else {
                        assert_eq!(
                            work.rational().interval(x, y),
                            Err(IntervalError::ReversedEndpoints)
                        );
                    }
                }
            }
        }
    }
}
#[test]
fn exact_admission_limits_are_shared_with_conversions_and_retry() {
    let x = Rational::fraction(13, 17).unwrap();
    let y = Rational::fraction(-19, 23).unwrap();
    let run = |work: &mut DyadicBudget| -> Result<_, IntervalError> {
        let order = work.rational().compare(&x, &y)?;
        let sum = work.rational().add(&x, &y)?;
        let d = work.rational().sub(&x, &y)?;
        let m = work.rational().mul(&sum, &d)?;
        let image = DyadicInterval::from_rational(&m, -64, work)?;
        Ok((order, sum, d, m, image))
    };
    let mut measured = b();
    let expected = run(&mut measured).unwrap();
    let used = b().remaining_work() - measured.remaining_work();
    let mut exact = DyadicBudget::new(16384, used);
    assert_eq!(run(&mut exact).unwrap(), expected);
    assert_eq!(exact.remaining_work(), 0);
    assert_eq!(
        run(&mut DyadicBudget::new(16384, used - 1)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(run(&mut b()).unwrap(), expected);
    let mut two = DyadicBudget::new(16384, 2 * used);
    assert_eq!(run(&mut two).unwrap(), expected);
    assert_eq!(run(&mut two).unwrap(), expected);
    assert_eq!(two.remaining_work(), 0);
    // Stored sizes are 5 and 5; addition reserves the carry bit before cheap cancellation.
    assert_eq!(
        DyadicBudget::new(10, u64::MAX).rational().sub(&y, &y),
        Err(IntervalError::SizeExhausted)
    );
    let mut exact = DyadicBudget::new(11, 16 * 11 * 11);
    assert_eq!(exact.rational().sub(&y, &y).unwrap(), Rational::zero());
    assert_eq!(exact.remaining_work(), 0);
}
#[test]
fn large_and_unreduced_operands_are_admitted_before_cache_work() {
    for bits in [128_usize, 4096, 131072] {
        let x = Rational::from_bigint_fraction(
            BigInt::from((BigUint::from(1_u8) << bits) + BigUint::from(1_u8)),
            BigUint::from(3_u8),
        )
        .unwrap();
        let one = Rational::one();
        let mut small = DyadicBudget::new(64, 1000);
        assert_eq!(
            small.rational().compare(&x, &one),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(
            small.rational().add(&x, &one),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(
            small.rational().sub(&x, &x),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(
            small.rational().mul(&x, &one),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(small.remaining_work(), 1000);
        let mut retry = DyadicBudget::new(300000, 10_000_000_000_000);
        assert_eq!(retry.rational().sub(&x, &x).unwrap(), Rational::zero());
        assert_eq!(
            retry.rational().compare(&x, &one).unwrap(),
            core::cmp::Ordering::Greater
        );
    }
    let line = hyperreal::ExactDyadicLine2::from_f64([0.0, 0.0], [6.0, 6.0]).unwrap();
    let (_, point) = line.intersection_point_f64([0.0, 3.0], [6.0, 3.0]).unwrap();
    let mut unreduced = 0;
    for coordinate in &point {
        let x = coordinate.exact_rational_ref().unwrap();
        let raw = x.ratio_parts();
        let mut cold = b();
        assert_eq!(
            cold.rational().sub(x, &Rational::one()).unwrap(),
            Rational::from(2)
        );
        unreduced += usize::from(raw.1 != x.numerator() || raw.2 != x.denominator());
        let mut warm = b();
        assert_eq!(
            warm.rational().sub(x, &Rational::one()).unwrap(),
            Rational::from(2)
        );
        assert_eq!(cold.remaining_work(), warm.remaining_work());
        assert_eq!(x.ratio_parts(), raw);
    }
    assert!(unreduced > 0);
}
