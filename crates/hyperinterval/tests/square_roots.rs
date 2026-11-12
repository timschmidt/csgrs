#![cfg(feature = "dyadic")]
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{DomainCoverage, DyadicBudget, DyadicInterval, IntervalError};

fn budget() -> DyadicBudget {
    DyadicBudget::new(16384, 100_000_000_000)
}
fn d(a: i64, b: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), exponent).unwrap()
}

#[test]
fn square_root_reports_domain_and_preserves_empty() {
    for precision in [i32::MIN, -128, 0, 32, i32::MAX] {
        let empty = DyadicInterval::empty()
            .sqrt(precision, &mut DyadicBudget::new(0, 0))
            .unwrap();
        assert!(empty.range.is_empty());
        assert_eq!(empty.domain, DomainCoverage::Complete);
        assert_eq!(
            d(-4, -1, 0).sqrt(precision, &mut budget()),
            Err(IntervalError::EmptyDomain)
        );
        assert_eq!(
            d(-4, 0, 0).sqrt(precision, &mut budget()).unwrap().range,
            d(0, 0, 0)
        );
    }
    for (input, expected, domain) in [
        (d(-1, 4, 0), d(0, 2, 0), DomainCoverage::Partial),
        (d(0, 4, 0), d(0, 2, 0), DomainCoverage::Complete),
        (d(1, 9, 0), d(1, 3, 0), DomainCoverage::Complete),
        (d(2, 2, 0), d(362, 363, -8), DomainCoverage::Complete),
    ] {
        let result = input.sqrt(-8, &mut budget()).unwrap();
        assert_eq!(result.range, expected);
        assert_eq!(result.domain, domain);
    }
}

#[test]
fn budgets_refuse_before_growth_and_allow_independent_retry() {
    let x = d(2, 3, 0);
    for precision in [i32::MIN, i32::MAX] {
        let mut work = DyadicBudget::new(64, 100_000);
        assert_eq!(
            x.sqrt(precision, &mut work),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(work.remaining_work(), 99_999);
    }
    assert_eq!(
        x.sqrt(-32, &mut DyadicBudget::new(1024, 0)),
        Err(IntervalError::WorkExhausted)
    );
    // The division stage fits, but the root-extraction reservation does not.
    assert_eq!(
        d(2, 2, 0).sqrt(0, &mut DyadicBudget::new(3, 1000)),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        d(2, 2, 0).sqrt(0, &mut DyadicBudget::new(4, 37)),
        Err(IntervalError::WorkExhausted)
    );
    let result = x.sqrt(-32, &mut budget()).unwrap();
    assert!(
        x.subset_of(&result.range.square(&mut budget()).unwrap(), &mut budget())
            .unwrap()
    );
    // Shared accounting: first attempt completes, then no budget for another.
    let mut measured = budget();
    x.sqrt(-32, &mut measured).unwrap();
    let used = budget().remaining_work() - measured.remaining_work();
    let mut shared = DyadicBudget::new(16384, used);
    assert_eq!(x.sqrt(-32, &mut shared).unwrap(), result);
    assert_eq!(shared.remaining_work(), 0);
    assert_eq!(x.sqrt(-32, &mut shared), Err(IntervalError::WorkExhausted));
    assert_eq!(x.sqrt(-32, &mut budget()).unwrap(), result);
    // Full input exponent range; choose a nearby output grid so no giant
    // integer is materialized merely to accommodate the scale tag.
    assert_eq!(
        d(1, 1, i32::MIN)
            .sqrt(i32::MIN / 2, &mut budget())
            .unwrap()
            .range,
        d(1, 1, i32::MIN / 2)
    );
    assert_eq!(
        d(1, 1, i32::MAX)
            .sqrt(i32::MAX / 2, &mut budget())
            .unwrap()
            .range,
        d(1, 2, i32::MAX / 2)
    );
}

#[cfg(feature = "serde")]
#[test]
fn replay_refinement_and_further_operations_preserve_enclosures() {
    let source = d(1, 7, -3);
    let coarse = source.sqrt(-8, &mut budget()).unwrap();
    let fine = source.sqrt(-64, &mut budget()).unwrap();
    assert!(fine.range.subset_of(&coarse.range, &mut budget()).unwrap());
    let bytes = serde_json::to_vec(&fine.range).unwrap();
    let restored: DyadicInterval = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(restored, fine.range);
    assert_eq!(serde_json::to_vec(&restored).unwrap(), bytes);
    let fourth_root = restored.sqrt(-96, &mut budget()).unwrap().range;
    let fourth_power = fourth_root.checked_pow(4, &mut budget()).unwrap();
    assert!(source.subset_of(&fourth_power, &mut budget()).unwrap());
    let inverse = restored.reciprocal(-96, &mut budget()).unwrap();
    let product = restored.mul(&inverse, &mut budget()).unwrap();
    assert!(product.contains_point(&1.into(), 0, &mut budget()).unwrap());
}

#[cfg(not(target_env = "msvc"))]
#[path = "support/sqrt_oracle.rs"]
mod sqrt_oracle;

#[cfg(not(target_env = "msvc"))]
mod independent {
    use super::sqrt_oracle::{q, verify};
    use super::*;
    #[cfg(feature = "rational")]
    use rug::{Integer, Rational as Q};

    #[test]
    fn dyadic_roots_are_tight_on_small_large_and_near_square_inputs() {
        for a in -8..=16 {
            for b in a..=16 {
                for exponent in [-15, -1, 0, 1, 15] {
                    for precision in [-32, -1, 0, 16] {
                        let x = d(a, b, exponent);
                        let result = x.sqrt(precision, &mut budget());
                        if b < 0 {
                            assert_eq!(result, Err(IntervalError::EmptyDomain));
                            continue;
                        }
                        let result = result.unwrap();
                        assert_eq!(
                            result.domain,
                            if a < 0 {
                                DomainCoverage::Partial
                            } else {
                                DomainCoverage::Complete
                            }
                        );
                        verify(
                            &result.range,
                            q(&a.max(0), exponent),
                            q(&b, exponent),
                            precision,
                        );
                    }
                }
            }
        }
        for bits in [64usize, 127, 513, 1024] {
            let square = num_bigint::BigInt::from(1) << (2 * bits);
            for delta in [-1i32, 0, 1] {
                let input = &square + delta;
                for exponent in [-2000, -1074, 0, 2000] {
                    let x = DyadicInterval::point(input.clone(), exponent).unwrap();
                    for precision in [exponent / 2 - 32, exponent / 2, exponent / 2 + 32] {
                        let result = x.sqrt(precision, &mut budget()).unwrap();
                        let source = q(&input, exponent);
                        verify(&result.range, source.clone(), source, precision);
                    }
                }
            }
        }
    }

    #[cfg(feature = "rational")]
    #[test]
    fn rational_roots_are_tight_without_rounding_the_source_first() {
        use hyperinterval::RationalInterval;
        use hyperreal::Rational;
        for precision in [i32::MIN, -32, 0, i32::MAX] {
            assert!(
                RationalInterval::empty()
                    .sqrt(precision, &mut DyadicBudget::new(0, 0))
                    .unwrap()
                    .range
                    .is_empty()
            );
            assert_eq!(
                RationalInterval::point((-1).into()).sqrt(precision, &mut budget()),
                Err(IntervalError::EmptyDomain)
            );
            let zero = RationalInterval::new((-1).into(), 0.into())
                .unwrap()
                .sqrt(precision, &mut budget())
                .unwrap();
            assert_eq!(zero.range, d(0, 0, 0));
            assert_eq!(zero.domain, DomainCoverage::Partial);
        }
        for denominator in [1u64, 3, 7, 23, 65537] {
            for a in -4..=12 {
                for b in a.max(0)..=12 {
                    let x = RationalInterval::new(
                        Rational::fraction(a, denominator).unwrap(),
                        Rational::fraction(b, denominator).unwrap(),
                    )
                    .unwrap();
                    for precision in [-48, -3, 0, 8] {
                        let result = x.sqrt(precision, &mut budget()).unwrap();
                        assert_eq!(
                            result.domain,
                            if a < 0 {
                                DomainCoverage::Partial
                            } else {
                                DomainCoverage::Complete
                            }
                        );
                        verify(
                            &result.range,
                            Q::from((a.max(0), denominator)),
                            Q::from((b, denominator)),
                            precision,
                        );
                    }
                }
            }
        }
        // Huge denominators and rational values immediately around a perfect
        // square exercise exactness decisions that a rounded source could lose.
        let large = Integer::from(1) << 1024u32;
        for source in [
            Q::from((Integer::from(1), large.clone() + 1)),
            Q::from((large.clone() - 1, large.clone())),
            Q::from((large.clone() + 1, large)),
        ] {
            let x = RationalInterval::point(source.to_string().parse::<Rational>().unwrap());
            for precision in [-600, -32, 0] {
                let result = x.sqrt(precision, &mut budget()).unwrap();
                verify(&result.range, source.clone(), source.clone(), precision);
            }
        }
        let huge = (Integer::from(1) << 1024u32) + 1;
        let source = Q::from((huge, Integer::from(7)));
        let exact = source.to_string().parse::<Rational>().unwrap();
        let x = RationalInterval::point(exact);
        let root = x.sqrt(-128, &mut budget()).unwrap();
        verify(&root.range, source.clone(), source, -128);
        assert_eq!(
            x.sqrt(-128, &mut DyadicBudget::new(64, 1000)),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(
            x.sqrt(-128, &mut DyadicBudget::new(16384, 0)),
            Err(IntervalError::WorkExhausted)
        );
        assert_eq!(x.sqrt(-128, &mut budget()).unwrap(), root);
    }
}

#[cfg(feature = "rational")]
#[test]
fn rational_domain_and_second_endpoint_failure_are_portable() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational as R;
    assert_eq!(
        I::empty()
            .sqrt(0, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .domain,
        DomainCoverage::Complete
    );
    for (a, b, domain, expected) in [
        (-1, 4, DomainCoverage::Partial, d(0, 2, 0)),
        (0, 4, DomainCoverage::Complete, d(0, 2, 0)),
        (1, 9, DomainCoverage::Complete, d(1, 3, 0)),
        (-1, 0, DomainCoverage::Partial, d(0, 0, 0)),
    ] {
        let x = I::new(a.into(), b.into()).unwrap();
        let result = x.sqrt(-32, &mut budget()).unwrap();
        assert_eq!(result.range, expected);
        assert_eq!(result.domain, domain);
    }
    let x = I::new(R::zero(), R::from(2)).unwrap();
    // Zero lower endpoint succeeds; no incomplete result is returned when
    // the nonzero upper endpoint cannot reserve its materialization.
    assert_eq!(
        x.sqrt(-32, &mut DyadicBudget::new(64, 1)),
        Err(IntervalError::WorkExhausted)
    );
    let retry = x.sqrt(-32, &mut budget()).unwrap();
    assert!(
        x.subset_of(
            &retry
                .range
                .square(&mut budget())
                .unwrap()
                .to_rational(&mut budget())
                .unwrap()
        )
    );
    assert_eq!(
        I::point(R::fraction(9, 16).unwrap())
            .sqrt(-2, &mut budget())
            .unwrap()
            .range,
        d(3, 3, -2)
    );
    assert_eq!(
        I::point(R::from(-2)).sqrt(0, &mut budget()),
        Err(IntervalError::EmptyDomain)
    );
}
