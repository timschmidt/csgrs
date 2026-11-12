#![cfg(feature = "dyadic")]
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{DomainCoverage, DyadicBudget, DyadicInterval, IntervalError};
fn budget() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
#[test]
fn exponential_empty_zero_and_negative_tail() {
    for precision in [i32::MIN, -128, 0, 1, i32::MAX] {
        let empty = DyadicInterval::empty()
            .exp(precision, &mut DyadicBudget::new(0, 0))
            .unwrap();
        assert!(empty.range.is_empty());
        assert_eq!(empty.domain, DomainCoverage::Complete);
        assert_eq!(
            d(0, 0, 0).exp(precision, &mut budget()).unwrap().range,
            if precision <= 0 {
                d(1, 1, 0)
            } else {
                d(0, 1, precision)
            }
        );
        assert_eq!(
            d(-1, -1, i32::MAX)
                .exp(precision, &mut budget())
                .unwrap()
                .range,
            d(0, 1, precision)
        );
    }
    assert_eq!(
        d(-3, -3, -1).exp(-1, &mut budget()).unwrap().range,
        d(0, 1, -1)
    );
    assert_eq!(
        d(-1, -1, -1).exp(-1, &mut budget()).unwrap().range,
        d(1, 2, -1)
    );
    assert_eq!(d(1, 1, 0).exp(0, &mut budget()).unwrap().range, d(2, 3, 0));
    assert_eq!(
        d(-1, -1, 0).exp(-3, &mut budget()).unwrap().range,
        d(2, 3, -3)
    );
    assert_eq!(
        d(1, 1, 0).exp(-32, &mut DyadicBudget::new(64, 0)),
        Err(IntervalError::WorkExhausted)
    );
}

#[cfg(not(target_env = "msvc"))]
#[path = "support/exp_oracle.rs"]
mod oracle;

#[cfg(not(target_env = "msvc"))]
#[test]
fn dyadic_exp_matches_independent_directed_bounds() {
    for exponent in [-8, -1, 0, 3] {
        for a in -12..=12 {
            for b in [a, a + 1, 12].into_iter().filter(|b| *b >= a) {
                for precision in [-64, -16, -1, 0, 8] {
                    let result = d(a, b, exponent).exp(precision, &mut budget()).unwrap();
                    assert_eq!(result.domain, DomainCoverage::Complete);
                    oracle::verify(
                        &result.range,
                        &oracle::q(&a, exponent),
                        &oracle::q(&b, exponent),
                        precision,
                    );
                }
            }
        }
    }
    for (a, e, p) in [
        (1, -1074, -128),
        (-1, -1074, -128),
        (1, 10, -64),
        (-1, 10, -2048),
        (1, 0, -1024),
    ] {
        let result = d(a, a, e).exp(p, &mut budget()).unwrap();
        oracle::verify(&result.range, &oracle::q(&a, e), &oracle::q(&a, e), p);
    }
    // Tiny but nonzero source values still select the correct side of one;
    // their enormous denominator need never be materialized.
    assert_eq!(
        d(1, 1, i32::MIN).exp(-8, &mut budget()).unwrap().range,
        d(256, 257, -8)
    );
    assert_eq!(
        d(-1, -1, i32::MIN).exp(-8, &mut budget()).unwrap().range,
        d(255, 256, -8)
    );
}

#[cfg(all(feature = "rational", not(target_env = "msvc")))]
#[test]
fn rational_exp_preserves_exact_source_and_matches_mpfr() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational as R;
    use rug::Rational as Q;
    for denominator in [1u64, 3, 7, 65537] {
        for a in -8..=8 {
            let b = a + 3;
            let x = I::new(
                R::fraction(a, denominator).unwrap(),
                R::fraction(b, denominator).unwrap(),
            )
            .unwrap();
            for precision in [-128, -32, 0, 8] {
                let result = x.exp(precision, &mut budget()).unwrap();
                oracle::verify(
                    &result.range,
                    &Q::from((a, denominator)),
                    &Q::from((b, denominator)),
                    precision,
                );
            }
        }
    }
    let huge = rug::Integer::from(1) << 1024u32;
    for source in [
        Q::from((huge.clone() + 1, huge.clone())),
        Q::from((1, huge.clone() + 1)),
        Q::from((-1, huge)),
    ] {
        let x = I::point(source.to_string().parse::<R>().unwrap());
        let result = x.exp(-128, &mut budget()).unwrap();
        oracle::verify(&result.range, &source, &source, -128);
        assert_eq!(
            x.exp(-128, &mut DyadicBudget::new(64, 1_000_000)),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(x.exp(-128, &mut budget()).unwrap(), result);
    }
    for source in ["0", "1/3", "-1/3"] {
        let x = I::point(source.parse::<R>().unwrap());
        let result = x.exp(-128, &mut budget()).unwrap();
        let q = source.parse::<Q>().unwrap();
        oracle::verify(&result.range, &q, &q, -128);
    }
    assert!(
        I::empty()
            .exp(i32::MIN, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .range
            .is_empty()
    );
}

#[test]
fn resource_refusal_retry_and_successive_operations() {
    let x = d(-1, 1, 0);
    assert_eq!(
        x.exp(-64, &mut DyadicBudget::new(16, 1_000_000)),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        x.exp(-64, &mut DyadicBudget::new(1024, 10)),
        Err(IntervalError::WorkExhausted)
    );
    // A completed first endpoint must not be published when the second fails.
    assert_eq!(
        d(0, 1, 0).exp(-32, &mut DyadicBudget::new(1, 1000)),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        d(1, 1, 0).exp(i32::MIN, &mut budget()),
        Err(IntervalError::Overflow)
    );
    let first = x.exp(-64, &mut budget()).unwrap();
    assert_eq!(first, x.exp(-64, &mut budget()).unwrap());
    let coarse = x.exp(-16, &mut budget()).unwrap();
    assert!(first.range.subset_of(&coarse.range, &mut budget()).unwrap());
    let next = first.range.exp(-16, &mut budget()).unwrap();
    assert!(!next.range.is_empty());
    let positive = d(1, 1, 0).exp(-64, &mut budget()).unwrap().range;
    let negative = d(-1, -1, 0).exp(-64, &mut budget()).unwrap().range;
    assert!(
        positive
            .mul(&negative, &mut budget())
            .unwrap()
            .contains_point(&1.into(), 0, &mut budget())
            .unwrap()
    );
    assert_eq!(
        d(1, 1, i32::MAX).exp(0, &mut DyadicBudget::new(256, 1_000_000_000_000)),
        Err(IntervalError::SizeExhausted)
    );
    #[cfg(feature = "serde")]
    {
        let bytes = serde_json::to_vec(&first.range).unwrap();
        let restored: DyadicInterval = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(restored, first.range);
        assert_eq!(serde_json::to_vec(&restored).unwrap(), bytes);
        assert_eq!(restored.exp(-16, &mut budget()).unwrap(), next);
    }
}

#[test]
fn interval_assembly_selects_only_extreme_endpoint_bounds() {
    assert_eq!(
        d(-2, -1, i32::MAX)
            .exp(i32::MIN, &mut budget())
            .unwrap()
            .range,
        d(0, 1, i32::MIN)
    );

    assert_eq!(
        d(-1, 0, i32::MAX)
            .exp(i32::MIN, &mut budget())
            .unwrap()
            .range,
        d(0, 1, 0)
    );
    #[cfg(feature = "rational")]
    {
        use hyperinterval::RationalInterval as I;
        use hyperreal::Rational as R;
        let a = R::from_bigint(-(num_bigint::BigInt::from(1) << 1024usize));
        assert_eq!(
            I::new(a, R::zero())
                .unwrap()
                .exp(i32::MIN, &mut budget())
                .unwrap()
                .range,
            d(0, 1, 0)
        );
    }
}
