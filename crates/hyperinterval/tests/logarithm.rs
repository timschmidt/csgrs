#![cfg(feature = "dyadic")]
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedIntervalView, IntervalError,
};
fn budget() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
#[test]
fn logarithm_reports_empty_partial_and_unbounded_images() {
    assert!(
        DyadicInterval::empty()
            .log(0, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .range
            .is_empty()
    );
    assert_eq!(
        d(-1, 0, 0).log(0, &mut budget()),
        Err(IntervalError::EmptyDomain)
    );
    let result = d(-1, 4, 0).log(-8, &mut budget()).unwrap();
    assert_eq!(result.domain, DomainCoverage::Partial);
    assert!(matches!(
        result.range.view(),
        ExtendedIntervalView::Below(_)
    ));
    let unit = d(1, 1, 0).log(-32, &mut budget()).unwrap();
    assert_eq!(unit.domain, DomainCoverage::Complete);
    assert_eq!(unit.range.as_finite(), Some(&d(0, 0, 0)));
}

#[cfg(not(target_env = "msvc"))]
#[path = "support/log_oracle.rs"]
mod oracle;

#[cfg(not(target_env = "msvc"))]
#[test]
fn dyadic_log_matches_directed_mpfr_across_domains_and_scales() {
    for e in [-12, -1, 0, 8] {
        for a in -3..=12 {
            for b in a.max(1)..=12 {
                for precision in [-64, -8, 0, 8] {
                    let result = d(a, b, e).log(precision, &mut budget()).unwrap();
                    assert_eq!(
                        result.domain,
                        if a <= 0 {
                            DomainCoverage::Partial
                        } else {
                            DomainCoverage::Complete
                        }
                    );
                    oracle::verify(
                        &result.range,
                        &oracle::q(&a, e),
                        &oracle::q(&b, e),
                        precision,
                    );
                }
            }
        }
    }
    for e in [i32::MIN, -1074, -512, 0, 512, i32::MAX] {
        let result = d(1, 1, e).log(-64, &mut budget()).unwrap();
        oracle::log_two_multiple(&result.range, i64::from(e), -64);
    }
}
#[cfg(all(feature = "rational", not(target_env = "msvc")))]
#[test]
fn rational_log_preserves_near_one_relations_and_extreme_coefficients() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational as R;
    use rug::{Integer, Rational as Q};
    for denominator in [1u64, 3, 7, 65537] {
        for a in -2..=12 {
            let b = a + 3;
            let x = I::new(
                R::fraction(a, denominator).unwrap(),
                R::fraction(b, denominator).unwrap(),
            )
            .unwrap();
            for precision in [-128, -16, 0] {
                let result = x.log(precision, &mut budget()).unwrap();
                oracle::verify(
                    &result.range,
                    &Q::from((a, denominator)),
                    &Q::from((b, denominator)),
                    precision,
                );
            }
        }
    }
    let large = Integer::from(1) << 1024u32;
    for source in [
        Q::from((large.clone() - 1, large.clone())),
        Q::from((large.clone() + 1, large.clone())),
        Q::from((1, large.clone() + 1)),
        Q::from(large),
    ] {
        let input = I::point(source.to_string().parse::<R>().unwrap());
        let result = input
            .log(-128, &mut DyadicBudget::new(4096, 1_000_000_000))
            .unwrap();
        oracle::verify(&result.range, &source, &source, -128);
        assert_eq!(
            input.log(-128, &mut DyadicBudget::new(32, 10000)),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(input.log(-128, &mut budget()).unwrap(), result);
    }
    assert!(
        I::empty()
            .log(i32::MIN, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .range
            .is_empty()
    );
    assert_eq!(
        I::point(R::zero()).log(0, &mut budget()),
        Err(IntervalError::EmptyDomain)
    );
}
#[test]
fn logarithm_replay_composition_and_resource_failure() {
    let x = d(1, 7, -2);
    assert_eq!(
        x.log(-32, &mut DyadicBudget::new(4096, 0)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        x.log(-128, &mut DyadicBudget::new(16, 1_000_000)),
        Err(IntervalError::SizeExhausted)
    );
    let result = x.log(-64, &mut budget()).unwrap();
    let coarse = x.log(-16, &mut budget()).unwrap();
    assert!(
        result
            .range
            .subset_of(&coarse.range, &mut budget())
            .unwrap()
    );
    let replay = result.range.exp(-64, &mut budget()).unwrap();
    assert!(
        x.subset_of(replay.range.as_finite().unwrap(), &mut budget())
            .unwrap()
    );
    let partial = d(-1, 4, 0).log(-64, &mut budget()).unwrap();
    assert_eq!(partial.domain, DomainCoverage::Partial);
    let reconstructed = partial.range.exp(-64, &mut budget()).unwrap();
    assert_eq!(reconstructed.domain, DomainCoverage::Complete); // operation-local; retain the earlier Partial receipt
    assert!(
        d(0, 4, 0)
            .subset_of(reconstructed.range.as_finite().unwrap(), &mut budget())
            .unwrap()
    );
    assert_eq!(
        partial.range.finite_range(&mut budget()),
        Err(IntervalError::UnboundedImage)
    );
    for precision in [i32::MIN, 0, i32::MAX] {
        assert_eq!(
            d(1, 1, 0)
                .log(precision, &mut budget())
                .unwrap()
                .range
                .as_finite(),
            Some(&d(0, 0, 0))
        );
        assert_eq!(
            d(-1, 0, 0).log(precision, &mut DyadicBudget::new(0, 0)),
            Err(IntervalError::EmptyDomain)
        );
    }
    #[cfg(feature = "serde")]
    {
        use hyperinterval::ExtendedInterval;
        let bytes = serde_json::to_vec(&partial.range).unwrap();
        let restored: ExtendedInterval<DyadicInterval> = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(restored, partial.range);
        assert_eq!(serde_json::to_vec(&restored).unwrap(), bytes);
        assert_eq!(restored.exp(-64, &mut budget()).unwrap(), reconstructed);
    }
}
