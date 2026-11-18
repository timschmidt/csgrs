#![cfg(feature = "dyadic")]

use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedIntervalView, IntervalError,
};

fn budget() -> DyadicBudget {
    DyadicBudget::new(16_384, 1_000_000_000_000)
}

fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}

#[test]
fn empty_exact_values_and_excluded_domains() {
    for precision in [i32::MIN, -64, 0, i32::MAX] {
        let empty = DyadicInterval::empty();
        for image in [
            empty.asin(precision, &mut DyadicBudget::new(0, 0)),
            empty.acos(precision, &mut DyadicBudget::new(0, 0)),
            empty.acosh(precision, &mut DyadicBudget::new(0, 0)),
        ] {
            let image = image.unwrap();
            assert!(image.range.is_empty());
            assert_eq!(image.domain, DomainCoverage::Complete);
        }
        let image = empty
            .atanh(precision, &mut DyadicBudget::new(0, 0))
            .unwrap();
        assert!(image.range.is_empty());
        assert_eq!(image.domain, DomainCoverage::Complete);
        assert_eq!(
            d(0, 0, 0).asin(precision, &mut budget()).unwrap().range,
            d(0, 0, 0)
        );
        assert_eq!(
            d(1, 1, 0).acos(precision, &mut budget()).unwrap().range,
            d(0, 0, 0)
        );
        assert_eq!(
            d(1, 1, 0).acosh(precision, &mut budget()).unwrap().range,
            d(0, 0, 0)
        );
        assert_eq!(
            d(0, 0, 0)
                .atanh(precision, &mut budget())
                .unwrap()
                .range
                .as_finite(),
            Some(&d(0, 0, 0))
        );
    }
    let zero = d(0, 0, 0);
    assert_eq!(zero.asin(-64, &mut budget()).unwrap().range, zero);
    assert_eq!(d(1, 1, 0).acosh(-64, &mut budget()).unwrap().range, zero);
    assert_eq!(
        zero.atanh(-64, &mut budget()).unwrap().range.as_finite(),
        Some(&zero)
    );
    assert!(
        d(1, 1, 0)
            .acos(-64, &mut budget())
            .unwrap()
            .range
            .contains_point(&0.into(), 0, &mut budget())
            .unwrap()
    );
    for source in [d(-3, -2, 0), d(2, 3, 0)] {
        assert_eq!(
            source.asin(-32, &mut budget()),
            Err(IntervalError::EmptyDomain)
        );
        assert_eq!(
            source.acos(-32, &mut budget()),
            Err(IntervalError::EmptyDomain)
        );
        assert_eq!(
            source.atanh(-32, &mut budget()),
            Err(IntervalError::EmptyDomain)
        );
    }
    assert_eq!(
        d(-5, 0, 0).acosh(-32, &mut budget()),
        Err(IntervalError::EmptyDomain)
    );
    for source in [d(-2, -1, 0), d(1, 2, 0), d(-1, -1, 0), d(1, 1, 0)] {
        assert_eq!(
            source.atanh(-32, &mut budget()),
            Err(IntervalError::EmptyDomain)
        );
    }
}

#[test]
fn coverage_and_unbounded_limits_are_not_silently_finite() {
    for function in [DyadicInterval::asin, DyadicInterval::acos] {
        let complete = function(&d(-1, 1, 0), -32, &mut budget()).unwrap();
        assert_eq!(complete.domain, DomainCoverage::Complete);
        let partial = function(&d(-2, 2, 0), -32, &mut budget()).unwrap();
        assert_eq!(partial.domain, DomainCoverage::Partial);
        assert_eq!(partial.range, complete.range);
    }
    let partial = d(0, 2, 0).acosh(-32, &mut budget()).unwrap();
    assert_eq!(partial.domain, DomainCoverage::Partial);
    assert_eq!(
        partial.range,
        d(1, 2, 0).acosh(-32, &mut budget()).unwrap().range
    );
    let both = d(-2, 2, 0).atanh(-32, &mut budget()).unwrap();
    assert!(both.range.is_entire());
    assert_eq!(both.domain, DomainCoverage::Partial);
    let below = d(-2, 0, 0).atanh(-32, &mut budget()).unwrap();
    assert_eq!(below.domain, DomainCoverage::Partial);
    assert!(matches!(below.range.view(), ExtendedIntervalView::Below(x) if x == &d(0,0,0)));
    let above = d(0, 2, 0).atanh(-32, &mut budget()).unwrap();
    assert_eq!(above.domain, DomainCoverage::Partial);
    assert!(matches!(above.range.view(), ExtendedIntervalView::Above(x) if x == &d(0,0,0)));
    assert_eq!(
        d(-1, 1, -1).atanh(-32, &mut budget()).unwrap().domain,
        DomainCoverage::Complete
    );
}

#[test]
fn images_are_consumable_and_budgets_retry_without_cached_exhaustion() {
    let source = d(-1, 1, -1);
    let original = budget();
    let mut consumed = original.clone();
    let expected = source.asin(-32, &mut consumed).unwrap();
    let used = original.remaining_work() - consumed.remaining_work();
    for work in [0, 1, used / 2, used - 1] {
        assert_eq!(
            source.asin(-32, &mut DyadicBudget::new(16_384, work)),
            Err(IntervalError::WorkExhausted)
        );
    }
    assert_eq!(
        source
            .asin(-32, &mut DyadicBudget::new(16_384, used))
            .unwrap(),
        expected
    );
    assert_eq!(
        source.asin(-32, &mut DyadicBudget::new(0, u64::MAX)),
        Err(IntervalError::SizeExhausted)
    );
    let recovered = expected.range.sin(-32, &mut budget()).unwrap();
    assert!(source.subset_of(&recovered.range, &mut budget()).unwrap());
    let twice = source.atanh(-32, &mut budget()).unwrap().range;
    let exponential = twice.exp(-32, &mut budget()).unwrap();
    assert_eq!(exponential.domain, DomainCoverage::Complete);
    assert!(!exponential.range.is_empty());
}

#[cfg(not(target_env = "msvc"))]
#[path = "support/grid_oracle.rs"]
mod grid_oracle;
#[cfg(not(target_env = "msvc"))]
use grid_oracle::q;
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;

#[cfg(not(target_env = "msvc"))]
#[test]
fn directed_mpfr_endpoint_oracle_across_domains_and_grids() {
    use rug::{Float, Rational, float::Round};
    mpfr_cache::cleanup_on_thread_exit();
    let directed = |n: i64, e: i32, function: usize, rounding| {
        let (mut value, _) = Float::with_val_round(512, grid_oracle::q(&n, e), rounding);
        match function {
            0 => {
                value.asin_round(rounding);
            }
            1 => {
                value.acos_round(rounding);
            }
            2 => {
                value.acosh_round(rounding);
            }
            _ => {
                value.atanh_round(rounding);
            }
        }
        value.to_rational().unwrap()
    };
    let enclose = |result: &DyadicInterval, lo: Rational, hi: Rational, precision| {
        let (lo, hi) = grid_oracle::bounds(lo, hi, precision);
        let (a, b, e) = result.bounds().unwrap();
        assert!(grid_oracle::q(a, e) <= lo, "lower {result:?}");
        assert!(grid_oracle::q(b, e) >= hi, "upper {result:?}");
        // The backend promises an outward grid, not a tight hull.
        assert!(e >= precision || (a == &0.into() && b == &0.into()));
    };
    for precision in [-64, -12, 0, 5] {
        for a in -8..=8 {
            for b in a..=8 {
                let source = d(a, b, -3);
                for (function, call) in [
                    (
                        0,
                        DyadicInterval::asin as fn(&DyadicInterval, i32, &mut DyadicBudget) -> _,
                    ),
                    (1, DyadicInterval::acos),
                ] {
                    let result = call(&source, precision, &mut budget()).unwrap();
                    let (lo, hi) = if function == 1 { (b, a) } else { (a, b) };
                    enclose(
                        &result.range,
                        directed(lo, -3, function, Round::Down),
                        directed(hi, -3, function, Round::Up),
                        precision,
                    );
                    assert_eq!(result.domain, DomainCoverage::Complete);
                }
                if a > -8 && b < 8 {
                    let result = source.atanh(precision, &mut budget()).unwrap();
                    enclose(
                        result.range.as_finite().unwrap(),
                        directed(a, -3, 3, Round::Down),
                        directed(b, -3, 3, Round::Up),
                        precision,
                    );
                    assert_eq!(result.domain, DomainCoverage::Complete);
                }
            }
        }
        for a in 4..=16 {
            for b in a..=16 {
                let result = d(a, b, -2).acosh(precision, &mut budget()).unwrap();
                enclose(
                    &result.range,
                    directed(a, -2, 2, Round::Down),
                    directed(b, -2, 2, Round::Up),
                    precision,
                );
                assert_eq!(result.domain, DomainCoverage::Complete);
            }
        }
    }
}

#[cfg(not(target_env = "msvc"))]
#[path = "support/atanh_oracle.rs"]
mod atanh_oracle;

#[cfg(not(target_env = "msvc"))]
#[test]
fn atanh_one_grid_step_either_side_of_each_pole() {
    for fraction_bits in [2, 16, 32, 60] {
        let one = 1_i64 << fraction_bits;
        let endpoints = [-one - 1, -one, -one + 1, 0, one - 1, one, one + 1];
        for (index, &a) in endpoints.iter().enumerate() {
            for &b in &endpoints[index..] {
                for precision in [-64, -8, 0, 5] {
                    atanh_oracle::verify(a, b, fraction_bits, precision);
                }
            }
        }
    }
}
