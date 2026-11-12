#![cfg(feature = "dyadic")]
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{DomainCoverage, DyadicBudget, DyadicInterval};
#[test]
fn atan_zero_empty_and_domain() {
    let budget = || DyadicBudget::new(4096, 1_000_000_000);
    let zero = DyadicInterval::point(0.into(), 0).unwrap();
    let image = zero.atan(-64, &mut budget()).unwrap();
    assert_eq!(image.range, zero);
    assert_eq!(image.domain, DomainCoverage::Complete);
    assert!(
        DyadicInterval::empty()
            .atan(i32::MIN, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .range
            .is_empty()
    );
}

fn budget() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
#[cfg(not(target_env = "msvc"))]
#[path = "support/atan_oracle.rs"]
mod oracle;
#[cfg(not(target_env = "msvc"))]
#[test]
fn directed_mpfr_across_reductions_and_scales() {
    for e in [-12, -1, 0, 8] {
        for a in -8..=8 {
            for b in a..=8 {
                for p in [-64, -8, 0, 8] {
                    let result = d(a, b, e).atan(p, &mut budget()).unwrap();
                    assert_eq!(result.domain, DomainCoverage::Complete);
                    oracle::verify(&result.range, &oracle::q(&a, e), &oracle::q(&b, e), p);
                }
            }
        }
    }
    for precision in [-512, -2048] {
        for (a, e) in [(1, -1), (1, 0), (2, 0), (1, -512)] {
            let result = d(a, a, e).atan(precision, &mut budget()).unwrap();
            oracle::verify(
                &result.range,
                &oracle::q(&a, e),
                &oracle::q(&a, e),
                precision,
            );
        }
    }
    for e in [-1074, -512, 512, 4096] {
        for sign in [-1, 1] {
            let x = d(sign, sign, e);
            let y = x.atan(-256, &mut budget()).unwrap();
            oracle::verify(&y.range, &oracle::q(&sign, e), &oracle::q(&sign, e), -256);
        }
    }
    // atan(x)=pi/2-atan(1/x), and 0<atan(1/x)<1/x<2^-1024.
    let (lo, hi) = oracle::half_pi();
    let epsilon = rug::Rational::from((1, rug::Integer::from(1) << 1024u32));
    for p in [-512, -64, 0, 1, 8] {
        let result = d(1, 1, i32::MAX).atan(p, &mut budget()).unwrap();
        oracle::finite(&result.range, lo.clone() - &epsilon, hi.clone(), p);
        assert_eq!(
            d(-1, -1, i32::MAX).atan(p, &mut budget()).unwrap().range,
            result.range.neg(&mut budget()).unwrap()
        );
    }
    for p in [i32::MIN, -256, 0, i32::MAX] {
        let tiny = d(1, 1, i32::MIN).atan(p, &mut budget()).unwrap();
        assert_eq!(tiny.range, d(0, 1, p));
        assert_eq!(
            d(-1, -1, i32::MIN).atan(p, &mut budget()).unwrap().range,
            d(-1, 0, p)
        );
    }
}
#[cfg(all(feature = "rational", not(target_env = "msvc")))]
#[test]
fn rational_maintains_exact_boundaries_and_large_coefficients() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational as R;
    use rug::{Integer, Rational as Q};
    for denominator in [3u64, 7, 65537] {
        for a in -8..=8 {
            let b = a + 3;
            let input = I::new(
                R::fraction(a, denominator).unwrap(),
                R::fraction(b, denominator).unwrap(),
            )
            .unwrap();
            for p in [-128, -16, 0] {
                let result = input.atan(p, &mut budget()).unwrap();
                assert_eq!(result.domain, DomainCoverage::Complete);
                oracle::verify(
                    &result.range,
                    &Q::from((a, denominator)),
                    &Q::from((b, denominator)),
                    p,
                );
            }
        }
    }
    let base = Integer::from(1) << 1024u32;
    let step = Q::from((1, base.clone()));
    let mut cases = vec![Q::from((1, base.clone() + 1)), Q::from(base)];
    for boundary in [Q::from((1, 2)), Q::from(1), Q::from(2)] {
        cases.push(boundary.clone() - &step);
        cases.push(boundary.clone());
        cases.push(boundary + &step);
    }
    for source in cases {
        for sign in [-1, 1] {
            let source: Q = source.clone() * sign;
            let input = I::point(source.to_string().parse::<R>().unwrap());
            let result = input
                .atan(-128, &mut DyadicBudget::new(4096, 1_000_000_000))
                .unwrap();
            oracle::verify(&result.range, &source, &source, -128);
            if source.denom().significant_bits() > 32 || source.numer().significant_bits() > 32 {
                assert_eq!(
                    input.atan(-128, &mut DyadicBudget::new(32, 1_000_000)),
                    Err(hyperinterval::IntervalError::SizeExhausted)
                );
            }
            assert_eq!(input.atan(-128, &mut budget()).unwrap(), result);
        }
    }
    assert!(
        I::empty()
            .atan(i32::MIN, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .range
            .is_empty()
    );
}
#[test]
fn resource_retry_refinement_and_further_operations() {
    use hyperinterval::{ExtendedInterval as E, IntervalError};
    let x = d(-3, 7, -1);
    assert_eq!(
        x.atan(-64, &mut DyadicBudget::new(4096, 0)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        x.atan(-128, &mut DyadicBudget::new(16, 1_000_000)),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        x.atan(i32::MIN, &mut budget()),
        Err(IntervalError::Overflow)
    );
    let result = x.atan(-64, &mut budget()).unwrap();
    let coarse = x.atan(-16, &mut budget()).unwrap();
    assert!(
        result
            .range
            .subset_of(&coarse.range, &mut budget())
            .unwrap()
    );
    assert_eq!(
        x.neg(&mut budget())
            .unwrap()
            .atan(-64, &mut budget())
            .unwrap()
            .range,
        result.range.neg(&mut budget()).unwrap()
    );
    assert!(
        !result
            .range
            .atan(-64, &mut budget())
            .unwrap()
            .range
            .is_empty()
    );
    assert!(
        !result
            .range
            .exp(-64, &mut budget())
            .unwrap()
            .range
            .is_empty()
    );
    let logarithm = d(-1, 4, 0).log(-64, &mut budget()).unwrap();
    assert_eq!(logarithm.domain, DomainCoverage::Partial);
    assert_eq!(
        logarithm.range.atan(-64, &mut budget()).unwrap().domain,
        DomainCoverage::Complete
    );
    for p in [i32::MIN, 0, i32::MAX] {
        assert_eq!(d(0, 0, 0).atan(p, &mut budget()).unwrap().range, d(0, 0, 0));
    }
    assert!(
        E::<DyadicInterval>::empty()
            .atan(i32::MIN, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .range
            .is_empty()
    );
    assert_eq!(E::from_finite(x).atan(-64, &mut budget()).unwrap(), result);
    #[cfg(feature = "serde")]
    {
        let bytes = serde_json::to_vec(&result.range).unwrap();
        let restored: DyadicInterval = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(restored, result.range);
        assert_eq!(serde_json::to_vec(&restored).unwrap(), bytes);
        assert_eq!(
            restored.atan(-64, &mut budget()).unwrap(),
            result.range.atan(-64, &mut budget()).unwrap()
        );
    }
}
#[cfg(not(target_env = "msvc"))]
#[test]
fn unbounded_sources_use_certified_limits() {
    use hyperinterval::ExtendedInterval as E;
    let (lo, hi) = oracle::half_pi();
    for p in [-256, -64, 0, 1, i32::MAX] {
        let image = E::<DyadicInterval>::entire()
            .atan(p, &mut budget())
            .unwrap();
        // Avoid oracle allocation at the deliberately extreme coarse grid.
        if p == i32::MAX {
            assert_eq!(image.range, d(-1, 1, p));
        } else {
            oracle::finite(&image.range, -hi.clone(), hi.clone(), p);
        }
        assert_eq!(image.domain, DomainCoverage::Complete);
        if p != i32::MAX {
            let below = E::below(d(0, 0, 0))
                .unwrap()
                .atan(p, &mut budget())
                .unwrap();
            oracle::finite(&below.range, -hi.clone(), 0.into(), p);
            let above = E::above(d(0, 0, 0))
                .unwrap()
                .atan(p, &mut budget())
                .unwrap();
            oracle::finite(&above.range, 0.into(), hi.clone(), p);
            assert_eq!(above.range.neg(&mut budget()).unwrap(), below.range);
            let whole = above.range.hull(&below.range, &mut budget()).unwrap();
            assert_eq!(whole, image.range);
        }
    }
    assert!(lo < hi);
}
