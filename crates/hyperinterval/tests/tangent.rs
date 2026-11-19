#![cfg(feature = "dyadic")]
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::ExtendedInterval as E;
#[cfg(feature = "serde")]
use hyperinterval::IntervalSet2;
use hyperinterval::{DomainCoverage, DyadicBudget, DyadicInterval, IntervalError};
fn budget() -> DyadicBudget {
    DyadicBudget::new(32768, 10_000_000_000_000)
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
#[test]
fn pole_images_and_exact_endpoint_pole() {
    let x = DyadicInterval::new(1.into(), 2.into(), 0).unwrap();
    let image = x
        .tan(-64, &mut DyadicBudget::new(32768, 10_000_000_000_000))
        .unwrap();
    assert_eq!(image.domain, DomainCoverage::Partial);
    assert_eq!(image.range.components().len(), 2);
    assert_eq!(
        DyadicInterval::point(0.into(), 0)
            .unwrap()
            .cot(-64, &mut DyadicBudget::new(0, 0)),
        Err(IntervalError::EmptyDomain)
    );
}

#[test]
fn exact_zero_empty_endpoint_limits_and_whole_branches() {
    for cot in [false, true] {
        let empty = E::<DyadicInterval>::empty();
        let mut no_work = DyadicBudget::new(0, 0);
        let finite_empty = DyadicInterval::empty();
        let finite_result = if cot {
            finite_empty.cot(i32::MIN, &mut no_work)
        } else {
            finite_empty.tan(i32::MIN, &mut no_work)
        }
        .unwrap();
        assert!(finite_result.range.is_empty());
        assert_eq!(finite_result.domain, DomainCoverage::Complete);
        assert_eq!(
            finite_result.range.exp(i32::MIN, &mut no_work).unwrap(),
            finite_result
        );
        assert_eq!(no_work.remaining_work(), 0);
        let result = if cot {
            empty.cot(i32::MIN, &mut no_work)
        } else {
            empty.tan(i32::MIN, &mut no_work)
        }
        .unwrap();
        assert!(result.range.is_empty());
        assert_eq!(result.domain, DomainCoverage::Complete);
        for source in [
            E::entire(),
            E::below(d(1, 1, 0)).unwrap(),
            E::above(d(-1, -1, 0)).unwrap(),
        ] {
            let result = if cot {
                source.cot(0, &mut no_work)
            } else {
                source.tan(0, &mut no_work)
            }
            .unwrap();
            assert_eq!(result.range.components(), &[E::entire()]);
            assert_eq!(result.domain, DomainCoverage::Partial);
        }
        for source in [d(-8, 8, 0), d(0, 4, 0), d(-4, 0, 0)] {
            let result = if cot {
                source.cot(-64, &mut budget())
            } else {
                source.tan(-64, &mut budget())
            }
            .unwrap();
            assert_eq!(result.range.components(), &[E::entire()]);
            assert_eq!(result.domain, DomainCoverage::Partial);
        }
    }
    let zero = d(0, 0, 0).tan(i32::MIN, &mut budget()).unwrap();
    assert_eq!(zero.range.components(), &[E::from_finite(d(0, 0, 0))]);
    assert_eq!(zero.domain, DomainCoverage::Complete);
    let right = d(0, 1, 0).cot(-64, &mut budget()).unwrap();
    let left = d(-1, 0, 0).cot(-64, &mut budget()).unwrap();
    assert_eq!(right.domain, DomainCoverage::Partial);
    assert_eq!(left.domain, DomainCoverage::Partial);
    assert_eq!(right.range.neg(&mut budget()).unwrap(), left.range);
    assert!(matches!(
        right.range.components()[0].view(),
        hyperinterval::ExtendedIntervalView::Above(_)
    ));
}

#[cfg(feature = "rational")]
#[test]
fn finite_and_empty_adapters_preserve_domain_errors_and_receipts() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational as R;
    let empty = I::empty();
    let mut no_work = DyadicBudget::new(0, 0);
    for result in [
        empty.tan(i32::MIN, &mut no_work),
        empty.cot(i32::MIN, &mut no_work),
    ] {
        let result = result.unwrap();
        assert!(result.range.is_empty());
        assert_eq!(result.domain, DomainCoverage::Complete);
    }
    let zero = I::point(R::new(0));
    assert_eq!(zero.cot(-64, &mut no_work), Err(IntervalError::EmptyDomain));
    assert_eq!(
        zero.tan(-64, &mut budget()).unwrap().range.components(),
        &[E::from_finite(d(0, 0, 0))]
    );
    assert_eq!(
        E::from_finite(d(0, 0, 0)).cot(-64, &mut no_work),
        Err(IntervalError::EmptyDomain)
    );
    let finite = d(1, 2, 0);
    let extended = E::from_finite(finite.clone());
    assert_eq!(
        extended.tan(-64, &mut budget()).unwrap(),
        finite.tan(-64, &mut budget()).unwrap()
    );
    assert_eq!(
        extended.cot(-64, &mut budget()).unwrap(),
        finite.cot(-64, &mut budget()).unwrap()
    );
}

#[cfg(not(target_env = "msvc"))]
#[path = "support/tangent_oracle.rs"]
mod oracle;
#[cfg(not(target_env = "msvc"))]
#[test]
fn periodic_poles_and_monotone_ranges_match_mpfr() {
    use oracle::End::Value as V;
    for e in [-8, -1, 2] {
        for a in -3..=3 {
            for b in a..=3 {
                for p in [-64, 0, 4] {
                    let source = d(a, b, e);
                    let exact = Some((V(oracle::q(&a, e)), V(oracle::q(&b, e))));
                    oracle::verify(source.tan(p, &mut budget()), &exact, p, false);
                    oracle::verify(source.cot(p, &mut budget()), &exact, p, true);
                }
            }
        }
    }
    for (a, b) in [(1, 2), (-2, -1), (-1, 1), (3, 4), (-4, -3), (0, 1), (-1, 0)] {
        let exact = Some((V(a.into()), V(b.into())));
        oracle::verify(d(a, b, 0).tan(-128, &mut budget()), &exact, -128, false);
        oracle::verify(d(a, b, 0).cot(-128, &mut budget()), &exact, -128, true);
    }
}

#[cfg(all(feature = "rational", not(target_env = "msvc")))]
#[test]
fn exact_rational_sources_near_poles_and_huge_phases() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational as R;
    use oracle::End::Value as V;
    use rug::{Integer, Rational as Q};
    let verify = |a: Q, b: Q, p| {
        let x = I::new(
            a.to_string().parse::<R>().unwrap(),
            b.to_string().parse::<R>().unwrap(),
        )
        .unwrap();
        let exact = Some((V(a), V(b)));
        oracle::verify(x.tan(p, &mut budget()), &exact, p, false);
        oracle::verify(x.cot(p, &mut budget()), &exact, p, true);
    };
    for denominator in [3, 7, 65537] {
        for a in -3..=3 {
            verify(
                Q::from((a, denominator)),
                Q::from((a + 2, denominator + 1)),
                -64,
            );
        }
    }
    let (pi, _) = oracle::half_pi();
    let scale = Integer::from(1) << 128u32;
    // Include pi/4, where tan and cot both cross the output grid at one,
    // as well as the poles of tan at pi/2 and cot at pi.
    for quarters in [-10_i32, -8, -2, 0, 1, 2, 4, 8, 10] {
        let at = (pi.clone() * quarters * &scale / 2_i32)
            .floor()
            .numer()
            .clone();
        let a = Q::from((at.clone() - 1, scale.clone()));
        let b = Q::from((at + 2, scale.clone()));
        verify(a.clone(), a.clone(), -64);
        verify(b.clone(), b.clone(), -64);
        if matches!(quarters, 1 | 2 | 4) {
            verify(Q::from((1, 4)), a.clone(), -64);
            verify(Q::from((1, 4)), b.clone(), -64);
        }
        verify(a, b, -64);
    }
    for e in [128u32, 512] {
        let huge = Q::from(Integer::from(1) << e);
        verify(huge.clone(), huge.clone(), -64);
        verify(huge.clone(), huge + Q::from((1, 1024)), -64);
    }
}

#[cfg(not(target_env = "msvc"))]
#[test]
fn tiny_and_large_sources_refuse_then_retry_without_false_poles() {
    use oracle::End::Value as V;
    for (a, e, p) in [
        (1, -1074, -64),
        (1, -1074, -1074),
        (-1, -1074, -64),
        (1, -512, -64),
        (1, 128, -64),
        (1, 0, -256),
    ] {
        let source = d(a, a, e);
        let exact = Some((V(oracle::q(&a, e)), V(oracle::q(&a, e))));
        oracle::verify(source.tan(p, &mut budget()), &exact, p, false);
        oracle::verify(source.cot(p, &mut budget()), &exact, p, true);
    }
    let source = d(1, 2, 0);
    assert_eq!(
        source.tan(-64, &mut DyadicBudget::new(32768, 1)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        d(1, 1, 512).cot(-64, &mut DyadicBudget::new(64, u64::MAX)),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        d(1, 1, i32::MAX).tan(-64, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        d(1, 1, i32::MIN)
            .tan(-64, &mut budget())
            .unwrap()
            .range
            .components(),
        &[E::from_finite(d(0, 1, -64))]
    );
    let completed = source.tan(-64, &mut budget()).unwrap();
    assert_eq!(completed.domain, DomainCoverage::Partial);
    assert_eq!(completed.range.components().len(), 2);
    assert_eq!(source.tan(-64, &mut budget()).unwrap(), completed);
    let exp = completed.range.exp(-32, &mut budget()).unwrap();
    assert_eq!(exp.domain, DomainCoverage::Complete);
    assert_eq!(exp.range.components().len(), 2);
}

#[cfg(all(feature = "rational", not(target_env = "msvc")))]
#[test]
fn near_pole_uncertainty_is_not_a_proved_pole_and_rounding_can_close_gaps() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational as R;
    use rug::{Integer, Rational as Q};
    let (pi, _) = oracle::half_pi();
    let scale = Integer::from(1) << 128u32;
    let n = (pi * &scale).floor().numer().clone();
    let q = Q::from((n, scale));
    let source = I::point(q.to_string().parse::<R>().unwrap());
    let completed = source.tan(-64, &mut budget()).unwrap();
    assert_eq!(completed.domain, DomainCoverage::Complete);
    assert_eq!(completed.range.components().len(), 1);
    assert!(completed.range.components()[0].as_finite().is_some());
    for work in [10000, 1_000_000, 100_000_000] {
        match source.tan(-64, &mut DyadicBudget::new(32768, work)) {
            Ok(result) => assert_eq!(result, completed),
            Err(error) => assert_eq!(error, IntervalError::WorkExhausted),
        }
    }
    assert_eq!(source.tan(-64, &mut budget()).unwrap(), completed);
    for (source, cot) in [(d(1, 2, 0), false), (d(-1, 1, 0), true)] {
        let fine = if cot {
            source.cot(-64, &mut budget())
        } else {
            source.tan(-64, &mut budget())
        }
        .unwrap();
        let coarse = if cot {
            source.cot(4, &mut budget())
        } else {
            source.tan(4, &mut budget())
        }
        .unwrap();
        assert_eq!(fine.range.components().len(), 2);
        assert_eq!(coarse.range.components(), &[E::entire()]);
        assert_eq!(coarse.domain, DomainCoverage::Partial);
    }
}

#[cfg(feature = "serde")]
#[test]
fn replay_then_another_pole_operation_in_fresh_process() {
    const ENV: &str = "HYPERINTERVAL_TANGENT_REPLAY";
    if let Ok(bytes) = std::env::var(ENV) {
        let set: IntervalSet2<DyadicInterval> = serde_json::from_str(&bytes).unwrap();
        assert_eq!(serde_json::to_string(&set).unwrap(), bytes);
        assert!(!set.contains_point(&0.into(), 0, &mut budget()).unwrap());
        // A single pole lies between 1 and 2. The finite grid endpoints are
        // ceil(2^64 tan(2)) and floor(2^64 tan(1)), checked with directed MPFR.
        let point = |n: i128| DyadicInterval::point(n.into(), -64).unwrap();
        assert_eq!(
            set.components(),
            &[
                E::below(point(-40_306_871_148_438_554_474)).unwrap(),
                E::above(point(28_729_101_715_127_294_858)).unwrap(),
            ]
        );
        for component in set.components() {
            let further = component.cot(-32, &mut budget()).unwrap();
            assert_eq!(further.range.components(), &[E::entire()]);
            assert_eq!(further.domain, DomainCoverage::Partial);
        }
        return;
    }
    let original = d(1, 2, 0).tan(-64, &mut budget()).unwrap();
    assert_eq!(original.domain, DomainCoverage::Partial);
    assert!(
        std::process::Command::new(replay_binary::path().unwrap())
            .arg("--exact")
            .arg("replay_then_another_pole_operation_in_fresh_process")
            .env(ENV, serde_json::to_string(&original.range).unwrap())
            .status()
            .unwrap()
            .success()
    );
}

#[cfg(feature = "serde")]
#[path = "support/replay_binary.rs"]
mod replay_binary;
