#![cfg(feature = "dyadic")]
#[cfg(not(target_env = "msvc"))]
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{DyadicBudget, DyadicInterval};
fn budget() -> DyadicBudget {
    DyadicBudget::new(32768, 10_000_000_000_000)
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
#[test]
fn extrema_are_not_endpoint_only() {
    assert_eq!(
        d(0, 7, 0).sin(-64, &mut budget()).unwrap().range,
        d(-1, 1, 0)
    );
    assert_eq!(
        d(0, 7, 0).cos(-64, &mut budget()).unwrap().range,
        d(-1, 1, 0)
    );
}
#[cfg(not(target_env = "msvc"))]
#[path = "support/trig_oracle.rs"]
mod oracle;
#[cfg(not(target_env = "msvc"))]
#[test]
fn periodic_extrema_and_exact_sources_match_mpfr() {
    for e in [-8, -1, 0, 4] {
        for a in -6..=6 {
            for b in a..=6 {
                for p in [-64, -8, 0, 4] {
                    let source = d(a, b, e);
                    for (result, cosine) in [
                        (source.sin(p, &mut budget()).unwrap(), false),
                        (source.cos(p, &mut budget()).unwrap(), true),
                    ] {
                        assert_eq!(result.domain, hyperinterval::DomainCoverage::Complete);
                        oracle::verify(
                            &result.range,
                            &oracle::q(&a, e),
                            &oracle::q(&b, e),
                            p,
                            cosine,
                        );
                    }
                }
            }
        }
    }
    for (a, b) in [(1, 2), (3, 4), (-1, 1), (-4, -3), (-2, -1), (6, 7)] {
        let source = d(a, b, 0);
        for (result, cosine) in [
            (source.sin(-128, &mut budget()).unwrap(), false),
            (source.cos(-128, &mut budget()).unwrap(), true),
        ] {
            oracle::verify(&result.range, &a.into(), &b.into(), -128, cosine);
        }
    }
}
#[cfg(all(feature = "rational", not(target_env = "msvc")))]
#[test]
fn rational_near_periodic_boundaries_and_huge_phases() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational as R;
    use rug::{Integer, Rational as Q};
    let verify = |a: Q, b: Q, p| {
        let source = I::new(
            a.to_string().parse::<R>().unwrap(),
            b.to_string().parse::<R>().unwrap(),
        )
        .unwrap();
        for (result, cosine) in [
            (source.sin(p, &mut budget()).unwrap(), false),
            (source.cos(p, &mut budget()).unwrap(), true),
        ] {
            oracle::verify(&result.range, &a, &b, p, cosine);
        }
    };
    for denominator in [3, 7, 65537] {
        for a in -6..=6 {
            verify(
                Q::from((a, denominator)),
                Q::from((a + 2, denominator + 1)),
                -64,
            );
        }
    }
    let (pi, _) = oracle::half_pi();
    let scale = Integer::from(1) << 256u32;
    for k in -8i32..=8 {
        let x = (pi.clone() * k * &scale).floor().numer().clone();
        let a = Q::from((x.clone() - 1, scale.clone()));
        let b = Q::from((x + 2, scale.clone()));
        verify(a.clone(), a.clone(), -128);
        verify(b.clone(), b.clone(), -128);
        // One endpoint is well inside the quadrant while the other must
        // distinguish the two sides of an irrational quarter turn.
        if matches!(k, 1 | 2) {
            verify(Q::from((1, 4)), a.clone(), -128);
            verify(Q::from((1, 4)), b.clone(), -128);
        }
        verify(a, b, -128);
    }
    for e in [128u32, 512] {
        let huge = Q::from(Integer::from(1) << e);
        verify(huge.clone(), huge.clone(), -128);
        verify(huge.clone(), huge + Q::from((1, 1024)), -128);
    }
}
#[cfg(not(target_env = "msvc"))]
#[test]
fn tiny_huge_and_high_precision_inputs() {
    for (a, e, p) in [
        (1, -1074, -64),
        (-1, -1074, -64),
        (1, -512, -1024),
        (1, 128, -64),
        (1, 512, -128),
        (1, 0, -1024),
    ] {
        let source = d(a, a, e);
        for (result, cosine) in [
            (source.sin(p, &mut budget()).unwrap(), false),
            (source.cos(p, &mut budget()).unwrap(), true),
        ] {
            oracle::verify(
                &result.range,
                &oracle::q(&a, e),
                &oracle::q(&a, e),
                p,
                cosine,
            );
        }
    }
    for a in [-1, 1] {
        let source = d(a, a, i32::MIN);
        // 0 < 1-cos(x) < x*x/2 < 2^-64 for these nonzero tiny sources.
        let one: num_bigint::BigInt = num_bigint::BigInt::from(1) << 64usize;
        assert_eq!(
            source.cos(-64, &mut budget()).unwrap().range,
            DyadicInterval::new(&one - 1, one, -64).unwrap()
        );
        let result = source.sin(-64, &mut budget()).unwrap();
        assert_eq!(
            result.range,
            if a > 0 { d(0, 1, -64) } else { d(-1, 0, -64) }
        );
    }
}
#[test]
fn domain_replay_refinement_and_resources() {
    use hyperinterval::{DomainCoverage, ExtendedInterval as E, IntervalError};
    let empty = DyadicInterval::empty();
    assert!(
        empty
            .sin(i32::MIN, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .range
            .is_empty()
    );
    assert!(
        empty
            .cos(i32::MIN, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .range
            .is_empty()
    );
    assert_eq!(
        d(0, 0, 0).sin(i32::MIN, &mut budget()).unwrap().range,
        d(0, 0, 0)
    );
    assert_eq!(
        d(0, 0, 0).cos(i32::MIN, &mut budget()).unwrap().range,
        d(1, 1, 0)
    );
    for p in [i32::MIN, -64, 0, 4, i32::MAX] {
        let expected = d(-1, 1, p.max(0));
        assert_eq!(d(0, 8, 0).sin(p, &mut budget()).unwrap().range, expected);
        assert_eq!(
            d(-1, 1, i32::MAX).cos(p, &mut budget()).unwrap().range,
            expected
        );
        for unbounded in [
            E::<DyadicInterval>::entire(),
            E::above(d(0, 0, 0)).unwrap(),
            E::below(d(1, 1, 0)).unwrap(),
        ] {
            assert_eq!(unbounded.sin(p, &mut budget()).unwrap().range, expected);
            assert_eq!(unbounded.cos(p, &mut budget()).unwrap().range, expected);
        }
    }
    let x = d(1, 2, -3);
    assert_eq!(
        x.sin(-64, &mut DyadicBudget::new(4096, 0)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        x.cos(-128, &mut DyadicBudget::new(16, 1_000_000)),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        d(1, 1, i32::MAX).sin(-64, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    let fine = x.sin(-64, &mut budget()).unwrap();
    assert!(
        fine.range
            .subset_of(&x.sin(-16, &mut budget()).unwrap().range, &mut budget())
            .unwrap()
    );
    assert_eq!(
        x.neg(&mut budget())
            .unwrap()
            .sin(-64, &mut budget())
            .unwrap()
            .range,
        fine.range.neg(&mut budget()).unwrap()
    );
    assert_eq!(
        x.neg(&mut budget())
            .unwrap()
            .cos(-64, &mut budget())
            .unwrap(),
        x.cos(-64, &mut budget()).unwrap()
    );
    assert!(!fine.range.cos(-64, &mut budget()).unwrap().range.is_empty());
    let partial = d(-1, 4, 0).log(-64, &mut budget()).unwrap();
    assert_eq!(partial.domain, DomainCoverage::Partial);
    let next = partial.range.sin(-64, &mut budget()).unwrap();
    assert_eq!(next.domain, DomainCoverage::Complete);
    assert_eq!(next.range, d(-1, 1, 0));
    #[cfg(feature = "serde")]
    {
        let bytes = serde_json::to_vec(&fine.range).unwrap();
        let restored: DyadicInterval = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(restored, fine.range);
        assert_eq!(serde_json::to_vec(&restored).unwrap(), bytes);
        assert_eq!(
            restored.cos(-64, &mut budget()).unwrap(),
            fine.range.cos(-64, &mut budget()).unwrap()
        );
    }
}

#[cfg(not(target_env = "msvc"))]
#[test]
fn near_dyadic_output_boundaries_refine_and_retry_without_publishing_partial_bounds() {
    use hyperinterval::{DomainCoverage, IntervalError};
    use rug::Integer;

    // Directed MPFR brackets pi/6 and pi/3 independently of the native
    // reduction/series. Adjacent dyadic sources straddle sin(x)=1/2 and
    // cos(x)=1/2, far from extrema or poles but very close to an output grid.
    let half_pi = oracle::half_pi();
    for cosine in [false, true] {
        let multiplier = if cosine { 2_i32 } else { 1_i32 };
        let lower = half_pi.0.clone() * multiplier / 3_i32;
        let upper = half_pi.1.clone() * multiplier / 3_i32;
        for bits in [64_i32, 256, 512] {
            let scale = Integer::from(1) << bits as u32;
            let floor = (lower.clone() * &scale).floor().numer().clone();
            assert_eq!(floor, (upper.clone() * &scale).floor().numer().clone());
            for offset in [0_i32, 1] {
                let mantissa = floor.clone() + offset;
                let source =
                    DyadicInterval::point(mantissa.to_string().parse().unwrap(), -bits).unwrap();
                let evaluate = |x: &DyadicInterval, work: &mut DyadicBudget| {
                    if cosine {
                        x.cos(-8, work)
                    } else {
                        x.sin(-8, work)
                    }
                };
                let span = d(0, 0, 0).hull(&source, &mut budget()).unwrap();
                let point = oracle::q(&mantissa, -bits);
                let (lo, hi) = if cosine == (offset == 0) {
                    (128, 129)
                } else {
                    (127, 128)
                };
                // Both functions are monotone here. At zero, sine is zero
                // and cosine is one (256 units on the requested grid).
                let span_image = if cosine { d(lo, 256, -8) } else { d(0, hi, -8) };
                for (input, left, expected) in [
                    (&source, &point, d(lo, hi, -8)),
                    (&span, &0.into(), span_image),
                ] {
                    let mut work = budget();
                    let available = work.remaining_work();
                    let result = evaluate(input, &mut work).unwrap();
                    assert_eq!(result.domain, DomainCoverage::Complete);
                    oracle::verify(&result.range, left, &point, -8, cosine);

                    assert_eq!(result.range, expected);

                    let consumed = available - work.remaining_work();
                    assert!(consumed > 0);
                    assert_eq!(
                        evaluate(input, &mut DyadicBudget::new(32768, consumed - 1)),
                        Err(IntervalError::WorkExhausted)
                    );
                    let mut retry = DyadicBudget::new(32768, consumed);
                    assert_eq!(evaluate(input, &mut retry).unwrap(), result);
                    assert_eq!(retry.remaining_work(), 0);
                    assert!(!result.range.square(&mut budget()).unwrap().is_empty());
                }
            }
        }
    }
}
