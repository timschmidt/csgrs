#![cfg(feature = "dyadic")]
use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval as E, IntervalError,
    IntervalSet2,
};
use num_bigint::BigInt;
fn budget() -> DyadicBudget {
    DyadicBudget::new(32768, 10_000_000_000_000)
}
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
#[test]
fn cube_root_of_eight_is_exact() {
    let x = DyadicInterval::point(8.into(), 0).unwrap();
    let result = x
        .pow_rational(
            &1.into(),
            &3.into(),
            -64,
            &mut DyadicBudget::new(4096, 1_000_000_000),
        )
        .unwrap();
    assert_eq!(
        result.range.components()[0].as_finite(),
        Some(&DyadicInterval::point(2.into(), 0).unwrap())
    );
}

#[test]
fn zero_exponent_obeys_the_requested_coarse_grid() {
    let result = d(-3, 5, 0)
        .pow_rational(&0.into(), &7.into(), 4, &mut budget())
        .unwrap();
    assert_eq!(result.range.components(), &[E::from_finite(d(0, 1, 4))]);
}

#[test]
fn reduced_exponents_domains_poles_and_limits() {
    let split = d(-8, 8, 0)
        .pow_rational(&(-1).into(), &3.into(), -32, &mut budget())
        .unwrap();
    assert_eq!(split.domain, DomainCoverage::Partial);
    assert_eq!(
        split.range,
        IntervalSet2::new(
            E::below(d(-1, -1, -1)).unwrap(),
            E::above(d(1, 1, -1)).unwrap(),
            &mut budget()
        )
        .unwrap()
    );
    for (n, q) in [(2, 6), (-2, -6), (3, 9), (5, 15)] {
        let result = d(-8, 27, 0)
            .pow_rational(&n.into(), &q.into(), -32, &mut budget())
            .unwrap();
        assert_eq!(result.domain, DomainCoverage::Complete);
        assert_eq!(result.range.components(), &[E::from_finite(d(-2, 3, 0))]);
    }
    for (n, q) in [(1, 2), (2, 4), (-1, -2)] {
        let result = d(-8, 4, 0)
            .pow_rational(&n.into(), &q.into(), -32, &mut budget())
            .unwrap();
        assert_eq!(result.domain, DomainCoverage::Partial);
        assert_eq!(result.range.components(), &[E::from_finite(d(0, 2, 0))]);
        assert_eq!(
            d(-8, -1, 0).pow_rational(&n.into(), &q.into(), 0, &mut budget()),
            Err(IntervalError::EmptyDomain)
        );
    }
    assert_eq!(
        d(-8, 27, 0)
            .pow_rational(&2.into(), &3.into(), -32, &mut budget())
            .unwrap()
            .range
            .components(),
        &[E::from_finite(d(0, 9, 0))]
    );
    assert_eq!(
        d(0, 0, 0).pow_rational(&(-1).into(), &3.into(), 0, &mut budget()),
        Err(IntervalError::EmptyDomain)
    );
    assert!(
        DyadicInterval::empty()
            .pow_rational(&1.into(), &3.into(), i32::MIN, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .range
            .is_empty()
    );
    assert_eq!(
        DyadicInterval::empty().pow_rational(&0.into(), &0.into(), 0, &mut DyadicBudget::new(0, 0)),
        Err(IntervalError::ContainsZero)
    );
    assert_eq!(
        d(0, 0, 0)
            .pow_rational(&0.into(), &7.into(), i32::MIN, &mut budget())
            .unwrap()
            .range
            .components(),
        &[E::from_finite(d(1, 1, 0))]
    );
    let entire = E::<DyadicInterval>::entire()
        .pow_rational(&(-2).into(), &3.into(), -32, &mut budget())
        .unwrap();
    assert_eq!(entire.domain, DomainCoverage::Partial);
    assert_eq!(entire.range.components(), &[E::above(d(0, 0, 0)).unwrap()]);
}

#[test]
fn compact_exact_roots_extreme_scales_and_arbitrary_exponents() {
    for e in [i32::MIN, -1074, 0, i32::MAX - 1] {
        let source = d(1, 1, e);
        let result = source
            .pow_rational(&1.into(), &2.into(), i32::MIN, &mut budget())
            .unwrap();
        assert_eq!(result.range.components(), &[E::from_finite(d(1, 1, e / 2))]);
    }
    let huge: BigInt = BigInt::from(1) << 512usize;
    for n in [huge.clone(), &huge + 1, -&huge - 1] {
        let result = d(1, 1, 0)
            .pow_rational(&n, &(&huge + 3), i32::MIN, &mut budget())
            .unwrap();
        assert_eq!(result.range.components(), &[E::from_finite(d(1, 1, 0))]);
    }
    let result = d(2, 2, 0)
        .pow_rational(&1.into(), &(&huge + 1), 0, &mut budget())
        .unwrap();
    assert_eq!(result.range.components(), &[E::from_finite(d(1, 2, 0))]);
    let result = d(1, 1, i32::MIN)
        .pow_rational(&3.into(), &1.into(), -64, &mut budget())
        .unwrap();
    assert_eq!(result.range.components(), &[E::from_finite(d(0, 1, -64))]);
    let result = d(1, 1, i32::MIN)
        .pow_rational(&(-1).into(), &1.into(), i32::MAX, &mut budget())
        .unwrap();
    assert_eq!(
        result.range.components(),
        &[E::from_finite(d(2, 2, i32::MAX))]
    );
}

#[test]
fn shared_work_refusal_retry_and_continued_operations() {
    let source = d(-3, 5, 0);
    let completed = source
        .pow_rational(&(-3).into(), &5.into(), -64, &mut budget())
        .unwrap();
    assert_eq!(
        source.pow_rational(
            &(-3).into(),
            &5.into(),
            -64,
            &mut DyadicBudget::new(32768, 100)
        ),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        source.pow_rational(
            &(-3).into(),
            &5.into(),
            -64,
            &mut DyadicBudget::new(128, u64::MAX)
        ),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(
        source
            .pow_rational(&(-3).into(), &5.into(), -64, &mut budget())
            .unwrap(),
        completed
    );
    let exp = completed.range.exp(-32, &mut budget()).unwrap();
    assert_eq!(exp.domain, DomainCoverage::Complete);
    assert_eq!(completed.domain, DomainCoverage::Partial);
    assert_eq!(exp.range.components().len(), 2);
    for piece in completed.range.components() {
        let square = piece
            .pow_rational(&2.into(), &1.into(), -32, &mut budget())
            .unwrap();
        assert_eq!(square.domain, DomainCoverage::Complete);
        assert!(!square.range.signs().contains_negative());
    }
}

#[test]
fn extreme_requests_exhaust_declared_reservations() {
    let huge: BigInt = BigInt::from(1) << 512usize;
    for (n, q, p) in [
        (BigInt::from(1), &huge + 1, -1),
        (&huge + 1, BigInt::from(1), -64),
    ] {
        assert_eq!(
            d(2, 2, 0).pow_rational(&n, &q, p, &mut budget()),
            Err(IntervalError::SizeExhausted)
        );
    }
    assert_eq!(
        d(3, 3, 0).pow_rational(&1.into(), &3.into(), i32::MIN, &mut budget()),
        Err(IntervalError::SizeExhausted)
    );
    let reduced = d(-8, 27, 0)
        .pow_rational(&huge, &(&huge * 3), -64, &mut budget())
        .unwrap();
    assert_eq!(reduced.range.components(), &[E::from_finite(d(-2, 3, 0))]);
    let coarse = d(3, 3, i32::MAX)
        .pow_rational(&1.into(), &3.into(), i32::MAX, &mut budget())
        .unwrap();
    assert_eq!(
        coarse.range.components(),
        &[E::from_finite(d(0, 1, i32::MAX))]
    );
}

#[cfg(not(target_env = "msvc"))]
#[path = "support/power_oracle.rs"]
mod oracle;
#[cfg(not(target_env = "msvc"))]
#[test]
fn independent_gmp_roots_domains_and_mpfr_point_bounds() {
    use oracle::End::{Minus as M, Plus as P, Value as V};
    let mut cases = vec![(E::empty(), None), (E::entire(), Some((M, P)))];
    for n in [-8, -1, 0, 1, 8] {
        cases.push((E::below(d(n, n, 0)).unwrap(), Some((M, V(n.into())))));
        cases.push((E::above(d(n, n, 0)).unwrap(), Some((V(n.into()), P))));
        cases.push((E::from_finite(d(n, n, 0)), Some((V(n.into()), V(n.into())))));
    }
    for (a, b) in [(-8, -1), (-8, 0), (-8, 27), (-1, 8), (0, 8), (1, 8)] {
        cases.push((E::from_finite(d(a, b, 0)), Some((V(a.into()), V(b.into())))));
    }
    for (x, qx) in cases {
        for n in -5..=5 {
            for den in [-6, -3, 0, 1, 2, 3, 4, 5, 6] {
                for p in [-64, -8, 0, 4] {
                    oracle::verify(
                        x.pow_rational(&n.into(), &den.into(), p, &mut budget()),
                        &qx,
                        n,
                        den,
                        p,
                    );
                }
            }
        }
    }
    for e in [-1074, -8, 0, 8, 512] {
        for a in [-7, -2, 2, 7] {
            for (n, den) in [(-3, 5), (-2, 3), (1, 3), (3, 5)] {
                let x = d(a, a, e);
                let q = oracle::q(&a, e);
                let result = x
                    .pow_rational(&n.into(), &den.into(), -64, &mut budget())
                    .unwrap();
                oracle::verify(
                    Ok(result.clone()),
                    &Some((V(q.clone()), V(q.clone()))),
                    n,
                    den,
                    -64,
                );
                oracle::mpfr_point(&result.range, &q, n, den as u32);
            }
        }
    }
}

#[cfg(all(feature = "rational", not(target_env = "msvc")))]
#[test]
fn rational_endpoints_retain_exact_source_and_perfect_roots() {
    use hyperinterval::RationalInterval as I;
    use hyperreal::Rational as R;
    use oracle::End::Value as V;
    use rug::Rational as Q;
    for denominator in [1, 3, 7, 65537] {
        for a in [-9, -1, 0, 1, 9] {
            for (n, q) in [(-3, 5), (-1, 3), (1, 2), (2, 3), (3, 2)] {
                let x = I::new(
                    R::fraction(a, denominator).unwrap(),
                    R::fraction(a + 5, denominator).unwrap(),
                )
                .unwrap();
                for p in [-128, -8, 4] {
                    oracle::verify(
                        x.pow_rational(&n.into(), &q.into(), p, &mut budget()),
                        &Some((
                            V(Q::from((a, denominator))),
                            V(Q::from((a + 5, denominator))),
                        )),
                        n,
                        q,
                        p,
                    );
                }
            }
        }
    }
    let x = I::point(R::fraction(8, 27).unwrap());
    let result = x
        .pow_rational(&1.into(), &3.into(), -64, &mut budget())
        .unwrap();
    oracle::verify(
        Ok(result),
        &Some((V(Q::from((8, 27))), V(Q::from((8, 27))))),
        1,
        3,
        -64,
    );
    let x = I::point(R::fraction(27, 8).unwrap());
    assert_eq!(
        x.pow_rational(&1.into(), &3.into(), i32::MIN, &mut budget())
            .unwrap()
            .range
            .components(),
        &[E::from_finite(d(3, 3, -1))]
    );
    let large = rug::Integer::from(1) << 1024u32;
    for (a, b) in [
        (large.clone() + 1, large.clone() - 1),
        (large.clone() - 1, large + 1),
    ] {
        let q = Q::from((a, b));
        let x = I::point(q.to_string().parse::<R>().unwrap());
        let image = x
            .pow_rational(&1.into(), &7.into(), -64, &mut budget())
            .unwrap();
        oracle::verify(Ok(image.clone()), &Some((V(q.clone()), V(q))), 1, 7, -64);
        assert_eq!(
            x.pow_rational(
                &1.into(),
                &7.into(),
                -64,
                &mut DyadicBudget::new(32768, 10000)
            ),
            Err(IntervalError::WorkExhausted)
        );
        assert_eq!(
            x.pow_rational(&1.into(), &7.into(), -64, &mut budget())
                .unwrap(),
            image
        );
    }
}

#[cfg(feature = "serde")]
#[test]
fn exact_replay_then_further_power_in_fresh_process() {
    const ENV: &str = "HYPERINTERVAL_POWER_REPLAY";
    if let Ok(bytes) = std::env::var(ENV) {
        let set: IntervalSet2<DyadicInterval> = serde_json::from_str(&bytes).unwrap();
        assert_eq!(serde_json::to_string(&set).unwrap(), bytes);
        assert!(!set.contains_point(&0.into(), 0, &mut budget()).unwrap());
        for piece in set.components() {
            let result = piece
                .pow_rational(&(-3).into(), &1.into(), -32, &mut budget())
                .unwrap();
            assert_eq!(result.domain, DomainCoverage::Complete);
            assert_eq!(result.range.components().len(), 1);
            assert!(result.range.components()[0].as_finite().is_some());
        }
        return;
    }
    let image = d(-8, 8, 0)
        .pow_rational(&(-1).into(), &3.into(), -64, &mut budget())
        .unwrap();
    assert_eq!(image.domain, DomainCoverage::Partial);
    let bytes = serde_json::to_string(&image.range).unwrap();
    assert!(
        std::process::Command::new(std::env::current_exe().unwrap())
            .arg("--exact")
            .arg("exact_replay_then_further_power_in_fresh_process")
            .env(ENV, bytes)
            .status()
            .unwrap()
            .success()
    );
}
