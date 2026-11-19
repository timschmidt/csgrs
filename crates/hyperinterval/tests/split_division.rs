#![cfg(feature = "dyadic")]
use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval as E, IntervalSet2,
};
fn budget() -> DyadicBudget {
    DyadicBudget::new(4096, 10_000_000_000)
}
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
#[test]
fn reciprocal_retains_both_unbounded_branches() {
    let image = E::from_finite(d(-1, 1))
        .reciprocal(-64, &mut budget())
        .unwrap();
    assert_eq!(image.domain, DomainCoverage::Partial);
    let expected = IntervalSet2::new(
        E::below(d(-1, -1)).unwrap(),
        E::above(d(1, 1)).unwrap(),
        &mut budget(),
    )
    .unwrap();
    assert_eq!(image.range, expected);
}
#[cfg(not(target_env = "msvc"))]
#[path = "support/division_oracle.rs"]
mod oracle;
#[cfg(not(target_env = "msvc"))]
#[test]
fn exact_four_corner_oracle_over_finite_and_unbounded_sources() {
    use oracle::End::{Minus as M, Plus as P, Value as V};
    let finite = |a, b| Some((V(a), V(b)));
    let mut cases = vec![(E::empty(), None), (E::entire(), Some((M, P)))];
    for n in [-4i64, -1, 0, 1, 4] {
        cases.push((E::below(d(n, n)).unwrap(), Some((M, V(n.into())))));
        cases.push((E::above(d(n, n)).unwrap(), Some((V(n.into()), P))));
        cases.push((E::from_finite(d(n, n)), finite(n.into(), n.into())));
    }
    for (a, b) in [(-4, -1), (-4, 0), (-4, 1), (-1, 4), (0, 4), (1, 4)] {
        cases.push((E::from_finite(d(a, b)), finite(a.into(), b.into())));
    }
    for (a, qa) in &cases {
        for (b, qb) in &cases {
            for p in [-8, 0, 4] {
                oracle::verify(a.div(b, p, &mut budget()), qa, qb, p);
            }
        }
    }
}
#[test]
fn normalized_components_preserve_gaps_and_order() {
    use hyperinterval::SignSet;
    let left = E::from_finite(d(-3, -1));
    let right = E::from_finite(d(1, 3));
    let set = IntervalSet2::new(right.clone(), left.clone(), &mut budget()).unwrap();
    assert_eq!(set.components(), &[left.clone(), right.clone()]);
    assert_eq!(set.signs(), SignSet::Nonzero);
    assert!(!set.is_empty());
    assert!(!set.is_singleton());
    assert!(!set.contains_point(&0.into(), 0, &mut budget()).unwrap());
    assert!(set.contains_point(&2.into(), 0, &mut budget()).unwrap());
    assert_eq!(set.neg(&mut budget()).unwrap(), set);
    let connected = IntervalSet2::from_interval(E::from_finite(d(-3, 3)));
    assert!(set.subset_of(&connected, &mut budget()).unwrap());
    assert!(!connected.subset_of(&set, &mut budget()).unwrap());
    assert_eq!(set.hull(&mut budget()).unwrap(), E::from_finite(d(-3, 3)));
    assert_eq!(
        IntervalSet2::new(left.clone(), E::from_finite(d(-1, 3)), &mut budget()).unwrap(),
        connected
    );
    assert_eq!(
        IntervalSet2::new(left.clone(), left.clone(), &mut budget()).unwrap(),
        IntervalSet2::from_interval(left)
    );
    assert!(
        IntervalSet2::<DyadicInterval>::empty()
            .subset_of(&set, &mut budget())
            .unwrap()
    );
    assert!(
        !set.subset_of(&IntervalSet2::empty(), &mut budget())
            .unwrap()
    );
    assert_eq!(
        IntervalSet2::new(
            E::<DyadicInterval>::empty(),
            E::empty(),
            &mut DyadicBudget::new(0, 0)
        )
        .unwrap(),
        IntervalSet2::empty()
    );
    let one = IntervalSet2::from_interval(E::from_finite(d(1, 1)));
    assert!(one.is_singleton());
    let gap = DyadicInterval::point(1.into(), -1024).unwrap();
    let tiny = IntervalSet2::new(
        E::below(d(0, 0)).unwrap(),
        E::above(gap).unwrap(),
        &mut budget(),
    )
    .unwrap();
    assert_eq!(tiny.components().len(), 2);
    assert!(
        !tiny
            .contains_point(&1.into(), -1025, &mut budget())
            .unwrap()
    );
}
#[test]
fn limits_domains_retry_and_continued_operations() {
    use hyperinterval::{IntervalError, SignSet};
    let zero = E::from_finite(d(0, 0));
    let empty = E::<DyadicInterval>::empty();
    assert_eq!(
        zero.reciprocal(0, &mut DyadicBudget::new(0, 0)),
        Err(IntervalError::EmptyDomain)
    );
    assert_eq!(
        empty.div(&zero, 0, &mut DyadicBudget::new(0, 0)),
        Err(IntervalError::EmptyDomain)
    );
    assert!(
        empty
            .reciprocal(i32::MIN, &mut DyadicBudget::new(0, 0))
            .unwrap()
            .range
            .is_empty()
    );
    let crossing = E::from_finite(d(-1, 1));
    let image = crossing.reciprocal(-64, &mut budget()).unwrap();
    assert_eq!(image.range.signs(), SignSet::Nonzero);
    let round = crossing.reciprocal(4, &mut budget()).unwrap();
    assert_eq!(round.range.components(), &[E::entire()]);
    assert_eq!(round.domain, DomainCoverage::Partial);
    assert_eq!(
        crossing.reciprocal(-64, &mut DyadicBudget::new(4096, 0)),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(
        E::from_finite(d(3, 3)).reciprocal(-128, &mut DyadicBudget::new(16, 1_000_000)),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(crossing.reciprocal(-64, &mut budget()).unwrap(), image);
    let next = image.range.exp(-64, &mut budget()).unwrap();
    assert_eq!(next.domain, DomainCoverage::Complete);
    assert_eq!(next.range.components().len(), 2);
    assert_eq!(image.domain, DomainCoverage::Partial);
    assert!(
        !next
            .range
            .contains_point(&1.into(), 0, &mut budget())
            .unwrap()
    );
    let huge = E::from_finite(DyadicInterval::point(1.into(), i32::MAX).unwrap());
    let tiny = huge.reciprocal(-64, &mut budget()).unwrap();
    assert_eq!(
        tiny.range,
        IntervalSet2::from_interval(E::from_finite(
            DyadicInterval::new(0.into(), 1.into(), -64).unwrap()
        ))
    );
    let unit = E::from_finite(d(1, 1))
        .reciprocal(i32::MIN, &mut budget())
        .unwrap();
    assert_eq!(
        unit.range,
        IntervalSet2::from_interval(E::from_finite(d(1, 1)))
    );
    let coarse = E::from_finite(DyadicInterval::point(1.into(), i32::MIN).unwrap())
        .reciprocal(i32::MAX, &mut budget())
        .unwrap();
    assert_eq!(
        coarse.range,
        IntervalSet2::from_interval(E::from_finite(
            DyadicInterval::point(2.into(), i32::MAX).unwrap()
        ))
    );
    // Exact large-coefficient ratio below one: its distance from one is
    // 2/(2^1024+1), strictly between zero and the requested 2^-64 grid unit.
    let base: num_bigint::BigInt = num_bigint::BigInt::from(1) << 1024usize;
    let a = E::from_finite(DyadicInterval::point(&base - 1, 0).unwrap());
    let b = E::from_finite(DyadicInterval::point(&base + 1, 0).unwrap());
    assert_eq!(
        a.div(&b, -64, &mut DyadicBudget::new(4096, 10000)),
        Err(IntervalError::WorkExhausted)
    );
    let one: num_bigint::BigInt = num_bigint::BigInt::from(1) << 64usize;
    assert_eq!(
        a.div(&b, -64, &mut budget()).unwrap().range,
        IntervalSet2::from_interval(E::from_finite(
            DyadicInterval::new(&one - 1, one, -64).unwrap()
        ))
    );
    assert_eq!(
        d(1, 1).div(&d(-1, 1), -64, &mut budget()),
        Err(IntervalError::ContainsZero)
    );
}
#[cfg(feature = "serde")]
#[test]
fn replay_validates_canonical_components() {
    type Set = IntervalSet2<DyadicInterval>;
    let left = E::below(d(-1, -1)).unwrap();
    let right = E::above(d(1, 1)).unwrap();
    let split = Set::new(left.clone(), right.clone(), &mut budget()).unwrap();
    for set in [
        Set::empty(),
        Set::from_interval(E::entire()),
        Set::from_interval(E::from_finite(d(1, 1))),
        split.clone(),
    ] {
        let bytes = serde_json::to_vec(&set).unwrap();
        let restored: Set = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(restored, set);
        assert_eq!(serde_json::to_vec(&restored).unwrap(), bytes);
        assert_eq!(
            restored.exp(-32, &mut budget()).unwrap(),
            set.exp(-32, &mut budget()).unwrap()
        );
    }
    for pair in [
        [right, left.clone()],
        [left.clone(), left],
        [E::below(d(0, 0)).unwrap(), E::above(d(0, 0)).unwrap()],
        [E::empty(), E::entire()],
    ] {
        let bytes = format!("{{\"Two\":{}}}", serde_json::to_string(&pair).unwrap());
        assert!(serde_json::from_str::<Set>(&bytes).is_err());
    }
    assert_eq!(
        serde_json::from_str::<Set>("{\"One\":\"Empty\"}").unwrap(),
        Set::empty()
    );
}

#[cfg(feature = "serde")]
#[test]
fn replay_in_fresh_process() {
    const PAYLOAD: &str = "HYPERINTERVAL_SPLIT_REPLAY_PAYLOAD";
    if let Ok(payload) = std::env::var(PAYLOAD) {
        let restored: IntervalSet2<DyadicInterval> = serde_json::from_str(&payload).unwrap();
        assert_eq!(
            restored.components(),
            &[E::below(d(-1, -1)).unwrap(), E::above(d(1, 1)).unwrap()]
        );
        assert!(
            !restored
                .contains_point(&0.into(), 0, &mut budget())
                .unwrap()
        );
        for n in [-1, 1] {
            assert!(
                restored
                    .contains_point(&n.into(), 0, &mut budget())
                    .unwrap()
            );
        }
        assert_eq!(serde_json::to_string(&restored).unwrap(), payload);
        assert_eq!(restored.neg(&mut budget()).unwrap(), restored);
        assert!(
            !restored
                .exp(-64, &mut budget())
                .unwrap()
                .range
                .contains_point(&1.into(), 0, &mut budget())
                .unwrap()
        );
        return;
    }
    let image = E::from_finite(d(-1, 1))
        .reciprocal(-64, &mut budget())
        .unwrap();
    let child = std::process::Command::new(replay_binary::path().unwrap())
        .args(["--exact", "replay_in_fresh_process", "--nocapture"])
        .env(PAYLOAD, serde_json::to_string(&image.range).unwrap())
        .status()
        .unwrap();
    assert!(child.success());
}

#[test]
fn exact_quotients_do_not_expand_an_irrelevant_output_grid() {
    for (a, b, c, lo, hi, e) in [(1, 2, 1, 1, 2, 0), (6, 9, 3, 2, 3, 0), (1, 3, 2, 1, 3, -1)] {
        let image = E::from_finite(d(a, b))
            .div(&E::from_finite(d(c, c)), i32::MIN, &mut budget())
            .unwrap();
        assert_eq!(
            image.range,
            IntervalSet2::from_interval(E::from_finite(
                DyadicInterval::new(lo.into(), hi.into(), e).unwrap()
            ))
        );
    }
}

#[cfg(feature = "serde")]
#[path = "support/replay_binary.rs"]
mod replay_binary;
