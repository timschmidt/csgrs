use super::*;
use crate::memo_leaf_trace::{Counts, Event, with_recording};

fn raw(approximation: Approximation) -> Computable {
    Computable {
        internal: Arc::new(Node::new(
            approximation,
            BoundCache::Invalid,
            ExactSignCache::Unknown,
        )),
        signal: None,
    }
}

#[test]
fn diagnostic_memo_buckets_and_payload_metadata_are_independent_of_owners() {
    let mut kinds = vec![
        (0, raw(Approximation::Int(BigInt::from(7)))),
        (1, raw(Approximation::One)),
        (2, raw(Approximation::Ratio(Rational::new(7)))),
        (
            9,
            raw(Approximation::AtanRational(
                Rational::fraction(3, 7).unwrap(),
            )),
        ),
    ];
    for (constant, bucket) in [
        (SharedConstant::E, 8),
        (SharedConstant::Pi, 3),
        (SharedConstant::InvPi, 4),
        (SharedConstant::Tau, 5),
        (SharedConstant::Ln2, 8),
        (SharedConstant::Ln3, 8),
        (SharedConstant::Ln5, 8),
        (SharedConstant::Ln6, 8),
        (SharedConstant::Ln7, 8),
        (SharedConstant::Ln10, 8),
        (SharedConstant::Sqrt2, 6),
        (SharedConstant::Sqrt3, 7),
        (SharedConstant::Acosh2, 8),
        (SharedConstant::Asinh1, 8),
        (SharedConstant::AtanInv2, 8),
        (SharedConstant::AtanInv5, 8),
        (SharedConstant::Atan2, 8),
        (SharedConstant::AtanThreeHalves, 8),
    ] {
        kinds.push((bucket, raw(Approximation::Constant(constant))));
    }
    let owner = raw(Approximation::AtanRational(
        Rational::fraction(2, 7).unwrap(),
    ));
    let weak = Arc::downgrade(&owner.internal);
    let values = [
        None,
        Some(PiLaurentPolynomial { terms: vec![] }),
        Some(PiLaurentPolynomial::monomial(
            3,
            RadicalCoefficient::rational(Rational::new(7)),
        )),
        Some(PiLaurentPolynomial::monomial(
            -2,
            RadicalCoefficient::radical(Rational::one(), Rational::new(2)),
        )),
        Some(PiLaurentPolynomial {
            terms: vec![
                (0, RadicalCoefficient::rational(Rational::one())),
                (1, RadicalCoefficient::rational(Rational::new(3))),
            ],
        }),
        Some(PiLaurentPolynomial::monomial(
            0,
            RadicalCoefficient::atom(owner.clone()),
        )),
    ];
    let owners = Arc::strong_count(&owner.internal);
    let ((), counts) = with_recording(|| {
        for (_, value) in &kinds {
            for payload in &values {
                for event in [Event::Store, Event::Hit] {
                    record_memo_payload::<true>(&value.internal.approximation, payload, event);
                    record_memo_payload::<false>(&value.internal.approximation, payload, event);
                }
            }
        }
    });
    // Six independently built payloads: None, empty, one rational coefficient
    // at a nonzero pi power, one radical, two closed terms, and one opaque atom.
    let per_kind = [6_u64, 1, 1, 1, 1, 1, 1, 5, 1, 1];
    for (bucket, modes) in counts.iter().enumerate() {
        let multiplicity = if bucket == 8 { 13 } else { 1 };
        for events in modes {
            for counter in events {
                assert_eq!(*counter, per_kind.map(|n| n * multiplicity));
            }
        }
    }
    assert_eq!(Arc::strong_count(&owner.internal), owners);
    record_memo_payload::<true>(&owner.internal.approximation, &values[5], Event::Store);
    let ((), reset) = with_recording(|| {});
    assert_eq!(reset, [[[[0; 10]; 2]; 2]; 10]);
    drop(values);
    assert_eq!(Arc::strong_count(&owner.internal), 1);
    drop(owner);
    assert!(weak.upgrade().is_none());
}

fn run_site_control<const RETAIN: bool>() {
    use crate::construction_cache_trace::{Event as OldEvent, with_recording as old_recording};
    let wide = (BigInt::one() << 128_usize) + 1_u32;
    let values = [
        (0, raw(Approximation::Int(wide.clone()))),
        (1, raw(Approximation::One)),
        (
            2,
            raw(Approximation::Ratio(Rational::fraction(3, 7).unwrap())),
        ),
        (3, raw(Approximation::Constant(SharedConstant::Pi))),
        (4, raw(Approximation::Constant(SharedConstant::InvPi))),
        (5, raw(Approximation::Constant(SharedConstant::Tau))),
        (6, raw(Approximation::Constant(SharedConstant::Sqrt2))),
        (7, raw(Approximation::Constant(SharedConstant::Sqrt3))),
        (8, raw(Approximation::Constant(SharedConstant::E))),
        (
            9,
            raw(Approximation::AtanRational(
                Rational::fraction(3, 7).unwrap(),
            )),
        ),
    ];
    for (bucket, value) in values {
        let weak = Arc::downgrade(&value.internal);
        assert_eq!(Arc::strong_count(&value.internal), 1);
        let mut memo = None;
        let mut work = 512;
        let mut reused = false;
        let (single, counters) = with_recording(|| {
            value.pi_laurent_polynomial_retained::<RETAIN>(64, &mut work, &mut memo, &mut reused)
        });
        assert!(single.is_some());
        assert_eq!(work, 511);
        assert!(memo.is_none());
        assert_eq!(counters, [[[[0; 10]; 2]; 2]; 10]);
        drop(single);
        assert_eq!(Arc::strong_count(&value.internal), 1);
        let clone = value.clone();
        work = 512;
        let (((first, warm), old), counters) = with_recording(|| {
            old_recording(|| {
                let first = value
                    .pi_laurent_polynomial_retained::<RETAIN>(64, &mut work, &mut memo, &mut reused)
                    .unwrap();
                assert_eq!(work, 511);
                work = 0;
                let warm = clone
                    .pi_laurent_polynomial_retained::<RETAIN>(0, &mut work, &mut memo, &mut reused)
                    .unwrap();
                (first, warm)
            })
        });
        assert_eq!(work, 0);
        assert!(!reused);
        assert_eq!(memo.as_ref().unwrap().len(), 1);
        assert_eq!(
            memo.as_ref().unwrap()[0].0,
            Arc::as_ptr(&value.internal) as usize
        );
        assert_eq!(old[OldEvent::LocalMemoStore as usize], 1);
        assert_eq!(old[OldEvent::LocalMemoHit as usize], 1);
        assert_eq!(old[OldEvent::NormalizerWorkVisit as usize], 1);
        assert_eq!(old[OldEvent::NormalizerBudgetDecline as usize], 0);
        assert_eq!(
            old[OldEvent::RetainedLookup as usize],
            if RETAIN { 2 } else { 0 }
        );
        assert_eq!(old[OldEvent::StoreAttempt as usize], 0);
        assert_eq!(old[OldEvent::RetainedHit as usize], 0);
        let mut expected: Counts = [[[[0; 10]; 2]; 2]; 10];
        let shape = if bucket >= 8 {
            6
        } else if bucket >= 6 {
            4
        } else {
            3
        };
        for counter in &mut expected[bucket][usize::from(!RETAIN)] {
            counter[0] = 1;
            counter[shape] = 1;
            counter[7] = 1;
            counter[8] = u64::from(matches!(bucket, 6 | 7));
            counter[9] = u64::from(bucket >= 8);
        }
        assert_eq!(counters, expected);
        for proof in [&first, &warm] {
            assert_eq!(proof.terms.len(), 1);
            let (power, coefficient) = &proof.terms[0];
            assert_eq!(
                *power,
                match bucket {
                    3 | 5 => 1,
                    4 => -1,
                    _ => 0,
                }
            );
            match bucket {
                0 => assert_eq!(coefficient.rational.to_big_integer().as_ref(), Some(&wide)),
                1 | 3 | 4 => assert_eq!(coefficient.rational, Rational::one()),
                2 => assert_eq!(coefficient.rational, Rational::fraction(3, 7).unwrap()),
                5 => assert_eq!(coefficient.rational, Rational::new(2)),
                6 | 7 => {
                    assert_eq!(coefficient.rational, Rational::zero());
                    assert_eq!(
                        coefficient.radicals.as_slice(),
                        [(
                            Rational::new(if bucket == 6 { 2 } else { 3 }),
                            Rational::one()
                        )]
                    );
                }
                8 | 9 => {
                    assert_eq!(coefficient.rational, Rational::zero());
                    assert_eq!(coefficient.atoms.len(), 1);
                    assert!(Arc::ptr_eq(
                        &coefficient.atoms[0].0.internal,
                        &value.internal
                    ));
                    assert_eq!(coefficient.atoms[0].1, Rational::one());
                }
                _ => unreachable!(),
            }
        }
        assert!(value.cached().is_none());
        assert!(value.internal.cache.pi_laurent_polynomial().is_none());
        drop(first);
        drop(warm);
        drop(clone);
        assert_eq!(
            Arc::strong_count(&value.internal),
            if bucket >= 8 { 2 } else { 1 }
        );
        drop(memo);
        assert_eq!(Arc::strong_count(&value.internal), 1);
        drop(value);
        assert!(weak.upgrade().is_none());
    }
}

#[test]
fn diagnostic_memo_sites_preserve_leaf_replay_and_graph_lifetimes() {
    run_site_control::<true>();
    run_site_control::<false>();
}
