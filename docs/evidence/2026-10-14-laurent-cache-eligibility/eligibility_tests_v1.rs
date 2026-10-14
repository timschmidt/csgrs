use super::*;

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

fn ratio(value: i64) -> Computable {
    raw(Approximation::Ratio(Rational::from(value)))
}

fn cases() -> [(Computable, Rational); 8] {
    [
        (
            raw(Approximation::Inverse(ratio(2))),
            Rational::fraction(1, 2).unwrap(),
        ),
        (raw(Approximation::Negate(ratio(3))), Rational::from(-3)),
        (
            raw(Approximation::Add(ratio(3), ratio(5))),
            Rational::from(8),
        ),
        (
            raw(Approximation::Multiply(ratio(3), ratio(5))),
            Rational::from(15),
        ),
        (
            raw(Approximation::LinearCombination3(Box::new(
                LinearCombination3 {
                    coefficients: [ratio(2), ratio(3), ratio(5)],
                    values: [Rational::from(7), Rational::from(11), Rational::from(13)],
                },
            ))),
            Rational::from(112),
        ),
        (raw(Approximation::Square(ratio(7))), Rational::from(49)),
        (
            raw(Approximation::Offset(ratio(5), -2)),
            Rational::fraction(5, 4).unwrap(),
        ),
        (raw(Approximation::Sqrt(ratio(9))), Rational::from(3)),
    ]
}

fn retained_rational(
    value: &Computable,
    depth: usize,
    work: usize,
) -> (Option<Rational>, usize, bool) {
    let mut remaining = work;
    let mut reused = false;
    let polynomial =
        value.pi_laurent_polynomial_retained::<true>(depth, &mut remaining, &mut None, &mut reused);
    let exact = polynomial.and_then(|polynomial| match polynomial.terms.as_slice() {
        [] => Some(Rational::zero()),
        [(0, coefficient)] if coefficient.radicals.is_empty() && coefficient.atoms.is_empty() => {
            Some(coefficient.rational.clone())
        }
        _ => None,
    });
    (exact, remaining, reused)
}

#[test]
fn zero_budget_proofs_survive_dropping_other_owners_for_all_publishing_kinds() {
    for (value, expected) in cases() {
        let weak = Arc::downgrade(&value.internal);
        assert_eq!(retained_rational(&value, 0, 0), (None, 0, false));
        let clone = value.clone();
        assert_eq!(retained_rational(&value, 64, 512).0, Some(expected.clone()));
        assert!(value.internal.cache.pi_laurent_polynomial().is_some());
        drop(clone);
        assert_eq!(Arc::strong_count(&value.internal), 1);
        assert_eq!(
            retained_rational(&value, 0, 0),
            (Some(expected.clone()), 0, true)
        );
        assert_eq!(value.bounded_laurent_rational(0), Some(expected));
        assert!(value.cached().is_none());
        drop(value);
        assert!(weak.upgrade().is_none());
    }
}

#[test]
fn leaf_and_opaque_queries_preserve_other_cache_payloads_and_answers() {
    for (value, expected) in [
        (raw(Approximation::Int(BigInt::from(7))), Rational::from(7)),
        (raw(Approximation::One), Rational::one()),
        (
            raw(Approximation::Ratio(Rational::fraction(-7, 3).unwrap())),
            Rational::fraction(-7, 3).unwrap(),
        ),
    ] {
        // Integer/One normally bypass the approximation cache. Seed it with
        // their actual approximation so every case checks coexistence with
        // both a valid approximation and a valid quadratic-tower certificate.
        value.internal.cache.store(-32, value.approx(-32));
        value
            .internal
            .cache
            .store_quadratic_tower(&Tower::rational(expected.clone()));
        let cached = value.cached();
        assert!(cached.is_some());
        let clone = value.clone();
        for query in [&value, &clone] {
            assert_eq!(query.bounded_laurent_rational(2), Some(expected.clone()));
            assert_eq!(retained_rational(query, 0, 0), (None, 0, false));
            assert!(query.internal.cache.pi_laurent_polynomial().is_none());
            assert_eq!(query.cached(), cached);
            let tower = query.internal.cache.quadratic_tower().unwrap();
            assert_eq!(tower.even.rational, expected);
            assert!(tower.even.scale.is_zero() && tower.even.disc.is_none());
            assert!(tower.odd.is_zero() && tower.radicand.is_none());
        }
        let weak = Arc::downgrade(&value.internal);
        drop(clone);
        drop(value);
        assert!(weak.upgrade().is_none());
    }

    let value = raw(Approximation::AtanRational(
        Rational::fraction(3, 4).unwrap(),
    ));
    let _ = value.approx(-32);
    let cached = value.cached();
    assert!(cached.is_some());
    let clone = value.clone();
    for query in [&value, &clone] {
        assert!(query.bounded_laurent_rational(64).is_none());
        assert_eq!(retained_rational(query, 0, 0), (None, 0, false));
        assert!(query.internal.cache.pi_laurent_polynomial().is_none());
        assert_eq!(query.cached(), cached);
    }
    let weak = Arc::downgrade(&value.internal);
    drop(clone);
    drop(value);
    assert!(weak.upgrade().is_none());
}

#[cfg(feature = "serde")]
#[test]
fn serde_resets_proof_storage_and_preserves_node_shape_and_value() {
    for (value, expected) in cases() {
        let clone = value.clone();
        assert_eq!(value.bounded_laurent_rational(64), Some(expected.clone()));
        assert!(value.internal.cache.pi_laurent_polynomial().is_some());
        let bytes = serde_json::to_vec(&value).unwrap();
        let restored: Computable = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(
            core::mem::discriminant(&value.internal.approximation),
            core::mem::discriminant(&restored.internal.approximation)
        );
        assert!(restored.internal.cache.pi_laurent_polynomial().is_none());
        assert_eq!(retained_rational(&restored, 0, 0), (None, 0, false));
        let restored_clone = restored.clone();
        assert_eq!(
            restored.bounded_laurent_rational(64),
            Some(expected.clone())
        );
        drop(restored_clone);
        assert_eq!(Arc::strong_count(&restored.internal), 1);
        assert_eq!(
            retained_rational(&restored, 0, 0),
            (Some(expected.clone()), 0, true)
        );
        drop(clone);
        assert_eq!(retained_rational(&value, 0, 0), (Some(expected), 0, true));
    }
}
