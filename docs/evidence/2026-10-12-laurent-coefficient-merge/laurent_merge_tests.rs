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

fn root(radicand: i64) -> Computable {
    raw(Approximation::Sqrt(Computable::rational(Rational::new(
        radicand,
    ))))
}

fn root_sum(radicands: &[i64], constant: i64) -> Computable {
    radicands.iter().fold(
        Computable::rational(Rational::new(constant)),
        |sum, &radicand| raw(Approximation::Add(sum, root(radicand))),
    )
}

#[test]
fn cancellation_order_preserves_the_eight_radical_frontier() {
    let primes = [2, 3, 5, 7, 11, 13, 17, 19];
    for reverse in [false, true] {
        let left = root_sum(&primes, 0);
        let remove = raw(Approximation::Negate(root(2)));
        let add = root(23);
        let right = if reverse {
            raw(Approximation::Add(add, remove))
        } else {
            raw(Approximation::Add(remove, add))
        };
        let value = raw(Approximation::Add(left, right));
        let clone = value.clone();
        for query in [&value, &clone, &value] {
            // Both values equal the same sum of eight independent roots.
            // The ordered bounded recognizer only admits the form when the
            // cancellation occurs before the ninth radical is inserted.
            let closed = query.project_pi_laurent_polynomial(64, |polynomial| {
                ClosedPiLaurentPolynomial::from_polynomial(&polynomial)
            });
            assert_eq!(closed.is_some(), !reverse);
            assert_eq!(
                query.internal.cache.pi_laurent_polynomial().is_some(),
                !reverse,
            );
            if let Some(closed) = closed {
                assert_eq!(closed.terms.len(), 1);
                let (power, coefficient) = &closed.terms[0];
                assert_eq!(*power, 0);
                assert!(coefficient.rational.is_zero());
                let expected = [3, 5, 7, 11, 13, 17, 19, 23]
                    .map(|prime| (Rational::new(prime), Rational::one()));
                assert_eq!(coefficient.radicals, expected);
            }
            assert!(query.cached().is_none());
        }
        if !reverse {
            let remaining = root_sum(&primes[1..], 0);
            let isolated = raw(Approximation::Add(
                value.clone(),
                raw(Approximation::Negate(remaining)),
            ));
            assert_eq!(
                isolated.exact_quadratic_surd_parts(),
                Some((Rational::zero(), Rational::one(), Some(Rational::new(23)))),
            );
        }
    }
}

#[test]
fn failed_merge_does_not_publish_partial_coefficients() {
    let left = root_sum(&[2, 3, 5, 7, 11, 13, 17, 19], 7);
    let right = raw(Approximation::Add(
        Computable::rational(Rational::new(11)),
        root(23),
    ));
    let value = raw(Approximation::Add(left.clone(), right));
    let clone = value.clone();
    let weak = Arc::downgrade(&value.internal);
    let left_weak = Arc::downgrade(&left.internal);
    assert!(
        left.project_pi_laurent_polynomial(64, |polynomial| {
            ClosedPiLaurentPolynomial::from_polynomial(&polynomial)
        })
        .is_some()
    );
    for query in [&value, &clone, &value] {
        // Merging the constants succeeds before the ninth radical is rejected.
        // The partial result must never become a certificate for either node.
        assert!(query.exact_quadratic_surd_parts().is_none());
        assert!(query.bounded_laurent_rational(64).is_none());
        assert!(query.internal.cache.pi_laurent_polynomial().is_none());
        let retained = left.internal.cache.pi_laurent_polynomial().unwrap();
        assert_eq!(retained.terms.len(), 1);
        assert_eq!(retained.terms[0].1.rational, Rational::new(7));
        assert_eq!(retained.terms[0].1.radicals.len(), 8);
        assert!(query.cached().is_none());
    }
    let cancellation = raw(Approximation::Add(
        value.clone(),
        raw(Approximation::Negate(value.clone())),
    ));
    assert_eq!(
        cancellation.bounded_laurent_rational(64),
        Some(Rational::zero()),
    );
    drop(cancellation);
    drop(clone);
    drop(value);
    drop(left);
    assert!(weak.upgrade().is_none());
    assert!(left_weak.upgrade().is_none());
}

#[test]
fn opaque_cancellation_preserves_owners_and_the_exact_remainder() {
    let atoms = (1..=16)
        .map(|numerator| {
            raw(Approximation::AtanRational(
                Rational::fraction(numerator, 37).unwrap(),
            ))
        })
        .collect::<Vec<_>>();
    let weak = atoms
        .iter()
        .map(|atom| Arc::downgrade(&atom.internal))
        .collect::<Vec<_>>();
    let sum = atoms.iter().fold(Computable::zero(), |sum, atom| {
        raw(Approximation::Add(sum, atom.clone()))
    });
    let value = raw(Approximation::Add(
        raw(Approximation::Add(sum.clone(), root(13))),
        raw(Approximation::Negate(sum.clone())),
    ));
    let clone = value.clone();
    let owners = atoms
        .iter()
        .map(|atom| Arc::strong_count(&atom.internal))
        .collect::<Vec<_>>();
    for query in [&value, &clone, &value] {
        assert_eq!(
            query.exact_quadratic_surd_parts(),
            Some((Rational::zero(), Rational::one(), Some(Rational::new(13)))),
        );
        assert!(sum.internal.cache.pi_laurent_polynomial().is_none());
        assert!(query.cached().is_none());
        assert_eq!(
            atoms
                .iter()
                .map(|atom| Arc::strong_count(&atom.internal))
                .collect::<Vec<_>>(),
            owners,
        );
    }
    drop(clone);
    drop(value);
    drop(sum);
    drop(atoms);
    assert!(weak.iter().all(|owner| owner.upgrade().is_none()));
}
