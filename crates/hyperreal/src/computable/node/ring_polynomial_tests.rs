#[cfg(test)]
mod ring_polynomial_tests {
    use super::*;

    fn raw(approximation: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    fn binomial_identity() -> Computable {
        let a = Computable::rational(Rational::fraction(1, 3).unwrap()).sin();
        let b = Computable::rational(Rational::fraction(2, 5).unwrap()).sin();
        let sum = raw(Approximation::Add(a.clone(), b.clone()));
        let square = raw(Approximation::Square(sum));
        let ab = raw(Approximation::Multiply(a.clone(), b.clone()));
        let twice_ab = raw(Approximation::Offset(ab, 1));
        let squares = raw(Approximation::Add(
            raw(Approximation::Square(a)),
            raw(Approximation::Square(b)),
        ));
        raw(Approximation::Add(
            square,
            raw(Approximation::Negate(raw(Approximation::Add(
                squares, twice_ab,
            )))),
        ))
    }

    #[test]
    fn ring_limits_decline_without_caching_and_allow_later_recovery() {
        let value = binomial_identity();
        let mut nodes = RingRecognition::new(&value);
        nodes.nodes_left = 2;
        assert!(nodes.expand(&value, RingRecognition::MAX_DEPTH).is_none());
        let mut work = RingRecognition::new(&value);
        work.operations_left = 1;
        assert!(work.expand(&value, RingRecognition::MAX_DEPTH).is_none());
        assert!(RingRecognition::new(&value).expand(&value, 1).is_none());
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        assert!(value.cached().is_none());
        assert_eq!(value.bounded_ring_rational(), Some(Rational::zero()));
        assert!(value.cached().is_none(), "the proof does not approximate");
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Valid(Sign::NoSign)
        ));
    }

    #[test]
    fn aborted_ring_proof_does_not_publish_and_can_recover() {
        let mut value = binomial_identity();
        let clone = value.clone();
        let signal = Arc::new(AtomicBool::new(true));
        value.abort(signal.clone());
        assert_eq!(value.bounded_ring_rational(), None);
        assert!(matches!(
            clone.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(value.bounded_ring_rational(), Some(Rational::zero()));
        assert!(matches!(
            clone.internal.facts.exact_sign(),
            ExactSignCache::Valid(Sign::NoSign)
        ));
    }

    #[test]
    fn polynomial_term_degree_and_atom_caps_are_independent() {
        let atoms: Vec<_> = (1..=33)
            .map(|i| Computable::rational(Rational::fraction(i, 100).unwrap()).sin())
            .collect();
        let mut recognition = RingRecognition::new(&atoms[0]);
        for atom in &atoms[..32] {
            assert!(recognition.atom(atom, RingRecognition::MAX_DEPTH).is_some());
        }
        assert!(recognition.atom(&atoms[32], RingRecognition::MAX_DEPTH).is_none());
        let mut polynomial = RingPolynomial::default();
        let monomials: Vec<_> = (0_u8..12)
            .flat_map(|a| (a..12).map(move |b| vec![a, b]))
            .collect();
        for term in &monomials[..64] {
            assert!(
                recognition
                    .insert(&mut polynomial, term.clone(), Rational::one())
                    .is_some()
            );
        }
        assert!(
            recognition
                .insert(&mut polynomial, monomials[64].clone(), Rational::one())
                .is_none()
        );
        assert!(
            recognition
                .insert(&mut RingPolynomial::default(), vec![0; 9], Rational::one())
                .is_none()
        );
    }

    #[test]
    fn coefficient_and_shift_limits_precede_unbounded_expansion() {
        let within = Rational::from_bigint(BigInt::one() << 4094_usize);
        assert!(RingRecognition::constant(within).is_some());
        let excessive = Rational::from_bigint(BigInt::one() << 4096_usize);
        assert!(RingRecognition::constant(excessive).is_none());
        let fresh = raw(Approximation::PrescaledSin(Computable::rational(
            Rational::fraction(1, 3).unwrap(),
        )));
        let large =
            RingRecognition::constant(Rational::from_bigint(BigInt::one() << 2048_usize)).unwrap();
        for value in [Computable::pi(), fresh.clone()] {
            assert!(
                RingRecognition::new(&value)
                    .multiply(&large, &large)
                    .is_none()
            );
            for shift in [i32::MIN, i32::MAX] {
                // Exercise only this recognizer: older optional recognizers
                // have separate limits and are not bounded by this test.
                let shifted = raw(Approximation::Offset(value.clone(), shift));
                assert_eq!(shifted.bounded_ring_rational(), None);
            }
        }
        // Other tests can warm pi's shared cache. This expression is private
        // to the test, so its empty cache establishes no numerical refinement.
        assert!(fresh.cached().is_none());
    }
}
