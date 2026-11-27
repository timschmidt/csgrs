#[cfg(test)]
/// Qualification of plain polynomial recognition, separate caps and abort-safe publication/retry.
mod ring_polynomial_tests {
    use super::*;

    /// Construct a native fixture with uninitialized facts and no stop handle, bypassing simplification.
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

    /// Retain (a+b)^2-(a^2+b^2+2ab) with two different rational-angle sine atoms.
    /// The independent ring identity is zero for any defined a,b; fixture construction
    /// keeps the composite arithmetic unsimplified so the polynomial path must prove it.
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
    /// Exhaust node, work and depth allowances separately on the retained binomial identity.
    /// Require no global sign or numeric cache from declines, then a fresh ordinary
    /// attempt must prove zero and publish its sign without numeric refinement.
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
    /// Stop a polynomial proof, then reset its handle and retry the same shared node.
    /// A clone must observe no premature sign and then the successful exact-zero sign,
    /// showing shared proof publication and retry rather than cached failure.
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
    /// Fill all 32 opaque-atom slots and 64 polynomial terms, then test each next insertion.
    /// Separately reject degree nine in an ordinary degree-eight attempt. Fixtures
    /// use distinct small sine recipes and sorted two-factor monomials; this tests
    /// the independent caps, not a total allocation or runtime bound.
    fn polynomial_term_degree_and_atom_caps_are_independent() {
        let atoms: Vec<_> = (1..=33)
            .map(|i| Computable::rational(Rational::fraction(i, 100).unwrap()).sin())
            .collect();
        let mut recognition = RingRecognition::new(&atoms[0]);
        for atom in &atoms[..32] {
            assert!(recognition.atom(atom, RingRecognition::MAX_DEPTH).is_some());
        }
        assert!(
            recognition
                .atom(&atoms[32], RingRecognition::MAX_DEPTH)
                .is_none()
        );
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
    /// Check coefficient admission around the 4096-bit cap and reject excessive products/shifts.
    /// An integer 2^4094 fits, while 2^4096 and the square of 2^2048 do not. Both
    /// i32 shift extremes must decline through this recognizer before coefficient
    /// materialization. The private sine remains numerically cold; other recognizers
    /// and process-global pi-cache warmth are outside these assertions.
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
