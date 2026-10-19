#[cfg(test)]
mod ring_quotient_tests {
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

    fn positive_denominator() -> Computable {
        raw(Approximation::Add(
            raw(Approximation::Int(BigInt::from(4))),
            raw(Approximation::PrescaledSin(Computable::rational(
                Rational::fraction(1, 3).unwrap(),
            ))),
        ))
    }

    fn inverse_identity(denominator: &Computable) -> Computable {
        raw(Approximation::Add(
            raw(Approximation::Multiply(
                denominator.clone(),
                raw(Approximation::Inverse(denominator.clone())),
            )),
            raw(Approximation::Int(BigInt::from(-1))),
        ))
    }

    fn assert_unproved(value: &Computable) {
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        assert!(value.cached().is_none());
    }

    #[test]
    #[cfg(feature = "serde")]
    fn domain_contradictions_never_publish_conditional_facts() {
        let atom = raw(Approximation::Sqrt(raw(Approximation::Negate(raw(
            Approximation::AtanRational(Rational::one()),
        )))));
        let value = raw(Approximation::Multiply(Computable::zero(), atom.clone()));
        // If the atom denotes a real, this product is zero. That condition
        // suffices to reject its reciprocal, but does not define the product.
        assert!(value.bounded_domain_contradiction(Rational::is_zero));
        assert!(!value.bounded_domain_contradiction(|r| r.sign() == Sign::Minus));
        assert_unproved(&value);
        assert_unproved(&atom);
        assert_eq!(value.bounded_defined_rational(), None);
        assert_unproved(&value);

        let distinct = raw(Approximation::Add(Computable::pi(), Computable::one()));
        assert!(!distinct.bounded_domain_contradiction(|_| true));
        assert_unproved(&distinct);
    }

    #[test]
    #[cfg(feature = "serde")]
    fn domain_contradiction_limits_and_abort_preserve_facts_and_recover() {
        let atom = raw(Approximation::PrescaledSin(Computable::rational(
            Rational::fraction(1, 2).unwrap(),
        )));
        let mut value = raw(Approximation::Add(
            atom.clone(),
            raw(Approximation::Negate(atom.clone())),
        ));
        let mut deep = value.clone();
        for _ in 0..RingRecognition::MAX_DEPTH {
            deep = raw(Approximation::Negate(deep));
        }
        assert!(!deep.bounded_domain_contradiction(Rational::is_zero));
        assert_unproved(&deep);
        assert_unproved(&value);
        let signal = Arc::new(AtomicBool::new(true));
        value.abort(signal.clone());
        assert!(!value.bounded_domain_contradiction(Rational::is_zero));
        assert_unproved(&value);
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        for precision in [-8, -64, -8] {
            assert_eq!(atom.sign_until(precision), Some(RealSign::Positive));
            assert!(value.bounded_domain_contradiction(Rational::is_zero));
            assert_unproved(&value);
            assert_eq!(atom.immediate_sign(), Some(RealSign::Positive));
        }
    }

    #[test]
    fn quotient_reuse_requires_a_real_nonzero_proof_and_can_recover() {
        let denominator = positive_denominator();
        let value = inverse_identity(&denominator);
        assert_eq!(denominator.immediate_sign(), None);
        assert_eq!(value.bounded_quotient_rational(), None);
        assert_unproved(&value);
        assert_unproved(&denominator);
        denominator
            .internal
            .facts
            .replace_exact_sign(ExactSignCache::Unknown);
        assert_eq!(value.bounded_quotient_rational(), None);
        assert_unproved(&value);
        assert_eq!(denominator.sign_until(-64), Some(RealSign::Positive));
        assert_eq!(denominator.immediate_sign(), Some(RealSign::Positive));
        assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
        assert_eq!(value.sign_until(0), Some(RealSign::Zero));
        assert!(value.cached().is_none());
    }

    #[test]
    fn quotient_rejects_zero_and_unproved_denominators() {
        let a = positive_denominator();
        let cancelled = raw(Approximation::Add(a.clone(), raw(Approximation::Negate(a))));
        for denominator in [
            Computable::zero(),
            raw(Approximation::Int(BigInt::zero())),
            cancelled,
            positive_denominator(),
        ] {
            let value = inverse_identity(&denominator);
            assert_eq!(value.bounded_quotient_rational(), None);
            assert_unproved(&value);
        }
    }

    #[test]
    fn quotient_can_prove_a_nonzero_denominator_without_refinement() {
        let a = positive_denominator();
        let denominator = raw(Approximation::Add(
            raw(Approximation::Add(
                a.clone(),
                raw(Approximation::Int(BigInt::from(2))),
            )),
            raw(Approximation::Negate(a)),
        ));
        assert_eq!(denominator.immediate_sign(), None);
        let value = inverse_identity(&denominator);
        assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
        // The local denominator witness was not published as a partial root
        // result. Only the completed queried expression receives a proof.
        assert_unproved(&denominator);
        assert!(value.cached().is_none());
    }

    #[test]
    fn quotient_limits_and_cancellation_publish_no_partial_proof() {
        let denominator = positive_denominator();
        assert_eq!(denominator.sign_until(-64), Some(RealSign::Positive));
        let mut value = inverse_identity(&denominator);
        let mut nodes = RingRecognition::new(&value);
        nodes.nodes_left = 2;
        assert!(
            nodes
                .expand_quotient(&value, RingRecognition::MAX_DEPTH)
                .is_none()
        );
        let mut work = RingRecognition::new(&value);
        work.operations_left = 1;
        assert!(
            work.expand_quotient(&value, RingRecognition::MAX_DEPTH)
                .is_none()
        );
        assert!(
            RingRecognition::new(&value)
                .expand_quotient(&value, 1)
                .is_none()
        );
        assert_unproved(&value);
        let signal = Arc::new(AtomicBool::new(true));
        value.abort(signal.clone());
        assert_eq!(value.bounded_quotient_rational(), None);
        assert_unproved(&value);
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
        assert!(value.cached().is_none());
    }

    #[test]
    fn quotient_expansion_preserves_degree_and_coefficient_limits() {
        let denominator = positive_denominator();
        assert_eq!(denominator.sign_until(-64), Some(RealSign::Positive));
        let mut value = raw(Approximation::Inverse(denominator.clone()));
        for _ in 0..4 {
            value = raw(Approximation::Square(value));
        }
        assert_eq!(value.bounded_quotient_rational(), None);
        assert_unproved(&value);
        for shift in [i32::MIN, i32::MAX] {
            let value = raw(Approximation::Offset(inverse_identity(&denominator), shift));
            assert_eq!(value.bounded_quotient_rational(), None);
            assert_unproved(&value);
        }
    }

    #[test]
    fn shared_quotient_subexpressions_fit_the_existing_expansion_budget() {
        let denominator = positive_denominator();
        assert_eq!(denominator.sign_until(-64), Some(RealSign::Positive));
        let mut sum = denominator.clone();
        for _ in 0..7 {
            sum = raw(Approximation::Add(sum.clone(), sum));
        }
        // 128*d/d - 128 = 0, with d > 0. The balanced shared sum contains
        // only seven new nodes, but unfolding it revisits d 128 times and
        // exceeds 512 arithmetic visits. Keep the original expansion caps.
        let value = raw(Approximation::Add(
            raw(Approximation::Multiply(
                sum,
                raw(Approximation::Inverse(denominator)),
            )),
            raw(Approximation::Int(BigInt::from(-128))),
        ));
        assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
        assert!(value.cached().is_none());
    }

    #[test]
    fn reused_quotient_proofs_still_charge_work_and_observe_cancellation() {
        let denominator = positive_denominator();
        assert_eq!(denominator.sign_until(-64), Some(RealSign::Positive));
        let mut value = inverse_identity(&denominator);
        let signal = Arc::new(AtomicBool::new(false));
        value.abort(signal.clone());
        let mut recognition = RingRecognition::new(&value);
        let first = recognition
            .expand_quotient(&value, RingRecognition::MAX_DEPTH)
            .unwrap();
        assert_eq!(recognition.project_quotient(&first), Some(Rational::zero()));
        recognition.nodes_left = 0;
        recognition.operations_left = 0;
        assert!(recognition.expand_quotient(&value, 1).is_none());
        recognition.operations_left = 1;
        signal.store(true, std::sync::atomic::Ordering::Relaxed);
        assert!(recognition.expand_quotient(&value, 1).is_none());
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert!(recognition.expand_quotient(&value, 0).is_none());
        assert!(recognition.expand_quotient(&value, 1).is_some());
        assert_eq!(recognition.operations_left, 0);
        // Local projection reuse alone does not publish any root fact.
        assert_unproved(&value);
        drop(recognition);
        assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
    }

    #[test]
    fn zero_quotients_do_not_accumulate_irrelevant_denominators() {
        let denominator = positive_denominator();
        assert_eq!(denominator.sign_until(-64), Some(RealSign::Positive));
        let reciprocal = raw(Approximation::Inverse(denominator));
        let zero = raw(Approximation::Add(
            reciprocal.clone(),
            raw(Approximation::Negate(reciprocal.clone())),
        ));
        let mut eighth_power = reciprocal;
        for _ in 0..3 {
            eighth_power = raw(Approximation::Square(eighth_power));
        }
        // Each child fits degree 8. Once 1/d - 1/d is proved zero,
        // multiplying its unused denominator by d^8 serves no proof.
        for (left, right) in [
            (zero.clone(), eighth_power.clone()),
            (eighth_power.clone(), zero),
        ] {
            let product = raw(Approximation::Multiply(left.clone(), right.clone()));
            assert_eq!(product.bounded_quotient_rational(), Some(Rational::zero()));
            let difference = raw(Approximation::Add(
                raw(Approximation::Add(left, right)),
                raw(Approximation::Negate(eighth_power.clone())),
            ));
            assert_eq!(
                difference.bounded_quotient_rational(),
                Some(Rational::zero())
            );
            assert!(product.cached().is_none());
            assert!(difference.cached().is_none());
        }
    }

    #[test]
    fn cancelled_cross_numerators_need_no_denominator_product() {
        let denominator = positive_denominator();
        assert_eq!(denominator.sign_until(-64), Some(RealSign::Positive));
        let fourth = raw(Approximation::Square(raw(Approximation::Square(
            denominator,
        ))));
        let eighth = raw(Approximation::Square(fourth.clone()));
        assert_eq!(fourth.sign_until(-64), Some(RealSign::Positive));
        assert_eq!(eighth.sign_until(-64), Some(RealSign::Positive));
        // 1/d^4 - d^4/d^8 = 0. The cross numerator has degree 8;
        // constructing d^12 after it cancels would exceed the same cap.
        let value = raw(Approximation::Add(
            raw(Approximation::Inverse(fourth.clone())),
            raw(Approximation::Negate(raw(Approximation::Multiply(
                fourth,
                raw(Approximation::Inverse(eighth)),
            )))),
        ));
        assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
        assert!(value.cached().is_none());
    }

    #[test]
    fn zero_absorption_does_not_hide_unproved_or_zero_divisors() {
        for denominator in [positive_denominator(), Computable::zero()] {
            let reciprocal = raw(Approximation::Inverse(denominator));
            for (left, right) in [
                (Computable::zero(), reciprocal.clone()),
                (reciprocal, Computable::zero()),
            ] {
                let value = raw(Approximation::Multiply(left, right));
                assert_eq!(value.bounded_quotient_rational(), None);
                assert_unproved(&value);
            }
        }
    }
}
