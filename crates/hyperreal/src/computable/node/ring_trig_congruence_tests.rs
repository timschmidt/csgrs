#[cfg(test)]
mod ring_trig_congruence_tests {
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
    fn number(n: i32) -> Computable {
        raw(Approximation::Ratio(Rational::from(n)))
    }
    fn add(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, b))
    }
    fn multiply(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Multiply(a, b))
    }
    fn negate(a: Computable) -> Computable {
        raw(Approximation::Negate(a))
    }
    fn function(cosine: bool, argument: Computable) -> Computable {
        raw(if cosine {
            Approximation::PrescaledCos(argument)
        } else {
            Approximation::PrescaledSin(argument)
        })
    }
    fn arguments() -> (Computable, Computable) {
        let a = raw(Approximation::AtanRational(
            Rational::fraction(3, 4).unwrap(),
        ));
        let pi = Computable::pi();
        // 3*(a+pi)-3*pi = 3*a, independently of the actual value of a.
        (
            add(
                multiply(number(3), add(a.clone(), pi.clone())),
                negate(multiply(number(3), pi)),
            ),
            multiply(number(3), a),
        )
    }
    fn identity(cosine: bool) -> Computable {
        let (a, b) = arguments();
        add(function(cosine, a), negate(function(cosine, b)))
    }

    #[test]
    fn exact_arguments_share_only_the_same_function() {
        for cosine in [false, true] {
            for residual in [-7, 0, 11] {
                let value = add(identity(cosine), number(residual));
                assert_eq!(value.bounded_root_square_rational(), None);
                assert_eq!(
                    value.bounded_trig_congruence_rational(),
                    Some(Rational::from(residual))
                );
                assert!(
                    value.cached().is_none(),
                    "no numeric refinement is part of congruence"
                );
            }
            let (a, b) = arguments();
            let mixed = add(function(cosine, a), negate(function(!cosine, b)));
            assert_eq!(mixed.bounded_trig_congruence_rational(), None);
            let (a, b) = arguments();
            let gap = Computable::rational(Rational::fraction(1, 1_000_000_000_000_u64).unwrap());
            let nearby = add(function(cosine, a), negate(function(cosine, add(b, gap))));
            assert_eq!(nearby.bounded_trig_congruence_rational(), None);
        }
    }

    #[test]
    fn nested_functions_and_quotients_reuse_the_same_proof_scope() {
        let (a, b) = arguments();
        let first = function(true, function(false, a));
        let second = function(true, function(false, b));
        let value = add(first, negate(second));
        assert_eq!(
            value.bounded_trig_congruence_rational(),
            Some(Rational::zero())
        );
        let (a, b) = arguments();
        let divide = |x| multiply(x, raw(Approximation::Inverse(number(3))));
        let value = add(
            function(false, divide(a)),
            negate(function(false, divide(b))),
        );
        assert_eq!(
            value.bounded_trig_congruence_rational(),
            Some(Rational::zero())
        );
    }

    #[test]
    fn invalid_inverse_arguments_cannot_cancel_through_function_congruence() {
        for cosine in [false, true] {
            let (a, b) = arguments();
            let invalid = || raw(Approximation::Inverse(number(0)));
            let value = add(
                function(cosine, multiply(a, invalid())),
                negate(function(cosine, multiply(b, invalid()))),
            );
            assert_eq!(value.bounded_trig_congruence_rational(), None);
            assert!(matches!(
                value.internal.facts.exact_sign(),
                ExactSignCache::Invalid
            ));
        }
    }

    #[test]
    fn exhaustion_and_cancellation_preserve_retry() {
        let mut value = identity(true);
        for (nodes, work, depth) in [(2, 8192, 64), (512, 2, 64), (512, 8192, 2)] {
            let mut recognition = RingRecognition::new(&value);
            recognition.trig_arguments = Some(Vec::new());
            recognition.nodes_left = nodes;
            recognition.operations_left = work;
            assert!(recognition.expand_quotient(&value, depth).is_none());
        }
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        let signal = Arc::new(AtomicBool::new(true));
        value.abort(signal.clone());
        assert_eq!(value.bounded_trig_congruence_rational(), None);
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(
            value.bounded_trig_congruence_rational(),
            Some(Rational::zero())
        );
    }
}
