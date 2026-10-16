#[cfg(test)]
mod ring_root_square_tests {
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
    fn mul(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Multiply(a, b))
    }
    fn neg(a: Computable) -> Computable {
        raw(Approximation::Negate(a))
    }
    fn atom(n: u64) -> Computable {
        raw(Approximation::PrescaledSin(Computable::rational(
            Rational::fraction(1, n).unwrap(),
        )))
    }
    fn identity(root: SharedConstant, square: i32) -> Computable {
        // (a+qb)(a-qb) - a² + square*b², independent of a and b.
        let (a, b) = (atom(3), atom(5));
        let qb = mul(raw(Approximation::Constant(root)), b.clone());
        add(
            add(
                mul(add(a.clone(), qb.clone()), add(a.clone(), neg(qb))),
                neg(mul(a.clone(), a)),
            ),
            mul(number(square), mul(b.clone(), b)),
        )
    }

    #[test]
    fn root_square_projection_combines_exact_roots_and_opaque_atoms() {
        for (root, square) in [(SharedConstant::Sqrt2, 2), (SharedConstant::Sqrt3, 3)] {
            for residual in [-7, 0, 11] {
                let value = add(identity(root, square), number(residual));
                assert_eq!(value.bounded_quotient_rational(), None);
                assert_eq!(
                    value.bounded_root_square_rational(),
                    Some(Rational::from(residual))
                );
                assert!(
                    value.cached().is_none(),
                    "the certificate does not approximate"
                );
            }
        }
        let mixed = add(
            identity(SharedConstant::Sqrt2, 2),
            identity(SharedConstant::Sqrt3, 3),
        );
        assert_eq!(mixed.bounded_root_square_rational(), Some(Rational::zero()));
    }

    #[test]
    fn root_square_projection_keeps_domain_obligations_and_unknowns() {
        let identity = identity(SharedConstant::Sqrt2, 2);
        let invalid = mul(identity.clone(), raw(Approximation::Inverse(number(0))));
        assert_eq!(invalid.bounded_root_square_rational(), None);
        assert!(matches!(
            invalid.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        let inverse_zero = raw(Approximation::Inverse(identity.clone()));
        assert_eq!(inverse_zero.bounded_root_square_rational(), None);
        let unrelated = add(identity, add(mul(atom(7), atom(7)), number(-2)));
        assert_eq!(unrelated.bounded_root_square_rational(), None);
        assert!(matches!(
            unrelated.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        // An incorrect square relation must not be admitted.
        let wrong = identity_with_wrong_square();
        assert_eq!(wrong.bounded_root_square_rational(), None);
    }

    fn identity_with_wrong_square() -> Computable {
        identity(SharedConstant::Sqrt2, 3)
    }

    #[test]
    fn root_square_limits_and_cancellation_do_not_poison_recovery() {
        let mut value = identity(SharedConstant::Sqrt2, 2);
        for (nodes, work, depth) in [(2, 8192, 64), (512, 1, 64), (512, 8192, 1)] {
            let mut recognition = RingRecognition::new(&value);
            recognition.root_squares = true;
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
        assert_eq!(value.bounded_root_square_rational(), None);
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(value.bounded_root_square_rational(), Some(Rational::zero()));
    }
}
