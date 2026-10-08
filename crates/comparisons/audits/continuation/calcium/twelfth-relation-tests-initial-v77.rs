#[cfg(test)]
mod twelfth_relation_tests {
    use super::*;

    fn raw(approximation: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(approximation, BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    fn difference(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, raw(Approximation::Negate(b))))
    }

    fn sine15() -> Computable {
        raw(Approximation::PrescaledSin(raw(Approximation::Multiply(
            Computable::pi(), Computable::rational(Rational::fraction(1,12).unwrap()),
        ))))
    }

    fn radical15() -> Computable {
        difference(raw(Approximation::Sqrt(Computable::rational(Rational::new(6)))),
            Computable::sqrt2()).shift_right(2)
    }

    #[test]
    fn proof_preserves_nodes_and_numeric_caches() {
        let sine = sine15();
        let radical = radical15();
        let zero = difference(sine.clone(), radical.clone());
        assert!(sine.cached().is_none());
        assert!(radical.cached().is_none());
        assert_eq!(zero.exact_twelfth_relation_sign(), Some(Sign::NoSign));
        assert_eq!(zero.exact_sign(), Some(Sign::NoSign));
        assert!(zero.cached().is_none());
        assert!(sine.cached().is_none());
        assert!(radical.cached().is_none());
        assert!(matches!(sine.internal.approximation, Approximation::PrescaledSin(_)));
    }

    #[test]
    fn descendant_queries_and_unknown_children_do_not_hide_proof() {
        for warm_child in [false, true] {
            let sine = sine15();
            if warm_child { assert_eq!(sine.exact_sign(), None); }
            let zero = difference(sine, radical15());
            let parent = raw(Approximation::Square(zero.clone()));
            assert_eq!(parent.exact_sign(), Some(Sign::NoSign));
            assert_eq!(zero.exact_sign(), Some(Sign::NoSign));
            for n in [-1, 1] {
                let nonzero = raw(Approximation::Add(zero.clone(), Computable::rational(Rational::fraction(n,1024).unwrap())));
                assert_eq!(nonzero.exact_sign(), Some(if n < 0 { Sign::Minus } else { Sign::Plus }));
            }
        }
    }

    #[test]
    fn rejects_unproved_angles_domains_and_excessive_work() {
        for bad in [
            raw(Approximation::PrescaledSin(Computable::pi().multiply(Computable::rational(Rational::fraction(1,7).unwrap())))),
            raw(Approximation::PrescaledSin(Computable::rational(Rational::fraction(1,12).unwrap()))),
            raw(Approximation::Inverse(Computable::zero())),
            raw(Approximation::Sqrt(Computable::rational(Rational::new(-2)))),
            raw(Approximation::Sqrt(Computable::rational(Rational::new(5)))),
            raw(Approximation::PrescaledTan(Computable::pi().shift_right(1))),
            raw(Approximation::PrescaledCot(Computable::zero())),
            Computable::rational(Rational::from_bigint(BigInt::one() << 2048usize)),
        ] {
            assert!(difference(sine15(), bad).exact_twelfth_relation_sign().is_none());
        }
        let mut deep = sine15();
        for _ in 0..130 { deep = raw(Approximation::Negate(deep)); }
        assert!(difference(deep, radical15()).exact_twelfth_relation_sign().is_none());
        assert!(difference(raw(Approximation::Offset(sine15(), i32::MAX)), radical15()).exact_twelfth_relation_sign().is_none());
    }
}
