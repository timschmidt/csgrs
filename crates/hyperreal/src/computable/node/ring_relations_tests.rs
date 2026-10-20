#[cfg(test)]
mod ring_relations_tests {
    use super::*;

    #[cfg(feature = "serde")]
    #[test]
    fn captured_cone_projections_fit_the_existing_attempt_limits() {
        let rows: Vec<serde_json::Value> = serde_json::from_str(include_str!(
            "../../../tests/fixtures/cone-continuation-zeros.json"
        ))
        .unwrap();
        let mut failures = Vec::new();
        for row in rows {
            let root: Computable =
                serde_json::from_value(row["value"]["computable"].clone()).unwrap();
            let mut recognition = RingRecognition::new(&root);
            recognition.root_squares = true;
            recognition.trig_arguments = Some(Vec::new());
            recognition.rational_squares = Some(Vec::new());
            let result = recognition
                .expand_quotient(&root, RingRecognition::MAX_DEPTH)
                .and_then(|q| recognition.project_quotient(&q));
            if result != Some(Rational::zero()) {
                failures.push(format!(
                    "{}: {:?}, nodes={}, work={}, atoms={}, roots={}, circles={}",
                    row["name"],
                    result,
                    recognition.nodes_left,
                    recognition.operations_left,
                    recognition.atoms.len(),
                    recognition.rational_squares.as_ref().unwrap().len(),
                    recognition.unit_circle_pairs.len()
                ));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

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
    fn sqrt(a: Computable) -> Computable {
        raw(Approximation::Sqrt(a))
    }
    fn angle() -> Computable {
        raw(Approximation::AtanRational(
            Rational::fraction(3, 4).unwrap(),
        ))
    }
    fn root_identity(radicand: Computable, square: Computable) -> Computable {
        let root = sqrt(radicand);
        add(
            mul(mul(angle(), root.clone()), root),
            neg(mul(angle(), square)),
        )
    }
    fn cone_identity() -> Computable {
        let a = angle();
        let root = sqrt(number(5));
        add(
            mul(
                raw(Approximation::Ratio(Rational::fraction(1, 5).unwrap())),
                mul(
                    Computable::pi(),
                    mul(
                        mul(a.clone(), root.clone()),
                        mul(raw(Approximation::Constant(SharedConstant::InvPi)), root),
                    ),
                ),
            ),
            neg(a),
        )
    }

    #[test]
    fn rational_root_squares_and_builtin_reciprocals_are_exact() {
        for square in [
            Rational::zero(),
            Rational::from(5),
            Rational::fraction(7, 11).unwrap(),
        ] {
            for residual in [-7, 0, 11] {
                let q = raw(Approximation::Ratio(square.clone()));
                let value = add(root_identity(q.clone(), q), number(residual));
                assert_eq!(
                    value.bounded_relation_rational(),
                    Some(Rational::from(residual))
                );
                assert!(value.cached().is_none());
            }
        }
        for residual in [-7, 0, 11] {
            let value = add(cone_identity(), number(residual));
            assert_eq!(value.bounded_trig_congruence_rational(), None);
            assert_eq!(
                value.bounded_relation_rational(),
                Some(Rational::from(residual))
            );
            assert!(value.cached().is_none());
        }
        let nested = root_identity(sqrt(number(25)), number(5));
        assert_eq!(nested.bounded_relation_rational(), Some(Rational::zero()));
    }

    #[test]
    fn domain_failures_and_unproved_relations_remain_unresolved() {
        let inverse = |x| raw(Approximation::Inverse(x));
        for value in [
            root_identity(number(-5), number(-5)),
            root_identity(inverse(number(0)), number(0)),
            mul(cone_identity(), inverse(number(0))),
            inverse(cone_identity()),
            root_identity(number(5), number(6)),
            root_identity(angle(), number(5)),
            mul(cone_identity(), inverse(add(angle(), number(-1)))),
        ] {
            assert_eq!(value.bounded_relation_rational(), None);
            assert!(matches!(
                value.internal.facts.exact_sign(),
                ExactSignCache::Invalid
            ));
        }
        let positive = add(number(8), neg(number(3)));
        assert_eq!(
            root_identity(positive, number(5)).bounded_relation_rational(),
            Some(Rational::zero())
        );
    }

    #[test]
    fn relations_preserve_nearby_values_and_function_identity() {
        let epsilon = Rational::fraction(1, 1_000_000_000_000_u64).unwrap();
        for sign in [-1, 1] {
            let offset = Rational::from(sign) * &epsilon;
            let value = add(cone_identity(), raw(Approximation::Ratio(offset.clone())));
            assert_eq!(value.bounded_relation_rational(), Some(offset));
        }
        let a = add(angle(), cone_identity());
        let b = angle();
        let sin = |x| raw(Approximation::PrescaledSin(x));
        let cos = |x| raw(Approximation::PrescaledCos(x));
        assert_eq!(
            add(sin(a.clone()), neg(sin(b.clone()))).bounded_relation_rational(),
            Some(Rational::zero())
        );
        assert_eq!(
            add(sin(a), neg(cos(b))).bounded_relation_rational(),
            Some(Rational::fraction(-1, 5).unwrap())
        );
    }

    #[test]
    fn relation_limits_and_cancellation_allow_a_fresh_retry() {
        let mut value = cone_identity();
        for (nodes, work, depth) in [(2, 8192, 64), (512, 2, 64), (512, 8192, 2)] {
            let mut recognition = RingRecognition::new(&value);
            recognition.root_squares = true;
            recognition.rational_squares = Some(Vec::new());
            recognition.nodes_left = nodes;
            recognition.operations_left = work;
            assert!(recognition.expand_quotient(&value, depth).is_none());
        }
        let large = raw(Approximation::Int(BigInt::from(1) << 4096_u32));
        assert_eq!(
            root_identity(large.clone(), large).bounded_relation_rational(),
            None
        );
        let shifted = raw(Approximation::Offset(number(1), i32::MAX));
        assert_eq!(
            root_identity(shifted.clone(), shifted).bounded_relation_rational(),
            None
        );
        let signal = Arc::new(AtomicBool::new(true));
        value.abort(signal.clone());
        assert_eq!(value.bounded_relation_rational(), None);
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(value.bounded_relation_rational(), Some(Rational::zero()));
    }
}
