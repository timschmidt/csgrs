#[cfg(test)]
mod ring_native_sign_tests {
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

    fn q(value: i32) -> Computable {
        Computable::rational(Rational::from(value))
    }

    fn add(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, b))
    }

    fn mul(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Multiply(a, b))
    }

    fn root_pair(n: i32, certify: bool) -> (Computable, Computable) {
        let a = raw(Approximation::Sqrt(q(n)));
        let b = raw(Approximation::Sqrt(Computable::rational(
            Rational::fraction(1, n as u64).unwrap(),
        )));
        if certify {
            assert_eq!(a.sign_until(-32), Some(RealSign::Positive));
            assert_eq!(b.sign_until(-32), Some(RealSign::Positive));
        }
        (a, b)
    }

    fn native(value: &Computable) -> RingRecognition<'_> {
        let mut recognition = RingRecognition::new(value);
        recognition.native_relations = true;
        recognition.root_completion = true;
        recognition
    }

    fn unproved(value: &Computable) {
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        assert!(value.cached().is_none());
        assert!(value.retained_rational_reduction().is_none());
    }

    #[test]
    fn signed_projection_replays_positive_root_branches_and_signed_factors() {
        for n in [2, 3, 7] {
            let (a, b) = root_pair(n, true);
            let product = mul(a, b);
            for offset in [-3, -1, 0, 2] {
                let value = add(product.clone(), q(offset));
                assert_eq!(
                    value.bounded_native_relation_rational(),
                    Some(Rational::from(offset + 1))
                );
                assert!(value.cached().is_none());
            }
            for sign in [-1, 1] {
                let atom = raw(Approximation::PrescaledSin(Computable::rational(
                    Rational::fraction(sign, 3).unwrap(),
                )));
                assert_eq!(
                    atom.sign_until(-32),
                    Some(if sign < 0 {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    })
                );
                let value = mul(add(q(1), atom), add(product.clone(), q(-1)));
                assert_eq!(
                    value.bounded_native_relation_rational(),
                    Some(Rational::zero())
                );
                assert!(value.cached().is_none());
            }
        }
    }

    #[test]
    fn equal_squares_do_not_certify_the_opposite_branch_or_a_perturbation() {
        let (a, b) = root_pair(2, true);
        let wrong_branch = add(mul(a.clone(), b), q(1));
        assert_eq!(
            wrong_branch.bounded_native_relation_rational(),
            Some(Rational::from(2))
        );
        let wrong_root = raw(Approximation::Sqrt(Computable::rational(
            Rational::fraction(1, 3).unwrap(),
        )));
        assert_eq!(wrong_root.sign_until(-32), Some(RealSign::Positive));
        let wrong = add(mul(a, wrong_root), q(-1));
        assert_eq!(wrong.bounded_native_relation_rational(), None);
        unproved(&wrong);
        for sign in [-1, 1] {
            let (a, b) = root_pair(2, true);
            let delta = Rational::fraction(sign, 1_000_000_000).unwrap();
            let value = add(add(mul(a, b), q(-1)), Computable::rational(delta.clone()));
            assert_eq!(value.bounded_native_relation_rational(), Some(delta));
        }
    }

    #[test]
    fn rational_root_domains_replay_without_numeric_root_caches() {
        let (a, b) = root_pair(2, false);
        assert_eq!(a.immediate_sign(), None);
        assert_eq!(b.immediate_sign(), None);
        let value = add(mul(a.clone(), b.clone()), q(-1));
        assert_eq!(
            value.bounded_native_relation_rational(),
            Some(Rational::zero())
        );
        unproved(&a);
        unproved(&b);
        assert!(value.cached().is_none());
    }

    #[test]
    fn unresolved_root_domain_declines_and_recovers_after_domain_certification() {
        let radicand = raw(Approximation::PrescaledSin(Computable::rational(
            Rational::fraction(1, 3).unwrap(),
        )));
        let a = raw(Approximation::Sqrt(radicand.clone()));
        let b = raw(Approximation::Sqrt(raw(Approximation::Inverse(
            radicand.clone(),
        ))));
        let value = add(mul(a.clone(), b.clone()), q(-1));
        assert_eq!(value.bounded_native_relation_rational(), None);
        unproved(&value);
        unproved(&radicand);
        unproved(&a);
        unproved(&b);
        assert_eq!(radicand.sign_until(-32), Some(RealSign::Positive));
        assert_eq!(
            value.bounded_native_relation_rational(),
            Some(Rational::zero())
        );
        unproved(&a);
        unproved(&b);
        assert!(value.cached().is_none());
    }

    #[test]
    fn root_completion_handoff_preserves_remaining_query_budgets() {
        let (a, b) = root_pair(2, false);
        let value = add(mul(a, b), q(-1));
        let mut recognition = RingRecognition::new(&value);
        recognition.native_relations = true;
        assert_eq!(recognition.recognize_native(&value), None);
        assert!(!recognition.atoms.is_empty());
        let nodes_left = recognition.nodes_left;
        let operations_left = recognition.operations_left;
        assert!(nodes_left < RingRecognition::MAX_NODES);
        assert!(operations_left < RingRecognition::MAX_OPERATIONS);
        recognition.begin_root_completion().unwrap();
        assert!(recognition.root_completion);
        assert_eq!(recognition.nodes_left, nodes_left);
        assert_eq!(recognition.operations_left, operations_left - 1);
        assert!(recognition.atoms.is_empty());
        assert!(recognition.quotients.is_empty());
        assert!(recognition.relations.is_empty());
        assert!(recognition.native_signs.is_empty());
        assert!(recognition.jacobi_bases.is_empty());
        assert!(recognition.parameter_atoms.is_empty());
        assert!(!recognition.root_relation_seen);
        assert_eq!(recognition.recognize_native(&value), Some(Rational::zero()));
        assert!(recognition.nodes_left < nodes_left);
        assert!(recognition.operations_left < operations_left - 1);
        // Query-local proofs alone never publish a global certificate.
        unproved(&value);
    }

    #[test]
    fn root_completion_cannot_reset_exhaustion_or_cancellation() {
        let (a, b) = root_pair(2, false);
        let mut value = add(mul(a, b), q(-1));
        let signal = Arc::new(AtomicBool::new(false));
        value.abort(signal.clone());
        for (nodes, operations, cancelled) in [(0, 128, false), (128, 0, false), (128, 128, true)] {
            let mut recognition = RingRecognition::new(&value);
            recognition.native_relations = true;
            recognition.nodes_left = nodes;
            recognition.operations_left = operations;
            signal.store(cancelled, std::sync::atomic::Ordering::Relaxed);
            let handoff = recognition.begin_root_completion();
            assert_eq!(handoff.is_some(), operations > 0 && !cancelled);
            assert_eq!(recognition.nodes_left, nodes);
            assert_eq!(
                recognition.operations_left,
                operations - usize::from(handoff.is_some())
            );
            assert_eq!(recognition.recognize_native(&value), None);
            unproved(&value);
        }
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(
            value.bounded_native_relation_rational(),
            Some(Rational::zero())
        );
        assert!(value.cached().is_none());
    }

    #[test]
    fn fixed_constant_intervals_prove_only_separated_polynomial_signs() {
        let pi = raw(Approximation::Constant(SharedConstant::Pi));
        let mut recognition = native(&pi);
        recognition.atom(&pi, 64).unwrap();
        for (power, offset, expected) in [
            (1, Rational::from(-3), Some(RealSign::Positive)),
            (1, Rational::from(-4), Some(RealSign::Negative)),
            (2, Rational::from(-10), Some(RealSign::Negative)),
            (1, -Rational::fraction(333, 106).unwrap(), None),
            (1, -Rational::fraction(355, 113).unwrap(), None),
            (
                1,
                -Rational::fraction(22, 7).unwrap(),
                Some(RealSign::Negative),
            ),
        ] {
            let polynomial = RingPolynomial {
                terms: vec![(vec![], offset), (vec![0; power], Rational::one())],
            };
            assert_eq!(recognition.native_polynomial_sign(&polynomial), expected);
        }
        let zero = RingPolynomial::default();
        assert_eq!(
            recognition.native_polynomial_sign(&zero),
            Some(RealSign::Zero)
        );
        // Shared constants use process-global numeric caches, which other
        // parallel tests may warm. This query must leave its own node cold.
        assert!(pi.internal.cached_value().is_none());
        assert!(pi.retained_rational_reduction().is_none());
        assert!(matches!(
            pi.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
    }

    #[test]
    fn domain_replay_is_conservative_and_never_refines_opaque_atoms() {
        for numerator in [-1, 0, 1] {
            for tangent in [false, true] {
                let argument = Rational::from(numerator);
                let value = raw(if tangent {
                    Approximation::PrescaledTanRational(argument)
                } else {
                    Approximation::PrescaledSinRational(argument)
                });
                assert_eq!(
                    native(&value).replayed_sign(&value, 64),
                    Some(public_sign(Rational::from(numerator).sign()))
                );
                unproved(&value);
            }
        }
        for approximation in [
            Approximation::PrescaledSinRational(Rational::from(2)),
            Approximation::PrescaledTanRational(Rational::from(-2)),
            Approximation::PrescaledSin(q(1)),
            Approximation::PrescaledTan(q(1)),
            Approximation::Sqrt(q(-1)),
            Approximation::Inverse(q(0)),
        ] {
            let value = raw(approximation);
            assert_eq!(native(&value).replayed_sign(&value, 64), None);
            unproved(&value);
        }
    }

    #[test]
    fn domain_replay_obeys_depth_node_work_coefficient_and_cancel_limits() {
        let mut value = raw(Approximation::Ratio(Rational::fraction(1, 3).unwrap()));
        let signal = Arc::new(AtomicBool::new(false));
        value.abort(signal.clone());
        for (nodes, operations, depth, cancelled) in [
            (512, 8192, 0, false),
            (0, 8192, 64, false),
            (512, 0, 64, false),
            (512, 8192, 64, true),
        ] {
            let mut recognition = native(&value);
            recognition.nodes_left = nodes;
            recognition.operations_left = operations;
            signal.store(cancelled, std::sync::atomic::Ordering::Relaxed);
            assert_eq!(recognition.replayed_sign(&value, depth), None);
            unproved(&value);
        }
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(
            native(&value).replayed_sign(&value, 64),
            Some(RealSign::Positive)
        );
        unproved(&value);
        let excessive = raw(Approximation::Int(BigInt::one() << 4096_usize));
        assert_eq!(native(&excessive).replayed_sign(&excessive, 64), None);
        unproved(&excessive);
    }

    #[test]
    fn sign_proofs_preserve_root_and_original_inverse_domain_guards() {
        let unknown = raw(Approximation::PrescaledSin(Computable::rational(
            Rational::fraction(1, 3).unwrap(),
        )));
        for radicand in [q(-1), unknown.clone()] {
            let bad_root = raw(Approximation::Sqrt(radicand));
            let value = add(mul(bad_root.clone(), bad_root), q(-1));
            assert_eq!(value.bounded_native_relation_rational(), None);
            unproved(&value);
        }
        let (a, b) = root_pair(2, true);
        let zero = add(mul(a, b), q(-1));
        for denominator in [q(0), unknown] {
            let value = mul(zero.clone(), raw(Approximation::Inverse(denominator)));
            assert_eq!(value.bounded_native_relation_rational(), None);
            unproved(&value);
        }
    }

    #[test]
    fn sign_square_attempt_obeys_work_cancellation_degree_and_coefficient_limits() {
        let (a, b) = root_pair(2, true);
        let mut value = add(mul(a, b), q(-1));
        let signal = Arc::new(AtomicBool::new(false));
        value.abort(signal.clone());
        let mut recognition = native(&value);
        let expanded = recognition
            .expand_quotient(&value, RingRecognition::MAX_DEPTH)
            .unwrap();
        let polynomial = expanded.numerator;
        recognition.operations_left = 0;
        assert!(
            recognition
                .prove_zero_by_nonnegative_squares(&polynomial)
                .is_none()
        );
        unproved(&value);
        recognition.operations_left = RingRecognition::MAX_OPERATIONS;
        signal.store(true, std::sync::atomic::Ordering::Relaxed);
        assert!(
            recognition
                .prove_zero_by_nonnegative_squares(&polynomial)
                .is_none()
        );
        unproved(&value);
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        let excessive_degree = RingPolynomial {
            terms: vec![(vec![0; 7], Rational::one())],
        };
        assert!(
            recognition
                .prove_zero_by_nonnegative_squares(&excessive_degree)
                .is_none()
        );
        let excessive_coefficient = RingPolynomial {
            terms: vec![(vec![], Rational::from_bigint(BigInt::one() << 2048_usize))],
        };
        assert!(
            recognition
                .prove_zero_by_nonnegative_squares(&excessive_coefficient)
                .is_none()
        );
        unproved(&value);
        assert!(
            recognition
                .prove_zero_by_nonnegative_squares(&polynomial)
                .is_some()
        );
        unproved(&value);
        assert_eq!(
            value.bounded_native_relation_rational(),
            Some(Rational::zero())
        );
    }

    #[test]
    fn zero_sides_need_no_nonzero_conjugate_or_division() {
        let zero_root = raw(Approximation::Sqrt(q(0)));
        assert_eq!(zero_root.sign_until(-32), Some(RealSign::Zero));
        let mut recognition = native(&zero_root);
        let polynomial = recognition
            .atom(&zero_root, RingRecognition::MAX_DEPTH)
            .unwrap();
        assert!(
            recognition
                .prove_zero_by_nonnegative_squares(&polynomial)
                .is_some()
        );
        assert!(
            recognition
                .prove_zero_by_nonnegative_squares(&RingPolynomial::default())
                .is_some()
        );
    }

    #[test]
    fn rational_root_products_collapse_before_the_retained_degree_limit() {
        for (numerator, denominator) in [(0, 1), (1, 3), (2, 1), (3, 1), (7, 5)] {
            let radicand =
                Computable::rational(Rational::fraction(numerator, denominator).unwrap());
            assert_eq!(
                radicand.exact_sign(),
                Some(if numerator == 0 {
                    Sign::NoSign
                } else {
                    Sign::Plus
                })
            );
            let root = raw(Approximation::Sqrt(radicand));
            let pi = Computable::pi();
            let mut recognition = native(&root);
            recognition.atom(&pi, 64).unwrap();
            recognition.atom(&root, 64).unwrap();
            assert!(
                recognition.relations[1].is_some(),
                "the rational root must be admitted first"
            );
            for exponent in 2..=12 {
                let a = [vec![0; 10], vec![1, 1]].concat();
                let b = vec![1; exponent - 2];
                let (term, coefficient) = recognition
                    .multiply_native_root_terms(&a, &b, Rational::one())
                    .unwrap();
                let expected = Rational::fraction(numerator, denominator)
                    .unwrap()
                    .powi_i64((exponent / 2) as i64)
                    .unwrap();
                assert_eq!(coefficient, expected);
                assert_eq!(term, [vec![0; 10], vec![1; exponent % 2]].concat());
                assert!(term.len() <= recognition.degree_limit());
            }
            let excessive = vec![0; 12];
            assert!(
                recognition
                    .multiply_native_root_terms(&excessive, &[0], Rational::one())
                    .is_none()
            );
        }
    }

    #[test]
    fn early_root_products_keep_domain_coefficient_and_work_guards() {
        let good = raw(Approximation::Sqrt(q(2)));
        let unknown = raw(Approximation::PrescaledSin(Computable::rational(
            Rational::fraction(1, 3).unwrap(),
        )));
        for radicand in [q(-1), unknown] {
            let root = raw(Approximation::Sqrt(radicand));
            let mut recognition = native(&root);
            recognition.atom(&good, 64).unwrap();
            recognition.atom(&root, 64).unwrap();
            assert!(recognition.relations[1].is_none());
            let (term, coefficient) = recognition
                .multiply_native_root_terms(&[1], &[1], Rational::one())
                .unwrap();
            assert_eq!(term, vec![1, 1]);
            assert_eq!(coefficient, Rational::one());
            assert!(
                recognition
                    .multiply_native_root_terms(&[1; 12], &[1], Rational::one())
                    .is_none()
            );
        }
        let mut zero_root = raw(Approximation::Sqrt(q(0)));
        let signal = Arc::new(AtomicBool::new(false));
        zero_root.abort(signal.clone());
        let mut recognition = native(&zero_root);
        recognition.atom(&zero_root, 64).unwrap();
        let excessive = Rational::from_bigint(BigInt::one() << 4096_usize);
        assert!(
            recognition
                .multiply_native_root_terms(&[0], &[0], excessive)
                .is_none()
        );
        recognition.operations_left = 0;
        assert!(
            recognition
                .multiply_native_root_terms(&[0], &[0], Rational::one())
                .is_none()
        );
        recognition.operations_left = RingRecognition::MAX_OPERATIONS;
        signal.store(true, std::sync::atomic::Ordering::Relaxed);
        assert!(
            recognition
                .multiply_native_root_terms(&[], &[], Rational::one())
                .is_none()
        );
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(
            recognition
                .multiply_native_root_terms(&[0], &[0], Rational::one())
                .unwrap(),
            (vec![], Rational::zero())
        );
    }

    #[test]
    fn cold_arithmetic_sign_replay_is_exact_and_conservative() {
        let one = raw(Approximation::One);
        assert_eq!(
            native(&one).replayed_sign(&one, 64),
            Some(RealSign::Positive)
        );
        unproved(&one);
        for a in [-3_i32, 0, 2] {
            let x = raw(Approximation::Int(BigInt::from(a)));
            for (value, expected) in [
                (raw(Approximation::Negate(x.clone())), -a),
                (raw(Approximation::Offset(x.clone(), -100)), a),
                (raw(Approximation::Square(x.clone())), a * a),
            ] {
                assert_eq!(
                    native(&value).replayed_sign(&value, 64),
                    Some(public_sign(Rational::from(expected).sign()))
                );
                unproved(&value);
                unproved(&x);
            }
            for b in [-2_i32, 0, 5] {
                let y = raw(Approximation::Int(BigInt::from(b)));
                let product = mul(x.clone(), y.clone());
                assert_eq!(
                    native(&product).replayed_sign(&product, 64),
                    Some(public_sign(Rational::from(a * b).sign()))
                );
                let sum = add(x.clone(), y.clone());
                let expected = if a == 0 || b == 0 || a.signum() == b.signum() {
                    Some(public_sign(Rational::from(a + b).sign()))
                } else {
                    None
                };
                assert_eq!(native(&sum).replayed_sign(&sum, 64), expected);
                unproved(&product);
                unproved(&sum);
                unproved(&x);
                unproved(&y);
            }
        }
    }

    #[test]
    fn mixed_nonconstant_signs_and_oversized_interval_coefficients_decline() {
        let sine = raw(Approximation::PrescaledSinRational(
            Rational::fraction(1, 3).unwrap(),
        ));
        let mut recognition = native(&sine);
        recognition.atom(&sine, 64).unwrap();
        let polynomial = RingPolynomial {
            terms: vec![(vec![], -Rational::one()), (vec![0], Rational::one())],
        };
        assert_eq!(recognition.native_polynomial_sign(&polynomial), None);
        unproved(&sine);

        let pi = raw(Approximation::Constant(SharedConstant::Pi));
        let mut recognition = native(&pi);
        recognition.atom(&pi, 64).unwrap();
        let large = Rational::from_bigint(BigInt::one() << 4090_usize);
        assert!(RingRecognition::coefficient_fits(&large));
        let scaled = RingPolynomial {
            terms: vec![(vec![], -Rational::one()), (vec![0], large)],
        };
        assert_eq!(recognition.native_polynomial_sign(&scaled), None);
        let a = Rational::from_bigint((BigInt::one() << 1400_usize) - 1)
            .inverse()
            .unwrap();
        let b = Rational::from_bigint((BigInt::one() << 1400_usize) + 1)
            .inverse()
            .unwrap();
        assert!(RingRecognition::coefficient_fits(&a));
        assert!(RingRecognition::coefficient_fits(&b));
        let sum = RingPolynomial {
            terms: vec![(vec![], -a), (vec![0], b)],
        };
        assert_eq!(recognition.native_polynomial_sign(&sum), None);
        assert!(pi.internal.cached_value().is_none());
    }

    #[test]
    fn zero_root_domain_replays_but_does_not_admit_a_zero_inverse() {
        let zero = add(q(1), q(-1));
        let root = raw(Approximation::Sqrt(zero.clone()));
        let mut recognition = native(&root);
        let relation = recognition.completed_root_relation(&root, 64).unwrap();
        assert_eq!(relation.root_sign, Some(RealSign::Zero));
        unproved(&root);
        unproved(&zero);
        let inverse = raw(Approximation::Inverse(zero));
        assert!(native(&inverse).expand_quotient(&inverse, 64).is_none());
        unproved(&inverse);
    }

    #[test]
    fn quotient_replay_preserves_linear_forms_and_inverse_guards() {
        for root_completion in [false, true] {
            for constant in [0, 5] {
                let root = raw(Approximation::Sqrt(q(2)));
                let value = raw(Approximation::LinearCombination3(Box::new(
                    LinearCombination3 {
                        coefficients: [root.clone(), root, q(1)],
                        values: [
                            Rational::from(3),
                            Rational::from(-3),
                            Rational::from(constant),
                        ],
                    },
                )));
                let inverse = raw(Approximation::Inverse(value.clone()));
                let mut recognition = native(&inverse);
                recognition.root_completion = root_completion;
                let quotient = recognition.expand_quotient(&inverse, 64);
                if constant == 0 {
                    assert!(quotient.is_none());
                } else {
                    assert_eq!(
                        recognition.project_native_relations(quotient.unwrap()),
                        Some(Rational::fraction(1, 5).unwrap())
                    );
                }
                unproved(&value);
                unproved(&inverse);
                assert!(recognition.expand_quotient_uncached(&value, 0).is_none());
            }
            let excessive_shift = raw(Approximation::Offset(q(1), 4096));
            let mut recognition = native(&excessive_shift);
            recognition.root_completion = root_completion;
            assert!(recognition.expand_quotient(&excessive_shift, 64).is_none());
            unproved(&excessive_shift);
        }
    }
}
