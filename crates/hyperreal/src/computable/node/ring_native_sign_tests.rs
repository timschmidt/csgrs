#[cfg(test)]
/// Qualification of query-local sign replay, root-branch completion and preserved inverse-domain guards.
mod ring_native_sign_tests {
    use super::*;

    /// Build a native recipe with uninitialized bounds/signs and no stop handle.
    /// Bypass constructor simplification to expose admission and query-local proof behavior.
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

    /// Construct an exact rational integer fixture through the ordinary computable constructor.
    fn q(value: i32) -> Computable {
        Computable::rational(Rational::from(value))
    }

    /// Retain an unsimplified sum of two fixtures with fresh unproved parent facts.
    fn add(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, b))
    }

    /// Retain an unsimplified product of two fixtures with fresh unproved parent facts.
    fn mul(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Multiply(a, b))
    }

    /// Build principal sqrt(n) and sqrt(1/n), optionally certifying their signs first.
    /// Tests supply positive n; the reciprocal conversion and sign assertions panic
    /// if those fixture assumptions fail. Their product is independently exactly one.
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

    /// Create a fresh recognition attempt with native relations and root completion enabled.
    /// Keep the standard limits and borrow the fixture's stop handle.
    fn native(value: &Computable) -> RingRecognition<'_> {
        let mut recognition = RingRecognition::new(value);
        recognition.native_relations = true;
        recognition.root_completion = true;
        recognition
    }

    /// Require no exact-sign slot, numeric sample or retained rational reduction on this fixture.
    /// This does not inspect every cache/fact field or establish that the value is
    /// mathematically unknown; shared constant samples require separate handling.
    fn unproved(value: &Computable) {
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        assert!(value.cached().is_none());
        assert!(value.retained_rational_reduction().is_none());
    }

    #[test]
    /// Project twelve shifted reciprocal-root products and six signed-factor zero identities.
    /// Positive branch certification makes sqrt(n)*sqrt(1/n)=1 for n=2,3,7.
    /// Require exact rational results and no numeric sample on each composite parent.
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
    /// Distinguish the positive reciprocal-root product plus one from zero and reject a mismatched root.
    /// Both signed one-billionth perturbations must project to their exact rational
    /// value; equal-square reasoning must not discard branch or perturbation information.
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
    /// Prove sqrt(2)*sqrt(1/2)-1 exactly zero from initially unproved rational root domains.
    /// The root nodes retain unproved global state while the composite gains its
    /// completed reduction, with no numeric sample on the composite.
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
    /// Decline reciprocal roots of an opaque sine until its positive domain is certified.
    /// Retry the same composite successfully after sign_until supplies that fact;
    /// root children remain globally unproved and the composite numeric cache stays empty.
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
    /// Check the first native attempt declines, then root completion uses its remaining allowance.
    /// Require one operation charged for handoff, unchanged node count, cleared
    /// strategy tables and successful query-local rational projection. The root
    /// receives no global certificate from these direct helper calls.
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
    /// Exercise exhausted nodes, exhausted operations and a stop handle across strategy handoff.
    /// A handoff may succeed with zero nodes but subsequent recognition still declines.
    /// Require retained counters and no global proof, then a fresh unstopped public
    /// attempt must complete the original identity without a numeric sample.
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
    /// Check six pi polynomials and the empty polynomial using fixed rational constant bounds.
    /// The two close convergents remain unresolved while coarse separated cases get
    /// exact signs. Assert the pi node's own caches/facts stay cold, without assuming
    /// its process-wide numeric cache is cold under concurrent tests.
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
    /// Replay six unit-range rational sine/tangent signs and decline six unsupported or invalid recipes.
    /// Require no published sign, sample or retained rational reduction on the tested
    /// root nodes; a decline is not by itself proof of invalidity.
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
    /// Independently exhaust depth, nodes, operations and cancellation on an unproved rational leaf.
    /// Reset the stop and require a fresh successful query; also reject a 4097-bit
    /// integer. Check that all such query-local attempts leave global proof state absent.
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
    /// Reject negative/unproved square-root domains and zero/unproved inverse domains.
    /// A factor independently equal to zero must not hide the original inverse
    /// obligation; each failed composite remains globally unproved.
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
    /// Decline signed-square proofs on work/stop exhaustion and on degree/coefficient growth.
    /// Reset scratch limits, prove the original identity locally without publication,
    /// then require the public projection to return exact zero. Explicit scratch
    /// resets are test setup, not automatic production budget replenishment.
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
    /// Prove the already-certified zero-root polynomial and the empty polynomial zero.
    /// The sufficient nonnegative-square argument must include zero sides without
    /// requiring a nonzero conjugate or numerical division.
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
    /// Check 55 monomial products against exact powers of five nonnegative rational radicands.
    /// Collapse root pairs before testing the retained degree-12 bound, preserving
    /// ten unrelated pi factors and root parity. An unrelated thirteenth factor
    /// must still exceed the cap, including for the zero-radicand fixture.
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
    /// Keep negative/unproved roots opaque and enforce retained-degree limits on their powers.
    /// For a valid zero root, reject excessive coefficients, spent work and a stop
    /// before accepting its exact zero product after explicit scratch reset.
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
    /// Check a cold unit, nine unary cases and nine products/sums against integer signs.
    /// Opposite nonzero summands deliberately decline; supported cases must preserve
    /// unproved global state on each checked parent and operand.
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
    /// Decline mixed sine terms and fixed-constant interval arithmetic that exceeds coefficient caps.
    /// Input coefficients individually fit but products or sums need not; require
    /// no numeric refinement on the tested sine or pi node.
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
    /// Admit a principal root of an exactly cancelling sum while rejecting its reciprocal.
    /// Require a zero root-sign relation, with no global proof published by either
    /// direct query-local helper.
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
    /// In both native strategies, cancel two equal root terms before inverse-domain admission.
    /// An exact residual five gives 1/5 while zero rejects inversion; depth zero and
    /// shift 4096 also decline. Direct helper calls must leave the fixtures unproved.
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
