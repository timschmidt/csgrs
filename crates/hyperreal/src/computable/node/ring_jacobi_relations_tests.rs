#[cfg(test)]
/// Native Jacobi proof obligations, retained scalar composition and resource regressions.
mod ring_jacobi_relations_tests {
    use super::*;
    use crate::Real;

    /// Build an unsimplified node with invalid bound/sign facts, no sample and no stop signal.
    /// Fixtures may deliberately contain unproved domains; no validation occurs here.
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

    /// Build a raw sum preserving operand order and shared child nodes.
    fn add(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, b))
    }
    /// Build a raw product without simplifying factors or erasing domain obligations.
    fn mul(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Multiply(a, b))
    }
    /// Wrap an operand in raw negation without canonicalization.
    fn neg(a: Computable) -> Computable {
        raw(Approximation::Negate(a))
    }
    /// Wrap a shared operand in a raw square recipe without publishing its sign.
    fn sq(a: Computable) -> Computable {
        raw(Approximation::Square(a))
    }
    /// Create an exact rational i32 through the normal Computable rational constructor.
    /// Unlike raw Ratio fixtures, this may carry the constructor's exact facts.
    fn q(n: i32) -> Computable {
        Computable::rational(Rational::from(n))
    }
    /// Build an unsampled sine of exactly 1/3 for use as a retained Jacobi amplitude.
    /// The raw wrapper initially has no sign proof, although its positivity can be proved
    /// later by the bounded exact small-angle rules without a numerical sample.
    fn fresh_angle() -> Computable {
        raw(Approximation::PrescaledSin(Computable::rational(
            Rational::fraction(1, 3).unwrap(),
        )))
    }
    /// Start a query-local recognizer with native relations enabled and default budgets.
    /// This helper does not itself run parameter collection or enable root completion.
    fn native(value: &Computable) -> RingRecognition<'_> {
        let mut recognition = RingRecognition::new(value);
        recognition.native_relations = true;
        recognition
    }

    /// Build sn²+cn² from the same retained amplitude and log-complement payloads.
    /// For defined real payloads this equals one; the helper does not admit their domains.
    fn unit_relation(angle: &Computable, log: &Computable) -> Computable {
        add(
            sq(angle.clone().jacobi_component(log.clone(), 0)),
            sq(angle.clone().jacobi_component(log.clone(), 1)),
        )
    }

    /// Require an invalid queried exact sign and no numerical approximation sample.
    /// This is not an inventory of every other cache entry or descendant fact.
    fn assert_unproved(value: &Computable) {
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        assert!(value.cached().is_none());
    }

    #[test]
    /// Reject candidate-only rational values for individual components and misleading sums.
    /// Prove sn²+cn²=1 only with its relation, then prove cn*(sn²+cn²-1)=0 without
    /// assuming cn nonzero. The shared amplitude must remain numerically unsampled.
    fn candidate_assignments_never_replace_a_relation_proof() {
        let angle = fresh_angle();
        let log = Computable::ln2();
        for kind in 0..3 {
            let value = angle.clone().jacobi_component(log.clone(), kind);
            assert_eq!(value.bounded_native_relation_rational(), None);
        }
        let sn = angle.clone().jacobi_component(log.clone(), 0);
        let cn = angle.clone().jacobi_component(log.clone(), 1);
        let misleading = add(sn, cn.clone());
        assert_eq!(misleading.bounded_native_relation_rational(), None);
        assert_unproved(&misleading);
        assert_eq!(
            mul(q(2), cn.clone()).bounded_native_relation_rational(),
            None
        );
        let norm = unit_relation(&angle, &log);
        assert_eq!(
            norm.bounded_native_relation_rational(),
            Some(Rational::one())
        );
        // This factor is allowed to vanish: discarding it is only a sufficient
        // zero proof, never a license to invert it or assert a nonzero value.
        let zero = mul(cn, add(norm, q(-1)));
        assert_eq!(
            zero.bounded_native_relation_rational(),
            Some(Rational::zero())
        );
        assert!(angle.cached().is_none());
    }

    #[test]
    /// Decline sn²+cn² mixtures with a changed amplitude or log-complement parameter.
    /// A shared component name alone must not justify importing another component's relation.
    fn amplitude_and_parameter_identity_are_both_required() {
        let angle = fresh_angle();
        let log = Computable::ln2();
        let sn = angle.clone().jacobi_component(log.clone(), 0);
        for (other_angle, other_log) in [
            (q(2), log.clone()),
            (angle, Computable::ln_constant(3).unwrap()),
        ] {
            let wrong = add(
                sq(sn.clone()),
                sq(other_angle.jacobi_component(other_log, 1)),
            );
            assert_eq!(wrong.bounded_native_relation_rational(), None);
            assert_unproved(&wrong);
        }
    }

    #[test]
    /// Check immediate radicand evidence and retained-logarithm domain admission.
    /// Reject negative/unproved roots and invalid/unproved logarithm recipes, but admit
    /// zero's square relation. Preserve the original sin(1/3) logarithm case: exact
    /// positivity admits h=1+sin(1/3), yielding dn²=1+sin(1/3)*sn² without sampling.
    /// A separately proved positive sqrt(2) may then be consumed by its reciprocal.
    fn root_and_logarithm_rules_require_domain_evidence() {
        let angle = fresh_angle();
        let mut recognition = native(&angle);
        for radicand in [q(-1), fresh_angle()] {
            let root = raw(Approximation::Sqrt(radicand));
            assert!(recognition.native_square_relation(&root, 64).is_none());
        }
        let zero_root = raw(Approximation::Sqrt(q(0)));
        assert!(
            recognition
                .native_square_relation(&zero_root, 64)
                .unwrap()
                .numerator
                .terms
                .is_empty()
        );
        for log in [
            raw(Approximation::PrescaledLnRational(Rational::from(-2))),
            raw(Approximation::PrescaledLn(raw(
                Approximation::PrescaledSin(Computable::e_constant()),
            ))),
        ] {
            let dn = angle.clone().jacobi_component(log, 2);
            assert!(recognition.native_square_relation(&dn, 64).is_none());
        }
        // Retain the original sin(1/3) case. Its sign now has a bounded
        // exact proof: 0 < 1/3 < 1 < pi/2, so 1+sin(1/3)>1. Replaying that
        // domain must admit the logarithm and its Jacobi square relation.
        let residual = fresh_angle();
        let log = raw(Approximation::PrescaledLn(residual.clone()));
        let complement = log.retained_log_argument(&mut 64).unwrap();
        assert!(Computable::internal_structural_eq(
            &complement,
            &residual.clone().add(q(1)),
        ));
        assert_eq!(residual.immediate_sign(), Some(RealSign::Positive));
        let dn = angle.clone().jacobi_component(log.clone(), 2);
        assert!(recognition.native_square_relation(&dn, 64).is_some());
        let sn = angle.clone().jacobi_component(log, 0);
        // h=exp(log)=1+residual and dn²=1-(1-h)sn².
        let identity = add(add(sq(dn), q(-1)), neg(mul(residual.clone(), sq(sn))));
        assert_eq!(
            identity.bounded_native_relation_rational(),
            Some(Rational::zero())
        );
        assert!(residual.cached().is_none());
        let root = raw(Approximation::Sqrt(q(2)));
        assert_eq!(root.sign_until(-32), Some(RealSign::Positive));
        let identity = mul(root.clone(), raw(Approximation::Inverse(root)));
        assert_eq!(
            identity.bounded_native_relation_rational(),
            Some(Rational::one())
        );
        assert!(angle.cached().is_none());
    }

    #[test]
    /// Prove exp(a)*exp(-a+k*ln(2))=2^k for k=-3,0,3 through expanded products.
    /// The (exp(a)+1)*b-b form exposes both exponential atoms. An unmatched -2*a
    /// exponent must decline without a root proof or amplitude sample.
    fn inverse_exponential_rules_admit_only_exact_log_sums() {
        for shift in [-3, 0, 3] {
            let angle = fresh_angle();
            let a = raw(Approximation::PrescaledExp(angle.clone()));
            let b = raw(Approximation::PrescaledExp(add(
                neg(angle.clone()),
                mul(q(shift), Computable::ln2()),
            )));
            // (a+1)b-b = ab. Expanded arithmetic exposes both exponential
            // atoms, independently of the optional opaque-product shortcut.
            let product = add(mul(add(a.clone(), q(1)), b.clone()), neg(b));
            assert_eq!(
                product.bounded_native_relation_rational(),
                Some(Computable::power_of_two_rational(shift))
            );
            let wrong = raw(Approximation::PrescaledExp(neg(mul(q(2), angle.clone()))));
            let product = add(mul(add(a, q(1)), wrong.clone()), neg(wrong));
            assert_eq!(product.bounded_native_relation_rational(), None);
            assert_unproved(&product);
            assert!(angle.cached().is_none());
        }
    }

    #[test]
    /// Keep original inverse guards even when multiplying by a proved Jacobi norm.
    /// Literal zero, an unproved sine and a cancelled zero divisor all prevent d/d
    /// from yielding a completed native proof.
    fn native_relations_never_discharge_unproved_original_inverses() {
        let angle = fresh_angle();
        let norm = unit_relation(&angle, &Computable::ln2());
        for denominator in [q(0), fresh_angle(), add(angle.clone(), neg(angle))] {
            let identity = mul(
                denominator.clone(),
                raw(Approximation::Inverse(denominator)),
            );
            let value = mul(norm.clone(), identity);
            assert_eq!(value.bounded_native_relation_rational(), None);
            assert_unproved(&value);
        }
    }

    #[test]
    /// Check node/work/depth refusal, cancellation, retained rational reuse and four readers.
    /// After retry proves the norm one, verify its retained rational reduction, then
    /// require zero-work and stopped attempts to decline despite that cache. Four
    /// scoped threads may subsequently recover the same value without numerical samples;
    /// this finite concurrency check is not a general race-freedom proof.
    fn native_limits_and_abort_leave_no_false_fact_and_allow_recovery() {
        let angle = fresh_angle();
        let mut value = unit_relation(&angle, &Computable::ln2());
        let mut nodes = native(&value);
        nodes.nodes_left = 1;
        assert!(nodes.collect_parameter_atoms(&value).is_none());
        let mut work = native(&value);
        work.operations_left = 1;
        assert!(work.expand_quotient(&value, 64).is_none());
        assert!(native(&value).expand_quotient(&value, 1).is_none());
        assert_unproved(&value);
        let signal = Arc::new(AtomicBool::new(true));
        value.abort(signal.clone());
        assert_eq!(value.bounded_native_relation_rational(), None);
        assert_unproved(&value);
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(
            value.bounded_native_relation_rational(),
            Some(Rational::one())
        );
        assert_eq!(value.retained_rational_reduction(), Some(Rational::one()));
        let mut exhausted = native(&value);
        exhausted.operations_left = 0;
        assert!(exhausted.expand_quotient(&value, 64).is_none());
        signal.store(true, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(value.bounded_native_relation_rational(), None);
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        std::thread::scope(|scope| {
            for _ in 0..4 {
                let value = &value;
                scope.spawn(move || {
                    assert_eq!(
                        value.bounded_native_relation_rational(),
                        Some(Rational::one())
                    )
                });
            }
        });
        assert!(value.cached().is_none());
        assert!(angle.cached().is_none());
    }

    #[test]
    /// Keep one basis choice per structurally equal log parameter within each attempt.
    /// Test cold-first and warm-first insertion separately, requiring one table entry
    /// and reuse within each. The test does not require both attempts to choose the
    /// same basis; warming may change the first choice while preserving its consistency.
    fn native_basis_is_stable_across_differently_warmed_equal_parameters() {
        let a = raw(Approximation::PrescaledLn(q(1)));
        let b = raw(Approximation::PrescaledLn(q(1)));
        assert_eq!(a.immediate_sign(), None);
        assert_eq!(b.sign_until(-32), Some(RealSign::Positive));
        assert!(Computable::internal_structural_eq(&a, &b));
        let mut recognition = native(&a);
        let cold = recognition.jacobi_cosine_basis(&a).unwrap();
        assert_eq!(recognition.jacobi_cosine_basis(&b), Some(cold));
        assert_eq!(recognition.jacobi_bases.len(), 1);
        let mut recognition = native(&b);
        let warm = recognition.jacobi_cosine_basis(&b).unwrap();
        assert_eq!(recognition.jacobi_cosine_basis(&a), Some(warm));
        assert_eq!(recognition.jacobi_bases.len(), 1);
    }

    #[test]
    /// Discover parameter atoms only after hidden arithmetic inverse guards are discharged.
    /// Exercise zero, initially unproved sine and shifted-sine divisors; later positive
    /// sign proofs may admit a compressed identity and copies must reuse one entry.
    /// Also check shallow arithmetic rejection and a 64-negation prepass depth failure.
    /// The amplitude sample remains absent; not every valid-domain attempt must compress.
    fn parameter_compression_checks_hidden_inverse_domains_and_depth() {
        let angle = fresh_angle();
        for denominator in [q(0), fresh_angle(), add(q(4), fresh_angle())] {
            let quotient = mul(
                angle.clone(),
                raw(Approximation::Inverse(denominator.clone())),
            );
            let log = raw(Approximation::PrescaledLn(raw(Approximation::Offset(
                neg(quotient.clone()),
                -2,
            ))));
            let mut recognition = native(&log);
            recognition.parameter_from_logarithm(&log, 8).unwrap();
            assert!(recognition.parameter_atoms.is_empty());
            assert!(
                recognition
                    .guard_parameter_arithmetic(&quotient, 1)
                    .is_none()
            );
            if denominator.sign_until(-32) == Some(RealSign::Positive) {
                recognition.parameter_from_logarithm(&log, 8).unwrap();
                assert_eq!(recognition.parameter_atoms.len(), 1);
                assert_eq!(recognition.is_parameter_atom(&quotient), Some(true));
                recognition
                    .parameter_from_logarithm(&log.clone(), 8)
                    .unwrap();
                assert_eq!(
                    recognition.parameter_atoms.len(),
                    1,
                    "copies reuse an admitted identity"
                );
                let root = angle.clone().jacobi_component(log, 0);
                let mut scan = native(&root);
                scan.collect_parameter_atoms(&root).unwrap();
                assert_eq!(scan.is_parameter_atom(&quotient), Some(true));
            }
        }
        assert!(angle.cached().is_none());
        let mut deep = angle.clone();
        for _ in 0..64 {
            deep = raw(Approximation::Negate(deep));
        }
        assert!(native(&deep).collect_parameter_atoms(&deep).is_none());
    }

    #[test]
    /// Exercise separate capacities: 32 atoms/bases, 64 terms, degree twelve and coefficient bits.
    /// Reject each next entry, degree-thirteen multiplication/powers, an oversized exact
    /// integer and both extreme dyadic shifts. Formal insertion probes use synthetic
    /// atom IDs to isolate storage guards; they are not completed geometric admissions.
    /// Raw sine atoms at integer inputs are not numerically evaluated by this test.
    fn native_atom_term_degree_and_coefficient_limits_remain_independent() {
        let root = fresh_angle();
        let mut recognition = native(&root);
        for index in 0..32 {
            let atom = raw(Approximation::PrescaledSin(q(index)));
            assert!(
                recognition
                    .native_relation_atom(std::borrow::Cow::Owned(atom), 64)
                    .is_some()
            );
        }
        assert!(
            recognition
                .native_relation_atom(
                    std::borrow::Cow::Owned(raw(Approximation::PrescaledSin(q(33)))),
                    64
                )
                .is_none()
        );
        let mut recognition = native(&root);
        let mut polynomial = RingPolynomial::default();
        let terms: Vec<_> = (0_u8..12)
            .flat_map(|a| (a..12).map(move |b| vec![a, b]))
            .collect();
        for term in &terms[..64] {
            recognition
                .insert(&mut polynomial, term.clone(), Rational::one())
                .unwrap();
        }
        assert!(
            recognition
                .insert(&mut polynomial, terms[64].clone(), Rational::one())
                .is_none()
        );
        let mut polynomial = RingPolynomial::default();
        recognition
            .insert(&mut polynomial, vec![0; 12], Rational::one())
            .unwrap();
        assert!(
            recognition
                .insert(&mut polynomial, vec![0; 13], Rational::one())
                .is_none()
        );
        assert!(
            recognition
                .insert(
                    &mut polynomial,
                    vec![1],
                    Rational::from_bigint(BigInt::one() << 4096_usize)
                )
                .is_none()
        );
        let atom = RingPolynomial {
            terms: vec![(vec![0], Rational::one())],
        };
        assert!(recognition.multiply(&polynomial, &atom).is_none());
        assert!(recognition.polynomial_powers(&atom, 13).is_none());
        for shift in [i32::MIN, i32::MAX] {
            let value = raw(Approximation::Offset(root.clone(), shift));
            assert_eq!(value.bounded_native_relation_rational(), None);
            assert_unproved(&value);
        }
        let mut recognition = native(&root);
        for residual in 0..32 {
            let log = raw(Approximation::PrescaledLnRational(Rational::from(residual)));
            assert!(recognition.jacobi_cosine_basis(&log).is_some());
        }
        let log = raw(Approximation::PrescaledLnRational(Rational::from(33)));
        assert!(recognition.jacobi_cosine_basis(&log).is_none());
    }

    #[test]
    /// Replay derived coefficient arithmetic to 3*2+2*3²+(-4)/2=22 exactly.
    /// Reuse a retained unit norm, reject three zero inverse forms, and admit inverse
    /// (1+1) by a local proof. Check oversized/shifted inputs, depth-zero declines,
    /// scalar-product shape refusal and the built-in sqrt(2)/sqrt(3) square relations.
    /// No numerical sample is needed for the replayed root or rejected inverse recipes.
    fn owned_coefficient_replay_preserves_values_domains_and_limits() {
        let root = fresh_angle();
        let value = Computable::linear_combination3(
            [
                q(2),
                sq(add(q(2), q(1))),
                raw(Approximation::Offset(neg(q(1)), 2)),
            ],
            [
                Rational::from(3),
                Rational::from(2),
                Rational::fraction(1, 2).unwrap(),
            ],
        );
        let mut recognition = native(&root);
        let replayed = recognition.expand_relation_value(&value, 64).unwrap();
        assert_eq!(
            recognition.project_quotient(&replayed),
            Some(Rational::from(22))
        );
        assert!(value.cached().is_none());
        let norm = unit_relation(&root, &Computable::ln2());
        assert_eq!(
            norm.bounded_native_relation_rational(),
            Some(Rational::one())
        );
        let replayed = recognition.expand_relation_value(&norm, 64).unwrap();
        assert_eq!(
            recognition.project_quotient(&replayed),
            Some(Rational::one())
        );
        for denominator in [
            q(0),
            raw(Approximation::Int(BigInt::zero())),
            add(q(1), q(-1)),
        ] {
            let inverse = raw(Approximation::Inverse(denominator));
            assert!(native(&root).expand_relation_value(&inverse, 64).is_none());
            assert_unproved(&inverse);
        }
        let denominator = add(q(1), q(1));
        assert_eq!(denominator.immediate_sign(), None);
        let inverse = raw(Approximation::Inverse(denominator));
        let replayed = recognition.expand_relation_value(&inverse, 64).unwrap();
        assert_eq!(
            recognition.project_quotient(&replayed),
            Some(Rational::fraction(1, 2).unwrap())
        );
        for value in [
            raw(Approximation::Int(BigInt::one() << 4096_usize)),
            raw(Approximation::Offset(q(1), i32::MAX)),
        ] {
            assert!(native(&root).expand_relation_value(&value, 64).is_none());
            assert_unproved(&value);
        }
        assert!(recognition.expand_relation_value(&value, 0).is_none());
        assert!(
            recognition
                .native_square_relation(&sq(root.clone()), 0)
                .is_none()
        );
        assert_eq!(recognition.scalar_product_shape(&root, 0), Some(false));
        for (constant, expected) in [(SharedConstant::Sqrt2, 2), (SharedConstant::Sqrt3, 3)] {
            let value = raw(Approximation::Constant(constant));
            let relation = recognition.native_square_relation(&value, 64).unwrap();
            assert_eq!(
                recognition.project_quotient(&relation),
                Some(Rational::from(expected))
            );
        }
    }

    #[test]
    /// Traverse square/negation/offset/linear-form wrappers before compressing a quotient.
    /// Denominator four is admitted and zero is not. A depth-zero log prepass records
    /// nothing; scanning a containing Jacobi linear form records only the valid parameter
    /// atom. The consumer remains without a numerical sample.
    fn parameter_prepass_checks_inverse_domains_through_linear_combinations() {
        for denominator in [0, 4] {
            let denominator = q(denominator);
            let valid = denominator.exact_sign() == Some(Sign::Plus);
            let quotient = mul(fresh_angle(), raw(Approximation::Inverse(denominator)));
            let wrapped = sq(neg(raw(Approximation::Offset(quotient.clone(), 1))));
            let combination = Computable::linear_combination3(
                [q(1), wrapped, q(2)],
                [Rational::one(), Rational::one(), Rational::one()],
            );
            let mut recognition = native(&combination);
            assert_eq!(
                recognition.guard_parameter_arithmetic(&combination, 64),
                Some(valid)
            );
            let log = mul(raw(Approximation::PrescaledLn(neg(quotient.clone()))), q(2));
            recognition.parameter_from_logarithm(&log, 0).unwrap();
            assert!(recognition.parameter_atoms.is_empty());
            let component = fresh_angle().jacobi_component(log, 0);
            let consumer = Computable::linear_combination3(
                [component, q(1), q(2)],
                [Rational::one(), Rational::one(), Rational::one()],
            );
            let mut recognition = native(&consumer);
            recognition.collect_parameter_atoms(&consumer).unwrap();
            assert_eq!(recognition.is_parameter_atom(&quotient), Some(valid));
            assert!(consumer.cached().is_none());
        }
    }

    #[test]
    /// Reject an uncancelled rational exponent offset and the dyadic scale 2^4096.
    /// Neither failed inverse-exponential relation may publish a sign or sample on its
    /// candidate; the shared amplitude must also remain unsampled.
    fn exponential_pairing_declines_unmatched_and_oversized_log_scales() {
        let root = fresh_angle();
        let exponential = raw(Approximation::PrescaledExp(root.clone()));
        let mut recognition = native(&root);
        recognition.atom(&root, 64).unwrap();
        recognition.atom(&exponential, 64).unwrap();
        for exponent in [
            add(neg(root.clone()), q(1)),
            add(neg(root.clone()), mul(Computable::ln2(), q(4096))),
        ] {
            let other = raw(Approximation::PrescaledExp(exponent));
            assert!(recognition.native_inverse_exp_relation(&other).is_none());
            assert_unproved(&other);
        }
        assert!(root.cached().is_none());
    }

    /// Evaluate the rectangular Jacobi identity fixture at x=pi and y=abs(parameter).
    /// Delegate to the explicit-coordinate helper; its domain and inverse requirements
    /// are fixture preconditions, not checked construction outcomes returned to callers.
    fn rectangular_relations(parameter: &Real) -> [Real; 3] {
        rectangular_relations_at(Real::pi(), parameter.abs(), parameter)
    }

    /// Build three real-part identities for rectangular complex Jacobi addition formulas.
    /// Let sn(x), cn(x), dn(x) use parameter m. Combine them with complementary-parameter
    /// values at y to obtain sn=a+i*b, cn=f-i*g and dn=h-i*j.
    /// Return a²-b²+f²-g², h²-j²+m*(a²-b²), and
    /// h²-j²-m*(f²-g²)-(1-m), expected to be one, one and zero.
    ///
    /// For a proved negative m, transform the complementary evaluations using
    /// sqrt(1-m) and parameter 1/(1-m); otherwise evaluate at 1-m directly. The tested
    /// parameters and coordinates must admit all Jacobi calls, roots, quotients and the
    /// nonzero addition denominator: failures panic through unwrap. Unknown sign takes
    /// the nonnegative branch, so callers must select fixtures with an appropriate
    /// available sign decision. This helper constructs symbolic expressions; it does
    /// not itself assert the identities, measure cost or prove arbitrary input domains.
    fn rectangular_relations_at(x: Real, y: Real, parameter: &Real) -> [Real; 3] {
        let (s, c, d) = x.jacobi_sncndn(parameter).unwrap();
        let complement = Real::one() - parameter;
        let negative = parameter.certified_sign_until(-128).sign() == Some(RealSign::Negative);
        let (big_s, big_c, big_d, inverse) = if negative {
            let scale = complement.clone().sqrt().unwrap();
            let (big_s, big_c, big_d) = (y * &scale)
                .jacobi_sncndn(&complement.clone().inverse().unwrap())
                .unwrap();
            let inverse = (&big_c * &big_c
                + (-parameter / complement).unwrap() * (&c * &c) * (&big_s * &big_s))
                .inverse()
                .unwrap();
            ((big_s / scale).unwrap(), big_d, big_c, inverse)
        } else {
            let (big_s, big_c, big_d) = y.jacobi_sncndn(&complement).unwrap();
            let inverse = (&big_c * &big_c + parameter * (&s * &s) * (&big_s * &big_s))
                .inverse()
                .unwrap();
            (big_s, big_c, big_d, inverse)
        };
        let a = &s * &big_d * &inverse;
        let b = &c * &d * &big_s * &big_c * &inverse;
        let f = &c * &big_c * &inverse;
        let g = &s * &d * &big_s * &big_d * &inverse;
        let h = &d * &big_c * &big_d * &inverse;
        let j = parameter * &s * &c * &big_s * &inverse;
        [
            (&a * &a - &b * &b) + (&f * &f - &g * &g),
            (&h * &h - &j * &j) + parameter * (&a * &a - &b * &b),
            (&h * &h - &j * &j) - parameter * (&f * &f - &g * &g) - (Real::one() - parameter),
        ]
    }

    #[test]
    /// Prove all three rectangular identities with m=±v/(4*(v+1)).
    /// Use v=pi and pi-3, then fold each result and require exact native projections
    /// one, one and zero. This exercises generated symbolic parameter expressions as inputs.
    fn symbolic_jacobi_parameter_survives_rational_composition() {
        for v in [Real::pi(), Real::pi() - Real::from(3)] {
            let parameter = (&v / (Real::from(4) * (&v + Real::one()))).unwrap();
            for parameter in [parameter.clone(), -parameter] {
                for (real, expected) in rectangular_relations(&parameter).into_iter().zip([1, 1, 0])
                {
                    let value = real.fold();
                    let rational = value.bounded_native_relation_rational();
                    assert_eq!(rational, Some(Rational::from(expected)));
                }
            }
        }
    }

    #[test]
    /// Consume a proved rectangular norm through axis arithmetic, hypot and division.
    /// For m=1/2 and -3, norm=1 gives axis=3, hypot(axis,0)=3 and final position four.
    /// Require the final difference to project to exact zero; this is a scalar consumer
    /// chain, not construction or validation of an entire ellipsoid surface.
    fn proved_norm_remains_consumable_as_an_ellipsoid_axis() {
        for m in [
            Real::from(Rational::fraction(1, 2).unwrap()),
            Real::from(-3),
        ] {
            let [norm, _, _] = rectangular_relations(&m);
            assert_eq!(norm.exact_rational_normal_form(), Some(Rational::one()));
            let axis = &norm + Real::from(2);
            let denominator = Real::hypot2(&axis, &Real::zero()).unwrap();
            let surface = ((&axis * &axis) / denominator).unwrap();
            let position = surface + norm;
            assert_eq!(
                (position - Real::from(4)).exact_rational_normal_form(),
                Some(Rational::zero())
            );
        }
    }

    #[test]
    /// Publish zero for a+a-2*a and require sin(0)=0 and cos(0)=1 to consume that fact.
    /// The shared raw sine amplitude stays unsampled; the endpoint queries need no
    /// additional numerical refinement of its cancelling expression.
    fn proved_zero_is_consumed_by_trigonometric_endpoints_without_refinement() {
        let angle = fresh_angle();
        let zero = add(
            add(angle.clone(), angle.clone()),
            neg(raw(Approximation::Offset(angle.clone(), 1))),
        );
        assert_eq!(zero.bounded_ring_rational(), Some(Rational::zero()));
        assert_eq!(zero.immediate_sign(), Some(RealSign::Zero));
        assert_eq!(zero.clone().sin().exact_rational(), Some(Rational::zero()));
        assert_eq!(zero.cos().exact_rational(), Some(Rational::one()));
        assert!(angle.cached().is_none());
    }

    #[test]
    /// Prove the rectangular identities with rational x and quadratic y and m.
    /// Use x=3/2, m=±x/(4*(x+sqrt(2))) and y=sqrt(2)/(8*(sqrt(2)+1));
    /// folded native projections must preserve one, one and zero for both parameter signs.
    fn quadratic_parameter_preserves_rectangular_norms() {
        let x = Real::from(Rational::fraction(3, 2).unwrap());
        let root = Real::from(2).sqrt().unwrap();
        let m = (&x / (Real::from(4) * (&x + &root))).unwrap();
        let y = (&root / (Real::from(8) * (&root + Real::one()))).unwrap();
        for parameter in [m.clone(), -m] {
            for (real, expected) in rectangular_relations_at(x.clone(), y.clone(), &parameter)
                .into_iter()
                .zip([1, 1, 0])
            {
                let value = real.fold();
                assert_eq!(
                    value.bounded_native_relation_rational(),
                    Some(Rational::from(expected))
                );
            }
        }
    }

    #[test]
    /// Check five (a+b*sqrt(q)) reciprocals against exact product-one reduction.
    /// Include signs, a zero rational part and q=0, all without sampling the root.
    /// Decline 1+sqrt(1) despite its nonzero value because its conjugate norm vanishes;
    /// also reject an oversized norm and a zero-work attempt. These are sufficient
    /// recognizer conditions, not a complete reciprocal-existence test.
    fn quadratic_reciprocals_require_a_nonzero_rational_norm() {
        for (radicand, a, b) in [(2, 3, 1), (2, -3, 1), (2, 0, 1), (3, 2, 2), (0, 3, 1)] {
            let input = q(radicand);
            assert_eq!(
                input.exact_sign(),
                Some(if radicand == 0 {
                    Sign::NoSign
                } else {
                    Sign::Plus
                })
            );
            assert!(input.cached().is_none());
            let root = raw(Approximation::Sqrt(input));
            let mut recognition = native(&root);
            let radical = recognition.atom(&root, 64).unwrap();
            let scaled = recognition.scale(radical, Rational::from(b)).unwrap();
            let polynomial = recognition
                .add(
                    scaled,
                    RingRecognition::constant(Rational::from(a)).unwrap(),
                )
                .unwrap();
            let reciprocal = recognition
                .reciprocal_quadratic_polynomial(&polynomial)
                .unwrap_or_else(|| {
                    panic!("quadratic reciprocal: radicand={radicand}, a={a}, b={b}")
                });
            let product = recognition.multiply(&polynomial, &reciprocal).unwrap();
            let reduced = recognition
                .reduce_native_relations(RingQuotient::polynomial(product))
                .unwrap();
            assert_eq!(
                recognition.project_quotient(&reduced),
                Some(Rational::one())
            );
            assert!(root.cached().is_none());
        }
        let input = q(1);
        assert_eq!(input.exact_sign(), Some(Sign::Plus));
        let root = raw(Approximation::Sqrt(input));
        let mut recognition = native(&root);
        let radical = recognition.atom(&root, 64).unwrap();
        let polynomial = recognition
            .add(radical, RingRecognition::constant(Rational::one()).unwrap())
            .unwrap();
        assert!(
            recognition
                .reciprocal_quadratic_polynomial(&polynomial)
                .is_none(),
            "1+sqrt(1) is nonzero, but its conjugate norm vanishes"
        );
        let excessive = RingPolynomial {
            terms: vec![
                (vec![], Rational::from_bigint(BigInt::one() << 2048_usize)),
                (vec![0], Rational::one()),
            ],
        };
        assert!(
            recognition
                .reciprocal_quadratic_polynomial(&excessive)
                .is_none()
        );
        recognition.operations_left = 0;
        assert!(
            recognition
                .reciprocal_quadratic_polynomial(&polynomial)
                .is_none()
        );
    }
}
