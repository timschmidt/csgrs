#[cfg(test)]
/// Common-factor zero proofs, perturbations, domain obligations and late-retry limits.
mod ring_product_sum_tests {
    use super::*;

    /// Create an unsimplified recipe with invalid bound/sign facts and no stop signal.
    /// The fresh node has no numerical sample, so fixtures expose recovery's own work.
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

    /// Represent an i64 exactly as a raw Ratio node without constructor simplification.
    fn number(value: i64) -> Computable {
        raw(Approximation::Ratio(Rational::from(value)))
    }

    /// Build a raw sum sharing the supplied operand nodes in left-to-right order.
    fn add(a: &Computable, b: &Computable) -> Computable {
        raw(Approximation::Add(a.clone(), b.clone()))
    }

    /// Wrap a shared operand in raw negation without simplifying its sign or recipe.
    fn neg(a: &Computable) -> Computable {
        raw(Approximation::Negate(a.clone()))
    }

    /// Build a raw product sharing operands, retaining their order and domain obligations.
    fn mul(a: &Computable, b: &Computable) -> Computable {
        raw(Approximation::Multiply(a.clone(), b.clone()))
    }

    /// Build the large cancelling fixture with zero perturbation and an optional factor.
    /// The expected zero requires that optional factor to denote a defined real value.
    fn identity(extra_factor: Option<Computable>) -> Computable {
        perturbed_identity(extra_factor, Rational::zero())
    }

    /// Combine fixture parts as left+12*right after perturbing the inner sum.
    /// With optional extra factor E (one if absent), the value is -residual*z*h²*E;
    /// perturbation tests must therefore distinguish a tiny nonzero value from zero.
    fn perturbed_identity(extra_factor: Option<Computable>, residual: Rational) -> Computable {
        let (left, right) = identity_parts(extra_factor, residual);
        add(&left, &mul(&number(12), &right))
    }

    /// Construct opposite product expressions over four distinct sine atoms.
    /// Use sixth-step affine interpolants z and y to form h, and s=(6-u)hx+xh(u+6).
    /// Without perturbation s=12xh, giving left=-12*right; each complete product has
    /// 145 expanded terms before their cancellation. Add the exact residual to s and
    /// include any supplied extra factor on both sides. Raw wrappers preserve sharing
    /// and the large intermediate representation that recovery is intended to avoid.
    fn identity_parts(
        extra_factor: Option<Computable>,
        residual: Rational,
    ) -> (Computable, Computable) {
        let [a, b, c, d] = [3, 5, 7, 11].map(|denominator| {
            raw(Approximation::PrescaledSin(Computable::rational(
                Rational::fraction(1, denominator).unwrap(),
            )))
        });
        let six = number(6);
        let twelve = number(12);
        let sixth = raw(Approximation::Ratio(Rational::fraction(1, 6).unwrap()));
        let a = add(&mul(&number(2), &a), &six);
        let x = add(&mul(&number(2), &b), &six);
        let z = add(&a, &mul(&sixth, &add(&x, &neg(&a))));
        let u = mul(&number(2), &c);
        let d = add(&mul(&number(2), &d), &six);
        let y = add(&d, &mul(&sixth, &add(&add(&u, &six), &neg(&d))));
        let h = add(
            &mul(&z, &add(&u, &neg(&six))),
            &neg(&mul(&add(&y, &neg(&twelve)), &x)),
        );
        // s = (6-u) h x + x h (u+6) = 12 x h. The complete
        // opposite products have 145 terms before their cancellation.
        let mut s = add(
            &mul(&mul(&add(&six, &neg(&u)), &h), &x),
            &mul(&mul(&x, &h), &add(&u, &six)),
        );
        if !residual.is_zero() {
            s = add(&s, &Computable::rational(residual));
        }
        let mut common = mul(&z, &h);
        if let Some(extra) = extra_factor {
            common = mul(&common, &extra);
        }
        (
            neg(&mul(&mul(&h, &s), &common)),
            mul(&mul(&h, &mul(&x, &h)), &common),
        )
    }

    /// Require the queried node to retain an invalid exact sign and no numerical sample.
    /// This does not assert that child nodes or every other kind of cached fact are empty.
    fn assert_unproved(value: &Computable) {
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
        assert!(value.cached().is_none());
    }

    #[test]
    /// Recover the large common-factor identity plus zero or an exact ±10^-12 displacement.
    /// The returned rational must retain that displacement and require no root sample.
    fn shared_products_cancel_before_large_polynomial_expansion() {
        for displacement in [0, -1, 1] {
            let epsilon = Rational::fraction(displacement, 1_000_000_000_000).unwrap();
            let value = add(&identity(None), &Computable::rational(epsilon.clone()));
            assert_eq!(value.bounded_quotient_rational(), Some(epsilon));
            assert!(value.cached().is_none());
        }
    }

    #[test]
    /// Decline zero/unproved inverse factors and an ordinary degree-nine summand.
    /// Even a shared factor cannot evade domain admission or the ordinary degree cap.
    fn shared_product_cancellation_preserves_operand_domains_and_degree() {
        for divisor in [number(0), raw(Approximation::PrescaledSin(number(1)))] {
            let value = identity(Some(raw(Approximation::Inverse(divisor))));
            assert_eq!(value.bounded_quotient_rational(), None);
            assert_unproved(&value);
        }
        let atom = raw(Approximation::PrescaledSin(number(2)));
        let value = identity(Some(atom));
        // Each summand now has degree nine, above the existing degree cap.
        assert_eq!(value.bounded_quotient_rational(), None);
        assert_unproved(&value);
    }

    #[test]
    /// Perturb the inner sum by ±10^-12 and prohibit a false zero certificate.
    /// Independent sine bounds give z>0 and h≠0, so -epsilon*z*h² has the opposite sign;
    /// refinement must confirm that sign for each perturbed raw graph.
    fn shared_product_cancellation_rejects_a_nonzero_residual() {
        // x-x^3/6 < sin(x) < x for these positive inputs gives 6 < z < 7
        // and -1 < h < -1/2. Perturbing s changes the result to
        // -epsilon*z*h^2, whose sign is opposite epsilon's.
        for sign in [-1, 1] {
            let epsilon = Rational::fraction(sign, 1_000_000_000_000).unwrap();
            let value = perturbed_identity(None, epsilon);
            assert_ne!(value.bounded_quotient_rational(), Some(Rational::zero()));
            assert_eq!(
                value.sign_until(-128),
                Some(if sign < 0 {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                })
            );
        }
    }

    #[test]
    /// Check 3*(left+5*right)+7*(left+15*right)=0 when left=-12*right.
    /// Each inner sum exceeds ordinary term expansion; scalar distribution must preserve
    /// the common-factor proof and avoid numerical sampling of the completed root.
    fn common_factors_survive_scaled_nested_product_sums() {
        let (left, right) = identity_parts(None, Rational::zero());
        // left = -12*right, so 3*(left+5*right)+7*(left+15*right)=0.
        // Each inner sum is nonzero and exceeds the expansion term limit.
        let value = add(
            &mul(&number(3), &add(&left, &mul(&number(5), &right))),
            &mul(&number(7), &add(&left, &mul(&number(15), &right))),
        );
        assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
        assert!(value.cached().is_none());
    }

    #[test]
    /// Decline a sixteen-charge attempt and a stopped query without publishing a root proof.
    /// Clearing cancellation must permit a fresh ordinary quotient proof of the identity.
    fn shared_product_cancellation_preserves_budgets_and_recovers_after_cancel() {
        let mut value = identity(None);
        let mut recognition = RingRecognition::new(&value);
        recognition.operations_left = 16;
        assert!(
            recognition
                .expand_quotient(&value, RingRecognition::MAX_DEPTH)
                .is_none()
        );
        assert_unproved(&value);
        drop(recognition);
        let signal = Arc::new(AtomicBool::new(true));
        value.abort(signal.clone());
        assert_eq!(value.bounded_quotient_rational(), None);
        assert_unproved(&value);
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(value.bounded_quotient_rational(), Some(Rational::zero()));
    }

    /// Subtract forward and reversed products of count positive quadratic sine factors.
    /// Each factor is 1+sin(1/(3+i))². Append an optional shared factor to exercise
    /// inverse admission. The count parameter controls formal degree before cancellation.
    fn high_degree_opposite_products(count: usize, extra: Option<Computable>) -> Computable {
        let mut factors: Vec<_> = (0..count)
            .map(|index| {
                let angle = raw(Approximation::Ratio(
                    Rational::fraction(1, 3 + index as u64).unwrap(),
                ));
                let sine = raw(Approximation::PrescaledSin(angle));
                add(&raw(Approximation::Square(sine)), &number(1))
            })
            .collect();
        if let Some(extra) = extra {
            factors.push(extra);
        }
        let left = factors.iter().fold(number(1), |a, b| mul(&a, b));
        let right = factors.iter().rev().fold(number(1), |a, b| mul(&a, b));
        add(&left, &neg(&right))
    }

    #[test]
    /// Recover seven or twelve quadratic factors only through the late extended retry.
    /// Thirteen retained factors must still decline, and successful roots need no sample.
    fn late_product_cancellation_bounds_factors_without_expanding_their_total_degree() {
        for count in [7, 12] {
            let value = high_degree_opposite_products(count, None);
            assert_eq!(value.bounded_quotient_rational(), None);
            assert_unproved(&value);
            assert_eq!(
                value.bounded_extended_relation_rational(),
                Some(Rational::zero())
            );
            assert!(value.cached().is_none());
        }
        let value = high_degree_opposite_products(13, None);
        assert_eq!(value.bounded_extended_relation_rational(), None);
        assert_unproved(&value);
    }

    #[test]
    /// Check that late cancellation admits 1/2 but rejects zero or unproved inverse factors.
    /// Adding either exact ±10^-12 displacement to the large identity must not prove zero.
    fn late_product_cancellation_keeps_domains_and_nonzero_residuals() {
        for divisor in [number(0), raw(Approximation::PrescaledSin(number(1)))] {
            let value =
                high_degree_opposite_products(7, Some(raw(Approximation::Inverse(divisor))));
            assert_eq!(value.bounded_extended_relation_rational(), None);
            assert_unproved(&value);
        }
        let value = high_degree_opposite_products(7, Some(raw(Approximation::Inverse(number(2)))));
        assert_eq!(
            value.bounded_extended_relation_rational(),
            Some(Rational::zero())
        );
        for sign in [-1, 1] {
            let value = add(
                &high_degree_opposite_products(7, None),
                &raw(Approximation::Ratio(
                    Rational::fraction(sign, 1_000_000_000_000).unwrap(),
                )),
            );
            assert_ne!(
                value.bounded_extended_relation_rational(),
                Some(Rational::zero())
            );
        }
    }

    #[test]
    /// Measure the successful late retry's charge count, then test its exact work boundary.
    /// The measured allowance must succeed and one fewer charge must fail; zero work,
    /// node or depth allowances and cancellation also decline. Clearing the signal permits
    /// a fresh complete query. These dynamic checks are not fixed dispatch-count assertions.
    fn late_product_cancellation_preserves_remaining_work_depth_nodes_and_cancel() {
        let mut value = high_degree_opposite_products(7, None);
        let attempt = |value: &Computable, budget: usize, nodes: usize, depth: usize| {
            let mut recognition = RingRecognition::new(value);
            recognition.extended_degree = true;
            recognition.operations_left = budget;
            recognition.nodes_left = nodes;
            let Approximation::Add(a, b) = &value.internal.approximation else {
                panic!()
            };
            let result = recognition.recover_factored_zero::<true>(a, b, depth);
            (result.is_some(), budget - recognition.operations_left)
        };
        let (success, used) = attempt(
            &value,
            RingRecognition::EXTENDED_OPERATIONS,
            RingRecognition::MAX_NODES,
            RingRecognition::MAX_DEPTH - 1,
        );
        assert!(success && used > 0);
        assert!(
            attempt(
                &value,
                used,
                RingRecognition::MAX_NODES,
                RingRecognition::MAX_DEPTH - 1
            )
            .0
        );
        assert!(
            !attempt(
                &value,
                used - 1,
                RingRecognition::MAX_NODES,
                RingRecognition::MAX_DEPTH - 1
            )
            .0
        );
        assert!(
            !attempt(
                &value,
                0,
                RingRecognition::MAX_NODES,
                RingRecognition::MAX_DEPTH - 1
            )
            .0
        );
        assert!(
            !attempt(
                &value,
                RingRecognition::EXTENDED_OPERATIONS,
                0,
                RingRecognition::MAX_DEPTH - 1
            )
            .0
        );
        assert!(
            !attempt(
                &value,
                RingRecognition::EXTENDED_OPERATIONS,
                RingRecognition::MAX_NODES,
                0
            )
            .0
        );
        assert_unproved(&value);
        let signal = Arc::new(AtomicBool::new(true));
        value.abort(signal.clone());
        assert_eq!(value.bounded_extended_relation_rational(), None);
        assert_unproved(&value);
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(
            value.bounded_extended_relation_rational(),
            Some(Rational::zero())
        );
    }

    /// Build C*(sin(1/3)²+cos(1/3)²)-C with a degree-twelve common factor C.
    /// An optional extra factor remains a domain obligation even though C cancels.
    fn high_degree_unit_circle_product(extra: Option<Computable>) -> Computable {
        high_degree_circle_product(extra, 3)
    }

    /// Build C*(sin(1/3)²+cos(1/d)²)-C, with C=1+sin(1/7)^12 times an optional factor.
    /// Use a nonzero cosine denominator d. Equal angles give the unit-circle identity;
    /// d=5 yields a positive residual and tests that unmatched angles cannot share proof.
    /// Raw wrappers keep the high-degree common factor and optional inverse visible.
    fn high_degree_circle_product(
        extra: Option<Computable>,
        cosine_denominator: u64,
    ) -> Computable {
        let atom = raw(Approximation::PrescaledSin(raw(Approximation::Ratio(
            Rational::fraction(1, 7).unwrap(),
        ))));
        let power = (0..12).fold(number(1), |value, _| mul(&value, &atom));
        let mut common = add(&power, &number(1));
        if let Some(extra) = extra {
            common = mul(&common, &extra);
        }
        let angle = raw(Approximation::Ratio(Rational::fraction(1, 3).unwrap()));
        let sine = raw(Approximation::PrescaledSin(angle.clone()));
        let cosine = raw(Approximation::PrescaledCos(raw(Approximation::Ratio(
            Rational::fraction(1, cosine_denominator).unwrap(),
        ))));
        let circle = add(
            &raw(Approximation::Square(sine)),
            &raw(Approximation::Square(cosine)),
        );
        add(&mul(&common, &circle), &neg(&common))
    }

    #[test]
    /// Require a registered equal-angle relation to prove the factored residual zero.
    /// Reject zero/unproved inverse factors, admit 1/2, and distinguish the positive
    /// sin(1/3)²-sin(1/5)² residual by both nonpublication and numerical sign refinement.
    fn late_factor_residual_uses_only_proved_unit_circle_relations() {
        let value = high_degree_unit_circle_product(None);
        assert_eq!(value.bounded_quotient_rational(), None);
        assert_unproved(&value);
        assert_eq!(
            value.bounded_extended_relation_rational(),
            Some(Rational::zero())
        );
        for divisor in [number(0), raw(Approximation::PrescaledSin(number(1)))] {
            let value = high_degree_unit_circle_product(Some(raw(Approximation::Inverse(divisor))));
            assert_eq!(value.bounded_extended_relation_rational(), None);
            assert_unproved(&value);
        }
        // sin is strictly increasing on (0, pi/2), so sin(1/3)^2 +
        // cos(1/5)^2 - 1 = sin(1/3)^2 - sin(1/5)^2 is positive.
        let distinct = high_degree_circle_product(None, 5);
        assert_eq!(distinct.bounded_extended_relation_rational(), None);
        assert_unproved(&distinct);
        assert_eq!(distinct.sign_until(-128), Some(RealSign::Positive));
        // The defined inverse's nonzero witness remains sufficient.
        let value = high_degree_unit_circle_product(Some(raw(Approximation::Inverse(number(2)))));
        assert_eq!(
            value.bounded_extended_relation_rational(),
            Some(Rational::zero())
        );
    }

    #[test]
    /// Include failed initial expansion in the measured late-recovery work allowance.
    /// The exact measured budget succeeds and one fewer charge declines. Set cancellation
    /// after expansion but before recovery to verify the remaining path observes it;
    /// a fresh uncancelled query must still recover the unit-circle identity.
    fn late_factor_residual_charges_all_work_and_observes_mid_attempt_cancel() {
        let mut value = high_degree_unit_circle_product(None);
        let attempt = |value: &Computable, budget| {
            let mut recognition = RingRecognition::new(value);
            recognition.root_squares = true;
            recognition.trig_arguments = Some(Vec::new());
            recognition.rational_squares = Some(Vec::new());
            recognition.extended_degree = true;
            recognition.operations_left = budget;
            let original = recognition.expand_quotient(value, RingRecognition::MAX_DEPTH);
            assert!(original.is_none());
            let Approximation::Add(a, b) = &value.internal.approximation else {
                panic!()
            };
            let result = recognition
                .recover_factored_zero::<true>(a, b, RingRecognition::MAX_DEPTH - 1)
                .and_then(|quotient| recognition.project_quotient(&quotient));
            (result, budget - recognition.operations_left)
        };
        let (result, used) = attempt(&value, RingRecognition::EXTENDED_OPERATIONS);
        assert_eq!(result, Some(Rational::zero()));
        assert_eq!(attempt(&value, used).0, Some(Rational::zero()));
        assert_eq!(attempt(&value, used - 1).0, None);
        assert_unproved(&value);

        let signal = Arc::new(AtomicBool::new(false));
        value.abort(signal.clone());
        let mut recognition = RingRecognition::new(&value);
        recognition.root_squares = true;
        recognition.trig_arguments = Some(Vec::new());
        recognition.rational_squares = Some(Vec::new());
        recognition.extended_degree = true;
        recognition.operations_left = RingRecognition::EXTENDED_OPERATIONS;
        assert!(
            recognition
                .expand_quotient(&value, RingRecognition::MAX_DEPTH)
                .is_none()
        );
        signal.store(true, std::sync::atomic::Ordering::Relaxed);
        let Approximation::Add(a, b) = &value.internal.approximation else {
            panic!()
        };
        assert!(
            recognition
                .recover_factored_zero::<true>(a, b, RingRecognition::MAX_DEPTH - 1)
                .is_none()
        );
        assert_unproved(&value);
        drop(recognition);
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(
            value.bounded_extended_relation_rational(),
            Some(Rational::zero())
        );
    }
}
