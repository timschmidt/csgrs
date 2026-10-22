#[cfg(test)]
mod ring_trig_relations_tests {
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
    fn rational(n: i32, d: i32) -> Computable {
        raw(Approximation::Ratio(
            Rational::fraction(i64::from(n), u64::try_from(d).unwrap()).unwrap(),
        ))
    }
    fn add(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, b))
    }
    fn mul(a: Computable, b: Computable) -> Computable {
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
    fn angle(n: i32, d: i32) -> Computable {
        raw(Approximation::AtanRational(
            Rational::fraction(i64::from(n), u64::try_from(d).unwrap()).unwrap(),
        ))
    }
    fn shifted_angle(turns: i32) -> Computable {
        add(angle(3, 4), mul(rational(turns, 2), Computable::pi()))
    }
    fn circle(a: Computable, b: Computable) -> Computable {
        let sine = function(false, a);
        let cosine = function(true, b);
        add(
            add(mul(sine.clone(), sine), mul(cosine.clone(), cosine)),
            rational(-1, 1),
        )
    }

    #[test]
    fn rational_quarter_turn_values_keep_every_quadrant_and_sign() {
        for (n, d, h) in [(0, 1, 1), (3, 4, 5), (5, 12, 13), (8, 15, 17)] {
            for sign in [-1, 1] {
                for coefficient in [-1, 1] {
                    for turns in -9_i32..=9 {
                        let a = add(
                            mul(rational(coefficient, 1), angle(sign * n, d)),
                            mul(rational(turns, 2), Computable::pi()),
                        );
                        let s = Rational::fraction(i64::from(coefficient * sign * n), h).unwrap();
                        let c = Rational::fraction(i64::from(d), h).unwrap();
                        let (s, c) = match turns.rem_euclid(4) {
                            0 => (s, c),
                            1 => (c, -s),
                            2 => (-s, -c),
                            _ => (-c, s),
                        };
                        for (cosine, expected) in [(false, s), (true, c)] {
                            assert_eq!(
                                function(cosine, a.clone()).bounded_relation_rational(),
                                Some(expected)
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn unit_circle_requires_equal_admitted_arguments_and_keeps_offsets() {
        let a = mul(rational(1, 2), angle(3, 4));
        let b = add(add(a.clone(), Computable::pi()), negate(Computable::pi()));
        let unit = add(circle(a.clone(), b.clone()), rational(1, 1));
        let fourth = add(mul(unit.clone(), unit.clone()), rational(-1, 1));
        assert_eq!(fourth.bounded_relation_rational(), Some(Rational::zero()));
        let inverse = raw(Approximation::Inverse(unit));
        assert_eq!(inverse.bounded_relation_rational(), Some(Rational::one()));
        let invalid_inverse = raw(Approximation::Inverse(circle(a.clone(), b.clone())));
        assert_eq!(invalid_inverse.bounded_relation_rational(), None);
        for residual in [-7, 0, 11] {
            assert_eq!(
                add(circle(a.clone(), b.clone()), rational(residual, 1))
                    .bounded_relation_rational(),
                Some(Rational::from(residual))
            );
        }
        let gap = raw(Approximation::Ratio(
            Rational::fraction(1, 1_000_000_000_000_u64).unwrap(),
        ));
        assert_eq!(
            circle(a.clone(), add(b.clone(), gap)).bounded_relation_rational(),
            None
        );
        assert_eq!(
            circle(a.clone(), negate(b)).bounded_relation_rational(),
            None
        );
        let invalid = raw(Approximation::Inverse(rational(0, 1)));
        assert_eq!(
            circle(mul(a.clone(), invalid.clone()), mul(a, invalid)).bounded_relation_rational(),
            None
        );
    }

    fn pi_turn(n: i32, d: i32) -> Computable {
        mul(Computable::pi(), rational(n, d))
    }

    fn squares_minus_one(first: Computable, second: Computable) -> Computable {
        add(
            add(mul(first.clone(), first), mul(second.clone(), second)),
            rational(-1, 1),
        )
    }

    #[test]
    fn complementary_sines_and_cosines_lie_on_the_unit_circle() {
        // Exact cos(pi * r) is stored as sin(pi * (1/2 - r)), so the sine and
        // cosine of one rational turn reach the recognizer as two sines. The
        // cyclotomic proof cannot reach turns of order above 2^14 such as
        // these, which 1/10000, 1/360 and 12.345 degrees give.
        for (n, d) in [(1, 1_800_000), (1, 64_800), (2469, 36_000)] {
            let complement = pi_turn(d - 2 * n, 2 * d);
            for cosine in [false, true] {
                let first = function(cosine, pi_turn(n, d));
                let second = function(cosine, complement.clone());
                assert_eq!(
                    squares_minus_one(first.clone(), second.clone()).bounded_relation_rational(),
                    Some(Rational::zero()),
                    "{n}/{d} pi, cosine {cosine}"
                );
                // Either operand order and a residual constant keep the value.
                assert_eq!(
                    add(squares_minus_one(second, first), rational(5, 7))
                        .bounded_relation_rational(),
                    Some(Rational::fraction(5, 7).unwrap())
                );
            }
        }
    }

    #[test]
    fn complementary_arguments_need_not_be_rational_turns() {
        // atan(1/2) has no rational sine, so both functions stay atoms, and
        // the sum pi/2 - atan(1/2) + atan(1/2) cancels in quotient arithmetic.
        let a = angle(1, 2);
        let b = add(mul(rational(1, 2), Computable::pi()), negate(a.clone()));
        for cosine in [false, true] {
            assert_eq!(
                squares_minus_one(function(cosine, a.clone()), function(cosine, b.clone()))
                    .bounded_relation_rational(),
                Some(Rational::zero())
            );
        }
    }

    #[test]
    fn complementary_pairs_require_an_exact_half_pi_sum() {
        let a = pi_turn(1, 1_800_000);
        let complement = pi_turn(899_999, 1_800_000);
        let gap = raw(Approximation::Ratio(
            Rational::fraction(1, 1_000_000_000_000_u64).unwrap(),
        ));
        for second in [
            // Misses pi/2 by 10^-12.
            add(complement.clone(), gap),
            // sin(pi/2 + a) is cos(a), but the sum is not exactly pi/2.
            pi_turn(900_001, 1_800_000),
            // The same argument: sin(a)^2 + sin(a)^2 is not one.
            a.clone(),
        ] {
            assert_eq!(
                squares_minus_one(function(false, a.clone()), function(false, second))
                    .bounded_relation_rational(),
                None
            );
        }
        // A sine and a cosine still need equal arguments, not complementary ones.
        assert_eq!(
            squares_minus_one(function(false, a.clone()), function(true, complement))
                .bounded_relation_rational(),
            None
        );
    }

    #[test]
    fn unsupported_and_oversized_trig_projections_do_not_guess() {
        for a in [
            angle(1, 1),
            add(angle(3, 4), rational(1, 1000)),
            mul(rational(2, 1), angle(3, 4)),
        ] {
            assert_eq!(function(false, a).bounded_relation_rational(), None);
        }
        let large = Rational::from_bigint(BigInt::from(1) << 3000_u32);
        assert_eq!(
            function(false, raw(Approximation::AtanRational(large))).bounded_relation_rational(),
            None
        );
        let bad = raw(Approximation::Sqrt(rational(-1, 1)));
        let value = add(function(false, bad.clone()), negate(function(false, bad)));
        assert_eq!(value.bounded_relation_rational(), None);
        assert!(matches!(
            value.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
    }

    #[test]
    fn positive_function_reuse_preserves_cancellation_depth_and_capacity() {
        let values: Vec<_> = (0..34).map(|t| function(false, shifted_angle(t))).collect();
        let signal = Arc::new(AtomicBool::new(false));
        let mut root = rational(0, 1);
        root.abort(signal.clone());
        let mut recognition = RingRecognition::new(&root);
        recognition.root_squares = true;
        recognition.rational_squares = Some(Vec::new());
        recognition.trig_arguments = Some(Vec::new());
        for (i, value) in values.iter().enumerate() {
            let projection = recognition
                .expand_quotient(value, RingRecognition::MAX_DEPTH)
                .unwrap();
            let (n, d) = [(3, 5), (4, 5), (-3, 5), (-4, 5)][i % 4];
            assert_eq!(
                recognition.project_quotient(&projection),
                Some(Rational::fraction(i64::from(n), u64::try_from(d).unwrap()).unwrap())
            );
        }
        assert_eq!(
            recognition.rational_trig_values.len(),
            RingRecognition::MAX_ATOMS
        );
        let independent = function(false, shifted_angle(0));
        let before = recognition.nodes_left;
        let projection = recognition
            .expand_quotient(&independent, RingRecognition::MAX_DEPTH)
            .unwrap();
        assert_eq!(recognition.nodes_left, before - 1);
        assert_eq!(
            recognition.project_quotient(&projection),
            Some(Rational::fraction(3, 5).unwrap())
        );
        let shallow = function(false, shifted_angle(0));
        assert!(recognition.expand_quotient(&shallow, 1).is_none());
        signal.store(true, std::sync::atomic::Ordering::Relaxed);
        assert!(
            recognition
                .expand_quotient(&independent, RingRecognition::MAX_DEPTH)
                .is_none()
        );
        signal.store(false, std::sync::atomic::Ordering::Relaxed);
        assert!(
            recognition
                .expand_quotient(&independent, RingRecognition::MAX_DEPTH)
                .is_some()
        );
        recognition.rational_trig_values.clear();
        let after_clear = function(false, shifted_angle(0));
        let projection = recognition
            .expand_quotient(&after_clear, RingRecognition::MAX_DEPTH)
            .unwrap();
        assert_eq!(
            recognition.project_quotient(&projection),
            Some(Rational::fraction(3, 5).unwrap())
        );
    }
}
