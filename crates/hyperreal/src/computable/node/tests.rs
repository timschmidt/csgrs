#[cfg(test)]
mod tests {
    use super::*;
    use num::Signed;
    use num::bigint::BigUint;
    use std::mem::size_of;

    #[test]
    fn compare() {
        let six: BigInt = "6".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let four: BigInt = "4".parse().unwrap();
        let six = Computable::integer(six.clone());
        let five = Computable::integer(five.clone());
        let four = Computable::integer(four.clone());

        assert_eq!(six.try_compare_to(&five), Some(Ordering::Greater));
        assert_eq!(five.try_compare_to(&six), Some(Ordering::Less));
        assert_eq!(four.try_compare_to(&six), Some(Ordering::Less));
    }

    #[test]
    fn binary64_filter_constant_enclosures_cover_mpfr_values() {
        use rug::{Float, float::Constant};

        let pi = Float::with_val(256, Constant::Pi);
        let pi_lower = Float::with_val(256, f64::from_bits(0x4009_21fb_5444_2d17));
        let pi_upper = Float::with_val(256, f64::from_bits(0x4009_21fb_5444_2d19));
        assert!(pi_lower < pi && pi < pi_upper);

        let e = Float::with_val(256, 1).exp();
        let e_lower = Float::with_val(256, f64::from_bits(0x4005_bf0a_8b14_5768));
        let e_upper = Float::with_val(256, f64::from_bits(0x4005_bf0a_8b14_576a));
        assert!(e_lower < e && e < e_upper);

        assert_eq!(Binary64Interval::power_of_two(1_024), f64::INFINITY);
        assert_eq!(Binary64Interval::power_of_two(1_023).to_bits(), 0x7fe0_0000_0000_0000);
        assert_eq!(Binary64Interval::power_of_two(-1_022).to_bits(), 1_u64 << 52);
        assert_eq!(Binary64Interval::power_of_two(-1_023).to_bits(), 1_u64 << 51);
        assert_eq!(Binary64Interval::power_of_two(-1_074).to_bits(), 1);
        assert_eq!(Binary64Interval::power_of_two(-1_075).to_bits(), 0);
    }

    #[test]
    fn demand_sized_sqrt_seeds_match_directed_mpfr() {
        use rug::{Float, Integer, Rational as RugRational, float::Round};

        let check = |numerator: BigInt, denominator: BigUint, precisions: &[i32]| {
            let exact = Rational::from_bigint_fraction(numerator.clone(), denominator.clone())
                .expect("positive denominator");
            let oracle = RugRational::from((
                Integer::from_str_radix(&numerator.to_string(), 10).unwrap(),
                Integer::from_str_radix(&denominator.to_string(), 10).unwrap(),
            ));
            let mut lower = Float::with_val_round(2_048, &oracle, Round::Down).0;
            let mut upper = Float::with_val_round(2_048, &oracle, Round::Up).0;
            lower.sqrt_round(Round::Down);
            upper.sqrt_round(Round::Up);
            for &precision in precisions {
                // Bypass symbolic square folding to exercise the numeric seed,
                // including exact squares and values near a rounding boundary.
                let root = Computable {
                    internal: Arc::new(Node::new(
                        Approximation::Sqrt(Computable::rational(exact.clone())),
                        BoundCache::Invalid,
                        ExactSignCache::Invalid,
                    )),
                    signal: None,
                };
                let actual = root.approx(precision);
                let actual = Float::with_val(
                    2_048,
                    Integer::from_str_radix(&actual.to_string(), 10).unwrap(),
                );
                let mut low = lower.clone();
                let mut high = upper.clone();
                low >>= precision;
                high >>= precision;
                assert!(
                    Float::with_val(2_048, &actual - low).abs() <= 1
                        && Float::with_val(2_048, &actual - high).abs() <= 1,
                    "sqrt({exact:?}) exceeds one ulp at precision {precision}"
                );
            }
        };

        for exponent in [-601_i32, -600, -80, 0, 80, 600, 601] {
            let precisions = [
                0, 1, 16, 32, 56, 57, 58, 59, 60, 61, 63, 64, 95, 96, 97, 128, 139, 140, 141, 256,
                1_024,
            ]
            .map(|digits| exponent / 2 - digits);
            for n in [2_u32, 3, 5, 17] {
                let mut numerator = BigInt::from(n);
                let mut denominator = BigUint::one();
                if exponent >= 0 {
                    numerator <<= exponent as usize;
                } else {
                    denominator <<= (-exponent) as usize;
                }
                check(numerator, denominator, &precisions);
            }
        }
        let denominator = BigUint::one() << 256_usize;
        for n in [1_u32, 2, 17] {
            for delta in [-1_i32, 0, 1] {
                let numerator = (BigInt::from(n * n) << 256_usize) + delta;
                check(
                    numerator,
                    denominator.clone(),
                    &[0, -16, -56, -59, -60, -64, -96, -128, -140, -256],
                );
            }
        }
    }

    #[test]
    fn direct_nth_root_approximations_match_mpfr() {
        use rug::{Float, float::Round};

        for degree in 3_u32..=Computable::MAX_DIRECT_NTH_ROOT_DEGREE {
            for radicand in [2_i64, 3, 17, 257] {
                let root = Computable::integer(BigInt::from(radicand)).nth_root(degree);
                assert!(matches!(
                    &root.internal.approximation,
                    Approximation::NthRoot(_, stored_degree) if *stored_degree == degree
                ));
                for precision in [4_i32, 0, -1, -7, -31, -64, -255, -1_024] {
                    let actual = root.approx(precision);
                    let oracle_precision = u32::try_from(precision.saturating_neg())
                        .unwrap_or_default()
                        .saturating_add(256);
                    let mut oracle = Float::with_val(oracle_precision, radicand).root(degree);
                    oracle <<= precision.saturating_neg();
                    let expected = oracle
                        .to_integer_round(Round::Nearest)
                        .expect("finite positive nth-root oracle")
                        .0
                        .to_string()
                        .parse::<BigInt>()
                        .expect("MPFR integer parses as BigInt");
                    let error = (&actual - expected).abs();
                    assert!(
                        error <= BigInt::one(),
                        "root({radicand}, {degree}) at {precision} differs by {error} ulps"
                    );
                }
            }

            for (numerator, denominator) in [(1_i64, 257_u64), (1, 3), (2, 3), (17, 19), (257, 3)] {
                let root = Computable::rational(
                    Rational::fraction(numerator, denominator).unwrap(),
                )
                .nth_root(degree);
                for precision in [4_i32, 0, -7, -31, -128, -511] {
                    let actual = root.approx(precision);
                    let oracle_precision = u32::try_from(precision.saturating_neg())
                        .unwrap_or_default()
                        .saturating_add(256);
                    let mut oracle = Float::with_val(oracle_precision, numerator);
                    oracle /= denominator;
                    oracle.root_mut(degree);
                    oracle <<= precision.saturating_neg();
                    let expected = oracle
                        .to_integer_round(Round::Nearest)
                        .expect("finite positive rational nth-root oracle")
                        .0
                        .to_string()
                        .parse::<BigInt>()
                        .expect("MPFR integer parses as BigInt");
                    let error = (&actual - expected).abs();
                    assert!(
                        error <= BigInt::one(),
                        "root({numerator}/{denominator}, {degree}) at {precision} differs by {error} ulps"
                    );
                }
            }

            for exponent in [-2_048_i32, -257, 257, 2_048] {
                let radicand = if exponent >= 0 {
                    Rational::from_bigint(BigInt::one() << exponent as usize)
                } else {
                    Rational::from_bigint_fraction(
                        BigInt::one(),
                        BigUint::one() << exponent.unsigned_abs() as usize,
                    )
                    .unwrap()
                };
                let root = Computable::rational(radicand).nth_root(degree);
                for precision in [32_i32, 0, -127, -1_024] {
                    let actual = root.approx(precision);
                    let oracle_precision = u32::try_from(precision.saturating_neg())
                        .unwrap_or_default()
                        .saturating_add(exponent.max(0) as u32 / degree)
                        .saturating_add(256);
                    let mut oracle = Float::with_val(oracle_precision, 1);
                    if exponent >= 0 {
                        oracle <<= exponent as u32;
                    } else {
                        oracle >>= exponent.unsigned_abs();
                    }
                    oracle.root_mut(degree);
                    oracle <<= precision.saturating_neg();
                    let expected = oracle
                        .to_integer_round(Round::Nearest)
                        .expect("finite dyadic nth-root oracle")
                        .0
                        .to_string()
                        .parse::<BigInt>()
                        .expect("MPFR integer parses as BigInt");
                    let error = (&actual - expected).abs();
                    assert!(
                        error <= BigInt::one(),
                        "root(2^{exponent}, {degree}) at {precision} differs by {error} ulps"
                    );
                }
            }
        }
    }

    fn positive_rational_nth_root(numerator: i64, denominator: u64, degree: u32) -> Computable {
        Computable::rational(Rational::fraction(numerator, denominator).unwrap()).nth_root(degree)
    }

    #[test]
    fn algebraic_separation_bounds_certify_archived_radical_identities() {
        let cbrt2 = positive_rational_nth_root(2, 1, 3);
        let cbrt4 = positive_rational_nth_root(4, 1, 3);
        let cbrt5 = positive_rational_nth_root(5, 1, 3);
        let cbrt20 = positive_rational_nth_root(20, 1, 3);
        let cbrt25 = positive_rational_nth_root(25, 1, 3);
        let first = cbrt5
            .add(cbrt4.negate())
            .sqrt()
            .multiply(Computable::integer(BigInt::from(3_u8)))
            .add(cbrt2.add(cbrt20).add(cbrt25.negate()).negate());

        let second_left = positive_rational_nth_root(2, 1, 3)
            .add(Computable::integer(BigInt::from(-1_i8)))
            .nth_root(3);
        let second_right = positive_rational_nth_root(1, 9, 3)
            .add(positive_rational_nth_root(2, 9, 3).negate())
            .add(positive_rational_nth_root(4, 9, 3));
        let second = second_left.add(second_right.negate());

        let fifth2 = positive_rational_nth_root(2, 1, 5);
        let c10_inner = Computable::integer(BigInt::from(7_u8))
            .add(fifth2.clone())
            .add(
                positive_rational_nth_root(8, 1, 5)
                    .multiply(Computable::integer(BigInt::from(-5_i8))),
            );
        let c10 = c10_inner
            .nth_root(3)
            .add(positive_rational_nth_root(4, 1, 5))
            .add(fifth2.negate())
            .add(Computable::integer(BigInt::from(-1_i8)));

        for (name, expected_bound, value) in [
            ("ramanujan-1", 85_u64, first),
            ("ramanujan-2", 376, second),
            ("c10", 70, c10),
        ] {
            let bound = value
                .algebraic_separation_bound_bits()
                .unwrap_or_else(|| panic!("{name} should have bounded algebraic metadata"));
            assert_eq!(bound, expected_bound);
            assert!(bound <= 2_046, "{name} bound {bound} should fit the public floor");
            assert_eq!(value.sign_until(0), None, "{name} should need exact certification");
            assert_eq!(
                value.sign_until(-2_048),
                Some(RealSign::Zero),
                "{name} should certify exact zero"
            );
            let (precision, approximation) = value
                .internal
                .cache_snapshot()
                .expect("zero certificate retains its final approximation");
            assert!(approximation.abs() <= BigInt::one());
            assert!(
                precision > -512,
                "{name} should stop at its separation threshold, got {precision}"
            );
        }
    }

    #[test]
    fn algebraic_generator_dependencies_and_perturbations_are_exact() {
        let dyadic = |power: usize| {
            Rational::from_bigint_fraction(
                BigInt::one(),
                BigUint::one() << power,
            )
            .unwrap()
        };

        for degree in 3_u32..=Computable::MAX_DIRECT_NTH_ROOT_DEGREE {
            let first = Computable::rational(Rational::new(2)).nth_root(degree);
            let second = Computable::rational(Rational::new(3)).nth_root(degree);
            let mut radicand = Rational::one();
            let mut expected = Computable::one();
            for _ in 0..degree.saturating_sub(1) {
                radicand *= Rational::new(2);
                expected = expected.multiply(first.clone());
            }
            radicand *= Rational::new(3);
            expected = expected.multiply(second);
            for _ in 0..degree {
                radicand *= Rational::new(2);
            }
            expected = expected.multiply(Computable::rational(Rational::new(2)));

            let dependent = Computable::rational(radicand).nth_root(degree);
            let identity = expected.add(dependent.negate());
            let positive = identity
                .clone()
                .add(Computable::rational(dyadic(256)));
            let negative = identity
                .clone()
                .add(Computable::rational(dyadic(256).neg()));

            assert_eq!(positive.sign_until(-512), Some(RealSign::Positive));
            assert_eq!(negative.sign_until(-512), Some(RealSign::Negative));
            let bound = identity
                .algebraic_separation_bound_bits()
                .expect("generated radical identity should have a finite bound");
            let floor = -i32::try_from(bound + 2).expect("bounded test precision");
            assert_eq!(
                identity.sign_until(floor),
                Some(RealSign::Zero),
                "degree {degree}, bound {bound}"
            );
        }

        let root = Computable::rational(Rational::new(17)).nth_root(5);
        let reciprocal_identity = root
            .clone()
            .inverse()
            .multiply(root)
            .add(Computable::integer(BigInt::from(-1_i8)));
        assert_eq!(
            reciprocal_identity.sign_until(-512),
            Some(RealSign::Zero)
        );

        assert!(
            Computable::pi()
                .add(Computable::rational(Rational::new(-3)))
                .algebraic_separation_bound_bits()
                .is_none()
        );
    }

    #[test]
    fn algebraic_separation_resource_caps_fail_closed() {
        let primes = [
            2_i64, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59,
        ];
        let mut metadata =
            AlgebraicSeparation::rational(&Rational::one()).expect("unit metadata");
        for prime in &primes[..AlgebraicSeparation::MAX_GENERATORS] {
            let root = Computable::rational(Rational::new(*prime)).sqrt();
            metadata
                .insert_generator(root, 2)
                .expect("sixteen quadratic generators fit both caps");
        }
        assert_eq!(
            metadata.generators.len(),
            AlgebraicSeparation::MAX_GENERATORS
        );
        assert!(
            metadata
                .clone()
                .insert_generator(
                    Computable::rational(Rational::new(primes[16])).sqrt(),
                    2,
                )
                .is_none(),
            "a seventeenth generator must fail closed"
        );

        let mut too_deep = Computable::one();
        for _ in 0..257 {
            too_deep = Computable {
                internal: Arc::new(Node::new(
                    Approximation::Negate(too_deep),
                    BoundCache::Invalid,
                    ExactSignCache::Invalid,
                )),
                signal: None,
            };
        }
        assert!(too_deep.algebraic_separation_bound_bits().is_none());
    }

    #[test]
    fn binary64_filter_respects_floor_and_matches_fine_certified_signs() {
        let near_pi = Computable::pi().add(Computable::rational(
            -Rational::fraction(103_993, 33_102).unwrap(),
        ));
        assert_eq!(near_pi.binary64_filter_sign_until(0), None);
        assert_eq!(near_pi.binary64_filter_sign_until(-64), Some(Sign::Plus));

        let matrix_like = Computable::pi().square().add(
            Computable::e().multiply(Computable::rational(
                -Rational::fraction(29, 21).unwrap(),
            )),
        );
        assert_eq!(
            matrix_like.binary64_filter_sign_until(0),
            Some(Sign::Plus)
        );

        let supported_wrappers = [
            Computable::pi()
                .add(Computable::one())
                .inverse()
                .shift_left(4)
                .negate(),
            Computable::e()
                .add(Computable::rational(Rational::new(-3)))
                .square(),
            Computable::linear_combination3(
                [Computable::pi(), Computable::e(), Computable::one()],
                [
                    Rational::new(2),
                    Rational::new(-1),
                    Rational::fraction(1, 3).unwrap(),
                ],
            ),
        ];
        for (index, value) in supported_wrappers.into_iter().enumerate() {
            let filtered = value
                .binary64_filter_sign_until(-64)
                .unwrap_or_else(|| panic!("supported wrapper {index} should be separated"));
            let approximation = value.approx(-256);
            assert!(approximation.abs() > BigInt::one());
            assert_eq!(filtered, approximation.sign());
        }

        let mut over_budget = Computable::e();
        for _ in 0..40 {
            over_budget = over_budget.multiply(Computable::pi());
        }
        assert_eq!(over_budget.binary64_filter_sign_until(0), None);

        let mut state = 0x7d31_9a5b_4c27_18e3_u64;
        for _ in 0..10_000 {
            let mut next = || {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                state
            };
            let coefficient = |bits: u64, denominator_bits: u64| {
                let numerator = i64::try_from(bits % 401).unwrap() - 200;
                let denominator = denominator_bits % 97 + 1;
                Rational::fraction(numerator, denominator).unwrap()
            };
            let a = coefficient(next(), next());
            let b = coefficient(next(), next());
            let c = coefficient(next(), next());
            let value = Computable::pi()
                .multiply(Computable::rational(a))
                .add(Computable::e().multiply(Computable::rational(b)))
                .add(Computable::rational(c));

            for floor in [0, -16, -64] {
                let Some(filtered) = value.binary64_filter_sign_until(floor) else {
                    continue;
                };
                let approximation = value.approx(-256);
                assert!(
                    approximation.abs() > BigInt::one() || filtered == Sign::NoSign,
                    "fine approximation should separate every filtered nonzero value"
                );
                assert_eq!(filtered, approximation.sign());
            }
        }
    }

    #[test]
    fn bigger() {
        let six: BigInt = "6".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let four: BigInt = "4".parse().unwrap();
        let a = Computable::integer(six.clone());
        let b = Computable::integer(five.clone());
        assert_eq!(a.compare_absolute(&b, 0), Ordering::Greater);
        let c = Computable::integer(four.clone());
        assert_eq!(c.compare_absolute(&a, 0), Ordering::Less);
        assert_eq!(b.compare_absolute(&b, 0), Ordering::Equal);
    }

    #[test]
    fn shifted() {
        let one = BigInt::one();
        let two = &one + &one;
        assert_eq!(one, shift(two, -1));
    }

    #[test]
    fn prec() {
        let nine: BigInt = "9".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let two: BigInt = "2".parse().unwrap();
        let one = BigInt::one();
        let a = Computable::integer(nine.clone());
        assert_eq!(nine, a.approx(0));
        assert_eq!(five, a.approx(1));
        assert_eq!(two, a.approx(2));
        assert_eq!(one, a.approx(3));
        assert!(a.internal.cache_snapshot().is_none());
    }

    #[test]
    fn prec_pi() {
        let three: BigInt = "3".parse().unwrap();
        let six: BigInt = "6".parse().unwrap();
        let thirteen: BigInt = "13".parse().unwrap();
        let four_zero_two: BigInt = "402".parse().unwrap();
        let a = Computable::pi();
        assert_eq!(four_zero_two, a.approx(-7));
        assert_eq!(three, a.approx(0));
        assert_eq!(six, a.approx(-1));
        assert_eq!(thirteen, a.approx(-2));
        assert!(a.cached().is_some_and(|(precision, _)| precision <= -7));
    }

    #[test]
    fn rational_zero_and_one_use_dedicated_nodes() {
        let zero = Computable::rational(Rational::zero());
        let one = Computable::rational(Rational::one());

        // These identities are pervasive in higher-level constructors. Keep
        // them on the dedicated nodes so structural facts are available without
        // forcing the generic Ratio approximation path.
        assert!(matches!(&zero.internal.approximation, Approximation::Int(value) if value.is_zero()));
        assert!(matches!(
            &one.internal.approximation,
            Approximation::One
        ));
        assert_eq!(zero.zero_status(), ZeroKnowledge::Zero);
        assert_eq!(one.zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(zero.exact_sign(), Some(Sign::NoSign));
        assert_eq!(one.exact_sign(), Some(Sign::Plus));
    }

    #[test]
    fn clones_share_immutable_expression_nodes() {
        let value = Computable::pi().add(Computable::one());
        let clone = value.clone();

        assert!(Arc::ptr_eq(&value.internal, &clone.internal));
        assert_eq!(value.approx(-32), clone.approx(-32));
    }

    #[test]
    fn computable_is_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}

        assert_send_sync::<Computable>();
        assert_send_sync::<Rational>();
        assert_send_sync::<crate::Real>();
    }

    #[test]
    fn cloned_expression_can_be_refined_concurrently() {
        let expression = || {
            Computable::pi()
                .multiply(Computable::rational(Rational::fraction(7, 3).unwrap()))
                .sin()
                .exp()
        };
        let expected = expression().approx(-192);
        let value = expression();

        std::thread::scope(|scope| {
            for precision in [-32, -64, -96, -128, -160, -192] {
                let clone = value.clone();
                let expected = expected.clone();
                scope.spawn(move || {
                    let actual = clone.approx(precision);
                    let expected = scale(expected, -192 - precision);
                    assert!((actual - expected).abs() <= BigInt::one());
                });
            }
        });

        assert!(matches!(value.internal.cache_snapshot(), Some((p, _)) if p <= -192));
    }

    #[test]
    fn aborted_approximation_is_not_shared_through_cache() {
        use std::sync::atomic::AtomicBool;

        let value = Computable::prescaled_atan(BigInt::from(5));
        let signal = Some(Arc::new(AtomicBool::new(true)));
        let _ = value.approx_signal(&signal, -128);

        assert!(value.internal.cache_snapshot().is_none());
    }

    #[test]
    fn layout_sizes() {
        assert!(
            size_of::<Computable>() <= 16,
            "Computable grew to {} bytes",
            size_of::<Computable>()
        );
        assert!(
            size_of::<Approximation>() <= 40,
            "Approximation grew to {} bytes",
            size_of::<Approximation>()
        );
        assert!(
            size_of::<Node>() <= 56,
            "shared expression node grew to {} bytes",
            size_of::<Node>()
        );
        assert!(
            size_of::<BoundCache>() <= 12,
            "BoundCache grew to {} bytes",
            size_of::<BoundCache>()
        );
        assert!(
            size_of::<ExactSignCache>() <= 1,
            "ExactSignCache grew to {} bytes",
            size_of::<ExactSignCache>()
        );
    }

    #[test]
    fn certified_small_angle_quotients_match_mpfr() {
        use rug::{Float, float::Round};

        fn rounded_scaled(mut value: Float, precision: Precision) -> BigInt {
            value <<= precision.saturating_neg();
            value
                .to_integer_round(Round::Nearest)
                .expect("finite small-angle oracle")
                .0
                .to_string()
                .parse()
                .expect("MPFR integer parses as BigInt")
        }

        let mut state = 0x6a09_e667_f3bc_c909_u64;
        let mut inputs = vec![
            (0_i64, 1_u64),
            (-1, 2),
            (1, 2),
            (-255, 512),
            (255, 512),
            (-639, 1_024),
            (639, 1_024),
        ];
        for _ in 0..128 {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let numerator = i64::try_from(state % 1_001).unwrap() - 500;
            inputs.push((numerator, 1_024));
        }

        for (numerator, denominator) in inputs {
            let argument = Computable::rational(
                Rational::fraction(numerator, denominator).expect("nonzero denominator"),
            );
            let sinc = argument
                .clone()
                .sinc_small_if_certified()
                .expect("test input is within the certified small-angle window");
            let cosc = argument
                .cosc_small_if_certified()
                .expect("test input is within the certified small-angle window");

            for precision in [4_i32, 1, 0, -1, -7, -31, -128, -511, -1_024] {
                let oracle_precision = precision
                    .saturating_neg()
                    .unsigned_abs()
                    .saturating_add(256);
                let mut x = Float::with_val(oracle_precision, numerator);
                x /= denominator;

                let sinc_oracle = if numerator == 0 {
                    Float::with_val(oracle_precision, 1)
                } else {
                    let mut value = x.clone().sin();
                    value /= &x;
                    value
                };
                let cosc_oracle = if numerator == 0 {
                    Float::with_val(oracle_precision, 0.5)
                } else {
                    let mut value = Float::with_val(oracle_precision, 1);
                    value -= x.clone().cos();
                    let mut square = x.clone();
                    square.square_mut();
                    value /= square;
                    value
                };

                let expected_sinc = rounded_scaled(sinc_oracle, precision);
                let expected_cosc = rounded_scaled(cosc_oracle, precision);
                let sinc_error = (sinc.approx(precision) - expected_sinc).abs();
                let cosc_error = (cosc.approx(precision) - expected_cosc).abs();
                assert!(
                    sinc_error <= BigInt::one(),
                    "sinc({numerator}/{denominator}) at {precision} differs by {sinc_error} ulps"
                );
                assert!(
                    cosc_error <= BigInt::one(),
                    "cosc({numerator}/{denominator}) at {precision} differs by {cosc_error} ulps"
                );
            }
        }

        assert!(
            Computable::rational(Rational::new(1))
                .sinc_small_if_certified()
                .is_none(),
            "the local series must reject an uncertified large argument"
        );

        use std::sync::{Arc, atomic::AtomicBool};
        let signal = Arc::new(AtomicBool::new(false));
        let mut signaled = Computable::rational(Rational::fraction(1, 2).unwrap());
        signaled.abort(Arc::clone(&signal));
        let continued = signaled
            .sinc_small_if_certified()
            .expect("a signaled small input remains certified");
        assert!(
            continued
                .signal
                .as_ref()
                .is_some_and(|attached| Arc::ptr_eq(attached, &signal)),
            "the removable quotient must retain its caller's abort signal"
        );
    }

    #[test]
    fn prec_atan_5() {
        let five: BigInt = "5".parse().unwrap();
        let atan_5 = Computable::prescaled_atan(five);
        let two_zero_two: BigInt = "202".parse().unwrap();
        assert_eq!(two_zero_two, atan_5.approx(-10));
        let at_twenty: BigInt = "206984".parse().unwrap();
        assert_eq!(at_twenty, atan_5.approx(-20));
    }

    #[test]
    fn prec_atan_239() {
        let two_three_nine: BigInt = "239".parse().unwrap();
        let atan_239 = Computable::prescaled_atan(two_three_nine);
        let four: BigInt = "4".parse().unwrap();
        assert_eq!(four, atan_239.approx(-10));
        let at_twenty: BigInt = "4387".parse().unwrap();
        assert_eq!(at_twenty, atan_239.approx(-20));
    }

    #[test]
    fn msd() {
        let one: BigInt = "1".parse().unwrap();
        let a = Computable::integer(one.clone());
        assert_eq!(Some(0), a.msd(-4));
        let three: BigInt = "3".parse().unwrap();
        let d = Computable::integer(three.clone());
        assert_eq!(Some(1), d.msd(-4));
        let five: BigInt = "5".parse().unwrap();
        let e = Computable::integer(five.clone());
        assert_eq!(Some(2), e.msd(-4));
        let seven: BigInt = "7".parse().unwrap();
        let f = Computable::integer(seven.clone());
        assert_eq!(Some(2), f.msd(-4));
        let eight: BigInt = "8".parse().unwrap();
        let g = Computable::integer(eight.clone());
        assert_eq!(Some(3), g.msd(-4));
    }

    #[test]
    fn iter_msd() {
        let one = Computable::one();
        assert_eq!(one.iter_msd(), 0);
        let pi = Computable::pi();
        assert_eq!(pi.iter_msd(), 1);
        let five = Rational::new(5);
        let e = Computable::exp_rational(five);
        assert_eq!(e.iter_msd(), 7);
    }

    #[test]
    fn e_constant_cache_is_shared() {
        let e = Computable::e_constant();
        let _ = e.approx(-32);

        let cached = Computable::e_constant()
            .cached()
            .expect("e cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn exp_one_uses_dedicated_e_constant() {
        let e = Computable::rational(Rational::one()).exp();
        assert!(matches!(
            &e.internal.approximation,
            Approximation::Constant(SharedConstant::E)
        ));
    }

    #[test]
    fn pi_cache_is_shared() {
        let pi = Computable::pi();
        let _ = pi.approx(-32);

        let cached = Computable::pi()
            .cached()
            .expect("pi cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn tau_cache_is_shared() {
        let tau = Computable::tau();
        let _ = tau.approx(-32);

        let cached = Computable::tau()
            .cached()
            .expect("tau cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn tau_cache_reuses_warmed_pi_cache() {
        std::thread::spawn(|| {
            let pi = Computable::pi();
            let _ = pi.approx(-64);

            let tau_appr = Computable::tau().approx(-32);
            let pi_scaled_as_tau = Computable::pi().approx(-33);
            assert_eq!(tau_appr, pi_scaled_as_tau);

            let cached = Computable::tau()
                .cached()
                .expect("tau cache should be filled from pi cache");
            assert!(cached.0 <= -32);
            assert_eq!(Computable::tau().approx(-32), tau_appr);
        })
        .join()
        .expect("tau cache test thread should finish");
    }

    #[test]
    fn pi_cache_reuses_warmed_tau_cache() {
        std::thread::spawn(|| {
            let tau = Computable::tau();
            let _ = tau.approx(-65);

            let pi_appr = Computable::pi().approx(-64);
            let tau_scaled_as_pi = Computable::tau().approx(-63);
            assert_eq!(pi_appr, tau_scaled_as_pi);

            let cached = Computable::pi()
                .cached()
                .expect("pi cache should be filled from tau cache");
            assert!(cached.0 <= -64);
            assert_eq!(Computable::pi().approx(-64), pi_appr);
        })
        .join()
        .expect("pi cache test thread should finish");
    }

    #[test]
    fn shared_constant_cache_reuses_work_across_threads() {
        let expected = std::thread::spawn(|| Computable::pi().approx(-384))
            .join()
            .expect("constant producer thread should finish");
        let (cached, actual) = std::thread::spawn(|| {
            (Computable::pi().cached(), Computable::pi().approx(-384))
        })
        .join()
        .expect("constant consumer thread should finish");

        assert!(cached.is_some_and(|(precision, _)| precision <= -384));
        assert_eq!(actual, expected);
    }

    #[test]
    fn ln_constant_cache_is_shared() {
        let ln2 = Computable::ln_constant(2).unwrap();
        let _ = ln2.approx(-32);

        let cached = Computable::ln_constant(2)
            .unwrap()
            .cached()
            .expect("ln constant cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn negate() {
        let fifteen: BigInt = "15".parse().unwrap();
        let a = Computable::integer(fifteen.clone());
        let b = Computable::negate(a);
        let answer: BigInt = "-7".parse().unwrap();
        assert_eq!(answer, b.approx(1));
    }

    #[test]
    fn multiply() {
        let four: BigInt = "4".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(four);
        let b = Computable::prescaled_atan(five);
        let m = Computable::multiply(a, b);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn multiply_opposite() {
        let four: BigInt = "4".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(four);
        let b = Computable::prescaled_atan(five);
        let m = Computable::multiply(b, a);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn rational() {
        let sixth: Rational = "1/6".parse().unwrap();
        let c = Computable::rational(sixth);
        let zero = BigInt::zero();
        let one = BigInt::one();
        let ten: BigInt = "10".parse().unwrap();
        let eighty_five: BigInt = "85".parse().unwrap();
        assert_eq!(zero, c.approx(0));
        assert_eq!(zero, c.approx(-1));
        assert_eq!(zero, c.approx(-2));
        assert_eq!(one, c.approx(-3));
        assert_eq!(ten, c.approx(-6));
        assert_eq!(eighty_five, c.approx(-9));
    }

    #[test]
    fn scaled_ln1() {
        let zero = Computable::integer(BigInt::zero());
        let ln = Computable {
            internal: Arc::new(Node::new(Approximation::PrescaledLn(zero), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        };
        let zero = BigInt::zero();
        assert_eq!(zero, ln.approx(100));
    }

    #[test]
    fn scaled_ln1_4() {
        let zero_4: Rational = "0.4".parse().unwrap();
        let rational = Computable::rational(zero_4);
        let ln = Computable {
            internal: Arc::new(Node::new(Approximation::PrescaledLn(rational), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        };
        let five: BigInt = "5".parse().unwrap();
        assert_eq!(five, ln.approx(-4));
    }

    #[test]
    fn ln() {
        let five: BigInt = "5".parse().unwrap();
        let integer = Computable::integer(five);
        let ln = Computable::ln(integer);
        let correct: BigInt = "1769595698905".parse().unwrap();
        assert_eq!(ln.approx(-40), correct);
    }

    #[test]
    fn exp_and_ln_round_trip() {
        let seven_fifths = Computable::rational(Rational::fraction(7, 5).unwrap());
        assert_close(seven_fifths.clone().exp().ln(), seven_fifths, -40, 2);
    }

    #[test]
    fn exact_transcendental_identities() {
        let zero = Computable::rational(Rational::zero());
        let one = Computable::rational(Rational::one());
        assert_close(zero.clone().exp(), one.clone(), -40, 0);
        assert_close(one.ln(), zero.clone(), -40, 0);
        assert_close(zero.clone().sin(), zero.clone(), -40, 0);
        assert_close(zero.clone().cos(), Computable::one(), -40, 0);
        assert_close(zero.tan(), Computable::rational(Rational::zero()), -40, 0);
    }

    #[test]
    fn compare_to_uses_exact_sign_and_rational_shortcuts() {
        let minus_pi = Computable::pi().negate();
        let pi = Computable::pi();
        assert_eq!(minus_pi.try_compare_to(&pi), Some(Ordering::Less));

        let left = Computable::rational(Rational::fraction(7, 8).unwrap());
        let right = Computable::rational(Rational::fraction(9, 10).unwrap());
        assert_eq!(left.try_compare_to(&right), Some(Ordering::Less));
    }

    #[test]
    fn try_compare_to_handles_identical_symbolic_values() {
        let pi = Computable::pi();
        assert_eq!(pi.try_compare_to(&pi), Some(Ordering::Equal));

        let left = Computable::rational(Rational::fraction(3, 7).unwrap());
        let right = Computable::rational(Rational::fraction(3, 7).unwrap());
        assert_eq!(left.try_compare_to(&right), Some(Ordering::Equal));
    }

    #[test]
    fn compare_to_uses_exact_msd_gap_shortcut() {
        let base = Computable::pi();
        base.approx(-16);
        let huge = base
            .clone()
            .multiply(Computable::rational(Rational::from_bigint(
                BigInt::from(1_u8) << 200,
            )));
        assert_eq!(huge.try_compare_to(&base), Some(Ordering::Greater));
        assert_eq!(base.try_compare_to(&huge), Some(Ordering::Less));

        let minus_base = base.negate();
        let minus_huge = huge.negate();
        assert_eq!(minus_huge.try_compare_to(&minus_base), Some(Ordering::Less));
        assert_eq!(
            minus_base.try_compare_to(&minus_huge),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn compare_to_accepts_clone_shared_composite_expression() {
        let expression = Computable::pi()
            .multiply(Computable::e())
            .add(Computable::rational(Rational::fraction(7, 13).unwrap()));
        let shared = expression.clone();

        assert!(Arc::ptr_eq(&expression.internal, &shared.internal));
        assert_eq!(expression.try_compare_to(&shared), Some(Ordering::Equal));
    }

    #[test]
    fn compare_absolute_uses_exact_shortcuts() {
        let zero = Computable::rational(Rational::zero());
        let tiny = Computable::rational(Rational::fraction(1, 1024).unwrap());
        assert_eq!(zero.compare_absolute(&tiny, -40), Ordering::Less);

        let left = Computable::rational(Rational::fraction(-7, 8).unwrap());
        let right = Computable::rational(Rational::fraction(9, 10).unwrap());
        assert_eq!(left.compare_absolute(&right, -40), Ordering::Less);
    }

    #[test]
    fn compare_absolute_orders_mixed_exact_leaf_kinds_by_signed_value() {
        let one = Computable::one();
        let negative_large =
            Computable::rational(Rational::fraction(-100, 3).expect("three is nonzero"));
        let positive_large =
            Computable::rational(Rational::fraction(100, 3).expect("three is nonzero"));
        let zero = Computable::rational(Rational::zero());

        // `One` versus `Ratio` reaches the general exact-rational shortcut
        // rather than the same-kind leaf shortcut.
        assert_eq!(
            one.compare_absolute(&negative_large, -40),
            Ordering::Greater
        );
        assert_eq!(
            negative_large.compare_absolute(&one, -40),
            Ordering::Less
        );
        assert_eq!(zero.compare_absolute(&negative_large, -40), Ordering::Greater);
        assert_eq!(negative_large.compare_absolute(&zero, -40), Ordering::Less);
        assert_eq!(zero.compare_absolute(&positive_large, -40), Ordering::Less);
        assert_eq!(positive_large.compare_absolute(&zero, -40), Ordering::Greater);
    }

    #[test]
    fn compare_absolute_orders_zero_and_signed_irrationals_by_value() {
        let zero = Computable::zero();
        let pi = Computable::pi();
        let minus_pi = pi.clone().negate();

        assert_eq!(zero.compare_absolute(&minus_pi, -16), Ordering::Greater);
        assert_eq!(minus_pi.compare_absolute(&zero, -16), Ordering::Less);
        assert_eq!(zero.compare_absolute(&pi, -16), Ordering::Less);
        assert_eq!(pi.compare_absolute(&zero, -16), Ordering::Greater);
    }

    #[test]
    fn compare_to_orders_structurally_shared_signed_perturbations() {
        let cases = [
            (Computable::pi(), Computable::one(), Ordering::Greater),
            (
                Computable::pi(),
                Computable::one().negate(),
                Ordering::Less,
            ),
            (
                Computable::pi().negate(),
                Computable::one(),
                Ordering::Greater,
            ),
            (
                Computable::pi().negate(),
                Computable::one().negate(),
                Ordering::Less,
            ),
        ];

        for (base, perturbation, expected) in cases {
            let perturbed = base.clone().add(perturbation);
            assert_eq!(
                perturbed.try_compare_to(&base),
                Some(expected),
                "perturbed value should be ordered by the signed perturbation"
            );
            assert_eq!(
                base.try_compare_to(&perturbed),
                Some(expected.reverse()),
                "reverse comparison should reverse the signed perturbation"
            );
        }
    }

    #[test]
    fn compare_to_retains_exact_perturbation_sign_below_refinement_floor() {
        let base = Computable::pi();
        let tiny = Computable::rational(
            Rational::new(2)
                .powi(BigInt::from(-100))
                .expect("two is nonzero"),
        );
        let above = base.clone().add(tiny.clone());
        let below = base.clone().add(tiny.negate());

        assert_eq!(
            above.try_compare_to_until(&base, -16),
            Some(Ordering::Greater)
        );
        assert_eq!(
            base.try_compare_to_until(&above, -16),
            Some(Ordering::Less)
        );
        assert_eq!(below.try_compare_to_until(&base, -16), Some(Ordering::Less));
        assert_eq!(
            base.try_compare_to_until(&below, -16),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn certified_compare_cascade_matches_separated_full_evaluation_corpus() {
        fn evaluated_order(
            left: &Computable,
            right: &Computable,
            precision: Precision,
        ) -> Option<Ordering> {
            let left = left.approx(precision);
            let right = right.approx(precision);
            let error_width = BigInt::from(2_u8);
            if left > &right + &error_width {
                Some(Ordering::Greater)
            } else if right > left + error_width {
                Some(Ordering::Less)
            } else {
                None
            }
        }

        let pi = Computable::pi();
        let e = Computable::e();
        let root_two = Computable::rational(Rational::new(2)).sqrt();
        let atan_third =
            Computable::rational(Rational::fraction(1, 3).unwrap()).atan();
        let values = [
            Computable::zero(),
            Computable::one(),
            Computable::one().negate(),
            pi.clone(),
            pi.clone().negate(),
            e.clone(),
            e.negate(),
            root_two.clone(),
            root_two.negate(),
            pi.clone().add(Computable::one()),
            pi.clone().add(Computable::one().negate()),
            pi.clone().negate().add(Computable::one()),
            pi.clone().negate().add(Computable::one().negate()),
            atan_third.clone(),
            atan_third.negate(),
            pi.clone().inverse(),
            pi.inverse().negate(),
        ];

        for left in &values {
            for right in &values {
                let Some(expected) = evaluated_order(left, right, -256) else {
                    continue;
                };
                assert_eq!(
                    left.try_compare_to_until(right, -128),
                    Some(expected),
                    "certified comparison disagreed with separated evaluation"
                );
            }
        }
    }

    #[test]
    fn compare_absolute_uses_exact_msd_gap_shortcut() {
        let base = Computable::pi();
        base.approx(-16);
        let huge = base
            .clone()
            .multiply(Computable::rational(Rational::from_bigint(
                BigInt::from(1_u8) << 200,
            )));
        assert_eq!(huge.compare_absolute(&base, -40), Ordering::Greater);
        assert_eq!(base.compare_absolute(&huge, -40), Ordering::Less);
    }

    #[test]
    fn unsigned_magnitude_facts_do_not_determine_signed_order() {
        fn with_unsigned_magnitude(value: Computable, msd: Precision) -> Computable {
            Computable {
                internal: Arc::new(Node::new(
                    Approximation::Offset(value, 0),
                    BoundCache::Valid(BoundInfo::NonZero {
                        sign: None,
                        msd: Some(msd),
                        exact_msd: true,
                    }),
                    ExactSignCache::Unknown,
                )),
                signal: None,
            }
        }

        let large_negative =
            with_unsigned_magnitude(Computable::rational(Rational::new(-8)), 3);
        let small_negative = with_unsigned_magnitude(
            Computable::rational(Rational::fraction(-1, 8).unwrap()),
            -3,
        );

        assert_eq!(
            large_negative.try_compare_to_until(&small_negative, 0),
            Some(Ordering::Less)
        );
        assert_eq!(
            small_negative.try_compare_to_until(&large_negative, 0),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn warmed_zero_sum_product_stays_zero() {
        let zero = Computable::pi().add(Computable::pi().negate());
        zero.approx(-128);
        let product = zero.multiply(Computable::pi());
        assert_eq!(product.approx(-128), BigInt::zero());
    }

    #[test]
    fn exp_negative_is_inverse() {
        let eleven_tenths = Computable::rational(Rational::fraction(11, 10).unwrap());
        let product = eleven_tenths
            .clone()
            .exp()
            .multiply(eleven_tenths.negate().exp());
        assert_close(product, Computable::one(), -40, 2);
    }

    #[test]
    fn exp_near_prescaled_limit_round_trip() {
        let half = Computable::rational(Rational::fraction(1, 2).unwrap());
        assert_close(half.clone().exp().ln(), half, -40, 2);
    }

    #[test]
    fn exp_large_argument_reduces_by_ln2() {
        let exponent = BigInt::from(200);
        let offset = Computable::rational(Rational::fraction(7, 20).unwrap());
        let value = Computable::ln2()
            .multiply(Computable::integer(exponent.clone()))
            .add(offset.clone());
        let expected = offset.exp().shift_left(200);

        assert_close(value.exp(), expected, -80, 2);
    }

    #[test]
    fn bounded_integer_exp_matches_ln2_reduction() {
        fn compare(exponent: i32, precision: Precision) {
            let input = Computable::rational(Rational::new(i64::from(exponent)));
            let optimized = input.clone().exp();
            let (reduced, multiple) = input
                .reduce_by_divisor(&Computable::ln2(), -4, 64)
                .expect("bounded integer has an ln2 reduction");
            let reference = reduced.prescaled_exp().shift_left(
                multiple
                    .try_into()
                    .expect("bounded integer reduction fits i32"),
            );
            let optimized = optimized.approx(precision);
            let reference = reference.approx(precision);
            let error = (&optimized - &reference).abs();
            assert!(
                error <= BigInt::from(2),
                "exp({exponent}) at {precision}: optimized {optimized}, reference {reference}, error {error}"
            );
        }

        for exponent in 2..=256 {
            compare(exponent, -40);
        }
        for exponent in [2, 13, 128, 256] {
            compare(exponent, -128);
        }
    }

    #[test]
    fn integer_exp_uses_binary_power_only_through_the_retained_limit() {
        let at_limit = Computable::rational(Rational::new(256)).exp();
        assert!(matches!(
            &at_limit.internal.approximation,
            Approximation::Square(_)
        ));

        let above_limit = Computable::rational(Rational::new(257)).exp();
        assert!(matches!(
            &above_limit.internal.approximation,
            Approximation::Offset(child, _)
                if matches!(&child.internal.approximation, Approximation::PrescaledExp(_))
        ));
    }

    #[test]
    fn cos_zero() {
        let zero = Computable::rational(Rational::zero());
        let cos = zero.cos();
        let correct: BigInt = "4294967296".parse().unwrap();
        assert_eq!(cos.approx(-32), correct);
    }

    #[test]
    fn cos_one() {
        let one = Computable::one();
        let cos = one.cos();
        let correct: BigInt = "2320580734".parse().unwrap();
        assert_eq!(cos.approx(-32), correct);
    }

    fn assert_approx(c: Computable, p: Precision, expected: &str, max_error: i32) {
        let actual = c.approx(p);
        let expected: BigInt = expected.parse().unwrap();
        let error = (&actual - &expected).abs();
        let max_error = BigInt::from(max_error);
        assert!(
            error <= max_error,
            "actual {actual}, expected {expected}, error {error}"
        );
    }

    fn assert_close(left: Computable, right: Computable, p: Precision, max_error: i32) {
        let left = left.approx(p);
        let right = right.approx(p);
        let error = (&left - &right).abs();
        let max_error = BigInt::from(max_error);
        assert!(
            error <= max_error,
            "left {left}, right {right}, error {error}"
        );
    }

    fn pi_times(r: Rational) -> Computable {
        Computable::pi().multiply(Computable::rational(r))
    }

    fn shifted_cos_sin(c: Computable) -> Computable {
        pi_times(Rational::fraction(1, 2).unwrap())
            .add(c.negate())
            .cos()
    }

    #[test]
    fn sin_small_arguments() {
        let one_fifth = Computable::rational(Rational::fraction(1, 5).unwrap());
        assert_approx(one_fifth.sin(), -32, "853278278", 1);

        let zero = Computable::rational(Rational::zero());
        assert_eq!(BigInt::zero(), zero.sin().approx(-32));
    }

    #[test]
    fn sin_medium_arguments() {
        let three: BigInt = "3".parse().unwrap();
        let three = Computable::integer(three);
        assert_approx(three.sin(), -32, "606105819", 1);
    }

    #[test]
    fn sin_cos_direct_medium_exact_rationals_match_reduced_forms() {
        for rational in [
            Rational::fraction(6, 5).unwrap(),
            Rational::fraction(7, 5).unwrap(),
            Rational::fraction(47, 32).unwrap(),
            Rational::try_from(1.23456789_f64).unwrap(),
        ] {
            let value = Computable::rational(rational);
            let complement =
                pi_times(Rational::fraction(1, 2).unwrap()).add(value.clone().negate());

            assert_close(value.clone().sin(), complement.clone().cos(), -96, 2);
            assert_close(value.clone().cos(), complement.sin(), -96, 2);
            assert_close(
                value.clone().negate().sin(),
                value.clone().sin().negate(),
                -96,
                2,
            );
            assert_close(value.clone().negate().cos(), value.cos(), -96, 2);
        }
    }

    #[test]
    fn owned_rational_trig_helpers_match_generic_paths() {
        for rational in [
            Rational::fraction(-1, 5).unwrap(),
            Rational::fraction(1, 5).unwrap(),
            Rational::fraction(6, 5).unwrap(),
            Rational::fraction(7, 5).unwrap(),
            Rational::new(1_000_000),
        ] {
            let generic = Computable::rational(rational.clone());

            assert_close(
                Computable::sin_rational(rational.clone()),
                generic.clone().sin(),
                -80,
                8,
            );
            assert_close(
                Computable::cos_rational(rational.clone()),
                generic.clone().cos(),
                -80,
                8,
            );
            assert_close(Computable::tan_rational(rational), generic.tan(), -80, 16);
        }
    }

    #[test]
    fn sin_large_arguments() {
        let one_two_three: BigInt = "123".parse().unwrap();
        let one_two_three = Computable::integer(one_two_three);
        assert_approx(one_two_three.sin(), -32, "-1975270452", 1);
    }

    #[test]
    fn sin_negative_arguments() {
        let negative_three_fifths = Computable::rational(Rational::fraction(-3, 5).unwrap());
        assert_approx(negative_three_fifths.sin(), -32, "-2425120957", 1);
    }

    #[test]
    fn sin_near_pi_multiples() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let pi_plus_epsilon = Computable::pi().add(epsilon.clone());
        let two_pi_minus_epsilon = pi_times(Rational::new(2)).add(epsilon.clone().negate());

        assert_approx(pi_plus_epsilon.sin(), -32, "-67106133", 1);
        assert_approx(two_pi_minus_epsilon.sin(), -32, "-67106133", 1);
    }

    #[test]
    fn sin_near_half_pi() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let half_pi = pi_times(Rational::fraction(1, 2).unwrap());
        let half_pi_plus_epsilon = half_pi.clone().add(epsilon.clone());
        let half_pi_minus_epsilon = half_pi.add(epsilon.negate());

        assert_approx(half_pi_plus_epsilon.sin(), -32, "4294443019", 1);
        assert_approx(half_pi_minus_epsilon.sin(), -32, "4294443019", 1);
    }

    #[test]
    fn sin_matches_shifted_cos_identity() {
        for r in ["-12", "-3/5", "0", "1/5", "3", "123"] {
            let r: Rational = r.parse().unwrap();
            let c = Computable::rational(r);
            assert_close(c.clone().sin(), shifted_cos_sin(c), -40, 1);
        }

        for r in ["-7/3", "-1/2", "1/2", "2", "41/6"] {
            let r: Rational = r.parse().unwrap();
            let c = pi_times(r);
            assert_close(c.clone().sin(), shifted_cos_sin(c), -40, 1);
        }
    }

    #[test]
    fn inverse_trig_computable_kernels_approximate_expected_values() {
        let value = Computable::rational(Rational::fraction(7, 10).unwrap());
        let negative_value = Computable::rational(Rational::fraction(-7, 10).unwrap());

        assert_approx(value.clone().asin(), -40, "852558563672", 2);
        assert_approx(negative_value.asin(), -40, "-852558563672", 2);
        assert_approx(value.acos(), -40, "874550262507", 2);
    }

    #[test]
    fn endpoint_inverse_trig_computable_kernels_approximate_expected_values() {
        let tiny = Computable::rational(Rational::fraction(1, 1_000_000_000_000).unwrap());
        let near_one = Computable::rational(Rational::fraction(999_999, 1_000_000).unwrap());

        assert_approx(tiny.clone().asin(), -80, "1208925819615", 2);
        assert_approx(tiny.clone().acos(), -40, "1727108826178", 2);
        assert_approx(tiny.atanh(), -80, "1208925819615", 2);
        assert_approx(near_one.clone().asin(), -40, "1725553881793", 2);
        assert_approx(near_one.clone().acos(), -40, "1554944386", 2);
        assert_approx(near_one.atanh(), -40, "7976218668587", 2);
    }

    #[test]
    fn signed_rational_asin_deferred_node_matches_acos_complement() {
        for input in [
            Rational::fraction(-999_999, 1_000_000).unwrap(),
            Rational::fraction(-7, 8).unwrap(),
            Rational::fraction(-7, 10).unwrap(),
            Rational::fraction(-3, 10).unwrap(),
            Rational::fraction(-1, 8).unwrap(),
            Rational::fraction(1, 8).unwrap(),
            Rational::fraction(3, 10).unwrap(),
            Rational::fraction(7, 10).unwrap(),
            Rational::fraction(7, 8).unwrap(),
            Rational::fraction(999_999, 1_000_000).unwrap(),
        ] {
            let direct = Computable::rational(input.clone()).asin();
            assert!(matches!(
                &direct.internal.approximation,
                Approximation::AsinRational(stored) if stored == &input
            ));

            let sign = input.sign();
            let magnitude = if sign == Sign::Minus {
                -input
            } else {
                input
            };
            let complement = Computable::pi()
                .shift_right(1)
                .add(Computable::rational(magnitude).acos().negate());
            let reference = if sign == Sign::Minus {
                complement.negate()
            } else {
                complement
            };

            for precision in [-16, -40, -96, -256] {
                assert_close(direct.clone(), reference.clone(), precision, 2);
            }
        }
    }

    #[test]
    fn tiny_non_rational_asin_uses_prescaled_series() {
        let tiny = Computable::rational(Rational::new(2))
            .sqrt()
            .shift_left(-20);
        let result = tiny.clone().asin();

        assert!(matches!(
            &result.internal.approximation,
            Approximation::PrescaledAsin(_)
        ));
        assert_close(result, Computable::asin_deferred(tiny), -80, 4);
    }

    #[test]
    fn tiny_non_rational_atanh_uses_prescaled_series() {
        let tiny = Computable::rational(Rational::new(2))
            .sqrt()
            .shift_left(-20);
        let result = tiny.clone().atanh();

        assert!(matches!(
            &result.internal.approximation,
            Approximation::PrescaledAtanh(_)
        ));
        assert_close(result, Computable::atanh_direct_deferred(tiny), -80, 4);
    }

    #[test]
    fn inverse_hyperbolic_computable_kernels_approximate_expected_values() {
        let half = Computable::rational(Rational::fraction(1, 2).unwrap());
        let negative_half = Computable::rational(Rational::fraction(-1, 2).unwrap());
        let two = Computable::rational(Rational::new(2));

        assert_approx(half.clone().asinh(), -40, "529097997076", 2);
        assert_approx(negative_half.clone().asinh(), -40, "-529097997076", 2);
        assert_approx(two.acosh(), -40, "1448010520960", 2);
        assert_approx(
            Computable::rational(Rational::new(2)).sqrt().acosh(),
            -40,
            "969080507343",
            2,
        );
        assert_approx(half.atanh(), -40, "603968492904", 2);
        assert_approx(negative_half.atanh(), -40, "-603968492904", 2);
    }

    #[test]
    fn asinh_uses_series_only_below_the_half_boundary() {
        let quarter = Computable::rational(Rational::fraction(1, 4).unwrap()).asinh();
        let half = Computable::rational(Rational::fraction(1, 2).unwrap()).asinh();
        let three_quarters = Computable::rational(Rational::fraction(3, 4).unwrap()).asinh();

        assert!(matches!(
            &quarter.internal.approximation,
            Approximation::AsinhRational(_)
        ));
        assert!(!matches!(
            &half.internal.approximation,
            Approximation::AsinhRational(_)
        ));
        assert!(!matches!(
            &three_quarters.internal.approximation,
            Approximation::AsinhRational(_)
        ));
        assert_approx(three_quarters, -40, "762123384786", 2);
    }

    #[test]
    fn deep_add_chain_approximates_without_recursive_walk() {
        let mut value = Computable::one();
        for _ in 0..5000 {
            value = value.add(Computable::one());
        }

        assert_eq!(value.approx(0), BigInt::from(5001));
    }

    #[test]
    fn deep_multiply_chain_of_ones_stays_exact() {
        let mut value = Computable::one();
        for _ in 0..5000 {
            value = value.multiply(Computable::one());
        }

        assert_eq!(value.approx(0), BigInt::from(1));
    }

    #[test]
    fn deep_multiply_chain_by_one_preserves_irrational() {
        let mut value = Computable::pi();
        for _ in 0..5000 {
            value = value.multiply(Computable::one());
        }

        assert_close(value, Computable::pi(), -40, 2);
    }

    #[test]
    fn rational_msd_exact_for_small_fraction() {
        let third = Computable::rational(Rational::fraction(1, 3).unwrap());
        assert_eq!(third.msd(-4), Some(-2));
    }

    #[test]
    fn multiply_combines_exact_scales() {
        let scale = Computable::rational(Rational::fraction(7, 8).unwrap());
        let combined = Computable::pi()
            .multiply(scale.clone())
            .multiply(scale.clone())
            .multiply(scale);
        let expected =
            Computable::pi().multiply(Computable::rational(Rational::fraction(343, 512).unwrap()));
        assert_close(combined, expected, -60, 2);
    }

    #[test]
    fn square_of_scaled_irrational_reuses_exact_scale() {
        let scaled =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        let expected = Computable::pi()
            .square()
            .multiply(Computable::rational(Rational::fraction(49, 64).unwrap()));
        assert_close(scaled.square(), expected, -60, 2);
    }

    #[test]
    fn inverse_of_exact_fraction_has_structural_bound() {
        let third = Computable::rational(Rational::fraction(1, 3).unwrap());
        let inverse = third.inverse();
        assert_eq!(inverse.sign_until(0), Some(RealSign::Positive));
        assert_eq!(inverse.msd(-4), Some(1));
    }

    #[test]
    fn inverse_of_scaled_irrational_uses_structural_msd() {
        let scale = Rational::fraction(7, 8).unwrap();
        let base = Computable::pi();
        base.approx(-16);
        let value = base.multiply(Computable::rational(scale.clone()));
        assert_eq!(value.planning_msd(), Some(Some(0)));
        assert_eq!(value.msd(-4), Some(1));
        let inverse = value.inverse();
        let expected = Computable::pi()
            .inverse()
            .multiply(Computable::rational(scale.inverse().unwrap()));
        assert_close(inverse, expected, -60, 2);

        let negative_scale = Rational::fraction(-7, 8).unwrap();
        let negative_value = Computable::pi().multiply(Computable::rational(negative_scale));
        let normalized = negative_value.inverse().negate();
        let expected = Computable::pi()
            .inverse()
            .multiply(Computable::rational(Rational::fraction(8, 7).unwrap()));
        assert_close(normalized, expected, -60, 2);
    }

    #[test]
    fn square_of_negative_fraction_has_structural_bound() {
        let value = Computable::rational(Rational::fraction(-3, 8).unwrap()).square();
        assert_eq!(value.sign_until(0), Some(RealSign::Positive));
        assert_eq!(value.msd(-4), Some(-3));
    }

    #[test]
    fn sqrt_of_scaled_square_tracks_structural_msd() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(7, 8).unwrap()))
            .square()
            .sqrt();
        let expected =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        assert_eq!(value.sign_until(0), Some(RealSign::Positive));
        assert_eq!(value.msd(-4), Some(1));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_square_of_negative_value_returns_absolute_value() {
        let value = Computable::rational(Rational::fraction(-3, 8).unwrap())
            .square()
            .sqrt();
        assert_eq!(
            value.approx(-8),
            Computable::rational(Rational::fraction(3, 8).unwrap()).approx(-8)
        );
    }

    #[test]
    fn double_negate_collapses_at_construction() {
        let value = Computable::pi().negate().negate();
        assert_close(value, Computable::pi(), -60, 2);
    }

    #[test]
    fn inverse_of_inverse_of_nonzero_value_collapses_at_construction() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().inverse().inverse();
        assert_close(value, base, -60, 2);
    }

    #[test]
    fn inverse_of_square_of_nonzero_value_collapses_at_construction() {
        let base =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        let value = base.clone().square().inverse();
        let expected = base.inverse().square();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn nested_offsets_collapse_at_construction() {
        let value = Computable::pi().shift_left(5).shift_right(3);
        let expected = Computable::pi().shift_left(2);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn square_of_negative_value_collapses_to_square_of_positive_value() {
        let value = Computable::pi().negate().square();
        let expected = Computable::pi().square();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn inverse_of_negative_nonzero_value_normalizes_sign() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().negate().inverse();
        let expected = base.inverse().negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_negative_one_collapses_to_negate() {
        let minus_one = Computable::rational(Rational::one().neg());
        let value = Computable::pi().multiply(minus_one);
        let expected = Computable::pi().negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_power_of_two_fraction_collapses_to_shift() {
        let value =
            Computable::pi().multiply(Computable::rational(Rational::fraction(1, 8).unwrap()));
        let expected = Computable::pi().shift_right(3);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_negative_power_of_two_fraction_collapses_to_shift_and_negate() {
        let value =
            Computable::pi().multiply(Computable::rational(Rational::fraction(-1, 8).unwrap()));
        let expected = Computable::pi().shift_right(3).negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn square_of_power_of_two_scaled_value_collapses_to_shifted_square() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::new(8)))
            .square();
        let expected = Computable::pi().square().shift_left(6);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_exactly_scaled_square_collapses_at_construction() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(7, 8).unwrap()))
            .square()
            .sqrt();
        let expected =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_exact_rational_square_is_exact() {
        let value = Computable::rational(Rational::fraction(49, 64).unwrap()).sqrt();
        let expected = Computable::rational(Rational::fraction(7, 8).unwrap());
        assert_close(value, expected, -60, 0);
    }

    #[test]
    fn sqrt_squarefree_two_three_reuses_shared_constants() {
        let sqrt_twelve = Computable::rational(Rational::new(12)).sqrt();
        let expected = Computable::sqrt_constant(3)
            .unwrap()
            .multiply(Computable::rational(Rational::new(2)));
        assert_close(sqrt_twelve, expected, -60, 2);
    }

    #[test]
    fn independently_expanded_quadratic_products_cancel_exactly() {
        let left_sqrt = Computable::sqrt_rational(Rational::new(2));
        let right_sqrt = Computable::sqrt_rational(Rational::new(2));
        let positive = left_sqrt
            .clone()
            .multiply(right_sqrt.clone().shift_left(1))
            .shift_right(1);
        let negative = left_sqrt
            .multiply(right_sqrt.shift_left(1).negate())
            .shift_right(1);
        assert_eq!(positive.add(negative).exact_sign(), Some(Sign::NoSign));
    }

    #[test]
    fn sign_normalization_exposes_scaled_product_cancellation() {
        let left = Computable::sqrt_rational(Rational::new(2));
        let right = Computable::sqrt_rational(Rational::new(2)).shift_left(1);
        let positive = left.clone().multiply(right.clone()).shift_right(1);
        let negative = left.multiply(right.negate()).shift_right(1);
        let sum = positive.add(negative);
        assert_eq!(sum.exact_rational(), Some(Rational::zero()));
    }

    #[test]
    fn square_of_sqrt_of_positive_value_collapses_at_construction() {
        let value = Computable::rational(Rational::new(2)).sqrt().square();
        let expected = Computable::rational(Rational::new(2));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn inverse_of_shifted_nonzero_value_collapses_to_shifted_inverse() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().shift_left(5).inverse();
        let expected = base.inverse().shift_right(5);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn structural_facts_for_exact_rationals() {
        let zero = Computable::rational(Rational::zero()).structural_facts();
        assert_eq!(zero.sign, Some(RealSign::Zero));
        assert_eq!(zero.zero, ZeroKnowledge::Zero);
        assert!(zero.exact_rational);
        assert_eq!(zero.magnitude, None);

        let negative = Computable::rational(Rational::fraction(-7, 8).unwrap()).structural_facts();
        assert_eq!(negative.sign, Some(RealSign::Negative));
        assert_eq!(negative.zero, ZeroKnowledge::NonZero);
        assert!(negative.exact_rational);
        assert_eq!(
            negative.magnitude,
            Some(MagnitudeBits {
                msd: -1,
                exact_msd: true,
            })
        );
    }

    #[test]
    fn structural_facts_for_shared_constant() {
        let facts = Computable::pi().structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Positive));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert!(!facts.exact_rational);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: 1,
                exact_msd: true,
            })
        );
    }

    #[test]
    fn structural_facts_for_constant_rational_offset_certificates() {
        let pi_minus_three = Computable::pi().add(Computable::rational(Rational::new(-3)));
        let facts = pi_minus_three.structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Positive));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: -3,
                exact_msd: true,
            })
        );
        assert_eq!(pi_minus_three.sign_until(0), Some(RealSign::Positive));

        let three_minus_pi = Computable::rational(Rational::new(3)).add(Computable::pi().negate());
        let facts = three_minus_pi.structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Negative));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: -3,
                exact_msd: true,
            })
        );

        let two_pi_minus_six = Computable::pi()
            .shift_left(1)
            .add(Computable::rational(Rational::new(-6)));
        let facts = two_pi_minus_six.structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Positive));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: -2,
                exact_msd: true,
            })
        );
    }

    #[test]
    fn zero_status_uses_structural_facts_without_refinement() {
        assert_eq!(
            Computable::rational(Rational::zero()).zero_status(),
            ZeroKnowledge::Zero
        );
        assert_eq!(
            Computable::rational(Rational::fraction(-7, 8).unwrap()).zero_status(),
            ZeroKnowledge::NonZero
        );
        assert_eq!(Computable::pi().zero_status(), ZeroKnowledge::NonZero);

        let near_pi =
            Computable::pi().add(Computable::rational(Rational::fraction(-22, 7).unwrap()));
        assert_eq!(near_pi.zero_status(), ZeroKnowledge::NonZero);
    }

    #[test]
    fn sign_until_respects_precision_floor() {
        let near_pi = Computable::pi().add(Computable::rational(Rational::new(-3)));

        assert_eq!(near_pi.sign_until(0), Some(RealSign::Positive));
        assert_eq!(near_pi.sign_until(-8), Some(RealSign::Positive));
    }

    #[test]
    fn sign_until_uses_structural_bounds_without_refinement() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(-7, 8).unwrap()))
            .inverse()
            .negate();

        assert_eq!(value.sign_until(0), Some(RealSign::Positive));
    }

    #[test]
    fn add_with_dominant_term_has_structural_bound() {
        let value = Computable::integer(BigInt::from(8))
            .add(Computable::rational(Rational::fraction(-1, 8).unwrap()));
        assert_eq!(value.sign_until(0), Some(RealSign::Positive));
        assert_eq!(value.msd(-4), Some(2));
    }

    #[test]
    fn opposite_sign_quadratic_surd_is_certified_nonzero() {
        let half = Computable::rational(Rational::fraction(1, 2).unwrap());
        let negative_radical = Computable::rational(Rational::new(2))
            .sqrt()
            .multiply_rational(Rational::fraction(-3, 8).unwrap());
        let sum = half.add(negative_radical);

        assert_eq!(sum.zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(sum.sign_until(-64), Some(RealSign::Negative));
    }

    #[test]
    fn add_ignores_tiny_term_at_target_precision() {
        let big = Computable::pi();
        let tiny = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(1), BigUint::from(1_u8) << 200).unwrap(),
        );
        assert_eq!(
            big.clone().add(tiny).compare_absolute(&big, -128),
            Ordering::Equal
        );
    }

    #[test]
    fn add_does_not_ignore_tiny_opposite_sign_term() {
        let big = Computable::pi();
        let tiny = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(-1), BigUint::from(1_u8) << 200).unwrap(),
        );
        let sum = big.clone().add(tiny.clone());
        let delta = sum.add(big.negate());
        assert_eq!(delta.compare_absolute(&tiny, -180), Ordering::Equal);
    }

    #[test]
    fn deep_structural_bound_chain() {
        let scale = Computable::rational(Rational::fraction(-7, 8).unwrap());
        let mut value = Computable::pi();
        value.approx(-16);
        for _ in 0..2000 {
            value = value.multiply(scale.clone()).inverse().negate();
        }
        assert_eq!(value.sign_until(0), Some(RealSign::Positive));
        assert_close(value, Computable::pi(), -60, 2);
    }

    #[test]
    fn structural_bound_walk_reuses_shared_dag_subtrees() {
        // `exp(pi) - 100` has no cheap structural bound. Repeatedly sharing it
        // under addition makes the logical expression tree exponential while
        // the immutable DAG remains tiny. The nonrecursive bound walk must
        // cache each completed node so the second edge reuses the first proof.
        let mut value = Computable::pi()
            .exp()
            .add(Computable::integer(BigInt::from(-100)));
        for _ in 0..128 {
            value = value.clone().add(value);
        }

        assert_eq!(value.cheap_bound(), BoundInfo::Unknown);
    }

    #[test]
    fn structural_equality_visits_shared_dag_pairs_once() {
        let mut left = Computable::pi()
            .exp()
            .add(Computable::integer(BigInt::from(-100)));
        let mut right = Computable::pi()
            .exp()
            .add(Computable::integer(BigInt::from(-100)));
        let mut different = Computable::pi()
            .exp()
            .add(Computable::integer(BigInt::from(-101)));
        for _ in 0..128 {
            left = left.clone().add(left);
            right = right.clone().add(right);
            different = different.clone().add(different);
        }

        assert!(Computable::internal_structural_eq(&left, &right));
        assert!(!Computable::internal_structural_eq(&left, &different));
    }

    #[test]
    fn huge_trig_arguments_reduce_correctly() {
        let huge_multiple = BigInt::from(1_u8) << 200;
        let offset = Computable::rational(Rational::fraction(7, 5).unwrap());
        let huge = Computable::pi()
            .multiply(Computable::integer(huge_multiple))
            .add(offset.clone());

        assert_eq!(
            huge.clone()
                .sin()
                .compare_absolute(&offset.clone().sin(), -80),
            Ordering::Equal
        );
        assert_eq!(
            huge.clone()
                .cos()
                .compare_absolute(&offset.clone().cos(), -80),
            Ordering::Equal
        );
        assert_eq!(
            huge.tan().compare_absolute(&offset.tan(), -72),
            Ordering::Equal
        );
    }

    #[test]
    fn exact_large_rational_trig_uses_correct_quadrant() {
        let million = Computable::rational(Rational::new(1_000_000));

        assert_approx(million.clone().sin(), -32, "-1503210646", 8);
        assert_approx(million.clone().cos(), -32, "4023319752", 8);
        assert_approx(million.tan(), -32, "-1604704811", 8);
    }

    #[test]
    fn exact_huge_rational_trig_uses_correct_quadrant() {
        let huge = Rational::new(10).powi(BigInt::from(30)).unwrap();
        let direct = Computable::rational(huge.clone());

        assert_approx(direct.clone().sin(), -72, "-425565037129932206620", 8);
        assert_approx(direct.clone().cos(), -72, "-4703152091704373381319", 8);
        assert_approx(direct.tan(), -72, "427303652622316740317", 16);
    }

    #[test]
    fn many_digits_exact_integer_trig_uses_one_high_precision_pi_pass() {
        // Many Digits C08: the exact argument has 120,605 bits. Besides being
        // a useful quadrant oracle, this crosses the cache-aware reduction
        // threshold so the quotient estimate warms pi deeply enough for the
        // final residual instead of immediately recomputing pi.
        let argument = BigInt::from(6_u8).pow(6_u32.pow(6));
        let direct = Computable::integer(argument);

        assert_approx(
            direct.clone().sin(),
            -128,
            "324613637756746780497943680818791265673",
            8,
        );
        assert_approx(
            direct.clone().cos(),
            -128,
            "-102068973834597634010659370575740411979",
            8,
        );
        assert_approx(
            direct.tan(),
            -128,
            "-1082212280978570270357125515810836172498",
            16,
        );
    }

    #[test]
    fn tan_small_and_medium_arguments() {
        let one_fifth = Computable::rational(Rational::fraction(1, 5).unwrap());
        assert_approx(one_fifth.tan(), -32, "870632973", 2);

        let seven_fifths = Computable::rational(Rational::fraction(7, 5).unwrap());
        assert_approx(seven_fifths.tan(), -32, "24901720944", 2);
    }

    #[test]
    fn tan_near_half_pi() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let near_half_pi = pi_times(Rational::fraction(1, 2).unwrap()).add(epsilon.negate());
        assert_approx(near_half_pi.tan(), -32, "274855536959", 8);
    }

    #[test]
    fn ln_sqrt_pi() {
        let pi = Computable::pi();
        let sqrt = Computable::sqrt(pi);
        let ln = Computable::ln(sqrt);
        let correct: BigInt = "629321910077".parse().unwrap();
        assert_eq!(ln.approx(-40), correct);
    }

    #[test]
    fn ln_large_power_of_two() {
        let value = Computable::rational(Rational::new(1024));
        let ten = Computable::rational(Rational::new(10));
        assert_close(value.ln(), ten.multiply(Computable::ln2()), -40, 2);
    }

    #[test]
    fn ln_tiny_power_of_two() {
        let denominator = BigUint::from(1_u8) << 10;
        let value = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(1), denominator).unwrap(),
        );
        let ten = Computable::rational(Rational::new(10));
        assert_close(value.ln(), ten.multiply(Computable::ln2()).negate(), -40, 2);
    }

    #[test]
    fn ln_exact_binary_scaled_rational() {
        let denominator = BigUint::from(1_u8) << 10;
        let value = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(3), denominator).unwrap(),
        );
        let expected = Computable::rational(Rational::new(3))
            .ln()
            .add(Computable::rational(Rational::new(-10)).multiply(Computable::ln2()));
        assert_close(value.ln(), expected, -40, 2);
    }

    #[test]
    fn ln_smooth_rational_reuses_shared_prime_logs() {
        let value = Computable::rational(Rational::fraction(45, 14).unwrap());
        let expected = Computable::ln_constant(3)
            .unwrap()
            .multiply(Computable::rational(Rational::new(2)))
            .add(Computable::ln_constant(5).unwrap())
            .add(Computable::ln_constant(2).unwrap().negate())
            .add(Computable::ln_constant(7).unwrap().negate());
        assert_close(value.ln(), expected, -50, 3);
    }

    #[test]
    fn sqrt_square_round_trip() {
        let two = Computable::rational(Rational::new(2));
        let sqrt_two = two.clone().sqrt();
        assert_close(sqrt_two.square(), two, -40, 2);
    }

    #[test]
    fn ln_near_prescaled_limit_round_trip() {
        let value = Computable::rational(Rational::fraction(47, 32).unwrap());
        assert_close(value.clone().ln().exp(), value, -40, 2);
    }

    #[test]
    fn erf_known_values() {
        assert_close(
            Computable::zero().erf(),
            Computable::zero(),
            -160,
            2,
        );
        assert_close(
            Computable::rational(Rational::fraction(1, 2).unwrap()).erf(),
            Computable::rational("0.5204998778130465376827466538919645287364".parse().unwrap()),
            -90,
            2,
        );
        assert_close(
            Computable::one().erf(),
            Computable::rational("0.8427007929497148693412206350826092592960".parse().unwrap()),
            -90,
            2,
        );
        assert_close(
            Computable::one().erfc(),
            Computable::rational("0.1572992070502851306587793649173907407040".parse().unwrap()),
            -90,
            2,
        );
        assert_close(
            Computable::one().erfcx(),
            Computable::rational("0.4275835761558070044107503444905151808202".parse().unwrap()),
            -90,
            2,
        );
    }

    #[test]
    fn normal_density_and_cdf_known_values() {
        assert_close(
            Computable::zero().dnorm(),
            Computable::rational("0.39894228040143267793994605993438186847585863".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::one().pnorm(),
            Computable::rational("0.8413447460685429485852325456320379224779".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::one().normal_sf(),
            Computable::rational("0.1586552539314570514147674543679620775221".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::normal_interval(Computable::zero(), Computable::one()),
            Computable::rational("0.3413447460685429485852325456320379224779".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::zero().log_pnorm(),
            Computable::rational("-0.6931471805599453094172321214581765680755".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::zero().log_normal_sf(),
            Computable::rational("-0.6931471805599453094172321214581765680755".parse().unwrap()),
            -120,
            2,
        );
        assert_close(
            Computable::zero().log_dnorm(),
            Computable::rational("-0.9189385332046727417803297364056176398614".parse().unwrap()),
            -120,
            2,
        );
    }

    #[test]
    fn normal_tail_nodes_have_structural_signs() {
        let x = Computable::one();
        assert_eq!(x.clone().erfc().exact_sign(), Some(Sign::Plus));
        assert_eq!(x.clone().erfcx().exact_sign(), Some(Sign::Plus));
        assert_eq!(x.clone().normal_sf().exact_sign(), Some(Sign::Plus));
        assert_eq!(
            Computable::normal_interval(Computable::zero(), Computable::one()).exact_sign(),
            Some(Sign::Plus)
        );
        assert_eq!(
            Computable::normal_interval(Computable::one(), Computable::one()).exact_sign(),
            Some(Sign::NoSign)
        );
        assert_eq!(x.clone().log_pnorm().exact_sign(), Some(Sign::Minus));
        assert_eq!(x.clone().log_normal_sf().exact_sign(), Some(Sign::Minus));
        assert_eq!(x.log_dnorm().exact_sign(), Some(Sign::Minus));
    }

    #[test]
    fn exact_endpoint_replays_do_not_gain_nonzero_certificates() {
        let root_two = Computable::sqrt_rational(Rational::new(2));
        let replayed_one = root_two
            .clone()
            .multiply(root_two)
            .multiply_rational(Rational::fraction(1, 2).unwrap());
        assert_eq!(replayed_one.exact_rational(), None);

        let raw_acos = Computable::acos_positive(replayed_one.clone());
        assert_eq!(raw_acos.approx(-96), BigInt::zero());
        assert_eq!(raw_acos.exact_sign(), None);
        assert_eq!(raw_acos.inverse_trig_linear_sign(), Some(Sign::NoSign));
        assert_eq!(
            raw_acos.try_compare_to_until(&Computable::zero(), -64),
            None
        );

        let normalized_acos = replayed_one.clone().acos();
        assert_eq!(normalized_acos.exact_sign(), Some(Sign::NoSign));
        assert_eq!(
            normalized_acos.try_compare_to(&Computable::zero()),
            Some(Ordering::Equal)
        );

        let interval = Computable::normal_interval(replayed_one, Computable::one());
        assert_eq!(interval.approx(-96), BigInt::zero());
        assert_eq!(interval.exact_sign(), Some(Sign::NoSign));

        let reversed =
            Computable::normal_interval(Computable::one(), Computable::zero());
        assert_eq!(reversed.exact_sign(), Some(Sign::Minus));
    }

    #[test]
    fn expm1_preserves_small_argument_and_sign() {
        let tiny = Computable::rational(Rational::fraction(1, 1_000_000).unwrap());
        assert_eq!(tiny.clone().expm1().exact_sign(), Some(Sign::Plus));
        assert_eq!(tiny.clone().negate().expm1().exact_sign(), Some(Sign::Minus));
        assert_close(
            tiny.expm1(),
            Computable::rational("0.0000010000005000001666667083333416667".parse().unwrap()),
            -120,
            2,
        );
    }

    #[test]
    fn normal_quantile_inverts_cdf() {
        let two = Computable::rational(Rational::new(2));
        let p = two.clone().pnorm();
        let seed = BigInt::from((1.9999_f64 * f64::from(1_u32 << 13)).round() as i64);
        let q = Computable::normal_quantile(p, seed, -13);
        assert_close(q, two, -120, 2);
    }

    #[test]
    fn add() {
        let three: BigInt = "3".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(three);
        let b = Computable::integer(five);
        let c = Computable::add(a, b);
        let answer: BigInt = "256".parse().unwrap();
        assert_eq!(answer, c.approx(-5));
    }

    #[test]
    fn add_cancels_structurally_shared_term_across_nested_sum() {
        let tiny = Computable::rational(
            Rational::from_bigint_fraction(
                BigInt::one(),
                BigUint::one() << 5000_usize,
            )
            .unwrap(),
        );
        let pi = Computable::pi();

        for reduced in [
            pi.clone().add(tiny.clone()).add(pi.clone().negate()),
            pi.clone().negate().add(tiny.clone().add(pi.clone())),
            tiny.clone().add(pi.clone()).add(pi.negate()),
        ] {
            assert!(Computable::internal_structural_eq(&reduced, &tiny));
            assert_eq!(reduced.exact_sign(), Some(Sign::Plus));
        }
    }

    #[test]
    fn add_cancels_a_commuted_exact_pair_without_general_reordering() {
        let root_two = Computable::sqrt_rational(Rational::new(2));
        let root_three = Computable::sqrt_rational(Rational::new(3));
        let forward = root_two.clone().add(root_three.clone());
        let reverse = root_three.clone().add(root_two.clone());

        for zero in [
            forward.clone().add(reverse.clone().negate()),
            reverse.negate().add(forward),
        ] {
            assert_eq!(zero.exact_rational(), Some(Rational::zero()));
            assert_eq!(zero.exact_sign(), Some(Sign::NoSign));
        }

        let nonzero = root_two
            .clone()
            .add(Computable::one())
            .add(
                Computable::one()
                    .add(root_two.multiply_rational(Rational::new(2)))
                    .negate(),
            );
        assert_eq!(nonzero.exact_sign(), Some(Sign::Minus));
    }

    #[test]
    fn add_folds_rational_terms_across_nested_sum() {
        let root_five = Computable::sqrt_rational(Rational::new(5)).negate();
        for reduced in [
            Computable::integer(BigInt::from(-3))
                .add(root_five.clone())
                .add(Computable::integer(BigInt::from(3))),
            Computable::integer(BigInt::from(3)).add(
                root_five
                    .clone()
                    .add(Computable::integer(BigInt::from(-3))),
            ),
        ] {
            assert!(Computable::internal_structural_eq(&reduced, &root_five));
        }
    }

    #[test]
    fn exact_sign_reduces_quadratic_surd_field_identities() {
        let root_two = Computable::sqrt_rational(Rational::new(2));
        let collapse_distance = Computable::rational(Rational::new(4))
            .add(root_two.clone().multiply_rational(Rational::new(-2)));
        let expanded_zero = collapse_distance
            .multiply(root_two.clone())
            .add(Computable::rational(Rational::new(4)))
            .add(root_two.clone().multiply_rational(Rational::new(-4)));
        assert_eq!(expanded_zero.exact_sign(), Some(Sign::NoSign));

        let conjugate_zero = Computable::one()
            .add(root_two.clone())
            .inverse()
            .add(Computable::one())
            .add(root_two.negate());
        assert_eq!(conjugate_zero.exact_sign(), Some(Sign::NoSign));
    }

    #[test]
    fn exact_sign_orders_nonzero_quadratic_surds() {
        let root_two = Computable::sqrt_rational(Rational::new(2));
        let positive = root_two
            .clone()
            .add(Computable::rational(Rational::new(-1)));
        let negative = root_two.add(Computable::rational(Rational::new(-2)));

        assert_eq!(positive.exact_sign(), Some(Sign::Plus));
        assert_eq!(negative.exact_sign(), Some(Sign::Minus));
    }

    #[test]
fn inverse_atan_linear_sign_includes_the_argument_sign() {
        let negative_argument = Computable::sqrt_rational(Rational::new(2))
            .add(Computable::rational(Rational::new(-2)));
        assert_eq!(negative_argument.exact_sign(), Some(Sign::Minus));

        let positive_atan_term = negative_argument
            .atan()
            .multiply_rational(Rational::new(-2));
        let opposed = positive_atan_term
            .clone()
            .add(Computable::pi().multiply_rational(Rational::fraction(-1, 8).unwrap()));

        assert!(opposed.approx(-64) > BigInt::zero());
        assert_eq!(opposed.inverse_trig_linear_sign(), None);
        assert_eq!(opposed.exact_sign(), Some(Sign::Plus));

        let aligned = positive_atan_term
            .add(Computable::pi().multiply_rational(Rational::fraction(1, 8).unwrap()));
        assert_eq!(aligned.inverse_trig_linear_sign(), Some(Sign::Plus));
        assert_eq!(aligned.exact_sign(), Some(Sign::Plus));
    }

    #[test]
    fn shared_pi_reciprocals_cancel_across_a_product() {
        let argument = Computable::sqrt_rational(Rational::new(15)).atan();

        let right_nested = Computable::pi().multiply(
            argument
                .clone()
                .multiply(Computable::pi_inverse_constant()),
        );
        let left_nested = Computable::pi_inverse_constant()
            .multiply(argument.clone())
            .multiply(Computable::pi());

        assert!(Arc::ptr_eq(&right_nested.internal, &argument.internal));
        assert!(Arc::ptr_eq(&left_nested.internal, &argument.internal));
    }

    #[test]
    fn normalized_half_pi_complement_retains_its_atan_argument() {
        let argument = Computable::sqrt_rational(Rational::new(15))
            .inverse()
            .atan();
        let normalized = Computable::one().add(
            argument
                .clone()
                .multiply(Computable::pi_inverse_constant())
                .shift_left(1)
                .negate(),
        );
        let angle = Computable::pi().multiply(normalized).shift_right(1);

        let (orientation, retained) = angle
            .signed_half_pi_minus_atan_argument()
            .expect("pi/2 - atan argument must remain structurally available");
        assert_eq!(orientation, Sign::Plus);
        let expected = argument
            .atan_argument()
            .expect("the test expression is an atan node");
        assert!(Arc::ptr_eq(&retained.internal, &expected.internal));
    }

    #[test]
    fn quadratic_surd_atan_anchors_survive_computable_composition() {
        let root_three = Computable::sqrt_rational(Rational::new(3));
        let inverse_root_three = root_three.clone().inverse();
        let third_pi = Computable::pi().multiply_rational(
            Rational::fraction(1, 3).expect("three is nonzero"),
        );
        let sixth_pi = Computable::pi().multiply_rational(
            Rational::fraction(1, 6).expect("six is nonzero"),
        );

        assert!(Computable::internal_structural_eq(
            &root_three.atan(),
            &third_pi,
        ));
        assert!(Computable::internal_structural_eq(
            &inverse_root_three.atan(),
            &sixth_pi,
        ));
    }

    #[test]
    fn integer_pi_atan_replay_survives_nested_half_pi_cancellation() {
        let argument = Computable::rational(
            Rational::fraction(3, 4).expect("four is nonzero"),
        );
        let atan = argument.clone().atan();
        let angle = atan.negate().add(Computable::pi());
        let (_, _, retained) = angle
            .integer_pi_plus_or_minus_atan_argument()
            .expect("pi minus atan must retain its inverse-trig argument");
        assert!(Computable::internal_structural_eq(&retained, &argument));

        let half_pi = Computable::pi().shift_right(1);
        let reduced = angle
            .add(half_pi.clone().negate())
            .add(half_pi.negate());
        let (_, _, retained) = reduced
            .integer_pi_plus_or_minus_atan_argument()
            .expect("nested half-pi cancellation must retain its inverse-trig argument");
        assert!(Computable::internal_structural_eq(&retained, &argument));
        assert!(Computable::internal_structural_eq(
            &reduced.clone().sin(),
            &Computable::rational(Rational::fraction(-3, 5).expect("five is nonzero")),
        ));
        assert!(Computable::internal_structural_eq(
            &reduced.cos(),
            &Computable::rational(Rational::fraction(4, 5).expect("five is nonzero")),
        ));
    }

    #[test]
    fn pi_laurent_arithmetic_canonicalizes_nested_parameter_replay() {
        let pi = Computable::pi();
        let pi_squared = pi.clone().multiply(pi.clone());
        let one_third = Rational::fraction(1, 3).expect("three is nonzero");
        let third_pi = pi.clone().multiply_rational(one_third.clone());
        let domain_parameter = pi.clone().multiply(
            third_pi
                .clone()
                .multiply(pi.clone())
                .multiply(pi_squared.clone().inverse()),
        );
        let pi_minus_one = pi.clone().add(Computable::rational(Rational::new(-1)));
        let contact_parameter = pi
            .clone()
            .multiply(
                pi_minus_one
                    .clone()
                    .add(third_pi.negate())
                    .multiply(pi)
                    .multiply(pi_squared.inverse()),
            )
            .negate()
            .add(pi_minus_one);

        assert_eq!(
            domain_parameter.pi_rational_multiple(),
            Some(one_third.clone())
        );
        assert_eq!(
            contact_parameter.pi_rational_multiple(),
            Some(one_third)
        );
        assert_eq!(
            contact_parameter
                .add(domain_parameter.negate())
                .bounded_laurent_rational(48),
            Some(Rational::zero())
        );

        let root_two = Computable::sqrt_rational(Rational::new(2));
        assert_eq!(
            root_two
                .clone()
                .shift_right(2)
                .multiply(root_two.clone().inverse().shift_left(2))
                .bounded_laurent_rational(48),
            Some(Rational::one())
        );
        assert!(Computable::internal_structural_eq(
            &root_two
                .clone()
                .shift_right(2)
                .multiply(root_two.clone().inverse().shift_left(2))
                .negate()
                .atan(),
            &Computable::pi().shift_right(2).negate()
        ));
        let root_two_over_pi =
            Computable::pi_inverse_constant().multiply(root_two.clone());
        let shared_offset = root_two
            .clone()
            .multiply_rational(Rational::new(3))
            .add(Computable::rational(
                Rational::fraction(1, 2).expect("two is nonzero"),
            ));
        let contact_parameter = root_two
            .clone()
            .shift_left(2)
            .add(shared_offset.clone().negate())
            .multiply(Computable::pi())
            .multiply(root_two_over_pi.clone())
            .multiply_rational(Rational::fraction(1, 4).expect("four is nonzero"));
        let domain_parameter = root_two
            .shift_left(1)
            .add(shared_offset.negate())
            .multiply(Computable::pi())
            .multiply(root_two_over_pi)
            .shift_right(2)
            .add(Computable::one());

        assert_eq!(
            contact_parameter
                .add(domain_parameter.negate())
                .bounded_laurent_rational(48),
            Some(Rational::zero())
        );

        let pi = Computable::pi();
        let pi_squared = pi.clone().multiply(pi.clone());
        let angle = Computable::sqrt_rational(Rational::new(2))
            .add(Computable::one())
            .atan();
        let shifted_angle = angle.add(pi.clone());
        let contact_parameter = pi
            .clone()
            .add(shifted_angle.clone().negate())
            .multiply(pi.clone())
            .multiply(pi_squared.clone().inverse())
            .multiply(pi.clone())
            .negate()
            .add(pi.clone());
        let domain_parameter = pi
            .clone()
            .shift_right(1)
            .add(shifted_angle.negate())
            .multiply(pi.clone())
            .multiply(pi_squared.inverse())
            .multiply(pi.clone())
            .negate()
            .add(pi.shift_right(1));

        assert_eq!(
            contact_parameter
                .add(domain_parameter.negate())
                .bounded_laurent_rational(48),
            Some(Rational::zero())
        );
    }

    #[test]
    fn rational_normal_form_retains_only_successful_sign_proofs() {
        let opaque = |approximation| Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Unknown,
            )),
            signal: None,
        };
        let pi = Computable::pi();
        let root = Computable::sqrt_rational(Rational::new(5));
        let atom = Computable::rational(Rational::fraction(3, 4).unwrap()).atan();
        let tiny = Rational::from_bigint_fraction(
            BigInt::one(),
            BigUint::one() << 2_000,
        )
        .unwrap();
        for budget in [48, 64] {
            for zero in [
                Approximation::Add(pi.clone(), pi.clone().negate()),
                Approximation::Add(
                    opaque(Approximation::Square(root.clone())),
                    Computable::rational(Rational::new(-5)),
                ),
                Approximation::Add(atom.clone(), atom.clone().negate()),
            ] {
                let zero = opaque(zero);
                for expected in [
                    Rational::zero(),
                    Rational::fraction(7, 9).unwrap(),
                    Rational::fraction(-7, 9).unwrap(),
                    tiny.clone(),
                    -tiny.clone(),
                ] {
                    let value = opaque(Approximation::Add(
                        zero.clone(),
                        Computable::rational(expected.clone()),
                    ));
                    let shared = value.clone();
                    assert_eq!(value.immediate_sign(), None);
                    // Exhausting a proof budget must not manufacture a fact.
                    assert_eq!(value.bounded_laurent_rational(0), None);
                    assert_eq!(value.immediate_sign(), None);
                    assert_eq!(
                        value.bounded_laurent_rational(budget),
                        Some(expected.clone()),
                    );
                    let sign = public_sign(expected.sign());
                    assert_eq!(value.immediate_sign(), Some(sign));
                    assert_eq!(shared.immediate_sign(), Some(sign));
                    assert_eq!(value.sign_until(0), Some(sign));
                    assert!(value.cached().is_none(), "a symbolic proof must not approximate");
                }
            }
        }
        let different_atom =
            Computable::rational(Rational::fraction(4, 5).unwrap()).atan();
        let unresolved = opaque(Approximation::Add(atom, different_atom.negate()));
        assert_eq!(unresolved.bounded_laurent_rational(64), None);
        assert_eq!(unresolved.immediate_sign(), None);
        assert!(unresolved.cached().is_none());
        assert_eq!(unresolved.sign_until(-128), Some(RealSign::Negative));
    }

    #[test]
    fn pi_laurent_normal_form_declines_wide_opaque_expression_dags() {
        let denominator = Rational::new(4_099);
        let mut layer = (2..1_026)
            .map(|numerator| {
                Computable::rational(
                    Rational::new(numerator) / denominator.clone(),
                )
                .atan()
            })
            .collect::<Vec<_>>();
        while layer.len() > 1 {
            let mut next = Vec::with_capacity(layer.len().div_ceil(2));
            let mut values = layer.into_iter();
            while let Some(left) = values.next() {
                next.push(match values.next() {
                    Some(right) => left.add(right),
                    None => left,
                });
            }
            layer = next;
        }
        let expression = layer.pop().expect("the balanced expression is nonempty");

        assert!(expression.bounded_laurent_rational(48).is_none());
    }

    #[test]
    fn scale_up() {
        let ten: BigInt = "10".parse().unwrap();
        let three: BigInt = "3".parse().unwrap();
        assert_eq!(ten, scale(ten.clone(), 0));
        let a = scale(ten.clone(), -2);
        assert_eq!(three, a);
        let forty: BigInt = "40".parse().unwrap();
        let b = scale(ten.clone(), 2);
        assert_eq!(forty, b);
    }

    #[test]
    fn approximation_bounds_preserve_the_unit_error_boundary() {
        for value in [-1, 0, 1] {
            assert_eq!(
                Computable::bound_from_approx(-37, &BigInt::from(value)),
                BoundInfo::Unknown
            );
        }

        assert_eq!(
            Computable::bound_from_approx(-37, &BigInt::from(-2)),
            BoundInfo::NonZero {
                sign: Some(Sign::Minus),
                msd: Some(-36),
                exact_msd: false,
            }
        );
        assert_eq!(
            Computable::bound_from_approx(-37, &BigInt::from(2)),
            BoundInfo::NonZero {
                sign: Some(Sign::Plus),
                msd: Some(-36),
                exact_msd: false,
            }
        );
    }

    #[test]
    fn approximation_cache_does_not_weaken_exact_structural_bound() {
        let value = Computable::pi();
        let exact_bound = value.internal.facts.bound();

        value.store_cache_value(&None, -4, BigInt::from(50));

        assert_eq!(value.internal.facts.bound(), exact_bound);
        assert!(matches!(
            exact_bound,
            BoundCache::Valid(BoundInfo::NonZero {
                exact_msd: true,
                ..
            })
        ));
    }

    #[test]
    fn msd_refines_ambiguous_unit_approximations_at_binary_boundaries() {
        for value in [
            Computable::rational(Rational::fraction(1, 4).unwrap()).atan(),
            Computable::rational(Rational::fraction(1, 4).unwrap()).asinh(),
        ] {
            assert_eq!(value.msd(-128), Some(-3));
        }
    }
}

#[test]
fn inverse_trig_presence_survives_bound_cache_updates() {
    let angle = Computable::rational(Rational::fraction(3, 4).unwrap()).atan();
    let expression = angle.add(Computable::one());

    assert!(expression.internal.contains_inverse_trig_or_pi());
    let _ = expression.cheap_bound();
    assert!(expression.internal.contains_inverse_trig_or_pi());
}
