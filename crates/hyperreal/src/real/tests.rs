#[cfg(test)]
#[allow(clippy::module_inception)]
mod tests {
    use core::cmp::Ordering;

    use crate::real::arithmetic::curve;
    use crate::{
        CertifiedRealEquality, CertifiedRealOrdering, CertifiedRealSign, DomainStatus,
        ExactDyadicLine2, ExpressionDegree, MagnitudeBits, PrimitiveFloatStatus, Problem, Rational,
        RationalLinearForm4Filter, RationalLinearForm4Query, RationalStorageClass, Real,
        RealEqualityCertificate, RealExactSetDenominatorKind, RealExactSetDyadicExponentClass,
        RealExactSetFacts, RealExactSetSignPattern, RealOrderingCertificate, RealSign,
        RealSignCertificate, RealStructuralFacts, StructuralComparison, StructuralKind,
        SymbolicDependencyMask, ZeroKnowledge, ZeroOneMinusOneStatus,
    };
    use num::Signed;

    fn rational_linear_form4_filter_matches_exact_sum(
        coefficients: [Rational; 4],
        point: [Rational; 4],
    ) -> bool {
        let coefficient_reals = coefficients.clone().map(Real::new);
        let filter = RationalLinearForm4Filter::from_reals([
            &coefficient_reals[0],
            &coefficient_reals[1],
            &coefficient_reals[2],
            &coefficient_reals[3],
        ])
        .expect("the test corpus stays within the normalized filter range");
        let query =
            RationalLinearForm4Query::from_rationals([&point[0], &point[1], &point[2], &point[3]])
                .expect("the test corpus stays within the normalized query range");
        let Some(actual) = filter.sign(&query) else {
            return false;
        };
        let expected = match Rational::signed_product_sum_ordering(
            [true; 4],
            [
                [&coefficients[0], &point[0]],
                [&coefficients[1], &point[1]],
                [&coefficients[2], &point[2]],
                [&coefficients[3], &point[3]],
            ],
        ) {
            Ordering::Less => RealSign::Negative,
            Ordering::Equal => RealSign::Zero,
            Ordering::Greater => RealSign::Positive,
        };
        assert_eq!(actual, expected);
        true
    }

    #[test]
    fn zero() {
        assert_eq!(Real::zero(), Real::zero());
    }

    #[test]
    fn one_constructor_matches_integer_conversion() {
        let one = Real::one();
        assert_eq!(one, Real::new(Rational::one()));
        assert_eq!(one, Real::from(1_i32));
        assert_eq!(one.zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(one.structural_facts().sign, Some(RealSign::Positive));
    }

    #[test]
    fn rational_linear_form4_filter_never_disagrees_with_exact_sum() {
        fn next_rational(state: &mut u64) -> Rational {
            *state ^= *state << 13;
            *state ^= *state >> 7;
            *state ^= *state << 17;
            let numerator = i64::try_from(*state % 2001).unwrap() - 1000;
            *state = state
                .wrapping_mul(0x9e37_79b9_7f4a_7c15)
                .wrapping_add(0xbf58_476d_1ce4_e5b9);
            let denominator = *state % 97 + 1;
            Rational::fraction(numerator, denominator).unwrap()
        }

        let mut state = 0x1234_5678_9abc_def0;
        let mut certified = 0;
        for _ in 0..4096 {
            let coefficients: [Rational; 4] = std::array::from_fn(|_| next_rational(&mut state));
            let point: [Rational; 4] = std::array::from_fn(|_| next_rational(&mut state));
            certified += usize::from(rational_linear_form4_filter_matches_exact_sum(
                coefficients,
                point,
            ));
        }
        assert!(certified > 4000);
    }

    #[test]
    fn rational_linear_form4_filter_never_disagrees_at_low_normal_scales() {
        fn next_rational(state: &mut u64) -> Rational {
            *state ^= *state << 13;
            *state ^= *state >> 7;
            *state ^= *state << 17;
            let magnitude = i64::try_from(64 + *state % 64).unwrap();
            let numerator = if *state & 128 == 0 {
                magnitude
            } else {
                -magnitude
            };
            Rational::fraction(numerator, 64).unwrap()
        }

        let mut state = 0xd1b5_4a32_d192_ed03;
        let mut certified = 0;
        for (coefficient_exponent, point_exponent) in
            [(1, 1), (1, 47), (47, 1), (17, 31), (48, 2), (256, 17)]
        {
            let coefficient_scale =
                Rational::try_from(f64::from_bits(coefficient_exponent << 52)).unwrap();
            let point_scale = Rational::try_from(f64::from_bits(point_exponent << 52)).unwrap();
            for _ in 0..256 {
                let coefficients: [Rational; 4] =
                    std::array::from_fn(|_| next_rational(&mut state) * &coefficient_scale);
                let point: [Rational; 4] =
                    std::array::from_fn(|_| next_rational(&mut state) * &point_scale);
                certified += usize::from(rational_linear_form4_filter_matches_exact_sum(
                    coefficients,
                    point,
                ));
            }
        }
        assert!(certified > 1_500);
    }

    #[test]
    fn rational_linear_form4_filter_never_disagrees_across_wide_exponent_spans() {
        fn next_small_rational(state: &mut u64) -> Rational {
            *state ^= *state << 13;
            *state ^= *state >> 7;
            *state ^= *state << 17;
            Rational::from(if *state & 8 == 0 { 1 } else { -1 })
        }

        let mut state = 0xa409_3822_299f_31d0;
        let mut certified = 0;
        for span in [501_u64, 512, 900, 1022] {
            let scale = Rational::try_from(f64::from_bits((1023 - span) << 52)).unwrap();
            for _ in 0..512 {
                let coefficient_base: [Rational; 4] =
                    std::array::from_fn(|_| next_small_rational(&mut state));
                let point_base: [Rational; 4] =
                    std::array::from_fn(|_| next_small_rational(&mut state));
                let coefficients = [
                    coefficient_base[0].clone(),
                    &coefficient_base[1] * &scale,
                    coefficient_base[2].clone(),
                    &coefficient_base[3] * &scale,
                ];
                let point = [
                    &point_base[0] * &scale,
                    point_base[1].clone(),
                    point_base[2].clone(),
                    &point_base[3] * &scale,
                ];
                certified += usize::from(rational_linear_form4_filter_matches_exact_sum(
                    coefficients,
                    point,
                ));
            }
        }
        assert!(certified > 2_000);
    }

    #[test]
    fn rational_linear_form4_filter_normalizes_representable_spans_and_bounds_underflow() {
        fn rational(value: f64) -> Rational {
            Rational::try_from(value).unwrap()
        }

        let one = rational(1.0);
        let zero = Rational::zero();
        let minimum_safe = rational(f64::from_bits((1023_u64 - 500) << 52));
        let wide_lane = rational(f64::from_bits((1023_u64 - 501) << 52));

        let safe_coefficients = [
            Real::new(one.clone()),
            Real::new(minimum_safe.clone()),
            Real::zero(),
            Real::zero(),
        ];
        let safe_filter = RationalLinearForm4Filter::from_reals([
            &safe_coefficients[0],
            &safe_coefficients[1],
            &safe_coefficients[2],
            &safe_coefficients[3],
        ])
        .expect("a 500-bit coefficient span keeps every product normal");
        let safe_query =
            RationalLinearForm4Query::from_rationals([&one, &minimum_safe, &zero, &zero])
                .expect("a 500-bit query span keeps every product normal");
        assert_eq!(safe_filter.sign(&safe_query), Some(RealSign::Positive),);

        let wide_coefficients = [
            Real::new(one.clone()),
            Real::new(wide_lane.clone()),
            Real::zero(),
            Real::zero(),
        ];
        let wide_filter = RationalLinearForm4Filter::from_reals([
            &wide_coefficients[0],
            &wide_coefficients[1],
            &wide_coefficients[2],
            &wide_coefficients[3],
        ])
        .expect("every normalized lane remains a normal binary64 value");
        let wide_query = RationalLinearForm4Query::from_rationals([&zero, &wide_lane, &one, &zero])
            .expect("the 501-bit query span remains representable");
        assert_eq!(wide_filter.sign(&wide_query), Some(RealSign::Positive));
        assert!(RationalLinearForm4Query::from_affine_point3([&wide_lane, &zero, &zero]).is_some());

        let underflow_lane = rational(f64::from_bits((1023_u64 - 512) << 52));
        let underflow_coefficients = [
            Real::new(one.clone()),
            Real::new(underflow_lane.clone()),
            Real::zero(),
            Real::zero(),
        ];
        let underflow_filter = RationalLinearForm4Filter::from_reals([
            &underflow_coefficients[0],
            &underflow_coefficients[1],
            &underflow_coefficients[2],
            &underflow_coefficients[3],
        ])
        .expect("the 512-bit coefficient span remains representable");
        let underflow_query =
            RationalLinearForm4Query::from_rationals([&zero, &underflow_lane, &one, &zero])
                .expect("the 512-bit query span remains representable");
        assert_eq!(
            underflow_filter.sign(&underflow_query),
            None,
            "the absolute error floor must decline when the only nonzero product underflows",
        );

        let largest_power = rational(f64::from_bits(2046_u64 << 52));
        let upper_safe = rational(f64::from_bits((2046_u64 - 500) << 52));
        assert!(
            RationalLinearForm4Query::from_rationals([&largest_power, &upper_safe, &zero, &zero,])
                .is_some(),
            "normalization must also work when its exact reciprocal is subnormal",
        );
        let tiny_but_convertible = rational(f64::from_bits(50_u64 << 52));
        assert!(
            RationalLinearForm4Query::from_rationals([
                &largest_power,
                &tiny_but_convertible,
                &zero,
                &zero,
            ])
            .is_none(),
            "a nonzero lane that would scale to zero must use exact fallback",
        );
    }

    #[test]
    fn parse() {
        let counting: Real = "123456789".parse().unwrap();
        let answer = Real::new(Rational::new(123456789));
        assert_eq!(counting, answer);
    }

    #[test]
    fn parse_large() {
        let input: Real = "378089444731722233953867379643788100".parse().unwrap();
        let root = Rational::new(614889782588491410);
        let answer = Real::new(root.clone() * root);
        assert_eq!(input, answer);
    }

    #[test]
    fn parse_fraction() {
        let input: Real = "98760/123450".parse().unwrap();
        let answer = Real::new(Rational::fraction(9876, 12345).unwrap());
        assert_eq!(input, answer);
    }

    #[test]
    fn parse_scientific_notation_exactly() {
        let input: Real = "-7.78437e-005".parse().unwrap();
        let answer: Real = "-0.0000778437".parse().unwrap();
        assert_eq!(input, answer);
        assert_eq!(
            input.exact_rational(),
            Some("-778437/10000000000".parse().unwrap())
        );
    }

    #[test]
    fn parse_extreme_scientific_notation_as_a_lazy_exact_real() {
        assert_eq!("1e2000000".parse::<Rational>(), Err(Problem::Exhausted));

        let input = "1e2000000".parse::<Real>().unwrap();
        assert_eq!(input.exact_rational(), None);
        assert_eq!(input.structural_facts().sign, Some(RealSign::Positive));

        let extreme_exponent = format!("1e{}", "9".repeat(400));
        assert!(extreme_exponent.parse::<Real>().is_ok());
        let extreme_negative_exponent = format!("1e-{}", "9".repeat(400));
        assert!(extreme_negative_exponent.parse::<Real>().is_ok());
    }

    #[test]
    fn root_divide() {
        let twenty: Real = 20.into();
        let five: Real = 5.into();
        let a = twenty.sqrt().unwrap();
        let b = five.sqrt().unwrap().inverse().unwrap();
        let answer = a * b;
        let two: Real = 2.into();
        assert_eq!(answer, two);

        let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
        let sqrt_three = Real::new(Rational::new(3)).sqrt().unwrap();
        let product = &sqrt_two * &sqrt_three;
        let quotient = (&sqrt_two / &sqrt_three).unwrap();
        assert_eq!(product, Real::new(Rational::new(6)).sqrt().unwrap());
        assert_eq!(
            quotient * Real::new(Rational::new(3)),
            Real::new(Rational::new(6)).sqrt().unwrap()
        );

        let magnitude = Real::new(Rational::new(293)).sqrt().unwrap();
        let first = magnitude.clone().inverse().unwrap();
        let second = magnitude.inverse().unwrap();
        assert!(std::ptr::eq(&*first.rational, &*second.rational));
    }

    #[test]
    fn division_checks_the_denominator_before_identity_rewrites() {
        let zero = Real::zero();
        assert_eq!(&zero / &zero, Err(Problem::DivideByZero));
        assert_eq!(&zero / &Real::one(), Ok(Real::zero()));
    }

    #[test]
    fn homogeneous_quadratic_interpolation_division_preserves_nonzero_numerator() {
        let third = Real::new(Rational::fraction(1, 3).unwrap());
        let two_thirds = Real::new(Rational::fraction(2, 3).unwrap());
        let weight = (Real::from(2_i32).sqrt().unwrap() / Real::from(2_i32)).unwrap();

        let first_y = &weight * &third;
        let second_y = (&weight * &two_thirds) + &third;
        let homogeneous_y = (&first_y * &two_thirds) + (&second_y * &third);

        let first_weight = &two_thirds + (&weight * &third);
        let second_weight = (&weight * &two_thirds) + &third;
        let homogeneous_weight = (&first_weight * &two_thirds) + (&second_weight * &third);

        assert!(!homogeneous_y.definitely_zero());
        assert_close(homogeneous_y.clone(), 0.42538079163846554, 1e-12);
        assert_close(homogeneous_weight.clone(), 0.8698252360829101, 1e-12);
        assert_close(
            homogeneous_weight.inverse_ref().unwrap(),
            1.1496562280755465,
            1e-12,
        );
        let coordinate = (&homogeneous_y / &homogeneous_weight).unwrap();
        assert_close(coordinate, 0.4890416764108682, 1e-12);
    }

    #[test]
    fn rational() {
        let two: Real = 2.into();
        assert_ne!(two, Real::zero());
        let four: Real = 4.into();
        let answer = four - two;
        let two: Real = 2.into();
        assert_eq!(answer, two);
        let zero = answer - two;
        assert_eq!(zero, Real::zero());
        let six_half: Real = "13/2".parse().unwrap();
        let opposite = six_half.inverse().unwrap();
        let expected: Real = "2/13".parse().unwrap();
        assert_eq!(opposite, expected);
    }

    // Perfect-square roots must remain exact.
    #[test]
    fn perfect_square() {
        let four: Real = 4.into();
        let two: Real = 2.into();
        let calc = four.sqrt().unwrap() - two;
        assert_eq!(calc, Real::zero());
    }

    #[test]
    fn one_over_e() {
        let one: Real = 1.into();
        let e = Real::e();
        let e_inverse = Real::e().inverse().unwrap();
        let answer = e * e_inverse;
        assert_eq!(one, answer);
        let again = answer.sqrt().unwrap();
        assert_eq!(one, again);
    }

    #[test]
    fn unlike_sqrts() {
        let thirty: Real = 30.into();
        let ten: Real = 10.into();
        let answer = thirty.sqrt().unwrap() * ten.sqrt().unwrap();
        let ten: Real = 10.into();
        let three: Real = 3.into();
        let or = ten * three.sqrt().unwrap();
        assert_eq!(answer, or);
    }

    #[test]
    fn zero_pi() {
        let pi = Real::pi();
        let z1 = pi - Real::pi();
        let pi2 = Real::pi() + Real::pi();
        let z2 = pi2 * Real::zero();
        assert!(z1.definitely_zero());
        assert!(z2.definitely_zero());
        let two_pi = Real::pi() + Real::pi();
        let two: Real = 2.into();
        assert_eq!(two_pi, two * Real::pi());
        assert_ne!(two_pi, Rational::new(2));
    }

    #[test]
    fn zero_status_uses_structural_facts_without_refinement() {
        assert_eq!(Real::zero().zero_status(), ZeroKnowledge::Zero);
        assert_eq!(
            Real::new(Rational::fraction(-7, 8).unwrap()).zero_status(),
            ZeroKnowledge::NonZero
        );
        assert_eq!(Real::pi().zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(Real::e().zero_status(), ZeroKnowledge::NonZero);

        let near_pi = Real::pi() - Real::new(Rational::fraction(22, 7).unwrap());
        assert_eq!(near_pi.zero_status(), ZeroKnowledge::NonZero);
    }

    #[test]
    fn const_offsets_certify_simple_pi_and_e_gaps() {
        use crate::real::Class::{ConstOffset, Irrational};

        let pi_minus_three = Real::pi() - Real::new(Rational::new(3));
        assert!(matches!(pi_minus_three.class, ConstOffset(_)));
        assert_eq!(pi_minus_three.zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(
            pi_minus_three.structural_facts().sign,
            Some(RealSign::Positive)
        );

        let two_pi_minus_six = Real::new(Rational::new(2)) * Real::pi() - Real::from(6_i32);
        assert!(matches!(two_pi_minus_six.class, ConstOffset(_)));
        assert_eq!(
            two_pi_minus_six.structural_facts().sign,
            Some(RealSign::Positive)
        );

        let e_minus_two = Real::e() - Real::new(Rational::new(2));
        assert!(matches!(e_minus_two.class, ConstOffset(_)));
        assert_eq!(
            e_minus_two.structural_facts().sign,
            Some(RealSign::Positive)
        );

        let close_rational = Real::pi() - Real::new(Rational::fraction(22, 7).unwrap());
        assert!(matches!(close_rational.class, Irrational));
    }

    #[test]
    fn ln_zero() {
        let zero = Real::zero();
        assert_eq!(zero.ln(), Err(Problem::NotANumber));
    }

    #[test]
    fn sqrt_exact() {
        let big: Real = 40_000.into();
        let small: Rational = Rational::new(200);
        let answer = big.sqrt().unwrap();
        assert_eq!(answer, small);
    }

    #[test]
    fn sqrt_scaled_squarefree_reuses_symbolic_residual() {
        let answer = Real::from(18_i32).sqrt().unwrap();
        let expected = Real::from(3_i32) * Real::from(2_i32).sqrt().unwrap();
        assert_eq!(answer, expected);
    }

    #[test]
    fn sqrt_rational_denominator_uses_the_canonical_quadratic_surd() {
        let sqrt_half = Real::new(Rational::fraction(1, 2).unwrap()).sqrt().unwrap();
        let sqrt_two_over_two = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
        assert_eq!(sqrt_half, sqrt_two_over_two);
        assert_eq!(
            (sqrt_half - sqrt_two_over_two).zero_status(),
            ZeroKnowledge::Zero
        );
    }

    #[test]
    fn sqrt_recovers_quadratic_surds_with_perfect_square_norms() {
        let root_two = Real::from(2_i32).sqrt().unwrap();
        for (input, expected) in [
            (
                Real::from(3_i32) + Real::from(2_i32) * &root_two,
                root_two.clone() + Real::one(),
            ),
            (
                Real::from(3_i32) - Real::from(2_i32) * &root_two,
                root_two.clone() - Real::one(),
            ),
            (
                Real::from(17_i32) + Real::from(12_i32) * &root_two,
                Real::from(3_i32) + Real::from(2_i32) * &root_two,
            ),
            (
                Real::from(12_i32) + Real::from(8_i32) * &root_two,
                Real::from(2_i32) * &root_two + Real::from(2_i32),
            ),
        ] {
            let root = input.clone().sqrt().unwrap();
            assert_eq!(
                root.certified_eq_until(&expected, -256).as_bool(),
                Some(true)
            );
            assert_eq!(
                (&root * &root).certified_eq_until(&input, -256).as_bool(),
                Some(true)
            );
        }

        for input in [Real::one() + &root_two, Real::from(5_i32) + &root_two] {
            let root = input.clone().sqrt().unwrap();
            assert_eq!(
                root.detailed_facts().symbolic.kind,
                StructuralKind::ComputableOpaque
            );
        }
    }

    #[test]
    fn sqrt_sum_and_difference_identity_is_certified_for_rational_pairs() {
        let mut state = 0x9e37_79b9_7f4a_7c15_u64;
        for case in 0..128 {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let x = Rational::fraction(
                (state % 1_000_003 + 1) as i64,
                state.rotate_left(17) % 10_007 + 1,
            )
            .unwrap();
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let y = Rational::fraction(
                (state % 1_000_033 + 1) as i64,
                state.rotate_left(29) % 10_009 + 1,
            )
            .unwrap();

            let x_real = Real::new(x.clone());
            let y_real = Real::new(y.clone());
            let x_root = x_real.clone().sqrt().unwrap();
            let y_root = y_real.clone().sqrt().unwrap();
            let cross = Real::from(2_i32) * (x_real.clone() * y_real.clone()).sqrt().unwrap();
            let plus = (x_real.clone() + y_real.clone() + &cross).sqrt().unwrap();
            assert_eq!(
                (x_root.clone() + y_root.clone())
                    .certified_eq_until(&plus, -2_048)
                    .as_bool(),
                Some(true),
                "plus case {case}: x={x:?}, y={y:?}"
            );

            let minus = (x_real + y_real - cross).sqrt().unwrap();
            let expected_minus = if x >= y {
                x_root - y_root
            } else {
                y_root - x_root
            };
            assert_eq!(
                expected_minus.certified_eq_until(&minus, -2_048).as_bool(),
                Some(true),
                "minus case {case}: x={x:?}, y={y:?}"
            );
        }
    }

    #[test]
    fn square_sqrt() {
        let two: Real = 2.into();
        let three: Real = 3.into();
        let small = three.sqrt().expect("Should be able to sqrt(n)");
        let a = small * two;
        let three: Real = 3.into();
        let small = three.sqrt().expect("Should be able to sqrt(n)");
        let three: Real = 3.into();
        let b = small * three;
        let answer = a * b;
        let eighteen: Rational = Rational::new(18);
        assert_eq!(answer, eighteen);
    }

    #[test]
    fn adding_one_works() {
        let pi = Real::pi();
        let one: Real = 1.into();
        let plus_one = pi + one;
        let float: f64 = plus_one.into();
        assert_eq!(float, 4.141592653589793);
    }

    #[test]
    fn sin_easy() {
        let pi = Real::pi();
        let zero = Real::zero();
        let two: Real = 2.into();
        let two_pi = pi.clone() * two;
        assert_eq!(zero.clone().sin(), zero);
        assert_eq!(pi.clone().sin(), zero);
        assert_eq!(two_pi.clone().sin(), zero);
    }

    #[test]
    fn cos_easy() {
        let pi = Real::pi();
        let zero = Real::zero();
        let one: Real = 1.into();
        let two: Real = 2.into();
        let two_pi = pi.clone() * two;
        let minus_one: Real = (-1).into();
        assert_eq!(zero.clone().cos(), one);
        assert_eq!(pi.clone().cos(), minus_one);
        assert_eq!(two_pi.clone().cos(), one);
    }

    fn pi_fraction(n: i64, d: u64) -> Real {
        Real::new(Rational::fraction(n, d).unwrap()) * Real::pi()
    }

    #[test]
    fn sin_pi_rational_multiples() {
        let zero = Real::zero();
        let one: Real = 1.into();
        let minus_one: Real = (-1).into();
        let half: Real = "1/2".parse().unwrap();
        let minus_half: Real = "-1/2".parse().unwrap();
        let sqrt_two_over_two = Real::new(Rational::fraction(1, 2).unwrap())
            * Real::new(Rational::new(2)).sqrt().unwrap();
        let sqrt_three_over_two = Real::new(Rational::fraction(1, 2).unwrap())
            * Real::new(Rational::new(3)).sqrt().unwrap();

        assert_eq!(pi_fraction(0, 1).sin(), zero);
        assert_eq!(pi_fraction(1, 6).sin(), half);
        assert_eq!(pi_fraction(1, 4).sin(), sqrt_two_over_two);
        assert_eq!(pi_fraction(1, 3).sin(), sqrt_three_over_two);
        assert_eq!(pi_fraction(1, 2).sin(), one);
        assert_eq!(pi_fraction(5, 6).sin(), half);
        assert_eq!(pi_fraction(7, 6).sin(), minus_half);
        assert_eq!(pi_fraction(3, 2).sin(), minus_one);
        assert_eq!(pi_fraction(-1, 6).sin(), minus_half);
        assert_eq!(pi_fraction(2, 1).sin(), zero);
    }

    #[test]
    fn sin_pi_rational_multiples_fold_to_same_curve() {
        assert_eq!(pi_fraction(1, 5).sin(), pi_fraction(4, 5).sin());
        assert_eq!(pi_fraction(6, 5).sin(), -pi_fraction(1, 5).sin());
        assert_eq!(pi_fraction(-4, 5).sin(), -pi_fraction(1, 5).sin());
        assert_eq!(pi_fraction(11, 5).sin(), pi_fraction(1, 5).sin());
    }

    #[test]
    fn distinct_opaque_irrationals_do_not_share_an_algebraic_basis() {
        let left = pi_fraction(1, 5).sin() + Real::one();
        let right = pi_fraction(1, 7).sin() + Real::one();

        assert_ne!(left, right);
        assert_ne!(&left - &right, Real::zero());
        assert_ne!((&left / &right).unwrap(), Real::one());

        let clone = left.clone();
        assert_eq!(left, clone);
        assert_eq!(&left - &clone, Real::zero());
        assert_eq!((&left / &clone).unwrap(), Real::one());
    }

    #[test]
    fn opposite_sign_sum_does_not_certify_sign_from_inexact_msd() {
        let five_pi_over_four =
            Real::tau() * (Real::from(20_u8) / Real::from(32_u8)).expect("nonzero sample count");
        let offset_sample = (Real::one() / Real::from(2_u8)).unwrap()
            + (Real::from(3_u8) / Real::from(4_u8)).unwrap() * five_pi_over_four.sin();

        assert_eq!(
            offset_sample.refine_sign_until(-4096),
            Some(RealSign::Negative),
            "offset sample facts were {:#?}",
            offset_sample.structural_facts()
        );
        let approximation = offset_sample.to_f64_lossy().unwrap();
        assert!(
            (approximation - (0.5 - 0.375 * std::f64::consts::SQRT_2)).abs() < 1.0e-12,
            "5pi/4 offset sample was {approximation}"
        );
    }

    #[test]
    fn cos_pi_rational_multiples_shift_through_sin() {
        let zero = Real::zero();
        let one: Real = 1.into();
        let minus_one: Real = (-1).into();
        let half: Real = "1/2".parse().unwrap();
        let minus_half: Real = "-1/2".parse().unwrap();
        let sqrt_two_over_two = Real::new(Rational::fraction(1, 2).unwrap())
            * Real::new(Rational::new(2)).sqrt().unwrap();
        let sqrt_three_over_two = Real::new(Rational::fraction(1, 2).unwrap())
            * Real::new(Rational::new(3)).sqrt().unwrap();

        assert_eq!(pi_fraction(0, 1).cos(), one);
        assert_eq!(pi_fraction(1, 6).cos(), sqrt_three_over_two);
        assert_eq!(pi_fraction(1, 4).cos(), sqrt_two_over_two);
        assert_eq!(pi_fraction(1, 3).cos(), half);
        assert_eq!(pi_fraction(1, 2).cos(), zero);
        assert_eq!(pi_fraction(2, 3).cos(), minus_half);
        assert_eq!(pi_fraction(4, 3).cos(), minus_half);
        assert_eq!(pi_fraction(5, 3).cos(), half);
        assert_eq!(pi_fraction(-4, 3).cos(), minus_half);
        assert_eq!(pi_fraction(1, 1).cos(), minus_one);
        assert_eq!(pi_fraction(3, 2).cos(), zero);
        assert_eq!(pi_fraction(-1, 3).cos(), half);
        assert_eq!(pi_fraction(2, 1).cos(), one);
    }

    #[test]
    fn non_tabulated_cos_pi_reuses_direct_sin_pi_certificates() {
        for (numerator, denominator) in [
            (-17, 11),
            (-9, 7),
            (-2, 7),
            (1, 7),
            (5, 7),
            (9, 7),
            (17, 11),
        ] {
            let turn = Rational::fraction(numerator, denominator).unwrap();
            let direct = pi_fraction(numerator, denominator).cos();
            let scaled = Real::new(turn.clone()).cos_pi();
            let complementary = Real::new(turn + Rational::fraction(1, 2).unwrap()).sin_pi();

            assert_eq!(direct, scaled);
            assert_eq!(direct, complementary);
            assert!(
                (direct.to_f64_lossy().unwrap()
                    - (std::f64::consts::PI * numerator as f64 / denominator as f64).cos())
                .abs()
                    < 1.0e-12
            );
        }
    }

    #[test]
    fn public_pi_scaled_trig_uses_exact_rational_turns() {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        let sqrt_two_over_two = Real::new(Rational::fraction(1, 2).unwrap())
            * Real::new(Rational::new(2)).sqrt().unwrap();
        let sqrt_three = Real::new(Rational::new(3)).sqrt().unwrap();
        let sqrt_three_over_three =
            Real::new(Rational::fraction(1, 3).unwrap()) * sqrt_three.clone();

        assert_eq!(Real::new(Rational::fraction(1, 6).unwrap()).sin_pi(), half);
        assert_eq!(
            Real::new(Rational::fraction(1, 4).unwrap()).cos_pi(),
            sqrt_two_over_two
        );
        assert_eq!(
            Real::new(Rational::fraction(1, 4).unwrap())
                .tan_pi()
                .unwrap(),
            Real::one()
        );
        assert_eq!(
            Real::new(Rational::fraction(1, 6).unwrap())
                .tan_pi()
                .unwrap(),
            sqrt_three_over_three
        );
        assert_eq!(
            Real::new(Rational::fraction(1, 3).unwrap())
                .tan_pi()
                .unwrap(),
            sqrt_three
        );
        assert_eq!(
            Real::new(Rational::fraction(1, 2).unwrap())
                .tan_pi()
                .unwrap_err(),
            Problem::NotANumber
        );
    }

    #[test]
    fn cotangent_preserves_exact_turns_poles_and_inverse_trig_images() {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        let two = Real::from(2_i32);
        let sqrt_three = Real::from(3_i32).sqrt().unwrap();
        let sqrt_three_over_three =
            Real::new(Rational::fraction(1, 3).unwrap()) * sqrt_three.clone();

        assert_eq!(pi_fraction(1, 6).cot().unwrap(), sqrt_three);
        assert_eq!(pi_fraction(1, 4).cot().unwrap(), Real::one());
        assert_eq!(pi_fraction(1, 3).cot().unwrap(), sqrt_three_over_three);
        assert_eq!(pi_fraction(1, 2).cot().unwrap(), Real::zero());
        assert_eq!(pi_fraction(3, 4).cot().unwrap(), -Real::one());
        assert_eq!(pi_fraction(-1, 4).cot().unwrap(), -Real::one());

        for pole in [
            Real::zero(),
            Real::pi(),
            -Real::pi(),
            Real::from(2_i32) * Real::pi(),
        ] {
            assert_eq!(pole.cot(), Err(Problem::NotANumber));
        }

        assert_eq!(half.clone().cot_pi().unwrap(), Real::zero());
        assert_eq!(Real::zero().cot_pi(), Err(Problem::NotANumber));
        assert_eq!(Real::one().cot_pi(), Err(Problem::NotANumber));
        assert_eq!(
            Real::new(Rational::fraction(1, 5).unwrap())
                .cot_pi()
                .unwrap(),
            pi_fraction(1, 5).cot().unwrap()
        );

        let atan_two = two.clone().atan().unwrap();
        assert_eq!(atan_two.clone().cot().unwrap(), half.clone());
        assert_eq!((-atan_two.clone()).cot().unwrap(), -half.clone());
        assert_eq!((Real::pi() + atan_two.clone()).cot().unwrap(), half.clone());
        assert_eq!((Real::pi() - atan_two.clone()).cot().unwrap(), -half);
        assert_eq!((pi_fraction(1, 2) - atan_two).cot().unwrap(), two);

        let three_fifths = Real::new(Rational::fraction(3, 5).unwrap());
        assert_eq!(
            three_fifths.clone().asin().unwrap().cot().unwrap(),
            Real::new(Rational::fraction(4, 3).unwrap())
        );
        assert_eq!(
            three_fifths.acos().unwrap().cot().unwrap(),
            Real::new(Rational::fraction(3, 4).unwrap())
        );
    }

    #[test]
    fn small_angle_helpers_remove_zero_singularities() {
        assert_eq!(Real::zero().sinc().unwrap(), Real::one());
        assert_eq!(Real::zero().sinc_pi().unwrap(), Real::one());
        assert_eq!(
            Real::zero().cosc().unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );

        assert_eq!(
            Real::new(Rational::fraction(1, 2).unwrap())
                .sinc_pi()
                .unwrap(),
            (Real::from(2_i32) / Real::pi()).unwrap()
        );
        assert!(
            Real::new(Rational::new(1))
                .sinc_pi()
                .unwrap()
                .definitely_zero()
        );

        // This identity is mathematically zero but deliberately remains an
        // opaque computable graph: resolving its sign by bounded refinement is
        // impossible. The analytic small-angle nodes must remove the three
        // singularities without misclassifying that uncertainty as exact zero.
        let angle = Real::e();
        let sine = angle.clone().sin();
        let cosine = angle.cos();
        let opaque_zero = sine.clone() * sine + cosine.clone() * cosine - Real::one();
        assert_eq!(opaque_zero.zero_status(), ZeroKnowledge::Unknown);

        let sinc = opaque_zero.clone().sinc().expect("opaque-zero sinc");
        let sinc_pi = opaque_zero
            .clone()
            .sinc_pi()
            .expect("opaque-zero normalized sinc");
        let cosc = opaque_zero.cosc().expect("opaque-zero cosc");
        assert_eq!(sinc.zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(sinc_pi.zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(cosc.zero_status(), ZeroKnowledge::NonZero);

        for precision in [-16, -64, -256, -1_024] {
            let expected_one = Real::one().fold_ref().approx(precision);
            let expected_half = Real::new(Rational::fraction(1, 2).unwrap())
                .fold_ref()
                .approx(precision);
            assert!(
                (sinc.fold_ref().approx(precision) - &expected_one).abs() <= num::BigInt::from(1),
                "opaque-zero sinc missed its limit at {precision}"
            );
            assert!(
                (sinc_pi.fold_ref().approx(precision) - &expected_one).abs()
                    <= num::BigInt::from(1),
                "opaque-zero sinc_pi missed its limit at {precision}"
            );
            assert!(
                (cosc.fold_ref().approx(precision) - &expected_half).abs() <= num::BigInt::from(1),
                "opaque-zero cosc missed its limit at {precision}"
            );
        }
        assert_eq!(format!("{sinc:#.12}"), "1.000000000000");
        assert_eq!(format!("{sinc_pi:#.12}"), "1.000000000000");
        assert_eq!(format!("{cosc:#.12}"), "0.500000000000");

        use std::sync::{Arc, atomic::AtomicBool};
        let signal = Arc::new(AtomicBool::new(false));
        let mut signaled = {
            let angle = Real::e();
            let sine = angle.clone().sin();
            let cosine = angle.cos();
            sine.clone() * sine + cosine.clone() * cosine - Real::one()
        };
        signaled.abort(Arc::clone(&signal));
        let continued = signaled.sinc().expect("untriggered signal permits sinc");
        assert!(
            continued
                .abort_signal()
                .is_some_and(|attached| Arc::ptr_eq(attached, &signal))
        );

        signal.store(true, std::sync::atomic::Ordering::Relaxed);
        let mut pre_aborted = {
            let angle = Real::e();
            let sine = angle.clone().sin();
            let cosine = angle.cos();
            sine.clone() * sine + cosine.clone() * cosine - Real::one()
        };
        pre_aborted.abort(signal);
        assert_eq!(pre_aborted.sinc(), Err(Problem::UnknownZero));
    }

    #[test]
    fn trig_integer_pi_offsets_reduce_to_residual() {
        use crate::real::Class::ConstOffset;

        let eps: Real = "0.00000000000000000001".parse().unwrap();
        let even = Real::pi() * Real::from(1000_i32) + eps.clone();
        assert!(matches!(even.class, ConstOffset(_)));

        let expected_sin: f64 = eps.clone().sin().into();
        let expected_cos: f64 = eps.clone().cos().into();
        let expected_tan: f64 = eps.clone().tan().unwrap().into();

        assert!(closest_f64(even.clone().sin(), expected_sin));
        assert!(closest_f64(even.clone().cos(), expected_cos));
        assert!(closest_f64(even.tan().unwrap(), expected_tan));

        let odd = Real::pi() * Real::from(1001_i32) + eps.clone();
        assert!(matches!(odd.class, ConstOffset(_)));
        let expected_odd_sin: f64 = (-eps.clone().sin()).into();
        let expected_odd_cos: f64 = (-eps.clone().cos()).into();

        assert!(closest_f64(odd.clone().sin(), expected_odd_sin));
        assert!(closest_f64(odd.cos(), expected_odd_cos));
    }

    #[test]
    fn tan_irrational_argument() {
        let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
        let answer = sqrt_two.tan().unwrap();
        let actual: f64 = answer.into();
        assert!((actual - 6.3341191670421955).abs() < 1e-12, "{actual}");
    }

    #[test]
    fn exact_rational_is_owned_and_public() {
        let value = Real::new(Rational::fraction(9, 18).unwrap());
        assert_eq!(
            value.exact_rational(),
            Some(Rational::fraction(1, 2).unwrap())
        );
        assert!(value.is_exact_dyadic_rational());

        let decimal = Real::new(Rational::fraction(1, 10).unwrap());
        assert!(!decimal.is_exact_dyadic_rational());

        let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
        assert_eq!(sqrt_two.exact_rational(), None);
        assert!(!sqrt_two.is_exact_dyadic_rational());

        let exp_ln_8 = Real::new(Rational::new(8)).ln().unwrap().exp().unwrap();
        assert_eq!(exp_ln_8.exact_rational(), Some(Rational::new(8)));
        assert!(exp_ln_8.is_exact_dyadic_rational());
    }

    #[test]
    fn real_structural_facts_for_rational_and_constants() {
        let negative = Real::new(Rational::fraction(-7, 8).unwrap()).structural_facts();
        assert_eq!(
            negative,
            RealStructuralFacts {
                sign: Some(RealSign::Negative),
                zero: ZeroKnowledge::NonZero,
                exact_rational: true,
                magnitude: Some(MagnitudeBits {
                    msd: -1,
                    exact_msd: true,
                }),
            }
        );

        let pi = Real::pi().structural_facts();
        assert_eq!(pi.sign, Some(RealSign::Positive));
        assert_eq!(pi.zero, ZeroKnowledge::NonZero);
        assert!(!pi.exact_rational);
        assert_eq!(pi.magnitude.map(|m| m.msd), Some(1));

        let e = Real::e().structural_facts();
        assert_eq!(e.sign, Some(RealSign::Positive));
        assert_eq!(e.zero, ZeroKnowledge::NonZero);

        let e = Real::e().detailed_facts();
        assert_eq!(e.ordering.cmp_one, StructuralComparison::Greater);
        assert_eq!(e.ordering.abs_cmp_one, StructuralComparison::Greater);
        assert_eq!(e.domains.acosh, DomainStatus::Valid);

        let inverse_pi = Real::pi().inverse().unwrap().detailed_facts();
        assert_eq!(inverse_pi.ordering.cmp_one, StructuralComparison::Less);
        assert_eq!(inverse_pi.ordering.abs_cmp_one, StructuralComparison::Less);
        assert_eq!(inverse_pi.domains.acosh, DomainStatus::Invalid);
        assert_eq!(inverse_pi.domains.atanh, DomainStatus::Valid);

        let negative_e = (-Real::e()).detailed_facts();
        assert_eq!(negative_e.ordering.cmp_one, StructuralComparison::Less);
        assert_eq!(
            negative_e.ordering.abs_cmp_one,
            StructuralComparison::Greater
        );

        // Exact MSDs of two non-unit factors cannot simply be added: their
        // product may carry into the next binade. Keep that comparison unknown
        // until an exact predicate resolves it.
        let scaled_e = Real::e() * Real::new(Rational::fraction(3, 8).unwrap());
        assert_eq!(
            scaled_e.detailed_facts().ordering.cmp_one,
            StructuralComparison::Unknown
        );
        assert!(scaled_e.acosh().is_ok());
    }

    #[test]
    fn real_detailed_facts_report_cheap_rational_and_symbolic_structure() {
        let half = Real::new(Rational::fraction(1, 2).unwrap()).detailed_facts();
        assert!(half.base.exact_rational);
        assert_eq!(
            half.identity.zero_one_or_minus_one,
            ZeroOneMinusOneStatus::NeitherOrUnknown
        );
        assert!(half.rational.exact_dyadic);
        assert!(!half.rational.exact_integer);
        assert_eq!(half.ordering.abs_cmp_one, StructuralComparison::Less);
        assert_eq!(half.domains.reciprocal, DomainStatus::Valid);
        assert_eq!(half.domains.asin_acos, DomainStatus::Valid);
        assert_eq!(half.domains.unit_interval_closed, DomainStatus::Valid);
        assert_eq!(half.domains.unit_interval_open, DomainStatus::Valid);
        assert_eq!(half.domains.atanh, DomainStatus::Valid);
        assert_eq!(half.primitive.f64, PrimitiveFloatStatus::NormalFinite);
        assert_eq!(half.symbolic.kind, StructuralKind::ExactRational);
        assert_eq!(half.symbolic.degree, ExpressionDegree::Constant);
        assert!(half.symbolic.dependencies.is_empty());

        let two = Real::new(Rational::new(2)).detailed_facts();
        assert_eq!(
            two.identity.zero_one_or_minus_one,
            ZeroOneMinusOneStatus::NeitherOrUnknown
        );
        assert!(two.rational.exact_integer);
        assert!(two.rational.exact_small_integer_i64);
        assert!(two.rational.power_of_two);
        assert_eq!(two.rational.storage, RationalStorageClass::WordSized);
        assert_eq!(two.primitive.f32, PrimitiveFloatStatus::NormalFinite);
        assert_eq!(two.ordering.cmp_one, StructuralComparison::Greater);
        assert_eq!(two.domains.asin_acos, DomainStatus::Invalid);
        assert_eq!(two.domains.unit_interval_closed, DomainStatus::Invalid);
        assert_eq!(two.domains.acosh, DomainStatus::Valid);
        assert_eq!(two.domains.atanh, DomainStatus::Invalid);

        let pi_sqrt_two = Real::pi() * Real::from(2_i32).sqrt().unwrap();
        let symbolic = pi_sqrt_two.detailed_facts();
        assert_eq!(
            symbolic.identity.zero_one_or_minus_one,
            ZeroOneMinusOneStatus::NeitherOrUnknown
        );
        assert_eq!(symbolic.symbolic.kind, StructuralKind::SqrtLike);
        assert_eq!(symbolic.symbolic.degree, ExpressionDegree::Constant);
        assert!(symbolic.symbolic.has_pi_factor);
        assert!(symbolic.symbolic.has_sqrt_factor);
        assert!(
            symbolic
                .symbolic
                .dependencies
                .contains(SymbolicDependencyMask::PI)
        );
        assert!(
            symbolic
                .symbolic
                .dependencies
                .contains(SymbolicDependencyMask::SQRT)
        );
        assert!(
            !symbolic
                .symbolic
                .dependencies
                .contains(SymbolicDependencyMask::LOG)
        );
        assert_eq!(symbolic.base.sign, Some(RealSign::Positive));
    }

    #[test]
    fn symbolic_facts_report_dependency_families_and_degree() {
        let pi_exp = Real::pi() * Real::e();
        let facts = pi_exp.detailed_facts().symbolic;
        assert_eq!(facts.degree, ExpressionDegree::Constant);
        assert!(facts.dependencies.contains(SymbolicDependencyMask::PI));
        assert!(facts.dependencies.contains(SymbolicDependencyMask::EXP));
        assert!(facts.has_pi_factor);
        assert!(facts.has_exp_factor);
        assert!(!facts.has_log_factor);
        assert!(!facts.has_trig_factor);

        let log_facts = Real::from(2_i32).ln().unwrap().detailed_facts().symbolic;
        assert_eq!(log_facts.degree, ExpressionDegree::Constant);
        assert!(log_facts.dependencies.contains(SymbolicDependencyMask::LOG));
        assert!(log_facts.has_log_factor);

        let rational_power_facts = Real::new(Rational::fraction(1, 7).unwrap())
            .exp2()
            .unwrap()
            .detailed_facts()
            .symbolic;
        assert_eq!(rational_power_facts.kind, StructuralKind::ExpLike);
        assert_eq!(rational_power_facts.degree, ExpressionDegree::Constant);
        assert!(
            rational_power_facts
                .dependencies
                .contains(SymbolicDependencyMask::EXP)
        );
        assert!(
            rational_power_facts
                .dependencies
                .contains(SymbolicDependencyMask::LOG)
        );

        let trig_facts = pi_fraction(1, 5).sin().detailed_facts().symbolic;
        assert_eq!(trig_facts.degree, ExpressionDegree::Constant);
        assert!(
            trig_facts
                .dependencies
                .contains(SymbolicDependencyMask::TRIG)
        );
        assert!(trig_facts.dependencies.contains(SymbolicDependencyMask::PI));
        assert!(trig_facts.has_trig_factor);
    }

    #[test]
    fn real_domain_accessors_expose_structural_certificates_without_refinement() {
        let zero = Real::zero();
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        let minus_two = Real::from(-2_i32);
        let pi = Real::pi();

        assert_eq!(zero.reciprocal_domain(), DomainStatus::Invalid);
        assert_eq!(zero.sqrt_domain(), DomainStatus::Valid);
        assert_eq!(zero.log_domain(), DomainStatus::Invalid);
        assert_eq!(zero.asin_acos_domain(), DomainStatus::Valid);
        assert_eq!(zero.atanh_domain(), DomainStatus::Valid);

        assert_eq!(half.reciprocal_domain(), DomainStatus::Valid);
        assert_eq!(half.sqrt_domain(), DomainStatus::Valid);
        assert_eq!(half.log_domain(), DomainStatus::Valid);
        assert_eq!(half.asin_acos_domain(), DomainStatus::Valid);
        assert_eq!(half.atanh_domain(), DomainStatus::Valid);

        assert_eq!(minus_two.sqrt_domain(), DomainStatus::Invalid);
        assert_eq!(minus_two.log_domain(), DomainStatus::Invalid);
        assert_eq!(minus_two.asin_acos_domain(), DomainStatus::Invalid);
        assert_eq!(minus_two.acosh_domain(), DomainStatus::Invalid);
        assert_eq!(minus_two.atanh_domain(), DomainStatus::Invalid);

        assert_eq!(pi.domain_facts().sqrt, DomainStatus::Valid);
        assert_eq!(pi.domain_facts().reciprocal, DomainStatus::Valid);
        assert_eq!(pi.asin_acos_domain(), DomainStatus::Invalid);
        assert_eq!(pi.acosh_domain(), DomainStatus::Valid);
    }

    #[test]
    fn zero_one_or_minus_one_reports_signed_unit_identity() {
        assert_eq!(
            Real::zero().zero_one_or_minus_one(),
            ZeroOneMinusOneStatus::Zero
        );
        assert_eq!(
            Real::one().zero_one_or_minus_one(),
            ZeroOneMinusOneStatus::One
        );
        assert_eq!(
            (-Real::one()).zero_one_or_minus_one(),
            ZeroOneMinusOneStatus::MinusOne
        );
        assert_eq!(
            Real::new(Rational::fraction(1, 2).unwrap()).zero_one_or_minus_one(),
            ZeroOneMinusOneStatus::NeitherOrUnknown
        );
        assert_eq!(
            Real::pi().zero_one_or_minus_one(),
            ZeroOneMinusOneStatus::NeitherOrUnknown
        );
    }

    #[test]
    fn pi_exp_products_remain_symbolically_combinable() {
        let left = Real::pi() * Real::new(Rational::fraction(7, 8).unwrap());
        let right = Real::e() * Real::new(Rational::fraction(5, 6).unwrap());
        let product = &left * &right;
        let doubled = &product + &product;

        assert_eq!(product.zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(doubled.zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(doubled, product.clone() * Real::new(Rational::new(2)));
        assert_eq!(doubled.structural_facts().sign, Some(RealSign::Positive));

        let pi_square = &Real::pi() * &Real::pi();
        assert_eq!(
            &pi_square + &pi_square,
            pi_square.clone() * Real::from(2_i32)
        );

        let pi_sqrt_two = &Real::pi() * Real::from(2_i32).sqrt().unwrap();
        assert_eq!(
            &pi_sqrt_two + &pi_sqrt_two,
            pi_sqrt_two.clone() * Real::from(2_i32)
        );

        let ln_product = Real::from(2_i32).ln().unwrap() * Real::from(3_i32).ln().unwrap();
        assert_eq!(
            &ln_product + &ln_product,
            ln_product.clone() * Real::from(2_i32)
        );
    }

    #[test]
    fn symbolic_constant_multiplication_and_division_reduce() {
        use crate::real::Class::ConstProductSqrt;

        let pi = Real::pi();
        let e = Real::e();
        let pi_square = &pi * &pi;

        let pi_e = &Real::pi() * &Real::e();
        let pi_e_square = &pi_e * &pi_e;
        assert_eq!((&pi_e_square / &pi_e).unwrap(), pi_e);

        let e_three = Real::new(Rational::new(3)).exp().unwrap();
        let e_two = Real::new(Rational::new(2)).exp().unwrap();
        assert_eq!((&e_three / &e).unwrap(), e_two.clone());
        assert_eq!(
            (&Real::new(Rational::one()) / &e).unwrap(),
            e.clone().inverse().unwrap()
        );

        let pi_over_e = (&Real::pi() / &Real::e()).unwrap();
        assert_eq!(&pi_over_e * &Real::e(), Real::pi());
        let inverse_pi = Real::pi().inverse().unwrap();
        assert_eq!(&inverse_pi * &Real::pi(), Real::new(Rational::one()));
        assert_eq!(
            (&Real::new(Rational::one()) / &Real::pi()).unwrap(),
            inverse_pi
        );
        assert_eq!((&Real::e() / &Real::pi()).unwrap() * &Real::pi(), Real::e());

        let pi_cube_e_five =
            &(&pi_square * &Real::pi()) * &Real::new(Rational::new(5)).exp().unwrap();
        let pi_e_two = &Real::pi() * &e_two;
        let quotient = (&pi_cube_e_five / &pi_e_two).unwrap();
        let expected = &pi_square * Real::new(Rational::new(3)).exp().unwrap();
        assert_eq!(quotient, expected);
        let inverse_pi_e = pi_e.clone().inverse().unwrap();
        assert_eq!(inverse_pi_e * &pi_e, Real::new(Rational::one()));

        let sqrt_two = Real::from(2_i32).sqrt().unwrap();
        let pi_e_sqrt_two = &pi_e * &sqrt_two;
        assert!(matches!(pi_e_sqrt_two.class, ConstProductSqrt(_)));
        assert_eq!(&pi_e_sqrt_two * &sqrt_two, Real::from(2_i32) * &pi_e);
        assert_eq!((&pi_e_sqrt_two / &e).unwrap(), &pi * &sqrt_two);
        assert_eq!(
            pi_e_sqrt_two.clone().inverse().unwrap() * &pi_e_sqrt_two,
            Real::new(Rational::one())
        );
    }

    #[test]
    fn iterator_products_balance_exact_prefixes_and_preserve_mixed_order() {
        let empty: Vec<Real> = Vec::new();
        assert_eq!(empty.clone().into_iter().product::<Real>(), Real::one());
        assert_eq!(empty.iter().product::<Real>(), Real::one());

        let wallis_rationals: Vec<Rational> = (1_i64..=513)
            .map(|index| {
                let square4 = 4 * index * index;
                Rational::fraction(square4, u64::try_from(square4 - 1).unwrap()).unwrap()
            })
            .collect();
        let expected_rational = wallis_rationals
            .iter()
            .fold(Rational::one(), |product, factor| &product * factor);
        let exact_factors: Vec<Real> = wallis_rationals.into_iter().map(Real::new).collect();
        assert_eq!(
            exact_factors.clone().into_iter().product::<Real>(),
            Real::new(expected_rational.clone())
        );
        assert_eq!(
            exact_factors.iter().product::<Real>(),
            Real::new(expected_rational)
        );

        let mixed = [
            Real::from(2_i32),
            Real::from(3_i32),
            Real::from(5_i32),
            Real::pi(),
            Real::e(),
            Real::from(7_i32),
        ];
        let sequential = mixed
            .iter()
            .fold(Real::one(), |product, factor| &product * factor);
        assert_eq!(mixed.clone().into_iter().product::<Real>(), sequential);
        assert_eq!(mixed.iter().product::<Real>(), sequential);

        assert_eq!(
            [Real::from(2_i32), Real::zero(), Real::pi(), Real::e()]
                .into_iter()
                .product::<Real>(),
            Real::zero()
        );
    }

    #[test]
    fn ln_scaled_exp_reduces_to_log_scale_plus_exponent() {
        use crate::real::Class::LnAffine;

        let scaled = Real::new(Rational::new(2)) * Real::e();
        let expected = Real::new(Rational::new(2)).ln().unwrap() + Real::new(Rational::one());
        let actual = scaled.ln().unwrap();
        assert!(matches!(actual.class, LnAffine(_)));
        assert!(closest_f64(actual, expected.into()));
    }

    #[test]
    fn real_refine_sign_until_handles_refined_and_unresolved_cases() {
        let tiny = Real::new(
            Rational::from_bigint_fraction(num::BigInt::from(1), num::BigUint::from(1_u8) << 64)
                .unwrap(),
        );
        let near_pi = Real::pi() - tiny;
        assert_eq!(near_pi.refine_sign_until(-8), Some(RealSign::Positive));

        let certified = Real::pi() - Real::new(Rational::new(3));
        assert_eq!(certified.refine_sign_until(0), Some(RealSign::Positive));
    }

    #[test]
    fn oversized_rational_sqrt_retains_exact_computable_without_recursive_reconstruction() {
        let radicand = Rational::from_bigint(
            (num::BigInt::from(1_u8) << 5_001_usize) + num::BigInt::from(3_u8),
        );
        assert!(!radicand.extract_square_will_succeed());

        let root = Real::new(radicand.clone()).sqrt().unwrap();

        assert_eq!(
            root.detailed_facts().symbolic.kind,
            StructuralKind::ComputableOpaque
        );
        assert_eq!(root.refine_sign_until(0), Some(RealSign::Positive));
        assert_eq!(root.fold_ref().square().exact_rational(), Some(radicand));
    }

    #[test]
    fn certified_dyadic_interval_is_exact_for_rationals_and_bounds_symbolic_values() {
        let exact = Rational::fraction(7, 3).unwrap();
        assert_eq!(
            Real::new(exact.clone()).certified_dyadic_interval(-32),
            Some([exact.clone(), exact]),
        );

        let [pi_lower, pi_upper] = Real::pi().certified_dyadic_interval(-32).unwrap();
        assert!(pi_lower < pi_upper);
        assert!(pi_lower > Rational::new(3));
        assert!(pi_upper < Rational::new(4));

        let negative = -(Real::from(3) * Real::pi());
        let [lower, upper] = negative.certified_dyadic_interval(-32).unwrap();
        assert!(lower <= upper);
        assert!(lower > Rational::new(-10));
        assert!(upper < Rational::new(-9));
    }

    #[test]
    fn certified_sign_until_reports_proof_source_without_lossy_approximation() {
        let exact = Real::from(-7);
        assert_eq!(
            exact.certified_sign_until(-16),
            CertifiedRealSign::Known {
                sign: RealSign::Negative,
                certificate: RealSignCertificate::StructuralFacts,
            }
        );

        let zero_scale = Real::zero() * Real::pi();
        assert_eq!(
            zero_scale.certified_sign_until(-16),
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                certificate: RealSignCertificate::StructuralFacts,
            }
        );

        let bounded = Real::pi() - Real::new(Rational::fraction(103_993, 33_102).unwrap());
        assert_eq!(bounded.structural_facts().sign, None);
        assert_eq!(
            bounded.certified_sign_until(-64),
            CertifiedRealSign::Known {
                sign: RealSign::Positive,
                certificate: RealSignCertificate::BoundedRefinement { min_precision: -64 },
            }
        );

        let unresolved = Real::pi() - Real::new(Rational::fraction(103_993, 33_102).unwrap());
        assert_eq!(
            unresolved.certified_sign_until(0),
            CertifiedRealSign::Unknown { min_precision: 0 }
        );
        assert_eq!(unresolved.refine_sign_until(0), None);
    }

    #[test]
    fn certified_sign_refines_mixed_pi_atan_terms() {
        let negative_argument = Real::from(2).sqrt().unwrap() - Real::from(2);
        let value = Real::from(-2) * negative_argument.atan().unwrap() - pi_fraction(1, 8);

        assert_eq!(value.structural_facts().sign, None);
        let [lower, upper] = value.certified_dyadic_interval(-64).unwrap();
        assert!(lower > Rational::zero());
        assert!(upper >= lower);
        assert_eq!(
            value.certified_sign_until(-64).sign(),
            Some(RealSign::Positive)
        );
        assert_eq!(
            value.certified_cmp_until(&Real::zero(), -64).ordering(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn immediate_sign_never_walks_or_refines_an_opaque_expression() {
        assert_eq!(Real::from(-7).immediate_sign(), Some(RealSign::Negative));
        assert_eq!(Real::zero().immediate_sign(), Some(RealSign::Zero));
        assert_eq!(Real::pi().immediate_sign(), Some(RealSign::Positive));

        let value = Real::pi() - Real::new(Rational::fraction(103_993, 33_102).unwrap());
        assert_eq!(value.immediate_sign(), None);
        assert_eq!(
            value.certified_sign_until(-64).sign(),
            Some(RealSign::Positive)
        );
        assert_eq!(value.immediate_sign(), Some(RealSign::Positive));
        assert_eq!((-value).immediate_sign(), Some(RealSign::Negative));
    }

    #[test]
    fn cycloidal_root_angles_compare_in_geometric_order() {
        let generating_radius = Real::new(Rational::fraction(3, 4).unwrap());
        let root_phase = Real::new(Rational::fraction(-121, 174).unwrap())
            .acos()
            .unwrap();
        let root_parameter = (root_phase.clone() * generating_radius.clone() / Real::from(8))
            .expect("pitch radius is nonzero");
        let hypocycle_radius = Real::new(Rational::fraction(29, 4).unwrap());
        let hypocycle_ratio = (hypocycle_radius.clone() / generating_radius.clone())
            .expect("generating radius is nonzero");
        let parameter = -root_parameter;
        let rolling = hypocycle_ratio * parameter.clone();
        let x = hypocycle_radius.clone() * parameter.clone().cos()
            + generating_radius.clone() * rolling.clone().cos();
        let y = hypocycle_radius * parameter.sin() - generating_radius * rolling.sin();

        let angular_pitch = (Real::tau() / Real::from(16)).expect("tooth count is nonzero");
        let flank_rotation = -(angular_pitch.clone() / Real::from(4)).expect("four is nonzero");
        let flank_sine = flank_rotation.clone().sin();
        let flank_cosine = flank_rotation.cos();
        let rotated_x = x.clone() * flank_cosine.clone() - y.clone() * flank_sine.clone();
        let rotated_y = x * flank_sine + y * flank_cosine;
        let right_root = rotated_y.atan2(rotated_x);
        let left_root = -right_root.clone();
        let next_right_root = angular_pitch + right_root;
        let value = left_root - next_right_root;

        let [lower, upper] = value.certified_dyadic_interval(-64).unwrap();
        assert!(lower > Rational::zero());
        assert!(upper >= lower);
        assert_eq!(
            value.certified_sign_until(-64).sign(),
            Some(RealSign::Positive)
        );
        assert_eq!(
            value.certified_cmp_until(&Real::zero(), -64).ordering(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn certified_eq_until_reports_structural_and_exact_rational_results() {
        let two = Real::from(2);
        assert_eq!(
            two.certified_eq_until(&Real::from(2), -16),
            CertifiedRealEquality::Equal {
                certificate: RealEqualityCertificate::StructuralEquality,
            }
        );
        assert_eq!(
            two.certified_eq_until(&Real::from(2), -16).as_bool(),
            Some(true)
        );

        assert_eq!(
            two.certified_eq_until(&Real::from(3), -16),
            CertifiedRealEquality::NotEqual {
                certificate: RealEqualityCertificate::ExactRationalComparison,
            }
        );
        assert_eq!(
            two.certified_eq_until(&Real::from(3), -16).as_bool(),
            Some(false)
        );
    }

    #[test]
    fn certified_eq_until_proves_semantic_equality_through_difference() {
        let left = Real::new(Rational::new(1024)).ln().unwrap();
        let right = Real::new(Rational::new(10)) * Real::new(Rational::new(2)).ln().unwrap();

        assert_eq!(left.certified_eq_until(&right, -64).as_bool(), Some(true));
    }

    #[test]
    fn certified_eq_until_structurally_separates_nearby_rational_and_irrational_values() {
        let near_pi = Real::new(Rational::fraction(103_993, 33_102).unwrap());

        for min_precision in [0, -64] {
            assert_eq!(
                Real::pi().certified_eq_until(&near_pi, min_precision),
                CertifiedRealEquality::NotEqual {
                    certificate: RealEqualityCertificate::StructuralFacts,
                }
            );
        }
    }

    #[test]
    fn certified_cmp_until_reports_structural_exact_and_refined_ordering() {
        use core::cmp::Ordering;

        let two = Real::from(2);
        assert_eq!(
            two.certified_cmp_until(&Real::from(2), -16),
            CertifiedRealOrdering::Known {
                ordering: Ordering::Equal,
                certificate: RealOrderingCertificate::StructuralEquality,
            }
        );
        assert_eq!(
            two.certified_cmp_until(&Real::from(3), -16),
            CertifiedRealOrdering::Known {
                ordering: Ordering::Less,
                certificate: RealOrderingCertificate::ExactRationalComparison,
            }
        );

        let near_pi = Real::new(Rational::fraction(103_993, 33_102).unwrap());
        assert_eq!(
            Real::pi().certified_cmp_until(&near_pi, 0),
            CertifiedRealOrdering::Unknown { min_precision: 0 }
        );
        assert_eq!(
            Real::pi().certified_cmp_until(&near_pi, -64),
            CertifiedRealOrdering::Known {
                ordering: Ordering::Greater,
                certificate: RealOrderingCertificate::BoundedRefinement { min_precision: -64 },
            }
        );
    }

    #[test]
    fn certified_cmp_until_uses_operand_sign_and_magnitude_facts() {
        let pi = Real::pi();
        let minus_pi = -pi.clone();
        assert_eq!(
            minus_pi.certified_cmp_until(&pi, 0),
            CertifiedRealOrdering::Known {
                ordering: Ordering::Less,
                certificate: RealOrderingCertificate::StructuralFacts,
            }
        );

        let scaled_pi = &pi * &Real::from(1_u64 << 40);
        assert_eq!(
            pi.certified_cmp_until(&scaled_pi, 0),
            CertifiedRealOrdering::Known {
                ordering: Ordering::Less,
                certificate: RealOrderingCertificate::StructuralFacts,
            }
        );
    }

    #[test]
    fn scaled_irrational_magnitude_does_not_claim_an_exact_product_binade() {
        let radius = Real::new(Rational::fraction(225, 8).unwrap())
            .sqrt()
            .unwrap();
        let magnitude = radius
            .structural_facts()
            .magnitude
            .expect("positive quadratic surd has magnitude facts");

        assert!(!magnitude.exact_msd);
        assert_eq!(
            radius.certified_cmp_until(&Real::from(4), -64).ordering(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn quadratic_surd_weight_discriminant_retains_its_positive_sign() {
        let weight = Real::from(2_i8).sqrt().unwrap();
        let b = Real::from(2_i8) * (&weight - Real::one());
        let a = Real::from(2_i8) * (Real::one() - &weight);
        let delta = Real::from(4_i8) * a - &b * &b;
        let discriminant = Real::zero() - delta;

        assert_eq!(
            discriminant
                .certified_cmp_until(&Real::zero(), -512)
                .ordering(),
            Some(Ordering::Greater)
        );
        assert_eq!(discriminant.sqrt().unwrap(), Real::from(2_i8));
    }

    #[test]
    fn partial_ord_uses_certified_real_comparison() {
        use core::cmp::Ordering;

        assert_eq!(
            Real::from(1).partial_cmp(&Real::from(2)),
            Some(Ordering::Less)
        );

        let near_pi = Real::new(Rational::fraction(103_993, 33_102).unwrap());
        assert_eq!(Real::pi().partial_cmp(&near_pi), Some(Ordering::Greater));
    }

    #[test]
    fn powi() {
        let base: Real = 4.into();
        let five_over_two: Real = "5/2".parse().unwrap();
        let answer = base.pow(five_over_two).unwrap();
        let correct: Real = 32.into();
        assert_eq!(answer, correct);
    }

    #[test]
    fn powi_i64_matches_arbitrary_precision_exponents() {
        let values = [
            Real::new(Rational::fraction(7, 5).unwrap()),
            Real::new(Rational::new(3)).sqrt().unwrap(),
            Real::pi(),
            Real::pi() - Real::new(Rational::fraction(103_993, 33_102).unwrap()),
        ];
        for value in values {
            for exponent in [-3_i64, -1, 0, 1, 2, 5, 17] {
                assert_eq!(
                    value.clone().powi_i64(exponent),
                    value.clone().powi(num::BigInt::from(exponent))
                );
            }
        }

        assert_eq!(Real::zero().powi_i64(0), Err(Problem::NotANumber));
        assert_eq!(Real::zero().powi_i64(-2), Err(Problem::NotANumber));
        assert_eq!(Real::from(-1).powi_i64(i64::MIN), Ok(Real::one()));
    }

    #[test]
    fn large_rational_powers_use_the_exact_lazy_fallback() {
        let exponent = 20_000_i64;
        let via_i64 = Real::from(10).powi_i64(exponent).unwrap();
        let via_bigint = Real::from(10).powi(num::BigInt::from(exponent)).unwrap();

        assert!(via_i64.computable.is_some());
        assert!(via_bigint.computable.is_some());
        assert_eq!(
            via_i64.certified_cmp_until(&via_bigint, -512),
            CertifiedRealOrdering::Known {
                ordering: core::cmp::Ordering::Equal,
                certificate: RealOrderingCertificate::StructuralEquality,
            }
        );
    }

    #[test]
    fn powi_negative_unknown_sign_matches_inverse() {
        let near_pi = Real::pi() - Real::new(Rational::fraction(103_993, 33_102).unwrap());
        assert_eq!(near_pi.structural_facts().sign, None);

        let pow = near_pi.clone().powi(num::BigInt::from(-1)).unwrap();
        let inverse = near_pi.inverse().unwrap();
        let actual: f64 = pow.into();
        let expected: f64 = inverse.into();
        assert!(((actual - expected) / expected).abs() < 1e-8);

        let zero = Real::pi() - Real::pi();
        assert!(zero.powi(num::BigInt::from(-1)).is_err());
    }

    #[test]
    fn powi_negative_one_reuses_symbolic_inverse() {
        let pow = Real::pi().powi(num::BigInt::from(-1)).unwrap();
        let inverse = Real::pi().inverse().unwrap();

        assert_eq!(pow, inverse);
        assert_eq!(pow * Real::pi(), Real::new(Rational::one()));
    }

    #[test]
    fn sqrt_3045512() {
        use crate::real::Class::Sqrt;

        let n: Real = 3045512.into();
        let sqrt = n.sqrt().unwrap();
        let root = Rational::new(1234);
        assert_eq!(sqrt.rational, root);
        let two = Rational::new(2);
        assert_eq!(sqrt.class, Sqrt(two));
    }

    #[test]
    fn nth_roots_and_rational_powers_preserve_exact_cases() {
        assert_eq!(Real::from(27_i32).cbrt().unwrap(), Real::from(3_i32));
        assert_eq!(Real::from(-27_i32).cbrt().unwrap(), Real::from(-3_i32));
        assert_eq!(
            Real::new(Rational::fraction(8, 27).unwrap())
                .root_n(3)
                .unwrap(),
            Real::new(Rational::fraction(2, 3).unwrap())
        );
        assert_eq!(Real::from(81_i32).root_n(4).unwrap(), Real::from(3_i32));
        assert_eq!(Real::from(5_i32).root_n(1).unwrap(), Real::from(5_i32));
        assert_eq!(Real::zero().root_n(7).unwrap(), Real::zero());
        assert_eq!(Real::from(16_i32).root_n(0), Err(Problem::NotANumber));
        assert_eq!(Real::from(-16_i32).root_n(4), Err(Problem::SqrtNegative));

        let two_thirds = Rational::fraction(2, 3).unwrap();
        assert_eq!(
            Real::from(-8_i32).pow_rational(two_thirds).unwrap(),
            Real::from(4_i32)
        );
        assert_eq!(
            Real::from(16_i32)
                .pow_rational(Rational::fraction(3, 2).unwrap())
                .unwrap(),
            Real::from(64_i32)
        );

        let cube_root_two = Real::from(2_i32)
            .pow_rational(Rational::fraction(1, 3).unwrap())
            .unwrap();
        let reconstructed =
            cube_root_two.clone() * cube_root_two.clone() * cube_root_two - Real::from(2_i32);
        assert_eq!(reconstructed.refine_sign_until(-512), Some(RealSign::Zero));
        let generic_cube_root = Real::from(2_i32)
            .pow(Real::new(Rational::fraction(1, 3).unwrap()))
            .unwrap();
        let generic_reconstructed =
            generic_cube_root.clone() * generic_cube_root.clone() * generic_cube_root
                - Real::from(2_i32);
        assert_eq!(
            generic_reconstructed.refine_sign_until(-512),
            Some(RealSign::Zero)
        );
        assert_eq!(
            Real::from(-8_i32)
                .pow(Real::new(Rational::fraction(1, 3).unwrap()))
                .unwrap(),
            Real::from(-2_i32)
        );

        let two_fifths = Real::from(2_i32)
            .pow_rational(Rational::fraction(2, 5).unwrap())
            .unwrap();
        let fifth_power = (0..5).fold(Real::one(), |product, _| product * two_fifths.clone());
        assert_eq!(
            (fifth_power - Real::from(4_i32)).refine_sign_until(-512),
            Some(RealSign::Zero)
        );

        let inverse_cube_root = Real::from(2_i32)
            .pow_rational(Rational::fraction(-1, 3).unwrap())
            .unwrap();
        let inverse_identity = (0..3).fold(Real::one(), |product, _| {
            product * inverse_cube_root.clone()
        }) * Real::from(2_i32)
            - Real::one();
        assert_eq!(
            inverse_identity.refine_sign_until(-512),
            Some(RealSign::Zero)
        );

        let ninth_root = Real::from(17_i32).root_n(9).unwrap();
        let ninth_power = (0..9).fold(Real::one(), |product, _| product * ninth_root.clone());
        assert_eq!(
            (ninth_power - Real::from(17_i32)).refine_sign_until(-1_024),
            Some(RealSign::Zero)
        );
    }

    fn closest_f64(r: Real, f: f64) -> bool {
        let left = f64::from_bits(f.to_bits() - 1);
        let right = f64::from_bits(f.to_bits() + 1);
        let f: f64 = r.into();
        if right > left {
            left < f && right > f
        } else {
            left > f && right < f
        }
    }

    #[test]
    fn pow_pi() {
        let pi = Real::pi();
        let sq = pi.pow(Real::pi()).unwrap();
        assert!(closest_f64(sq.clone(), 36.46215960720791));
        let sqsq = sq.pow(Real::pi()).unwrap();
        assert!(closest_f64(sqsq, 80662.6659385546));
    }

    #[test]
    fn pow_fract() {
        let frac: Real = "-1.3".parse().unwrap();
        let five: Real = 5.into();
        let answer = frac.pow(five).unwrap();
        assert!(closest_f64(answer, -3.71293));
    }

    #[test]
    fn pow_of_sine() {
        let sin_10 = Real::new(Rational::new(10)).sin();
        let answer = (sin_10.clone()).pow(Real::new(Rational::new(2))).unwrap();
        assert!(closest_f64(
            answer,
            // Value from wolframalpha.com
            0.295_958_969_093_304
        ));
    }

    #[test]
    fn curves() {
        let eighty = Rational::fraction(80, 100).unwrap();
        let twenty = Rational::fraction(20, 100).unwrap();
        assert_eq!(curve(eighty), (false, twenty.clone()));
        let forty = Rational::fraction(40, 100).unwrap();
        let sixty = Rational::fraction(60, 100).unwrap();
        assert_eq!(curve(sixty), (false, forty));
        let otf = Rational::fraction(124, 100).unwrap();
        let tf = Rational::fraction(24, 100).unwrap();
        assert_eq!(curve(otf), (true, tf.clone()));
        let minus_twenty = Rational::fraction(-20, 100).unwrap();
        assert_eq!(curve(minus_twenty), (true, twenty));
        let minus_otf = Rational::fraction(-124, 100).unwrap();
        assert_eq!(curve(minus_otf), (false, tf));
    }

    #[test]
    fn exp_pi() {
        let pi = Real::pi();
        assert_eq!(format!("{pi:.2e}"), "3.14e0");
        assert_eq!(format!("{pi:.4E}"), "3.1416E0");
        assert_eq!(format!("{pi:.8e}"), "3.14159265e0");
        assert_eq!(format!("{pi:.16E}"), "3.1415926535897932E0");
        assert_eq!(format!("{pi:.32e}"), "3.14159265358979323846264338327950e0");
        assert_eq!(format!("{pi:e}"), "3.1415926535897932384626433832795e0");
    }

    #[test]
    fn ln_division() {
        let fifth = Rational::fraction(2, 10).unwrap();
        let twenty_fifth = Rational::fraction(4, 100).unwrap();
        let ln_5th = Real::new(fifth).ln().unwrap();
        let ln_25th = Real::new(twenty_fifth).ln().unwrap();
        let answer = ln_25th / ln_5th;
        assert_eq!(answer.unwrap(), Rational::new(2));
    }

    #[test]
    fn retained_roots_and_rational_powers_have_exact_log_exp_round_trips() {
        let half = Rational::fraction(1, 2).unwrap();
        let minus_half = Rational::fraction(-1, 2).unwrap();
        let three_halves = Rational::fraction(3, 2).unwrap();
        let third = Rational::fraction(1, 3).unwrap();
        let ln_two = Real::from(2_i32).ln().unwrap();
        let ln_ten = Real::from(10_i32).ln().unwrap();
        let sqrt_two = Real::from(2_i32).sqrt().unwrap();

        assert_eq!(
            sqrt_two.clone().ln().unwrap(),
            Real::new(half.clone()) * ln_two.clone()
        );
        assert_eq!(
            Real::from(8_i32).sqrt().unwrap().ln().unwrap(),
            Real::new(three_halves.clone()) * ln_two.clone()
        );
        assert_eq!(
            (sqrt_two.clone() / Real::from(2_i32))
                .unwrap()
                .ln()
                .unwrap(),
            -Real::new(half.clone()) * ln_two.clone()
        );
        assert_eq!(
            (Real::new(half.clone()) * ln_two.clone()).exp().unwrap(),
            sqrt_two
        );
        assert_eq!(
            (Real::new(minus_half) * ln_two.clone()).exp().unwrap(),
            Real::from(2_i32).sqrt().unwrap().inverse().unwrap()
        );
        assert_eq!(
            (Real::new(three_halves) * ln_two.clone()).exp().unwrap(),
            Real::from(8_i32).sqrt().unwrap()
        );

        let two_to_third = Real::new(third.clone()).exp2().unwrap();
        assert_eq!(
            two_to_third.clone().ln().unwrap(),
            Real::new(third.clone()) * ln_two.clone()
        );
        assert_eq!(
            (Real::new(third.clone()) * ln_two).exp().unwrap(),
            Real::from(2_i32).pow_rational(third.clone()).unwrap()
        );
        assert_eq!(
            (Real::from(4_i32) * two_to_third).ln().unwrap(),
            Real::new(Rational::fraction(7, 3).unwrap()) * Real::from(2_i32).ln().unwrap()
        );
        let ten_to_third = Real::new(third.clone()).exp10().unwrap();
        assert_eq!(
            ten_to_third.clone().ln().unwrap(),
            Real::new(third.clone()) * ln_ten.clone()
        );
        assert_eq!(
            (Real::new(third.clone()) * ln_ten).exp().unwrap(),
            Real::from(10_i32).pow_rational(third).unwrap()
        );
        assert_eq!(
            (Real::from(1_000_i32) * ten_to_third).ln().unwrap(),
            Real::new(Rational::fraction(10, 3).unwrap()) * Real::from(10_i32).ln().unwrap()
        );

        for base in [2_i32, 3, 5, 6, 7, 10, 11, 17] {
            let base_real = Real::from(base);
            let root = base_real.clone().sqrt().unwrap();
            let log = base_real.clone().ln().unwrap();
            for power in -3_i32..=3 {
                let scale = base_real.clone().powi(num::BigInt::from(power)).unwrap();
                let exponent = Rational::fraction(i64::from(2 * power + 1), 2).unwrap();
                assert_eq!(
                    (scale * root.clone()).ln().unwrap(),
                    Real::new(exponent) * log.clone(),
                    "sqrt/log base {base}, outer power {power}"
                );
            }
        }

        for base in [2_i32, 3, 5, 7, 10, 17] {
            let base_real = Real::from(base);
            let log = base_real.clone().ln().unwrap();
            for denominator in 2_u64..=9 {
                for numerator in -5_i64..=5 {
                    if numerator == 0 {
                        continue;
                    }
                    let exponent = Rational::fraction(numerator, denominator).unwrap();
                    assert_eq!(
                        (Real::new(exponent.clone()) * log.clone()).exp().unwrap(),
                        base_real.clone().pow_rational(exponent).unwrap(),
                        "exp/log base {base}, exponent {numerator}/{denominator}"
                    );
                }
            }
        }
    }

    #[test]
    fn ln_large_positive_does_not_panic() {
        let ln = Real::from(1_000_001_i32).ln().unwrap();
        assert!(closest_f64(ln, 13.815511557963774));
    }

    #[test]
    fn ln_large_computable_positive_does_not_panic() {
        let value = Real::from(100_i32) + Real::from(2_i32).sqrt().unwrap();
        let ln = value.ln().unwrap();
        let actual: f64 = ln.into();
        assert!((actual - 4.619213444287964).abs() < 1e-6);
    }

    #[test]
    fn integer_logs() {
        for (n, log) in [
            (1, 0),
            (10, 1),
            (10_000_000_000_000_000, 16),
            (100_000_000_000_000_000, 17),
            (1_000_000_000_000_000_000, 18),
        ] {
            let n = Real::new(Rational::new(n));
            let answer = n.log10().unwrap();
            assert_eq!(answer, Rational::new(log));
        }
    }

    #[test]
    fn base_two_and_ten_exponentials_preserve_exact_rational_powers() {
        assert_eq!(Real::from(10_i32).exp2().unwrap(), Real::from(1024_i32));
        assert_eq!(Real::from(3_i32).exp10().unwrap(), Real::from(1000_i32));
        assert_eq!(
            Real::from(-3_i32).exp2().unwrap(),
            Real::new(Rational::fraction(1, 8).unwrap())
        );

        let half = Real::new(Rational::fraction(1, 2).unwrap());
        assert_eq!(
            half.clone().exp2().unwrap(),
            Real::from(2_i32).sqrt().unwrap()
        );
        assert_eq!(half.exp10().unwrap(), Real::from(10_i32).sqrt().unwrap());
    }

    #[test]
    fn base_two_and_ten_logarithm_round_trips_are_exact() {
        for denominator in 1_u64..=12 {
            for numerator in 1_i64..=24 {
                let real = Real::new(Rational::fraction(numerator, denominator).unwrap());
                assert_eq!(real.clone().log2().unwrap().exp2().unwrap(), real);
                assert_eq!(real.clone().log10().unwrap().exp10().unwrap(), real);
            }
        }
    }

    #[test]
    fn rational_base_power_logarithm_round_trips_are_exact() {
        for denominator in 1_u64..=16 {
            for numerator in -32_i64..=32 {
                let exponent = Rational::fraction(numerator, denominator).unwrap();
                let expected = Real::new(exponent);
                assert_eq!(expected.clone().exp2().unwrap().log2().unwrap(), expected);
                assert_eq!(expected.clone().exp10().unwrap().log10().unwrap(), expected);
            }
        }

        // LOG10HAF's hard exponent is deliberately outside the small grid.
        for exponent in [
            Rational::fraction(-6411, 4096).unwrap(),
            Rational::fraction(6411, 4096).unwrap(),
        ] {
            let expected = Real::new(exponent);
            let power_two = expected.clone().exp2().unwrap();
            let power_ten = expected.clone().exp10().unwrap();

            // Exercise both clone reconstruction from the compact certificate
            // and the original retained computable payload.
            assert_eq!(power_two.clone().log2().unwrap(), expected);
            assert_eq!(power_ten.clone().log10().unwrap(), expected);
            assert_eq!(power_two.log2().unwrap(), expected);
            assert_eq!(power_ten.log10().unwrap(), expected);
        }

        let one_seventh = Real::new(Rational::fraction(1, 7).unwrap());
        let power = one_seventh.clone().exp10().unwrap();
        assert_ne!(power.log2().unwrap(), one_seventh);
    }

    #[test]
    fn generic_power_uses_exact_retained_base_logarithm_certificates() {
        let log2_three = Real::from(3_i32).log2().unwrap();
        assert_eq!(
            Real::from(2_i32).pow(log2_three).unwrap(),
            Real::from(3_i32)
        );

        let log10_two = Real::from(2_i32).log10().unwrap();
        assert_eq!(
            Real::from(10_i32).pow(log10_two).unwrap(),
            Real::from(2_i32)
        );

        let scaled_log2_nine =
            Real::new(Rational::fraction(1, 2).unwrap()) * Real::from(9_i32).log2().unwrap();
        assert_eq!(scaled_log2_nine.exp2().unwrap(), Real::from(3_i32));

        for scale in [
            Rational::fraction(-3, 2).unwrap(),
            Rational::fraction(-1, 3).unwrap(),
            Rational::fraction(2, 3).unwrap(),
            Rational::fraction(5, 2).unwrap(),
        ] {
            let log2_five = Real::new(scale.clone()) * Real::from(5_i32).log2().unwrap();
            let expected = Real::from(5_i32).pow_rational(scale.clone()).unwrap();
            assert_eq!(log2_five.exp2().unwrap(), expected);

            let log10_three = Real::new(scale.clone()) * Real::from(3_i32).log10().unwrap();
            let expected = Real::from(3_i32).pow_rational(scale).unwrap();
            assert_eq!(log10_three.exp10().unwrap(), expected);
        }
    }

    #[test]
    fn base_two_and_ten_exponentials_match_binary64_for_irrational_exponents() {
        let exponent = Real::from(2_i32).sqrt().unwrap();
        assert_close(
            exponent.clone().exp2().unwrap(),
            2_f64.powf(2_f64.sqrt()),
            1e-12,
        );
        assert_close(exponent.exp10().unwrap(), 10_f64.powf(2_f64.sqrt()), 1e-12);
    }

    #[test]
    fn inverse_trig_exact_values() {
        assert_eq!(
            Real::new(Rational::fraction(1, 2).unwrap()).asin().unwrap(),
            pi_fraction(1, 6)
        );
        assert_eq!(
            Real::new(Rational::fraction(1, 2).unwrap()).acos().unwrap(),
            pi_fraction(1, 3)
        );
        assert_eq!(
            Real::new(Rational::new(1)).atan().unwrap(),
            pi_fraction(1, 4)
        );

        let sine = pi_fraction(1, 5).sin();
        assert_eq!(sine.asin().unwrap(), pi_fraction(1, 5));

        let tangent = pi_fraction(1, 5).tan().unwrap();
        assert_eq!(tangent.atan().unwrap(), pi_fraction(1, 5));
    }

    #[test]
    fn inverse_trig_exact_principal_branches() {
        assert_eq!(pi_fraction(6, 7).sin().asin().unwrap(), pi_fraction(1, 7));
        assert_eq!(pi_fraction(-6, 7).sin().asin().unwrap(), pi_fraction(-1, 7));
        assert_eq!(pi_fraction(9, 7).cos().acos().unwrap(), pi_fraction(5, 7));
        assert_eq!(
            pi_fraction(6, 7).tan().unwrap().atan().unwrap(),
            pi_fraction(-1, 7)
        );
        assert_eq!(
            pi_fraction(-6, 7).tan().unwrap().atan().unwrap(),
            pi_fraction(1, 7)
        );
    }

    #[test]
    fn inverse_trig_general_values() {
        assert!(closest_f64(
            Real::new(Rational::fraction(3, 10).unwrap())
                .asin()
                .unwrap(),
            0.3046926540153975
        ));
        assert!(closest_f64(
            Real::new(Rational::fraction(3, 10).unwrap())
                .acos()
                .unwrap(),
            1.2661036727794992
        ));
        assert!(closest_f64(
            Real::new(Rational::new(2)).atan().unwrap(),
            1.1071487177940904
        ));
    }

    #[test]
    fn inverse_trig_compositions_retain_exact_structural_images() {
        let three_fifths = Real::new(Rational::fraction(3, 5).unwrap());
        let two_fifths = Real::new(Rational::fraction(2, 5).unwrap());

        assert_eq!(three_fifths.clone().asin().unwrap().sin(), three_fifths);
        assert_eq!(two_fifths.clone().acos().unwrap().cos(), two_fifths);

        let slope = Real::new(Rational::fraction(2, 3).unwrap());
        let angle = slope.clone().atan().unwrap();
        let denominator = (Real::one() + &slope * &slope).sqrt().unwrap();
        assert_eq!(angle.clone().sin(), (&slope / &denominator).unwrap());
        assert_eq!(angle.cos(), (Real::one() / denominator).unwrap());

        let root_five = Real::from(5).sqrt().unwrap();
        let radical_slope = (-root_five.clone() / Real::from(2)).unwrap();
        let radical_angle = radical_slope.atan().unwrap();
        assert_eq!(
            radical_angle.clone().sin(),
            (-root_five / Real::from(3)).unwrap()
        );
        assert_eq!(
            radical_angle.cos(),
            (Real::from(2) / Real::from(3)).unwrap()
        );

        let displaced_height = Real::from(-3) - Real::from(5).sqrt().unwrap() + Real::from(3);
        let latitude = (displaced_height.clone() / Real::from(3))
            .unwrap()
            .asin()
            .unwrap();
        assert_eq!(
            Real::from(3) * latitude.sin(),
            -Real::from(5).sqrt().unwrap()
        );

        let sixth = Real::new(Rational::fraction(1, 6).unwrap());
        let shifted_acos = (-sixth.clone()).acos().unwrap() - pi_fraction(1, 2);
        assert!(
            shifted_acos
                .fold_ref()
                .signed_acos_minus_half_pi_argument()
                .is_some()
        );
        let shifted_sine = shifted_acos.clone().sin();
        assert_eq!(shifted_sine.exact_rational(), sixth.exact_rational());
        let shifted_cosine = shifted_acos.cos();
        assert_eq!(
            shifted_cosine,
            (Real::from(35).sqrt().unwrap() / Real::from(6)).unwrap()
        );
        let reflected_shift = pi_fraction(1, 2) - (-sixth).acos().unwrap();
        assert_eq!(
            reflected_shift.sin().exact_rational(),
            Some(-Rational::fraction(1, 6).unwrap())
        );
        let anchored_slope = (-Real::from(35).sqrt().unwrap() / Real::from(35)).unwrap();
        let anchored_angle = anchored_slope.atan().unwrap();
        assert_eq!(
            anchored_angle
                .fold_ref()
                .asin_argument()
                .and_then(|argument| argument.exact_rational()),
            Some(-Rational::fraction(1, 6).unwrap())
        );
        assert_eq!(
            anchored_angle.sin().exact_rational(),
            Some(-Rational::fraction(1, 6).unwrap())
        );

        let half = Real::new(Rational::fraction(1, 2).unwrap());
        let sixth_turn = half.clone().asin().unwrap();
        assert_eq!(
            sixth_turn.fold_ref().pi_rational_multiple(),
            Some(Rational::fraction(1, 6).unwrap())
        );
        assert_eq!(sixth_turn.sin(), half);
    }

    #[test]
    fn scaled_acos_trig_composition_remains_bounded() {
        let phase = Real::new(Rational::fraction(-14, 31).unwrap())
            .acos()
            .unwrap();
        let carrier =
            (phase.clone() * Real::new(Rational::fraction(4, 81).unwrap())) + pi_fraction(1, 18);
        let rolling = (phase * Real::new(Rational::fraction(31, 81).unwrap())) + pi_fraction(1, 18);

        assert!(carrier.clone().sin().to_f64_lossy().is_some());
        assert!(carrier.clone().cos().to_f64_lossy().is_some());
        assert!(rolling.clone().sin().to_f64_lossy().is_some());
        assert!(rolling.clone().cos().to_f64_lossy().is_some());

        let carrier_radius = Real::new(Rational::fraction(31, 2).unwrap());
        let generator = Real::from(2_i8);
        let x = carrier_radius.clone() * carrier.clone().cos()
            - generator.clone() * rolling.clone().cos();
        let y = carrier_radius * carrier.sin() - generator * rolling.sin();
        assert!(x.to_f64_lossy().is_some());
        assert!(y.to_f64_lossy().is_some());

        let phase = Real::new(Rational::fraction(1, 18).unwrap())
            .acos()
            .unwrap();
        let carrier =
            (phase.clone() * Real::new(Rational::fraction(1, 24).unwrap())) + pi_fraction(1, 32);
        let rolling = (phase * Real::new(Rational::fraction(3, 8).unwrap())) + pi_fraction(1, 32);
        let x = Real::from(9_i8) * carrier.clone().cos() - rolling.clone().cos();
        let y = Real::from(9_i8) * carrier.sin() - rolling.sin();
        assert!(x.to_f64_lossy().is_some());
        assert!(y.to_f64_lossy().is_some());

        let phase = Real::new(Rational::fraction(-71, 224).unwrap())
            .acos()
            .unwrap();
        let carrier =
            (phase.clone() * Real::new(Rational::fraction(1, 16).unwrap())) + pi_fraction(1, 32);
        let rolling = (phase * Real::new(Rational::fraction(7, 16).unwrap())) - pi_fraction(1, 32);
        let carrier_cos = carrier.clone().cos();
        let rolling_cos = rolling.clone().cos();
        let x = Real::from(7_i8) * carrier_cos + rolling_cos;
        let carrier_sin = carrier.sin();
        let rolling_sin = rolling.sin();
        let y = Real::from(7_i8) * carrier_sin - rolling_sin;
        assert!(x.to_f64_lossy().is_some());
        assert!(y.to_f64_lossy().is_some());

        // A dense exact cycloidal tip arc combines an acos phase with an
        // atan2-derived endpoint. Low-precision quadrant selection used to
        // bounce between public sin/cos constructors for one of these samples.
        let phase = Real::new(Rational::fraction(1, 18).unwrap())
            .acos()
            .unwrap();
        let tip_parameter = phase.clone() * Real::new(Rational::fraction(1, 8).unwrap());
        let tip_argument = (-phase.clone().sin()).atan2(Real::from(9_i8) - phase.cos());
        let right = -(tip_parameter + tip_argument + pi_fraction(1, 32));
        let left = -right.clone();
        for sample in 1..=32 {
            let u = Real::new(Rational::fraction(sample, 32).unwrap());
            let angle = right.clone() + u * (left.clone() - right.clone());
            assert!(angle.clone().sin().to_f64_lossy().is_some());
            assert!(angle.cos().to_f64_lossy().is_some());
        }
    }

    #[test]
    fn inverse_trig_domain_boundaries() {
        assert_eq!(
            Real::new(Rational::new(1)).asin().unwrap(),
            pi_fraction(1, 2)
        );
        assert_eq!(
            Real::new(Rational::new(-1)).asin().unwrap(),
            pi_fraction(-1, 2)
        );
        assert_eq!(Real::new(Rational::new(1)).acos().unwrap(), Real::zero());
        assert_eq!(Real::new(Rational::new(-1)).acos().unwrap(), Real::pi());

        for value in [
            Real::new(Rational::fraction(11, 10).unwrap()),
            Real::new(Rational::fraction(-11, 10).unwrap()),
            Real::new(Rational::new(2)).sqrt().unwrap(),
        ] {
            assert_eq!(value.clone().asin(), Err(Problem::NotANumber));
            assert_eq!(value.acos(), Err(Problem::NotANumber));
        }
    }

    #[test]
    fn inverse_hyperbolic_values() {
        assert_eq!(Real::zero().asinh().unwrap(), Real::zero());
        assert_eq!(Real::zero().atanh().unwrap(), Real::zero());
        assert_eq!(Real::new(Rational::new(1)).acosh().unwrap(), Real::zero());

        assert!(closest_f64(
            Real::new(Rational::fraction(3, 10).unwrap())
                .asinh()
                .unwrap(),
            0.29567304756342244
        ));
        assert!(closest_f64(
            Real::new(Rational::fraction(-1, 1_000_000_000_000).unwrap())
                .asinh()
                .unwrap(),
            -1.0e-12
        ));
        assert!(closest_f64(
            Real::new(Rational::new(2)).sqrt().unwrap().asinh().unwrap(),
            1.1462158347805889
        ));
        assert!(closest_f64(
            Real::new(Rational::new(2)).acosh().unwrap(),
            1.3169578969248168
        ));
        assert!(closest_f64(
            Real::new(Rational::new(2)).sqrt().unwrap().acosh().unwrap(),
            0.881373587019543
        ));
        assert!(closest_f64(
            Real::new(Rational::fraction(3, 10).unwrap())
                .atanh()
                .unwrap(),
            0.3095196042031117
        ));
    }

    #[test]
    fn inverse_hyperbolic_domain_boundaries() {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        let ln_three_over_two = Real::new(Rational::new(3)).ln().unwrap()
            * Real::new(Rational::fraction(1, 2).unwrap());

        assert_eq!(half.clone().atanh().unwrap(), ln_three_over_two);
        assert!(closest_f64(
            Real::new(Rational::fraction(-1, 2).unwrap())
                .atanh()
                .unwrap(),
            -0.5493061443340549
        ));
        assert!(closest_f64(
            Real::new(Rational::new(-2)).asinh().unwrap(),
            -1.4436354751788103
        ));

        for value in [Real::new(Rational::new(1)), Real::new(Rational::new(-1))] {
            assert_eq!(value.atanh(), Err(Problem::Infinity));
        }

        for value in [
            Real::new(Rational::fraction(11, 10).unwrap()),
            Real::new(Rational::fraction(-11, 10).unwrap()),
        ] {
            assert_eq!(value.atanh(), Err(Problem::NotANumber));
        }

        for value in [
            Real::zero(),
            Real::new(Rational::fraction(1, 2).unwrap()),
            Real::new(Rational::fraction(1, 2).unwrap())
                * Real::new(Rational::new(2)).sqrt().unwrap(),
            -Real::new(Rational::new(2)).sqrt().unwrap(),
            Real::new(Rational::new(-2)),
        ] {
            assert_eq!(value.acosh(), Err(Problem::NotANumber));
        }

        let sqrt_half = Real::new(Rational::fraction(1, 2).unwrap())
            * Real::new(Rational::new(2)).sqrt().unwrap();
        let asinh_one = Real::one().asinh().unwrap();
        let positive_diff: f64 = (sqrt_half.clone().atanh().unwrap() - asinh_one.clone()).into();
        let negative_diff: f64 = ((-sqrt_half.clone()).atanh().unwrap() + asinh_one).into();
        assert!(positive_diff.abs() < 1e-14);
        assert!(negative_diff.abs() < 1e-14);
        assert!(closest_f64(sqrt_half.atanh().unwrap(), 0.881373587019543));
        assert_eq!(
            Real::new(Rational::new(2)).sqrt().unwrap().atanh(),
            Err(Problem::NotANumber)
        );
        let sqrt_endpoint = Real::new(Rational::new(4)).sqrt().unwrap()
            * Real::new(Rational::fraction(1, 2).unwrap());
        assert_eq!(sqrt_endpoint.atanh(), Err(Problem::Infinity));
    }

    #[test]
    fn asinh_large_positive_does_not_panic() {
        let y = Real::from(1_000_000_i32).asinh();
        assert!(y.is_ok());
        let actual: f64 = y.unwrap().into();
        assert!((actual - 14.508657738524219).abs() < 1e-12);
    }

    #[test]
    fn asinh_large_negative_and_float_do_not_panic() {
        let negative = Real::from(-1_000_000_i32).asinh().unwrap();
        let actual: f64 = negative.into();
        assert!((actual + 14.508657738524219).abs() < 1e-12);

        let from_float = Real::try_from(1.0e6_f64).unwrap().asinh().unwrap();
        let actual: f64 = from_float.into();
        assert!((actual - 14.508657738524219).abs() < 1e-12);
    }

    #[test]
    fn sinh_of_zero_is_exact_zero() {
        assert_eq!(Real::zero().sinh().unwrap(), Real::zero());
    }

    #[test]
    fn cosh_of_zero_is_exact_one() {
        assert_eq!(Real::zero().cosh().unwrap(), Real::one());
    }

    #[test]
    fn sinh_rational_matches_f64() {
        let one = Real::one();
        let actual: f64 = one.sinh().unwrap().into();
        assert!((actual - 1.0_f64.sinh()).abs() < 1e-14);

        let two: f64 = Real::from(2_i32).sinh().unwrap().into();
        assert!((two - 2.0_f64.sinh()).abs() < 1e-13);
    }

    #[test]
    fn cosh_rational_matches_f64() {
        let one = Real::one();
        let actual: f64 = one.cosh().unwrap().into();
        assert!((actual - 1.0_f64.cosh()).abs() < 1e-14);

        let two: f64 = Real::from(2_i32).cosh().unwrap().into();
        assert!((two - 2.0_f64.cosh()).abs() < 1e-13);
    }

    #[test]
    fn sinh_is_odd_symmetry() {
        let x = Real::new(Rational::fraction(3, 4).unwrap());
        let lhs = x.clone().sinh().unwrap();
        let rhs = (-x).sinh().unwrap();
        let lhs_f64: f64 = lhs.into();
        let rhs_f64: f64 = rhs.into();
        assert!((lhs_f64 + rhs_f64).abs() < 1e-14);
    }

    #[test]
    fn cosh_is_even_symmetry() {
        let x = Real::new(Rational::fraction(3, 4).unwrap());
        let lhs: f64 = x.clone().cosh().unwrap().into();
        let rhs: f64 = (-x).cosh().unwrap().into();
        assert!((lhs - rhs).abs() < 1e-14);
    }

    #[test]
    fn sinh_of_integer_ln_is_exact_rational() {
        // sinh(ln(2)) = (2 - 1/2)/2 = 3/4
        let value = Real::from(2_i32).ln().unwrap().sinh().unwrap();
        assert_eq!(value, Real::new(Rational::fraction(3, 4).unwrap()));

        // sinh(2*ln(3)) = (9 - 1/9)/2 = 40/9
        let value = (Real::from(2_i32) * Real::from(3_i32).ln().unwrap())
            .sinh()
            .unwrap();
        assert_eq!(value, Real::new(Rational::fraction(40, 9).unwrap()));
    }

    #[test]
    fn cosh_of_integer_ln_is_exact_rational() {
        // cosh(ln(2)) = (2 + 1/2)/2 = 5/4
        let value = Real::from(2_i32).ln().unwrap().cosh().unwrap();
        assert_eq!(value, Real::new(Rational::fraction(5, 4).unwrap()));

        // cosh(2*ln(3)) = (9 + 1/9)/2 = 41/9
        let value = (Real::from(2_i32) * Real::from(3_i32).ln().unwrap())
            .cosh()
            .unwrap();
        assert_eq!(value, Real::new(Rational::fraction(41, 9).unwrap()));
    }

    #[test]
    fn cosh_squared_minus_sinh_squared_is_one() {
        let x = Real::new(Rational::fraction(7, 5).unwrap());
        let s = x.clone().sinh().unwrap();
        let c = x.cosh().unwrap();
        let identity = c.clone() * c - s.clone() * s;
        let actual: f64 = identity.into();
        assert!((actual - 1.0).abs() < 1e-12);
    }

    #[test]
    fn sinh_of_irrational_argument_matches_f64() {
        // sinh(sqrt(2)) — generic identity path with irrational argument.
        let sqrt_two = Real::from(2_i32).sqrt().unwrap();
        let value: f64 = sqrt_two.sinh().unwrap().into();
        let expected = 2.0_f64.sqrt().sinh();
        assert!((value - expected).abs() < 1e-12);
    }

    #[test]
    fn cosh_of_irrational_argument_matches_f64() {
        let sqrt_two = Real::from(2_i32).sqrt().unwrap();
        let value: f64 = sqrt_two.cosh().unwrap().into();
        let expected = 2.0_f64.sqrt().cosh();
        assert!((value - expected).abs() < 1e-12);
    }

    #[test]
    fn tanh_of_zero_is_exact_zero() {
        assert_eq!(Real::zero().tanh().unwrap(), Real::zero());
    }

    #[test]
    fn tanh_rational_matches_f64() {
        let value: f64 = Real::one().tanh().unwrap().into();
        assert!((value - 1.0_f64.tanh()).abs() < 1e-14);

        let value: f64 = Real::from(2_i32).tanh().unwrap().into();
        assert!((value - 2.0_f64.tanh()).abs() < 1e-13);
    }

    #[test]
    fn forward_hyperbolics_stay_stable_for_tiny_and_large_rationals() {
        for value in [
            Rational::fraction(1, 1_000_000_000_000_u64).unwrap(),
            Rational::fraction(-1, 1_000_000_000_000_u64).unwrap(),
            Rational::new(20),
            Rational::new(-20),
        ] {
            let input = Real::new(value);
            let primitive = f64::from(input.clone());
            for (actual, expected) in [
                (input.clone().sinh().unwrap(), primitive.sinh()),
                (input.clone().cosh().unwrap(), primitive.cosh()),
                (input.clone().tanh().unwrap(), primitive.tanh()),
            ] {
                let actual: f64 = actual.into();
                let scale = expected.abs().max(1.0);
                assert!((actual - expected).abs() <= 1e-14 * scale);
            }
        }
    }

    #[test]
    fn tanh_is_odd_symmetry() {
        let x = Real::new(Rational::fraction(3, 4).unwrap());
        let lhs: f64 = x.clone().tanh().unwrap().into();
        let rhs: f64 = (-x).tanh().unwrap().into();
        assert!((lhs + rhs).abs() < 1e-14);
    }

    #[test]
    fn tanh_of_integer_ln_is_exact_rational() {
        // tanh(ln(2)) = (4 - 1)/(4 + 1) = 3/5
        let value = Real::from(2_i32).ln().unwrap().tanh().unwrap();
        assert_eq!(value, Real::new(Rational::fraction(3, 5).unwrap()));

        // tanh(2*ln(3)) = (81 - 1)/(81 + 1) = 80/82 = 40/41
        let value = (Real::from(2_i32) * Real::from(3_i32).ln().unwrap())
            .tanh()
            .unwrap();
        assert_eq!(value, Real::new(Rational::fraction(40, 41).unwrap()));
    }

    #[test]
    fn tanh_matches_sinh_over_cosh() {
        let x = Real::new(Rational::fraction(7, 5).unwrap());
        let direct: f64 = x.clone().tanh().unwrap().into();
        let via_identity: f64 = (x.clone().sinh().unwrap() / x.cosh().unwrap())
            .unwrap()
            .into();
        assert!((direct - via_identity).abs() < 1e-13);
    }

    #[test]
    fn tanh_of_irrational_argument_matches_f64() {
        let sqrt_two = Real::from(2_i32).sqrt().unwrap();
        let value: f64 = sqrt_two.tanh().unwrap().into();
        let expected = 2.0_f64.sqrt().tanh();
        assert!((value - expected).abs() < 1e-12);
    }

    #[test]
    fn log2_of_powers_of_two_is_exact_integer() {
        for k in 0_i64..=20 {
            let n = Real::new(Rational::new(1_i64 << k));
            let answer = n.log2().unwrap();
            assert_eq!(answer, Rational::new(k));
        }
    }

    #[test]
    fn log2_of_one_is_zero() {
        assert_eq!(Real::one().log2().unwrap(), Real::zero());
    }

    #[test]
    fn log2_of_one_half_is_negative_one() {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        assert_eq!(half.log2().unwrap(), Rational::new(-1));
    }

    #[test]
    fn log2_of_inverse_power_of_two_is_negative_integer() {
        for k in 1_i64..=12 {
            let n = Real::new(Rational::fraction(1, 1_u64 << k).unwrap());
            let answer = n.log2().unwrap();
            assert_eq!(answer, Rational::new(-k));
        }
    }

    #[test]
    fn log2_of_rational_matches_f64() {
        for &n in &[3_i64, 5, 7, 9, 11, 13, 17] {
            let value: f64 = Real::new(Rational::new(n)).log2().unwrap().into();
            let expected = (n as f64).log2();
            assert!(
                (value - expected).abs() < 1e-12,
                "log2({n}) = {value}, expected {expected}"
            );
        }
    }

    #[test]
    fn log2_of_fractional_non_power_rational_matches_f64() {
        for (numerator, denominator) in [(3_i64, 8_u64), (5, 12), (17, 1024)] {
            let value = Real::new(Rational::fraction(numerator, denominator).unwrap())
                .log2()
                .unwrap();
            assert_close(
                value,
                ((numerator as f64) / (denominator as f64)).log2(),
                1e-12,
            );
        }
    }

    #[test]
    fn log2_of_negative_errors() {
        let negative = Real::new(Rational::new(-3));
        assert_eq!(negative.log2(), Err(Problem::NotANumber));
    }

    #[test]
    fn log2_of_zero_errors() {
        assert_eq!(Real::zero().log2(), Err(Problem::NotANumber));
    }

    #[test]
    fn log2_matches_ln_div_ln2() {
        let x = Real::new(Rational::new(7));
        let direct = x.clone().log2().unwrap();
        let via_quotient = (x.ln().unwrap() / Real::new(Rational::new(2)).ln().unwrap()).unwrap();
        let difference: f64 = (direct - via_quotient).into();
        assert!(difference.abs() < 1e-14);
    }

    #[test]
    fn log2_of_sqrt_two_is_half() {
        let sqrt_two = Real::from(2_i32).sqrt().unwrap();
        let value: f64 = sqrt_two.log2().unwrap().into();
        assert!((value - 0.5).abs() < 1e-12);
    }

    #[test]
    fn log2_of_irrational_argument_matches_f64() {
        let value = Real::from(2_i32) + Real::from(3_i32).sqrt().unwrap();
        let actual: f64 = value.log2().unwrap().into();
        let expected = (2.0_f64 + 3.0_f64.sqrt()).log2();
        assert!((actual - expected).abs() < 1e-12);
    }

    #[test]
    fn log2_ln_quotient_folds_to_log2_class() {
        let numerator = Real::new(Rational::new(5)).ln().unwrap();
        let denominator = Real::new(Rational::new(2)).ln().unwrap();
        let quotient = (numerator / denominator).unwrap();
        let expected = Real::new(Rational::new(5)).log2().unwrap();
        assert_eq!(quotient, expected);
    }

    #[test]
    fn log2_ln_quotient_preserves_exact_scaled_logs() {
        let numerator = Real::new(Rational::new(9)).ln().unwrap();
        let denominator = Real::new(Rational::new(4)).ln().unwrap();
        let quotient = (numerator / denominator).unwrap();
        let expected = Real::new(Rational::new(3)).log2().unwrap();
        assert_eq!(quotient, expected);

        let numerator = Real::new(Rational::new(32)).ln().unwrap();
        let denominator = Real::new(Rational::fraction(1, 2).unwrap()).ln().unwrap();
        assert_eq!((numerator / denominator).unwrap(), Rational::new(-5));
    }

    #[test]
    fn log2_ln_quotient_ignores_warmed_numerator_cache() {
        let numerator = Real::new(Rational::new(5)).ln().unwrap();
        let warmed = numerator.to_f64_lossy().unwrap();
        assert!((warmed - 5.0_f64.ln()).abs() < 1e-12);

        let denominator = Real::new(Rational::new(2)).ln().unwrap();
        let quotient = (numerator / denominator).unwrap();
        assert_close(quotient, 5.0_f64.log2(), 1e-12);
    }

    fn assert_close(value: Real, expected: f64, tolerance: f64) {
        let actual: f64 = value.into();
        let scale = expected.abs().max(1.0);
        assert!(
            (actual - expected).abs() <= tolerance * scale,
            "actual {actual}, expected {expected}, tolerance {tolerance}"
        );
    }

    fn normal_case_real(num: &str, den: &str) -> Real {
        let n: num::BigInt = num.parse().unwrap();
        let d: num::BigUint = den.parse().unwrap();
        Real::new(Rational::from_bigint_fraction(n, d).unwrap())
    }

    fn trunc_str(real: &Real, n: usize) -> String {
        let neg = real.best_sign() == num::bigint::Sign::Minus;
        let c = real.fold_ref();
        let bits = -((n as i32) * 3322 / 1000 + 64);
        let appr = c.approx(bits).magnitude().clone();
        let ten_n: num::BigInt = num::pow::Pow::pow(num::BigInt::from(10), n as u32);
        let scaled = (num::BigInt::from(appr) * ten_n) >> ((-bits) as usize);
        let mut s = scaled.to_string();
        if s.len() <= n {
            s = format!("{}{}", "0".repeat(n - s.len() + 1), s);
        }
        let (int_part, frac_part) = s.split_at(s.len() - n);
        format!("{}{}.{}", if neg { "-" } else { "" }, int_part, frac_part)
    }

    #[test]
    fn stable_substrate_functions() {
        assert!(Real::zero().ln_1p().unwrap().definitely_zero());
        assert!(Real::zero().log1p().unwrap().definitely_zero());
        assert!(Real::zero().ln_1m().unwrap().definitely_zero());
        assert!(Real::zero().log1m().unwrap().definitely_zero());
        assert!(Real::zero().expm1().definitely_zero());
        assert_eq!(
            Real::zero().sigmoid().unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );
        assert!(
            Real::new(Rational::fraction(1, 2).unwrap())
                .logit()
                .unwrap()
                .definitely_zero()
        );

        let tiny = Real::new(Rational::fraction(1, 1_000_000).unwrap());
        assert_close(tiny.clone().ln_1p().unwrap(), 0.000001_f64.ln_1p(), 1e-18);
        assert_close(
            tiny.clone().ln_1m().unwrap(),
            (-0.000001_f64).ln_1p(),
            1e-18,
        );
        assert_close(tiny.clone().expm1(), 0.000001_f64.exp_m1(), 1e-18);
        assert_close(
            Real::from(2_i32).sigmoid().unwrap(),
            1.0 / (1.0 + (-2.0_f64).exp()),
            1e-14,
        );
        assert_close(
            Real::from(2_i32).softplus().unwrap(),
            (1.0 + 2.0_f64.exp()).ln(),
            1e-14,
        );
        assert_eq!(
            Real::from(2_i32).ln().unwrap().softplus().unwrap(),
            Real::from(3_i32).ln().unwrap()
        );
        assert_eq!(
            Real::from(3_i32).ln().unwrap().sigmoid().unwrap(),
            Real::new(Rational::fraction(3, 4).unwrap())
        );
        assert_eq!(Real::from(2_i32).ln().unwrap().expm1(), Real::one());
        assert_eq!(
            Real::logaddexp(&Real::zero(), &Real::zero()).unwrap(),
            Real::from(2_i32).ln().unwrap()
        );
        assert_eq!(
            Real::logaddexp(
                &Real::from(2_i32).ln().unwrap(),
                &Real::from(3_i32).ln().unwrap()
            )
            .unwrap(),
            Real::from(5_i32).ln().unwrap()
        );
        assert_close(
            Real::logsubexp(&Real::from(2_i32).ln().unwrap(), &Real::zero()).unwrap(),
            0.0,
            1e-14,
        );
        assert_close(
            Real::logaddexp(&Real::from(2_i32), &Real::zero()).unwrap(),
            (2.0_f64.exp() + 1.0).ln(),
            1e-14,
        );
        assert_close(
            Real::logsubexp(&Real::from(2_i32), &Real::zero()).unwrap(),
            (2.0_f64.exp() - 1.0).ln(),
            1e-14,
        );

        assert_eq!(Real::from(-1_i32).ln_1p(), Err(Problem::NotANumber));
        assert_eq!(Real::one().ln_1m(), Err(Problem::NotANumber));
        assert_eq!(Real::zero().logit(), Err(Problem::NotANumber));
        assert_eq!(Real::one().logit(), Err(Problem::NotANumber));
        assert_eq!(
            Real::logsubexp(&Real::zero(), &Real::zero()),
            Err(Problem::NotANumber)
        );
        assert_eq!(
            Real::logsubexp(&Real::zero(), &Real::one()),
            Err(Problem::NotANumber)
        );

        assert!(Real::zero().sqrt1pm1().unwrap().definitely_zero());
        assert!(Real::zero().sqrt1m1().unwrap().definitely_zero());
        assert_eq!(Real::from(-1_i32).sqrt1pm1().unwrap(), Real::from(-1_i32));
        assert_eq!(Real::one().sqrt1m1().unwrap(), Real::from(-1_i32));
        assert_close(
            tiny.clone().sqrt1pm1().unwrap(),
            (1.0 + 0.000001_f64).sqrt() - 1.0,
            1e-16,
        );
        assert_close(
            tiny.sqrt1m1().unwrap(),
            (1.0 - 0.000001_f64).sqrt() - 1.0,
            1e-16,
        );
        assert_eq!(Real::from(-2_i32).sqrt1pm1(), Err(Problem::SqrtNegative));
        assert_eq!(Real::from(2_i32).sqrt1m1(), Err(Problem::SqrtNegative));
    }

    #[test]
    fn normal_exact_cases() {
        assert!(Real::zero().erf().definitely_zero());
        assert_eq!(Real::zero().erfc(), Real::one());
        assert_eq!(Real::zero().erfcx().unwrap(), Real::one());
        assert_eq!(
            Real::zero().pnorm().unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );
        assert_eq!(
            Real::zero().normal_sf().unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );
        assert!(
            Real::normal_interval(&Real::one(), &Real::one())
                .unwrap()
                .definitely_zero()
        );
        assert!(
            Real::new(Rational::fraction(1, 2).unwrap())
                .qnorm()
                .unwrap()
                .definitely_zero()
        );
        assert!(Real::zero().erfinv().unwrap().definitely_zero());
        assert!(Real::one().erfcinv().unwrap().definitely_zero());
        assert!(
            Real::new(Rational::fraction(1, 2).unwrap())
                .qnorm_upper()
                .unwrap()
                .definitely_zero()
        );
        assert_eq!(
            Real::from(2_i32)
                .normal_cdf(&Real::from(2_i32), &Real::from(3_i32))
                .unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );
        assert_eq!(
            Real::from(2_i32)
                .normal_survival(&Real::from(2_i32), &Real::from(3_i32))
                .unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );
        assert_eq!(
            Real::new(Rational::fraction(1, 2).unwrap())
                .normal_quantile(&Real::from(2_i32), &Real::from(3_i32))
                .unwrap(),
            Real::from(2_i32)
        );
    }

    #[test]
    fn normal_known_values() {
        assert_close(Real::one().erf(), 0.8427007929497149, 1e-15);
        assert_close(Real::one().erfc(), 0.15729920705028513, 1e-15);
        assert_close(Real::one().erfcx().unwrap(), 0.427583576155807, 1e-15);
        assert_close(Real::from(-1_i32).erf(), -0.8427007929497149, 1e-15);
        assert_close(Real::zero().dnorm().unwrap(), 0.3989422804014327, 1e-15);
        assert_close(Real::one().dnorm().unwrap(), 0.24197072451914337, 1e-15);
        assert_close(Real::one().pnorm().unwrap(), 0.8413447460685429, 1e-15);
        assert_close(Real::one().normal_sf().unwrap(), 0.15865525393145707, 1e-15);
        assert_close(
            Real::one().pnorm_upper().unwrap(),
            0.15865525393145707,
            1e-15,
        );
        assert_close(
            Real::normal_interval(&Real::zero(), &Real::one()).unwrap(),
            0.3413447460685429,
            1e-15,
        );
        assert_close(
            Real::pnorm_diff(&Real::zero(), &Real::one()).unwrap(),
            0.3413447460685429,
            1e-15,
        );
        assert_close(
            Real::zero().log_pnorm().unwrap(),
            -std::f64::consts::LN_2,
            1e-15,
        );
        assert_close(
            Real::zero().log_normal_sf().unwrap(),
            -std::f64::consts::LN_2,
            1e-15,
        );
        assert_close(
            Real::zero().log_dnorm().unwrap(),
            -0.9189385332046727,
            1e-15,
        );
        assert_close(
            Real::from(2_i32).log_dnorm().unwrap(),
            -2.9189385332046727,
            1e-15,
        );
        assert_close(
            Real::new(Rational::fraction(975, 1000).unwrap())
                .qnorm()
                .unwrap(),
            1.959963984540054,
            1e-14,
        );
        assert_close(Real::one().erf().erfinv().unwrap(), 1.0, 1e-12);
        assert_close(Real::one().erfc().erfcinv().unwrap(), 1.0, 1e-12);
        assert_close(
            Real::new(Rational::fraction(25, 1000).unwrap())
                .qnorm_upper()
                .unwrap(),
            1.959963984540054,
            1e-14,
        );
        assert_close(
            Real::from(5_i32)
                .normal_pdf(&Real::from(2_i32), &Real::from(3_i32))
                .unwrap(),
            0.08065690817304778,
            1e-15,
        );
        assert_close(
            Real::from(5_i32)
                .normal_cdf(&Real::from(2_i32), &Real::from(3_i32))
                .unwrap(),
            0.8413447460685429,
            1e-15,
        );
        assert_close(
            Real::from(5_i32)
                .normal_survival(&Real::from(2_i32), &Real::from(3_i32))
                .unwrap(),
            0.15865525393145707,
            1e-15,
        );
        assert_close(
            Real::new(Rational::fraction(975, 1000).unwrap())
                .normal_quantile(&Real::from(2_i32), &Real::from(3_i32))
                .unwrap(),
            7.879891953620163,
            1e-14,
        );
        assert_close(
            Real::zero().normal_mills().unwrap(),
            1.2533141373155001,
            1e-15,
        );
        assert_close(
            Real::zero().normal_hazard().unwrap(),
            0.7978845608028654,
            1e-15,
        );
        assert_close(
            Real::zero().normal_log_hazard().unwrap(),
            -0.22579135264472738,
            1e-15,
        );
        assert_close(
            Real::zero().normal_inverse_mills().unwrap(),
            0.7978845608028654,
            1e-15,
        );
        assert_close(
            Real::one().normal_mills().unwrap(),
            0.6556795424187986,
            1e-15,
        );
        assert_close(
            Real::one().normal_hazard().unwrap(),
            1.525135276160981,
            1e-15,
        );
        assert_close(
            Real::one().normal_log_hazard().unwrap(),
            0.4220831118045907,
            1e-15,
        );
        assert_close(
            Real::one().normal_inverse_mills().unwrap(),
            0.2875999709391784,
            1e-15,
        );
        assert_eq!(
            Real::hermite_probabilists(0, &Real::from(2_i32)),
            Real::one()
        );
        assert_eq!(
            Real::hermite_probabilists(1, &Real::from(2_i32)),
            Real::from(2_i32)
        );
        assert_eq!(
            Real::hermite_probabilists(2, &Real::from(2_i32)),
            Real::from(3_i32)
        );
        assert_eq!(
            Real::hermite_probabilists(3, &Real::from(2_i32)),
            Real::from(2_i32)
        );
        assert_close(
            Real::one().dnorm_derivative(1).unwrap(),
            -0.24197072451914337,
            1e-15,
        );
        assert_close(Real::one().dnorm_derivative(2).unwrap(), 0.0, 1e-15);
        assert_close(
            Real::one().gaussian_derivative(3).unwrap(),
            0.48394144903828673,
            1e-15,
        );
        assert_eq!(Real::standard_normal_moment(0), Real::one());
        assert!(Real::standard_normal_moment(1).definitely_zero());
        assert_eq!(Real::standard_normal_moment(2), Real::one());
        assert_eq!(Real::standard_normal_moment(4), Real::from(3_i32));
        assert_eq!(Real::standard_normal_moment(6), Real::from(15_i32));
        assert_close(
            Real::normal_interval_moment(&Real::zero(), &Real::one(), 0).unwrap(),
            0.3413447460685429,
            1e-15,
        );
        assert_close(
            Real::normal_interval_moment(&Real::zero(), &Real::one(), 1).unwrap(),
            0.15697155588228934,
            1e-15,
        );
        assert_close(
            Real::normal_interval_moment(&Real::zero(), &Real::one(), 2).unwrap(),
            0.09937402154939956,
            1e-15,
        );
        assert_eq!(
            format!(
                "{:#}",
                Real::truncated_normal_mean(&Real::zero(), &Real::one()).unwrap()
            ),
            "0.45986222928642650033302670255646"
        );
        assert_eq!(
            format!(
                "{:#}",
                Real::truncated_normal_variance(&Real::zero(), &Real::one()).unwrap()
            ),
            "0.07965182484851131233334055314679"
        );
        assert_eq!(Real::from(5_i32).gamma().unwrap(), Real::from(24_i32));
        assert_close(
            Real::new(Rational::fraction(1, 2).unwrap())
                .gamma()
                .unwrap(),
            std::f64::consts::PI.sqrt(),
            1e-15,
        );
        assert_close(
            Real::new(Rational::fraction(-1, 2).unwrap())
                .gamma()
                .unwrap(),
            -2.0 * std::f64::consts::PI.sqrt(),
            1e-15,
        );
        assert_eq!(
            Real::beta(&Real::from(2_i32), &Real::from(3_i32)).unwrap(),
            Real::new(Rational::fraction(1, 12).unwrap())
        );
        assert_close(
            Real::beta(
                &Real::new(Rational::fraction(1, 2).unwrap()),
                &Real::new(Rational::fraction(1, 2).unwrap()),
            )
            .unwrap(),
            std::f64::consts::PI,
            1e-15,
        );
        assert_close(
            Real::ln_beta(&Real::from(2_i32), &Real::from(3_i32)).unwrap(),
            (1.0_f64 / 12.0).ln(),
            1e-15,
        );
        assert_eq!(
            Real::regularized_beta(
                &Real::from(2_i32),
                &Real::from(3_i32),
                &Real::new(Rational::fraction(1, 2).unwrap())
            )
            .unwrap(),
            Real::new(Rational::fraction(11, 16).unwrap())
        );
        assert_eq!(
            Real::regularized_beta(
                &Real::one(),
                &Real::from(3_i32),
                &Real::new(Rational::fraction(1, 2).unwrap())
            )
            .unwrap(),
            Real::new(Rational::fraction(7, 8).unwrap())
        );
        assert_eq!(
            Real::regularized_beta_q(
                &Real::from(2_i32),
                &Real::from(3_i32),
                &Real::new(Rational::fraction(1, 2).unwrap())
            )
            .unwrap(),
            Real::new(Rational::fraction(5, 16).unwrap())
        );
        assert_eq!(
            Real::regularized_beta_q(
                &Real::one(),
                &Real::from(3_i32),
                &Real::new(Rational::fraction(1, 2).unwrap())
            )
            .unwrap(),
            Real::new(Rational::fraction(1, 8).unwrap())
        );
        assert_close(
            Real::regularized_gamma_p(&Real::new(Rational::fraction(3, 2).unwrap()), &Real::one())
                .unwrap(),
            0.4275932955291202,
            1e-15,
        );
        assert_close(
            Real::regularized_gamma_q(&Real::new(Rational::fraction(3, 2).unwrap()), &Real::one())
                .unwrap(),
            0.5724067044708798,
            1e-15,
        );
        assert_close(
            Real::regularized_gamma_p(&Real::from(2_i32), &Real::from(3_i32)).unwrap(),
            0.8008517265285442,
            1e-15,
        );
        assert_close(
            Real::regularized_gamma_q(&Real::from(2_i32), &Real::from(3_i32)).unwrap(),
            0.19914827347145578,
            1e-15,
        );
        assert_close(
            Real::chi_square_cdf(&Real::from(2_i32), 2).unwrap(),
            0.6321205588285577,
            1e-15,
        );
        assert_close(
            Real::chi_square_sf(&Real::one(), 1).unwrap(),
            0.31731050786291404,
            1e-15,
        );
    }

    #[test]
    fn balanced_gamma_and_beta_products_match_sequential_closed_forms() {
        fn sequential_factorial(n: u64) -> num::BigUint {
            let mut result = num::BigUint::from(1_u8);
            for factor in 2..=n {
                result *= num::BigUint::from(factor);
            }
            result
        }

        fn exact_ratio(numerator: num::BigUint, denominator: num::BigUint, negative: bool) -> Real {
            Real::new(
                Rational::from_bigint_fraction(
                    num::BigInt::from_biguint(
                        if negative {
                            num::bigint::Sign::Minus
                        } else {
                            num::bigint::Sign::Plus
                        },
                        numerator,
                    ),
                    denominator,
                )
                .unwrap(),
            )
        }

        fn reference_gamma(twice: i64) -> Real {
            if twice > 0 && twice % 2 == 0 {
                return exact_ratio(
                    sequential_factorial((twice / 2) as u64 - 1),
                    num::BigUint::from(1_u8),
                    false,
                );
            }

            let sqrt_pi = Real::pi().sqrt().unwrap();
            if twice > 0 {
                let k = ((twice - 1) / 2) as u64;
                return exact_ratio(
                    sequential_factorial(2 * k),
                    (num::BigUint::from(1_u8) << (2 * k)) * sequential_factorial(k),
                    false,
                ) * sqrt_pi;
            }

            let m = ((1 - twice) / 2) as u64;
            exact_ratio(
                (num::BigUint::from(1_u8) << (2 * m)) * sequential_factorial(m),
                sequential_factorial(2 * m),
                m % 2 == 1,
            ) * sqrt_pi
        }

        for n in [
            0_u64, 1, 2, 19, 20, 21, 31, 32, 511, 512, 513, 1_000, 10_000,
        ] {
            assert_eq!(
                Real::from((n + 1) as i64).gamma().unwrap(),
                exact_ratio(sequential_factorial(n), num::BigUint::from(1_u8), false,)
            );
        }

        for twice in (-199_i64..=201).filter(|twice| *twice > 0 || twice % 2 != 0) {
            assert_eq!(
                Real::new(Rational::fraction(twice, 2).unwrap())
                    .gamma()
                    .unwrap(),
                reference_gamma(twice),
                "twice the gamma argument was {twice}"
            );
        }

        for a in [1_u64, 2, 3, 7, 20, 65] {
            for b in [1_u64, 2, 3, 11, 32, 67] {
                let expected = exact_ratio(
                    sequential_factorial(a - 1) * sequential_factorial(b - 1),
                    sequential_factorial(a + b - 1),
                    false,
                );
                assert_eq!(
                    Real::beta(&Real::from(a as i64), &Real::from(b as i64)).unwrap(),
                    expected,
                    "beta arguments were ({a}, {b})"
                );
            }
        }
    }

    #[test]
    fn normal_round_trips_and_symmetry() {
        for x in [
            Real::from(2_i32),
            Real::from(-1_i32),
            Real::new(Rational::fraction(3, 2).unwrap()),
        ] {
            let p = x.clone().pnorm().unwrap();
            let round_trip = p.qnorm().unwrap();
            assert_close(round_trip, x.clone().into(), 1e-12);

            let symmetry = x.clone().pnorm().unwrap() + (-x.clone()).pnorm().unwrap();
            assert_close(symmetry, 1.0, 1e-12);

            let complement = x.clone().pnorm().unwrap() + x.normal_sf().unwrap();
            assert_close(complement, 1.0, 1e-12);
        }
    }

    #[test]
    fn normal_domain_errors() {
        assert_eq!(Real::zero().qnorm().unwrap_err(), Problem::NotANumber);
        assert_eq!(Real::one().qnorm().unwrap_err(), Problem::NotANumber);
        assert_eq!(Real::from(2_i32).qnorm().unwrap_err(), Problem::NotANumber);
        assert_eq!(Real::from(-1_i32).qnorm().unwrap_err(), Problem::NotANumber);
        assert_eq!(Real::one().erfinv().unwrap_err(), Problem::NotANumber);
        assert_eq!(
            Real::from(-1_i32).erfinv().unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(Real::from(2_i32).erfinv().unwrap_err(), Problem::NotANumber);
        assert_eq!(Real::zero().erfcinv().unwrap_err(), Problem::NotANumber);
        assert_eq!(
            Real::from(2_i32).erfcinv().unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::from(-1_i32).erfcinv().unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::from(3_i32).erfcinv().unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(Real::zero().qnorm_upper().unwrap_err(), Problem::NotANumber);
        assert_eq!(Real::one().qnorm_upper().unwrap_err(), Problem::NotANumber);
        assert_eq!(
            Real::from(-1_i32).qnorm_upper().unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::from(2_i32).qnorm_upper().unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::from(5_i32)
                .normal_pdf(&Real::zero(), &Real::zero())
                .unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::from(5_i32)
                .normal_cdf(&Real::zero(), &Real::from(-1_i32))
                .unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::from(5_i32)
                .normal_survival(&Real::zero(), &Real::from(-1_i32))
                .unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::new(Rational::fraction(1, 2).unwrap())
                .normal_quantile(&Real::zero(), &Real::zero())
                .unwrap_err(),
            Problem::NotANumber
        );

        assert_eq!(Real::from(11_i32).pnorm().unwrap_err(), Problem::Exhausted);
        assert_eq!(
            Real::from(11_i32).normal_sf().unwrap_err(),
            Problem::Exhausted
        );
        assert_eq!(
            Real::normal_interval(&Real::from(2_i32), &Real::from(1_i32)).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::normal_interval(&Real::from(-11_i32), &Real::zero()).unwrap_err(),
            Problem::Exhausted
        );
        assert_eq!(
            Real::from(11_i32).log_pnorm().unwrap_err(),
            Problem::Exhausted
        );
        assert_eq!(
            Real::from(11_i32).log_normal_sf().unwrap_err(),
            Problem::Exhausted
        );
        assert_eq!(
            Real::from(11_i32).normal_log_hazard().unwrap_err(),
            Problem::Exhausted
        );
        assert_eq!(
            Real::from(11_i32).normal_inverse_mills().unwrap_err(),
            Problem::Exhausted
        );
        assert_eq!(
            Real::from(11_i32).dnorm_derivative(1).unwrap_err(),
            Problem::Exhausted
        );
        assert_eq!(
            Real::from(11_i32).gaussian_derivative(1).unwrap_err(),
            Problem::Exhausted
        );
        assert_eq!(
            Real::normal_interval_moment(&Real::from(2_i32), &Real::from(1_i32), 1).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::normal_interval_moment(&Real::from(-11_i32), &Real::zero(), 1).unwrap_err(),
            Problem::Exhausted
        );
        assert_eq!(
            Real::truncated_normal_mean(&Real::one(), &Real::one()).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::truncated_normal_variance(&Real::from(2_i32), &Real::from(1_i32)).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(Real::zero().gamma().unwrap_err(), Problem::NotANumber);
        assert_eq!(
            Real::new(Rational::fraction(1, 3).unwrap())
                .gamma()
                .unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::from(-2_i32).lgamma().unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::beta(&Real::zero(), &Real::one()).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::beta(&Real::new(Rational::fraction(1, 3).unwrap()), &Real::one()).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::regularized_beta(&Real::zero(), &Real::one(), &Real::one()).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::regularized_beta(
                &Real::one(),
                &Real::new(Rational::fraction(1, 3).unwrap()),
                &Real::one()
            )
            .unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::regularized_beta(&Real::one(), &Real::one(), &Real::from(-1_i32)).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::regularized_beta_q(&Real::one(), &Real::one(), &Real::from(2_i32)).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::regularized_gamma_p(&Real::zero(), &Real::one()).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::regularized_gamma_p(&Real::new(Rational::fraction(1, 3).unwrap()), &Real::one())
                .unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::regularized_gamma_q(&Real::one(), &Real::from(-1_i32)).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::chi_square_cdf(&Real::one(), 0).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::chi_square_sf(&Real::from(-1_i32), 1).unwrap_err(),
            Problem::NotANumber
        );
        assert_eq!(
            Real::from(-600_i32).dnorm().unwrap_err(),
            Problem::Exhausted
        );

        let tiny = Real::new(
            Rational::from_bigint_fraction(
                num::BigInt::from(1_u8),
                num::BigUint::from(10_u8).pow(30),
            )
            .unwrap(),
        );
        assert_eq!(tiny.clone().qnorm().unwrap_err(), Problem::Exhausted);
        assert_eq!(
            (Real::one() - tiny).qnorm().unwrap_err(),
            Problem::Exhausted
        );
    }

    #[test]
    fn normal_against_mpmath_references() {
        for &(kind, num, den, expected) in crate::real::normal_reference::CASES {
            let arg = normal_case_real(num, den);
            let value = if kind == "pnorm" {
                arg.pnorm().unwrap()
            } else {
                arg.qnorm().unwrap()
            };
            let got = trunc_str(&value, 1000);
            assert_eq!(got, expected, "{kind}({num}/{den}) disagrees with mpmath");
        }
    }

    fn adversarial_tiny() -> Real {
        Real::new(Rational::fraction(1, 1_000_000_000_000).unwrap())
    }

    fn adversarial_near_one() -> Real {
        Real::new(Rational::fraction(999_999, 1_000_000).unwrap())
    }

    #[test]
    fn adversarial_trig_tiny_huge_and_near_pole_cases() {
        use num::bigint::{BigInt, BigUint};

        let tiny = adversarial_tiny();
        let tiny_f64 = 1e-12_f64;
        assert_close(tiny.clone().sin(), tiny_f64.sin(), 1e-14);
        assert_close(tiny.clone().cos(), tiny_f64.cos(), 1e-14);
        assert_close(tiny.clone().tan().unwrap(), tiny_f64.tan(), 1e-14);

        let medium = Real::new(Rational::fraction(7, 5).unwrap());
        let medium_f64 = 7.0_f64 / 5.0_f64;
        assert_close(medium.clone().sin(), medium_f64.sin(), 1e-14);
        assert_close(medium.clone().cos(), medium_f64.cos(), 1e-14);
        assert_close(medium.clone().tan().unwrap(), medium_f64.tan(), 1e-14);

        let large = Real::new(Rational::new(1_000_000));
        let large_f64 = 1_000_000_f64;
        assert_close(large.clone().sin(), large_f64.sin(), 1e-12);
        assert_close(large.cos(), large_f64.cos(), 1e-12);

        let huge_even_pi_multiple = Real::new(Rational::from_bigint(BigInt::from(1_u8) << 128))
            * Real::pi()
            + medium.clone();
        assert_close(huge_even_pi_multiple.clone().sin(), medium_f64.sin(), 1e-12);
        assert_close(huge_even_pi_multiple.clone().cos(), medium_f64.cos(), 1e-12);
        assert_close(
            huge_even_pi_multiple.tan().unwrap(),
            medium_f64.tan(),
            1e-12,
        );

        let near_half_pi = pi_fraction(1, 2)
            - Real::new(
                Rational::from_bigint_fraction(BigInt::from(1_u8), BigUint::from(1_u8) << 40)
                    .unwrap(),
            );
        let near_half_pi_f64 = std::f64::consts::FRAC_PI_2 - 2_f64.powi(-40);
        assert_close(near_half_pi.clone().sin(), near_half_pi_f64.sin(), 1e-12);
        assert_close(near_half_pi.cos(), near_half_pi_f64.cos(), 1e-10);
    }

    #[test]
    fn adversarial_inverse_trig_endpoint_and_symmetry_cases() {
        let tiny = adversarial_tiny();
        let tiny_f64 = 1e-12_f64;
        assert_close(tiny.clone().asin().unwrap(), tiny_f64.asin(), 1e-14);
        assert_close(tiny.clone().acos().unwrap(), tiny_f64.acos(), 1e-14);
        assert_close(tiny.clone().atan().unwrap(), tiny_f64.atan(), 1e-14);

        let near_one = adversarial_near_one();
        let near_one_f64 = 0.999999_f64;
        assert_close(near_one.clone().asin().unwrap(), near_one_f64.asin(), 1e-12);
        assert_close(near_one.clone().acos().unwrap(), near_one_f64.acos(), 1e-12);

        let near_minus_one = -near_one;
        assert_close(
            near_minus_one.clone().asin().unwrap(),
            (-near_one_f64).asin(),
            1e-12,
        );
        assert_close(
            near_minus_one.acos().unwrap(),
            (-near_one_f64).acos(),
            1e-12,
        );

        let huge = Real::new(Rational::new(1_000_000));
        assert_close(huge.atan().unwrap(), 1_000_000_f64.atan(), 1e-14);

        let just_outside = Real::new(Rational::one()) + tiny;
        assert_eq!(just_outside.clone().asin(), Err(Problem::NotANumber));
        assert_eq!(just_outside.acos(), Err(Problem::NotANumber));
    }

    #[test]
    fn adversarial_inverse_hyperbolic_endpoint_cases() {
        let tiny = adversarial_tiny();
        let tiny_f64 = 1e-12_f64;
        assert_close(tiny.clone().asinh().unwrap(), tiny_f64.asinh(), 1e-14);
        assert_close((-tiny.clone()).asinh().unwrap(), (-tiny_f64).asinh(), 1e-14);
        assert_close(tiny.clone().atanh().unwrap(), tiny_f64.atanh(), 1e-14);

        let near_one = adversarial_near_one();
        let near_one_f64 = 0.999999_f64;
        assert_close(
            near_one.clone().atanh().unwrap(),
            near_one_f64.atanh(),
            5e-12,
        );
        assert_close((-near_one).atanh().unwrap(), (-near_one_f64).atanh(), 5e-12);

        let one_plus_tiny = Real::new(Rational::one()) + tiny.clone();
        assert_close(
            one_plus_tiny.clone().acosh().unwrap(),
            (1.0_f64 + tiny_f64).acosh(),
            1e-9,
        );

        let large = Real::new(Rational::new(1_000_000));
        assert_close(large.clone().asinh().unwrap(), 1_000_000_f64.asinh(), 1e-14);
        assert_close(large.acosh().unwrap(), 1_000_000_f64.acosh(), 1e-14);

        let one_minus_tiny = Real::new(Rational::one()) - tiny;
        assert_eq!(one_minus_tiny.acosh(), Err(Problem::NotANumber));
        assert_eq!(Real::new(Rational::one()).atanh(), Err(Problem::Infinity));
        assert_eq!(one_plus_tiny.atanh(), Err(Problem::NotANumber));
    }

    #[test]
    fn dot_products_match_generic_real_arithmetic() {
        let left = [
            Real::new(Rational::fraction(6, 5).unwrap()),
            Real::new(Rational::fraction(3, 10).unwrap()),
            Real::new(Rational::fraction(-7, 10).unwrap()),
            Real::new(Rational::new(2)),
        ];
        let right = [
            Real::new(Rational::fraction(-4, 5).unwrap()),
            Real::new(Rational::fraction(11, 10).unwrap()),
            Real::new(Rational::fraction(1, 2).unwrap()),
            Real::new(Rational::new(-3)),
        ];
        let expected = &(&left[0] * &right[0])
            + &(&left[1] * &right[1])
            + &(&left[2] * &right[2])
            + &(&left[3] * &right[3]);

        assert_eq!(
            Real::dot4_refs(
                [&left[0], &left[1], &left[2], &left[3]],
                [&right[0], &right[1], &right[2], &right[3]],
            ),
            expected
        );
    }

    #[test]
    fn borrowed_exact_rational_preserves_lazy_storage_until_numeric_observation() {
        let lazy =
            Rational::from_parts_raw_unreduced(num::bigint::Sign::Plus, 6_u8.into(), 4_u8.into());
        assert!(lazy.is_internally_unreduced());
        let lazy_identity = lazy.storage_identity();
        let value = Real::new(lazy);

        let borrowed = value.exact_rational_ref().unwrap();
        assert_eq!(borrowed.storage_identity(), lazy_identity);
        assert_eq!(borrowed, &Rational::fraction(3, 2).unwrap());

        let owned = value.exact_rational().unwrap();
        assert_ne!(owned.storage_identity(), lazy_identity);
        assert_eq!(owned.numerator(), &3_u8.into());
        assert_eq!(owned.denominator(), &2_u8.into());
    }

    #[test]
    fn dense_self_dot_reuses_exact_result_after_observation() {
        let values = [
            Real::new(Rational::fraction(456_789_012_345_671_i64, 1_u64 << 50).unwrap()),
            Real::new(Rational::fraction(-567_890_123_456_781_i64, 1_u64 << 49).unwrap()),
            Real::new(Rational::fraction(678_901_234_567_893_i64, 1_u64 << 48).unwrap()),
        ];
        let refs = [&values[0], &values[1], &values[2]];
        let expected = Real::dot3_refs(refs, refs);
        let second = Real::dot3_refs(refs, refs);
        let third = Real::dot3_refs(refs, refs);

        assert_eq!(second, expected);
        assert_eq!(third, expected);
        let second = second.exact_rational_ref().unwrap();
        let third = third.exact_rational_ref().unwrap();
        assert!(std::ptr::eq(&**second, &**third));

        let values = [
            Real::from(1_000_000_000_i64),
            Real::from(-1_000_000_000_i64),
            Real::one(),
            -Real::one(),
        ];
        let refs = [&values[0], &values[1], &values[2], &values[3]];
        let _ = Real::dot4_refs(refs, refs);
        let second = Real::dot4_refs(refs, refs);
        let third = Real::dot4_refs(refs, refs);
        let second = second.exact_rational_ref().unwrap();
        let third = third.exact_rational_ref().unwrap();
        assert!(std::ptr::eq(&**second, &**third));
    }

    #[test]
    fn exact_rational_signed_product_sum_matches_generic_arithmetic() {
        let terms = [
            [
                Real::new(Rational::fraction(3, 8).unwrap()),
                Real::new(Rational::fraction(-5, 12).unwrap()),
                Real::new(Rational::fraction(7, 11).unwrap()),
            ],
            [
                Real::new(Rational::fraction(13, 9).unwrap()),
                Real::new(Rational::fraction(17, 25).unwrap()),
                Real::new(Rational::fraction(-19, 6).unwrap()),
            ],
            [
                Real::new(Rational::fraction(-23, 10).unwrap()),
                Real::new(Rational::fraction(29, 14).unwrap()),
                Real::new(Rational::fraction(31, 15).unwrap()),
            ],
        ];
        let expected = &(&terms[0][0] * &terms[0][1] * &terms[0][2])
            - &(&terms[1][0] * &terms[1][1] * &terms[1][2])
            + &(&terms[2][0] * &terms[2][1] * &terms[2][2]);

        assert_eq!(
            Real::exact_rational_signed_product_sum(
                [true, false, true],
                [
                    [&terms[0][0], &terms[0][1], &terms[0][2]],
                    [&terms[1][0], &terms[1][1], &terms[1][2]],
                    [&terms[2][0], &terms[2][1], &terms[2][2]],
                ],
            ),
            Some(expected)
        );
    }

    #[test]
    fn exact_rational_signed_product_sum_rejects_symbolic_terms() {
        let one = Real::one();
        let pi = Real::pi();
        let two = Real::from(2_i32);
        let three = Real::from(3_i32);

        assert_eq!(
            Real::exact_rational_signed_product_sum([true, false], [[&one, &two], [&pi, &three]]),
            None
        );
    }

    #[test]
    fn exact_dyadic_parameterized_point_matches_expanded_arithmetic() {
        let q =
            |numerator, denominator| Real::new(Rational::fraction(numerator, denominator).unwrap());
        let origin = [q(3, 8), q(-5, 4)];
        let delta = [q(7, 16), q(9, 8)];
        let numerator = q(5, 32);
        let denominator = q(11, 64);
        let expected_parameter = (&numerator / &denominator).unwrap();
        let expected_point = [
            Real::affine(&origin[0], &expected_parameter, &delta[0]),
            Real::affine(&origin[1], &expected_parameter, &delta[1]),
        ];

        let (parameter, point) = Real::exact_rational_parameterized_point2_known_dyadic(
            [&origin[0], &origin[1]],
            [&delta[0], &delta[1]],
            &numerator,
            &denominator,
        )
        .unwrap();
        assert_eq!(parameter, expected_parameter);
        assert_eq!(point, expected_point);
        assert_eq!(
            Real::exact_rational_parameterized_point2_known_dyadic(
                [&origin[0], &origin[1]],
                [&delta[0], &delta[1]],
                &numerator,
                &Real::zero(),
            ),
            Err(Problem::DivideByZero)
        );
    }

    #[test]
    fn exact_dyadic_interpolated_point3_matches_expanded_arithmetic() {
        let q =
            |numerator, denominator| Real::new(Rational::fraction(numerator, denominator).unwrap());
        let origin = [q(3, 8), q(-5, 4), q(13, 32)];
        let delta = [q(7, 16), q(9, 8), q(-15, 64)];
        let numerator = q(5, 32);
        let denominator = q(11, 64);
        let expected_parameter = (&numerator / &denominator).unwrap();
        let expected_point = std::array::from_fn(|index| {
            Real::affine(&origin[index], &expected_parameter, &delta[index])
        });

        let end: [Real; 3] = std::array::from_fn(|index| &origin[index] + &delta[index]);
        assert_eq!(
            Real::exact_rational_interpolate_point3_known_dyadic(
                [&origin[0], &origin[1], &origin[2]],
                [&end[0], &end[1], &end[2]],
                &numerator,
                &denominator,
            )
            .unwrap(),
            expected_point
        );
        for seed in 1_i64..=256 {
            let dyadic = |multiplier: i64, addend: i64, shift: u32| {
                q(
                    (seed * multiplier + addend).rem_euclid(97) - 48,
                    1_u64 << shift,
                )
            };
            let start = [dyadic(5, 1, 3), dyadic(7, 2, 5), dyadic(11, 3, 7)];
            let end = [dyadic(13, 4, 4), dyadic(17, 5, 6), dyadic(19, 6, 8)];
            let numerator = dyadic(23, 7, 5);
            let denominator = q((seed * 29).rem_euclid(31) + 1, 1_u64 << (seed as u32 % 9));
            let parameter = (&numerator / &denominator).unwrap();
            let expected: [Real; 3] = std::array::from_fn(|index| {
                Real::affine(&start[index], &parameter, &(&end[index] - &start[index]))
            });
            assert_eq!(
                Real::exact_rational_interpolate_point3_known_dyadic(
                    [&start[0], &start[1], &start[2]],
                    [&end[0], &end[1], &end[2]],
                    &numerator,
                    &denominator,
                )
                .unwrap(),
                expected,
                "seed={seed}",
            );
        }
        assert_eq!(
            Real::exact_rational_interpolate_point3_known_dyadic(
                [&origin[0], &origin[1], &origin[2]],
                [&end[0], &end[1], &end[2]],
                &numerator,
                &Real::zero(),
            ),
            Err(Problem::DivideByZero)
        );
    }

    #[test]
    fn exact_dyadic_parameterized_point_reduces_wide_affine_numerators_exactly() {
        let dyadic = |sign: i8, odd: u64, shift: usize| {
            let magnitude = num::BigInt::from(odd) << 220_usize;
            let magnitude = if sign < 0 { -magnitude } else { magnitude };
            Real::new(
                Rational::from_bigint_fraction(magnitude, num::BigUint::from(1_u8) << shift)
                    .unwrap(),
            )
        };
        for origin_sign in [-1_i8, 1] {
            for delta_sign in [-1_i8, 1] {
                for parameter_sign in [-1_i8, 1] {
                    for (denominator_magnitude, denominator_shift) in
                        [(12_u8, 0_usize), (45, 0), (45, 7), (45, 129)]
                    {
                        let origin = [dyadic(origin_sign, 9, 5), dyadic(-origin_sign, 15, 11)];
                        let delta = [
                            Real::new(Rational::fraction(i64::from(delta_sign) * 35, 64).unwrap()),
                            Real::new(Rational::fraction(i64::from(delta_sign) * 77, 16).unwrap()),
                        ];
                        let numerator = Real::new(
                            Rational::fraction(i64::from(parameter_sign) * 55, 32).unwrap(),
                        );
                        let denominator = Real::new(
                            Rational::from_bigint_fraction(
                                num::BigInt::from(denominator_magnitude),
                                num::BigUint::from(1_u8) << denominator_shift,
                            )
                            .unwrap(),
                        );
                        let parameter = (&numerator / &denominator).unwrap();
                        let expected = [
                            Real::affine(&origin[0], &parameter, &delta[0]),
                            Real::affine(&origin[1], &parameter, &delta[1]),
                        ];

                        let (actual_parameter, actual) =
                            Real::exact_rational_parameterized_point2_known_dyadic(
                                [&origin[0], &origin[1]],
                                [&delta[0], &delta[1]],
                                &numerator,
                                &denominator,
                            )
                            .unwrap();
                        assert_eq!(actual_parameter, parameter);
                        assert_eq!(
                            actual, expected,
                            "origin_sign={origin_sign}, delta_sign={delta_sign}, parameter_sign={parameter_sign}, denominator_magnitude={denominator_magnitude}, denominator_shift={denominator_shift}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn fused_exact_rational_line_intersection_matches_expanded_exact_arithmetic() {
        let mut state = 0x510e_527f_ade6_82d1_u64;
        let mut nonparallel_cases = 0;
        let mut nondyadic_cases = 0;
        for _ in 0..512 {
            let points: [[Real; 2]; 4] = core::array::from_fn(|_| {
                core::array::from_fn(|_| {
                    state = state
                        .wrapping_mul(2_862_933_555_777_941_757)
                        .wrapping_add(3_037_000_493);
                    let numerator = i64::try_from(state % 2049).unwrap() - 1024;
                    let denominator = 3 + 2 * ((state >> 32) % 15);
                    Real::new(Rational::fraction(numerator, denominator).unwrap())
                })
            });
            let [first_start, first_end, second_start, second_end] = &points;
            let first_delta = [
                &first_end[0] - &first_start[0],
                &first_end[1] - &first_start[1],
            ];
            let second_delta = [
                &second_end[0] - &second_start[0],
                &second_end[1] - &second_start[1],
            ];
            let denominator = Real::diff_of_products(
                &first_delta[0],
                &second_delta[1],
                &first_delta[1],
                &second_delta[0],
            );
            if denominator == Real::zero() {
                assert_eq!(
                    Real::exact_rational_line_intersection2_point_known_exact(
                        [&first_start[0], &first_start[1]],
                        [&first_end[0], &first_end[1]],
                        [&second_start[0], &second_start[1]],
                        [&second_end[0], &second_end[1]],
                    ),
                    None
                );
                continue;
            }

            nonparallel_cases += 1;
            nondyadic_cases += usize::from(
                points
                    .iter()
                    .flatten()
                    .any(|coordinate| !coordinate.is_exact_dyadic_rational()),
            );
            let start_delta = [
                &second_start[0] - &first_start[0],
                &second_start[1] - &first_start[1],
            ];
            let numerator = Real::diff_of_products(
                &start_delta[0],
                &second_delta[1],
                &start_delta[1],
                &second_delta[0],
            );
            let parameter = (&numerator / &denominator).unwrap();
            let expected = [
                Real::affine(&first_start[0], &parameter, &first_delta[0]),
                Real::affine(&first_start[1], &parameter, &first_delta[1]),
            ];
            assert_eq!(
                Real::exact_rational_line_intersection2_point_known_exact(
                    [&first_start[0], &first_start[1]],
                    [&first_end[0], &first_end[1]],
                    [&second_start[0], &second_start[1]],
                    [&second_end[0], &second_end[1]],
                ),
                Some(expected)
            );
        }
        assert!(nonparallel_cases > 500);
        assert!(nondyadic_cases > 500);

        let zero = Real::zero();
        let one = Real::one();
        let two = Real::from(2_i8);
        assert_eq!(
            Real::exact_rational_line_intersection2_point_known_exact(
                [&zero, &zero],
                [&one, &one],
                [&one, &one],
                [&two, &two],
            ),
            None
        );
        assert_eq!(
            Real::exact_rational_line_intersection2_point_known_exact(
                [&Real::pi(), &zero],
                [&one, &one],
                [&zero, &one],
                [&one, &zero],
            ),
            None
        );
    }

    #[test]
    fn fused_dyadic_line_intersection_matches_expanded_exact_arithmetic() {
        let mut state = 0x6a09_e667_f3bc_c909_u64;
        let mut fused_cases = 0;
        let mut retained_cases = Vec::new();
        for _ in 0..512 {
            let points: [[Real; 2]; 4] = core::array::from_fn(|_| {
                core::array::from_fn(|_| {
                    state = state
                        .wrapping_mul(6_364_136_223_846_793_005)
                        .wrapping_add(1_442_695_040_888_963_407);
                    let numerator = i64::try_from(state % 257).unwrap() - 128;
                    let denominator = 1_u64 << u32::try_from((state >> 32) % 8).unwrap();
                    Real::new(Rational::fraction(numerator, denominator).unwrap())
                })
            });
            let [first_start, first_end, second_start, second_end] = &points;
            let first_delta = [
                &first_end[0] - &first_start[0],
                &first_end[1] - &first_start[1],
            ];
            let second_delta = [
                &second_end[0] - &second_start[0],
                &second_end[1] - &second_start[1],
            ];
            let start_delta = [
                &second_start[0] - &first_start[0],
                &second_start[1] - &first_start[1],
            ];
            let denominator = Real::diff_of_products(
                &first_delta[0],
                &second_delta[1],
                &first_delta[1],
                &second_delta[0],
            );
            if denominator == Real::zero() {
                continue;
            }
            let Some((first_parameter, second_parameter, point)) =
                Real::exact_rational_line_intersection2_known_dyadic(
                    [&first_start[0], &first_start[1]],
                    [&first_end[0], &first_end[1]],
                    [&second_start[0], &second_start[1]],
                    [&second_end[0], &second_end[1]],
                )
            else {
                continue;
            };
            let (retained_parameters, point_only) =
                Real::exact_rational_line_intersection2_point_known_dyadic(
                    [&first_start[0], &first_start[1]],
                    [&first_end[0], &first_end[1]],
                    [&second_start[0], &second_start[1]],
                    [&second_end[0], &second_end[1]],
                )
                .expect("the point-only kernel shares the fused path's checked bounds");
            let first_line = ExactDyadicLine2::from_reals(
                [&first_start[0], &first_start[1]],
                [&first_end[0], &first_end[1]],
            )
            .expect("the fused path's first line should fit");
            let (line_parameters, line_point) = first_line
                .intersection_point(
                    [&second_start[0], &second_start[1]],
                    [&second_end[0], &second_end[1]],
                )
                .expect("retained-line and one-shot compact paths share bounds");
            let exact_f64_point = |point: &[Real; 2]| {
                [
                    point[0]
                        .to_f64_exact_dyadic()
                        .expect("the compact oracle coordinates fit binary64"),
                    point[1]
                        .to_f64_exact_dyadic()
                        .expect("the compact oracle coordinates fit binary64"),
                ]
            };
            let f64_line = ExactDyadicLine2::from_f64(
                exact_f64_point(first_start),
                exact_f64_point(first_end),
            )
            .expect("the compact binary64 line should fit directly");
            let (f64_parameters, f64_point) = f64_line
                .intersection_point_f64(exact_f64_point(second_start), exact_f64_point(second_end))
                .expect("direct binary64 and retained-rational paths share bounds");
            let (retained_f64_parameters, retained_f64_point) = f64_line
                .retained_intersection_point_f64(
                    exact_f64_point(second_start),
                    exact_f64_point(second_end),
                )
                .expect("retained and materialized binary64 paths share bounds");
            fused_cases += 1;
            let first_numerator = Real::diff_of_products(
                &start_delta[0],
                &second_delta[1],
                &start_delta[1],
                &second_delta[0],
            );
            let second_numerator = Real::diff_of_products(
                &start_delta[0],
                &first_delta[1],
                &start_delta[1],
                &first_delta[0],
            );
            let expected_first = (&first_numerator / &denominator).unwrap();
            let expected_second = (&second_numerator / &denominator).unwrap();
            assert_eq!(first_parameter, expected_first);
            assert_eq!(second_parameter, expected_second);
            assert_eq!(
                retained_parameters.materialize_first_parameter(),
                expected_first
            );
            assert_eq!(
                retained_parameters.materialize_second_parameter(),
                expected_second
            );
            assert_eq!(
                point,
                [
                    Real::affine(&first_start[0], &expected_first, &first_delta[0]),
                    Real::affine(&first_start[1], &expected_first, &first_delta[1]),
                ]
            );
            assert_eq!(point_only, point);
            assert_eq!(line_point, point);
            assert_eq!(f64_point, point);
            assert_eq!(retained_f64_point.materialize(), point);
            assert_eq!(
                line_parameters.materialize_first_parameter(),
                expected_first
            );
            assert_eq!(
                line_parameters.materialize_second_parameter(),
                expected_second
            );
            assert_eq!(f64_parameters.materialize_first_parameter(), expected_first);
            assert_eq!(
                f64_parameters.materialize_second_parameter(),
                expected_second
            );
            assert_eq!(
                retained_f64_parameters.materialize_first_parameter(),
                expected_first
            );
            assert_eq!(
                retained_f64_parameters.materialize_second_parameter(),
                expected_second
            );
            retained_cases.push((retained_parameters, expected_first, expected_second));
        }
        assert!(
            fused_cases > 400,
            "only {fused_cases} cases used the fused path"
        );
        for pair in retained_cases.windows(2) {
            let [
                (left, left_first, left_second),
                (right, right_first, right_second),
            ] = pair
            else {
                unreachable!("a two-element window has two retained parameter cases");
            };
            assert_eq!(
                left.compare_first_parameter(right),
                left_first.partial_cmp(right_first).unwrap()
            );
            assert_eq!(
                left.compare_second_parameter(right),
                left_second.partial_cmp(right_second).unwrap()
            );
        }
    }

    #[test]
    fn fused_dyadic_line_intersection_defers_wide_inputs() {
        let wide = Real::new(
            Rational::from_bigint_fraction(
                num::BigInt::from(1_u8) << 192_usize,
                num::BigUint::from(1_u8),
            )
            .unwrap(),
        );
        let zero = Real::zero();
        let one = Real::one();
        assert_eq!(
            Real::exact_rational_line_intersection2_known_dyadic(
                [&wide, &zero],
                [&one, &one],
                [&zero, &one],
                [&one, &zero],
            ),
            None
        );
        assert!(
            Real::exact_rational_line_intersection2_point_known_dyadic(
                [&wide, &zero],
                [&one, &one],
                [&zero, &one],
                [&one, &zero],
            )
            .is_none()
        );
        assert!(
            Real::exact_rational_line_intersection2_point_known_dyadic_wide(
                [&wide, &zero],
                [&one, &one],
                [&zero, &one],
                [&one, &zero],
            )
            .is_none()
        );
        assert!(ExactDyadicLine2::from_reals([&wide, &zero], [&one, &one]).is_none());
    }

    #[test]
    fn fused_wide_dyadic_line_intersection_retains_large_determinants() {
        let extent = (1_u128 << 100) + 3;
        let integer = |value| Real::new(Rational::from_bigint(num::BigInt::from(value)));
        let zero = Real::zero();
        let first_start = [zero.clone(), zero.clone()];
        let first_end = [integer(extent), integer(extent - 1)];
        let second_start = [zero.clone(), integer(extent - 1)];
        let second_end = [integer(extent), zero.clone()];

        assert!(
            Real::exact_rational_line_intersection2_point_known_dyadic(
                [&first_start[0], &first_start[1]],
                [&first_end[0], &first_end[1]],
                [&second_start[0], &second_start[1]],
                [&second_end[0], &second_end[1]],
            )
            .is_none()
        );
        let (parameters, point) = Real::exact_rational_line_intersection2_point_known_dyadic_wide(
            [&first_start[0], &first_start[1]],
            [&first_end[0], &first_end[1]],
            [&second_start[0], &second_start[1]],
            [&second_end[0], &second_end[1]],
        )
        .expect("the four-limb determinant carrier covers this crossing");
        let half: Real = "1/2".parse().unwrap();
        assert_eq!(parameters.materialize_first_parameter(), half);
        assert_eq!(parameters.materialize_second_parameter(), half);
        assert_eq!(
            point,
            [
                Real::new(
                    Rational::from_bigint_fraction(
                        num::BigInt::from(extent),
                        num::BigUint::from(2_u8),
                    )
                    .unwrap(),
                ),
                Real::new(
                    Rational::from_bigint_fraction(
                        num::BigInt::from(extent - 1),
                        num::BigUint::from(2_u8),
                    )
                    .unwrap(),
                ),
            ]
        );

        let three = Real::from(3_i8);
        let one = Real::one();
        let minus_one = Real::from(-1_i8);
        let (compact, _) = Real::exact_rational_line_intersection2_point_known_dyadic(
            [&zero, &zero],
            [&three, &zero],
            [&one, &minus_one],
            [&one, &one],
        )
        .unwrap();
        assert_eq!(
            parameters.compare_first_parameter_to_compact(&compact),
            std::cmp::Ordering::Greater
        );
        assert_eq!(
            parameters.compare_second_parameter_to_compact(&compact),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            parameters.compare_first_parameter(&parameters),
            std::cmp::Ordering::Equal
        );

        let binary64_extent = 2_f64.powi(100);
        let binary64_near_extent = f64::from_bits(binary64_extent.to_bits() - 1);
        let binary64_line =
            ExactDyadicLine2::from_f64([0.0, 0.0], [binary64_extent, binary64_near_extent])
                .expect("word-sized binary64 endpoints should fit");
        assert!(
            binary64_line
                .intersection_point_f64([0.0, binary64_near_extent], [binary64_extent, 0.0],)
                .is_none(),
            "the native determinant carrier should reject the 200-bit product"
        );
        let (binary64_parameters, binary64_point) = binary64_line
            .wide_intersection_point_f64([0.0, binary64_near_extent], [binary64_extent, 0.0])
            .expect("the fixed wide carrier should retain the binary64 crossing");
        let (retained_binary64_parameters, retained_binary64_point) = binary64_line
            .wide_retained_intersection_point_f64(
                [0.0, binary64_near_extent],
                [binary64_extent, 0.0],
            )
            .expect("the deferred wide carrier should retain the binary64 crossing");
        assert_eq!(binary64_parameters.materialize_first_parameter(), half);
        assert_eq!(binary64_parameters.materialize_second_parameter(), half);
        assert_eq!(
            binary64_point,
            [
                Real::try_from(binary64_extent / 2.0).unwrap(),
                Real::try_from(binary64_near_extent / 2.0).unwrap(),
            ]
        );
        assert_eq!(
            retained_binary64_parameters.materialize_first_parameter(),
            half
        );
        assert_eq!(
            retained_binary64_parameters.materialize_second_parameter(),
            half
        );
        assert_eq!(retained_binary64_point.materialize(), binary64_point);
        assert!(
            ExactDyadicLine2::from_f64([f64::INFINITY, 0.0], [binary64_extent, 0.0],).is_none()
        );
        assert!(
            ExactDyadicLine2::from_f64([2_f64.powi(200), 0.0], [binary64_extent, 0.0],).is_none()
        );
    }

    #[test]
    fn fused_wide_dyadic_line_intersection_matches_expanded_exact_arithmetic() {
        let mut state = 0xbb67_ae85_84ca_a73b_u64;
        let extent = (num::BigInt::from(1_u8) << 100_usize) + 3_u8;
        let mut retained_cases = Vec::new();
        for _ in 0..256 {
            let points: [[Real; 2]; 4] = core::array::from_fn(|_| {
                core::array::from_fn(|_| {
                    state = state
                        .wrapping_mul(2_862_933_555_777_941_757)
                        .wrapping_add(3_037_000_493);
                    let coefficient = i64::try_from(state % 2049).unwrap() - 1024;
                    let denominator =
                        num::BigUint::from(1_u8) << usize::try_from((state >> 32) % 8).unwrap();
                    Real::new(
                        Rational::from_bigint_fraction(
                            num::BigInt::from(coefficient) * &extent,
                            denominator,
                        )
                        .unwrap(),
                    )
                })
            });
            let [first_start, first_end, second_start, second_end] = &points;
            let first_delta = [
                &first_end[0] - &first_start[0],
                &first_end[1] - &first_start[1],
            ];
            let second_delta = [
                &second_end[0] - &second_start[0],
                &second_end[1] - &second_start[1],
            ];
            let start_delta = [
                &second_start[0] - &first_start[0],
                &second_start[1] - &first_start[1],
            ];
            let denominator = Real::diff_of_products(
                &first_delta[0],
                &second_delta[1],
                &first_delta[1],
                &second_delta[0],
            );
            if denominator == Real::zero() {
                continue;
            }
            let Some((parameters, point)) =
                Real::exact_rational_line_intersection2_point_known_dyadic_wide(
                    [&first_start[0], &first_start[1]],
                    [&first_end[0], &first_end[1]],
                    [&second_start[0], &second_start[1]],
                    [&second_end[0], &second_end[1]],
                )
            else {
                continue;
            };
            let first_line = ExactDyadicLine2::from_reals(
                [&first_start[0], &first_start[1]],
                [&first_end[0], &first_end[1]],
            )
            .expect("the wide determinant path still uses word-sized source deltas");
            let (line_parameters, line_point) = first_line
                .wide_intersection_point(
                    [&second_start[0], &second_start[1]],
                    [&second_end[0], &second_end[1]],
                )
                .expect("retained-line and one-shot wide paths share bounds");
            let first_numerator = Real::diff_of_products(
                &start_delta[0],
                &second_delta[1],
                &start_delta[1],
                &second_delta[0],
            );
            let second_numerator = Real::diff_of_products(
                &start_delta[0],
                &first_delta[1],
                &start_delta[1],
                &first_delta[0],
            );
            let expected_first = (&first_numerator / &denominator).unwrap();
            let expected_second = (&second_numerator / &denominator).unwrap();
            assert_eq!(parameters.materialize_first_parameter(), expected_first);
            assert_eq!(parameters.materialize_second_parameter(), expected_second);
            assert_eq!(
                point,
                [
                    Real::affine(&first_start[0], &expected_first, &first_delta[0]),
                    Real::affine(&first_start[1], &expected_first, &first_delta[1]),
                ]
            );
            assert_eq!(line_point, point);
            assert_eq!(
                line_parameters.materialize_first_parameter(),
                expected_first
            );
            assert_eq!(
                line_parameters.materialize_second_parameter(),
                expected_second
            );
            retained_cases.push((parameters, expected_first, expected_second));
        }
        assert!(
            retained_cases.len() > 240,
            "only {} cases used the wide fused path",
            retained_cases.len()
        );
        for pair in retained_cases.windows(2) {
            let [
                (left, left_first, left_second),
                (right, right_first, right_second),
            ] = pair
            else {
                unreachable!("a two-element window has two retained parameter cases");
            };
            assert_eq!(
                left.compare_first_parameter(right),
                left_first.partial_cmp(right_first).unwrap()
            );
            assert_eq!(
                left.compare_second_parameter(right),
                left_second.partial_cmp(right_second).unwrap()
            );
        }
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn fused_dyadic_line_intersection_canonicalizes_coordinates_only_on_observation() {
        let point = |x: i32, y: i32| [Real::from(x), Real::from(y)];
        let first_start = point(0, 1);
        let first_end = point(3, 1);
        let second_start = point(2, 0);
        let second_end = point(2, 3);

        crate::dispatch_trace::reset();
        let (_, _, intersection) = crate::dispatch_trace::with_recording(|| {
            Real::exact_rational_line_intersection2_known_dyadic(
                [&first_start[0], &first_start[1]],
                [&first_end[0], &first_end[1]],
                [&second_start[0], &second_start[1]],
                [&second_end[0], &second_end[1]],
            )
            .unwrap()
        });
        let construction_trace = crate::dispatch_trace::take_trace();
        assert_eq!(
            construction_trace.path_count(
                "rational",
                "canonicalization",
                "lazy-internal-coordinate"
            ),
            0
        );

        crate::dispatch_trace::reset();
        let coordinates = crate::dispatch_trace::with_recording(|| {
            intersection.map(|coordinate| coordinate.exact_rational().unwrap())
        });
        let observation_trace = crate::dispatch_trace::take_trace();
        assert_eq!(coordinates, [Rational::new(2), Rational::new(1)]);
        assert_eq!(
            observation_trace.path_count(
                "rational",
                "canonicalization",
                "lazy-internal-coordinate"
            ),
            2
        );
    }

    #[test]
    fn exact_dyadic_quotient_matches_general_division_across_scales() {
        for numerator in -12_i64..=12 {
            for denominator in (-12_i64..=12).filter(|value| *value != 0) {
                for numerator_shift in 0..=7 {
                    for denominator_shift in 0..=7 {
                        let numerator = Real::new(
                            Rational::fraction(numerator, 1_u64 << numerator_shift).unwrap(),
                        );
                        let denominator = Real::new(
                            Rational::fraction(denominator, 1_u64 << denominator_shift).unwrap(),
                        );
                        assert_eq!(
                            Real::exact_rational_quotient_known_dyadic(&numerator, &denominator)
                                .unwrap(),
                            (&numerator / &denominator).unwrap()
                        );
                    }
                }
            }
        }

        // Word-sized magnitudes can still need an arbitrary-precision result
        // after applying the net dyadic scale. Keep that overflow on the
        // general fallback, alongside inputs whose magnitude already exceeds
        // the native reducer.
        let large_scale_denominator = Real::new(
            Rational::from_bigint_fraction(
                num::BigInt::from(3_u8),
                num::BigUint::from(1_u8) << 200,
            )
            .unwrap(),
        );
        let moderate_scale_denominator = Real::new(
            Rational::from_bigint_fraction(
                num::BigInt::from(3_u8),
                num::BigUint::from(1_u8) << 100,
            )
            .unwrap(),
        );
        let wide_word_integer = Real::new(Rational::from_bigint(
            (num::BigInt::from(1_u8) << 100_usize) + 1,
        ));
        let three_over_moderate_scale = Real::new(
            Rational::from_bigint_fraction(
                num::BigInt::from(3_u8),
                num::BigUint::from(1_u8) << 100,
            )
            .unwrap(),
        );
        let wide_numerator: Real = "680564733841876926926749214863536422913".parse().unwrap();
        let three_eighths: Real = "3/8".parse().unwrap();
        for (numerator, denominator) in [
            (Real::one(), large_scale_denominator),
            (wide_word_integer.clone(), moderate_scale_denominator),
            (three_over_moderate_scale, wide_word_integer),
            (wide_numerator, three_eighths),
        ] {
            assert_eq!(
                Real::exact_rational_quotient_known_dyadic(&numerator, &denominator).unwrap(),
                (&numerator / &denominator).unwrap()
            );
        }
    }

    #[test]
    fn exact_rational_matrix3_inverse_uses_shared_exact_aggregate() {
        let q =
            |numerator, denominator| Real::new(Rational::fraction(numerator, denominator).unwrap());
        let matrix = [
            [q(1, 2), q(1, 1), q(3, 2)],
            [q(0, 1), q(1, 4), q(1, 1)],
            [q(5, 8), q(3, 4), q(0, 1)],
        ];
        let actual = Real::exact_rational_matrix3_inverse_known_exact([
            [&matrix[0][0], &matrix[0][1], &matrix[0][2]],
            [&matrix[1][0], &matrix[1][1], &matrix[1][2]],
            [&matrix[2][0], &matrix[2][1], &matrix[2][2]],
        ])
        .unwrap();
        let actual_dyadic = Real::exact_rational_matrix3_inverse_known_dyadic([
            [&matrix[0][0], &matrix[0][1], &matrix[0][2]],
            [&matrix[1][0], &matrix[1][1], &matrix[1][2]],
            [&matrix[2][0], &matrix[2][1], &matrix[2][2]],
        ])
        .unwrap();
        let expected = [
            [Real::from(-48), Real::from(72), Real::from(40)],
            [Real::from(40), Real::from(-60), Real::from(-32)],
            [Real::from(-10), Real::from(16), Real::from(8)],
        ];
        assert_eq!(actual, expected);
        assert_eq!(actual_dyadic, expected);

        let singular = [
            [Real::from(1), Real::from(2), Real::from(3)],
            [Real::from(1), Real::from(2), Real::from(3)],
            [Real::from(0), Real::from(0), Real::from(1)],
        ];
        assert_eq!(
            Real::exact_rational_matrix3_inverse_known_exact([
                [&singular[0][0], &singular[0][1], &singular[0][2]],
                [&singular[1][0], &singular[1][1], &singular[1][2]],
                [&singular[2][0], &singular[2][1], &singular[2][2]],
            ]),
            Err(Problem::DivideByZero)
        );
    }

    #[test]
    fn exact_rational_matrix4_inverse_uses_shared_exact_aggregate() {
        let q =
            |numerator, denominator| Real::new(Rational::fraction(numerator, denominator).unwrap());
        let matrix = [
            [q(1, 1), q(2, 1), q(3, 1), q(4, 1)],
            [q(0, 1), q(1, 1), q(4, 1), q(2, 1)],
            [q(5, 1), q(6, 1), q(0, 1), q(1, 1)],
            [q(2, 1), q(7, 1), q(1, 1), q(3, 1)],
        ];
        let actual = Real::exact_rational_matrix4_inverse_known_exact([
            [&matrix[0][0], &matrix[0][1], &matrix[0][2], &matrix[0][3]],
            [&matrix[1][0], &matrix[1][1], &matrix[1][2], &matrix[1][3]],
            [&matrix[2][0], &matrix[2][1], &matrix[2][2], &matrix[2][3]],
            [&matrix[3][0], &matrix[3][1], &matrix[3][2], &matrix[3][3]],
        ])
        .unwrap();
        let actual_dyadic = Real::exact_rational_matrix4_inverse_known_dyadic([
            [&matrix[0][0], &matrix[0][1], &matrix[0][2], &matrix[0][3]],
            [&matrix[1][0], &matrix[1][1], &matrix[1][2], &matrix[1][3]],
            [&matrix[2][0], &matrix[2][1], &matrix[2][2], &matrix[2][3]],
            [&matrix[3][0], &matrix[3][1], &matrix[3][2], &matrix[3][3]],
        ])
        .unwrap();
        let expected = [
            [q(1, 6), q(-1, 18), q(5, 18), q(-5, 18)],
            [q(-7, 33), q(10, 99), q(-5, 99), q(23, 99)],
            [q(-1, 6), q(7, 18), q(1, 18), q(-1, 18)],
            [q(29, 66), q(-65, 198), q(-17, 198), q(-1, 198)],
        ];
        assert_eq!(actual, expected);
        assert_eq!(actual_dyadic, expected);

        let scaled_upper = [
            [q(1, 2), q(1, 1), q(0, 1), q(0, 1)],
            [q(0, 1), q(1, 4), q(1, 1), q(0, 1)],
            [q(0, 1), q(0, 1), q(1, 8), q(1, 1)],
            [q(0, 1), q(0, 1), q(0, 1), q(1, 16)],
        ];
        let scaled_actual = Real::exact_rational_matrix4_inverse_known_dyadic([
            [
                &scaled_upper[0][0],
                &scaled_upper[0][1],
                &scaled_upper[0][2],
                &scaled_upper[0][3],
            ],
            [
                &scaled_upper[1][0],
                &scaled_upper[1][1],
                &scaled_upper[1][2],
                &scaled_upper[1][3],
            ],
            [
                &scaled_upper[2][0],
                &scaled_upper[2][1],
                &scaled_upper[2][2],
                &scaled_upper[2][3],
            ],
            [
                &scaled_upper[3][0],
                &scaled_upper[3][1],
                &scaled_upper[3][2],
                &scaled_upper[3][3],
            ],
        ])
        .unwrap();
        assert_eq!(
            scaled_actual,
            [
                [q(2, 1), q(-8, 1), q(64, 1), q(-1024, 1)],
                [q(0, 1), q(4, 1), q(-32, 1), q(512, 1)],
                [q(0, 1), q(0, 1), q(8, 1), q(-128, 1)],
                [q(0, 1), q(0, 1), q(0, 1), q(16, 1)],
            ]
        );

        let singular = [
            [Real::from(1), Real::from(2), Real::from(3), Real::from(4)],
            [Real::from(1), Real::from(2), Real::from(3), Real::from(4)],
            [Real::from(0), Real::from(1), Real::from(0), Real::from(0)],
            [Real::from(0), Real::from(0), Real::from(1), Real::from(0)],
        ];
        assert_eq!(
            Real::exact_rational_matrix4_inverse_known_exact([
                [
                    &singular[0][0],
                    &singular[0][1],
                    &singular[0][2],
                    &singular[0][3]
                ],
                [
                    &singular[1][0],
                    &singular[1][1],
                    &singular[1][2],
                    &singular[1][3]
                ],
                [
                    &singular[2][0],
                    &singular[2][1],
                    &singular[2][2],
                    &singular[2][3]
                ],
                [
                    &singular[3][0],
                    &singular[3][1],
                    &singular[3][2],
                    &singular[3][3]
                ],
            ]),
            Err(Problem::DivideByZero)
        );
        assert_eq!(
            Real::exact_rational_matrix4_inverse_known_dyadic([
                [
                    &singular[0][0],
                    &singular[0][1],
                    &singular[0][2],
                    &singular[0][3]
                ],
                [
                    &singular[1][0],
                    &singular[1][1],
                    &singular[1][2],
                    &singular[1][3]
                ],
                [
                    &singular[2][0],
                    &singular[2][1],
                    &singular[2][2],
                    &singular[2][3]
                ],
                [
                    &singular[3][0],
                    &singular[3][1],
                    &singular[3][2],
                    &singular[3][3]
                ],
            ]),
            Err(Problem::DivideByZero)
        );
    }

    #[test]
    fn row_scaled_dyadic_matrix4_inverse_matches_general_exact_kernel() {
        let mut state = 0x9e37_79b9_7f4a_7c15_u64;
        for _ in 0..128 {
            let matrix: [[Real; 4]; 4] = core::array::from_fn(|_| {
                core::array::from_fn(|_| {
                    state = state
                        .wrapping_mul(6_364_136_223_846_793_005)
                        .wrapping_add(1_442_695_040_888_963_407);
                    let numerator = i64::try_from(state % 17).unwrap() - 8;
                    let denominator = 1_u64 << u32::try_from((state >> 32) % 7).unwrap();
                    Real::new(Rational::fraction(numerator, denominator).unwrap())
                })
            });
            let refs = [
                [&matrix[0][0], &matrix[0][1], &matrix[0][2], &matrix[0][3]],
                [&matrix[1][0], &matrix[1][1], &matrix[1][2], &matrix[1][3]],
                [&matrix[2][0], &matrix[2][1], &matrix[2][2], &matrix[2][3]],
                [&matrix[3][0], &matrix[3][1], &matrix[3][2], &matrix[3][3]],
            ];
            assert_eq!(
                Real::exact_rational_matrix4_inverse_known_dyadic(refs),
                Real::exact_rational_matrix4_inverse_known_exact(refs)
            );
        }
    }

    #[test]
    fn exact_rational_normalize_cancels_common_denominator() {
        let values = [
            Real::new(Rational::fraction(3, 2).unwrap()),
            Real::from(2),
            Real::zero(),
        ];
        assert_eq!(
            Real::exact_rational_normalize_known_exact([&values[0], &values[1], &values[2],])
                .unwrap(),
            [
                Real::new(Rational::fraction(3, 5).unwrap()),
                Real::new(Rational::fraction(4, 5).unwrap()),
                Real::zero(),
            ]
        );
        let zero = Real::zero();
        assert_eq!(
            Real::exact_rational_normalize_known_exact([&zero, &zero, &zero]),
            Err(Problem::DivideByZero)
        );
    }

    #[test]
    fn exact_set_facts_report_dyadic_and_shared_denominator_routes() {
        let dyadic = [
            Real::new(Rational::fraction(1, 4).unwrap()),
            Real::new(Rational::fraction(-3, 4).unwrap()),
            Real::zero(),
        ];
        let dyadic_facts = Real::exact_set_facts(dyadic.iter());
        assert_eq!(dyadic_facts.len, 3);
        assert!(dyadic_facts.is_nonempty_exact_rational());
        assert!(dyadic_facts.has_dyadic_schedule());
        assert!(!dyadic_facts.has_shared_denominator_schedule());
        assert_eq!(dyadic_facts.known_zero_count, 1);
        assert_eq!(dyadic_facts.known_nonzero_count, 2);
        assert_eq!(dyadic_facts.unknown_zero_count, 0);
        assert_eq!(dyadic_facts.known_positive_count, 1);
        assert_eq!(dyadic_facts.known_negative_count, 1);
        assert_eq!(dyadic_facts.exact_integer_count, 1);
        assert_eq!(dyadic_facts.exact_power_of_two_count, 1);
        assert_eq!(dyadic_facts.known_one_count, 0);
        assert_eq!(dyadic_facts.known_minus_one_count, 0);
        assert!(!dyadic_facts.has_integer_grid_schedule());
        assert!(!dyadic_facts.has_signed_unit_schedule());
        assert_eq!(
            dyadic_facts.sign_pattern(),
            RealExactSetSignPattern::MixedKnown
        );
        assert_eq!(
            dyadic_facts.max_dyadic_exponent_class,
            Some(RealExactSetDyadicExponentClass::Small)
        );

        let quarters = [
            Real::new(Rational::fraction(1, 4).unwrap()),
            Real::new(Rational::fraction(-3, 4).unwrap()),
        ];
        let quarter_facts = Real::exact_set_facts(quarters.iter());
        assert!(quarter_facts.has_shared_denominator_schedule());
        assert_eq!(
            quarter_facts.shared_denominator_kind(),
            Some(RealExactSetDenominatorKind::Dyadic)
        );
        assert_eq!(
            quarter_facts.max_rational_storage,
            Some(RationalStorageClass::WordSized)
        );
        assert_eq!(
            quarter_facts.max_dyadic_exponent_class,
            Some(RealExactSetDyadicExponentClass::Small)
        );

        let integers = [Real::from(7_i32), Real::from(-11_i32), Real::zero()];
        let integer_facts = Real::exact_set_facts(integers.iter());
        assert_eq!(integer_facts.exact_integer_count, 3);
        assert!(integer_facts.has_integer_grid_schedule());
        assert_eq!(
            integer_facts.sign_pattern(),
            RealExactSetSignPattern::MixedKnown
        );
        assert_eq!(
            integer_facts.max_dyadic_exponent_class,
            Some(RealExactSetDyadicExponentClass::Integer)
        );

        let positives = [Real::from(7_i32), Real::from(11_i32)];
        assert_eq!(
            Real::exact_set_facts(positives.iter()).sign_pattern(),
            RealExactSetSignPattern::AllPositive
        );

        let negatives = [Real::from(-7_i32), Real::from(-11_i32)];
        assert_eq!(
            Real::exact_set_facts(negatives.iter()).sign_pattern(),
            RealExactSetSignPattern::AllNegative
        );

        let zeros = [Real::zero(), Real::zero()];
        let zero_facts = Real::exact_set_facts(zeros.iter());
        assert_eq!(zero_facts.sign_pattern(), RealExactSetSignPattern::AllZero);
        assert!(zero_facts.has_signed_unit_schedule());

        let signed_units = [Real::one(), -Real::one(), Real::zero()];
        let signed_unit_facts = Real::exact_set_facts(signed_units.iter());
        assert_eq!(signed_unit_facts.known_one_count, 1);
        assert_eq!(signed_unit_facts.known_minus_one_count, 1);
        assert_eq!(signed_unit_facts.exact_power_of_two_count, 2);
        assert!(signed_unit_facts.has_integer_grid_schedule());
        assert!(signed_unit_facts.has_signed_unit_schedule());

        let thirds = [
            Real::new(Rational::fraction(1, 3).unwrap()),
            Real::new(Rational::fraction(2, 3).unwrap()),
        ];
        let third_facts = Real::exact_set_facts(thirds.iter());
        assert!(third_facts.all_exact_rational);
        assert_eq!(third_facts.exact_integer_count, 0);
        assert!(!third_facts.has_integer_grid_schedule());
        assert!(!third_facts.all_dyadic);
        assert!(third_facts.shared_denominator);
        assert_eq!(
            third_facts.shared_denominator_kind(),
            Some(RealExactSetDenominatorKind::SharedNonDyadic)
        );
        assert_eq!(third_facts.max_dyadic_exponent_class, None);

        let mixed = [Real::pi(), Real::one()];
        let mixed_facts = Real::exact_set_facts(mixed.iter());
        assert_eq!(mixed_facts.exact_rational_count, 1);
        assert_eq!(mixed_facts.known_positive_count, 2);
        assert_eq!(
            mixed_facts.sign_pattern(),
            RealExactSetSignPattern::AllPositive
        );
        assert!(!mixed_facts.all_exact_rational);
        assert!(!mixed_facts.shared_denominator);
        assert_eq!(mixed_facts.shared_denominator_kind(), None);
        assert_eq!(mixed_facts.max_dyadic_exponent_class, None);

        let unknown_sign = Real::pi() - Real::new(Rational::fraction(103_993, 33_102).unwrap());
        let exact_one = Real::one();
        let uncertain = [&unknown_sign, &exact_one];
        assert_eq!(
            RealExactSetFacts::from_reals(uncertain).sign_pattern(),
            RealExactSetSignPattern::Unknown
        );

        let empty: [&Real; 0] = [];
        assert_eq!(
            RealExactSetFacts::from_reals(empty).sign_pattern(),
            RealExactSetSignPattern::Empty
        );
    }

    #[test]
    fn exact_rational_reuse_evidence_distinguishes_isolated_and_shared_storage() {
        let value = Real::new(Rational::fraction(1_000_003, 1_000_033).unwrap());
        assert_eq!(value.exact_rational_reuse_evidence(), Some(false));
        assert_eq!(value.exact_rational_reuse_evidence(), Some(true));

        let shared = value.clone();
        assert_eq!(value.exact_rational_reuse_evidence(), Some(true));
        assert_eq!(shared.exact_rational_reuse_evidence(), Some(true));
        assert_eq!(Real::pi().exact_rational_reuse_evidence(), None);
    }

    #[test]
    fn signed_product_sum_preserves_mixed_symbolic_products() {
        let pi = Real::pi();
        let e = Real::e();
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        let third = Real::new(Rational::fraction(1, 3).unwrap());
        let neg_five = Real::from(-5_i32);
        let zero = Real::zero();

        let actual = Real::signed_product_sum(
            [true, false, true],
            [[&pi, &half, &e], [&e, &third, &pi], [&zero, &neg_five, &pi]],
        );
        let expected = &(&pi * &half * &e) - &(&e * &third * &pi) + &(&zero * &neg_five * &pi);

        assert!((actual.to_f64_lossy().unwrap() - expected.to_f64_lossy().unwrap()).abs() < 1e-12);
    }

    #[test]
    fn exact_normal_form_cancels_shared_opaque_affine_atoms() {
        let atom = (Real::from(2_i32).sqrt().unwrap() + Real::one()).sin();
        let x = &atom - Real::one();
        let y = &atom + Real::from(2_i32);
        let determinant = Real::diff_of_products(&Real::one(), &y, &Real::one(), &x);

        assert_eq!(
            determinant.exact_rational_normal_form(),
            Some(Rational::new(3)),
        );
    }

    #[test]
    fn dot_products_handle_mixed_symbolic_structural_terms() {
        let left = [
            Real::one(),
            Real::zero(),
            Real::from(2_i32),
            Real::pi() * Real::new(Rational::fraction(5, 7).unwrap()),
        ];
        let right = [
            Real::pi(),
            Real::e(),
            Real::e() * Real::new(Rational::fraction(3, 5).unwrap()),
            Real::zero(),
        ];
        let expected = &(&left[0] * &right[0])
            + &(&left[1] * &right[1])
            + &(&left[2] * &right[2])
            + &(&left[3] * &right[3]);

        let actual = Real::dot4_refs(
            [&left[0], &left[1], &left[2], &left[3]],
            [&right[0], &right[1], &right[2], &right[3]],
        );
        assert!((actual.to_f64_lossy().unwrap() - expected.to_f64_lossy().unwrap()).abs() < 1e-12);

        let expected = &(&left[0] * &right[0]) + &(&left[1] * &right[1]) + &(&left[2] * &right[2]);
        let actual = Real::dot3_refs(
            [&left[0], &left[1], &left[2]],
            [&right[0], &right[1], &right[2]],
        );
        assert!((actual.to_f64_lossy().unwrap() - expected.to_f64_lossy().unwrap()).abs() < 1e-12);

        let expected = &(&left[0] * &right[0]) + &(&left[1] * &right[1]);
        let actual = Real::dot2_refs([&left[0], &left[1]], [&right[0], &right[1]]);
        assert!((actual.to_f64_lossy().unwrap() - expected.to_f64_lossy().unwrap()).abs() < 1e-12);
    }

    #[test]
    fn active_dot3_retains_symbolic_by_rational_linear_combination() {
        let symbolic = [Real::pi(), Real::e(), Real::from(2_i32).sqrt().unwrap()];
        let rational = [
            Real::new(Rational::fraction(5, 7).unwrap()),
            Real::new(Rational::fraction(-11, 13).unwrap()),
            Real::new(Rational::fraction(17, 19).unwrap()),
        ];
        let expected = &(&symbolic[0] * &rational[0])
            + &(&symbolic[1] * &rational[1])
            + &(&symbolic[2] * &rational[2]);
        let expected_approximation = expected.to_f64_lossy().unwrap();

        for actual in [
            Real::active_dot3_refs(
                [&symbolic[0], &symbolic[1], &symbolic[2]],
                [&rational[0], &rational[1], &rational[2]],
            ),
            Real::active_dot3_refs(
                [&rational[0], &rational[1], &rational[2]],
                [&symbolic[0], &symbolic[1], &symbolic[2]],
            ),
        ] {
            assert!((actual.to_f64_lossy().unwrap() - expected_approximation).abs() < 1e-12);
        }
    }

    #[test]
    fn dot2_refs_matches_pairwise_rational_arithmetic() {
        let left = [
            &Real::new(Rational::fraction(6, 5).unwrap()),
            &Real::new(Rational::fraction(-7, 10).unwrap()),
        ];
        let right = [
            &Real::new(Rational::fraction(-4, 5).unwrap()),
            &Real::new(Rational::fraction(11, 10).unwrap()),
        ];
        let expected = &(left[0] * right[0]) + &(left[1] * right[1]);
        assert_eq!(Real::dot2_refs(left, right), expected);
    }

    #[test]
    fn atan2_origin_returns_zero() {
        assert_eq!(Real::zero().atan2(Real::zero()), Real::zero());
    }

    #[test]
    fn atan2_positive_x_axis_is_zero() {
        assert_eq!(Real::zero().atan2(Real::from(3_i32)), Real::zero());
    }

    #[test]
    fn atan2_negative_x_axis_is_pi() {
        assert_eq!(Real::zero().atan2(Real::from(-5_i32)), Real::pi());
    }

    #[test]
    fn atan2_positive_y_axis_is_half_pi() {
        assert_eq!(
            Real::from(7_i32).atan2(Real::zero()),
            (Real::pi() / Real::from(2_i32)).unwrap(),
        );
    }

    #[test]
    fn atan2_negative_y_axis_is_minus_half_pi() {
        assert_eq!(
            Real::from(-9_i32).atan2(Real::zero()),
            -(Real::pi() / Real::from(2_i32)).unwrap(),
        );
    }

    #[test]
    fn atan2_quadrant_one_uses_atan_special_form() {
        // atan2(1, 1) = pi/4 exactly via Real::atan's exact special form.
        assert_eq!(
            Real::one().atan2(Real::one()),
            (Real::pi() / Real::from(4_i32)).unwrap(),
        );
    }

    #[test]
    fn atan2_quadrant_two_uses_atan_plus_pi() {
        assert_eq!(
            Real::one().atan2(-Real::one()),
            Real::pi() * Real::new(Rational::fraction(3, 4).unwrap()),
        );
    }

    #[test]
    fn atan2_quadrant_three_uses_atan_minus_pi() {
        assert_eq!(
            (-Real::one()).atan2(-Real::one()),
            Real::pi() * Real::new(Rational::fraction(-3, 4).unwrap()),
        );
    }

    #[test]
    fn atan2_quadrant_four_uses_negative_atan() {
        assert_eq!(
            (-Real::one()).atan2(Real::one()),
            (Real::pi() / Real::from(-4_i32)).unwrap(),
        );
    }

    #[test]
    fn integer_pi_shifted_atan_trig_replays_exactly() {
        let root_thirteen = Real::from(13_i32).sqrt().unwrap();

        for pi_multiple in -2_i32..=2 {
            let parity = if pi_multiple.rem_euclid(2) == 0 {
                1
            } else {
                -1
            };
            for atan_orientation in [-1_i32, 1] {
                for argument_numerator in [-2_i32, 2] {
                    let argument = (Real::from(argument_numerator) / Real::from(3_i32)).unwrap();
                    let angle = Real::from(pi_multiple) * Real::pi()
                        + Real::from(atan_orientation) * argument.atan().unwrap();
                    let sine_numerator = parity * atan_orientation * argument_numerator;
                    let cosine_numerator = parity * 3;

                    assert_eq!(
                        angle.clone().sin(),
                        (Real::from(sine_numerator) / root_thirteen.clone()).unwrap(),
                        "pi_multiple={pi_multiple}, atan_orientation={atan_orientation}, \
                         argument_numerator={argument_numerator}",
                    );
                    assert_eq!(
                        angle.cos(),
                        (Real::from(cosine_numerator) / root_thirteen.clone()).unwrap(),
                        "pi_multiple={pi_multiple}, atan_orientation={atan_orientation}, \
                         argument_numerator={argument_numerator}",
                    );
                }
            }
        }

        let argument = (Real::one() / Real::from(3_i32)).unwrap();
        let double_angle = Real::from(2_i32) * argument.atan().unwrap();
        let expected_sine = Real::new(Rational::fraction(3, 5).unwrap());
        let expected_cosine = Real::new(Rational::fraction(4, 5).unwrap());
        for difference in [
            double_angle.clone().sin() - expected_sine,
            double_angle.cos() - expected_cosine,
        ] {
            let [lower, upper] = difference.certified_dyadic_interval(-96).unwrap();
            assert!(lower <= Rational::zero());
            assert!(upper >= Rational::zero());
        }
    }

    #[test]
    fn nested_half_pi_reduction_preserves_rational_atan_trig() {
        let argument = Real::new(Rational::fraction(3, 4).unwrap());
        let angle = Real::pi() - argument.atan().unwrap();
        let half_pi = (Real::pi() / Real::from(2)).unwrap();
        let reduced = angle - &half_pi - half_pi;

        assert_eq!(
            reduced.clone().sin(),
            Real::new(Rational::fraction(-3, 5).unwrap())
        );
        assert_eq!(reduced.cos(), Real::new(Rational::fraction(4, 5).unwrap()));
    }

    #[test]
    fn atan2_sqrt_three_anchor_matches_pi_third() {
        // atan2(sqrt(3), 1) = pi/3 exactly via Real::atan's sqrt(3) anchor.
        let sqrt_three = Real::from(3_i32).sqrt().unwrap();
        assert_eq!(
            sqrt_three.atan2(Real::one()),
            (Real::pi() / Real::from(3_i32)).unwrap(),
        );
    }

    #[test]
    fn trig_retains_atan_after_pi_reciprocal_cancellation() {
        let root_fifteen = Real::from(15).sqrt().unwrap();
        let angle = root_fifteen.clone().atan().unwrap();
        let normalized = (&angle / Real::pi()).unwrap();
        let replayed = Real::pi() * normalized;

        assert_eq!(
            replayed.clone().sin(),
            (root_fifteen.clone() / Real::from(4)).unwrap()
        );
        assert_eq!(replayed.cos(), (Real::from(1) / Real::from(4)).unwrap());

        let complementary = (Real::pi() / Real::from(2)).unwrap()
            - (Real::one() / root_fifteen.clone())
                .unwrap()
                .atan()
                .unwrap();
        assert_eq!(angle, complementary);
        let inverse_angle = (Real::one() / root_fifteen.clone())
            .unwrap()
            .atan()
            .unwrap();
        let expanded = ((Real::pi() - Real::from(2) * inverse_angle) / Real::from(2)).unwrap();
        assert_eq!(
            angle.certified_cmp_until(&expanded, -512).ordering(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            complementary.clone().sin(),
            (root_fifteen.clone() / Real::from(4)).unwrap()
        );
        assert_eq!(
            complementary.cos(),
            (Real::from(1) / Real::from(4)).unwrap()
        );
    }

    #[test]
    fn atan2_generic_quadrants_match_f64() {
        // Coords chosen so |y/x| lands in working atan kernel paths
        // (unit fraction or integer >= 2). atan_rational has a pre-existing
        // bug for rationals in (1/2, 1) with numerator > 1, intentionally
        // avoided here so the quadrant logic is what's tested.
        let cases: [(i32, i32); 8] = [
            (1, 2),
            (-1, 2),
            (1, -2),
            (-1, -2),
            (3, 1),
            (-3, 1),
            (3, -1),
            (-3, -1),
        ];
        for (y, x) in cases {
            let y_real = Real::from(y);
            let x_real = Real::from(x);
            let got: f64 = y_real.atan2(x_real).into();
            let want = (y as f64).atan2(x as f64);
            assert!(
                (got - want).abs() < 1e-12,
                "atan2({y}, {x}): got {got}, want {want}",
            );
        }
    }

    #[test]
    fn atan2_shared_cancellation_resolves_positive_y_below_refinement_floor() {
        let tiny = Real::new(
            Rational::from_bigint_fraction(
                num::BigInt::from(1_u8),
                num::BigUint::from(1_u8) << 2500,
            )
            .unwrap(),
        );
        let y = (Real::pi() + tiny.clone()) - Real::pi();
        assert_eq!(y.structural_facts().sign, Some(RealSign::Positive));

        let got = y.atan2(Real::one());
        let expected = tiny.atan2(Real::one());
        assert_ne!(got, Real::zero());
        assert_eq!(got.to_f64_lossy(), expected.to_f64_lossy());
    }

    #[test]
    fn atan2_is_consistent_under_uniform_positive_scaling() {
        // atan2(ky, kx) = atan2(y, x) for k > 0. Pick coords whose |y/x|
        // ratio (1/3 here) lands in the working atan kernel range.
        let y = Real::from(1_i32);
        let x = Real::from(-3_i32);
        let scale = Real::from(11_i32);
        let unscaled: f64 = y.clone().atan2(x.clone()).into();
        let scaled: f64 = (y * scale.clone()).atan2(x * scale).into();
        assert!((unscaled - scaled).abs() < 1e-12);
    }

    #[test]
    fn rational_atan2_axes_and_origin() {
        assert_eq!(Real::zero().atan2(Real::zero()), Real::zero());
        assert_eq!(Real::zero().atan2(Real::from(2)), Real::zero());
        assert_eq!(Real::zero().atan2(Real::from(-2)), Real::pi());
    }

    #[test]
    fn dot2_refs_handles_symbolic_lanes() {
        let left = [Real::pi(), Real::e()];
        let right = [Real::e(), Real::pi()];
        let expected = &(&left[0] * &right[0]) + &(&left[1] * &right[1]);
        let actual = Real::dot2_refs([&left[0], &left[1]], [&right[0], &right[1]]);
        assert!((actual.to_f64_lossy().unwrap() - expected.to_f64_lossy().unwrap()).abs() < 1e-12);
        assert_eq!(
            Real::dot2_refs([&left[0], &left[1]], [&right[0], &right[1]]),
            expected,
        );
    }

    #[test]
    fn dot2_refs_zero_lane_shortcut() {
        let left = [Real::zero(), Real::from(3_i32)];
        let right = [Real::pi(), Real::e()];
        let expected = &left[1] * &right[1];
        let actual = Real::dot2_refs([&left[0], &left[1]], [&right[0], &right[1]]);
        assert!((actual.to_f64_lossy().unwrap() - expected.to_f64_lossy().unwrap()).abs() < 1e-12)
    }

    #[test]
    fn computable_atan2_axes() {
        use crate::Computable;
        use num::Zero;
        // Axis cases are validated through approximations because these values
        // exercise the symbolic zero branches in the computable kernel.
        let zero_plus = Computable::zero().atan2(Computable::one());
        assert!(zero_plus.approx(-30).is_zero());
        let zero_minus = Computable::zero().atan2(Computable::one().negate());
        assert_eq!(zero_minus.approx(-30), Computable::pi().approx(-30));
        let plus_y = Computable::one().atan2(Computable::zero());
        let half_pi = Computable::pi().multiply(Computable::one().add(Computable::one()).inverse());
        assert_eq!(plus_y.approx(-30), half_pi.approx(-30));
    }

    #[test]
    fn computable_atan2_quadrants_match_f64() {
        use crate::Computable;
        use num::ToPrimitive;
        let cases: [(i64, i64); 4] = [(1, 2), (-1, 2), (1, -2), (-1, -2)];
        for (y, x) in cases {
            let y_c = Computable::rational(Rational::new(y));
            let x_c = Computable::rational(Rational::new(x));
            // approx returns a BigInt scaled by 2^p; using p=-60 buys ~18 decimal digits.
            let scaled = y_c.atan2(x_c).approx(-60);
            let got_f = scaled.to_f64().expect("BigInt fits in f64") * 2_f64.powi(-60);
            let want = (y as f64).atan2(x as f64);
            assert!(
                (got_f - want).abs() < 1e-12,
                "computable atan2({y}, {x}): got {got_f}, want {want}",
            );
        }
    }

    #[test]
    fn computable_atan2_shared_cancellation_resolves_positive_y() {
        use crate::Computable;
        use num::Zero;

        let tiny = Rational::from_bigint_fraction(
            num::BigInt::from(1_u8),
            num::BigUint::from(1_u8) << 2500,
        )
        .unwrap();
        let y = Computable::pi()
            .add(Computable::rational(tiny.clone()))
            .add(Computable::pi().negate());
        assert_eq!(y.sign_until(0), Some(RealSign::Positive));

        let got = y.atan2(Computable::one()).approx(-2600);
        let expected = Computable::rational(tiny)
            .atan2(Computable::one())
            .approx(-2600);
        assert!(!got.is_zero());
        assert_eq!(got, expected);
    }

    #[test]
    fn computable_atan2_shared_cancellation_resolves_negative_y() {
        use crate::Computable;

        let tiny = Rational::from_bigint_fraction(
            num::BigInt::from(1_u8),
            num::BigUint::from(1_u8) << 2500,
        )
        .unwrap();
        let y = Computable::pi()
            .add(Computable::rational(tiny.clone()).negate())
            .add(Computable::pi().negate());
        assert_eq!(y.sign_until(0), Some(RealSign::Negative));

        let got = y.atan2(Computable::one().negate()).approx(-2600);
        let expected = Computable::rational(tiny)
            .negate()
            .atan2(Computable::one().negate())
            .approx(-2600);
        assert_eq!(got, expected);
    }

    #[test]
    fn unresolved_opaque_sign_is_not_treated_as_exact_zero() {
        use crate::Computable;

        let tiny = Rational::from_bigint_fraction(
            num::BigInt::from(1_u8),
            num::BigUint::from(1_u8) << 5000,
        )
        .unwrap();
        // Keep the positive difference opaque: exact affine cancellation of
        // shared `pi` terms is intentionally recognized by the constructor.
        let left = Computable::pi()
            .add(Computable::rational(&tiny + &tiny))
            .exp();
        let right = Computable::pi().add(Computable::rational(tiny)).exp();
        let opaque_positive = left.add(right.negate());

        assert_eq!(opaque_positive.sign_until(-4096), None);
        assert_eq!(opaque_positive.sign_until(-2000), None);
        assert_eq!(
            opaque_positive.try_compare_to_until(&Computable::zero(), -4096),
            None
        );
        assert!(
            opaque_positive
                .clone()
                .try_atan2_until(Computable::one().negate(), -4096)
                .is_none()
        );

        let value = Real::irrational_from_computable(opaque_positive);
        assert_eq!(f64::from(value.clone()), 0.0);
        assert_eq!(value.best_sign(), num::bigint::Sign::NoSign);
        assert_eq!(value.clone().sqrt(), Err(Problem::Exhausted));
        assert_eq!(value.clone().inverse(), Err(Problem::UnknownZero));
        assert_eq!(value.clone().cot(), Err(Problem::UnknownZero));
        assert_eq!(&Real::one() / &value, Err(Problem::UnknownZero));
        assert_eq!(&value / &value, Err(Problem::UnknownZero));
        assert_eq!(
            value.clone().powi(num::BigInt::from(0_u8)),
            Err(Problem::UnknownZero)
        );
        assert_eq!(
            (Real::one() + value.clone()).acos(),
            Err(Problem::Exhausted)
        );
        assert_eq!(value.try_atan2(-Real::one()), Err(Problem::Exhausted));
    }

    #[test]
    fn retained_classes_conservatively_certify_irrationality() {
        use crate::Computable;

        let third = Rational::fraction(1, 3).unwrap();
        let fifth = Rational::fraction(1, 5).unwrap();
        let sqrt_two = Real::from(2_i32).sqrt().unwrap();
        let ln_two = Real::from(2_i32).ln().unwrap();
        let known = [
            Real::pi(),
            Real::pi() * Real::pi(),
            Real::pi().inverse().unwrap(),
            sqrt_two.clone(),
            Real::e(),
            ln_two.clone(),
            Real::pi() + Real::from(3_i32),
            Real::e() + Real::from(2_i32),
            Real::from(3_i32).log2().unwrap(),
            Real::from(2_i32).log10().unwrap(),
            Real::new(third.clone()).exp2().unwrap(),
            Real::new(third).exp10().unwrap(),
            Real::new(fifth.clone()).sin_pi(),
            Real::new(fifth).tan_pi().unwrap(),
            Real::pi() * sqrt_two.clone(),
            Real::e() * sqrt_two,
        ];
        let exact_rational = Real::from(1_234_i32);
        for value in &known {
            assert!(value.definitely_irrational());
            assert_eq!(
                value.certified_eq_until(&exact_rational, 0),
                CertifiedRealEquality::NotEqual {
                    certificate: RealEqualityCertificate::StructuralFacts,
                }
            );
        }

        let unresolved = [
            Real::from(7_i32),
            Real::pi() * Real::e(),
            ln_two * Real::from(3_i32).ln().unwrap(),
            Real::pi() * Real::e() * Real::from(2_i32).sqrt().unwrap(),
            Real::irrational_from_computable(Computable::pi()),
            Real::zero() * Real::pi(),
        ];
        assert!(
            unresolved
                .iter()
                .all(|value| !value.definitely_irrational())
        );
    }

    #[test]
    fn certified_equality_separates_irrational_from_nearby_rational() {
        let pi = Real::pi();
        let bits = 4_096_usize;
        let nearby = Rational::from_bigint_fraction(
            pi.fold_ref().approx(-(bits as i32)),
            num::BigUint::from(1_u8) << bits,
        )
        .unwrap();
        let nearby = Real::new(nearby);

        assert_eq!(
            pi.certified_eq_until(&nearby, 0),
            CertifiedRealEquality::NotEqual {
                certificate: RealEqualityCertificate::StructuralFacts,
            }
        );
        assert_eq!(
            nearby.certified_eq_until(&pi, 0),
            CertifiedRealEquality::NotEqual {
                certificate: RealEqualityCertificate::StructuralFacts,
            }
        );
    }

    #[test]
    fn dot2_refs_all_zero_returns_zero() {
        let left = [Real::zero(), Real::zero()];
        let right = [Real::pi(), Real::e()];
        assert_eq!(
            Real::dot2_refs([&left[0], &left[1]], [&right[0], &right[1]]),
            Real::zero(),
        );
    }

    #[test]
    fn active_dot2_refs_matches_dot2_refs_when_all_lanes_active() {
        let left = [
            Real::pi(),
            Real::e() * Real::new(Rational::fraction(3, 5).unwrap()),
        ];
        let right = [
            Real::e() * Real::new(Rational::fraction(2, 7).unwrap()),
            Real::pi(),
        ];
        let expected = Real::dot2_refs([&left[0], &left[1]], [&right[0], &right[1]]);
        let actual = Real::active_dot2_refs([&left[0], &left[1]], [&right[0], &right[1]]);
        assert!((actual.to_f64_lossy().unwrap() - expected.to_f64_lossy().unwrap()).abs() < 1e-12);
    }

    #[test]
    fn signed_product_sum_certifies_complementary_sin_pi_squares() {
        let angle = Rational::fraction(1, 1_800_000).unwrap();
        let sine = Real::new(angle.clone()).sin_pi();
        let cosine = Real::new(Rational::fraction(1, 2).unwrap() - angle).sin_pi();

        assert_eq!(
            Real::signed_product_sum([true, true], [[&sine, &sine], [&cosine, &cosine]],),
            Real::one()
        );
    }

    #[test]
    fn signed_product_sum_certifies_orthonormal_trig_zero_polynomial() {
        let angle = Rational::fraction(1, 1_800_000).unwrap();
        let sine = Real::new(angle.clone()).sin_pi();
        let cosine = Real::new(Rational::fraction(1, 2).unwrap() - angle).sin_pi();
        let one = Real::one();

        assert_eq!(
            Real::signed_product_sum(
                [true, true, false],
                [[&sine, &sine], [&cosine, &cosine], [&one, &one],],
            ),
            Real::zero()
        );
    }

    #[test]
    fn rational_offsets_retain_strict_sin_pi_endpoint_signs() {
        let angle = Rational::fraction(899_999, 1_800_000).unwrap();
        let sine = Real::new(angle).sin_pi();

        assert_eq!(
            (sine.clone() - Real::one()).structural_facts().sign,
            Some(RealSign::Negative)
        );
        assert_eq!(
            (Real::one() - sine).structural_facts().sign,
            Some(RealSign::Positive)
        );
    }
}
