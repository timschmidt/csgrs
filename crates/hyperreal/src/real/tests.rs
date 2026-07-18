#[cfg(test)]
#[allow(clippy::module_inception)]
mod tests {
    use crate::real::arithmetic::curve;
    use crate::{
        CertifiedRealEquality, CertifiedRealOrdering, CertifiedRealSign, DomainStatus,
        ExpressionDegree, MagnitudeBits, PrimitiveFloatStatus, Problem, Rational,
        RationalStorageClass, Real, RealEqualityCertificate, RealExactSetDenominatorKind,
        RealExactSetDyadicExponentClass, RealExactSetFacts, RealExactSetSignPattern,
        RealOrderingCertificate, RealSign, RealSignCertificate, RealStructuralFacts,
        StructuralComparison, StructuralKind, SymbolicDependencyMask, ZeroKnowledge,
        ZeroOneMinusOneStatus,
    };

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
    fn certified_eq_until_refines_nearby_values_or_reports_unknown() {
        let near_pi = Real::new(Rational::fraction(103_993, 33_102).unwrap());

        assert_eq!(
            Real::pi().certified_eq_until(&near_pi, 0),
            CertifiedRealEquality::Unknown { min_precision: 0 }
        );
        assert_eq!(
            Real::pi().certified_eq_until(&near_pi, -64),
            CertifiedRealEquality::NotEqual {
                certificate: RealEqualityCertificate::BoundedRefinement { min_precision: -64 },
            }
        );
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
        assert!(closest_f64(answer, -3.7129299999999996));
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
            1.266103672779499
        ));
        assert!(closest_f64(
            Real::new(Rational::new(2)).atan().unwrap(),
            1.1071487177940904
        ));
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
            1.3169578969248166
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
            -0.5493061443340548
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
    fn atan2_sqrt_three_anchor_matches_pi_third() {
        // atan2(sqrt(3), 1) = pi/3 exactly via Real::atan's sqrt(3) anchor.
        let sqrt_three = Real::from(3_i32).sqrt().unwrap();
        assert_eq!(
            sqrt_three.atan2(Real::one()),
            (Real::pi() / Real::from(3_i32)).unwrap(),
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
    fn atan2_unresolved_positive_y_does_not_collapse_to_axis() {
        let tiny = Real::new(
            Rational::from_bigint_fraction(
                num::BigInt::from(1_u8),
                num::BigUint::from(1_u8) << 2500,
            )
            .unwrap(),
        );
        let y = (Real::pi() + tiny.clone()) - Real::pi();
        assert_eq!(y.structural_facts().sign, None);

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
    fn computable_atan2_unresolved_positive_y_does_not_collapse_to_axis() {
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
        assert_eq!(y.sign(), num::bigint::Sign::NoSign);

        let got = y.atan2(Computable::one()).approx(-2600);
        let expected = Computable::rational(tiny)
            .atan2(Computable::one())
            .approx(-2600);
        assert!(!got.is_zero());
        assert_eq!(got, expected);
    }

    #[test]
    fn computable_atan2_unresolved_negative_y_on_negative_x_keeps_lower_branch() {
        use crate::Computable;

        let tiny = Rational::from_bigint_fraction(
            num::BigInt::from(1_u8),
            num::BigUint::from(1_u8) << 2500,
        )
        .unwrap();
        let y = Computable::pi()
            .add(Computable::rational(tiny.clone()).negate())
            .add(Computable::pi().negate());
        assert_eq!(y.sign(), num::bigint::Sign::NoSign);

        let got = y.atan2(Computable::one().negate()).approx(-2600);
        let expected = Computable::rational(tiny)
            .negate()
            .atan2(Computable::one().negate())
            .approx(-2600);
        assert_eq!(got, expected);
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
}
