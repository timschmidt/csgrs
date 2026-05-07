#[cfg(test)]
mod tests {
    use super::super::curve;
    use crate::{
        MagnitudeBits, Problem, Rational, Real, RealSign, RealStructuralFacts, ZeroKnowledge,
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

    // https://devblogs.microsoft.com/oldnewthing/?p=93765
    // "Why does the Windows calculator generate tiny errors when calculating the square root of a
    // perfect square?" (fixed in 2018)
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
        assert_eq!(pi_fraction(1, 1).cos(), minus_one);
        assert_eq!(pi_fraction(3, 2).cos(), zero);
        assert_eq!(pi_fraction(-1, 3).cos(), half);
        assert_eq!(pi_fraction(2, 1).cos(), one);
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

        let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
        assert_eq!(sqrt_two.exact_rational(), None);

        let exp_ln_8 = Real::new(Rational::new(8)).ln().unwrap().exp().unwrap();
        assert_eq!(exp_ln_8.exact_rational(), Some(Rational::new(8)));
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
    fn powi() {
        let base: Real = 4.into();
        let five_over_two: Real = "5/2".parse().unwrap();
        let answer = base.pow(five_over_two).unwrap();
        let correct: Real = 32.into();
        assert_eq!(answer, correct);
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

        assert!(closest_f64(
            (Real::new(Rational::fraction(1, 2).unwrap())
                * Real::new(Rational::new(2)).sqrt().unwrap())
            .atanh()
            .unwrap(),
            0.881373587019543
        ));
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

    fn assert_close(value: Real, expected: f64, tolerance: f64) {
        let actual: f64 = value.into();
        let scale = expected.abs().max(1.0);
        assert!(
            (actual - expected).abs() <= tolerance * scale,
            "actual {actual}, expected {expected}, tolerance {tolerance}"
        );
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
}
