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
    fn real_refine_sign_until_handles_refined_and_unresolved_cases() {
        let tiny = Real::new(
            Rational::from_bigint_fraction(num::BigInt::from(1), num::BigUint::from(1_u8) << 64)
                .unwrap(),
        );
        let near_pi = Real::pi() - tiny;
        assert_eq!(near_pi.refine_sign_until(-8), Some(RealSign::Positive));

        let unresolved = Real::pi() - Real::new(Rational::new(3));
        assert_eq!(unresolved.refine_sign_until(0), None);
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
            0.29595896909330400696886606953617752145
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
    fn integer_logs() {
        for (n, log) in [
            (1, 0),
            (10, 1),
            (10_000_000_000_000_000, 16),
            (100_000_000_000_000_000, 17),
            (1000_000_000_000_000_000, 18),
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
            Real::new(Rational::new(2)).acosh().unwrap(),
            1.3169578969248166
        ));
        assert!(closest_f64(
            Real::new(Rational::fraction(3, 10).unwrap())
                .atanh()
                .unwrap(),
            0.3095196042031117
        ));
    }
}
