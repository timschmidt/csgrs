use hyperreal::{Computable, Problem, Rational, Real};
use num::{BigInt, Signed};
use rug::{Float, Integer, Rational as RugRational, float::Round};

fn rational(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn finite(value: &Real, context: &str) -> f64 {
    value
        .to_f64_lossy()
        .unwrap_or_else(|| panic!("{context} must have a finite approximation"))
}

fn assert_close(actual: Real, expected: f64, context: &str) {
    let borrowed = finite(&actual, context);
    let owned: f64 = actual.into();
    let tolerance = 2.0e-11 * expected.abs().max(1.0);
    assert!(
        (owned - expected).abs() <= tolerance,
        "{context} owned: actual={owned:.17e}, expected={expected:.17e}, tolerance={tolerance:.3e}"
    );
    assert!(
        (borrowed - expected).abs() <= tolerance,
        "{context} borrowed: actual={borrowed:.17e}, expected={expected:.17e}, tolerance={tolerance:.3e}"
    );
}

#[test]
fn core2_zimmermann_exp_inputs_match_mpfr_at_256_bits() {
    for input in [
        "0.09407822313572878",
        "0.0000000000000009999999999999995",
        "0.5091077534282133",
        "0.7906867968553504",
        "0.001548443067391468",
        "0.2953379504777270",
        "0.000000006581539478341669",
        "0.00000002662858264545929",
        "0.00000003639588333766983",
    ] {
        let exact = input.parse::<Rational>().expect("exact decimal input");
        let actual = Computable::rational(exact).exp().approx(-256);

        let parsed = Float::parse(input).expect("MPFR decimal input");
        let mut oracle = Float::with_val(512, parsed).exp();
        oracle <<= 256;
        let expected = oracle
            .to_integer_round(Round::Nearest)
            .expect("finite MPFR exponential")
            .0
            .to_string()
            .parse::<BigInt>()
            .expect("MPFR integer parses as BigInt");
        let error = (actual - expected).abs();
        assert!(
            error <= BigInt::from(1),
            "exp({input}) differs from MPFR by {error} ulps"
        );
    }
}

#[test]
fn core_compare_nested_radical_identity_is_certified_exactly() {
    for (x, y) in [
        (Rational::new(1_234_567_890), Rational::new(9_876_543_210)),
        (
            Rational::fraction(1_000_003, 1_000_033).unwrap(),
            Rational::fraction(1_000_037, 1_000_081).unwrap(),
        ),
        (
            Rational::fraction(3, 5).unwrap(),
            Rational::fraction(7, 11).unwrap(),
        ),
    ] {
        let x = Real::new(x);
        let y = Real::new(y);
        let expanded = x.clone().sqrt().unwrap() + y.clone().sqrt().unwrap();
        let nested = (x.clone() + y.clone() + Real::from(2_i32) * (x * y).sqrt().unwrap())
            .sqrt()
            .unwrap();

        assert_eq!(
            expanded
                .certified_eq_until(&nested, Real::PARTIAL_CMP_MIN_PRECISION)
                .as_bool(),
            Some(true)
        );
    }
}

#[test]
fn core_heron_needle_triangles_match_exact_kahan_rearrangement_and_mpfr() {
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum AreaStatus {
        Positive,
        Zero,
        Invalid,
    }

    for (case, a, b, c, expected_status) in [
        (1, "10", "10", "10", AreaStatus::Positive),
        (2, "-3", "5", "2", AreaStatus::Invalid),
        (3, "100000", "99999.99979", "0.00029", AreaStatus::Positive),
        (4, "100000", "100000", "1.00005", AreaStatus::Positive),
        (
            5,
            "99999.99996",
            "99999.99994",
            "0.00003",
            AreaStatus::Positive,
        ),
        (
            6,
            "99999.99996",
            "0.00003",
            "99999.99994",
            AreaStatus::Positive,
        ),
        (7, "10000", "5000.000001", "15000", AreaStatus::Positive),
        (
            8,
            "99999.99999",
            "99999.99999",
            "200000",
            AreaStatus::Invalid,
        ),
        (
            9,
            "5278.64055",
            "94721.35941",
            "99999.99996",
            AreaStatus::Zero,
        ),
        (10, "100002", "100002", "200004", AreaStatus::Zero),
        (
            11,
            "31622.77662",
            "0.000023",
            "31622.77661",
            AreaStatus::Positive,
        ),
        (
            12,
            "31622.77662",
            "0.0155555",
            "31622.77661",
            AreaStatus::Positive,
        ),
    ] {
        let mut sides = [a, b, c].map(|value| value.parse::<Rational>().unwrap());
        let semiperimeter = (&sides[0] + &sides[1] + &sides[2]) / Rational::new(2);
        let invalid = sides
            .iter()
            .any(|side| side.sign() == num::bigint::Sign::Minus || semiperimeter < *side);
        let direct = &semiperimeter
            * (&semiperimeter - &sides[0])
            * (&semiperimeter - &sides[1])
            * (&semiperimeter - &sides[2]);

        sides.sort_by(|left, right| right.partial_cmp(left).unwrap());
        let [largest, middle, smallest] = sides;
        let largest_minus_middle = &largest - &middle;
        let stable_product = (&largest + (&middle + &smallest))
            * (&smallest - &largest_minus_middle)
            * (&smallest + &largest_minus_middle)
            * (&largest + (&middle - &smallest));
        let stable = stable_product / Rational::new(16);
        assert_eq!(direct, stable, "Heron rearrangement case {case}");

        let status = match (invalid, direct.sign()) {
            (true, _) | (false, num::bigint::Sign::Minus) => AreaStatus::Invalid,
            (false, num::bigint::Sign::NoSign) => AreaStatus::Zero,
            (false, num::bigint::Sign::Plus) => AreaStatus::Positive,
        };
        assert_eq!(status, expected_status, "Heron classification case {case}");

        match status {
            AreaStatus::Invalid => {
                // A negative side can make one Heron factor zero while another
                // is negative (case 2), so the geometry-domain check must not
                // be inferred from the radicand's sign alone.
                if direct.sign() == num::bigint::Sign::Minus {
                    assert_eq!(Real::new(direct).sqrt(), Err(Problem::SqrtNegative));
                }
            }
            AreaStatus::Zero => {
                assert_eq!(Real::new(direct).sqrt().unwrap(), Real::zero());
            }
            AreaStatus::Positive => {
                let actual = Computable::rational(direct.clone()).sqrt().approx(-256);
                let oracle_numerator = direct.numerator().to_string().parse::<Integer>().unwrap();
                let oracle_denominator =
                    direct.denominator().to_string().parse::<Integer>().unwrap();
                let oracle_rational = RugRational::from((oracle_numerator, oracle_denominator));
                let mut oracle = Float::with_val(512, oracle_rational).sqrt();
                oracle <<= 256;
                let expected = oracle
                    .to_integer_round(Round::Nearest)
                    .expect("positive finite MPFR area")
                    .0
                    .to_string()
                    .parse::<BigInt>()
                    .unwrap();
                let error = (&actual - expected).abs();
                assert!(
                    error <= BigInt::from(1),
                    "Heron case {case} differs from MPFR by {error} ulps"
                );
            }
        }
    }
}

#[test]
fn rational_pi_turns_match_all_f64_quadrants_and_periods() {
    for denominator in 1_u64..=16 {
        for numerator in -64_i64..=64 {
            let turns = rational(numerator, denominator);
            let radians = std::f64::consts::PI * numerator as f64 / denominator as f64;
            assert_close(
                turns.clone().sin_pi(),
                radians.sin(),
                &format!("sin_pi({numerator}/{denominator})"),
            );
            assert_close(
                turns.clone().cos_pi(),
                radians.cos(),
                &format!("cos_pi({numerator}/{denominator})"),
            );

            if radians.cos().abs() > 1.0e-12 {
                assert_close(
                    turns
                        .clone()
                        .tan_pi()
                        .unwrap_or_else(|error| panic!("tan_pi failed: {error:?}")),
                    radians.tan(),
                    &format!("tan_pi({numerator}/{denominator})"),
                );
            } else {
                assert_eq!(turns.clone().tan_pi(), Err(Problem::NotANumber));
            }
            if radians.sin().abs() > 1.0e-12 {
                assert_close(
                    turns
                        .cot_pi()
                        .unwrap_or_else(|error| panic!("cot_pi failed: {error:?}")),
                    1.0 / radians.tan(),
                    &format!("cot_pi({numerator}/{denominator})"),
                );
            } else {
                assert_eq!(turns.cot_pi(), Err(Problem::NotANumber));
            }
        }
    }
}

#[test]
fn rational_radian_trig_matches_f64_over_multiple_periods() {
    for denominator in 1_u64..=8 {
        for numerator in -96_i64..=96 {
            let value = rational(numerator, denominator);
            let expected = numerator as f64 / denominator as f64;
            assert_close(
                value.clone().sin(),
                expected.sin(),
                &format!("sin({numerator}/{denominator})"),
            );
            assert_close(
                value.clone().cos(),
                expected.cos(),
                &format!("cos({numerator}/{denominator})"),
            );
            if expected.cos().abs() > 1.0e-10 {
                assert_close(
                    value.clone().tan().expect("finite rational tangent"),
                    expected.tan(),
                    &format!("tan({numerator}/{denominator})"),
                );
            }
            if expected.sin().abs() > 1.0e-10 {
                assert_close(
                    value.cot().expect("finite rational cotangent"),
                    1.0 / expected.tan(),
                    &format!("cot({numerator}/{denominator})"),
                );
            } else {
                assert_eq!(value.cot(), Err(Problem::NotANumber));
            }
        }
    }
}

#[test]
fn cotangent_refines_near_zero_and_preserves_odd_symmetry() {
    for numerator in [-1_i64, 1] {
        let value = rational(numerator, 1_u64 << 40);
        let expected = 1.0 / (numerator as f64 * 2_f64.powi(-40)).tan();
        assert_close(
            value.clone().cot().expect("nonzero rational cotangent"),
            expected,
            &format!("cot({numerator}/2^40)"),
        );
    }

    let value = rational(7, 5);
    assert_eq!((-value.clone()).cot().unwrap(), -value.cot().unwrap());
}

#[test]
fn elementary_functions_match_f64_on_moderate_rational_grid() {
    for denominator in 1_u64..=8 {
        for numerator in -32_i64..=32 {
            let value = rational(numerator, denominator);
            let expected = numerator as f64 / denominator as f64;

            assert_close(
                value.clone().exp().expect("finite exponential"),
                expected.exp(),
                &format!("exp({numerator}/{denominator})"),
            );
            assert_close(
                value.clone().exp2().expect("finite base-two exponential"),
                expected.exp2(),
                &format!("exp2({numerator}/{denominator})"),
            );
            assert_close(
                value.clone().exp10().expect("finite base-ten exponential"),
                10_f64.powf(expected),
                &format!("exp10({numerator}/{denominator})"),
            );
            assert_close(
                value.clone().expm1(),
                expected.exp_m1(),
                &format!("expm1({numerator}/{denominator})"),
            );
            assert_close(
                value.clone().sinh().expect("finite sinh"),
                expected.sinh(),
                &format!("sinh({numerator}/{denominator})"),
            );
            assert_close(
                value.clone().cosh().expect("finite cosh"),
                expected.cosh(),
                &format!("cosh({numerator}/{denominator})"),
            );
            assert_close(
                value.clone().tanh().expect("finite tanh"),
                expected.tanh(),
                &format!("tanh({numerator}/{denominator})"),
            );
            assert_close(
                value.clone().asinh().expect("finite asinh"),
                expected.asinh(),
                &format!("asinh({numerator}/{denominator})"),
            );

            if numerator > 0 {
                assert_close(
                    value.clone().ln().expect("positive logarithm"),
                    expected.ln(),
                    &format!("ln({numerator}/{denominator})"),
                );
                assert_close(
                    value.clone().log2().expect("positive log2"),
                    expected.log2(),
                    &format!("log2({numerator}/{denominator})"),
                );
                assert_close(
                    value.clone().log10().expect("positive log10"),
                    expected.log10(),
                    &format!("log10({numerator}/{denominator})"),
                );
                assert_close(
                    value.clone().sqrt().expect("nonnegative square root"),
                    expected.sqrt(),
                    &format!("sqrt({numerator}/{denominator})"),
                );
            }
            assert_close(
                value.cbrt().expect("finite cube root"),
                expected.cbrt(),
                &format!("cbrt({numerator}/{denominator})"),
            );
        }
    }
}

#[test]
fn inverse_trig_and_hyperbolic_functions_match_principal_branches() {
    for numerator in -32_i64..=32 {
        let value = rational(numerator, 32);
        let expected = numerator as f64 / 32.0;
        assert_close(
            value.clone().asin().expect("asin domain"),
            expected.asin(),
            &format!("asin({numerator}/32)"),
        );
        assert_close(
            value.clone().acos().expect("acos domain"),
            expected.acos(),
            &format!("acos({numerator}/32)"),
        );
        assert_close(
            value.clone().atan().expect("finite atan"),
            expected.atan(),
            &format!("atan({numerator}/32)"),
        );
        if numerator.abs() < 32 {
            assert_close(
                value.atanh().expect("atanh open domain"),
                expected.atanh(),
                &format!("atanh({numerator}/32)"),
            );
        }
    }

    for numerator in 32_i64..=128 {
        let value = rational(numerator, 32);
        let expected = numerator as f64 / 32.0;
        assert_close(
            value.acosh().expect("acosh domain"),
            expected.acosh(),
            &format!("acosh({numerator}/32)"),
        );
    }
}

#[test]
fn atan2_matches_f64_in_every_quadrant_and_under_scaling() {
    for y in -8_i64..=8 {
        for x in -8_i64..=8 {
            let expected = (y as f64).atan2(x as f64);
            let actual = rational(y, 4).atan2(rational(x, 4));
            assert_close(actual, expected, &format!("atan2({y}, {x})"));

            if x != 0 || y != 0 {
                let scaled = rational(y * 7, 12).atan2(rational(x * 7, 12));
                assert_close(scaled, expected, &format!("scaled atan2({y}, {x})"));
            }
        }
    }
}

#[test]
fn direct_computable_atan_quarter_matches_reference() {
    for numerator in [-1, 1] {
        let actual = Computable::rational(Rational::fraction(numerator, 4).unwrap())
            .atan()
            .approx(-32);
        let expected = ((numerator as f64 / 4.0).atan() * 2.0_f64.powi(32)).round() as i64;
        assert!(
            (&actual - expected).abs() <= 1.into(),
            "atan({numerator}/4) fixed-point mismatch: {actual} != {expected}"
        );
    }
}

#[test]
fn tangent_preserves_odd_symmetry_across_medium_reduction() {
    for numerator in [-2, 2] {
        let value = Computable::rational(Rational::new(numerator)).tan();
        let actual = value.approx(-32);
        let expected = ((numerator as f64).tan() * 2.0_f64.powi(32)).round() as i64;
        assert!((&actual - expected).abs() <= 1.into());
    }
}

#[test]
fn tangent_matches_reference_across_certified_two_half_pi_interval() {
    for (numerator, denominator) in [
        (-39_i64, 10_u64),
        (-847, 219),
        (-7, 2),
        (7, 2),
        (847, 219),
        (39, 10),
    ] {
        let x = numerator as f64 / denominator as f64;
        let actual = Computable::rational(Rational::fraction(numerator, denominator).unwrap())
            .tan()
            .approx(-32);
        let expected = (x.tan() * 2.0_f64.powi(32)).round() as i64;
        assert!(
            (&actual - expected).abs() <= 1.into(),
            "tan({numerator}/{denominator}) fixed-point mismatch: {actual} != {expected}"
        );
    }
}

#[test]
fn real_atan_is_stable_after_sibling_inverse_trig_calls() {
    for numerator in [-1, 1] {
        let value = rational(numerator, 4);
        let expected = (numerator as f64 / 4.0).atan();
        let cold = value.clone().atan().unwrap();
        assert_close(cold, expected, "cold Real::atan");
        let _ = value.clone().asin().unwrap().to_f64_lossy();
        let _ = value.clone().acos().unwrap().to_f64_lossy();
        assert_close(value.atan().unwrap(), expected, "warmed Real::atan");
    }
}

#[test]
fn magnitude_detection_refines_ambiguous_unit_approximations() {
    for value in [
        Computable::rational(Rational::fraction(1, 4).unwrap()).atan(),
        Computable::rational(Rational::fraction(1, 4).unwrap()).asinh(),
    ] {
        assert_eq!(value.approx(-2), 1.into());
    }
}

#[test]
fn stable_small_argument_functions_match_cancellation_free_references() {
    for exponent in 1..=40 {
        let denominator = 1_u64 << exponent;
        for numerator in [-1_i64, 1] {
            let x = numerator as f64 / denominator as f64;
            let value = rational(numerator, denominator);

            assert_close(
                value.clone().ln_1p().expect("ln1p domain"),
                x.ln_1p(),
                &format!("ln_1p({numerator}/2^{exponent})"),
            );
            assert_close(
                value.clone().ln_1m().expect("ln1m domain"),
                (-x).ln_1p(),
                &format!("ln_1m({numerator}/2^{exponent})"),
            );
            assert_close(
                value.clone().sqrt1pm1().expect("sqrt1pm1 domain"),
                x / ((1.0 + x).sqrt() + 1.0),
                &format!("sqrt1pm1({numerator}/2^{exponent})"),
            );
            assert_close(
                value.clone().sqrt1m1().expect("sqrt1m1 domain"),
                -x / ((1.0 - x).sqrt() + 1.0),
                &format!("sqrt1m1({numerator}/2^{exponent})"),
            );
            assert_close(
                value.clone().sinc().expect("finite sinc"),
                x.sin() / x,
                &format!("sinc({numerator}/2^{exponent})"),
            );
            let half = x / 2.0;
            assert_close(
                value.clone().cosc().expect("finite cosc"),
                0.5 * (half.sin() / half).powi(2),
                &format!("cosc({numerator}/2^{exponent})"),
            );

            let sigmoid = if x >= 0.0 {
                1.0 / (1.0 + (-x).exp())
            } else {
                x.exp() / (1.0 + x.exp())
            };
            assert_close(
                value.clone().sigmoid().expect("finite sigmoid"),
                sigmoid,
                &format!("sigmoid({numerator}/2^{exponent})"),
            );
            assert_close(
                value.clone().softplus().expect("finite softplus"),
                x.max(0.0) + (-x.abs()).exp().ln_1p(),
                &format!("softplus({numerator}/2^{exponent})"),
            );
        }
    }
}

#[test]
fn tangent_refines_for_a_finite_rational_extremely_close_to_a_pole() {
    // 104348/33215 is a convergent to pi, so half of it is close enough to
    // pi/2 to require substantially more than a fixed quotient guard. The
    // expected value is rounded from an 80-digit evaluation; f64::tan loses
    // several output digits here because its rounded input is ill-conditioned.
    for numerator in [-52_174_i64, 52_174] {
        let expected = -(numerator.signum() as f64) * 6_030_857_371.821_142;
        assert_close(
            rational(numerator, 33_215)
                .tan()
                .expect("rational is not an exact tangent pole"),
            expected,
            &format!("tan({numerator}/33215)"),
        );
    }
}
