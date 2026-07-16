use hyperreal::{Computable, Problem, Rational, Real};
use num::Signed;

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
                        .tan_pi()
                        .unwrap_or_else(|error| panic!("tan_pi failed: {error:?}")),
                    radians.tan(),
                    &format!("tan_pi({numerator}/{denominator})"),
                );
            } else {
                assert_eq!(turns.tan_pi(), Err(Problem::NotANumber));
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
                    value.tan().expect("finite rational tangent"),
                    expected.tan(),
                    &format!("tan({numerator}/{denominator})"),
                );
            }
        }
    }
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
