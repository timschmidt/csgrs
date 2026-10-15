#[path = "support/mpfr.rs"]
mod mpfr;

use hyperreal::{Problem, Real, ScientificCase};
use rug::float::Round;
use rug::{Float, Rational as Mpq};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

fn scalar(text: &str) -> Real {
    text.parse().unwrap()
}

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

fn original_hypot() -> Real {
    let delta = (Real::one() + unresolved_zero()) - Real::one();
    Real::hypot2(&delta, &Real::zero()).unwrap()
}

fn mpfr_text(value: &Float, places: usize, case: ScientificCase) -> String {
    let (negative, digits, exponent) =
        value.to_sign_string_exp_round(10, Some(places + 1), Round::Nearest);
    let (digits, exponent) = if value.is_zero() {
        ("0".repeat(places + 1), 0)
    } else {
        (digits, exponent.unwrap() - 1)
    };
    let sign = if negative && !value.is_zero() {
        "-"
    } else {
        ""
    };
    let fraction = if places == 0 {
        String::new()
    } else {
        format!(".{}", &digits[1..])
    };
    let marker = match case {
        ScientificCase::Lower => 'e',
        ScientificCase::Upper => 'E',
    };
    format!("{sign}{}{fraction}{marker}{exponent}", &digits[..1])
}

#[test]
fn rational_outputs_preserve_sign_precision_scale_and_case() {
    for (input, places, expected) in [
        ("0", 6, "0.000000e0"),
        ("-0", 0, "0e0"),
        ("7/8", 6, "8.750000e-1"),
        ("-7/8", 6, "-8.750000e-1"),
        ("100", 3, "1.000e2"),
        ("0.001", 3, "1.000e-3"),
        ("1e400", 6, "1.000000e400"),
        ("-1e-400", 6, "-1.000000e-400"),
        ("1e-400", 0, "1e-400"),
    ] {
        let value = scalar(input);
        assert_eq!(
            value.try_scientific(places, 0, ScientificCase::Lower),
            Ok(expected.into())
        );
        assert_eq!(
            value.try_scientific(places, 0, ScientificCase::Upper),
            Ok(expected.replace('e', "E"))
        );
    }
}

#[test]
fn exact_halfway_values_round_to_even_including_decade_carries() {
    for (input, places, expected) in [
        ("2.5", 0, "2e0"),
        ("3.5", 0, "4e0"),
        ("9.5", 0, "1e1"),
        ("998.5", 2, "9.98e2"),
        ("999.5", 2, "1.00e3"),
        ("99.95", 2, "1.00e2"),
        ("0.009995", 2, "1.00e-2"),
        ("1.234565", 5, "1.23456e0"),
        ("1.234575", 5, "1.23458e0"),
    ] {
        for negative in [false, true] {
            let prefix = if negative { "-" } else { "" };
            let value = scalar(&format!("{prefix}{input}"));
            assert_eq!(
                value.try_scientific(places, -128, ScientificCase::Lower),
                Ok(format!("{prefix}{expected}"))
            );
        }
    }
}

#[test]
fn generated_exact_rationals_match_directed_mpfr_reference_brackets() {
    mpfr::cleanup_on_thread_exit();
    for denominator in [3_i64, 7, 11, 13, 17, 19, 23, 31, 97] {
        for index in -32_i64..32 {
            let numerator = index * 7919;
            let exact = Mpq::from((numerator, denominator));
            let (lower, _) = Float::with_val_round(256, &exact, Round::Down);
            let (upper, _) = Float::with_val_round(256, &exact, Round::Up);
            let value = scalar(&format!("{numerator}/{denominator}"));
            for places in [0, 1, 6, 20] {
                for case in [ScientificCase::Lower, ScientificCase::Upper] {
                    let expected = mpfr_text(&lower, places, case);
                    assert_eq!(
                        expected,
                        mpfr_text(&upper, places, case),
                        "reference rounding must be certified by both bounds"
                    );
                    assert_eq!(
                        value.try_scientific(places, 0, case),
                        Ok(expected),
                        "{numerator}/{denominator}, {places}, {case:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn pi_and_square_root_match_directed_mpfr_reference_brackets() {
    mpfr::cleanup_on_thread_exit();
    let pi = Real::pi();
    let root = Real::from(2).sqrt().unwrap();
    let (pi_lower, _) = Float::with_val_round(256, rug::float::Constant::Pi, Round::Down);
    let (pi_upper, _) = Float::with_val_round(256, rug::float::Constant::Pi, Round::Up);
    let mut root_lower = Float::with_val(256, 2);
    let mut root_upper = root_lower.clone();
    root_lower.sqrt_round(Round::Down);
    root_upper.sqrt_round(Round::Up);
    for (value, lower, upper) in [(pi, pi_lower, pi_upper), (root, root_lower, root_upper)] {
        for places in [0, 6, 20, 50] {
            for case in [ScientificCase::Lower, ScientificCase::Upper] {
                let expected = mpfr_text(&lower, places, case);
                assert_eq!(expected, mpfr_text(&upper, places, case));
                assert_eq!(value.try_scientific(places, -512, case), Ok(expected));
            }
        }
    }
}

#[test]
fn rational_scale_is_preserved_outside_primitive_float_range() {
    for (scale, expected) in [
        ("1e400", "3.141593e400"),
        ("1e-400", "3.141593e-400"),
        ("-1e-400", "-3.141593e-400"),
    ] {
        let value = Real::pi() * scalar(scale);
        assert_eq!(
            value.try_scientific(6, -128, ScientificCase::Lower),
            Ok(expected.into())
        );
    }
}

#[test]
fn original_unresolved_zero_forms_return_typed_failure() {
    for value in [
        unresolved_zero(),
        (Real::one() + unresolved_zero()) - Real::one(),
        original_hypot(),
    ] {
        for floor in [-64, -128, -256, -512] {
            for case in [ScientificCase::Lower, ScientificCase::Upper] {
                assert_eq!(
                    value.try_scientific(6, floor, case),
                    Err(Problem::UnknownZero)
                );
                assert!(value.certified_rational_interval(-512).is_some());
                assert_eq!(
                    value.clone().try_scientific(6, floor, case),
                    Err(Problem::UnknownZero)
                );
            }
        }
    }
}

#[test]
fn tiny_unresolved_nonzero_values_need_sufficient_enclosure_precision() {
    for (offset, expected) in [("1e-40", "1.000000e-40"), ("-1e-40", "-1.000000e-40")] {
        let value = unresolved_zero() + scalar(offset);
        assert_eq!(
            value.try_scientific(6, -64, ScientificCase::Lower),
            Err(Problem::UnknownZero)
        );
        assert_eq!(
            value.try_scientific(6, -256, ScientificCase::Lower),
            Ok(expected.into())
        );
        assert!(value.certified_rational_interval(-512).is_some());
        assert_eq!(
            value.try_scientific(6, -64, ScientificCase::Lower),
            Err(Problem::UnknownZero)
        );
        assert_eq!(
            value.clone().try_scientific(6, -256, ScientificCase::Upper),
            Ok(expected.replace('e', "E"))
        );
    }
}

#[test]
fn ambiguous_rounding_is_distinct_from_unresolved_zero() {
    let value = unresolved_zero() + scalar("1.234565");
    assert_eq!(
        value.try_scientific(5, -256, ScientificCase::Lower),
        Err(Problem::Exhausted)
    );
    assert_eq!(
        scalar("1.234565").try_scientific(5, -256, ScientificCase::Lower),
        Ok("1.23456e0".into())
    );
}

#[test]
fn final_value_abort_is_honored_after_warming_and_clearing() {
    let flag = Arc::new(AtomicBool::new(true));
    let mut value = unresolved_zero() + scalar("1e-40");
    value.abort(flag.clone());
    assert!(value.certified_rational_interval(-256).is_none());
    assert_eq!(
        value.try_scientific(6, -256, ScientificCase::Lower),
        Err(Problem::Exhausted)
    );
    assert!(value.certified_rational_interval(-256).is_none());
    flag.store(false, Ordering::Relaxed);
    assert_eq!(
        value.try_scientific(6, -256, ScientificCase::Lower),
        Ok("1.000000e-40".into())
    );
    flag.store(true, Ordering::Relaxed);
    assert!(value.certified_rational_interval(-256).is_none());
    assert_eq!(
        value.try_scientific(6, -256, ScientificCase::Lower),
        Err(Problem::Exhausted)
    );
    flag.store(false, Ordering::Relaxed);
    assert_eq!(
        value.try_scientific(6, -256, ScientificCase::Upper),
        Ok("1.000000E-40".into())
    );
}

#[test]
fn direct_rational_output_does_not_require_numeric_refinement() {
    let flag = Arc::new(AtomicBool::new(true));
    for input in ["0", "1/8"] {
        let mut value = scalar(input);
        value.abort(flag.clone());
        assert!(value.try_scientific(6, 0, ScientificCase::Lower).is_ok());
    }
}

#[test]
fn invalid_or_excessive_requests_fail_before_formatting() {
    let value = Real::one();
    assert_eq!(
        value.try_scientific(6, 1, ScientificCase::Lower),
        Err(Problem::OutOfRange)
    );
    assert_eq!(
        value.try_scientific(usize::MAX, -128, ScientificCase::Lower),
        Err(Problem::Exhausted)
    );
    assert_eq!(
        value.try_scientific(1_000_001, -128, ScientificCase::Lower),
        Err(Problem::Exhausted)
    );
    assert_eq!(
        value.try_scientific(6, i32::MIN, ScientificCase::Lower),
        Err(Problem::Exhausted)
    );
    let huge = Real::new(hyperreal::Rational::from_bigint(
        num::BigInt::from(1_u8) << 4_000_000_usize,
    ));
    assert_eq!(
        huge.try_scientific(6, 0, ScientificCase::Lower),
        Err(Problem::Exhausted)
    );
}

#[test]
fn existing_fixed_decimal_view_remains_separate() {
    assert_eq!(
        format!("{:.6}", original_hypot().fixed_decimal()),
        "0.000000"
    );
    let tiny = scalar("-1e-400");
    assert_eq!(format!("{:.6}", tiny.fixed_decimal()), "-0.000000");
    assert_eq!(
        tiny.try_scientific(6, -128, ScientificCase::Lower),
        Ok("-1.000000e-400".into())
    );
}
