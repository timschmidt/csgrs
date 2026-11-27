//! Regression corpus for the approximate fixed-fractional-place display view.
//!
//! These strings test presentation behavior, not certified rounding or exact
//! zero recognition. The checked decimal/scientific suites own typed failures.

use hyperreal::Real;

/// Construct the unresolved zero identity sin(e)^2 + cos(e)^2 - 1 without replacing its graph by zero.
fn cancellation() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
/// Require six-place output for unresolved cancellation and its hypot2 distance to zero.
/// The zero-looking string is an approximate display result, not an exact-zero proof.
fn unresolved_zero_and_native_distance_need_no_nonzero_certificate() {
    let value = cancellation();
    assert_eq!(format!("{:.6}", value.fixed_decimal()), "0.000000");
    let distance = Real::hypot2(&value, &Real::zero()).unwrap();
    assert_eq!(format!("{:.6}", distance.fixed_decimal()), "0.000000");
}

#[test]
/// Check fixed fractional padding for five rational fixtures, plus explicit positive sign and default trimming.
fn explicit_fractional_precision_and_sign_are_preserved() {
    for (text, expected) in [
        ("0", "0.000"),
        ("1/8", "0.125"),
        ("-1/8", "-0.125"),
        ("2", "2.000"),
        ("1/3", "0.333"),
    ] {
        let value: Real = text.parse().unwrap();
        assert_eq!(format!("{:.3}", value.fixed_decimal()), expected);
    }
    assert_eq!(format!("{:+.0}", Real::one().fixed_decimal()), "+1");
    assert_eq!(format!("{}", Real::one().fixed_decimal()), "1");
}

#[test]
/// Check six-place pi and positive sqrt(2) strings against literal display expectations.
fn symbolic_values_use_the_requested_decimal_places() {
    assert_eq!(format!("{:.6}", Real::pi().fixed_decimal()), "3.141593");
    assert_eq!(
        format!("{:.6}", Real::from(2).sqrt().unwrap().fixed_decimal()),
        "1.414214"
    );
}

#[test]
/// For both signs of pi*10^-100, preserve six-place fixed output after 110-place scientific display.
/// Assert the sign of rounded display zero; the scientific string itself is discarded.
fn tiny_symbolic_values_keep_fixed_notation_after_cache_warming() {
    for sign in [1, -1] {
        let tiny: Real = "1e-100".parse().unwrap();
        let value = Real::from(sign) * Real::pi() * tiny;
        let before = format!("{:.6}", value.fixed_decimal());
        let _ = format!("{value:.110e}");
        assert_eq!(format!("{:.6}", value.fixed_decimal()), before);
        assert_eq!(before, if sign > 0 { "0.000000" } else { "-0.000000" });
    }
}

#[test]
/// Require 10^40 to print all forty trailing integer zeros and two fractional zeros without an exponent.
fn large_values_do_not_switch_notation() {
    let value: Real = "1e40".parse().unwrap();
    assert_eq!(
        format!("{:.2}", value.fixed_decimal()),
        format!("1{}.00", "0".repeat(40))
    );
}
