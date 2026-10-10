use hyperreal::{Problem, Real};
use std::sync::{Arc, atomic::AtomicBool};

fn scalar(token: &str) -> Real {
    token.parse().unwrap()
}

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
fn decimal_rounding_keeps_fixed_places_scientific_places_and_exact_ties_distinct() {
    for (token, places, expected) in [
        ("1/8", 3, "0.125"),
        ("1/8", 6, "0.125000"),
        ("1/8", 0, "0"),
        ("-1/8", 0, "-0"),
        ("1/2", 0, "0"),
        ("3/2", 0, "2"),
        ("5/2", 0, "2"),
        ("-5/2", 0, "-2"),
        ("1.2345665", 6, "1.234566"),
        ("1.2345675", 6, "1.234568"),
        ("9.9999995", 6, "10.000000"),
        ("0", 6, "0.000000e0"),
        ("1e-40", 6, "1.000000e-40"),
        ("1e-400", 3, "1.000e-400"),
        ("1e400", 3, "1.000e400"),
        ("-1e-40", 3, "-1.000e-40"),
        ("1.25e-40", 1, "1.2e-40"),
        ("1.35e-40", 1, "1.4e-40"),
    ] {
        assert_eq!(
            scalar(token).try_decimal(places, 0).unwrap(),
            expected,
            "{token}"
        );
    }
}

#[test]
fn notation_boundaries_are_defined_by_exact_magnitude() {
    for (token, scientific) in [
        ("1/524288", false),
        ("-1/524288", false),
        ("1/1048576", true),
        ("-1/1048576", true),
        ("1152921504606846975", false),
        ("-1152921504606846975", false),
        ("1152921504606846976", true),
        ("-1152921504606846976", true),
    ] {
        let value = scalar(token);
        for prepared in [false, true] {
            if prepared {
                let _ = value.certified_rational_interval(-256);
            }
            assert_eq!(
                value.try_decimal(6, -256).unwrap().contains('e'),
                scientific,
                "{token}"
            );
        }
    }
}

#[test]
fn computable_values_use_certified_enclosures_in_both_notations() {
    for (value, expected) in [
        (Real::pi(), "3.141593"),
        (Real::from(2).sqrt().unwrap(), "1.414214"),
        (Real::pi() * scalar("1e-40"), "3.141593e-40"),
        (Real::pi() * scalar("1e40"), "3.141593e40"),
    ] {
        assert_eq!(value.try_decimal(6, -256).unwrap(), expected);
        let _ = value.certified_rational_interval(-512);
        assert_eq!(value.try_decimal(6, -256).unwrap(), expected);
    }
}

#[test]
fn unresolved_zero_rounding_and_notation_remain_typed() {
    for (value, error) in [
        (unresolved_zero(), Problem::UnknownZero),
        (unresolved_zero() + scalar("1.2345665"), Problem::Exhausted),
        (unresolved_zero() + scalar("1/524288"), Problem::Exhausted),
    ] {
        assert_eq!(value.try_decimal(6, -256), Err(error));
        let _ = value.certified_rational_interval(-512);
        assert_eq!(value.try_decimal(6, -256), Err(error));
    }
    assert_eq!(Real::pi().try_decimal(6, 0), Err(Problem::Exhausted));
}

#[test]
fn final_value_abort_does_not_become_a_number() {
    for prepared in [false, true] {
        let mut value = Real::pi();
        if prepared {
            let _ = value.certified_rational_interval(-256);
        }
        value.abort(Arc::new(AtomicBool::new(true)));
        assert_eq!(value.try_decimal(6, -256), Err(Problem::Exhausted));
        assert_eq!(
            format!("{:.6}", value.diagnostic_decimal(-256)),
            "[Exhausted]"
        );
    }
}

#[test]
fn diagnostic_text_trims_only_fractional_zeros_and_preserves_formatter_signs() {
    for (token, expected) in [
        ("0", "0e0"),
        ("1/8", "0.125"),
        ("1200", "1200"),
        ("1e-40", "1e-40"),
        ("1e-400", "1e-400"),
        ("1e400", "1e400"),
    ] {
        assert_eq!(
            format!("{}", scalar(token).diagnostic_decimal(-256)),
            expected
        );
    }
    assert_eq!(
        format!("{:+012.3}", scalar("1/8").diagnostic_decimal(-256)),
        "+0000000.125"
    );
    assert_eq!(
        format!("{:#.6}", scalar("1e-40").diagnostic_decimal(-256)),
        "1.000000e-40"
    );
    assert_eq!(
        format!("{}", unresolved_zero().diagnostic_decimal(-256)),
        "[UnknownZero]"
    );
}

#[test]
fn invalid_or_excessive_requests_fail_before_decimal_expansion() {
    assert_eq!(Real::one().try_decimal(6, 1), Err(Problem::OutOfRange));
    assert_eq!(
        Real::one().try_decimal(1_000_001, -256),
        Err(Problem::Exhausted)
    );
    assert_eq!(
        Real::one().try_decimal(6, -1_000_001),
        Err(Problem::Exhausted)
    );
    assert_eq!(
        format!("{:.6}", Real::one().diagnostic_decimal(-1_000_001)),
        "[Exhausted]"
    );
}
