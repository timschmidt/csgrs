//! Numeric ingress and exact rational interchange regression corpus (serde feature).
//!
//! Fixed fixtures check values/structural facts and typed nonfinite failures;
//! generated cases exercise finite IEEE bits and bounded numeric text. JSON and
//! CBOR replay are separate checks from owned float export of the original Real.
//! Signed-zero bit preservation, fresh-process replay, byte determinism and
//! arbitrary symbolic certificate completeness are not established by this file.

#![cfg(feature = "serde")]

use ciborium::Value;
use hyperreal::{Problem, Rational, Real, ZeroKnowledge};
use proptest::prelude::*;

/// Build an exact fixture fraction and panic if its denominator violates the constructor contract.
fn q(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).unwrap()
}

/// Replay a Real through JSON and CBOR separately, requiring Real equality and identical structural facts.
/// Does not compare bytes, caches, allocation identity or the behavior of later modeling operations.
fn assert_lossless_real_roundtrip(value: Real) {
    let json = value.to_json();
    let json_roundtrip = Real::from_json(&json).unwrap();
    assert_eq!(json_roundtrip, value);
    assert_eq!(json_roundtrip.structural_facts(), value.structural_facts());

    let bytes = value.to_bytes();
    let bytes_roundtrip = Real::from_bytes(&bytes).unwrap();
    assert_eq!(bytes_roundtrip, value);
    assert_eq!(bytes_roundtrip.structural_facts(), value.structural_facts());
}

#[test]
/// Check ten signed cases and five unsigned maxima after widening to i128/u128.
/// These exercise the wide integer constructors, not each narrow From implementation;
/// the lowest signed fixture is i128::MIN+1. Require exact rational values and both replay formats.
fn integer_primitives_roundtrip_losslessly_through_real() {
    let signed_cases = [
        i128::from(i8::MIN),
        i128::from(i8::MAX),
        i128::from(i16::MIN),
        i128::from(i16::MAX),
        i128::from(i32::MIN),
        i128::from(i32::MAX),
        i128::from(i64::MIN),
        i128::from(i64::MAX),
        i128::MIN + 1,
        i128::MAX,
    ];
    for value in signed_cases {
        let real = Real::from(value);
        assert_eq!(real.exact_rational(), Some(Rational::from(value)));
        assert_lossless_real_roundtrip(real);
    }

    let unsigned_cases = [
        u128::from(u8::MAX),
        u128::from(u16::MAX),
        u128::from(u32::MAX),
        u128::from(u64::MAX),
        u128::MAX,
    ];
    for value in unsigned_cases {
        let real = Real::from(value);
        assert_eq!(real.exact_rational(), Some(Rational::from(value)));
        assert_lossless_real_roundtrip(real);
    }
}

#[test]
/// Check 17 exact zero, integer, fraction, decimal and scientific fixtures against rational expectations.
/// Require both parsers, exact-rational extraction and both replay formats to retain the selected values.
fn supported_numeric_text_forms_parse_losslessly() {
    let cases = [
        ("0", Rational::zero()),
        ("-0", Rational::zero()),
        (
            "123456789012345678901234567890",
            "123456789012345678901234567890".parse().unwrap(),
        ),
        (
            "-98765432109876543210987654321",
            "-98765432109876543210987654321".parse().unwrap(),
        ),
        ("1/2", q(1, 2)),
        ("-7/13", q(-7, 13)),
        ("98760/123450", q(4, 5)),
        ("0.0", Rational::zero()),
        ("0.125", q(1, 8)),
        (".5", q(1, 2)),
        ("5.", Rational::from(5_i32)),
        ("-7.875", q(-63, 8)),
        ("123456.000001", q(123456000001, 1_000_000)),
        ("1e3", Rational::from(1_000_i32)),
        ("2.5E-3", q(1, 400)),
        ("-7.78437e-005", q(-778_437, 10_000_000_000)),
        ("+3.125e+2", q(625, 2)),
    ];

    for (text, expected) in cases {
        let rational: Rational = text.parse().unwrap();
        let real: Real = text.parse().unwrap();
        assert_eq!(rational, expected);
        assert_eq!(real, expected);
        assert_eq!(real.exact_rational(), Some(expected));
        assert_lossless_real_roundtrip(real);
    }
}

#[test]
/// Require six malformed or unsupported literals to fail Real parsing; no particular error category is asserted.
fn unsupported_numeric_text_forms_fail_instead_of_rounding() {
    for text in ["", "NaN", "inf", "1 1/2", "1e", "1e1e2"] {
        let parsed: Result<Real, _> = text.parse();
        assert!(
            parsed.is_err(),
            "{text:?} should not parse as a lossless Real numeric literal"
        );
    }
}

#[test]
/// Require Infinity for both signed infinities and NotANumber for canonical NaN at both primitive widths.
fn non_finite_float_imports_are_rejected_losslessly() {
    assert_eq!(Real::try_from(f32::INFINITY), Err(Problem::Infinity));
    assert_eq!(Real::try_from(f32::NEG_INFINITY), Err(Problem::Infinity));
    assert_eq!(Real::try_from(f32::NAN), Err(Problem::NotANumber));
    assert_eq!(Real::try_from(f64::INFINITY), Err(Problem::Infinity));
    assert_eq!(Real::try_from(f64::NEG_INFINITY), Err(Problem::Infinity));
    assert_eq!(Real::try_from(f64::NAN), Err(Problem::NotANumber));
}

#[test]
/// Check four CBOR Value ingress forms against exact values: integer, dyadic float, fraction text and pair.
/// Despite the test name, these assertions do not encode or reimport the resulting Real.
fn cbor_value_numeric_forms_roundtrip_losslessly() {
    let integer = Value::Integer(123_i64.into());
    assert_eq!(Real::try_from(&integer).unwrap(), Real::from(123_i64));

    let float = Value::Float(0.125);
    assert_eq!(Real::try_from(&float).unwrap(), Real::new(q(1, 8)));

    let text = Value::Text("-7/11".to_string());
    assert_eq!(Real::try_from(&text).unwrap(), Real::new(q(-7, 11)));

    let rational_pair = Value::Array(vec![
        Value::Integer((-9_i64).into()),
        Value::Integer(14_i64.into()),
    ]);
    assert_eq!(
        Real::try_from(&rational_pair).unwrap(),
        Real::new(q(-9, 14))
    );
}

/// Generate arbitrary binary32 bit patterns, filtering out infinities and NaNs.
/// Subnormals and both zero signs are eligible; finite random coverage does not guarantee every boundary.
fn finite_f32_bits() -> impl Strategy<Value = u32> {
    any::<u32>().prop_filter("finite f32 bits", |bits| f32::from_bits(*bits).is_finite())
}

/// Generate arbitrary binary64 bit patterns, filtering out infinities and NaNs.
/// Retain subnormals and signed zeros in the admitted generator domain.
fn finite_f64_bits() -> impl Strategy<Value = u64> {
    any::<u64>().prop_filter("finite f64 bits", |bits| f64::from_bits(*bits).is_finite())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(1024))]

    #[test]
    /// For generated finite binary32 values, require zero knowledge, rational storage and JSON/CBOR equality.
    /// Owned export of the original Real must retain nonzero bits; zero is compared numerically, not by sign bits.
    fn finite_f32_values_roundtrip_through_exact_dyadic_real(bits in finite_f32_bits()) {
        let value = f32::from_bits(bits);
        let real = Real::try_from(value).unwrap();

        prop_assert_eq!(real.zero_status(), if value == 0.0 { ZeroKnowledge::Zero } else { ZeroKnowledge::NonZero });
        prop_assert!(real.exact_rational().is_some());
        prop_assert_eq!(Real::from_json(&real.to_json()).unwrap(), real.clone());
        prop_assert_eq!(Real::from_bytes(&real.to_bytes()).unwrap(), real.clone());

        let exported = f32::from(real.clone());
        if value == 0.0 {
            prop_assert_eq!(exported, 0.0);
        } else {
            prop_assert_eq!(exported.to_bits(), value.to_bits());
        }
    }

    #[test]
    /// Apply the finite-float ingress/replay witnesses to binary64, exporting the original Real.
    /// Require bit equality for nonzero values and numeric zero equality for either zero sign.
    fn finite_f64_values_roundtrip_through_exact_dyadic_real(bits in finite_f64_bits()) {
        let value = f64::from_bits(bits);
        let real = Real::try_from(value).unwrap();

        prop_assert_eq!(real.zero_status(), if value == 0.0 { ZeroKnowledge::Zero } else { ZeroKnowledge::NonZero });
        prop_assert!(real.exact_rational().is_some());
        prop_assert_eq!(Real::from_json(&real.to_json()).unwrap(), real.clone());
        prop_assert_eq!(Real::from_bytes(&real.to_bytes()).unwrap(), real.clone());

        let exported = f64::from(real.clone());
        if value == 0.0 {
            prop_assert_eq!(exported, 0.0);
        } else {
            prop_assert_eq!(exported.to_bits(), value.to_bits());
        }
    }

    #[test]
    /// Generate signed whole parts in +/-1,000,000 with six fractional digits and compare both exact parsers.
    /// Require rational extraction and JSON/CBOR replay; agreement between these parsers is not an independent decimal oracle.
    fn generated_decimal_text_roundtrips_as_exact_rational(whole in -1_000_000_i64..=1_000_000, fractional in 0_u32..1_000_000) {
        let text = format!("{whole}.{fractional:06}");
        let rational: Rational = text.parse().unwrap();
        let real: Real = text.parse().unwrap();

        prop_assert_eq!(real.clone(), rational.clone());
        prop_assert_eq!(real.exact_rational(), Some(rational));
        prop_assert_eq!(Real::from_json(&real.to_json()).unwrap(), real.clone());
        prop_assert_eq!(Real::from_bytes(&real.to_bytes()).unwrap(), real.clone());
    }

    #[test]
    /// Generate six-digit decimal mantissas, exponents -18..=18, either exponent case and optional nonnegative plus signs.
    /// Compare scientific parsing with the parsed mantissa times exact 10^exponent, then check rational storage and replay.
    fn generated_scientific_text_matches_exact_decimal_scaling(
        whole in -1_000_000_i64..=1_000_000,
        fractional in 0_u32..1_000_000,
        exponent in -18_i32..=18,
        uppercase in any::<bool>(),
        explicit_plus in any::<bool>(),
    ) {
        let mantissa_text = format!("{whole}.{fractional:06}");
        let exponent_marker = if uppercase { 'E' } else { 'e' };
        let exponent_text = if explicit_plus && exponent >= 0 {
            format!("+{exponent}")
        } else {
            exponent.to_string()
        };
        let text = format!("{mantissa_text}{exponent_marker}{exponent_text}");
        let mantissa = mantissa_text.parse::<Rational>().unwrap();
        let scale = Rational::new(10).powi(exponent.into()).unwrap();
        let expected = mantissa * scale;
        let rational: Rational = text.parse().unwrap();
        let real: Real = text.parse().unwrap();

        prop_assert_eq!(rational.clone(), expected);
        prop_assert_eq!(real.clone(), rational.clone());
        prop_assert_eq!(real.exact_rational(), Some(rational));
        prop_assert_eq!(Real::from_json(&real.to_json()).unwrap(), real.clone());
        prop_assert_eq!(Real::from_bytes(&real.to_bytes()).unwrap(), real);
    }

    #[test]
    /// Generate numerators in +/-1,000,000 and positive denominators 1..=1,000,000.
    /// Require parser agreement, rational extraction and both replay formats, without assuming unique sampled values.
    fn generated_fraction_text_roundtrips_as_exact_rational(numerator in -1_000_000_i64..=1_000_000, denominator in 1_u64..=1_000_000) {
        let text = format!("{numerator}/{denominator}");
        let rational: Rational = text.parse().unwrap();
        let real: Real = text.parse().unwrap();

        prop_assert_eq!(real.clone(), rational.clone());
        prop_assert_eq!(real.exact_rational(), Some(rational));
        prop_assert_eq!(Real::from_json(&real.to_json()).unwrap(), real.clone());
        prop_assert_eq!(Real::from_bytes(&real.to_bytes()).unwrap(), real.clone());
    }
}
