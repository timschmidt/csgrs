use ciborium::Value;
use hyperreal::{Problem, Rational, Real, ZeroKnowledge};
use proptest::prelude::*;

fn q(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).unwrap()
}

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
        ("-7.875", q(-63, 8)),
        ("123456.000001", q(123456000001, 1_000_000)),
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
fn unsupported_numeric_text_forms_fail_instead_of_rounding() {
    for text in ["", "1e3", "NaN", "inf", "1 1/2"] {
        let parsed: Result<Real, _> = text.parse();
        assert!(
            parsed.is_err(),
            "{text:?} should not parse as a lossless Real numeric literal"
        );
    }
}

#[test]
fn non_finite_float_imports_are_rejected_losslessly() {
    assert_eq!(Real::try_from(f32::INFINITY), Err(Problem::Infinity));
    assert_eq!(Real::try_from(f32::NEG_INFINITY), Err(Problem::Infinity));
    assert_eq!(Real::try_from(f32::NAN), Err(Problem::NotANumber));
    assert_eq!(Real::try_from(f64::INFINITY), Err(Problem::Infinity));
    assert_eq!(Real::try_from(f64::NEG_INFINITY), Err(Problem::Infinity));
    assert_eq!(Real::try_from(f64::NAN), Err(Problem::NotANumber));
}

#[test]
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

fn finite_f32_bits() -> impl Strategy<Value = u32> {
    any::<u32>().prop_filter("finite f32 bits", |bits| f32::from_bits(*bits).is_finite())
}

fn finite_f64_bits() -> impl Strategy<Value = u64> {
    any::<u64>().prop_filter("finite f64 bits", |bits| f64::from_bits(*bits).is_finite())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(1024))]

    #[test]
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
