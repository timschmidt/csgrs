#![cfg(feature = "serde")]

use hyperreal::{Rational, Real};
use serde_json::Value;

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::from(Rational::fraction(numerator, denominator).unwrap())
}

// Public recipes cover every named certificate. These bounds follow from
// 3 < pi < 22/7, 8/3 < e < 11/4, 7/5 < sqrt(2) < 3/2, monotonicity,
// and the defining power/logarithm identities; they are not float samples.
// In particular ln(2) < 3/4 and ln(3) < 5/4 imply ln(2)*ln(3) < 1.
fn certificates() -> Vec<(&'static str, Real, i64, i64)> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().unwrap();
    let ln_two = Real::from(2).ln().unwrap();
    let ln_three = Real::from(3).ln().unwrap();
    vec![
        ("One", fraction(3, 2), 1, 2),
        ("Pi", pi.clone(), 3, 4),
        ("PiPow", pi_squared.clone(), 9, 10),
        ("PiInv", pi.clone().inverse().unwrap(), 0, 1),
        ("PiExp", &pi * &e, 8, 9),
        ("PiInvExp", (&e / &pi).unwrap(), 0, 1),
        ("PiSqrt", &pi * &sqrt_two, 4, 5),
        ("ConstProduct", &pi_squared * &e, 24, 28),
        ("ConstOffset", &pi - Real::from(3), 0, 1),
        ("ConstProductSqrt", &(&pi_squared * &e) * &sqrt_two, 33, 42),
        ("Sqrt", sqrt_two, 1, 2),
        ("Exp", Real::from(2).exp().unwrap(), 7, 8),
        ("Ln", ln_three.clone(), 1, 2),
        ("LnAffine", (Real::from(2) * &e).ln().unwrap(), 1, 2),
        ("LnProduct", &ln_two * &ln_three, 0, 1),
        ("Log10", Real::from(2).log10().unwrap(), 0, 1),
        ("Log2", Real::from(3).log2().unwrap(), 1, 2),
        ("Pow10", fraction(1, 3).exp10().unwrap(), 2, 3),
        ("Pow2", fraction(1, 3).exp2().unwrap(), 1, 2),
        ("SinPi", fraction(1, 5).sin_pi(), 0, 1),
        ("TanPi", fraction(1, 5).tan_pi().unwrap(), 0, 1),
    ]
}

fn assert_tag(wire: &Value, expected: &str) {
    let tag = &wire["class"];
    assert!(
        tag == expected
            || tag
                .as_object()
                .is_some_and(|map| map.contains_key(expected))
    );
}

fn lazy_wire(value: &Real, scale: i64, omit: bool) -> Value {
    let mut wire = serde_json::to_value(value).unwrap();
    let original_scale: Rational = serde_json::from_value(wire["rational"].clone()).unwrap();
    wire["rational"] = serde_json::to_value(original_scale * Rational::from(scale)).unwrap();
    if omit {
        wire.as_object_mut().unwrap().remove("computable");
    } else {
        wire["computable"] = Value::Null;
    }
    wire
}

fn decode_json(wire: &Value) -> Real {
    Real::from_json(&wire.to_string()).unwrap()
}

fn decode_cbor(wire: &Value) -> Real {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(wire, &mut bytes).unwrap();
    Real::from_bytes(&bytes).unwrap()
}

fn assert_round_trip(value: Real, scale: i64, lower: i64, upper: i64, context: &str) {
    // Consume this exact object: cloning a lazy value can materialize its graph
    // and conceal a bug in the missing-payload branch of owned folding.
    let result = value.exp().unwrap().ln().unwrap();
    if scale == 0 {
        assert_eq!(result.exact_rational_normal_form(), Some(Rational::zero()));
    } else {
        let (lower, upper) = if scale < 0 {
            (upper * scale, lower * scale)
        } else {
            (lower * scale, upper * scale)
        };
        assert_bounds(
            &result,
            Rational::from(lower),
            Rational::from(upper),
            context,
        );
    }
}

fn assert_bounds(value: &Real, lower: Rational, upper: Rational, context: &str) {
    let [lo, hi] = value.certified_rational_interval(-32).unwrap();
    assert!(
        lower < lo && lo <= hi && hi < upper,
        "{context}: [{lo}, {hi}]"
    );
}

fn lazy_matrix(decode: fn(&Value) -> Real) {
    for (name, value, lower, upper) in certificates() {
        assert_tag(&serde_json::to_value(&value).unwrap(), name);
        for scale in [0, 1, -3] {
            for omit in [false, true] {
                let wire = lazy_wire(&value, scale, omit);
                let decoded = decode(&wire);
                assert!(serde_json::to_value(&decoded).unwrap()["computable"].is_null());
                assert_round_trip(decoded, scale, lower, upper, &wire.to_string());
            }
        }
    }
}

#[test]
fn json_lazy_certificates_preserve_value_when_consumed() {
    lazy_matrix(decode_json);
}

#[test]
fn cbor_lazy_certificates_preserve_value_when_consumed() {
    lazy_matrix(decode_cbor);
}

#[test]
fn materialized_certificates_keep_owned_composition_semantics() {
    for (name, value, lower, upper) in certificates() {
        assert_tag(&serde_json::to_value(&value).unwrap(), name);
        for scale in [0, 1, -3] {
            assert_round_trip(&value * Real::from(scale), scale, lower, upper, name);
        }
    }
}

fn forward_bounds(decode: fn(&Value) -> Real) {
    // 3 < pi < 4 and 2 < e < 3 imply 8 < exp(pi) < 81.
    // Cubing and reciprocating give the nonunit-scale bounds. Separately,
    // sinh(1) < pi < sinh(2) and sinh(2) < 3*pi < sinh(4).
    for omit in [false, true] {
        for scale in [0, 1, 3, -3] {
            let wire = lazy_wire(&Real::pi(), scale, omit);
            let exponential = decode(&wire).exp().unwrap();
            let inverse_sine = decode(&wire).asinh().unwrap();
            let context = wire.to_string();
            match scale {
                0 => {
                    assert_eq!(
                        exponential.exact_rational_normal_form(),
                        Some(Rational::one())
                    );
                    assert_eq!(
                        inverse_sine.exact_rational_normal_form(),
                        Some(Rational::zero())
                    );
                }
                1 => {
                    assert_bounds(
                        &exponential,
                        Rational::from(8),
                        Rational::from(81),
                        &context,
                    );
                    assert_bounds(&inverse_sine, Rational::one(), Rational::from(2), &context);
                }
                3 => {
                    assert_bounds(
                        &exponential,
                        Rational::from(512),
                        Rational::from(531441),
                        &context,
                    );
                    assert_bounds(
                        &inverse_sine,
                        Rational::from(2),
                        Rational::from(4),
                        &context,
                    );
                }
                -3 => {
                    assert_bounds(
                        &exponential,
                        Rational::fraction(1, 531441).unwrap(),
                        Rational::fraction(1, 512).unwrap(),
                        &context,
                    );
                    assert_bounds(
                        &inverse_sine,
                        Rational::from(-4),
                        Rational::from(-2),
                        &context,
                    );
                }
                _ => unreachable!(),
            }
        }
    }
}

#[test]
fn json_lazy_pi_has_independent_forward_function_bounds() {
    forward_bounds(decode_json);
}

#[test]
fn cbor_lazy_pi_has_independent_forward_function_bounds() {
    forward_bounds(decode_cbor);
}
