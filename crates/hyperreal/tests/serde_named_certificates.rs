#![cfg(feature = "serde")]

use hyperreal::{Rational, Real, RealSign};
use serde_json::{Value, json};

#[path = "support/representations.rs"]
mod representations;

fn q(n: i64, d: u64) -> Value {
    serde_json::to_value(Rational::fraction(n, d).unwrap()).unwrap()
}

fn native(n: i64) -> Value {
    json!({"internal": {"Ratio": q(n, 1)}})
}

fn known_invalid_cases() -> Vec<(&'static str, Value, &'static str, bool)> {
    vec![
        (
            "pi-negative-payload",
            json!({"class":"Pi", "computable":native(-1)}),
            "pi is positive; supplied payload is -1",
            true,
        ),
        (
            "pi-positive-wrong-payload",
            json!({"class":"Pi", "computable":native(2)}),
            "3 < pi < 4, so payload 2 contradicts pi despite positive sign",
            true,
        ),
        (
            "one-negative-payload",
            json!({"class":"One", "computable":native(-1)}),
            "unit basis must equal 1, not -1",
            true,
        ),
        (
            "pi-pow-zero",
            json!({"class":{"PiPow":0}}),
            "pi^0 = 1 is rational; PiPow certificate requires n >= 2",
            true,
        ),
        (
            "sqrt-negative",
            json!({"class":{"Sqrt":q(-1,1)}}),
            "real sqrt(-1) is undefined",
            false,
        ),
        (
            "sqrt-zero",
            json!({"class":{"Sqrt":q(0,1)}}),
            "sqrt(0) = 0 contradicts nonzero basis",
            true,
        ),
        (
            "sqrt-square",
            json!({"class":{"Sqrt":q(4,1)}}),
            "sqrt(4) = 2 contradicts irrationality",
            true,
        ),
        (
            "pisqrt-negative",
            json!({"class":{"PiSqrt":q(-1,1)}}),
            "pi*sqrt(-1) is undefined over reals",
            false,
        ),
        (
            "pisqrt-zero",
            json!({"class":{"PiSqrt":q(0,1)}}),
            "pi*sqrt(0) = 0 contradicts nonzero basis",
            true,
        ),
        (
            "exp-zero",
            json!({"class":{"Exp":q(0,1)}}),
            "exp(0) = 1 contradicts irrationality",
            true,
        ),
        (
            "ln-half",
            json!({"class":{"Ln":q(1,2)}}),
            "ln(1/2) is negative",
            true,
        ),
        ("ln-one", json!({"class":{"Ln":q(1,1)}}), "ln(1) = 0", true),
        (
            "ln-zero",
            json!({"class":{"Ln":q(0,1)}}),
            "real ln(0) is undefined",
            false,
        ),
        (
            "log10-one",
            json!({"class":{"Log10":q(1,1)}}),
            "log10(1) = 0",
            true,
        ),
        (
            "log10-ten",
            json!({"class":{"Log10":q(10,1)}}),
            "log10(10) = 1 is rational",
            true,
        ),
        (
            "log10-hundred",
            json!({"class":{"Log10":q(100,1)}}),
            "log10(100) = 2 is rational",
            true,
        ),
        (
            "log2-one",
            json!({"class":{"Log2":q(1,1)}}),
            "log2(1) = 0",
            true,
        ),
        (
            "log2-eight",
            json!({"class":{"Log2":q(8,1)}}),
            "log2(8) = 3 is rational",
            true,
        ),
        (
            "pow10-zero",
            json!({"class":{"Pow10":q(0,1)}}),
            "10^0 = 1 is rational",
            true,
        ),
        (
            "pow10-two",
            json!({"class":{"Pow10":q(2,1)}}),
            "10^2 = 100 is rational",
            true,
        ),
        (
            "pow2-zero",
            json!({"class":{"Pow2":q(0,1)}}),
            "2^0 = 1 is rational",
            true,
        ),
        (
            "pow2-three",
            json!({"class":{"Pow2":q(3,1)}}),
            "2^3 = 8 is rational",
            true,
        ),
        (
            "sinpi-zero",
            json!({"class":{"SinPi":q(0,1)}}),
            "sin(0) = 0",
            true,
        ),
        (
            "sinpi-negative",
            json!({"class":{"SinPi":q(-1,5)}}),
            "sin(-pi/5) is negative",
            true,
        ),
        (
            "sinpi-sixth",
            json!({"class":{"SinPi":q(1,6)}}),
            "sin(pi/6) = 1/2 is rational",
            true,
        ),
        (
            "tanpi-zero",
            json!({"class":{"TanPi":q(0,1)}}),
            "tan(0) = 0",
            true,
        ),
        (
            "tanpi-quarter",
            json!({"class":{"TanPi":q(1,4)}}),
            "tan(pi/4) = 1 is rational",
            true,
        ),
        (
            "tanpi-pole",
            json!({"class":{"TanPi":q(1,2)}}),
            "tan(pi/2) is undefined",
            false,
        ),
        (
            "constoffset-negative",
            json!({"class":{"ConstOffset":{"pi_power":1,"exp_power":q(0,1),"offset":q(-4,1)}}}),
            "pi-4 is negative",
            true,
        ),
        (
            "constproduct-zero-power",
            json!({"class":{"ConstProduct":{"pi_power":0,"exp_power":q(1,1)}}}),
            "pi^0*exp(1) = e, independent interval (2,3)",
            true,
        ),
        (
            "constproductsqrt-negative",
            json!({"class":{"ConstProductSqrt":{"pi_power":2,"exp_power":q(1,1),"radicand":q(-1,1)}}}),
            "negative square root is undefined",
            false,
        ),
        (
            "constproductsqrt-zero",
            json!({"class":{"ConstProductSqrt":{"pi_power":2,"exp_power":q(1,1),"radicand":q(0,1)}}}),
            "zero root makes entire basis zero",
            true,
        ),
        (
            "lnaffine-negative",
            json!({"class":{"LnAffine":{"offset":q(-3,1),"base":q(2,1)}}}),
            "ln(2)-3 is negative",
            true,
        ),
        (
            "lnaffine-zero-base",
            json!({"class":{"LnAffine":{"offset":q(1,1),"base":q(0,1)}}}),
            "ln(0) is undefined",
            false,
        ),
        (
            "lnproduct-zero",
            json!({"class":{"LnProduct":{"left":q(1,1),"right":q(2,1)}}}),
            "ln(1)*ln(2) = 0",
            true,
        ),
        (
            "lnproduct-negative",
            json!({"class":{"LnProduct":{"left":q(1,2),"right":q(2,1)}}}),
            "ln(1/2)*ln(2) is negative",
            true,
        ),
    ]
}

#[test]
fn known_invalid_named_certificates_fail_before_publication() {
    let mut accepted = Vec::new();
    for (name, mut wire, _, _) in known_invalid_cases() {
        // This parameter tuple denotes e. Its reconstruction bug needs a value
        // repair, not rejection of a valid constant-product certificate.
        if name == "constproduct-zero-power" {
            continue;
        }
        if wire.get("computable").is_none() {
            wire["computable"] = Value::Null;
        }
        for scale in [0, 1, -3] {
            wire["rational"] = q(scale, 1);
            let mut bytes = Vec::new();
            ciborium::ser::into_writer(&wire, &mut bytes).unwrap();
            let json_result = Real::from_json(&wire.to_string());
            let cbor_result = Real::from_bytes(&bytes);
            if json_result != Err(hyperreal::Problem::ParseError)
                || cbor_result != Err(hyperreal::Problem::ParseError)
            {
                accepted.push(format!("{name}/{scale}"));
            }
        }
    }
    assert!(
        accepted.is_empty(),
        "invalid certificates published: {accepted:?}"
    );
}

#[test]
fn zero_pi_power_retains_the_constant_product_value() {
    let mut incorrect = Vec::new();
    for exponent in [0, 1] {
        let wire = json!({
            "rational": q(1, 1),
            "class": {"ConstProduct": {"pi_power": 0, "exp_power": q(exponent, 1)}},
            "computable": null,
        });
        let mut bytes = Vec::new();
        ciborium::ser::into_writer(&wire, &mut bytes).unwrap();
        for value in [
            Real::from_json(&wire.to_string()).unwrap(),
            Real::from_bytes(&bytes).unwrap(),
        ] {
            // pi^0 is one. At exponent zero this is exactly one; at exponent
            // one, 2 < e < 3 gives an independent bound.
            let [lo, hi] = value.certified_rational_interval(-32).unwrap();
            let (lower, upper) = if exponent == 0 {
                let unit = Rational::fraction(1, 1_u64 << 32).unwrap();
                (Rational::one() - &unit, Rational::one() + unit)
            } else {
                (Rational::from(2), Rational::from(3))
            };
            if lo < lower || hi > upper {
                incorrect.push(format!("{wire}: [{lo}, {hi}]"));
            }
        }
    }
    assert!(
        incorrect.is_empty(),
        "wrong constant products: {incorrect:?}"
    );
}

fn cbor(wire: &impl serde::Serialize) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(wire, &mut bytes).unwrap();
    bytes
}

#[test]
fn public_named_payload_variants_and_lazy_certificates_keep_their_contract() {
    for case in representations::optimized_certificate_representatives() {
        let _ = case.public_kind;
        let value = case.value;
        let mut variants = vec![
            value.clone(),
            -value.clone(),
            &value * Real::from(3),
            &value + &value,
            &value * &value,
        ];
        let inverse = value.inverse_ref().unwrap();
        variants.push(inverse.inverse_ref().unwrap());
        variants.push(inverse);
        variants.push(value.sqrt().unwrap());
        for (index, value) in variants.into_iter().enumerate() {
            // All source recipes are strictly positive. Only the negated
            // variant changes sign; the other seven operations preserve it.
            let expected = if index == 1 {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            let json = value.to_json();
            let bytes = value.to_bytes();
            let named = serde_json::to_value(&value).unwrap()["class"] != "Irrational";
            for restored in [
                Real::from_json(&json).unwrap(),
                Real::from_bytes(&bytes).unwrap(),
            ] {
                assert_eq!(restored.to_json(), json, "{}", case.certificate);
                assert_eq!(restored.to_bytes(), bytes, "{}", case.certificate);
                let cold_sign = restored.structural_facts().sign;
                if named || cold_sign.is_some() {
                    assert_eq!(cold_sign, Some(expected), "{}: {json}", case.certificate);
                }
                // Opaque numeric caches were never serialized. In particular,
                // (pi - 3)^2 has a warm sign before export and a cold cache
                // after import. Require the independent mathematical sign and
                // its retention after refinement, not identical cache history.
                for precision in [-128, 0, -256, 0] {
                    assert_eq!(
                        restored.certified_sign_until(precision).sign(),
                        Some(expected),
                        "{}: {json}",
                        case.certificate,
                    );
                }
                assert_eq!(restored.to_json(), json);
                assert_eq!(restored.to_bytes(), bytes);
            }
            let mut lazy = serde_json::to_value(value).unwrap();
            if lazy["class"] != "Irrational" {
                lazy["computable"] = Value::Null;
                assert!(Real::from_json(&lazy.to_string()).is_ok(), "{lazy}");
                assert!(Real::from_bytes(&cbor(&lazy)).is_ok(), "{lazy}");
            }
        }
    }
}

#[test]
fn unproved_payload_equality_remains_distinct_from_invalid_input() {
    // pi + (sin(1) - sin(1)) is defined and equal to pi. The import proof
    // grammar intentionally does not cancel arbitrary opaque subexpressions:
    // the same rewrite would hide an undefined child in a hostile graph.
    let sine = json!({"internal":{"PrescaledSin":native(1)}});
    let zero = json!({"internal":{"Add":[sine, {"internal":{"Negate":sine}}]}});
    let graph = json!({"internal":{"Add":[{"internal":{"Constant":"Pi"}}, zero]}});
    for scale in [0, 1, -3] {
        let wire = json!({"rational":q(scale,1), "class":"Pi", "computable":graph});
        assert_eq!(
            Real::from_json(&wire.to_string()),
            Err(hyperreal::Problem::UnknownCertificate)
        );
        assert_eq!(
            Real::from_bytes(&cbor(&wire)),
            Err(hyperreal::Problem::UnknownCertificate)
        );
        let value = ciborium::Value::serialized(&wire).unwrap();
        assert_eq!(
            Real::try_from(&value),
            Err(hyperreal::Problem::UnknownCertificate)
        );
        assert_eq!(
            Real::try_from(value),
            Err(hyperreal::Problem::UnknownCertificate)
        );
        let collection = json!([Real::one(), wire]);
        let error = serde_json::from_value::<Vec<Real>>(collection.clone()).unwrap_err();
        assert!(error.to_string().contains("UnknownCertificate"));
        let error =
            ciborium::de::from_reader::<Vec<Real>, _>(cbor(&collection).as_slice()).unwrap_err();
        assert!(error.to_string().contains("UnknownCertificate"));
    }
}

#[test]
fn zero_factors_cannot_certify_undefined_payloads() {
    let zero = json!({"internal":{"Add":[native(1),native(-1)]}});
    let undefined = json!({"internal":{"Inverse":zero}});
    let hidden = json!({"internal":{"Multiply":[native(0),undefined]}});
    let exponent = json!({"internal":{"Add":[native(1),hidden]}});
    let graph = json!({"internal":{"PrescaledExp":exponent}});
    for scale in [0, 1] {
        let wire = json!({"rational":q(scale,1),"class":{"Exp":q(1,1)},"computable":graph});
        assert_eq!(
            Real::from_json(&wire.to_string()),
            Err(hyperreal::Problem::ParseError)
        );
        assert_eq!(
            Real::from_bytes(&cbor(&wire)),
            Err(hyperreal::Problem::ParseError)
        );
    }
}
