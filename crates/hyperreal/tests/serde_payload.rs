//! Opaque-payload presence admission through JSON, CBOR and Value adapters.
//!
//! Malformed opaque definitions must fail even under zero outer scale or inside
//! a collection. Selected valid definitions are replayed, queried and translated.
//! Interval containment and byte stability are separate assertions; they are not
//! by themselves a universal proof of the intended scalar value or graph identity.

#![cfg(feature = "serde")]

use ciborium::Value;
use hyperreal::{Problem, Rational, Real};
use serde_json::json;

/// Build six invalid opaque wire objects: scales 0, 1 and -3, each with null or omitted computable payload.
/// These preserve the missing-definition defect even when the outer numeric scale is zero.
fn missing_payloads() -> Vec<serde_json::Value> {
    // A zero outer scale must not conceal an invalid inner representation.
    [0, 1, -3]
        .into_iter()
        .flat_map(|scale| {
            let mut wire = json!({
                "rational": Rational::new(scale),
                "class": "Irrational",
                "computable": null,
            });
            let explicit_null = wire.clone();
            wire.as_object_mut().unwrap().remove("computable");
            [explicit_null, wire]
        })
        .collect()
}

/// Serialize a fixture into an owned CBOR vector; panic on encoding failure, without validating its Real meaning.
fn cbor_bytes(value: &impl serde::Serialize) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

#[test]
/// Require direct JSON Serde admission to reject every missing-payload fixture and retain the payload diagnostic text.
fn json_deserializer_rejects_missing_opaque_payload() {
    for wire in missing_payloads() {
        let error = serde_json::from_value::<Real>(wire).unwrap_err();
        assert!(error.to_string().contains("Computable payload"));
    }
}

#[test]
/// Require the public JSON wrapper to retain ParseError for all six malformed opaque definitions.
fn json_wrapper_preserves_missing_payload_failure() {
    for wire in missing_payloads() {
        assert_eq!(Real::from_json(&wire.to_string()), Err(Problem::ParseError));
    }
}

#[test]
/// Require direct CBOR Serde admission to reject each malformed fixture with the payload diagnostic text.
fn cbor_deserializer_rejects_missing_opaque_payload() {
    for wire in missing_payloads() {
        let bytes = cbor_bytes(&wire);
        let error = ciborium::de::from_reader::<Real, _>(bytes.as_slice()).unwrap_err();
        assert!(error.to_string().contains("Computable payload"));
    }
}

#[test]
/// Require the public CBOR byte wrapper to retain ParseError for each encoded malformed fixture.
fn cbor_wrapper_preserves_missing_payload_failure() {
    for wire in missing_payloads() {
        assert_eq!(
            Real::from_bytes(&cbor_bytes(&wire)),
            Err(Problem::ParseError)
        );
    }
}

#[test]
/// Require borrowed and owned CBOR Value adapters to reject the same six malformed definitions with ParseError.
fn cbor_value_conversion_preserves_missing_payload_failure() {
    for wire in missing_payloads() {
        let value = Value::serialized(&wire).unwrap();
        assert_eq!(Real::try_from(&value), Err(Problem::ParseError));
        assert_eq!(Real::try_from(value), Err(Problem::ParseError));
    }
}

#[test]
/// Place each malformed definition after a valid one and require whole `Vec<Real>` decoding to fail in both formats.
/// No partial collection result is accepted; this does not inspect temporary allocations or decoder side effects.
fn enclosing_collections_cannot_publish_missing_payloads() {
    for wire in missing_payloads() {
        let json = json!([Real::one(), wire]);
        assert!(serde_json::from_value::<Vec<Real>>(json.clone()).is_err());
        assert!(ciborium::de::from_reader::<Vec<Real>, _>(cbor_bytes(&json).as_slice()).is_err());
    }
}

/// Return six value/bound fixtures: zero, -3/7, pi, e, sqrt(2), and opaque pi+e.
/// The integer bounds follow exact rationals and elementary constant inequalities, not sampled float approximations.
fn valid_values() -> Vec<(Real, i64, i64)> {
    vec![
        (Real::zero(), 0, 0),
        (Real::new(Rational::fraction(-3, 7).unwrap()), -1, 0),
        (Real::pi(), 3, 4),
        (Real::e(), 2, 3),
        (Real::from(2).sqrt().unwrap(), 1, 2),
        // 3 < pi < 22/7 and 8/3 < e < 11/4 imply 5 < pi + e < 6.
        (Real::pi() + Real::e(), 5, 6),
    ]
}

/// Require a certified interval at precision -32 inside the supplied integer bounds,
/// then translate by exact one and require correspondingly translated bounds.
/// The small fixture bounds make integer endpoint arithmetic safe; containment alone does not identify a unique real.
fn check_bounds(value: &Real, lower: i64, upper: i64) {
    let [lo, hi] = value.certified_rational_interval(-32).unwrap();
    assert!(lo >= Rational::new(lower) && hi <= Rational::new(upper));
    let translated = value + Real::one();
    let [lo, hi] = translated.certified_rational_interval(-32).unwrap();
    assert!(lo >= Rational::new(lower + 1) && hi <= Rational::new(upper + 1));
}

#[test]
/// Confirm pi+e uses the opaque tag, then replay all six fixtures through JSON and CBOR.
/// After bound/translation checks, require both encodings to reproduce the original bytes/text.
/// This combines numerical enclosure and deterministic encoding witnesses without asserting graph identity.
fn valid_payloads_preserve_values_and_serialized_bytes() {
    let values = valid_values();
    assert_eq!(
        serde_json::to_value(&values.last().unwrap().0).unwrap()["class"],
        "Irrational"
    );
    for (value, lower, upper) in values {
        let json = value.to_json();
        let bytes = value.to_bytes();
        for decoded in [
            Real::from_json(&json).unwrap(),
            Real::from_bytes(&bytes).unwrap(),
        ] {
            check_bounds(&decoded, lower, upper);
            assert_eq!(decoded.to_json(), json);
            assert_eq!(decoded.to_bytes(), bytes);
        }
    }
}

#[test]
/// Remove the payload from the five nonopaque fixtures and decode through both formats.
/// Require bound/translation checks; this test does not assert post-query payload absence or identical encoding.
fn symbolic_certificates_remain_valid_without_materialized_payloads() {
    for (value, lower, upper) in valid_values().into_iter().take(5) {
        let mut wire = serde_json::to_value(value).unwrap();
        assert_ne!(wire["class"], "Irrational");
        wire.as_object_mut().unwrap().remove("computable");
        for decoded in [
            Real::from_json(&wire.to_string()).unwrap(),
            Real::from_bytes(&cbor_bytes(&wire)).unwrap(),
        ] {
            check_bounds(&decoded, lower, upper);
        }
    }
}
