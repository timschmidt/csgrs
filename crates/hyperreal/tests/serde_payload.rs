#![cfg(feature = "serde")]

use ciborium::Value;
use hyperreal::{Problem, Rational, Real};
use serde_json::json;

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

fn cbor_bytes(value: &impl serde::Serialize) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

#[test]
fn json_deserializer_rejects_missing_opaque_payload() {
    for wire in missing_payloads() {
        let error = serde_json::from_value::<Real>(wire).unwrap_err();
        assert!(error.to_string().contains("Computable payload"));
    }
}

#[test]
fn json_wrapper_preserves_missing_payload_failure() {
    for wire in missing_payloads() {
        assert_eq!(Real::from_json(&wire.to_string()), Err(Problem::ParseError));
    }
}

#[test]
fn cbor_deserializer_rejects_missing_opaque_payload() {
    for wire in missing_payloads() {
        let bytes = cbor_bytes(&wire);
        let error = ciborium::de::from_reader::<Real, _>(bytes.as_slice()).unwrap_err();
        assert!(error.to_string().contains("Computable payload"));
    }
}

#[test]
fn cbor_wrapper_preserves_missing_payload_failure() {
    for wire in missing_payloads() {
        assert_eq!(
            Real::from_bytes(&cbor_bytes(&wire)),
            Err(Problem::ParseError)
        );
    }
}

#[test]
fn cbor_value_conversion_preserves_missing_payload_failure() {
    for wire in missing_payloads() {
        let value = Value::serialized(&wire).unwrap();
        assert_eq!(Real::try_from(&value), Err(Problem::ParseError));
        assert_eq!(Real::try_from(value), Err(Problem::ParseError));
    }
}

#[test]
fn enclosing_collections_cannot_publish_missing_payloads() {
    for wire in missing_payloads() {
        let json = json!([Real::one(), wire]);
        assert!(serde_json::from_value::<Vec<Real>>(json.clone()).is_err());
        assert!(ciborium::de::from_reader::<Vec<Real>, _>(cbor_bytes(&json).as_slice()).is_err());
    }
}

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

fn check_bounds(value: &Real, lower: i64, upper: i64) {
    let [lo, hi] = value.certified_rational_interval(-32).unwrap();
    assert!(lo >= Rational::new(lower) && hi <= Rational::new(upper));
    let translated = value + Real::one();
    let [lo, hi] = translated.certified_rational_interval(-32).unwrap();
    assert!(lo >= Rational::new(lower + 1) && hi <= Rational::new(upper + 1));
}

#[test]
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
