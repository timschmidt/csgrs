#![cfg(feature = "serde")]

use hyperreal::{Computable, Problem, Rational, Real, RealSign};
use serde_json::{Value, json};

fn q(n: i64) -> Value {
    serde_json::to_value(Rational::new(n)).unwrap()
}

fn node(kind: &str, value: Value) -> Value {
    json!({"internal": {kind: value}})
}

fn bytes(value: &Value) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

#[test]
fn imported_atan_preserves_refinement_interchange_and_following_arithmetic() {
    // For x >= 2, atan(x) = pi/2 - atan(1/x), 3 < pi < 4, and
    // 0 < atan(1/x) < 1/x <= 1/2 give 1 < atan(x) < 2.
    // The symbolic arguments pi and sqrt(5) are also greater than two.
    // These bounds are independent of the implementation being tested.
    let zero = node("Ratio", q(0));
    for positive in [
        node("Ratio", q(2)),
        node("Add", json!([node("Ratio", q(2)), zero])),
        node("Ratio", q(4)),
        node("Add", json!([node("Ratio", q(4)), zero])),
        node("Constant", json!("Pi")),
        node("Sqrt", node("Ratio", q(5))),
    ] {
        for sign in [-1, 1] {
            let argument = if sign < 0 {
                node("Negate", positive.clone())
            } else {
                positive.clone()
            };
            let graph = node("PrescaledAtan", argument);
            let collection = json!([node("Ratio", q(1)), graph]);
            let mut json_collection: Vec<Computable> =
                serde_json::from_value(collection.clone()).unwrap();
            let mut cbor_collection: Vec<Computable> =
                ciborium::de::from_reader(bytes(&collection).as_slice()).unwrap();
            let values = [
                serde_json::from_value::<Computable>(graph.clone()).unwrap(),
                ciborium::de::from_reader(bytes(&graph).as_slice()).unwrap(),
                json_collection.pop().unwrap(),
                cbor_collection.pop().unwrap(),
            ];
            for value in values {
                assert_eq!(serde_json::to_value(&value).unwrap(), graph);
                for p in [0, -64, 0] {
                    let approximation = value.approx(p);
                    assert!(if sign > 0 {
                        approximation > 0.into()
                    } else {
                        approximation < 0.into()
                    });
                }
                assert_eq!(serde_json::to_value(value).unwrap(), graph);
            }
            for scale in [0, 1, -3] {
                let wire = json!({"rational":q(scale),"class":"Irrational","computable":graph});
                let cbor = ciborium::Value::serialized(&wire).unwrap();
                let collection = json!([Real::one(), wire]);
                let mut values = vec![
                    Real::from_json(&wire.to_string()).unwrap(),
                    Real::from_bytes(&bytes(&wire)).unwrap(),
                    Real::try_from(&cbor).unwrap(),
                    Real::try_from(cbor).unwrap(),
                ];
                let mut json_collection: Vec<Real> =
                    serde_json::from_value(collection.clone()).unwrap();
                let mut cbor_collection: Vec<Real> =
                    ciborium::de::from_reader(bytes(&collection).as_slice()).unwrap();
                values.push(json_collection.pop().unwrap());
                values.push(cbor_collection.pop().unwrap());
                let factor = sign * scale;
                let expected_sign = match factor.cmp(&0) {
                    std::cmp::Ordering::Less => RealSign::Negative,
                    std::cmp::Ordering::Equal => RealSign::Zero,
                    std::cmp::Ordering::Greater => RealSign::Positive,
                };
                let (lower, upper) = if factor >= 0 {
                    (factor, 2 * factor)
                } else {
                    (2 * factor, factor)
                };
                for value in values {
                    let json = value.to_json();
                    let cbor = value.to_bytes();
                    for p in [2, 0, -32, -128, 0] {
                        let [lo, hi] = value.certified_rational_interval(p).unwrap();
                        if factor == 0 {
                            assert!(lo <= Rational::zero() && hi >= Rational::zero());
                        } else {
                            assert!(lo < Rational::new(upper) && hi > Rational::new(lower));
                            if p < 0 {
                                assert!(lo > Rational::new(lower) && hi < Rational::new(upper));
                            }
                        }
                        assert_eq!(value.certified_sign_until(p).sign(), Some(expected_sign));
                    }
                    let translated = &value + Real::one();
                    let [lo, hi] = translated.certified_rational_interval(-32).unwrap();
                    assert!(lo >= Rational::new(lower + 1) && hi <= Rational::new(upper + 1));
                    assert_eq!(value.to_json(), json);
                    assert_eq!(value.to_bytes(), cbor);
                }
            }
        }
    }
}

#[test]
fn imported_atan_preserves_typed_rejection_of_invalid_children() {
    for child in [
        node("Sqrt", node("Ratio", q(-1))),
        node("Inverse", node("Ratio", q(0))),
        node("PrescaledLn", node("Ratio", q(-2))),
    ] {
        let atan = node("PrescaledAtan", child);
        for graph in [
            atan.clone(),
            node("Multiply", json!([node("Ratio", q(0)), atan])),
        ] {
            assert!(serde_json::from_value::<Computable>(graph.clone()).is_err());
            assert!(ciborium::de::from_reader::<Computable, _>(bytes(&graph).as_slice()).is_err());
            for scale in [0, 1, -3] {
                let wire = json!({"rational":q(scale),"class":"Irrational","computable":graph});
                let cbor = ciborium::Value::serialized(&wire).unwrap();
                for result in [
                    Real::from_json(&wire.to_string()),
                    Real::from_bytes(&bytes(&wire)),
                    Real::try_from(&cbor),
                    Real::try_from(cbor),
                ] {
                    assert!(matches!(result, Err(Problem::ParseError)));
                }
                let collection = json!([Real::one(), wire]);
                assert!(serde_json::from_value::<Vec<Real>>(collection.clone()).is_err());
                assert!(
                    ciborium::de::from_reader::<Vec<Real>, _>(bytes(&collection).as_slice())
                        .is_err()
                );
            }
        }
    }
}
