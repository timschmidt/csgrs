#![cfg(feature = "serde")]

use hyperreal::{Computable, Problem, Rational, Real, RealSign};
use serde_json::{Value, json};

fn node(kind: &str, argument: Value) -> Value {
    json!({"internal": {kind: argument}})
}

fn bytes(value: &Value) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

#[test]
fn imported_integral_atan_completes_and_preserves_following_arithmetic() {
    // atan(1)=pi/4 and 3<pi<22/7 independently give 3/4<atan(1)<11/14.
    // These broad bounds supplement the directed MPFR enclosure unit tests.
    let positive_lo = Rational::fraction(3, 4).unwrap();
    let positive_hi = Rational::fraction(11, 14).unwrap();
    for sign in [-1, 1] {
        let (lower, upper) = if sign < 0 {
            (-positive_hi.clone(), -positive_lo.clone())
        } else {
            (positive_lo.clone(), positive_hi.clone())
        };
        let graph = node("IntegralAtan", json!(num::BigInt::from(sign)));
        let collection = json!([node("Ratio", json!(Rational::one())), graph]);
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
                let unit: Rational = if p == 0 {
                    Rational::one()
                } else {
                    format!("1/{}", 1_u128 << -p).parse().unwrap()
                };
                let center = Rational::from_bigint(value.approx(p)) * &unit;
                assert!(&center - &unit < upper && &center + &unit > lower);
                if p == -64 {
                    assert!(&center - &unit > lower && &center + &unit < upper);
                }
            }
            assert_eq!(serde_json::to_value(value).unwrap(), graph);
        }
        for scale in [0, 1, -3] {
            let wire =
                json!({"rational":Rational::new(scale),"class":"Irrational","computable":graph});
            let cbor = ciborium::Value::serialized(&wire).unwrap();
            let collection = json!([Real::one(), wire]);
            let mut values = vec![
                Real::from_json(&wire.to_string()).unwrap(),
                Real::from_bytes(&bytes(&wire)).unwrap(),
                Real::try_from(&cbor).unwrap(),
                Real::try_from(cbor).unwrap(),
            ];
            let mut a: Vec<Real> = serde_json::from_value(collection.clone()).unwrap();
            let mut b: Vec<Real> =
                ciborium::de::from_reader(bytes(&collection).as_slice()).unwrap();
            values.push(a.pop().unwrap());
            values.push(b.pop().unwrap());
            let factor = Rational::new(scale);
            let (lo_bound, hi_bound) = if scale >= 0 {
                (&lower * &factor, &upper * &factor)
            } else {
                (&upper * &factor, &lower * &factor)
            };
            let expected_sign = if scale == 0 {
                RealSign::Zero
            } else if lo_bound > Rational::zero() {
                RealSign::Positive
            } else {
                RealSign::Negative
            };
            for value in values {
                let json = value.to_json();
                let cbor = value.to_bytes();
                for p in [0, -64, 0] {
                    let [lo, hi] = value.certified_rational_interval(p).unwrap();
                    if scale == 0 {
                        assert!(lo <= Rational::zero() && hi >= Rational::zero());
                    } else {
                        assert!(lo < hi_bound && hi > lo_bound);
                        if p == -64 {
                            assert!(lo > lo_bound && hi < hi_bound);
                        }
                    }
                    assert_eq!(value.certified_sign_until(p).sign(), Some(expected_sign));
                }
                let [lo, hi] = (&value + Real::one())
                    .certified_rational_interval(-32)
                    .unwrap();
                assert!(lo >= &lo_bound + Rational::one() && hi <= &hi_bound + Rational::one());
                assert_eq!(value.to_json(), json);
                assert_eq!(value.to_bytes(), cbor);
            }
        }
    }
}

#[test]
fn integral_atan_import_preserves_typed_zero_denominator_failures() {
    let graph = node("IntegralAtan", json!(num::BigInt::from(0)));
    for invalid in [
        graph.clone(),
        node(
            "Multiply",
            json!([node("Ratio", json!(Rational::zero())), graph]),
        ),
    ] {
        assert!(serde_json::from_value::<Computable>(invalid.clone()).is_err());
        assert!(ciborium::de::from_reader::<Computable, _>(bytes(&invalid).as_slice()).is_err());
        let collection = json!([node("Ratio", json!(Rational::one())), invalid]);
        assert!(serde_json::from_value::<Vec<Computable>>(collection.clone()).is_err());
        assert!(
            ciborium::de::from_reader::<Vec<Computable>, _>(bytes(&collection).as_slice()).is_err()
        );
        for scale in [0, 1, -3] {
            let wire =
                json!({"rational":Rational::new(scale),"class":"Irrational","computable":invalid});
            let cbor = ciborium::Value::serialized(&wire).unwrap();
            assert!(matches!(
                Real::from_json(&wire.to_string()),
                Err(Problem::ParseError)
            ));
            assert!(matches!(
                Real::from_bytes(&bytes(&wire)),
                Err(Problem::ParseError)
            ));
            assert!(matches!(Real::try_from(&cbor), Err(Problem::ParseError)));
            assert!(matches!(Real::try_from(cbor), Err(Problem::ParseError)));
            let collection = json!([Real::one(), wire]);
            assert!(serde_json::from_value::<Vec<Real>>(collection.clone()).is_err());
            assert!(
                ciborium::de::from_reader::<Vec<Real>, _>(bytes(&collection).as_slice()).is_err()
            );
        }
    }
}
