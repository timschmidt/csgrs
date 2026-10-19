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
fn imported_rational_atanh_preserves_enclosures_and_following_arithmetic() {
    // atanh(31/32)=ln(63)/2. From 8/3<e<11/4, e^4<63<e^6,
    // hence 2<atanh(31/32)<3 independently of the scalar evaluator.
    let mut cases = vec![(
        Rational::fraction(31, 32).unwrap(),
        1,
        Rational::new(2),
        Rational::new(3),
    )];
    // atanh((2^256-1)/(2^256+1))=128*ln(2). The independent
    // bounds e^2<8 and e>2 give 256/3 < 128*ln(2) < 128.
    let power: num::BigInt = num::BigInt::from(1) << 256;
    let near_one = Rational::from_bigint_fraction(
        &power - 1,
        (&power + num::BigInt::from(1)).to_biguint().unwrap(),
    )
    .unwrap();
    cases.push((
        near_one,
        4,
        Rational::fraction(256, 3).unwrap(),
        Rational::new(128),
    ));
    for (magnitude, coarse, positive_lo, positive_hi) in cases {
        for negative in [false, true] {
            let (argument, lower, upper) = if negative {
                (
                    -magnitude.clone(),
                    -positive_hi.clone(),
                    -positive_lo.clone(),
                )
            } else {
                (magnitude.clone(), positive_lo.clone(), positive_hi.clone())
            };
            let graph = node("AtanhRational", serde_json::to_value(argument).unwrap());
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
                for p in [coarse, -64, coarse] {
                    let a = Rational::from_bigint(value.approx(p));
                    let unit: Rational = if p >= 0 {
                        Rational::new(1_i64 << p)
                    } else {
                        format!("1/{}", 1_u128 << -p).parse().unwrap()
                    };
                    let center = a * &unit;
                    assert!(&center - &unit < upper && &center + &unit > lower);
                }
                assert_eq!(serde_json::to_value(value).unwrap(), graph);
            }
            for scale in [0, 1, -3] {
                let wire = json!({"rational":Rational::new(scale),"class":"Irrational","computable":graph});
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
                let sign = if scale == 0 {
                    RealSign::Zero
                } else if lo_bound > Rational::zero() {
                    RealSign::Positive
                } else {
                    RealSign::Negative
                };
                for value in values {
                    let json = value.to_json();
                    let cbor = value.to_bytes();
                    for p in [coarse, -64, coarse] {
                        let [lo, hi] = value.certified_rational_interval(p).unwrap();
                        if scale == 0 {
                            assert!(lo <= Rational::zero() && hi >= Rational::zero());
                        } else {
                            assert!(lo < hi_bound && hi > lo_bound);
                            if p == -64 {
                                assert!(lo > lo_bound && hi < hi_bound);
                            }
                        }
                        assert_eq!(value.certified_sign_until(p).sign(), Some(sign));
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
}

#[test]
fn rational_atanh_import_preserves_typed_domain_failures() {
    for argument in [-2, -1, 1, 2] {
        let graph = node("AtanhRational", json!(Rational::new(argument)));
        for invalid in [
            graph.clone(),
            node(
                "Multiply",
                json!([node("Ratio", json!(Rational::zero())), graph]),
            ),
        ] {
            assert!(serde_json::from_value::<Computable>(invalid.clone()).is_err());
            assert!(
                ciborium::de::from_reader::<Computable, _>(bytes(&invalid).as_slice()).is_err()
            );
            let collection = json!([node("Ratio", json!(Rational::one())), invalid]);
            assert!(serde_json::from_value::<Vec<Computable>>(collection.clone()).is_err());
            assert!(
                ciborium::de::from_reader::<Vec<Computable>, _>(bytes(&collection).as_slice())
                    .is_err()
            );
            for scale in [0, 1, -3] {
                let wire = json!({"rational":Rational::new(scale),"class":"Irrational","computable":invalid});
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
                    ciborium::de::from_reader::<Vec<Real>, _>(bytes(&collection).as_slice())
                        .is_err()
                );
            }
        }
    }
}
