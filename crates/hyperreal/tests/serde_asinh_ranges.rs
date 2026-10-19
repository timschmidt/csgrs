#![cfg(feature = "serde")]

use hyperreal::{Computable, Rational, Real, RealSign};
use serde_json::{Value, json};

fn q(n: i64, d: u64) -> Value {
    serde_json::to_value(Rational::fraction(n, d).unwrap()).unwrap()
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
fn imported_asinh_kernels_enclose_large_values_before_and_after_refinement() {
    // e < 11/4 gives sinh(2) < 121/32 < 4. Also e > 8/3 gives
    // sinh(3) > ((8/3)^3 - 1)/2 > 4. Thus 2 < asinh(4) < 3.
    // These bounds are independent of the implementation being tested.
    for sign in [-1, 1] {
        let argument = node("Ratio", q(4 * sign, 1));
        for graph in [
            node("AsinhRational", q(4 * sign, 1)),
            node("PrescaledAsinh", argument.clone()),
            node(
                "PrescaledAsinh",
                node("Add", json!([argument, node("Ratio", q(0, 1))])),
            ),
        ] {
            let a: Computable = serde_json::from_value(graph.clone()).unwrap();
            let b: Computable = ciborium::de::from_reader(bytes(&graph).as_slice()).unwrap();
            for value in [a, b] {
                let wire = serde_json::to_value(&value).unwrap();
                for p in [1, -64, 1] {
                    let approximation = value.approx(p);
                    assert!(if sign > 0 {
                        approximation > 0.into()
                    } else {
                        approximation < 0.into()
                    });
                }
                assert_eq!(serde_json::to_value(value).unwrap(), wire);
            }
            for scale in [0, 1, -3] {
                let wire = json!({"rational":q(scale, 1),"class":"Irrational","computable":graph});
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
                    (2 * factor, 3 * factor)
                } else {
                    (3 * factor, 2 * factor)
                };
                for value in values {
                    let json = value.to_json();
                    let cbor = value.to_bytes();
                    for p in [1, -64, 1] {
                        let [lo, hi] = value.certified_rational_interval(p).unwrap();
                        if factor == 0 {
                            assert!(lo <= Rational::zero() && hi >= Rational::zero());
                        } else {
                            assert!(lo < Rational::new(upper) && hi > Rational::new(lower));
                            if p == -64 {
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
fn asinh_series_accept_symbolic_arguments_and_small_values() {
    // sinh(1) < (e-1/e)/2 < (11/4-4/11)/2 < sqrt(2), while
    // sinh(2) > ((8/3)^2-(3/8)^2)/2 > 22/7 > pi. Both values lie in (1,2).
    for constant in ["Pi", "Sqrt2"] {
        for sign in [-1, 1] {
            let argument = node("Constant", json!(constant));
            let argument = if sign < 0 {
                node("Negate", argument)
            } else {
                argument
            };
            let graph = node("PrescaledAsinh", argument);
            let wire = json!({"rational":q(1, 1),"class":"Irrational","computable":graph});
            let value = Real::from_json(&wire.to_string()).unwrap();
            let [lo, hi] = value.certified_rational_interval(-64).unwrap();
            let (lower, upper) = if sign > 0 { (1, 2) } else { (-2, -1) };
            assert!(lo > Rational::new(lower) && hi < Rational::new(upper));
        }
    }
    for n in [-1, 0, 1] {
        for kind in ["PrescaledAsinh", "AsinhRational"] {
            let argument = if kind == "PrescaledAsinh" {
                node("Ratio", q(n, 16))
            } else {
                q(n, 16)
            };
            let graph = node(kind, argument);
            let wire = json!({"rational":q(1, 1),"class":"Irrational","computable":graph});
            let value = Real::from_bytes(&bytes(&wire)).unwrap();
            let [lo, hi] = value.certified_rational_interval(-64).unwrap();
            // asinh is odd and strictly increasing, with |asinh(x)| < |x|
            // for nonzero x. Include the exact-zero continuation separately.
            if n == 0 {
                assert!(lo <= Rational::zero() && hi >= Rational::zero());
            } else if n > 0 {
                assert!(lo > Rational::zero() && hi < Rational::fraction(1, 16).unwrap());
            } else {
                assert!(lo > Rational::fraction(-1, 16).unwrap() && hi < Rational::zero());
            }
        }
    }
}
