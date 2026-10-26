#![cfg(feature = "serde")]

use hyperreal::{Problem, Rational, Real, RealSign};
use num::{BigInt, BigUint};
use serde_json::{Value, json};

// floor(pi * 2^2304), independently bracketed using exact alternating
// series for 16 atan(1/5) - 4 atan(1/239). The next integer is above pi.
const PI_BELOW_2304: &str = "3243f6a8885a308d313198a2e03707344a4093822299f31d0082efa98ec4e6c89452821e638d01377be5466cf34e90c6cc0ac29b7c97c50dd3f84d5b5b54709179216d5d98979fb1bd1310ba698dfb5ac2ffd72dbd01adfb7b8e1afed6a267e96ba7c9045f12c7f9924a19947b3916cf70801f2e2858efc16636920d871574e69a458fea3f4933d7e0d95748f728eb658718bcd5882154aee7b54a41dc25a59b59c30d5392af26013c5d1b023286085f0ca417918b8db38ef8e79dcb0603a180e6c9e0e8bb01e8a3ed71577c1bd314b2778af2fda55605c60e65525f3aa55ab945748986263e8144055ca396a2aab10b6b4cc5c341141e8cea15486af7c72e993b3ee1411636fbc2a2ba9c55d741831f6ce5c3e169b87931eafd6ba336c24cf5c";

#[test]
fn native_near_pole_cotangent_keeps_its_checked_domain_after_import() {
    let below = BigInt::parse_bytes(PI_BELOW_2304.as_bytes(), 16).unwrap();
    for above in [false, true] {
        let n = &below + if above { 1 } else { 0 };
        let q = Rational::from_bigint_fraction(n, BigUint::from(1u8) << 2304).unwrap();
        let native = Real::new(q).cot().unwrap();
        let expected = if above {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        let json = native.to_json();
        let cbor = native.to_bytes();
        assert_eq!(native.certified_sign_until(-2048).sign(), Some(expected));
        drop(native);
        let encoded: ciborium::Value = ciborium::de::from_reader(cbor.as_slice()).unwrap();
        for value in [
            Real::from_json(&json),
            Real::from_bytes(&cbor),
            Real::try_from(&encoded),
        ] {
            let restored = value.unwrap();
            assert_eq!(restored.to_json(), json);
            assert_eq!(restored.to_bytes(), cbor);
            assert_eq!(restored.certified_sign_until(-2048).sign(), Some(expected));
            let residual = restored.clone().inverse().unwrap() * restored - Real::one();
            // The scalar sign query does not run the full relation cascade.
            // Request the same exact algebraic proof used by Hyperlimit before
            // requiring its retained sign. No approximate terminal is allowed.
            assert_eq!(
                residual
                    .exact_rational_normal_form()
                    .or_else(|| residual.extended_rational_normal_form()),
                Some(Rational::zero())
            );
            assert_eq!(
                residual.certified_sign_until(-2048).sign(),
                Some(RealSign::Zero)
            );
        }
    }
}

fn near_pi() -> Rational {
    Rational::from_bigint_fraction(
        BigInt::parse_bytes(PI_BELOW_2304.as_bytes(), 16).unwrap(),
        BigUint::from(1u8) << 2304,
    )
    .unwrap()
}

fn node(kind: &str, argument: Value) -> Value {
    json!({"internal": {kind: argument}})
}

fn imports(wire: &Value) -> [Result<Real, Problem>; 4] {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(wire, &mut bytes).unwrap();
    let encoded = ciborium::Value::serialized(wire).unwrap();
    [
        Real::from_json(&wire.to_string()),
        Real::from_bytes(&bytes),
        Real::try_from(&encoded),
        Real::try_from(encoded),
    ]
}

#[test]
fn extreme_small_trig_imports_preserve_exact_sign_and_reciprocal_continuation() {
    for negative in [false, true] {
        let argument = node(
            "Offset",
            json!([
                node(
                    "Add",
                    json!([
                        node("Ratio", json!(Rational::one())),
                        node("Constant", json!("Pi")),
                    ])
                ),
                i32::MIN,
            ]),
        );
        let argument = if negative {
            node("Negate", argument)
        } else {
            argument
        };
        // 3 < pi < 4 proves the unsigned argument is strictly between 0
        // and 1. Sine, tangent and cotangent have its sign; cosine is positive.
        for kind in [
            "PrescaledSin",
            "PrescaledCos",
            "PrescaledTan",
            "PrescaledCot",
        ] {
            for reciprocal in [false, true] {
                let image = node(kind, argument.clone());
                let graph = if reciprocal {
                    node("Inverse", image)
                } else {
                    image
                };
                let wire =
                    json!({"rational":Rational::one(),"class":"Irrational","computable":graph});
                let expected = if negative && kind != "PrescaledCos" {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                };
                for restored in imports(&wire) {
                    let restored = restored.unwrap();
                    assert_eq!(serde_json::to_value(&restored).unwrap(), wire);
                    for precision in [-8, -2048, -8] {
                        assert_eq!(
                            restored.certified_sign_until(precision).sign(),
                            Some(expected)
                        );
                    }
                    let inverse = restored.inverse_ref().unwrap();
                    assert_eq!(inverse.certified_sign_until(-2048).sign(), Some(expected));
                    let residual = inverse * restored - Real::one();
                    assert_eq!(
                        residual
                            .exact_rational_normal_form()
                            .or_else(|| residual.extended_rational_normal_form()),
                        Some(Rational::zero())
                    );
                }
            }
        }
    }
}

#[test]
fn signed_scales_and_valid_zero_wrappers_preserve_cold_cotangent_domains() {
    for negative in [false, true] {
        for scale in [2, -3] {
            let q = if negative { -near_pi() } else { near_pi() };
            let value = Real::new(q).cot().unwrap() * Real::from(scale);
            let wire = serde_json::to_value(&value).unwrap();
            drop(value);
            let expected = if negative == (scale > 0) {
                RealSign::Positive
            } else {
                RealSign::Negative
            };
            for value in imports(&wire) {
                let value = value.unwrap();
                assert_eq!(serde_json::to_value(&value).unwrap(), wire);
                assert_eq!(value.certified_sign_until(-2048).sign(), Some(expected));
                let identity = value.inverse_ref().unwrap() * value - Real::one();
                assert_eq!(
                    identity
                        .exact_rational_normal_form()
                        .or_else(|| identity.extended_rational_normal_form()),
                    Some(Rational::zero())
                );
            }
        }
    }
    let source = serde_json::to_value(Real::new(near_pi()).cot().unwrap()).unwrap();
    let graph = source["computable"].clone();
    let zero = node("Ratio", json!(Rational::zero()));
    for (scale, graph) in [
        (0, graph.clone()),
        (1, node("Multiply", json!([zero, graph]))),
        (
            -3,
            node("Add", json!([graph, node("Negate", graph.clone())])),
        ),
    ] {
        let wire = json!({"rational":Rational::new(scale),"class":"Irrational","computable":graph});
        for value in imports(&wire) {
            let value = value.unwrap();
            assert_eq!(serde_json::to_value(&value).unwrap(), wire);
            // The strict cascade consumes a structural zero before requesting
            // a rational normal form; an opaque zero outer scale need not
            // normalize its nonrational payload. Every route still proves zero.
            if let Some(sign) = value.certified_sign_until(-2048).sign() {
                assert_eq!(sign, RealSign::Zero);
            } else {
                assert_eq!(
                    value
                        .exact_rational_normal_form()
                        .or_else(|| value.extended_rational_normal_form()),
                    Some(Rational::zero())
                );
                assert_eq!(
                    value.certified_sign_until(-2048).sign(),
                    Some(RealSign::Zero)
                );
            }
        }
    }
}

#[test]
fn nonpole_replay_cannot_hide_invalid_or_unproved_children() {
    let pi = node("Constant", json!("Pi"));
    let zero = node("Ratio", json!(Rational::zero()));
    let q = node("Ratio", json!(near_pi()));
    let affine = node("Add", json!([q, node("Negate", pi.clone())]));
    let bad_children = [
        (node("Inverse", zero.clone()), Problem::ParseError),
        (
            node("Sqrt", node("Negate", pi.clone())),
            Problem::ParseError,
        ),
        (
            node("Inverse", node("PrescaledSin", pi)),
            Problem::UnknownCertificate,
        ),
    ];
    for (bad, expected) in bad_children {
        for hidden in [
            node("Multiply", json!([zero, bad])),
            node("Add", json!([bad, node("Negate", bad.clone())])),
        ] {
            let argument = node("Add", json!([affine, hidden]));
            for kind in ["PrescaledSin", "PrescaledCos"] {
                let reciprocal = node("Inverse", node("Negate", node(kind, argument.clone())));
                for graph in [
                    reciprocal.clone(),
                    node("Multiply", json!([zero, reciprocal])),
                    node(
                        "Add",
                        json!([reciprocal, node("Negate", reciprocal.clone())]),
                    ),
                ] {
                    for scale in [0, 1, -3] {
                        let wire = json!({"rational":Rational::new(scale),"class":"Irrational","computable":graph});
                        for value in imports(&wire) {
                            assert_eq!(value.map(|_| ()), Err(expected));
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn aborted_native_query_does_not_poison_cold_domain_replay_or_retry() {
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };

    let mut value = Real::new(near_pi()).cot().unwrap();
    let wire = value.to_json();
    let stopped = Arc::new(AtomicBool::new(true));
    value.abort(stopped.clone());
    assert_eq!(value.certified_sign_until(-2048).sign(), None);
    assert_eq!(value.to_json(), wire);
    // Signals are process-local controls, not serialized mathematical facts.
    let restored = Real::from_json(&wire).unwrap();
    assert_eq!(
        restored.certified_sign_until(-2048).sign(),
        Some(RealSign::Negative)
    );
    stopped.store(false, Ordering::Relaxed);
    assert_eq!(
        value.certified_sign_until(-2048).sign(),
        Some(RealSign::Negative)
    );
    assert_eq!(value.to_json(), wire);
}
