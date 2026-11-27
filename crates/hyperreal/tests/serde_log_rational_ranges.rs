//! Logarithm replay witnesses for domain validation, intervals and subsequent arithmetic.
//!
//! Raw rational residuals and binary-scaled logarithms exercise native and
//! checked-Real import routes. Broad independent bounds, certified API outcomes
//! and serialization stability are checked separately.

#![cfg(feature = "serde")]

use hyperreal::{Computable, Problem, Rational, Real, RealSign};
use serde_json::{Value, json};

/// Serialize an exact n/d rational for a raw import fixture, requiring d != 0.
/// Unexpected rational-construction or JSON-encoding failures panic in the test.
fn q(n: i64, d: u64) -> Value {
    serde_json::to_value(Rational::fraction(n, d).unwrap()).unwrap()
}

/// Wrap an argument in the named raw Computable syntax tag for import testing.
/// This builds JSON shape only; it deliberately does not admit the mathematical
/// domain or prove any stored facts.
fn node(kind: &str, argument: Value) -> Value {
    json!({"internal": {kind: argument}})
}

/// Encode a JSON fixture value as CBOR, panicking on an unexpected writer failure.
fn bytes(value: &Value) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

#[test]
/// Exercise eight valid raw log graphs with independent broad logarithm bounds.
/// Rational bounds on e give 1 < ln(5) < 2, -3 < ln(1/16) < -2 and
/// 2/3 < ln(2) < 1. Through two native import formats, check 48 approximation
/// interval overlaps over coarse/fine/coarse histories and preserve JSON values.
/// Overlap with these broad bounds alone does not certify the exact logarithm.
///
/// For each graph, three outer scales and six checked-Real import routes produce
/// 144 values. Check intervals and signs at three precisions, require strict
/// containment inside the broad bounds at -64 for nonzero scales, then test
/// containment after adding one. Preserve JSON and CBOR bytes separately from
/// those semantic checks. Zero scales require intervals containing zero. This
/// is same-process replay with shared caches, not a universal or fresh-process
/// replay proof.
fn imported_rational_logs_preserve_enclosures_and_following_arithmetic() {
    let mut cases = Vec::new();
    // e < 11/4 and e > 8/3 prove 1 < ln(5) < 2 and
    // -3 < ln(1/16) < -2 independently of another scalar evaluator.
    for (n, d, lower, upper) in [(4, 1, 1, 2), (-15, 16, -3, -2)] {
        for graph in [
            node("PrescaledLnRational", q(n, d)),
            node(
                "BinaryScaledLnRational",
                json!({"residual": q(n, d), "shift": 0}),
            ),
        ] {
            cases.push((graph, 0, Rational::new(lower), Rational::new(upper)));
        }
    }
    // e^2 < 8 and e > 2 give 2/3 < ln(2) < 1. These exact dyadic
    // arguments give (16+shift)*ln(2) or (-16+shift)*ln(2).
    for (n, d, exponent) in [(65535, 1, 16), (-65535, 65536, -16)] {
        for shift in [-3, 3] {
            let coefficient = exponent + shift;
            let a = Rational::fraction(2 * coefficient, 3).unwrap();
            let b = Rational::new(coefficient);
            let (lo, hi) = if coefficient > 0 { (a, b) } else { (b, a) };
            let graph = node(
                "BinaryScaledLnRational",
                json!({"residual": q(n, d), "shift": shift}),
            );
            cases.push((graph, 2, lo, hi));
        }
    }
    for (graph, coarse, lower, upper) in cases {
        let a: Computable = serde_json::from_value(graph.clone()).unwrap();
        let b: Computable = ciborium::de::from_reader(bytes(&graph).as_slice()).unwrap();
        for value in [a, b] {
            let wire = serde_json::to_value(&value).unwrap();
            for p in [coarse, -64, coarse] {
                let a: Rational = value.approx(p).to_string().parse().unwrap();
                let unit = if p >= 0 {
                    Rational::new(1_i64 << p)
                } else {
                    format!("1/{}", 1_u128 << -p).parse().unwrap()
                };
                let center = a * &unit;
                assert!(&center - &unit < upper && &center + &unit > lower);
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

#[test]
/// Reject residuals -1, -2 and -3/2 in three raw log forms, also beneath multiplication by zero.
/// Eighteen graphs give 36 native JSON/CBOR deserialization-error assertions.
/// For three outer scales, including zero, four checked-Real adapters must each
/// return Problem::ParseError (216 assertions); two collection routes must fail
/// through their deserializer errors (108 assertions). Thus neither a zero outer
/// scale nor a zero product may conceal an invalid logarithm. This exercises
/// import validation, not numerical evaluation or a promise of a more specific
/// domain Problem at this serialization boundary.
fn rational_log_import_keeps_invalid_residuals_as_typed_failures() {
    for (n, d) in [(-1, 1), (-2, 1), (-3, 2)] {
        for graph in [
            node("PrescaledLnRational", q(n, d)),
            node(
                "BinaryScaledLnRational",
                json!({"residual": q(n, d), "shift": 0}),
            ),
            node(
                "BinaryScaledLnRational",
                json!({"residual": q(n, d), "shift": 31}),
            ),
        ] {
            for graph in [
                graph.clone(),
                node("Multiply", json!([node("Ratio", q(0, 1)), graph])),
            ] {
                assert!(serde_json::from_value::<Computable>(graph.clone()).is_err());
                assert!(
                    ciborium::de::from_reader::<Computable, _>(bytes(&graph).as_slice()).is_err()
                );
                for scale in [0, 1, -3] {
                    let wire =
                        json!({"rational":q(scale, 1),"class":"Irrational","computable":graph});
                    let cbor = ciborium::Value::serialized(&wire).unwrap();
                    assert_eq!(Real::from_json(&wire.to_string()), Err(Problem::ParseError));
                    assert_eq!(Real::from_bytes(&bytes(&wire)), Err(Problem::ParseError));
                    assert_eq!(Real::try_from(&cbor), Err(Problem::ParseError));
                    assert_eq!(Real::try_from(cbor), Err(Problem::ParseError));
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
}
