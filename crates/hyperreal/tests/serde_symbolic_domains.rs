#![cfg(feature = "serde")]

use hyperreal::{Computable, Problem, Rational, Real};
use serde_json::{Value, json};

fn q(n: i64, d: u64) -> Value {
    serde_json::to_value(Rational::fraction(n, d).unwrap()).unwrap()
}

fn node(kind: &str, argument: Value) -> Value {
    json!({"internal": {kind: argument}})
}

fn leaf(n: i64, d: u64) -> Value {
    node("Ratio", q(n, d))
}

fn add(a: Value, b: Value) -> Value {
    node("Add", json!([a, b]))
}

fn multiply(a: Value, b: Value) -> Value {
    node("Multiply", json!([a, b]))
}

fn cbor(value: &impl serde::Serialize) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

// Each form is n/d whenever x denotes a real. This is a conditional ring
// identity, not a proof that x or the resulting expression is defined.
fn spellings(x: &Value, n: i64, d: u64) -> Vec<Value> {
    let cancelled = add(x.clone(), node("Negate", x.clone()));
    let square = node("Square", x.clone());
    let expanded = multiply(add(x.clone(), leaf(1, 1)), add(x.clone(), leaf(-1, 1)));
    vec![
        add(cancelled.clone(), leaf(n, d)),
        add(multiply(leaf(0, 1), x.clone()), leaf(n, d)),
        add(node("Offset", json!([cancelled.clone(), 2])), leaf(n, d)),
        add(
            add(
                square.clone(),
                node("Negate", multiply(x.clone(), x.clone())),
            ),
            leaf(n, d),
        ),
        add(add(expanded, node("Negate", square)), leaf(n + d as i64, d)),
        node(
            "LinearCombination3",
            json!({
                "coefficients": [x, x, leaf(n, d)], "values": [q(2, 1), q(-2, 1), q(1, 1)],
            }),
        ),
        add(
            node("Inverse", add(cancelled, leaf(2, 1))),
            leaf(2 * n - d as i64, 2 * d),
        ),
    ]
}

fn function(kind: &str, argument: Value) -> Value {
    match kind {
        "NthRoot" => node(kind, json!([argument, 3])),
        "NormalQuantile" => node(kind, json!({"p":argument,"seed":[0,[]],"seed_prec":-16})),
        _ => node(kind, argument),
    }
}

fn atoms() -> Vec<Value> {
    vec![
        node("PrescaledSin", leaf(1, 2)),
        node("Constant", json!("Pi")),
        node("Sqrt", leaf(2, 1)),
        // Unknown to rational admission, although sqrt(-atan(1)) is undefined.
        node("Sqrt", node("Negate", node("AtanRational", q(1, 1)))),
    ]
}

#[test]
fn symbolic_domain_contradictions_fail_before_wrappers_can_publish_values() {
    for x in atoms() {
        for (kind, n, d) in [
            ("Inverse", 0, 1),
            ("PrescaledCot", 0, 1),
            ("Sqrt", -1, 1_000_000_000_000),
            ("NthRoot", -1, 1_000_000_000_000),
            ("PrescaledLn", -1, 1),
            ("PrescaledAsin", 1001, 1000),
            ("AsinDeferred", -1001, 1000),
            ("AcosPositive", -1, 1000),
            ("PrescaledAtanh", -1, 1),
            ("AtanhDirect", 1, 1),
            ("AcoshNearOne", 999, 1000),
            ("AcoshDirect", 999, 1000),
            ("NormalQuantile", 1, 1),
            ("SincSmall", 751, 1000),
            ("CoscSmall", -751, 1000),
        ] {
            for argument in spellings(&x, n, d) {
                let invalid = function(kind, argument);
                for graph in [
                    invalid.clone(),
                    multiply(leaf(0, 1), invalid.clone()),
                    multiply(invalid.clone(), leaf(0, 1)),
                    add(invalid.clone(), node("Negate", invalid.clone())),
                    node(
                        "LinearCombination3",
                        json!({
                            "coefficients": [leaf(1,1),invalid,leaf(2,1)],
                            "values": [q(1,1),q(0,1),q(1,1)],
                        }),
                    ),
                ] {
                    let error = serde_json::from_value::<Computable>(graph.clone()).unwrap_err();
                    let message = error.to_string();
                    assert!(
                        message.contains("domain") || message.contains("kernel range"),
                        "{message}"
                    );
                    assert!(
                        ciborium::de::from_reader::<Computable, _>(cbor(&graph).as_slice())
                            .is_err()
                    );
                    for scale in [0, 1, -3] {
                        let wire =
                            json!({"rational":q(scale,1),"class":"Irrational","computable":graph});
                        assert_eq!(Real::from_json(&wire.to_string()), Err(Problem::ParseError));
                        assert_eq!(Real::from_bytes(&cbor(&wire)), Err(Problem::ParseError));
                        let value = ciborium::Value::serialized(&wire).unwrap();
                        assert_eq!(Real::try_from(&value), Err(Problem::ParseError));
                        assert_eq!(Real::try_from(value), Err(Problem::ParseError));
                        let collection = json!([Real::one(), wire]);
                        assert!(serde_json::from_value::<Vec<Real>>(collection.clone()).is_err());
                        assert!(
                            ciborium::de::from_reader::<Vec<Real>, _>(cbor(&collection).as_slice())
                                .is_err()
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn valid_symbolic_cancellation_keeps_graphs_and_composes_after_import() {
    // Use only defined x here. The function bounds are independent elementary
    // inequalities; agreement of JSON and CBOR alone is not the oracle.
    for x in atoms().into_iter().take(3) {
        for (kind, n, d, lower, upper) in [
            ("Inverse", 2, 1, 0, 1),
            ("PrescaledCot", 1, 2, 1, 2),
            ("Sqrt", 4, 1, 2, 2),
            ("NthRoot", 8, 1, 2, 2),
            ("PrescaledLn", 1, 2, 0, 1),
            ("PrescaledAsin", 1, 2, 0, 1),
            ("AsinDeferred", -1, 2, -1, 0),
            ("AcosPositive", 1, 2, 1, 2),
            ("PrescaledAtanh", -1, 2, -1, 0),
            ("AtanhDirect", 1, 2, 0, 1),
            ("AcoshNearOne", 2, 1, 1, 2),
            ("AcoshDirect", 2, 1, 1, 2),
            ("NormalQuantile", 1, 2, 0, 0),
            ("SincSmall", 1, 2, 0, 1),
            ("CoscSmall", -1, 2, 0, 1),
        ] {
            for argument in spellings(&x, n, d) {
                let graph = function(kind, argument);
                let wire = json!({"rational":q(1,1),"class":"Irrational","computable":graph});
                let source = Real::from_json(&wire.to_string()).unwrap();
                assert_eq!(serde_json::to_value(&source).unwrap(), wire);
                let bytes = source.to_bytes();
                let restored = Real::from_bytes(&bytes).unwrap();
                assert_eq!(restored.to_bytes(), bytes);
                let mut known = restored.structural_facts().sign;
                let unit = Rational::fraction(1, 1_u64 << 32).unwrap();
                for precision in [-8, -32, -8] {
                    let [lo, hi] = restored.certified_rational_interval(precision).unwrap();
                    if precision == -32 {
                        assert!(
                            lo >= Rational::new(lower) - &unit
                                && hi <= Rational::new(upper) + &unit,
                            "{kind}: [{lo},{hi}]"
                        );
                    }
                    let facts = restored.structural_facts().sign;
                    if known.is_some() {
                        assert_eq!(facts, known);
                    }
                    known = facts;
                }
                assert_eq!(serde_json::to_value(&restored).unwrap(), wire);
                let [lo, hi] = (restored + Real::one())
                    .certified_rational_interval(-32)
                    .unwrap();
                assert!(
                    lo >= Rational::new(lower + 1) - &unit
                        && hi <= Rational::new(upper + 1) + &unit
                );
            }
        }
    }
}

#[test]
fn distinct_symbolic_inputs_are_not_cancelled_and_declined_proofs_can_recover() {
    let sine = node("PrescaledSin", leaf(1, 2));
    let other = node("PrescaledSin", leaf(1, 4));
    // Sine is increasing on [1/4,1/2], so this reciprocal is defined and > 1.
    let graph = node("Inverse", add(sine.clone(), node("Negate", other)));
    let wire = json!({"rational":q(1,1),"class":"Irrational","computable":graph});
    let value = Real::from_json(&wire.to_string()).unwrap();
    assert_eq!(serde_json::to_value(&value).unwrap(), wire);
    assert!(value.certified_rational_interval(-32).unwrap()[0] > Rational::one());

    // Exceed only optional proof depth. Acceptance is not a validity proof.
    let mut deep = sine.clone();
    for _ in 0..70 {
        deep = node("Negate", deep);
    }
    let graph = node("Inverse", add(deep, leaf(1, 1)));
    let value: Computable = ciborium::Value::serialized(&graph)
        .unwrap()
        .deserialized()
        .unwrap();
    assert_eq!(serde_json::to_value(value).unwrap(), graph);
    let invalid = node("Inverse", add(sine.clone(), node("Negate", sine)));
    for _ in 0..3 {
        assert!(serde_json::from_value::<Computable>(invalid.clone()).is_err());
    }
}
