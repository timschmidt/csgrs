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

fn cbor(value: &impl serde::Serialize) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

// Each recipe has independently derived value n/d. Exercise every arithmetic
// tag without relying on the implementation's rational normal-form result.
fn spellings(n: i64, d: u64) -> Vec<Value> {
    vec![
        node("Add", json!([leaf(n, d), leaf(0, 1)])),
        node("Multiply", json!([leaf(n, d), leaf(1, 1)])),
        node("Negate", leaf(-n, d)),
        node("Offset", json!([leaf(n, 2 * d), 1])),
        node(
            "Add",
            json!([node("Square", leaf(2, 1)), leaf(n - 4 * d as i64, d)]),
        ),
        node(
            "Add",
            json!([node("Inverse", leaf(2, 1)), leaf(2 * n - d as i64, 2 * d)]),
        ),
        node(
            "LinearCombination3",
            json!({
                "coefficients": [leaf(n, d), leaf(123, 1), leaf(-123, 1)],
                "values": [q(1, 1), q(1, 1), q(1, 1)],
            }),
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

fn invalid_cases() -> Vec<Value> {
    let mut cases = Vec::new();
    for (kind, inputs) in [
        ("Inverse", vec![(0, 1)]),
        ("PrescaledCot", vec![(0, 1)]),
        ("Sqrt", vec![(-2, 1), (-1, 1_000_000_000_000)]),
        ("NthRoot", vec![(-2, 1), (-1, 1_000_000_000_000)]),
        ("PrescaledLn", vec![(-2, 1), (-1, 1)]),
        ("PrescaledAsin", vec![(-1001, 1000), (1001, 1000)]),
        ("AsinDeferred", vec![(-1001, 1000), (1001, 1000)]),
        ("AcosPositive", vec![(-1, 1000), (1001, 1000)]),
        ("PrescaledAtanh", vec![(-2, 1), (-1, 1), (1, 1), (2, 1)]),
        ("AtanhDirect", vec![(-2, 1), (-1, 1), (1, 1), (2, 1)]),
        ("AcoshNearOne", vec![(999, 1000)]),
        ("AcoshDirect", vec![(999, 1000)]),
        ("NormalQuantile", vec![(-1, 1), (0, 1), (1, 1), (2, 1)]),
        ("SincSmall", vec![(-751, 1000), (751, 1000)]),
        ("CoscSmall", vec![(-751, 1000), (751, 1000)]),
    ] {
        for (n, d) in inputs {
            for argument in spellings(n, d) {
                cases.push(function(kind, argument));
            }
        }
    }
    cases
}

#[test]
fn arithmetic_domain_and_kernel_range_violations_fail_native_import() {
    for graph in invalid_cases() {
        let json = serde_json::from_value::<Computable>(graph.clone()).unwrap_err();
        let binary =
            ciborium::de::from_reader::<Computable, _>(cbor(&graph).as_slice()).unwrap_err();
        for message in [json.to_string(), binary.to_string()] {
            assert!(
                message.contains("domain") || message.contains("kernel range"),
                "{message}"
            );
        }
    }
}

#[test]
fn arithmetic_domain_errors_survive_identities_wrappers_and_collections() {
    for invalid in invalid_cases() {
        for graph in [
            invalid.clone(),
            node("Multiply", json!([leaf(0, 1), invalid])),
            node("Multiply", json!([invalid, leaf(0, 1)])),
            node("Add", json!([invalid, node("Negate", invalid.clone())])),
            node(
                "LinearCombination3",
                json!({"coefficients":[leaf(1,1),invalid,leaf(2,1)],"values":[q(1,1),q(0,1),q(1,1)]}),
            ),
        ] {
            for scale in [0, 1, -3] {
                let wire = json!({"rational":q(scale,1),"class":"Irrational","computable":graph});
                let value = ciborium::Value::serialized(&wire).unwrap();
                assert_eq!(Real::from_json(&wire.to_string()), Err(Problem::ParseError));
                assert_eq!(Real::from_bytes(&cbor(&wire)), Err(Problem::ParseError));
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

#[test]
fn valid_arithmetic_arguments_keep_their_graph_and_compose_after_import() {
    // These closed rational bounds follow elementary function inequalities,
    // not agreement between wrappers. For the quantiles, Phi(-1)<1/4 and
    // Phi(1)>3/4 give -1<quantile(p)<1 for these three probabilities.
    for (kind, inputs, lower, upper) in [
        ("Inverse", vec![(-2, 1), (2, 1)], -1, 1),
        ("PrescaledCot", vec![(-1, 2), (1, 2)], -2, 2),
        ("Sqrt", vec![(0, 1), (1, 4), (4, 1)], 0, 2),
        ("NthRoot", vec![(0, 1), (1, 4), (4, 1)], 0, 2),
        ("PrescaledLn", vec![(-1, 2), (0, 1), (1, 2)], -1, 1),
        ("PrescaledAsin", vec![(-1, 2), (0, 1), (1, 2)], -1, 1),
        ("AsinDeferred", vec![(-1, 1), (0, 1), (1, 1)], -2, 2),
        ("AcosPositive", vec![(0, 1), (1, 2), (1, 1)], 0, 2),
        ("PrescaledAtanh", vec![(-1, 2), (0, 1), (1, 2)], -1, 1),
        ("AtanhDirect", vec![(-1, 2), (0, 1), (1, 2)], -1, 1),
        ("AcoshNearOne", vec![(1, 1), (3, 2), (2, 1)], 0, 2),
        ("AcoshDirect", vec![(1, 1), (3, 2), (2, 1)], 0, 2),
        ("NormalQuantile", vec![(1, 4), (1, 2), (3, 4)], -1, 1),
        ("SincSmall", vec![(-3, 4), (0, 1), (3, 4)], 0, 1),
        ("CoscSmall", vec![(-3, 4), (0, 1), (3, 4)], 0, 1),
    ] {
        for (n, d) in inputs {
            for argument in spellings(n, d) {
                let graph = function(kind, argument);
                let wire = json!({"rational":q(1,1),"class":"Irrational","computable":graph});
                let value = Real::from_json(&wire.to_string()).unwrap();
                assert_eq!(serde_json::to_value(&value).unwrap(), wire);
                let bytes = value.to_bytes();
                let restored = Real::from_bytes(&bytes).unwrap();
                assert_eq!(restored.to_bytes(), bytes);
                let cold = restored.structural_facts().sign;
                // Allow the certified interval's dyadic enclosure at an exact
                // endpoint; this is not a predicate tolerance or input change.
                let unit = Rational::fraction(1, 1_u64 << 32).unwrap();
                for precision in [-8, -32, -8] {
                    let [lo, hi] = restored.certified_rational_interval(precision).unwrap();
                    if precision == -32 {
                        assert!(
                            lo >= Rational::new(lower) - &unit
                                && hi <= Rational::new(upper) + &unit,
                            "{kind}({n}/{d}): [{lo},{hi}]"
                        );
                    }
                    if let Some(sign) = cold {
                        assert_eq!(restored.structural_facts().sign, Some(sign));
                    }
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
fn large_leaves_keep_existing_checks_and_optional_proof_limits_do_not_reject_valid_graphs() {
    // Large direct leaves remain checked even beyond the rational proof's
    // 4096-bit coefficient limit. The same positive leaf remains importable.
    let huge = num::BigInt::from(1) << 5000_u32;
    let positive = node("Int", serde_json::to_value(&huge).unwrap());
    let negative = node("Negate", positive.clone());
    let negative_leaf = node("Int", serde_json::to_value(-huge).unwrap());
    assert!(serde_json::from_value::<Computable>(node("Sqrt", negative_leaf)).is_err());
    for graph in [
        node("Sqrt", positive.clone()),
        node("Sqrt", node("Square", negative)),
        node("Inverse", node("Add", json!([positive, leaf(1, 1)]))),
        node("Inverse", node("Offset", json!([leaf(1, 1), 5000]))),
    ] {
        let value: Computable = serde_json::from_value(graph.clone()).unwrap();
        assert_eq!(serde_json::to_value(value).unwrap(), graph);
    }
    // Ciborium Value avoids the JSON text parser's separate recursion limit.
    let mut deep = leaf(1, 1);
    for _ in 0..70 {
        deep = node("Negate", deep);
    }
    let graph = node("Inverse", deep);
    let value: Computable = ciborium::Value::serialized(&graph)
        .unwrap()
        .deserialized()
        .unwrap();
    assert_eq!(serde_json::to_value(value).unwrap(), graph);
    // A declined large/deep attempt must not poison a subsequent small proof.
    let invalid = node("Inverse", node("Add", json!([leaf(1, 1), leaf(-1, 1)])));
    assert!(serde_json::from_value::<Computable>(invalid).is_err());
}
