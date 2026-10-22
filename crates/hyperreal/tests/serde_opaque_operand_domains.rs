#![cfg(feature = "serde")]

//! Real import proves the real domain of every partial operation whose
//! operand is not an exact rational, operands first. Computable decoding keeps
//! its rejection-only checks, matching its unchecked native constructors.

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

fn negate(a: Value) -> Value {
    node("Negate", a)
}

fn pi() -> Value {
    node("Constant", json!("Pi"))
}

fn sqrt2() -> Value {
    node("Sqrt", leaf(2, 1))
}

/// sqrt(2) * sqrt(2) - 2 is exactly zero. Rational admission does not see it.
fn hidden_zero() -> Value {
    add(multiply(sqrt2(), sqrt2()), leaf(-2, 1))
}

fn function(kind: &str, argument: Value) -> Value {
    match kind {
        "NthRoot" => node(kind, json!([argument, 3])),
        "NormalQuantile" => node(kind, json!({"p":argument,"seed":[0,[]],"seed_prec":-16})),
        _ => node(kind, argument),
    }
}

fn cbor(value: &impl serde::Serialize) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

fn wire(scale: i64, graph: &Value) -> Value {
    json!({"rational": q(scale, 1), "class": "Irrational", "computable": graph})
}

fn import_outcomes(wire: &Value) -> [Result<Real, Problem>; 3] {
    let value = ciborium::Value::serialized(wire).unwrap();
    [
        Real::from_json(&wire.to_string()),
        Real::from_bytes(&cbor(wire)),
        Real::try_from(&value),
    ]
}

/// Operations whose real domain excludes an opaque operand. Each operand is
/// built from defined nodes, so only the outer operation is undefined.
fn undefined_operations() -> Vec<(&'static str, Value)> {
    let below_minus_one = negate(pi());
    let above_one = pi();
    let negative = add(sqrt2(), leaf(-2, 1));
    let below_one = add(sqrt2(), leaf(-1, 1));
    vec![
        ("Sqrt", below_minus_one.clone()),
        ("Sqrt", negative.clone()),
        ("NthRoot", negative),
        ("Inverse", hidden_zero()),
        ("PrescaledCot", hidden_zero()),
        ("PrescaledLn", below_minus_one.clone()),
        ("PrescaledAsin", above_one.clone()),
        ("AsinDeferred", below_minus_one.clone()),
        ("AcosPositive", above_one.clone()),
        ("AcosPositive", negate(node("Inverse", pi()))),
        ("AcoshNearOne", below_one.clone()),
        ("AcoshDirect", below_one),
        ("AtanhDirect", above_one.clone()),
        ("PrescaledAtanh", below_minus_one),
        ("NormalQuantile", above_one),
        ("SincSmall", sqrt2()),
        ("CoscSmall", negate(sqrt2())),
    ]
}

#[test]
fn undefined_opaque_operands_fail_before_wrappers_can_publish_values() {
    let mut accepted = Vec::new();
    for (kind, operand) in undefined_operations() {
        let invalid = function(kind, operand);
        // Native Computable constructors do not check domains either, and
        // rational admission cannot refute these operands.
        assert!(
            serde_json::from_value::<Computable>(invalid.clone()).is_ok(),
            "{kind} is not an opaque-operand case"
        );
        for graph in [
            invalid.clone(),
            multiply(leaf(0, 1), invalid.clone()),
            multiply(invalid.clone(), leaf(0, 1)),
            add(invalid.clone(), negate(invalid.clone())),
            add(invalid.clone(), leaf(1, 1)),
            node("Square", invalid.clone()),
        ] {
            for scale in [0, 1, -3] {
                let wire = wire(scale, &graph);
                if import_outcomes(&wire)
                    .iter()
                    .any(|outcome| outcome != &Err(Problem::ParseError))
                {
                    accepted.push(wire.clone());
                }
                let collection = json!([Real::one(), wire]);
                if serde_json::from_value::<Vec<Real>>(collection.clone()).is_ok()
                    || ciborium::de::from_reader::<Vec<Real>, _>(cbor(&collection).as_slice())
                        .is_ok()
                {
                    accepted.push(collection);
                }
            }
        }
    }
    assert!(accepted.is_empty(), "undefined imports: {accepted:#?}");
}

#[test]
fn poles_of_stored_tangent_kernels_are_rejected() {
    // tan(pi/2) and cot(pi) have no real value. Both operands are total.
    for graph in [
        node("PrescaledTan", node("Offset", json!([pi(), -1]))),
        node("PrescaledCot", pi()),
    ] {
        assert!(serde_json::from_value::<Computable>(graph.clone()).is_ok());
        for outcome in import_outcomes(&wire(1, &graph)) {
            assert_eq!(outcome.map(|_| ()), Err(Problem::ParseError));
        }
    }
}

#[test]
fn undecided_operand_domains_are_unknown_certificates_not_values() {
    // sin(pi) is zero, but bounded refinement and the algebraic separation
    // bound cannot prove a transcendental zero. Its reciprocal is undefined;
    // an undecided domain is neither a value nor a proof of invalidity.
    let graph = node("Inverse", node("PrescaledSin", pi()));
    for outcome in import_outcomes(&wire(1, &graph)) {
        assert_eq!(outcome.map(|_| ()), Err(Problem::UnknownCertificate));
    }
}

#[test]
fn proved_boundary_operands_are_admitted() {
    // sqrt(0) is defined. The hidden zero is proved exactly, not refined to a
    // tolerance, so the import keeps the supplied graph and its exact value.
    let root_of_zero = wire(1, &node("Sqrt", hidden_zero()));
    for outcome in import_outcomes(&root_of_zero) {
        let value = outcome.unwrap();
        assert_eq!(serde_json::to_value(&value).unwrap(), root_of_zero);
        assert!(value.certified_sign_until(-64).sign() == Some(hyperreal::RealSign::Zero));
    }
    // Real::abs stores sqrt(x^2) when the sign of x is unresolved. That is
    // defined for every defined x, including the undecidable zero sin(pi).
    let absolute = wire(1, &node("Sqrt", node("Square", node("PrescaledSin", pi()))));
    for outcome in import_outcomes(&absolute) {
        assert_eq!(serde_json::to_value(outcome.unwrap()).unwrap(), absolute);
    }
    let native = (Real::pi() - Real::new(Rational::fraction(355, 113).unwrap())).abs();
    assert_eq!(
        Real::from_json(&native.to_json()).unwrap().to_json(),
        native.to_json()
    );
}

#[test]
fn defined_native_values_with_opaque_operands_round_trip_unchanged() {
    let pi = Real::pi;
    let third = Rational::fraction(1, 3).unwrap();
    let values = [
        (pi() - Real::from(3)).inverse().unwrap(),
        pi().sqrt().unwrap(),
        (pi() - Real::from(3)).sqrt().unwrap(),
        pi().root_n(3).unwrap(),
        pi().inverse().unwrap().asin().unwrap(),
        (-pi().inverse().unwrap()).asin().unwrap(),
        pi().inverse().unwrap().acos().unwrap(),
        pi().acosh().unwrap(),
        pi().inverse().unwrap().atanh().unwrap(),
        pi().ln().unwrap(),
        (pi() - Real::from(3)).ln_1p().unwrap(),
        Real::from(2).sqrt().unwrap().tan().unwrap(),
        Real::e().tan().unwrap(),
        (pi() / Real::from(7)).unwrap().cot().unwrap(),
        pi().inverse().unwrap().qnorm().unwrap(),
        (pi() * Real::from(third)).sinc().unwrap(),
        (pi().inverse().unwrap()).cosc().unwrap(),
    ];
    for value in values {
        let json = value.to_json();
        let bytes = value.to_bytes();
        let [lo, hi] = value.certified_rational_interval(-64).unwrap();
        for restored in [
            Real::from_json(&json).unwrap(),
            Real::from_bytes(&bytes).unwrap(),
        ] {
            assert_eq!(restored.to_json(), json);
            assert_eq!(restored.to_bytes(), bytes);
            let [restored_lo, restored_hi] = restored.certified_rational_interval(-64).unwrap();
            assert!(restored_lo <= hi && lo <= restored_hi, "{json}");
        }
    }
}
