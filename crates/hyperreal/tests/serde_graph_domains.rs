#![cfg(feature = "serde")]

use hyperreal::{Computable, Problem, Rational, Real};
use serde_json::{Value, json};

fn rational(numerator: i64, denominator: u64) -> Value {
    serde_json::to_value(Rational::fraction(numerator, denominator).unwrap()).unwrap()
}

fn leaf(numerator: i64, denominator: u64) -> Value {
    json!({"internal": {"Ratio": rational(numerator, denominator)}})
}

fn node(kind: &str, argument: Value) -> Value {
    json!({"internal": {kind: argument}})
}

fn cbor(value: &impl serde::Serialize) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

fn invalid_nodes() -> Vec<Value> {
    let mut cases = vec![
        node("Inverse", leaf(0, 1)),
        node("PrescaledCot", leaf(0, 1)),
        node("Sqrt", leaf(-1, 1)),
        node("NthRoot", json!([leaf(-1, 1), 3])),
        node("IntegralAtan", json!([0, []])),
    ];
    // The logarithm variants store x for ln(1+x), whose domain is x > -1.
    for n in [-2, -1] {
        cases.push(node("PrescaledLn", leaf(n, 1)));
        cases.push(node("PrescaledLnRational", rational(n, 1)));
        cases.push(node(
            "BinaryScaledLnRational",
            json!({"residual": rational(n, 1), "shift": 7}),
        ));
    }
    for n in [-2, 2] {
        cases.push(node("AsinRational", rational(n, 1)));
        for kind in ["PrescaledAsin", "AsinDeferred"] {
            cases.push(node(kind, leaf(n, 1)));
        }
    }
    // The negative-endpoint acos variant stores |x|, not the signed x.
    for n in [-1, 2] {
        for kind in ["AcosPositiveRational", "AcosNegativeRational"] {
            cases.push(node(kind, rational(n, 1)));
        }
        cases.push(node("AcosPositive", leaf(n, 1)));
    }
    for n in [-2, -1, 1, 2] {
        cases.push(node("AtanhRational", rational(n, 1)));
        for kind in ["PrescaledAtanh", "AtanhDirect"] {
            cases.push(node(kind, leaf(n, 1)));
        }
    }
    for kind in ["AcoshNearOne", "AcoshDirect"] {
        cases.push(node(kind, leaf(1, 2)));
    }
    for n in [-1, 0, 1, 2] {
        cases.push(node(
            "NormalQuantile",
            json!({"p": leaf(n, 1), "seed": [0, []], "seed_prec": -16}),
        ));
    }
    // Int and One leaves share the same mathematical domain rules as Ratio.
    cases.push(node("Inverse", json!({"internal": {"Int": [0, []]}})));
    cases.push(node("Sqrt", json!({"internal": {"Int": [-1, [1]]}})));
    cases.push(node("AtanhDirect", json!({"internal": "One"})));
    cases
}

#[test]
fn native_graph_import_rejects_exact_out_of_domain_arguments() {
    let mut accepted = Vec::new();
    for wire in invalid_nodes() {
        let json = serde_json::from_value::<Computable>(wire.clone());
        let binary = ciborium::de::from_reader::<Computable, _>(cbor(&wire).as_slice());
        match (json, binary) {
            (Err(json), Err(binary)) => {
                assert!(json.to_string().contains("domain"));
                assert!(binary.to_string().contains("domain"));
            }
            _ => accepted.push(wire),
        }
    }
    assert!(
        accepted.is_empty(),
        "accepted undefined nodes: {accepted:?}"
    );
}

#[test]
fn undefined_children_cannot_be_hidden_by_outer_identities_or_collections() {
    let mut accepted = Vec::new();
    for invalid in invalid_nodes() {
        for graph in [
            invalid.clone(),
            node("Multiply", json!([leaf(0, 1), invalid])),
            node("Add", json!([invalid, leaf(1, 1)])),
            node("Square", invalid.clone()),
        ] {
            for scale in [0, 1, -3] {
                let wire = json!({
                    "rational": rational(scale, 1),
                    "class": "Irrational",
                    "computable": graph,
                });
                let bytes = cbor(&wire);
                if Real::from_json(&wire.to_string()) != Err(Problem::ParseError)
                    || Real::from_bytes(&bytes) != Err(Problem::ParseError)
                    || Real::try_from(ciborium::Value::serialized(&wire).unwrap())
                        != Err(Problem::ParseError)
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
    assert!(
        accepted.is_empty(),
        "undefined pipeline inputs: {accepted:?}"
    );
}

#[test]
fn valid_partial_functions_preserve_values_and_wire_representation() {
    let mut values = vec![
        (Real::zero().sqrt().unwrap(), 0, 0),
        (Real::from(2).sqrt().unwrap(), 1, 2),
        (Real::from(2).ln().unwrap(), 0, 1),
        (Real::from(2).acosh().unwrap(), 1, 2),
        (
            Real::from(Rational::fraction(1, 2).unwrap())
                .atanh()
                .unwrap(),
            0,
            1,
        ),
        (Real::from(2).inverse().unwrap(), 0, 1),
    ];
    for n in [-1, 0, 1] {
        values.push((Real::from(n).asin().unwrap(), -2, 2));
        values.push((Real::from(n).acos().unwrap(), 0, 4));
    }
    // Keep the full negative and large log1p residuals from the native guard
    // regression; their stored arguments must not be mistaken for ln inputs.
    for (numerator, denominator, lower, upper) in
        [(-255, 256, -6, -5), (-1, 2, -1, 0), (1000, 1, 6, 7)]
    {
        values.push((
            Real::from(Rational::fraction(numerator, denominator).unwrap())
                .ln_1p()
                .unwrap(),
            lower,
            upper,
        ));
    }
    // Defined opaque operands must not be rejected for lacking a leaf proof.
    // 25/8 < pi < 22/7 implies 7 < 1/(pi - 3) < 8 and 1 < sqrt(pi) < 2.
    values.push(((Real::pi() - Real::from(3)).inverse().unwrap(), 7, 8));
    values.push((Real::pi().sqrt().unwrap(), 1, 2));
    for (value, lower, upper) in values {
        let json = value.to_json();
        let bytes = value.to_bytes();
        for restored in [
            Real::from_json(&json).unwrap(),
            Real::from_bytes(&bytes).unwrap(),
        ] {
            let [lo, hi] = restored.certified_rational_interval(-32).unwrap();
            assert!(lo >= Rational::new(lower) && hi <= Rational::new(upper));
            assert_eq!(restored.to_json(), json);
            assert_eq!(restored.to_bytes(), bytes);
            let [lo, hi] = (restored + Real::one())
                .certified_rational_interval(-32)
                .unwrap();
            assert!(lo >= Rational::new(lower + 1) && hi <= Rational::new(upper + 1));
        }
    }
}

#[test]
fn logarithm_residuals_preserve_zero_and_both_sides_of_one() {
    // For |x| <= 1/4, |ln(1+x)| < 1/2. At x=0 the result is exactly zero.
    // Exercise every stored logarithm residual representation directly.
    for numerator in [-1, 0, 1] {
        for graph in [
            node("PrescaledLn", leaf(numerator, 4)),
            node("PrescaledLnRational", rational(numerator, 4)),
            node(
                "BinaryScaledLnRational",
                json!({"residual": rational(numerator, 4), "shift": 0}),
            ),
        ] {
            let wire = json!({
                "rational": rational(1, 1),
                "class": "Irrational",
                "computable": graph,
            });
            let bytes = cbor(&wire);
            for value in [
                Real::from_json(&wire.to_string()).unwrap(),
                Real::from_bytes(&bytes).unwrap(),
            ] {
                let [lo, hi] = value.certified_rational_interval(-32).unwrap();
                let half = Rational::fraction(1, 2).unwrap();
                assert!(lo > -half.clone() && hi < half);
                match numerator {
                    -1 => assert!(hi < Rational::zero()),
                    1 => assert!(lo > Rational::zero()),
                    0 => {
                        let unit = Rational::fraction(1, 1_u64 << 32).unwrap();
                        assert!(lo <= Rational::zero() && hi >= Rational::zero());
                        assert!(lo >= -unit.clone() && hi <= unit);
                    }
                    _ => unreachable!(),
                }
                assert_eq!(serde_json::to_value(&value).unwrap(), wire);
            }
        }
    }
}
