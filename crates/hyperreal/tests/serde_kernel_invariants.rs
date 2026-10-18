#![cfg(feature = "serde")]

use hyperreal::{Computable, Problem, Rational, Real, RealSign, ZeroKnowledge};
use serde_json::{Value, json};

fn q(n: i64, d: u64) -> Value {
    serde_json::to_value(Rational::fraction(n, d).unwrap()).unwrap()
}

fn leaf(n: i64, d: u64) -> Value {
    json!({"internal": {"Ratio": q(n, d)}})
}

fn node(kind: &str, argument: Value) -> Value {
    json!({"internal": {kind: argument}})
}

fn cbor(value: &Value) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

#[test]
fn imported_private_trig_kernels_require_their_stored_parameter_ranges() {
    let mut invalid = Vec::new();
    // These dedicated rational kernels store |r| < 1; public constructors
    // route larger values through residual or general argument reduction.
    for kind in [
        "PrescaledCosRational",
        "PrescaledSinRational",
        "PrescaledTanRational",
    ] {
        for n in [-4, -2, -1, 1, 2, 4] {
            invalid.push(node(kind, q(n, 1)));
        }
    }
    // Complement kernels store a positive magnitude 1 <= r < 3/2.
    for kind in [
        "PrescaledCosHalfPiMinusRational",
        "PrescaledSinHalfPiMinusRational",
        "PrescaledCotHalfPiMinusRational",
    ] {
        for (n, d) in [(-4, 1), (-1, 1), (0, 1), (1, 2), (3, 2), (2, 1), (4, 1)] {
            invalid.push(node(kind, q(n, d)));
        }
    }
    // Small-angle continuations carry the separate |x| <= 3/4 contract.
    for kind in ["SincSmall", "CoscSmall"] {
        for n in [-4, -1, 1, 4] {
            invalid.push(node(kind, leaf(n, 1)));
        }
        for constant in ["Pi", "Tau", "Sqrt2"] {
            invalid.push(node(kind, json!({"internal": {"Constant": constant}})));
        }
    }
    let mut accepted = Vec::new();
    for graph in invalid {
        let direct_json = serde_json::from_value::<Computable>(graph.clone());
        let direct_cbor = ciborium::de::from_reader::<Computable, _>(cbor(&graph).as_slice());
        if direct_json.is_ok() || direct_cbor.is_ok() {
            accepted.push(graph.clone());
        }
        for graph in [graph.clone(), node("Multiply", json!([leaf(0, 1), graph]))] {
            for scale in [0, 1, -3] {
                let wire = json!({"rational":q(scale, 1),"class":"Irrational","computable":graph});
                let a = Real::from_json(&wire.to_string());
                let b = Real::from_bytes(&cbor(&wire));
                assert_eq!(a.is_err(), b.is_err());
                if a != Err(Problem::ParseError) || b != Err(Problem::ParseError) {
                    accepted.push(wire.clone());
                }
                let container = json!([Real::one(), wire]);
                let a = serde_json::from_value::<Vec<Real>>(container.clone());
                let b = ciborium::de::from_reader::<Vec<Real>, _>(cbor(&container).as_slice());
                if a.is_ok() || b.is_ok() {
                    accepted.push(container);
                }
            }
        }
    }
    assert!(
        accepted.is_empty(),
        "{} unchecked private kernel inputs; first: {:?}",
        accepted.len(),
        accepted.first()
    );
}

#[test]
fn inverse_cosine_endpoints_preserve_zero_facts_and_downstream_failure() {
    let one = leaf(1, 1);
    let sum_one = node("Add", json!([leaf(2, 1), leaf(-1, 1)]));
    let inverse_one = node("Inverse", one.clone());
    let mut cases = vec![node("AcosPositiveRational", q(1, 1))];
    for kind in ["AcoshNearOne", "AcoshDirect"] {
        for argument in [one.clone(), sum_one.clone(), inverse_one.clone()] {
            cases.push(node(kind, argument));
        }
    }
    for graph in cases {
        // acos(1)=acosh(1)=0, independent of the graph's spelling or cache.
        let native: Computable = serde_json::from_value(graph.clone()).unwrap();
        assert_eq!(
            native.structural_facts().sign,
            Some(RealSign::Zero),
            "{graph}"
        );
        assert_eq!(native.structural_facts().zero, ZeroKnowledge::Zero);
        let wire = json!({"rational":q(1, 1),"class":"Irrational","computable":graph});
        for restored in [
            Real::from_json(&wire.to_string()).unwrap(),
            Real::from_bytes(&cbor(&wire)).unwrap(),
        ] {
            let json = restored.to_json();
            let bytes = restored.to_bytes();
            for precision in [0, -64, 0] {
                assert_eq!(restored.refine_sign_until(precision), Some(RealSign::Zero));
            }
            assert_eq!(restored.to_json(), json);
            assert_eq!(restored.to_bytes(), bytes);
            assert_eq!(restored.inverse_ref(), Err(Problem::DivideByZero));
            assert_eq!(
                (Real::one() + &restored).exact_rational_normal_form(),
                Some(Rational::one())
            );
            assert_eq!(
                (restored + Real::one()).exact_rational_normal_form(),
                Some(Rational::one())
            );
        }
    }
}

#[test]
fn endpoint_proofs_do_not_erase_unresolved_domain_obligations() {
    let denominator = node("Add", json!([leaf(1, 1), leaf(-1, 1)]));
    let invalid = node("Inverse", denominator);
    let hidden = node("Multiply", json!([leaf(0, 1), invalid]));
    let value: Computable = serde_json::from_value(hidden.clone()).unwrap();
    // General graph admission still accepts this undefined expression. Its
    // existing sign fact is not a domain proof and must not erase the inverse.
    let _ = value.structural_facts();
    let sum = value.add(Computable::one());
    let wire = serde_json::to_value(&sum).unwrap();
    assert_eq!(wire["internal"]["Add"][0], hidden);

    let argument = node("Add", json!([hidden, leaf(1, 1)]));
    for kind in ["AcoshNearOne", "AcoshDirect"] {
        let graph = node(kind, argument.clone());
        let native: Computable = serde_json::from_value(graph.clone()).unwrap();
        assert_eq!(native.structural_facts().sign, None);
        let sum = native.add(Computable::one());
        let wire = serde_json::to_value(sum).unwrap();
        assert_eq!(wire["internal"]["Add"][0], graph);
    }
}

#[test]
fn valid_private_parameters_and_public_trig_outputs_remain_consumable() {
    let mut graphs = Vec::new();
    for kind in [
        "PrescaledCosRational",
        "PrescaledSinRational",
        "PrescaledTanRational",
    ] {
        for n in [-3, -2, 0, 2, 3] {
            graphs.push(node(kind, q(n, 4)));
        }
    }
    for kind in [
        "PrescaledCosHalfPiMinusRational",
        "PrescaledSinHalfPiMinusRational",
        "PrescaledCotHalfPiMinusRational",
    ] {
        for (n, d) in [(1, 1), (7, 5), (149, 100)] {
            graphs.push(node(kind, q(n, d)));
        }
    }
    for kind in ["SincSmall", "CoscSmall"] {
        for n in [-3, -1, 0, 1, 3] {
            graphs.push(node(kind, leaf(n, 4)));
        }
    }
    for graph in graphs {
        let wire = json!({"rational":q(1, 1),"class":"Irrational","computable":graph});
        let source = Real::from_json(&wire.to_string()).unwrap();
        let bytes = source.to_bytes();
        let restored = Real::from_bytes(&bytes).unwrap();
        assert_eq!(restored.to_bytes(), bytes);
        let [lo, hi] = restored.certified_rational_interval(-32).unwrap();
        // |sin(x)|, |cos(x)| <= 1; |tan(x)| < 1 for |x| <= 3/4.
        // For the medium cotangent, pi > 25/8 and tan(t)>t give cot(t)<16.
        // The small-angle integral forms bound sinc by 1 and cosc by 1/2.
        assert!(lo > Rational::new(-2) && hi < Rational::new(16));
        assert!(
            (restored + Real::one())
                .certified_rational_interval(-32)
                .is_some()
        );
    }
    for n in [
        -200, -150, -140, -100, -75, -1, 0, 1, 75, 100, 140, 150, 200,
    ] {
        let input = Real::from(Rational::fraction(n, 100).unwrap());
        for value in [
            input.clone().sin(),
            input.clone().cos(),
            input.tan().unwrap(),
        ] {
            let json = value.to_json();
            let bytes = value.to_bytes();
            for restored in [
                Real::from_json(&json).unwrap(),
                Real::from_bytes(&bytes).unwrap(),
            ] {
                assert_eq!(restored.to_json(), json);
                assert_eq!(restored.to_bytes(), bytes);
                assert!(
                    (restored + Real::one())
                        .certified_rational_interval(-32)
                        .is_some()
                );
            }
        }
    }
}
