//! Native and checked-Real interchange witnesses for exact arctangent/pi anchors.
//!
//! JSON/CBOR, collections, warming and legacy graph replay exercise retained
//! trigonometric facts. Same-process replay, exact square identities and finite
//! certified enclosures are separate checks; none is a universal replay proof.

#![cfg(feature = "serde")]

use hyperreal::{Computable, Problem, Rational, Real, RealSign};
use num::{BigInt, BigUint, One};
use rug::{Float, Rational as Q, float::Round};
use serde_json::{Value, json};
use std::cmp::Ordering;

#[path = "support/mpfr.rs"]
/// Shared thread-exit cleanup support for the independent MPFR oracle; it does not supply Hyperreal results.
mod mpfr;

/// Encode a JSON fixture value as CBOR, panicking on an unexpected serialization failure.
fn bytes(value: &Value) -> Vec<u8> {
    let mut result = Vec::new();
    ciborium::ser::into_writer(value, &mut result).unwrap();
    result
}

/// Require a Computable square to equal its exact rational expectation by a zero-sign decision at precision zero.
/// The square alone does not establish the sign of the original value.
fn native_square(value: Computable, square: &Rational) {
    assert_eq!(
        value
            .square()
            .add(Computable::rational(-square.clone()))
            .sign_until(0),
        Some(RealSign::Zero)
    );
}

/// Require the checked Real square comparison to decide Equal at precision zero.
/// Read ordering from the returned comparison outcome; the assertion concerns squared
/// magnitude and does not by itself select the original signed branch.
fn real_square(value: Real, square: &Rational) {
    assert_eq!(
        (&value * &value)
            .certified_cmp_until(&Real::from(square.clone()), 0)
            .ordering(),
        Some(Ordering::Equal)
    );
}

/// Return the independent rational sin(pi/d)² expectation for d=3,4,6.
/// Other denominators violate this fixture helper's domain and panic.
fn sine_square(denominator: u64) -> Rational {
    Rational::fraction(
        match denominator {
            3 => 3,
            4 => 2,
            6 => 1,
            _ => unreachable!(),
        },
        4,
    )
    .unwrap()
}

#[test]
/// Exercise eighteen signed/period-shifted anchor angles through five native carriers and four scales through six Real import routes.
/// Before and after explicit warming, verify sine/cosine squares after further
/// JSON/CBOR replay. Checked Real paths also verify the unit-circle relation,
/// cotangent-square and cotangent*sine=cosine, or typed rejection of cotangent
/// at scale zero. Preserve original angle serialization independently of these
/// semantic assertions. This does not reset global caches or spawn a fresh process.
fn atan_pi_anchors_retain_trig_facts_through_interchange() {
    for d in [3, 4, 6] {
        for sign in [-1, 1] {
            for period in [-1, 0, 1] {
                let q = Rational::new(period) + Rational::fraction(sign, d).unwrap();
                let angle = Computable::pi().multiply(Computable::rational(q));
                let graph = serde_json::to_value(&angle).unwrap();
                let collection = json!([graph]);
                let mut json_values: Vec<Computable> =
                    serde_json::from_value(collection.clone()).unwrap();
                let mut cbor_values: Vec<Computable> =
                    ciborium::de::from_reader(bytes(&collection).as_slice()).unwrap();
                let expected = [sine_square(d), Rational::one() - sine_square(d)];
                for angle in [
                    angle,
                    serde_json::from_value(graph.clone()).unwrap(),
                    ciborium::de::from_reader(bytes(&graph).as_slice()).unwrap(),
                    json_values.pop().unwrap(),
                    cbor_values.pop().unwrap(),
                ] {
                    for warm in [false, true] {
                        if warm {
                            let _ = angle.approx(-128);
                        }
                        for (i, value) in [angle.clone().sin(), angle.clone().cos()]
                            .into_iter()
                            .enumerate()
                        {
                            let wire = serde_json::to_value(&value).unwrap();
                            native_square(value.clone(), &expected[i]);
                            native_square(
                                serde_json::from_value(wire.clone()).unwrap(),
                                &expected[i],
                            );
                            native_square(
                                ciborium::de::from_reader(bytes(&wire).as_slice()).unwrap(),
                                &expected[i],
                            );
                            assert_eq!(serde_json::to_value(value).unwrap(), wire);
                        }
                    }
                    assert_eq!(serde_json::to_value(angle).unwrap(), graph);
                }
                for scale in [0, -1, 1, 2] {
                    let wire = json!({"rational":Rational::new(scale),"class":"Irrational","computable":graph});
                    let encoded = bytes(&wire);
                    let value = ciborium::Value::serialized(&wire).unwrap();
                    let collection = json!([wire]);
                    let mut json_values: Vec<Real> =
                        serde_json::from_value(collection.clone()).unwrap();
                    let mut cbor_values: Vec<Real> =
                        ciborium::de::from_reader(bytes(&collection).as_slice()).unwrap();
                    for angle in [
                        Real::from_json(&wire.to_string()).unwrap(),
                        Real::from_bytes(&encoded).unwrap(),
                        Real::try_from(&value).unwrap(),
                        Real::try_from(value).unwrap(),
                        json_values.pop().unwrap(),
                        cbor_values.pop().unwrap(),
                    ] {
                        let original_json = angle.to_json();
                        let original_cbor = angle.to_bytes();
                        for warm in [false, true] {
                            if warm {
                                let _ = angle.certified_rational_interval(-128).unwrap();
                            }
                            let squares = match scale {
                                0 => [Rational::zero(), Rational::one()],
                                2 => {
                                    // sin(2x)^2=4sin(x)^2cos(x)^2; cos(2x)^2=1-sin(2x)^2.
                                    let sine = Rational::new(4) * &expected[0] * &expected[1];
                                    [sine.clone(), Rational::one() - sine]
                                }
                                _ => expected.clone(),
                            };
                            let sine = angle.clone().sin();
                            let cosine = angle.clone().cos();
                            if scale == 0 {
                                assert!(matches!(angle.clone().cot(), Err(Problem::NotANumber)));
                            } else {
                                let cotangent = angle.clone().cot().unwrap();
                                real_square(cotangent.clone(), &(&squares[1] / &squares[0]));
                                assert_eq!(
                                    (cotangent * &sine - &cosine).certified_sign_until(0).sign(),
                                    Some(RealSign::Zero)
                                );
                            }
                            assert_eq!(
                                (&sine * &sine + &cosine * &cosine)
                                    .certified_cmp_until(&Real::one(), 0)
                                    .ordering(),
                                Some(Ordering::Equal)
                            );
                            for (i, value) in [sine, cosine].into_iter().enumerate() {
                                let json = value.to_json();
                                let cbor = value.to_bytes();
                                real_square(value, &squares[i]);
                                real_square(Real::from_json(&json).unwrap(), &squares[i]);
                                real_square(Real::from_bytes(&cbor).unwrap(), &squares[i]);
                            }
                        }
                        assert_eq!(angle.to_json(), original_json);
                        assert_eq!(angle.to_bytes(), original_cbor);
                    }
                }
            }
        }
    }
}

/// Build an independent directed 512-bit MPFR enclosure of the chosen signed square root.
/// Convert the nonnegative rational square exactly through GMP rational text;
/// negation reverses endpoints. Fixtures have finite roots, justifying conversion unwraps.
fn bounds(square: &Rational, negative: bool) -> [Rational; 2] {
    let square = square.to_string().parse::<Q>().unwrap();
    let mut lo = Float::with_val_round(512, &square, Round::Down).0;
    let mut hi = Float::with_val_round(512, &square, Round::Up).0;
    lo.sqrt_round(Round::Down);
    hi.sqrt_round(Round::Up);
    let [lo, hi] = [lo, hi].map(|v| {
        v.to_rational()
            .unwrap()
            .to_string()
            .parse::<Rational>()
            .unwrap()
    });
    if negative { [-hi, -lo] } else { [lo, hi] }
}

#[test]
/// Replay all 48 checked-in legacy trig rows through JSON and CBOR with independent signed-root enclosures.
/// Reconstruct each authored source and require its JSON value to match source_wire
/// separately from output validation. At precisions 0,-128,0, require the legacy
/// approximation interval to contain the reference enclosure and preserve output
/// serialization. This gives 288 finite enclosure checks, not symbolic equality
/// of every legacy graph. History/mode and prior certification flags in the fixture
/// are not consumed as proof. Register MPFR cleanup; replay stays in this process.
fn prior_atan_pi_anchor_graphs_remain_consumable() {
    mpfr::cleanup_on_thread_exit();
    let rows: Vec<Value> =
        serde_json::from_str(include_str!("fixtures/atan_pi_anchor_legacy_trig.json")).unwrap();
    assert_eq!(rows.len(), 48);
    for row in rows {
        let d = row["denominator"].as_u64().unwrap();
        let sign = row["sign"].as_i64().unwrap();
        let sine = row["function"] == "sin";
        let s2 = sine_square(d);
        let c2 = Rational::one() - s2.clone();
        let (squares, signs) = match row["composition"].as_str().unwrap() {
            "direct" => ([s2, c2], [sign, 1]),
            "pi-plus" => ([s2, c2], [-sign, -1]),
            "half-minus" => ([c2, s2], [1, sign]),
            "negative-half-minus" => ([c2, s2], [-1, -sign]),
            _ => unreachable!(),
        };
        let i = usize::from(!sine);
        let expected = bounds(&squares[i], signs[i] < 0);
        let argument = match d {
            3 => Computable::rational(Rational::new(3)).sqrt(),
            4 => Computable::one(),
            6 => Computable::rational(Rational::new(3)).sqrt().inverse(),
            _ => unreachable!(),
        }
        .multiply(Computable::rational(Rational::new(sign)))
        .atan();
        let source = match row["composition"].as_str().unwrap() {
            "direct" => argument,
            "pi-plus" => Computable::pi().add(argument),
            "half-minus" => Computable::pi()
                .multiply(Computable::rational(Rational::fraction(1, 2).unwrap()))
                .add(argument.negate()),
            "negative-half-minus" => Computable::pi()
                .multiply(Computable::rational(Rational::fraction(-1, 2).unwrap()))
                .add(argument.negate()),
            _ => unreachable!(),
        };
        assert_eq!(serde_json::to_value(source).unwrap(), row["source_wire"]);
        let wire = &row["legacy_output"];
        for value in [
            serde_json::from_value::<Computable>(wire.clone()).unwrap(),
            ciborium::de::from_reader(bytes(wire).as_slice()).unwrap(),
        ] {
            let original = serde_json::to_value(&value).unwrap();
            for p in [0, -128, 0] {
                let unit =
                    Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << -p).unwrap();
                let center = Rational::from_bigint(value.approx(p)) * &unit;
                assert!(&center - &unit <= expected[0] && &center + &unit >= expected[1]);
            }
            assert_eq!(serde_json::to_value(value).unwrap(), original);
        }
    }
}
