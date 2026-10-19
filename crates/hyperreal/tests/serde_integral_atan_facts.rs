#![cfg(feature = "serde")]

use hyperreal::{Computable, Problem, Rational, Real, RealSign};
use num::{BigInt, BigUint, One};
use rug::{Float, Rational as Q, float::Round};
use serde_json::{Value, json};
use std::cmp::Ordering;

#[path = "support/mpfr.rs"]
mod mpfr;

fn bytes(value: &Value) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

fn squared_expectations(n: &BigInt) -> [Rational; 2] {
    let square = n * n;
    let denominator = (&square + BigInt::one()).to_biguint().unwrap();
    [
        Rational::from_bigint_fraction(BigInt::one(), denominator.clone()).unwrap(),
        Rational::from_bigint_fraction(square, denominator).unwrap(),
    ]
}

fn native_square(value: Computable, expected: &Rational) {
    assert_eq!(
        value
            .square()
            .add(Computable::rational(-expected.clone()))
            .sign_until(0),
        Some(RealSign::Zero)
    );
}

fn real_square(value: &Real, expected: &Rational) {
    assert_eq!(
        (value * value)
            .certified_cmp_until(&Real::from(expected.clone()), 0)
            .ordering(),
        Some(Ordering::Equal)
    );
}

#[test]
fn integral_atan_retains_trig_facts_through_interchange() {
    let denominators = [
        BigInt::one(),
        BigInt::from(2),
        BigInt::from(5),
        BigInt::from(239),
        BigInt::one() << 512,
    ];
    for n in denominators.into_iter().flat_map(|n| [-n.clone(), n]) {
        // theta=atan(1/n): sin(theta)^2=1/(n^2+1),
        // cos(theta)^2=n^2/(n^2+1), cot(theta)=n.
        let expected = squared_expectations(&n);
        let graph = json!({"internal":{"IntegralAtan":n}});
        let collection = json!([graph]);
        let mut json_values: Vec<Computable> = serde_json::from_value(collection.clone()).unwrap();
        let mut cbor_values: Vec<Computable> =
            ciborium::de::from_reader(bytes(&collection).as_slice()).unwrap();
        let angles = [
            serde_json::from_value::<Computable>(graph.clone()).unwrap(),
            ciborium::de::from_reader(bytes(&graph).as_slice()).unwrap(),
            json_values.pop().unwrap(),
            cbor_values.pop().unwrap(),
        ];
        for angle in angles {
            assert_eq!(serde_json::to_value(&angle).unwrap(), graph);
            for warm in [false, true] {
                if warm {
                    let _ = angle.approx(-128);
                }
                for (value, square) in [
                    (angle.clone().sin(), &expected[0]),
                    (angle.clone().cos(), &expected[1]),
                ] {
                    let wire = serde_json::to_value(&value).unwrap();
                    let mut encoded = Vec::new();
                    ciborium::ser::into_writer(&value, &mut encoded).unwrap();
                    native_square(value, square);
                    native_square(serde_json::from_value(wire).unwrap(), square);
                    native_square(
                        ciborium::de::from_reader(encoded.as_slice()).unwrap(),
                        square,
                    );
                }
            }
            assert_eq!(serde_json::to_value(angle).unwrap(), graph);
        }
        for scale in [0, 1, -1, 2] {
            let wire =
                json!({"rational":Rational::new(scale),"class":"Irrational","computable":graph});
            let cbor = ciborium::Value::serialized(&wire).unwrap();
            let collection = json!([wire]);
            let mut json_values: Vec<Real> = serde_json::from_value(collection.clone()).unwrap();
            let mut cbor_values: Vec<Real> =
                ciborium::de::from_reader(bytes(&collection).as_slice()).unwrap();
            let angles = [
                Real::from_json(&wire.to_string()).unwrap(),
                Real::from_bytes(&bytes(&wire)).unwrap(),
                Real::try_from(&cbor).unwrap(),
                Real::try_from(cbor).unwrap(),
                json_values.pop().unwrap(),
                cbor_values.pop().unwrap(),
            ];
            for angle in angles {
                let original_json = angle.to_json();
                let original_cbor = angle.to_bytes();
                for warm in [false, true] {
                    if warm {
                        let _ = angle.certified_rational_interval(-128).unwrap();
                    }
                    let sine = angle.clone().sin();
                    let cosine = angle.clone().cos();
                    if scale == 2 {
                        // A scaled atan must not be mistaken for the unscaled
                        // angle: sin(2 theta)=2n/(n^2+1),
                        // cos(2 theta)=(n^2-1)/(n^2+1).
                        let square = &n * &n;
                        let denominator = (&square + BigInt::one()).to_biguint().unwrap();
                        let twice_sine = Rational::from_bigint_fraction(
                            &n * BigInt::from(2),
                            denominator.clone(),
                        )
                        .unwrap();
                        let twice_cosine =
                            Rational::from_bigint_fraction(square - BigInt::one(), denominator)
                                .unwrap();
                        for (value, target) in [(sine, twice_sine), (cosine, twice_cosine)] {
                            let [lo, hi] = value.certified_rational_interval(-128).unwrap();
                            assert!(lo <= target && hi >= target);
                        }
                    } else {
                        let squares = if scale == 0 {
                            [Rational::zero(), Rational::one()]
                        } else {
                            expected.clone()
                        };
                        for (value, square) in [(sine, &squares[0]), (cosine, &squares[1])] {
                            real_square(&value, square);
                            let json = value.to_json();
                            let cbor = value.to_bytes();
                            real_square(&Real::from_json(&json).unwrap(), square);
                            real_square(&Real::from_bytes(&cbor).unwrap(), square);
                            assert_eq!(value.to_json(), json);
                            assert_eq!(value.to_bytes(), cbor);
                        }
                        if scale == 0 {
                            assert!(matches!(angle.clone().cot(), Err(Problem::NotANumber)));
                        } else {
                            let cotangent = angle.clone().cot().unwrap();
                            let expected = Real::from(Rational::from_bigint(&n * scale));
                            assert_eq!(
                                cotangent.certified_cmp_until(&expected, 0).ordering(),
                                Some(Ordering::Equal)
                            );
                            assert_eq!(
                                (cotangent - expected).certified_sign_until(0).sign(),
                                Some(RealSign::Zero)
                            );
                        }
                    }
                }
                assert_eq!(angle.to_json(), original_json);
                assert_eq!(angle.to_bytes(), original_cbor);
            }
        }
    }
}

fn legacy_bounds(n: i64, sine: bool) -> [Rational; 2] {
    let square = Q::from((if sine { 1 } else { n * n }, n * n + 1));
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
    if sine && n < 0 { [-hi, -lo] } else { [lo, hi] }
}

#[test]
fn prior_integral_atan_trig_graphs_remain_consumable() {
    mpfr::cleanup_on_thread_exit();
    let fixture: Value =
        serde_json::from_str(include_str!("fixtures/integral_atan_legacy_trig.json")).unwrap();
    assert_eq!(
        fixture["target_revision"],
        "d5b16c26471768ff53a59fbce8285e288c447e2f"
    );
    let rows = fixture["rows"].as_array().unwrap();
    assert_eq!(rows.len(), 24);
    for row in rows {
        let bounds = legacy_bounds(
            row["denominator"].as_i64().unwrap(),
            row["operation"] == "sin",
        );
        let wire = &row["derived_json"];
        let encoded: Vec<u8> = serde_json::from_value(row["derived_cbor"].clone()).unwrap();
        if row["mode"] == "native" {
            for value in [
                serde_json::from_value::<Computable>(wire.clone()).unwrap(),
                ciborium::de::from_reader(encoded.as_slice()).unwrap(),
            ] {
                let before = serde_json::to_value(&value).unwrap();
                for p in [0, -128, 0] {
                    let unit = Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << -p)
                        .unwrap();
                    let center = Rational::from_bigint(value.approx(p)) * &unit;
                    assert!(&center - &unit <= bounds[0] && &center + &unit >= bounds[1]);
                }
                assert_eq!(serde_json::to_value(&value).unwrap(), before);
            }
        } else {
            assert_eq!(row["mode"], "real");
            for value in [
                Real::from_json(&wire.to_string()).unwrap(),
                Real::from_bytes(&encoded).unwrap(),
            ] {
                let before = value.to_json();
                for p in [0, -128, 0] {
                    let [lo, hi] = value.certified_rational_interval(p).unwrap();
                    assert!(lo <= bounds[0] && hi >= bounds[1]);
                }
                assert_eq!(value.to_json(), before);
            }
        }
    }
}
