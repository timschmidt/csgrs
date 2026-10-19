#![cfg(feature = "serde")]

use hyperreal::{Computable, Problem, Rational, Real, RealSign};
use rug::{Float, float::Round};
use serde_json::{Value, json};

#[path = "support/mpfr.rs"]
mod mpfr;

fn node(kind: &str, argument: Value) -> Value {
    json!({"internal": {kind: argument}})
}

fn bytes(value: &Value) -> Vec<u8> {
    let mut bytes = Vec::new();
    ciborium::ser::into_writer(value, &mut bytes).unwrap();
    bytes
}

fn oracle(magnitude: &Rational) -> [Rational; 2] {
    let x: rug::Rational = (-magnitude.clone()).to_string().parse().unwrap();
    // acos reverses the input bounds because it is decreasing.
    let mut lo = Float::with_val_round(2048, &x, Round::Up).0;
    let mut hi = Float::with_val_round(2048, &x, Round::Down).0;
    lo.acos_round(Round::Down);
    hi.acos_round(Round::Up);
    [lo, hi].map(|value| value.to_rational().unwrap().to_string().parse().unwrap())
}

#[test]
fn acos_complement_preserves_adjacent_pi_cache_enclosures() {
    mpfr::cleanup_on_thread_exit();
    // acos(-49/50)=pi-2*atan(1/sqrt(99)) lies between 5/2 and 3:
    // 3<pi<22/7, 4<sqrt(99)<10, and 2*atan(1/10)>299/1500>1/7.
    // The adjacent-precision pi sample exposed the unguarded subtraction.
    let _ = Computable::pi().approx(-2);
    for (magnitude, lower, upper) in [
        (
            Rational::fraction(49, 50).unwrap(),
            Rational::fraction(5, 2).unwrap(),
            Rational::new(3),
        ),
        // acos(-1/2)=2*pi/3 lies between 2 and 5/2.
        (
            Rational::fraction(1, 2).unwrap(),
            Rational::new(2),
            Rational::fraction(5, 2).unwrap(),
        ),
    ] {
        let bounds = oracle(&magnitude);
        let graph = node("AcosNegativeRational", json!(magnitude));
        let collection = json!([node("Ratio", json!(Rational::one())), graph]);
        let mut json_collection: Vec<Computable> =
            serde_json::from_value(collection.clone()).unwrap();
        let mut cbor_collection: Vec<Computable> =
            ciborium::de::from_reader(bytes(&collection).as_slice()).unwrap();
        let values = [
            Computable::rational(-magnitude.clone()).acos(),
            serde_json::from_value::<Computable>(graph.clone()).unwrap(),
            ciborium::de::from_reader(bytes(&graph).as_slice()).unwrap(),
            json_collection.pop().unwrap(),
            cbor_collection.pop().unwrap(),
        ];
        for value in values {
            assert_eq!(serde_json::to_value(&value).unwrap(), graph);
            for p in [-1, -64, -1] {
                let unit: Rational = if p == -1 {
                    Rational::fraction(1, 2).unwrap()
                } else {
                    format!("1/{}", 1_u128 << -p).parse().unwrap()
                };
                let center = Rational::from_bigint(value.approx(p)) * &unit;
                assert!(&center - &unit <= bounds[0] && &center + &unit >= bounds[1]);
                assert!(&center - &unit < upper && &center + &unit > lower);
                if p == -64 {
                    assert!(&center - &unit > lower && &center + &unit < upper);
                }
            }
            assert_eq!(serde_json::to_value(value).unwrap(), graph);
        }
        for scale in [0, 1, -3] {
            let wire =
                json!({"rational":Rational::new(scale),"class":"Irrational","computable":graph});
            let cbor = ciborium::Value::serialized(&wire).unwrap();
            let collection = json!([Real::one(), wire]);
            let mut values = vec![
                Real::from(Rational::new(scale)) * Real::from(-magnitude.clone()).acos().unwrap(),
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
            let (oracle_lo, oracle_hi) = if scale >= 0 {
                (&bounds[0] * &factor, &bounds[1] * &factor)
            } else {
                (&bounds[1] * &factor, &bounds[0] * &factor)
            };
            let expected_sign = if scale == 0 {
                RealSign::Zero
            } else if lo_bound > Rational::zero() {
                RealSign::Positive
            } else {
                RealSign::Negative
            };
            for value in values {
                let json = value.to_json();
                let cbor = value.to_bytes();
                for p in [-1, -64, -1] {
                    let [lo, hi] = value.certified_rational_interval(p).unwrap();
                    assert!(lo <= oracle_lo && hi >= oracle_hi);
                    if scale == 0 {
                        assert!(lo <= Rational::zero() && hi >= Rational::zero());
                    } else {
                        assert!(lo < hi_bound && hi > lo_bound);
                        if p == -64 {
                            assert!(lo > lo_bound && hi < hi_bound);
                        }
                    }
                    assert_eq!(value.certified_sign_until(p).sign(), Some(expected_sign));
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
fn negative_acos_import_preserves_typed_magnitude_failures() {
    for argument in [-2, 2] {
        assert!(matches!(
            Real::from(Rational::new(argument)).acos(),
            Err(Problem::NotANumber)
        ));
    }
    for argument in [-1, 2] {
        let graph = node("AcosNegativeRational", json!(Rational::new(argument)));
        for invalid in [
            graph.clone(),
            node(
                "Multiply",
                json!([node("Ratio", json!(Rational::zero())), graph]),
            ),
        ] {
            assert!(serde_json::from_value::<Computable>(invalid.clone()).is_err());
            assert!(
                ciborium::de::from_reader::<Computable, _>(bytes(&invalid).as_slice()).is_err()
            );
            let collection = json!([node("Ratio", json!(Rational::one())), invalid]);
            assert!(serde_json::from_value::<Vec<Computable>>(collection.clone()).is_err());
            assert!(
                ciborium::de::from_reader::<Vec<Computable>, _>(bytes(&collection).as_slice())
                    .is_err()
            );
            for scale in [0, 1, -3] {
                let wire = json!({"rational":Rational::new(scale),"class":"Irrational","computable":invalid});
                let cbor = ciborium::Value::serialized(&wire).unwrap();
                assert!(matches!(
                    Real::from_json(&wire.to_string()),
                    Err(Problem::ParseError)
                ));
                assert!(matches!(
                    Real::from_bytes(&bytes(&wire)),
                    Err(Problem::ParseError)
                ));
                assert!(matches!(Real::try_from(&cbor), Err(Problem::ParseError)));
                assert!(matches!(Real::try_from(cbor), Err(Problem::ParseError)));
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
