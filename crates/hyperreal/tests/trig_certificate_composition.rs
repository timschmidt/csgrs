#![cfg(feature = "serde")]

use hyperreal::{Rational, Real};
use serde_json::Value;

fn reciprocal(value: &Real) {
    assert_eq!(
        (value * value.inverse_ref().unwrap()).exact_rational_normal_form(),
        Some(Rational::one()),
        "{}",
        value.to_json(),
    );
}

#[test]
fn cold_and_reimported_trig_certificates_compose_through_scaling() {
    for value in [
        Real::from(Rational::fraction(1, 5).unwrap()).sin_pi(),
        Real::from(Rational::fraction(1, 5).unwrap())
            .tan_pi()
            .unwrap(),
    ] {
        let wire = serde_json::to_value(&value).unwrap();
        for lazy in [false, true] {
            let mut wire = wire.clone();
            if lazy {
                wire["computable"] = Value::Null;
            }
            let value = Real::from_json(&wire.to_string()).unwrap();
            let before = value.to_json();
            for warm in [false, true] {
                if warm {
                    value.certified_rational_interval(-128).unwrap();
                }
                reciprocal(&value);
                for result in [
                    &value * Real::from(2),
                    &value * Real::from(-3),
                    (&value / Real::from(2)).unwrap(),
                    &value + &value,
                ] {
                    reciprocal(&result);
                    // A fresh import must be able to recover its proof from
                    // the represented graph or certificate, without cache bits.
                    reciprocal(&Real::from_bytes(&result.to_bytes()).unwrap());
                }
            }
            assert_eq!(value.to_json(), before);
        }
    }
}
