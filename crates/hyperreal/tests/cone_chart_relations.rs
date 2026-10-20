//! A cone's retained planar chart must preserve the exact endpoint identity.
#![cfg(feature = "serde")]

use hyperreal::{Rational, Real};
use num::BigInt;

const JOINT: &str = include_str!("fixtures/cone-chart-joint.json");

fn fresh() -> Real {
    serde_json::from_str(JOINT).unwrap()
}

#[test]
fn cold_cone_chart_joint_has_an_exact_zero_projection() {
    let value = fresh();
    assert!(value.exact_rational_ref().is_none());
    assert_eq!(value.exact_rational_normal_form(), Some(Rational::zero()));
    assert_eq!(fresh().floor_certified().unwrap(), BigInt::from(0));
    assert_eq!(fresh().ceil_certified().unwrap(), BigInt::from(0));
}

#[test]
fn cone_chart_projection_keeps_signed_nearby_offsets() {
    let epsilon = Rational::fraction(1, 1_000_000_000_000_u64).unwrap();
    for integer in [-7, 0, 11] {
        for sign in [-1, 0, 1] {
            let offset = Rational::from(integer) + Rational::from(sign) * &epsilon;
            let value = fresh() + Real::new(offset.clone());
            assert_eq!(value.exact_rational_normal_form(), Some(offset));
            let bytes = serde_json::to_vec(&value).unwrap();
            let replay: Real = serde_json::from_slice(&bytes).unwrap();
            assert_eq!(
                replay.exact_rational_normal_form(),
                value.exact_rational_normal_form()
            );
        }
    }
}

#[test]
fn captured_cone_endpoints_and_interior_witnesses_replay_exactly() {
    let rows: Vec<serde_json::Value> =
        serde_json::from_str(include_str!("fixtures/cone-continuation-zeros.json")).unwrap();
    for row in rows {
        let value: Real = serde_json::from_value(row["value"].clone()).unwrap();
        assert_eq!(
            value.exact_rational_normal_form(),
            Some(Rational::zero()),
            "{}",
            row["name"]
        );
    }
}
