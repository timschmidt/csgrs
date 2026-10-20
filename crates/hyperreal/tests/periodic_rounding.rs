//! Exact chart offsets must round without a preceding predicate warming them.
#![cfg(feature = "serde")]

use hyperreal::Real;
use num::BigInt;

const ZEROS: &str = include_str!("fixtures/periodic-rounding-zeros.json");

fn fresh(index: usize) -> Real {
    let values: Vec<Real> = serde_json::from_str(ZEROS).unwrap();
    values[index].clone()
}

#[test]
fn periodic_zero_offsets_round_from_cold_serialized_expressions() {
    for index in 0..3 {
        let floor = fresh(index);
        assert!(floor.exact_rational_ref().is_none());
        assert_eq!(floor.floor_certified().unwrap(), BigInt::from(0));
        let ceil = fresh(index);
        assert!(ceil.exact_rational_ref().is_none());
        assert_eq!(ceil.ceil_certified().unwrap(), BigInt::from(0));
    }
}

#[test]
fn translated_integer_boundaries_and_tiny_offsets_keep_their_direction() {
    let gap = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    for index in 0..3 {
        for integer in [-100, -2, -1, 0, 1, 2, 100] {
            for (offset, floor, ceil) in [
                (-gap.clone(), integer - 1, integer),
                (Real::zero(), integer, integer),
                (gap.clone(), integer, integer + 1),
            ] {
                let value = || fresh(index) + Real::from(integer) + &offset;
                assert_eq!(value().floor_certified().unwrap(), BigInt::from(floor));
                assert_eq!(value().ceil_certified().unwrap(), BigInt::from(ceil));
            }
        }
    }
}

#[test]
fn exact_half_boundaries_round_away_from_zero_after_replay() {
    let half = (Real::one() / Real::from(2)).unwrap();
    for index in 0..3 {
        for sign in [-1, 1] {
            let value = || fresh(index) + Real::from(sign) * &half;
            assert_eq!(value().trunc_certified().unwrap(), BigInt::from(0));
            assert_eq!(value().round_certified().unwrap(), BigInt::from(sign));
        }
    }
}
