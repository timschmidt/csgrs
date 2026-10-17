//! Independently serialized circle endpoints retain their common exact angle.
#![cfg(feature = "serde")]
use hyperreal::{Rational, Real};

const PAIRS: &str = include_str!("fixtures/periodic-endpoint-pairs.json");

#[test]
fn replayed_endpoint_differences_have_exact_zero_certificates() {
    let pairs: Vec<[Real; 2]> = serde_json::from_str(PAIRS).unwrap();
    for [a, b] in pairs {
        assert_eq!(
            (&a - &b).exact_rational_normal_form(),
            Some(Rational::zero())
        );
        assert_eq!(
            (&b - &a).exact_rational_normal_form(),
            Some(Rational::zero())
        );
    }
}

#[test]
fn endpoint_proofs_preserve_small_signed_coordinate_offsets() {
    let epsilon = Rational::fraction(1, 1_000_000_000_000_u64).unwrap();
    for sign in [-1, 1] {
        let offset = &epsilon * Rational::from(sign);
        let pairs: Vec<[Real; 2]> = serde_json::from_str(PAIRS).unwrap();
        for [a, b] in pairs {
            assert_eq!(
                (a - b + Real::new(offset.clone())).exact_rational_normal_form(),
                Some(offset.clone())
            );
        }
    }
}
