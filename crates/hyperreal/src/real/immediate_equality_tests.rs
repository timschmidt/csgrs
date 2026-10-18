use super::Real;
use crate::Rational;
use num::{BigInt, BigRational, One};

#[test]
fn immediate_rational_equality_matches_an_independent_fraction_oracle() {
    let mut values = Vec::new();
    for n in -7..=7 {
        for d in [1, 2, 3, 7, 11] {
            values.push((
                Real::new(Rational::fraction(n, d).unwrap()),
                BigRational::new(BigInt::from(n), BigInt::from(d)),
            ));
        }
    }
    for bits in [129_usize, 521, 4097] {
        let wide = BigInt::one() << bits;
        for offset in [1, 3, 7] {
            let n: BigInt = &wide + offset;
            let d: BigInt = &wide + 11;
            values.push((
                Real::new(
                    Rational::from_bigint_fraction(n.clone(), d.magnitude().clone()).unwrap(),
                ),
                BigRational::new(n, d),
            ));
        }
    }
    let mut decided = 0;
    for (left, a) in &values {
        for (right, b) in &values {
            if let Some(equal) = left.immediate_equality(right) {
                assert_eq!(equal, a == b);
                assert_eq!(right.immediate_equality(left), Some(equal));
                decided += 1;
            }
        }
    }
    assert!(
        decided >= 75 * 75,
        "the bounded integer/fraction domain must decide"
    );
}

#[test]
fn immediate_equality_does_not_turn_symbolic_uncertainty_into_inequality() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Real::one();
    let origin = Real::zero();
    for precision in [-8, -128, -16, -96] {
        zero.certified_rational_interval(precision).unwrap();
        assert_ne!(zero.immediate_equality(&origin), Some(false));
        assert_ne!(origin.immediate_equality(&zero), Some(false));
    }
}
