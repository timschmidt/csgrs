//! Exact polynomial factors retain their divisor-domain obligations.
use hyperreal::{Rational, Real, RealSign};

fn quotient_and_polynomial(scale: Rational, remainder: Rational) -> (Real, Real) {
    let a = Real::from(4) + Real::e().sin();
    let b = Real::one() + (Real::pi() / Real::from(8)).unwrap().sin();
    let numerator = &a * &a - &b * &b + Real::from(remainder);
    let denominator = &a - &b;
    let scale = Real::from(scale);
    // a > 3 and b < 2, so a-b > 1; both tested scales are nonzero.
    (
        ((numerator * &scale) / (denominator * scale)).unwrap(),
        a + b,
    )
}

fn eighth_power(mut value: Real) -> Real {
    for _ in 0..3 {
        value = &value * &value;
    }
    value
}

#[test]
fn polynomial_quotients_compose_without_expanding_unused_factors() {
    for scale in [
        Rational::one(),
        Rational::fraction(-1, 1_000_000_000_000).unwrap(),
    ] {
        let (quotient, polynomial) = quotient_and_polynomial(scale, Rational::zero());
        let value = eighth_power(quotient) - eighth_power(polynomial);
        #[cfg(feature = "serde")]
        let before = serde_json::to_string(&value).unwrap();
        assert_eq!(value.exact_rational_normal_form(), Some(Rational::zero()));
        assert_eq!(value.certified_sign_until(0).sign(), Some(RealSign::Zero));
        #[cfg(feature = "serde")]
        assert_eq!(serde_json::to_string(&value).unwrap(), before);
        for numerator in [-1, 1] {
            let displacement = Rational::fraction(numerator, 1_000_000_000_000).unwrap();
            assert_eq!(
                (&value + Real::from(displacement.clone())).exact_rational_normal_form(),
                Some(displacement)
            );
        }
    }
}

#[test]
fn a_nonzero_polynomial_remainder_is_never_cancelled() {
    for numerator in [-1, 1] {
        let remainder = Rational::fraction(numerator, 1_000_000_000_000).unwrap();
        let (quotient, polynomial) = quotient_and_polynomial(Rational::one(), remainder);
        let difference = eighth_power(quotient) - eighth_power(polynomial);
        assert_ne!(
            difference.exact_rational_normal_form(),
            Some(Rational::zero())
        );
        // Both bases are positive. Division by a-b > 1 and the strictly
        // increasing eighth power preserve the sign of the tiny remainder.
        assert_eq!(
            difference.certified_sign_until(-512).sign(),
            Some(if numerator < 0 {
                RealSign::Negative
            } else {
                RealSign::Positive
            })
        );
    }
}
