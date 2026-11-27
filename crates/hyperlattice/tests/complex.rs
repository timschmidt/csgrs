//! Finite exact-rational complex arithmetic and display regressions.
//!
//! Expectations use scalar formulas or exact literal values; these checks cover the
//! listed fixtures and ownership forms, not every symbolic representation or domain.

/// Shared exact-rational fixture constructors used by these tests.
mod common;

use common::{frac, r};
use hyperlattice::{Complex, Problem, Real};

/// Check the BitXor power alias sends i squared to exact -1+0i.
#[test]
fn complex_i_squared() {
    assert_eq!((Complex::i() ^ 2).unwrap(), Complex::new(r(-1), r(0)));
}

/// Check all four ownership forms against the scalar ac-bd/ad+bc formulas for one rational fixture; structural equality suffices for these exact components.
#[test]
fn complex_multiplication_preserves_exact_components_for_all_ownership_forms() {
    let lhs = Complex::new(frac(3, 7), frac(-5, 11));
    let rhs = Complex::new(frac(13, 17), frac(19, 23));
    let expected = Complex::new(
        &lhs.re * &rhs.re - &lhs.im * &rhs.im,
        &lhs.re * &rhs.im + &lhs.im * &rhs.re,
    );

    assert_eq!(lhs.clone() * rhs.clone(), expected);
    assert_eq!(lhs.clone() * &rhs, expected);
    assert_eq!(&lhs * rhs.clone(), expected);
    assert_eq!(&lhs * &rhs, expected);
}

/// Check four division ownership forms and checked division against the conjugate/norm scalar formulas for one nonzero rational divisor.
#[test]
fn complex_division_preserves_exact_components_for_all_ownership_forms() {
    let lhs = Complex::new(frac(3, 7), frac(-5, 11));
    let rhs = Complex::new(frac(13, 17), frac(19, 23));
    let denominator = &rhs.re * &rhs.re + &rhs.im * &rhs.im;
    let expected = Complex::new(
        (((&lhs.re * &rhs.re) + (&lhs.im * &rhs.im)) / &denominator).unwrap(),
        (((&lhs.im * &rhs.re) - (&lhs.re * &rhs.im)) / &denominator).unwrap(),
    );

    assert_eq!((lhs.clone() / rhs.clone()).unwrap(), expected);
    assert_eq!((lhs.clone() / &rhs).unwrap(), expected);
    assert_eq!((&lhs / rhs.clone()).unwrap(), expected);
    assert_eq!((&lhs / &rhs).unwrap(), expected);
    assert_eq!(lhs.div_checked(rhs).unwrap(), expected);
}

/// Compare exponent -1 with reciprocal on 3+4i; this consistency assertion does not independently check the reciprocal formula or trace path.
#[test]
fn complex_negative_one_power_uses_reciprocal_semantics() {
    let value = Complex::new(r(3), r(4));

    assert_eq!(value.clone().powi(-1).unwrap(), value.reciprocal().unwrap());
}

/// Check default rational and alternate decimal spelling for 1/2+2i; does not establish forwarding of outer width/precision flags.
#[test]
fn complex_display_forwards_real_formatting() {
    let value = Complex::new(frac(1, 2), r(2));

    assert_eq!(format!("{value}"), "(1/2 + 2i)");
    assert_eq!(format!("{value:#}"), "(0.5 + 2i)");
}

/// Check selected ordinary/checked zero-divisor errors, checked zero-to-zero power, and exact division by real two; unknown-zero cases are not covered here.
#[test]
fn checked_complex_operations_reject_zero_denominators() {
    let value = Complex::new(r(3), r(4));
    let zero_complex: Complex = Complex::zero();

    assert_eq!(
        zero_complex.clone().reciprocal(),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        value.clone() / zero_complex.clone(),
        Err(Problem::DivideByZero)
    );
    assert_eq!(value.clone() / Real::zero(), Err(Problem::DivideByZero));
    assert_eq!(zero_complex.clone().powi(-1), Err(Problem::DivideByZero));
    assert_eq!(
        zero_complex.clone().powi_checked(0),
        Err(Problem::NotANumber)
    );
    assert_eq!(
        zero_complex.reciprocal_checked(),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        value.clone().div_real_checked(Real::zero()),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        value.div_real_checked(r(2)).unwrap(),
        Complex::new(frac(3, 2), frac(2, 1))
    );
}
