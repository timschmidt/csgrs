mod common;

use common::{frac, r};
use hyperlattice::{Complex, Problem, i, zero};

#[test]
fn complex_i_squared() {
    assert_eq!((i() ^ 2).unwrap(), Complex::new(r(-1), r(0)));
}

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

#[test]
fn complex_negative_one_power_uses_reciprocal_semantics() {
    let value = Complex::new(r(3), r(4));

    assert_eq!(value.clone().powi(-1).unwrap(), value.reciprocal().unwrap());
}

#[test]
fn complex_display_forwards_real_formatting() {
    let value = Complex::new(frac(1, 2), r(2));

    assert_eq!(format!("{value}"), "(1/2 + 2i)");
    assert_eq!(format!("{value:#}"), "(0.5 + 2i)");
}

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
    assert_eq!(value.clone() / zero(), Err(Problem::DivideByZero));
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
        value.clone().div_real_checked(zero()),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        value.div_real_checked(r(2)).unwrap(),
        Complex::new(frac(3, 2), frac(2, 1))
    );
}
