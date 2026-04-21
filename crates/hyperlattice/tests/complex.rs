mod common;

use common::r;
use realistic_blas::{Complex, Problem, Rational, i, zero};

#[test]
fn complex_i_squared() {
    assert_eq!((i() ^ 2).unwrap(), Complex::new(r(-1), r(0)));
}

#[test]
fn complex_display_forwards_real_formatting() {
    let value = Complex::new(Rational::fraction(1, 2).unwrap().into(), r(2));

    #[cfg(feature = "realistic-backend")]
    assert_eq!(format!("{value}"), "(1/2 + 2i)");
    #[cfg(not(feature = "realistic-backend"))]
    assert_eq!(format!("{value}"), "(0.5 + 2i)");
    assert_eq!(format!("{value:#}"), "(0.5 + 2i)");
}

#[test]
fn checked_complex_operations_reject_zero_denominators() {
    let value = Complex::new(r(3), r(4));

    assert_eq!(Complex::zero().reciprocal(), Err(Problem::DivideByZero));
    assert_eq!(value.clone() / Complex::zero(), Err(Problem::DivideByZero));
    assert_eq!(value.clone() / zero(), Err(Problem::DivideByZero));
    assert_eq!(Complex::zero().powi(-1), Err(Problem::DivideByZero));
    assert_eq!(Complex::zero().powi_checked(0), Err(Problem::NotANumber));
    assert_eq!(
        Complex::zero().reciprocal_checked(),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        value.clone().div_real_checked(zero()),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        value.div_real_checked(r(2)).unwrap(),
        Complex::new(
            Rational::fraction(3, 2).unwrap().into(),
            Rational::fraction(2, 1).unwrap().into()
        )
    );
}
