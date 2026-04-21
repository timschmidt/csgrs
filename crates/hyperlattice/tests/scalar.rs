mod common;

use common::{r, unknown_zero};
use realistic_blas::{
    BlasProblem, Problem, Rational, ZeroStatus, acos, acosh, asin, atanh, ln, one, pi, powi,
    reciprocal, reciprocal_checked, sin, sqrt, tan, tau, zero, zero_status,
};

#[test]
fn scalar_functions() {
    assert_eq!(tau(), r(2) * pi());
    assert_eq!(sqrt(9.into()).unwrap(), r(3));
    assert_eq!(sin(pi()), zero());
    assert_eq!(ln(realistic_blas::e()).unwrap(), one());
}

#[test]
fn scalar_functions_reject_invalid_domains() {
    assert_eq!(reciprocal(zero()), Err(Problem::DivideByZero));
    assert_eq!(sqrt((-1).into()), Err(Problem::SqrtNegative));
    assert_eq!(ln(zero()), Err(Problem::NotANumber));
    assert_eq!(ln((-1).into()), Err(Problem::NotANumber));
    assert_eq!(asin(r(2)), Err(Problem::NotANumber));
    assert_eq!(acos(r(2)), Err(Problem::NotANumber));
    assert_eq!(acosh(zero()), Err(Problem::NotANumber));
    assert_eq!(atanh(one()), Err(Problem::Infinity));
    assert_eq!(powi(zero(), 0), Err(Problem::NotANumber));

    let half_pi = (pi() / r(2)).unwrap();
    assert_eq!(tan(half_pi), Err(Problem::NotANumber));
}

#[test]
fn zero_status_classifies_basic_values() {
    assert_eq!(zero_status(&zero()), ZeroStatus::Zero);
    assert_eq!(zero_status(&r(7)), ZeroStatus::NonZero);
    assert_eq!(zero_status(&pi()), ZeroStatus::NonZero);
    assert_eq!(zero_status(&unknown_zero()), ZeroStatus::Unknown);
}

#[test]
fn checked_scalar_reciprocal_rejects_zero() {
    assert_eq!(
        reciprocal_checked(zero()),
        Err(BlasProblem::Real(Problem::DivideByZero))
    );
    assert_eq!(
        reciprocal_checked(r(4)).unwrap(),
        Rational::fraction(1, 4).unwrap()
    );
}

#[test]
fn checked_scalar_reciprocal_rejects_unknown_zero() {
    assert_eq!(
        reciprocal_checked(unknown_zero()),
        Err(BlasProblem::UnknownZero)
    );
}
