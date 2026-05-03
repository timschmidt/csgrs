mod common;

use common::{abort_signal, frac, r, unknown_zero};
use realistic_blas::{
    Problem, ZeroStatus, acos, acosh, asin, asin_with_abort, atanh, ln, log10, log10_with_abort,
    one, pi, powi, reciprocal, reciprocal_checked, reciprocal_checked_with_abort, sin, sqrt, tan,
    tau, zero, zero_status, zero_status_with_abort,
};

#[test]
fn scalar_functions() {
    assert_eq!(tau(), r(2) * pi());
    assert_eq!(sqrt(9.into()).unwrap(), r(3));
    assert_eq!(sin(pi()), zero());
    assert_eq!(ln(realistic_blas::e()).unwrap(), one());
    assert_eq!(log10(r(100)).unwrap(), r(2));

    let signal = abort_signal();
    assert_eq!(log10_with_abort(r(1_000), &signal).unwrap(), r(3));
}

#[test]
fn scalar_functions_reject_invalid_domains() {
    assert_eq!(reciprocal(zero()), Err(Problem::DivideByZero));
    assert_eq!(sqrt(r(-1)), Err(Problem::SqrtNegative));
    assert_eq!(ln(zero()), Err(Problem::NotANumber));
    assert_eq!(ln(r(-1)), Err(Problem::NotANumber));
    assert_eq!(log10(zero()), Err(Problem::NotANumber));
    assert_eq!(log10(r(-1)), Err(Problem::NotANumber));
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

    let signal = abort_signal();
    assert_eq!(
        zero_status_with_abort(&unknown_zero(), &signal),
        ZeroStatus::Unknown
    );
}

#[test]
fn checked_scalar_reciprocal_rejects_zero() {
    assert_eq!(reciprocal_checked(zero()), Err(Problem::DivideByZero));
    assert_eq!(reciprocal_checked(r(4)).unwrap(), frac(1, 4));
}

#[test]
fn checked_scalar_reciprocal_rejects_unknown_zero() {
    assert_eq!(
        reciprocal_checked(unknown_zero()),
        Err(Problem::UnknownZero)
    );

    let signal = abort_signal();
    assert_eq!(
        reciprocal_checked_with_abort(unknown_zero(), &signal),
        Err(Problem::UnknownZero)
    );
}

#[cfg(not(feature = "hyperreal-backend"))]
#[test]
fn approx_scalar_tracks_unknown_zero_intervals() {
    let near_zero = realistic_blas::Scalar::approx(0.0, 0.25).unwrap();
    let nonzero = realistic_blas::Scalar::approx(1.0, 0.25).unwrap();
    let negative_interval = realistic_blas::Scalar::approx(-4.0, 1.0).unwrap();
    let mixed_interval = realistic_blas::Scalar::approx(1.0, 2.0).unwrap();
    let touching_zero = realistic_blas::Scalar::approx(0.25, 0.25).unwrap();
    let half = realistic_blas::Scalar::try_from(0.5).unwrap();

    assert_eq!(zero_status(&near_zero), ZeroStatus::Unknown);
    assert_eq!(zero_status(&nonzero), ZeroStatus::NonZero);
    assert_eq!(reciprocal_checked(near_zero), Err(Problem::UnknownZero));
    assert_eq!(
        realistic_blas::pow(negative_interval, half.clone()),
        Err(Problem::NotANumber)
    );
    assert_eq!(
        realistic_blas::pow(mixed_interval, half),
        Err(Problem::UnknownZero)
    );

    assert_eq!(
        zero_status(&realistic_blas::sqrt(touching_zero).unwrap()),
        ZeroStatus::Unknown
    );
}

#[test]
fn inverse_scalar_helpers_accept_abort_signal() {
    let signal = abort_signal();

    assert_eq!(asin_with_abort(zero(), &signal).unwrap(), zero());
}
