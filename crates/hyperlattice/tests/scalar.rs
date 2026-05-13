mod common;

use common::{abort_signal, frac, r, unknown_zero};
use hyperlattice::{
    Problem, ScalarFacts, ScalarSign, ZeroStatus, acos, acosh, asin, asin_with_abort, atanh, ln,
    log10, log10_with_abort, one, pi, powi, reciprocal, reciprocal_checked,
    reciprocal_checked_with_abort, reciprocal_ref, reciprocal_ref_checked, sin, sqrt, tan, tau,
    zero, zero_status, zero_status_with_abort,
};

#[test]
fn scalar_functions() {
    assert_eq!(tau(), r(2) * pi());
    assert_eq!(sqrt(9.into()).unwrap(), r(3));
    assert_eq!(sin(pi()), zero());
    assert_eq!(ln(hyperlattice::e()).unwrap(), one());
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
fn scalar_structural_facts_classify_basic_values() {
    let zero_facts = zero().structural_facts();
    assert_eq!(zero_facts.sign, Some(ScalarSign::Zero));
    assert_eq!(zero_facts.zero, ZeroStatus::Zero);
    assert!(zero().definitely_zero());

    let positive_facts = r(7).structural_facts();
    assert_eq!(positive_facts.sign, Some(ScalarSign::Positive));
    assert_eq!(positive_facts.zero, ZeroStatus::NonZero);
    assert!(!r(7).definitely_zero());

    let negative_facts = r(-7).structural_facts();
    assert_eq!(negative_facts.sign, Some(ScalarSign::Negative));
    assert_eq!(negative_facts.zero, ZeroStatus::NonZero);

    assert_eq!(zero().refine_sign_until(-64), Some(ScalarSign::Zero));
    assert_eq!(r(9).refine_sign_until(-64), Some(ScalarSign::Positive));
    assert_eq!(r(-9).refine_sign_until(-64), Some(ScalarSign::Negative));
}

#[test]
fn scalar_to_f64_approx_is_borrowed() {
    let value = r(7);
    assert_eq!(value.to_f64_approx(), Some(7.0));
    assert_eq!(value, r(7));
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn hyperreal_scalar_forwards_symbolic_structural_facts() {
    let pi_facts = pi().structural_facts();
    assert_eq!(pi_facts.sign, Some(ScalarSign::Positive));
    assert_eq!(pi_facts.zero, ZeroStatus::NonZero);
    assert!(!pi_facts.exact_rational);
    assert!(pi_facts.magnitude.is_some());

    let sqrt_two = sqrt(r(2)).unwrap();
    let sqrt_facts = sqrt_two.structural_facts();
    assert_eq!(sqrt_facts.sign, Some(ScalarSign::Positive));
    assert_eq!(sqrt_facts.zero, ZeroStatus::NonZero);
    assert!(!sqrt_facts.exact_rational);

    let rational_facts = frac(1, 2).structural_facts();
    assert_eq!(rational_facts.sign, Some(ScalarSign::Positive));
    assert_eq!(rational_facts.zero, ZeroStatus::NonZero);
    assert!(rational_facts.exact_rational);
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn hyperreal_scalar_keeps_unknown_structural_facts_unknown() {
    assert_eq!(
        unknown_zero().structural_facts(),
        ScalarFacts {
            sign: None,
            zero: ZeroStatus::Unknown,
            exact_rational: false,
            magnitude: None,
        }
    );
    assert_eq!(unknown_zero().refine_sign_until(-1), None);
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn hyperreal_inverse_trig_helpers_preserve_exact_paths() {
    assert_eq!(asin(frac(1, 2)).unwrap(), (pi() / r(6)).unwrap());
    assert_eq!(acos(frac(1, 2)).unwrap(), (pi() / r(3)).unwrap());
    assert_eq!(hyperlattice::atan(one()).unwrap(), (pi() / r(4)).unwrap());

    assert_eq!(one().atan().unwrap(), (pi() / r(4)).unwrap());
}

#[test]
fn checked_scalar_reciprocal_rejects_zero() {
    assert_eq!(reciprocal_checked(zero()), Err(Problem::DivideByZero));
    assert_eq!(reciprocal_checked(r(4)).unwrap(), frac(1, 4));
}

#[test]
fn borrowed_scalar_reciprocal_does_not_consume_input() {
    let value = r(4);
    assert_eq!(reciprocal_ref(&value).unwrap(), frac(1, 4));
    assert_eq!(reciprocal_ref_checked(&value).unwrap(), frac(1, 4));
    assert_eq!(value, r(4));
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
    let near_zero = hyperlattice::Scalar::approx(0.0, 0.25).unwrap();
    let nonzero = hyperlattice::Scalar::approx(1.0, 0.25).unwrap();
    let negative_interval = hyperlattice::Scalar::approx(-4.0, 1.0).unwrap();
    let mixed_interval = hyperlattice::Scalar::approx(1.0, 2.0).unwrap();
    let touching_zero = hyperlattice::Scalar::approx(0.25, 0.25).unwrap();
    let half = hyperlattice::Scalar::try_from(0.5).unwrap();

    assert_eq!(zero_status(&near_zero), ZeroStatus::Unknown);
    assert_eq!(zero_status(&nonzero), ZeroStatus::NonZero);
    assert_eq!(reciprocal_checked(near_zero), Err(Problem::UnknownZero));
    assert_eq!(
        hyperlattice::pow(negative_interval, half.clone()),
        Err(Problem::NotANumber)
    );
    assert_eq!(
        hyperlattice::pow(mixed_interval, half),
        Err(Problem::UnknownZero)
    );

    assert_eq!(
        zero_status(&hyperlattice::sqrt(touching_zero).unwrap()),
        ZeroStatus::Unknown
    );
}

#[cfg(not(feature = "hyperreal-backend"))]
#[test]
fn approx_scalar_structural_facts_track_intervals() {
    let near_zero = hyperlattice::Scalar::approx(0.0, 0.25).unwrap();
    let positive = hyperlattice::Scalar::approx(4.0, 0.25).unwrap();
    let negative = hyperlattice::Scalar::approx(-4.0, 0.25).unwrap();

    assert_eq!(
        near_zero.structural_facts(),
        ScalarFacts {
            sign: None,
            zero: ZeroStatus::Unknown,
            exact_rational: false,
            magnitude: None,
        }
    );

    let positive_facts = positive.structural_facts();
    assert_eq!(positive_facts.sign, Some(ScalarSign::Positive));
    assert_eq!(positive_facts.zero, ZeroStatus::NonZero);
    assert!(!positive_facts.exact_rational);
    assert!(positive_facts.magnitude.is_some());
    assert_eq!(positive.to_f64_approx(), Some(4.0));
    assert_eq!(positive.refine_sign_until(-64), Some(ScalarSign::Positive));

    let negative_facts = negative.structural_facts();
    assert_eq!(negative_facts.sign, Some(ScalarSign::Negative));
    assert_eq!(negative_facts.zero, ZeroStatus::NonZero);
    assert!(!negative_facts.exact_rational);
    assert!(negative_facts.magnitude.is_some());
    assert_eq!(negative.to_f64_approx(), Some(-4.0));
}

#[cfg(feature = "approx-backend")]
#[test]
fn approx_scalar_partial_eq_handles_overflowed_infinities() {
    let huge = hyperlattice::Scalar::<hyperlattice::ApproxBackend>::try_from(1.0e308).unwrap();
    let positive_inf = huge.clone() * huge.clone();
    let same_positive_inf = huge.clone() * huge.clone();
    let negative_inf = -positive_inf.clone();

    assert_eq!(positive_inf, same_positive_inf);
    assert_ne!(positive_inf, negative_inf);
}

#[test]
fn inverse_scalar_helpers_accept_abort_signal() {
    let signal = abort_signal();

    assert_eq!(asin_with_abort(zero(), &signal).unwrap(), zero());
}
