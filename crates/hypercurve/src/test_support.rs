//! Shared unit-test helpers.

use hyperreal::Real;

/// A mathematical zero whose identity is intentionally outside bounded exact
/// normalization, so STRICT remains undecided and APPROXIMATE_512 may consume
/// only its terminal equality predicate.
pub(crate) fn terminally_unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let original = &sine * &sine + &cosine * &cosine - Real::one();
    // Preserve the original identity as a newly certified proof case.
    assert_eq!(
        original.exact_rational_normal_form(),
        Some(hyperreal::Rational::zero())
    );
    // A separate angle-addition identity retains the terminal-policy control.
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}

/// The decided value of a test classification. Uncertainty fails the test at
/// the caller, naming its reason.
#[track_caller]
pub(crate) fn decided<T>(classification: crate::Classification<T>) -> T {
    match classification {
        crate::Classification::Decided(value) => value,
        crate::Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

/// [`decided`] of a successful result; an error fails the test at the caller.
#[track_caller]
pub(crate) fn decided_ok<T, E: std::fmt::Debug>(result: Result<crate::Classification<T>, E>) -> T {
    match result {
        Ok(classification) => decided(classification),
        Err(error) => panic!("unexpected error: {error:?}"),
    }
}
