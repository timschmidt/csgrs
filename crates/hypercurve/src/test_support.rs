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
