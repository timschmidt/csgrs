//! Uncertainty controls for consumers of newly admitted scalar proofs.

use hyperreal::Real;

/// Retain the previous fixture as a certified zero, then exercise terminal
/// policy with the independent sine addition identity. The exact mathematical
/// value remains zero; the latter identity is outside the admitted normal form.
pub(crate) fn terminal_zero(proved_zero: Real) -> Real {
    assert_eq!(
        proved_zero.exact_rational_normal_form(),
        Some(hyperreal::Rational::zero())
    );
    let zero = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert!(matches!(
        hyperlimit::compare_reals(&zero, &Real::zero(), hyperlimit::PredicatePolicy::STRICT),
        hyperlimit::PredicateOutcome::Unknown { .. }
    ));
    zero
}
