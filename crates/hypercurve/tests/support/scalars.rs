#![allow(dead_code)]

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
    // Angle addition keeps a separate exact zero outside the current proof frontier.
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}

pub(crate) fn terminally_equal_pair(value: Real) -> (Real, Real) {
    let other = value.clone() + terminally_unresolved_zero();
    (value, other)
}
