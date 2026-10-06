//! Shared unit-test helpers.

use hyperreal::Real;

/// A mathematical zero whose identity is intentionally outside bounded exact
/// normalization, so STRICT remains undecided and APPROXIMATE_512 may consume
/// only its terminal equality predicate.
pub(crate) fn terminally_unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}
