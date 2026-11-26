//! Shared exact symbolic fixtures for predicate policy and normal-form regressions.

use hyperreal::Real;

/// Exactly 2^-3000 as a retained symbolic geometric expression.
///
/// The contact and domain terms cancel algebraically: each reduces to
/// `1/2 - sqrt(2)/8`. Native scalar recognition and policy replay may both
/// prove the remaining positive power of two; consumers must not require
/// recognition to fail at an earlier layer. Division and square-root unwraps
/// use positive constants and the nonzero constant pi.
pub(crate) fn exact_normal_positive() -> Real {
    let root_two = Real::from(2).sqrt().unwrap();
    let root_two_over_pi = (root_two.clone() / Real::pi()).unwrap();
    let half = (Real::from(1) / Real::from(2)).unwrap();
    let shared_offset = root_two.clone() * Real::from(3) + half;
    let contact = (((root_two.clone() * Real::from(4) - shared_offset.clone()) * Real::pi())
        * root_two_over_pi.clone()
        / Real::from(4))
    .unwrap();
    let domain = (((root_two * Real::from(2) - shared_offset) * Real::pi()) * root_two_over_pi
        / Real::from(4))
    .unwrap()
        + Real::one();
    contact - domain + Real::from(2).powi_i64(-3000).unwrap()
}

/// An exact angle-addition zero outside the admitted normal-form frontier.
/// Terminal-policy tests must consume an actual unresolved decision.
pub(crate) fn terminally_unresolved_zero() -> Real {
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}
