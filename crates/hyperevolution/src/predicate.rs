//! Crate-internal access to Hyperlimit's centralized scalar decision policy.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, Sign};
use hyperreal::Real;

const POLICY: PredicatePolicy = PredicatePolicy::STRICT;

#[inline]
pub(crate) fn compare(left: &Real, right: &Real) -> Option<Ordering> {
    hyperlimit::compare_reals(left, right, POLICY).value()
}

#[inline]
pub(crate) fn equal(left: &Real, right: &Real) -> Option<bool> {
    Some(compare(left, right)?.is_eq())
}

#[inline]
pub(crate) fn leq(left: &Real, right: &Real) -> Option<bool> {
    Some(!compare(left, right)?.is_gt())
}

#[inline]
pub(crate) fn positive(value: &Real) -> Option<bool> {
    match hyperlimit::classify_real_sign(value, POLICY).value()? {
        Sign::Positive => Some(true),
        Sign::Negative | Sign::Zero => Some(false),
    }
}

#[cfg(test)]
mod tests {
    use hyperreal::Rational;

    use super::*;

    #[test]
    fn centralized_policy_resolves_beyond_a_local_64_bit_cutoff() {
        let truncated_pi: Rational = concat!(
            "3.14159265358979323846264338327950288419716939937510",
            "58209749445923078164062862089986280348253421170679"
        )
        .parse()
        .unwrap();
        let residual = Real::pi() - Real::new(truncated_pi);

        assert_eq!(residual.refine_sign_until(-64), None);
        assert_eq!(positive(&residual), Some(true));
    }
}
