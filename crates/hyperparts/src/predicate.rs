//! Crate-internal access to Hyperlimit's centralized scalar decision policy.

use std::cmp::Ordering;

use hyperlimit::PredicatePolicy;
use hyperreal::Real;

const POLICY: PredicatePolicy = PredicatePolicy::STRICT;

#[inline]
pub(crate) fn compare(left: &Real, right: &Real) -> Option<Ordering> {
    hyperlimit::compare_reals(left, right, POLICY).value()
}

#[inline]
fn leq(left: &Real, right: &Real) -> Option<bool> {
    Some(!compare(left, right)?.is_gt())
}

/// Classifies overlap of two closed intervals with truth-dominant uncertainty.
#[inline]
pub(crate) fn closed_intervals_overlap(
    left_min: &Real,
    left_max: &Real,
    right_min: &Real,
    right_max: &Real,
) -> Option<bool> {
    let left_before_right_end = leq(left_min, right_max);
    if left_before_right_end == Some(false) {
        return Some(false);
    }
    let right_before_left_end = leq(right_min, left_max);
    if right_before_left_end == Some(false) {
        return Some(false);
    }
    match (left_before_right_end, right_before_left_end) {
        (Some(true), Some(true)) => Some(true),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use hyperreal::{Rational, RealSign};

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
        assert_eq!(compare(&residual, &Real::zero()), Some(Ordering::Greater));
        assert_eq!(residual.refine_sign_until(-512), Some(RealSign::Positive));
    }

    #[test]
    fn certified_disjointness_dominates_an_unknown_opposite_bound() {
        assert_eq!(
            closed_intervals_overlap(
                &Real::from(10),
                &Real::from(11),
                &Real::from(0),
                &Real::from(1),
            ),
            Some(false)
        );
    }
}
