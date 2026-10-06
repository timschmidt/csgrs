//! Crate-internal access to Hyperlimit's centralized scalar decision policy.

use std::cmp::Ordering;

use hyperlimit::{PredicateOutcome, PredicatePolicy, Sign};
use hyperreal::{Real, RealSign};

const POLICY: PredicatePolicy = PredicatePolicy::STRICT;

/// Three-valued conjunction with decisive-false short-circuiting.
macro_rules! decide_all {
    ($($decision:expr),+ $(,)?) => {{
        let mut saw_unknown = false;
        'decision: {
            $(
                match $decision {
                    Some(true) => {}
                    Some(false) => break 'decision Some(false),
                    None => saw_unknown = true,
                }
            )+
            break 'decision if saw_unknown { None } else { Some(true) };
        }
    }};
}

/// Three-valued disjunction with decisive-true short-circuiting.
macro_rules! decide_any {
    ($($decision:expr),+ $(,)?) => {{
        let mut saw_unknown = false;
        'decision: {
            $(
                match $decision {
                    Some(true) => break 'decision Some(true),
                    Some(false) => {}
                    None => saw_unknown = true,
                }
            )+
            break 'decision if saw_unknown { None } else { Some(false) };
        }
    }};
}

pub(crate) use decide_all;
pub(crate) use decide_any;

/// Classifies one exact scalar through Hyperlimit's active certainty cascade.
#[inline]
pub(crate) fn sign(value: &Real) -> Option<RealSign> {
    match hyperlimit::classify_real_sign(value, POLICY) {
        PredicateOutcome::Decided {
            value: Sign::Negative,
            ..
        } => Some(RealSign::Negative),
        PredicateOutcome::Decided {
            value: Sign::Zero, ..
        } => Some(RealSign::Zero),
        PredicateOutcome::Decided {
            value: Sign::Positive,
            ..
        } => Some(RealSign::Positive),
        PredicateOutcome::Unknown { .. } => None,
    }
}

/// Compares two exact scalars through the same centralized cascade.
#[inline]
pub(crate) fn compare(left: &Real, right: &Real) -> Option<Ordering> {
    hyperlimit::compare_reals(left, right, POLICY).value()
}

/// Returns whether equality is certified by the centralized cascade.
#[inline]
pub(crate) fn equal(left: &Real, right: &Real) -> bool {
    compare(left, right).is_some_and(Ordering::is_eq)
}

#[cfg(test)]
mod tests {
    use hyperreal::Rational;

    use super::*;

    #[test]
    fn centralized_policy_resolves_beyond_the_removed_64_bit_cutoff() {
        let truncated_pi: Rational = concat!(
            "3.14159265358979323846264338327950288419716939937510",
            "58209749445923078164062862089986280348253421170679"
        )
        .parse()
        .unwrap();
        let residual = Real::pi() - Real::new(truncated_pi);

        assert_eq!(residual.refine_sign_until(-64), None);
        assert_eq!(sign(&residual), Some(RealSign::Positive));
    }

    #[test]
    fn three_valued_combinators_preserve_decisive_truth_values() {
        assert_eq!(decide_all!(None, Some(true), Some(false)), Some(false));
        assert_eq!(decide_all!(Some(true), None), None);
        assert_eq!(decide_any!(None, Some(false), Some(true)), Some(true));
        assert_eq!(decide_any!(Some(false), None), None);
    }
}
