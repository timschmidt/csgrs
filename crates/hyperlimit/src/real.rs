//! Real-specific structural facts used by geometry predicates.

use hyperreal::{CertifiedRealSign, Real, RealSign, RealStructuralFacts, ZeroKnowledge};

use crate::predicate::{Sign, SignKnowledge};

/// Real-specific helpers for predicate code.
///
/// The extension trait keeps predicate-specific sign conversion and bounded
/// refinement readable without reintroducing a generic numeric abstraction.
/// Scalar structural and zero facts use `Real`'s inherent APIs directly.
pub trait RealPredicateExt {
    /// Return known sign information without forcing full predicate evaluation.
    fn known_sign(&self) -> SignKnowledge;

    /// Refine the sign through `hyperreal`'s exact/computable machinery.
    fn refine_sign_knowledge_until(&self, min_precision: i32) -> SignKnowledge;
}

impl RealPredicateExt for Real {
    #[inline(always)]
    fn known_sign(&self) -> SignKnowledge {
        crate::trace_dispatch!("hyperlimit", "real", "known-sign");
        sign_knowledge_from_real_facts(self.structural_facts())
    }

    #[inline(always)]
    fn refine_sign_knowledge_until(&self, min_precision: i32) -> SignKnowledge {
        match self.certified_sign_until(min_precision) {
            CertifiedRealSign::Known { sign, .. } => {
                crate::trace_dispatch!("hyperlimit", "real", "refine-hit");
                SignKnowledge::exact(map_real_sign(sign))
            }
            CertifiedRealSign::Unknown { .. } => {
                crate::trace_dispatch!("hyperlimit", "real", "refine-unknown");
                SignKnowledge::Unknown
            }
        }
    }
}

/// Map a `hyperreal` sign into the predicate sign domain.
#[inline(always)]
pub fn map_real_sign(sign: RealSign) -> Sign {
    match sign {
        RealSign::Negative => Sign::Negative,
        RealSign::Zero => Sign::Zero,
        RealSign::Positive => Sign::Positive,
    }
}

/// Convert structural Real facts into predicate sign knowledge.
#[inline(always)]
pub fn sign_knowledge_from_real_facts(facts: RealStructuralFacts) -> SignKnowledge {
    if let Some(sign) = facts.sign {
        SignKnowledge::exact(map_real_sign(sign))
    } else if matches!(facts.zero, ZeroKnowledge::Zero) {
        SignKnowledge::exact(Sign::Zero)
    } else if matches!(facts.zero, ZeroKnowledge::NonZero) {
        SignKnowledge::NonZero
    } else {
        SignKnowledge::Unknown
    }
}

/// Add two borrowed Real values.
#[inline(always)]
pub(crate) fn add_ref(left: &Real, right: &Real) -> Real {
    crate::trace_dispatch!("hyperlimit", "real_op", "add-ref");
    left + right
}

/// Subtract two borrowed Real values.
#[inline(always)]
pub(crate) fn sub_ref(left: &Real, right: &Real) -> Real {
    crate::trace_dispatch!("hyperlimit", "real_op", "sub-ref");
    left - right
}

/// Multiply two borrowed Real values.
#[inline(always)]
pub(crate) fn mul_ref(left: &Real, right: &Real) -> Real {
    crate::trace_dispatch!("hyperlimit", "real_op", "mul-ref");
    left * right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn real_zero_knowledge_maps_to_predicate_signs() {
        assert_eq!(Real::from(0).known_sign(), SignKnowledge::exact(Sign::Zero));
        assert_eq!(
            Real::from(3).known_sign(),
            SignKnowledge::exact(Sign::Positive)
        );
        assert_eq!(
            Real::from(-3).known_sign(),
            SignKnowledge::exact(Sign::Negative)
        );

        let without_sign = |zero| RealStructuralFacts {
            sign: None,
            zero,
            exact_rational: false,
            magnitude: None,
        };
        assert_eq!(
            sign_knowledge_from_real_facts(without_sign(ZeroKnowledge::Zero)),
            SignKnowledge::exact(Sign::Zero)
        );
        assert_eq!(
            sign_knowledge_from_real_facts(without_sign(ZeroKnowledge::NonZero)),
            SignKnowledge::NonZero
        );
    }
}
