//! Real-specific structural facts used by geometry predicates.

use hyperreal::{CertifiedRealSign, Real, RealSign, RealStructuralFacts, ZeroKnowledge};

use crate::predicate::{Sign, SignKnowledge};

/// Structural facts attached to the primary predicate Real.
pub type RealFacts = RealStructuralFacts;

/// Cheap zero/nonzero knowledge exposed by [`Real`].
pub type RealZeroKnowledge = ZeroKnowledge;

/// Real-specific helpers for predicate code.
///
/// The extension trait keeps call sites readable without reintroducing a
/// generic numeric abstraction. All information comes directly from
/// `hyperreal::Real::structural_facts`; no primitive-float filter is consulted.
pub trait RealPredicateExt {
    /// Return cheap exact structural facts for this value.
    fn real_facts(&self) -> RealFacts;

    /// Return known sign information without forcing full predicate evaluation.
    fn known_sign(&self) -> SignKnowledge;

    /// Return cheap zero/nonzero knowledge for this value.
    fn zero_knowledge(&self) -> RealZeroKnowledge;

    /// Refine the sign through `hyperreal`'s exact/computable machinery.
    fn refine_sign_knowledge_until(&self, min_precision: i32) -> SignKnowledge;
}

impl RealPredicateExt for Real {
    #[inline(always)]
    fn real_facts(&self) -> RealFacts {
        crate::trace_dispatch!("hyperlimit", "real", "structural-facts");
        self.structural_facts()
    }

    #[inline(always)]
    fn known_sign(&self) -> SignKnowledge {
        crate::trace_dispatch!("hyperlimit", "real", "known-sign");
        sign_knowledge_from_real_facts(self.structural_facts())
    }

    #[inline(always)]
    fn zero_knowledge(&self) -> RealZeroKnowledge {
        crate::trace_dispatch!("hyperlimit", "real", "zero-knowledge");
        self.structural_facts().zero
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
pub fn sign_knowledge_from_real_facts(facts: RealFacts) -> SignKnowledge {
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

/// Decide a Real sign from structural facts only.
#[inline(always)]
pub fn exact_sign_from_real_facts(facts: RealFacts) -> Option<Sign> {
    match sign_knowledge_from_real_facts(facts) {
        SignKnowledge::Known { sign, .. } => Some(sign),
        SignKnowledge::NonZero | SignKnowledge::Unknown => None,
    }
}

/// Return whether a Real is known to be exactly zero.
#[inline(always)]
pub fn real_is_zero(facts: RealFacts) -> Option<bool> {
    match facts.zero {
        ZeroKnowledge::Zero => Some(true),
        ZeroKnowledge::NonZero => Some(false),
        ZeroKnowledge::Unknown => None,
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
    }
}
