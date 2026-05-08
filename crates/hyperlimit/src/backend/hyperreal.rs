//! Adapter for `hyperreal` structural scalar facts.

use crate::backend::BackendCapabilities;
use crate::predicate::{Sign, SignKnowledge};
use crate::scalar::{MagnitudeBounds, PredicateScalar, ScalarFacts, StructuralScalar};

/// Capabilities exposed by `hyperreal` 0.10's structural APIs.
pub const CAPABILITIES: BackendCapabilities = BackendCapabilities {
    structural_signs: true,
    exact_zero: true,
    magnitude_bounds: true,
    exact_arithmetic: true,
    adaptive_refinement: true,
    robust_fallback: false,
};

impl StructuralScalar for hyperreal::Real {
    fn scalar_facts(&self) -> ScalarFacts {
        crate::trace_dispatch!("predicated_hyperreal_adapter", "structural", "scalar-facts");
        scalar_facts_from_hyperreal(self.structural_facts())
    }

    fn known_sign(&self) -> SignKnowledge {
        crate::trace_dispatch!("predicated_hyperreal_adapter", "structural", "known-sign");
        scalar_facts_from_hyperreal(self.structural_facts()).sign_knowledge()
    }

    fn refine_sign_until(&self, min_precision: i32) -> SignKnowledge {
        match self.refine_sign_until(min_precision) {
            Some(sign) => {
                crate::trace_dispatch!("predicated_hyperreal_adapter", "structural", "refine-hit");
                SignKnowledge::exact(map_sign(sign))
            }
            None => {
                crate::trace_dispatch!(
                    "predicated_hyperreal_adapter",
                    "structural",
                    "refine-unknown"
                );
                SignKnowledge::Unknown
            }
        }
    }
}

impl PredicateScalar for hyperreal::Real {
    #[inline]
    fn to_f64(&self) -> Option<f64> {
        crate::trace_dispatch!(
            "predicated_hyperreal_adapter",
            "conversion",
            "to-f64-approx"
        );
        self.to_f64_approx()
    }

    #[inline(always)]
    fn prefer_f64_filter_before_arithmetic() -> bool {
        // Hyperreal expression construction is expensive enough that a proven f64 filter is
        // worth trying before exact predicate arithmetic.
        crate::trace_dispatch!(
            "predicated_hyperreal_adapter",
            "policy",
            "prefer-f64-prefilter"
        );
        true
    }
}

fn map_sign(sign: hyperreal::RealSign) -> Sign {
    match sign {
        hyperreal::RealSign::Negative => Sign::Negative,
        hyperreal::RealSign::Zero => Sign::Zero,
        hyperreal::RealSign::Positive => Sign::Positive,
    }
}

fn map_magnitude(magnitude: hyperreal::MagnitudeBits) -> Option<MagnitudeBounds> {
    magnitude_bits_to_bounds(magnitude.msd, magnitude.exact_msd)
}

fn scalar_facts_from_hyperreal(facts: hyperreal::RealStructuralFacts) -> ScalarFacts {
    // Preserve hyperreal's cheap sign/zero/magnitude certificates exactly as
    // predicate facts. This lets filters decide `pi - 3` style expressions
    // without forcing a full exact predicate fallback.
    ScalarFacts {
        sign: facts.sign.map(map_sign),
        exact_zero: Some(matches!(facts.zero, hyperreal::ZeroKnowledge::Zero)),
        provably_nonzero: match facts.zero {
            hyperreal::ZeroKnowledge::Zero => Some(false),
            hyperreal::ZeroKnowledge::NonZero => Some(true),
            hyperreal::ZeroKnowledge::Unknown => None,
        },
        exact: Some(facts.exact_rational),
        rational_only: Some(facts.exact_rational),
        magnitude: facts.magnitude.and_then(map_magnitude),
    }
}

pub(crate) fn magnitude_bits_to_bounds(msd: i32, exact_msd: bool) -> Option<MagnitudeBounds> {
    // Convert binary magnitude certificates into coarse f64 absolute bounds.
    // These are only used for dominance filters, never for reconstructing values.
    let upper_exp = msd.checked_add(1)?;
    let abs_upper = pow2(upper_exp)?;
    let abs_lower = if exact_msd { pow2(msd)? } else { 0.0 };
    Some(MagnitudeBounds {
        abs_lower,
        abs_upper,
    })
}

fn pow2(exp: i32) -> Option<f64> {
    let value = 2.0_f64.powi(exp);
    value.is_finite().then_some(value)
}
