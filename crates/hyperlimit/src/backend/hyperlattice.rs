//! Adapter for `hyperlattice::Scalar` structural scalar facts.

use crate::backend::BackendCapabilities;
#[cfg(feature = "hyperreal")]
use crate::backend::hyperreal::magnitude_bits_to_bounds;
use crate::predicate::{Sign, SignKnowledge};
use crate::scalar::{MagnitudeBounds, PredicateScalar, ScalarFacts, StructuralScalar};

/// Capabilities forwarded through `hyperlattice::Scalar`.
pub const CAPABILITIES: BackendCapabilities = BackendCapabilities {
    structural_signs: true,
    exact_zero: true,
    magnitude_bounds: true,
    exact_arithmetic: true,
    adaptive_refinement: true,
    robust_fallback: false,
};

impl<B: ::hyperlattice::Backend> StructuralScalar for ::hyperlattice::Scalar<B> {
    #[inline(always)]
    fn scalar_facts(&self) -> ScalarFacts {
        crate::trace_dispatch!(
            "hyperlimit_hyperlattice_adapter",
            "structural",
            "scalar-facts"
        );
        scalar_facts_from_hyperlattice(self.structural_facts())
    }

    #[inline(always)]
    fn known_sign(&self) -> SignKnowledge {
        crate::trace_dispatch!(
            "hyperlimit_hyperlattice_adapter",
            "structural",
            "known-sign"
        );
        scalar_facts_from_hyperlattice(self.structural_facts()).sign_knowledge()
    }

    #[inline(always)]
    fn refine_sign_until(&self, min_precision: i32) -> SignKnowledge {
        match self.refine_sign_until(min_precision) {
            Some(sign) => {
                crate::trace_dispatch!(
                    "hyperlimit_hyperlattice_adapter",
                    "structural",
                    "refine-hit"
                );
                SignKnowledge::exact(map_sign(sign))
            }
            None => {
                crate::trace_dispatch!(
                    "hyperlimit_hyperlattice_adapter",
                    "structural",
                    "refine-unknown"
                );
                SignKnowledge::Unknown
            }
        }
    }
}

impl<B: ::hyperlattice::Backend> PredicateScalar for ::hyperlattice::Scalar<B> {
    #[inline(always)]
    fn to_f64(&self) -> Option<f64> {
        crate::trace_dispatch!(
            "hyperlimit_hyperlattice_adapter",
            "conversion",
            "to-f64-approx"
        );
        self.to_f64_approx()
    }

    #[inline(always)]
    fn prefer_f64_filter_before_arithmetic() -> bool {
        // Hyperlattice scalars may wrap exact symbolic values; conservative f64 filtering
        // avoids building those trees when the determinant sign is already certified.
        crate::trace_dispatch!(
            "hyperlimit_hyperlattice_adapter",
            "policy",
            "prefer-f64-prefilter"
        );
        true
    }

    #[inline(always)]
    fn prefer_plane_f64_filter_before_arithmetic() -> bool {
        crate::trace_dispatch!(
            "hyperlimit_hyperlattice_adapter",
            "policy",
            "prefer-plane-f64-prefilter"
        );
        true
    }

    #[inline(always)]
    fn prefer_orientation_f64_filter_before_arithmetic() -> bool {
        crate::trace_dispatch!(
            "hyperlimit_hyperlattice_adapter",
            "policy",
            "prefer-orientation-f64-prefilter"
        );
        true
    }

    #[inline(always)]
    fn prefer_insphere_f64_filter_before_arithmetic() -> bool {
        crate::trace_dispatch!(
            "hyperlimit_hyperlattice_adapter",
            "policy",
            "prefer-insphere-f64-prefilter"
        );
        true
    }
}

#[inline]
fn map_sign(sign: ::hyperlattice::ScalarSign) -> Sign {
    match sign {
        ::hyperlattice::ScalarSign::Negative => Sign::Negative,
        ::hyperlattice::ScalarSign::Zero => Sign::Zero,
        ::hyperlattice::ScalarSign::Positive => Sign::Positive,
    }
}

#[inline]
fn map_magnitude(magnitude: ::hyperlattice::ScalarMagnitudeBits) -> Option<MagnitudeBounds> {
    magnitude_bits_to_bounds_local(magnitude.msd, magnitude.exact_msd)
}

fn scalar_facts_from_hyperlattice(facts: ::hyperlattice::ScalarFacts) -> ScalarFacts {
    // Forward hyperlattice facts without asking the wrapped scalar for an
    // exact value. This keeps predicate filters on the borrowed structural path.
    ScalarFacts {
        sign: facts.sign.map(map_sign),
        exact_zero: Some(matches!(facts.zero, ::hyperlattice::ZeroStatus::Zero)),
        provably_nonzero: match facts.zero {
            ::hyperlattice::ZeroStatus::Zero => Some(false),
            ::hyperlattice::ZeroStatus::NonZero => Some(true),
            ::hyperlattice::ZeroStatus::Unknown => None,
        },
        exact: Some(facts.exact_rational),
        rational_only: Some(facts.exact_rational),
        magnitude: facts.magnitude.and_then(map_magnitude),
    }
}

#[cfg(feature = "hyperreal")]
fn magnitude_bits_to_bounds_local(msd: i32, exact_msd: bool) -> Option<MagnitudeBounds> {
    magnitude_bits_to_bounds(msd, exact_msd)
}

#[cfg(not(feature = "hyperreal"))]
fn magnitude_bits_to_bounds_local(msd: i32, exact_msd: bool) -> Option<MagnitudeBounds> {
    let upper_exp = msd.checked_add(1)?;
    let abs_upper = pow2(upper_exp)?;
    let abs_lower = if exact_msd { pow2(msd)? } else { 0.0 };
    Some(MagnitudeBounds {
        abs_lower,
        abs_upper,
    })
}

#[cfg(not(feature = "hyperreal"))]
fn pow2(exp: i32) -> Option<f64> {
    let value = 2.0_f64.powi(exp);
    value.is_finite().then_some(value)
}
