//! Lattice-specific zero guards and cancellation attachment.
//!
//! Scalar arithmetic and structural queries use the native `Real` API.

use crate::{AbortSignal, BlasResult, CheckedBlasResult, Problem, Real, ZeroKnowledge};

#[inline(always)]
pub(crate) fn with_abort(mut value: Real, signal: &AbortSignal) -> Real {
    crate::trace_dispatch!("hyperlattice", "abort", "attach-owned-real");
    value.abort(signal.clone());
    value
}

#[inline(always)]
pub(crate) fn clone_with_abort(value: &Real, signal: &AbortSignal) -> Real {
    crate::trace_dispatch!("hyperlattice", "abort", "clone-and-attach");
    with_abort(value.clone(), signal)
}

#[inline(always)]
pub(crate) fn reject_definite_zero(value: &Real) -> BlasResult<()> {
    if value.definitely_zero() {
        crate::trace_dispatch!("hyperlattice", "zero_guard", "definite-zero-rejected");
        Err(Problem::DivideByZero)
    } else {
        crate::trace_dispatch!("hyperlattice", "zero_guard", "not-definitely-zero");
        Ok(())
    }
}

#[inline(always)]
pub(crate) fn require_known_nonzero(value: &Real) -> CheckedBlasResult<()> {
    match value.zero_status() {
        ZeroKnowledge::Zero => {
            crate::trace_dispatch!("hyperlattice", "zero_guard", "checked-zero-rejected");
            Err(Problem::DivideByZero)
        }
        ZeroKnowledge::NonZero => {
            crate::trace_dispatch!("hyperlattice", "zero_guard", "checked-nonzero");
            Ok(())
        }
        ZeroKnowledge::Unknown => {
            crate::trace_dispatch!("hyperlattice", "zero_guard", "checked-unknown-rejected");
            Err(Problem::UnknownZero)
        }
    }
}

/// Returns the multiplicative inverse after rejecting zero and unknown-zero values.
///
/// Unlike `Real::inverse`, this requires structural nonzero knowledge and does
/// not refine an unknown divisor.
pub fn reciprocal_checked(value: Real) -> CheckedBlasResult<Real> {
    require_known_nonzero(&value)?;
    value.inverse()
}

/// Returns the checked multiplicative inverse without consuming `value`.
///
/// Like [`reciprocal_checked`], this rejects unknown-zero values without refinement.
pub fn reciprocal_ref_checked(value: &Real) -> CheckedBlasResult<Real> {
    require_known_nonzero(value)?;
    value.inverse_ref()
}
