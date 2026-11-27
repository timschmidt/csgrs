//! Structural zero guards and cooperative signal attachment for lattice operations.
//!
//! These helpers keep Hyperreal as the arithmetic/fact authority. A definite-zero
//! guard admits unknown values, whereas the checked guard rejects them. Neither
//! helper starts a sign-refinement loop. Existing cached proofs may affect the
//! facts observed. Abort attachment is not a preemptive deadline or rollback.

use crate::{AbortSignal, BlasResult, CheckedBlasResult, Problem, Real, ZeroKnowledge};

/// Attach a cloned Arc cancellation flag to the owned Real and return it.
///
/// Real::abort clears its primitive approximation cache and attaches the flag to a
/// computable payload, materializing that payload if needed. This helper does not
/// poll the flag or return a cancellation error; subsequent operations decide which
/// paths consult it. No independent cache isolation or total-work bound is supplied.
#[inline(always)]
pub(crate) fn with_abort(mut value: Real, signal: &AbortSignal) -> Real {
    crate::trace_dispatch!("hyperlattice", "abort", "attach-owned-real");
    value.abort(signal.clone());
    value
}

/// Clone the borrowed Real, then attach the shared signal through with_abort.
///
/// Records clone-and-attach before the owned attachment trace. Cloning does not imply
/// independent underlying expression/proof caches; the AtomicBool remains shared.
#[inline(always)]
pub(crate) fn clone_with_abort(value: &Real, signal: &AbortSignal) -> Real {
    crate::trace_dispatch!("hyperlattice", "abort", "clone-and-attach");
    with_abort(value.clone(), signal)
}

/// Reject Real::definitely_zero with DivideByZero, accepting every other result.
///
/// At this revision that query checks the scalar's zero rational scale. Ok does not
/// prove nonzero and can admit an opaque mathematical zero. This guard does not
/// refine, inspect cancellation, or decide the full scalar domain.
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

/// Accept ZeroKnowledge::NonZero; map Zero to DivideByZero and Unknown to UnknownZero.
///
/// Uses zero_status's structural/cached facts without initiating refinement here.
/// Unknown is not mathematical zero and does not become a successful domain proof.
/// The helper does not separately poll a cancellation flag.
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

/// Require known nonzero status before consuming the Real in its native inverse operation.
///
/// The guard rejects Zero and Unknown without refinement, returning DivideByZero or
/// UnknownZero respectively. Accepted values proceed through Real::inverse and any
/// scalar construction error propagates. This returns a scalar result, not a
/// predicate-policy or inherited-certainty receipt.
pub fn reciprocal_checked(value: Real) -> CheckedBlasResult<Real> {
    require_known_nonzero(&value)?;
    value.inverse()
}

/// Borrowed counterpart of reciprocal_checked using Real::inverse_ref after the same guard.
///
/// Rejects zero/unknown status before inverse construction and propagates scalar
/// errors. Does not consume the input or start refinement to resolve the guard;
/// shared scalar proof/cache state need not be independent of the result.
pub fn reciprocal_ref_checked(value: &Real) -> CheckedBlasResult<Real> {
    require_known_nonzero(value)?;
    value.inverse_ref()
}
