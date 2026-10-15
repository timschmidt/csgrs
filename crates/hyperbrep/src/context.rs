//! Authoritative operation scopes shared by BREP entry points.
//!
//! Hyperlimit receives an explicit strict policy at each predicate call.
//! Hypercurve's principal APIs additionally need a strict scope, because a
//! caller may have an active provisional or display-preview operation.

/// Suspends an enclosing curve approximation/preview policy for one BREP
/// operation. Hypercurve restores the caller's scope on return or unwinding.
///
/// This controls predicate authority only. Explicit work budgets, cancellation
/// and operation receipts are separate parts of the operation-context work.
#[inline]
pub(crate) fn strict<T>(evaluate: impl FnOnce() -> T) -> T {
    hypercurve::evaluate_under(crate::STRICT_PREDICATES, evaluate)
        .certified()
        .expect("Hypercurve's STRICT scope always returns a certified value")
}
