//! Helpers shared by the Hypercurve fuzz targets.

use hypercurve::CurveContext;

/// Runs a principal exact operation directly under STRICT, or inside
/// `hypercurve::provisional` for any other policy.
#[allow(dead_code)]
pub fn under<T>(policy: &CurveContext, operation: impl FnOnce() -> T) -> T {
    if *policy == CurveContext::STRICT {
        operation()
    } else {
        hypercurve::provisional(operation).into_unverified()
    }
}

/// Runs a principal exact operation under `policy`, keeping its value only
/// when it succeeded and every decision behind it was certified.
#[allow(dead_code)]
pub fn certified_under<T, E>(
    policy: &CurveContext,
    operation: impl FnOnce() -> Result<T, E>,
) -> Option<T> {
    if *policy == CurveContext::STRICT {
        operation().ok()
    } else {
        hypercurve::provisional(operation).certified()?.ok()
    }
}

/// [`under`] for operations whose result never carried certainty.
#[allow(dead_code)]
pub fn under_value<T>(policy: &CurveContext, operation: impl FnOnce() -> T) -> T {
    under(policy, operation)
}

/// Runs an exact principal query under `policy`, keeping an undecided
/// predicate as `Classification::Uncertain` and any other error as `Err`.
#[allow(dead_code)]
pub fn under_classified_result<T>(
    policy: &CurveContext,
    operation: impl FnOnce() -> hypercurve::ExactCurveResult<T>,
) -> hypercurve::ExactCurveResult<hypercurve::Classification<T>> {
    match under(policy, operation) {
        Ok(value) => Ok(hypercurve::Classification::Decided(value)),
        Err(hypercurve::ExactCurveError::Blocked(blocker)) => {
            Ok(hypercurve::Classification::Uncertain(blocker.reason()))
        }
        Err(error) => Err(error),
    }
}

/// [`under_classified_result`] for queries that formerly also reported
/// their certainty; the value is unverified outside STRICT.
#[allow(dead_code)]
pub fn under_outcome_classified<T>(
    policy: &CurveContext,
    operation: impl FnOnce() -> hypercurve::ExactCurveResult<T>,
) -> hypercurve::ExactCurveResult<hypercurve::Classification<T>> {
    under_classified_result(policy, operation)
}

/// Runs an exact principal predicate under `policy` as a classification.
///
/// The predicate's inputs are valid by construction, so an invalid-state
/// error is a finding.
#[allow(dead_code)]
pub fn under_classified<T>(
    policy: &CurveContext,
    operation: impl FnOnce() -> hypercurve::ExactCurveResult<T>,
) -> hypercurve::Classification<T> {
    under_classified_result(policy, operation)
        .unwrap_or_else(|error| panic!("exact predicate rejected its input: {error:?}"))
}
