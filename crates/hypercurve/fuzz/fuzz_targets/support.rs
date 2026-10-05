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
