//! Cheap numeric filters.

use crate::predicate::{Sign, SignKnowledge};

/// Conservative-ish filter for a determinant-like floating value.
///
/// The caller supplies the determinant and a scale estimate. If the determinant
/// is comfortably outside the roundoff envelope, the sign is returned.
pub fn det_sign_filter(det: f64, scale: f64, epsilon_multiplier: f64) -> SignKnowledge {
    // This is the shared front door for predicate f64 shortcuts. It is deliberately
    // allocation-free and only certifies signs outside the error envelope; everything else
    // must continue to structural, exact, or robust fallback paths.
    if !det.is_finite() || !scale.is_finite() {
        crate::trace_dispatch!("predicated", "det_sign_filter", "non-finite");
        return SignKnowledge::Unknown;
    }

    if det == 0.0 {
        crate::trace_dispatch!("predicated", "det_sign_filter", "exact-float-zero");
        return SignKnowledge::Unknown;
    }

    let threshold = f64::EPSILON * epsilon_multiplier * scale.max(1.0);
    if det.abs() > threshold {
        crate::trace_dispatch!("predicated", "det_sign_filter", "outside-error-envelope");
        SignKnowledge::filtered(if det > 0.0 {
            Sign::Positive
        } else {
            Sign::Negative
        })
    } else {
        crate::trace_dispatch!("predicated", "det_sign_filter", "inside-error-envelope");
        SignKnowledge::Unknown
    }
}
