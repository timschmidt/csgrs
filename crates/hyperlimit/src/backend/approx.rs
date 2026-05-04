//! Approximate primitive backend.
//!
//! The primitive `f32` and `f64` implementations live in `scalar`. This module
//! exists as the named extension point for approximate-only geometry pipelines.

use crate::backend::BackendCapabilities;

/// Capabilities of primitive floating point predicates.
pub const CAPABILITIES: BackendCapabilities = BackendCapabilities {
    structural_signs: true,
    exact_zero: true,
    magnitude_bounds: true,
    exact_arithmetic: false,
    adaptive_refinement: false,
    robust_fallback: false,
};
