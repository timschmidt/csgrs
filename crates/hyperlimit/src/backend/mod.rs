//! Backend integration points.

pub mod approx;

#[cfg(feature = "hyperreal")]
pub mod hyperreal;

/// Capabilities a backend can advertise to predicate pipelines.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BackendCapabilities {
    pub structural_signs: bool,
    pub exact_zero: bool,
    pub magnitude_bounds: bool,
    pub exact_arithmetic: bool,
    pub adaptive_refinement: bool,
    pub robust_fallback: bool,
}
