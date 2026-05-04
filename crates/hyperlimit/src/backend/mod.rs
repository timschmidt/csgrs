//! Backend integration points.

pub mod approx;

#[cfg(feature = "geogram")]
pub mod geogram;

#[cfg(feature = "hyperreal")]
pub mod hyperreal;

#[cfg(feature = "interval")]
pub mod interval;

#[cfg(feature = "realistic-blas")]
pub mod realistic_blas;

#[cfg(feature = "robust")]
pub mod robust;

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
