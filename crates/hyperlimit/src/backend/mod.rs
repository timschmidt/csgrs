//! Backend integration points.

pub mod approx;

#[cfg(feature = "geogram")]
pub mod geogram;

#[cfg(feature = "hyperreal")]
pub mod hyperreal;

#[cfg(feature = "interval")]
pub mod interval;

#[cfg(feature = "hyperlattice")]
pub mod hyperlattice;

#[cfg(feature = "robust")]
pub mod robust;

/// Capabilities a backend can advertise to predicate pipelines.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BackendCapabilities {
    /// Backend can expose exact or conservative signs without full predicates.
    pub structural_signs: bool,
    /// Backend can prove exact zero/nonzero status structurally.
    pub exact_zero: bool,
    /// Backend can expose conservative magnitude bounds for scalar values.
    pub magnitude_bounds: bool,
    /// Backend supports exact arithmetic fallback for predicate terms.
    pub exact_arithmetic: bool,
    /// Backend can refine scalar signs adaptively.
    pub adaptive_refinement: bool,
    /// Backend supplies a robust floating-point predicate fallback.
    pub robust_fallback: bool,
}
