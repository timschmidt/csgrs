//! Public status vocabulary for retained exact-geometry carriers.
//!
//! Retained carriers are exact construction evidence that may or may not be
//! admissible as topology in the current kernel. The status values here keep
//! that distinction visible to callers instead of collapsing every non-native
//! case into an approximation or a boolean failure. This is the object/predicate
//! separation advocated by exact-computation discipline: preserve exact objects, then
//! branch only on certified predicates and explicitly named capability
//! boundaries.

/// Topology-readiness status for a retained curve or span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetainedTopologyStatus {
    /// The retained object has exact native topology in this kernel.
    NativeExact,
    /// The object has a certified approximation but not native exact topology.
    CertifiedApproximation,
    /// The object is suitable only for display/export evidence.
    DisplayOrExport,
    /// The object was imported through a lossy source tolerance or conversion.
    ImportedLossy,
    /// The exact object is retained, but this kernel has no topology model for it.
    Unsupported,
    /// The topology status depends on predicates that were not decided.
    Unresolved,
}

impl RetainedTopologyStatus {
    /// Returns true only for exact native topology.
    pub const fn is_native_exact(self) -> bool {
        matches!(self, Self::NativeExact)
    }

    /// Returns true for exact retained evidence that must not be consumed as
    /// native topology by the current kernel.
    pub const fn is_retained_evidence(self) -> bool {
        matches!(self, Self::Unsupported | Self::Unresolved)
    }

    /// Returns true when the object crossed a lossy import boundary.
    pub const fn is_imported_lossy(self) -> bool {
        matches!(self, Self::ImportedLossy)
    }
}
