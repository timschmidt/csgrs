//! Runtime numeric policy for curve operations.
//!
//! `Certified` and `ExactSymbolic` modes are for topology decisions that should
//! follow robust-predicate practice as in adaptive robust predicates.
//! `EdgePreview` is reserved for rendering, diagnostics, and compatibility
//! boundaries where finite-output segment intersection finite-output concerns are accepted explicitly.

/// Runtime numeric mode for whole curve operations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NumericMode {
    /// Explicit preview or compatibility boundary mode.
    ///
    /// This mode may use named lossy estimates for diagnostics and rendering
    /// preview, but core hyper topology should use [`NumericMode::Certified`]
    /// or [`NumericMode::ExactSymbolic`].
    EdgePreview,
    /// Hybrid mode. Filtered or structural witnesses are accepted only when certified.
    Certified,
    /// Exact/symbolic mode. Tolerance collapse is not allowed.
    ExactSymbolic,
}

/// Optional absolute/relative tolerance metadata for edge-preview operations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tolerance {
    /// Absolute tolerance.
    pub absolute: f64,
    /// Relative tolerance.
    pub relative: f64,
}

impl Tolerance {
    /// Constructs a tolerance pair.
    pub const fn new(absolute: f64, relative: f64) -> Self {
        Self { absolute, relative }
    }
}

/// Operation-level policy for curve algorithms.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePolicy {
    /// Numeric mode requested by the caller.
    pub numeric_mode: NumericMode,
    /// Predicate escalation policy used by topology decisions.
    #[cfg(feature = "predicates")]
    pub predicate_policy: hyperlimit::PredicatePolicy,
    /// Optional tolerance carried by edge-preview operations.
    pub tolerance: Option<Tolerance>,
}

impl CurvePolicy {
    /// Conservative topology policy.
    pub const fn certified() -> Self {
        Self {
            numeric_mode: NumericMode::Certified,
            #[cfg(feature = "predicates")]
            predicate_policy: hyperlimit::PredicatePolicy::STRICT,
            tolerance: None,
        }
    }

    /// Edge-preview policy for diagnostics and exploratory rendering.
    ///
    /// This policy is intentionally not the default. It exists for code that is
    /// already at an IO, rendering, or compatibility boundary. Hyperlimit
    /// predicates still run under the strict exact policy; the tolerance is
    /// available only to curve-local preview operations.
    pub const fn edge_preview(tolerance: Tolerance) -> Self {
        Self {
            numeric_mode: NumericMode::EdgePreview,
            #[cfg(feature = "predicates")]
            predicate_policy: hyperlimit::PredicatePolicy::STRICT,
            tolerance: Some(tolerance),
        }
    }

    /// Exact/symbolic policy.
    pub const fn exact_symbolic() -> Self {
        Self {
            numeric_mode: NumericMode::ExactSymbolic,
            #[cfg(feature = "predicates")]
            predicate_policy: hyperlimit::PredicatePolicy::STRICT,
            tolerance: None,
        }
    }
}

impl Default for CurvePolicy {
    fn default() -> Self {
        Self::certified()
    }
}
