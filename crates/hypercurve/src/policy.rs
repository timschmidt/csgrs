//! Runtime numeric policy for curve operations.

/// Runtime numeric mode for whole curve operations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NumericMode {
    /// Fast preview mode. Results may be approximate and must say so.
    Approximate,
    /// Hybrid mode. Approximate witnesses are allowed only when certified.
    Certified,
    /// Exact/symbolic mode. Tolerance collapse is not allowed.
    ExactSymbolic,
}

/// Optional absolute/relative tolerance metadata for approximate operations.
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
    /// Optional tolerance carried by approximate operations.
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

    /// Fast approximate policy for previews and exploratory work.
    pub const fn approximate(tolerance: Tolerance) -> Self {
        Self {
            numeric_mode: NumericMode::Approximate,
            #[cfg(feature = "predicates")]
            predicate_policy: hyperlimit::PredicatePolicy::APPROXIMATE,
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
