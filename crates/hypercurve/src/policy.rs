//! Predicate-controlled topology and explicit edge-preview policy.

/// Numeric policy for a curve operation.
///
/// The default and [`CurvePolicy::certified`] policy exhaust exact and
/// certified-refinement stages before applying Hyperlimit's current terminal
/// predicate policy. [`CurvePolicy::edge_preview`] additionally permits lossy
/// finite views at rendering and diagnostics boundaries.
///
/// The representation is intentionally closed: callers cannot combine a
/// certified operation with preview tolerances, request preview behavior
/// without tolerances, or weaken the predicate policy used by topology.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePolicy {
    pub(crate) mode: NumericMode,
    #[cfg(feature = "predicates")]
    pub(crate) predicate_policy: hyperlimit::PredicatePolicy,
    pub(crate) preview_tolerance: Option<PreviewTolerance>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum NumericMode {
    EdgePreview,
    Certified,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PreviewTolerance {
    pub(crate) absolute: f64,
    pub(crate) relative: f64,
}

impl CurvePolicy {
    /// Topology policy backed by the workspace predicate policy.
    pub const fn certified() -> Self {
        Self {
            mode: NumericMode::Certified,
            #[cfg(feature = "predicates")]
            predicate_policy: hyperlimit::PredicatePolicy,
            preview_tolerance: None,
        }
    }

    /// Edge-preview policy for diagnostics and exploratory rendering.
    ///
    /// This policy is intentionally not the default. It exists for code that is
    /// already at an IO, rendering, or compatibility boundary. Hyperlimit
    /// predicates still use the centralized workspace policy. The tolerances
    /// are available only to named curve-local preview operations.
    pub const fn edge_preview(absolute_tolerance: f64, relative_tolerance: f64) -> Self {
        Self {
            mode: NumericMode::EdgePreview,
            #[cfg(feature = "predicates")]
            predicate_policy: hyperlimit::PredicatePolicy,
            preview_tolerance: Some(PreviewTolerance {
                absolute: absolute_tolerance,
                relative: relative_tolerance,
            }),
        }
    }
}

impl Default for CurvePolicy {
    fn default() -> Self {
        Self::certified()
    }
}

#[cfg(all(test, feature = "predicates"))]
mod tests {
    use super::CurvePolicy;

    #[test]
    fn curve_modes_follow_the_central_workspace_predicate_policy() {
        assert_eq!(
            CurvePolicy::certified().predicate_policy,
            hyperlimit::PredicatePolicy
        );
        assert_eq!(
            CurvePolicy::edge_preview(1.0e-6, 1.0e-6).predicate_policy,
            hyperlimit::PredicatePolicy
        );
    }
}
