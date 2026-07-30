//! Predicate-controlled topology and explicit edge-preview policy.

/// Numeric policy for a curve operation.
///
/// [`CurvePolicy::STRICT`] accepts only exact or certified-refinement
/// predicate decisions. [`CurvePolicy::APPROXIMATE_512`] additionally permits
/// Hyperlimit's terminal 512-bit interpretation. The named edge-preview
/// constructors permit lossy finite views only at rendering and diagnostics
/// boundaries.
///
/// The representation is intentionally closed: callers cannot combine a
/// certified operation with preview tolerances or request preview behavior
/// without tolerances.
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
    /// Topology accepts only certified predicate decisions.
    pub const STRICT: Self = Self {
        mode: NumericMode::Certified,
        #[cfg(feature = "predicates")]
        predicate_policy: hyperlimit::PredicatePolicy::STRICT,
        preview_tolerance: None,
    };

    /// Topology may consume Hyperlimit's terminal 512-bit interpretation.
    pub const APPROXIMATE_512: Self = Self {
        mode: NumericMode::Certified,
        #[cfg(feature = "predicates")]
        predicate_policy: hyperlimit::PredicatePolicy::APPROXIMATE_512,
        preview_tolerance: None,
    };

    /// Strict-predicate edge preview for diagnostics and exploratory rendering.
    ///
    /// The tolerances are available only to named curve-local preview
    /// operations.
    pub const fn edge_preview_strict(absolute_tolerance: f64, relative_tolerance: f64) -> Self {
        Self {
            mode: NumericMode::EdgePreview,
            #[cfg(feature = "predicates")]
            predicate_policy: hyperlimit::PredicatePolicy::STRICT,
            preview_tolerance: Some(PreviewTolerance {
                absolute: absolute_tolerance,
                relative: relative_tolerance,
            }),
        }
    }

    /// Approximate-512-predicate edge preview for diagnostics and rendering.
    pub const fn edge_preview_approximate_512(
        absolute_tolerance: f64,
        relative_tolerance: f64,
    ) -> Self {
        Self {
            mode: NumericMode::EdgePreview,
            #[cfg(feature = "predicates")]
            predicate_policy: hyperlimit::PredicatePolicy::APPROXIMATE_512,
            preview_tolerance: Some(PreviewTolerance {
                absolute: absolute_tolerance,
                relative: relative_tolerance,
            }),
        }
    }
}

#[cfg(all(test, feature = "predicates"))]
mod tests {
    use super::CurvePolicy;

    #[test]
    fn curve_modes_name_both_hyperlimit_policies() {
        assert_eq!(
            CurvePolicy::STRICT.predicate_policy,
            hyperlimit::PredicatePolicy::STRICT
        );
        assert_eq!(
            CurvePolicy::APPROXIMATE_512.predicate_policy,
            hyperlimit::PredicatePolicy::APPROXIMATE_512
        );
        assert_eq!(
            CurvePolicy::edge_preview_strict(1.0e-6, 1.0e-6).predicate_policy,
            hyperlimit::PredicatePolicy::STRICT
        );
        assert_eq!(
            CurvePolicy::edge_preview_approximate_512(1.0e-6, 1.0e-6).predicate_policy,
            hyperlimit::PredicatePolicy::APPROXIMATE_512
        );
    }
}
