//! Exact arrangement regularization policy and blockers.
//!
//! Regularization in the exact stack is a topology policy, not a tolerance
//! repair pass. Lower-dimensional leftovers, undecidable predicates, and
//! unsupported primitive families are retained as explicit blockers or
//! artifacts according to caller policy.

use super::graph::{IntersectionGraphValidationError, SplitPlanDiagnosticKind};

/// Policy for lower-dimensional arrangement remnants.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactLowerDimensionalPolicy {
    /// Drop lower-dimensional contacts from regularized solid output.
    Drop,
    /// Retain lower-dimensional contacts as separate artifacts.
    RetainArtifacts,
    /// Report lower-dimensional contacts as blockers.
    ReportBlocker,
}

/// Policy for exact predicates or constructions that do not resolve.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactUnresolvedPolicy {
    /// Block the operation when an exact decision is unresolved.
    Block,
    /// Retain unresolved evidence as an artifact for later replay.
    RetainArtifacts,
}

/// Regularization policy for arrangement/cell-complex operations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactRegularizationPolicy {
    /// How lower-dimensional contacts are handled.
    pub lower_dimensional: ExactLowerDimensionalPolicy,
    /// How unresolved predicates/constructions are handled.
    pub unresolved: ExactUnresolvedPolicy,
}

impl ExactRegularizationPolicy {
    /// Regularized solid policy: drop lower-dimensional leftovers and block on
    /// unresolved exact decisions.
    pub const REGULARIZED_SOLID: Self = Self {
        lower_dimensional: ExactLowerDimensionalPolicy::Drop,
        unresolved: ExactUnresolvedPolicy::Block,
    };

    /// Diagnostic policy: keep lower-dimensional and unresolved evidence as
    /// artifacts where the downstream type can represent them.
    pub const RETAIN_ARTIFACTS: Self = Self {
        lower_dimensional: ExactLowerDimensionalPolicy::RetainArtifacts,
        unresolved: ExactUnresolvedPolicy::RetainArtifacts,
    };
}

impl Default for ExactRegularizationPolicy {
    fn default() -> Self {
        Self::REGULARIZED_SOLID
    }
}

/// Blocker emitted by exact arrangement and cell-complex stages.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactArrangementBlocker {
    /// Exact ordering could not be certified.
    UndecidableOrdering,
    /// An intersection predicate or construction did not resolve.
    UnresolvedIntersection,
    /// The retained cell complex is not manifold under the requested policy.
    NonManifoldCellComplex,
    /// The primitive family is outside the current exact arrangement kernel.
    UnsupportedCurvedPrimitive,
    /// The requested output requires an explicit approximation/export policy.
    ApproximationPolicyRequired,
    /// Retained intersection graph evidence was structurally invalid.
    InvalidIntersectionGraph(IntersectionGraphValidationError),
    /// Retained split-plan evidence was structurally invalid.
    InvalidSplitPlan(SplitPlanDiagnosticKind),
    /// Exact winding/inside-outside classification could not decide.
    UnresolvedRegionClassification,
    /// Lower-dimensional contact was produced but policy does not retain it.
    LowerDimensionalContact,
}
