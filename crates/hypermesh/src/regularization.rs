//! Exact arrangement regularization policy and blockers.
//!
//! Regularization in the exact stack is a topology policy, not a tolerance
//! repair pass. Lower-dimensional leftovers, undecidable predicates, and
//! unsupported primitive families are retained as explicit blockers or
//! artifacts according to caller policy.

use super::graph::{IntersectionGraphValidationError, SplitPlanBlockerKind};

/// Policy for lower-dimensional arrangement remnants.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactLowerDimensionalPolicy {
    /// Drop lower-dimensional contacts from regularized solid output.
    Drop,
    /// Retain lower-dimensional contacts as separate artifacts.
    RetainArtifacts,
}

/// Policy for exact predicates or constructions that do not resolve.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactUnresolvedPolicy {
    /// Block the operation when an exact decision is unresolved.
    Block,
    /// Retain unresolved evidence as an artifact for later replay.
    RetainArtifacts,
}

/// Regularization policy for arrangement/cell-complex operations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ExactRegularizationPolicy {
    /// How lower-dimensional contacts are handled.
    pub(crate) lower_dimensional: ExactLowerDimensionalPolicy,
    /// How unresolved predicates/constructions are handled.
    pub(crate) unresolved: ExactUnresolvedPolicy,
}

impl ExactRegularizationPolicy {
    /// Regularized solid policy: drop lower-dimensional leftovers and block on
    /// unresolved exact decisions.
    pub(crate) const REGULARIZED_SOLID: Self = Self {
        lower_dimensional: ExactLowerDimensionalPolicy::Drop,
        unresolved: ExactUnresolvedPolicy::Block,
    };

    /// Diagnostic policy: keep lower-dimensional and unresolved evidence as
    /// artifacts where the downstream type can represent them.
    pub(crate) const RETAIN_ARTIFACTS: Self = Self {
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
pub(crate) enum ExactArrangementBlocker {
    /// Exact ordering could not be certified.
    UndecidableOrdering,
    /// An intersection predicate or construction did not resolve.
    UnresolvedIntersection,
    /// The retained cell complex is not manifold under the requested policy.
    NonManifoldCellComplex,
    /// Source boundary sheets overlap or cross in the arrangement, but the
    /// retained face-cells have not yet been regularized into volume-boundary
    /// cells. This is the exact arrangement handoff for closed-solid cases
    /// that still need cell-complex volume construction rather than a generic
    /// non-manifold topology failure.
    UnregularizedCoincidentSheetComplex,
    /// Source boundary sheets have been split into an open mixed-source sheet
    /// complex. The exact intersections are retained, but the arrangement has
    /// not yet reconstructed the missing regularized volume-boundary cells
    /// needed to form closed shells.
    UnregularizedOpenSheetComplex,
    /// Retained intersection graph evidence was structurally invalid.
    InvalidIntersectionGraph(ExactArrangementGraphBlockerKind),
    /// Retained split-plan evidence was structurally invalid.
    InvalidSplitPlan(ExactArrangementSplitPlanBlockerKind),
    /// Exact winding/inside-outside classification could not decide.
    UnresolvedRegionClassification,
}

/// Stable public category for retained intersection-graph blockers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactArrangementGraphBlockerKind {
    /// A retained face-pair record references a missing source face.
    FaceIndexOutOfRange,
    /// A retained event references a missing source vertex or face.
    EventSourceOutOfRange,
    /// A retained event does not belong to the retained face pair.
    EventSourceMismatch,
    /// A rejected face-pair relation retained graph-construction events.
    RejectedPairHasEvents,
    /// A non-rejected face-pair relation retained no event evidence.
    RetainedPairHasNoEvents,
    /// An unknown face pair did not retain an unknown marker.
    UnknownPairMissingUnknownEvent,
    /// A coplanar face pair did not retain its certified projection.
    CoplanarPairMissingProjection,
    /// A non-coplanar relation retained a coplanar projection.
    NonCoplanarPairHasProjection,
    /// A coplanar face pair retained non-coplanar segment-plane evidence.
    CoplanarPairHasSegmentPlaneEvent,
    /// A non-coplanar face pair retained coplanar edge or vertex evidence.
    NonCoplanarPairHasCoplanarEvent,
    /// A segment/plane event retained a disjoint relation.
    DisjointSegmentPlaneEvent,
    /// A segment/plane event has inconsistent side facts or construction data.
    InvalidSegmentPlaneEvent,
    /// A coplanar edge event retained a disjoint relation.
    DisjointCoplanarEdgeEvent,
    /// A coplanar vertex event retained an outside or degenerate location.
    NonConstructiveCoplanarVertexEvent,
    /// Source replay did not reproduce the retained graph artifact.
    SourceReplayMismatch,
}

impl From<IntersectionGraphValidationError> for ExactArrangementGraphBlockerKind {
    fn from(error: IntersectionGraphValidationError) -> Self {
        match error {
            IntersectionGraphValidationError::FaceIndexOutOfRange => Self::FaceIndexOutOfRange,
            IntersectionGraphValidationError::EventSourceOutOfRange => Self::EventSourceOutOfRange,
            IntersectionGraphValidationError::EventSourceMismatch => Self::EventSourceMismatch,
            IntersectionGraphValidationError::RejectedPairHasEvents => Self::RejectedPairHasEvents,
            IntersectionGraphValidationError::RetainedPairHasNoEvents => {
                Self::RetainedPairHasNoEvents
            }
            IntersectionGraphValidationError::UnknownPairMissingUnknownEvent => {
                Self::UnknownPairMissingUnknownEvent
            }
            IntersectionGraphValidationError::CoplanarPairMissingProjection => {
                Self::CoplanarPairMissingProjection
            }
            IntersectionGraphValidationError::NonCoplanarPairHasProjection => {
                Self::NonCoplanarPairHasProjection
            }
            IntersectionGraphValidationError::CoplanarPairHasSegmentPlaneEvent => {
                Self::CoplanarPairHasSegmentPlaneEvent
            }
            IntersectionGraphValidationError::NonCoplanarPairHasCoplanarEvent => {
                Self::NonCoplanarPairHasCoplanarEvent
            }
            IntersectionGraphValidationError::DisjointSegmentPlaneEvent => {
                Self::DisjointSegmentPlaneEvent
            }
            IntersectionGraphValidationError::InvalidSegmentPlaneEvent => {
                Self::InvalidSegmentPlaneEvent
            }
            IntersectionGraphValidationError::DisjointCoplanarEdgeEvent => {
                Self::DisjointCoplanarEdgeEvent
            }
            IntersectionGraphValidationError::NonConstructiveCoplanarVertexEvent => {
                Self::NonConstructiveCoplanarVertexEvent
            }
            IntersectionGraphValidationError::SourceReplayMismatch => Self::SourceReplayMismatch,
        }
    }
}

/// Stable public category for retained split-plan blockers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactArrangementSplitPlanBlockerKind {
    /// Exact parameter ordering could not be certified.
    UnknownOrdering,
    /// Exact split-point equality could not be certified.
    UnresolvedEquality,
    /// A split point could not be matched to a graph vertex.
    UnresolvedVertexLookup,
    /// A segment/plane split point is missing endpoint side facts.
    MissingEndpointSideFacts,
    /// A segment/plane split point was not certified by opposite strict sides.
    NonCrossingEndpointSideFacts,
    /// A retained split-point determinant ratio does not match its parameter.
    InvalidConstructionRatio,
    /// A split chain has no usable endpoint-to-endpoint path.
    EmptyOrShortEdgeChain,
    /// A split chain does not begin at its directed edge start.
    WrongChainStart,
    /// A split chain does not end at its directed edge end.
    WrongChainEnd,
    /// An original vertex node appears on the wrong mesh side.
    ChainSideMismatch,
    /// A graph-vertex reference is out of range.
    GraphVertexOutOfRange,
    /// A merged graph vertex has no source uses.
    EmptyGraphVertexUses,
    /// A face split work item has no split edges.
    EmptyFaceSplit,
    /// A face split edge has no graph vertices.
    EmptyFaceSplitEdge,
    /// A face split plan repeats the same original edge for one face.
    DuplicateFaceSplitEdge,
    /// A face split edge references a graph vertex with no matching source use.
    MissingFaceSplitSourceUse,
    /// Boundary incidence against the original face plane could not be decided.
    UnknownBoundaryIncidence,
    /// A split boundary node is not on the original face plane.
    BoundaryNodeOffFacePlane,
    /// A retained split face or region carried a mismatched source triangle.
    SourceTriangleMismatch,
    /// A split face region has fewer than three boundary nodes.
    EmptyOrShortRegionBoundary,
    /// A split face region contains consecutive duplicate boundary nodes.
    DuplicateConsecutiveRegionNode,
    /// A split-boundary chain references an edge that is not on the source triangle.
    BoundaryChainEdgeNotOnTriangle,
    /// A retained boundary original node references a missing source vertex.
    BoundaryNodeSourceVertexOutOfRange,
    /// A retained boundary original node is not part of its source triangle.
    BoundaryNodeSourceVertexNotOnTriangle,
    /// A retained boundary original node point no longer matches its source vertex.
    BoundaryNodeSourcePointMismatch,
}

impl From<SplitPlanBlockerKind> for ExactArrangementSplitPlanBlockerKind {
    fn from(kind: SplitPlanBlockerKind) -> Self {
        match kind {
            SplitPlanBlockerKind::UnknownOrdering => Self::UnknownOrdering,
            SplitPlanBlockerKind::UnresolvedEquality => Self::UnresolvedEquality,
            SplitPlanBlockerKind::UnresolvedVertexLookup => Self::UnresolvedVertexLookup,
            SplitPlanBlockerKind::MissingEndpointSideFacts => Self::MissingEndpointSideFacts,
            SplitPlanBlockerKind::NonCrossingEndpointSideFacts => {
                Self::NonCrossingEndpointSideFacts
            }
            SplitPlanBlockerKind::InvalidConstructionRatio => Self::InvalidConstructionRatio,
            SplitPlanBlockerKind::EmptyOrShortEdgeChain => Self::EmptyOrShortEdgeChain,
            SplitPlanBlockerKind::WrongChainStart => Self::WrongChainStart,
            SplitPlanBlockerKind::WrongChainEnd => Self::WrongChainEnd,
            SplitPlanBlockerKind::ChainSideMismatch => Self::ChainSideMismatch,
            SplitPlanBlockerKind::GraphVertexOutOfRange => Self::GraphVertexOutOfRange,
            SplitPlanBlockerKind::EmptyGraphVertexUses => Self::EmptyGraphVertexUses,
            SplitPlanBlockerKind::EmptyFaceSplit => Self::EmptyFaceSplit,
            SplitPlanBlockerKind::EmptyFaceSplitEdge => Self::EmptyFaceSplitEdge,
            SplitPlanBlockerKind::DuplicateFaceSplitEdge => Self::DuplicateFaceSplitEdge,
            SplitPlanBlockerKind::MissingFaceSplitSourceUse => Self::MissingFaceSplitSourceUse,
            SplitPlanBlockerKind::UnknownBoundaryIncidence => Self::UnknownBoundaryIncidence,
            SplitPlanBlockerKind::BoundaryNodeOffFacePlane => Self::BoundaryNodeOffFacePlane,
            #[cfg(test)]
            SplitPlanBlockerKind::SourceReplayMismatch => Self::SourceTriangleMismatch,
            SplitPlanBlockerKind::SourceTriangleMismatch => Self::SourceTriangleMismatch,
            SplitPlanBlockerKind::EmptyOrShortRegionBoundary => Self::EmptyOrShortRegionBoundary,
            SplitPlanBlockerKind::DuplicateConsecutiveRegionNode => {
                Self::DuplicateConsecutiveRegionNode
            }
            SplitPlanBlockerKind::BoundaryChainEdgeNotOnTriangle => {
                Self::BoundaryChainEdgeNotOnTriangle
            }
            SplitPlanBlockerKind::BoundaryNodeSourceVertexOutOfRange => {
                Self::BoundaryNodeSourceVertexOutOfRange
            }
            SplitPlanBlockerKind::BoundaryNodeSourceVertexNotOnTriangle => {
                Self::BoundaryNodeSourceVertexNotOnTriangle
            }
            SplitPlanBlockerKind::BoundaryNodeSourcePointMismatch => {
                Self::BoundaryNodeSourcePointMismatch
            }
        }
    }
}
