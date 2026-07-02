//! Exact intersection graph event extraction.
//!
//! The graph here is intentionally an event graph, not yet a mutable boolean
//! topology. It converts certified face-pair classifications into stable
//! records for split points, coplanar edge contacts, containment facts, and
//! split geometry. Predicates and constructions produce auditable events first;
//! mesh mutation consumes those events only after validation.
//!
//! The event categories separate plane-side rejection, non-coplanar
//! segment/plane crossings, and coplanar overlap through projected segment and
//! containment predicates.

pub(crate) mod intersection;
pub(crate) mod key;

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::rc::Rc;

use hyperlimit::{
    PlaneSide, Point3, SegmentIntersection, SegmentPlaneConstructionFailure,
    SegmentPlaneIntersection, SegmentPlaneParameterRatio, SegmentPlaneRelation, Sign,
    TriangleLocation, compare_reals, interpolate_point3, orient3d_report, point_on_segment,
    project_point3, projected_line_parameter3, projected_segment_parameter3,
};

use super::bounds::try_visit_candidate_face_pairs_one_shot;
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError, ExactMeshSourceSide};
use super::prepared::PreparedMeshPair;
use super::{ExactMesh, point3_exact_equal, triangle_edges};
use hyperlimit::{CoplanarProjection, CoplanarTriangleClassification};
use hyperreal::Real;
use intersection::{
    MeshFacePairClassification, MeshFacePairRelation, classify_mesh_face_pair_unchecked,
};
use key::{ExactPoint3Key, exact_point3_key};

/// Side of a two-mesh graph event.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub(crate) enum MeshSide {
    /// The first mesh passed to graph construction.
    Left,
    /// The second mesh passed to graph construction.
    Right,
}

impl MeshSide {
    pub(crate) fn mesh<'a>(self, left: &'a ExactMesh, right: &'a ExactMesh) -> &'a ExactMesh {
        match self {
            Self::Left => left,
            Self::Right => right,
        }
    }
}

/// Exact intersection event extracted from a retained face pair.
///
/// The segment-plane variant intentionally retains the full exact construction
/// certificate inline so graph validation can replay predicate, ratio, and
/// computation history as part of the exact object boundary; boxing the fields
/// would reduce enum size but not the retained state that downstream audit
/// paths must carry.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum IntersectionEvent {
    /// A triangle edge intersects the opposite triangle plane.
    SegmentPlane {
        /// Mesh owning the segment edge.
        segment_side: MeshSide,
        /// Edge endpoints in that mesh's vertex index space.
        edge: [usize; 2],
        /// Mesh owning the plane face.
        plane_side: MeshSide,
        /// Face index of the plane triangle.
        plane_face: usize,
        /// Coarse segment/plane relation.
        relation: SegmentPlaneRelation,
        /// Exact point for endpoint and proper-crossing events.
        point: Option<Point3>,
        /// Exact edge parameter when available.
        parameter: Option<Real>,
        /// Determinant ratio that produced the exact edge parameter for a
        /// proper crossing.
        parameter_ratio: Option<SegmentPlaneParameterRatio>,
        /// Structured failure reason when endpoint predicates certified a
        /// crossing but exact point construction failed.
        construction_failure: Option<SegmentPlaneConstructionFailure>,
        /// Certified endpoint side facts retained from segment/plane
        /// classification.
        endpoint_sides: [Option<PlaneSide>; 2],
    },
    /// A projected coplanar edge-pair relation.
    CoplanarEdge {
        /// Edge in the left mesh.
        left_edge: [usize; 2],
        /// Edge in the right mesh.
        right_edge: [usize; 2],
        /// Exact projected segment relation.
        relation: SegmentIntersection,
    },
    /// A projected coplanar vertex containment fact.
    CoplanarVertex {
        /// Mesh owning the tested vertex.
        vertex_side: MeshSide,
        /// Vertex index in that mesh.
        vertex: usize,
        /// Mesh owning the containing face.
        triangle_side: MeshSide,
        /// Face index in the containing mesh.
        triangle_face: usize,
        /// Exact projected point/triangle location.
        location: TriangleLocation,
    },
    /// A retained pair could not be completely decided.
    Unknown,
}

impl IntersectionEvent {
    /// Return whether this retained event still carries undecided evidence.
    ///
    /// `IntersectionEvent::Unknown` is the explicit graph marker. A
    /// segment/plane event whose retained relation is `Unknown` is equally a
    /// refinement blocker even though it has structured endpoint-side evidence.
    /// Keeping this on the graph event type makes boolean/report summaries use
    /// the same exact-evidence boundary as graph routing.
    pub const fn has_unknown_relation(&self) -> bool {
        matches!(
            self,
            Self::Unknown
                | Self::SegmentPlane {
                    relation: SegmentPlaneRelation::Unknown,
                    ..
                }
        )
    }
}

/// Retained projected edge contact in a coplanar face-pair overlap graph.
///
/// These records are arrangement inputs, not final topology. They retain the
/// coplanar decomposition while keeping mutation deferred until split planning.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct CoplanarEdgeOverlap {
    /// Edge in the left face.
    pub left_edge: [usize; 2],
    /// Edge in the right face.
    pub right_edge: [usize; 2],
    /// Certified projected segment relation.
    pub relation: SegmentIntersection,
}

/// Retained vertex containment/touching fact in a coplanar overlap graph.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct CoplanarVertexOverlap {
    /// Mesh owning the retained vertex.
    pub vertex_side: MeshSide,
    /// Vertex index in the owning mesh.
    pub vertex: usize,
    /// Opposite mesh owning the containing/touching triangle.
    pub triangle_side: MeshSide,
    /// Face index of the opposite triangle.
    pub triangle_face: usize,
    /// Certified projected point/triangle location.
    pub location: TriangleLocation,
}

/// Non-mutating exact coplanar overlap graph for one retained face pair.
///
/// This is the first explicit arrangement artifact for coplanar triangle
/// pairs. It groups edge contacts and vertex-in-triangle facts that were
/// already certified by `hyperlimit`, but deliberately avoids inventing split
/// vertices or planar cells until a later exact arrangement stage can retain
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct CoplanarOverlapGraph {
    /// Face index in the left mesh.
    pub left_face: usize,
    /// Face index in the right mesh.
    pub right_face: usize,
    /// Coarse coplanar face-pair relation.
    pub relation: MeshFacePairRelation,
    /// Certified projection used for the retained 2D predicates.
    pub projection: CoplanarProjection,
    /// Non-disjoint projected edge contacts.
    pub edge_overlaps: Vec<CoplanarEdgeOverlap>,
    /// Constructive vertex/triangle facts.
    pub vertex_overlaps: Vec<CoplanarVertexOverlap>,
}

/// Exact split point constructed from one coplanar projected edge contact.
///
/// Proper edge crossings retain the determinant-ratio parameters on both
/// participating edges. Endpoint touches retain exact endpoint parameters.
/// Collinear positive-length overlaps retain exact interval endpoints so a
/// later planar arrangement stage can order interval topology without
/// recovering it from primitive coordinates.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CoplanarEdgeSplitPoint {
    /// Exact 3D point on the shared coplanar face plane.
    pub point: Point3,
    /// Parameter on [`CoplanarEdgeOverlap::left_edge`].
    pub left_parameter: Real,
    /// Parameter on [`CoplanarEdgeOverlap::right_edge`].
    pub right_parameter: Real,
}

/// Exact endpoint pair for a positive-length coplanar edge interval.
///
/// The endpoint order is by the left-edge parameter. Retaining both endpoint
/// sort and merge interval topology from exact object facts rather than from
/// projected labels or primitive-float coordinates.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CoplanarEdgeInterval {
    /// Certified closed interval endpoints.
    pub endpoints: [CoplanarEdgeSplitPoint; 2],
}

/// Retained split construction for one coplanar edge contact.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CoplanarEdgeSplitConstruction {
    /// Original edge contact record.
    pub overlap: CoplanarEdgeOverlap,
    /// Constructed point events for proper crossings or endpoint touches.
    pub points: Vec<CoplanarEdgeSplitPoint>,
    /// Whether the contact is a positive-length collinear interval whose
    /// endpoint construction remains a later planar-arrangement step.
    pub interval_overlap: bool,
    /// Retained exact endpoints for a positive-length collinear interval.
    pub interval: Option<CoplanarEdgeInterval>,
}

/// Non-mutating split-construction plan for retained coplanar overlap graphs.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CoplanarOverlapSplitPlan {
    /// Per-face-pair overlap graph split records.
    pub graphs: Vec<CoplanarOverlapSplitGraph>,
}

/// Split construction records for one coplanar face-pair overlap graph.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CoplanarOverlapSplitGraph {
    /// Face index in the left mesh.
    pub left_face: usize,
    /// Face index in the right mesh.
    pub right_face: usize,
    /// Projection used to construct the split records.
    pub projection: CoplanarProjection,
    /// Edge split/interval constructions.
    pub edge_splits: Vec<CoplanarEdgeSplitConstruction>,
    /// Vertex overlap facts copied from the source overlap graph.
    pub vertex_overlaps: Vec<CoplanarVertexOverlap>,
}

/// Status of retained coplanar evidence for exact planar-cell extraction.
///
/// The graph retains certified coplanar edge and vertex-contact facts. Positive
/// area overlaps require planar cells before named boolean output can be
/// materialized.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CoplanarArrangementEvidenceStatus {
    /// No retained coplanar overlap graph exists.
    NoCoplanarOverlap,
    /// Retained coplanar graphs contain boundary-only touching evidence.
    BoundaryOnly,
    /// At least one retained positive-area coplanar overlap needs planar-cell
    /// extraction before named union/difference output can be materialized.
    NeedsPlanarCells,
}

/// Auditable summary of retained coplanar overlap evidence.
///
/// This report is intentionally compact: it does not replace the underlying
/// [`CoplanarOverlapGraph`] or [`CoplanarOverlapSplitPlan`], but summarizes
/// their validated counts before exact planar-cell extraction. That lets
/// blockers retain actionable provenance instead of a plain unsupported flag.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct CoplanarArrangementEvidence {
    /// Coarse state of the retained coplanar arrangement evidence.
    pub(crate) status: CoplanarArrangementEvidenceStatus,
    /// Number of retained coplanar overlap graphs.
    pub(crate) graph_count: usize,
    /// Number of graphs whose coarse relation is positive-area overlap.
    pub(crate) overlapping_graphs: usize,
    /// Number of graphs whose coarse relation is boundary-only touching.
    pub(crate) touching_graphs: usize,
    /// Number of retained non-disjoint edge contacts.
    pub(crate) edge_overlap_count: usize,
    /// Number of retained vertex-in-triangle or vertex-on-triangle facts.
    pub(crate) vertex_overlap_count: usize,
    /// Number of exact point split constructions retained for proper or
    /// endpoint edge contacts.
    pub(crate) point_split_count: usize,
    /// Number of positive-length collinear interval contacts retained for
    /// planar-cell extraction.
    pub(crate) interval_overlap_count: usize,
    /// Number of exact interval endpoint facts retained for collinear contacts.
    pub(crate) interval_endpoint_count: usize,
}

/// Structural inconsistency in a retained coplanar overlap graph.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CoplanarOverlapGraphValidationError {
    /// The graph relation is not a coplanar retained relation.
    NonCoplanarRelation,
    /// The graph has no edge or vertex evidence.
    EmptyOverlapGraph,
    /// An edge record retained a disjoint relation.
    DisjointEdgeOverlap,
    /// A vertex record retained an outside or degenerate location.
    NonConstructiveVertexOverlap,
    /// A vertex record does not connect left and right meshes.
    SameSideVertexOverlap,
    /// Recomputing coplanar overlap graphs from the supplied source meshes did
    /// not reproduce this retained graph.
    #[cfg(test)]
    SourceReplayMismatch,
}

/// Structural inconsistency in retained coplanar split construction records.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CoplanarOverlapSplitValidationError {
    /// A proper or endpoint contact did not retain exactly one split point.
    MissingPointConstruction,
    /// A disjoint edge contact appeared in the split plan.
    DisjointEdgeSplit,
    /// A collinear interval contact did not retain interval state.
    MissingIntervalConstruction,
    /// A collinear interval contact retained only the older interval flag
    /// without exact interval endpoints.
    MissingIntervalEndpoints,
    /// A non-interval contact unexpectedly retained interval state.
    UnexpectedIntervalConstruction,
    /// A collinear interval unexpectedly retained point construction.
    UnexpectedPointConstruction,
    /// A retained split parameter is outside the closed source-edge interval.
    SplitParameterOutOfRange,
    /// A retained split parameter could not be certified against the
    /// source-edge interval.
    UnknownSplitParameterOrder,
    /// A retained endpoint touch did not keep at least one endpoint parameter.
    EndpointTouchWithoutEndpointParameter,
    /// A retained proper crossing used an endpoint parameter.
    ProperCrossingEndpointParameter,
    /// A retained interval has duplicate endpoint parameters.
    DegenerateInterval,
    /// A retained interval endpoint order could not be certified.
    UnknownIntervalOrder,
    /// A retained split point is not the exact interpolation of its left edge
    /// at the stored parameter.
    SplitPointDoesNotMatchLeftParameter,
    /// A retained split point is not the exact interpolation of its right edge
    /// at the stored parameter.
    SplitPointDoesNotMatchRightParameter,
    /// A retained split point could not be certified against replayed source
    /// edge interpolation.
    UnknownSplitPointEquality,
    /// A retained copied vertex-overlap fact does not connect left and right
    /// meshes.
    SameSideVertexOverlap,
    /// A retained copied vertex-overlap fact is outside or degenerate.
    NonConstructiveVertexOverlap,
    /// Recomputing coplanar split constructions from the supplied source
    /// meshes did not reproduce this retained split artifact.
    #[cfg(test)]
    SourceReplayMismatch,
}

/// Structural inconsistency in a coplanar arrangement evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CoplanarArrangementEvidenceError {
    /// `NoCoplanarOverlap` retained nonzero graph or event counts.
    NoOverlapWithEvidence,
    /// Boundary-only status retained positive-area overlap graphs.
    BoundaryOnlyHasOverlap,
    /// Boundary-only status retained no touching graphs.
    BoundaryOnlyMissingTouchingGraph,
    /// Planar-cell status retained no positive-area overlap graphs.
    NeedsCellsMissingOverlap,
    /// A nonempty graph summary retained neither edge nor vertex evidence.
    MissingOverlapEvidence,
    /// Graph-count fields are internally inconsistent.
    GraphCountMismatch,
    /// Retained point/interval split counts exceed retained edge contacts.
    SplitCountExceedsEdgeEvidence,
    /// Retained interval endpoint facts do not match retained interval
    /// contacts.
    IntervalEndpointCountMismatch,
    /// Recomputing the evidence summary from the supplied source meshes did
    /// not reproduce this retained report.
    #[cfg(test)]
    SourceReplayMismatch,
}

impl CoplanarOverlapGraph {
    /// Validate that this grouped overlap graph is coherent.
    pub fn validate(&self) -> Result<(), CoplanarOverlapGraphValidationError> {
        if !matches!(
            self.relation,
            MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping
        ) {
            return Err(CoplanarOverlapGraphValidationError::NonCoplanarRelation);
        }
        if self.edge_overlaps.is_empty() && self.vertex_overlaps.is_empty() {
            return Err(CoplanarOverlapGraphValidationError::EmptyOverlapGraph);
        }
        for edge in &self.edge_overlaps {
            if edge.relation == SegmentIntersection::Disjoint {
                return Err(CoplanarOverlapGraphValidationError::DisjointEdgeOverlap);
            }
        }
        for vertex in &self.vertex_overlaps {
            if vertex.vertex_side == vertex.triangle_side {
                return Err(CoplanarOverlapGraphValidationError::SameSideVertexOverlap);
            }
            if matches!(
                vertex.location,
                TriangleLocation::Outside | TriangleLocation::Degenerate
            ) {
                return Err(CoplanarOverlapGraphValidationError::NonConstructiveVertexOverlap);
            }
        }
        Ok(())
    }

    /// Validate this overlap graph against the source meshes that produced it.
    ///
    /// Structural validation proves that retained edge and vertex facts are
    /// locally coherent. Source replay rebuilds the exact intersection graph
    /// from `left` and `right`, extracts all coplanar overlap graphs, and
    /// evidence must remain tied to the operands whose predicates produced it.
    #[cfg(test)]
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), CoplanarOverlapGraphValidationError> {
        self.validate()?;
        let replay = build_unvalidated_intersection_graph(left, right)
            .and_then(|graph| {
                graph
                    .coplanar_overlap_graphs()
                    .map_err(|error| retained_coplanar_graph_error(error, "replay overlap graph"))
            })
            .map_err(|_| CoplanarOverlapGraphValidationError::SourceReplayMismatch)?;
        if replay.iter().any(|graph| graph == self) {
            Ok(())
        } else {
            Err(CoplanarOverlapGraphValidationError::SourceReplayMismatch)
        }
    }
}

impl CoplanarArrangementEvidence {
    /// Return whether later planar-cell extraction is required.
    pub const fn needs_planar_cells(&self) -> bool {
        matches!(
            self.status,
            CoplanarArrangementEvidenceStatus::NeedsPlanarCells
        )
    }

    /// Validate that the compact evidence summary is internally coherent.
    ///
    /// The report validates counts, not source geometry. That still keeps the
    /// compact retained-state summary auditable before it influences
    /// combinatorial output.
    pub fn validate(&self) -> Result<(), CoplanarArrangementEvidenceError> {
        let Some(graph_count) = self.overlapping_graphs.checked_add(self.touching_graphs) else {
            return Err(CoplanarArrangementEvidenceError::GraphCountMismatch);
        };
        if self.graph_count != graph_count {
            return Err(CoplanarArrangementEvidenceError::GraphCountMismatch);
        }
        // Split summaries must be dominated by the edge contacts that produced
        // them.
        let Some(edge_split_constructions) = self
            .point_split_count
            .checked_add(self.interval_overlap_count)
        else {
            return Err(CoplanarArrangementEvidenceError::SplitCountExceedsEdgeEvidence);
        };
        if edge_split_constructions > self.edge_overlap_count {
            return Err(CoplanarArrangementEvidenceError::SplitCountExceedsEdgeEvidence);
        }
        let Some(interval_endpoint_count) = self.interval_overlap_count.checked_mul(2) else {
            return Err(CoplanarArrangementEvidenceError::IntervalEndpointCountMismatch);
        };
        if self.interval_endpoint_count != interval_endpoint_count {
            return Err(CoplanarArrangementEvidenceError::IntervalEndpointCountMismatch);
        }
        if self.graph_count > 0 && self.edge_overlap_count == 0 && self.vertex_overlap_count == 0 {
            return Err(CoplanarArrangementEvidenceError::MissingOverlapEvidence);
        }
        match self.status {
            CoplanarArrangementEvidenceStatus::NoCoplanarOverlap => {
                if self.graph_count == 0
                    && self.edge_overlap_count == 0
                    && self.vertex_overlap_count == 0
                    && self.point_split_count == 0
                    && self.interval_overlap_count == 0
                    && self.interval_endpoint_count == 0
                {
                    Ok(())
                } else {
                    Err(CoplanarArrangementEvidenceError::NoOverlapWithEvidence)
                }
            }
            CoplanarArrangementEvidenceStatus::BoundaryOnly => {
                if self.overlapping_graphs != 0 {
                    return Err(CoplanarArrangementEvidenceError::BoundaryOnlyHasOverlap);
                }
                if self.touching_graphs == 0 {
                    return Err(CoplanarArrangementEvidenceError::BoundaryOnlyMissingTouchingGraph);
                }
                Ok(())
            }
            CoplanarArrangementEvidenceStatus::NeedsPlanarCells => {
                if self.overlapping_graphs > 0 {
                    Ok(())
                } else {
                    Err(CoplanarArrangementEvidenceError::NeedsCellsMissingOverlap)
                }
            }
        }
    }

    /// Validate this evidence report against the source meshes that produced it.
    ///
    /// Local validation proves only that the compact counters are internally
    /// coherent. Source replay rebuilds the exact intersection graph and
    /// coplanar split summaries from `left` and `right`, then compares the
    /// retained summary with the rebuilt predicate and construction history.
    #[cfg(test)]
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), CoplanarArrangementEvidenceError> {
        self.validate()?;
        let replay = build_unvalidated_intersection_graph(left, right)
            .and_then(|graph| graph.coplanar_arrangement_evidence(left, right))
            .map_err(|_| CoplanarArrangementEvidenceError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(CoplanarArrangementEvidenceError::SourceReplayMismatch)
        }
    }
}

impl CoplanarOverlapSplitPlan {
    /// Validate every retained coplanar split construction record.
    #[cfg(test)]
    pub fn validate(&self) -> Result<(), CoplanarOverlapSplitValidationError> {
        for graph in &self.graphs {
            graph.validate()?;
        }
        Ok(())
    }

    /// Validate this split plan by replaying it from source operands.
    ///
    /// Mesh validation checks each retained split point against source-edge
    /// interpolation. This stronger audit also rebuilds the coplanar overlap
    /// graphs and split constructions from `left` and `right`, then compares
    /// them with the retained plan.
    #[cfg(test)]
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), CoplanarOverlapSplitValidationError> {
        self.validate()?;
        let replay = build_unvalidated_intersection_graph(left, right)
            .and_then(|graph| graph.coplanar_overlap_split_plan(left, right))
            .map_err(|_| CoplanarOverlapSplitValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(CoplanarOverlapSplitValidationError::SourceReplayMismatch)
        }
    }
}

impl CoplanarOverlapSplitGraph {
    /// Validate split-point and interval construction consistency.
    pub fn validate(&self) -> Result<(), CoplanarOverlapSplitValidationError> {
        for split in &self.edge_splits {
            validate_coplanar_edge_split(split)?;
        }
        for vertex in &self.vertex_overlaps {
            if vertex.vertex_side == vertex.triangle_side {
                return Err(CoplanarOverlapSplitValidationError::SameSideVertexOverlap);
            }
            if matches!(
                vertex.location,
                TriangleLocation::Outside | TriangleLocation::Degenerate
            ) {
                return Err(CoplanarOverlapSplitValidationError::NonConstructiveVertexOverlap);
            }
        }
        Ok(())
    }

    /// Validate split records against exact source mesh edge geometry.
    pub fn validate_against_meshes(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactMeshError> {
        self.validate()
            .map_err(coplanar_split_validation_mesh_error)?;
        for split in &self.edge_splits {
            let left_edge = edge_points(left, split.overlap.left_edge)?;
            let right_edge = edge_points(right, split.overlap.right_edge)?;
            validate_coplanar_edge_split_against_edges(split, left_edge, right_edge)
                .map_err(coplanar_split_validation_mesh_error)?;
        }
        Ok(())
    }
}

/// Structural inconsistency in a retained intersection graph event.
///
/// This validates the graph record before split extraction or topology
/// construction artifacts as the boundary between numerical decisions and
/// combinatorial mutation; a graph event whose coarse relation and retained
/// payload disagree must be rejected at that boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum IntersectionGraphValidationError {
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
    /// A coplanar face-pair relation retained non-coplanar segment-plane evidence.
    CoplanarPairHasSegmentPlaneEvent,
    /// A non-coplanar face-pair relation retained coplanar edge or vertex evidence.
    NonCoplanarPairHasCoplanarEvent,
    /// A segment/plane graph event retained a disjoint relation.
    DisjointSegmentPlaneEvent,
    /// A segment/plane event has inconsistent side facts or construction data.
    InvalidSegmentPlaneEvent,
    /// A coplanar edge event retained a disjoint relation.
    DisjointCoplanarEdgeEvent,
    /// A coplanar vertex event retained an outside or degenerate location.
    NonConstructiveCoplanarVertexEvent,
    /// Recomputing graph events from the supplied source meshes did not
    /// reproduce this retained graph artifact.
    SourceReplayMismatch,
}

/// Event records for one retained face pair.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FacePairEvents {
    /// Face index in the left mesh.
    pub left_face: usize,
    /// Face index in the right mesh.
    pub right_face: usize,
    /// Coarse relation that caused retention.
    pub relation: MeshFacePairRelation,
    /// Projection used by coplanar events, if any.
    pub projection: Option<CoplanarProjection>,
    /// Extracted exact events.
    pub events: Vec<IntersectionEvent>,
}

impl FacePairEvents {
    /// Validate one retained face-pair event record.
    ///
    /// This is a structural audit of the event graph object, not a recomputed
    /// triangle/triangle classification. It keeps the retained relation,
    /// projection, and event payloads consistent before downstream split
    /// planning converts construction records into topology.
    pub fn validate(&self) -> Result<(), IntersectionGraphValidationError> {
        if !matches!(
            self.relation,
            MeshFacePairRelation::Candidate
                | MeshFacePairRelation::CoplanarTouching
                | MeshFacePairRelation::CoplanarOverlapping
                | MeshFacePairRelation::Unknown
        ) {
            return if self.events.is_empty() {
                Ok(())
            } else {
                Err(IntersectionGraphValidationError::RejectedPairHasEvents)
            };
        }
        if self.events.is_empty() {
            return Err(IntersectionGraphValidationError::RetainedPairHasNoEvents);
        }

        match self.relation {
            MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping => {
                if self.projection.is_none() {
                    return Err(IntersectionGraphValidationError::CoplanarPairMissingProjection);
                }
            }
            MeshFacePairRelation::Candidate | MeshFacePairRelation::Unknown => {
                if self.projection.is_some() {
                    return Err(IntersectionGraphValidationError::NonCoplanarPairHasProjection);
                }
            }
            MeshFacePairRelation::PlaneSeparated => {}
        }

        if self.relation == MeshFacePairRelation::Unknown
            && !self
                .events
                .iter()
                .any(|event| matches!(event, IntersectionEvent::Unknown))
        {
            return Err(IntersectionGraphValidationError::UnknownPairMissingUnknownEvent);
        }

        for event in &self.events {
            match (self.relation, event) {
                (
                    MeshFacePairRelation::CoplanarTouching
                    | MeshFacePairRelation::CoplanarOverlapping,
                    IntersectionEvent::SegmentPlane { .. },
                ) => {
                    return Err(IntersectionGraphValidationError::CoplanarPairHasSegmentPlaneEvent);
                }
                (
                    MeshFacePairRelation::Candidate | MeshFacePairRelation::Unknown,
                    IntersectionEvent::CoplanarEdge { .. }
                    | IntersectionEvent::CoplanarVertex { .. },
                ) => {
                    return Err(IntersectionGraphValidationError::NonCoplanarPairHasCoplanarEvent);
                }
                _ => {}
            }
            validate_intersection_event(event)?;
        }
        Ok(())
    }

    /// Validate retained event handles against the exact source meshes.
    ///
    /// Plain [`FacePairEvents::validate`] checks relation/payload shape. Source
    /// validation also checks that every retained face, edge, and vertex handle
    /// still belongs to the source meshes and to this face pair.
    pub fn validate_against_meshes(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), IntersectionGraphValidationError> {
        self.validate()?;
        let left_tri = left
            .facts()
            .faces
            .get(self.left_face)
            .ok_or(IntersectionGraphValidationError::FaceIndexOutOfRange)?
            .triangle
            .vertices;
        let right_tri = right
            .facts()
            .faces
            .get(self.right_face)
            .ok_or(IntersectionGraphValidationError::FaceIndexOutOfRange)?
            .triangle
            .vertices;
        for event in &self.events {
            validate_intersection_event_sources(event, self, left, right, left_tri, right_tri)?;
        }
        Ok(())
    }

    /// Group retained coplanar events into a non-mutating overlap graph.
    ///
    /// The returned graph is a structural arrangement input: it records which
    /// projected edges and vertices participate in the coplanar contact while
    /// leaving exact split construction and cell extraction to later stages.
    pub fn coplanar_overlap_graph(
        &self,
    ) -> Result<Option<CoplanarOverlapGraph>, IntersectionGraphValidationError> {
        if !matches!(
            self.relation,
            MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping
        ) {
            return Ok(None);
        }
        let projection = self
            .projection
            .ok_or(IntersectionGraphValidationError::CoplanarPairMissingProjection)?;

        let mut edge_overlaps = Vec::with_capacity(self.events.len());
        let mut vertex_overlaps = Vec::with_capacity(self.events.len());
        for event in &self.events {
            match event {
                IntersectionEvent::CoplanarEdge {
                    left_edge,
                    right_edge,
                    relation,
                } if *relation != SegmentIntersection::Disjoint => {
                    edge_overlaps.push(CoplanarEdgeOverlap {
                        left_edge: *left_edge,
                        right_edge: *right_edge,
                        relation: *relation,
                    });
                }
                IntersectionEvent::CoplanarVertex {
                    vertex_side,
                    vertex,
                    triangle_side,
                    triangle_face,
                    location:
                        location @ (TriangleLocation::Inside
                        | TriangleLocation::OnEdge
                        | TriangleLocation::OnVertex),
                } => vertex_overlaps.push(CoplanarVertexOverlap {
                    vertex_side: *vertex_side,
                    vertex: *vertex,
                    triangle_side: *triangle_side,
                    triangle_face: *triangle_face,
                    location: *location,
                }),
                _ => {}
            }
        }

        Ok(Some(CoplanarOverlapGraph {
            left_face: self.left_face,
            right_face: self.right_face,
            relation: self.relation,
            projection,
            edge_overlaps,
            vertex_overlaps,
        }))
    }
}

pub(crate) fn intersection_graph_validation_error(
    error: IntersectionGraphValidationError,
    context: &str,
) -> ExactMeshError {
    let kind = match error {
        IntersectionGraphValidationError::FaceIndexOutOfRange
        | IntersectionGraphValidationError::EventSourceOutOfRange
        | IntersectionGraphValidationError::EventSourceMismatch
        | IntersectionGraphValidationError::SourceReplayMismatch => {
            ExactMeshBlockerKind::StaleFactReplay
        }
        IntersectionGraphValidationError::RetainedPairHasNoEvents
        | IntersectionGraphValidationError::UnknownPairMissingUnknownEvent
        | IntersectionGraphValidationError::CoplanarPairMissingProjection => {
            ExactMeshBlockerKind::MissingRequiredEvidence
        }
        IntersectionGraphValidationError::RejectedPairHasEvents
        | IntersectionGraphValidationError::NonCoplanarPairHasProjection
        | IntersectionGraphValidationError::CoplanarPairHasSegmentPlaneEvent
        | IntersectionGraphValidationError::NonCoplanarPairHasCoplanarEvent
        | IntersectionGraphValidationError::DisjointSegmentPlaneEvent
        | IntersectionGraphValidationError::InvalidSegmentPlaneEvent
        | IntersectionGraphValidationError::DisjointCoplanarEdgeEvent
        | IntersectionGraphValidationError::NonConstructiveCoplanarVertexEvent => {
            ExactMeshBlockerKind::ExactConstructionFailure
        }
    };
    ExactMeshError::one(ExactMeshBlocker::new(kind, format!("{context}: {error:?}")))
}

fn retained_coplanar_graph_error(
    error: IntersectionGraphValidationError,
    context: &'static str,
) -> ExactMeshError {
    intersection_graph_validation_error(
        error,
        &format!("retained coplanar overlap graph failed to {context}"),
    )
}

/// Exact intersection event graph for two meshes.
#[derive(Clone, Debug)]
pub(crate) struct ExactIntersectionGraph {
    /// Retained face-pair event records.
    pub face_pairs: Vec<FacePairEvents>,
    /// Whether this graph has replayed successfully against its source meshes.
    pub(crate) source_replay_validated: bool,
    summary: ExactIntersectionGraphSummary,
}

impl PartialEq for ExactIntersectionGraph {
    fn eq(&self, other: &Self) -> bool {
        self.face_pairs == other.face_pairs && self.summary == other.summary
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ExactIntersectionGraphSummary {
    event_count: usize,
    has_unknowns: bool,
    coplanar_overlap_graph_count: usize,
}

impl ExactIntersectionGraph {
    pub(crate) fn from_face_pairs(face_pairs: Vec<FacePairEvents>) -> Self {
        let mut event_count = 0;
        let mut has_unknowns = false;
        let mut coplanar_overlap_graph_count = 0;

        for pair in &face_pairs {
            event_count += pair.events.len();
            has_unknowns |= pair.relation == MeshFacePairRelation::Unknown
                || pair
                    .events
                    .iter()
                    .any(IntersectionEvent::has_unknown_relation);
            if matches!(
                pair.relation,
                MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping
            ) {
                coplanar_overlap_graph_count += 1;
            }
        }

        Self {
            face_pairs,
            source_replay_validated: false,
            summary: ExactIntersectionGraphSummary {
                event_count,
                has_unknowns,
                coplanar_overlap_graph_count,
            },
        }
    }

    /// Count all retained events.
    pub const fn event_count(&self) -> usize {
        self.summary.event_count
    }

    /// Return whether any retained pair still needs a policy decision or
    /// additional refinement.
    pub const fn has_unknowns(&self) -> bool {
        self.summary.has_unknowns
    }

    /// Validate all retained face-pair event records.
    ///
    /// Graph validation is the checked handoff between exact face-pair
    /// classification and split planning. It verifies that every retained
    /// event is structurally compatible with its coarse relation before edge
    /// parameters are sorted or graph vertices are merged.
    pub fn validate(&self) -> Result<(), IntersectionGraphValidationError> {
        for pair in &self.face_pairs {
            pair.validate()?;
        }
        Ok(())
    }

    /// Validate retained face-pair events against their source meshes.
    ///
    /// This is the graph-level source-aware handoff for downstream exact
    /// topology construction. It replays each retained event's source handles
    /// against the left/right meshes before any split ordering or planar-cell
    /// extraction consumes the graph.
    pub fn validate_against_meshes(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), IntersectionGraphValidationError> {
        for pair in &self.face_pairs {
            pair.validate_against_meshes(left, right)?;
        }
        Ok(())
    }

    /// Validate this graph by checking retained source handles, then replaying
    /// it from source operands unless it already carries a current certificate.
    ///
    /// [`Self::validate_against_meshes`] checks that retained event handles
    /// still belong to `left` and `right`. A current replay certificate means
    /// the graph was either built from those operands or retained through a
    /// prepared pair source-stamp check. Uncertified source replay rebuilds the
    /// graph from those operands and requires the same retained face-pair
    /// records. Pair traversal order is an acceleration detail, so source
    /// replay compares by source face handles instead of by vector position.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), IntersectionGraphValidationError> {
        self.validate_against_meshes(left, right)?;
        if self.source_replay_validated {
            return Ok(());
        }
        let replay = build_unvalidated_intersection_graph(left, right)
            .map_err(|_| IntersectionGraphValidationError::SourceReplayMismatch)?;
        if intersection_graphs_have_same_face_pair_records(self, &replay) {
            Ok(())
        } else {
            Err(IntersectionGraphValidationError::SourceReplayMismatch)
        }
    }

    /// Extract grouped coplanar overlap graphs from retained face-pair records.
    pub(crate) fn coplanar_overlap_graphs(
        &self,
    ) -> Result<Vec<CoplanarOverlapGraph>, IntersectionGraphValidationError> {
        let mut graphs = Vec::with_capacity(self.summary.coplanar_overlap_graph_count);
        for pair in &self.face_pairs {
            if let Some(graph) = pair.coplanar_overlap_graph()? {
                graphs.push(graph);
            }
        }
        Ok(graphs)
    }

    /// Construct exact split-point/interval records for coplanar overlap graphs.
    pub fn coplanar_overlap_split_plan(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<CoplanarOverlapSplitPlan, ExactMeshError> {
        let graphs = self
            .coplanar_overlap_graphs()
            .map_err(|error| retained_coplanar_graph_error(error, "extract split plan"))?;
        let mut split_graphs = Vec::with_capacity(graphs.len());
        for graph in graphs {
            split_graphs.push(coplanar_overlap_split_graph(&graph, left, right)?);
        }
        Ok(CoplanarOverlapSplitPlan {
            graphs: split_graphs,
        })
    }

    /// Summarize retained coplanar overlap evidence for planar-cell extraction.
    ///
    /// The report first validates each retained overlap graph and its split
    /// construction records, then collapses them to counts that explain whether
    /// a named operation is blocked on boundary-only contact or true planar-cell
    /// evidence is preserved and checked, while the missing cell extraction
    /// algorithm remains an explicit status rather than a tolerance fallback.
    pub(crate) fn coplanar_arrangement_evidence(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<CoplanarArrangementEvidence, ExactMeshError> {
        // Coplanar arrangement evidence is a compact view of retained graph state.
        // Before collapsing counts, replay the graph's face/edge/vertex handles
        // against the source meshes and later replay split parameters against
        // state; stale handles must not survive simply because the summary
        // counters are internally coherent.
        self.validate_against_meshes(left, right).map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("retained coplanar arrangement graph failed source replay: {error:?}"),
            ))
        })?;
        let mut graph_count = 0;
        let mut overlapping_graphs = 0;
        let mut touching_graphs = 0;
        let mut edge_overlap_count = 0;
        let mut vertex_overlap_count = 0;
        let mut point_split_count = 0;
        let mut interval_overlap_count = 0;
        let mut interval_endpoint_count = 0;

        let graphs = self
            .coplanar_overlap_graphs()
            .map_err(|error| retained_coplanar_graph_error(error, "summarize evidence"))?;
        for graph in graphs {
            graph_count += 1;
            graph.validate().map_err(|_| ExactMeshError {
                blockers: vec![ExactMeshBlocker::new(
                    ExactMeshBlockerKind::ExactConstructionFailure,
                    "retained coplanar overlap graph failed evidence validation",
                )],
            })?;
            match graph.relation {
                MeshFacePairRelation::CoplanarOverlapping => overlapping_graphs += 1,
                MeshFacePairRelation::CoplanarTouching => touching_graphs += 1,
                _ => {}
            }
            edge_overlap_count += graph.edge_overlaps.len();
            vertex_overlap_count += graph.vertex_overlaps.len();

            let split = coplanar_overlap_split_graph(&graph, left, right)?;
            split
                .validate_against_meshes(left, right)
                .map_err(|error| {
                    ExactMeshError::one(ExactMeshBlocker::new(
                        ExactMeshBlockerKind::StaleFactReplay,
                        format!(
                            "retained coplanar split construction failed source replay: {error:?}"
                        ),
                    ))
                })?;
            for edge_split in split.edge_splits {
                point_split_count += edge_split.points.len();
                if edge_split.interval_overlap {
                    interval_overlap_count += 1;
                    if let Some(interval) = &edge_split.interval {
                        interval_endpoint_count += interval.endpoints.len();
                    }
                }
            }
        }

        let status = if overlapping_graphs > 0 {
            CoplanarArrangementEvidenceStatus::NeedsPlanarCells
        } else if graph_count > 0 {
            CoplanarArrangementEvidenceStatus::BoundaryOnly
        } else {
            CoplanarArrangementEvidenceStatus::NoCoplanarOverlap
        };
        let report = CoplanarArrangementEvidence {
            status,
            graph_count,
            overlapping_graphs,
            touching_graphs,
            edge_overlap_count,
            vertex_overlap_count,
            point_split_count,
            interval_overlap_count,
            interval_endpoint_count,
        };
        report.validate().map_err(|_| ExactMeshError {
            blockers: vec![ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                "coplanar arrangement evidence failed validation",
            )],
        })?;
        Ok(report)
    }

    /// Build a non-mutating split-topology plan after validating split events.
    ///
    /// This checked entry point enforces the edge-split handoff contract before
    /// graph-vertex merging. It is the preferred path for production exact
    /// boolean topology because it rejects missing side facts, non-crossing
    /// split facts, and uncertified edge ordering before later stages can
    /// constructions become topology only after their combinatorial
    /// assumptions have been validated.
    pub fn checked_split_topology_plan(
        &self,
    ) -> Result<ExactSplitTopologyPlan, SplitPlanValidationReport> {
        let edge_splits = edge_split_plan(self);
        let edge_report = validate_edge_split_plan(&edge_splits);
        if !edge_report.blockers.is_empty() {
            return Err(edge_report);
        }
        let graph_vertices = graph_vertex_plan(&edge_splits);
        let graph_report = {
            let mut blockers = Vec::new();
            for _ in 0..graph_vertices.unresolved_equalities {
                blockers.push(SplitPlanBlocker::new(
                    SplitPlanBlockerKind::UnresolvedEquality,
                    "graph-vertex equality could not be certified",
                ));
            }
            for index in 0..graph_vertices.vertices.len() {
                let vertex = &graph_vertices.vertices[index];
                if vertex.uses.is_empty() {
                    blockers.push(SplitPlanBlocker {
                        graph_vertex: Some(index),
                        ..SplitPlanBlocker::new(
                            SplitPlanBlockerKind::EmptyGraphVertexUses,
                            "graph vertex has no exact source uses",
                        )
                    });
                    continue;
                }
                for vertex_use in &vertex.uses {
                    push_graph_vertex_source_use_blockers(
                        &mut blockers,
                        index,
                        vertex_use,
                        "graph vertex source use determinant ratio does not match its parameter",
                        "graph vertex source use was not certified by opposite strict endpoint sides",
                        "graph vertex source use is missing endpoint side facts",
                    );
                }
            }
            SplitPlanValidationReport { blockers }
        };
        if !graph_report.blockers.is_empty() {
            return Err(graph_report);
        }
        let topology = split_topology_plan(&edge_splits, &graph_vertices);
        let topology_report = validate_split_topology_plan(&topology);
        if topology_report.blockers.is_empty() {
            Ok(topology)
        } else {
            Err(topology_report)
        }
    }

    /// Build exact face-boundary geometry for later triangulation.
    ///
    /// This resolves face split work items into original and constructed
    /// boundary nodes with exact coordinates. It remains a pre-mutation handoff:
    /// no halfedges are created and no winding decision is inferred here. The
    /// predicates and constructions are validated before combinatorial edits.
    pub fn face_split_geometry_plan(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactFaceSplitGeometryPlan, ExactMeshError> {
        let topology = self
            .checked_split_topology_plan()
            .map_err(split_plan_report_to_mesh_error)?;
        let face_plan = face_split_plan(&topology);
        let face_report = validate_face_split_plan(&face_plan, &topology);
        if !face_report.blockers.is_empty() {
            return Err(split_plan_report_to_mesh_error(face_report));
        }
        face_split_geometry_plan(left, right, &topology, &face_plan)
    }
}

fn intersection_graphs_have_same_face_pair_records(
    retained: &ExactIntersectionGraph,
    replay: &ExactIntersectionGraph,
) -> bool {
    if retained.face_pairs.len() != replay.face_pairs.len() {
        return false;
    }
    let mut retained_pairs = BTreeMap::new();
    for pair in &retained.face_pairs {
        if retained_pairs
            .insert((pair.left_face, pair.right_face), pair)
            .is_some()
        {
            return false;
        }
    }
    for pair in &replay.face_pairs {
        let Some(retained_pair) = retained_pairs.remove(&(pair.left_face, pair.right_face)) else {
            return false;
        };
        if retained_pair != pair {
            return false;
        }
    }
    retained_pairs.is_empty()
}

/// Build an exact event graph from two exact meshes without validating the
/// retained event handles against source replay.
///
/// This is for replay comparisons and tests that intentionally retain stale
/// graphs. Ordinary algorithm consumers should use
/// [`build_validated_intersection_graph`].
pub(crate) fn build_unvalidated_intersection_graph(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactIntersectionGraph, ExactMeshError> {
    if let Err(error) = left.validate_retained_bounds_certificate() {
        return Err(ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::StaleFactReplay,
            format!("exact mesh retained broad-phase certificate failed: {error:?}"),
        )));
    }
    if let Err(error) = right.validate_retained_bounds_certificate() {
        return Err(ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::StaleFactReplay,
            format!("exact mesh retained broad-phase certificate failed: {error:?}"),
        )));
    }
    let mut face_pairs = Vec::new();
    try_visit_candidate_face_pairs_one_shot(left.bounds(), right.bounds(), &mut |[
        left_face,
        right_face,
    ]| {
        let classification = classify_mesh_face_pair_unchecked(left, left_face, right, right_face);
        if classification.needs_graph_construction() {
            face_pairs.push(events_for_face_pair(left, right, &classification));
        }
        Ok::<(), ExactMeshError>(())
    })?;
    Ok(ExactIntersectionGraph::from_face_pairs(face_pairs))
}

/// Build an exact event graph and replay it against the source meshes before use.
pub(crate) fn build_validated_intersection_graph(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactIntersectionGraph, ExactMeshError> {
    let mut graph = build_unvalidated_intersection_graph(left, right)?;
    graph
        .validate_against_meshes(left, right)
        .map_err(|error| {
            intersection_graph_validation_error(
                error,
                "exact intersection graph failed source replay",
            )
        })?;
    graph.source_replay_validated = true;
    Ok(graph)
}

/// Build a shared exact event graph from a retained prepared pair session.
pub(crate) fn build_unvalidated_intersection_graph_from_prepared_pair_rc(
    pair: &PreparedMeshPair<'_, '_>,
) -> Result<Rc<ExactIntersectionGraph>, ExactMeshError> {
    if let Some(graph) = pair.retained_intersection_graph_for_validation()? {
        return Ok(graph);
    }

    let left = pair.left_view.mesh;
    let right = pair.right_view.mesh;
    let mut face_pairs = Vec::with_capacity(pair.candidate_pair_capacity_hint);
    pair.try_visit_candidate_face_pairs_uncached(&mut |[left_face, right_face]| {
        let classification = classify_mesh_face_pair_unchecked(left, left_face, right, right_face);
        if classification.needs_graph_construction() {
            face_pairs.push(events_for_face_pair(left, right, &classification));
        }
        Ok::<(), ExactMeshError>(())
    })?;
    let graph = ExactIntersectionGraph::from_face_pairs(face_pairs);
    Ok(pair.retain_intersection_graph(graph))
}

/// Exact split points for one directed mesh edge.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct EdgeSplit {
    /// Mesh side owning the edge.
    pub side: MeshSide,
    /// Directed edge endpoints in that mesh's vertex index space.
    pub edge: [usize; 2],
    /// Ordered split points when exact parameter comparisons were available.
    pub points: Vec<EdgeSplitPoint>,
}

/// One exact split point on an edge.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct EdgeSplitPoint {
    /// Face pair that produced the split.
    pub face_pair: [usize; 2],
    /// Opposite face whose plane produced the split.
    pub plane_face: usize,
    /// Exact parameter on the directed edge.
    pub parameter: Real,
    /// Determinant ratio that produced [`Self::parameter`].
    pub parameter_ratio: SegmentPlaneParameterRatio,
    /// Exact constructed point.
    pub point: Point3,
    /// Endpoint side facts that certified this split event.
    pub endpoint_sides: [Option<PlaneSide>; 2],
}

/// Edge split extraction result.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactEdgeSplitPlan {
    /// Per-edge split points.
    pub splits: Vec<EdgeSplit>,
    /// Number of parameter comparisons that could not be certified.
    pub unknown_orderings: usize,
}

/// One merged exact graph vertex.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactGraphVertex {
    /// Representative exact point.
    pub point: Point3,
    /// Split-point uses that are exactly coincident with the representative.
    pub uses: Vec<ExactGraphVertexUse>,
}

/// One source use of a merged graph vertex.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactGraphVertexUse {
    /// Mesh side owning the split edge.
    pub side: MeshSide,
    /// Directed edge endpoints in that mesh's vertex index space.
    pub edge: [usize; 2],
    /// Face pair that produced the split.
    pub face_pair: [usize; 2],
    /// Opposite face whose plane produced the split.
    pub plane_face: usize,
    /// Exact parameter on the directed edge for this source use.
    pub parameter: Real,
    /// Determinant ratio that produced [`Self::parameter`].
    pub parameter_ratio: SegmentPlaneParameterRatio,
    /// Endpoint side facts that certified this source use.
    pub endpoint_sides: [Option<PlaneSide>; 2],
}

/// Exact graph-vertex merge result.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactGraphVertexPlan {
    /// Merged graph vertices.
    pub vertices: Vec<ExactGraphVertex>,
    /// Equality checks that could not be certified.
    pub unresolved_equalities: usize,
}

/// One node in an ordered split-edge chain.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum SplitEdgeNode {
    /// Original mesh vertex.
    OriginalVertex {
        /// Mesh side owning the original vertex.
        side: MeshSide,
        /// Vertex index in that mesh.
        vertex: usize,
    },
    /// Merged exact graph vertex.
    GraphVertex {
        /// Index in [`ExactSplitTopologyPlan::graph_vertices`].
        graph_vertex: usize,
    },
}

/// Ordered split chain for one original edge.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct SplitEdgeChain {
    /// Mesh side owning the edge.
    pub side: MeshSide,
    /// Directed original edge.
    pub edge: [usize; 2],
    /// Chain from original start through split graph vertices to original end.
    pub nodes: Vec<SplitEdgeNode>,
}

/// Non-mutating exact split topology plan.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactSplitTopologyPlan {
    /// Merged exact graph vertices used by edge chains.
    pub graph_vertices: Vec<ExactGraphVertex>,
    /// Ordered edge chains to materialize.
    pub edge_chains: Vec<SplitEdgeChain>,
    /// Number of split points that could not be matched back to a graph vertex.
    pub unresolved_vertex_lookups: usize,
    /// Number of equality checks that could not be certified while merging.
    pub unresolved_equalities: usize,
    /// Number of edge parameter comparisons that could not be certified.
    pub unknown_orderings: usize,
}

/// One split edge chain as used by an affected face.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct FaceSplitEdge {
    /// Original face edge endpoints.
    pub edge: [usize; 2],
    /// Graph vertices on that edge in directed edge order.
    pub graph_vertices: Vec<usize>,
}

/// Face-local split work item.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct FaceSplitPlan {
    /// Mesh side owning the face.
    pub side: MeshSide,
    /// Face index.
    pub face: usize,
    /// Split boundary edges for this face.
    pub edges: Vec<FaceSplitEdge>,
}

/// Non-mutating exact face split plan.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactFaceSplitPlan {
    /// Per-face split work items.
    pub faces: Vec<FaceSplitPlan>,
}

/// Stable category for split-plan validation blockers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum SplitPlanBlockerKind {
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
    /// A public split-region artifact no longer matches source replay.
    #[cfg(test)]
    SourceReplayMismatch,
    /// A retained split face or region carried a source triangle that no
    /// longer matches the retained source face handle.
    SourceTriangleMismatch,
    /// A split face region has fewer than three boundary nodes.
    EmptyOrShortRegionBoundary,
    /// A split face region contains consecutive duplicate boundary nodes.
    DuplicateConsecutiveRegionNode,
    /// A split-boundary chain references an edge that is not on the retained
    /// source triangle.
    BoundaryChainEdgeNotOnTriangle,
    /// A retained boundary original node references a missing source vertex.
    BoundaryNodeSourceVertexOutOfRange,
    /// A retained boundary original node is not part of its source triangle.
    BoundaryNodeSourceVertexNotOnTriangle,
    /// A retained boundary original node point no longer matches its source
    /// vertex coordinate.
    BoundaryNodeSourcePointMismatch,
}

/// One split-plan validation blocker.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct SplitPlanBlocker {
    /// Stable blocker category.
    pub kind: SplitPlanBlockerKind,
    /// Human-readable detail.
    pub message: String,
    /// Optional mesh side.
    pub side: Option<MeshSide>,
    /// Optional face index.
    pub face: Option<usize>,
    /// Optional directed edge.
    pub edge: Option<[usize; 2]>,
    /// Optional graph-vertex index.
    pub graph_vertex: Option<usize>,
}

impl SplitPlanBlocker {
    fn new(kind: SplitPlanBlockerKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
            side: None,
            face: None,
            edge: None,
            graph_vertex: None,
        }
    }
}

fn split_plan_report_to_mesh_error(report: SplitPlanValidationReport) -> ExactMeshError {
    ExactMeshError::new(
        report
            .blockers
            .into_iter()
            .map(|blocker| {
                let kind = match blocker.kind {
                    SplitPlanBlockerKind::UnknownOrdering
                    | SplitPlanBlockerKind::UnresolvedEquality
                    | SplitPlanBlockerKind::UnknownBoundaryIncidence => {
                        ExactMeshBlockerKind::UndecidablePredicate
                    }
                    #[cfg(test)]
                    SplitPlanBlockerKind::SourceReplayMismatch => {
                        ExactMeshBlockerKind::StaleFactReplay
                    }
                    SplitPlanBlockerKind::SourceTriangleMismatch
                    | SplitPlanBlockerKind::BoundaryNodeSourceVertexOutOfRange
                    | SplitPlanBlockerKind::BoundaryNodeSourceVertexNotOnTriangle
                    | SplitPlanBlockerKind::BoundaryNodeSourcePointMismatch => {
                        ExactMeshBlockerKind::StaleFactReplay
                    }
                    SplitPlanBlockerKind::UnresolvedVertexLookup
                    | SplitPlanBlockerKind::MissingEndpointSideFacts
                    | SplitPlanBlockerKind::NonCrossingEndpointSideFacts
                    | SplitPlanBlockerKind::InvalidConstructionRatio
                    | SplitPlanBlockerKind::EmptyOrShortEdgeChain
                    | SplitPlanBlockerKind::WrongChainStart
                    | SplitPlanBlockerKind::WrongChainEnd
                    | SplitPlanBlockerKind::ChainSideMismatch
                    | SplitPlanBlockerKind::GraphVertexOutOfRange
                    | SplitPlanBlockerKind::EmptyGraphVertexUses
                    | SplitPlanBlockerKind::EmptyFaceSplit
                    | SplitPlanBlockerKind::EmptyFaceSplitEdge
                    | SplitPlanBlockerKind::DuplicateFaceSplitEdge
                    | SplitPlanBlockerKind::MissingFaceSplitSourceUse
                    | SplitPlanBlockerKind::BoundaryNodeOffFacePlane
                    | SplitPlanBlockerKind::EmptyOrShortRegionBoundary
                    | SplitPlanBlockerKind::DuplicateConsecutiveRegionNode
                    | SplitPlanBlockerKind::BoundaryChainEdgeNotOnTriangle => {
                        ExactMeshBlockerKind::ExactConstructionFailure
                    }
                };
                let mut mesh = ExactMeshBlocker::new(kind, blocker.message);
                if let Some(side) = blocker.side {
                    mesh = mesh.with_source_side(match side {
                        MeshSide::Left => ExactMeshSourceSide::Left,
                        MeshSide::Right => ExactMeshSourceSide::Right,
                    });
                }
                if let Some(face) = blocker.face {
                    mesh = mesh.with_face(face);
                }
                if let Some(edge) = blocker.edge {
                    mesh = mesh.with_edge(edge);
                }
                mesh
            })
            .collect(),
    )
}

/// Validation report for exact split topology and face split plans.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct SplitPlanValidationReport {
    /// Blockers collected during validation.
    pub(crate) blockers: Vec<SplitPlanBlocker>,
}

/// Exact boundary node for a split face.
///
/// The variants distinguish original source vertices, retained intersection
/// graph vertices, and later exact face-interior constructions. Keeping those
/// explicit construction evidence instead of relabeling coordinates as if they
/// came from an older source object.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum FaceSplitBoundaryNode {
    /// Original mesh vertex with its exact point.
    OriginalVertex {
        /// Vertex index in the source mesh.
        vertex: usize,
        /// Exact point carried into the split boundary.
        point: Point3,
    },
    /// Constructed graph vertex with its exact point.
    GraphVertex {
        /// Index in [`ExactSplitTopologyPlan::graph_vertices`].
        graph_vertex: usize,
        /// Exact constructed point.
        point: Point3,
    },
    /// Exact point constructed in the interior of a source face.
    ///
    /// This is used when constrained planar cell subdivision appends a Steiner
    /// vertex at an exact constraint crossing. The point is not an original
    /// mesh vertex and not a global intersection-graph vertex; it is a local
    /// source-face witness whose incidence must still replay against the
    /// owning face before region assembly consumes it.
    FaceInterior {
        /// Exact constructed point on the source face.
        point: Point3,
    },
}

/// Exact boundary chain for one split edge of an original face.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FaceSplitBoundaryChain {
    /// Original directed face edge.
    pub edge: [usize; 2],
    /// Boundary nodes in directed edge order.
    pub nodes: Vec<FaceSplitBoundaryNode>,
}

/// Exact geometry handoff for one split face.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FaceSplitGeometry {
    /// Mesh side owning the face.
    pub side: MeshSide,
    /// Face index in the owning mesh.
    pub face: usize,
    /// Original triangle vertices.
    pub triangle: [usize; 3],
    /// Boundary chains that contain exact graph vertices.
    pub boundary_chains: Vec<FaceSplitBoundaryChain>,
}

/// Non-mutating exact split-face geometry plan.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactFaceSplitGeometryPlan {
    /// Per-face exact boundary geometry.
    pub faces: Vec<FaceSplitGeometry>,
}

impl ExactFaceSplitGeometryPlan {
    /// Build full face-region boundary loops for downstream exact triangulation.
    ///
    /// The geometry handoff stores only split edge chains. This method expands
    /// each affected triangle into one boundary loop in original face-edge
    /// order, inserting exact graph vertices along the split edges. It still
    /// does not decide winding, ownership, or boolean output; those decisions
    /// computation separation.
    pub fn region_plan(&self, left: &ExactMesh, right: &ExactMesh) -> ExactFaceRegionPlan {
        let mut regions = Vec::with_capacity(self.faces.len());
        for face in &self.faces {
            let mesh = face.side.mesh(left, right);
            let triangle = face.triangle;
            let mut chains = face
                .boundary_chains
                .iter()
                .map(|chain| ((chain.edge[0], chain.edge[1]), chain))
                .collect::<BTreeMap<_, _>>();
            let mut boundary = Vec::new();

            for edge in triangle_edges(triangle) {
                let nodes = if let Some(chain) = chains.remove(&(edge[0], edge[1])) {
                    chain.nodes.clone()
                } else {
                    vec![
                        FaceSplitBoundaryNode::OriginalVertex {
                            vertex: edge[0],
                            point: mesh
                                .view()
                                .vertex(edge[0])
                                .expect("region plan references a missing source edge start")
                                .point()
                                .clone(),
                        },
                        FaceSplitBoundaryNode::OriginalVertex {
                            vertex: edge[1],
                            point: mesh
                                .view()
                                .vertex(edge[1])
                                .expect("region plan references a missing source edge end")
                                .point()
                                .clone(),
                        },
                    ]
                };
                for node in nodes {
                    if boundary
                        .last()
                        .is_none_or(|last| boundary_nodes_equal(last, &node) != Some(true))
                    {
                        boundary.push(node);
                    }
                }
            }
            if boundary
                .first()
                .zip(boundary.last())
                .is_some_and(|(first, last)| boundary_nodes_equal(first, last) == Some(true))
            {
                boundary.pop();
            }

            regions.push(FaceRegionBoundary {
                side: face.side,
                face: face.face,
                triangle,
                boundary,
            });
        }

        ExactFaceRegionPlan { regions }
    }
}

/// One pre-triangulation boundary loop for an affected face.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FaceRegionBoundary {
    /// Mesh side owning the source face.
    pub side: MeshSide,
    /// Face index in the source mesh.
    pub face: usize,
    /// Original triangle vertices.
    pub triangle: [usize; 3],
    /// Boundary loop in source triangle order, with split graph vertices
    /// inserted along each affected edge.
    pub boundary: Vec<FaceSplitBoundaryNode>,
}

/// Exact pre-triangulation region plan for affected faces.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactFaceRegionPlan {
    /// One boundary loop per affected source face.
    pub regions: Vec<FaceRegionBoundary>,
}

fn events_for_face_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    classification: &MeshFacePairClassification,
) -> FacePairEvents {
    let left_tri = left
        .view()
        .face(classification.left_face)
        .expect("face-pair event generation references a missing left face")
        .vertex_indices();
    let right_tri = right
        .view()
        .face(classification.right_face)
        .expect("face-pair event generation references a missing right face")
        .vertex_indices();
    let left_edges = triangle_edges(left_tri);
    let right_edges = triangle_edges(right_tri);
    let mut event_capacity = usize::from(classification.relation == MeshFacePairRelation::Unknown);
    if let Some(triangle) = &classification.triangle {
        event_capacity += triangle
            .right_edge_events
            .as_ref()
            .map_or(0, |events| events.len());
        event_capacity += triangle
            .left_edge_events
            .as_ref()
            .map_or(0, |events| events.len());
        if let Some(coplanar) = &triangle.coplanar {
            event_capacity += coplanar.edge_intersections.len();
            event_capacity += coplanar.right_vertices_in_left.len();
            event_capacity += coplanar.left_vertices_in_right.len();
        }
    }
    let mut events = Vec::with_capacity(event_capacity);
    let mut projection = None;

    if let Some(triangle) = &classification.triangle {
        append_segment_plane_events(
            &mut events,
            MeshSide::Right,
            &right_edges,
            MeshSide::Left,
            classification.left_face,
            triangle
                .right_edge_events
                .as_ref()
                .map_or(&[], |events| events.as_slice()),
        );
        append_segment_plane_events(
            &mut events,
            MeshSide::Left,
            &left_edges,
            MeshSide::Right,
            classification.right_face,
            triangle
                .left_edge_events
                .as_ref()
                .map_or(&[], |events| events.as_slice()),
        );

        if let Some(coplanar) = &triangle.coplanar {
            projection = coplanar.projection;
            append_coplanar_events(
                &mut events,
                classification.left_face,
                classification.right_face,
                left_tri,
                right_tri,
                &left_edges,
                &right_edges,
                coplanar,
            );
        }
    }

    if classification.relation == MeshFacePairRelation::Unknown {
        events.push(IntersectionEvent::Unknown);
    }

    FacePairEvents {
        left_face: classification.left_face,
        right_face: classification.right_face,
        relation: classification.relation,
        projection,
        events,
    }
}

fn edge_split_plan(graph: &ExactIntersectionGraph) -> ExactEdgeSplitPlan {
    let mut grouped = BTreeMap::<(MeshSide, usize, usize), EdgeSplit>::new();
    for pair in &graph.face_pairs {
        for event in &pair.events {
            let IntersectionEvent::SegmentPlane {
                segment_side,
                edge,
                plane_face,
                point: Some(point),
                parameter: Some(parameter),
                parameter_ratio: Some(parameter_ratio),
                endpoint_sides,
                ..
            } = event
            else {
                continue;
            };
            let key = (*segment_side, edge[0], edge[1]);
            grouped
                .entry(key)
                .or_insert_with(|| EdgeSplit {
                    side: *segment_side,
                    edge: *edge,
                    points: Vec::with_capacity(1),
                })
                .points
                .push(EdgeSplitPoint {
                    face_pair: [pair.left_face, pair.right_face],
                    plane_face: *plane_face,
                    parameter: parameter.clone(),
                    parameter_ratio: parameter_ratio.clone(),
                    point: point.clone(),
                    endpoint_sides: *endpoint_sides,
                });
        }
    }

    let mut unknown_orderings = 0;
    let mut splits = grouped.into_values().collect::<Vec<_>>();
    for split in &mut splits {
        split.points.sort_by(|left, right| {
            match compare_reals(&left.parameter, &right.parameter).value() {
                Some(ordering) => ordering,
                None => {
                    unknown_orderings += 1;
                    Ordering::Equal
                }
            }
        });
    }
    ExactEdgeSplitPlan {
        splits,
        unknown_orderings,
    }
}

fn graph_vertex_plan(split_plan: &ExactEdgeSplitPlan) -> ExactGraphVertexPlan {
    let split_point_count = split_plan
        .splits
        .iter()
        .map(|split| split.points.len())
        .sum();
    let mut vertices = Vec::<ExactGraphVertex>::with_capacity(split_point_count);
    let mut point_key_buckets = BTreeMap::<ExactPoint3Key, Vec<usize>>::new();
    let mut unkeyed_vertices = Vec::<usize>::with_capacity(split_point_count);
    let mut unresolved_equalities = 0;

    for split in &split_plan.splits {
        for point in &split.points {
            let vertex_use = ExactGraphVertexUse {
                side: split.side,
                edge: split.edge,
                face_pair: point.face_pair,
                plane_face: point.plane_face,
                parameter: point.parameter.clone(),
                parameter_ratio: point.parameter_ratio.clone(),
                endpoint_sides: point.endpoint_sides,
            };

            let point_key = exact_point3_key(&point.point);
            let matched = 'matching_vertex: {
                if let Some(key) = point_key.as_ref() {
                    if let Some(bucket) = point_key_buckets.get(key) {
                        for &index in bucket {
                            match point3_exact_equal(&point.point, &vertices[index].point) {
                                Some(true) => break 'matching_vertex Some(index),
                                Some(false) => {}
                                None => unresolved_equalities += 1,
                            }
                        }
                    }
                    for &index in &unkeyed_vertices {
                        match point3_exact_equal(&point.point, &vertices[index].point) {
                            Some(true) => break 'matching_vertex Some(index),
                            Some(false) => {}
                            None => unresolved_equalities += 1,
                        }
                    }
                    None
                } else {
                    for bucket in point_key_buckets.values() {
                        for &index in bucket {
                            match point3_exact_equal(&point.point, &vertices[index].point) {
                                Some(true) => break 'matching_vertex Some(index),
                                Some(false) => {}
                                None => unresolved_equalities += 1,
                            }
                        }
                    }
                    for &index in &unkeyed_vertices {
                        match point3_exact_equal(&point.point, &vertices[index].point) {
                            Some(true) => break 'matching_vertex Some(index),
                            Some(false) => {}
                            None => unresolved_equalities += 1,
                        }
                    }
                    None
                }
            };

            if let Some(index) = matched {
                vertices[index].uses.push(vertex_use);
            } else {
                let index = vertices.len();
                if let Some(key) = point_key {
                    point_key_buckets.entry(key).or_default().push(index);
                } else {
                    unkeyed_vertices.push(index);
                }
                vertices.push(ExactGraphVertex {
                    point: point.point.clone(),
                    uses: vec![vertex_use],
                });
            }
        }
    }

    ExactGraphVertexPlan {
        vertices,
        unresolved_equalities,
    }
}

fn push_graph_vertex_source_use_blockers(
    blockers: &mut Vec<SplitPlanBlocker>,
    graph_vertex: usize,
    vertex_use: &ExactGraphVertexUse,
    ratio_message: &'static str,
    non_crossing_message: &'static str,
    missing_message: &'static str,
) {
    // construction object, not only the rounded coordinate it produced. Every
    // later graph/topology handoff therefore rechecks the determinant ratio and
    if !ratio_matches_parameter(&vertex_use.parameter_ratio, &vertex_use.parameter) {
        blockers.push(SplitPlanBlocker {
            graph_vertex: Some(graph_vertex),
            side: Some(vertex_use.side),
            edge: Some(vertex_use.edge),
            ..SplitPlanBlocker::new(
                SplitPlanBlockerKind::InvalidConstructionRatio,
                ratio_message,
            )
        });
    }

    match vertex_use.endpoint_sides {
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
        | [Some(PlaneSide::Below), Some(PlaneSide::Above)] => {}
        [Some(_), Some(_)] => blockers.push(SplitPlanBlocker {
            graph_vertex: Some(graph_vertex),
            side: Some(vertex_use.side),
            edge: Some(vertex_use.edge),
            ..SplitPlanBlocker::new(
                SplitPlanBlockerKind::NonCrossingEndpointSideFacts,
                non_crossing_message,
            )
        }),
        _ => blockers.push(SplitPlanBlocker {
            graph_vertex: Some(graph_vertex),
            side: Some(vertex_use.side),
            edge: Some(vertex_use.edge),
            ..SplitPlanBlocker::new(
                SplitPlanBlockerKind::MissingEndpointSideFacts,
                missing_message,
            )
        }),
    }
}

fn validate_intersection_event(
    event: &IntersectionEvent,
) -> Result<(), IntersectionGraphValidationError> {
    match event {
        IntersectionEvent::SegmentPlane {
            relation,
            point,
            parameter,
            parameter_ratio,
            construction_failure,
            endpoint_sides,
            ..
        } => validate_graph_segment_plane_event(
            *relation,
            point,
            parameter,
            parameter_ratio,
            construction_failure,
            *endpoint_sides,
        ),
        IntersectionEvent::CoplanarEdge { relation, .. } => {
            if *relation == SegmentIntersection::Disjoint {
                Err(IntersectionGraphValidationError::DisjointCoplanarEdgeEvent)
            } else {
                Ok(())
            }
        }
        IntersectionEvent::CoplanarVertex { location, .. } => match location {
            TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex => {
                Ok(())
            }
            TriangleLocation::Outside | TriangleLocation::Degenerate => {
                Err(IntersectionGraphValidationError::NonConstructiveCoplanarVertexEvent)
            }
        },
        IntersectionEvent::Unknown => Ok(()),
    }
}

fn validate_intersection_event_sources(
    event: &IntersectionEvent,
    pair: &FacePairEvents,
    left: &ExactMesh,
    right: &ExactMesh,
    left_tri: [usize; 3],
    right_tri: [usize; 3],
) -> Result<(), IntersectionGraphValidationError> {
    match event {
        IntersectionEvent::SegmentPlane {
            segment_side,
            edge,
            plane_side,
            plane_face,
            ..
        } => {
            if segment_side == plane_side {
                return Err(IntersectionGraphValidationError::EventSourceMismatch);
            }
            let (segment_tri, plane_pair_face, plane_mesh) = match (*segment_side, *plane_side) {
                (MeshSide::Left, MeshSide::Right) => (left_tri, pair.right_face, right),
                (MeshSide::Right, MeshSide::Left) => (right_tri, pair.left_face, left),
                _ => return Err(IntersectionGraphValidationError::EventSourceMismatch),
            };
            if *plane_face != plane_pair_face {
                return Err(IntersectionGraphValidationError::EventSourceMismatch);
            }
            if plane_mesh.view().face(*plane_face).is_err() {
                return Err(IntersectionGraphValidationError::EventSourceOutOfRange);
            }
            validate_vertex(*segment_side, edge[0], left, right)?;
            validate_vertex(*segment_side, edge[1], left, right)?;
            if edge[0] == edge[1]
                || !segment_tri.contains(&edge[0])
                || !segment_tri.contains(&edge[1])
            {
                return Err(IntersectionGraphValidationError::EventSourceMismatch);
            }
            Ok(())
        }
        IntersectionEvent::CoplanarEdge {
            left_edge,
            right_edge,
            ..
        } => {
            validate_vertex(MeshSide::Left, left_edge[0], left, right)?;
            validate_vertex(MeshSide::Left, left_edge[1], left, right)?;
            validate_vertex(MeshSide::Right, right_edge[0], left, right)?;
            validate_vertex(MeshSide::Right, right_edge[1], left, right)?;
            if left_edge[0] == left_edge[1]
                || !left_tri.contains(&left_edge[0])
                || !left_tri.contains(&left_edge[1])
                || right_edge[0] == right_edge[1]
                || !right_tri.contains(&right_edge[0])
                || !right_tri.contains(&right_edge[1])
            {
                return Err(IntersectionGraphValidationError::EventSourceMismatch);
            }
            Ok(())
        }
        IntersectionEvent::CoplanarVertex {
            vertex_side,
            vertex,
            triangle_side,
            triangle_face,
            ..
        } => {
            if vertex_side == triangle_side {
                return Err(IntersectionGraphValidationError::EventSourceMismatch);
            }
            let (vertex_tri, expected_triangle_face, triangle_mesh) =
                match (*vertex_side, *triangle_side) {
                    (MeshSide::Left, MeshSide::Right) => (left_tri, pair.right_face, right),
                    (MeshSide::Right, MeshSide::Left) => (right_tri, pair.left_face, left),
                    _ => return Err(IntersectionGraphValidationError::EventSourceMismatch),
                };
            if *triangle_face != expected_triangle_face {
                return Err(IntersectionGraphValidationError::EventSourceMismatch);
            }
            if triangle_mesh.view().face(*triangle_face).is_err() {
                return Err(IntersectionGraphValidationError::EventSourceOutOfRange);
            }
            validate_vertex(*vertex_side, *vertex, left, right)?;
            if !vertex_tri.contains(vertex) {
                return Err(IntersectionGraphValidationError::EventSourceMismatch);
            }
            Ok(())
        }
        IntersectionEvent::Unknown => Ok(()),
    }
}

fn validate_vertex(
    side: MeshSide,
    vertex: usize,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<(), IntersectionGraphValidationError> {
    let mesh = side.mesh(left, right);
    if mesh.view().vertex(vertex).is_ok() {
        Ok(())
    } else {
        Err(IntersectionGraphValidationError::EventSourceOutOfRange)
    }
}

fn validate_graph_segment_plane_event(
    relation: SegmentPlaneRelation,
    point: &Option<Point3>,
    parameter: &Option<Real>,
    parameter_ratio: &Option<SegmentPlaneParameterRatio>,
    construction_failure: &Option<SegmentPlaneConstructionFailure>,
    endpoint_sides: [Option<PlaneSide>; 2],
) -> Result<(), IntersectionGraphValidationError> {
    let endpoints_are_opposite_strict_sides = matches!(
        endpoint_sides,
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
            | [Some(PlaneSide::Below), Some(PlaneSide::Above)]
    );
    match relation {
        SegmentPlaneRelation::Disjoint => {
            if construction_failure.is_none() {
                Err(IntersectionGraphValidationError::DisjointSegmentPlaneEvent)
            } else {
                Err(IntersectionGraphValidationError::InvalidSegmentPlaneEvent)
            }
        }
        SegmentPlaneRelation::Coplanar => {
            if endpoint_sides == [Some(PlaneSide::On), Some(PlaneSide::On)]
                && point.is_none()
                && parameter.is_none()
                && parameter_ratio.is_none()
                && construction_failure.is_none()
            {
                Ok(())
            } else {
                Err(IntersectionGraphValidationError::InvalidSegmentPlaneEvent)
            }
        }
        SegmentPlaneRelation::EndpointOnPlane => {
            if point.is_some()
                && parameter.is_some()
                && parameter_ratio.is_none()
                && construction_failure.is_none()
                && endpoint_sides.contains(&Some(PlaneSide::On))
            {
                Ok(())
            } else {
                Err(IntersectionGraphValidationError::InvalidSegmentPlaneEvent)
            }
        }
        SegmentPlaneRelation::ProperCrossing => {
            if let (Some(parameter), Some(ratio)) = (parameter, parameter_ratio) {
                if point.is_some()
                    && endpoints_are_opposite_strict_sides
                    && construction_failure.is_none()
                    && ratio_matches_parameter(ratio, parameter)
                {
                    Ok(())
                } else {
                    Err(IntersectionGraphValidationError::InvalidSegmentPlaneEvent)
                }
            } else {
                Err(IntersectionGraphValidationError::InvalidSegmentPlaneEvent)
            }
        }
        SegmentPlaneRelation::Unknown => {
            if endpoint_sides.iter().any(Option::is_none)
                && point.is_none()
                && parameter.is_none()
                && parameter_ratio.is_none()
                && construction_failure.is_none()
            {
                Ok(())
            } else {
                Err(IntersectionGraphValidationError::InvalidSegmentPlaneEvent)
            }
        }
        SegmentPlaneRelation::ConstructionFailed => {
            if endpoints_are_opposite_strict_sides
                && point.is_none()
                && parameter.is_none()
                && parameter_ratio.is_none()
                && construction_failure.is_some()
            {
                Ok(())
            } else {
                Err(IntersectionGraphValidationError::InvalidSegmentPlaneEvent)
            }
        }
    }
}

fn ratio_matches_parameter(ratio: &SegmentPlaneParameterRatio, parameter: &Real) -> bool {
    if matches!(
        compare_reals(&ratio.denominator, &Real::from(0)).value(),
        Some(Ordering::Equal) | None
    ) {
        return false;
    }
    let Some(value) = (&ratio.numerator / &ratio.denominator).ok() else {
        return false;
    };
    matches!(
        compare_reals(&value, parameter).value(),
        Some(Ordering::Equal)
    )
}

fn split_topology_plan(
    split_plan: &ExactEdgeSplitPlan,
    graph_vertices: &ExactGraphVertexPlan,
) -> ExactSplitTopologyPlan {
    let mut unresolved_vertex_lookups = 0;
    let mut unresolved_equalities = graph_vertices.unresolved_equalities;
    let mut edge_chains = Vec::new();
    for split in &split_plan.splits {
        let mut nodes = Vec::with_capacity(split.points.len() + 2);
        nodes.push(SplitEdgeNode::OriginalVertex {
            side: split.side,
            vertex: split.edge[0],
        });
        for point in &split.points {
            match graph_vertex_lookup_for_split_point(point, &graph_vertices.vertices) {
                GraphVertexLookup::Matched(index) => nodes.push(SplitEdgeNode::GraphVertex {
                    graph_vertex: index,
                }),
                GraphVertexLookup::Missing { saw_undecidable } => {
                    unresolved_vertex_lookups += 1;
                    if saw_undecidable {
                        unresolved_equalities += 1;
                    }
                }
            }
        }
        nodes.push(SplitEdgeNode::OriginalVertex {
            side: split.side,
            vertex: split.edge[1],
        });
        edge_chains.push(SplitEdgeChain {
            side: split.side,
            edge: split.edge,
            nodes,
        });
    }

    ExactSplitTopologyPlan {
        graph_vertices: graph_vertices.vertices.clone(),
        edge_chains,
        unresolved_vertex_lookups,
        unresolved_equalities,
        unknown_orderings: split_plan.unknown_orderings,
    }
}

enum GraphVertexLookup {
    Matched(usize),
    Missing { saw_undecidable: bool },
}

fn graph_vertex_lookup_for_split_point(
    point: &EdgeSplitPoint,
    vertices: &[ExactGraphVertex],
) -> GraphVertexLookup {
    let mut saw_undecidable = false;
    for (index, vertex) in vertices.iter().enumerate() {
        match point3_exact_equal(&point.point, &vertex.point) {
            Some(true) => return GraphVertexLookup::Matched(index),
            Some(false) => {}
            None => saw_undecidable = true,
        }
    }
    GraphVertexLookup::Missing { saw_undecidable }
}

pub(crate) fn validate_edge_split_plan(
    split_plan: &ExactEdgeSplitPlan,
) -> SplitPlanValidationReport {
    let mut blockers = Vec::new();

    for _ in 0..split_plan.unknown_orderings {
        blockers.push(SplitPlanBlocker::new(
            SplitPlanBlockerKind::UnknownOrdering,
            "edge split parameters have an uncertified ordering",
        ));
    }

    for split in &split_plan.splits {
        for point in &split.points {
            if !ratio_matches_parameter(&point.parameter_ratio, &point.parameter) {
                blockers.push(SplitPlanBlocker {
                    side: Some(split.side),
                    edge: Some(split.edge),
                    ..SplitPlanBlocker::new(
                        SplitPlanBlockerKind::InvalidConstructionRatio,
                        "edge split point determinant ratio does not match its parameter",
                    )
                });
            }
            match point.endpoint_sides {
                [Some(PlaneSide::Above), Some(PlaneSide::Below)]
                | [Some(PlaneSide::Below), Some(PlaneSide::Above)] => {}
                [Some(_), Some(_)] => blockers.push(SplitPlanBlocker {
                    side: Some(split.side),
                    edge: Some(split.edge),
                    ..SplitPlanBlocker::new(
                        SplitPlanBlockerKind::NonCrossingEndpointSideFacts,
                        "edge split point was not certified by opposite strict endpoint sides",
                    )
                }),
                _ => blockers.push(SplitPlanBlocker {
                    side: Some(split.side),
                    edge: Some(split.edge),
                    ..SplitPlanBlocker::new(
                        SplitPlanBlockerKind::MissingEndpointSideFacts,
                        "edge split point is missing endpoint side facts",
                    )
                }),
            }
        }
    }

    SplitPlanValidationReport { blockers }
}

#[cfg(test)]
fn validate_edge_split_plan_against_sources(
    split_plan: &ExactEdgeSplitPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut report = validate_edge_split_plan(split_plan);
    if !report.blockers.is_empty() {
        return report;
    }

    let replay =
        build_unvalidated_intersection_graph(left, right).map(|graph| edge_split_plan(&graph));
    match replay {
        Ok(replay) if replay == *split_plan => report,
        Ok(_) => {
            report.blockers.push(SplitPlanBlocker::new(
                SplitPlanBlockerKind::SourceReplayMismatch,
                "edge split plan does not match exact replay from source operands",
            ));
            report
        }
        Err(error) => {
            report.blockers.push(SplitPlanBlocker::new(
                SplitPlanBlockerKind::SourceReplayMismatch,
                format!("edge split plan source replay failed: {error}"),
            ));
            report
        }
    }
}

fn face_split_plan(topology: &ExactSplitTopologyPlan) -> ExactFaceSplitPlan {
    let mut faces = BTreeMap::<(MeshSide, usize), FaceSplitPlan>::new();
    for chain in &topology.edge_chains {
        let graph_vertices = chain
            .nodes
            .iter()
            .filter_map(|node| match node {
                SplitEdgeNode::GraphVertex { graph_vertex } => Some(*graph_vertex),
                SplitEdgeNode::OriginalVertex { .. } => None,
            })
            .collect::<Vec<_>>();
        if graph_vertices.is_empty() {
            continue;
        }
        let face_indices = graph_vertices
            .iter()
            .flat_map(|&index| topology.graph_vertices[index].uses.iter())
            .filter(|vertex_use| vertex_use.side == chain.side && vertex_use.edge == chain.edge)
            .map(|vertex_use| match chain.side {
                MeshSide::Left => vertex_use.face_pair[0],
                MeshSide::Right => vertex_use.face_pair[1],
            })
            .collect::<BTreeSet<_>>();
        for face in face_indices {
            faces
                .entry((chain.side, face))
                .or_insert_with(|| FaceSplitPlan {
                    side: chain.side,
                    face,
                    edges: Vec::new(),
                })
                .edges
                .push(FaceSplitEdge {
                    edge: chain.edge,
                    graph_vertices: graph_vertices.clone(),
                });
        }
    }
    ExactFaceSplitPlan {
        faces: faces.into_values().collect(),
    }
}

pub(crate) fn validate_split_topology_plan(
    topology: &ExactSplitTopologyPlan,
) -> SplitPlanValidationReport {
    let mut blockers = Vec::new();

    for _ in 0..topology.unknown_orderings {
        blockers.push(SplitPlanBlocker::new(
            SplitPlanBlockerKind::UnknownOrdering,
            "edge split parameters have an uncertified ordering",
        ));
    }
    for _ in 0..topology.unresolved_equalities {
        blockers.push(SplitPlanBlocker::new(
            SplitPlanBlockerKind::UnresolvedEquality,
            "graph-vertex equality could not be certified",
        ));
    }
    for _ in 0..topology.unresolved_vertex_lookups {
        blockers.push(SplitPlanBlocker::new(
            SplitPlanBlockerKind::UnresolvedVertexLookup,
            "split point could not be matched to a graph vertex",
        ));
    }

    for (index, vertex) in topology.graph_vertices.iter().enumerate() {
        if vertex.uses.is_empty() {
            blockers.push(SplitPlanBlocker {
                graph_vertex: Some(index),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::EmptyGraphVertexUses,
                    "graph vertex has no exact source uses",
                )
            });
        }

        for vertex_use in &vertex.uses {
            push_graph_vertex_source_use_blockers(
                &mut blockers,
                index,
                vertex_use,
                "split topology graph vertex determinant ratio does not match its parameter",
                "split topology graph vertex was not certified by opposite strict endpoint sides",
                "split topology graph vertex is missing endpoint side facts",
            );
        }
    }

    for chain in &topology.edge_chains {
        if chain.nodes.len() < 2 {
            blockers.push(SplitPlanBlocker {
                side: Some(chain.side),
                edge: Some(chain.edge),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::EmptyOrShortEdgeChain,
                    "split edge chain does not connect both original endpoints",
                )
            });
            continue;
        }

        if chain.nodes.first()
            != Some(&SplitEdgeNode::OriginalVertex {
                side: chain.side,
                vertex: chain.edge[0],
            })
        {
            blockers.push(SplitPlanBlocker {
                side: Some(chain.side),
                edge: Some(chain.edge),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::WrongChainStart,
                    "split edge chain does not start at the directed edge start",
                )
            });
        }

        if chain.nodes.last()
            != Some(&SplitEdgeNode::OriginalVertex {
                side: chain.side,
                vertex: chain.edge[1],
            })
        {
            blockers.push(SplitPlanBlocker {
                side: Some(chain.side),
                edge: Some(chain.edge),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::WrongChainEnd,
                    "split edge chain does not end at the directed edge end",
                )
            });
        }

        for node in &chain.nodes {
            match node {
                SplitEdgeNode::OriginalVertex { side, .. } if *side != chain.side => {
                    blockers.push(SplitPlanBlocker {
                        side: Some(chain.side),
                        edge: Some(chain.edge),
                        ..SplitPlanBlocker::new(
                            SplitPlanBlockerKind::ChainSideMismatch,
                            "original vertex node is on a different mesh side from its chain",
                        )
                    });
                }
                SplitEdgeNode::GraphVertex { graph_vertex }
                    if *graph_vertex >= topology.graph_vertices.len() =>
                {
                    blockers.push(SplitPlanBlocker {
                        graph_vertex: Some(*graph_vertex),
                        side: Some(chain.side),
                        edge: Some(chain.edge),
                        ..SplitPlanBlocker::new(
                            SplitPlanBlockerKind::GraphVertexOutOfRange,
                            "split edge chain references a missing graph vertex",
                        )
                    });
                }
                _ => {}
            }
        }
    }

    SplitPlanValidationReport { blockers }
}

pub(crate) fn validate_face_split_plan(
    face_plan: &ExactFaceSplitPlan,
    topology: &ExactSplitTopologyPlan,
) -> SplitPlanValidationReport {
    let mut blockers = Vec::new();

    for face in &face_plan.faces {
        if face.edges.is_empty() {
            blockers.push(SplitPlanBlocker {
                side: Some(face.side),
                face: Some(face.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::EmptyFaceSplit,
                    "face split work item has no split edges",
                )
            });
        }

        let mut seen_edges = BTreeSet::new();
        for edge in &face.edges {
            if !seen_edges.insert(edge.edge) {
                blockers.push(SplitPlanBlocker {
                    side: Some(face.side),
                    face: Some(face.face),
                    edge: Some(edge.edge),
                    ..SplitPlanBlocker::new(
                        SplitPlanBlockerKind::DuplicateFaceSplitEdge,
                        "face split work item repeats an original edge",
                    )
                });
            }

            if edge.graph_vertices.is_empty() {
                blockers.push(SplitPlanBlocker {
                    side: Some(face.side),
                    face: Some(face.face),
                    edge: Some(edge.edge),
                    ..SplitPlanBlocker::new(
                        SplitPlanBlockerKind::EmptyFaceSplitEdge,
                        "face split edge has no graph vertices",
                    )
                });
            }

            for &graph_vertex in &edge.graph_vertices {
                let Some(vertex) = topology.graph_vertices.get(graph_vertex) else {
                    blockers.push(SplitPlanBlocker {
                        graph_vertex: Some(graph_vertex),
                        side: Some(face.side),
                        face: Some(face.face),
                        edge: Some(edge.edge),
                        ..SplitPlanBlocker::new(
                            SplitPlanBlockerKind::GraphVertexOutOfRange,
                            "face split edge references a missing graph vertex",
                        )
                    });
                    continue;
                };

                let matching_uses = vertex.uses.iter().filter(|vertex_use| {
                    vertex_use.side == face.side
                        && vertex_use.edge == edge.edge
                        && match face.side {
                            MeshSide::Left => vertex_use.face_pair[0] == face.face,
                            MeshSide::Right => vertex_use.face_pair[1] == face.face,
                        }
                });
                let mut matched_source = false;
                for vertex_use in matching_uses {
                    matched_source = true;
                    push_graph_vertex_source_use_blockers(
                        &mut blockers,
                        graph_vertex,
                        vertex_use,
                        "face split source use determinant ratio does not match its parameter",
                        "face split source use was not certified by opposite strict endpoint sides",
                        "face split source use is missing endpoint side facts",
                    );
                }

                if !matched_source {
                    blockers.push(SplitPlanBlocker {
                        graph_vertex: Some(graph_vertex),
                        side: Some(face.side),
                        face: Some(face.face),
                        edge: Some(edge.edge),
                        ..SplitPlanBlocker::new(
                            SplitPlanBlockerKind::MissingFaceSplitSourceUse,
                            "face split edge graph vertex has no exact source use on this face edge",
                        )
                    });
                }
            }
        }
    }

    SplitPlanValidationReport { blockers }
}

#[cfg(test)]
fn validate_face_split_plan_against_sources(
    face_plan: &ExactFaceSplitPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let topology = match build_unvalidated_intersection_graph(left, right) {
        Ok(graph) => match graph.checked_split_topology_plan() {
            Ok(topology) => topology,
            Err(report) => return report,
        },
        Err(error) => {
            return SplitPlanValidationReport {
                blockers: vec![SplitPlanBlocker::new(
                    SplitPlanBlockerKind::SourceReplayMismatch,
                    format!("face split plan source replay failed: {error}"),
                )],
            };
        }
    };

    let mut report = validate_face_split_plan(face_plan, &topology);
    if !report.blockers.is_empty() {
        return report;
    }

    let replay = face_split_plan(&topology);
    if replay != *face_plan {
        report.blockers.push(SplitPlanBlocker::new(
            SplitPlanBlockerKind::SourceReplayMismatch,
            "face split plan does not match exact replay from source operands",
        ));
    }
    report
}

fn face_split_geometry_plan(
    left: &ExactMesh,
    right: &ExactMesh,
    topology: &ExactSplitTopologyPlan,
    face_plan: &ExactFaceSplitPlan,
) -> Result<ExactFaceSplitGeometryPlan, ExactMeshError> {
    let chains = topology
        .edge_chains
        .iter()
        .map(|chain| ((chain.side, chain.edge[0], chain.edge[1]), chain))
        .collect::<BTreeMap<_, _>>();

    let mut faces = Vec::with_capacity(face_plan.faces.len());
    for face in &face_plan.faces {
        let mesh = face.side.mesh(left, right);
        let source_face = mesh.view().face(face.face)?;
        let triangle = source_face.vertex_indices();
        let mut boundary_chains = Vec::with_capacity(face.edges.len());
        for edge in &face.edges {
            let chain = chains
                .get(&(face.side, edge.edge[0], edge.edge[1]))
                .ok_or_else(|| {
                    ExactMeshError::one(
                        ExactMeshBlocker::new(
                            ExactMeshBlockerKind::IndexOutOfBounds,
                            "face split geometry references a missing split edge chain",
                        )
                        .with_face(face.face)
                        .with_edge(edge.edge),
                    )
                })?;
            for &graph_vertex in &edge.graph_vertices {
                if graph_vertex >= topology.graph_vertices.len() {
                    return Err(ExactMeshError::one(
                        ExactMeshBlocker::new(
                            ExactMeshBlockerKind::IndexOutOfBounds,
                            "face split geometry references a missing graph vertex",
                        )
                        .with_face(face.face)
                        .with_edge(edge.edge),
                    ));
                }
            }
            let mut nodes = Vec::with_capacity(chain.nodes.len());
            for node in &chain.nodes {
                nodes.push(match node {
                    SplitEdgeNode::OriginalVertex {
                        side: vertex_side,
                        vertex,
                    } if *vertex_side == face.side => {
                        let point = mesh.view().vertex(*vertex)?;
                        FaceSplitBoundaryNode::OriginalVertex {
                            vertex: *vertex,
                            point: point.point().clone(),
                        }
                    }
                    SplitEdgeNode::GraphVertex { graph_vertex } => {
                        let vertex =
                            topology.graph_vertices.get(*graph_vertex).ok_or_else(|| {
                                ExactMeshError::one(
                                    ExactMeshBlocker::new(
                                        ExactMeshBlockerKind::IndexOutOfBounds,
                                        "split boundary references a missing graph vertex",
                                    )
                                    .with_vertex(*graph_vertex),
                                )
                            })?;
                        FaceSplitBoundaryNode::GraphVertex {
                            graph_vertex: *graph_vertex,
                            point: vertex.point.clone(),
                        }
                    }
                    SplitEdgeNode::OriginalVertex { vertex, .. } => {
                        return Err(ExactMeshError::one(
                            ExactMeshBlocker::new(
                                ExactMeshBlockerKind::IndexOutOfBounds,
                                "split boundary original vertex is on the wrong mesh side",
                            )
                            .with_vertex(*vertex),
                        ));
                    }
                });
            }
            boundary_chains.push(FaceSplitBoundaryChain {
                edge: edge.edge,
                nodes,
            });
        }
        faces.push(FaceSplitGeometry {
            side: face.side,
            face: face.face,
            triangle,
            boundary_chains,
        });
    }

    Ok(ExactFaceSplitGeometryPlan { faces })
}

pub(crate) fn validate_face_split_geometry_incidence(
    geometry: &ExactFaceSplitGeometryPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut blockers = Vec::new();

    for face in &geometry.faces {
        let mesh = face.side.mesh(left, right);
        let Ok(source_face) = mesh.view().face(face.face) else {
            blockers.push(SplitPlanBlocker {
                side: Some(face.side),
                face: Some(face.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::GraphVertexOutOfRange,
                    "split-face geometry references a missing retained face fact",
                )
            });
            continue;
        };

        let triangle = source_face.vertex_indices();
        if face.triangle != triangle {
            blockers.push(SplitPlanBlocker {
                side: Some(face.side),
                face: Some(face.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::SourceTriangleMismatch,
                    "split-face geometry source triangle does not match its source face",
                )
            });
            continue;
        }

        if face.boundary_chains.is_empty() {
            blockers.push(SplitPlanBlocker {
                side: Some(face.side),
                face: Some(face.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::EmptyFaceSplit,
                    "split-face geometry has no retained boundary chains",
                )
            });
        }

        let triangle_edge_set = triangle_edges(triangle)
            .into_iter()
            .collect::<BTreeSet<_>>();
        let mut seen_edges = BTreeSet::new();
        let Ok([a, b, c]) = source_face.vertices() else {
            blockers.push(SplitPlanBlocker {
                side: Some(face.side),
                face: Some(face.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::SourceTriangleMismatch,
                    "split-face geometry source triangle references a missing source vertex",
                )
            });
            continue;
        };
        for chain in &face.boundary_chains {
            if !seen_edges.insert(chain.edge) {
                blockers.push(SplitPlanBlocker {
                    side: Some(face.side),
                    face: Some(face.face),
                    edge: Some(chain.edge),
                    ..SplitPlanBlocker::new(
                        SplitPlanBlockerKind::DuplicateFaceSplitEdge,
                        "split-face geometry repeats a retained boundary chain edge",
                    )
                });
            }
            if !triangle_edge_set.contains(&chain.edge) {
                blockers.push(SplitPlanBlocker {
                    side: Some(face.side),
                    face: Some(face.face),
                    edge: Some(chain.edge),
                    ..SplitPlanBlocker::new(
                        SplitPlanBlockerKind::BoundaryChainEdgeNotOnTriangle,
                        "split-face geometry boundary chain edge is not on its source triangle",
                    )
                });
                continue;
            }
            validate_face_split_boundary_chain_shape(
                &mut blockers,
                mesh,
                face.side,
                face.face,
                chain,
            );
            for node in &chain.nodes {
                let point = boundary_node_point(node);
                match orient3d_report(a, b, c, point).value() {
                    Some(Sign::Zero) => {}
                    Some(Sign::Negative | Sign::Positive) => blockers.push(SplitPlanBlocker {
                        side: Some(face.side),
                        face: Some(face.face),
                        edge: Some(chain.edge),
                        ..SplitPlanBlocker::new(
                            SplitPlanBlockerKind::BoundaryNodeOffFacePlane,
                            "split boundary node is not incident to its original face plane",
                        )
                    }),
                    None => blockers.push(SplitPlanBlocker {
                        side: Some(face.side),
                        face: Some(face.face),
                        edge: Some(chain.edge),
                        ..SplitPlanBlocker::new(
                            SplitPlanBlockerKind::UnknownBoundaryIncidence,
                            "split boundary node incidence could not be certified",
                        )
                    }),
                }
            }
        }
    }

    SplitPlanValidationReport { blockers }
}

fn validate_face_split_boundary_chain_shape(
    blockers: &mut Vec<SplitPlanBlocker>,
    mesh: &ExactMesh,
    side: MeshSide,
    face: usize,
    chain: &FaceSplitBoundaryChain,
) {
    if chain.nodes.len() < 2 {
        blockers.push(SplitPlanBlocker {
            side: Some(side),
            face: Some(face),
            edge: Some(chain.edge),
            ..SplitPlanBlocker::new(
                SplitPlanBlockerKind::EmptyOrShortEdgeChain,
                "split-face geometry boundary chain does not connect both edge endpoints",
            )
        });
        return;
    }

    let expected_start = Some(chain.edge[0]);
    let expected_end = Some(chain.edge[1]);
    let actual_start = match chain.nodes.first() {
        Some(FaceSplitBoundaryNode::OriginalVertex { vertex, .. }) => Some(*vertex),
        Some(
            FaceSplitBoundaryNode::GraphVertex { .. } | FaceSplitBoundaryNode::FaceInterior { .. },
        )
        | None => None,
    };
    if actual_start != expected_start {
        blockers.push(SplitPlanBlocker {
            side: Some(side),
            face: Some(face),
            edge: Some(chain.edge),
            ..SplitPlanBlocker::new(
                SplitPlanBlockerKind::WrongChainStart,
                "split-face geometry boundary chain does not start at its source edge start",
            )
        });
    }
    let actual_end = match chain.nodes.last() {
        Some(FaceSplitBoundaryNode::OriginalVertex { vertex, .. }) => Some(*vertex),
        Some(
            FaceSplitBoundaryNode::GraphVertex { .. } | FaceSplitBoundaryNode::FaceInterior { .. },
        )
        | None => None,
    };
    if actual_end != expected_end {
        blockers.push(SplitPlanBlocker {
            side: Some(side),
            face: Some(face),
            edge: Some(chain.edge),
            ..SplitPlanBlocker::new(
                SplitPlanBlockerKind::WrongChainEnd,
                "split-face geometry boundary chain does not end at its source edge end",
            )
        });
    }

    for node in &chain.nodes {
        let FaceSplitBoundaryNode::OriginalVertex { vertex, point } = node else {
            continue;
        };
        validate_original_boundary_source_point(
            blockers,
            mesh,
            Some(side),
            Some(face),
            Some(chain.edge),
            *vertex,
            point,
            "split-face geometry boundary node references a missing source vertex",
            "split-face geometry original boundary node point does not match its source vertex",
            "split-face geometry original boundary node source-point equality is undecidable",
        );
    }
}

#[cfg(test)]
fn validate_face_split_geometry_against_sources(
    geometry: &ExactFaceSplitGeometryPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut report = validate_face_split_geometry_incidence(geometry, left, right);
    if !report.blockers.is_empty() {
        return report;
    }

    let replay = build_unvalidated_intersection_graph(left, right)
        .and_then(|graph| graph.face_split_geometry_plan(left, right));
    match replay {
        Ok(replay) if replay == *geometry => report,
        Ok(_) => {
            report.blockers.push(SplitPlanBlocker::new(
                SplitPlanBlockerKind::SourceReplayMismatch,
                "split-face geometry does not match exact replay from source operands",
            ));
            report
        }
        Err(error) => {
            report.blockers.push(SplitPlanBlocker::new(
                SplitPlanBlockerKind::SourceReplayMismatch,
                format!("split-face geometry source replay failed: {error}"),
            ));
            report
        }
    }
}

pub(crate) fn validate_face_region_plan(
    plan: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut blockers = Vec::new();
    for region in &plan.regions {
        if region.boundary.len() < 3 {
            blockers.push(SplitPlanBlocker {
                side: Some(region.side),
                face: Some(region.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::EmptyOrShortRegionBoundary,
                    "face region boundary has fewer than three nodes",
                )
            });
        }

        for window in region.boundary.windows(2) {
            if boundary_nodes_equal(&window[0], &window[1]) == Some(true) {
                blockers.push(SplitPlanBlocker {
                    side: Some(region.side),
                    face: Some(region.face),
                    ..SplitPlanBlocker::new(
                        SplitPlanBlockerKind::DuplicateConsecutiveRegionNode,
                        "face region boundary contains consecutive duplicate nodes",
                    )
                });
            }
        }
        if region
            .boundary
            .first()
            .zip(region.boundary.last())
            .is_some_and(|(first, last)| boundary_nodes_equal(first, last) == Some(true))
        {
            blockers.push(SplitPlanBlocker {
                side: Some(region.side),
                face: Some(region.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::DuplicateConsecutiveRegionNode,
                    "face region boundary repeats its first node at the end",
                )
            });
        }

        let mesh = region.side.mesh(left, right);
        let Ok(source_face) = mesh.view().face(region.face) else {
            blockers.push(SplitPlanBlocker {
                side: Some(region.side),
                face: Some(region.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::GraphVertexOutOfRange,
                    "face region references a missing retained face fact",
                )
            });
            continue;
        };

        let triangle = source_face.vertex_indices();
        if region.triangle != triangle {
            blockers.push(SplitPlanBlocker {
                side: Some(region.side),
                face: Some(region.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::SourceTriangleMismatch,
                    "face region source triangle does not match its source face",
                )
            });
            continue;
        }
        validate_face_region_original_boundary_nodes(&mut blockers, mesh, region);
        let Ok([a, b, c]) = source_face.vertices() else {
            blockers.push(SplitPlanBlocker {
                side: Some(region.side),
                face: Some(region.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::SourceTriangleMismatch,
                    "face region source triangle references a missing source vertex",
                )
            });
            continue;
        };
        for node in &region.boundary {
            let point = boundary_node_point(node);
            match orient3d_report(a, b, c, point).value() {
                Some(Sign::Zero) => {}
                Some(Sign::Negative | Sign::Positive) => blockers.push(SplitPlanBlocker {
                    side: Some(region.side),
                    face: Some(region.face),
                    ..SplitPlanBlocker::new(
                        SplitPlanBlockerKind::BoundaryNodeOffFacePlane,
                        "face region boundary node is not incident to its source face plane",
                    )
                }),
                None => blockers.push(SplitPlanBlocker {
                    side: Some(region.side),
                    face: Some(region.face),
                    ..SplitPlanBlocker::new(
                        SplitPlanBlockerKind::UnknownBoundaryIncidence,
                        "face region boundary incidence could not be certified",
                    )
                }),
            }
        }
    }

    SplitPlanValidationReport { blockers }
}

fn validate_face_region_original_boundary_nodes(
    blockers: &mut Vec<SplitPlanBlocker>,
    mesh: &ExactMesh,
    region: &FaceRegionBoundary,
) {
    for node in &region.boundary {
        let FaceSplitBoundaryNode::OriginalVertex { vertex, point } = node else {
            continue;
        };
        if !region.triangle.contains(vertex) {
            blockers.push(SplitPlanBlocker {
                side: Some(region.side),
                face: Some(region.face),
                ..SplitPlanBlocker::new(
                    SplitPlanBlockerKind::BoundaryNodeSourceVertexNotOnTriangle,
                    "face region original boundary node is not part of its source triangle",
                )
            });
        }
        validate_original_boundary_source_point(
            blockers,
            mesh,
            Some(region.side),
            Some(region.face),
            None,
            *vertex,
            point,
            "face region original boundary node references a missing source vertex",
            "face region original boundary node point does not match its source vertex",
            "face region original boundary node source-point equality is undecidable",
        );
    }
}

fn validate_original_boundary_source_point(
    blockers: &mut Vec<SplitPlanBlocker>,
    mesh: &ExactMesh,
    side: Option<MeshSide>,
    face: Option<usize>,
    edge: Option<[usize; 2]>,
    vertex: usize,
    point: &Point3,
    missing_vertex_message: &'static str,
    mismatch_message: &'static str,
    unknown_message: &'static str,
) {
    let Ok(source_point) = mesh.view().vertex(vertex).map(|vertex| vertex.point()) else {
        blockers.push(SplitPlanBlocker {
            side,
            face,
            edge,
            ..SplitPlanBlocker::new(
                SplitPlanBlockerKind::BoundaryNodeSourceVertexOutOfRange,
                missing_vertex_message,
            )
        });
        return;
    };
    match point3_exact_equal(point, source_point) {
        Some(true) => {}
        Some(false) => blockers.push(SplitPlanBlocker {
            side,
            face,
            edge,
            ..SplitPlanBlocker::new(
                SplitPlanBlockerKind::BoundaryNodeSourcePointMismatch,
                mismatch_message,
            )
        }),
        None => blockers.push(SplitPlanBlocker {
            side,
            face,
            edge,
            ..SplitPlanBlocker::new(
                SplitPlanBlockerKind::UnknownBoundaryIncidence,
                unknown_message,
            )
        }),
    }
}

#[cfg(test)]
fn validate_face_region_plan_against_sources(
    plan: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut report = validate_face_region_plan(plan, left, right);
    if !report.blockers.is_empty() {
        return report;
    }

    let replay = build_unvalidated_intersection_graph(left, right)
        .and_then(|graph| graph.face_split_geometry_plan(left, right))
        .map(|geometry| geometry.region_plan(left, right));
    match replay {
        Ok(replay) if replay == *plan => report,
        Ok(_) => {
            report.blockers.push(SplitPlanBlocker::new(
                SplitPlanBlockerKind::SourceReplayMismatch,
                "face region plan does not match exact replay from source operands",
            ));
            report
        }
        Err(error) => {
            report.blockers.push(SplitPlanBlocker::new(
                SplitPlanBlockerKind::SourceReplayMismatch,
                format!("face region plan source replay failed: {error}"),
            ));
            report
        }
    }
}

fn coplanar_overlap_split_graph(
    graph: &CoplanarOverlapGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<CoplanarOverlapSplitGraph, ExactMeshError> {
    let edge_splits = graph
        .edge_overlaps
        .iter()
        .map(|overlap| coplanar_edge_split_construction(overlap, graph.projection, left, right))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(CoplanarOverlapSplitGraph {
        left_face: graph.left_face,
        right_face: graph.right_face,
        projection: graph.projection,
        edge_splits,
        vertex_overlaps: graph.vertex_overlaps.clone(),
    })
}

fn coplanar_edge_split_construction(
    overlap: &CoplanarEdgeOverlap,
    projection: CoplanarProjection,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<CoplanarEdgeSplitConstruction, ExactMeshError> {
    let left_edge = edge_points(left, overlap.left_edge)?;
    let right_edge = edge_points(right, overlap.right_edge)?;
    let (points, interval_overlap, interval) = match overlap.relation {
        SegmentIntersection::Disjoint => (Vec::new(), false, None),
        SegmentIntersection::EndpointTouch => {
            let point = endpoint_touch_split_point(left_edge, right_edge, projection)
                .map_err(coplanar_split_validation_mesh_error)?;
            (vec![point], false, None)
        }
        SegmentIntersection::Proper => {
            let point = proper_coplanar_edge_split_point(left_edge, right_edge, projection)
                .map_err(coplanar_split_validation_mesh_error)?;
            (vec![point], false, None)
        }
        SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
            let interval = coplanar_edge_interval(left_edge, right_edge, projection)
                .map_err(coplanar_split_validation_mesh_error)?;
            (Vec::new(), true, Some(interval))
        }
    };
    let split = CoplanarEdgeSplitConstruction {
        overlap: overlap.clone(),
        points,
        interval_overlap,
        interval,
    };
    validate_coplanar_edge_split_against_edges(&split, left_edge, right_edge)
        .map_err(coplanar_split_validation_mesh_error)?;
    Ok(split)
}

fn validate_coplanar_edge_split(
    split: &CoplanarEdgeSplitConstruction,
) -> Result<(), CoplanarOverlapSplitValidationError> {
    let zero = Real::from(0);
    let one = Real::from(1);
    let validate_unit_parameter =
        |parameter: &Real| -> Result<(), CoplanarOverlapSplitValidationError> {
            match (
                compare_reals(parameter, &zero).value(),
                compare_reals(parameter, &one).value(),
            ) {
                (Some(Ordering::Less), _) | (_, Some(Ordering::Greater)) => {
                    Err(CoplanarOverlapSplitValidationError::SplitParameterOutOfRange)
                }
                (Some(_), Some(_)) => Ok(()),
                _ => Err(CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder),
            }
        };

    match split.overlap.relation {
        SegmentIntersection::Disjoint => {
            Err(CoplanarOverlapSplitValidationError::DisjointEdgeSplit)
        }
        SegmentIntersection::EndpointTouch | SegmentIntersection::Proper => {
            if split.interval_overlap {
                return Err(CoplanarOverlapSplitValidationError::UnexpectedIntervalConstruction);
            }
            if split.interval.is_some() {
                return Err(CoplanarOverlapSplitValidationError::UnexpectedIntervalConstruction);
            }
            if split.points.len() != 1 {
                return Err(CoplanarOverlapSplitValidationError::MissingPointConstruction);
            }
            let point = &split.points[0];
            validate_unit_parameter(&point.left_parameter)?;
            validate_unit_parameter(&point.right_parameter)?;
            let parameter_position = |parameter: &Real| -> Result<
                (bool, bool),
                CoplanarOverlapSplitValidationError,
            > {
                match (
                    compare_reals(parameter, &zero).value(),
                    compare_reals(parameter, &one).value(),
                ) {
                    (Some(Ordering::Equal), _) | (_, Some(Ordering::Equal)) => Ok((true, false)),
                    (Some(Ordering::Greater), Some(Ordering::Less)) => Ok((false, true)),
                    (Some(_), Some(_)) => Ok((false, false)),
                    _ => Err(CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder),
                }
            };
            let (left_endpoint, left_strict_interior) = parameter_position(&point.left_parameter)?;
            let (right_endpoint, right_strict_interior) =
                parameter_position(&point.right_parameter)?;
            // These edge parameters are the retained structure sorted and
            // merged by planar-cell extraction, so endpoint/proper relation
            // labels must agree with certified parameter positions before the
            // record can be consumed.
            if split.overlap.relation == SegmentIntersection::EndpointTouch {
                if left_endpoint || right_endpoint {
                    Ok(())
                } else {
                    Err(CoplanarOverlapSplitValidationError::EndpointTouchWithoutEndpointParameter)
                }
            } else if left_strict_interior && right_strict_interior {
                Ok(())
            } else {
                Err(CoplanarOverlapSplitValidationError::ProperCrossingEndpointParameter)
            }
        }
        SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
            if !split.points.is_empty() {
                return Err(CoplanarOverlapSplitValidationError::UnexpectedPointConstruction);
            }
            if !split.interval_overlap {
                return Err(CoplanarOverlapSplitValidationError::MissingIntervalConstruction);
            }
            let interval = split
                .interval
                .as_ref()
                .ok_or(CoplanarOverlapSplitValidationError::MissingIntervalEndpoints)?;
            validate_unit_parameter(&interval.endpoints[0].left_parameter)?;
            validate_unit_parameter(&interval.endpoints[0].right_parameter)?;
            validate_unit_parameter(&interval.endpoints[1].left_parameter)?;
            validate_unit_parameter(&interval.endpoints[1].right_parameter)?;
            match compare_reals(
                &interval.endpoints[0].left_parameter,
                &interval.endpoints[1].left_parameter,
            )
            .value()
            {
                Some(Ordering::Less) => Ok(()),
                Some(Ordering::Equal | Ordering::Greater) => {
                    Err(CoplanarOverlapSplitValidationError::DegenerateInterval)
                }
                None => Err(CoplanarOverlapSplitValidationError::UnknownIntervalOrder),
            }
        }
    }
}

fn validate_coplanar_edge_split_against_edges(
    split: &CoplanarEdgeSplitConstruction,
    left_edge: BorrowedEdgePoints<'_>,
    right_edge: BorrowedEdgePoints<'_>,
) -> Result<(), CoplanarOverlapSplitValidationError> {
    validate_coplanar_edge_split(split)?;

    for point in &split.points {
        validate_split_point_against_edges(point, left_edge, right_edge)?;
    }
    if let Some(interval) = &split.interval {
        for endpoint in &interval.endpoints {
            validate_split_point_against_edges(endpoint, left_edge, right_edge)?;
        }
    }
    Ok(())
}

fn validate_split_point_against_edges(
    point: &CoplanarEdgeSplitPoint,
    left_edge: BorrowedEdgePoints<'_>,
    right_edge: BorrowedEdgePoints<'_>,
) -> Result<(), CoplanarOverlapSplitValidationError> {
    let left_replayed = interpolate_point3(left_edge[0], left_edge[1], &point.left_parameter);
    match point3_exact_equal(&point.point, &left_replayed) {
        Some(true) => {}
        Some(false) => {
            return Err(CoplanarOverlapSplitValidationError::SplitPointDoesNotMatchLeftParameter);
        }
        None => return Err(CoplanarOverlapSplitValidationError::UnknownSplitPointEquality),
    }

    let right_replayed = interpolate_point3(right_edge[0], right_edge[1], &point.right_parameter);
    match point3_exact_equal(&point.point, &right_replayed) {
        Some(true) => Ok(()),
        Some(false) => {
            Err(CoplanarOverlapSplitValidationError::SplitPointDoesNotMatchRightParameter)
        }
        None => Err(CoplanarOverlapSplitValidationError::UnknownSplitPointEquality),
    }
}

fn coplanar_split_validation_mesh_error(
    error: CoplanarOverlapSplitValidationError,
) -> ExactMeshError {
    let kind = match error {
        CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder
        | CoplanarOverlapSplitValidationError::UnknownIntervalOrder
        | CoplanarOverlapSplitValidationError::UnknownSplitPointEquality => {
            ExactMeshBlockerKind::UndecidablePredicate
        }
        CoplanarOverlapSplitValidationError::SplitPointDoesNotMatchLeftParameter
        | CoplanarOverlapSplitValidationError::SplitPointDoesNotMatchRightParameter
        | CoplanarOverlapSplitValidationError::SameSideVertexOverlap
        | CoplanarOverlapSplitValidationError::NonConstructiveVertexOverlap => {
            ExactMeshBlockerKind::StaleFactReplay
        }
        #[cfg(test)]
        CoplanarOverlapSplitValidationError::SourceReplayMismatch => {
            ExactMeshBlockerKind::StaleFactReplay
        }
        CoplanarOverlapSplitValidationError::MissingPointConstruction
        | CoplanarOverlapSplitValidationError::DisjointEdgeSplit
        | CoplanarOverlapSplitValidationError::MissingIntervalConstruction
        | CoplanarOverlapSplitValidationError::MissingIntervalEndpoints
        | CoplanarOverlapSplitValidationError::UnexpectedIntervalConstruction
        | CoplanarOverlapSplitValidationError::UnexpectedPointConstruction
        | CoplanarOverlapSplitValidationError::SplitParameterOutOfRange
        | CoplanarOverlapSplitValidationError::EndpointTouchWithoutEndpointParameter
        | CoplanarOverlapSplitValidationError::ProperCrossingEndpointParameter
        | CoplanarOverlapSplitValidationError::DegenerateInterval => {
            ExactMeshBlockerKind::ExactConstructionFailure
        }
    };
    ExactMeshError {
        blockers: vec![ExactMeshBlocker::new(
            kind,
            format!(
                "retained coplanar split construction failed source-edge validation: {error:?}"
            ),
        )],
    }
}

type BorrowedEdgePoints<'a> = [&'a Point3; 2];

fn edge_points(
    mesh: &ExactMesh,
    edge: [usize; 2],
) -> Result<BorrowedEdgePoints<'_>, ExactMeshError> {
    let start = mesh.view().vertex(edge[0])?;
    let end = mesh.view().vertex(edge[1])?;
    Ok([start.point(), end.point()])
}

fn endpoint_touch_split_point(
    left: BorrowedEdgePoints<'_>,
    right: BorrowedEdgePoints<'_>,
    projection: CoplanarProjection,
) -> Result<CoplanarEdgeSplitPoint, CoplanarOverlapSplitValidationError> {
    for (left_index, left_point) in left.into_iter().enumerate() {
        for (right_index, right_point) in right.into_iter().enumerate() {
            let equal = projected_points_equal(left_point, right_point, projection)
                .ok_or(CoplanarOverlapSplitValidationError::UnknownSplitPointEquality)?;
            if equal {
                return Ok(CoplanarEdgeSplitPoint {
                    point: (*left_point).clone(),
                    left_parameter: Real::from(left_index as i64),
                    right_parameter: Real::from(right_index as i64),
                });
            }
        }
    }
    let right_start = project_point3(right[0], projection);
    let right_end = project_point3(right[1], projection);
    for (left_index, left_point) in left.into_iter().enumerate() {
        let projected = project_point3(left_point, projection);
        match point_on_segment(&right_start, &right_end, &projected).value() {
            Some(true) => {
                let right_parameter =
                    projected_segment_parameter3(left_point, right[0], right[1], projection)
                        .ok_or(CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder)?;
                return Ok(CoplanarEdgeSplitPoint {
                    point: (*left_point).clone(),
                    left_parameter: Real::from(left_index as i64),
                    right_parameter,
                });
            }
            Some(false) => {}
            None => return Err(CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder),
        }
    }

    let left_start = project_point3(left[0], projection);
    let left_end = project_point3(left[1], projection);
    for (right_index, right_point) in right.into_iter().enumerate() {
        let projected = project_point3(right_point, projection);
        match point_on_segment(&left_start, &left_end, &projected).value() {
            Some(true) => {
                let left_parameter =
                    projected_segment_parameter3(right_point, left[0], left[1], projection)
                        .ok_or(CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder)?;
                return Ok(CoplanarEdgeSplitPoint {
                    point: (*right_point).clone(),
                    left_parameter,
                    right_parameter: Real::from(right_index as i64),
                });
            }
            Some(false) => {}
            None => return Err(CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder),
        }
    }
    Err(CoplanarOverlapSplitValidationError::MissingPointConstruction)
}

fn coplanar_edge_interval(
    left: BorrowedEdgePoints<'_>,
    right: BorrowedEdgePoints<'_>,
    projection: CoplanarProjection,
) -> Result<CoplanarEdgeInterval, CoplanarOverlapSplitValidationError> {
    let mut endpoints = Vec::new();
    for (left_index, point) in left.into_iter().enumerate() {
        if let Some(right_parameter) =
            certified_endpoint_parameter_on_segment(point, right[0], right[1], projection)
        {
            push_interval_endpoint(
                &mut endpoints,
                CoplanarEdgeSplitPoint {
                    point: (*point).clone(),
                    left_parameter: Real::from(left_index as i64),
                    right_parameter,
                },
                projection,
            )?;
        }
    }
    for (right_index, point) in right.into_iter().enumerate() {
        if let Some(left_parameter) =
            certified_endpoint_parameter_on_segment(point, left[0], left[1], projection)
        {
            push_interval_endpoint(
                &mut endpoints,
                CoplanarEdgeSplitPoint {
                    point: (*point).clone(),
                    left_parameter,
                    right_parameter: Real::from(right_index as i64),
                },
                projection,
            )?;
        }
    }
    if endpoints.len() != 2 {
        return Err(CoplanarOverlapSplitValidationError::MissingIntervalEndpoints);
    }
    let order = compare_reals(&endpoints[0].left_parameter, &endpoints[1].left_parameter)
        .value()
        .ok_or(CoplanarOverlapSplitValidationError::UnknownIntervalOrder)?;
    if order == Ordering::Greater {
        endpoints.swap(0, 1);
    } else if order == Ordering::Equal {
        return Err(CoplanarOverlapSplitValidationError::DegenerateInterval);
    }
    Ok(CoplanarEdgeInterval {
        endpoints: [endpoints.remove(0), endpoints.remove(0)],
    })
}

fn certified_endpoint_parameter_on_segment(
    point: &Point3,
    start: &Point3,
    end: &Point3,
    projection: CoplanarProjection,
) -> Option<Real> {
    let projected_start = project_point3(start, projection);
    let projected_end = project_point3(end, projection);
    let projected_point = project_point3(point, projection);
    if point_on_segment(&projected_start, &projected_end, &projected_point).value() == Some(true) {
        projected_segment_parameter3(point, start, end, projection)
    } else {
        None
    }
}

fn push_interval_endpoint(
    endpoints: &mut Vec<CoplanarEdgeSplitPoint>,
    candidate: CoplanarEdgeSplitPoint,
    projection: CoplanarProjection,
) -> Result<(), CoplanarOverlapSplitValidationError> {
    for endpoint in endpoints.iter_mut() {
        let equal = projected_points_equal(&endpoint.point, &candidate.point, projection)
            .ok_or(CoplanarOverlapSplitValidationError::UnknownSplitPointEquality)?;
        if equal {
            return Ok(());
        }
    }
    endpoints.push(candidate);
    Ok(())
}

fn proper_coplanar_edge_split_point(
    left: BorrowedEdgePoints<'_>,
    right: BorrowedEdgePoints<'_>,
    projection: CoplanarProjection,
) -> Result<CoplanarEdgeSplitPoint, CoplanarOverlapSplitValidationError> {
    let left_parameter =
        projected_line_parameter3(left[0], left[1], right[0], right[1], projection)
            .ok_or(CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder)?;
    let right_parameter =
        projected_line_parameter3(right[0], right[1], left[0], left[1], projection)
            .ok_or(CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder)?;
    let point = interpolate_point3(left[0], left[1], &left_parameter);
    Ok(CoplanarEdgeSplitPoint {
        point,
        left_parameter,
        right_parameter,
    })
}

fn boundary_node_point(node: &FaceSplitBoundaryNode) -> &Point3 {
    match node {
        FaceSplitBoundaryNode::OriginalVertex { point, .. }
        | FaceSplitBoundaryNode::GraphVertex { point, .. }
        | FaceSplitBoundaryNode::FaceInterior { point } => point,
    }
}

fn boundary_nodes_equal(
    left: &FaceSplitBoundaryNode,
    right: &FaceSplitBoundaryNode,
) -> Option<bool> {
    point3_exact_equal(boundary_node_point(left), boundary_node_point(right))
}

fn append_segment_plane_events(
    events: &mut Vec<IntersectionEvent>,
    segment_side: MeshSide,
    edges: &[[usize; 2]; 3],
    plane_side: MeshSide,
    plane_face: usize,
    segment_events: &[SegmentPlaneIntersection],
) {
    for (edge, event) in edges.iter().zip(segment_events) {
        if matches!(event.relation, SegmentPlaneRelation::Disjoint) {
            continue;
        }
        events.push(IntersectionEvent::SegmentPlane {
            segment_side,
            edge: *edge,
            plane_side,
            plane_face,
            relation: event.relation,
            point: event.point.clone(),
            parameter: event.parameter.clone(),
            parameter_ratio: event.parameter_ratio.clone(),
            construction_failure: event.construction_failure,
            endpoint_sides: event.endpoint_sides,
        });
    }
}

#[allow(clippy::too_many_arguments)]
fn append_coplanar_events(
    events: &mut Vec<IntersectionEvent>,
    left_face: usize,
    right_face: usize,
    left_tri: [usize; 3],
    right_tri: [usize; 3],
    left_edges: &[[usize; 2]; 3],
    right_edges: &[[usize; 2]; 3],
    coplanar: &CoplanarTriangleClassification,
) {
    for (index, relation) in coplanar.edge_intersections.iter().copied().enumerate() {
        let left_edge = left_edges[index / 3];
        let right_edge = right_edges[index % 3];
        if relation != SegmentIntersection::Disjoint {
            events.push(IntersectionEvent::CoplanarEdge {
                left_edge,
                right_edge,
                relation,
            });
        }
    }

    for (vertex_side, vertices, triangle_side, triangle_face, locations) in [
        (
            MeshSide::Right,
            right_tri,
            MeshSide::Left,
            left_face,
            coplanar.right_vertices_in_left,
        ),
        (
            MeshSide::Left,
            left_tri,
            MeshSide::Right,
            right_face,
            coplanar.left_vertices_in_right,
        ),
    ] {
        for (vertex, location) in vertices.into_iter().zip(locations) {
            match location {
                Some(
                    location @ (TriangleLocation::Inside
                    | TriangleLocation::OnEdge
                    | TriangleLocation::OnVertex),
                ) => events.push(IntersectionEvent::CoplanarVertex {
                    vertex_side,
                    vertex,
                    triangle_side,
                    triangle_face,
                    location,
                }),
                None => events.push(IntersectionEvent::Unknown),
                Some(TriangleLocation::Outside | TriangleLocation::Degenerate) => {}
            }
        }
    }
}

fn projected_points_equal(
    left: &Point3,
    right: &Point3,
    projection: CoplanarProjection,
) -> Option<bool> {
    let left = project_point3(left, projection);
    let right = project_point3(right, projection);
    let x = compare_reals(&left.x, &right.x).value()?;
    let y = compare_reals(&left.y, &right.y).value()?;
    Some(x == Ordering::Equal && y == Ordering::Equal)
}

#[cfg(test)]
mod tests;
