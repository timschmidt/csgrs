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

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{
    PlaneSide, Point3, SegmentIntersection, Sign, TriangleLocation, compare_reals,
    interpolate_point3, orient3d_report, point_on_segment, project_point3,
    projected_line_parameter3, projected_segment_parameter3,
};

use super::construction::{
    SegmentPlaneConstructionFailure, SegmentPlaneIntersection, SegmentPlaneParameterRatio,
    SegmentPlaneRelation,
};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::intersection::{
    MeshFacePairClassification, MeshFacePairRelation, classify_mesh_face_pairs,
};
use super::mesh::ExactMesh;
use hyperlimit::{CoplanarProjection, CoplanarTriangleClassification};
use hyperreal::Real;

/// Side of a two-mesh graph event.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshSide {
    /// The first mesh passed to graph construction.
    Left,
    /// The second mesh passed to graph construction.
    Right,
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
pub enum IntersectionEvent {
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

/// Retained projected edge contact in a coplanar face-pair overlap graph.
///
/// These records are arrangement inputs, not final topology. They retain the
/// coplanar decomposition while keeping mutation deferred until the full
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CoplanarEdgeOverlap {
    /// Edge in the left face.
    pub left_edge: [usize; 2],
    /// Edge in the right face.
    pub right_edge: [usize; 2],
    /// Certified projected segment relation.
    pub relation: SegmentIntersection,
}

/// Retained vertex containment/touching fact in a coplanar overlap graph.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CoplanarVertexOverlap {
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
pub struct CoplanarOverlapGraph {
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
pub struct CoplanarEdgeSplitPoint {
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
pub struct CoplanarEdgeInterval {
    /// Certified closed interval endpoints.
    pub endpoints: [CoplanarEdgeSplitPoint; 2],
}

/// Retained split construction for one coplanar edge contact.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarEdgeSplitConstruction {
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

impl CoplanarEdgeSplitConstruction {
    /// Validate point-vs-interval construction consistency for one edge contact.
    pub fn validate(&self) -> Result<(), CoplanarOverlapSplitValidationError> {
        validate_coplanar_edge_split(self)
    }

    /// Validate construction consistency and exact source-edge incidence.
    ///
    /// This is the geometry-aware version of [`Self::validate`]. It checks
    /// that each retained split point is exactly the interpolation of both
    /// source edges at the stored parameters. That retained construction check
    /// but those parameters must still replay to retained object geometry
    /// before they become combinatorial evidence.
    pub fn validate_against_edges(
        &self,
        left_edge: [Point3; 2],
        right_edge: [Point3; 2],
    ) -> Result<(), CoplanarOverlapSplitValidationError> {
        validate_coplanar_edge_split(self)?;
        validate_coplanar_edge_split_against_edges(self, &left_edge, &right_edge)
    }
}

/// Non-mutating split-construction plan for retained coplanar overlap graphs.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarOverlapSplitPlan {
    /// Per-face-pair overlap graph split records.
    pub graphs: Vec<CoplanarOverlapSplitGraph>,
}

/// Split construction records for one coplanar face-pair overlap graph.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarOverlapSplitGraph {
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

/// Readiness status for the future exact planar-cell extraction stage.
///
/// The current port can already retain certified coplanar edge and
/// vertex-contact facts. Full planar arrangements need a later stage that
/// callers infer it from a generic unsupported boolean result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarArrangementReadinessStatus {
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
/// their validated counts at the API boundary where the exact port still lacks
/// general multi-component planar-cell extraction. It gives fuzzing,
/// benchmarks, and downstream planners a checked handoff rather than a plain
/// "not implemented" flag.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CoplanarArrangementReadinessReport {
    /// Coarse state of the retained coplanar arrangement evidence.
    pub status: CoplanarArrangementReadinessStatus,
    /// Number of retained coplanar overlap graphs.
    pub graph_count: usize,
    /// Number of graphs whose coarse relation is positive-area overlap.
    pub overlapping_graphs: usize,
    /// Number of graphs whose coarse relation is boundary-only touching.
    pub touching_graphs: usize,
    /// Number of retained non-disjoint edge contacts.
    pub edge_overlap_count: usize,
    /// Number of retained vertex-in-triangle or vertex-on-triangle facts.
    pub vertex_overlap_count: usize,
    /// Number of exact point split constructions retained for proper or
    /// endpoint edge contacts.
    pub point_split_count: usize,
    /// Number of positive-length collinear interval contacts retained for the
    /// future planar arrangement pass.
    pub interval_overlap_count: usize,
    /// Number of exact interval endpoint facts retained for collinear contacts.
    pub interval_endpoint_count: usize,
}

/// Structural inconsistency in a retained coplanar overlap graph.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarOverlapGraphValidationError {
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
    SourceReplayMismatch,
}

/// Structural inconsistency in retained coplanar split construction records.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarOverlapSplitValidationError {
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
    /// Recomputing coplanar split constructions from the supplied source
    /// meshes did not reproduce this retained split artifact.
    SourceReplayMismatch,
}

/// Structural inconsistency in a coplanar arrangement readiness report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarArrangementReadinessValidationError {
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
    /// Recomputing the readiness summary from the supplied source meshes did
    /// not reproduce this retained report.
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
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), CoplanarOverlapGraphValidationError> {
        self.validate()?;
        let replay = build_intersection_graph(left, right)
            .map(|graph| graph.coplanar_overlap_graphs())
            .map_err(|_| CoplanarOverlapGraphValidationError::SourceReplayMismatch)?;
        if replay.iter().any(|graph| graph == self) {
            Ok(())
        } else {
            Err(CoplanarOverlapGraphValidationError::SourceReplayMismatch)
        }
    }

    /// Construct exact point/interval records for this coplanar overlap graph.
    ///
    /// This is still a pre-topology artifact. It constructs point events for
    /// proper crossings and endpoint touches, and explicitly marks collinear
    /// interval contacts as interval topology for a later exact planar
    /// discipline: keep construction evidence with the graph instead of using
    /// projected predicate labels as if they were enough to mutate topology.
    pub fn split_constructions(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<CoplanarOverlapSplitGraph, MeshError> {
        coplanar_overlap_split_graph(self, left, right)
    }
}

impl CoplanarArrangementReadinessReport {
    /// Return whether later planar-cell extraction is required.
    pub const fn needs_planar_cells(&self) -> bool {
        matches!(
            self.status,
            CoplanarArrangementReadinessStatus::NeedsPlanarCells
        )
    }

    /// Validate that the compact readiness summary is internally coherent.
    ///
    /// The report validates counts, not source geometry. That is still useful
    /// because exact topology staging often crosses API or serialization
    /// model requires those retained numerical-structure summaries to be
    /// auditable before they influence combinatorial output.
    pub fn validate(&self) -> Result<(), CoplanarArrangementReadinessValidationError> {
        if self.graph_count != self.overlapping_graphs + self.touching_graphs {
            return Err(CoplanarArrangementReadinessValidationError::GraphCountMismatch);
        }
        // exact state. A compact planar-readiness report is therefore allowed
        // to summarize split constructions, but those summaries must still be
        // dominated by the edge contacts that produced them.
        if self.point_split_count + self.interval_overlap_count > self.edge_overlap_count {
            return Err(CoplanarArrangementReadinessValidationError::SplitCountExceedsEdgeEvidence);
        }
        if self.interval_endpoint_count != self.interval_overlap_count.saturating_mul(2) {
            return Err(CoplanarArrangementReadinessValidationError::IntervalEndpointCountMismatch);
        }
        if self.graph_count > 0 && self.edge_overlap_count == 0 && self.vertex_overlap_count == 0 {
            return Err(CoplanarArrangementReadinessValidationError::MissingOverlapEvidence);
        }
        match self.status {
            CoplanarArrangementReadinessStatus::NoCoplanarOverlap => {
                if self.graph_count == 0
                    && self.edge_overlap_count == 0
                    && self.vertex_overlap_count == 0
                    && self.point_split_count == 0
                    && self.interval_overlap_count == 0
                    && self.interval_endpoint_count == 0
                {
                    Ok(())
                } else {
                    Err(CoplanarArrangementReadinessValidationError::NoOverlapWithEvidence)
                }
            }
            CoplanarArrangementReadinessStatus::BoundaryOnly => {
                if self.overlapping_graphs != 0 {
                    return Err(
                        CoplanarArrangementReadinessValidationError::BoundaryOnlyHasOverlap,
                    );
                }
                if self.touching_graphs == 0 {
                    return Err(
                        CoplanarArrangementReadinessValidationError::BoundaryOnlyMissingTouchingGraph,
                    );
                }
                Ok(())
            }
            CoplanarArrangementReadinessStatus::NeedsPlanarCells => {
                if self.overlapping_graphs > 0 {
                    Ok(())
                } else {
                    Err(CoplanarArrangementReadinessValidationError::NeedsCellsMissingOverlap)
                }
            }
        }
    }

    /// Validate this readiness report against the source meshes that produced it.
    ///
    /// Local validation proves only that the compact counters are internally
    /// coherent. Source replay rebuilds the exact intersection graph and
    /// coplanar split summaries from `left` and `right`, then requires the
    /// summarized exact-topology handoff must remain attached to the predicate
    /// and construction history that produced its numerical structure.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), CoplanarArrangementReadinessValidationError> {
        self.validate()?;
        let replay = build_intersection_graph(left, right)
            .and_then(|graph| graph.coplanar_arrangement_readiness_report(left, right))
            .map_err(|_| CoplanarArrangementReadinessValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(CoplanarArrangementReadinessValidationError::SourceReplayMismatch)
        }
    }
}

impl CoplanarOverlapSplitPlan {
    /// Validate every retained coplanar split construction record.
    pub fn validate(&self) -> Result<(), CoplanarOverlapSplitValidationError> {
        for graph in &self.graphs {
            graph.validate()?;
        }
        Ok(())
    }

    /// Validate split records against the exact source meshes they reference.
    ///
    /// Plain split validation checks the self-contained construction record.
    /// This method additionally replays retained parameters against source
    /// edge geometry, which is the stronger handoff future planar-cell
    /// extraction should use when mesh handles are available.
    pub fn validate_against_meshes(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        for graph in &self.graphs {
            graph.validate_against_meshes(left, right)?;
        }
        Ok(())
    }

    /// Validate this split plan by replaying it from source operands.
    ///
    /// Mesh validation checks each retained split point against source-edge
    /// interpolation. This stronger audit also rebuilds the coplanar overlap
    /// graphs and split constructions from `left` and `right`, then compares
    /// should consume only split records whose construction history still
    /// replays from the source operands.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), CoplanarOverlapSplitValidationError> {
        self.validate()?;
        for graph in &self.graphs {
            graph.validate_against_sources(left, right)?;
        }
        let replay = build_intersection_graph(left, right)
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
        Ok(())
    }

    /// Validate split records against exact source mesh edge geometry.
    pub fn validate_against_meshes(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()
            .map_err(coplanar_split_validation_mesh_error)?;
        for split in &self.edge_splits {
            let left_edge = edge_points(left, split.overlap.left_edge)?;
            let right_edge = edge_points(right, split.overlap.right_edge)?;
            validate_coplanar_edge_split_against_edges(split, &left_edge, &right_edge)
                .map_err(coplanar_split_validation_mesh_error)?;
        }
        Ok(())
    }

    /// Validate this split graph by replaying it from source operands.
    ///
    /// This combines exact source-edge interpolation checks with a full replay
    /// of the coplanar split plan, then requires this graph to appear
    /// unchanged. It keeps interval and point-split construction records as
    /// certified objects rather than detachable projected labels, matching the
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), CoplanarOverlapSplitValidationError> {
        self.validate()?;
        for split in &self.edge_splits {
            let left_edge = edge_points(left, split.overlap.left_edge)
                .map_err(|_| CoplanarOverlapSplitValidationError::SourceReplayMismatch)?;
            let right_edge = edge_points(right, split.overlap.right_edge)
                .map_err(|_| CoplanarOverlapSplitValidationError::SourceReplayMismatch)?;
            validate_coplanar_edge_split_against_edges(split, &left_edge, &right_edge)?;
        }
        let replay = build_intersection_graph(left, right)
            .and_then(|graph| graph.coplanar_overlap_split_plan(left, right))
            .map_err(|_| CoplanarOverlapSplitValidationError::SourceReplayMismatch)?;
        if replay.graphs.iter().any(|graph| graph == self) {
            Ok(())
        } else {
            Err(CoplanarOverlapSplitValidationError::SourceReplayMismatch)
        }
    }
}

/// Structural inconsistency in a retained intersection graph event.
///
/// This validates the graph record before split extraction or topology
/// construction artifacts as the boundary between numerical decisions and
/// combinatorial mutation; a graph event whose coarse relation and retained
/// payload disagree must be rejected at that boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IntersectionGraphValidationError {
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
pub struct FacePairEvents {
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
    /// Return whether the pair contains at least one event that can drive graph
    /// construction.
    pub fn has_constructive_events(&self) -> bool {
        self.events.iter().any(|event| {
            !matches!(
                event,
                IntersectionEvent::CoplanarEdge {
                    relation: SegmentIntersection::Disjoint,
                    ..
                } | IntersectionEvent::CoplanarVertex {
                    location: TriangleLocation::Outside | TriangleLocation::Degenerate,
                    ..
                }
            )
        })
    }

    /// Validate one retained face-pair event record.
    ///
    /// This is a structural audit of the event graph object, not a recomputed
    /// triangle/triangle classification. It keeps the retained relation,
    /// projection, and event payloads consistent before downstream split
    /// planning converts construction records into topology.
    pub fn validate(&self) -> Result<(), IntersectionGraphValidationError> {
        if !face_pair_relation_needs_graph_construction(self.relation) {
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
            MeshFacePairRelation::BoundsDisjoint | MeshFacePairRelation::PlaneSeparated => {}
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
            validate_intersection_event(event)?;
        }
        Ok(())
    }

    /// Validate retained event handles against the exact source meshes.
    ///
    /// Plain [`FacePairEvents::validate`] checks relation/payload shape. This
    /// stronger handoff also checks that every retained face, edge, and vertex
    /// handle still belongs to the source meshes and to the face pair that
    /// handles as part of the exact state: a later topology stage must not
    /// consume predicate evidence after it has been relabeled onto a different
    /// source object.
    pub fn validate_against_meshes(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), IntersectionGraphValidationError> {
        self.validate()?;
        let left_tri = left
            .triangles()
            .get(self.left_face)
            .ok_or(IntersectionGraphValidationError::FaceIndexOutOfRange)?
            .0;
        let right_tri = right
            .triangles()
            .get(self.right_face)
            .ok_or(IntersectionGraphValidationError::FaceIndexOutOfRange)?
            .0;
        for event in &self.events {
            validate_intersection_event_sources(event, self, left, right, left_tri, right_tri)?;
        }
        Ok(())
    }

    /// Validate this face-pair event record by replaying source classification.
    ///
    /// Source-handle validation proves the retained events still point into
    /// the supplied meshes. This method additionally rebuilds the exact
    /// intersection graph from `left` and `right`, then requires this pair to
    /// event records are certified numerical/combinatorial objects, not labels
    /// that can be copied between face pairs.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), IntersectionGraphValidationError> {
        self.validate_against_meshes(left, right)?;
        let replay = build_intersection_graph(left, right)
            .map_err(|_| IntersectionGraphValidationError::SourceReplayMismatch)?;
        if replay.face_pairs.iter().any(|pair| pair == self) {
            Ok(())
        } else {
            Err(IntersectionGraphValidationError::SourceReplayMismatch)
        }
    }

    /// Group retained coplanar events into a non-mutating overlap graph.
    ///
    /// The returned graph is a structural arrangement input: it records which
    /// projected edges and vertices participate in the coplanar contact while
    /// leaving exact split construction and cell extraction to later stages.
    pub fn coplanar_overlap_graph(&self) -> Option<CoplanarOverlapGraph> {
        let projection = self.projection?;
        if !matches!(
            self.relation,
            MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping
        ) {
            return None;
        }

        let mut edge_overlaps = Vec::new();
        let mut vertex_overlaps = Vec::new();
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

        Some(CoplanarOverlapGraph {
            left_face: self.left_face,
            right_face: self.right_face,
            relation: self.relation,
            projection,
            edge_overlaps,
            vertex_overlaps,
        })
    }
}

/// Exact intersection event graph for two meshes.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactIntersectionGraph {
    /// Retained face-pair event records.
    pub face_pairs: Vec<FacePairEvents>,
}

impl ExactIntersectionGraph {
    /// Count all retained events.
    pub fn event_count(&self) -> usize {
        self.face_pairs.iter().map(|pair| pair.events.len()).sum()
    }

    /// Return whether any retained pair still needs a policy decision or
    /// additional refinement.
    pub fn has_unknowns(&self) -> bool {
        self.face_pairs.iter().any(|pair| {
            pair.relation == MeshFacePairRelation::Unknown
                || pair
                    .events
                    .iter()
                    .any(|event| matches!(event, IntersectionEvent::Unknown))
        })
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

    /// Validate this graph by replaying it from source operands.
    ///
    /// [`Self::validate_against_meshes`] checks that retained event handles
    /// still belong to `left` and `right`. Source replay rebuilds the graph
    /// from those operands and requires exact equality, making the whole graph
    /// geometric-computation boundary.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), IntersectionGraphValidationError> {
        self.validate_against_meshes(left, right)?;
        let replay = build_intersection_graph(left, right)
            .map_err(|_| IntersectionGraphValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(IntersectionGraphValidationError::SourceReplayMismatch)
        }
    }

    /// Return grouped coplanar overlap graphs for retained coplanar face pairs.
    pub fn coplanar_overlap_graphs(&self) -> Vec<CoplanarOverlapGraph> {
        self.face_pairs
            .iter()
            .filter_map(FacePairEvents::coplanar_overlap_graph)
            .collect()
    }

    /// Construct exact split-point/interval records for coplanar overlap graphs.
    pub fn coplanar_overlap_split_plan(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<CoplanarOverlapSplitPlan, MeshError> {
        let graphs = self
            .coplanar_overlap_graphs()
            .iter()
            .map(|graph| graph.split_constructions(left, right))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(CoplanarOverlapSplitPlan { graphs })
    }

    /// Summarize retained coplanar overlap evidence for planar-cell extraction.
    ///
    /// The report first validates each retained overlap graph and its split
    /// construction records, then collapses them to counts that explain whether
    /// a named operation is blocked on boundary policy or true planar-cell
    /// evidence is preserved and checked, while the missing cell extraction
    /// algorithm remains an explicit status rather than a tolerance fallback.
    pub fn coplanar_arrangement_readiness_report(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<CoplanarArrangementReadinessReport, MeshError> {
        // Planar-readiness is a public compact view of retained graph state.
        // Before collapsing counts, replay the graph's face/edge/vertex handles
        // against the source meshes and later replay split parameters against
        // state; stale handles must not survive simply because the summary
        // counters are internally coherent.
        self.validate_against_meshes(left, right).map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("retained coplanar arrangement graph failed source replay: {error:?}"),
            ))
        })?;
        let overlap_graphs = self.coplanar_overlap_graphs();
        if overlap_graphs.is_empty() {
            return Ok(CoplanarArrangementReadinessReport {
                status: CoplanarArrangementReadinessStatus::NoCoplanarOverlap,
                graph_count: 0,
                overlapping_graphs: 0,
                touching_graphs: 0,
                edge_overlap_count: 0,
                vertex_overlap_count: 0,
                point_split_count: 0,
                interval_overlap_count: 0,
                interval_endpoint_count: 0,
            });
        }

        let mut overlapping_graphs = 0;
        let mut touching_graphs = 0;
        let mut edge_overlap_count = 0;
        let mut vertex_overlap_count = 0;
        let mut point_split_count = 0;
        let mut interval_overlap_count = 0;
        let mut interval_endpoint_count = 0;

        for graph in &overlap_graphs {
            graph.validate().map_err(|_| MeshError {
                diagnostics: vec![MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::UnsupportedExactOperation,
                    "retained coplanar overlap graph failed readiness validation",
                )],
            })?;
            match graph.relation {
                MeshFacePairRelation::CoplanarOverlapping => overlapping_graphs += 1,
                MeshFacePairRelation::CoplanarTouching => touching_graphs += 1,
                _ => {}
            }
            edge_overlap_count += graph.edge_overlaps.len();
            vertex_overlap_count += graph.vertex_overlaps.len();

            let split = graph.split_constructions(left, right)?;
            split
                .validate_against_meshes(left, right)
                .map_err(|error| {
                    MeshError::one(MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::UnsupportedExactOperation,
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
            CoplanarArrangementReadinessStatus::NeedsPlanarCells
        } else {
            CoplanarArrangementReadinessStatus::BoundaryOnly
        };
        let report = CoplanarArrangementReadinessReport {
            status,
            graph_count: overlap_graphs.len(),
            overlapping_graphs,
            touching_graphs,
            edge_overlap_count,
            vertex_overlap_count,
            point_split_count,
            interval_overlap_count,
            interval_endpoint_count,
        };
        report.validate().map_err(|_| MeshError {
            diagnostics: vec![MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                "coplanar arrangement readiness report failed validation",
            )],
        })?;
        Ok(report)
    }

    /// Extract exact edge split parameters from segment/plane events.
    ///
    /// The plan keeps split points grouped by directed mesh edge. Parameters
    /// are sorted only through `hyperlimit::compare_reals`; unresolved
    /// comparisons are counted rather than replaced with a primitive-float
    /// fallback.
    pub fn edge_split_plan(&self) -> ExactEdgeSplitPlan {
        edge_split_plan(self)
    }

    /// Merge coincident exact split points into graph vertices.
    ///
    /// Equality is tested coordinate-by-coordinate through
    /// `hyperlimit::compare_reals`. Unknown comparisons do not merge points;
    /// they increment [`ExactGraphVertexPlan::unresolved_equalities`] so a
    /// caller can choose a refinement or unsupported-degeneracy policy.
    pub fn graph_vertex_plan(&self) -> ExactGraphVertexPlan {
        graph_vertex_plan(&self.edge_split_plan())
    }

    /// Merge coincident split points after validating edge split facts.
    ///
    /// This checked entry point rejects invalid segment/plane construction
    /// facts before point equality is used to form graph vertices. That keeps
    /// topology consume coordinates whose construction context has already
    pub fn checked_graph_vertex_plan(
        &self,
    ) -> Result<ExactGraphVertexPlan, SplitPlanValidationReport> {
        let edge_splits = self.edge_split_plan();
        let edge_report = edge_splits.validate();
        if !edge_report.is_valid() {
            return Err(edge_report);
        }
        let graph_vertices = graph_vertex_plan(&edge_splits);
        let graph_report = graph_vertices.validate();
        if graph_report.is_valid() {
            Ok(graph_vertices)
        } else {
            Err(graph_report)
        }
    }

    /// Build a non-mutating split-topology plan.
    ///
    /// The plan maps each split edge to an ordered chain from the original
    /// start vertex through merged exact graph vertices to the original end
    /// vertex. It is deliberately still a plan, not a halfedge mutation.
    pub fn split_topology_plan(&self) -> ExactSplitTopologyPlan {
        let edge_splits = self.edge_split_plan();
        let graph_vertices = graph_vertex_plan(&edge_splits);
        split_topology_plan(&edge_splits, &graph_vertices)
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
        let edge_splits = self.edge_split_plan();
        let edge_report = edge_splits.validate();
        if !edge_report.is_valid() {
            return Err(edge_report);
        }
        let graph_vertices = graph_vertex_plan(&edge_splits);
        let graph_report = graph_vertices.validate();
        if !graph_report.is_valid() {
            return Err(graph_report);
        }
        let topology = split_topology_plan(&edge_splits, &graph_vertices);
        let topology_report = topology.validate();
        if topology_report.is_valid() {
            Ok(topology)
        } else {
            Err(topology_report)
        }
    }

    /// Build face-local split work items from the split topology plan.
    ///
    /// The result tells later triangulation which original face boundary edges
    /// gained graph vertices. It does not infer a polygonization or winding
    /// decision; those remain exact downstream steps.
    pub fn face_split_plan(&self) -> ExactFaceSplitPlan {
        face_split_plan(&self.split_topology_plan())
    }

    /// Build and validate face-local split work items from checked topology.
    ///
    /// This keeps the face-local handoff explicit: topology chains are checked
    /// first, then face work items must prove every referenced graph vertex has
    /// a matching exact source use on that face edge before boundary geometry
    /// is materialized.
    pub fn checked_face_split_plan(&self) -> Result<ExactFaceSplitPlan, SplitPlanValidationReport> {
        let topology = self.checked_split_topology_plan()?;
        let face_plan = face_split_plan(&topology);
        let face_report = face_plan.validate_against_topology(&topology);
        if face_report.is_valid() {
            Ok(face_plan)
        } else {
            Err(face_report)
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
    ) -> Result<ExactFaceSplitGeometryPlan, MeshError> {
        let topology = self
            .checked_split_topology_plan()
            .map_err(split_plan_report_to_mesh_error)?;
        let face_plan = face_split_plan(&topology);
        let face_report = face_plan.validate_against_topology(&topology);
        if !face_report.is_valid() {
            return Err(split_plan_report_to_mesh_error(face_report));
        }
        face_split_geometry_plan(left, right, &topology, &face_plan)
    }
}

/// Build an exact event graph from two exact meshes.
pub fn build_intersection_graph(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactIntersectionGraph, MeshError> {
    let classifications = classify_mesh_face_pairs(left, right)?;
    let face_pairs = classifications
        .iter()
        .map(|classification| events_for_face_pair(left, right, classification))
        .collect();
    Ok(ExactIntersectionGraph { face_pairs })
}

/// Exact split points for one directed mesh edge.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeSplit {
    /// Mesh side owning the edge.
    pub side: MeshSide,
    /// Directed edge endpoints in that mesh's vertex index space.
    pub edge: [usize; 2],
    /// Ordered split points when exact parameter comparisons were available.
    pub points: Vec<EdgeSplitPoint>,
}

/// One exact split point on an edge.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeSplitPoint {
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
pub struct ExactEdgeSplitPlan {
    /// Per-edge split points.
    pub splits: Vec<EdgeSplit>,
    /// Number of parameter comparisons that could not be certified.
    pub unknown_orderings: usize,
}

impl ExactEdgeSplitPlan {
    /// Count split points across all edges.
    pub fn point_count(&self) -> usize {
        self.splits.iter().map(|split| split.points.len()).sum()
    }

    /// Validate exact edge split events before graph-vertex merging.
    ///
    /// This is the first handoff after segment/plane construction. It keeps
    /// point still carries certified opposite endpoint-side facts before later
    /// stages collapse points into graph vertices and topology chains. See
    pub fn validate(&self) -> SplitPlanValidationReport {
        validate_edge_split_plan(self)
    }

    /// Validate edge split extraction by replaying from source operands.
    ///
    /// This rebuilds the exact intersection graph from `left` and `right`,
    /// extracts its edge split plan, and compares it with this artifact after
    /// local construction-fact validation. Replaying the first split handoff
    /// keeps segment/plane certificates attached to their original operands,
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> SplitPlanValidationReport {
        validate_edge_split_plan_against_sources(self, left, right)
    }
}

/// One merged exact graph vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactGraphVertex {
    /// Representative exact point.
    pub point: Point3,
    /// Split-point uses that are exactly coincident with the representative.
    pub uses: Vec<ExactGraphVertexUse>,
}

/// One source use of a merged graph vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactGraphVertexUse {
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
pub struct ExactGraphVertexPlan {
    /// Merged graph vertices.
    pub vertices: Vec<ExactGraphVertex>,
    /// Equality checks that could not be certified.
    pub unresolved_equalities: usize,
}

impl ExactGraphVertexPlan {
    /// Count retained source uses across all graph vertices.
    pub fn source_use_count(&self) -> usize {
        self.vertices.iter().map(|vertex| vertex.uses.len()).sum()
    }

    /// Validate merged graph vertices before topology consumes them.
    ///
    /// The graph-vertex plan is the first place where multiple exact
    /// facts instead of trusting the representative coordinate alone.
    pub fn validate(&self) -> SplitPlanValidationReport {
        validate_graph_vertex_plan(self)
    }

    /// Validate graph-vertex merging by replaying from source operands.
    ///
    /// Merged graph vertices are only meaningful for the exact split events
    /// that produced them. This method rebuilds those events from `left` and
    /// `right`, redoes the merge, and requires the public artifact to match the
    /// and later combinatorial topology.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> SplitPlanValidationReport {
        validate_graph_vertex_plan_against_sources(self, left, right)
    }
}

/// One node in an ordered split-edge chain.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SplitEdgeNode {
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
pub struct SplitEdgeChain {
    /// Mesh side owning the edge.
    pub side: MeshSide,
    /// Directed original edge.
    pub edge: [usize; 2],
    /// Chain from original start through split graph vertices to original end.
    pub nodes: Vec<SplitEdgeNode>,
}

/// Non-mutating exact split topology plan.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactSplitTopologyPlan {
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

impl ExactSplitTopologyPlan {
    /// Count new graph vertices referenced by all split edge chains.
    pub fn referenced_graph_vertices(&self) -> usize {
        self.edge_chains
            .iter()
            .flat_map(|chain| chain.nodes.iter())
            .filter(|node| matches!(node, SplitEdgeNode::GraphVertex { .. }))
            .count()
    }

    /// Validate the non-mutating split-topology contract.
    ///
    /// events from combinatorial edits. This report is the handoff check: it
    /// rejects unresolved exact comparisons and malformed chain references
    pub fn validate(&self) -> SplitPlanValidationReport {
        validate_split_topology_plan(self)
    }

    /// Validate split topology by replaying from source operands.
    ///
    /// The topology plan orders original edge endpoints and exact graph
    /// vertices into non-mutating chains. This source replay rebuilds the
    /// graph, graph-vertex merge, and topology from `left` and `right` before
    /// decisions remain tied to exact predicate and construction evidence.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> SplitPlanValidationReport {
        validate_split_topology_plan_against_sources(self, left, right)
    }
}

/// One split edge chain as used by an affected face.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FaceSplitEdge {
    /// Original face edge endpoints.
    pub edge: [usize; 2],
    /// Graph vertices on that edge in directed edge order.
    pub graph_vertices: Vec<usize>,
}

/// Face-local split work item.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FaceSplitPlan {
    /// Mesh side owning the face.
    pub side: MeshSide,
    /// Face index.
    pub face: usize,
    /// Split boundary edges for this face.
    pub edges: Vec<FaceSplitEdge>,
}

/// Non-mutating exact face split plan.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactFaceSplitPlan {
    /// Per-face split work items.
    pub faces: Vec<FaceSplitPlan>,
}

impl ExactFaceSplitPlan {
    /// Count graph-vertex references across all face work items.
    pub fn graph_vertex_references(&self) -> usize {
        self.faces
            .iter()
            .flat_map(|face| face.edges.iter())
            .map(|edge| edge.graph_vertices.len())
            .sum()
    }

    /// Validate face-local split work items against a split-topology plan.
    ///
    /// The face plan is still deliberately pre-triangulation: it only says
    /// which original face boundary edges were split by exact graph vertices.
    /// Validation keeps that narrow API honest by checking graph-vertex ranges,
    /// duplicate face-edge instructions, and that each referenced graph vertex
    /// has an exact source use on the requested face edge whose retained
    /// construction facts are still valid.
    pub fn validate_against_topology(
        &self,
        topology: &ExactSplitTopologyPlan,
    ) -> SplitPlanValidationReport {
        validate_face_split_plan(self, topology)
    }

    /// Validate face-local split work items by replaying from source operands.
    ///
    /// [`Self::validate_against_topology`] is useful when a caller already has
    /// a checked topology handoff. This source replay rebuilds the exact graph,
    /// topology, and face-local work items from `left` and `right`, then
    /// compares the rebuilt plan with this public artifact. That keeps the
    /// copied face work list tied to the certified predicate/construction
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> SplitPlanValidationReport {
        validate_face_split_plan_against_sources(self, left, right)
    }
}

/// Stable category for split-plan validation diagnostics.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SplitPlanDiagnosticKind {
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
    SourceReplayMismatch,
    /// A split face region has fewer than three boundary nodes.
    EmptyOrShortRegionBoundary,
    /// A split face region contains consecutive duplicate boundary nodes.
    DuplicateConsecutiveRegionNode,
}

/// One split-plan validation diagnostic.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SplitPlanDiagnostic {
    /// Stable diagnostic category.
    pub kind: SplitPlanDiagnosticKind,
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

/// Error returned when a split-plan validation report is itself malformed.
///
/// Split-plan diagnostics are public handoff evidence for exact graph,
/// topology, and region stages. A diagnostic without the location data needed
/// to interpret it is not useful to downstream exact-policy code. Keeping that
/// explicit artifacts, not prose-only failures.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SplitPlanReportValidationError {
    /// A diagnostic message was empty or whitespace only.
    EmptyMessage,
    /// A diagnostic is missing the mesh side required by its kind.
    MissingSide,
    /// A diagnostic is missing the face index required by its kind.
    MissingFace,
    /// A diagnostic is missing the directed edge required by its kind.
    MissingEdge,
    /// A diagnostic is missing the graph-vertex index required by its kind.
    MissingGraphVertex,
    /// A missing-source-face diagnostic did not retain either a face or graph
    /// vertex location.
    MissingLocation,
}

impl SplitPlanDiagnostic {
    fn new(kind: SplitPlanDiagnosticKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
            side: None,
            face: None,
            edge: None,
            graph_vertex: None,
        }
    }

    const fn with_side(mut self, side: MeshSide) -> Self {
        self.side = Some(side);
        self
    }

    const fn with_face(mut self, face: usize) -> Self {
        self.face = Some(face);
        self
    }

    const fn with_edge(mut self, edge: [usize; 2]) -> Self {
        self.edge = Some(edge);
        self
    }

    const fn with_graph_vertex(mut self, graph_vertex: usize) -> Self {
        self.graph_vertex = Some(graph_vertex);
        self
    }
}

fn split_plan_report_to_mesh_error(report: SplitPlanValidationReport) -> MeshError {
    MeshError::new(
        report
            .diagnostics
            .into_iter()
            .map(|diagnostic| {
                let mut mesh = MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::UnsupportedExactOperation,
                    diagnostic.message,
                );
                if let Some(face) = diagnostic.face {
                    mesh = mesh.with_face(face);
                }
                if let Some(edge) = diagnostic.edge {
                    mesh = mesh.with_edge(edge);
                }
                mesh
            })
            .collect(),
    )
}

/// Validation report for exact split topology and face split plans.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SplitPlanValidationReport {
    /// Diagnostics collected during validation.
    pub diagnostics: Vec<SplitPlanDiagnostic>,
}

impl SplitPlanValidationReport {
    /// Return whether the checked split plan is ready for the next exact stage.
    pub fn is_valid(&self) -> bool {
        self.diagnostics.is_empty()
    }

    /// Validate that diagnostics retain the structured locations their kinds
    /// require.
    ///
    /// This does not decide whether the underlying split plan is valid; use
    /// [`Self::is_valid`] for that. It audits the report object so callers can
    /// rely on its diagnostics as machine-readable exact handoff evidence.
    pub fn validate(&self) -> Result<(), SplitPlanReportValidationError> {
        for diagnostic in &self.diagnostics {
            validate_split_plan_diagnostic(diagnostic)?;
        }
        Ok(())
    }
}

fn validate_split_plan_diagnostic(
    diagnostic: &SplitPlanDiagnostic,
) -> Result<(), SplitPlanReportValidationError> {
    if diagnostic.message.trim().is_empty() {
        return Err(SplitPlanReportValidationError::EmptyMessage);
    }
    match diagnostic.kind {
        SplitPlanDiagnosticKind::UnknownOrdering
        | SplitPlanDiagnosticKind::UnresolvedEquality
        | SplitPlanDiagnosticKind::UnresolvedVertexLookup
        | SplitPlanDiagnosticKind::SourceReplayMismatch => Ok(()),
        SplitPlanDiagnosticKind::MissingEndpointSideFacts
        | SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts
        | SplitPlanDiagnosticKind::InvalidConstructionRatio
        | SplitPlanDiagnosticKind::EmptyOrShortEdgeChain
        | SplitPlanDiagnosticKind::WrongChainStart
        | SplitPlanDiagnosticKind::WrongChainEnd
        | SplitPlanDiagnosticKind::ChainSideMismatch => {
            require_side(diagnostic)?;
            require_edge(diagnostic)
        }
        SplitPlanDiagnosticKind::GraphVertexOutOfRange => {
            require_side(diagnostic)?;
            if diagnostic.graph_vertex.is_some() || diagnostic.face.is_some() {
                Ok(())
            } else {
                Err(SplitPlanReportValidationError::MissingLocation)
            }
        }
        SplitPlanDiagnosticKind::EmptyGraphVertexUses => require_graph_vertex(diagnostic),
        SplitPlanDiagnosticKind::EmptyFaceSplit
        | SplitPlanDiagnosticKind::EmptyOrShortRegionBoundary
        | SplitPlanDiagnosticKind::DuplicateConsecutiveRegionNode => {
            require_side(diagnostic)?;
            require_face(diagnostic)
        }
        SplitPlanDiagnosticKind::EmptyFaceSplitEdge
        | SplitPlanDiagnosticKind::DuplicateFaceSplitEdge => {
            require_side(diagnostic)?;
            require_face(diagnostic)?;
            require_edge(diagnostic)
        }
        SplitPlanDiagnosticKind::MissingFaceSplitSourceUse => {
            require_side(diagnostic)?;
            require_face(diagnostic)?;
            require_edge(diagnostic)?;
            require_graph_vertex(diagnostic)
        }
        SplitPlanDiagnosticKind::UnknownBoundaryIncidence
        | SplitPlanDiagnosticKind::BoundaryNodeOffFacePlane => {
            require_side(diagnostic)?;
            require_face(diagnostic)
        }
    }
}

fn require_side(diagnostic: &SplitPlanDiagnostic) -> Result<(), SplitPlanReportValidationError> {
    diagnostic
        .side
        .map(|_| ())
        .ok_or(SplitPlanReportValidationError::MissingSide)
}

fn require_face(diagnostic: &SplitPlanDiagnostic) -> Result<(), SplitPlanReportValidationError> {
    diagnostic
        .face
        .map(|_| ())
        .ok_or(SplitPlanReportValidationError::MissingFace)
}

fn require_edge(diagnostic: &SplitPlanDiagnostic) -> Result<(), SplitPlanReportValidationError> {
    diagnostic
        .edge
        .map(|_| ())
        .ok_or(SplitPlanReportValidationError::MissingEdge)
}

fn require_graph_vertex(
    diagnostic: &SplitPlanDiagnostic,
) -> Result<(), SplitPlanReportValidationError> {
    diagnostic
        .graph_vertex
        .map(|_| ())
        .ok_or(SplitPlanReportValidationError::MissingGraphVertex)
}

/// Exact boundary node for a split face.
///
/// The variants distinguish original source vertices, retained intersection
/// graph vertices, and later exact face-interior constructions. Keeping those
/// explicit construction evidence instead of relabeling coordinates as if they
/// came from an older source object.
#[derive(Clone, Debug, PartialEq)]
pub enum FaceSplitBoundaryNode {
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
pub struct FaceSplitBoundaryChain {
    /// Original directed face edge.
    pub edge: [usize; 2],
    /// Boundary nodes in directed edge order.
    pub nodes: Vec<FaceSplitBoundaryNode>,
}

/// Exact geometry handoff for one split face.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceSplitGeometry {
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
pub struct ExactFaceSplitGeometryPlan {
    /// Per-face exact boundary geometry.
    pub faces: Vec<FaceSplitGeometry>,
}

impl ExactFaceSplitGeometryPlan {
    /// Count exact graph vertices referenced by boundary geometry.
    pub fn graph_vertex_references(&self) -> usize {
        self.faces
            .iter()
            .flat_map(|face| face.boundary_chains.iter())
            .flat_map(|chain| chain.nodes.iter())
            .filter(|node| matches!(node, FaceSplitBoundaryNode::GraphVertex { .. }))
            .count()
    }

    /// Validate that every split boundary node lies on its original face plane.
    ///
    /// Segment/plane crossings create points that should be incident to the
    /// face whose boundary they are splitting. This check replays that
    /// incidence as exact `hyperlimit::orient3d_report` predicates rather than
    pub fn validate_boundary_incidence(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> SplitPlanValidationReport {
        validate_face_split_geometry_incidence(self, left, right)
    }

    /// Validate split-boundary geometry by replaying it from source operands.
    ///
    /// Boundary incidence proves that each retained point lies on the source
    /// face plane. This check also rebuilds the exact intersection graph,
    /// topology, and split-boundary geometry from `left` and `right`, then
    /// compares the rebuilt artifact with this value. The replay boundary is
    /// combinatorics are consumed only with their certified construction
    /// history still attached to the original operands.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> SplitPlanValidationReport {
        validate_face_split_geometry_against_sources(self, left, right)
    }

    /// Build full face-region boundary loops for downstream exact triangulation.
    ///
    /// The geometry handoff stores only split edge chains. This method expands
    /// each affected triangle into one boundary loop in original face-edge
    /// order, inserting exact graph vertices along the split edges. It still
    /// does not decide winding, ownership, or boolean output; those decisions
    /// computation separation.
    pub fn region_plan(&self, left: &ExactMesh, right: &ExactMesh) -> ExactFaceRegionPlan {
        face_region_plan(self, left, right)
    }
}

/// One pre-triangulation boundary loop for an affected face.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceRegionBoundary {
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
pub struct ExactFaceRegionPlan {
    /// One boundary loop per affected source face.
    pub regions: Vec<FaceRegionBoundary>,
}

impl ExactFaceRegionPlan {
    /// Count graph vertices referenced by all region loops.
    pub fn graph_vertex_references(&self) -> usize {
        self.regions
            .iter()
            .flat_map(|region| region.boundary.iter())
            .filter(|node| matches!(node, FaceSplitBoundaryNode::GraphVertex { .. }))
            .count()
    }

    /// Validate boundary-loop structure and original-face incidence.
    ///
    /// Region loops are the direct input expected by exact triangulation. This
    /// check rejects malformed loops and reuses exact plane-incidence
    /// predicates so downstream triangulation does not inherit unchecked
    /// construction assumptions.
    pub fn validate(&self, left: &ExactMesh, right: &ExactMesh) -> SplitPlanValidationReport {
        validate_face_region_plan(self, left, right)
    }

    /// Validate this region plan by replaying it from its source operands.
    ///
    /// Local loop validation proves that boundary nodes are structurally usable
    /// and incident to their source face planes. This stronger check rebuilds
    /// the exact intersection graph, split topology, split boundary geometry,
    /// and final region loops from `left` and `right`, then requires the public
    /// algorithms should pass certified algebraic artifacts across topology
    /// boundaries instead of trusting copied combinatorial state.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> SplitPlanValidationReport {
        validate_face_region_plan_against_sources(self, left, right)
    }
}

fn events_for_face_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    classification: &MeshFacePairClassification,
) -> FacePairEvents {
    let left_tri = left.triangles()[classification.left_face].0;
    let right_tri = right.triangles()[classification.right_face].0;
    let left_edges = triangle_edges(left_tri);
    let right_edges = triangle_edges(right_tri);
    let mut events = Vec::new();
    let mut projection = None;

    if let Some(triangle) = &classification.triangle {
        append_segment_plane_events(
            &mut events,
            MeshSide::Right,
            &right_edges,
            MeshSide::Left,
            classification.left_face,
            &triangle.right_edge_events,
        );
        append_segment_plane_events(
            &mut events,
            MeshSide::Left,
            &left_edges,
            MeshSide::Right,
            classification.right_face,
            &triangle.left_edge_events,
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
    let mut grouped = BTreeMap::<(u8, usize, usize), EdgeSplit>::new();
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
            let key = (side_key(*segment_side), edge[0], edge[1]);
            grouped
                .entry(key)
                .or_insert_with(|| EdgeSplit {
                    side: *segment_side,
                    edge: *edge,
                    points: Vec::new(),
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
        unknown_orderings += sort_split_points(&mut split.points);
    }
    ExactEdgeSplitPlan {
        splits,
        unknown_orderings,
    }
}

fn graph_vertex_plan(split_plan: &ExactEdgeSplitPlan) -> ExactGraphVertexPlan {
    let mut vertices = Vec::<ExactGraphVertex>::new();
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

            let mut matched = None;
            for (index, vertex) in vertices.iter().enumerate() {
                match points_equal(&point.point, &vertex.point) {
                    Some(true) => {
                        matched = Some(index);
                        break;
                    }
                    Some(false) => {}
                    None => unresolved_equalities += 1,
                }
            }

            if let Some(index) = matched {
                vertices[index].uses.push(vertex_use);
            } else {
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

fn validate_graph_vertex_plan(plan: &ExactGraphVertexPlan) -> SplitPlanValidationReport {
    let mut diagnostics = Vec::new();

    for _ in 0..plan.unresolved_equalities {
        diagnostics.push(SplitPlanDiagnostic::new(
            SplitPlanDiagnosticKind::UnresolvedEquality,
            "graph-vertex equality could not be certified",
        ));
    }

    for (index, vertex) in plan.vertices.iter().enumerate() {
        if vertex.uses.is_empty() {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::EmptyGraphVertexUses,
                    "graph vertex has no exact source uses",
                )
                .with_graph_vertex(index),
            );
            continue;
        }

        for vertex_use in &vertex.uses {
            push_graph_vertex_source_use_diagnostics(
                &mut diagnostics,
                index,
                vertex_use,
                "graph vertex source use determinant ratio does not match its parameter",
                "graph vertex source use was not certified by opposite strict endpoint sides",
                "graph vertex source use is missing endpoint side facts",
            );
        }
    }

    SplitPlanValidationReport { diagnostics }
}

fn validate_graph_vertex_plan_against_sources(
    plan: &ExactGraphVertexPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut report = validate_graph_vertex_plan(plan);
    if !report.is_valid() {
        return report;
    }

    let replay = build_intersection_graph(left, right).map(|graph| graph.graph_vertex_plan());
    match replay {
        Ok(replay) if replay == *plan => report,
        Ok(_) => {
            report.diagnostics.push(SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::SourceReplayMismatch,
                "graph-vertex plan does not match exact replay from source operands",
            ));
            report
        }
        Err(error) => {
            report.diagnostics.push(SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::SourceReplayMismatch,
                format!("graph-vertex plan source replay failed: {error}"),
            ));
            report
        }
    }
}

fn push_graph_vertex_source_use_diagnostics(
    diagnostics: &mut Vec<SplitPlanDiagnostic>,
    graph_vertex: usize,
    vertex_use: &ExactGraphVertexUse,
    ratio_message: &'static str,
    non_crossing_message: &'static str,
    missing_message: &'static str,
) {
    // construction object, not only the rounded coordinate it produced. Every
    // later graph/topology handoff therefore rechecks the determinant ratio and
    if !ratio_matches_parameter(&vertex_use.parameter_ratio, &vertex_use.parameter) {
        diagnostics.push(
            SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::InvalidConstructionRatio,
                ratio_message,
            )
            .with_side(vertex_use.side)
            .with_edge(vertex_use.edge)
            .with_graph_vertex(graph_vertex),
        );
    }

    match vertex_use.endpoint_sides {
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
        | [Some(PlaneSide::Below), Some(PlaneSide::Above)] => {}
        [Some(_), Some(_)] => diagnostics.push(
            SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts,
                non_crossing_message,
            )
            .with_side(vertex_use.side)
            .with_edge(vertex_use.edge)
            .with_graph_vertex(graph_vertex),
        ),
        _ => diagnostics.push(
            SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::MissingEndpointSideFacts,
                missing_message,
            )
            .with_side(vertex_use.side)
            .with_edge(vertex_use.edge)
            .with_graph_vertex(graph_vertex),
        ),
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
            if plane_mesh.triangles().get(*plane_face).is_none() {
                return Err(IntersectionGraphValidationError::EventSourceOutOfRange);
            }
            validate_edge_vertices(*segment_side, *edge, left, right)?;
            if !edge_in_triangle(*edge, segment_tri) {
                return Err(IntersectionGraphValidationError::EventSourceMismatch);
            }
            Ok(())
        }
        IntersectionEvent::CoplanarEdge {
            left_edge,
            right_edge,
            ..
        } => {
            validate_edge_vertices(MeshSide::Left, *left_edge, left, right)?;
            validate_edge_vertices(MeshSide::Right, *right_edge, left, right)?;
            if !edge_in_triangle(*left_edge, left_tri) || !edge_in_triangle(*right_edge, right_tri)
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
            if triangle_mesh.triangles().get(*triangle_face).is_none() {
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

fn validate_edge_vertices(
    side: MeshSide,
    edge: [usize; 2],
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<(), IntersectionGraphValidationError> {
    validate_vertex(side, edge[0], left, right)?;
    validate_vertex(side, edge[1], left, right)
}

fn validate_vertex(
    side: MeshSide,
    vertex: usize,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<(), IntersectionGraphValidationError> {
    let vertex_count = match side {
        MeshSide::Left => left.vertices().len(),
        MeshSide::Right => right.vertices().len(),
    };
    if vertex < vertex_count {
        Ok(())
    } else {
        Err(IntersectionGraphValidationError::EventSourceOutOfRange)
    }
}

fn edge_in_triangle(edge: [usize; 2], triangle: [usize; 3]) -> bool {
    triangle.contains(&edge[0]) && triangle.contains(&edge[1]) && edge[0] != edge[1]
}

fn face_pair_relation_needs_graph_construction(relation: MeshFacePairRelation) -> bool {
    matches!(
        relation,
        MeshFacePairRelation::Candidate
            | MeshFacePairRelation::CoplanarTouching
            | MeshFacePairRelation::CoplanarOverlapping
            | MeshFacePairRelation::Unknown
    )
}

fn validate_graph_segment_plane_event(
    relation: SegmentPlaneRelation,
    point: &Option<Point3>,
    parameter: &Option<Real>,
    parameter_ratio: &Option<SegmentPlaneParameterRatio>,
    construction_failure: &Option<SegmentPlaneConstructionFailure>,
    endpoint_sides: [Option<PlaneSide>; 2],
) -> Result<(), IntersectionGraphValidationError> {
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
                    && opposite_strict_sides(endpoint_sides)
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
            if opposite_strict_sides(endpoint_sides)
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

fn opposite_strict_sides(sides: [Option<PlaneSide>; 2]) -> bool {
    matches!(
        sides,
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
            | [Some(PlaneSide::Below), Some(PlaneSide::Above)]
    )
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
    let mut edge_chains = Vec::new();
    for split in &split_plan.splits {
        let mut nodes = Vec::with_capacity(split.points.len() + 2);
        nodes.push(SplitEdgeNode::OriginalVertex {
            side: split.side,
            vertex: split.edge[0],
        });
        for point in &split.points {
            match find_graph_vertex(&point.point, graph_vertices) {
                Some(index) => nodes.push(SplitEdgeNode::GraphVertex {
                    graph_vertex: index,
                }),
                None => unresolved_vertex_lookups += 1,
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
        unresolved_equalities: graph_vertices.unresolved_equalities,
        unknown_orderings: split_plan.unknown_orderings,
    }
}

fn validate_edge_split_plan(split_plan: &ExactEdgeSplitPlan) -> SplitPlanValidationReport {
    let mut diagnostics = Vec::new();

    for _ in 0..split_plan.unknown_orderings {
        diagnostics.push(SplitPlanDiagnostic::new(
            SplitPlanDiagnosticKind::UnknownOrdering,
            "edge split parameters have an uncertified ordering",
        ));
    }

    for split in &split_plan.splits {
        for point in &split.points {
            if !ratio_matches_parameter(&point.parameter_ratio, &point.parameter) {
                diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::InvalidConstructionRatio,
                        "edge split point determinant ratio does not match its parameter",
                    )
                    .with_side(split.side)
                    .with_edge(split.edge),
                );
            }
            match point.endpoint_sides {
                [Some(PlaneSide::Above), Some(PlaneSide::Below)]
                | [Some(PlaneSide::Below), Some(PlaneSide::Above)] => {}
                [Some(_), Some(_)] => diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts,
                        "edge split point was not certified by opposite strict endpoint sides",
                    )
                    .with_side(split.side)
                    .with_edge(split.edge),
                ),
                _ => diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::MissingEndpointSideFacts,
                        "edge split point is missing endpoint side facts",
                    )
                    .with_side(split.side)
                    .with_edge(split.edge),
                ),
            }
        }
    }

    SplitPlanValidationReport { diagnostics }
}

fn validate_edge_split_plan_against_sources(
    split_plan: &ExactEdgeSplitPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut report = validate_edge_split_plan(split_plan);
    if !report.is_valid() {
        return report;
    }

    let replay = build_intersection_graph(left, right).map(|graph| graph.edge_split_plan());
    match replay {
        Ok(replay) if replay == *split_plan => report,
        Ok(_) => {
            report.diagnostics.push(SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::SourceReplayMismatch,
                "edge split plan does not match exact replay from source operands",
            ));
            report
        }
        Err(error) => {
            report.diagnostics.push(SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::SourceReplayMismatch,
                format!("edge split plan source replay failed: {error}"),
            ));
            report
        }
    }
}

fn face_split_plan(topology: &ExactSplitTopologyPlan) -> ExactFaceSplitPlan {
    let mut faces = BTreeMap::<(u8, usize), FaceSplitPlan>::new();
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
                .entry((side_key(chain.side), face))
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

fn validate_split_topology_plan(topology: &ExactSplitTopologyPlan) -> SplitPlanValidationReport {
    let mut diagnostics = Vec::new();

    for _ in 0..topology.unknown_orderings {
        diagnostics.push(SplitPlanDiagnostic::new(
            SplitPlanDiagnosticKind::UnknownOrdering,
            "edge split parameters have an uncertified ordering",
        ));
    }
    for _ in 0..topology.unresolved_equalities {
        diagnostics.push(SplitPlanDiagnostic::new(
            SplitPlanDiagnosticKind::UnresolvedEquality,
            "graph-vertex equality could not be certified",
        ));
    }
    for _ in 0..topology.unresolved_vertex_lookups {
        diagnostics.push(SplitPlanDiagnostic::new(
            SplitPlanDiagnosticKind::UnresolvedVertexLookup,
            "split point could not be matched to a graph vertex",
        ));
    }

    for (index, vertex) in topology.graph_vertices.iter().enumerate() {
        if vertex.uses.is_empty() {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::EmptyGraphVertexUses,
                    "graph vertex has no exact source uses",
                )
                .with_graph_vertex(index),
            );
        }

        for vertex_use in &vertex.uses {
            push_graph_vertex_source_use_diagnostics(
                &mut diagnostics,
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
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::EmptyOrShortEdgeChain,
                    "split edge chain does not connect both original endpoints",
                )
                .with_side(chain.side)
                .with_edge(chain.edge),
            );
            continue;
        }

        if chain.nodes.first()
            != Some(&SplitEdgeNode::OriginalVertex {
                side: chain.side,
                vertex: chain.edge[0],
            })
        {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::WrongChainStart,
                    "split edge chain does not start at the directed edge start",
                )
                .with_side(chain.side)
                .with_edge(chain.edge),
            );
        }

        if chain.nodes.last()
            != Some(&SplitEdgeNode::OriginalVertex {
                side: chain.side,
                vertex: chain.edge[1],
            })
        {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::WrongChainEnd,
                    "split edge chain does not end at the directed edge end",
                )
                .with_side(chain.side)
                .with_edge(chain.edge),
            );
        }

        for node in &chain.nodes {
            match node {
                SplitEdgeNode::OriginalVertex { side, .. } if *side != chain.side => {
                    diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::ChainSideMismatch,
                            "original vertex node is on a different mesh side from its chain",
                        )
                        .with_side(chain.side)
                        .with_edge(chain.edge),
                    );
                }
                SplitEdgeNode::GraphVertex { graph_vertex }
                    if *graph_vertex >= topology.graph_vertices.len() =>
                {
                    diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::GraphVertexOutOfRange,
                            "split edge chain references a missing graph vertex",
                        )
                        .with_side(chain.side)
                        .with_edge(chain.edge)
                        .with_graph_vertex(*graph_vertex),
                    );
                }
                _ => {}
            }
        }
    }

    SplitPlanValidationReport { diagnostics }
}

fn validate_split_topology_plan_against_sources(
    topology: &ExactSplitTopologyPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut report = validate_split_topology_plan(topology);
    if !report.is_valid() {
        return report;
    }

    let replay = build_intersection_graph(left, right).map(|graph| graph.split_topology_plan());
    match replay {
        Ok(replay) if replay == *topology => report,
        Ok(_) => {
            report.diagnostics.push(SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::SourceReplayMismatch,
                "split topology plan does not match exact replay from source operands",
            ));
            report
        }
        Err(error) => {
            report.diagnostics.push(SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::SourceReplayMismatch,
                format!("split topology plan source replay failed: {error}"),
            ));
            report
        }
    }
}

fn validate_face_split_plan(
    face_plan: &ExactFaceSplitPlan,
    topology: &ExactSplitTopologyPlan,
) -> SplitPlanValidationReport {
    let mut diagnostics = Vec::new();

    for face in &face_plan.faces {
        if face.edges.is_empty() {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::EmptyFaceSplit,
                    "face split work item has no split edges",
                )
                .with_side(face.side)
                .with_face(face.face),
            );
        }

        let mut seen_edges = BTreeSet::new();
        for edge in &face.edges {
            if !seen_edges.insert(edge.edge) {
                diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::DuplicateFaceSplitEdge,
                        "face split work item repeats an original edge",
                    )
                    .with_side(face.side)
                    .with_face(face.face)
                    .with_edge(edge.edge),
                );
            }

            if edge.graph_vertices.is_empty() {
                diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::EmptyFaceSplitEdge,
                        "face split edge has no graph vertices",
                    )
                    .with_side(face.side)
                    .with_face(face.face)
                    .with_edge(edge.edge),
                );
            }

            for &graph_vertex in &edge.graph_vertices {
                let Some(vertex) = topology.graph_vertices.get(graph_vertex) else {
                    diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::GraphVertexOutOfRange,
                            "face split edge references a missing graph vertex",
                        )
                        .with_side(face.side)
                        .with_face(face.face)
                        .with_edge(edge.edge)
                        .with_graph_vertex(graph_vertex),
                    );
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
                    push_graph_vertex_source_use_diagnostics(
                        &mut diagnostics,
                        graph_vertex,
                        vertex_use,
                        "face split source use determinant ratio does not match its parameter",
                        "face split source use was not certified by opposite strict endpoint sides",
                        "face split source use is missing endpoint side facts",
                    );
                }

                if !matched_source {
                    diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::MissingFaceSplitSourceUse,
                            "face split edge graph vertex has no exact source use on this face edge",
                        )
                        .with_side(face.side)
                        .with_face(face.face)
                        .with_edge(edge.edge)
                        .with_graph_vertex(graph_vertex),
                    );
                }
            }
        }
    }

    SplitPlanValidationReport { diagnostics }
}

fn validate_face_split_plan_against_sources(
    face_plan: &ExactFaceSplitPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let topology =
        match build_intersection_graph(left, right).map(|graph| graph.split_topology_plan()) {
            Ok(topology) => topology,
            Err(error) => {
                return SplitPlanValidationReport {
                    diagnostics: vec![SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::SourceReplayMismatch,
                        format!("face split plan source replay failed: {error}"),
                    )],
                };
            }
        };

    let mut report = validate_face_split_plan(face_plan, &topology);
    if !report.is_valid() {
        return report;
    }

    let replay = face_split_plan(&topology);
    if replay != *face_plan {
        report.diagnostics.push(SplitPlanDiagnostic::new(
            SplitPlanDiagnosticKind::SourceReplayMismatch,
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
) -> Result<ExactFaceSplitGeometryPlan, MeshError> {
    if let Some(diagnostic) = first_face_geometry_error(left, right, topology, face_plan) {
        return Err(MeshError::one(diagnostic));
    }

    let chains = topology
        .edge_chains
        .iter()
        .map(|chain| ((side_key(chain.side), chain.edge[0], chain.edge[1]), chain))
        .collect::<BTreeMap<_, _>>();

    let mut faces = Vec::with_capacity(face_plan.faces.len());
    for face in &face_plan.faces {
        let mesh = mesh_for_side(face.side, left, right);
        let triangle = mesh.triangles()[face.face].0;
        let mut boundary_chains = Vec::with_capacity(face.edges.len());
        for edge in &face.edges {
            let chain = chains[&(side_key(face.side), edge.edge[0], edge.edge[1])];
            boundary_chains.push(FaceSplitBoundaryChain {
                edge: edge.edge,
                nodes: chain
                    .nodes
                    .iter()
                    .map(|node| face_boundary_node(face.side, node, left, right, topology))
                    .collect::<Result<Vec<_>, _>>()?,
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

fn first_face_geometry_error(
    left: &ExactMesh,
    right: &ExactMesh,
    topology: &ExactSplitTopologyPlan,
    face_plan: &ExactFaceSplitPlan,
) -> Option<MeshDiagnostic> {
    let chains = topology
        .edge_chains
        .iter()
        .map(|chain| ((side_key(chain.side), chain.edge[0], chain.edge[1]), chain))
        .collect::<BTreeMap<_, _>>();

    for face in &face_plan.faces {
        let mesh = mesh_for_side(face.side, left, right);
        if face.face >= mesh.triangles().len() {
            return Some(
                MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::IndexOutOfBounds,
                    "face split geometry references a missing face",
                )
                .with_face(face.face),
            );
        }
        for edge in &face.edges {
            if !chains.contains_key(&(side_key(face.side), edge.edge[0], edge.edge[1])) {
                return Some(
                    MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::IndexOutOfBounds,
                        "face split geometry references a missing split edge chain",
                    )
                    .with_face(face.face)
                    .with_edge(edge.edge),
                );
            }
            for &graph_vertex in &edge.graph_vertices {
                if graph_vertex >= topology.graph_vertices.len() {
                    return Some(
                        MeshDiagnostic::new(
                            Severity::Error,
                            DiagnosticKind::IndexOutOfBounds,
                            "face split geometry references a missing graph vertex",
                        )
                        .with_face(face.face)
                        .with_edge(edge.edge),
                    );
                }
            }
        }
    }

    None
}

fn face_boundary_node(
    side: MeshSide,
    node: &SplitEdgeNode,
    left: &ExactMesh,
    right: &ExactMesh,
    topology: &ExactSplitTopologyPlan,
) -> Result<FaceSplitBoundaryNode, MeshError> {
    match node {
        SplitEdgeNode::OriginalVertex {
            side: vertex_side,
            vertex,
        } if *vertex_side == side => {
            let mesh = mesh_for_side(side, left, right);
            let point = mesh.vertices().get(*vertex).ok_or_else(|| {
                MeshError::one(
                    MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::IndexOutOfBounds,
                        "split boundary references a missing original vertex",
                    )
                    .with_vertex(*vertex),
                )
            })?;
            Ok(FaceSplitBoundaryNode::OriginalVertex {
                vertex: *vertex,
                point: point.clone(),
            })
        }
        SplitEdgeNode::GraphVertex { graph_vertex } => {
            let vertex = topology.graph_vertices.get(*graph_vertex).ok_or_else(|| {
                MeshError::one(
                    MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::IndexOutOfBounds,
                        "split boundary references a missing graph vertex",
                    )
                    .with_vertex(*graph_vertex),
                )
            })?;
            Ok(FaceSplitBoundaryNode::GraphVertex {
                graph_vertex: *graph_vertex,
                point: vertex.point.clone(),
            })
        }
        SplitEdgeNode::OriginalVertex { vertex, .. } => Err(MeshError::one(
            MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexOutOfBounds,
                "split boundary original vertex is on the wrong mesh side",
            )
            .with_vertex(*vertex),
        )),
    }
}

fn mesh_for_side<'a>(side: MeshSide, left: &'a ExactMesh, right: &'a ExactMesh) -> &'a ExactMesh {
    match side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    }
}

fn validate_face_split_geometry_incidence(
    geometry: &ExactFaceSplitGeometryPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut diagnostics = Vec::new();

    for face in &geometry.faces {
        let mesh = mesh_for_side(face.side, left, right);
        if face.face >= mesh.triangles().len() {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::GraphVertexOutOfRange,
                    "split-face geometry references a missing source face",
                )
                .with_side(face.side)
                .with_face(face.face),
            );
            continue;
        }

        let triangle = mesh.triangles()[face.face].0;
        let a = mesh.vertices()[triangle[0]].clone();
        let b = mesh.vertices()[triangle[1]].clone();
        let c = mesh.vertices()[triangle[2]].clone();
        for chain in &face.boundary_chains {
            for node in &chain.nodes {
                let point = boundary_node_point(node);
                match orient3d_report(&a, &b, &c, point).value() {
                    Some(Sign::Zero) => {}
                    Some(Sign::Negative | Sign::Positive) => diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::BoundaryNodeOffFacePlane,
                            "split boundary node is not incident to its original face plane",
                        )
                        .with_side(face.side)
                        .with_face(face.face)
                        .with_edge(chain.edge),
                    ),
                    None => diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::UnknownBoundaryIncidence,
                            "split boundary node incidence could not be certified",
                        )
                        .with_side(face.side)
                        .with_face(face.face)
                        .with_edge(chain.edge),
                    ),
                }
            }
        }
    }

    SplitPlanValidationReport { diagnostics }
}

fn validate_face_split_geometry_against_sources(
    geometry: &ExactFaceSplitGeometryPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut report = validate_face_split_geometry_incidence(geometry, left, right);
    if !report.is_valid() {
        return report;
    }

    let replay = build_intersection_graph(left, right)
        .and_then(|graph| graph.face_split_geometry_plan(left, right));
    match replay {
        Ok(replay) if replay == *geometry => report,
        Ok(_) => {
            report.diagnostics.push(SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::SourceReplayMismatch,
                "split-face geometry does not match exact replay from source operands",
            ));
            report
        }
        Err(error) => {
            report.diagnostics.push(SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::SourceReplayMismatch,
                format!("split-face geometry source replay failed: {error}"),
            ));
            report
        }
    }
}

fn face_region_plan(
    geometry: &ExactFaceSplitGeometryPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactFaceRegionPlan {
    let mut regions = Vec::with_capacity(geometry.faces.len());
    for face in &geometry.faces {
        let mesh = mesh_for_side(face.side, left, right);
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
                    original_boundary_node(mesh, edge[0]),
                    original_boundary_node(mesh, edge[1]),
                ]
            };
            for node in nodes {
                push_boundary_node(&mut boundary, node);
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

fn validate_face_region_plan(
    plan: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut diagnostics = Vec::new();
    for region in &plan.regions {
        if region.boundary.len() < 3 {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::EmptyOrShortRegionBoundary,
                    "face region boundary has fewer than three nodes",
                )
                .with_side(region.side)
                .with_face(region.face),
            );
        }

        for window in region.boundary.windows(2) {
            if boundary_nodes_equal(&window[0], &window[1]) == Some(true) {
                diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::DuplicateConsecutiveRegionNode,
                        "face region boundary contains consecutive duplicate nodes",
                    )
                    .with_side(region.side)
                    .with_face(region.face),
                );
            }
        }

        let mesh = mesh_for_side(region.side, left, right);
        if region.face >= mesh.triangles().len() {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::GraphVertexOutOfRange,
                    "face region references a missing source face",
                )
                .with_side(region.side)
                .with_face(region.face),
            );
            continue;
        }

        let triangle = mesh.triangles()[region.face].0;
        let a = mesh.vertices()[triangle[0]].clone();
        let b = mesh.vertices()[triangle[1]].clone();
        let c = mesh.vertices()[triangle[2]].clone();
        for node in &region.boundary {
            let point = boundary_node_point(node);
            match orient3d_report(&a, &b, &c, point).value() {
                Some(Sign::Zero) => {}
                Some(Sign::Negative | Sign::Positive) => diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::BoundaryNodeOffFacePlane,
                        "face region boundary node is not incident to its source face plane",
                    )
                    .with_side(region.side)
                    .with_face(region.face),
                ),
                None => diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::UnknownBoundaryIncidence,
                        "face region boundary incidence could not be certified",
                    )
                    .with_side(region.side)
                    .with_face(region.face),
                ),
            }
        }
    }

    SplitPlanValidationReport { diagnostics }
}

fn validate_face_region_plan_against_sources(
    plan: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut report = validate_face_region_plan(plan, left, right);
    if !report.is_valid() {
        return report;
    }

    let replay = build_intersection_graph(left, right)
        .and_then(|graph| graph.face_split_geometry_plan(left, right))
        .map(|geometry| geometry.region_plan(left, right));
    match replay {
        Ok(replay) if replay == *plan => report,
        Ok(_) => {
            report.diagnostics.push(SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::SourceReplayMismatch,
                "face region plan does not match exact replay from source operands",
            ));
            report
        }
        Err(error) => {
            report.diagnostics.push(SplitPlanDiagnostic::new(
                SplitPlanDiagnosticKind::SourceReplayMismatch,
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
) -> Result<CoplanarOverlapSplitGraph, MeshError> {
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
) -> Result<CoplanarEdgeSplitConstruction, MeshError> {
    let left_edge = edge_points(left, overlap.left_edge)?;
    let right_edge = edge_points(right, overlap.right_edge)?;
    let (points, interval_overlap, interval) = match overlap.relation {
        SegmentIntersection::Disjoint => (Vec::new(), false, None),
        SegmentIntersection::EndpointTouch => {
            let point = endpoint_touch_split_point(&left_edge, &right_edge, projection);
            (point.into_iter().collect(), false, None)
        }
        SegmentIntersection::Proper => {
            let point = proper_coplanar_edge_split_point(&left_edge, &right_edge, projection);
            (point.into_iter().collect(), false, None)
        }
        SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => (
            Vec::new(),
            true,
            coplanar_edge_interval(&left_edge, &right_edge, projection),
        ),
    };
    let split = CoplanarEdgeSplitConstruction {
        overlap: overlap.clone(),
        points,
        interval_overlap,
        interval,
    };
    validate_coplanar_edge_split_against_edges(&split, &left_edge, &right_edge)
        .map_err(coplanar_split_validation_mesh_error)?;
    Ok(split)
}

fn validate_coplanar_edge_split(
    split: &CoplanarEdgeSplitConstruction,
) -> Result<(), CoplanarOverlapSplitValidationError> {
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
            // numerical structure needed by later combinatorial decisions.
            // These edge parameters are the compact structure a future
            // planar-cell extractor will sort and merge, so endpoint/proper
            // relation labels must agree with certified parameter positions
            // before the record can cross an API boundary.
            match split.overlap.relation {
                SegmentIntersection::EndpointTouch => {
                    if parameter_is_endpoint(&point.left_parameter)?
                        || parameter_is_endpoint(&point.right_parameter)?
                    {
                        Ok(())
                    } else {
                        Err(
                            CoplanarOverlapSplitValidationError::EndpointTouchWithoutEndpointParameter,
                        )
                    }
                }
                SegmentIntersection::Proper => {
                    if parameter_is_strict_interior(&point.left_parameter)?
                        && parameter_is_strict_interior(&point.right_parameter)?
                    {
                        Ok(())
                    } else {
                        Err(CoplanarOverlapSplitValidationError::ProperCrossingEndpointParameter)
                    }
                }
                SegmentIntersection::Disjoint
                | SegmentIntersection::CollinearOverlap
                | SegmentIntersection::Identical => unreachable!("outer relation arm filtered"),
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
            validate_interval_endpoint(&interval.endpoints[0])?;
            validate_interval_endpoint(&interval.endpoints[1])?;
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

fn validate_interval_endpoint(
    point: &CoplanarEdgeSplitPoint,
) -> Result<(), CoplanarOverlapSplitValidationError> {
    validate_unit_parameter(&point.left_parameter)?;
    validate_unit_parameter(&point.right_parameter)?;
    Ok(())
}

fn validate_coplanar_edge_split_against_edges(
    split: &CoplanarEdgeSplitConstruction,
    left_edge: &[Point3; 2],
    right_edge: &[Point3; 2],
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
    left_edge: &[Point3; 2],
    right_edge: &[Point3; 2],
) -> Result<(), CoplanarOverlapSplitValidationError> {
    let left_replayed = interpolate_point3(&left_edge[0], &left_edge[1], &point.left_parameter);
    match points_equal(&point.point, &left_replayed) {
        Some(true) => {}
        Some(false) => {
            return Err(CoplanarOverlapSplitValidationError::SplitPointDoesNotMatchLeftParameter);
        }
        None => return Err(CoplanarOverlapSplitValidationError::UnknownSplitPointEquality),
    }

    let right_replayed = interpolate_point3(&right_edge[0], &right_edge[1], &point.right_parameter);
    match points_equal(&point.point, &right_replayed) {
        Some(true) => Ok(()),
        Some(false) => {
            Err(CoplanarOverlapSplitValidationError::SplitPointDoesNotMatchRightParameter)
        }
        None => Err(CoplanarOverlapSplitValidationError::UnknownSplitPointEquality),
    }
}

fn coplanar_split_validation_mesh_error(error: CoplanarOverlapSplitValidationError) -> MeshError {
    MeshError {
        diagnostics: vec![MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!(
                "retained coplanar split construction failed source-edge validation: {error:?}"
            ),
        )],
    }
}

fn validate_unit_parameter(parameter: &Real) -> Result<(), CoplanarOverlapSplitValidationError> {
    let zero = Real::from(0);
    let one = Real::from(1);
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
}

fn parameter_is_endpoint(parameter: &Real) -> Result<bool, CoplanarOverlapSplitValidationError> {
    let zero = Real::from(0);
    let one = Real::from(1);
    match (
        compare_reals(parameter, &zero).value(),
        compare_reals(parameter, &one).value(),
    ) {
        (Some(Ordering::Equal), _) | (_, Some(Ordering::Equal)) => Ok(true),
        (Some(_), Some(_)) => Ok(false),
        _ => Err(CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder),
    }
}

fn parameter_is_strict_interior(
    parameter: &Real,
) -> Result<bool, CoplanarOverlapSplitValidationError> {
    let zero = Real::from(0);
    let one = Real::from(1);
    match (
        compare_reals(parameter, &zero).value(),
        compare_reals(parameter, &one).value(),
    ) {
        (Some(Ordering::Greater), Some(Ordering::Less)) => Ok(true),
        (Some(_), Some(_)) => Ok(false),
        _ => Err(CoplanarOverlapSplitValidationError::UnknownSplitParameterOrder),
    }
}

fn edge_points(mesh: &ExactMesh, edge: [usize; 2]) -> Result<[Point3; 2], MeshError> {
    let start = mesh.vertices().get(edge[0]).ok_or_else(|| {
        MeshError::one(
            MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexOutOfBounds,
                "coplanar overlap edge references a missing start vertex",
            )
            .with_vertex(edge[0]),
        )
    })?;
    let end = mesh.vertices().get(edge[1]).ok_or_else(|| {
        MeshError::one(
            MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexOutOfBounds,
                "coplanar overlap edge references a missing end vertex",
            )
            .with_vertex(edge[1]),
        )
    })?;
    Ok([start.clone(), end.clone()])
}

fn endpoint_touch_split_point(
    left: &[Point3; 2],
    right: &[Point3; 2],
    projection: CoplanarProjection,
) -> Option<CoplanarEdgeSplitPoint> {
    for (left_index, left_point) in left.iter().enumerate() {
        for (right_index, right_point) in right.iter().enumerate() {
            if projected_points_equal(left_point, right_point, projection)? {
                return Some(CoplanarEdgeSplitPoint {
                    point: left_point.clone(),
                    left_parameter: Real::from(left_index as i64),
                    right_parameter: Real::from(right_index as i64),
                });
            }
        }
    }
    let right_start = project_point3(&right[0], projection);
    let right_end = project_point3(&right[1], projection);
    for (left_index, left_point) in left.iter().enumerate() {
        let projected = project_point3(left_point, projection);
        if point_on_segment(&right_start, &right_end, &projected).value() == Some(true) {
            return Some(CoplanarEdgeSplitPoint {
                point: left_point.clone(),
                left_parameter: Real::from(left_index as i64),
                right_parameter: endpoint_parameter_on_segment(
                    left_point, &right[0], &right[1], projection,
                )?,
            });
        }
    }

    let left_start = project_point3(&left[0], projection);
    let left_end = project_point3(&left[1], projection);
    for (right_index, right_point) in right.iter().enumerate() {
        let projected = project_point3(right_point, projection);
        if point_on_segment(&left_start, &left_end, &projected).value() == Some(true) {
            return Some(CoplanarEdgeSplitPoint {
                point: right_point.clone(),
                left_parameter: endpoint_parameter_on_segment(
                    right_point,
                    &left[0],
                    &left[1],
                    projection,
                )?,
                right_parameter: Real::from(right_index as i64),
            });
        }
    }
    None
}

fn coplanar_edge_interval(
    left: &[Point3; 2],
    right: &[Point3; 2],
    projection: CoplanarProjection,
) -> Option<CoplanarEdgeInterval> {
    let mut endpoints = Vec::new();
    for (left_index, point) in left.iter().enumerate() {
        if let Some(right_parameter) =
            certified_endpoint_parameter_on_segment(point, &right[0], &right[1], projection)
        {
            push_interval_endpoint(
                &mut endpoints,
                CoplanarEdgeSplitPoint {
                    point: point.clone(),
                    left_parameter: Real::from(left_index as i64),
                    right_parameter,
                },
                projection,
            )?;
        }
    }
    for (right_index, point) in right.iter().enumerate() {
        if let Some(left_parameter) =
            certified_endpoint_parameter_on_segment(point, &left[0], &left[1], projection)
        {
            push_interval_endpoint(
                &mut endpoints,
                CoplanarEdgeSplitPoint {
                    point: point.clone(),
                    left_parameter,
                    right_parameter: Real::from(right_index as i64),
                },
                projection,
            )?;
        }
    }
    if endpoints.len() != 2 {
        return None;
    }
    endpoints.sort_by(
        |a, b| match compare_reals(&a.left_parameter, &b.left_parameter).value() {
            Some(ordering) => ordering,
            None => Ordering::Equal,
        },
    );
    if compare_reals(&endpoints[0].left_parameter, &endpoints[1].left_parameter).value()
        != Some(Ordering::Less)
    {
        return None;
    }
    Some(CoplanarEdgeInterval {
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
        endpoint_parameter_on_segment(point, start, end, projection)
    } else {
        None
    }
}

fn push_interval_endpoint(
    endpoints: &mut Vec<CoplanarEdgeSplitPoint>,
    candidate: CoplanarEdgeSplitPoint,
    projection: CoplanarProjection,
) -> Option<()> {
    for endpoint in endpoints.iter_mut() {
        if projected_points_equal(&endpoint.point, &candidate.point, projection)? {
            return Some(());
        }
    }
    endpoints.push(candidate);
    Some(())
}

fn endpoint_parameter_on_segment(
    point: &Point3,
    start: &Point3,
    end: &Point3,
    projection: CoplanarProjection,
) -> Option<Real> {
    projected_segment_parameter3(point, start, end, projection)
}

fn proper_coplanar_edge_split_point(
    left: &[Point3; 2],
    right: &[Point3; 2],
    projection: CoplanarProjection,
) -> Option<CoplanarEdgeSplitPoint> {
    let left_parameter =
        projected_line_parameter3(&left[0], &left[1], &right[0], &right[1], projection)?;
    let right_parameter =
        projected_line_parameter3(&right[0], &right[1], &left[0], &left[1], projection)?;
    let point = interpolate_point3(&left[0], &left[1], &left_parameter);
    Some(CoplanarEdgeSplitPoint {
        point,
        left_parameter,
        right_parameter,
    })
}

fn original_boundary_node(mesh: &ExactMesh, vertex: usize) -> FaceSplitBoundaryNode {
    FaceSplitBoundaryNode::OriginalVertex {
        vertex,
        point: mesh.vertices()[vertex].clone(),
    }
}

fn push_boundary_node(boundary: &mut Vec<FaceSplitBoundaryNode>, node: FaceSplitBoundaryNode) {
    if boundary
        .last()
        .is_some_and(|last| boundary_nodes_equal(last, &node) == Some(true))
    {
        return;
    }
    boundary.push(node);
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
    points_equal(boundary_node_point(left), boundary_node_point(right))
}

fn find_graph_vertex(point: &Point3, graph_vertices: &ExactGraphVertexPlan) -> Option<usize> {
    graph_vertices
        .vertices
        .iter()
        .position(|vertex| points_equal(point, &vertex.point) == Some(true))
}

fn sort_split_points(points: &mut [EdgeSplitPoint]) -> usize {
    let mut unknown_orderings = 0;
    points.sort_by(
        |left, right| match compare_reals(&left.parameter, &right.parameter).value() {
            Some(ordering) => ordering,
            None => {
                unknown_orderings += 1;
                Ordering::Equal
            }
        },
    );
    unknown_orderings
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

    for (vertex, location) in right_tri.into_iter().zip(coplanar.right_vertices_in_left) {
        append_vertex_event(
            events,
            MeshSide::Right,
            vertex,
            MeshSide::Left,
            left_face,
            location,
        );
    }
    for (vertex, location) in left_tri.into_iter().zip(coplanar.left_vertices_in_right) {
        append_vertex_event(
            events,
            MeshSide::Left,
            vertex,
            MeshSide::Right,
            right_face,
            location,
        );
    }
}

fn append_vertex_event(
    events: &mut Vec<IntersectionEvent>,
    vertex_side: MeshSide,
    vertex: usize,
    triangle_side: MeshSide,
    triangle_face: usize,
    location: Option<TriangleLocation>,
) {
    match location {
        Some(
            location @ (TriangleLocation::Inside
            | TriangleLocation::OnEdge
            | TriangleLocation::OnVertex),
        ) => {
            events.push(IntersectionEvent::CoplanarVertex {
                vertex_side,
                vertex,
                triangle_side,
                triangle_face,
                location,
            });
        }
        None => events.push(IntersectionEvent::Unknown),
        Some(TriangleLocation::Outside | TriangleLocation::Degenerate) => {}
    }
}

fn triangle_edges(tri: [usize; 3]) -> [[usize; 2]; 3] {
    [[tri[0], tri[1]], [tri[1], tri[2]], [tri[2], tri[0]]]
}

fn side_key(side: MeshSide) -> u8 {
    match side {
        MeshSide::Left => 0,
        MeshSide::Right => 1,
    }
}

fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    let x = compare_reals(&left.x, &right.x).value()?;
    let y = compare_reals(&left.y, &right.y).value()?;
    let z = compare_reals(&left.z, &right.z).value()?;
    Some(x == Ordering::Equal && y == Ordering::Equal && z == Ordering::Equal)
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
