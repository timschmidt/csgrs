//! Exact planar-arrangement evidence for retained coplanar graph state.
//!
//! This module is deliberately a certificate layer, not a general arrangement
//! materializer. It packages the retained coplanar overlap graph, exact split
//! constructions, and readiness counters into a replayable report that names
//! the specific obstacle left for the general planar-cell extractor. Exact
//! numerical and combinatorial evidence is retained and validated before a
//! later topology algorithm is allowed to mutate mesh cells.
//!
//! The obstacle vocabulary follows standard planar subdivision concerns:
//! vertices, edges, faces, incidences, and special cases such as overlapping
//! edges or high-valence branch vertices. Hypermesh keeps those cases explicit
//! until a certified cell traversal owns them.

use std::cmp::Ordering;

use hyperlimit::{Point2, Point3, compare_reals, project_point3};

use super::coplanar::CoplanarProjection;
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{
    CoplanarArrangementReadinessReport, CoplanarArrangementReadinessStatus,
    CoplanarOverlapSplitPlan, MeshSide, build_intersection_graph,
};
use super::mesh::ExactMesh;

/// The most specific retained obstacle for a general coplanar arrangement.
///
/// A value of [`Self::NoCoplanarOverlap`] or [`Self::BoundaryOnly`] means the
/// retained graph does not itself require a positive-area planar-cell
/// materializer. The remaining variants identify exact graph evidence that
/// must be consumed by a certified general planar arrangement stage before a
/// named boolean output can safely be produced.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlanarArrangementObstacle {
    /// No retained coplanar overlap graph exists.
    NoCoplanarOverlap,
    /// Retained evidence is lower-dimensional boundary contact only.
    BoundaryOnly,
    /// Positive-area coplanar overlap exists, but no more specific obstacle
    /// dominates the retained evidence.
    NeedsPlanarCells,
    /// More than one coplanar graph must be assembled into common cells.
    MultipleCoplanarGraphs,
    /// At least one projected edge pair overlaps over a positive-length
    /// interval.
    PositiveLengthEdgeOverlap,
    /// Retained boundary evidence is point-only.
    PointOnlyContact,
    /// Retained projected split points contain a high-valence branch point.
    BranchPoint,
    /// Retained evidence mixes projected edge contacts and vertex-in-triangle
    /// facts in the same general arrangement handoff.
    MixedEdgeAndVertexEvidence,
}

impl PlanarArrangementObstacle {
    /// Return whether this obstacle still needs general arrangement topology.
    pub const fn requires_general_arrangement(self) -> bool {
        matches!(
            self,
            Self::NeedsPlanarCells
                | Self::MultipleCoplanarGraphs
                | Self::PositiveLengthEdgeOverlap
                | Self::PointOnlyContact
                | Self::BranchPoint
                | Self::MixedEdgeAndVertexEvidence
        )
    }
}

/// Validation failure for an exact planar-arrangement evidence report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactPlanarArrangementEvidenceError {
    /// The embedded readiness report is not internally coherent.
    InvalidReadiness,
    /// The replayed coplanar split plan is not internally coherent.
    InvalidSplitPlan,
    /// The report's split graph count does not match readiness evidence.
    SplitGraphCountMismatch,
    /// The report's split edge count is inconsistent with split evidence.
    SplitEdgeCountMismatch,
    /// The report's point split count does not match readiness evidence.
    PointSplitCountMismatch,
    /// The report's interval overlap count does not match readiness evidence.
    IntervalOverlapCountMismatch,
    /// The report's interval endpoint count does not match readiness evidence.
    IntervalEndpointCountMismatch,
    /// The report's vertex overlap count does not match readiness evidence.
    VertexOverlapCountMismatch,
    /// The report's point-only contact count is inconsistent with status.
    PointOnlyContactCountMismatch,
    /// Branch count and maximum incident edge count disagree.
    BranchPointCountMismatch,
    /// Branch side-participation counters are inconsistent with branch points.
    BranchSideCountMismatch,
    /// The derived obstacle does not match the retained report.
    ObstacleMismatch,
    /// A projected split-point equality could not be certified exactly.
    UnresolvedProjectedEquality,
    /// Recomputing the report from source meshes did not reproduce it.
    SourceReplayMismatch,
}

/// Freshness status for retained planar-arrangement evidence.
///
/// This enum is intentionally more specific than a boolean cache-valid flag.
/// combinatorial state as separate artifacts that must replay together. A
/// copied planar-arrangement handoff can therefore be locally malformed,
/// locally stale with respect to one family of counters, or locally coherent
/// but no longer produced by the source meshes it names.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactPlanarArrangementEvidenceFreshness {
    /// The report validates locally and replays exactly from the source meshes.
    Current,
    /// The embedded coplanar-readiness summary is no longer coherent.
    InvalidReadiness,
    /// The split-plan replay that produced the report was malformed.
    InvalidSplitPlan,
    /// Retained split graph, edge, point, interval, or vertex counts drifted.
    StaleSplitCounts,
    /// Point-only contact counts no longer match the readiness status.
    StalePointOnlyContacts,
    /// Branch-point counters no longer match retained projected incidence.
    StaleBranchPoints,
    /// The named obstacle no longer matches retained exact evidence.
    StaleObstacle,
    /// Exact projected equality could not be certified during replay.
    UnresolvedProjectedEquality,
    /// The report is locally valid but no longer replays from the sources.
    SourceReplayMismatch,
}

impl From<ExactPlanarArrangementEvidenceError> for ExactPlanarArrangementEvidenceFreshness {
    fn from(error: ExactPlanarArrangementEvidenceError) -> Self {
        match error {
            ExactPlanarArrangementEvidenceError::InvalidReadiness => Self::InvalidReadiness,
            ExactPlanarArrangementEvidenceError::InvalidSplitPlan => Self::InvalidSplitPlan,
            ExactPlanarArrangementEvidenceError::SplitGraphCountMismatch
            | ExactPlanarArrangementEvidenceError::SplitEdgeCountMismatch
            | ExactPlanarArrangementEvidenceError::PointSplitCountMismatch
            | ExactPlanarArrangementEvidenceError::IntervalOverlapCountMismatch
            | ExactPlanarArrangementEvidenceError::IntervalEndpointCountMismatch
            | ExactPlanarArrangementEvidenceError::VertexOverlapCountMismatch => {
                Self::StaleSplitCounts
            }
            ExactPlanarArrangementEvidenceError::PointOnlyContactCountMismatch => {
                Self::StalePointOnlyContacts
            }
            ExactPlanarArrangementEvidenceError::BranchPointCountMismatch
            | ExactPlanarArrangementEvidenceError::BranchSideCountMismatch => {
                Self::StaleBranchPoints
            }
            ExactPlanarArrangementEvidenceError::ObstacleMismatch => Self::StaleObstacle,
            ExactPlanarArrangementEvidenceError::UnresolvedProjectedEquality => {
                Self::UnresolvedProjectedEquality
            }
            ExactPlanarArrangementEvidenceError::SourceReplayMismatch => Self::SourceReplayMismatch,
        }
    }
}

/// Replayable evidence summary for the missing general planar arrangement.
///
/// The report carries only stable counts and the validated readiness summary,
/// not raw point coordinates. Raw coplanar split records remain in
/// [`CoplanarOverlapSplitPlan`]. This compact boundary is useful for reports,
/// fuzzing, and downstream schedulers while still requiring source replay
/// before the evidence can be reused.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactPlanarArrangementEvidenceReport {
    /// Compact readiness summary derived from retained coplanar graph state.
    pub readiness: CoplanarArrangementReadinessReport,
    /// Number of split graphs retained for coplanar face pairs.
    pub split_graph_count: usize,
    /// Number of retained edge split construction records.
    pub split_edge_count: usize,
    /// Number of exact point split constructions.
    pub point_split_count: usize,
    /// Number of positive-length interval overlap constructions.
    pub interval_overlap_count: usize,
    /// Number of exact interval endpoints retained for interval overlaps.
    pub interval_endpoint_count: usize,
    /// Number of copied vertex containment/touch facts.
    pub vertex_overlap_count: usize,
    /// Number of lower-dimensional point-only contacts when the retained graph
    /// is boundary-only.
    pub point_only_contact_count: usize,
    /// Number of projected split points with more than two incident source
    /// edges in the retained split evidence.
    pub branch_point_count: usize,
    /// Number of branch points touched by at least one left-mesh source edge or
    /// vertex event.
    ///
    /// This keeps source-side ownership at the exact handoff boundary instead
    /// of collapsing a high-valence projected point into an unlabeled count.
    /// is the incidence information a later arrangement traversal needs in the
    pub left_branch_point_count: usize,
    /// Number of branch points touched by at least one right-mesh source edge
    /// or vertex event.
    pub right_branch_point_count: usize,
    /// Number of branch points with source incidence from both operands.
    pub mixed_side_branch_point_count: usize,
    /// Largest retained projected split-point incidence count.
    pub max_incident_edges_at_projected_point: usize,
    /// Most specific obstacle exposed by the retained evidence.
    pub obstacle: PlanarArrangementObstacle,
}

impl ExactPlanarArrangementEvidenceReport {
    /// Build a compact report from already-validated readiness and split data.
    ///
    /// This method still validates the split plan locally and derives
    /// branch-point evidence by merging projected split coordinates with exact
    /// discipline by rejecting unresolved projected equality instead of using a
    /// primitive-float merge tolerance.
    pub fn from_split_plan(
        readiness: CoplanarArrangementReadinessReport,
        split_plan: &CoplanarOverlapSplitPlan,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Self, ExactPlanarArrangementEvidenceError> {
        readiness
            .validate()
            .map_err(|_| ExactPlanarArrangementEvidenceError::InvalidReadiness)?;
        split_plan
            .validate()
            .map_err(|_| ExactPlanarArrangementEvidenceError::InvalidSplitPlan)?;

        let mut split_edge_count = 0;
        let mut point_split_count = 0;
        let mut interval_overlap_count = 0;
        let mut interval_endpoint_count = 0;
        let mut vertex_overlap_count = 0;
        let mut branch_point_count = 0;
        let mut left_branch_point_count = 0;
        let mut right_branch_point_count = 0;
        let mut mixed_side_branch_point_count = 0;
        let mut max_incident_edges_at_projected_point = 0;

        for graph in &split_plan.graphs {
            let mut incidents = Vec::new();
            split_edge_count += graph.edge_splits.len();
            vertex_overlap_count += graph.vertex_overlaps.len();

            for split in &graph.edge_splits {
                point_split_count += split.points.len();
                for point in &split.points {
                    merge_projected_incident(
                        &mut incidents,
                        &point.point,
                        graph.projection,
                        SideIncidentCounts::edge_pair(),
                    )?;
                }
                if split.interval_overlap {
                    interval_overlap_count += 1;
                    if let Some(interval) = &split.interval {
                        interval_endpoint_count += interval.endpoints.len();
                        for endpoint in &interval.endpoints {
                            merge_projected_incident(
                                &mut incidents,
                                &endpoint.point,
                                graph.projection,
                                SideIncidentCounts::edge_pair(),
                            )?;
                        }
                    }
                }
            }

            for vertex in &graph.vertex_overlaps {
                let point = vertex_overlap_point(vertex.vertex_side, vertex.vertex, left, right)
                    .ok_or(ExactPlanarArrangementEvidenceError::SourceReplayMismatch)?;
                merge_projected_incident(
                    &mut incidents,
                    &point,
                    graph.projection,
                    SideIncidentCounts::vertex(vertex.vertex_side),
                )?;
            }

            for incident in &incidents {
                max_incident_edges_at_projected_point =
                    max_incident_edges_at_projected_point.max(incident.total());
                if incident.is_branch() {
                    branch_point_count += 1;
                    if incident.left_incident_count > 0 {
                        left_branch_point_count += 1;
                    }
                    if incident.right_incident_count > 0 {
                        right_branch_point_count += 1;
                    }
                    if incident.left_incident_count > 0 && incident.right_incident_count > 0 {
                        mixed_side_branch_point_count += 1;
                    }
                }
            }
        }

        let point_only_contact_count = if matches!(
            readiness.status,
            CoplanarArrangementReadinessStatus::BoundaryOnly
        ) {
            point_split_count + vertex_overlap_count
        } else {
            0
        };
        let obstacle = derive_obstacle(
            &readiness,
            point_only_contact_count,
            branch_point_count,
            split_plan.graphs.len(),
        );
        let report = Self {
            readiness,
            split_graph_count: split_plan.graphs.len(),
            split_edge_count,
            point_split_count,
            interval_overlap_count,
            interval_endpoint_count,
            vertex_overlap_count,
            point_only_contact_count,
            branch_point_count,
            left_branch_point_count,
            right_branch_point_count,
            mixed_side_branch_point_count,
            max_incident_edges_at_projected_point,
            obstacle,
        };
        report.validate()?;
        Ok(report)
    }

    /// Validate the compact report without source meshes.
    ///
    /// This checks that the report's counters are dominated by the embedded
    /// readiness evidence and that the named obstacle can be re-derived from
    /// those counters. Source replay is intentionally separate because copied
    /// evidence must be auditable both locally and against its originating
    /// operands.
    pub fn validate(&self) -> Result<(), ExactPlanarArrangementEvidenceError> {
        self.readiness
            .validate()
            .map_err(|_| ExactPlanarArrangementEvidenceError::InvalidReadiness)?;
        if self.split_graph_count != self.readiness.graph_count {
            return Err(ExactPlanarArrangementEvidenceError::SplitGraphCountMismatch);
        }
        if self.split_edge_count < self.interval_overlap_count {
            return Err(ExactPlanarArrangementEvidenceError::SplitEdgeCountMismatch);
        }
        if self.split_edge_count < self.point_split_count {
            return Err(ExactPlanarArrangementEvidenceError::SplitEdgeCountMismatch);
        }
        if self.point_split_count != self.readiness.point_split_count {
            return Err(ExactPlanarArrangementEvidenceError::PointSplitCountMismatch);
        }
        if self.interval_overlap_count != self.readiness.interval_overlap_count {
            return Err(ExactPlanarArrangementEvidenceError::IntervalOverlapCountMismatch);
        }
        if self.interval_endpoint_count != self.readiness.interval_endpoint_count {
            return Err(ExactPlanarArrangementEvidenceError::IntervalEndpointCountMismatch);
        }
        if self.vertex_overlap_count != self.readiness.vertex_overlap_count {
            return Err(ExactPlanarArrangementEvidenceError::VertexOverlapCountMismatch);
        }

        let expected_point_only_contact_count = if matches!(
            self.readiness.status,
            CoplanarArrangementReadinessStatus::BoundaryOnly
        ) {
            self.point_split_count + self.vertex_overlap_count
        } else {
            0
        };
        if self.point_only_contact_count != expected_point_only_contact_count {
            return Err(ExactPlanarArrangementEvidenceError::PointOnlyContactCountMismatch);
        }
        if (self.branch_point_count == 0 && self.max_incident_edges_at_projected_point > 2)
            || (self.branch_point_count > 0 && self.max_incident_edges_at_projected_point <= 2)
        {
            return Err(ExactPlanarArrangementEvidenceError::BranchPointCountMismatch);
        }
        if self.branch_point_count == 0 {
            if self.left_branch_point_count != 0
                || self.right_branch_point_count != 0
                || self.mixed_side_branch_point_count != 0
            {
                return Err(ExactPlanarArrangementEvidenceError::BranchSideCountMismatch);
            }
        } else if self.left_branch_point_count > self.branch_point_count
            || self.right_branch_point_count > self.branch_point_count
            || self.mixed_side_branch_point_count > self.left_branch_point_count
            || self.mixed_side_branch_point_count > self.right_branch_point_count
            || match self
                .left_branch_point_count
                .checked_add(self.right_branch_point_count)
            {
                Some(side_total) => side_total < self.branch_point_count,
                None => true,
            }
        {
            return Err(ExactPlanarArrangementEvidenceError::BranchSideCountMismatch);
        }
        let expected_obstacle = derive_obstacle(
            &self.readiness,
            self.point_only_contact_count,
            self.branch_point_count,
            self.split_graph_count,
        );
        if self.obstacle != expected_obstacle {
            return Err(ExactPlanarArrangementEvidenceError::ObstacleMismatch);
        }
        Ok(())
    }

    /// Validate this compact report by replaying it from exact source meshes.
    ///
    /// Source replay rebuilds the intersection graph, split plan, readiness
    /// summary, branch-point evidence, and obstacle classification from `left`
    /// computation contract: a retained arrangement handoff must still be the
    /// artifact produced by the operands it claims to summarize.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactPlanarArrangementEvidenceError> {
        self.validate()?;
        let replay = certify_planar_arrangement_evidence(left, right)
            .map_err(|_| ExactPlanarArrangementEvidenceError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactPlanarArrangementEvidenceError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained report is fresh for the source meshes.
    ///
    /// The check first validates local report integrity and only then replays
    /// the report from `left` and `right`. This preserves the object/predicate
    /// report whose internal counters were mutated from a coherent report that
    /// simply belongs to an older source configuration.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactPlanarArrangementEvidenceFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match certify_planar_arrangement_evidence(left, right) {
            Ok(replay) if self == &replay => ExactPlanarArrangementEvidenceFreshness::Current,
            Ok(_) | Err(_) => ExactPlanarArrangementEvidenceFreshness::SourceReplayMismatch,
        }
    }
}

/// Certify retained coplanar graph evidence for the general arrangement stage.
///
/// The function builds and validates the exact intersection graph, checks
/// coplanar split constructions against source edges, derives a compact
/// obstacle report, and validates that report before returning it. It does not
/// triangulate or materialize a general planar arrangement.
pub fn certify_planar_arrangement_evidence(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactPlanarArrangementEvidenceReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    graph
        .validate_against_meshes(left, right)
        .map_err(planar_graph_mesh_error)?;
    let readiness = graph.coplanar_arrangement_readiness_report(left, right)?;
    let split_plan = graph.coplanar_overlap_split_plan(left, right)?;
    split_plan.validate_against_meshes(left, right)?;
    let report =
        ExactPlanarArrangementEvidenceReport::from_split_plan(readiness, &split_plan, left, right)
            .map_err(planar_evidence_mesh_error)?;
    report.validate().map_err(planar_evidence_mesh_error)?;
    Ok(report)
}

#[derive(Clone, Debug)]
struct ProjectedIncidentPoint {
    point: Point2,
    left_incident_count: usize,
    right_incident_count: usize,
}

impl ProjectedIncidentPoint {
    fn total(&self) -> usize {
        self.left_incident_count
            .saturating_add(self.right_incident_count)
    }

    fn is_branch(&self) -> bool {
        self.total() > 2
    }
}

#[derive(Clone, Copy, Debug)]
struct SideIncidentCounts {
    left: usize,
    right: usize,
}

impl SideIncidentCounts {
    fn edge_pair() -> Self {
        Self { left: 1, right: 1 }
    }

    fn vertex(side: MeshSide) -> Self {
        match side {
            MeshSide::Left => Self { left: 1, right: 0 },
            MeshSide::Right => Self { left: 0, right: 1 },
        }
    }
}

fn derive_obstacle(
    readiness: &CoplanarArrangementReadinessReport,
    point_only_contact_count: usize,
    branch_point_count: usize,
    split_graph_count: usize,
) -> PlanarArrangementObstacle {
    match readiness.status {
        CoplanarArrangementReadinessStatus::NoCoplanarOverlap => {
            PlanarArrangementObstacle::NoCoplanarOverlap
        }
        CoplanarArrangementReadinessStatus::BoundaryOnly => {
            if branch_point_count > 0 {
                PlanarArrangementObstacle::BranchPoint
            } else if readiness.interval_overlap_count > 0 {
                PlanarArrangementObstacle::PositiveLengthEdgeOverlap
            } else if point_only_contact_count > 0 {
                PlanarArrangementObstacle::PointOnlyContact
            } else {
                PlanarArrangementObstacle::BoundaryOnly
            }
        }
        CoplanarArrangementReadinessStatus::NeedsPlanarCells => {
            if branch_point_count > 0 {
                PlanarArrangementObstacle::BranchPoint
            } else if split_graph_count > 1 {
                PlanarArrangementObstacle::MultipleCoplanarGraphs
            } else if readiness.interval_overlap_count > 0 {
                PlanarArrangementObstacle::PositiveLengthEdgeOverlap
            } else if readiness.edge_overlap_count > 0 && readiness.vertex_overlap_count > 0 {
                PlanarArrangementObstacle::MixedEdgeAndVertexEvidence
            } else {
                PlanarArrangementObstacle::NeedsPlanarCells
            }
        }
    }
}

fn merge_projected_incident(
    incidents: &mut Vec<ProjectedIncidentPoint>,
    point: &Point3,
    projection: CoplanarProjection,
    side_counts: SideIncidentCounts,
) -> Result<(), ExactPlanarArrangementEvidenceError> {
    let projected = project_point3(point, projection);
    for incident in incidents.iter_mut() {
        if projected_points_equal(&incident.point, &projected)? {
            incident.left_incident_count = incident
                .left_incident_count
                .saturating_add(side_counts.left);
            incident.right_incident_count = incident
                .right_incident_count
                .saturating_add(side_counts.right);
            return Ok(());
        }
    }
    incidents.push(ProjectedIncidentPoint {
        point: projected,
        left_incident_count: side_counts.left,
        right_incident_count: side_counts.right,
    });
    Ok(())
}

fn projected_points_equal(
    left: &Point2,
    right: &Point2,
) -> Result<bool, ExactPlanarArrangementEvidenceError> {
    let x_order = compare_reals(&left.x, &right.x)
        .value()
        .ok_or(ExactPlanarArrangementEvidenceError::UnresolvedProjectedEquality)?;
    let y_order = compare_reals(&left.y, &right.y)
        .value()
        .ok_or(ExactPlanarArrangementEvidenceError::UnresolvedProjectedEquality)?;
    Ok(x_order == Ordering::Equal && y_order == Ordering::Equal)
}

fn vertex_overlap_point(
    side: MeshSide,
    vertex: usize,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<Point3> {
    match side {
        MeshSide::Left => left.vertices().get(vertex),
        MeshSide::Right => right.vertices().get(vertex),
    }
    .map(|point| point.clone())
}

fn planar_evidence_mesh_error(error: ExactPlanarArrangementEvidenceError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact planar-arrangement evidence failed validation: {error:?}"),
    ))
}

fn planar_graph_mesh_error(error: super::graph::IntersectionGraphValidationError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("retained intersection graph failed exact source-mesh validation: {error:?}"),
    ))
}
