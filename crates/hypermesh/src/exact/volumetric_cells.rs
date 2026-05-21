//! Exact evidence for coplanar source-face cells in volumetric booleans.
//!
//! This module reports the retained graph state that separates ordinary
//! non-coplanar winding materialization from the remaining general coplanar
//! volumetric-cell extractor. It does not materialize arbitrary cells. Instead
//! it records whether a closed-solid overlap contains coplanar source-face
//! evidence, whether that evidence is only a boundary-contact candidate, and
//! whether it is mixed with non-coplanar crossing events that require a later
//! certified cell materializer.
//!
//! The boundary follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): exact predicates and construction
//! events are retained as auditable objects, and a missing topological
//! algorithm is represented as explicit exact state rather than as a tolerance
//! fallback. The cell-obstacle vocabulary is also aligned with Weiler and
//! Atherton, "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH
//! Computer Graphics* 11.2 (1977), where surface classification depends on
//! correctly retaining face/edge intersection structure before traversal.

use hyperlimit::{SegmentIntersection, TriangleLocation};

use super::construction::SegmentPlaneRelation;
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{ExactIntersectionGraph, IntersectionEvent, build_intersection_graph};
use super::intersection::MeshFacePairRelation;
use super::mesh::ExactMesh;

/// Most specific retained obstacle for volumetric coplanar source-face cells.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarVolumetricCellObstacle {
    /// At least one operand is not a closed two-manifold, so volumetric cell
    /// evidence is not semantically applicable.
    NonClosedOperand,
    /// The exact intersection graph retained no constructive face pairs.
    NoRetainedOverlap,
    /// Retained graph evidence has no coplanar source-face component.
    NonCoplanarOnly,
    /// Retained coplanar evidence is lower-dimensional boundary contact only.
    BoundaryOnlyContact,
    /// Coplanar source-face evidence remains, but no non-coplanar proper
    /// crossing was retained in the same graph.
    NeedsCoplanarVolumetricCells,
    /// Coplanar source-face cells are mixed with non-coplanar proper
    /// crossings and must be assembled by a certified volumetric cell pass.
    MixedCoplanarAndCrossingCells,
    /// Retained graph evidence contains unknown face pairs or unknown events.
    UnknownGraphEvidence,
    /// Retained graph evidence contains a failed exact construction event.
    ConstructionFailureEvidence,
}

impl CoplanarVolumetricCellObstacle {
    /// Return whether a certified coplanar volumetric-cell materializer is the
    /// next required topology stage.
    pub const fn requires_coplanar_volumetric_cells(self) -> bool {
        matches!(
            self,
            Self::NeedsCoplanarVolumetricCells | Self::MixedCoplanarAndCrossingCells
        )
    }
}

/// Validation failure for a coplanar volumetric-cell evidence report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarVolumetricCellEvidenceError {
    /// Relation counters do not sum to the retained face-pair count.
    FacePairCountMismatch,
    /// Candidate-pair counters contradict retained candidate count.
    CandidatePairCountMismatch,
    /// Segment/plane event counters contradict retained segment/plane count.
    SegmentPlaneEventCountMismatch,
    /// Coplanar event counters contradict retained coplanar face-pair count.
    CoplanarEvidenceMismatch,
    /// The retained obstacle does not match the report counters.
    ObstacleMismatch,
    /// Recomputing the report from source meshes did not reproduce it.
    SourceReplayMismatch,
}

/// Replayable summary of retained volumetric coplanar-cell evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CoplanarVolumetricCellEvidenceReport {
    /// Whether the left operand is a closed two-manifold.
    pub left_closed_manifold: bool,
    /// Whether the right operand is a closed two-manifold.
    pub right_closed_manifold: bool,
    /// Retained constructive face pairs in the exact intersection graph.
    pub retained_face_pair_count: usize,
    /// Retained candidate face pairs requiring graph events.
    pub candidate_pairs: usize,
    /// Candidate pairs that retain at least one proper segment/plane crossing.
    pub proper_crossing_candidate_pairs: usize,
    /// Retained coplanar touching face pairs.
    pub coplanar_touching_pairs: usize,
    /// Retained positive-area coplanar overlap face pairs.
    pub coplanar_overlapping_pairs: usize,
    /// Retained unknown face pairs.
    pub unknown_pairs: usize,
    /// Retained segment/plane events.
    pub segment_plane_events: usize,
    /// Segment/plane events certified as proper crossings.
    pub proper_crossing_events: usize,
    /// Segment/plane events that are exact boundary contacts or disjoint.
    pub boundary_segment_events: usize,
    /// Segment/plane events whose construction failed after predicate
    /// classification.
    pub construction_failed_events: usize,
    /// Retained unknown graph events.
    pub unknown_events: usize,
    /// Retained non-disjoint coplanar edge events.
    pub coplanar_edge_events: usize,
    /// Retained constructive coplanar vertex/triangle events.
    pub coplanar_vertex_events: usize,
    /// Most specific obstacle exposed by the retained evidence.
    pub obstacle: CoplanarVolumetricCellObstacle,
}

impl CoplanarVolumetricCellEvidenceReport {
    /// Build a report from a validated exact intersection graph.
    ///
    /// The graph is only counted here; source replay remains the job of
    /// [`Self::validate_against_sources`]. Counting exact retained events is
    /// still meaningful because it prevents a caller from reducing a
    /// coplanar-volumetric blocker to a boolean flag after the original graph
    /// has crossed an API boundary.
    pub fn from_graph(graph: &ExactIntersectionGraph, left: &ExactMesh, right: &ExactMesh) -> Self {
        let mut report = Self {
            left_closed_manifold: left.facts().mesh.closed_manifold,
            right_closed_manifold: right.facts().mesh.closed_manifold,
            retained_face_pair_count: graph.face_pairs.len(),
            candidate_pairs: 0,
            proper_crossing_candidate_pairs: 0,
            coplanar_touching_pairs: 0,
            coplanar_overlapping_pairs: 0,
            unknown_pairs: 0,
            segment_plane_events: 0,
            proper_crossing_events: 0,
            boundary_segment_events: 0,
            construction_failed_events: 0,
            unknown_events: 0,
            coplanar_edge_events: 0,
            coplanar_vertex_events: 0,
            obstacle: CoplanarVolumetricCellObstacle::NoRetainedOverlap,
        };

        for pair in &graph.face_pairs {
            match pair.relation {
                MeshFacePairRelation::Candidate => {
                    report.candidate_pairs += 1;
                    if pair.events.iter().any(proper_crossing_event) {
                        report.proper_crossing_candidate_pairs += 1;
                    }
                }
                MeshFacePairRelation::CoplanarTouching => report.coplanar_touching_pairs += 1,
                MeshFacePairRelation::CoplanarOverlapping => report.coplanar_overlapping_pairs += 1,
                MeshFacePairRelation::Unknown => report.unknown_pairs += 1,
                MeshFacePairRelation::BoundsDisjoint | MeshFacePairRelation::PlaneSeparated => {}
            }

            for event in &pair.events {
                match event {
                    IntersectionEvent::SegmentPlane { relation, .. } => {
                        report.segment_plane_events += 1;
                        match relation {
                            SegmentPlaneRelation::ProperCrossing => {
                                report.proper_crossing_events += 1
                            }
                            SegmentPlaneRelation::ConstructionFailed => {
                                report.construction_failed_events += 1
                            }
                            SegmentPlaneRelation::Disjoint
                            | SegmentPlaneRelation::Coplanar
                            | SegmentPlaneRelation::EndpointOnPlane => {
                                report.boundary_segment_events += 1
                            }
                            SegmentPlaneRelation::Unknown => report.unknown_events += 1,
                        }
                    }
                    IntersectionEvent::CoplanarEdge { relation, .. } => {
                        if *relation != SegmentIntersection::Disjoint {
                            report.coplanar_edge_events += 1;
                        }
                    }
                    IntersectionEvent::CoplanarVertex { location, .. } => {
                        if matches!(
                            location,
                            TriangleLocation::Inside
                                | TriangleLocation::OnEdge
                                | TriangleLocation::OnVertex
                        ) {
                            report.coplanar_vertex_events += 1;
                        }
                    }
                    IntersectionEvent::Unknown => report.unknown_events += 1,
                }
            }
        }

        report.obstacle = derive_obstacle(&report);
        report
    }

    /// Return the number of retained coplanar face pairs.
    pub const fn coplanar_face_pairs(&self) -> usize {
        self.coplanar_touching_pairs + self.coplanar_overlapping_pairs
    }

    /// Validate the compact report without source meshes.
    ///
    /// This is an integrity check for copied report data. Source replay is
    /// separate because exact object handles must be compared against the
    /// operands that produced them before the report can authorize topology.
    pub fn validate(&self) -> Result<(), CoplanarVolumetricCellEvidenceError> {
        let relation_count = self.candidate_pairs + self.coplanar_face_pairs() + self.unknown_pairs;
        if relation_count != self.retained_face_pair_count {
            return Err(CoplanarVolumetricCellEvidenceError::FacePairCountMismatch);
        }
        if self.proper_crossing_candidate_pairs > self.candidate_pairs {
            return Err(CoplanarVolumetricCellEvidenceError::CandidatePairCountMismatch);
        }
        if self.proper_crossing_events
            + self.boundary_segment_events
            + self.construction_failed_events
            > self.segment_plane_events
        {
            return Err(CoplanarVolumetricCellEvidenceError::SegmentPlaneEventCountMismatch);
        }
        if self.coplanar_face_pairs() == 0
            && (self.coplanar_edge_events > 0 || self.coplanar_vertex_events > 0)
        {
            return Err(CoplanarVolumetricCellEvidenceError::CoplanarEvidenceMismatch);
        }
        let expected = derive_obstacle(self);
        if self.obstacle != expected {
            return Err(CoplanarVolumetricCellEvidenceError::ObstacleMismatch);
        }
        Ok(())
    }

    /// Validate this report by replaying exact source mesh evidence.
    ///
    /// Source replay rebuilds the intersection graph from `left` and `right`,
    /// validates that graph against the same meshes, reconstructs this compact
    /// evidence report, and requires byte-for-byte equality. This keeps
    /// coplanar volumetric-cell blockers attached to the exact source objects
    /// whose predicates produced them, as required by Yap's exact-geometric-
    /// computation model.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), CoplanarVolumetricCellEvidenceError> {
        self.validate()?;
        let replay = certify_coplanar_volumetric_cell_evidence(left, right)
            .map_err(|_| CoplanarVolumetricCellEvidenceError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(CoplanarVolumetricCellEvidenceError::SourceReplayMismatch)
        }
    }
}

/// Certify retained graph evidence for coplanar volumetric-cell extraction.
pub fn certify_coplanar_volumetric_cell_evidence(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<CoplanarVolumetricCellEvidenceReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    graph
        .validate_against_sources(left, right)
        .map_err(volumetric_cell_graph_mesh_error)?;
    let report = CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right);
    report.validate().map_err(volumetric_cell_mesh_error)?;
    Ok(report)
}

fn derive_obstacle(
    report: &CoplanarVolumetricCellEvidenceReport,
) -> CoplanarVolumetricCellObstacle {
    if !report.left_closed_manifold || !report.right_closed_manifold {
        return CoplanarVolumetricCellObstacle::NonClosedOperand;
    }
    if report.retained_face_pair_count == 0 {
        return CoplanarVolumetricCellObstacle::NoRetainedOverlap;
    }
    if report.unknown_pairs > 0 || report.unknown_events > 0 {
        return CoplanarVolumetricCellObstacle::UnknownGraphEvidence;
    }
    if report.construction_failed_events > 0 {
        return CoplanarVolumetricCellObstacle::ConstructionFailureEvidence;
    }
    if report.coplanar_face_pairs() == 0 {
        return CoplanarVolumetricCellObstacle::NonCoplanarOnly;
    }
    if report.proper_crossing_events > 0 {
        return CoplanarVolumetricCellObstacle::MixedCoplanarAndCrossingCells;
    }
    CoplanarVolumetricCellObstacle::BoundaryOnlyContact
}

fn proper_crossing_event(event: &IntersectionEvent) -> bool {
    matches!(
        event,
        IntersectionEvent::SegmentPlane {
            relation: SegmentPlaneRelation::ProperCrossing,
            ..
        }
    )
}

fn volumetric_cell_mesh_error(error: CoplanarVolumetricCellEvidenceError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("coplanar volumetric-cell evidence failed validation: {error:?}"),
    ))
}

fn volumetric_cell_graph_mesh_error(
    error: super::graph::IntersectionGraphValidationError,
) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("retained volumetric-cell graph failed source replay: {error:?}"),
    ))
}
