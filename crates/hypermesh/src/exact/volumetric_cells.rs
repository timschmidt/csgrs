//! Exact evidence for coplanar source-face cells in volumetric booleans.
//!
//! This module reports the retained graph state that separates ordinary
//! non-coplanar winding materialization from the remaining general coplanar
//! volumetric-cell extractor. It does not materialize arbitrary cells. Instead
//! it records whether a closed-solid overlap contains coplanar source-face
//! evidence, whether that evidence is only a boundary-contact candidate, and
//! whether it is mixed with non-coplanar crossing events that require a later
//! certified cell materializer.
//! Exact predicates and construction events are retained as auditable objects,
//! and a missing topological algorithm is represented as explicit exact state
//! rather than as a tolerance fallback. The cell-obstacle vocabulary is also
//! classification depends on correctly retaining face/edge intersection
//! structure before traversal.

use std::cmp::Ordering;

use hyperlimit::{
    CoplanarProjection, PlaneSide, Point3, SegmentIntersection, TriangleLocation,
    classify_point_triangle, compare_reals, project_point3,
};

use super::construction::SegmentPlaneRelation;
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{ExactIntersectionGraph, IntersectionEvent, MeshSide, build_intersection_graph};
use super::intersection::MeshFacePairRelation;
use super::mesh::ExactMesh;
use hyperreal::Real;

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
    /// Coplanar face-side counters contradict retained overlapping pairs.
    CoplanarSideEvidenceMismatch,
    /// The retained obstacle does not match the report counters.
    ObstacleMismatch,
    /// Recomputing the report from source meshes did not reproduce it.
    SourceReplayMismatch,
}

/// Freshness status for retained coplanar volumetric-cell evidence.
///
/// The variants separate local report drift from source-replay drift. That
/// predicate/construction state as auditable objects, then require those
/// objects to replay from their source operands before topology consumes them.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarVolumetricCellEvidenceFreshness {
    /// The report validates locally and replays exactly from the source meshes.
    Current,
    /// Face-pair relation counters no longer sum to retained graph evidence.
    StaleFacePairCounts,
    /// Candidate-pair counters no longer match retained candidate evidence.
    StaleCandidatePairCounts,
    /// Segment/plane event counters no longer match retained event evidence.
    StaleSegmentPlaneEventCounts,
    /// Coplanar face-pair and coplanar event counters disagree.
    StaleCoplanarEvidence,
    /// The named obstacle no longer matches retained exact evidence.
    StaleObstacle,
    /// The report is locally valid but no longer replays from the sources.
    SourceReplayMismatch,
}

impl From<CoplanarVolumetricCellEvidenceError> for CoplanarVolumetricCellEvidenceFreshness {
    fn from(error: CoplanarVolumetricCellEvidenceError) -> Self {
        match error {
            CoplanarVolumetricCellEvidenceError::FacePairCountMismatch => Self::StaleFacePairCounts,
            CoplanarVolumetricCellEvidenceError::CandidatePairCountMismatch => {
                Self::StaleCandidatePairCounts
            }
            CoplanarVolumetricCellEvidenceError::SegmentPlaneEventCountMismatch => {
                Self::StaleSegmentPlaneEventCounts
            }
            CoplanarVolumetricCellEvidenceError::CoplanarEvidenceMismatch => {
                Self::StaleCoplanarEvidence
            }
            CoplanarVolumetricCellEvidenceError::CoplanarSideEvidenceMismatch => {
                Self::StaleCoplanarEvidence
            }
            CoplanarVolumetricCellEvidenceError::ObstacleMismatch => Self::StaleObstacle,
            CoplanarVolumetricCellEvidenceError::SourceReplayMismatch => Self::SourceReplayMismatch,
        }
    }
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
    /// Coplanar overlap pairs whose retained edge/vertex facts certify a
    /// positive-area face overlap rather than only a positive-length edge
    /// interval.
    pub positive_area_coplanar_overlapping_pairs: usize,
    /// Positive-area coplanar face overlaps whose adjacent solids lie on
    /// opposite sides of the shared plane.
    ///
    /// These are boundary-only adjacencies, such as two closed solids sharing a
    /// full face. The side test replays exact off-plane vertices from each
    /// closed operand against the retained shared face plane. That distinction
    /// coplanar face-pair blocker should not be inferred from a sampled point
    /// near the shared face.
    pub opposite_side_coplanar_overlapping_pairs: usize,
    /// Positive-area coplanar face overlaps whose adjacent solids lie on the
    /// same side of the shared plane and therefore still require coplanar
    /// volumetric-cell ownership.
    pub same_side_coplanar_overlapping_pairs: usize,
    /// Positive-area coplanar face overlaps whose side ownership could not be
    /// certified from exact retained plane and orientation evidence.
    pub undecided_side_coplanar_overlapping_pairs: usize,
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
            positive_area_coplanar_overlapping_pairs: 0,
            opposite_side_coplanar_overlapping_pairs: 0,
            same_side_coplanar_overlapping_pairs: 0,
            undecided_side_coplanar_overlapping_pairs: 0,
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
                    if pair
                        .events
                        .iter()
                        .any(|event| proper_crossing_event(event, left, right))
                    {
                        report.proper_crossing_candidate_pairs += 1;
                    }
                }
                MeshFacePairRelation::CoplanarTouching => report.coplanar_touching_pairs += 1,
                MeshFacePairRelation::CoplanarOverlapping => {
                    report.coplanar_overlapping_pairs += 1;
                    if coplanar_pair_has_positive_area_overlap(&pair.events) {
                        report.positive_area_coplanar_overlapping_pairs += 1;
                        match classify_coplanar_overlap_sides(left, right, pair.left_face) {
                            Some(CoplanarOverlapSideEvidence::OppositeSides) => {
                                report.opposite_side_coplanar_overlapping_pairs += 1;
                            }
                            Some(CoplanarOverlapSideEvidence::SameSide) => {
                                report.same_side_coplanar_overlapping_pairs += 1;
                            }
                            None => report.undecided_side_coplanar_overlapping_pairs += 1,
                        }
                    }
                }
                MeshFacePairRelation::Unknown => report.unknown_pairs += 1,
                MeshFacePairRelation::BoundsDisjoint | MeshFacePairRelation::PlaneSeparated => {}
            }

            for event in &pair.events {
                match event {
                    IntersectionEvent::SegmentPlane { relation, .. } => {
                        report.segment_plane_events += 1;
                        match relation {
                            SegmentPlaneRelation::ProperCrossing => {
                                if proper_crossing_event(event, left, right) {
                                    report.proper_crossing_events += 1;
                                } else {
                                    report.boundary_segment_events += 1;
                                }
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
        let Some(side_count) = self
            .opposite_side_coplanar_overlapping_pairs
            .checked_add(self.same_side_coplanar_overlapping_pairs)
            .and_then(|count| count.checked_add(self.undecided_side_coplanar_overlapping_pairs))
        else {
            return Err(CoplanarVolumetricCellEvidenceError::CoplanarSideEvidenceMismatch);
        };
        if self.positive_area_coplanar_overlapping_pairs > self.coplanar_overlapping_pairs
            || side_count != self.positive_area_coplanar_overlapping_pairs
        {
            return Err(CoplanarVolumetricCellEvidenceError::CoplanarSideEvidenceMismatch);
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

    /// Classify whether this retained report is fresh for the source meshes.
    ///
    /// Local validation runs before source replay so a scheduler can report
    /// whether copied volumetric-cell evidence has mutated internally or has
    /// extraction explicit instead of collapsing it to a tolerance decision.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> CoplanarVolumetricCellEvidenceFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match certify_coplanar_volumetric_cell_evidence(left, right) {
            Ok(replay) if self == &replay => CoplanarVolumetricCellEvidenceFreshness::Current,
            Ok(_) | Err(_) => CoplanarVolumetricCellEvidenceFreshness::SourceReplayMismatch,
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
        .validate_against_meshes(left, right)
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
    if report.same_side_coplanar_overlapping_pairs > 0
        || report.undecided_side_coplanar_overlapping_pairs > 0
        || report.opposite_side_coplanar_overlapping_pairs
            < report.positive_area_coplanar_overlapping_pairs
    {
        return CoplanarVolumetricCellObstacle::NeedsCoplanarVolumetricCells;
    }
    CoplanarVolumetricCellObstacle::BoundaryOnlyContact
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CoplanarOverlapSideEvidence {
    OppositeSides,
    SameSide,
}

/// Classify local solid ownership across one positive-area coplanar face pair.
///
/// This does not decide a boolean output. It only separates full-face boundary
/// adjacency from coplanar volumetric cell ownership by replaying every
/// operand vertex against the retained plane of the left face. If all off-plane
/// vertices of an operand lie on one exact side, that side is used as the local
/// ownership witness. This is deliberately an object-level certificate in
/// same-side/opposite-side evidence instead of a tolerance-derived "touching"
/// label.
fn classify_coplanar_overlap_sides(
    left: &ExactMesh,
    right: &ExactMesh,
    left_face: usize,
) -> Option<CoplanarOverlapSideEvidence> {
    let left_plane = &left.facts().faces.get(left_face)?.plane;
    let left_side = mesh_off_plane_side(left, left_plane)?;
    let right_side = mesh_off_plane_side(right, left_plane)?;
    if left_side == right_side {
        Some(CoplanarOverlapSideEvidence::SameSide)
    } else {
        Some(CoplanarOverlapSideEvidence::OppositeSides)
    }
}

fn coplanar_pair_has_positive_area_overlap(events: &[IntersectionEvent]) -> bool {
    let mut identical_edges = 0usize;
    for event in events {
        match event {
            IntersectionEvent::CoplanarVertex {
                location: TriangleLocation::Inside,
                ..
            }
            | IntersectionEvent::CoplanarEdge {
                relation: SegmentIntersection::Proper,
                ..
            } => return true,
            IntersectionEvent::CoplanarEdge {
                relation: SegmentIntersection::Identical,
                ..
            } => identical_edges += 1,
            _ => {}
        }
    }
    identical_edges >= 3
}

fn mesh_off_plane_side(
    mesh: &ExactMesh,
    plane: &super::facts::FacePlaneFacts,
) -> Option<PlaneSide> {
    let mut side = None;
    for vertex in mesh.vertices() {
        match retained_plane_side(plane, &vertex.clone())? {
            PlaneSide::On => {}
            candidate => {
                if let Some(existing) = side {
                    if existing != candidate {
                        return None;
                    }
                } else {
                    side = Some(candidate);
                }
            }
        }
    }
    side
}

fn retained_plane_side(
    plane: &super::facts::FacePlaneFacts,
    point: &hyperlimit::Point3,
) -> Option<PlaneSide> {
    let x_term = &plane.normal[0] * &point.x;
    let y_term = &plane.normal[1] * &point.y;
    let z_term = &plane.normal[2] * &point.z;
    let value = &(&(&x_term + &y_term) + &z_term) + &plane.offset;
    match compare_reals(&value, &Real::from(0)).value()? {
        Ordering::Less => Some(PlaneSide::Above),
        Ordering::Equal => Some(PlaneSide::On),
        Ordering::Greater => Some(PlaneSide::Below),
    }
}

fn proper_crossing_event(event: &IntersectionEvent, left: &ExactMesh, right: &ExactMesh) -> bool {
    let IntersectionEvent::SegmentPlane {
        relation: SegmentPlaneRelation::ProperCrossing,
        plane_side,
        plane_face,
        point: Some(point),
        ..
    } = event
    else {
        return matches!(
            event,
            IntersectionEvent::SegmentPlane {
                relation: SegmentPlaneRelation::ProperCrossing,
                ..
            }
        );
    };
    let Some(triangle) = triangle_points(mesh_for_side(*plane_side, left, right), *plane_face)
    else {
        return true;
    };
    let Some(projection) = choose_triangle_projection(&triangle) else {
        return true;
    };
    classify_point_triangle(
        &project_point3(&triangle[0], projection),
        &project_point3(&triangle[1], projection),
        &project_point3(&triangle[2], projection),
        &project_point3(point, projection),
    )
    .value()
        == Some(TriangleLocation::Inside)
}

fn mesh_for_side<'a>(side: MeshSide, left: &'a ExactMesh, right: &'a ExactMesh) -> &'a ExactMesh {
    match side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    }
}

fn triangle_points(mesh: &ExactMesh, face: usize) -> Option<[Point3; 3]> {
    let triangle = mesh.triangles().get(face)?.0;
    Some([
        mesh.vertices().get(triangle[0])?.clone(),
        mesh.vertices().get(triangle[1])?.clone(),
        mesh.vertices().get(triangle[2])?.clone(),
    ])
}

fn choose_triangle_projection(points: &[Point3; 3]) -> Option<CoplanarProjection> {
    [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ]
    .into_iter()
    .find(|&projection| {
        let area = projected_area2_signed(points, projection);
        compare_reals(&area, &Real::from(0)).value() != Some(Ordering::Equal)
    })
}

fn projected_area2_signed(points: &[Point3; 3], projection: CoplanarProjection) -> Real {
    let mut sum = Real::from(0);
    for index in 0..3 {
        let current = project_point3(&points[index], projection);
        let next = project_point3(&points[(index + 1) % 3], projection);
        sum += &((current.x * &next.y) - &(current.y * &next.x));
    }
    sum
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
        format!("retained volumetric-cell graph failed source-mesh validation: {error:?}"),
    ))
}
