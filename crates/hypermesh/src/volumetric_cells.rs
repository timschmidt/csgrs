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
use std::collections::{BTreeSet, HashMap};

use hyperlimit::{
    CoplanarProjection, PlaneSide, Point3, SegmentIntersection, SegmentPlaneRelation,
    TriangleLocation, classify_point_triangle, compare_reals, project_point3,
};

use super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::graph::{
    ExactIntersectionGraph, IntersectionEvent, MeshSide, build_validated_intersection_graph,
};
use super::intersection::MeshFacePairRelation;
use super::mesh::ExactMesh;
use super::solid::{ClosedMeshOrientation, exact_mesh_orientation};
use super::topology::{mesh_for_side, sorted_edge};
use hyperreal::Real;

/// Most specific retained obstacle for volumetric coplanar source-face cells.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CoplanarVolumetricCellObstacle {
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
    pub(crate) const fn requires_coplanar_volumetric_cells(self) -> bool {
        matches!(
            self,
            Self::NeedsCoplanarVolumetricCells | Self::MixedCoplanarAndCrossingCells
        )
    }
}

/// Validation failure for a coplanar volumetric-cell evidence report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(clippy::enum_variant_names)]
pub(crate) enum CoplanarVolumetricCellEvidenceError {
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

/// Replayable summary of retained volumetric coplanar-cell evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct CoplanarVolumetricCellEvidenceReport {
    /// Whether the left operand is a closed two-manifold.
    pub(crate) left_closed_manifold: bool,
    /// Whether the right operand is a closed two-manifold.
    pub(crate) right_closed_manifold: bool,
    /// Retained constructive face pairs in the exact intersection graph.
    pub(crate) retained_face_pair_count: usize,
    /// Retained candidate face pairs requiring graph events.
    pub(crate) candidate_pairs: usize,
    /// Candidate pairs that retain at least one proper segment/plane crossing.
    pub(crate) proper_crossing_candidate_pairs: usize,
    /// Retained coplanar touching face pairs.
    pub(crate) coplanar_touching_pairs: usize,
    /// Retained positive-area coplanar overlap face pairs.
    pub(crate) coplanar_overlapping_pairs: usize,
    /// Coplanar overlap pairs whose retained edge/vertex facts certify a
    /// positive-area face overlap rather than only a positive-length edge
    /// interval.
    pub(crate) positive_area_coplanar_overlapping_pairs: usize,
    /// Positive-area coplanar face overlaps whose adjacent solids lie on
    /// opposite sides of the shared plane.
    ///
    /// These are boundary-only adjacencies, such as two closed solids sharing a
    /// full face. The side test replays exact off-plane vertices from each
    /// closed operand against the retained shared face plane. That distinction
    /// coplanar face-pair blocker should not be inferred from a sampled point
    /// near the shared face.
    pub(crate) opposite_side_coplanar_overlapping_pairs: usize,
    /// Positive-area coplanar face overlaps whose adjacent solids lie on the
    /// same side of the shared plane and therefore still require coplanar
    /// volumetric-cell ownership.
    pub(crate) same_side_coplanar_overlapping_pairs: usize,
    /// Positive-area coplanar face overlaps whose side ownership could not be
    /// certified from exact retained plane and orientation evidence.
    pub(crate) undecided_side_coplanar_overlapping_pairs: usize,
    /// Retained unknown face pairs.
    pub(crate) unknown_pairs: usize,
    /// Retained segment/plane events.
    pub(crate) segment_plane_events: usize,
    /// Segment/plane events certified as proper crossings.
    pub(crate) proper_crossing_events: usize,
    /// Segment/plane events that are exact boundary contacts or disjoint.
    pub(crate) boundary_segment_events: usize,
    /// Segment/plane events whose construction failed after predicate
    /// classification.
    pub(crate) construction_failed_events: usize,
    /// Segment/plane events whose endpoint-side relation stayed unknown.
    pub(crate) unknown_segment_plane_events: usize,
    /// Retained unknown graph events.
    pub(crate) unknown_events: usize,
    /// Retained non-disjoint coplanar edge events.
    pub(crate) coplanar_edge_events: usize,
    /// Retained constructive coplanar vertex/triangle events.
    pub(crate) coplanar_vertex_events: usize,
    /// Most specific obstacle exposed by the retained evidence.
    pub(crate) obstacle: CoplanarVolumetricCellObstacle,
}

impl CoplanarVolumetricCellEvidenceReport {
    /// Build a report from a validated exact intersection graph.
    ///
    /// The graph is only counted here; source replay remains the job of
    /// [`Self::validate_against_sources`]. Counting exact retained events is
    /// still meaningful because it prevents a caller from reducing a
    /// coplanar-volumetric blocker to a boolean flag after the original graph
    /// has crossed an API boundary.
    pub(crate) fn from_graph(
        graph: &ExactIntersectionGraph,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Self {
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
            unknown_segment_plane_events: 0,
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
                        match classify_coplanar_overlap_sides(
                            left,
                            right,
                            pair.left_face,
                            pair.right_face,
                        ) {
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
                MeshFacePairRelation::PlaneSeparated => {}
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
                            SegmentPlaneRelation::Unknown => {
                                report.unknown_segment_plane_events += 1;
                                report.unknown_events += 1;
                            }
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
    pub(crate) const fn coplanar_face_pairs(&self) -> usize {
        self.coplanar_touching_pairs
            .saturating_add(self.coplanar_overlapping_pairs)
    }

    /// Validate the compact report without source meshes.
    ///
    /// This is an integrity check for copied report data. Source replay is
    /// separate because exact object handles must be compared against the
    /// operands that produced them before the report can authorize topology.
    pub(crate) fn validate(&self) -> Result<(), CoplanarVolumetricCellEvidenceError> {
        let Some(coplanar_face_pairs) = self
            .coplanar_touching_pairs
            .checked_add(self.coplanar_overlapping_pairs)
        else {
            return Err(CoplanarVolumetricCellEvidenceError::FacePairCountMismatch);
        };
        let Some(relation_count) = self
            .candidate_pairs
            .checked_add(coplanar_face_pairs)
            .and_then(|count| count.checked_add(self.unknown_pairs))
        else {
            return Err(CoplanarVolumetricCellEvidenceError::FacePairCountMismatch);
        };
        if relation_count != self.retained_face_pair_count {
            return Err(CoplanarVolumetricCellEvidenceError::FacePairCountMismatch);
        }
        if self.proper_crossing_candidate_pairs > self.candidate_pairs {
            return Err(CoplanarVolumetricCellEvidenceError::CandidatePairCountMismatch);
        }
        if self.proper_crossing_candidate_pairs > self.proper_crossing_events {
            return Err(CoplanarVolumetricCellEvidenceError::CandidatePairCountMismatch);
        }
        let Some(segment_plane_partition) = self
            .proper_crossing_events
            .checked_add(self.boundary_segment_events)
            .and_then(|count| count.checked_add(self.construction_failed_events))
            .and_then(|count| count.checked_add(self.unknown_segment_plane_events))
        else {
            return Err(CoplanarVolumetricCellEvidenceError::SegmentPlaneEventCountMismatch);
        };
        if segment_plane_partition != self.segment_plane_events
            || self.unknown_segment_plane_events > self.unknown_events
        {
            return Err(CoplanarVolumetricCellEvidenceError::SegmentPlaneEventCountMismatch);
        }
        let Some(coplanar_event_count) = self
            .coplanar_edge_events
            .checked_add(self.coplanar_vertex_events)
        else {
            return Err(CoplanarVolumetricCellEvidenceError::CoplanarEvidenceMismatch);
        };
        if (coplanar_face_pairs == 0 && coplanar_event_count > 0)
            || coplanar_face_pairs > coplanar_event_count
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
    pub(crate) fn validate_against_sources(
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
pub(crate) fn certify_coplanar_volumetric_cell_evidence(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<CoplanarVolumetricCellEvidenceReport, ExactMeshError> {
    let graph = build_validated_intersection_graph(left, right)?;
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
    let requires_coplanar_volumetric_cells = report.same_side_coplanar_overlapping_pairs > 0
        || report.undecided_side_coplanar_overlapping_pairs > 0
        || report.opposite_side_coplanar_overlapping_pairs
            < report.positive_area_coplanar_overlapping_pairs;
    if report.proper_crossing_events > 0 && requires_coplanar_volumetric_cells {
        return CoplanarVolumetricCellObstacle::MixedCoplanarAndCrossingCells;
    }
    if requires_coplanar_volumetric_cells {
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
/// operand vertex against the retained plane of the left face. If local
/// adjacent off-plane vertices are not sufficient, the fallback uses the
/// closed-surface orientation and the retained oriented face plane as an exact
/// ownership witness. This is deliberately object-level certificate evidence
/// instead of a tolerance-derived "touching" label.
fn classify_coplanar_overlap_sides(
    left: &ExactMesh,
    right: &ExactMesh,
    left_face: usize,
    right_face: usize,
) -> Option<CoplanarOverlapSideEvidence> {
    let left_plane = &left.facts().faces.get(left_face)?.plane;
    let left_side = mesh_local_off_plane_side(left, left_face, left_plane)
        .or_else(|| mesh_oriented_face_interior_side(left, left_face, left_plane))
        .or_else(|| mesh_off_plane_side(left, left_plane))?;
    let right_side = mesh_local_off_plane_side(right, right_face, left_plane)
        .or_else(|| mesh_oriented_face_interior_side(right, right_face, left_plane))
        .or_else(|| mesh_off_plane_side(right, left_plane))?;
    if left_side == right_side {
        Some(CoplanarOverlapSideEvidence::SameSide)
    } else {
        Some(CoplanarOverlapSideEvidence::OppositeSides)
    }
}

fn mesh_local_off_plane_side(
    mesh: &ExactMesh,
    face: usize,
    plane: &super::facts::FacePlaneFacts,
) -> Option<PlaneSide> {
    if mesh.triangles().get(face).is_none() || !mesh_face_is_coplanar_with_plane(mesh, face, plane)
    {
        return None;
    }
    let edge_to_faces = mesh_edge_to_faces(mesh);
    let mut patch = BTreeSet::new();
    let mut stack = vec![face];
    while let Some(current) = stack.pop() {
        if !patch.insert(current) {
            continue;
        }
        for edge in mesh_face_edges(mesh, current)? {
            for &neighbor in edge_to_faces
                .get(&edge)
                .into_iter()
                .flat_map(|faces| faces.iter())
            {
                if !patch.contains(&neighbor)
                    && mesh_face_is_coplanar_with_plane(mesh, neighbor, plane)
                {
                    stack.push(neighbor);
                }
            }
        }
    }

    let mut side = None;
    for &patch_face in &patch {
        for edge in mesh_face_edges(mesh, patch_face)? {
            for &neighbor in edge_to_faces
                .get(&edge)
                .into_iter()
                .flat_map(|faces| faces.iter())
            {
                if patch.contains(&neighbor) {
                    continue;
                }
                let triangle = mesh.triangles().get(neighbor)?.0;
                for vertex in triangle {
                    if edge.contains(&vertex) {
                        continue;
                    }
                    match retained_plane_side(plane, mesh.vertices().get(vertex)?)? {
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
            }
        }
    }
    side
}

fn mesh_edge_to_faces(mesh: &ExactMesh) -> HashMap<[usize; 2], Vec<usize>> {
    let mut edge_to_faces = HashMap::<[usize; 2], Vec<usize>>::new();
    for face in 0..mesh.triangles().len() {
        if let Some(edges) = mesh_face_edges(mesh, face) {
            for edge in edges {
                edge_to_faces.entry(edge).or_default().push(face);
            }
        }
    }
    edge_to_faces
}

fn mesh_face_edges(mesh: &ExactMesh, face: usize) -> Option<[[usize; 2]; 3]> {
    let triangle = mesh.triangles().get(face)?.0;
    Some([
        sorted_edge([triangle[0], triangle[1]]),
        sorted_edge([triangle[1], triangle[2]]),
        sorted_edge([triangle[2], triangle[0]]),
    ])
}

fn mesh_face_is_coplanar_with_plane(
    mesh: &ExactMesh,
    face: usize,
    plane: &super::facts::FacePlaneFacts,
) -> bool {
    mesh.triangles().get(face).is_some_and(|triangle| {
        triangle.0.iter().all(|&vertex| {
            mesh.vertices()
                .get(vertex)
                .and_then(|point| retained_plane_side(plane, point))
                == Some(PlaneSide::On)
        })
    })
}

fn coplanar_pair_has_positive_area_overlap(events: &[IntersectionEvent]) -> bool {
    let mut identical_edges = 0usize;
    let mut left_vertices_in_right = BTreeSet::new();
    let mut right_vertices_in_left = BTreeSet::new();
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
            IntersectionEvent::CoplanarVertex {
                vertex_side,
                vertex,
                location: TriangleLocation::OnEdge | TriangleLocation::OnVertex,
                ..
            } => match vertex_side {
                MeshSide::Left => {
                    left_vertices_in_right.insert(*vertex);
                }
                MeshSide::Right => {
                    right_vertices_in_left.insert(*vertex);
                }
            },
            IntersectionEvent::CoplanarEdge {
                relation: SegmentIntersection::Identical,
                ..
            } => identical_edges += 1,
            _ => {}
        }
    }
    identical_edges >= 3 || left_vertices_in_right.len() >= 3 || right_vertices_in_left.len() >= 3
}

fn mesh_off_plane_side(
    mesh: &ExactMesh,
    plane: &super::facts::FacePlaneFacts,
) -> Option<PlaneSide> {
    let mut side = None;
    for vertex in mesh.vertices() {
        match retained_plane_side(plane, vertex)? {
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

fn mesh_oriented_face_interior_side(
    mesh: &ExactMesh,
    face: usize,
    reference_plane: &super::facts::FacePlaneFacts,
) -> Option<PlaneSide> {
    if !mesh.facts().mesh.closed_manifold {
        return None;
    }
    let orientation = exact_mesh_orientation(mesh);
    let face_plane = &mesh.facts().faces.get(face)?.plane;
    let dot = &(&reference_plane.normal[0] * &face_plane.normal[0])
        + &(&reference_plane.normal[1] * &face_plane.normal[1])
        + &(&reference_plane.normal[2] * &face_plane.normal[2]);
    let interior_direction_dot = match orientation {
        ClosedMeshOrientation::Positive => Real::from(0) - dot,
        ClosedMeshOrientation::Negative => dot,
        ClosedMeshOrientation::NotClosed | ClosedMeshOrientation::Unknown => return None,
    };
    match compare_reals(&interior_direction_dot, &Real::from(0)).value()? {
        Ordering::Less => Some(PlaneSide::Above),
        Ordering::Equal => None,
        Ordering::Greater => Some(PlaneSide::Below),
    }
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

fn volumetric_cell_mesh_error(error: CoplanarVolumetricCellEvidenceError) -> ExactMeshError {
    ExactMeshError::one(ExactMeshBlocker::new(
        ExactMeshBlockerKind::UnsupportedExactOperation,
        format!("coplanar volumetric-cell evidence failed validation: {error:?}"),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn two_tetrahedra_i64(tetrahedra: &[[[i64; 3]; 4]]) -> ExactMesh {
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for tetrahedron in tetrahedra {
            let start = vertices.len() / 3;
            for point in tetrahedron {
                vertices.extend(point);
            }
            triangles.extend([
                start,
                start + 2,
                start + 1,
                start,
                start + 1,
                start + 3,
                start + 1,
                start + 2,
                start + 3,
                start + 2,
                start,
                start + 3,
            ]);
        }
        ExactMesh::from_i64_triangles(&vertices, &triangles).unwrap()
    }

    fn skew_octahedron_i64() -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                0, 0, 2, //
                2, 0, 0, //
                0, 2, 0, //
                -2, 0, 8, //
                0, -2, 0, //
                0, 0, -2,
            ],
            &[
                0, 2, 1, //
                0, 3, 2, //
                0, 4, 3, //
                0, 1, 4, //
                5, 1, 2, //
                5, 2, 3, //
                5, 3, 4, //
                5, 4, 1,
            ],
        )
        .unwrap()
    }

    fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    fn crossing_with_coplanar_overlap_report() -> CoplanarVolumetricCellEvidenceReport {
        CoplanarVolumetricCellEvidenceReport {
            left_closed_manifold: true,
            right_closed_manifold: true,
            retained_face_pair_count: 2,
            candidate_pairs: 1,
            proper_crossing_candidate_pairs: 1,
            coplanar_touching_pairs: 0,
            coplanar_overlapping_pairs: 1,
            positive_area_coplanar_overlapping_pairs: 1,
            opposite_side_coplanar_overlapping_pairs: 0,
            same_side_coplanar_overlapping_pairs: 0,
            undecided_side_coplanar_overlapping_pairs: 0,
            unknown_pairs: 0,
            segment_plane_events: 1,
            proper_crossing_events: 1,
            boundary_segment_events: 0,
            construction_failed_events: 0,
            unknown_segment_plane_events: 0,
            unknown_events: 0,
            coplanar_edge_events: 3,
            coplanar_vertex_events: 0,
            obstacle: CoplanarVolumetricCellObstacle::NoRetainedOverlap,
        }
    }

    #[test]
    fn opposite_side_coplanar_contact_mixed_with_crossing_is_not_cell_blocker() {
        let mut report = crossing_with_coplanar_overlap_report();
        report.opposite_side_coplanar_overlapping_pairs = 1;
        report.obstacle = derive_obstacle(&report);

        assert_eq!(
            report.obstacle,
            CoplanarVolumetricCellObstacle::BoundaryOnlyContact
        );
        assert!(!report.obstacle.requires_coplanar_volumetric_cells());
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    fn same_side_coplanar_overlap_mixed_with_crossing_still_needs_cells() {
        let mut report = crossing_with_coplanar_overlap_report();
        report.same_side_coplanar_overlapping_pairs = 1;
        report.obstacle = derive_obstacle(&report);

        assert_eq!(
            report.obstacle,
            CoplanarVolumetricCellObstacle::MixedCoplanarAndCrossingCells
        );
        assert!(report.obstacle.requires_coplanar_volumetric_cells());
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    fn unknown_segment_plane_events_are_validated_as_segment_partition() {
        let mut report = crossing_with_coplanar_overlap_report();
        report.proper_crossing_events = 0;
        report.proper_crossing_candidate_pairs = 0;
        report.opposite_side_coplanar_overlapping_pairs = 1;
        report.unknown_segment_plane_events = 1;
        report.unknown_events = 1;
        report.obstacle = derive_obstacle(&report);
        assert_eq!(
            report.obstacle,
            CoplanarVolumetricCellObstacle::UnknownGraphEvidence
        );
        assert_eq!(report.validate(), Ok(()));

        report.unknown_segment_plane_events = 2;
        report.unknown_events = 2;
        report.obstacle = derive_obstacle(&report);
        assert_eq!(
            report.validate(),
            Err(CoplanarVolumetricCellEvidenceError::SegmentPlaneEventCountMismatch)
        );
    }

    #[test]
    fn segment_plane_events_must_be_fully_partitioned() {
        let mut report = crossing_with_coplanar_overlap_report();
        report.opposite_side_coplanar_overlapping_pairs = 1;
        report.segment_plane_events = 2;
        report.obstacle = derive_obstacle(&report);

        assert_eq!(
            report.validate(),
            Err(CoplanarVolumetricCellEvidenceError::SegmentPlaneEventCountMismatch)
        );
    }

    #[test]
    fn proper_crossing_candidate_pairs_require_crossing_events() {
        let mut report = crossing_with_coplanar_overlap_report();
        report.opposite_side_coplanar_overlapping_pairs = 1;
        report.proper_crossing_events = 0;
        report.boundary_segment_events = 1;
        report.obstacle = derive_obstacle(&report);

        assert_eq!(
            report.validate(),
            Err(CoplanarVolumetricCellEvidenceError::CandidatePairCountMismatch)
        );
    }

    #[test]
    fn coplanar_face_pairs_require_retained_coplanar_event_evidence() {
        let mut report = crossing_with_coplanar_overlap_report();
        report.opposite_side_coplanar_overlapping_pairs = 1;
        report.coplanar_edge_events = 0;
        report.obstacle = derive_obstacle(&report);

        assert_eq!(
            report.validate(),
            Err(CoplanarVolumetricCellEvidenceError::CoplanarEvidenceMismatch)
        );
    }

    #[test]
    fn coplanar_volumetric_evidence_rejects_overflowing_count_partitions() {
        let mut report = crossing_with_coplanar_overlap_report();
        report.opposite_side_coplanar_overlapping_pairs = 1;
        report.proper_crossing_events = usize::MAX;
        report.boundary_segment_events = 1;
        report.segment_plane_events = usize::MAX;
        report.obstacle = derive_obstacle(&report);

        assert_eq!(
            report.validate(),
            Err(CoplanarVolumetricCellEvidenceError::SegmentPlaneEventCountMismatch)
        );

        let mut report = crossing_with_coplanar_overlap_report();
        report.retained_face_pair_count = usize::MAX;
        report.candidate_pairs = 0;
        report.coplanar_touching_pairs = usize::MAX;
        report.coplanar_overlapping_pairs = 1;

        assert_eq!(
            report.validate(),
            Err(CoplanarVolumetricCellEvidenceError::FacePairCountMismatch)
        );
    }

    #[test]
    fn boundary_vertex_containment_certifies_positive_area_overlap() {
        let events = vec![
            IntersectionEvent::CoplanarVertex {
                vertex_side: MeshSide::Left,
                vertex: 0,
                triangle_side: MeshSide::Right,
                triangle_face: 0,
                location: TriangleLocation::OnVertex,
            },
            IntersectionEvent::CoplanarVertex {
                vertex_side: MeshSide::Left,
                vertex: 1,
                triangle_side: MeshSide::Right,
                triangle_face: 0,
                location: TriangleLocation::OnEdge,
            },
            IntersectionEvent::CoplanarVertex {
                vertex_side: MeshSide::Left,
                vertex: 2,
                triangle_side: MeshSide::Right,
                triangle_face: 0,
                location: TriangleLocation::OnEdge,
            },
        ];

        assert!(coplanar_pair_has_positive_area_overlap(&events));
    }

    #[test]
    fn source_replay_counts_boundary_vertex_coplanar_overlap_as_area() {
        let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let right = tetrahedron_i64([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);
        let report = certify_coplanar_volumetric_cell_evidence(&left, &right).unwrap();

        assert!(report.positive_area_coplanar_overlapping_pairs > 0);
        assert!(report.same_side_coplanar_overlapping_pairs > 0);
        assert!(report.obstacle.requires_coplanar_volumetric_cells());
        report.validate_against_sources(&left, &right).unwrap();
    }

    #[test]
    fn local_coplanar_patch_side_handles_disconnected_global_sides() {
        let mesh = two_tetrahedra_i64(&[
            [[0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, 2]],
            [[10, 0, -2], [12, 0, -2], [10, 2, -2], [10, 0, -4]],
        ]);
        let plane = &mesh.facts().faces[0].plane;

        assert_eq!(mesh_off_plane_side(&mesh, plane), None);
        assert_eq!(
            mesh_local_off_plane_side(&mesh, 0, plane),
            Some(PlaneSide::Above)
        );
    }

    #[test]
    fn oriented_face_side_certifies_when_adjacent_vertices_are_mixed() {
        let mesh = skew_octahedron_i64();
        let plane = &mesh.facts().faces[0].plane;

        assert_eq!(mesh_local_off_plane_side(&mesh, 0, plane), None);
        assert!(matches!(
            mesh_oriented_face_interior_side(&mesh, 0, plane),
            Some(PlaneSide::Above | PlaneSide::Below)
        ));
    }
}
