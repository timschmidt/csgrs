//! Exact full-face adjacency materialization for closed solids.
//!
//! Boundary-only contact is usually a policy surface because triangle meshes do
//! not carry lower-dimensional set output. A stricter case can be promoted
//! safely: two closed solids whose retained contact contains one or more whole
//! coincident triangular faces with opposite orientation and no strict
//! interior overlap. The regularized union is obtained by deleting those
//! internal faces and welding only their exact shared vertices.
//!
//! This module keeps that promotion as a source-replayed certificate rather
//! than a mesh rewrite heuristic. The object/predicate separation follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): exact boundary evidence is retained and replayed before it is
//! allowed to change output topology.

use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{Point3, SegmentIntersection, TriangleLocation, compare_reals};

use super::construction::SegmentPlaneRelation;
use super::graph::{FacePairEvents, IntersectionEvent, build_intersection_graph};
use super::intersection::MeshFacePairRelation;
use super::mesh::{ExactMesh, ExactMeshValidationError, Triangle};
use super::provenance::SourceProvenance;
use super::validation::ValidationPolicy;
use super::winding::{
    ClosedMeshWindingRelation, classify_mesh_vertices_against_closed_mesh_winding_report,
};

/// One exact whole-face adjacency consumed by a merged union.
///
/// The face pair is stored by source face index, not by output face index,
/// because the shared faces are removed from the materialized union. Keeping
/// source indices makes the deleted topology replayable against the original
/// meshes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FullFaceAdjacentFacePair {
    /// Face index in the left source mesh.
    pub left_face: usize,
    /// Face index in the right source mesh.
    pub right_face: usize,
}

/// Exact materialization of a closed-solid union across shared full faces.
#[derive(Clone, Debug, PartialEq)]
pub struct FullFaceAdjacentUnion {
    /// Source face pairs that were proven exactly coincident and removed.
    pub shared_faces: Vec<FullFaceAdjacentFacePair>,
    /// Closed output mesh after deleting shared faces and welding seam vertices.
    pub mesh: ExactMesh,
}

/// Validation failure for a retained full-face adjacency materialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FullFaceAdjacentUnionError {
    /// No shared whole-face certificate was retained.
    MissingSharedFace,
    /// A retained source face was paired more than once.
    DuplicateSharedFace,
    /// The retained output mesh no longer validates as a closed exact mesh.
    OutputMesh(ExactMeshValidationError),
    /// The retained output mesh is locally valid but is not a closed manifold.
    OutputNotClosed,
    /// Recomputing the materialization from source meshes did not reproduce it.
    SourceReplayMismatch,
}

impl FullFaceAdjacentUnion {
    /// Validate retained face-pair uniqueness and output mesh state.
    ///
    /// Local validation cannot prove the deleted faces still come from the
    /// original sources; [`Self::validate_against_sources`] performs that
    /// replay. This split mirrors Yap's exact-computation boundary: a copied
    /// construction artifact must be internally coherent before it can be
    /// checked against source objects.
    pub fn validate(&self) -> Result<(), FullFaceAdjacentUnionError> {
        if self.shared_faces.is_empty() {
            return Err(FullFaceAdjacentUnionError::MissingSharedFace);
        }
        let mut left_faces = BTreeSet::new();
        let mut right_faces = BTreeSet::new();
        for pair in &self.shared_faces {
            if !left_faces.insert(pair.left_face) || !right_faces.insert(pair.right_face) {
                return Err(FullFaceAdjacentUnionError::DuplicateSharedFace);
            }
        }
        self.mesh
            .validate_retained_state()
            .map_err(FullFaceAdjacentUnionError::OutputMesh)?;
        if !self.mesh.facts().mesh.closed_manifold {
            return Err(FullFaceAdjacentUnionError::OutputNotClosed);
        }
        Ok(())
    }

    /// Validate this retained union by replaying it from source meshes.
    ///
    /// The replay rebuilds the exact intersection graph, rechecks boundary-only
    /// winding evidence, rediscovers whole-face opposite-orientation pairs, and
    /// rematerializes the union. Equality to the retained artifact is then the
    /// certificate that the output still belongs to those exact sources.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), FullFaceAdjacentUnionError> {
        self.validate()?;
        let Some(replay) =
            materialize_full_face_adjacent_union(left, right, self.mesh.validation_policy())
        else {
            return Err(FullFaceAdjacentUnionError::SourceReplayMismatch);
        };
        if self == &replay {
            Ok(())
        } else {
            Err(FullFaceAdjacentUnionError::SourceReplayMismatch)
        }
    }
}

/// Return whether the sources can be unioned by exact full-face adjacency.
pub fn has_full_face_adjacent_union(left: &ExactMesh, right: &ExactMesh) -> bool {
    materialize_full_face_adjacent_union(left, right, ValidationPolicy::CLOSED).is_some()
}

/// Materialize a regularized union across exact coincident whole faces.
///
/// Only the vertices belonging to certified shared faces are welded. Other
/// lower-dimensional contacts remain separate mesh vertices, so this shortcut
/// does not silently identify point/edge contact that still belongs to the
/// boundary-policy layer.
pub fn materialize_full_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Option<FullFaceAdjacentUnion> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return None;
    }
    let graph = build_intersection_graph(left, right).ok()?;
    graph.validate_against_sources(left, right).ok()?;
    if graph.has_unknowns() || graph.face_pairs.is_empty() {
        return None;
    }
    if !closed_boundary_contact_only(left, right)? {
        return None;
    }

    let shared_faces = full_face_adjacencies(left, right)?;
    if shared_faces.is_empty() {
        return None;
    }
    if !graph_has_only_adjacency_contacts(left, right, &graph.face_pairs, &shared_faces) {
        return None;
    }

    let mesh = merged_union_mesh(left, right, &shared_faces, validation)?;
    let union = FullFaceAdjacentUnion { shared_faces, mesh };
    union.validate().ok()?;
    Some(union)
}

fn shared_face_pair(shared_faces: &[FullFaceAdjacentFacePair], pair: &FacePairEvents) -> bool {
    shared_faces
        .iter()
        .any(|shared| shared.left_face == pair.left_face && shared.right_face == pair.right_face)
}

fn graph_has_only_adjacency_contacts(
    left: &ExactMesh,
    right: &ExactMesh,
    pairs: &[FacePairEvents],
    shared_faces: &[FullFaceAdjacentFacePair],
) -> bool {
    pairs
        .iter()
        .all(|pair| adjacency_contact_pair(left, right, pair, shared_faces))
}

fn adjacency_contact_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    pair: &FacePairEvents,
    shared_faces: &[FullFaceAdjacentFacePair],
) -> bool {
    if shared_face_pair(shared_faces, pair) {
        return pair.relation == MeshFacePairRelation::CoplanarOverlapping;
    }

    match pair.relation {
        MeshFacePairRelation::CoplanarTouching => true,
        MeshFacePairRelation::CoplanarOverlapping => {
            // A side face may share only an exact source edge with the other
            // solid after a full-face weld. Positive-area coplanar overlap is
            // different topology and must wait for the general arrangement
            // path. Yap, "Towards Exact Geometric Computation," Comput. Geom.
            // 7.1-2 (1997), is the reason this remains an explicit certified
            // distinction instead of a tolerance-based merge.
            !same_whole_face_any_orientation(left, pair.left_face, right, pair.right_face)
                && !coplanar_pair_has_positive_area_evidence(pair)
        }
        MeshFacePairRelation::Candidate => pair.events.iter().all(boundary_candidate_event),
        MeshFacePairRelation::BoundsDisjoint
        | MeshFacePairRelation::PlaneSeparated
        | MeshFacePairRelation::Unknown => false,
    }
}

fn boundary_candidate_event(event: &IntersectionEvent) -> bool {
    match event {
        IntersectionEvent::SegmentPlane { relation, .. } => matches!(
            relation,
            SegmentPlaneRelation::Disjoint
                | SegmentPlaneRelation::Coplanar
                | SegmentPlaneRelation::EndpointOnPlane
        ),
        IntersectionEvent::CoplanarEdge { relation, .. } => {
            *relation != SegmentIntersection::Disjoint
        }
        IntersectionEvent::CoplanarVertex { location, .. } => matches!(
            location,
            TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
        ),
        IntersectionEvent::Unknown => false,
    }
}

fn coplanar_pair_has_positive_area_evidence(pair: &FacePairEvents) -> bool {
    pair.events.iter().any(|event| match event {
        IntersectionEvent::CoplanarEdge { relation, .. } => {
            *relation == SegmentIntersection::Proper
        }
        IntersectionEvent::CoplanarVertex { location, .. } => *location == TriangleLocation::Inside,
        IntersectionEvent::SegmentPlane { .. } | IntersectionEvent::Unknown => false,
    })
}

fn same_whole_face_any_orientation(
    left: &ExactMesh,
    left_face: usize,
    right: &ExactMesh,
    right_face: usize,
) -> bool {
    let Some(left_triangle) = left.triangles().get(left_face).map(|triangle| triangle.0) else {
        return false;
    };
    let Some(right_triangle) = right.triangles().get(right_face).map(|triangle| triangle.0) else {
        return false;
    };
    same_whole_face_vertices(left, left_triangle, right, right_triangle).unwrap_or(false)
}

fn closed_boundary_contact_only(left: &ExactMesh, right: &ExactMesh) -> Option<bool> {
    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right.validate_against_sources(left, right).ok()?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left.validate_against_sources(right, left).ok()?;
    Some(
        mesh_vertices_are_boundary_or_outside(&left_in_right)
            && mesh_vertices_are_boundary_or_outside(&right_in_left)
            && (mesh_vertices_touch_boundary(&left_in_right)
                || mesh_vertices_touch_boundary(&right_in_left)),
    )
}

fn mesh_vertices_are_boundary_or_outside(
    report: &super::winding::ClosedMeshWindingMeshReport,
) -> bool {
    report.target_closed
        && report.vertices.iter().all(|vertex| {
            matches!(
                vertex.relation,
                ClosedMeshWindingRelation::Outside | ClosedMeshWindingRelation::Boundary
            )
        })
}

fn mesh_vertices_touch_boundary(report: &super::winding::ClosedMeshWindingMeshReport) -> bool {
    report
        .vertices
        .iter()
        .any(|vertex| vertex.relation == ClosedMeshWindingRelation::Boundary)
}

fn full_face_adjacencies(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<Vec<FullFaceAdjacentFacePair>> {
    let mut pairs = Vec::new();
    let mut left_seen = BTreeSet::new();
    let mut right_seen = BTreeSet::new();
    for (left_face, left_triangle) in left.triangles().iter().enumerate() {
        for (right_face, right_triangle) in right.triangles().iter().enumerate() {
            if reversed_whole_face_vertex_map(left, left_triangle.0, right, right_triangle.0)
                .is_some()
            {
                if !left_seen.insert(left_face) || !right_seen.insert(right_face) {
                    return None;
                }
                pairs.push(FullFaceAdjacentFacePair {
                    left_face,
                    right_face,
                });
            }
        }
    }
    Some(pairs)
}

fn merged_union_mesh(
    left: &ExactMesh,
    right: &ExactMesh,
    shared_faces: &[FullFaceAdjacentFacePair],
    validation: ValidationPolicy,
) -> Option<ExactMesh> {
    let mut right_to_left = BTreeMap::<usize, usize>::new();
    let mut skip_left = BTreeSet::new();
    let mut skip_right = BTreeSet::new();
    for pair in shared_faces {
        let left_triangle = left.triangles().get(pair.left_face)?.0;
        let right_triangle = right.triangles().get(pair.right_face)?.0;
        let seam_map = reversed_whole_face_vertex_map(left, left_triangle, right, right_triangle)?;
        for (right_vertex, left_vertex) in right_triangle.into_iter().zip(seam_map) {
            match right_to_left.get(&right_vertex) {
                Some(&existing) if existing != left_vertex => return None,
                Some(_) => {}
                None => {
                    right_to_left.insert(right_vertex, left_vertex);
                }
            }
        }
        skip_left.insert(pair.left_face);
        skip_right.insert(pair.right_face);
    }

    let mut vertices = left.vertices().to_vec();
    let mut right_vertex_map = Vec::with_capacity(right.vertices().len());
    for (right_vertex, vertex) in right.vertices().iter().enumerate() {
        if let Some(&left_vertex) = right_to_left.get(&right_vertex) {
            right_vertex_map.push(left_vertex);
        } else {
            right_vertex_map.push(vertices.len());
            vertices.push(vertex.clone());
        }
    }

    let mut triangles = Vec::new();
    triangles.extend(
        left.triangles()
            .iter()
            .enumerate()
            .filter(|(face, _)| !skip_left.contains(face))
            .map(|(_, triangle)| *triangle),
    );
    triangles.extend(
        right
            .triangles()
            .iter()
            .enumerate()
            .filter(|(face, _)| !skip_right.contains(face))
            .map(|(_, triangle)| {
                Triangle([
                    right_vertex_map[triangle.0[0]],
                    right_vertex_map[triangle.0[1]],
                    right_vertex_map[triangle.0[2]],
                ])
            }),
    );

    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact full-face adjacent closed-solid union"),
        validation,
    )
    .ok()
}

fn reversed_whole_face_vertex_map(
    left: &ExactMesh,
    left_triangle: [usize; 3],
    right: &ExactMesh,
    right_triangle: [usize; 3],
) -> Option<[usize; 3]> {
    let left_points = triangle_points(left, left_triangle)?;
    let right_points = triangle_points(right, right_triangle)?;
    let mut labels = [usize::MAX; 3];
    for (right_corner, right_point) in right_points.iter().enumerate() {
        let label = left_points
            .iter()
            .position(|left_point| points_equal(left_point, right_point) == Some(true))?;
        labels[right_corner] = label;
    }
    if !is_reversed_cycle(labels) {
        return None;
    }
    Some([
        left_triangle[labels[0]],
        left_triangle[labels[1]],
        left_triangle[labels[2]],
    ])
}

fn same_whole_face_vertices(
    left: &ExactMesh,
    left_triangle: [usize; 3],
    right: &ExactMesh,
    right_triangle: [usize; 3],
) -> Option<bool> {
    let left_points = triangle_points(left, left_triangle)?;
    let right_points = triangle_points(right, right_triangle)?;
    for right_point in &right_points {
        if !left_points
            .iter()
            .any(|left_point| points_equal(left_point, right_point) == Some(true))
        {
            return Some(false);
        }
    }
    Some(true)
}

fn triangle_points(mesh: &ExactMesh, triangle: [usize; 3]) -> Option<[Point3; 3]> {
    Some([
        mesh.vertices().get(triangle[0])?.to_hyperlimit_point(),
        mesh.vertices().get(triangle[1])?.to_hyperlimit_point(),
        mesh.vertices().get(triangle[2])?.to_hyperlimit_point(),
    ])
}

const fn is_reversed_cycle(labels: [usize; 3]) -> bool {
    matches!(labels, [0, 2, 1] | [2, 1, 0] | [1, 0, 2])
}

fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    Some(
        compare_reals(&left.x, &right.x).value()? == std::cmp::Ordering::Equal
            && compare_reals(&left.y, &right.y).value()? == std::cmp::Ordering::Equal
            && compare_reals(&left.z, &right.z).value()? == std::cmp::Ordering::Equal,
    )
}
