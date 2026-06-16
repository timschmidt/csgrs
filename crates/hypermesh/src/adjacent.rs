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
//! allowed to change output topology.
//!
//! The union artifact is also the proof object used by named boolean dispatch
//! for the matching regularized intersection and difference shortcuts:
//! boundary-only full-face/fan-patch contact contributes no intersection
//! volume, and subtracting the adjacent right solid preserves the left solid.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{
    CoplanarProjection, Point3, SegmentIntersection, SegmentPlaneRelation, Sign, TriangleLocation,
    classify_point_triangle, compare_reals, orient3d_report, point_on_segment3, project_point3,
    projected_polygon_area2_value, projected_segment_parameter3,
};

use super::adjacent_polygon::polygon_patch_pairs;
use super::graph::{
    ExactIntersectionGraph, FacePairEvents, IntersectionEvent, MeshSide, build_intersection_graph,
};
use super::intersection::MeshFacePairRelation;
use super::mesh::{ExactMesh, ExactMeshValidationError, Triangle};
use super::validation::ValidationPolicy;
use super::winding::{
    ClosedMeshWindingRelation, classify_mesh_vertices_against_closed_mesh_winding_report,
};
use hyperlimit::SourceProvenance;
use hyperreal::Real;

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

/// One exact shared patch consumed by a merged union.
///
/// A patch can retain a nonconforming but bounded triangulation match, such as
/// one source triangle exactly covered by three opposite-oriented fan
/// triangles on the other solid. The certificate stores source face sets
/// rather than output faces because all patch faces are deleted from the
/// regularized union.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FullFaceAdjacentPatch {
    /// Face indices in the left source mesh that cover the shared patch.
    pub left_faces: Vec<usize>,
    /// Face indices in the right source mesh that cover the shared patch.
    pub right_faces: Vec<usize>,
}

/// Exact materialization of a closed-solid union across shared full faces.
#[derive(Clone, Debug, PartialEq)]
pub struct FullFaceAdjacentUnion {
    /// Source face pairs that were proven exactly coincident and removed.
    pub shared_faces: Vec<FullFaceAdjacentFacePair>,
    /// Source face patches that were proven exactly coincident and removed.
    pub shared_patches: Vec<FullFaceAdjacentPatch>,
    /// Closed output mesh after deleting shared faces and welding seam vertices.
    pub mesh: ExactMesh,
}

/// Opaque retained certificate for full-face adjacency.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct FullFaceAdjacentCertificate {
    inner: FullFaceAdjacencyCertificate,
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
}

impl FullFaceAdjacentUnion {
    /// Validate retained face-pair uniqueness and output mesh state.
    ///
    /// Local validation checks that the retained construction artifact is
    /// internally coherent before boolean code consumes it.
    pub fn validate(&self) -> Result<(), FullFaceAdjacentUnionError> {
        if self.shared_faces.is_empty() && self.shared_patches.is_empty() {
            return Err(FullFaceAdjacentUnionError::MissingSharedFace);
        }
        let mut left_faces = BTreeSet::new();
        let mut right_faces = BTreeSet::new();
        for pair in &self.shared_faces {
            if !left_faces.insert(pair.left_face) || !right_faces.insert(pair.right_face) {
                return Err(FullFaceAdjacentUnionError::DuplicateSharedFace);
            }
        }
        for patch in &self.shared_patches {
            if patch.left_faces.is_empty() || patch.right_faces.is_empty() {
                return Err(FullFaceAdjacentUnionError::MissingSharedFace);
            }
            for &left_face in &patch.left_faces {
                if !left_faces.insert(left_face) {
                    return Err(FullFaceAdjacentUnionError::DuplicateSharedFace);
                }
            }
            for &right_face in &patch.right_faces {
                if !right_faces.insert(right_face) {
                    return Err(FullFaceAdjacentUnionError::DuplicateSharedFace);
                }
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
}

/// Certify and materialize a full-face adjacent closed-solid union.
pub(crate) fn materialize_full_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Option<FullFaceAdjacentUnion> {
    let certificate = full_face_adjacent_certificate(left, right)?;
    materialize_full_face_adjacent_union_from_certificate(left, right, &certificate, validation)
}

/// Return the retained full-face adjacency certificate for these sources.
pub(crate) fn full_face_adjacent_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<FullFaceAdjacentCertificate> {
    full_face_adjacent_union_certificate(left, right)
        .map(|inner| FullFaceAdjacentCertificate { inner })
}

pub(crate) fn materialize_full_face_adjacent_union_from_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
    certificate: &FullFaceAdjacentCertificate,
    validation: ValidationPolicy,
) -> Option<FullFaceAdjacentUnion> {
    let certificate = &certificate.inner;
    let mesh = merged_union_mesh(left, right, certificate, validation)?;
    let union = FullFaceAdjacentUnion {
        shared_faces: certificate.shared_faces.clone(),
        shared_patches: certificate.shared_patches.clone(),
        mesh,
    };
    union.validate().ok()?;
    Some(union)
}

fn full_face_adjacent_union_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<FullFaceAdjacencyCertificate> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return None;
    }
    let graph = build_intersection_graph(left, right).ok()?;
    graph.validate_against_meshes(left, right).ok()?;
    full_face_adjacent_union_certificate_from_graph(left, right, &graph)
}

fn full_face_adjacent_union_certificate_from_graph(
    left: &ExactMesh,
    right: &ExactMesh,
    graph: &ExactIntersectionGraph,
) -> Option<FullFaceAdjacencyCertificate> {
    if graph.has_unknowns() {
        return None;
    }
    if !closed_boundary_contact_only(left, right)? {
        return None;
    }

    let certificate = full_face_adjacency_certificate(left, right)?;
    if certificate.is_empty() {
        return None;
    }
    if !graph.face_pairs.is_empty()
        && !graph_has_only_adjacency_contacts(left, right, &graph.face_pairs, &certificate)
    {
        return None;
    }

    Some(certificate)
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct FullFaceAdjacencyCertificate {
    shared_faces: Vec<FullFaceAdjacentFacePair>,
    shared_patches: Vec<FullFaceAdjacentPatch>,
}

impl FullFaceAdjacencyCertificate {
    fn is_empty(&self) -> bool {
        self.shared_faces.is_empty() && self.shared_patches.is_empty()
    }
}

fn shared_face_pair(certificate: &FullFaceAdjacencyCertificate, pair: &FacePairEvents) -> bool {
    certificate
        .shared_faces
        .iter()
        .any(|shared| shared.left_face == pair.left_face && shared.right_face == pair.right_face)
        || certificate.shared_patches.iter().any(|patch| {
            patch.left_faces.contains(&pair.left_face)
                && patch.right_faces.contains(&pair.right_face)
        })
}

fn consumed_by_certificate(
    certificate: &FullFaceAdjacencyCertificate,
    pair: &FacePairEvents,
) -> bool {
    face_consumed_by_certificate(certificate, MeshSide::Left, pair.left_face)
        && face_consumed_by_certificate(certificate, MeshSide::Right, pair.right_face)
}

fn face_consumed_by_certificate(
    certificate: &FullFaceAdjacencyCertificate,
    side: MeshSide,
    face: usize,
) -> bool {
    match side {
        MeshSide::Left => {
            certificate
                .shared_faces
                .iter()
                .any(|shared| shared.left_face == face)
                || certificate
                    .shared_patches
                    .iter()
                    .any(|patch| patch.left_faces.contains(&face))
        }
        MeshSide::Right => {
            certificate
                .shared_faces
                .iter()
                .any(|shared| shared.right_face == face)
                || certificate
                    .shared_patches
                    .iter()
                    .any(|patch| patch.right_faces.contains(&face))
        }
    }
}

fn one_face_consumed_by_certificate(
    certificate: &FullFaceAdjacencyCertificate,
    pair: &FacePairEvents,
) -> bool {
    let left_consumed = face_consumed_by_certificate(certificate, MeshSide::Left, pair.left_face);
    let right_consumed =
        face_consumed_by_certificate(certificate, MeshSide::Right, pair.right_face);
    left_consumed ^ right_consumed
}

fn consumed_boundary_candidate_event(event: &IntersectionEvent) -> bool {
    match event {
        IntersectionEvent::SegmentPlane { relation, .. } => matches!(
            relation,
            SegmentPlaneRelation::Disjoint
                | SegmentPlaneRelation::Coplanar
                | SegmentPlaneRelation::EndpointOnPlane
                | SegmentPlaneRelation::ProperCrossing
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

fn graph_has_only_adjacency_contacts(
    left: &ExactMesh,
    right: &ExactMesh,
    pairs: &[FacePairEvents],
    certificate: &FullFaceAdjacencyCertificate,
) -> bool {
    pairs
        .iter()
        .all(|pair| adjacency_contact_pair(left, right, pair, certificate))
}

fn adjacency_contact_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    pair: &FacePairEvents,
    certificate: &FullFaceAdjacencyCertificate,
) -> bool {
    if shared_face_pair(certificate, pair) {
        return matches!(
            pair.relation,
            MeshFacePairRelation::CoplanarOverlapping | MeshFacePairRelation::CoplanarTouching
        );
    }
    if consumed_by_certificate(certificate, pair) {
        // A bounded source disk may replay partly as exact whole-face pairs and
        // partly as a polygon patch. Cross-record coplanar edge contacts are
        // model requires that we keep this as certificate replay, not as a
        // loose tolerance merge.
        return matches!(
            pair.relation,
            MeshFacePairRelation::CoplanarOverlapping | MeshFacePairRelation::CoplanarTouching
        );
    }
    if one_face_consumed_by_certificate(certificate, pair)
        && pair.relation == MeshFacePairRelation::Candidate
    {
        // Retained side faces around a nonconvex source-owned disk can cross
        // the deleted cap triangulation at exact boundary points even when no
        // output-volume intersection exists. Exact boundary replay keeps
        // those proper crossings tied to a consumed source face instead of
        // relaxing the general candidate gate.
        return pair.events.iter().all(consumed_boundary_candidate_event);
    }

    match pair.relation {
        MeshFacePairRelation::CoplanarTouching => true,
        MeshFacePairRelation::CoplanarOverlapping => {
            // A side face may share only an exact source edge with the other
            // solid after a full-face weld. Positive-area coplanar overlap is
            // different topology and must wait for the general arrangement
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
    left_in_right.validate().ok()?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left.validate().ok()?;
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

fn full_face_adjacency_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<FullFaceAdjacencyCertificate> {
    let mut certificate = FullFaceAdjacencyCertificate::default();
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
                certificate.shared_faces.push(FullFaceAdjacentFacePair {
                    left_face,
                    right_face,
                });
            }
        }
    }

    for (left_faces, right_faces) in polygon_patch_pairs(left, &left_seen, right, &right_seen)? {
        if left_faces.iter().any(|face| left_seen.contains(face))
            || right_faces.iter().any(|face| right_seen.contains(face))
        {
            return None;
        }
        for &left_face in &left_faces {
            if !left_seen.insert(left_face) {
                return None;
            }
        }
        for &right_face in &right_faces {
            if !right_seen.insert(right_face) {
                return None;
            }
        }
        certificate.shared_patches.push(FullFaceAdjacentPatch {
            left_faces,
            right_faces,
        });
    }

    for left_face in 0..left.triangles().len() {
        if left_seen.contains(&left_face) {
            continue;
        }
        if let Some(right_faces) = fan_faces_cover_triangle(left, left_face, right, &right_seen)? {
            if !left_seen.insert(left_face) {
                return None;
            }
            for &right_face in &right_faces {
                if !right_seen.insert(right_face) {
                    return None;
                }
            }
            certificate.shared_patches.push(FullFaceAdjacentPatch {
                left_faces: vec![left_face],
                right_faces,
            });
        }
    }

    for right_face in 0..right.triangles().len() {
        if right_seen.contains(&right_face) {
            continue;
        }
        if let Some(left_faces) = fan_faces_cover_triangle(right, right_face, left, &left_seen)? {
            if !right_seen.insert(right_face) {
                return None;
            }
            for &left_face in &left_faces {
                if !left_seen.insert(left_face) {
                    return None;
                }
            }
            certificate.shared_patches.push(FullFaceAdjacentPatch {
                left_faces,
                right_faces: vec![right_face],
            });
        }
    }

    Some(certificate)
}

fn merged_union_mesh(
    left: &ExactMesh,
    right: &ExactMesh,
    certificate: &FullFaceAdjacencyCertificate,
    validation: ValidationPolicy,
) -> Option<ExactMesh> {
    let mut right_to_left = BTreeMap::<usize, usize>::new();
    let mut skip_left = BTreeSet::new();
    let mut skip_right = BTreeSet::new();

    for pair in &certificate.shared_faces {
        let left_triangle = left.triangles().get(pair.left_face)?.0;
        let right_triangle = right.triangles().get(pair.right_face)?.0;
        let seam_map = reversed_whole_face_vertex_map(left, left_triangle, right, right_triangle)?;
        insert_seam_map(&mut right_to_left, right_triangle.into_iter().zip(seam_map))?;
        skip_left.insert(pair.left_face);
        skip_right.insert(pair.right_face);
    }

    for patch in &certificate.shared_patches {
        insert_patch_seam_map(left, right, patch, &mut right_to_left)?;
        skip_left.extend(patch.left_faces.iter().copied());
        skip_right.extend(patch.right_faces.iter().copied());
    }

    let mut vertices = Vec::new();
    let mut left_vertex_map = vec![None; left.vertices().len()];
    let mut right_vertex_map = vec![None; right.vertices().len()];
    let mut triangles = Vec::new();
    let left_output_vertices = unskipped_vertices(left, &skip_left);
    let right_output_vertices = unskipped_vertices(right, &skip_right);

    for (face, triangle) in left.triangles().iter().enumerate() {
        if skip_left.contains(&face) {
            continue;
        }
        append_left_triangle_with_edge_splits(
            left,
            right,
            &right_to_left,
            &mut left_vertex_map,
            &mut right_vertex_map,
            &mut vertices,
            &mut triangles,
            triangle.0,
            &right_output_vertices,
        )?;
    }

    for (face, triangle) in right.triangles().iter().enumerate() {
        if skip_right.contains(&face) {
            continue;
        }
        append_right_triangle_with_edge_splits(
            left,
            right,
            &right_to_left,
            &mut left_vertex_map,
            &mut right_vertex_map,
            &mut vertices,
            &mut triangles,
            triangle.0,
            &left_output_vertices,
        )?;
    }

    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact full-face adjacent closed-solid union"),
        validation,
    )
    .ok()
}

fn insert_seam_map<I>(right_to_left: &mut BTreeMap<usize, usize>, pairs: I) -> Option<()>
where
    I: IntoIterator<Item = (usize, usize)>,
{
    for (right_vertex, left_vertex) in pairs {
        match right_to_left.get(&right_vertex) {
            Some(&existing) if existing != left_vertex => return None,
            Some(_) => {}
            None => {
                right_to_left.insert(right_vertex, left_vertex);
            }
        }
    }
    Some(())
}

fn insert_patch_seam_map(
    left: &ExactMesh,
    right: &ExactMesh,
    patch: &FullFaceAdjacentPatch,
    right_to_left: &mut BTreeMap<usize, usize>,
) -> Option<()> {
    let mut left_vertices = BTreeSet::new();
    for &left_face in &patch.left_faces {
        left_vertices.extend(left.triangles().get(left_face)?.0);
    }
    let mut right_vertices = BTreeSet::new();
    for &right_face in &patch.right_faces {
        right_vertices.extend(right.triangles().get(right_face)?.0);
    }

    let mut pairs = Vec::new();
    for right_vertex in right_vertices {
        let right_point = right.vertices().get(right_vertex)?.clone();
        if let Some(left_vertex) = left_vertices.iter().copied().find(|&left_vertex| {
            let left_point = left.vertices()[left_vertex].clone();
            points_equal(&left_point, &right_point) == Some(true)
        }) {
            pairs.push((right_vertex, left_vertex));
        }
    }
    insert_seam_map(right_to_left, pairs)
}

fn map_left_vertex(
    left: &ExactMesh,
    left_vertex_map: &mut [Option<usize>],
    vertices: &mut Vec<Point3>,
    vertex: usize,
) -> Option<usize> {
    if let Some(mapped) = left_vertex_map.get(vertex).copied().flatten() {
        return Some(mapped);
    }
    let mapped = vertices.len();
    vertices.push(left.vertices().get(vertex)?.clone());
    *left_vertex_map.get_mut(vertex)? = Some(mapped);
    Some(mapped)
}

fn map_right_vertex(
    left: &ExactMesh,
    right: &ExactMesh,
    right_to_left: &BTreeMap<usize, usize>,
    left_vertex_map: &mut [Option<usize>],
    right_vertex_map: &mut [Option<usize>],
    vertices: &mut Vec<Point3>,
    vertex: usize,
) -> Option<usize> {
    if let Some(&left_vertex) = right_to_left.get(&vertex) {
        return map_left_vertex(left, left_vertex_map, vertices, left_vertex);
    }
    if let Some(mapped) = right_vertex_map.get(vertex).copied().flatten() {
        return Some(mapped);
    }
    let mapped = vertices.len();
    vertices.push(right.vertices().get(vertex)?.clone());
    *right_vertex_map.get_mut(vertex)? = Some(mapped);
    Some(mapped)
}

fn unskipped_vertices(mesh: &ExactMesh, skipped_faces: &BTreeSet<usize>) -> BTreeSet<usize> {
    let mut vertices = BTreeSet::new();
    for (face, triangle) in mesh.triangles().iter().enumerate() {
        if skipped_faces.contains(&face) {
            continue;
        }
        vertices.extend(triangle.0);
    }
    vertices
}

fn append_left_triangle_with_edge_splits(
    left: &ExactMesh,
    right: &ExactMesh,
    right_to_left: &BTreeMap<usize, usize>,
    left_vertex_map: &mut [Option<usize>],
    right_vertex_map: &mut [Option<usize>],
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
    triangle: [usize; 3],
    right_candidates: &BTreeSet<usize>,
) -> Option<()> {
    let mapped = [
        map_left_vertex(left, left_vertex_map, vertices, triangle[0])?,
        map_left_vertex(left, left_vertex_map, vertices, triangle[1])?,
        map_left_vertex(left, left_vertex_map, vertices, triangle[2])?,
    ];
    let points = triangle_points(left, triangle)?;
    let mut splits = triangle_edge_split_buckets();
    for &right_vertex in right_candidates {
        let point = right.vertices().get(right_vertex)?.clone();
        let Some((edge, parameter)) = triangle_edge_split_parameter(&points, &point)? else {
            continue;
        };
        let mapped = map_right_vertex(
            left,
            right,
            right_to_left,
            left_vertex_map,
            right_vertex_map,
            vertices,
            right_vertex,
        )?;
        insert_triangle_edge_split(&mut splits[edge], mapped, point, parameter);
    }
    append_refined_triangle(&points, mapped, splits, triangles)
}

fn append_right_triangle_with_edge_splits(
    left: &ExactMesh,
    right: &ExactMesh,
    right_to_left: &BTreeMap<usize, usize>,
    left_vertex_map: &mut [Option<usize>],
    right_vertex_map: &mut [Option<usize>],
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
    triangle: [usize; 3],
    left_candidates: &BTreeSet<usize>,
) -> Option<()> {
    let mapped = [
        map_right_vertex(
            left,
            right,
            right_to_left,
            left_vertex_map,
            right_vertex_map,
            vertices,
            triangle[0],
        )?,
        map_right_vertex(
            left,
            right,
            right_to_left,
            left_vertex_map,
            right_vertex_map,
            vertices,
            triangle[1],
        )?,
        map_right_vertex(
            left,
            right,
            right_to_left,
            left_vertex_map,
            right_vertex_map,
            vertices,
            triangle[2],
        )?,
    ];
    let points = triangle_points(right, triangle)?;
    let mut splits = triangle_edge_split_buckets();
    for &left_vertex in left_candidates {
        let point = left.vertices().get(left_vertex)?.clone();
        let Some((edge, parameter)) = triangle_edge_split_parameter(&points, &point)? else {
            continue;
        };
        let mapped = map_left_vertex(left, left_vertex_map, vertices, left_vertex)?;
        insert_triangle_edge_split(&mut splits[edge], mapped, point, parameter);
    }
    append_refined_triangle(&points, mapped, splits, triangles)
}

#[derive(Clone, Debug)]
struct TriangleEdgeSplit {
    parameter: Real,
    mapped_vertex: usize,
    point: Point3,
}

fn triangle_edge_split_buckets() -> [Vec<TriangleEdgeSplit>; 3] {
    [Vec::new(), Vec::new(), Vec::new()]
}

fn triangle_edge_split_parameter(
    triangle_points: &[Point3; 3],
    point: &Point3,
) -> Option<Option<(usize, Real)>> {
    let projection = choose_triangle_projection(triangle_points)?;
    for edge in 0..3 {
        let start = &triangle_points[edge];
        let end = &triangle_points[(edge + 1) % 3];
        if !point_lies_strictly_on_segment3(start, end, point)? {
            continue;
        }
        let parameter = projected_segment_parameter3(&point, start, end, projection)?;
        return Some(Some((edge, parameter)));
    }
    Some(None)
}

fn insert_triangle_edge_split(
    splits: &mut Vec<TriangleEdgeSplit>,
    mapped_vertex: usize,
    point: Point3,
    parameter: Real,
) {
    if splits.iter().any(|split| {
        split.mapped_vertex == mapped_vertex || points_equal(&split.point, &point) == Some(true)
    }) {
        return;
    }
    splits.push(TriangleEdgeSplit {
        parameter,
        mapped_vertex,
        point,
    });
}

fn append_refined_triangle(
    triangle_points: &[Point3; 3],
    mapped_triangle: [usize; 3],
    mut splits: [Vec<TriangleEdgeSplit>; 3],
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    if splits.iter().all(Vec::is_empty) {
        triangles.push(Triangle(mapped_triangle));
        return Some(());
    }
    for edge_splits in &mut splits {
        sort_edge_splits(edge_splits)?;
    }

    let mut point_by_vertex = BTreeMap::<usize, Point3>::new();
    for index in 0..3 {
        point_by_vertex.insert(mapped_triangle[index], triangle_points[index].clone());
    }
    for edge_splits in &splits {
        for split in edge_splits {
            point_by_vertex.insert(split.mapped_vertex, split.point.clone());
        }
    }
    let mut refined = vec![Triangle(mapped_triangle)];
    for edge in 0..3 {
        for split in &splits[edge] {
            split_output_triangle_edge(&point_by_vertex, &mut refined, split.mapped_vertex)?;
        }
    }
    triangles.extend(refined);
    Some(())
}

fn split_output_triangle_edge(
    point_by_vertex: &BTreeMap<usize, Point3>,
    triangles: &mut Vec<Triangle>,
    split_vertex: usize,
) -> Option<()> {
    let split_point = point_by_vertex.get(&split_vertex)?;
    let mut triangle_index = 0;
    while triangle_index < triangles.len() {
        let triangle = triangles[triangle_index].0;
        if triangle.contains(&split_vertex) {
            triangle_index += 1;
            continue;
        }
        for edge in 0..3 {
            let a = triangle[edge];
            let b = triangle[(edge + 1) % 3];
            let opposite = triangle[(edge + 2) % 3];
            let a_point = point_by_vertex.get(&a)?;
            let b_point = point_by_vertex.get(&b)?;
            if point_lies_strictly_on_segment3(a_point, b_point, split_point)? {
                triangles.splice(
                    triangle_index..triangle_index + 1,
                    [
                        Triangle([a, split_vertex, opposite]),
                        Triangle([split_vertex, b, opposite]),
                    ],
                );
                return Some(());
            }
        }
        triangle_index += 1;
    }
    None
}

fn sort_edge_splits(splits: &mut Vec<TriangleEdgeSplit>) -> Option<()> {
    let mut ordered = Vec::<TriangleEdgeSplit>::with_capacity(splits.len());
    for split in splits.drain(..) {
        let mut insert_at = ordered.len();
        for (index, existing) in ordered.iter().enumerate() {
            if compare_reals(&split.parameter, &existing.parameter).value()? == Ordering::Less {
                insert_at = index;
                break;
            }
        }
        ordered.insert(insert_at, split);
    }
    *splits = ordered;
    Some(())
}

fn point_lies_strictly_on_segment3(start: &Point3, end: &Point3, point: &Point3) -> Option<bool> {
    if points_equal(point, start)? || points_equal(point, end)? {
        return Some(false);
    }
    point_on_segment3(start, end, point).value()
}

fn fan_faces_cover_triangle(
    whole_mesh: &ExactMesh,
    whole_face: usize,
    fan_mesh: &ExactMesh,
    consumed_fan_faces: &BTreeSet<usize>,
) -> Option<Option<Vec<usize>>> {
    // This is intentionally a source-triangle disk certificate, not a general
    // planar arrangement. One source triangle may be consumed by an
    // opposite-oriented coplanar triangulated disk whose boundary is a
    // subdivision of the source triangle boundary and whose exact projected
    // area matches. Interior vertices are deleted with the patch; boundary
    // split vertices are retained by refining copied side faces before mesh
    // handoff.
    let whole_triangle = whole_mesh.triangles().get(whole_face)?.0;
    let whole_points = triangle_points(whole_mesh, whole_triangle)?;
    let projection = choose_triangle_projection(&whole_points)?;
    let whole_area = projected_polygon_area2_value(&whole_points, projection);
    let whole_sign = real_sign(&whole_area)?;

    let mut fan_faces = Vec::new();
    let mut edge_counts = BTreeMap::<(usize, usize), usize>::new();
    let mut area_sum = Real::from(0);

    for (fan_face, fan_triangle) in fan_mesh.triangles().iter().enumerate() {
        if consumed_fan_faces.contains(&fan_face) {
            continue;
        }
        let Some(area_abs) = fan_triangle_in_whole_triangle(
            &whole_points,
            projection,
            whole_sign,
            fan_mesh,
            fan_triangle.0,
        )?
        else {
            continue;
        };

        for edge in normalized_triangle_edges(fan_triangle.0) {
            let count = edge_counts.entry(edge).or_default();
            *count += 1;
            if *count > 2 {
                return Some(None);
            }
        }
        area_sum += area_abs;
        fan_faces.push(fan_face);
    }

    if fan_faces.is_empty() {
        return Some(None);
    }
    let whole_area_abs = real_abs(&whole_area)?;
    if compare_reals(&area_sum, &whole_area_abs).value() != Some(Ordering::Equal) {
        return Some(None);
    }
    let boundary_edges = edge_counts
        .iter()
        .filter_map(|(&edge, &count)| (count == 1).then_some(edge))
        .collect::<Vec<_>>();
    if boundary_edges.len() < 3 {
        return Some(None);
    }
    let boundary_vertices = order_fan_boundary_cycle(&boundary_edges)?;
    if boundary_vertices.len() != boundary_edges.len() {
        return Some(None);
    }
    for whole_point in &whole_points {
        if !boundary_vertices.iter().any(|&vertex| {
            fan_mesh
                .vertices()
                .get(vertex)
                .is_some_and(|point| points_equal(whole_point, point) == Some(true))
        }) {
            return Some(None);
        }
    }
    for vertex in boundary_vertices {
        let point = fan_mesh.vertices().get(vertex)?;
        let projected = project_point3(point, projection);
        let location = classify_point_triangle(
            &project_point3(&whole_points[0], projection),
            &project_point3(&whole_points[1], projection),
            &project_point3(&whole_points[2], projection),
            &projected,
        )
        .value()?;
        if !matches!(
            location,
            TriangleLocation::OnEdge | TriangleLocation::OnVertex
        ) {
            return Some(None);
        }
    }
    Some(Some(fan_faces))
}

fn order_fan_boundary_cycle(edges: &[(usize, usize)]) -> Option<Vec<usize>> {
    let mut adjacency = BTreeMap::<usize, Vec<usize>>::new();
    for &(a, b) in edges {
        adjacency.entry(a).or_default().push(b);
        adjacency.entry(b).or_default().push(a);
    }
    if adjacency.len() < 3 || adjacency.values().any(|neighbors| neighbors.len() != 2) {
        return None;
    }
    let start = *adjacency.keys().next()?;
    let mut ordered = vec![start];
    let mut previous = usize::MAX;
    let mut current = start;
    loop {
        let neighbors = adjacency.get(&current)?;
        let next = neighbors
            .iter()
            .copied()
            .find(|&neighbor| neighbor != previous)?;
        if next == start {
            break;
        }
        if ordered.contains(&next) {
            return None;
        }
        ordered.push(next);
        previous = current;
        current = next;
        if ordered.len() > adjacency.len() {
            return None;
        }
    }
    (ordered.len() == adjacency.len()).then_some(ordered)
}

fn fan_triangle_in_whole_triangle(
    whole_points: &[Point3; 3],
    projection: CoplanarProjection,
    whole_sign: Sign,
    fan_mesh: &ExactMesh,
    fan_triangle: [usize; 3],
) -> Option<Option<Real>> {
    let fan_points = triangle_points(fan_mesh, fan_triangle)?;
    if !fan_points
        .iter()
        .all(|point| point_on_triangle_plane(whole_points, point) == Some(true))
    {
        return Some(None);
    }

    let fan_area = projected_polygon_area2_value(&fan_points, projection);
    let fan_sign = real_sign(&fan_area)?;
    if fan_sign == Sign::Zero || fan_sign == whole_sign {
        return Some(None);
    }

    for fan_point in &fan_points {
        let projected = project_point3(fan_point, projection);
        let location = classify_point_triangle(
            &project_point3(&whole_points[0], projection),
            &project_point3(&whole_points[1], projection),
            &project_point3(&whole_points[2], projection),
            &projected,
        )
        .value()?;
        if !matches!(
            location,
            TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
        ) {
            return Some(None);
        }
    }
    let area_abs = real_abs(&fan_area)?;
    if compare_reals(&area_abs, &Real::from(0)).value() != Some(Ordering::Greater) {
        return Some(None);
    }
    Some(Some(area_abs))
}

fn normalized_triangle_edges(triangle: [usize; 3]) -> [(usize, usize); 3] {
    [
        normalized_edge(triangle[0], triangle[1]),
        normalized_edge(triangle[1], triangle[2]),
        normalized_edge(triangle[2], triangle[0]),
    ]
}

const fn normalized_edge(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

fn point_on_triangle_plane(triangle: &[Point3; 3], point: &Point3) -> Option<bool> {
    Some(orient3d_report(&triangle[0], &triangle[1], &triangle[2], point).value()? == Sign::Zero)
}

fn choose_triangle_projection(points: &[Point3; 3]) -> Option<CoplanarProjection> {
    choose_polygon_projection(points)
}

fn choose_polygon_projection(points: &[Point3]) -> Option<CoplanarProjection> {
    [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ]
    .into_iter()
    .find(|&projection| {
        let area = projected_polygon_area2_value(points, projection);
        !matches!(real_sign(&area), Some(Sign::Zero) | None)
    })
}

fn real_abs(value: &Real) -> Option<Real> {
    match real_sign(value)? {
        Sign::Negative => Some(-value.clone()),
        Sign::Zero | Sign::Positive => Some(value.clone()),
    }
}

fn real_sign(value: &Real) -> Option<Sign> {
    match compare_reals(value, &Real::from(0)).value()? {
        Ordering::Less => Some(Sign::Negative),
        Ordering::Equal => Some(Sign::Zero),
        Ordering::Greater => Some(Sign::Positive),
    }
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
        mesh.vertices().get(triangle[0])?.clone(),
        mesh.vertices().get(triangle[1])?.clone(),
        mesh.vertices().get(triangle[2])?.clone(),
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
