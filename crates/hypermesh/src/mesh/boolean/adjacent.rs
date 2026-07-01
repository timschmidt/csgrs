//! Exact full-face adjacency materialization for closed solids.
//!
//! Boundary-only contact is usually a policy surface because triangle meshes do
//! not carry lower-dimensional set output. A stricter case can be promoted
//! safely: two closed solids whose retained contact contains one or more whole
//! coincident triangular faces with opposite orientation and no strict
//! interior overlap. The regularized union is obtained by deleting those
//! internal faces and welding only their exact shared vertices.
//!
//! This module keeps that promotion as a source-replayed certificate before it
//! is allowed to change output topology.
//!
//! The union artifact is also the proof object used by named boolean dispatch
//! for the matching regularized intersection and difference shortcuts:
//! boundary-only full-face/fan-patch contact contributes no intersection
//! volume, and subtracting the adjacent right solid preserves the left solid.

mod polygon;

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use super::super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::super::graph::intersection::MeshFacePairRelation;
use super::super::graph::{ExactIntersectionGraph, FacePairEvents, IntersectionEvent};
use super::super::validation::ExactMeshValidationPolicy;
use super::super::{ExactMesh, ExactMeshValidationError, Triangle, sorted_edge};
use super::{
    choose_nonzero_projected_polygon_area, closed_boundary_contact_only, point3_exact_equal,
    point3_lies_strictly_on_segment, split_output_triangle_edge,
};
use hyperlimit::SourceProvenance;
use hyperlimit::{
    CoplanarProjection, Point3, SegmentIntersection, SegmentPlaneRelation, Sign, TriangleLocation,
    classify_point_triangle, compare_reals, orient3d_report, project_point3,
    projected_polygon_area2_value, projected_segment_parameter3,
};
use hyperreal::Real;
use polygon::polygon_patch_pairs;

/// One exact whole-face adjacency consumed by a merged union.
///
/// The face pair is stored by source face index, not by output face index,
/// because the shared faces are removed from the materialized union. Keeping
/// source indices makes the deleted topology replayable against the original
/// meshes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct FullFaceAdjacentFacePair {
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
pub(crate) struct FullFaceAdjacentPatch {
    /// Face indices in the left source mesh that cover the shared patch.
    pub left_faces: Vec<usize>,
    /// Face indices in the right source mesh that cover the shared patch.
    pub right_faces: Vec<usize>,
}

/// Exact materialization of a closed-solid union across shared full faces.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FullFaceAdjacentUnion {
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
pub(crate) enum FullFaceAdjacentUnionError {
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
            .validate_retained_state_detail()
            .map_err(FullFaceAdjacentUnionError::OutputMesh)?;
        if !self.mesh.facts().mesh.closed_manifold {
            return Err(FullFaceAdjacentUnionError::OutputNotClosed);
        }
        Ok(())
    }
}

fn undecidable_shared_face_equality(left_face: usize, right_face: usize) -> ExactMeshError {
    ExactMeshError::one(
        ExactMeshBlocker::new(
            ExactMeshBlockerKind::UndecidablePredicate,
            format!(
                "full-face adjacent certificate could not decide whether left face {left_face} and right face {right_face} share the same exact vertices"
            ),
        )
        .with_face(left_face),
    )
}

/// Return the retained full-face adjacency certificate from a validated graph.
pub(crate) fn full_face_adjacent_certificate_from_graph(
    left: &ExactMesh,
    right: &ExactMesh,
    graph: &ExactIntersectionGraph,
) -> Result<Option<FullFaceAdjacentCertificate>, ExactMeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    if graph.has_unknowns() {
        return Ok(None);
    }
    if !closed_boundary_contact_only(left, right)? {
        return Ok(None);
    }

    let Some(certificate) = full_face_adjacency_certificate(left, right)? else {
        return Ok(None);
    };
    if certificate.shared_faces.is_empty() && certificate.shared_patches.is_empty() {
        return Ok(None);
    }
    for pair in &graph.face_pairs {
        if !adjacency_contact_pair(left, right, pair, &certificate)? {
            return Ok(None);
        }
    }

    Ok(Some(FullFaceAdjacentCertificate { inner: certificate }))
}

pub(crate) fn materialize_full_face_adjacent_union_from_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
    certificate: &FullFaceAdjacentCertificate,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<FullFaceAdjacentUnion>, ExactMeshError> {
    let certificate = &certificate.inner;
    let Some(mesh) = merged_union_mesh(left, right, certificate, validation)? else {
        return Ok(None);
    };
    let union = FullFaceAdjacentUnion {
        shared_faces: certificate.shared_faces.clone(),
        shared_patches: certificate.shared_patches.clone(),
        mesh,
    };
    union.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("full-face adjacent union retained output failed validation: {error:?}"),
        ))
    })?;
    Ok(Some(union))
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct FullFaceAdjacencyCertificate {
    shared_faces: Vec<FullFaceAdjacentFacePair>,
    shared_patches: Vec<FullFaceAdjacentPatch>,
}

fn adjacency_contact_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    pair: &FacePairEvents,
    certificate: &FullFaceAdjacencyCertificate,
) -> Result<bool, ExactMeshError> {
    if certificate
        .shared_faces
        .iter()
        .any(|shared| shared.left_face == pair.left_face && shared.right_face == pair.right_face)
        || certificate.shared_patches.iter().any(|patch| {
            patch.left_faces.contains(&pair.left_face)
                && patch.right_faces.contains(&pair.right_face)
        })
    {
        return Ok(matches!(
            pair.relation,
            MeshFacePairRelation::CoplanarOverlapping | MeshFacePairRelation::CoplanarTouching
        ));
    }
    let left_consumed = certificate
        .shared_faces
        .iter()
        .any(|shared| shared.left_face == pair.left_face)
        || certificate
            .shared_patches
            .iter()
            .any(|patch| patch.left_faces.contains(&pair.left_face));
    let right_consumed = certificate
        .shared_faces
        .iter()
        .any(|shared| shared.right_face == pair.right_face)
        || certificate
            .shared_patches
            .iter()
            .any(|patch| patch.right_faces.contains(&pair.right_face));
    if left_consumed && right_consumed {
        // A bounded source disk may replay partly as exact whole-face pairs and
        // partly as a polygon patch. Cross-record coplanar edge contacts are
        // model requires that we keep this as certificate replay, not as a
        // loose tolerance merge.
        return Ok(matches!(
            pair.relation,
            MeshFacePairRelation::CoplanarOverlapping | MeshFacePairRelation::CoplanarTouching
        ));
    }
    if (left_consumed ^ right_consumed) && pair.relation == MeshFacePairRelation::Candidate {
        // Retained side faces around a nonconvex source-owned disk can cross
        // the deleted cap triangulation at exact boundary points even when no
        // output-volume intersection exists. Exact boundary replay keeps
        // those proper crossings tied to a consumed source face instead of
        // relaxing the general candidate gate.
        return Ok(pair.events.iter().all(|event| match event {
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
        }));
    }

    let contact = match pair.relation {
        MeshFacePairRelation::CoplanarTouching => true,
        MeshFacePairRelation::CoplanarOverlapping => {
            // A side face may share only an exact source edge with the other
            // solid after a full-face weld. Positive-area coplanar overlap is
            // different topology and must wait for the general arrangement
            // distinction instead of a tolerance-based merge.
            !same_whole_face_any_orientation(left, pair.left_face, right, pair.right_face)?
                && !pair.events.iter().any(|event| match event {
                    IntersectionEvent::CoplanarEdge { relation, .. } => {
                        *relation == SegmentIntersection::Proper
                    }
                    IntersectionEvent::CoplanarVertex { location, .. } => {
                        *location == TriangleLocation::Inside
                    }
                    IntersectionEvent::SegmentPlane { .. } | IntersectionEvent::Unknown => false,
                })
        }
        MeshFacePairRelation::Candidate => pair.events.iter().all(|event| match event {
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
        }),
        MeshFacePairRelation::PlaneSeparated | MeshFacePairRelation::Unknown => false,
    };
    Ok(contact)
}

fn same_whole_face_any_orientation(
    left: &ExactMesh,
    left_face: usize,
    right: &ExactMesh,
    right_face: usize,
) -> Result<bool, ExactMeshError> {
    let left_triangle = left.view().face(left_face)?.vertex_indices();
    let right_triangle = right.view().face(right_face)?.vertex_indices();
    let left_points = triangle_point_refs(left, left_triangle)?;
    let right_points = triangle_point_refs(right, right_triangle)?;
    for right_point in &right_points {
        let mut matched = false;
        let mut undecided = false;
        for left_point in &left_points {
            match point3_exact_equal(left_point, right_point) {
                Some(true) => {
                    matched = true;
                    break;
                }
                Some(false) => {}
                None => undecided = true,
            }
        }
        if !matched {
            if undecided {
                return Err(undecidable_shared_face_equality(left_face, right_face));
            }
            return Ok(false);
        }
    }
    Ok(true)
}

fn full_face_adjacency_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<FullFaceAdjacencyCertificate>, ExactMeshError> {
    let mut certificate = FullFaceAdjacencyCertificate::default();
    let mut left_seen = BTreeSet::new();
    let mut right_seen = BTreeSet::new();

    for left_face in left.view().faces() {
        for right_face in right.view().faces() {
            if reversed_whole_face_vertex_map(
                left,
                left_face.vertex_indices(),
                right,
                right_face.vertex_indices(),
            )?
            .is_some()
            {
                if !left_seen.insert(left_face.index()) || !right_seen.insert(right_face.index()) {
                    return Ok(None);
                }
                certificate.shared_faces.push(FullFaceAdjacentFacePair {
                    left_face: left_face.index(),
                    right_face: right_face.index(),
                });
            }
        }
    }

    let Some(polygon_patch_pairs) = polygon_patch_pairs(left, &left_seen, right, &right_seen)
    else {
        return Ok(None);
    };
    for (left_faces, right_faces) in polygon_patch_pairs {
        if left_faces.iter().any(|face| left_seen.contains(face))
            || right_faces.iter().any(|face| right_seen.contains(face))
        {
            return Ok(None);
        }
        for &left_face in &left_faces {
            if !left_seen.insert(left_face) {
                return Ok(None);
            }
        }
        for &right_face in &right_faces {
            if !right_seen.insert(right_face) {
                return Ok(None);
            }
        }
        certificate.shared_patches.push(FullFaceAdjacentPatch {
            left_faces,
            right_faces,
        });
    }

    for left_face in left.view().faces() {
        let left_face = left_face.index();
        if left_seen.contains(&left_face) {
            continue;
        }
        match fan_faces_cover_triangle(left, left_face, right, &right_seen)? {
            Some(Some(right_faces)) => {
                if !left_seen.insert(left_face) {
                    return Ok(None);
                }
                for &right_face in &right_faces {
                    if !right_seen.insert(right_face) {
                        return Ok(None);
                    }
                }
                certificate.shared_patches.push(FullFaceAdjacentPatch {
                    left_faces: vec![left_face],
                    right_faces,
                });
            }
            Some(None) => {}
            None => return Ok(None),
        }
    }

    for right_face in right.view().faces() {
        let right_face = right_face.index();
        if right_seen.contains(&right_face) {
            continue;
        }
        match fan_faces_cover_triangle(right, right_face, left, &left_seen)? {
            Some(Some(left_faces)) => {
                if !right_seen.insert(right_face) {
                    return Ok(None);
                }
                for &left_face in &left_faces {
                    if !left_seen.insert(left_face) {
                        return Ok(None);
                    }
                }
                certificate.shared_patches.push(FullFaceAdjacentPatch {
                    left_faces,
                    right_faces: vec![right_face],
                });
            }
            Some(None) => {}
            None => return Ok(None),
        }
    }

    Ok(Some(certificate))
}

fn merged_union_mesh(
    left: &ExactMesh,
    right: &ExactMesh,
    certificate: &FullFaceAdjacencyCertificate,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    let mut right_to_left = BTreeMap::<usize, usize>::new();
    let mut skip_left = BTreeSet::new();
    let mut skip_right = BTreeSet::new();

    for pair in &certificate.shared_faces {
        let left_triangle = left.view().face(pair.left_face)?.vertex_indices();
        let right_triangle = right.view().face(pair.right_face)?.vertex_indices();
        let Some(seam_map) =
            reversed_whole_face_vertex_map(left, left_triangle, right, right_triangle)?
        else {
            return Ok(None);
        };
        if insert_seam_map(&mut right_to_left, right_triangle.into_iter().zip(seam_map)).is_none() {
            return Ok(None);
        }
        skip_left.insert(pair.left_face);
        skip_right.insert(pair.right_face);
    }

    for patch in &certificate.shared_patches {
        if insert_patch_seam_map(left, right, patch, &mut right_to_left)?.is_none() {
            return Ok(None);
        }
        skip_left.extend(patch.left_faces.iter().copied());
        skip_right.extend(patch.right_faces.iter().copied());
    }

    let mut vertices = Vec::new();
    let mut left_vertex_map = vec![None; left.vertices().len()];
    let mut right_vertex_map = vec![None; right.vertices().len()];
    let mut triangles = Vec::new();
    let left_output_vertices = {
        let mut vertices = BTreeSet::new();
        for face in left.view().faces() {
            if skip_left.contains(&face.index()) {
                continue;
            }
            vertices.extend(face.vertex_indices());
        }
        vertices
    };
    let right_output_vertices = {
        let mut vertices = BTreeSet::new();
        for face in right.view().faces() {
            if skip_right.contains(&face.index()) {
                continue;
            }
            vertices.extend(face.vertex_indices());
        }
        vertices
    };

    for face in left.view().faces() {
        if skip_left.contains(&face.index()) {
            continue;
        }
        if append_left_triangle_with_edge_splits(
            left,
            right,
            &right_to_left,
            &mut left_vertex_map,
            &mut right_vertex_map,
            &mut vertices,
            &mut triangles,
            face.vertex_indices(),
            &right_output_vertices,
        )?
        .is_none()
        {
            return Ok(None);
        }
    }

    for face in right.view().faces() {
        if skip_right.contains(&face.index()) {
            continue;
        }
        if append_right_triangle_with_edge_splits(
            left,
            right,
            &right_to_left,
            &mut left_vertex_map,
            &mut right_vertex_map,
            &mut vertices,
            &mut triangles,
            face.vertex_indices(),
            &left_output_vertices,
        )?
        .is_none()
        {
            return Ok(None);
        }
    }
    let mut seen = BTreeSet::new();
    triangles.retain(|triangle| {
        let mut key = triangle.0;
        key.sort_unstable();
        seen.insert(key)
    });

    let mesh = ExactMesh::new_with_policy_and_version(
        vertices,
        triangles,
        SourceProvenance::exact("exact full-face adjacent closed-solid union"),
        validation,
        1,
    )?;
    Ok(Some(mesh))
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
) -> Result<Option<()>, ExactMeshError> {
    let mut left_vertices = BTreeSet::new();
    for &left_face in &patch.left_faces {
        left_vertices.extend(left.view().face(left_face)?.vertex_indices());
    }
    let mut right_vertices = BTreeSet::new();
    for &right_face in &patch.right_faces {
        right_vertices.extend(right.view().face(right_face)?.vertex_indices());
    }

    let mut pairs = Vec::new();
    for right_vertex in right_vertices {
        let right_point = right.view().vertex(right_vertex)?.point();
        let mut matching_left_vertex = None;
        for left_vertex in left_vertices.iter().copied() {
            let left_point = left.view().vertex(left_vertex)?.point();
            let Some(equal) = point3_exact_equal(left_point, right_point) else {
                return Ok(None);
            };
            if equal {
                matching_left_vertex = Some(left_vertex);
                break;
            }
        }
        if let Some(left_vertex) = matching_left_vertex {
            pairs.push((right_vertex, left_vertex));
        }
    }
    Ok(insert_seam_map(right_to_left, pairs))
}

fn map_left_vertex(
    left: &ExactMesh,
    left_vertex_map: &mut [Option<usize>],
    vertices: &mut Vec<Point3>,
    vertex: usize,
) -> Result<Option<usize>, ExactMeshError> {
    if let Some(mapped) = left_vertex_map.get(vertex).copied().flatten() {
        return Ok(Some(mapped));
    }
    let mapped = vertices.len();
    vertices.push(left.view().vertex(vertex)?.point().clone());
    let Some(slot) = left_vertex_map.get_mut(vertex) else {
        return Ok(None);
    };
    *slot = Some(mapped);
    Ok(Some(mapped))
}

fn map_right_vertex(
    left: &ExactMesh,
    right: &ExactMesh,
    right_to_left: &BTreeMap<usize, usize>,
    left_vertex_map: &mut [Option<usize>],
    right_vertex_map: &mut [Option<usize>],
    vertices: &mut Vec<Point3>,
    vertex: usize,
) -> Result<Option<usize>, ExactMeshError> {
    if let Some(&left_vertex) = right_to_left.get(&vertex) {
        return map_left_vertex(left, left_vertex_map, vertices, left_vertex);
    }
    if let Some(mapped) = right_vertex_map.get(vertex).copied().flatten() {
        return Ok(Some(mapped));
    }
    let mapped = vertices.len();
    vertices.push(right.view().vertex(vertex)?.point().clone());
    let Some(slot) = right_vertex_map.get_mut(vertex) else {
        return Ok(None);
    };
    *slot = Some(mapped);
    Ok(Some(mapped))
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
) -> Result<Option<()>, ExactMeshError> {
    let mapped = [
        match map_left_vertex(left, left_vertex_map, vertices, triangle[0])? {
            Some(mapped) => mapped,
            None => return Ok(None),
        },
        match map_left_vertex(left, left_vertex_map, vertices, triangle[1])? {
            Some(mapped) => mapped,
            None => return Ok(None),
        },
        match map_left_vertex(left, left_vertex_map, vertices, triangle[2])? {
            Some(mapped) => mapped,
            None => return Ok(None),
        },
    ];
    let points = triangle_point_refs(left, triangle)?;
    let mut splits = [Vec::new(), Vec::new(), Vec::new()];
    for &right_vertex in right_candidates {
        let point = right.view().vertex(right_vertex)?.point();
        let Some(split) = triangle_edge_split_parameter(&points, point) else {
            return Ok(None);
        };
        let Some((edge, parameter)) = split else {
            continue;
        };
        let Some(mapped) = map_right_vertex(
            left,
            right,
            right_to_left,
            left_vertex_map,
            right_vertex_map,
            vertices,
            right_vertex,
        )?
        else {
            return Ok(None);
        };
        if insert_triangle_edge_split(&mut splits[edge], vertices, mapped, point, parameter)
            .is_none()
        {
            return Ok(None);
        }
    }
    Ok(append_refined_triangle(mapped, splits, vertices, triangles))
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
) -> Result<Option<()>, ExactMeshError> {
    let mapped = [
        match map_right_vertex(
            left,
            right,
            right_to_left,
            left_vertex_map,
            right_vertex_map,
            vertices,
            triangle[0],
        )? {
            Some(mapped) => mapped,
            None => return Ok(None),
        },
        match map_right_vertex(
            left,
            right,
            right_to_left,
            left_vertex_map,
            right_vertex_map,
            vertices,
            triangle[1],
        )? {
            Some(mapped) => mapped,
            None => return Ok(None),
        },
        match map_right_vertex(
            left,
            right,
            right_to_left,
            left_vertex_map,
            right_vertex_map,
            vertices,
            triangle[2],
        )? {
            Some(mapped) => mapped,
            None => return Ok(None),
        },
    ];
    let points = triangle_point_refs(right, triangle)?;
    let mut splits = [Vec::new(), Vec::new(), Vec::new()];
    for &left_vertex in left_candidates {
        let point = left.view().vertex(left_vertex)?.point();
        let Some(split) = triangle_edge_split_parameter(&points, point) else {
            return Ok(None);
        };
        let Some((edge, parameter)) = split else {
            continue;
        };
        let Some(mapped) = map_left_vertex(left, left_vertex_map, vertices, left_vertex)? else {
            return Ok(None);
        };
        if insert_triangle_edge_split(&mut splits[edge], vertices, mapped, point, parameter)
            .is_none()
        {
            return Ok(None);
        }
    }
    Ok(append_refined_triangle(mapped, splits, vertices, triangles))
}

#[derive(Clone, Debug)]
struct TriangleEdgeSplit {
    parameter: Real,
    mapped_vertex: usize,
}

fn triangle_edge_split_parameter(
    triangle_points: &[&Point3; 3],
    point: &Point3,
) -> Option<Option<(usize, Real)>> {
    let projected_triangle = [
        (*triangle_points[0]).clone(),
        (*triangle_points[1]).clone(),
        (*triangle_points[2]).clone(),
    ];
    let projection = choose_nonzero_projected_polygon_area(&projected_triangle)?;
    for edge in 0..3 {
        let start = triangle_points[edge];
        let end = triangle_points[(edge + 1) % 3];
        if !point3_lies_strictly_on_segment(start, end, point)? {
            continue;
        }
        let parameter = projected_segment_parameter3(point, start, end, projection)?;
        return Some(Some((edge, parameter)));
    }
    Some(None)
}

fn insert_triangle_edge_split(
    splits: &mut Vec<TriangleEdgeSplit>,
    vertices: &[Point3],
    mapped_vertex: usize,
    point: &Point3,
    parameter: Real,
) -> Option<()> {
    for split in splits.iter() {
        if split.mapped_vertex == mapped_vertex {
            return Some(());
        }
        if let Some(existing) = vertices.get(split.mapped_vertex)
            && point3_exact_equal(existing, point)?
        {
            return Some(());
        }
    }
    splits.push(TriangleEdgeSplit {
        parameter,
        mapped_vertex,
    });
    Some(())
}

fn append_refined_triangle(
    mapped_triangle: [usize; 3],
    mut splits: [Vec<TriangleEdgeSplit>; 3],
    vertices: &[Point3],
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    if splits.iter().all(Vec::is_empty) {
        triangles.push(Triangle(mapped_triangle));
        return Some(());
    }
    for edge_splits in &mut splits {
        sort_edge_splits(edge_splits)?;
    }

    let mut refined = vec![Triangle(mapped_triangle)];
    for edge_splits in &splits {
        for split in edge_splits {
            split_output_triangle_edge(vertices, &mut refined, split.mapped_vertex)?;
        }
    }
    triangles.extend(refined);
    Some(())
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

fn fan_faces_cover_triangle(
    whole_mesh: &ExactMesh,
    whole_face: usize,
    fan_mesh: &ExactMesh,
    consumed_fan_faces: &BTreeSet<usize>,
) -> Result<Option<Option<Vec<usize>>>, ExactMeshError> {
    // This is intentionally a source-triangle disk certificate, not a general
    // planar arrangement. One source triangle may be consumed by an
    // opposite-oriented coplanar triangulated disk whose boundary is a
    // subdivision of the source triangle boundary and whose exact projected
    // area matches. Interior vertices are deleted with the patch; boundary
    // split vertices are retained by refining copied side faces before mesh
    // handoff.
    let whole_triangle = whole_mesh.view().face(whole_face)?.vertex_indices();
    let whole_points = triangle_point_refs(whole_mesh, whole_triangle)?;
    let whole_projection_points = [
        (*whole_points[0]).clone(),
        (*whole_points[1]).clone(),
        (*whole_points[2]).clone(),
    ];
    let Some(projection) = choose_nonzero_projected_polygon_area(&whole_projection_points) else {
        return Ok(None);
    };
    let whole_area = projected_polygon_area2_value(&whole_projection_points, projection);
    let Some(whole_sign) = real_sign(&whole_area) else {
        return Ok(None);
    };

    let mut fan_faces = Vec::new();
    let mut edge_counts = BTreeMap::<[usize; 2], usize>::new();
    let mut area_sum = Real::from(0);

    for fan_face in fan_mesh.view().faces() {
        if consumed_fan_faces.contains(&fan_face.index()) {
            continue;
        }
        let fan_triangle = fan_face.vertex_indices();
        let area_abs = match fan_triangle_in_whole_triangle(
            whole_points,
            projection,
            whole_sign,
            fan_mesh,
            fan_triangle,
        )? {
            Some(Some(area_abs)) => area_abs,
            Some(None) => continue,
            None => return Ok(None),
        };

        for edge in [
            sorted_edge([fan_triangle[0], fan_triangle[1]]),
            sorted_edge([fan_triangle[1], fan_triangle[2]]),
            sorted_edge([fan_triangle[2], fan_triangle[0]]),
        ] {
            let count = edge_counts.entry(edge).or_default();
            *count += 1;
            if *count > 2 {
                return Ok(Some(None));
            }
        }
        area_sum += area_abs;
        fan_faces.push(fan_face.index());
    }

    if fan_faces.is_empty() {
        return Ok(Some(None));
    }
    let Some(whole_area_abs) = real_abs(&whole_area) else {
        return Ok(None);
    };
    if compare_reals(&area_sum, &whole_area_abs).value() != Some(Ordering::Equal) {
        return Ok(Some(None));
    }
    let boundary_edges = edge_counts
        .iter()
        .filter_map(|(&edge, &count)| (count == 1).then_some(edge))
        .collect::<Vec<_>>();
    if boundary_edges.len() < 3 {
        return Ok(Some(None));
    }
    let Some(boundary_vertices) = order_fan_boundary_cycle(&boundary_edges) else {
        return Ok(None);
    };
    if boundary_vertices.len() != boundary_edges.len() {
        return Ok(Some(None));
    }
    for whole_point in whole_points {
        let mut matches_boundary = false;
        for &vertex in &boundary_vertices {
            let vertex = fan_mesh.view().vertex(vertex)?;
            let Some(equal) = point3_exact_equal(whole_point, vertex.point()) else {
                return Ok(None);
            };
            if equal {
                matches_boundary = true;
                break;
            }
        }
        if !matches_boundary {
            return Ok(Some(None));
        }
    }
    for vertex in boundary_vertices {
        let point = fan_mesh.view().vertex(vertex)?.point();
        let projected = project_point3(point, projection);
        let Some(location) = classify_point_triangle(
            &project_point3(whole_points[0], projection),
            &project_point3(whole_points[1], projection),
            &project_point3(whole_points[2], projection),
            &projected,
        )
        .value() else {
            return Ok(None);
        };
        if !matches!(
            location,
            TriangleLocation::OnEdge | TriangleLocation::OnVertex
        ) {
            return Ok(Some(None));
        }
    }
    Ok(Some(Some(fan_faces)))
}

fn order_fan_boundary_cycle(edges: &[[usize; 2]]) -> Option<Vec<usize>> {
    let mut adjacency = BTreeMap::<usize, Vec<usize>>::new();
    for &[a, b] in edges {
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
    whole_points: [&Point3; 3],
    projection: CoplanarProjection,
    whole_sign: Sign,
    fan_mesh: &ExactMesh,
    fan_triangle: [usize; 3],
) -> Result<Option<Option<Real>>, ExactMeshError> {
    let fan_points = triangle_point_refs(fan_mesh, fan_triangle)?;
    if !fan_points.iter().all(|point| {
        point_on_triangle_plane(whole_points[0], whole_points[1], whole_points[2], point)
            == Some(true)
    }) {
        return Ok(Some(None));
    }

    let fan_projection_points = [
        (*fan_points[0]).clone(),
        (*fan_points[1]).clone(),
        (*fan_points[2]).clone(),
    ];
    let fan_area = projected_polygon_area2_value(&fan_projection_points, projection);
    let Some(fan_sign) = real_sign(&fan_area) else {
        return Ok(None);
    };
    if fan_sign == Sign::Zero || fan_sign == whole_sign {
        return Ok(Some(None));
    }

    for fan_point in &fan_points {
        let projected = project_point3(fan_point, projection);
        let Some(location) = classify_point_triangle(
            &project_point3(whole_points[0], projection),
            &project_point3(whole_points[1], projection),
            &project_point3(whole_points[2], projection),
            &projected,
        )
        .value() else {
            return Ok(None);
        };
        if !matches!(
            location,
            TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
        ) {
            return Ok(Some(None));
        }
    }
    let Some(area_abs) = real_abs(&fan_area) else {
        return Ok(None);
    };
    if compare_reals(&area_abs, &Real::from(0)).value() != Some(Ordering::Greater) {
        return Ok(Some(None));
    }
    Ok(Some(Some(area_abs)))
}

pub(super) fn point_on_triangle_plane(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
) -> Option<bool> {
    Some(orient3d_report(a, b, c, point).value()? == Sign::Zero)
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
) -> Result<Option<[usize; 3]>, ExactMeshError> {
    let left_points = triangle_point_refs(left, left_triangle)?;
    let right_points = triangle_point_refs(right, right_triangle)?;
    let mut labels = [usize::MAX; 3];
    for (right_corner, right_point) in right_points.iter().enumerate() {
        let mut label = None;
        for (left_corner, left_point) in left_points.iter().enumerate() {
            let Some(equal) = point3_exact_equal(left_point, right_point) else {
                return Ok(None);
            };
            if equal {
                label = Some(left_corner);
                break;
            }
        }
        let Some(label) = label else {
            return Ok(None);
        };
        labels[right_corner] = label;
    }
    if !matches!(labels, [0, 2, 1] | [2, 1, 0] | [1, 0, 2]) {
        return Ok(None);
    }
    Ok(Some([
        left_triangle[labels[0]],
        left_triangle[labels[1]],
        left_triangle[labels[2]],
    ]))
}

fn triangle_point_refs(
    mesh: &ExactMesh,
    triangle: [usize; 3],
) -> Result<[&Point3; 3], ExactMeshError> {
    Ok([
        mesh.view().vertex(triangle[0])?.point(),
        mesh.view().vertex(triangle[1])?.point(),
        mesh.view().vertex(triangle[2])?.point(),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_face_materialization_rejects_stale_retained_face_rows() {
        let mut left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 2, 0, 0, 0, 2, 0],
            &[0, 1, 2],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = left.clone();
        left.facts.faces.pop();

        let certificate = FullFaceAdjacencyCertificate {
            shared_faces: vec![FullFaceAdjacentFacePair {
                left_face: 0,
                right_face: 0,
            }],
            shared_patches: Vec::new(),
        };
        let error = merged_union_mesh(
            &left,
            &right,
            &certificate,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect_err("stale retained face rows should return a typed blocker");

        assert!(
            error.has_only_blocker_kinds(&[ExactMeshBlockerKind::StaleFactReplay]),
            "{error:?}"
        );
        assert_eq!(error.blockers()[0].face(), Some(0));
    }
}
