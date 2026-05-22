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
//!
//! The union artifact is also the proof object used by named boolean dispatch
//! for the matching regularized intersection and difference shortcuts:
//! boundary-only full-face/fan-patch contact contributes no intersection
//! volume, and subtracting the adjacent right solid preserves the left solid.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{
    CoplanarProjection, Point3, SegmentIntersection, Sign, TriangleLocation,
    classify_point_triangle, compare_reals, orient3d_report, project_point3,
    projected_polygon_area2_value,
};

use super::construction::SegmentPlaneRelation;
use super::graph::{FacePairEvents, IntersectionEvent, build_intersection_graph};
use super::intersection::MeshFacePairRelation;
use super::mesh::{ExactMesh, ExactMeshValidationError, ExactPoint3, Triangle};
use super::provenance::SourceProvenance;
use super::scalar::ExactReal;
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

    let certificate = full_face_adjacency_certificate(left, right)?;
    if certificate.is_empty() {
        return None;
    }
    if !graph_has_only_adjacency_contacts(left, right, &graph.face_pairs, &certificate) {
        return None;
    }

    let mesh = merged_union_mesh(left, right, &certificate, validation)?;
    let union = FullFaceAdjacentUnion {
        shared_faces: certificate.shared_faces,
        shared_patches: certificate.shared_patches,
        mesh,
    };
    union.validate().ok()?;
    Some(union)
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

    for (face, triangle) in left.triangles().iter().enumerate() {
        if skip_left.contains(&face) {
            continue;
        }
        triangles.push(Triangle([
            map_left_vertex(left, &mut left_vertex_map, &mut vertices, triangle.0[0])?,
            map_left_vertex(left, &mut left_vertex_map, &mut vertices, triangle.0[1])?,
            map_left_vertex(left, &mut left_vertex_map, &mut vertices, triangle.0[2])?,
        ]));
    }

    for (face, triangle) in right.triangles().iter().enumerate() {
        if skip_right.contains(&face) {
            continue;
        }
        triangles.push(Triangle([
            map_right_vertex(
                left,
                right,
                &right_to_left,
                &mut left_vertex_map,
                &mut right_vertex_map,
                &mut vertices,
                triangle.0[0],
            )?,
            map_right_vertex(
                left,
                right,
                &right_to_left,
                &mut left_vertex_map,
                &mut right_vertex_map,
                &mut vertices,
                triangle.0[1],
            )?,
            map_right_vertex(
                left,
                right,
                &right_to_left,
                &mut left_vertex_map,
                &mut right_vertex_map,
                &mut vertices,
                triangle.0[2],
            )?,
        ]));
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
        let right_point = right.vertices().get(right_vertex)?.to_hyperlimit_point();
        if let Some(left_vertex) = left_vertices.iter().copied().find(|&left_vertex| {
            let left_point = left.vertices()[left_vertex].to_hyperlimit_point();
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
    vertices: &mut Vec<ExactPoint3>,
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
    vertices: &mut Vec<ExactPoint3>,
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

fn fan_faces_cover_triangle(
    whole_mesh: &ExactMesh,
    whole_face: usize,
    fan_mesh: &ExactMesh,
    consumed_fan_faces: &BTreeSet<usize>,
) -> Option<Option<Vec<usize>>> {
    // This is intentionally a bounded certificate, not a general planar
    // arrangement. One source triangle may be consumed by exactly three
    // opposite-oriented coplanar triangles sharing one strict interior fan
    // point and covering the three boundary edges. The exact projected area
    // equality is a construction replay guard; the topological decision still
    // comes from retained point/plane/triangle predicates following Yap,
    // "Towards Exact Geometric Computation," Comput. Geom. 7.1-2 (1997).
    let whole_triangle = whole_mesh.triangles().get(whole_face)?.0;
    let whole_points = triangle_points(whole_mesh, whole_triangle)?;
    let projection = choose_triangle_projection(&whole_points)?;
    let whole_area = projected_polygon_area2_value(&whole_points, projection);
    let whole_sign = real_sign(&whole_area)?;

    let mut fan_faces = Vec::new();
    let mut covered_edges = BTreeSet::new();
    let mut interior_point = None::<Point3>;
    let mut area_sum = ExactReal::from(0);

    for (fan_face, fan_triangle) in fan_mesh.triangles().iter().enumerate() {
        if consumed_fan_faces.contains(&fan_face) {
            continue;
        }
        let Some(candidate) = fan_triangle_in_whole_triangle(
            &whole_points,
            projection,
            whole_sign,
            fan_mesh,
            fan_triangle.0,
        )?
        else {
            continue;
        };

        if !covered_edges.insert(candidate.covered_edge) {
            return Some(None);
        }
        match &interior_point {
            Some(existing) if points_equal(existing, &candidate.interior_point) != Some(true) => {
                return Some(None);
            }
            Some(_) => {}
            None => interior_point = Some(candidate.interior_point),
        }
        area_sum = area_sum + candidate.area_abs;
        fan_faces.push(fan_face);
    }

    if fan_faces.len() != 3 || covered_edges.len() != 3 {
        return Some(None);
    }
    let whole_area_abs = real_abs(&whole_area)?;
    if compare_reals(&area_sum, &whole_area_abs).value() != Some(Ordering::Equal) {
        return Some(None);
    }
    Some(Some(fan_faces))
}

#[derive(Clone, Debug, PartialEq)]
struct FanTriangleCandidate {
    covered_edge: usize,
    interior_point: Point3,
    area_abs: ExactReal,
}

fn fan_triangle_in_whole_triangle(
    whole_points: &[Point3; 3],
    projection: CoplanarProjection,
    whole_sign: Sign,
    fan_mesh: &ExactMesh,
    fan_triangle: [usize; 3],
) -> Option<Option<FanTriangleCandidate>> {
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

    let mut labels = Vec::new();
    let mut interior = None::<Point3>;
    for fan_point in &fan_points {
        if let Some(label) = whole_points
            .iter()
            .position(|whole_point| points_equal(whole_point, fan_point) == Some(true))
        {
            labels.push(label);
            continue;
        }
        let projected = project_point3(fan_point, projection);
        let location = classify_point_triangle(
            &project_point3(&whole_points[0], projection),
            &project_point3(&whole_points[1], projection),
            &project_point3(&whole_points[2], projection),
            &projected,
        )
        .value()?;
        if location != TriangleLocation::Inside || interior.is_some() {
            return Some(None);
        }
        interior = Some(fan_point.clone());
    }

    if labels.len() != 2 {
        return Some(None);
    }
    labels.sort_unstable();
    let covered_edge = match labels.as_slice() {
        [0, 1] => 0,
        [1, 2] => 1,
        [0, 2] => 2,
        _ => return Some(None),
    };
    let area_abs = real_abs(&fan_area)?;
    if compare_reals(&area_abs, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
        return Some(None);
    }
    Some(Some(FanTriangleCandidate {
        covered_edge,
        interior_point: interior?,
        area_abs,
    }))
}

fn point_on_triangle_plane(triangle: &[Point3; 3], point: &Point3) -> Option<bool> {
    Some(orient3d_report(&triangle[0], &triangle[1], &triangle[2], point).value()? == Sign::Zero)
}

fn choose_triangle_projection(points: &[Point3; 3]) -> Option<CoplanarProjection> {
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

fn real_abs(value: &ExactReal) -> Option<ExactReal> {
    match real_sign(value)? {
        Sign::Negative => Some(-value.clone()),
        Sign::Zero | Sign::Positive => Some(value.clone()),
    }
}

fn real_sign(value: &ExactReal) -> Option<Sign> {
    match compare_reals(value, &ExactReal::from(0)).value()? {
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
