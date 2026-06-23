//! Exact contained-face adjacency materialization for closed solids.
//!
//! Full-face adjacency can delete matching source faces directly. This module
//! handles the next bounded coplanar-volumetric case: one closed solid has a
//! source-owned boundary patch that strictly contains an opposite-oriented
//! boundary cap of the other solid. The regularized union removes the contained
//! cap and replaces the containing patch with an exact holed triangulation.
//!
//! The shortcut is intentionally narrow. It replays exact boundary-only
//! winding evidence, retained graph events, strict point-in-triangle facts, and
//! the holed face triangulation before it can authorize a closed output mesh.
//! Anything beyond this bounded topology change must be represented by the
//! general arrangement/cell-complex pipeline.
//!
//! The holed planar face uses the arrangement-backed ring triangulation model;
//! exact predicates supply the topology guards here instead of tolerance tests.

use hyperlimit::{
    CoplanarProjection, Point3, SegmentIntersection, SegmentPlaneRelation, Sign, TriangleLocation,
    classify_point_triangle, compare_reals, point_on_segment3, project_point3,
    projected_polygon_area2_value,
};

use super::arrangement2d::{ExactArrangement2dBoundaryPolicy, ExactArrangement2dSetOperation};
use super::boolean::{coplanar_mesh_overlay_carrier, materialize_coplanar_mesh_overlay_mesh};
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::graph::{
    ExactIntersectionGraph, FacePairEvents, IntersectionEvent, MeshSide,
    build_validated_intersection_graph,
};
use super::intersection::MeshFacePairRelation;
use super::mesh::{ExactMesh, ExactMeshValidationError, Triangle};
use super::topology::{mesh_for_side, triangle_tuple_edges};
use super::validation::ExactMeshValidationPolicy;
use super::winding::{
    ClosedMeshWindingRelation, classify_mesh_vertices_against_closed_mesh_winding_report,
};
use hyperlimit::SourceProvenance;
use hyperreal::Real;

use std::cmp::Ordering;
use std::collections::BTreeMap;

/// Exact materialization of a closed-solid union across contained boundary caps.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ContainedFaceAdjacentUnion {
    /// Source side whose face is replaced by a holed face remnant.
    pub containing_side: MeshSide,
    /// First face index on [`Self::containing_side`] that participates in the
    /// containing source patch.
    pub containing_face: usize,
    /// First face index on the opposite source mesh that is removed from the
    /// regularized union.
    pub contained_face: usize,
    /// All opposite-source faces removed from the regularized union.
    ///
    /// One-face certificates retain exactly this face in
    /// [`Self::contained_face`]. Multi-face certificates also replay the full
    /// set here, so copied artifacts cannot silently drop one internal cap or
    /// cap component.
    pub contained_faces: Vec<usize>,
    /// All source faces replaced by holed remnant patches.
    pub containing_faces: Vec<usize>,
    /// Closed output mesh after replacing the containing face and deleting the
    /// contained face.
    pub mesh: ExactMesh,
}

/// Opaque retained certificate for contained-face adjacency.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ContainedFaceAdjacentCertificate {
    inner: ContainedFaceAdjacencyCertificate,
}

/// Validation failure for a retained contained-face adjacency materialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ContainedFaceAdjacentUnionError {
    /// The retained source-face certificate shape is internally incoherent.
    InvalidCertificate,
    /// The retained output mesh no longer validates as an exact mesh.
    OutputMesh(ExactMeshValidationError),
    /// The retained output mesh is locally valid but is not a closed manifold.
    OutputNotClosed,
}

impl ContainedFaceAdjacentUnion {
    /// Validate the retained output mesh without consulting source operands.
    ///
    /// Local output validation and copied topology must first be a coherent
    /// exact object before boolean code consumes the retained certificate.
    pub fn validate(&self) -> Result<(), ContainedFaceAdjacentUnionError> {
        self.validate_certificate_shape()?;
        self.mesh
            .validate_retained_state_detail()
            .map_err(ContainedFaceAdjacentUnionError::OutputMesh)?;
        if !self.mesh.facts().mesh.closed_manifold {
            return Err(ContainedFaceAdjacentUnionError::OutputNotClosed);
        }
        Ok(())
    }

    fn validate_certificate_shape(&self) -> Result<(), ContainedFaceAdjacentUnionError> {
        if self.contained_faces.is_empty() || self.containing_faces.is_empty() {
            return Err(ContainedFaceAdjacentUnionError::InvalidCertificate);
        }
        if !self.contained_faces.contains(&self.contained_face)
            || !self.containing_faces.contains(&self.containing_face)
        {
            return Err(ContainedFaceAdjacentUnionError::InvalidCertificate);
        }

        let mut contained = std::collections::BTreeSet::new();
        for &face in &self.contained_faces {
            if !contained.insert(face) {
                return Err(ContainedFaceAdjacentUnionError::InvalidCertificate);
            }
        }
        let mut containing = std::collections::BTreeSet::new();
        for &face in &self.containing_faces {
            if !containing.insert(face) {
                return Err(ContainedFaceAdjacentUnionError::InvalidCertificate);
            }
        }
        Ok(())
    }
}

fn contained_face_adjacent_union_error(error: ContainedFaceAdjacentUnionError) -> ExactMeshError {
    exact_construction_failure(format!(
        "contained-face adjacent union retained output failed validation: {error:?}"
    ))
}

fn exact_construction_failure(message: impl Into<String>) -> ExactMeshError {
    ExactMeshError::one(ExactMeshBlocker::new(
        ExactMeshBlockerKind::ExactConstructionFailure,
        message,
    ))
}

/// Certify and materialize a contained-face adjacent closed-solid union.
pub(crate) fn materialize_contained_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ContainedFaceAdjacentUnion>, ExactMeshError> {
    let Some(certificate) = contained_face_adjacent_certificate(left, right)? else {
        return Ok(None);
    };
    materialize_contained_face_adjacent_union_from_certificate(
        left,
        right,
        &certificate,
        validation,
    )
}

/// Return the retained contained-face adjacency certificate for these sources.
pub(crate) fn contained_face_adjacent_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ContainedFaceAdjacentCertificate>, ExactMeshError> {
    Ok(contained_face_adjacent_union_certificate(left, right)?
        .map(|inner| ContainedFaceAdjacentCertificate { inner }))
}

/// Return the retained contained-face adjacency certificate from a validated graph.
pub(crate) fn contained_face_adjacent_certificate_from_graph(
    left: &ExactMesh,
    right: &ExactMesh,
    graph: &ExactIntersectionGraph,
) -> Result<Option<ContainedFaceAdjacentCertificate>, ExactMeshError> {
    Ok(
        contained_face_adjacent_union_certificate_from_graph(left, right, graph)?
            .map(|inner| ContainedFaceAdjacentCertificate { inner }),
    )
}

/// Materialize a contained-face adjacent union from an already-retained certificate.
pub(crate) fn materialize_contained_face_adjacent_union_from_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
    certificate: &ContainedFaceAdjacentCertificate,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ContainedFaceAdjacentUnion>, ExactMeshError> {
    let certificate = &certificate.inner;
    let Some(mesh) = contained_face_union_mesh(left, right, certificate, validation)? else {
        return Ok(None);
    };
    let union = ContainedFaceAdjacentUnion {
        containing_side: certificate.containing_side,
        containing_face: certificate.containing_face(),
        contained_face: certificate.contained_faces()[0],
        contained_faces: certificate.contained_faces(),
        containing_faces: certificate.containing_faces(),
        mesh,
    };
    union
        .validate()
        .map_err(contained_face_adjacent_union_error)?;
    Ok(Some(union))
}

fn contained_face_adjacent_union_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ContainedFaceAdjacencyCertificate>, ExactMeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    let graph = build_validated_intersection_graph(left, right)?;
    if graph.has_unknowns() || graph.face_pairs.is_empty() {
        return Ok(None);
    }
    if !closed_boundary_contact_only(left, right)? {
        return Ok(None);
    }

    Ok(contained_face_adjacency_certificate(
        left,
        right,
        &graph.face_pairs,
    ))
}

fn contained_face_adjacent_union_certificate_from_graph(
    left: &ExactMesh,
    right: &ExactMesh,
    graph: &ExactIntersectionGraph,
) -> Result<Option<ContainedFaceAdjacencyCertificate>, ExactMeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    if graph.has_unknowns() || graph.face_pairs.is_empty() {
        return Ok(None);
    }
    if !closed_boundary_contact_only(left, right)? {
        return Ok(None);
    }

    Ok(contained_face_adjacency_certificate(
        left,
        right,
        &graph.face_pairs,
    ))
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct ContainedFaceAdjacencyCertificate {
    containing_side: MeshSide,
    patches: Vec<ContainedFacePatch>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct ContainedFacePatch {
    containing_faces: Vec<usize>,
    contained_faces: Vec<usize>,
    projection: CoplanarProjection,
    containing_projected_sign: Sign,
}

impl ContainedFaceAdjacencyCertificate {
    fn containing_face(&self) -> usize {
        self.patches[0].containing_faces[0]
    }

    fn contained_faces(&self) -> Vec<usize> {
        let mut faces = Vec::new();
        for patch in &self.patches {
            for &face in &patch.contained_faces {
                if !faces.contains(&face) {
                    faces.push(face);
                }
            }
        }
        faces
    }

    fn containing_faces(&self) -> Vec<usize> {
        let mut faces = Vec::new();
        for patch in &self.patches {
            for &face in &patch.containing_faces {
                if !faces.contains(&face) {
                    faces.push(face);
                }
            }
        }
        faces
    }
}

fn contained_face_adjacency_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
    pairs: &[FacePairEvents],
) -> Option<ContainedFaceAdjacencyCertificate> {
    component_contained_adjacency_certificate(left, right, pairs).filter(|certificate| {
        pairs
            .iter()
            .all(|pair| contained_adjacency_contact_pair(left, right, pair, certificate))
    })
}

/// Certify one connected coplanar containing component with one or more
/// contained cap components.
///
/// This is still a bounded shortcut, not the arbitrary volumetric cell
/// materializer. It promotes only the case where all positive-area coplanar
/// overlaps form one convex containing surface and one strictly contained
/// convex cap set. The check is performed by replaying the retained source
/// face sets through the coplanar convex holed surface artifacts, preserving
/// exact source-owned representative geometry.
fn component_contained_adjacency_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
    pairs: &[FacePairEvents],
) -> Option<ContainedFaceAdjacencyCertificate> {
    let overlapping = pairs
        .iter()
        .filter(|pair| pair.relation == MeshFacePairRelation::CoplanarOverlapping)
        .collect::<Vec<_>>();
    if overlapping.is_empty() {
        return None;
    }

    let components = overlap_face_components(&overlapping)?;
    component_contained_adjacency_components_for_side(MeshSide::Left, left, right, &components)
        .or_else(|| {
            component_contained_adjacency_components_for_side(
                MeshSide::Right,
                right,
                left,
                &components,
            )
        })
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct OverlapFaceComponent {
    left_faces: Vec<usize>,
    right_faces: Vec<usize>,
}

/// Split retained coplanar overlaps into independent source-face components.
///
/// The component contained-face certificate is a bounded replacement for a
/// general coplanar volumetric cell materializer. Treating disconnected
/// source patches as one polygon would invent topology that no exact source
/// object owns. We therefore group the bipartite overlap graph by shared left
/// and right source ownership before topology can be promoted.
fn overlap_face_components(overlapping: &[&FacePairEvents]) -> Option<Vec<OverlapFaceComponent>> {
    if overlapping.is_empty() {
        return None;
    }
    let mut components = PairUnionFind::new(overlapping.len());
    for left in 0..overlapping.len() {
        for right in left + 1..overlapping.len() {
            if overlapping[left].left_face == overlapping[right].left_face
                || overlapping[left].right_face == overlapping[right].right_face
            {
                components.union(left, right);
            }
        }
    }

    let mut grouped = Vec::<OverlapFaceComponent>::new();
    let mut roots = Vec::<usize>::new();
    for (index, pair) in overlapping.iter().enumerate() {
        let root = components.find(index);
        let group_index = if let Some(existing) = roots.iter().position(|&seen| seen == root) {
            existing
        } else {
            roots.push(root);
            grouped.push(OverlapFaceComponent {
                left_faces: Vec::new(),
                right_faces: Vec::new(),
            });
            grouped.len() - 1
        };
        push_unique_face(&mut grouped[group_index].left_faces, pair.left_face);
        push_unique_face(&mut grouped[group_index].right_faces, pair.right_face);
    }
    Some(grouped)
}

fn component_contained_adjacency_components_for_side(
    containing_side: MeshSide,
    containing_source: &ExactMesh,
    contained_source: &ExactMesh,
    components: &[OverlapFaceComponent],
) -> Option<ContainedFaceAdjacencyCertificate> {
    let mut patches = Vec::with_capacity(components.len());
    for component in components {
        let (containing_faces, contained_faces) = match containing_side {
            MeshSide::Left => (component.left_faces.clone(), component.right_faces.clone()),
            MeshSide::Right => (component.right_faces.clone(), component.left_faces.clone()),
        };
        let certificate = component_contained_adjacency_for_side(
            containing_side,
            containing_source,
            contained_source,
            containing_faces,
            contained_faces,
        )?;
        patches.extend(certificate.patches);
    }
    Some(ContainedFaceAdjacencyCertificate {
        containing_side,
        patches,
    })
}

fn component_contained_adjacency_for_side(
    containing_side: MeshSide,
    containing_source: &ExactMesh,
    contained_source: &ExactMesh,
    containing_faces: Vec<usize>,
    contained_faces: Vec<usize>,
) -> Option<ContainedFaceAdjacencyCertificate> {
    if containing_faces.is_empty() || contained_faces.is_empty() {
        return None;
    }
    let containing_mesh = faces_mesh(
        containing_source,
        &containing_faces,
        "exact contained-face adjacency containing component",
    )?;
    let contained_mesh = faces_mesh(
        contained_source,
        &contained_faces,
        "exact contained-face adjacency contained component",
    )?;
    if connected_face_components(&containing_mesh)?.len() != 1 {
        return None;
    }
    let (_, arrangement_projection) =
        materialize_contained_patch_difference(&containing_mesh, &contained_mesh)?;
    let sign = first_projected_mesh_triangle_sign(&containing_mesh, arrangement_projection)?;
    if !mesh_projected_triangle_signs_match(&containing_mesh, arrangement_projection, sign)?
        || !mesh_projected_triangle_signs_match(
            &contained_mesh,
            arrangement_projection,
            opposite_sign(sign),
        )?
    {
        return None;
    }
    Some(ContainedFaceAdjacencyCertificate {
        containing_side,
        patches: vec![ContainedFacePatch {
            containing_faces,
            contained_faces,
            projection: arrangement_projection,
            containing_projected_sign: sign,
        }],
    })
}

fn push_unique_face(faces: &mut Vec<usize>, face: usize) {
    if !faces.contains(&face) {
        faces.push(face);
    }
}

fn contained_adjacency_contact_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    pair: &FacePairEvents,
    certificate: &ContainedFaceAdjacencyCertificate,
) -> bool {
    if certificate_face_pair_contains(certificate, pair.left_face, pair.right_face) {
        return pair.relation == MeshFacePairRelation::CoplanarOverlapping;
    }

    match pair.relation {
        MeshFacePairRelation::CoplanarTouching => true,
        MeshFacePairRelation::Candidate => pair
            .events
            .iter()
            .all(|event| boundary_candidate_event(left, right, event)),
        MeshFacePairRelation::PlaneSeparated => true,
        MeshFacePairRelation::CoplanarOverlapping | MeshFacePairRelation::Unknown => false,
    }
}

fn certificate_face_pair_contains(
    certificate: &ContainedFaceAdjacencyCertificate,
    left_face: usize,
    right_face: usize,
) -> bool {
    match certificate.containing_side {
        MeshSide::Left => certificate.patches.iter().any(|patch| {
            patch.containing_faces.contains(&left_face)
                && patch.contained_faces.contains(&right_face)
        }),
        MeshSide::Right => certificate.patches.iter().any(|patch| {
            patch.containing_faces.contains(&right_face)
                && patch.contained_faces.contains(&left_face)
        }),
    }
}

fn boundary_candidate_event(
    left: &ExactMesh,
    right: &ExactMesh,
    event: &IntersectionEvent,
) -> bool {
    match event {
        IntersectionEvent::SegmentPlane { relation, .. } => {
            matches!(
                relation,
                SegmentPlaneRelation::Disjoint
                    | SegmentPlaneRelation::Coplanar
                    | SegmentPlaneRelation::EndpointOnPlane
            ) || (*relation == SegmentPlaneRelation::ProperCrossing
                && retained_plane_crossing_is_not_inside_plane_face(left, right, event))
        }
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

fn retained_plane_crossing_is_not_inside_plane_face(
    left: &ExactMesh,
    right: &ExactMesh,
    event: &IntersectionEvent,
) -> bool {
    let IntersectionEvent::SegmentPlane {
        relation: SegmentPlaneRelation::ProperCrossing,
        plane_side,
        plane_face,
        point: Some(point),
        ..
    } = event
    else {
        return false;
    };
    // The graph may retain a source edge crossing the opposite face's
    // supporting plane even when the constructed point lies outside that
    // finite triangle, or exactly on its boundary. That construction is exact
    // evidence for splitting, not for volume overlap; preserving the
    // shortcut consumes. Strict interior crossings remain blockers.
    let Some(triangle) = triangle_points(mesh_for_side(*plane_side, left, right), *plane_face)
    else {
        return false;
    };
    let Some(projection) = choose_triangle_projection(&triangle) else {
        return false;
    };
    classify_point_triangle(
        &project_point3(&triangle[0], projection),
        &project_point3(&triangle[1], projection),
        &project_point3(&triangle[2], projection),
        &project_point3(point, projection),
    )
    .value()
    .is_some_and(|location| location != TriangleLocation::Inside)
}

fn contained_face_union_mesh(
    left: &ExactMesh,
    right: &ExactMesh,
    certificate: &ContainedFaceAdjacencyCertificate,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    if append_source_mesh_without_face(
        left,
        skip_faces_for_side(certificate, MeshSide::Left),
        &mut vertices,
        &mut triangles,
    )
    .is_none()
    {
        return Ok(None);
    }
    if append_source_mesh_without_face(
        right,
        skip_faces_for_side(certificate, MeshSide::Right),
        &mut vertices,
        &mut triangles,
    )
    .is_none()
    {
        return Ok(None);
    }
    for group in contained_face_patch_groups(certificate) {
        if append_contained_face_patch_group(
            left,
            right,
            certificate.containing_side,
            &group,
            &mut vertices,
            &mut triangles,
        )
        .is_none()
        {
            return Ok(None);
        }
    }

    let mesh = ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact contained-face adjacent closed-solid union"),
        validation,
    )?;
    Ok(Some(mesh))
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct ContainedFacePatchGroup {
    containing_faces: Vec<usize>,
    contained_faces: Vec<usize>,
    projection: CoplanarProjection,
    containing_projected_sign: Sign,
}

fn contained_face_patch_groups(
    certificate: &ContainedFaceAdjacencyCertificate,
) -> Vec<ContainedFacePatchGroup> {
    certificate
        .patches
        .iter()
        .map(|patch| ContainedFacePatchGroup {
            containing_faces: patch.containing_faces.clone(),
            contained_faces: patch.contained_faces.clone(),
            projection: patch.projection,
            containing_projected_sign: patch.containing_projected_sign,
        })
        .collect()
}

/// Append one retained holed replacement for a containing source face.
///
/// A single contained cap uses the one-hole triangle arrangement. Multiple
/// caps on the same containing face are first split into retained connected
/// face components: one connected cap uses the convex one-hole surface
/// artifact, while several independent caps use the convex multi-hole surface
/// artifact. Both retain exact rings and validate exact area before
/// than inferring holes from triangle soup. The ring triangulation handoff
fn append_contained_face_patch_group(
    left: &ExactMesh,
    right: &ExactMesh,
    containing_side: MeshSide,
    group: &ContainedFacePatchGroup,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    let containing_mesh = faces_mesh(
        mesh_for_side(containing_side, left, right),
        &group.containing_faces,
        "exact contained-face adjacency containing faces",
    )?;
    let contained_mesh = faces_mesh(
        mesh_for_side(opposite_side(containing_side), left, right),
        &group.contained_faces,
        "exact contained-face adjacency contained faces",
    )?;
    let (replacement, _) =
        materialize_contained_patch_difference(&containing_mesh, &contained_mesh)?;
    append_holed_replacement(
        &replacement,
        group.projection,
        group.containing_projected_sign,
        vertices,
        triangles,
    )
}

fn materialize_contained_patch_difference(
    containing_mesh: &ExactMesh,
    contained_mesh: &ExactMesh,
) -> Option<(ExactMesh, CoplanarProjection)> {
    let (_, projection) = coplanar_mesh_overlay_carrier(containing_mesh, contained_mesh)?;
    for boundary_policy in [
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
        ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
    ] {
        if let Some(mesh) = materialize_coplanar_mesh_overlay_mesh(
            containing_mesh,
            contained_mesh,
            ExactArrangement2dSetOperation::Difference,
            boundary_policy,
            "exact contained-adjacent arrangement patch difference",
            false,
        )
        .ok()
        .flatten()
        {
            return Some((mesh, projection));
        }
    }
    None
}

fn faces_mesh(mesh: &ExactMesh, faces: &[usize], label: &'static str) -> Option<ExactMesh> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for &face in faces {
        let triangle = mesh.triangles().get(face)?.0;
        triangles.push(Triangle([
            map_point(&mut vertices, &mesh.vertices().get(triangle[0])?.clone())?,
            map_point(&mut vertices, &mesh.vertices().get(triangle[1])?.clone())?,
            map_point(&mut vertices, &mesh.vertices().get(triangle[2])?.clone())?,
        ]));
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn connected_face_components(mesh: &ExactMesh) -> Option<Vec<Vec<usize>>> {
    if mesh.triangles().is_empty() {
        return None;
    }
    let mut visited = vec![false; mesh.triangles().len()];
    let mut components = Vec::new();
    for seed in 0..mesh.triangles().len() {
        if visited[seed] {
            continue;
        }
        let mut component = Vec::new();
        let mut stack = vec![seed];
        visited[seed] = true;
        while let Some(face) = stack.pop() {
            component.push(face);
            for (neighbor, neighbor_visited) in visited.iter_mut().enumerate() {
                if !*neighbor_visited
                    && triangles_share_edge(mesh.triangles()[face], mesh.triangles()[neighbor])
                {
                    *neighbor_visited = true;
                    stack.push(neighbor);
                }
            }
        }
        components.push(component);
    }
    Some(components)
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct PairUnionFind {
    parent: Vec<usize>,
}

impl PairUnionFind {
    fn new(len: usize) -> Self {
        Self {
            parent: (0..len).collect(),
        }
    }

    fn find(&mut self, index: usize) -> usize {
        let parent = self.parent[index];
        if parent == index {
            index
        } else {
            let root = self.find(parent);
            self.parent[index] = root;
            root
        }
    }

    fn union(&mut self, left: usize, right: usize) {
        let left_root = self.find(left);
        let right_root = self.find(right);
        if left_root != right_root {
            self.parent[right_root] = left_root;
        }
    }
}

fn triangles_share_edge(left: Triangle, right: Triangle) -> bool {
    triangle_tuple_edges(left).iter().any(|left| {
        triangle_tuple_edges(right)
            .iter()
            .any(|right| left == right)
    })
}

fn append_source_mesh_without_face(
    mesh: &ExactMesh,
    skip_faces: Vec<usize>,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    for (face, triangle) in mesh.triangles().iter().enumerate() {
        if skip_faces.contains(&face) {
            continue;
        }
        triangles.push(Triangle([
            map_point(vertices, &mesh.vertices().get(triangle.0[0])?.clone())?,
            map_point(vertices, &mesh.vertices().get(triangle.0[1])?.clone())?,
            map_point(vertices, &mesh.vertices().get(triangle.0[2])?.clone())?,
        ]));
    }
    Some(())
}

fn append_holed_replacement(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    target_sign: Sign,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    let source_sign = first_projected_mesh_triangle_sign(mesh, projection)?;
    let flip = source_sign != target_sign;
    for triangle in mesh.triangles() {
        let points = [
            mesh.vertices().get(triangle.0[0])?.clone(),
            mesh.vertices().get(triangle.0[1])?.clone(),
            mesh.vertices().get(triangle.0[2])?.clone(),
        ];
        let mapped = [
            map_point(vertices, &points[0])?,
            map_point(vertices, &points[1])?,
            map_point(vertices, &points[2])?,
        ];
        let (triangle_points, mapped_triangle) = if flip {
            (
                [points[0].clone(), points[2].clone(), points[1].clone()],
                [mapped[0], mapped[2], mapped[1]],
            )
        } else {
            (points, mapped)
        };
        append_triangle_with_existing_edge_splits(
            &triangle_points,
            mapped_triangle,
            vertices,
            triangles,
        )?;
    }
    Some(())
}

fn append_triangle_with_existing_edge_splits(
    triangle_points: &[Point3; 3],
    mapped_triangle: [usize; 3],
    vertices: &[Point3],
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    let mut point_by_vertex = BTreeMap::<usize, Point3>::new();
    for index in 0..3 {
        point_by_vertex.insert(mapped_triangle[index], triangle_points[index].clone());
    }
    let mut split_vertices = Vec::new();
    for (candidate, point) in vertices.iter().enumerate() {
        if mapped_triangle.contains(&candidate) {
            continue;
        }
        if triangle_edge_contains_strict_point(triangle_points, point)? {
            point_by_vertex.insert(candidate, point.clone());
            split_vertices.push(candidate);
        }
    }
    if split_vertices.is_empty() {
        triangles.push(Triangle(mapped_triangle));
        return Some(());
    }

    let mut refined = vec![Triangle(mapped_triangle)];
    for split_vertex in split_vertices {
        split_output_triangle_edge(&point_by_vertex, &mut refined, split_vertex)?;
    }
    triangles.extend(refined);
    Some(())
}

fn triangle_edge_contains_strict_point(
    triangle_points: &[Point3; 3],
    point: &Point3,
) -> Option<bool> {
    for edge in 0..3 {
        if point_lies_strictly_on_segment3(
            &triangle_points[edge],
            &triangle_points[(edge + 1) % 3],
            point,
        )? {
            return Some(true);
        }
    }
    Some(false)
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

fn point_lies_strictly_on_segment3(start: &Point3, end: &Point3, point: &Point3) -> Option<bool> {
    if points_equal(point, start)? || points_equal(point, end)? {
        return Some(false);
    }
    point_on_segment3(start, end, point).value()
}

fn map_point(vertices: &mut Vec<Point3>, point: &Point3) -> Option<usize> {
    if let Some(existing) = vertices
        .iter()
        .position(|candidate| points_equal(&candidate.clone(), point) == Some(true))
    {
        return Some(existing);
    }
    let mapped = vertices.len();
    vertices.push(point_to_exact(point));
    Some(mapped)
}

fn point_to_exact(point: &Point3) -> Point3 {
    Point3::new(point.x.clone(), point.y.clone(), point.z.clone())
}

fn skip_faces_for_side(
    certificate: &ContainedFaceAdjacencyCertificate,
    side: MeshSide,
) -> Vec<usize> {
    match (certificate.containing_side, side) {
        (MeshSide::Left, MeshSide::Left) | (MeshSide::Right, MeshSide::Right) => {
            certificate.containing_faces()
        }
        (MeshSide::Left, MeshSide::Right) | (MeshSide::Right, MeshSide::Left) => {
            certificate.contained_faces()
        }
    }
}

const fn opposite_side(side: MeshSide) -> MeshSide {
    match side {
        MeshSide::Left => MeshSide::Right,
        MeshSide::Right => MeshSide::Left,
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
    .find(|&projection| projected_triangle_sign(points, projection).is_some())
}

fn projected_triangle_sign(points: &[Point3; 3], projection: CoplanarProjection) -> Option<Sign> {
    let area = projected_polygon_area2_value(points, projection);
    match real_sign(&area)? {
        Sign::Zero => None,
        sign => Some(sign),
    }
}

fn first_projected_mesh_triangle_sign(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
) -> Option<Sign> {
    mesh.triangles().iter().find_map(|triangle| {
        let points = [
            mesh.vertices().get(triangle.0[0])?.clone(),
            mesh.vertices().get(triangle.0[1])?.clone(),
            mesh.vertices().get(triangle.0[2])?.clone(),
        ];
        projected_triangle_sign(&points, projection)
    })
}

fn mesh_projected_triangle_signs_match(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    expected: Sign,
) -> Option<bool> {
    for triangle in mesh.triangles() {
        let points = [
            mesh.vertices().get(triangle.0[0])?.clone(),
            mesh.vertices().get(triangle.0[1])?.clone(),
            mesh.vertices().get(triangle.0[2])?.clone(),
        ];
        if projected_triangle_sign(&points, projection)? != expected {
            return Some(false);
        }
    }
    Some(true)
}

const fn opposite_sign(sign: Sign) -> Sign {
    match sign {
        Sign::Negative => Sign::Positive,
        Sign::Positive => Sign::Negative,
        Sign::Zero => Sign::Zero,
    }
}

fn real_sign(value: &Real) -> Option<Sign> {
    match compare_reals(value, &Real::from(0)).value()? {
        Ordering::Less => Some(Sign::Negative),
        Ordering::Equal => Some(Sign::Zero),
        Ordering::Greater => Some(Sign::Positive),
    }
}

fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    Some(
        compare_reals(&left.x, &right.x).value()? == Ordering::Equal
            && compare_reals(&left.y, &right.y).value()? == Ordering::Equal
            && compare_reals(&left.z, &right.z).value()? == Ordering::Equal,
    )
}

fn closed_boundary_contact_only(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactMeshError> {
    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right.validate().map_err(|error| {
        exact_construction_failure(format!(
            "contained-face adjacent boundary-contact left-in-right winding report failed validation: {error:?}"
        ))
    })?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left.validate().map_err(|error| {
        exact_construction_failure(format!(
            "contained-face adjacent boundary-contact right-in-left winding report failed validation: {error:?}"
        ))
    })?;
    Ok(mesh_vertices_are_boundary_or_outside(&left_in_right)
        && mesh_vertices_are_boundary_or_outside(&right_in_left)
        && (mesh_vertices_touch_boundary(&left_in_right)
            || mesh_vertices_touch_boundary(&right_in_left)))
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
