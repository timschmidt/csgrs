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
    classify_point_triangle, orient2d_report, project_point3,
};

use super::super::arrangement3d::arrangement2d::{
    ExactArrangement2dBoundaryPolicy, ExactArrangement2dSetOperation,
};
use super::super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::super::graph::intersection::MeshFacePairRelation;
use super::super::graph::{ExactIntersectionGraph, FacePairEvents, IntersectionEvent, MeshSide};
use super::super::validation::ExactMeshValidationPolicy;
use super::super::{ExactMesh, ExactMeshValidationError, Triangle, sorted_edge};
use super::closed_boundary_contact_only;
use super::{
    DisjointSets, coplanar_mesh_overlay_carrier, materialize_coplanar_mesh_overlay_mesh,
    point3_exact_equal, point3_lies_strictly_on_segment, split_output_triangle_edge,
};
use hyperlimit::SourceProvenance;

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

/// Return the retained contained-face adjacency certificate from a validated graph.
pub(crate) fn contained_face_adjacent_certificate_from_graph(
    left: &ExactMesh,
    right: &ExactMesh,
    graph: &ExactIntersectionGraph,
) -> Result<Option<ContainedFaceAdjacentCertificate>, ExactMeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    if graph.has_unknowns() || graph.face_pairs.is_empty() {
        return Ok(None);
    }
    if !closed_boundary_contact_only(left, right)? {
        return Ok(None);
    }

    Ok(
        contained_face_adjacency_certificate(left, right, &graph.face_pairs)?
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
    let mut contained_faces = Vec::new();
    let mut containing_faces = Vec::new();
    for patch in &certificate.patches {
        for &face in &patch.contained_faces {
            if !contained_faces.contains(&face) {
                contained_faces.push(face);
            }
        }
        for &face in &patch.containing_faces {
            if !containing_faces.contains(&face) {
                containing_faces.push(face);
            }
        }
    }
    let (Some(containing_face), Some(contained_face)) = (
        containing_faces.first().copied(),
        contained_faces.first().copied(),
    ) else {
        return Ok(None);
    };
    let Some(mesh) = contained_face_union_mesh(
        left,
        right,
        certificate.containing_side,
        &certificate.patches,
        &containing_faces,
        &contained_faces,
        validation,
    )?
    else {
        return Ok(None);
    };
    let union = ContainedFaceAdjacentUnion {
        containing_side: certificate.containing_side,
        containing_face,
        contained_face,
        contained_faces,
        containing_faces,
        mesh,
    };
    union.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("contained-face adjacent union retained output failed validation: {error:?}"),
        ))
    })?;
    Ok(Some(union))
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

fn contained_face_adjacency_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
    pairs: &[FacePairEvents],
) -> Result<Option<ContainedFaceAdjacencyCertificate>, ExactMeshError> {
    let Some(certificate) = component_contained_adjacency_certificate(left, right, pairs)? else {
        return Ok(None);
    };
    for pair in pairs {
        if !contained_adjacency_contact_pair(left, right, pair, &certificate)? {
            return Ok(None);
        }
    }
    Ok(Some(certificate))
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
) -> Result<Option<ContainedFaceAdjacencyCertificate>, ExactMeshError> {
    let overlapping = pairs
        .iter()
        .filter(|pair| pair.relation == MeshFacePairRelation::CoplanarOverlapping)
        .collect::<Vec<_>>();
    if overlapping.is_empty() {
        return Ok(None);
    }

    let Some(components) = overlap_face_components(&overlapping) else {
        return Ok(None);
    };
    'sides: for (containing_side, containing_source, contained_source) in [
        (MeshSide::Left, left, right),
        (MeshSide::Right, right, left),
    ] {
        let mut patches = Vec::with_capacity(components.len());
        for component in &components {
            let (containing_faces, contained_faces) = match containing_side {
                MeshSide::Left => (
                    component.left_faces.as_slice(),
                    component.right_faces.as_slice(),
                ),
                MeshSide::Right => (
                    component.right_faces.as_slice(),
                    component.left_faces.as_slice(),
                ),
            };
            let Some(certificate) = component_contained_adjacency_for_side(
                containing_side,
                containing_source,
                contained_source,
                containing_faces,
                contained_faces,
            )?
            else {
                continue 'sides;
            };
            patches.extend(certificate.patches);
        }
        return Ok(Some(ContainedFaceAdjacencyCertificate {
            containing_side,
            patches,
        }));
    }
    Ok(None)
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
    let mut components = DisjointSets {
        parent: (0..overlapping.len()).collect(),
    };
    for left in 0..overlapping.len() {
        for right in left + 1..overlapping.len() {
            if overlapping[left].left_face == overlapping[right].left_face
                || overlapping[left].right_face == overlapping[right].right_face
            {
                let left_root = components.find(left);
                let right_root = components.find(right);
                if left_root != right_root {
                    components.parent[right_root] = left_root;
                }
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
        if !grouped[group_index].left_faces.contains(&pair.left_face) {
            grouped[group_index].left_faces.push(pair.left_face);
        }
        if !grouped[group_index].right_faces.contains(&pair.right_face) {
            grouped[group_index].right_faces.push(pair.right_face);
        }
    }
    Some(grouped)
}

fn component_contained_adjacency_for_side(
    containing_side: MeshSide,
    containing_source: &ExactMesh,
    contained_source: &ExactMesh,
    containing_faces: &[usize],
    contained_faces: &[usize],
) -> Result<Option<ContainedFaceAdjacencyCertificate>, ExactMeshError> {
    if containing_faces.is_empty() || contained_faces.is_empty() {
        return Ok(None);
    }
    let Some(containing_mesh) = faces_mesh(
        containing_source,
        &containing_faces,
        "exact contained-face adjacency containing component",
    )?
    else {
        return Ok(None);
    };
    let Some(contained_mesh) = faces_mesh(
        contained_source,
        &contained_faces,
        "exact contained-face adjacency contained component",
    )?
    else {
        return Ok(None);
    };
    let Some(component_count) = connected_face_component_count(&containing_mesh) else {
        return Ok(None);
    };
    if component_count != 1 {
        return Ok(None);
    }
    let (_, arrangement_projection) =
        match materialize_contained_patch_difference(&containing_mesh, &contained_mesh) {
            Some(materialized) => materialized,
            None => return Ok(None),
        };
    let Some(sign) =
        consistent_projected_mesh_triangle_sign(&containing_mesh, arrangement_projection)?
    else {
        return Ok(None);
    };
    let Some(contained_sign) =
        consistent_projected_mesh_triangle_sign(&contained_mesh, arrangement_projection)?
    else {
        return Ok(None);
    };
    if contained_sign
        != match sign {
            Sign::Negative => Sign::Positive,
            Sign::Positive => Sign::Negative,
            Sign::Zero => return Ok(None),
        }
    {
        return Ok(None);
    }
    Ok(Some(ContainedFaceAdjacencyCertificate {
        containing_side,
        patches: vec![ContainedFacePatch {
            containing_faces: containing_faces.to_vec(),
            contained_faces: contained_faces.to_vec(),
            projection: arrangement_projection,
            containing_projected_sign: sign,
        }],
    }))
}

fn contained_adjacency_contact_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    pair: &FacePairEvents,
    certificate: &ContainedFaceAdjacencyCertificate,
) -> Result<bool, ExactMeshError> {
    let certificate_contains_pair = match certificate.containing_side {
        MeshSide::Left => certificate.patches.iter().any(|patch| {
            patch.containing_faces.contains(&pair.left_face)
                && patch.contained_faces.contains(&pair.right_face)
        }),
        MeshSide::Right => certificate.patches.iter().any(|patch| {
            patch.containing_faces.contains(&pair.right_face)
                && patch.contained_faces.contains(&pair.left_face)
        }),
    };
    if certificate_contains_pair {
        return Ok(pair.relation == MeshFacePairRelation::CoplanarOverlapping);
    }

    match pair.relation {
        MeshFacePairRelation::CoplanarTouching => Ok(true),
        MeshFacePairRelation::Candidate => {
            for event in &pair.events {
                let contact = match event {
                    IntersectionEvent::SegmentPlane { relation, .. } => {
                        matches!(
                            relation,
                            SegmentPlaneRelation::Disjoint
                                | SegmentPlaneRelation::Coplanar
                                | SegmentPlaneRelation::EndpointOnPlane
                        ) || (*relation == SegmentPlaneRelation::ProperCrossing
                            && retained_plane_crossing_is_not_inside_plane_face(
                                left, right, event,
                            )?)
                    }
                    IntersectionEvent::CoplanarEdge { relation, .. } => {
                        *relation != SegmentIntersection::Disjoint
                    }
                    IntersectionEvent::CoplanarVertex { location, .. } => matches!(
                        location,
                        TriangleLocation::Inside
                            | TriangleLocation::OnEdge
                            | TriangleLocation::OnVertex
                    ),
                    IntersectionEvent::Unknown => false,
                };
                if !contact {
                    return Ok(false);
                }
            }
            Ok(true)
        }
        MeshFacePairRelation::PlaneSeparated => Ok(true),
        MeshFacePairRelation::CoplanarOverlapping | MeshFacePairRelation::Unknown => Ok(false),
    }
}

fn retained_plane_crossing_is_not_inside_plane_face(
    left: &ExactMesh,
    right: &ExactMesh,
    event: &IntersectionEvent,
) -> Result<bool, ExactMeshError> {
    let IntersectionEvent::SegmentPlane {
        relation: SegmentPlaneRelation::ProperCrossing,
        plane_side,
        plane_face,
        point: Some(point),
        ..
    } = event
    else {
        return Ok(false);
    };
    // The graph may retain a source edge crossing the opposite face's
    // supporting plane even when the constructed point lies outside that
    // finite triangle, or exactly on its boundary. That construction is exact
    // evidence for splitting, not for volume overlap; preserving the
    // shortcut consumes. Strict interior crossings remain blockers.
    let mesh = plane_side.mesh(left, right);
    let triangle = face_point_refs(mesh, *plane_face)?;
    let Some(projection) = [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ]
    .into_iter()
    .find(|&projection| projected_triangle_sign(triangle, projection).is_some()) else {
        return Ok(false);
    };
    Ok(
        match classify_point_triangle(
            &project_point3(&triangle[0], projection),
            &project_point3(&triangle[1], projection),
            &project_point3(&triangle[2], projection),
            &project_point3(point, projection),
        )
        .value()
        {
            Some(location) => location != TriangleLocation::Inside,
            None => false,
        },
    )
}

fn contained_face_union_mesh(
    left: &ExactMesh,
    right: &ExactMesh,
    containing_side: MeshSide,
    patches: &[ContainedFacePatch],
    containing_faces: &[usize],
    contained_faces: &[usize],
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    let (left_skip_faces, right_skip_faces) = match containing_side {
        MeshSide::Left => (containing_faces, contained_faces),
        MeshSide::Right => (contained_faces, containing_faces),
    };
    if append_source_mesh_without_face(left, left_skip_faces, &mut vertices, &mut triangles)?
        .is_none()
    {
        return Ok(None);
    }
    if append_source_mesh_without_face(right, right_skip_faces, &mut vertices, &mut triangles)?
        .is_none()
    {
        return Ok(None);
    }
    for patch in patches {
        if append_contained_face_patch_group(
            left,
            right,
            containing_side,
            patch,
            &mut vertices,
            &mut triangles,
        )?
        .is_none()
        {
            return Ok(None);
        }
    }
    let mut seen = std::collections::BTreeSet::new();
    triangles.retain(|triangle| {
        let mut key = triangle.0;
        key.sort_unstable();
        seen.insert(key)
    });

    let mesh = ExactMesh::new_with_policy_and_version(
        vertices,
        triangles,
        SourceProvenance::exact("exact contained-face adjacent closed-solid union"),
        validation,
        1,
    )?;
    Ok(Some(mesh))
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
    patch: &ContainedFacePatch,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<Option<()>, ExactMeshError> {
    let Some(containing_mesh) = faces_mesh(
        containing_side.mesh(left, right),
        &patch.containing_faces,
        "exact contained-face adjacency containing faces",
    )?
    else {
        return Ok(None);
    };
    let contained_side = match containing_side {
        MeshSide::Left => MeshSide::Right,
        MeshSide::Right => MeshSide::Left,
    };
    let Some(contained_mesh) = faces_mesh(
        contained_side.mesh(left, right),
        &patch.contained_faces,
        "exact contained-face adjacency contained faces",
    )?
    else {
        return Ok(None);
    };
    let Some((replacement, _)) =
        materialize_contained_patch_difference(&containing_mesh, &contained_mesh)
    else {
        return Ok(None);
    };
    append_holed_replacement(
        &replacement,
        patch.projection,
        patch.containing_projected_sign,
        vertices,
        triangles,
    )
}

fn materialize_contained_patch_difference(
    containing_mesh: &ExactMesh,
    contained_mesh: &ExactMesh,
) -> Option<(ExactMesh, CoplanarProjection)> {
    let (_, projection) = coplanar_mesh_overlay_carrier(containing_mesh, contained_mesh)
        .ok()
        .flatten()?;
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

fn faces_mesh(
    mesh: &ExactMesh,
    faces: &[usize],
    label: &'static str,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for &face in faces {
        let points = face_point_refs(mesh, face)?;
        let mapped = map_triangle_points(&mut vertices, points)?;
        triangles.push(Triangle(mapped));
    }
    Ok(Some(ExactMesh::new_with_policy_and_version(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        1,
    )?))
}

fn connected_face_component_count(mesh: &ExactMesh) -> Option<usize> {
    let face_count = mesh.view().faces().count();
    if face_count == 0 {
        return None;
    }
    let mut edge_faces = BTreeMap::<[usize; 2], Vec<usize>>::new();
    for face in mesh.view().faces() {
        for edge in face.directed_edges() {
            edge_faces
                .entry(sorted_edge(edge))
                .or_default()
                .push(face.index());
        }
    }

    let mut components = DisjointSets {
        parent: (0..face_count).collect(),
    };
    for faces in edge_faces.values() {
        for left in 0..faces.len() {
            for right in left + 1..faces.len() {
                let left_face = faces[left];
                let right_face = faces[right];
                if left_face >= face_count || right_face >= face_count {
                    return None;
                }
                let left_root = components.find(left_face);
                let right_root = components.find(right_face);
                if left_root != right_root {
                    components.parent[right_root] = left_root;
                }
            }
        }
    }
    let mut roots = Vec::new();
    let mut component_count = 0;
    for face in 0..face_count {
        let root = components.find(face);
        if !roots.contains(&root) {
            roots.push(root);
            component_count += 1;
        }
    }
    Some(component_count)
}

fn append_source_mesh_without_face(
    mesh: &ExactMesh,
    skip_faces: &[usize],
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<Option<()>, ExactMeshError> {
    for face in mesh.view().faces() {
        if skip_faces.contains(&face.index()) {
            continue;
        }
        let points = face.vertices()?;
        let mapped = map_triangle_points(vertices, points)?;
        triangles.push(Triangle(mapped));
    }
    Ok(Some(()))
}

fn append_holed_replacement(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    target_sign: Sign,
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<Option<()>, ExactMeshError> {
    let Some(source_sign) = consistent_projected_mesh_triangle_sign(mesh, projection)? else {
        return Ok(None);
    };
    let flip = source_sign != target_sign;
    for face in mesh.view().faces() {
        let points = face.vertices()?;
        let mapped = map_triangle_points(vertices, points)?;
        let mapped_triangle = if flip {
            [mapped[0], mapped[2], mapped[1]]
        } else {
            mapped
        };
        if append_triangle_with_existing_edge_splits(mapped_triangle, vertices, triangles).is_none()
        {
            return Ok(None);
        }
    }
    Ok(Some(()))
}

fn append_triangle_with_existing_edge_splits(
    mapped_triangle: [usize; 3],
    vertices: &[Point3],
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    let mut split_vertices = Vec::new();
    for (candidate, point) in vertices.iter().enumerate() {
        if mapped_triangle.contains(&candidate) {
            continue;
        }
        let mut lies_on_triangle_edge = false;
        for edge in 0..3 {
            let start = vertices.get(mapped_triangle[edge])?;
            let end = vertices.get(mapped_triangle[(edge + 1) % 3])?;
            if point3_lies_strictly_on_segment(start, end, point)? {
                lies_on_triangle_edge = true;
                break;
            }
        }
        if lies_on_triangle_edge {
            split_vertices.push(candidate);
        }
    }
    if split_vertices.is_empty() {
        triangles.push(Triangle(mapped_triangle));
        return Some(());
    }

    let mut refined = vec![Triangle(mapped_triangle)];
    for split_vertex in split_vertices {
        split_output_triangle_edge(vertices, &mut refined, split_vertex)?;
    }
    triangles.extend(refined);
    Some(())
}

fn map_point(vertices: &mut Vec<Point3>, point: &Point3) -> Result<usize, ExactMeshError> {
    for (existing, candidate) in vertices.iter().enumerate() {
        match point3_exact_equal(candidate, point) {
            Some(true) => return Ok(existing),
            Some(false) => {}
            None => {
                return Err(ExactMeshError::one(ExactMeshBlocker::new(
                    ExactMeshBlockerKind::UndecidablePredicate,
                    "contained-face adjacent output vertex equality is undecidable",
                )));
            }
        }
    }
    let mapped = vertices.len();
    vertices.push(point.clone());
    Ok(mapped)
}

fn map_triangle_points(
    vertices: &mut Vec<Point3>,
    points: [&Point3; 3],
) -> Result<[usize; 3], ExactMeshError> {
    Ok([
        map_point(vertices, points[0])?,
        map_point(vertices, points[1])?,
        map_point(vertices, points[2])?,
    ])
}

fn face_point_refs(mesh: &ExactMesh, face: usize) -> Result<[&Point3; 3], ExactMeshError> {
    mesh.view().face(face)?.vertices()
}

fn projected_triangle_sign(points: [&Point3; 3], projection: CoplanarProjection) -> Option<Sign> {
    let a = project_point3(points[0], projection);
    let b = project_point3(points[1], projection);
    let c = project_point3(points[2], projection);
    match orient2d_report(&a, &b, &c).value()? {
        Sign::Negative => Some(Sign::Negative),
        Sign::Zero => None,
        Sign::Positive => Some(Sign::Positive),
    }
}

fn consistent_projected_mesh_triangle_sign(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
) -> Result<Option<Sign>, ExactMeshError> {
    let mut sign = None;
    for face in mesh.view().faces() {
        let points = face.vertices()?;
        let Some(face_sign) = projected_triangle_sign(points, projection) else {
            return Ok(None);
        };
        match sign {
            Some(expected) if expected != face_sign => return Ok(None),
            Some(_) => {}
            None => sign = Some(face_sign),
        }
    }
    Ok(sign)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contained_face_component_rejects_stale_retained_face_rows() {
        let mut mesh = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 2, 0, 0, 0, 2, 0],
            &[0, 1, 2],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        mesh.facts.faces.pop();

        let error = faces_mesh(
            &mesh,
            &[0],
            "test contained-face component with stale retained face row",
        )
        .expect_err("stale retained face rows should return a typed blocker");

        assert!(
            error.has_only_blocker_kinds(&[ExactMeshBlockerKind::StaleFactReplay]),
            "{error:?}"
        );
        assert_eq!(error.blockers()[0].face(), Some(0));
    }
}
