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

use super::super::arrangement3d::arrangement2d::{
    ExactArrangement2dBoundaryPolicy, ExactArrangement2dSetOperation,
};
use super::super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::super::graph::intersection::MeshFacePairRelation;
use super::super::graph::{
    ExactIntersectionGraph, FacePairEvents, IntersectionEvent, MeshSide,
    build_validated_intersection_graph,
};
use super::super::validation::ExactMeshValidationPolicy;
use super::super::{ExactMesh, ExactMeshValidationError, Triangle, triangle_edges_tuple};
use super::winding::classify_mesh_vertices_against_closed_mesh_winding_report;
use super::{coplanar_mesh_overlay_carrier, materialize_coplanar_mesh_overlay_mesh};
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

fn exact_construction_failure(message: impl Into<String>) -> ExactMeshError {
    ExactMeshError::one(ExactMeshBlocker::new(
        ExactMeshBlockerKind::ExactConstructionFailure,
        message,
    ))
}

/// Return the retained contained-face adjacency certificate for these sources.
pub(crate) fn contained_face_adjacent_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ContainedFaceAdjacentCertificate>, ExactMeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    let graph = build_validated_intersection_graph(left, right)?;
    contained_face_adjacent_certificate_from_graph(left, right, &graph)
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
        contained_face_adjacency_certificate(left, right, &graph.face_pairs)
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
        exact_construction_failure(format!(
            "contained-face adjacent union retained output failed validation: {error:?}"
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
        if !grouped[group_index].left_faces.contains(&pair.left_face) {
            grouped[group_index].left_faces.push(pair.left_face);
        }
        if !grouped[group_index].right_faces.contains(&pair.right_face) {
            grouped[group_index].right_faces.push(pair.right_face);
        }
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
            match sign {
                Sign::Negative => Sign::Positive,
                Sign::Positive => Sign::Negative,
                Sign::Zero => Sign::Zero,
            },
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

fn contained_adjacency_contact_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    pair: &FacePairEvents,
    certificate: &ContainedFaceAdjacencyCertificate,
) -> bool {
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
    let Some(triangle) = triangle_points(plane_side.mesh(left, right), *plane_face) else {
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
    if append_source_mesh_without_face(left, left_skip_faces, &mut vertices, &mut triangles)
        .is_none()
    {
        return Ok(None);
    }
    if append_source_mesh_without_face(right, right_skip_faces, &mut vertices, &mut triangles)
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
        )
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

    let mesh = ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact contained-face adjacent closed-solid union"),
        validation,
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
) -> Option<()> {
    let containing_mesh = faces_mesh(
        containing_side.mesh(left, right),
        &patch.containing_faces,
        "exact contained-face adjacency containing faces",
    )?;
    let contained_side = match containing_side {
        MeshSide::Left => MeshSide::Right,
        MeshSide::Right => MeshSide::Left,
    };
    let contained_mesh = faces_mesh(
        contained_side.mesh(left, right),
        &patch.contained_faces,
        "exact contained-face adjacency contained faces",
    )?;
    let (replacement, _) =
        materialize_contained_patch_difference(&containing_mesh, &contained_mesh)?;
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
            let face_edges = triangle_edges_tuple(mesh.triangles()[face].0);
            for (neighbor, neighbor_visited) in visited.iter_mut().enumerate() {
                if *neighbor_visited {
                    continue;
                }
                let neighbor_edges = triangle_edges_tuple(mesh.triangles()[neighbor].0);
                let mut shares_edge = false;
                for left_edge in &face_edges {
                    for right_edge in &neighbor_edges {
                        if left_edge == right_edge {
                            shares_edge = true;
                            break;
                        }
                    }
                    if shares_edge {
                        break;
                    }
                }
                if shares_edge {
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

fn append_source_mesh_without_face(
    mesh: &ExactMesh,
    skip_faces: &[usize],
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
    vertices.push(point.clone());
    Some(mapped)
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
    Ok(left_in_right.vertices_are_boundary_or_outside()
        && right_in_left.vertices_are_boundary_or_outside()
        && (left_in_right.vertices_touch_boundary() || right_in_left.vertices_touch_boundary()))
}
