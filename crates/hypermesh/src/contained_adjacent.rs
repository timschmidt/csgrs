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
    classify_point_triangle, compare_reals, orient3d_report, project_point3,
    projected_polygon_area2_value,
};

use super::arrangement2d::{ExactArrangement2dBoundaryPolicy, ExactArrangement2dSetOperation};
use super::boolean::{coplanar_mesh_overlay_carrier, materialize_coplanar_mesh_overlay_mesh};
use super::graph::{FacePairEvents, IntersectionEvent, MeshSide, build_intersection_graph};
use super::intersection::MeshFacePairRelation;
use super::mesh::{ExactMesh, ExactMeshValidationError, Triangle};
use super::topology::{mesh_for_side, triangle_tuple_edges};
use super::validation::ValidationPolicy;
use super::winding::{
    ClosedMeshWindingRelation, classify_mesh_vertices_against_closed_mesh_winding_report,
};
use hyperlimit::SourceProvenance;
use hyperreal::Real;

use std::cmp::Ordering;

/// Exact materialization of a closed-solid union across contained boundary caps.
#[derive(Clone, Debug, PartialEq)]
pub struct ContainedFaceAdjacentUnion {
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
pub enum ContainedFaceAdjacentUnionError {
    /// The retained output mesh no longer validates as an exact mesh.
    OutputMesh(ExactMeshValidationError),
    /// The retained output mesh is locally valid but is not a closed manifold.
    OutputNotClosed,
    /// Replaying the retained source operands did not reproduce this union.
    SourceReplayMismatch,
}

/// Freshness status for a retained contained-face adjacent union.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContainedFaceAdjacentUnionFreshness {
    /// The retained union locally validates and replays from source operands.
    Current,
    /// The retained output mesh no longer passes local exact output validation.
    InvalidOutput,
    /// The artifact is locally valid but no longer replays from source operands.
    SourceReplayMismatch,
}

impl ContainedFaceAdjacentUnion {
    /// Validate the retained output mesh without consulting source operands.
    ///
    /// Local output validation and copied topology must first be a coherent
    /// exact object before boolean code consumes the retained certificate.
    pub fn validate(&self) -> Result<(), ContainedFaceAdjacentUnionError> {
        self.mesh
            .validate_retained_state()
            .map_err(ContainedFaceAdjacentUnionError::OutputMesh)?;
        if !self.mesh.facts().mesh.closed_manifold {
            return Err(ContainedFaceAdjacentUnionError::OutputNotClosed);
        }
        Ok(())
    }

    /// Validate this union by replaying the contained-face adjacency certificate
    /// and materialization from the source operands.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ContainedFaceAdjacentUnionError> {
        self.validate()?;
        let replay =
            materialize_contained_face_adjacent_union(left, right, self.mesh.validation_policy())
                .ok_or(ContainedFaceAdjacentUnionError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ContainedFaceAdjacentUnionError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained union is fresh for the source operands.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ContainedFaceAdjacentUnionFreshness {
        if self.validate().is_err() {
            return ContainedFaceAdjacentUnionFreshness::InvalidOutput;
        }
        match materialize_contained_face_adjacent_union(left, right, self.mesh.validation_policy())
        {
            Some(replay) if replay == *self => ContainedFaceAdjacentUnionFreshness::Current,
            Some(_) | None => ContainedFaceAdjacentUnionFreshness::SourceReplayMismatch,
        }
    }
}

/// Certify and materialize a contained-face adjacent closed-solid union.
pub fn materialize_contained_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Option<ContainedFaceAdjacentUnion> {
    let certificate = contained_face_adjacent_certificate(left, right)?;
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
) -> Option<ContainedFaceAdjacentCertificate> {
    contained_face_adjacent_union_certificate(left, right)
        .map(|inner| ContainedFaceAdjacentCertificate { inner })
}

/// Materialize a contained-face adjacent union from an already-retained certificate.
pub(crate) fn materialize_contained_face_adjacent_union_from_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
    certificate: &ContainedFaceAdjacentCertificate,
    validation: ValidationPolicy,
) -> Option<ContainedFaceAdjacentUnion> {
    let certificate = &certificate.inner;
    let mesh = contained_face_union_mesh(left, right, certificate, validation)?;
    let union = ContainedFaceAdjacentUnion {
        containing_side: certificate.containing_side,
        containing_face: certificate.containing_face(),
        contained_face: certificate.contained_faces()[0],
        contained_faces: certificate.contained_faces(),
        containing_faces: certificate.containing_faces(),
        mesh,
    };
    union.validate().ok()?;
    Some(union)
}

fn contained_face_adjacent_union_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ContainedFaceAdjacencyCertificate> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return None;
    }
    let graph = build_intersection_graph(left, right).ok()?;
    graph.validate_against_meshes(left, right).ok()?;
    if graph.has_unknowns() || graph.face_pairs.is_empty() {
        return None;
    }
    if !closed_boundary_contact_only(left, right)? {
        return None;
    }

    contained_face_adjacency_certificate(left, right, &graph.face_pairs)
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
    [
        single_face_contained_adjacency_certificate(left, right, pairs),
        component_contained_adjacency_certificate(left, right, pairs),
    ]
    .into_iter()
    .flatten()
    .find(|certificate| {
        pairs
            .iter()
            .all(|pair| contained_adjacency_contact_pair(left, right, pair, certificate))
    })
}

fn single_face_contained_adjacency_certificate(
    left: &ExactMesh,
    right: &ExactMesh,
    pairs: &[FacePairEvents],
) -> Option<ContainedFaceAdjacencyCertificate> {
    let mut certificate = None;
    for pair in pairs {
        if pair.relation != MeshFacePairRelation::CoplanarOverlapping {
            continue;
        }
        let candidate = contained_face_pair(left, right, pair)?;
        match &mut certificate {
            Some(existing) => merge_contained_face_candidate(existing, candidate)?,
            None => certificate = Some(candidate),
        }
    }
    certificate
}

fn contained_face_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    pair: &FacePairEvents,
) -> Option<ContainedFaceAdjacencyCertificate> {
    if let Some((projection, sign)) =
        face_strictly_contains_opposite_face(left, pair.left_face, right, pair.right_face)?
    {
        return Some(ContainedFaceAdjacencyCertificate {
            containing_side: MeshSide::Left,
            patches: vec![ContainedFacePatch {
                containing_faces: vec![pair.left_face],
                contained_faces: vec![pair.right_face],
                projection,
                containing_projected_sign: sign,
            }],
        });
    }
    if let Some((projection, sign)) =
        face_strictly_contains_opposite_face(right, pair.right_face, left, pair.left_face)?
    {
        return Some(ContainedFaceAdjacencyCertificate {
            containing_side: MeshSide::Right,
            patches: vec![ContainedFacePatch {
                containing_faces: vec![pair.right_face],
                contained_faces: vec![pair.left_face],
                projection,
                containing_projected_sign: sign,
            }],
        });
    }
    None
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
    if overlapping.len() < 2 {
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

/// Merge one contained-face candidate into a bounded multi-patch certificate.
///
/// The accepted topology replaces several independent source faces with
/// one-hole remnants, or several contained caps on the same source face, with
/// source-owned caps whose projection and orientation replay exactly. Arbitrary
/// branch graphs still remain explicit planar/coplanar-volumetric blockers.
fn merge_contained_face_candidate(
    existing: &mut ContainedFaceAdjacencyCertificate,
    candidate: ContainedFaceAdjacencyCertificate,
) -> Option<()> {
    if existing.containing_side != candidate.containing_side {
        return None;
    }
    for mut patch in candidate.patches {
        patch.containing_faces.sort_unstable();
        patch.contained_faces.sort_unstable();
        if existing
            .patches
            .iter()
            .any(|existing| faces_overlap(&existing.contained_faces, &patch.contained_faces))
        {
            return None;
        }
        if existing.patches.iter().any(|existing| {
            existing.containing_faces == patch.containing_faces
                && (existing.projection != patch.projection
                    || existing.containing_projected_sign != patch.containing_projected_sign)
        }) {
            return None;
        }
        if let Some(existing) = existing.patches.iter_mut().find(|existing| {
            existing.containing_faces == patch.containing_faces
                && existing.projection == patch.projection
                && existing.containing_projected_sign == patch.containing_projected_sign
        }) {
            existing.contained_faces.extend(patch.contained_faces);
            existing.contained_faces.sort_unstable();
            existing.contained_faces.dedup();
        } else {
            existing.patches.push(patch);
        }
    }
    Some(())
}

fn faces_overlap(left: &[usize], right: &[usize]) -> bool {
    left.iter().any(|face| right.contains(face))
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
        MeshFacePairRelation::BoundsDisjoint | MeshFacePairRelation::PlaneSeparated => true,
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

fn face_strictly_contains_opposite_face(
    containing_mesh: &ExactMesh,
    containing_face: usize,
    contained_mesh: &ExactMesh,
    contained_face: usize,
) -> Option<Option<(CoplanarProjection, Sign)>> {
    let containing = triangle_points(containing_mesh, containing_face)?;
    let contained = triangle_points(contained_mesh, contained_face)?;
    if !contained
        .iter()
        .all(|point| point_on_triangle_plane(&containing, point) == Some(true))
    {
        return Some(None);
    }
    let projection = choose_triangle_projection(&containing)?;
    let containing_sign = projected_triangle_sign(&containing, projection)?;
    let contained_sign = projected_triangle_sign(&contained, projection)?;
    if containing_sign == contained_sign {
        return Some(None);
    }

    let a = project_point3(&containing[0], projection);
    let b = project_point3(&containing[1], projection);
    let c = project_point3(&containing[2], projection);
    if !contained.iter().all(|point| {
        classify_point_triangle(&a, &b, &c, &project_point3(point, projection)).value()
            == Some(TriangleLocation::Inside)
    }) {
        return Some(None);
    }
    Some(Some((projection, containing_sign)))
}

fn contained_face_union_mesh(
    left: &ExactMesh,
    right: &ExactMesh,
    certificate: &ContainedFaceAdjacencyCertificate,
    validation: ValidationPolicy,
) -> Option<ExactMesh> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    append_source_mesh_without_face(
        left,
        skip_faces_for_side(certificate, MeshSide::Left),
        &mut vertices,
        &mut triangles,
    )?;
    append_source_mesh_without_face(
        right,
        skip_faces_for_side(certificate, MeshSide::Right),
        &mut vertices,
        &mut triangles,
    )?;
    for group in contained_face_patch_groups(certificate) {
        append_contained_face_patch_group(
            left,
            right,
            certificate.containing_side,
            &group,
            &mut vertices,
            &mut triangles,
        )?;
    }

    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact contained-face adjacent closed-solid union"),
        validation,
    )
    .ok()
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
        ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
    ] {
        if let Some(mesh) = materialize_coplanar_mesh_overlay_mesh(
            containing_mesh,
            contained_mesh,
            ExactArrangement2dSetOperation::Difference,
            boundary_policy,
            "exact contained-adjacent arrangement patch difference",
            false,
        ) {
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
        ValidationPolicy::ALLOW_BOUNDARY,
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
        let mapped = [
            map_point(vertices, &mesh.vertices().get(triangle.0[0])?.clone())?,
            map_point(vertices, &mesh.vertices().get(triangle.0[1])?.clone())?,
            map_point(vertices, &mesh.vertices().get(triangle.0[2])?.clone())?,
        ];
        triangles.push(if flip {
            Triangle([mapped[0], mapped[2], mapped[1]])
        } else {
            Triangle(mapped)
        });
    }
    Some(())
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
