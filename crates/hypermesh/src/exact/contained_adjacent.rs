//! Exact contained-face adjacency materialization for closed solids.
//!
//! Full-face adjacency can delete matching source faces directly. This module
//! handles the next bounded coplanar-volumetric case: one closed solid has a
//! triangular boundary face that strictly contains an opposite-oriented
//! triangular boundary face of the other solid. The regularized union removes
//! the contained face and replaces the containing face with an exact one-hole
//! triangulation, welding the hole to the contained solid's side faces.
//!
//! The shortcut is intentionally narrow. It replays exact boundary-only
//! winding evidence, retained graph events, strict point-in-triangle facts, and
//! the holed face triangulation before it can authorize a closed output mesh.
//! This is the same object/predicate boundary advocated by Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): a missing
//! general cell materializer becomes explicit state unless a retained exact
//! certificate owns this bounded topology change.
//!
//! The holed planar face uses the same earcut-compatible ring triangulation
//! surface as [`crate::exact::surface`]. See Held, "FIST: Fast
//! Industrial-Strength Triangulation of Polygons," *Algorithmica* 30 (2001),
//! for the ring triangulation model; exact Hyper predicates supply the
//! topology guards here instead of tolerance tests.

use hyperlimit::{
    CoplanarProjection, Point3, SegmentIntersection, Sign, TriangleLocation,
    classify_point_triangle, compare_reals, orient3d_report, project_point3,
    projected_polygon_area2_value,
};

use super::construction::SegmentPlaneRelation;
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{FacePairEvents, IntersectionEvent, MeshSide, build_intersection_graph};
use super::intersection::MeshFacePairRelation;
use super::mesh::{ExactMesh, ExactMeshValidationError, ExactPoint3, Triangle};
use super::provenance::SourceProvenance;
use super::scalar::ExactReal;
use super::surface::arrange_single_triangle_coplanar_holed_difference;
use super::validation::ValidationPolicy;
use super::winding::{
    ClosedMeshWindingRelation, classify_mesh_vertices_against_closed_mesh_winding_report,
};

use std::cmp::Ordering;

/// Exact materialization of a closed-solid union across one contained face.
#[derive(Clone, Debug, PartialEq)]
pub struct ContainedFaceAdjacentUnion {
    /// Source side whose face is replaced by a holed face remnant.
    pub containing_side: MeshSide,
    /// Face index on [`Self::containing_side`] that strictly contains the
    /// opposite face.
    pub containing_face: usize,
    /// Face index on the opposite source mesh that is removed from the
    /// regularized union.
    pub contained_face: usize,
    /// Closed output mesh after replacing the containing face and deleting the
    /// contained face.
    pub mesh: ExactMesh,
}

/// Validation failure for a retained contained-face adjacency materialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContainedFaceAdjacentUnionError {
    /// The retained output mesh no longer validates as an exact mesh.
    OutputMesh(ExactMeshValidationError),
    /// The retained output mesh is locally valid but is not a closed manifold.
    OutputNotClosed,
    /// Recomputing the materialization from source meshes did not reproduce it.
    SourceReplayMismatch,
}

impl ContainedFaceAdjacentUnion {
    /// Validate the retained output mesh without consulting source operands.
    ///
    /// Source face indices are replayed separately by
    /// [`Self::validate_against_sources`]. Keeping local output validation and
    /// source replay as separate checks follows Yap's retained-state model:
    /// copied topology must first be a coherent exact object, then it must be
    /// proven to still come from the named sources.
    pub fn validate(&self) -> Result<(), ContainedFaceAdjacentUnionError> {
        self.mesh
            .validate_retained_state()
            .map_err(ContainedFaceAdjacentUnionError::OutputMesh)?;
        if !self.mesh.facts().mesh.closed_manifold {
            return Err(ContainedFaceAdjacentUnionError::OutputNotClosed);
        }
        Ok(())
    }

    /// Validate this retained union by replaying the exact source certificate.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ContainedFaceAdjacentUnionError> {
        self.validate()?;
        let Some(replay) =
            materialize_contained_face_adjacent_union(left, right, self.mesh.validation_policy())
        else {
            return Err(ContainedFaceAdjacentUnionError::SourceReplayMismatch);
        };
        if self == &replay {
            Ok(())
        } else {
            Err(ContainedFaceAdjacentUnionError::SourceReplayMismatch)
        }
    }
}

/// Return whether the sources can be unioned by contained-face adjacency.
pub fn has_contained_face_adjacent_union(left: &ExactMesh, right: &ExactMesh) -> bool {
    materialize_contained_face_adjacent_union(left, right, ValidationPolicy::CLOSED).is_some()
}

/// Materialize a regularized union across one strictly contained shared face.
pub fn materialize_contained_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Option<ContainedFaceAdjacentUnion> {
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

    let certificate = contained_face_adjacency_certificate(left, right, &graph.face_pairs)?;
    let mesh = contained_face_union_mesh(left, right, &certificate, validation)?;
    let union = ContainedFaceAdjacentUnion {
        containing_side: certificate.containing_side,
        containing_face: certificate.containing_face,
        contained_face: certificate.contained_face,
        mesh,
    };
    union.validate().ok()?;
    Some(union)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ContainedFaceAdjacencyCertificate {
    containing_side: MeshSide,
    containing_face: usize,
    contained_face: usize,
    projection: CoplanarProjection,
    containing_projected_sign: Sign,
}

fn contained_face_adjacency_certificate(
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
        if certificate.replace(candidate).is_some() {
            return None;
        }
    }
    let certificate = certificate?;
    if pairs
        .iter()
        .all(|pair| contained_adjacency_contact_pair(left, right, pair, &certificate))
    {
        Some(certificate)
    } else {
        None
    }
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
            containing_face: pair.left_face,
            contained_face: pair.right_face,
            projection,
            containing_projected_sign: sign,
        });
    }
    if let Some((projection, sign)) =
        face_strictly_contains_opposite_face(right, pair.right_face, left, pair.left_face)?
    {
        return Some(ContainedFaceAdjacencyCertificate {
            containing_side: MeshSide::Right,
            containing_face: pair.right_face,
            contained_face: pair.left_face,
            projection,
            containing_projected_sign: sign,
        });
    }
    None
}

fn contained_adjacency_contact_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    pair: &FacePairEvents,
    certificate: &ContainedFaceAdjacencyCertificate,
) -> bool {
    if pair.left_face == left_face_for_certificate(certificate)
        && pair.right_face == right_face_for_certificate(certificate)
    {
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

const fn left_face_for_certificate(certificate: &ContainedFaceAdjacencyCertificate) -> usize {
    match certificate.containing_side {
        MeshSide::Left => certificate.containing_face,
        MeshSide::Right => certificate.contained_face,
    }
}

const fn right_face_for_certificate(certificate: &ContainedFaceAdjacencyCertificate) -> usize {
    match certificate.containing_side {
        MeshSide::Left => certificate.contained_face,
        MeshSide::Right => certificate.containing_face,
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
                && retained_plane_crossing_is_outside_plane_face(left, right, event))
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

fn retained_plane_crossing_is_outside_plane_face(
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
    // finite triangle. That construction is exact evidence for splitting,
    // not for volume overlap; preserving the distinction is the Yap-style
    // predicate/object boundary this bounded shortcut consumes.
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
        == Some(TriangleLocation::Outside)
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
    let containing_mesh = face_mesh(
        mesh_for_side(certificate.containing_side, left, right),
        certificate.containing_face,
        "exact contained-face adjacency containing face",
    )?;
    let contained_mesh = face_mesh(
        mesh_for_side(opposite_side(certificate.containing_side), left, right),
        certificate.contained_face,
        "exact contained-face adjacency contained face",
    )?;
    let holed =
        arrange_single_triangle_coplanar_holed_difference(&containing_mesh, &contained_mesh)?;

    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    append_source_mesh_without_face(
        left,
        skip_face_for_side(certificate, MeshSide::Left),
        &mut vertices,
        &mut triangles,
    )?;
    append_source_mesh_without_face(
        right,
        skip_face_for_side(certificate, MeshSide::Right),
        &mut vertices,
        &mut triangles,
    )?;
    append_holed_replacement(
        &holed.mesh,
        certificate.projection,
        certificate.containing_projected_sign,
        &mut vertices,
        &mut triangles,
    )?;

    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact contained-face adjacent closed-solid union"),
        validation,
    )
    .ok()
}

fn face_mesh(mesh: &ExactMesh, face: usize, label: &'static str) -> Option<ExactMesh> {
    let triangle = mesh.triangles().get(face)?.0;
    ExactMesh::new_with_policy(
        triangle
            .iter()
            .map(|&vertex| mesh.vertices().get(vertex).cloned())
            .collect::<Option<Vec<_>>>()?,
        vec![Triangle([0, 1, 2])],
        SourceProvenance::exact(label),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn append_source_mesh_without_face(
    mesh: &ExactMesh,
    skip_face: Option<usize>,
    vertices: &mut Vec<ExactPoint3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    for (face, triangle) in mesh.triangles().iter().enumerate() {
        if skip_face == Some(face) {
            continue;
        }
        triangles.push(Triangle([
            map_point(
                vertices,
                &mesh.vertices().get(triangle.0[0])?.to_hyperlimit_point(),
            )?,
            map_point(
                vertices,
                &mesh.vertices().get(triangle.0[1])?.to_hyperlimit_point(),
            )?,
            map_point(
                vertices,
                &mesh.vertices().get(triangle.0[2])?.to_hyperlimit_point(),
            )?,
        ]));
    }
    Some(())
}

fn append_holed_replacement(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    target_sign: Sign,
    vertices: &mut Vec<ExactPoint3>,
    triangles: &mut Vec<Triangle>,
) -> Option<()> {
    let source_sign = first_projected_mesh_triangle_sign(mesh, projection)?;
    let flip = source_sign != target_sign;
    for triangle in mesh.triangles() {
        let mapped = [
            map_point(
                vertices,
                &mesh.vertices().get(triangle.0[0])?.to_hyperlimit_point(),
            )?,
            map_point(
                vertices,
                &mesh.vertices().get(triangle.0[1])?.to_hyperlimit_point(),
            )?,
            map_point(
                vertices,
                &mesh.vertices().get(triangle.0[2])?.to_hyperlimit_point(),
            )?,
        ];
        triangles.push(if flip {
            Triangle([mapped[0], mapped[2], mapped[1]])
        } else {
            Triangle(mapped)
        });
    }
    Some(())
}

fn map_point(vertices: &mut Vec<ExactPoint3>, point: &Point3) -> Option<usize> {
    if let Some(existing) = vertices
        .iter()
        .position(|candidate| points_equal(&candidate.to_hyperlimit_point(), point) == Some(true))
    {
        return Some(existing);
    }
    let mapped = vertices.len();
    vertices.push(point_to_exact(point));
    Some(mapped)
}

fn point_to_exact(point: &Point3) -> ExactPoint3 {
    ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone())
}

const fn skip_face_for_side(
    certificate: &ContainedFaceAdjacencyCertificate,
    side: MeshSide,
) -> Option<usize> {
    match (certificate.containing_side, side) {
        (MeshSide::Left, MeshSide::Left) | (MeshSide::Right, MeshSide::Right) => {
            Some(certificate.containing_face)
        }
        (MeshSide::Left, MeshSide::Right) | (MeshSide::Right, MeshSide::Left) => {
            Some(certificate.contained_face)
        }
    }
}

const fn opposite_side(side: MeshSide) -> MeshSide {
    match side {
        MeshSide::Left => MeshSide::Right,
        MeshSide::Right => MeshSide::Left,
    }
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
        mesh.vertices().get(triangle[0])?.to_hyperlimit_point(),
        mesh.vertices().get(triangle[1])?.to_hyperlimit_point(),
        mesh.vertices().get(triangle[2])?.to_hyperlimit_point(),
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
            mesh.vertices().get(triangle.0[0])?.to_hyperlimit_point(),
            mesh.vertices().get(triangle.0[1])?.to_hyperlimit_point(),
            mesh.vertices().get(triangle.0[2])?.to_hyperlimit_point(),
        ];
        projected_triangle_sign(&points, projection)
    })
}

fn real_sign(value: &ExactReal) -> Option<Sign> {
    match compare_reals(value, &ExactReal::from(0)).value()? {
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

fn contained_adjacency_error(error: ContainedFaceAdjacentUnionError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact contained-face adjacent union/source replay failed: {error:?}"),
    ))
}

pub(crate) fn replay_contained_face_adjacent_union(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ContainedFaceAdjacentUnion, MeshError> {
    let union =
        materialize_contained_face_adjacent_union(left, right, validation).ok_or_else(|| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                "exact contained-face adjacent union certificate did not replay",
            ))
        })?;
    union
        .validate_against_sources(left, right)
        .map_err(contained_adjacency_error)?;
    Ok(union)
}
