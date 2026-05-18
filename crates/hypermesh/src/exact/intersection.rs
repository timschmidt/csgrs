//! Exact mesh intersection scheduling.
//!
//! This module joins the retained exact AABB broad phase with the certified
//! triangle/triangle coarse classifier.  It is still a scheduler and event
//! collector, not the final boolean graph builder: `BoundsDisjoint` and
//! `PlaneSeparated` may reject work, while coplanar and candidate outcomes must
//! continue into exact overlap-graph construction. Retained exact face-plane
//! coefficients are used as a cached plane-separation filter before the full
//! triangle classifier is rebuilt, and candidate split events reuse those
//! retained planes for segment/plane construction. That boundary follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): acceleration facts can remove impossible events, but topological
//! mutations wait for certified predicates and exact constructions.

use hyperlimit::PredicateOutcome;

use super::bounds::AabbIntersectionKind;
use super::construction::{SegmentPlaneIntersection, intersect_segment_with_retained_face_plane};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::ExactMesh;
use super::narrow::{
    TrianglePlaneRelation, TriangleTriangleClassification, TriangleTriangleRelation,
    classify_mesh_triangle_against_retained_face_plane, classify_triangle_triangle,
};

/// Coarse exact relation for one pair of mesh faces.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshFacePairRelation {
    /// Exact AABBs prove the faces cannot intersect.
    BoundsDisjoint,
    /// Exact triangle plane-side predicates prove the faces are separated.
    PlaneSeparated,
    /// The triangles are coplanar and touch at a vertex or edge.
    CoplanarTouching,
    /// The triangles are coplanar and overlap with positive area or a
    /// positive-length edge interval.
    CoplanarOverlapping,
    /// The triangles are non-coplanar candidates with retained split events.
    Candidate,
    /// A required exact predicate was undecided.
    Unknown,
}

/// Exact broad/narrow classification for one pair of mesh faces.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshFacePairClassification {
    /// Face index in the left mesh.
    pub left_face: usize,
    /// Face index in the right mesh.
    pub right_face: usize,
    /// Exact AABB relation, or unknown when bounds could not be certified.
    pub bounds: PredicateOutcome<AabbIntersectionKind>,
    /// Triangle classifier result when bounds did not reject the pair.
    pub triangle: Option<TriangleTriangleClassification>,
    /// Coarse scheduling relation.
    pub relation: MeshFacePairRelation,
}

impl MeshFacePairClassification {
    /// Return whether the pair must continue to exact graph construction.
    pub const fn needs_graph_construction(&self) -> bool {
        matches!(
            self.relation,
            MeshFacePairRelation::Candidate
                | MeshFacePairRelation::CoplanarTouching
                | MeshFacePairRelation::CoplanarOverlapping
                | MeshFacePairRelation::Unknown
        )
    }
}

/// Classify one face pair from two exact meshes.
pub fn classify_mesh_face_pair(
    left: &ExactMesh,
    left_face: usize,
    right: &ExactMesh,
    right_face: usize,
) -> Result<MeshFacePairClassification, MeshError> {
    validate_face(left, left_face, "left")?;
    validate_face(right, right_face, "right")?;

    let bounds =
        left.bounds().faces[left_face].classify_intersection(&right.bounds().faces[right_face]);
    if matches!(
        bounds,
        PredicateOutcome::Decided {
            value: AabbIntersectionKind::Disjoint,
            ..
        }
    ) {
        return Ok(MeshFacePairClassification {
            left_face,
            right_face,
            bounds,
            triangle: None,
            relation: MeshFacePairRelation::BoundsDisjoint,
        });
    }

    let right_against_left =
        classify_mesh_triangle_against_retained_face_plane(left, left_face, right, right_face)?;
    if triangle_is_strictly_one_sided(right_against_left.relation) {
        return Ok(MeshFacePairClassification {
            left_face,
            right_face,
            bounds,
            triangle: None,
            relation: MeshFacePairRelation::PlaneSeparated,
        });
    }

    let left_against_right =
        classify_mesh_triangle_against_retained_face_plane(right, right_face, left, left_face)?;
    if triangle_is_strictly_one_sided(left_against_right.relation) {
        return Ok(MeshFacePairClassification {
            left_face,
            right_face,
            bounds,
            triangle: None,
            relation: MeshFacePairRelation::PlaneSeparated,
        });
    }

    let mut points = left
        .vertices()
        .iter()
        .map(|point| point.to_hyperlimit_point())
        .collect::<Vec<_>>();
    let right_offset = points.len();
    points.extend(
        right
            .vertices()
            .iter()
            .map(|point| point.to_hyperlimit_point()),
    );

    let left_tri = left.triangles()[left_face].0;
    let mut right_tri = right.triangles()[right_face].0;
    right_tri
        .iter_mut()
        .for_each(|vertex| *vertex += right_offset);
    let mut triangle = classify_triangle_triangle(&points, left_tri, right_tri);
    if triangle.relation == TriangleTriangleRelation::Candidate {
        triangle.right_edge_events =
            retained_triangle_edge_events(left, left_face, right, right_face);
        triangle.left_edge_events =
            retained_triangle_edge_events(right, right_face, left, left_face);
    }
    let relation = mesh_relation_from_triangle(triangle.relation);

    Ok(MeshFacePairClassification {
        left_face,
        right_face,
        bounds,
        triangle: Some(triangle),
        relation,
    })
}

/// Classify every face pair that survives exact broad/narrow rejection.
///
/// The returned list excludes pairs proven impossible by exact AABB
/// disjointness, certified triangle plane separation, or exact coplanar
/// disjointness. Coplanar touching/overlapping, non-coplanar candidate, and
/// unknown pairs remain because they are exactly the cases that need
/// overlap-graph construction or a policy decision.
pub fn classify_mesh_face_pairs(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Vec<MeshFacePairClassification>, MeshError> {
    let mut retained = Vec::new();
    for left_face in 0..left.triangles().len() {
        for right_face in 0..right.triangles().len() {
            let classification = classify_mesh_face_pair(left, left_face, right, right_face)?;
            if classification.needs_graph_construction() {
                retained.push(classification);
            }
        }
    }
    Ok(retained)
}

fn triangle_is_strictly_one_sided(relation: TrianglePlaneRelation) -> bool {
    matches!(
        relation,
        TrianglePlaneRelation::StrictlyAbove | TrianglePlaneRelation::StrictlyBelow
    )
}

fn retained_triangle_edge_events(
    plane_mesh: &ExactMesh,
    plane_face: usize,
    segment_mesh: &ExactMesh,
    segment_face: usize,
) -> Vec<SegmentPlaneIntersection> {
    let plane = &plane_mesh.facts().faces[plane_face].plane;
    triangle_edges(segment_mesh.triangles()[segment_face].0)
        .into_iter()
        .map(|edge| {
            let p0 = segment_mesh.vertices()[edge[0]].to_hyperlimit_point();
            let p1 = segment_mesh.vertices()[edge[1]].to_hyperlimit_point();
            intersect_segment_with_retained_face_plane(plane, &p0, &p1)
        })
        .collect()
}

fn triangle_edges(triangle: [usize; 3]) -> [[usize; 2]; 3] {
    [
        [triangle[0], triangle[1]],
        [triangle[1], triangle[2]],
        [triangle[2], triangle[0]],
    ]
}

fn mesh_relation_from_triangle(relation: TriangleTriangleRelation) -> MeshFacePairRelation {
    match relation {
        TriangleTriangleRelation::SeparatedByFirstPlane
        | TriangleTriangleRelation::SeparatedBySecondPlane => MeshFacePairRelation::PlaneSeparated,
        TriangleTriangleRelation::CoplanarDisjoint => MeshFacePairRelation::PlaneSeparated,
        TriangleTriangleRelation::CoplanarTouching => MeshFacePairRelation::CoplanarTouching,
        TriangleTriangleRelation::CoplanarOverlapping => MeshFacePairRelation::CoplanarOverlapping,
        TriangleTriangleRelation::Candidate => MeshFacePairRelation::Candidate,
        TriangleTriangleRelation::Unknown => MeshFacePairRelation::Unknown,
    }
}

fn validate_face(mesh: &ExactMesh, face: usize, side: &str) -> Result<(), MeshError> {
    if face < mesh.triangles().len() {
        return Ok(());
    }
    Err(MeshError::one(
        MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::IndexOutOfBounds,
            format!(
                "{side} face {face} is out of range for {} triangles",
                mesh.triangles().len()
            ),
        )
        .with_face(face),
    ))
}
