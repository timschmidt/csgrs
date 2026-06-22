//! Exact mesh intersection scheduling.
//!
//! This module joins the retained exact AABB broad phase with the certified
//! triangle/triangle coarse classifier.  It is still a scheduler and event
//! collector, not the final boolean graph builder: broad-phase disjointness
//! and retained plane separation may reject work, while coplanar and candidate
//! outcomes must continue into exact overlap-graph construction. Retained exact
//! face-plane coefficients are used as a cached plane-separation filter before
//! the full triangle classifier is rebuilt. Candidate split events are retained
//! only after certified predicates and exact constructions agree.

use hyperlimit::{SegmentPlaneIntersection, TrianglePlaneRelation};

use super::construction::intersect_segment_with_retained_face_plane;
use super::error::ExactMeshError;
use super::mesh::ExactMesh;
use super::narrow::{
    TriangleTriangleClassification, TriangleTriangleRelation,
    classify_mesh_triangle_against_retained_face_plane_unchecked,
    classify_triangle_triangle_points_without_candidate_events,
};
use super::topology::triangle_edges;
use super::view::PreparedMeshPairView;

/// Coarse exact relation for one pair of mesh faces.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum MeshFacePairRelation {
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
pub(crate) struct MeshFacePairClassification {
    /// Face index in the left mesh.
    pub left_face: usize,
    /// Face index in the right mesh.
    pub right_face: usize,
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

fn classify_mesh_face_pair_unchecked(
    left: &ExactMesh,
    left_face: usize,
    right: &ExactMesh,
    right_face: usize,
) -> MeshFacePairClassification {
    let right_against_left = classify_mesh_triangle_against_retained_face_plane_unchecked(
        left, left_face, right, right_face,
    );
    if triangle_is_strictly_one_sided(right_against_left.relation) {
        return MeshFacePairClassification {
            left_face,
            right_face,
            triangle: None,
            relation: MeshFacePairRelation::PlaneSeparated,
        };
    }

    let left_against_right = classify_mesh_triangle_against_retained_face_plane_unchecked(
        right, right_face, left, left_face,
    );
    if triangle_is_strictly_one_sided(left_against_right.relation) {
        return MeshFacePairClassification {
            left_face,
            right_face,
            triangle: None,
            relation: MeshFacePairRelation::PlaneSeparated,
        };
    }

    let mut triangle =
        classify_mesh_triangles_without_candidate_events(left, left_face, right, right_face);
    if triangle.relation == TriangleTriangleRelation::Candidate {
        triangle.right_edge_events =
            retained_triangle_edge_events(left, left_face, right, right_face);
        triangle.left_edge_events =
            retained_triangle_edge_events(right, right_face, left, left_face);
    }
    let relation = mesh_relation_from_triangle(triangle.relation);

    MeshFacePairClassification {
        left_face,
        right_face,
        triangle: Some(triangle),
        relation,
    }
}

/// Visit every prepared face pair that survives exact broad/narrow rejection.
///
/// The visitor receives only pairs that were not proven impossible by exact
/// AABB disjointness, certified triangle plane separation, or exact coplanar
/// disjointness. Coplanar touching/overlapping, non-coplanar candidate, and
/// unknown pairs remain because they are exactly the cases that need
/// overlap-graph construction or a policy decision.
pub(crate) fn visit_prepared_mesh_pair_face_pair_classifications(
    pair: &PreparedMeshPairView<'_, '_>,
    mut visit: impl FnMut(MeshFacePairClassification) -> Result<(), ExactMeshError>,
) -> Result<(), ExactMeshError> {
    let left = pair.left().view().mesh();
    let right = pair.right().view().mesh();
    pair.try_visit_candidate_face_pairs(|[left_face, right_face]| {
        let classification = classify_mesh_face_pair_unchecked(left, left_face, right, right_face);
        if classification.needs_graph_construction() {
            visit(classification)?;
        }
        Ok(())
    })
}

fn triangle_is_strictly_one_sided(relation: TrianglePlaneRelation) -> bool {
    matches!(
        relation,
        TrianglePlaneRelation::StrictlyAbove | TrianglePlaneRelation::StrictlyBelow
    )
}

fn classify_mesh_triangles_without_candidate_events(
    left: &ExactMesh,
    left_face: usize,
    right: &ExactMesh,
    right_face: usize,
) -> TriangleTriangleClassification {
    let left_tri = left.triangles()[left_face].0;
    let right_tri = right.triangles()[right_face].0;
    classify_triangle_triangle_points_without_candidate_events(
        [
            &left.vertices()[left_tri[0]],
            &left.vertices()[left_tri[1]],
            &left.vertices()[left_tri[2]],
        ],
        [
            &right.vertices()[right_tri[0]],
            &right.vertices()[right_tri[1]],
            &right.vertices()[right_tri[2]],
        ],
    )
}

fn retained_triangle_edge_events(
    plane_mesh: &ExactMesh,
    plane_face: usize,
    segment_mesh: &ExactMesh,
    segment_face: usize,
) -> Vec<SegmentPlaneIntersection> {
    let plane = &plane_mesh.facts().faces[plane_face].plane;
    let mut events = Vec::with_capacity(3);
    for edge in triangle_edges(segment_mesh.triangles()[segment_face].0) {
        events.push(intersect_segment_with_retained_face_plane(
            plane,
            &segment_mesh.vertices()[edge[0]],
            &segment_mesh.vertices()[edge[1]],
        ));
    }
    events
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
