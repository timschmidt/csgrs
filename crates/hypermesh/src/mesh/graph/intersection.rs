//! Exact mesh intersection scheduling and narrow-phase classification.
//!
//! This module joins the retained exact AABB broad phase with the certified
//! triangle/triangle coarse classifier.  It is still a scheduler and event
//! collector, not the final boolean graph builder: broad-phase disjointness
//! and retained plane separation may reject work, while coplanar and candidate
//! outcomes must continue into exact overlap-graph construction. Retained exact
//! face-plane coefficients are used as cached plane-side facts before relation
//! assembly. Candidate split events are retained only after certified
//! predicates and exact constructions agree.
//!
//! Full triangle/triangle intersection is deliberately not reimplemented here
//! as another local tolerance algorithm. The narrow phase classifies triangle
//! vertices against retained oriented face planes, then stores exact
//! segment/plane construction events from `hyperlimit` for later graph stages.

use core::cmp::Ordering;

use hyperlimit::{
    CoplanarTriangleClassification, CoplanarTriangleRelation, PlaneSide, Point3,
    SegmentPlaneIntersection, TrianglePlaneClassification, TrianglePlaneRelation,
    classify_coplanar_triangle_points, compare_reals, intersect_segment_with_plane_values,
    triangle_plane_relation_from_sides,
};
use hyperreal::Real;

use super::super::Mesh;
use super::super::facts::FacePlaneFacts;
use super::super::triangle_edges;

/// Certified coarse relation between two exact triangles.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum TriangleTriangleRelation {
    /// The second triangle lies strictly on one side of the first triangle's
    /// plane.
    SeparatedByFirstPlane,
    /// The first triangle lies strictly on one side of the second triangle's
    /// plane.
    SeparatedBySecondPlane,
    /// Both triangles are coplanar but exact projected 2D predicates prove the
    /// closed triangles are disjoint.
    CoplanarDisjoint,
    /// Both triangles are coplanar and touch at a vertex or edge.
    CoplanarTouching,
    /// Both triangles are coplanar and overlap with positive area or a
    /// positive-length edge interval.
    CoplanarOverlapping,
    /// Plane-side predicates prove a non-coplanar candidate requiring exact
    /// segment/triangle and interval ordering.
    Candidate,
    /// At least one required plane-side predicate was undecided.
    Unknown,
}

/// Certified triangle/triangle coarse classification.
///
/// This intentionally stops before full intersection materialization.
/// Hypermesh performs that stage through retained segment/plane constructions
/// and keeps those events for the later exact splitter.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct TriangleTriangleClassification {
    /// Coarse relation.
    pub(crate) relation: TriangleTriangleRelation,
    /// Right-triangle edge events against the left plane.
    pub(crate) right_edge_events: Option<[SegmentPlaneIntersection; 3]>,
    /// Left-triangle edge events against the right plane.
    pub(crate) left_edge_events: Option<[SegmentPlaneIntersection; 3]>,
    /// Exact projected overlap result for coplanar pairs.
    pub(crate) coplanar: Option<CoplanarTriangleClassification>,
}

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

pub(crate) fn classify_mesh_face_pair_unchecked(
    left: &Mesh,
    left_face: usize,
    right: &Mesh,
    right_face: usize,
) -> MeshFacePairClassification {
    let right_against_left = classify_mesh_triangle_against_retained_face_plane_unchecked(
        left, left_face, right, right_face,
    );
    if matches!(
        right_against_left.relation,
        TrianglePlaneRelation::StrictlyAbove | TrianglePlaneRelation::StrictlyBelow
    ) {
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
    if matches!(
        left_against_right.relation,
        TrianglePlaneRelation::StrictlyAbove | TrianglePlaneRelation::StrictlyBelow
    ) {
        return MeshFacePairClassification {
            left_face,
            right_face,
            triangle: None,
            relation: MeshFacePairRelation::PlaneSeparated,
        };
    }

    let mut triangle = classify_mesh_triangles_from_retained_plane_relations(
        left,
        left_face,
        right,
        right_face,
        right_against_left.relation,
        left_against_right.relation,
    );
    if triangle.relation == TriangleTriangleRelation::Candidate {
        triangle.right_edge_events = Some(retained_triangle_edge_events(
            left, left_face, right, right_face,
        ));
        triangle.left_edge_events = Some(retained_triangle_edge_events(
            right, right_face, left, left_face,
        ));
    }
    let relation = mesh_relation_from_triangle(triangle.relation);

    MeshFacePairClassification {
        left_face,
        right_face,
        triangle: Some(triangle),
        relation,
    }
}

fn classify_mesh_triangles_from_retained_plane_relations(
    left: &Mesh,
    left_face: usize,
    right: &Mesh,
    right_face: usize,
    right_against_left_plane: TrianglePlaneRelation,
    left_against_right_plane: TrianglePlaneRelation,
) -> TriangleTriangleClassification {
    classify_triangle_triangle_points_from_plane_relations(
        retained_face_vertices_unchecked(left, left_face),
        retained_face_vertices_unchecked(right, right_face),
        right_against_left_plane,
        left_against_right_plane,
    )
}

/// Classify a mesh triangle against a retained exact face plane.
///
/// This cached-object path evaluates the unnormalized determinant-form plane
/// coefficients retained in [`FacePlaneFacts`] directly: object-level numerical
/// structure should survive so later topology stages can reuse exact facts
/// instead of reconstructing normals or representative floats.
fn classify_mesh_triangle_against_retained_face_plane_unchecked(
    plane_mesh: &Mesh,
    plane_face: usize,
    query_mesh: &Mesh,
    query_face: usize,
) -> TrianglePlaneClassification {
    let plane = retained_face_plane_unchecked(plane_mesh, plane_face);
    let query = retained_face_vertex_indices_unchecked(query_mesh, query_face);
    let mut sides = [None, None, None];
    for (side, vertex) in sides.iter_mut().zip(query) {
        *side = retained_plane_side_from_value(&retained_point_plane_value(
            plane,
            retained_vertex_point_unchecked(query_mesh, vertex),
        ));
    }

    TrianglePlaneClassification {
        relation: triangle_plane_relation_from_sides(sides),
        vertex_sides: sides,
        predicates: Vec::new(),
    }
}

/// Assemble a triangle-triangle classification from existing plane-side relations.
///
/// Mesh face-pair classification first uses retained face-plane facts for
/// cheap one-sided rejection. Non-separated pairs can reuse those exact results
/// here instead of replaying the same plane predicates from point triples.
/// Candidate edge events are still left empty because mesh classification
/// replaces them immediately with retained source-plane constructions.
fn classify_triangle_triangle_points_from_plane_relations(
    left: [&Point3; 3],
    right: [&Point3; 3],
    right_against_left_plane: TrianglePlaneRelation,
    left_against_right_plane: TrianglePlaneRelation,
) -> TriangleTriangleClassification {
    let mut relation =
        triangle_triangle_relation(right_against_left_plane, left_against_right_plane);
    let coplanar = if relation == TriangleTriangleRelation::CoplanarOverlapping {
        let coplanar = classify_coplanar_triangle_points(left, right);
        relation = match coplanar.relation {
            CoplanarTriangleRelation::Disjoint => TriangleTriangleRelation::CoplanarDisjoint,
            CoplanarTriangleRelation::Touching => TriangleTriangleRelation::CoplanarTouching,
            CoplanarTriangleRelation::Overlapping => TriangleTriangleRelation::CoplanarOverlapping,
            CoplanarTriangleRelation::Unknown => TriangleTriangleRelation::Unknown,
        };
        Some(coplanar)
    } else {
        None
    };

    TriangleTriangleClassification {
        relation,
        right_edge_events: None,
        left_edge_events: None,
        coplanar,
    }
}

fn retained_triangle_edge_events(
    plane_mesh: &Mesh,
    plane_face: usize,
    segment_mesh: &Mesh,
    segment_face: usize,
) -> [SegmentPlaneIntersection; 3] {
    let plane = retained_face_plane_unchecked(plane_mesh, plane_face);
    let segment = retained_face_vertex_indices_unchecked(segment_mesh, segment_face);
    triangle_edges(segment).map(|edge| {
        intersect_segment_with_retained_face_plane(
            plane,
            retained_vertex_point_unchecked(segment_mesh, edge[0]),
            retained_vertex_point_unchecked(segment_mesh, edge[1]),
        )
    })
}

fn retained_face_vertices_unchecked(mesh: &Mesh, face: usize) -> [&Point3; 3] {
    mesh.view()
        .face(face)
        .expect("retained face-pair classification references a missing face")
        .vertices()
        .expect("retained face-pair classification references a missing vertex")
}

fn retained_face_vertex_indices_unchecked(mesh: &Mesh, face: usize) -> [usize; 3] {
    mesh.view()
        .face(face)
        .expect("retained face-pair classification references a missing face")
        .vertex_indices()
}

fn retained_face_plane_unchecked(mesh: &Mesh, face: usize) -> &FacePlaneFacts {
    mesh.view()
        .face(face)
        .expect("retained face-pair classification references a missing face")
        .plane()
}

fn retained_vertex_point_unchecked(mesh: &Mesh, vertex: usize) -> &Point3 {
    mesh.view()
        .vertex(vertex)
        .expect("retained face-pair classification references a missing vertex")
        .point()
}

/// Intersect a closed segment with a retained exact face plane.
///
/// This cached construction path consumes determinant-form coefficients
/// retained in [`FacePlaneFacts`] and builds the same segment event as
/// `hyperlimit` plane intersection without reconstructing a structure as part
/// of the exact object model.
fn intersect_segment_with_retained_face_plane(
    plane: &FacePlaneFacts,
    p0: &Point3,
    p1: &Point3,
) -> SegmentPlaneIntersection {
    let d0 = retained_point_plane_value(plane, p0);
    let d1 = retained_point_plane_value(plane, p1);
    let sides = [
        retained_plane_side_from_value(&d0),
        retained_plane_side_from_value(&d1),
    ];

    intersect_segment_with_plane_values(&d0, &d1, p0, p1, sides, Vec::new())
}

fn retained_point_plane_value(plane: &FacePlaneFacts, point: &Point3) -> Real {
    let x_term = &plane.normal[0] * &point.x;
    let y_term = &plane.normal[1] * &point.y;
    let z_term = &plane.normal[2] * &point.z;
    &(&(&x_term + &y_term) + &z_term) + &plane.offset
}

fn retained_plane_side_from_value(value: &Real) -> Option<PlaneSide> {
    // `hyperlimit::orient3d_report(a, b, c, p)` uses the opposite sign
    // convention from this stored `(b - a) x (c - a)` dot-product form, so the
    // exact comparison is inverted to preserve the public `PlaneSide` contract.
    match compare_reals(value, &Real::from(0)).value()? {
        Ordering::Less => Some(PlaneSide::Above),
        Ordering::Equal => Some(PlaneSide::On),
        Ordering::Greater => Some(PlaneSide::Below),
    }
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

fn triangle_triangle_relation(
    right_against_left: TrianglePlaneRelation,
    left_against_right: TrianglePlaneRelation,
) -> TriangleTriangleRelation {
    if matches!(
        right_against_left,
        TrianglePlaneRelation::StrictlyAbove | TrianglePlaneRelation::StrictlyBelow
    ) {
        return TriangleTriangleRelation::SeparatedByFirstPlane;
    }
    if matches!(
        left_against_right,
        TrianglePlaneRelation::StrictlyAbove | TrianglePlaneRelation::StrictlyBelow
    ) {
        return TriangleTriangleRelation::SeparatedBySecondPlane;
    }
    if right_against_left == TrianglePlaneRelation::Unknown
        || left_against_right == TrianglePlaneRelation::Unknown
    {
        return TriangleTriangleRelation::Unknown;
    }
    if right_against_left == TrianglePlaneRelation::Coplanar
        && left_against_right == TrianglePlaneRelation::Coplanar
    {
        return TriangleTriangleRelation::CoplanarOverlapping;
    }
    TriangleTriangleRelation::Candidate
}
