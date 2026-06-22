//! Exact narrow-phase classification helpers.
//!
//! Full triangle/triangle intersection is deliberately not reimplemented here
//! as another local tolerance algorithm. Instead this module exposes certified
//! primitives that specialized kernels can migrate onto: classify triangle vertices
//! against an oriented face plane and retain the predicate route. Plane-side
//! orientation predicates come from `hyperlimit`, and each classification
//! retains the certificate that produced it.
//!
//! The plane-side rejection and candidate staging follows Moller, "A Fast
//! Triangle-Triangle Intersection Test," *Journal of Graphics Tools* 2.2
//! (1997). Coplanar overlap is delegated to exact orientation predicates in
//! the style of Guigue and Devillers, "Fast and Robust Triangle-Triangle
//! Overlap Test Using Orientation Predicates," *Journal of Graphics Tools* 8.1
//! (2003).

use core::cmp::Ordering;

use hyperlimit::{
    PlaneSide, Point3, SegmentPlaneIntersection, TrianglePlaneClassification,
    TrianglePlaneRelation, classify_coplanar_triangle_points, compare_reals,
    triangle_plane_relation_from_sides,
};

use super::mesh::ExactMesh;
use hyperlimit::{CoplanarTriangleClassification, CoplanarTriangleRelation};
use hyperreal::Real;

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

/// Classify a mesh triangle against a retained exact face plane.
///
/// This is the cached-object counterpart to
/// [`classify_triangle_against_face_plane`]. It evaluates the unnormalized
/// determinant-form plane coefficients retained in [`super::facts::FacePlaneFacts`]
/// this shape directly: object-level numerical structure should survive so
/// later topology stages can reuse exact facts instead of reconstructing
/// normals or representative floats.
pub(crate) fn classify_mesh_triangle_against_retained_face_plane_unchecked(
    plane_mesh: &ExactMesh,
    plane_face: usize,
    query_mesh: &ExactMesh,
    query_face: usize,
) -> TrianglePlaneClassification {
    let plane = &plane_mesh.facts().faces[plane_face].plane;
    let query = query_mesh.triangles()[query_face].0;
    let mut sides = [None, None, None];
    for (side, vertex) in sides.iter_mut().zip(query) {
        *side = retained_plane_side(plane, &query_mesh.vertices()[vertex]);
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
pub(crate) fn classify_triangle_triangle_points_from_plane_relations(
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

fn retained_plane_side(plane: &super::facts::FacePlaneFacts, point: &Point3) -> Option<PlaneSide> {
    let x_term = &plane.normal[0] * &point.x;
    let y_term = &plane.normal[1] * &point.y;
    let z_term = &plane.normal[2] * &point.z;
    let value = &(&(&x_term + &y_term) + &z_term) + &plane.offset;
    // `hyperlimit::orient3d_report(a, b, c, p)` uses the opposite sign
    // convention from this stored `(b - a) x (c - a)` dot-product form, so the
    // exact comparison is inverted to preserve the public `PlaneSide` contract.
    match compare_reals(&value, &Real::from(0)).value()? {
        Ordering::Less => Some(PlaneSide::Above),
        Ordering::Equal => Some(PlaneSide::On),
        Ordering::Greater => Some(PlaneSide::Below),
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
