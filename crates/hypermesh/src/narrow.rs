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
    TrianglePlaneRelation, classify_coplanar_triangle_points,
    classify_triangle_against_oriented_plane, compare_reals, triangle_plane_relation_from_sides,
};

use super::mesh::ExactMesh;
use hyperlimit::{CoplanarTriangleClassification, CoplanarTriangleRelation};
use hyperreal::Real;

/// Certified coarse relation between two exact triangles.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleTriangleRelation {
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
/// This intentionally stops before coplanar overlap and full intersection
/// other triangle's plane. Hypermesh performs that stage through
/// `hyperlimit::orient3d_report` and keeps the segment/plane construction
/// events needed by the later exact splitter.
#[derive(Clone, Debug, PartialEq)]
pub struct TriangleTriangleClassification {
    /// Coarse relation.
    pub relation: TriangleTriangleRelation,
    /// Classification of the right triangle against the left triangle's plane.
    pub right_against_left_plane: TrianglePlaneClassification,
    /// Classification of the left triangle against the right triangle's plane.
    pub left_against_right_plane: TrianglePlaneClassification,
    /// Right-triangle edge events against the left plane.
    pub right_edge_events: Vec<SegmentPlaneIntersection>,
    /// Left-triangle edge events against the right plane.
    pub left_edge_events: Vec<SegmentPlaneIntersection>,
    /// Exact projected overlap result for coplanar pairs.
    pub coplanar: Option<CoplanarTriangleClassification>,
}

/// Classify a query triangle against an oriented face plane.
fn classify_triangle_against_face_plane_points(
    face: [&Point3; 3],
    query: [&Point3; 3],
) -> TrianglePlaneClassification {
    classify_triangle_against_oriented_plane(face, query)
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

/// Classify two triangles without materializing generic candidate edge events.
///
/// Mesh face-pair classification replaces candidate edge events immediately
/// with retained face-plane constructions from the source meshes. This helper
/// keeps that path from building throwaway generic segment/plane evidence.
pub(crate) fn classify_triangle_triangle_points_without_candidate_events(
    left: [&Point3; 3],
    right: [&Point3; 3],
) -> TriangleTriangleClassification {
    classify_triangle_triangle_points_retained(left, right)
}

fn classify_triangle_triangle_points_retained(
    left: [&Point3; 3],
    right: [&Point3; 3],
) -> TriangleTriangleClassification {
    let right_against_left_plane = classify_triangle_against_face_plane_points(left, right);
    let left_against_right_plane = classify_triangle_against_face_plane_points(right, left);
    let mut relation = triangle_triangle_relation(
        right_against_left_plane.relation,
        left_against_right_plane.relation,
    );
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
        right_against_left_plane,
        left_against_right_plane,
        right_edge_events: Vec::new(),
        left_edge_events: Vec::new(),
        coplanar,
    }
}

fn retained_plane_side(plane: &super::facts::FacePlaneFacts, point: &Point3) -> Option<PlaneSide> {
    let value = add(
        &add(
            &add(
                &mul(&plane.normal[0], &point.x),
                &mul(&plane.normal[1], &point.y),
            ),
            &mul(&plane.normal[2], &point.z),
        ),
        &plane.offset,
    );
    // `hyperlimit::orient3d_report(a, b, c, p)` uses the opposite sign
    // convention from this stored `(b - a) x (c - a)` dot-product form, so the
    // exact comparison is inverted to preserve the public `PlaneSide` contract.
    match compare_reals(&value, &Real::from(0)).value()? {
        Ordering::Less => Some(PlaneSide::Above),
        Ordering::Equal => Some(PlaneSide::On),
        Ordering::Greater => Some(PlaneSide::Below),
    }
}

fn add(left: &Real, right: &Real) -> Real {
    left.clone() + right
}

fn mul(left: &Real, right: &Real) -> Real {
    left.clone() * right
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
