//! Exact construction helpers for mesh intersection events.
//!
//! The reusable segment/plane event model lives in `hyperlimit`: endpoint
//! predicates decide the combinatorial relation, and proper crossings retain
//! the exact determinant-ratio parameter `d0 / (d0 - d1)`. That is the
//! mesh-specific adapters here, such as indexing a face plane from a mesh or
//! reusing retained [`FacePlaneFacts`].

use hyperlimit::{PlaneSide, Point3, compare_reals, intersect_segment_with_plane_values};

use super::facts::FacePlaneFacts;
use hyperreal::Real;

pub use hyperlimit::{
    SegmentPlaneConstructionFailure, SegmentPlaneIntersection, SegmentPlaneParameterRatio,
    SegmentPlaneRelation, SegmentPlaneValidationError,
    construct_segment_plane_crossing_from_values, interpolate_point3,
    intersect_segment_with_oriented_plane, intersect_segment_with_plane, point_plane_value,
    segment_parameter_from_axis,
};

/// Intersect a mesh segment with the oriented plane of one triangular face.
///
/// The face orientation is the vertex order in `face`, matching
/// `hyperlimit::orient3d_report(a, b, c, point)`. A proper crossing constructs
/// `t = d0 / (d0 - d1)`, where `d0` and `d1` are exact evaluations of the same
/// oriented plane at the segment endpoints. This determinant-ratio form keeps
/// the construction exact and auditable for later edge ordering.
pub fn intersect_segment_with_face_plane(
    points: &[Point3],
    face: [usize; 3],
    segment: [usize; 2],
) -> SegmentPlaneIntersection {
    intersect_segment_with_oriented_plane(
        &points[face[0]],
        &points[face[1]],
        &points[face[2]],
        &points[segment[0]],
        &points[segment[1]],
    )
}

/// Intersect a closed segment with a retained exact face plane.
///
/// This cached construction path consumes determinant-form coefficients
/// retained in [`FacePlaneFacts`] and builds the same segment event as
/// [`intersect_segment_with_oriented_plane`] without reconstructing a
/// structure as part of the exact object model: constructions should reuse
/// certified object facts rather than reintroducing representative primitive
/// normals.
pub fn intersect_segment_with_retained_face_plane(
    plane: &FacePlaneFacts,
    p0: &Point3,
    p1: &Point3,
) -> SegmentPlaneIntersection {
    let d0 = retained_point_plane_value(plane, p0);
    let d1 = retained_point_plane_value(plane, p1);
    let sides = [retained_plane_side(&d0), retained_plane_side(&d1)];

    intersect_segment_with_plane_values(&d0, &d1, p0, p1, sides, Vec::new())
}

fn retained_point_plane_value(plane: &FacePlaneFacts, point: &Point3) -> Real {
    plane.normal[0].clone() * point.x.clone()
        + plane.normal[1].clone() * point.y.clone()
        + plane.normal[2].clone() * point.z.clone()
        + &plane.offset
}

fn retained_plane_side(value: &Real) -> Option<PlaneSide> {
    // `hyperlimit::orient3d_report(a, b, c, p)` uses the opposite sign
    // convention from the stored `(b - a) x (c - a)` dot-product form.
    match compare_reals(value, &Real::from(0)).value()? {
        core::cmp::Ordering::Less => Some(PlaneSide::Above),
        core::cmp::Ordering::Equal => Some(PlaneSide::On),
        core::cmp::Ordering::Greater => Some(PlaneSide::Below),
    }
}
