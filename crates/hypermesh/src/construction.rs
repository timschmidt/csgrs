//! Exact construction helpers for mesh intersection events.
//!
//! The reusable segment/plane event model lives in `hyperlimit`: endpoint
//! predicates decide the combinatorial relation, and proper crossings retain
//! the exact determinant-ratio parameter `d0 / (d0 - d1)`. This module
//! replays those retained [`FacePlaneFacts`] into construction records.

use hyperlimit::{
    PlaneSide, Point3, SegmentPlaneIntersection, compare_reals, intersect_segment_with_plane_values,
};

use super::facts::FacePlaneFacts;
use hyperreal::Real;

/// Intersect a closed segment with a retained exact face plane.
///
/// This cached construction path consumes determinant-form coefficients
/// retained in [`FacePlaneFacts`] and builds the same segment event as
/// `hyperlimit` plane intersection without reconstructing a structure as part
/// of the exact object model: constructions should reuse certified object
/// facts rather than reintroducing representative primitive normals.
pub(crate) fn intersect_segment_with_retained_face_plane(
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
    let x_term = &plane.normal[0] * &point.x;
    let y_term = &plane.normal[1] * &point.y;
    let z_term = &plane.normal[2] * &point.z;
    &(&(&x_term + &y_term) + &z_term) + &plane.offset
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
