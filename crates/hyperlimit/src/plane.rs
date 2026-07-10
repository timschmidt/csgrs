//! Plane classification helpers.

pub use crate::geometry::Plane3;
pub use crate::geometry::plane::{
    PlaneAabbReport, PlaneAabbReportValidationError, PreparedOrientedPlane3, PreparedPlane3,
    TrianglePlaneClassification, TrianglePlaneRelation, TrianglePlaneValidationError,
    classify_plane_aabb3, classify_plane_aabb3_report, classify_plane_segment,
    classify_plane_triangle, classify_point_oriented_plane, classify_point_plane,
    classify_triangle_against_oriented_plane, triangle_plane_relation_from_sides,
};
pub(crate) use crate::geometry::plane::{
    classify_point_oriented_plane_with_policy, classify_point_plane_with_policy,
};
