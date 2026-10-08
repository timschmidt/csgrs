//! Plane classification helpers.

pub use crate::geometry::Plane3;
pub use crate::geometry::plane::{
    OrientedPlane3Evidence, Plane3Evidence, PlaneAabbReport, PlaneAabbReportValidationError,
    TrianglePlaneRelation, TrianglePlaneReport, TrianglePlaneReportValidationError,
    classify_plane_aabb3_report_with_policy as classify_plane_aabb3_report,
    classify_plane_aabb3_with_policy as classify_plane_aabb3,
    classify_plane_segment_with_policy as classify_plane_segment,
    classify_plane_triangle_with_policy as classify_plane_triangle,
    classify_point_oriented_plane_with_evidence_and_policy as classify_point_oriented_plane_with_evidence,
    classify_point_oriented_plane_with_policy as classify_point_oriented_plane,
    classify_point_plane_with_evidence_and_policy as classify_point_plane_with_evidence,
    classify_point_plane_with_policy as classify_point_plane,
    classify_triangle_against_oriented_plane_with_policy as classify_triangle_against_oriented_plane,
    oriented_plane3_evidence, plane3_evidence, triangle_plane_relation_from_sides,
};
pub(crate) use crate::geometry::plane::{
    classify_point_oriented_plane_with_policy, classify_point_plane_with_policy,
    classify_point_plane_without_filter_with_policy,
};
