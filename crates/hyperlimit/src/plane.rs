//! Plane classification helpers.

pub use crate::geometry::plane::{
    PreparedOrientedPlane3, PreparedPlane3, classify_plane_segment,
    classify_plane_segment_with_policy, classify_plane_triangle,
    classify_plane_triangle_with_policy, classify_point_oriented_plane,
    classify_point_oriented_plane_with_policy, classify_point_plane,
    classify_point_plane_with_policy,
};
pub use crate::geometry::{Plane3, Plane3Facts};
