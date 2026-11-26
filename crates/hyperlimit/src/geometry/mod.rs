//! Predicate inputs, structural summaries and retained geometric evidence.
//!
//! Point and homogeneous storage are re-exported from Hyperlattice; Hyperlimit
//! owns predicate policy, classification and the associated reports. Structural
//! summaries expose only proven facts or explicit unknowns and must remain tied
//! to the source geometry when reused. Public record fields do not enforce that
//! association automatically.

/// Zero-support summaries for planar displacements, segments, triangles and boxes.
pub mod facts;
/// Shared homogeneous carriers with predicate-layer incidence and construction adapters.
pub mod homogeneous;
/// Oriented plane inputs, reusable predicate evidence and report validation.
pub mod plane;
/// Shared point carriers and point facts owned by Hyperlattice.
pub mod point;

pub use self::facts::{
    Aabb2Facts, CoordinateAxis2, Point2DisplacementFacts, Segment2Facts, Triangle2Facts,
    TriangleEdge2, aabb2_facts, point2_displacement_facts, segment2_facts, triangle2_facts,
};
pub use self::homogeneous::{
    HomogeneousLine3, HomogeneousPoint3, classify_homogeneous_point_plane_with_policy,
    intersect_homogeneous_line_plane, intersect_three_planes, intersect_two_planes,
};
pub use self::plane::{Plane3, Plane3Facts};
pub use self::point::{Point2, Point2Facts, Point3, Point3Facts, PointSharedScaleView};
