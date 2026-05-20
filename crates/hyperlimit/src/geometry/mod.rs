pub mod facts;
pub mod homogeneous;
pub mod plane;
pub mod point;

pub use self::facts::{
    Aabb2Facts, CoordinateAxis2, Point2DisplacementFacts, Segment2Facts, Triangle2Facts,
    TriangleEdge2, aabb2_facts, point2_displacement_facts, segment2_facts, triangle2_facts,
};
pub use self::homogeneous::{
    HomogeneousLine3, HomogeneousPoint3, classify_homogeneous_point_plane,
    classify_homogeneous_point_plane_with_policy, intersect_homogeneous_line_plane,
    intersect_three_planes, intersect_two_planes,
};
pub use self::plane::{Plane3, Plane3Facts};
pub use self::point::{Point2, Point2Facts, Point3, Point3Facts, PointSharedScaleView};
