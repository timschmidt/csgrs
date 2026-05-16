pub mod facts;
pub mod plane;
pub mod point;

pub use self::facts::{
    Aabb2Facts, CoordinateAxis2, Point2DisplacementFacts, Segment2Facts, Triangle2Facts,
    TriangleEdge2, aabb2_facts, point2_displacement_facts, segment2_facts, triangle2_facts,
};
pub use self::plane::{Plane3, Plane3Facts};
pub use self::point::{Point2, Point2Facts, Point3, Point3Facts, PointSharedScaleView};
