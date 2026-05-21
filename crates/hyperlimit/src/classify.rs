//! Classification enums for geometry helpers.

use crate::predicate::Sign;

/// Side of an oriented line in 2D.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineSide {
    /// Point lies to the right of the oriented line.
    Right,
    /// Point lies on the line.
    On,
    /// Point lies to the left of the oriented line.
    Left,
}

impl From<Sign> for LineSide {
    fn from(sign: Sign) -> Self {
        match sign {
            Sign::Negative => Self::Right,
            Sign::Zero => Self::On,
            Sign::Positive => Self::Left,
        }
    }
}

/// Side of an oriented plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlaneSide {
    /// Point lies below the oriented plane.
    Below,
    /// Point lies on the plane.
    On,
    /// Point lies above the oriented plane.
    Above,
}

/// Relation between a closed 3D AABB and an oriented plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlaneAabbRelation {
    /// The entire box lies below the plane.
    Below,
    /// The entire box lies above the plane.
    Above,
    /// The box intersects or touches the plane.
    Intersecting,
}

/// Relation between a closed 3D segment and an oriented plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlaneSegmentRelation {
    /// Both segment endpoints are below the plane.
    Below,
    /// Both segment endpoints are above the plane.
    Above,
    /// Both segment endpoints lie on the plane.
    Coplanar,
    /// The segment crosses the plane with endpoints on opposite sides.
    Crossing,
    /// Exactly one endpoint lies on the plane.
    EndpointTouch,
}

/// Relation between a 3D triangle and an oriented plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlaneTriangleRelation {
    /// All triangle vertices are below the plane.
    Below,
    /// All triangle vertices are above the plane.
    Above,
    /// All triangle vertices lie on the plane.
    Coplanar,
    /// The triangle has vertices strictly on both sides of the plane.
    Split,
    /// The triangle touches the plane at one or two vertices, while all
    /// remaining vertices are on the same strict side.
    BoundaryTouch,
}

impl From<Sign> for PlaneSide {
    fn from(sign: Sign) -> Self {
        match sign {
            Sign::Negative => Self::Below,
            Sign::Zero => Self::On,
            Sign::Positive => Self::Above,
        }
    }
}

/// Location of a point relative to a 2D triangle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleLocation {
    /// The triangle vertices are collinear.
    Degenerate,
    /// The point lies outside the triangle.
    Outside,
    /// The point lies strictly inside the triangle.
    Inside,
    /// The point lies on one triangle edge and is not a vertex.
    OnEdge,
    /// The point lies on a triangle vertex.
    OnVertex,
}

/// Location of a point relative to a 3D triangle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Triangle3Location {
    /// The triangle vertices are collinear or otherwise degenerate.
    Degenerate,
    /// The point is not on the triangle's supporting plane.
    OffPlane,
    /// The point lies on the supporting plane but outside the triangle.
    Outside,
    /// The point lies strictly inside the triangle.
    Inside,
    /// The point lies on one triangle edge and is not a vertex.
    OnEdge,
    /// The point lies on a triangle vertex.
    OnVertex,
}

/// Location of a point relative to a 3D tetrahedron.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TetrahedronLocation {
    /// The tetrahedron vertices are coplanar or otherwise degenerate.
    Degenerate,
    /// The point lies outside the tetrahedron.
    Outside,
    /// The point lies strictly inside the tetrahedron.
    Inside,
    /// The point lies on one tetrahedron face and is not on an edge.
    OnFace,
    /// The point lies on one tetrahedron edge and is not a vertex.
    OnEdge,
    /// The point lies on a tetrahedron vertex.
    OnVertex,
}

/// Location of a point relative to an explicit 3D sphere.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpherePointLocation {
    /// The point lies outside the sphere.
    Outside,
    /// The point lies on the sphere.
    On,
    /// The point lies inside the sphere.
    Inside,
}

/// Relation between a 2D circle boundary and an infinite line.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CircleLineRelation {
    /// The line does not meet the circle boundary.
    Disjoint,
    /// The line touches the circle boundary at one point.
    Tangent,
    /// The line crosses the circle boundary at two points.
    Secant,
    /// The supplied line endpoints are identical, so the query degenerates to
    /// point/circle classification.
    DegenerateLine,
}

/// Relation between a 2D circle boundary and a closed segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CircleSegmentRelation {
    /// The segment does not meet the circle boundary.
    Disjoint,
    /// The segment touches the circle boundary at exactly one point.
    Tangent,
    /// The segment crosses the circle boundary at two boundary points.
    Secant,
    /// The nondegenerate segment lies strictly inside the circle disk, so it
    /// has no boundary intersection.
    ContainedInside,
}

/// Location of a point relative to an oriented convex region.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexPointLocation {
    /// The input convex carrier is too small or structurally degenerate.
    Degenerate,
    /// The point lies outside at least one convex halfspace.
    Outside,
    /// The point lies on at least one boundary edge or face and outside none.
    Boundary,
    /// The point lies strictly inside every boundary halfspace.
    Inside,
}

impl ConvexPointLocation {
    /// Returns whether the point is inside the closed convex region.
    pub const fn is_inside_or_boundary(self) -> bool {
        matches!(self, Self::Inside | Self::Boundary)
    }
}

/// Intersection relation between two explicit 3D spheres.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SphereIntersection {
    /// The closed spheres have no point in common.
    Disjoint,
    /// The closed spheres touch at one or more boundary points.
    Touching,
    /// The closed spheres have positive-volume or contained overlap.
    Overlapping,
}

impl SphereIntersection {
    /// Returns whether the closed spheres intersect inclusively.
    pub const fn intersects(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Intersection relation between a closed 3D AABB and an explicit sphere.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AabbSphereIntersection {
    /// The box and sphere have no point in common.
    Disjoint,
    /// The box and sphere touch at a boundary point or boundary patch.
    Touching,
    /// The box and sphere overlap beyond boundary contact.
    Overlapping,
}

impl AabbSphereIntersection {
    /// Returns whether the closed box and closed sphere intersect inclusively.
    pub const fn intersects(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Location of a point relative to a closed 2D segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PointSegmentLocation {
    /// The point is not on the segment's supporting line.
    OffLine,
    /// The point is collinear with the segment but outside its closed interval.
    CollinearOutside,
    /// The point equals one of the segment endpoints.
    OnEndpoint,
    /// The point lies strictly inside the segment interval.
    OnSegment,
}

impl PointSegmentLocation {
    /// Returns whether the location is on the closed segment.
    pub const fn is_on_segment(self) -> bool {
        matches!(self, Self::OnEndpoint | Self::OnSegment)
    }
}

/// Classification of two closed 2D segments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentIntersection {
    /// The segments have no points in common.
    Disjoint,
    /// The segments cross at one point interior to both segments.
    Proper,
    /// The segments touch at a shared endpoint or at an endpoint lying on the
    /// other segment.
    EndpointTouch,
    /// The segments are collinear and overlap over a positive-length interval,
    /// but they are not identical as closed endpoint pairs.
    CollinearOverlap,
    /// The segments have the same two endpoints, in either order.
    Identical,
}

impl SegmentIntersection {
    /// Returns whether the closed segments have no point in common.
    pub const fn is_disjoint(self) -> bool {
        matches!(self, Self::Disjoint)
    }

    /// Returns whether the closed segments have at least one common point.
    pub const fn intersects(self) -> bool {
        !self.is_disjoint()
    }

    /// Returns whether the segments cross at one point interior to both
    /// segments.
    ///
    /// This is the proper-crossing case in the standard four-orientation
    /// segment classifier; see de Berg, Cheong, van Kreveld, and Overmars,
    /// *Computational Geometry: Algorithms and Applications*, 3rd ed., 2008.
    pub const fn is_proper_crossing(self) -> bool {
        matches!(self, Self::Proper)
    }

    /// Returns whether the only certified contact is endpoint/boundary contact.
    pub const fn is_endpoint_touch(self) -> bool {
        matches!(self, Self::EndpointTouch)
    }

    /// Returns whether the common set contains a positive-length interval.
    ///
    /// Identical closed segments are included because they have the strongest
    /// possible positive-length overlap. Keeping this policy on the
    /// classification enum avoids each caller needing an exhaustive local match
    /// when new exact topological distinctions are added, which follows Yap's
    /// guidance to keep combinatorial decisions explicit at the predicate
    /// object layer; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub const fn has_positive_length_overlap(self) -> bool {
        matches!(self, Self::CollinearOverlap | Self::Identical)
    }
}

/// Classification of two closed 3D segments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Segment3Intersection {
    /// The segments are not coplanar and therefore cannot intersect.
    SkewDisjoint,
    /// The segments are coplanar but have no points in common.
    CoplanarDisjoint,
    /// The segments cross at one point interior to both segments.
    Proper,
    /// The segments touch at a shared endpoint, or one endpoint lies on the
    /// other segment.
    EndpointTouch,
    /// The segments are collinear and overlap over a positive-length interval,
    /// but they are not identical as closed endpoint pairs.
    CollinearOverlap,
    /// The segments have the same two endpoints, in either order. Two equal
    /// degenerate point-segments are also classified as identical.
    Identical,
}

/// Intersection relation between a closed 3D segment and a triangle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentTriangleIntersection {
    /// The segment and triangle have no point in common.
    Disjoint,
    /// The segment crosses the relative interior of the triangle at an
    /// interior point of the segment.
    Proper,
    /// The segment touches a triangle edge, vertex, or touches the triangle at
    /// one of the segment endpoints.
    BoundaryTouch,
    /// The segment lies on the triangle's supporting plane. Coplanar
    /// segment/triangle overlap needs a planar arrangement predicate owned by a
    /// higher crate, so `hyperlimit` reports the exact coplanar relation
    /// explicitly instead of using a tolerance projection.
    Coplanar,
}

impl SegmentTriangleIntersection {
    /// Returns whether the relation has at least one shared point.
    pub const fn intersects(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Intersection relation between a 3D ray and a triangle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RayTriangleIntersection {
    /// The ray and triangle have no point in common.
    Disjoint,
    /// The ray intersects the relative interior of the triangle at positive
    /// ray parameter.
    Proper,
    /// The ray touches a triangle edge or vertex, or starts on the triangle.
    BoundaryTouch,
    /// The ray lies on the triangle's supporting plane.
    Coplanar,
}

impl RayTriangleIntersection {
    /// Returns whether the relation has at least one shared point.
    pub const fn intersects(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Location of a point relative to a closed 2D polygon ring.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RingPointLocation {
    /// The point lies outside the ring according to the selected ring rule.
    Outside,
    /// The point lies on a ring edge or vertex.
    Boundary,
    /// The point lies inside the ring according to the selected ring rule.
    Inside,
}

impl RingPointLocation {
    /// Returns whether the location is inside or on the ring boundary.
    pub const fn is_inside_or_boundary(self) -> bool {
        matches!(self, Self::Boundary | Self::Inside)
    }
}

/// Certified local turn consistency for a polygonal ring.
///
/// This is an object fact, not a polygon validity proof. It summarizes exact
/// local orientation predicates so higher crates can choose algorithms without
/// replacing later visibility, containment, or intersection predicates. That is
/// the object/predicate split advocated by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RingConvexity {
    /// Fewer than three useful vertices, or every certified local turn is zero.
    Degenerate,
    /// Every certified nonzero local turn has the same sign and no turn is unknown.
    LocallyConvex,
    /// Certified nonzero local turns contain both signs.
    MixedTurns,
    /// At least one local turn could not be certified under the requested policy.
    Unknown,
}

/// Location of a [`hyperreal::Real`] value relative to a closed Real interval.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RealIntervalLocation {
    /// The value is strictly below the interval's lower endpoint.
    Below,
    /// The value equals the interval's lower endpoint.
    AtLowerEndpoint,
    /// The value is strictly between the interval endpoints.
    Interior,
    /// The value equals the interval's upper endpoint.
    AtUpperEndpoint,
    /// The value is strictly above the interval's upper endpoint.
    Above,
}

impl RealIntervalLocation {
    /// Returns whether the value lies in the closed interval.
    pub const fn is_inside_or_boundary(self) -> bool {
        matches!(
            self,
            Self::AtLowerEndpoint | Self::Interior | Self::AtUpperEndpoint
        )
    }
}

/// Intersection relation between two closed Real intervals.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClosedIntervalIntersection {
    /// The closed intervals have no value in common.
    Disjoint,
    /// The closed intervals intersect at exactly one endpoint value.
    Touching,
    /// The closed intervals overlap over more than one value.
    Overlapping,
}

impl ClosedIntervalIntersection {
    /// Returns whether the intervals have at least one value in common.
    pub const fn intersects(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Location of a point relative to a closed 2D axis-aligned box.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Aabb2PointLocation {
    /// The point lies outside the box.
    Outside,
    /// The point lies on a box edge or corner.
    Boundary,
    /// The point lies strictly inside the box.
    Inside,
}

impl Aabb2PointLocation {
    /// Returns whether the location is inside or on the box boundary.
    pub const fn is_inside_or_boundary(self) -> bool {
        matches!(self, Self::Boundary | Self::Inside)
    }
}

/// Location of a point relative to a closed 3D axis-aligned box.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Aabb3PointLocation {
    /// The point lies outside the box.
    Outside,
    /// The point lies on a box face, edge, or corner.
    Boundary,
    /// The point lies strictly inside the box.
    Inside,
}

impl Aabb3PointLocation {
    /// Returns whether the location is inside or on the box boundary.
    pub const fn is_inside_or_boundary(self) -> bool {
        matches!(self, Self::Boundary | Self::Inside)
    }
}

/// Intersection relation between two closed 2D axis-aligned boxes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Aabb2Intersection {
    /// The boxes have no point in common.
    Disjoint,
    /// The boxes touch at a boundary point or boundary interval only.
    Touching,
    /// The boxes overlap with positive area.
    Overlapping,
}

impl Aabb2Intersection {
    /// Returns whether the boxes have at least one point in common.
    pub const fn intersects(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Intersection relation between two closed 3D axis-aligned boxes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Aabb3Intersection {
    /// The boxes are disjoint on at least one axis.
    Disjoint,
    /// The boxes intersect, but at least one axis has zero-width overlap.
    Touching,
    /// The boxes overlap with positive extent on all three axes.
    Overlapping,
}

impl Aabb3Intersection {
    /// Returns whether the boxes intersect inclusively.
    pub const fn intersects(self) -> bool {
        !matches!(self, Self::Disjoint)
    }

    /// Returns whether a broad-phase user must keep this pair for narrow phase.
    pub const fn needs_narrow_phase(self) -> bool {
        self.intersects()
    }
}

#[cfg(test)]
mod tests {
    use super::SegmentIntersection;

    #[test]
    fn segment_intersection_policy_helpers_cover_identical_overlap() {
        assert!(SegmentIntersection::Disjoint.is_disjoint());
        assert!(!SegmentIntersection::Disjoint.intersects());
        assert!(SegmentIntersection::Proper.is_proper_crossing());
        assert!(SegmentIntersection::EndpointTouch.is_endpoint_touch());
        assert!(SegmentIntersection::CollinearOverlap.has_positive_length_overlap());
        assert!(SegmentIntersection::Identical.has_positive_length_overlap());
        assert!(SegmentIntersection::Identical.intersects());
    }
}
