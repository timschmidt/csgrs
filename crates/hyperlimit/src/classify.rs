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
