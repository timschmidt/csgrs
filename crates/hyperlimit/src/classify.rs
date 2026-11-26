//! Classification enums for geometry helpers.
//!
//! These are relation labels, not geometry carriers or standalone proof objects.
//! Keep the producing predicate's certainty and input-domain contract with each
//! label. Conversions and Boolean helpers only inspect variants; they neither
//! validate source geometry nor recover a discarded receipt. In particular,
//! coplanarity can defer a planar overlap decision, and an identical segment
//! label does not distinguish a positive-length segment from a point-segment.

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
    /// Map negative/zero/positive orientation to right/on/left without a certainty receipt.
    fn from(sign: Sign) -> Self {
        match sign {
            Sign::Negative => Self::Right,
            Sign::Zero => Self::On,
            Sign::Positive => Self::Left,
        }
    }
}

/// Side of an oriented plane.
///
/// Below and above refer to the sign of `normal . point + offset`, not a
/// world-coordinate height. Reversing the plane orientation exchanges them.
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
    /// Map the signed plane expression to below/on/above without rechecking geometry.
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
    /// The point lies in the triangle's relative interior within its supporting plane.
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
    /// The supplied endpoints coincide, so they do not define a line.
    /// The classifier returns this label without deciding the point/circle relation.
    DegenerateLine,
}

/// Relation between a 2D circle boundary and a closed segment.
///
/// The labels follow the current endpoint-distance partition. In particular,
/// Tangent is not a differential tangency certificate and Secant does not
/// promise two boundary intersections. The circle is specified by its center
/// and nonnegative squared radius in the producing classifier.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CircleSegmentRelation {
    /// The segment does not meet the circle boundary.
    Disjoint,
    /// The point-segment is on the circle, one endpoint is on and the other
    /// strictly inside, or two outside endpoints have minimum distance equal
    /// to the radius. The endpoint case need not be geometrically tangent.
    Tangent,
    /// Both distinct endpoints are on the circle, one is on and the other
    /// outside, one is inside and the other outside, or two outside endpoints
    /// have minimum distance less than the radius.
    ///
    /// This can describe just one boundary point: for the unit circle, the
    /// segment from (1, 0) to (2, 0) receives this label.
    Secant,
    /// Both endpoints lie strictly inside the disk, including a coincident
    /// point-segment; there is no boundary intersection.
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

/// Feasibility status for a closed intersection of oriented plane halfspaces.
///
/// Halfspaces use `normal . point + offset <= 0`. This status alone contains
/// neither a witness nor a nonnegative-multiplier infeasibility certificate;
/// preserve the report and its outcome qualification for downstream replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HalfspaceFeasibility {
    /// At least one exact point satisfies every halfspace.
    Feasible,
    /// No exact point satisfies every halfspace.
    Infeasible,
}

/// Relation between a query interval and a retained support k-DOP slab set.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopRelation {
    /// The retained slab set is empty or contains an invalid interval.
    Degenerate,
    /// At least one support axis separates the query from the k-DOP.
    Separated,
    /// No support axis separates the query, and at least one tested interval
    /// touches only at a slab boundary.
    BoundaryTouch,
    /// All tested support-axis intervals overlap with positive width.
    ///
    /// For AABB queries this is a conservative interval relation against the
    /// retained k-DOP axes. It is exact for the tested slab projections, but it
    /// is not a constructive witness that the AABB intersects the underlying
    /// source object unless the caller's k-DOP axis family is complete for that
    /// domain.
    ConservativeOverlap,
}

impl SupportDopRelation {
    /// Returns whether the tested support intervals are not separated.
    pub const fn may_intersect(self) -> bool {
        !matches!(self, Self::Degenerate | Self::Separated)
    }
}

/// Relation between a retained support k-DOP and an oriented plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopPlaneRelation {
    /// The retained slab set is empty, invalid, or infeasible.
    Degenerate,
    /// Every feasible retained-DOP point lies below the oriented plane.
    Below,
    /// Every feasible retained-DOP point lies above the oriented plane.
    Above,
    /// The retained-DOP point set has at least one point on the plane.
    ///
    /// This includes strict crossings and tangential/coplanar contact. The
    /// report-bearing API keeps side feasibility witnesses when callers need to
    /// distinguish those cases at a higher object layer.
    Intersecting,
}

impl SupportDopPlaneRelation {
    /// Returns whether the retained DOP may touch or cross the plane.
    pub const fn intersects_plane(self) -> bool {
        matches!(self, Self::Intersecting)
    }
}

/// Intersection relation between two closed 3D balls, including their interiors.
///
/// The producer compares center-distance squared with the squared sum of the
/// nonnegative radii. It does not classify intersection of surface spheres or
/// distinguish containment and internal tangency.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SphereIntersection {
    /// The closed spheres have no point in common.
    Disjoint,
    /// Center distance equals the sum of radii (external contact, including zero-radius cases).
    Touching,
    /// Center distance is less than the sum of radii, including contained overlap.
    /// A contained zero-radius ball need not give positive intersection volume.
    Overlapping,
}

impl SphereIntersection {
    /// Returns whether the closed spheres intersect inclusively.
    pub const fn intersects(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Nearest-distance relation between a closed 3D AABB and a closed ball.
///
/// The producer compares the center's squared distance to the box with the
/// squared radius. Degenerate boxes and zero radius make the labels unsuitable
/// for inferring intersection dimension or positive volume.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AabbSphereIntersection {
    /// The box and sphere have no point in common.
    Disjoint,
    /// Nearest squared distance equals squared radius.
    /// This also labels a zero-radius ball whose center is anywhere in the box.
    Touching,
    /// Nearest squared distance is less than squared radius.
    /// At least one box point is in the ball's interior; the box may have zero volume.
    Overlapping,
}

impl AabbSphereIntersection {
    /// Returns whether the closed box and closed sphere intersect inclusively.
    pub const fn intersects(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Location of a point relative to a closed segment in 2D or 3D.
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
    /// The segments have the same two endpoints, in either order, including
    /// two identical degenerate point-segments.
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
    /// segment classifier used throughout computational geometry.
    pub const fn is_proper_crossing(self) -> bool {
        matches!(self, Self::Proper)
    }

    /// Returns whether the only certified contact is endpoint/boundary contact.
    pub const fn is_endpoint_touch(self) -> bool {
        matches!(self, Self::EndpointTouch)
    }

    /// Return true for CollinearOverlap or Identical.
    ///
    /// This proves positive-length overlap only when identical point-segments
    /// have been excluded separately. The enum does not retain segment length,
    /// so Identical returns true even for two equal zero-length inputs. Preserve
    /// source degeneracy evidence before using this helper as a dimension test.
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
    /// Coplanarity alone does not imply any shared point.
    Coplanar,
}

impl SegmentTriangleIntersection {
    /// Return true for every relation except Disjoint, retaining coplanar candidates.
    ///
    /// Coplanar returns true even when the segment lies outside the triangle.
    /// A planar overlap decision is still required to establish intersection.
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
    /// The ray lies on the triangle's supporting plane; overlap with the triangle
    /// has not been decided and the sets may be disjoint.
    Coplanar,
}

impl RayTriangleIntersection {
    /// Return true for every relation except Disjoint, retaining coplanar candidates.
    ///
    /// Coplanar is not an intersection witness; a separate planar ray/triangle
    /// query must decide whether any common point exists.
    pub const fn intersects(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Intersection relation between two closed 3D triangles.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleTriangleIntersection {
    /// At least one input triangle is degenerate, so no two-triangle area
    /// relation is asserted.
    Degenerate,
    /// The closed triangles have no point in common.
    Disjoint,
    /// The triangles meet along a non-coplanar set.
    ///
    /// This includes transverse line-segment intersections. Downstream
    /// arrangement builders can refine the retained edge reports into exact
    /// split events.
    NonCoplanarIntersection,
    /// The triangles meet only at certified boundary features.
    BoundaryTouch,
    /// The triangles are coplanar and disjoint after exact projection.
    CoplanarDisjoint,
    /// The triangles are coplanar and touch only on projected boundary.
    CoplanarTouching,
    /// The triangles are coplanar and overlap with positive projected area or
    /// a positive-length edge interval.
    CoplanarOverlapping,
}

impl TriangleTriangleIntersection {
    /// Returns whether the relation has at least one shared point.
    pub const fn intersects(self) -> bool {
        !matches!(
            self,
            Self::Degenerate | Self::Disjoint | Self::CoplanarDisjoint
        )
    }

    /// Returns whether the relation needs a downstream arrangement/split builder.
    pub const fn needs_construction(self) -> bool {
        matches!(
            self,
            Self::NonCoplanarIntersection
                | Self::BoundaryTouch
                | Self::CoplanarTouching
                | Self::CoplanarOverlapping
        )
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

/// Local turn consistency for a polygonal ring under the producer's policy.
///
/// This summarizes cyclic orientation signs, not polygon validity or absence
/// of self-intersections. The current ring classifier extracts child values
/// without retaining their certainty in this enum, so a result obtained under
/// an approximate policy is not by itself a certified turn-consistency proof.
/// Later visibility, containment and intersection still need their predicates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RingConvexity {
    /// Fewer than three vertices after removing a structurally repeated closing
    /// vertex, or every decided local turn is zero.
    Degenerate,
    /// Every decided nonzero local turn has the same sign and no turn is unknown.
    LocallyConvex,
    /// Visited nonzero turns contain both signs; the scan can stop before later turns.
    MixedTurns,
    /// A visited local turn remained unresolved under the requested policy.
    Unknown,
}

/// Location of a [`hyperreal::Real`] value relative to a closed Real interval.
///
/// The classifier orders the supplied endpoints before locating the value. If
/// both endpoints and the value coincide, AtLowerEndpoint takes precedence.
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
///
/// The current classifier's Overlapping variant also includes a zero-length
/// interval strictly inside another. Inspect source extents separately before
/// inferring positive-length intersection; retain the producer's certainty.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClosedIntervalIntersection {
    /// The closed intervals have no value in common.
    Disjoint,
    /// One normalized upper endpoint equals the other's lower endpoint.
    Touching,
    /// Neither normalized interval ends at or before the other's start.
    /// For positive-length inputs this implies more than one common value;
    /// a contained singleton interval also receives this label.
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
/// Direct variant-query witnesses; source geometry and degeneracy are not constructed here.
mod tests {
    use super::SegmentIntersection;

    #[test]
    /// Check helper truth tables, including Identical's unconditional overlap flag.
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
