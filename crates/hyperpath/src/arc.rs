//! Exact circular arc path carriers.
//!
//! Controller-level CAM and PCB routing both need arcs as first-class path
//! objects instead of tolerance-polylines. This module starts with cardinal
//! circular arcs, whose endpoints are exact axis points around a center. That
//! limited construction is still useful for G2/G3-style paths and gives
//! offset generation a precise radius update. The split follows Yap,
//! "Towards Exact Geometric Computation," and the arrangement/arc-kernel
//! discipline used by CGAL circular-arc traits: preserve curve structure first,
//! then ask exact predicates to certify topology.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Rational, Real, RealExactSetFacts, RealSign};

use crate::provenance::PathProvenance;
use crate::segment::{Axis, LinePathSegment};

/// Direction of traversal for a circular arc.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArcDirection {
    /// Counter-clockwise traversal.
    Ccw,
    /// Clockwise traversal.
    Cw,
}

/// Cardinal point on a circle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CardinalPoint {
    /// Positive X direction from the center.
    East,
    /// Positive Y direction from the center.
    North,
    /// Negative X direction from the center.
    West,
    /// Negative Y direction from the center.
    South,
}

/// Exact sweep classification for an explicit circular arc.
///
/// This is deliberately a topological fact, not a numeric angle. It is derived
/// from exact radial-vector cross products, following Yap's predicate-first
/// exact geometry model and the circular-arc orientation tests used by exact
/// arrangement kernels.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExplicitArcSweepClass {
    /// Start and end are the same retained point, representing a full circle.
    FullCircle,
    /// Directed sweep is certified less than a half turn.
    LessThanHalfTurn,
    /// Start and end are certified antipodal.
    HalfTurn,
    /// Directed sweep is certified greater than a half turn.
    GreaterThanHalfTurn,
    /// The retained facts did not certify the sweep class.
    Unknown,
}

/// Exact point-membership classification for an explicit circular arc.
///
/// This is an arrangement predicate result, not a sampled curve query. The
/// point is first checked against the retained circle equation, then directed
/// radial cross products decide whether the point lies inside the certified
/// sweep. The construction follows Yap's exact-geometric-computation split
/// and the circular-arc predicate style used by exact arrangement kernels such
/// as CGAL: topology is certified from retained algebraic structure, not from
/// approximate angles.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExplicitArcPointClassification {
    /// The point is certified on the retained arc sweep.
    OnArc,
    /// The point is certified on the retained circle but outside this sweep.
    OnCircleOutsideSweep,
    /// The point is certified off the retained circle.
    OffCircle,
    /// Circle incidence or sweep membership could not be certified.
    Unknown,
}

/// Exact intersection class for an axis-aligned segment and explicit arc.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineExplicitArcIntersectionClass {
    /// The segment and retained arc are certified disjoint.
    Disjoint,
    /// The segment touches the arc at exactly one certified point.
    Tangent,
    /// The segment crosses the arc at two certified points.
    Secant,
    /// The current exact package cannot certify the relation.
    Unknown,
}

/// Exact intersection report for an axis-aligned segment and explicit arc.
///
/// This is the first line/arc arrangement handoff for `hyperpath`. It uses the
/// retained axis alignment to solve the line/circle equation exactly, then
/// filters candidate points through segment bounds and
/// [`ExplicitCircularArc::classify_point`]. General line/circle quadratics and
/// full arc/arc arrangements remain later kernels; this report refuses to
/// choose topology from sampled coordinates. That is the Yap boundary and the
/// same object-then-predicate discipline used by CGAL circular-arc
/// arrangements.
#[derive(Clone, Debug, PartialEq)]
pub struct LineExplicitArcIntersectionReport {
    /// Certified relation class.
    pub class: LineExplicitArcIntersectionClass,
    /// Certified intersection points in construction order.
    pub points: Vec<Point2>,
}

/// Exact overlap class for two explicit circular arcs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExplicitArcOverlapClass {
    /// The retained circles are certified different.
    DifferentCircle,
    /// The two retained arc sweeps are certified disjoint.
    Disjoint,
    /// The arcs share only retained endpoints.
    EndpointTouch,
    /// The arcs have a nonzero-length common sweep, but neither covers the other.
    Overlap,
    /// The first arc covers the second arc.
    FirstCoversSecond,
    /// The second arc covers the first arc.
    SecondCoversFirst,
    /// The retained arcs are certified equal.
    Equal,
    /// The current exact facts cannot decide the relation.
    Unknown,
}

/// Exact same-circle explicit-arc overlap report.
///
/// This is a deliberately conservative arrangement predicate. It first
/// certifies that the circles match exactly, then uses endpoint equality and
/// [`ExplicitCircularArc::classify_point`] to classify interval overlap on the
/// retained circle. It does not sample angles or flatten arcs. Ambiguous
/// complementary-endpoint cases are reported as endpoint-only contact unless a
/// retained endpoint lies in the other arc's interior, preserving Yap's rule
/// that uncertain topology stays explicit.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitArcOverlapReport {
    /// Certified overlap relation.
    pub class: ExplicitArcOverlapClass,
    /// Endpoints shared exactly by the retained arcs.
    pub shared_endpoints: Vec<Point2>,
}

/// Exact relation between two retained explicit-arc circles.
///
/// This is a circle-level arrangement predicate, not an arc witness
/// constructor. It uses squared center distance and squared radius sums/
/// differences, so tangent and secant decisions are made without square-root
/// normalization. That follows Yap's exact predicate discipline and matches
/// the first relation stage used by exact circular-arc arrangement kernels
/// such as CGAL before they materialize algebraic intersection points.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExplicitCircleRelationClass {
    /// Centers and radii are certified equal.
    SameCircle,
    /// The circles are certified externally disjoint.
    Separate,
    /// The circles touch at one external tangent point.
    ExternallyTangent,
    /// The circles cross at two points.
    Secant,
    /// One circle touches the other internally at one point.
    InternallyTangent,
    /// One circle is strictly inside the other.
    Contained,
    /// The current exact comparisons cannot decide the relation.
    Unknown,
}

/// Exact circle-level relation report for two explicit arcs.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitCircleRelationReport {
    /// Certified relation class.
    pub class: ExplicitCircleRelationClass,
    /// Exact squared distance between retained centers.
    pub center_distance_squared: Real,
    /// Exact `(r0 + r1)^2`.
    pub radius_sum_squared: Real,
    /// Exact `(r0 - r1)^2`.
    pub radius_difference_squared: Real,
    /// Exact tangent point for external/internal tangent circles when available.
    pub tangent_point: Option<Point2>,
}

/// Exact tangent relation between two retained explicit arcs.
///
/// This composes the retained circle-circle tangent predicate with
/// [`ExplicitCircularArc::classify_point`] on the exact tangent witness. It is
/// intentionally narrower than a full arc/arc intersection kernel: secants
/// still require algebraic witness construction, while tangent circles have an
/// affine witness on the center line. That mirrors Yap's predicate-first
/// exact-computation model and the circular-arc arrangement staging used by
/// CGAL, where topology is certified before witness materialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExplicitArcTangentClass {
    /// The retained circles are tangent and the exact tangent point lies on both arc sweeps.
    TangentOnBoth,
    /// The retained circles are tangent, but the tangent point is outside at least one sweep.
    CircleTangentOutsideArcSweep,
    /// The retained circles are not externally or internally tangent.
    NotCircleTangent,
    /// The current exact comparisons cannot decide the relation.
    Unknown,
}

/// Exact arc-level tangent report for two explicit arcs.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitArcTangentReport {
    /// Certified arc-level tangent class.
    pub class: ExplicitArcTangentClass,
    /// Underlying retained circle relation.
    pub circle_relation: ExplicitCircleRelationClass,
    /// Exact affine tangent witness when the circle relation supplies one.
    pub tangent_point: Option<Point2>,
}

/// Exact point-intersection class for two explicit circular arcs.
///
/// Same-circle arcs are deliberately split out because their relation is
/// interval overlap, not isolated point intersection. For different circles,
/// the class is certified from the retained circle relation and exact
/// point-on-arc predicates. This follows Yap's exact-geometric-computation
/// boundary and the radical-axis witness stage used by exact circular-arc
/// arrangement kernels such as CGAL.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExplicitArcIntersectionClass {
    /// The arcs lie on the same retained circle; use same-circle overlap instead.
    SameCircle,
    /// The retained circles/arcs are certified disjoint.
    Disjoint,
    /// Exactly one retained intersection point lies on both arc sweeps.
    OnePoint,
    /// Two retained intersection points lie on both arc sweeps.
    TwoPoints,
    /// The retained circles intersect, but their witness points are outside the arc sweeps.
    CircleIntersectionsOutsideArcSweeps,
    /// The current exact package cannot certify the relation or witnesses.
    Unknown,
}

/// Exact point-intersection report for two explicit circular arcs.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitArcIntersectionReport {
    /// Certified point-intersection class.
    pub class: ExplicitArcIntersectionClass,
    /// Underlying retained circle relation.
    pub circle_relation: ExplicitCircleRelationClass,
    /// Certified point witnesses that lie on both arc sweeps.
    pub points: Vec<Point2>,
}

/// Exact arrangement class for two explicit circular arcs.
///
/// This is the public scheduling predicate for arc/arc arrangement. It does
/// not invent new topology; it dispatches to same-circle overlap or
/// different-circle point intersection and records which exact predicate path
/// certified the result. Yap's "Towards Exact Geometric Computation" frames
/// this as an object/predicate split: the scheduler chooses certified
/// topology, while later output kernels materialize regions or controller
/// moves only from the certified report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExplicitArcArrangementClass {
    /// Same retained circle with no common sweep points.
    SameCircleDisjoint,
    /// Same retained circle with endpoint-only contact.
    SameCircleEndpointTouch,
    /// Same retained circle with nonzero-length overlap.
    SameCircleOverlap,
    /// Same retained circle where the first arc covers the second.
    SameCircleFirstCoversSecond,
    /// Same retained circle where the second arc covers the first.
    SameCircleSecondCoversFirst,
    /// Same retained circle and equal retained arc sweep.
    SameCircleEqual,
    /// Different retained circles/arcs with no intersection points.
    DifferentCircleDisjoint,
    /// Different retained circles with one certified arc intersection point.
    DifferentCircleOnePoint,
    /// Different retained circles with two certified arc intersection points.
    DifferentCircleTwoPoints,
    /// Different retained circles intersect, but all circle witnesses are outside the arc sweeps.
    DifferentCircleOutsideArcSweeps,
    /// The current exact predicates cannot decide the arrangement.
    Unknown,
}

/// Exact arrangement report for two explicit circular arcs.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitArcArrangementReport {
    /// Certified arrangement class.
    pub class: ExplicitArcArrangementClass,
    /// Same-circle overlap report when the retained circles are equal.
    pub overlap: Option<ExplicitArcOverlapReport>,
    /// Different-circle point-intersection report when retained circles differ.
    pub intersection: Option<ExplicitArcIntersectionReport>,
}

/// Cached facts for a cardinal circular arc.
#[derive(Clone, Debug, PartialEq)]
pub struct CircularArcFacts {
    /// Exact-set facts across center coordinates and radius.
    pub exact: RealExactSetFacts,
    /// Squared radius as an exact scalar expression.
    pub radius_squared: Real,
    /// Number of quarter turns in the directed arc.
    pub quarter_turns: u8,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Cached facts for an explicit-endpoint circular arc.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitCircularArcFacts {
    /// Exact-set facts across center, endpoints, and radius.
    pub exact: RealExactSetFacts,
    /// Squared radius as an exact scalar expression.
    pub radius_squared: Real,
    /// Squared chord length from start to end.
    pub chord_length_squared: Real,
    /// Exact dot product of start and end radial vectors.
    pub radial_dot: Real,
    /// Exact cross product of start and end radial vectors.
    pub radial_cross: Real,
    /// Exact sweep class certified from radial-vector signs.
    pub sweep_class: ExplicitArcSweepClass,
    /// Whether start and end are structurally the same point.
    pub known_full_circle: bool,
    /// Source provenance.
    pub provenance: PathProvenance,
}

/// Exact cardinal circular arc.
#[derive(Clone, Debug, PartialEq)]
pub struct CircularArc {
    center: Point2,
    radius: Real,
    start_cardinal: CardinalPoint,
    end_cardinal: CardinalPoint,
    direction: ArcDirection,
    facts: CircularArcFacts,
}

/// Exact circular arc with explicit endpoints on a retained circle.
///
/// This carrier is the non-cardinal companion to [`CircularArc`]. It preserves
/// the exact center, radius, endpoints, and direction without deriving an
/// approximate angle. That matches Yap's exact-geometric-computation boundary
/// and CGAL's circular-arc kernel discipline: store the object exactly first,
/// then let exact predicates decide arrangement/topology later.
#[derive(Clone, Debug, PartialEq)]
pub struct ExplicitCircularArc {
    center: Point2,
    radius: Real,
    start: Point2,
    end: Point2,
    direction: ArcDirection,
    facts: ExplicitCircularArcFacts,
}

/// Errors while constructing exact circular arcs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CircularArcError {
    /// Radius was structurally negative.
    NegativeRadius,
    /// Radius was structurally zero.
    DegenerateRadius,
    /// Start endpoint is not exactly on the retained circle.
    StartPointOffCircle,
    /// End endpoint is not exactly on the retained circle.
    EndPointOffCircle,
}

impl CircularArc {
    /// Construct a cardinal circular arc with native provenance.
    pub fn cardinal(
        center: Point2,
        radius: Real,
        start: CardinalPoint,
        end: CardinalPoint,
        direction: ArcDirection,
    ) -> Result<Self, CircularArcError> {
        Self::cardinal_with_provenance(
            center,
            radius,
            start,
            end,
            direction,
            PathProvenance::native(),
        )
    }

    /// Construct a cardinal circular arc with source provenance.
    ///
    /// Equal start and end cardinal points represent a full circle in the
    /// chosen direction. This keeps full-circle drill/routing and contour arcs
    /// expressible without inventing an approximate angle representation.
    pub fn cardinal_with_provenance(
        center: Point2,
        radius: Real,
        start_cardinal: CardinalPoint,
        end_cardinal: CardinalPoint,
        direction: ArcDirection,
        provenance: PathProvenance,
    ) -> Result<Self, CircularArcError> {
        match radius.structural_facts().sign {
            Some(RealSign::Negative) => return Err(CircularArcError::NegativeRadius),
            Some(RealSign::Zero) => return Err(CircularArcError::DegenerateRadius),
            _ => {}
        }
        let facts = CircularArcFacts {
            exact: Real::exact_set_facts([&center.x, &center.y, &radius]),
            radius_squared: radius.clone() * radius.clone(),
            quarter_turns: quarter_turns(start_cardinal, end_cardinal, direction),
            provenance,
        };
        Ok(Self {
            center,
            radius,
            start_cardinal,
            end_cardinal,
            direction,
            facts,
        })
    }

    /// Return exact center.
    pub const fn center(&self) -> &Point2 {
        &self.center
    }

    /// Return exact radius.
    pub const fn radius(&self) -> &Real {
        &self.radius
    }

    /// Return start cardinal point.
    pub const fn start_cardinal(&self) -> CardinalPoint {
        self.start_cardinal
    }

    /// Return end cardinal point.
    pub const fn end_cardinal(&self) -> CardinalPoint {
        self.end_cardinal
    }

    /// Return traversal direction.
    pub const fn direction(&self) -> ArcDirection {
        self.direction
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &CircularArcFacts {
        &self.facts
    }

    /// Return path source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }

    /// Return exact start point.
    pub fn start(&self) -> Point2 {
        cardinal_point(&self.center, &self.radius, self.start_cardinal)
    }

    /// Return exact end point.
    pub fn end(&self) -> Point2 {
        cardinal_point(&self.center, &self.radius, self.end_cardinal)
    }

    /// Return exact chord length squared.
    pub fn chord_length_squared(&self) -> Real {
        let start = self.start();
        let end = self.end();
        let dx = end.x - start.x;
        let dy = end.y - start.y;
        Real::signed_product_sum([true, true], [[&dx, &dx], [&dy, &dy]])
    }

    /// Return the exact tangent vector at the start point.
    ///
    /// The vector is radius-scaled, not unit-normalized: for radial vector
    /// `(rx, ry)`, the CCW tangent is `(-ry, rx)` and the CW tangent is
    /// `(ry, -rx)`. Exact arrangement kernels such as CGAL's circular-arc
    /// traits use this radial/tangent structure for local ordering; exposing it
    /// directly also gives `hypersolve` tangent residuals a Yap-style exact
    /// object instead of a sampled finite-difference direction.
    pub fn start_tangent(&self) -> Point2 {
        tangent_from_radial(
            &(self.start().x - self.center.x.clone()),
            &(self.start().y - self.center.y.clone()),
            self.direction,
        )
    }

    /// Return the exact tangent vector at the end point.
    pub fn end_tangent(&self) -> Point2 {
        tangent_from_radial(
            &(self.end().x - self.center.x.clone()),
            &(self.end().y - self.center.y.clone()),
            self.direction,
        )
    }

    /// Return exact arc length for this cardinal arc.
    ///
    /// The value is `radius * pi * quarter_turns / 2`. It remains an exact
    /// `Real` expression because `pi` is represented symbolically by
    /// `hyperreal`, not rounded to a primitive float.
    pub fn exact_length(&self) -> Real {
        let factor = Real::new(Rational::fraction(i64::from(self.facts.quarter_turns), 2).unwrap());
        self.radius.clone() * Real::pi() * factor
    }
}

impl ExplicitCircularArc {
    /// Construct an explicit circular arc with native provenance.
    pub fn new(
        center: Point2,
        radius: Real,
        start: Point2,
        end: Point2,
        direction: ArcDirection,
    ) -> Result<Self, CircularArcError> {
        Self::with_provenance(
            center,
            radius,
            start,
            end,
            direction,
            PathProvenance::native(),
        )
    }

    /// Construct an explicit circular arc with source provenance.
    ///
    /// Both endpoints must satisfy the exact circle equation
    /// `(x - cx)^2 + (y - cy)^2 = radius^2`. The constructor rejects off-circle
    /// endpoints instead of projecting them, because projection would be a
    /// numeric repair step outside Yap's object construction boundary.
    pub fn with_provenance(
        center: Point2,
        radius: Real,
        start: Point2,
        end: Point2,
        direction: ArcDirection,
        provenance: PathProvenance,
    ) -> Result<Self, CircularArcError> {
        match radius.structural_facts().sign {
            Some(RealSign::Negative) => return Err(CircularArcError::NegativeRadius),
            Some(RealSign::Zero) => return Err(CircularArcError::DegenerateRadius),
            _ => {}
        }
        let radius_squared = radius.clone() * radius.clone();
        if point_radius_squared(&center, &start) != radius_squared {
            return Err(CircularArcError::StartPointOffCircle);
        }
        if point_radius_squared(&center, &end) != radius_squared {
            return Err(CircularArcError::EndPointOffCircle);
        }
        let chord_length_squared = point_distance_squared(&start, &end);
        let known_full_circle = chord_length_squared == Real::zero();
        let (radial_dot, radial_cross) = radial_dot_cross(&center, &start, &end);
        let sweep_class = classify_explicit_sweep(&radial_cross, known_full_circle, direction);
        let facts = ExplicitCircularArcFacts {
            exact: Real::exact_set_facts([
                &center.x, &center.y, &radius, &start.x, &start.y, &end.x, &end.y,
            ]),
            radius_squared,
            chord_length_squared,
            radial_dot,
            radial_cross,
            sweep_class,
            known_full_circle,
            provenance,
        };
        Ok(Self {
            center,
            radius,
            start,
            end,
            direction,
            facts,
        })
    }

    /// Return exact center.
    pub const fn center(&self) -> &Point2 {
        &self.center
    }

    /// Return exact radius.
    pub const fn radius(&self) -> &Real {
        &self.radius
    }

    /// Return exact start point.
    pub const fn start(&self) -> &Point2 {
        &self.start
    }

    /// Return exact end point.
    pub const fn end(&self) -> &Point2 {
        &self.end
    }

    /// Return traversal direction.
    pub const fn direction(&self) -> ArcDirection {
        self.direction
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &ExplicitCircularArcFacts {
        &self.facts
    }

    /// Return path source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.facts.provenance
    }

    /// Return exact chord length squared.
    pub fn chord_length_squared(&self) -> Real {
        self.facts.chord_length_squared.clone()
    }

    /// Return the exact tangent vector at the start point.
    ///
    /// This is the non-cardinal companion to [`CircularArc::start_tangent`].
    /// It rotates the retained radial vector exactly and does not evaluate a
    /// trigonometric angle or normalize by a square root.
    pub fn start_tangent(&self) -> Point2 {
        tangent_from_radial(
            &(self.start.x.clone() - self.center.x.clone()),
            &(self.start.y.clone() - self.center.y.clone()),
            self.direction,
        )
    }

    /// Return the exact tangent vector at the end point.
    pub fn end_tangent(&self) -> Point2 {
        tangent_from_radial(
            &(self.end.x.clone() - self.center.x.clone()),
            &(self.end.y.clone() - self.center.y.clone()),
            self.direction,
        )
    }

    /// Return exact arc length when the retained sweep certifies it.
    ///
    /// Full circles and half turns have exact symbolic lengths `2*pi*r` and
    /// `pi*r`. Minor and major non-cardinal arcs use the retained radial dot
    /// product to build the analytic central angle `acos(dot / r^2)` or its
    /// complement `2*pi - acos(dot / r^2)`. The angle remains a symbolic
    /// `Real`, not an approximate primitive float. This follows Yap's exact
    /// construction discipline: preserve the curve equation and certify the
    /// metric expression from retained facts, reporting `None` if the exact
    /// division or inverse-trig domain cannot be represented.
    pub fn certified_sweep_length(&self) -> Option<Real> {
        match self.facts.sweep_class {
            ExplicitArcSweepClass::FullCircle => {
                Some(self.radius.clone() * Real::pi() * Real::from(2))
            }
            ExplicitArcSweepClass::HalfTurn => Some(self.radius.clone() * Real::pi()),
            ExplicitArcSweepClass::LessThanHalfTurn => {
                Some(self.radius.clone() * explicit_arc_minor_angle(self)?)
            }
            ExplicitArcSweepClass::GreaterThanHalfTurn => {
                let minor = explicit_arc_minor_angle(self)?;
                Some(self.radius.clone() * (Real::pi() * Real::from(2) - minor))
            }
            ExplicitArcSweepClass::Unknown => None,
        }
    }

    /// Classify a point against this exact arc sweep.
    ///
    /// No trigonometric angle is evaluated. For minor arcs the point must lie
    /// to the directed left of the start radial and to the directed left of
    /// the point-to-end radial. For major arcs the complementary minor wedge is
    /// rejected instead. Half-turns use the directed start radial as the exact
    /// separating line. This gives later line/arc and arc/arc arrangement code
    /// a Yap-style predicate boundary before any export or flattening stage.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: PredicatePolicy,
    ) -> ExplicitArcPointClassification {
        match compare_reals_with_policy(
            &point_radius_squared(&self.center, point),
            &self.facts.radius_squared,
            policy,
        )
        .value()
        {
            Some(Ordering::Less | Ordering::Greater) => {
                return ExplicitArcPointClassification::OffCircle;
            }
            Some(Ordering::Equal) => {}
            None => return ExplicitArcPointClassification::Unknown,
        }

        match self.facts.sweep_class {
            ExplicitArcSweepClass::FullCircle => ExplicitArcPointClassification::OnArc,
            ExplicitArcSweepClass::Unknown => ExplicitArcPointClassification::Unknown,
            ExplicitArcSweepClass::LessThanHalfTurn => {
                let Some(start_to_point) =
                    directed_cross_sign(&self.center, &self.start, point, self.direction, policy)
                else {
                    return ExplicitArcPointClassification::Unknown;
                };
                let Some(point_to_end) =
                    directed_cross_sign(&self.center, point, &self.end, self.direction, policy)
                else {
                    return ExplicitArcPointClassification::Unknown;
                };
                if is_nonnegative(start_to_point) && is_nonnegative(point_to_end) {
                    ExplicitArcPointClassification::OnArc
                } else {
                    ExplicitArcPointClassification::OnCircleOutsideSweep
                }
            }
            ExplicitArcSweepClass::HalfTurn => {
                let Some(start_to_point) =
                    directed_cross_sign(&self.center, &self.start, point, self.direction, policy)
                else {
                    return ExplicitArcPointClassification::Unknown;
                };
                if is_nonnegative(start_to_point) {
                    ExplicitArcPointClassification::OnArc
                } else {
                    ExplicitArcPointClassification::OnCircleOutsideSweep
                }
            }
            ExplicitArcSweepClass::GreaterThanHalfTurn => {
                let Some(end_to_point) =
                    directed_cross_sign(&self.center, &self.end, point, self.direction, policy)
                else {
                    return ExplicitArcPointClassification::Unknown;
                };
                let Some(point_to_start) =
                    directed_cross_sign(&self.center, point, &self.start, self.direction, policy)
                else {
                    return ExplicitArcPointClassification::Unknown;
                };
                if is_positive(end_to_point) && is_positive(point_to_start) {
                    ExplicitArcPointClassification::OnCircleOutsideSweep
                } else {
                    ExplicitArcPointClassification::OnArc
                }
            }
        }
    }

    /// Intersect this arc with a certified axis-aligned line segment.
    ///
    /// The method solves the retained line/circle equation on the segment's
    /// fixed coordinate. Candidate roots are then accepted only when exact
    /// segment-bound comparisons and exact arc-sweep membership both certify
    /// them. Non-axis-aligned segments, negative radicands with undecided sign,
    /// and square-root failures are reported as [`LineExplicitArcIntersectionClass::Unknown`]
    /// rather than using an approximate fallback.
    pub fn intersect_axis_aligned_segment(
        &self,
        segment: &LinePathSegment,
        policy: PredicatePolicy,
    ) -> LineExplicitArcIntersectionReport {
        let Some(axis) = segment.facts().axis_aligned else {
            return line_arc_unknown_report();
        };
        let fixed_delta = match axis {
            Axis::X => segment.start().y.clone() - self.center.y.clone(),
            Axis::Y => segment.start().x.clone() - self.center.x.clone(),
        };
        let radicand = self.facts.radius_squared.clone() - fixed_delta.clone() * fixed_delta;
        let Some(radicand_order) =
            compare_reals_with_policy(&radicand, &Real::zero(), policy).value()
        else {
            return line_arc_unknown_report();
        };
        if radicand_order == Ordering::Less {
            return line_arc_report(Vec::new());
        }

        let mut candidates = Vec::new();
        if radicand_order == Ordering::Equal {
            candidates.push(point_on_axis_line(
                axis,
                segment,
                &self.center,
                Real::zero(),
            ));
        } else {
            let Ok(root) = radicand.sqrt() else {
                return line_arc_unknown_report();
            };
            candidates.push(point_on_axis_line(
                axis,
                segment,
                &self.center,
                root.clone(),
            ));
            candidates.push(point_on_axis_line(axis, segment, &self.center, -root));
        }

        let mut accepted = Vec::new();
        for candidate in candidates {
            match point_inside_segment_bounds(&candidate, segment, policy) {
                Some(true) => {}
                Some(false) => continue,
                None => return line_arc_unknown_report(),
            }
            match self.classify_point(&candidate, policy) {
                ExplicitArcPointClassification::OnArc => {
                    push_unique_point(&mut accepted, candidate, policy);
                }
                ExplicitArcPointClassification::OnCircleOutsideSweep
                | ExplicitArcPointClassification::OffCircle => {}
                ExplicitArcPointClassification::Unknown => return line_arc_unknown_report(),
            }
        }
        line_arc_report(accepted)
    }

    /// Classify overlap with another explicit arc on the same retained circle.
    ///
    /// Full circles are handled directly. For finite sweeps, the predicate
    /// checks which retained endpoints lie on the other arc and which of those
    /// are strict interior points rather than shared endpoints. This gives
    /// arrangement code a cheap exact overlap classifier for common routing and
    /// toolpath cases while leaving general arc/arc algebra to a later kernel.
    pub fn classify_same_circle_overlap(
        &self,
        other: &ExplicitCircularArc,
        policy: PredicatePolicy,
    ) -> ExplicitArcOverlapReport {
        if !same_circle(self, other, policy).unwrap_or(false) {
            return ExplicitArcOverlapReport {
                class: ExplicitArcOverlapClass::DifferentCircle,
                shared_endpoints: Vec::new(),
            };
        }

        let shared_endpoints = shared_arc_endpoints(self, other, policy);
        if self.facts.known_full_circle && other.facts.known_full_circle {
            return ExplicitArcOverlapReport {
                class: ExplicitArcOverlapClass::Equal,
                shared_endpoints,
            };
        }
        if self.facts.known_full_circle {
            return ExplicitArcOverlapReport {
                class: ExplicitArcOverlapClass::FirstCoversSecond,
                shared_endpoints,
            };
        }
        if other.facts.known_full_circle {
            return ExplicitArcOverlapReport {
                class: ExplicitArcOverlapClass::SecondCoversFirst,
                shared_endpoints,
            };
        }
        if same_arc_orientation_and_endpoints(self, other, policy).unwrap_or(false) {
            return ExplicitArcOverlapReport {
                class: ExplicitArcOverlapClass::Equal,
                shared_endpoints,
            };
        }

        let Some(first_start_on_second) = point_on_arc_bool(other, &self.start, policy) else {
            return arc_overlap_unknown_report();
        };
        let Some(first_end_on_second) = point_on_arc_bool(other, &self.end, policy) else {
            return arc_overlap_unknown_report();
        };
        let Some(second_start_on_first) = point_on_arc_bool(self, &other.start, policy) else {
            return arc_overlap_unknown_report();
        };
        let Some(second_end_on_first) = point_on_arc_bool(self, &other.end, policy) else {
            return arc_overlap_unknown_report();
        };

        let first_interior_on_second = (first_start_on_second
            && !point_is_arc_endpoint(other, &self.start, policy))
            || (first_end_on_second && !point_is_arc_endpoint(other, &self.end, policy));
        let second_interior_on_first = (second_start_on_first
            && !point_is_arc_endpoint(self, &other.start, policy))
            || (second_end_on_first && !point_is_arc_endpoint(self, &other.end, policy));

        let class = if second_start_on_first
            && second_end_on_first
            && !first_interior_on_second
            && !same_arc_orientation_and_endpoints(self, other, policy).unwrap_or(false)
        {
            ExplicitArcOverlapClass::FirstCoversSecond
        } else if first_start_on_second
            && first_end_on_second
            && !second_interior_on_first
            && !same_arc_orientation_and_endpoints(self, other, policy).unwrap_or(false)
        {
            ExplicitArcOverlapClass::SecondCoversFirst
        } else if first_interior_on_second || second_interior_on_first {
            ExplicitArcOverlapClass::Overlap
        } else if !shared_endpoints.is_empty() {
            ExplicitArcOverlapClass::EndpointTouch
        } else {
            ExplicitArcOverlapClass::Disjoint
        };
        ExplicitArcOverlapReport {
            class,
            shared_endpoints,
        }
    }

    /// Classify this arc's retained circle against another retained circle.
    ///
    /// The result intentionally stops at the relation class. Secant witness
    /// construction can require algebraic coordinates not yet representable as
    /// ordinary `Point2` values in all cases, so this predicate provides the
    /// exact topological dispatch needed before a later witness kernel. Equal
    /// circles should be handled by [`Self::classify_same_circle_overlap`].
    pub fn classify_circle_relation(
        &self,
        other: &ExplicitCircularArc,
        policy: PredicatePolicy,
    ) -> ExplicitCircleRelationReport {
        let center_distance_squared = point_distance_squared(&self.center, &other.center);
        let radius_sum = self.radius.clone() + other.radius.clone();
        let radius_sum_squared = radius_sum.clone() * radius_sum;
        let radius_difference = self.radius.clone() - other.radius.clone();
        let radius_difference_squared = radius_difference.clone() * radius_difference;

        let class = classify_circle_relation_from_squares(
            &center_distance_squared,
            &radius_sum_squared,
            &radius_difference_squared,
            same_circle(self, other, policy),
            policy,
        );
        ExplicitCircleRelationReport {
            class,
            center_distance_squared,
            radius_sum_squared,
            radius_difference_squared,
            tangent_point: circle_tangent_point(self, other, class),
        }
    }

    /// Classify whether a retained circle tangent is also an arc tangent.
    ///
    /// The method first certifies the circle-circle relation using squared
    /// distances. For internal/external tangent circles it then checks the
    /// exact tangent witness against both explicit arc sweeps. No angular
    /// parameterization, tolerance, or sampled fallback is used; undecided
    /// circle or sweep comparisons stay [`ExplicitArcTangentClass::Unknown`].
    pub fn classify_tangent_intersection(
        &self,
        other: &ExplicitCircularArc,
        policy: PredicatePolicy,
    ) -> ExplicitArcTangentReport {
        let circle_report = self.classify_circle_relation(other, policy);
        match circle_report.class {
            ExplicitCircleRelationClass::ExternallyTangent
            | ExplicitCircleRelationClass::InternallyTangent => {
                let Some(tangent_point) = circle_report.tangent_point.clone() else {
                    return ExplicitArcTangentReport {
                        class: ExplicitArcTangentClass::Unknown,
                        circle_relation: circle_report.class,
                        tangent_point: None,
                    };
                };
                let first = self.classify_point(&tangent_point, policy);
                let second = other.classify_point(&tangent_point, policy);
                let class = match (first, second) {
                    (
                        ExplicitArcPointClassification::OnArc,
                        ExplicitArcPointClassification::OnArc,
                    ) => ExplicitArcTangentClass::TangentOnBoth,
                    (ExplicitArcPointClassification::Unknown, _)
                    | (_, ExplicitArcPointClassification::Unknown) => {
                        ExplicitArcTangentClass::Unknown
                    }
                    _ => ExplicitArcTangentClass::CircleTangentOutsideArcSweep,
                };
                ExplicitArcTangentReport {
                    class,
                    circle_relation: circle_report.class,
                    tangent_point: Some(tangent_point),
                }
            }
            ExplicitCircleRelationClass::Unknown => ExplicitArcTangentReport {
                class: ExplicitArcTangentClass::Unknown,
                circle_relation: circle_report.class,
                tangent_point: circle_report.tangent_point,
            },
            _ => ExplicitArcTangentReport {
                class: ExplicitArcTangentClass::NotCircleTangent,
                circle_relation: circle_report.class,
                tangent_point: circle_report.tangent_point,
            },
        }
    }

    /// Intersect two explicit arcs as retained circle arcs.
    ///
    /// Same-circle arcs are reported as [`ExplicitArcIntersectionClass::SameCircle`]
    /// so callers can use [`Self::classify_same_circle_overlap`] for interval
    /// topology. For different circles, tangent witnesses come from the affine
    /// center-line construction and secant witnesses use the radical-axis
    /// formula:
    ///
    /// `p = c0 + ((r0^2 - r1^2 + d^2) / (2 d^2)) (c1 - c0)
    ///      +/- sqrt(4 d^2 r0^2 - (r0^2 - r1^2 + d^2)^2) / (2 d^2) R90(c1 - c0)`.
    ///
    /// This keeps the square root isolated in the perpendicular offset and
    /// avoids converting through approximate angles or normalized floating
    /// vectors. If the exact `Real` package cannot materialize or compare a
    /// witness, the result remains [`ExplicitArcIntersectionClass::Unknown`],
    /// preserving Yap's "decide exactly or say unknown" rule.
    pub fn intersect_arc(
        &self,
        other: &ExplicitCircularArc,
        policy: PredicatePolicy,
    ) -> ExplicitArcIntersectionReport {
        let circle_report = self.classify_circle_relation(other, policy);
        match circle_report.class {
            ExplicitCircleRelationClass::SameCircle => ExplicitArcIntersectionReport {
                class: ExplicitArcIntersectionClass::SameCircle,
                circle_relation: circle_report.class,
                points: shared_arc_endpoints(self, other, policy),
            },
            ExplicitCircleRelationClass::Separate | ExplicitCircleRelationClass::Contained => {
                ExplicitArcIntersectionReport {
                    class: ExplicitArcIntersectionClass::Disjoint,
                    circle_relation: circle_report.class,
                    points: Vec::new(),
                }
            }
            ExplicitCircleRelationClass::ExternallyTangent
            | ExplicitCircleRelationClass::InternallyTangent => {
                let Some(tangent_point) = circle_report.tangent_point else {
                    return arc_intersection_unknown_report(circle_report.class);
                };
                let Some(on_both) = point_on_both_arcs(self, other, &tangent_point, policy) else {
                    return arc_intersection_unknown_report(circle_report.class);
                };
                if on_both {
                    ExplicitArcIntersectionReport {
                        class: ExplicitArcIntersectionClass::OnePoint,
                        circle_relation: circle_report.class,
                        points: vec![tangent_point],
                    }
                } else {
                    ExplicitArcIntersectionReport {
                        class: ExplicitArcIntersectionClass::CircleIntersectionsOutsideArcSweeps,
                        circle_relation: circle_report.class,
                        points: Vec::new(),
                    }
                }
            }
            ExplicitCircleRelationClass::Secant => {
                let Some(candidates) = circle_secant_points(self, other, policy) else {
                    return arc_intersection_unknown_report(circle_report.class);
                };
                let mut accepted = Vec::new();
                for candidate in candidates {
                    let Some(on_both) = point_on_both_arcs(self, other, &candidate, policy) else {
                        return arc_intersection_unknown_report(circle_report.class);
                    };
                    if on_both {
                        push_unique_point(&mut accepted, candidate, policy);
                    }
                }
                let class = match accepted.len() {
                    0 => ExplicitArcIntersectionClass::CircleIntersectionsOutsideArcSweeps,
                    1 => ExplicitArcIntersectionClass::OnePoint,
                    2 => ExplicitArcIntersectionClass::TwoPoints,
                    _ => ExplicitArcIntersectionClass::Unknown,
                };
                ExplicitArcIntersectionReport {
                    class,
                    circle_relation: circle_report.class,
                    points: accepted,
                }
            }
            ExplicitCircleRelationClass::Unknown => {
                arc_intersection_unknown_report(circle_report.class)
            }
        }
    }

    /// Schedule exact arc/arc arrangement using retained predicate reports.
    ///
    /// Equal retained circles are classified by
    /// [`Self::classify_same_circle_overlap`]. Different circles are
    /// classified by [`Self::intersect_arc`], which handles disjoint,
    /// tangent, and secant point witnesses. The method keeps both source
    /// reports available so downstream CAM/EDA code can choose whether it
    /// needs point contacts, overlap intervals, or conservative `Unknown`
    /// propagation.
    pub fn arrange_with(
        &self,
        other: &ExplicitCircularArc,
        policy: PredicatePolicy,
    ) -> ExplicitArcArrangementReport {
        let intersection = self.intersect_arc(other, policy);
        if intersection.class == ExplicitArcIntersectionClass::SameCircle {
            let overlap = self.classify_same_circle_overlap(other, policy);
            let class = match overlap.class {
                ExplicitArcOverlapClass::DifferentCircle => ExplicitArcArrangementClass::Unknown,
                ExplicitArcOverlapClass::Disjoint => {
                    ExplicitArcArrangementClass::SameCircleDisjoint
                }
                ExplicitArcOverlapClass::EndpointTouch => {
                    ExplicitArcArrangementClass::SameCircleEndpointTouch
                }
                ExplicitArcOverlapClass::Overlap => ExplicitArcArrangementClass::SameCircleOverlap,
                ExplicitArcOverlapClass::FirstCoversSecond => {
                    ExplicitArcArrangementClass::SameCircleFirstCoversSecond
                }
                ExplicitArcOverlapClass::SecondCoversFirst => {
                    ExplicitArcArrangementClass::SameCircleSecondCoversFirst
                }
                ExplicitArcOverlapClass::Equal => ExplicitArcArrangementClass::SameCircleEqual,
                ExplicitArcOverlapClass::Unknown => ExplicitArcArrangementClass::Unknown,
            };
            ExplicitArcArrangementReport {
                class,
                overlap: Some(overlap),
                intersection: None,
            }
        } else {
            let class = match intersection.class {
                ExplicitArcIntersectionClass::SameCircle => ExplicitArcArrangementClass::Unknown,
                ExplicitArcIntersectionClass::Disjoint => {
                    ExplicitArcArrangementClass::DifferentCircleDisjoint
                }
                ExplicitArcIntersectionClass::OnePoint => {
                    ExplicitArcArrangementClass::DifferentCircleOnePoint
                }
                ExplicitArcIntersectionClass::TwoPoints => {
                    ExplicitArcArrangementClass::DifferentCircleTwoPoints
                }
                ExplicitArcIntersectionClass::CircleIntersectionsOutsideArcSweeps => {
                    ExplicitArcArrangementClass::DifferentCircleOutsideArcSweeps
                }
                ExplicitArcIntersectionClass::Unknown => ExplicitArcArrangementClass::Unknown,
            };
            ExplicitArcArrangementReport {
                class,
                overlap: None,
                intersection: Some(intersection),
            }
        }
    }
}

fn tangent_from_radial(rx: &Real, ry: &Real, direction: ArcDirection) -> Point2 {
    match direction {
        ArcDirection::Ccw => Point2::new(-ry.clone(), rx.clone()),
        ArcDirection::Cw => Point2::new(ry.clone(), -rx.clone()),
    }
}

fn cardinal_point(center: &Point2, radius: &Real, cardinal: CardinalPoint) -> Point2 {
    match cardinal {
        CardinalPoint::East => Point2::new(center.x.clone() + radius.clone(), center.y.clone()),
        CardinalPoint::North => Point2::new(center.x.clone(), center.y.clone() + radius.clone()),
        CardinalPoint::West => Point2::new(center.x.clone() - radius.clone(), center.y.clone()),
        CardinalPoint::South => Point2::new(center.x.clone(), center.y.clone() - radius.clone()),
    }
}

fn point_radius_squared(center: &Point2, point: &Point2) -> Real {
    point_distance_squared(center, point)
}

fn point_distance_squared(first: &Point2, second: &Point2) -> Real {
    let dx = second.x.clone() - first.x.clone();
    let dy = second.y.clone() - first.y.clone();
    Real::signed_product_sum([true, true], [[&dx, &dx], [&dy, &dy]])
}

fn point_on_axis_line(
    axis: Axis,
    segment: &LinePathSegment,
    center: &Point2,
    signed_root: Real,
) -> Point2 {
    match axis {
        Axis::X => Point2::new(center.x.clone() + signed_root, segment.start().y.clone()),
        Axis::Y => Point2::new(segment.start().x.clone(), center.y.clone() + signed_root),
    }
}

fn point_inside_segment_bounds(
    point: &Point2,
    segment: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<bool> {
    Some(
        real_between_closed(
            &point.x,
            &segment.bounds_min().x,
            &segment.bounds_max().x,
            policy,
        )? && real_between_closed(
            &point.y,
            &segment.bounds_min().y,
            &segment.bounds_max().y,
            policy,
        )?,
    )
}

fn real_between_closed(
    value: &Real,
    min: &Real,
    max: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    let lower = compare_reals_with_policy(value, min, policy).value()?;
    let upper = compare_reals_with_policy(value, max, policy).value()?;
    Some(
        matches!(lower, Ordering::Equal | Ordering::Greater)
            && matches!(upper, Ordering::Equal | Ordering::Less),
    )
}

fn push_unique_point(points: &mut Vec<Point2>, candidate: Point2, policy: PredicatePolicy) {
    if points.iter().any(|point| {
        compare_reals_with_policy(&point.x, &candidate.x, policy).value() == Some(Ordering::Equal)
            && compare_reals_with_policy(&point.y, &candidate.y, policy).value()
                == Some(Ordering::Equal)
    }) {
        return;
    }
    points.push(candidate);
}

fn same_circle(
    first: &ExplicitCircularArc,
    second: &ExplicitCircularArc,
    policy: PredicatePolicy,
) -> Option<bool> {
    Some(
        same_real_with_policy(&first.center.x, &second.center.x, policy)?
            && same_real_with_policy(&first.center.y, &second.center.y, policy)?
            && same_real_with_policy(&first.radius, &second.radius, policy)?,
    )
}

fn same_arc_orientation_and_endpoints(
    first: &ExplicitCircularArc,
    second: &ExplicitCircularArc,
    policy: PredicatePolicy,
) -> Option<bool> {
    Some(
        first.direction == second.direction
            && point_equal_with_policy(&first.start, &second.start, policy)?
            && point_equal_with_policy(&first.end, &second.end, policy)?,
    )
}

fn same_real_with_policy(first: &Real, second: &Real, policy: PredicatePolicy) -> Option<bool> {
    Some(compare_reals_with_policy(first, second, policy).value()? == Ordering::Equal)
}

fn point_equal_with_policy(
    first: &Point2,
    second: &Point2,
    policy: PredicatePolicy,
) -> Option<bool> {
    Some(
        same_real_with_policy(&first.x, &second.x, policy)?
            && same_real_with_policy(&first.y, &second.y, policy)?,
    )
}

fn point_on_arc_bool(
    arc: &ExplicitCircularArc,
    point: &Point2,
    policy: PredicatePolicy,
) -> Option<bool> {
    match arc.classify_point(point, policy) {
        ExplicitArcPointClassification::OnArc => Some(true),
        ExplicitArcPointClassification::OnCircleOutsideSweep
        | ExplicitArcPointClassification::OffCircle => Some(false),
        ExplicitArcPointClassification::Unknown => None,
    }
}

fn point_on_both_arcs(
    first: &ExplicitCircularArc,
    second: &ExplicitCircularArc,
    point: &Point2,
    policy: PredicatePolicy,
) -> Option<bool> {
    Some(point_on_arc_bool(first, point, policy)? && point_on_arc_bool(second, point, policy)?)
}

fn point_is_arc_endpoint(
    arc: &ExplicitCircularArc,
    point: &Point2,
    policy: PredicatePolicy,
) -> bool {
    point_equal_with_policy(point, &arc.start, policy).unwrap_or(false)
        || point_equal_with_policy(point, &arc.end, policy).unwrap_or(false)
}

fn shared_arc_endpoints(
    first: &ExplicitCircularArc,
    second: &ExplicitCircularArc,
    policy: PredicatePolicy,
) -> Vec<Point2> {
    let mut endpoints = Vec::new();
    for point in [&first.start, &first.end] {
        if point_is_arc_endpoint(second, point, policy) {
            push_unique_point(&mut endpoints, point.clone(), policy);
        }
    }
    endpoints
}

fn arc_intersection_unknown_report(
    circle_relation: ExplicitCircleRelationClass,
) -> ExplicitArcIntersectionReport {
    ExplicitArcIntersectionReport {
        class: ExplicitArcIntersectionClass::Unknown,
        circle_relation,
        points: Vec::new(),
    }
}

fn circle_secant_points(
    first: &ExplicitCircularArc,
    second: &ExplicitCircularArc,
    policy: PredicatePolicy,
) -> Option<Vec<Point2>> {
    let dx = second.center.x.clone() - first.center.x.clone();
    let dy = second.center.y.clone() - first.center.y.clone();
    let distance_squared = Real::signed_product_sum([true, true], [[&dx, &dx], [&dy, &dy]]);
    if compare_reals_with_policy(&distance_squared, &Real::zero(), policy).value()?
        == Ordering::Equal
    {
        return None;
    }

    let numerator = first.facts.radius_squared.clone() - second.facts.radius_squared.clone()
        + distance_squared.clone();
    let denominator = Real::from(2) * distance_squared.clone();
    let base_scale = (numerator.clone() / denominator.clone()).ok()?;
    let base = Point2::new(
        first.center.x.clone() + dx.clone() * base_scale.clone(),
        first.center.y.clone() + dy.clone() * base_scale,
    );

    let radicand = Real::from(4) * distance_squared * first.facts.radius_squared.clone()
        - numerator.clone() * numerator;
    let radicand_order = compare_reals_with_policy(&radicand, &Real::zero(), policy).value()?;
    if radicand_order == Ordering::Less {
        return None;
    }
    let root = if radicand_order == Ordering::Equal {
        Real::zero()
    } else {
        radicand.sqrt().ok()?
    };
    let offset_scale = (root / denominator).ok()?;
    let offset_x = -dy * offset_scale.clone();
    let offset_y = dx * offset_scale;
    let first_point = Point2::new(
        base.x.clone() + offset_x.clone(),
        base.y.clone() + offset_y.clone(),
    );
    let second_point = Point2::new(base.x - offset_x, base.y - offset_y);
    let mut points = Vec::new();
    push_unique_point(&mut points, first_point, policy);
    push_unique_point(&mut points, second_point, policy);
    Some(points)
}

fn arc_overlap_unknown_report() -> ExplicitArcOverlapReport {
    ExplicitArcOverlapReport {
        class: ExplicitArcOverlapClass::Unknown,
        shared_endpoints: Vec::new(),
    }
}

fn classify_circle_relation_from_squares(
    center_distance_squared: &Real,
    radius_sum_squared: &Real,
    radius_difference_squared: &Real,
    same_circle: Option<bool>,
    policy: PredicatePolicy,
) -> ExplicitCircleRelationClass {
    if same_circle == Some(true) {
        return ExplicitCircleRelationClass::SameCircle;
    }
    let Some(sum_ordering) =
        compare_reals_with_policy(center_distance_squared, radius_sum_squared, policy).value()
    else {
        return ExplicitCircleRelationClass::Unknown;
    };
    match sum_ordering {
        Ordering::Greater => ExplicitCircleRelationClass::Separate,
        Ordering::Equal => ExplicitCircleRelationClass::ExternallyTangent,
        Ordering::Less => {
            let Some(diff_ordering) = compare_reals_with_policy(
                center_distance_squared,
                radius_difference_squared,
                policy,
            )
            .value() else {
                return ExplicitCircleRelationClass::Unknown;
            };
            match diff_ordering {
                Ordering::Greater => ExplicitCircleRelationClass::Secant,
                Ordering::Equal => ExplicitCircleRelationClass::InternallyTangent,
                Ordering::Less => ExplicitCircleRelationClass::Contained,
            }
        }
    }
}

fn circle_tangent_point(
    first: &ExplicitCircularArc,
    second: &ExplicitCircularArc,
    class: ExplicitCircleRelationClass,
) -> Option<Point2> {
    match class {
        ExplicitCircleRelationClass::ExternallyTangent => {
            circle_affine_point(first, second, first.radius.clone() + second.radius.clone())
        }
        ExplicitCircleRelationClass::InternallyTangent => {
            let denominator = first.radius.clone() - second.radius.clone();
            circle_affine_point(first, second, denominator)
        }
        _ => None,
    }
}

fn circle_affine_point(
    first: &ExplicitCircularArc,
    second: &ExplicitCircularArc,
    denominator: Real,
) -> Option<Point2> {
    let dx = second.center.x.clone() - first.center.x.clone();
    let dy = second.center.y.clone() - first.center.y.clone();
    Some(Point2::new(
        first.center.x.clone() + ((dx * first.radius.clone()) / denominator.clone()).ok()?,
        first.center.y.clone() + ((dy * first.radius.clone()) / denominator).ok()?,
    ))
}

fn line_arc_report(points: Vec<Point2>) -> LineExplicitArcIntersectionReport {
    let class = match points.len() {
        0 => LineExplicitArcIntersectionClass::Disjoint,
        1 => LineExplicitArcIntersectionClass::Tangent,
        2 => LineExplicitArcIntersectionClass::Secant,
        _ => LineExplicitArcIntersectionClass::Unknown,
    };
    LineExplicitArcIntersectionReport { class, points }
}

fn line_arc_unknown_report() -> LineExplicitArcIntersectionReport {
    LineExplicitArcIntersectionReport {
        class: LineExplicitArcIntersectionClass::Unknown,
        points: Vec::new(),
    }
}

fn radial_dot_cross(center: &Point2, start: &Point2, end: &Point2) -> (Real, Real) {
    let sx = start.x.clone() - center.x.clone();
    let sy = start.y.clone() - center.y.clone();
    let ex = end.x.clone() - center.x.clone();
    let ey = end.y.clone() - center.y.clone();
    let dot = sx.clone() * ex.clone() + sy.clone() * ey.clone();
    let cross = sx * ey - sy * ex;
    (dot, cross)
}

fn explicit_arc_minor_angle(arc: &ExplicitCircularArc) -> Option<Real> {
    (arc.facts.radial_dot.clone() / arc.facts.radius_squared.clone())
        .ok()?
        .acos()
        .ok()
}

fn directed_cross_sign(
    center: &Point2,
    first: &Point2,
    second: &Point2,
    direction: ArcDirection,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    let first_x = first.x.clone() - center.x.clone();
    let first_y = first.y.clone() - center.y.clone();
    let second_x = second.x.clone() - center.x.clone();
    let second_y = second.y.clone() - center.y.clone();
    let ccw_cross = first_x * second_y - first_y * second_x;
    let directed = match direction {
        ArcDirection::Ccw => ccw_cross,
        ArcDirection::Cw => -ccw_cross,
    };
    compare_reals_with_policy(&directed, &Real::zero(), policy).value()
}

fn is_nonnegative(ordering: Ordering) -> bool {
    matches!(ordering, Ordering::Equal | Ordering::Greater)
}

fn is_positive(ordering: Ordering) -> bool {
    ordering == Ordering::Greater
}

fn classify_explicit_sweep(
    radial_cross: &Real,
    known_full_circle: bool,
    direction: ArcDirection,
) -> ExplicitArcSweepClass {
    if known_full_circle {
        return ExplicitArcSweepClass::FullCircle;
    }
    match (radial_cross.structural_facts().sign, direction) {
        (Some(RealSign::Zero), _) => ExplicitArcSweepClass::HalfTurn,
        (Some(RealSign::Positive), ArcDirection::Ccw)
        | (Some(RealSign::Negative), ArcDirection::Cw) => ExplicitArcSweepClass::LessThanHalfTurn,
        (Some(RealSign::Negative), ArcDirection::Ccw)
        | (Some(RealSign::Positive), ArcDirection::Cw) => {
            ExplicitArcSweepClass::GreaterThanHalfTurn
        }
        (None, _) => ExplicitArcSweepClass::Unknown,
    }
}

fn quarter_turns(start: CardinalPoint, end: CardinalPoint, direction: ArcDirection) -> u8 {
    let start_index = cardinal_index(start);
    let end_index = cardinal_index(end);
    let turns = match direction {
        ArcDirection::Ccw => (end_index + 4 - start_index) % 4,
        ArcDirection::Cw => (start_index + 4 - end_index) % 4,
    };
    if turns == 0 { 4 } else { turns }
}

fn cardinal_index(cardinal: CardinalPoint) -> u8 {
    match cardinal {
        CardinalPoint::East => 0,
        CardinalPoint::North => 1,
        CardinalPoint::West => 2,
        CardinalPoint::South => 3,
    }
}
