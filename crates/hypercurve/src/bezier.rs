//! Polynomial Bezier curve primitives.
//!
//! The types in this module are exact object carriers: control points are stored
//! as [`Real`](hyperreal::Real), evaluation is algebraic, and topology-sensitive
//! predicates are intentionally added separately. This follows Yap's exact
//! geometric computation split between exact representations, certified
//! predicates, and explicit approximate output adapters; see Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).

use hyperreal::{Real, ZeroKnowledge as ZeroStatus};

use crate::classify::is_zero;
use crate::{Aabb2, Classification, CurvePolicy, Point2};

/// An endpoint of a parametric Bezier segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierEndpoint {
    /// The endpoint at parameter `t = 0`.
    Start,
    /// The endpoint at parameter `t = 1`.
    End,
}

/// Exact first-derivative information at a Bezier endpoint.
///
/// The vector is the polynomial derivative at the endpoint: `degree *
/// (P1 - P0)` at the start or `degree * (Pn - Pn-1)` at the end. When this
/// vector is structurally zero, callers that need a geometric tangent should
/// continue to higher derivatives before making topology decisions. This
/// mirrors the endpoint-derivative treatment in Farin, *Curves and Surfaces
/// for Computer-Aided Geometric Design* (5th ed., 2002).
#[derive(Clone, Debug, PartialEq)]
pub struct EndpointTangent2 {
    dx: Real,
    dy: Real,
    zero_status: ZeroStatus,
}

impl EndpointTangent2 {
    /// Constructs endpoint derivative information from an exact vector.
    pub fn new(dx: Real, dy: Real) -> Self {
        let length_squared = &dx * &dx + &dy * &dy;
        let zero_status = length_squared.zero_status();
        Self {
            dx,
            dy,
            zero_status,
        }
    }

    /// Returns the derivative x component.
    pub const fn dx(&self) -> &Real {
        &self.dx
    }

    /// Returns the derivative y component.
    pub const fn dy(&self) -> &Real {
        &self.dy
    }

    /// Returns whether the derivative vector is structurally zero.
    pub const fn zero_status(&self) -> ZeroStatus {
        self.zero_status
    }
}

/// A polynomial quadratic Bezier segment with three exact control points.
///
/// The segment is represented by `(start, control, end)` and evaluated with
/// de Casteljau subdivision. De Casteljau's algorithm preserves affine
/// structure and is the standard numerically stable geometric construction for
/// Bezier curves; see de Casteljau, "Outillage methodes calcul," Andre
/// Citroen Automobiles SA, 1959.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezier2 {
    start: Point2,
    control: Point2,
    end: Point2,
}

impl QuadraticBezier2 {
    /// Constructs a quadratic Bezier segment.
    pub const fn new(start: Point2, control: Point2, end: Point2) -> Self {
        Self {
            start,
            control,
            end,
        }
    }

    /// Returns the start point.
    pub const fn start(&self) -> &Point2 {
        &self.start
    }

    /// Returns the single interior control point.
    pub const fn control(&self) -> &Point2 {
        &self.control
    }

    /// Returns the end point.
    pub const fn end(&self) -> &Point2 {
        &self.end
    }

    /// Returns the control points in polynomial order.
    pub fn control_points(&self) -> [&Point2; 3] {
        [&self.start, &self.control, &self.end]
    }

    /// Evaluates the curve at affine parameter `t`.
    ///
    /// This uses one step of de Casteljau subdivision instead of expanding the
    /// Bernstein polynomial. Keeping the affine construction visible gives
    /// later exact predicates an obvious place to reuse structural facts about
    /// `t`, endpoint grids, and shared denominator schedules.
    pub fn point_at(&self, t: Real) -> Point2 {
        let left = self.start.lerp(&self.control, t.clone());
        let right = self.control.lerp(&self.end, t.clone());
        left.lerp(&right, t)
    }

    /// Classifies whether `point` equals this curve at parameter `t`.
    ///
    /// This is a parameterized point-on-curve predicate, not an existential
    /// root solve. It is useful when another exact kernel has already produced
    /// a candidate parameter and the curve layer must certify the point before
    /// branching. The zero test is delegated to the same policy boundary as
    /// the rest of `hypercurve`, following Yap's exact predicate model.
    pub fn contains_point_at_parameter(
        &self,
        point: &Point2,
        t: Real,
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        point_equals_at_parameter(self.point_at(t), point, policy)
    }

    /// Returns a conservative convex-hull box for the control polygon.
    ///
    /// A Bezier segment lies inside the convex hull of its control polygon.
    /// The box is therefore a broad-phase envelope, not a topology decision.
    /// Predicate code must still certify actual intersections or containment.
    pub fn control_hull_box(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        Aabb2::from_points(self.control_points(), policy)
    }

    /// Returns whether the endpoints are structurally known to coincide.
    pub fn endpoints_coincident_status(&self) -> ZeroStatus {
        self.start.distance_squared(&self.end).zero_status()
    }

    /// Returns exact first-derivative information at one endpoint.
    pub fn endpoint_tangent(&self, endpoint: BezierEndpoint) -> EndpointTangent2 {
        let two = Real::from(2_i8);
        let (dx, dy) = match endpoint {
            BezierEndpoint::Start => self.control.delta_from(&self.start),
            BezierEndpoint::End => self.end.delta_from(&self.control),
        };
        EndpointTangent2::new(&two * dx, &two * dy)
    }

    /// Returns conservative structural facts for exact predicate scheduling.
    pub fn structural_facts(&self) -> crate::Bezier2Facts {
        crate::facts::quadratic_bezier_facts(self)
    }
}

/// A polynomial cubic Bezier segment with four exact control points.
///
/// Cubics are the first general free-form curve family in `hypercurve`. This
/// type deliberately stores only exact control geometry and cheap structural
/// facts; monotone splitting, inflection handling, and curve/curve predicates
/// are separate exact-kernel work items.
#[derive(Clone, Debug, PartialEq)]
pub struct CubicBezier2 {
    start: Point2,
    control1: Point2,
    control2: Point2,
    end: Point2,
}

impl CubicBezier2 {
    /// Constructs a cubic Bezier segment.
    pub const fn new(start: Point2, control1: Point2, control2: Point2, end: Point2) -> Self {
        Self {
            start,
            control1,
            control2,
            end,
        }
    }

    /// Returns the start point.
    pub const fn start(&self) -> &Point2 {
        &self.start
    }

    /// Returns the first interior control point.
    pub const fn control1(&self) -> &Point2 {
        &self.control1
    }

    /// Returns the second interior control point.
    pub const fn control2(&self) -> &Point2 {
        &self.control2
    }

    /// Returns the end point.
    pub const fn end(&self) -> &Point2 {
        &self.end
    }

    /// Returns the control points in polynomial order.
    pub fn control_points(&self) -> [&Point2; 4] {
        [&self.start, &self.control1, &self.control2, &self.end]
    }

    /// Evaluates the curve at affine parameter `t`.
    ///
    /// The nested de Casteljau construction keeps subdivision exact over
    /// [`Real`] inputs and avoids introducing an approximate polynomial adapter
    /// into the topology layer.
    pub fn point_at(&self, t: Real) -> Point2 {
        let p01 = self.start.lerp(&self.control1, t.clone());
        let p12 = self.control1.lerp(&self.control2, t.clone());
        let p23 = self.control2.lerp(&self.end, t.clone());
        let p012 = p01.lerp(&p12, t.clone());
        let p123 = p12.lerp(&p23, t.clone());
        p012.lerp(&p123, t)
    }

    /// Classifies whether `point` equals this curve at parameter `t`.
    pub fn contains_point_at_parameter(
        &self,
        point: &Point2,
        t: Real,
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        point_equals_at_parameter(self.point_at(t), point, policy)
    }

    /// Returns a conservative convex-hull box for the control polygon.
    pub fn control_hull_box(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        Aabb2::from_points(self.control_points(), policy)
    }

    /// Returns whether the endpoints are structurally known to coincide.
    pub fn endpoints_coincident_status(&self) -> ZeroStatus {
        self.start.distance_squared(&self.end).zero_status()
    }

    /// Returns exact first-derivative information at one endpoint.
    pub fn endpoint_tangent(&self, endpoint: BezierEndpoint) -> EndpointTangent2 {
        let three = Real::from(3_i8);
        let (dx, dy) = match endpoint {
            BezierEndpoint::Start => self.control1.delta_from(&self.start),
            BezierEndpoint::End => self.end.delta_from(&self.control2),
        };
        EndpointTangent2::new(&three * dx, &three * dy)
    }

    /// Returns conservative structural facts for exact predicate scheduling.
    pub fn structural_facts(&self) -> crate::Bezier2Facts {
        crate::facts::cubic_bezier_facts(self)
    }
}

fn point_equals_at_parameter(
    curve_point: Point2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    let distance_squared = curve_point.distance_squared(point);
    is_zero(&distance_squared, policy)
        .map(Classification::Decided)
        .unwrap_or(Classification::Uncertain(
            crate::UncertaintyReason::Ordering,
        ))
}
