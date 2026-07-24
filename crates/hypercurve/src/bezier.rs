//! Polynomial Bezier curve primitives.
//!
//! The types in this module are exact object carriers: control points are stored
//! as [`Real`](hyperreal::Real), evaluation is algebraic, and topology-sensitive
//! predicates are intentionally added separately. This follows the exactness model's exact
//! geometric computation split between exact representations, certified
//! predicates, and explicit approximate output adapters.

use hyperreal::{Real, ZeroKnowledge as ZeroStatus};

use std::cmp::Ordering;

use crate::classify::{compare_reals, is_zero};
use crate::{Aabb2, Classification, CurvePolicy, CurveResult, Point2, UncertaintyReason};

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
/// mirrors the endpoint-derivative treatment in the Bernstein and de Casteljau curve model.
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

    pub(crate) fn into_components(self) -> (Real, Real, ZeroStatus) {
        (self.dx, self.dy, self.zero_status)
    }
}

/// A polynomial quadratic Bezier segment with three exact control points.
///
/// The segment is represented by `(start, control, end)` and evaluated with
/// de Casteljau subdivision. De Casteljau's algorithm preserves affine
/// structure and is the standard numerically stable geometric construction for
/// Bezier curves using de Casteljau subdivision.
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

    /// Constructs the exact quadratic Bezier span through `point` at parameter `t`.
    ///
    /// The parameter must be certified strictly inside `(0, 1)`. Endpoint and
    /// out-of-domain parameters are returned as explicit boundary blockers
    /// because the single interior control point is not determined there.
    pub fn interpolate_point_at_parameter(
        start: Point2,
        t: Real,
        point: Point2,
        end: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if let Some(blocker) = quadratic_interpolation_parameter_blocker(&t, policy) {
            return Ok(Classification::Uncertain(blocker));
        }

        let one_minus_t = Real::one() - &t;
        let start_weight = &one_minus_t * &one_minus_t;
        let end_weight = &t * &t;
        let denominator = (Real::from(2_i8) * &one_minus_t) * &t;

        let point_x = point.x();
        let point_y = point.y();
        let control_x = (((point_x - &(start.x() * &start_weight)) - &(end.x() * &end_weight))
            / denominator.clone())?;
        let control_y =
            (((point_y - &(start.y() * &start_weight)) - &(end.y() * &end_weight)) / denominator)?;
        Ok(Classification::Decided(Self::new(
            start,
            Point2::new(control_x, control_y),
            end,
        )))
    }

    /// Constructs the exact quadratic Bezier span through `midpoint` at `t = 1/2`.
    ///
    /// This solves the Bernstein equation
    /// `B(1/2) = (start + 2 * control + end) / 4` exactly over [`Real`], then
    /// replays the retained midpoint constraint against the materialized curve.
    pub fn interpolate_midpoint(start: Point2, midpoint: Point2, end: Point2) -> CurveResult<Self> {
        let two = Real::from(2_i8);
        let half = (Real::one() / two.clone())?;
        let Classification::Decided(curve) = Self::interpolate_point_at_parameter(
            start,
            half,
            midpoint,
            end,
            &CurvePolicy::certified(),
        )?
        else {
            unreachable!("the exact half parameter is strictly interior")
        };
        Ok(curve)
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
    /// Exact-rational parameters use the quadratic polynomial in Horner form,
    /// reducing multiplication count without introducing an approximate
    /// adapter. Other parameters retain the affine de Casteljau expression
    /// graph used by downstream exact predicates.
    pub fn point_at(&self, t: Real) -> Point2 {
        if t.exact_rational_ref().is_none() {
            let one_minus_t = Real::one() - &t;
            let left = self
                .start
                .lerp_with_weights(&self.control, &one_minus_t, &t);
            let right = self.control.lerp_with_weights(&self.end, &one_minus_t, &t);
            return left.lerp_with_weights(&right, &one_minus_t, &t);
        }

        let two = Real::from(2);
        let x_linear = (self.control.x() - self.start.x()) * &two;
        let x_quadratic = self.start.x() - self.control.x() * &two + self.end.x();
        let y_linear = (self.control.y() - self.start.y()) * &two;
        let y_quadratic = self.start.y() - self.control.y() * &two + self.end.y();
        Point2::new(
            self.start.x() + (x_linear + x_quadratic * &t) * &t,
            self.start.y() + (y_linear + y_quadratic * &t) * &t,
        )
    }

    /// Classifies whether `point` equals this curve at parameter `t`.
    ///
    /// This is a parameterized point-on-curve predicate, not an existential
    /// root solve. It is useful when another exact kernel has already produced
    /// a candidate parameter and the curve layer must certify the point before
    /// branching. The zero test is delegated to the same policy boundary as
    /// the rest of `hypercurve`, following the exactness model's exact predicate model.
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

    /// Constructs the exact cubic Bezier span with retained endpoint derivatives.
    ///
    /// This is the standard cubic Hermite-to-Bezier conversion. The derivative
    /// constraints are exact endpoint derivative vectors, not normalized tangent
    /// directions, so no length fitting or approximate tangent scaling is used.
    pub fn interpolate_hermite(
        start: Point2,
        start_tangent: EndpointTangent2,
        end: Point2,
        end_tangent: EndpointTangent2,
    ) -> CurveResult<Self> {
        let three = Real::from(3_i8);
        let first_control_dx = (start_tangent.dx() / three.clone())?;
        let first_control_dy = (start_tangent.dy() / three.clone())?;
        let second_control_dx = (end_tangent.dx() / three.clone())?;
        let second_control_dy = (end_tangent.dy() / three)?;
        let control1 = start.translated(first_control_dx, first_control_dy);
        let control2 = end.translated(-second_control_dx, -second_control_dy);
        Ok(Self::new(start, control1, control2, end))
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
    /// Exact-rational parameters evaluate shared Bernstein weights across both
    /// coordinates. Other parameters retain the de Casteljau expression graph
    /// used by downstream exact predicates. Both paths remain exact over
    /// [`Real`] inputs.
    pub fn point_at(&self, t: Real) -> Point2 {
        if t.exact_rational_ref().is_none() {
            let one_minus_t = Real::one() - &t;
            let p01 = self
                .start
                .lerp_with_weights(&self.control1, &one_minus_t, &t);
            let p12 = self
                .control1
                .lerp_with_weights(&self.control2, &one_minus_t, &t);
            let p23 = self.control2.lerp_with_weights(&self.end, &one_minus_t, &t);
            let p012 = p01.lerp_with_weights(&p12, &one_minus_t, &t);
            let p123 = p12.lerp_with_weights(&p23, &one_minus_t, &t);
            return p012.lerp_with_weights(&p123, &one_minus_t, &t);
        }

        let one_minus_t = Real::one() - &t;
        let one_minus_t_squared = &one_minus_t * &one_minus_t;
        let t_squared = &t * &t;
        let start_weight = &one_minus_t_squared * &one_minus_t;
        let control1_weight = (&one_minus_t_squared * &t) * Real::from(3);
        let control2_weight = (&one_minus_t * &t_squared) * Real::from(3);
        let end_weight = &t_squared * &t;
        Point2::new(
            self.start.x() * &start_weight
                + self.control1.x() * &control1_weight
                + self.control2.x() * &control2_weight
                + self.end.x() * &end_weight,
            self.start.y() * &start_weight
                + self.control1.y() * &control1_weight
                + self.control2.y() * &control2_weight
                + self.end.y() * &end_weight,
        )
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

fn quadratic_interpolation_parameter_blocker(
    t: &Real,
    policy: &CurvePolicy,
) -> Option<UncertaintyReason> {
    let zero = Real::zero();
    let one = Real::one();
    let Some(lower) = compare_reals(t, &zero, policy) else {
        return Some(UncertaintyReason::Ordering);
    };
    let Some(upper) = compare_reals(t, &one, policy) else {
        return Some(UncertaintyReason::Ordering);
    };
    if !matches!(lower, Ordering::Greater) || !matches!(upper, Ordering::Less) {
        return Some(UncertaintyReason::Boundary);
    }
    None
}
