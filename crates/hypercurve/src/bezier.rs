//! Polynomial Bezier curve primitives.
//!
//! The types in this module are exact object carriers: control points are stored
//! as [`Real`](hyperreal::Real), evaluation is algebraic, and topology-sensitive
//! predicates are intentionally added separately. This follows Yap's exact
//! geometric computation split between exact representations, certified
//! predicates, and explicit approximate output adapters; see Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).

use hyperreal::{Real, ZeroKnowledge as ZeroStatus};

use std::cmp::Ordering;

use crate::classify::{compare_reals, is_zero};
use crate::{
    Aabb2, Classification, CurvePolicy, CurveResult, Point2, RetainedTopologyStatus,
    UncertaintyReason,
};

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

/// Report for exact quadratic interpolation through a retained midpoint.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezierMidpointInterpolationReport2 {
    stage: QuadraticBezierMidpointInterpolationStage2,
    interpolation_parameter: Real,
    start_point: Point2,
    midpoint_constraint: Point2,
    end_point: Point2,
    solved_control_point: Option<Point2>,
    replay_path: Option<BezierInterpolationReplayPath2>,
    replayed_midpoint: Option<Point2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by quadratic midpoint interpolation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QuadraticBezierMidpointInterpolationStage2 {
    /// The exact Bernstein control point was being solved.
    ControlSolve,
    /// The native quadratic span was materialized and replayed.
    SegmentMaterialization,
}

/// Result of exact quadratic interpolation through a retained midpoint.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezierMidpointInterpolationResult2 {
    curve: Option<QuadraticBezier2>,
    report: QuadraticBezierMidpointInterpolationReport2,
}

/// Report for exact quadratic interpolation through one retained parameter point.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezierPointInterpolationReport2 {
    stage: QuadraticBezierPointInterpolationStage2,
    interpolation_parameter: Real,
    start_point: Point2,
    interpolation_point: Point2,
    end_point: Point2,
    solved_control_point: Option<Point2>,
    replay_path: Option<BezierInterpolationReplayPath2>,
    replayed_point: Option<Point2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Furthest exact stage reached by quadratic point interpolation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QuadraticBezierPointInterpolationStage2 {
    /// The retained interpolation parameter was being validated.
    ParameterValidation,
    /// The exact Bernstein control point was being solved.
    ControlSolve,
    /// The native quadratic span was materialized and replayed.
    SegmentMaterialization,
}

/// Result of exact quadratic interpolation through one retained parameter point.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezierPointInterpolationResult2 {
    curve: Option<QuadraticBezier2>,
    report: QuadraticBezierPointInterpolationReport2,
}

/// Report for exact cubic Hermite interpolation from endpoint derivatives.
#[derive(Clone, Debug, PartialEq)]
pub struct CubicBezierHermiteInterpolationReport2 {
    stage: CubicBezierHermiteInterpolationStage2,
    start_point: Point2,
    start_tangent: EndpointTangent2,
    end_point: Point2,
    end_tangent: EndpointTangent2,
    solved_first_control_point: Option<Point2>,
    solved_second_control_point: Option<Point2>,
    replay_path: Option<BezierInterpolationReplayPath2>,
    replayed_start_tangent: Option<EndpointTangent2>,
    replayed_end_tangent: Option<EndpointTangent2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Replay path used to certify interpolated Bezier constraints after materialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierInterpolationReplayPath2 {
    /// Materialized control points were replayed with exact Bezier evaluation.
    ExactEvaluationReplay,
}

/// Furthest exact stage reached by cubic Hermite interpolation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CubicBezierHermiteInterpolationStage2 {
    /// The two cubic control points were being solved from endpoint derivatives.
    ControlSolve,
    /// The native cubic span was materialized and replayed.
    SegmentMaterialization,
}

/// Result of exact cubic Hermite interpolation from endpoint derivatives.
#[derive(Clone, Debug, PartialEq)]
pub struct CubicBezierHermiteInterpolationResult2 {
    curve: Option<CubicBezier2>,
    report: CubicBezierHermiteInterpolationReport2,
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

impl QuadraticBezierMidpointInterpolationReport2 {
    /// Returns the furthest exact interpolation stage reached.
    pub const fn stage(&self) -> QuadraticBezierMidpointInterpolationStage2 {
        self.stage
    }

    /// Returns the retained interpolation parameter.
    pub const fn interpolation_parameter(&self) -> &Real {
        &self.interpolation_parameter
    }

    /// Returns the exact start-point constraint.
    pub const fn start_point(&self) -> &Point2 {
        &self.start_point
    }

    /// Returns the exact midpoint constraint at `t = 1/2`.
    pub const fn midpoint_constraint(&self) -> &Point2 {
        &self.midpoint_constraint
    }

    /// Returns the exact end-point constraint.
    pub const fn end_point(&self) -> &Point2 {
        &self.end_point
    }

    /// Returns the solved quadratic control point, when materialized.
    pub const fn solved_control_point(&self) -> Option<&Point2> {
        self.solved_control_point.as_ref()
    }

    /// Returns the exact replay path used to validate the materialized span.
    pub const fn replay_path(&self) -> Option<BezierInterpolationReplayPath2> {
        self.replay_path
    }

    /// Returns the replayed curve point at the interpolation parameter.
    pub const fn replayed_midpoint(&self) -> Option<&Point2> {
        self.replayed_midpoint.as_ref()
    }

    /// Returns the interpolation status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized interpolation attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl QuadraticBezierMidpointInterpolationResult2 {
    /// Returns the materialized quadratic Bezier span, if supported.
    pub const fn curve(&self) -> Option<&QuadraticBezier2> {
        self.curve.as_ref()
    }

    /// Consumes this result and returns the materialized quadratic Bezier span, if any.
    pub fn into_curve(self) -> Option<QuadraticBezier2> {
        self.curve
    }

    /// Returns retained interpolation evidence.
    pub const fn report(&self) -> &QuadraticBezierMidpointInterpolationReport2 {
        &self.report
    }
}

impl QuadraticBezierPointInterpolationReport2 {
    /// Returns the furthest exact interpolation stage reached.
    pub const fn stage(&self) -> QuadraticBezierPointInterpolationStage2 {
        self.stage
    }

    /// Returns the retained interpolation parameter.
    pub const fn interpolation_parameter(&self) -> &Real {
        &self.interpolation_parameter
    }

    /// Returns the exact start-point constraint.
    pub const fn start_point(&self) -> &Point2 {
        &self.start_point
    }

    /// Returns the exact interpolation point constraint.
    pub const fn interpolation_point(&self) -> &Point2 {
        &self.interpolation_point
    }

    /// Returns the exact end-point constraint.
    pub const fn end_point(&self) -> &Point2 {
        &self.end_point
    }

    /// Returns the solved quadratic control point, when materialized.
    pub const fn solved_control_point(&self) -> Option<&Point2> {
        self.solved_control_point.as_ref()
    }

    /// Returns the exact replay path used to validate the materialized span.
    pub const fn replay_path(&self) -> Option<BezierInterpolationReplayPath2> {
        self.replay_path
    }

    /// Returns the replayed curve point at the interpolation parameter.
    pub const fn replayed_point(&self) -> Option<&Point2> {
        self.replayed_point.as_ref()
    }

    /// Returns the interpolation status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized interpolation attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl QuadraticBezierPointInterpolationResult2 {
    /// Returns the materialized quadratic Bezier span, if supported.
    pub const fn curve(&self) -> Option<&QuadraticBezier2> {
        self.curve.as_ref()
    }

    /// Consumes this result and returns the materialized quadratic Bezier span, if any.
    pub fn into_curve(self) -> Option<QuadraticBezier2> {
        self.curve
    }

    /// Returns retained interpolation evidence.
    pub const fn report(&self) -> &QuadraticBezierPointInterpolationReport2 {
        &self.report
    }
}

impl CubicBezierHermiteInterpolationReport2 {
    /// Returns the furthest exact interpolation stage reached.
    pub const fn stage(&self) -> CubicBezierHermiteInterpolationStage2 {
        self.stage
    }

    /// Returns the exact start-point constraint.
    pub const fn start_point(&self) -> &Point2 {
        &self.start_point
    }

    /// Returns the exact start derivative constraint.
    pub const fn start_tangent(&self) -> &EndpointTangent2 {
        &self.start_tangent
    }

    /// Returns the exact end-point constraint.
    pub const fn end_point(&self) -> &Point2 {
        &self.end_point
    }

    /// Returns the exact end derivative constraint.
    pub const fn end_tangent(&self) -> &EndpointTangent2 {
        &self.end_tangent
    }

    /// Returns the solved first cubic control point, when materialized.
    pub const fn solved_first_control_point(&self) -> Option<&Point2> {
        self.solved_first_control_point.as_ref()
    }

    /// Returns the solved second cubic control point, when materialized.
    pub const fn solved_second_control_point(&self) -> Option<&Point2> {
        self.solved_second_control_point.as_ref()
    }

    /// Returns the exact replay path used to validate the materialized span.
    pub const fn replay_path(&self) -> Option<BezierInterpolationReplayPath2> {
        self.replay_path
    }

    /// Returns the replayed start derivative from the materialized cubic span.
    pub const fn replayed_start_tangent(&self) -> Option<&EndpointTangent2> {
        self.replayed_start_tangent.as_ref()
    }

    /// Returns the replayed end derivative from the materialized cubic span.
    pub const fn replayed_end_tangent(&self) -> Option<&EndpointTangent2> {
        self.replayed_end_tangent.as_ref()
    }

    /// Returns the interpolation status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized interpolation attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CubicBezierHermiteInterpolationResult2 {
    /// Returns the materialized cubic Bezier span, if supported.
    pub const fn curve(&self) -> Option<&CubicBezier2> {
        self.curve.as_ref()
    }

    /// Consumes this result and returns the materialized cubic Bezier span, if any.
    pub fn into_curve(self) -> Option<CubicBezier2> {
        self.curve
    }

    /// Returns retained interpolation evidence.
    pub const fn report(&self) -> &CubicBezierHermiteInterpolationReport2 {
        &self.report
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
        let result =
            Self::interpolate_point_at_parameter_with_report(start, t, point, end, policy)?;
        let blocker = result
            .report()
            .blocker()
            .unwrap_or(UncertaintyReason::Unsupported);
        match result.into_curve() {
            Some(curve) => Ok(Classification::Decided(curve)),
            None => Ok(Classification::Uncertain(blocker)),
        }
    }

    /// Constructs the exact quadratic Bezier span through `point` at parameter `t`.
    ///
    /// The returned report records domain validation, the exact constraint
    /// points, retained parameter, solved control point, and replayed image.
    pub fn interpolate_point_at_parameter_with_report(
        start: Point2,
        t: Real,
        point: Point2,
        end: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<QuadraticBezierPointInterpolationResult2> {
        if let Some((status, blocker)) = quadratic_interpolation_parameter_blocker(&t, policy) {
            return Ok(QuadraticBezierPointInterpolationResult2 {
                curve: None,
                report: QuadraticBezierPointInterpolationReport2 {
                    stage: QuadraticBezierPointInterpolationStage2::ParameterValidation,
                    interpolation_parameter: t,
                    start_point: start,
                    interpolation_point: point,
                    end_point: end,
                    solved_control_point: None,
                    replay_path: None,
                    replayed_point: None,
                    status,
                    blocker: Some(blocker),
                },
            });
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
        let control = Point2::new(control_x, control_y);
        let curve = Self::new(start.clone(), control.clone(), end.clone());
        let replayed_point = curve.point_at(t.clone());

        Ok(QuadraticBezierPointInterpolationResult2 {
            curve: Some(curve),
            report: QuadraticBezierPointInterpolationReport2 {
                stage: QuadraticBezierPointInterpolationStage2::SegmentMaterialization,
                interpolation_parameter: t,
                start_point: start,
                interpolation_point: point,
                end_point: end,
                solved_control_point: Some(control),
                replay_path: Some(BezierInterpolationReplayPath2::ExactEvaluationReplay),
                replayed_point: Some(replayed_point),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    /// Constructs the exact quadratic Bezier span through `midpoint` at `t = 1/2`.
    ///
    /// This solves the Bernstein equation
    /// `B(1/2) = (start + 2 * control + end) / 4` exactly over [`Real`], then
    /// replays the retained midpoint constraint against the materialized curve.
    pub fn interpolate_midpoint(start: Point2, midpoint: Point2, end: Point2) -> CurveResult<Self> {
        Self::interpolate_midpoint_with_report(start, midpoint, end).map(|result| {
            result
                .into_curve()
                .expect("exact midpoint interpolation materializes")
        })
    }

    /// Constructs the exact quadratic Bezier span through `midpoint` at `t = 1/2`.
    ///
    /// The returned report records the exact constraint points, retained
    /// parameter, solved control point, and replayed midpoint image.
    pub fn interpolate_midpoint_with_report(
        start: Point2,
        midpoint: Point2,
        end: Point2,
    ) -> CurveResult<QuadraticBezierMidpointInterpolationResult2> {
        let two = Real::from(2_i8);
        let half = (Real::one() / two.clone())?;
        let result = Self::interpolate_point_at_parameter_with_report(
            start.clone(),
            half,
            midpoint.clone(),
            end.clone(),
            &CurvePolicy::certified(),
        )?;
        let control = result.report().solved_control_point().cloned();
        let replayed_midpoint = result.report().replayed_point().cloned();

        Ok(QuadraticBezierMidpointInterpolationResult2 {
            curve: result.into_curve(),
            report: QuadraticBezierMidpointInterpolationReport2 {
                stage: QuadraticBezierMidpointInterpolationStage2::SegmentMaterialization,
                interpolation_parameter: (Real::one() / two)?,
                start_point: start,
                midpoint_constraint: midpoint,
                end_point: end,
                solved_control_point: control,
                replay_path: Some(BezierInterpolationReplayPath2::ExactEvaluationReplay),
                replayed_midpoint,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
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
        Self::interpolate_hermite_with_report(start, start_tangent, end, end_tangent).map(
            |result| {
                result
                    .into_curve()
                    .expect("exact cubic Hermite interpolation materializes")
            },
        )
    }

    /// Constructs the exact cubic Bezier span with retained endpoint derivatives.
    ///
    /// The returned report records the endpoint constraints, solved Bezier
    /// control points, and replayed endpoint derivatives from the materialized
    /// cubic span.
    pub fn interpolate_hermite_with_report(
        start: Point2,
        start_tangent: EndpointTangent2,
        end: Point2,
        end_tangent: EndpointTangent2,
    ) -> CurveResult<CubicBezierHermiteInterpolationResult2> {
        let three = Real::from(3_i8);
        let first_control_dx = (start_tangent.dx() / three.clone())?;
        let first_control_dy = (start_tangent.dy() / three.clone())?;
        let second_control_dx = (end_tangent.dx() / three.clone())?;
        let second_control_dy = (end_tangent.dy() / three)?;
        let control1 = start.translated(first_control_dx, first_control_dy);
        let control2 = end.translated(-second_control_dx, -second_control_dy);
        let curve = Self::new(
            start.clone(),
            control1.clone(),
            control2.clone(),
            end.clone(),
        );
        let replayed_start_tangent = curve.endpoint_tangent(BezierEndpoint::Start);
        let replayed_end_tangent = curve.endpoint_tangent(BezierEndpoint::End);

        Ok(CubicBezierHermiteInterpolationResult2 {
            curve: Some(curve),
            report: CubicBezierHermiteInterpolationReport2 {
                stage: CubicBezierHermiteInterpolationStage2::SegmentMaterialization,
                start_point: start,
                start_tangent,
                end_point: end,
                end_tangent,
                solved_first_control_point: Some(control1),
                solved_second_control_point: Some(control2),
                replay_path: Some(BezierInterpolationReplayPath2::ExactEvaluationReplay),
                replayed_start_tangent: Some(replayed_start_tangent),
                replayed_end_tangent: Some(replayed_end_tangent),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
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

fn quadratic_interpolation_parameter_blocker(
    t: &Real,
    policy: &CurvePolicy,
) -> Option<(RetainedTopologyStatus, UncertaintyReason)> {
    let zero = Real::zero();
    let one = Real::one();
    let Some(lower) = compare_reals(t, &zero, policy) else {
        return Some((
            RetainedTopologyStatus::Unresolved,
            UncertaintyReason::Ordering,
        ));
    };
    let Some(upper) = compare_reals(t, &one, policy) else {
        return Some((
            RetainedTopologyStatus::Unresolved,
            UncertaintyReason::Ordering,
        ));
    };
    if !matches!(lower, Ordering::Greater) || !matches!(upper, Ordering::Less) {
        return Some((
            RetainedTopologyStatus::Unsupported,
            UncertaintyReason::Boundary,
        ));
    }
    None
}
