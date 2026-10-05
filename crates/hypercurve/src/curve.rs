//! Top-level exact curve carriers.

#[path = "curve_evaluation.rs"]
mod curve_evaluation;

#[path = "curve_subdivision.rs"]
mod curve_subdivision;
use crate::bezier_split::{CurveParameterDomain2, CurveParameterRange2};
use curve_subdivision::CurveSourceRange2;
pub(crate) use curve_subdivision::CurveSourceSpan2;

#[path = "curve_corner_reconstruction.rs"]
mod curve_corner_reconstruction;
use curve_corner_reconstruction::corner_has_native_reconstruction;

#[path = "curve_corner_domain.rs"]
mod curve_corner_domain;

#[path = "curve_corner_cuts.rs"]
mod curve_corner_cuts;
#[path = "curve_fillet.rs"]
mod curve_fillet;
#[path = "curve_fillet_centers.rs"]
mod curve_fillet_centers;
pub(crate) use curve_corner_cuts::*;
pub use curve_fillet::{CurveFillet2, CurveFilletContact2};
pub(crate) use curve_fillet::{FilletConstraintBinding2, FilletContactChart2};
use curve_fillet::{FilletCornerSelection2, fillet_corner_from_center};
pub(crate) use curve_fillet_centers::*;

use crate::CurvePointData2;
use std::sync::Arc;
use std::sync::OnceLock;

use hyperreal::RealSign;

use crate::arc_bezier::{
    circular_conic_provenance, decompose_circular_arc, rational_bezier_circular_arc,
    rational_quadratic_circular_arc,
};
use crate::policy::{
    PolicyEvaluationCache, resolve_cached_evaluation, resolve_certified_operation,
};
use crate::{
    Aabb2, BezierParallel2, BezierParameter2, BezierSubcurve2, CircularArc2, Classification,
    ContourPointLocation, CubicBezier2, CurveContext, CurveError, CurveOperation2, CurveOutcome,
    CurveParameter2, CurvePoint2, CurveRegionBoundaryLoop2, ExactCurveError, ExactCurveResult,
    LineSeg2, LineSide, NurbsCurve2, ParamRange, Point2, PolynomialSplineCurve2, QuadraticBezier2,
    RationalBezier2, RationalQuadraticBezier2, Real, Similarity2,
};
use crate::{BezierEndpoint, BezierParameterRange2};

/// Exact planar curve family.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum CurveFamily2 {
    /// Finite straight line segment.
    Line,
    /// Finite circular arc.
    CircularArc,
    /// Polynomial quadratic Bezier curve.
    QuadraticBezier,
    /// Polynomial cubic Bezier curve.
    CubicBezier,
    /// Rational quadratic Bezier/conic curve.
    RationalQuadraticBezier,
    /// General rational Bezier curve.
    RationalBezier,
    /// Polynomial B-spline curve.
    PolynomialBSpline,
    /// Rational B-spline/NURBS curve.
    Nurbs,
    /// Exact analytic parallel of a supported source curve.
    AnalyticParallel,
}

/// Exact derivative vector of a planar curve with respect to its public parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveDerivative2 {
    dx: Real,
    dy: Real,
    zero_status: hyperreal::ZeroKnowledge,
}

/// Exact derivative vector of a curve at a general public parameter.
///
/// Coordinates are either represented reals or selected algebraic values kept
/// in the field of the parameter that produced them. Selected coordinates are
/// never rounded; [`Self::represented_coordinates`] is the narrower query for
/// callers that need two represented reals.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveVector2 {
    data: CurveVectorData2,
}

#[derive(Clone, Debug, PartialEq)]
enum CurveVectorData2 {
    Represented(CurveDerivative2),
    /// A selected-field vector times an exact positive chart factor.
    Selected {
        vector: crate::BezierAlgebraicTangentVector2,
        chart_factor: Real,
    },
    /// A selected source velocity `v` times its parallel's speed ratio
    /// `1 - d cross(v, a) / |v|^3` at distance `d`, whose nonzero sign is
    /// certified at construction.
    SelectedParallel(Box<SelectedParallelVector2>),
}

#[derive(Clone, Debug, PartialEq)]
struct SelectedParallelVector2 {
    velocity: crate::BezierAlgebraicTangentVector2,
    acceleration: crate::BezierAlgebraicTangentVector2,
    distance: Real,
    ratio_sign: RealSign,
}

impl CurveVector2 {
    /// Returns the represented coordinates, when both are plain reals.
    pub fn represented_coordinates(&self) -> Option<(&Real, &Real)> {
        match &self.data {
            CurveVectorData2::Represented(derivative) => Some((derivative.dx(), derivative.dy())),
            CurveVectorData2::Selected { .. } | CurveVectorData2::SelectedParallel(_) => None,
        }
    }

    /// Decides the exact sign of one coordinate.
    pub fn coordinate_sign(
        &self,
        axis: crate::Axis2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<RealSign>> {
        let use_x = axis == crate::Axis2::X;
        match &self.data {
            CurveVectorData2::Represented(derivative) => {
                let value = if use_x {
                    derivative.dx()
                } else {
                    derivative.dy()
                };
                Ok(match crate::classify::real_sign(value, policy) {
                    Some(sign) => Classification::Decided(sign),
                    None => Classification::Uncertain(crate::UncertaintyReason::RealSign),
                })
            }
            // The chart factor is a positive power of 1/width.
            CurveVectorData2::Selected { vector, .. } => {
                vector.coordinate_sign(use_x, policy).map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Evaluation,
                        CurveFamily2::RationalBezier,
                        cause,
                    )
                })
            }
            CurveVectorData2::SelectedParallel(parallel) => Ok(parallel
                .velocity
                .coordinate_sign(use_x, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Evaluation,
                        CurveFamily2::RationalBezier,
                        cause,
                    )
                })?
                .map(|sign| match (sign, parallel.ratio_sign) {
                    (RealSign::Zero, _) | (_, RealSign::Positive) => sign,
                    (RealSign::Positive, _) => RealSign::Negative,
                    (RealSign::Negative, _) => RealSign::Positive,
                })),
        }
    }

    /// Scales a selected source velocity into its parallel's derivative.
    /// A zero speed ratio is the exact zero derivative of a parallel cusp.
    fn selected_parallel(
        velocity: crate::BezierAlgebraicTangentVector2,
        acceleration: crate::BezierAlgebraicTangentVector2,
        distance: Real,
        ratio_sign: RealSign,
    ) -> Self {
        if ratio_sign == RealSign::Zero {
            return Self::represented(CurveDerivative2::new(Real::zero(), Real::zero()));
        }
        Self {
            data: CurveVectorData2::SelectedParallel(Box::new(SelectedParallelVector2 {
                velocity,
                acceleration,
                distance,
                ratio_sign,
            })),
        }
    }

    fn selected(vector: crate::BezierAlgebraicTangentVector2) -> Self {
        Self {
            data: CurveVectorData2::Selected {
                vector,
                chart_factor: Real::one(),
            },
        }
    }

    fn represented(derivative: CurveDerivative2) -> Self {
        Self {
            data: CurveVectorData2::Represented(derivative),
        }
    }
}

impl From<CurveDerivative2> for CurveVector2 {
    fn from(derivative: CurveDerivative2) -> Self {
        Self::represented(derivative)
    }
}

/// Side policy for differential evaluation at a retained span boundary.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum CurveParameterSide2 {
    /// Require equal left and right derivatives when both spans contain the parameter.
    #[default]
    Automatic,
    /// Use the span immediately before an internal boundary.
    Left,
    /// Use the span immediately after an internal boundary.
    Right,
}

/// Whether a solved corner edit may extend incident carriers past the corner.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum CurveCornerMode2 {
    /// Keep both solved contacts strictly inside their incident curve domains.
    #[default]
    TrimOnly,
    /// Also retain exact solutions reached by extending either carrier past the corner.
    TrimOrExtend,
}

/// Exact reason that a supported corner solver produced no candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveCornerNoSolution2 {
    /// The radius or both chamfer setbacks are exactly zero.
    ZeroDesignValue,
    /// The incident tangents are parallel or coincident, so no finite fillet center exists.
    ParallelTangents,
    /// The exact radius-offset supports do not meet, so no tangent circle exists.
    NoTangentCircle,
    /// Every exact candidate lies outside the permitted trim domains.
    OutsideTrimDomain,
    /// No admissible candidate satisfies all supplied exact constraints.
    UnsatisfiedConstraints,
    /// Every candidate collapses the inserted corner carrier.
    DegenerateCandidate,
}

/// Complete exact solutions for one corner-edit request.
///
/// Candidate order is deterministic. Chamfers order trim/trim, trim/extension,
/// extension/trim, then extension/extension whenever those candidates exist.
#[derive(Clone, Debug, PartialEq)]
pub enum CurveCornerSolutions2<T> {
    /// The supported exact system has no admissible solution.
    NoSolution(CurveCornerNoSolution2),
    /// Exactly one admissible solution exists.
    Unique(T),
    /// More than one exact solution exists and the caller must select one.
    Multiple(Vec<T>),
}

impl<T> CurveCornerSolutions2<T> {
    /// Borrows the finite exact candidates in deterministic order.
    pub fn solutions(&self) -> &[T] {
        match self {
            Self::NoSolution(_) => &[],
            Self::Unique(candidate) => std::slice::from_ref(candidate),
            Self::Multiple(candidates) => candidates,
        }
    }

    /// Takes the finite exact candidates in deterministic order.
    pub fn into_solutions(self) -> Vec<T> {
        match self {
            Self::NoSolution(_) => Vec::new(),
            Self::Unique(candidate) => vec![candidate],
            Self::Multiple(candidates) => candidates,
        }
    }

    /// Returns the number of exact candidates.
    pub fn candidate_count(&self) -> usize {
        match self {
            Self::NoSolution(_) => 0,
            Self::Unique(_) => 1,
            Self::Multiple(candidates) => candidates.len(),
        }
    }

    /// Returns the no-solution reason, when no candidate exists.
    pub const fn no_solution_reason(&self) -> Option<CurveCornerNoSolution2> {
        match self {
            Self::NoSolution(reason) => Some(*reason),
            Self::Unique(_) | Self::Multiple(_) => None,
        }
    }
}

impl CurveDerivative2 {
    /// Constructs an exact derivative vector.
    pub fn new(dx: Real, dy: Real) -> Self {
        let zero_status = (&dx * &dx + &dy * &dy).zero_status();
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

    /// Returns whether the derivative is structurally zero.
    pub const fn zero_status(&self) -> hyperreal::ZeroKnowledge {
        self.zero_status
    }

    /// Scales this derivative by an exact parameter-chain factor.
    pub fn scaled(&self, factor: &Real) -> Self {
        Self::new(&self.dx * factor, &self.dy * factor)
    }
}

/// Geometry carried by a top-level exact planar curve.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum CurveGeometry2 {
    /// Finite straight line segment.
    Line(LineSeg2),
    /// Finite circular arc.
    CircularArc(CircularArc2),
    /// Polynomial quadratic Bezier curve.
    QuadraticBezier(QuadraticBezier2),
    /// Polynomial cubic Bezier curve.
    CubicBezier(CubicBezier2),
    /// Rational quadratic Bezier/conic curve.
    RationalQuadraticBezier(RationalQuadraticBezier2),
    /// General rational Bezier curve.
    RationalBezier(RationalBezier2),
    /// Polynomial B-spline curve.
    PolynomialBSpline(PolynomialSplineCurve2),
    /// Rational B-spline/NURBS curve.
    Nurbs(NurbsCurve2),
}

#[derive(Debug)]
struct CurveData2 {
    carrier: CurveCarrier2,
    lineage: Option<CurveParameterLineage2>,
    parameter_domain: OnceLock<crate::CurveParameterRange2>,
    native_bezier_fragments: PolicyEvaluationCache<Vec<NativeBezierFragment2>>,
    rational_evaluators: PolicyEvaluationCache<Vec<RationalBezier2>>,
    bounds: OnceLock<ExactCurveResult<Aabb2>>,
}

#[derive(Debug, PartialEq)]
enum CurveCarrier2 {
    Native(CurveGeometry2),
    Restricted(Box<CurveRestrictedCarrier2>),
    SourceRange(Box<CurveSourceRange2>),
}

#[derive(Debug)]
struct CurveRestrictedCarrier2 {
    fragment: Arc<crate::BezierSplitFragment2>,
    endpoints: [OnceLock<CurvePoint2>; 2],
}

impl PartialEq for CurveRestrictedCarrier2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.fragment, &other.fragment) || self.fragment == other.fragment
    }
}

#[derive(Clone, Debug)]
struct CurveParameterLineage2 {
    root: Arc<CurveParameterLineageRoot2>,
    range: ParamRange,
    /// One composed image map; sharing a parameter source alone does not
    /// certify coincidence after transporting its geometry.
    image_transform: Option<Arc<Similarity2>>,
}

#[derive(Debug)]
struct CurveParameterLineageRoot2 {
    domain: ParamRange,
    image_is_injective: OnceLock<bool>,
}

impl CurveParameterLineage2 {
    fn new(range: ParamRange) -> Self {
        Self {
            root: Arc::new(CurveParameterLineageRoot2 {
                domain: range.clone(),
                image_is_injective: OnceLock::new(),
            }),
            range,
            image_transform: None,
        }
    }

    fn reversed(&self) -> Self {
        Self {
            root: Arc::clone(&self.root),
            range: ParamRange::new(self.range.end().clone(), self.range.start().clone()),
            image_transform: self.image_transform.clone(),
        }
    }

    fn transformed(&self, transform: &Similarity2) -> Self {
        Self {
            root: Arc::clone(&self.root),
            range: self.range.clone(),
            image_transform: Some(Arc::new(
                self.image_transform
                    .as_ref()
                    .map_or_else(|| transform.clone(), |previous| previous.then(transform)),
            )),
        }
    }
}

/// Immutable top-level exact planar curve.
///
/// Clones share the exact carrier and its retained calculations. Operations
/// borrow this value directly, preserving the same caches and certificates.
/// Native curves and validated generated fragments convert directly into
/// this value, retaining their source domains and selected endpoint evidence.
#[derive(Clone, Debug)]
pub struct Curve2 {
    data: Arc<CurveData2>,
}

/// Ordered connected sequence of exact curves.
#[derive(Clone, Debug)]
pub struct CurvePath2 {
    data: Arc<CurvePathData2>,
}

#[derive(Debug)]
struct CurvePathData2 {
    curves: Vec<Curve2>,
    connectivity_policy: Option<CurveContext>,
    closure_policy: Option<CurveContext>,
    native_bezier_fragments: PolicyEvaluationCache<Vec<NativeBezierFragment2>>,
    boundary_loop: PolicyEvaluationCache<CurveRegionBoundaryLoop2>,
    bounds: OnceLock<ExactCurveResult<Aabb2>>,
}

/// Exact affine chart from a support parameter to its public curve parameter.
///
/// The endpoints are the images of local zero and one. Native span charts
/// ascend; a reversed retained span can have a descending chart.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveSpanRange2 {
    start: Real,
    end: Real,
}

/// Exact native Bezier/conic fragment and its public parameter interval.
#[derive(Clone, Debug)]
pub struct NativeBezierFragment2 {
    curve: BezierSubcurve2,
    span_range: CurveSpanRange2,
    lineage: CurveParameterLineage2,
}

impl PartialEq for NativeBezierFragment2 {
    fn eq(&self, other: &Self) -> bool {
        self.curve == other.curve && self.span_range == other.span_range
    }
}

impl CurveGeometry2 {
    pub(crate) fn from_bezier(curve: BezierSubcurve2) -> Self {
        match curve {
            BezierSubcurve2::Quadratic(curve) => Self::QuadraticBezier(curve),
            BezierSubcurve2::Cubic(curve) => Self::CubicBezier(curve),
            BezierSubcurve2::RationalQuadratic(curve) => Self::RationalQuadraticBezier(curve),
            BezierSubcurve2::Rational(curve) => Self::RationalBezier(curve),
        }
    }

    /// Returns this geometry's curve family.
    pub const fn family(&self) -> CurveFamily2 {
        match self {
            Self::Line(_) => CurveFamily2::Line,
            Self::CircularArc(_) => CurveFamily2::CircularArc,
            Self::QuadraticBezier(_) => CurveFamily2::QuadraticBezier,
            Self::CubicBezier(_) => CurveFamily2::CubicBezier,
            Self::RationalQuadraticBezier(_) => CurveFamily2::RationalQuadraticBezier,
            Self::RationalBezier(_) => CurveFamily2::RationalBezier,
            Self::PolynomialBSpline(_) => CurveFamily2::PolynomialBSpline,
            Self::Nurbs(_) => CurveFamily2::Nurbs,
        }
    }

    /// Returns the exact start point.
    pub fn start(&self) -> &Point2 {
        match self {
            Self::Line(curve) => curve.start(),
            Self::CircularArc(curve) => curve.start(),
            Self::QuadraticBezier(curve) => curve.start(),
            Self::CubicBezier(curve) => curve.start(),
            Self::RationalQuadraticBezier(curve) => curve.start(),
            Self::RationalBezier(curve) => curve.start(),
            Self::PolynomialBSpline(curve) => curve.start(),
            Self::Nurbs(curve) => curve.start(),
        }
    }

    /// Returns the exact end point.
    pub fn end(&self) -> &Point2 {
        match self {
            Self::Line(curve) => curve.end(),
            Self::CircularArc(curve) => curve.end(),
            Self::QuadraticBezier(curve) => curve.end(),
            Self::CubicBezier(curve) => curve.end(),
            Self::RationalQuadraticBezier(curve) => curve.end(),
            Self::RationalBezier(curve) => curve.end(),
            Self::PolynomialBSpline(curve) => curve.end(),
            Self::Nurbs(curve) => curve.end(),
        }
    }
}

impl Curve2 {
    /// Wraps exact geometry in a clone-shared carrier.
    pub fn new(geometry: CurveGeometry2) -> Self {
        let lineage = CurveParameterLineage2::new(geometry_parameter_range(&geometry));
        Self::from_geometry_with_lineage(geometry, lineage)
    }

    /// Constructs the exact nonzero line segment between two general points.
    ///
    /// Endpoints may be represented or retained algebraic points, including
    /// selected intersection and corner locations; neither is rounded nor
    /// projected into a common field. Coincidence that `policy` cannot decide
    /// is returned as uncertainty, and coincident endpoints are invalid.
    pub fn try_line(start: CurvePoint2, end: CurvePoint2) -> crate::ExactCurveResult<Self> {
        Self::try_line_with_policy(start, end, &crate::policy::principal_context())
            .map_err(|cause| {
                crate::ExactCurveError::invalid_unattributed(
                    crate::CurveOperation2::Construction,
                    cause,
                )
            })
            .and_then(|value| {
                crate::ExactCurveError::decided(crate::CurveOperation2::Construction, value)
            })
    }

    /// [`Self::try_line`] under an explicit predicate policy.
    pub(crate) fn try_line_with_policy(
        start: CurvePoint2,
        end: CurvePoint2,
        policy: &CurveContext,
    ) -> crate::CurveResult<Classification<Self>> {
        Ok(crate::BezierAlgebraicChord2::try_new(start, end, policy)?.map(Self::from))
    }

    /// Constructs an exact analytic Bezier parallel on a finite oriented
    /// source-parameter range.
    ///
    /// The range may extend beyond the authored unit chart. Source poles are
    /// excluded at every distance; zero distance permits stationary or constant
    /// sources without requiring a normal. Source singularities are forbidden
    /// on a nonzero-distance range. Parallel cusps may be range endpoints, where
    /// later arrangement splitting owns the vertex, but may not remain in the
    /// open interior. The curve retains the procedural parallel; no fitted
    /// Bezier or sampled endpoint is introduced.
    pub fn try_analytic_parallel(
        parallel: crate::BezierParallel2,
        range: crate::BezierParameterRange2,
    ) -> crate::ExactCurveResult<Self> {
        Self::try_analytic_parallel_with_policy(
            parallel,
            range,
            &crate::policy::principal_context(),
        )
        .map_err(|cause| {
            crate::ExactCurveError::invalid_unattributed(
                crate::CurveOperation2::Construction,
                cause,
            )
        })
        .and_then(|value| {
            crate::ExactCurveError::decided(crate::CurveOperation2::Construction, value)
        })
    }

    /// [`Self::try_analytic_parallel`] under an explicit predicate policy.
    pub(crate) fn try_analytic_parallel_with_policy(
        parallel: crate::BezierParallel2,
        range: crate::BezierParameterRange2,
        policy: &CurveContext,
    ) -> crate::CurveResult<Classification<Self>> {
        Ok(crate::BezierParallelFragment2::try_new(parallel, range, policy)?.map(Self::from))
    }

    /// Constructs an exact polynomial B-spline carrier under `policy`.
    pub fn try_polynomial_bspline(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
    ) -> crate::ExactCurveResult<Self> {
        Self::try_polynomial_bspline_with_policy(
            degree,
            control_points,
            knots,
            &crate::policy::principal_context(),
        )
        .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::try_polynomial_bspline`] under an explicit predicate policy.
    pub(crate) fn try_polynomial_bspline_with_policy(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        PolynomialSplineCurve2::try_new_with_policy(degree, control_points, knots, policy)
            .map(|outcome| outcome.map(|curve| Self::new(CurveGeometry2::PolynomialBSpline(curve))))
    }

    /// Constructs an exact NURBS carrier under `policy`.
    pub fn try_nurbs(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
    ) -> crate::ExactCurveResult<Self> {
        Self::try_nurbs_with_policy(
            degree,
            control_points,
            weights,
            knots,
            &crate::policy::principal_context(),
        )
        .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::try_nurbs`] under an explicit predicate policy.
    pub(crate) fn try_nurbs_with_policy(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        NurbsCurve2::try_new_with_policy(degree, control_points, weights, knots, policy)
            .map(|outcome| outcome.map(|curve| Self::new(CurveGeometry2::Nurbs(curve))))
    }

    /// Constructs a periodic polynomial B-spline from one period under `policy`.
    pub fn try_periodic_polynomial_bspline(
        degree: usize,
        control_points: Vec<Point2>,
        period_knots: Vec<Real>,
    ) -> crate::ExactCurveResult<Self> {
        Self::try_periodic_polynomial_bspline_with_policy(
            degree,
            control_points,
            period_knots,
            &crate::policy::principal_context(),
        )
        .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::try_periodic_polynomial_bspline`] under an explicit predicate policy.
    pub(crate) fn try_periodic_polynomial_bspline_with_policy(
        degree: usize,
        control_points: Vec<Point2>,
        period_knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        PolynomialSplineCurve2::try_new_periodic_with_policy(
            degree,
            control_points,
            period_knots,
            policy,
        )
        .map(|outcome| outcome.map(|curve| Self::new(CurveGeometry2::PolynomialBSpline(curve))))
    }

    /// Constructs a periodic NURBS from one period under `policy`.
    pub fn try_periodic_nurbs(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        period_knots: Vec<Real>,
    ) -> crate::ExactCurveResult<Self> {
        Self::try_periodic_nurbs_with_policy(
            degree,
            control_points,
            weights,
            period_knots,
            &crate::policy::principal_context(),
        )
        .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::try_periodic_nurbs`] under an explicit predicate policy.
    pub(crate) fn try_periodic_nurbs_with_policy(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        period_knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        NurbsCurve2::try_new_periodic_with_policy(
            degree,
            control_points,
            weights,
            period_knots,
            policy,
        )
        .map(|outcome| outcome.map(|curve| Self::new(CurveGeometry2::Nurbs(curve))))
    }

    /// Returns a directly stored native representation of this curve image.
    ///
    /// Selected domains and analytic images remain exact when this view is
    /// absent. General operations use the retained carrier and domain directly.
    pub fn geometry(&self) -> Option<&CurveGeometry2> {
        match &self.data.carrier {
            CurveCarrier2::Native(geometry) => Some(geometry),
            CurveCarrier2::Restricted(_) | CurveCarrier2::SourceRange(_) => None,
        }
    }

    /// Returns the curve family, including generated analytic parallels.
    pub fn family(&self) -> CurveFamily2 {
        if let Some(range) = self.source_range() {
            return range.source.family();
        }
        if let Some(geometry) = self.geometry() {
            return geometry.family();
        }
        match self.retained_fragment().expect("every curve has a carrier") {
            crate::BezierSplitFragment2::Materialized { curve, .. }
            | crate::BezierSplitFragment2::RetainedBezier {
                source_curve: curve,
                ..
            } => match curve {
                BezierSubcurve2::Quadratic(_) => CurveFamily2::QuadraticBezier,
                BezierSubcurve2::Cubic(_) => CurveFamily2::CubicBezier,
                BezierSubcurve2::RationalQuadratic(_) => CurveFamily2::RationalQuadraticBezier,
                BezierSubcurve2::Rational(_) => CurveFamily2::RationalBezier,
            },
            crate::BezierSplitFragment2::AnalyticParallel(_) => CurveFamily2::AnalyticParallel,
            crate::BezierSplitFragment2::AlgebraicChord(_) => CurveFamily2::Line,
            crate::BezierSplitFragment2::AlgebraicCuspSemicircle(_) => CurveFamily2::CircularArc,
            crate::BezierSplitFragment2::SelectedFiber(fragment) => {
                if fragment.rational_curve().is_some() {
                    CurveFamily2::RationalBezier
                } else {
                    CurveFamily2::AnalyticParallel
                }
            }
        }
    }

    /// Returns the exact start point without reconstructing selected coordinates.
    pub fn start(&self) -> CurvePoint2 {
        self.endpoint(true)
    }

    /// Returns the exact end point without reconstructing selected coordinates.
    pub fn end(&self) -> CurvePoint2 {
        self.endpoint(false)
    }

    fn endpoint(&self, start: bool) -> CurvePoint2 {
        if let Some(range) = self.source_range() {
            return range.endpoints[usize::from(start == range.reversed)].clone();
        }
        if let Some(geometry) = self.geometry() {
            return CurvePoint2::from(
                if start {
                    geometry.start()
                } else {
                    geometry.end()
                }
                .clone(),
            );
        }
        match self.retained_fragment().expect("every curve has a carrier") {
            crate::BezierSplitFragment2::AlgebraicChord(chord) => {
                if start { chord.start() } else { chord.end() }.clone()
            }
            crate::BezierSplitFragment2::SelectedFiber(fragment) => if start {
                fragment.start_point()
            } else {
                fragment.end_point()
            }
            .clone(),
            _ => {
                let CurveCarrier2::Restricted(carrier) = &self.data.carrier else {
                    unreachable!("native endpoints handled above")
                };
                carrier.endpoints[usize::from(!start)]
                    .get_or_init(|| {
                        CurvePoint2::from_endpoint(Arc::clone(&carrier.fragment), start)
                    })
                    .clone()
            }
        }
    }

    pub(crate) fn retained_fragment(&self) -> Option<&crate::BezierSplitFragment2> {
        match &self.data.carrier {
            CurveCarrier2::Native(_) | CurveCarrier2::SourceRange(_) => None,
            CurveCarrier2::Restricted(carrier) => Some(&carrier.fragment),
        }
    }

    pub(crate) fn from_retained_fragment(fragment: crate::BezierSplitFragment2) -> Self {
        if let crate::BezierSplitFragment2::Materialized { curve, .. } = fragment {
            // Region topology carries straight pieces as degree-elevated
            // quadratics. With a midpoint control the quadratic chart is the
            // line's own chart, so the line is the faithful public curve.
            // Parallel-tangency evidence lives on the quadratic; keep it there.
            if let crate::BezierSubcurve2::Quadratic(quadratic) = &curve
                && quadratic.retained_parallel_line_tangent_contacts().is_empty()
                && quadratic.retained_exact_line_image().is_some()
                && [
                    Real::from(2_i8) * quadratic.control().x()
                        - quadratic.start().x()
                        - quadratic.end().x(),
                    Real::from(2_i8) * quadratic.control().y()
                        - quadratic.start().y()
                        - quadratic.end().y(),
                ]
                .iter()
                .all(|offset| offset.zero_status() == hyperreal::ZeroKnowledge::Zero)
                // The fragment's own endpoints are the loop's shared vertex
                // representations; a cached image may hold equal copies.
                && let Some(image) = quadratic.retained_exact_line_image()
                && let Ok(line) = image.with_endpoint_representations(
                    quadratic.start().clone(),
                    quadratic.end().clone(),
                )
            {
                return Self::from(line);
            }
            return Self::from(curve);
        }
        Self {
            data: Arc::new(CurveData2 {
                carrier: CurveCarrier2::Restricted(Box::new(CurveRestrictedCarrier2 {
                    fragment: Arc::new(fragment),
                    endpoints: [OnceLock::new(), OnceLock::new()],
                })),
                lineage: None,
                parameter_domain: OnceLock::new(),
                native_bezier_fragments: PolicyEvaluationCache::new(),
                rational_evaluators: PolicyEvaluationCache::new(),
                bounds: OnceLock::new(),
            }),
        }
    }

    /// Returns the shared exact domain in the retained carrier's parameter chart.
    pub fn parameter_domain(&self) -> &crate::CurveParameterRange2 {
        if let Some(range) = self.source_range() {
            return &range.range;
        }
        self.data.parameter_domain.get_or_init(|| {
            if let Some(fragment) = self.retained_fragment() {
                return fragment.curve_region_parameter_range();
            }
            let range = geometry_parameter_range(self.geometry().expect("native curve"));
            crate::CurveParameterRange2::new_validated(
                CurveParameter2::from(range.start().clone()),
                CurveParameter2::from(range.end().clone()),
            )
        })
    }

    pub(crate) fn native_parameter_domain(&self) -> ExactCurveResult<ParamRange> {
        self.geometry()
            .map(geometry_parameter_range)
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    self.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })
    }

    /// Returns the exact period when this top-level curve is periodic.
    pub fn period(&self) -> Option<&Real> {
        match self.geometry() {
            Some(CurveGeometry2::PolynomialBSpline(curve)) => curve.period(),
            Some(CurveGeometry2::Nurbs(curve)) => curve.period(),
            _ => None,
        }
    }

    /// Returns whether this curve carries explicit periodic semantics.
    pub fn is_periodic(&self) -> bool {
        self.period().is_some()
    }

    /// Returns the same exact curve image with traversal direction reversed.
    ///
    /// Authored native curves reflect their public parameter mapping as
    /// `u -> start + end - u`. Retained source restrictions keep their source
    /// chart and reverse traversal independently of parameter order.
    pub fn reversed(&self) -> crate::ExactCurveResult<Self> {
        self.reversed_with_policy(&crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::reversed`] under an explicit predicate policy.
    pub(crate) fn reversed_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| self.reversed_raw(attempt))
    }

    pub(crate) fn reversed_raw(&self, policy: &CurveContext) -> ExactCurveResult<Self> {
        if self.source_range().is_some() {
            return self
                .reverse_source_range(policy)
                .map_err(|error| error.with_operation(CurveOperation2::Reversal));
        }
        if let Some(fragment) = self.retained_fragment() {
            return fragment
                .reversed()
                .map(Self::from_retained_fragment)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Reversal, self.family(), cause)
                });
        }
        let geometry = match self.geometry() {
            Some(CurveGeometry2::Line(curve)) => CurveGeometry2::Line(curve.reversed()),
            Some(CurveGeometry2::CircularArc(curve)) => {
                CurveGeometry2::CircularArc(curve.reversed())
            }
            Some(CurveGeometry2::QuadraticBezier(curve)) => CurveGeometry2::QuadraticBezier(
                curve.reversed_with_retained_provenance().map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Reversal,
                        CurveFamily2::QuadraticBezier,
                        cause,
                    )
                })?,
            ),
            Some(CurveGeometry2::CubicBezier(curve)) => {
                CurveGeometry2::CubicBezier(CubicBezier2::new(
                    curve.end().clone(),
                    curve.control2().clone(),
                    curve.control1().clone(),
                    curve.start().clone(),
                ))
            }
            Some(CurveGeometry2::RationalQuadraticBezier(curve)) => {
                CurveGeometry2::RationalQuadraticBezier(
                    RationalQuadraticBezier2::try_new_with_common_weight_sign_and_implicit_conic(
                        curve.end().clone(),
                        curve.control().clone(),
                        curve.start().clone(),
                        curve.end_weight().clone(),
                        curve.control_weight().clone(),
                        curve.start_weight().clone(),
                        curve.common_nonzero_weight_sign(policy),
                        curve.retained_implicit_quadratic_conic().cloned(),
                        curve.retained_circular_conic().cloned(),
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(
                            CurveOperation2::Reversal,
                            CurveFamily2::RationalQuadraticBezier,
                            cause,
                        )
                    })?,
                )
            }
            Some(CurveGeometry2::RationalBezier(curve)) => {
                CurveGeometry2::RationalBezier(curve.reversed())
            }
            Some(CurveGeometry2::PolynomialBSpline(curve)) => {
                CurveGeometry2::PolynomialBSpline(curve.reversed_raw(policy)?)
            }
            Some(CurveGeometry2::Nurbs(curve)) => {
                CurveGeometry2::Nurbs(curve.reversed_raw(policy)?)
            }
            None => unreachable!("retained reversal handled above"),
        };
        Ok(Self::from_geometry_with_lineage(
            geometry,
            self.data
                .lineage
                .as_ref()
                .expect("native lineage")
                .reversed(),
        ))
    }

    /// Applies an exact planar similarity while preserving curve family and source.
    pub fn transform_similarity(&self, transform: &Similarity2) -> crate::ExactCurveResult<Self> {
        self.transform_similarity_with_policy(transform, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::transform_similarity`] under an explicit predicate policy.
    pub(crate) fn transform_similarity_with_policy(
        &self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            self.transform_similarity_raw(transform, attempt)
        })
    }

    pub(crate) fn transform_similarity_raw(
        &self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if self.source_range().is_some() {
            return self
                .transform_source_range(transform, policy)
                .map_err(|error| error.with_operation(CurveOperation2::Transformation));
        }
        if let Some(fragment) = self.retained_fragment() {
            return crate::bezier_region::transform_curve_fragment_similarity(
                fragment, transform, policy,
            )
            .map(Self::from_retained_fragment);
        }
        let geometry = match self.geometry() {
            Some(CurveGeometry2::Line(curve)) => CurveGeometry2::Line(
                curve
                    .transform_similarity(transform)
                    .map_err(|cause| self.transform_error(cause))?,
            ),
            Some(CurveGeometry2::CircularArc(curve)) => CurveGeometry2::CircularArc(
                curve
                    .transform_similarity(transform)
                    .map_err(|cause| self.transform_error(cause))?,
            ),
            Some(CurveGeometry2::QuadraticBezier(curve)) => CurveGeometry2::QuadraticBezier(
                curve
                    .transform_similarity_with_retained_provenance(transform)
                    .map_err(|cause| self.transform_error(cause))?,
            ),
            Some(CurveGeometry2::CubicBezier(curve)) => {
                let points = curve
                    .control_points()
                    .map(|point| transform.transform_point(point));
                CurveGeometry2::CubicBezier(CubicBezier2::new(
                    points[0].clone(),
                    points[1].clone(),
                    points[2].clone(),
                    points[3].clone(),
                ))
            }
            Some(CurveGeometry2::RationalQuadraticBezier(curve)) => {
                let points = curve
                    .control_points()
                    .map(|point| transform.transform_point(point));
                CurveGeometry2::RationalQuadraticBezier(
                    RationalQuadraticBezier2::try_new(
                        points[0].clone(),
                        points[1].clone(),
                        points[2].clone(),
                        curve.start_weight().clone(),
                        curve.control_weight().clone(),
                        curve.end_weight().clone(),
                    )
                    .map_err(|cause| self.transform_error(cause))?,
                )
            }
            Some(CurveGeometry2::RationalBezier(curve)) => {
                CurveGeometry2::RationalBezier(curve.transform_similarity(transform))
            }
            Some(CurveGeometry2::PolynomialBSpline(curve)) => CurveGeometry2::PolynomialBSpline(
                curve.transform_similarity_raw(transform, policy)?,
            ),
            Some(CurveGeometry2::Nurbs(curve)) => {
                CurveGeometry2::Nurbs(curve.transform_similarity_raw(transform, policy)?)
            }
            None => unreachable!("retained transformation handled above"),
        };
        Ok(Self::from_geometry_with_lineage(
            geometry,
            self.data
                .lineage
                .as_ref()
                .expect("native lineage")
                .transformed(transform),
        ))
    }

    /// Splits this curve exactly at a strict interior public parameter.
    ///
    /// The two pieces are returned in traversal order. Selected parameters
    /// retain the source chart and endpoint evidence; repeated cuts share the
    /// original source. Native scalar subdivision uses `[0, 1]` domains except
    /// for splines, which retain the authored knot intervals. Inspect each
    /// result's [`Self::parameter_domain`] before evaluating it.
    ///
    /// At a discontinuous spline knot, each piece keeps its own one-sided
    /// endpoint. The returned [`CurveOutcome`] covers the complete split.
    #[inline(always)]
    pub fn split_at(&self, parameter: CurveParameter2) -> crate::ExactCurveResult<(Self, Self)> {
        self.split_at_with_policy(parameter, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::split_at`] under an explicit predicate policy.
    pub(crate) fn split_at_with_policy(
        &self,
        parameter: CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<(Self, Self)>> {
        resolve_certified_operation(policy, |attempt| {
            self.split_at_parameter(parameter, attempt)
        })
    }

    pub(crate) fn split_at_raw(
        &self,
        parameter: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<(Self, Self)> {
        let domain = self.native_parameter_domain()?;
        validate_strict_split_parameter(
            domain.start(),
            &parameter,
            domain.end(),
            self.family(),
            policy,
        )?;
        match self.geometry() {
            Some(CurveGeometry2::PolynomialBSpline(curve)) => {
                let (left, right) = curve.split_at_raw(parameter.clone(), policy)?;
                let left_lineage = self.lineage_subrange(domain.start(), &parameter)?;
                let right_lineage = self.lineage_subrange(&parameter, domain.end())?;
                Ok((
                    Self::from_geometry_with_lineage(
                        CurveGeometry2::PolynomialBSpline(left),
                        left_lineage,
                    ),
                    Self::from_geometry_with_lineage(
                        CurveGeometry2::PolynomialBSpline(right),
                        right_lineage,
                    ),
                ))
            }
            Some(CurveGeometry2::Nurbs(curve)) => {
                let (left, right) = curve.split_at_raw(parameter.clone(), policy)?;
                let left_lineage = self.lineage_subrange(domain.start(), &parameter)?;
                let right_lineage = self.lineage_subrange(&parameter, domain.end())?;
                Ok((
                    Self::from_geometry_with_lineage(CurveGeometry2::Nurbs(left), left_lineage),
                    Self::from_geometry_with_lineage(CurveGeometry2::Nurbs(right), right_lineage),
                ))
            }
            _ => Ok((
                self.subcurve_raw(domain.start().clone(), parameter.clone(), policy)?,
                self.subcurve_raw(parameter, domain.end().clone(), policy)?,
            )),
        }
    }

    /// Returns the exact curve image over a strictly ordered public range.
    ///
    /// A full-domain request returns a clone sharing retained facts. Selected
    /// ranges preserve the original source chart, curve family and endpoint
    /// evidence across all covered arc or spline spans. Traversal direction
    /// is unchanged. Native scalar ranges use `[0, 1]` result domains except
    /// for splines, which retain the authored knot interval.
    ///
    /// The returned [`CurveOutcome`] covers the complete exact extraction.
    #[inline(always)]
    pub fn subcurve(
        &self,
        start: CurveParameter2,
        end: CurveParameter2,
    ) -> crate::ExactCurveResult<Self> {
        self.subcurve_with_policy(start, end, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::subcurve`] under an explicit predicate policy.
    pub(crate) fn subcurve_with_policy(
        &self,
        start: CurveParameter2,
        end: CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        let domain = self.parameter_domain();
        if &start == domain.start() && &end == domain.end() {
            return Ok(CurveOutcome::new(
                self.clone(),
                crate::CurveCertainty::Certified,
            ));
        }
        resolve_certified_operation(policy, |attempt| {
            self.subcurve_at_parameters(start, end, attempt)
        })
    }

    pub(crate) fn subcurve_raw(
        &self,
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let domain = self.native_parameter_domain()?;
        if &start == domain.start() && &end == domain.end() {
            return Ok(self.clone());
        }
        validate_subcurve_range(
            domain.start(),
            &start,
            &end,
            domain.end(),
            self.family(),
            policy,
        )?;
        if crate::classify::compare_reals(&start, domain.start(), policy)
            == Some(std::cmp::Ordering::Equal)
            && crate::classify::compare_reals(&end, domain.end(), policy)
                == Some(std::cmp::Ordering::Equal)
        {
            return Ok(self.clone());
        }
        self.retain_root_image_injectivity(policy);
        let lineage = self.lineage_subrange(&start, &end)?;
        let geometry = match self.geometry() {
            None => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Subdivision,
                    self.family(),
                    crate::UncertaintyReason::Unsupported,
                ));
            }
            Some(CurveGeometry2::Line(curve)) => CurveGeometry2::Line(
                LineSeg2::try_new(curve.point_at(start), curve.point_at(end))
                    .map_err(|cause| self.subdivision_error(cause))?,
            ),
            Some(CurveGeometry2::CircularArc(curve)) => {
                let sub_start = self
                    .point_at_side_raw(&start, CurveParameterSide2::Automatic, policy)
                    .map_err(|error| remap_operation(error, CurveOperation2::Subdivision))?;
                let sub_end = self
                    .point_at_side_raw(&end, CurveParameterSide2::Automatic, policy)
                    .map_err(|error| remap_operation(error, CurveOperation2::Subdivision))?;
                let constructor = if curve.endpoints_on_stored_circle_are_certified() {
                    CircularArc2::new_with_certified_radius
                } else {
                    CircularArc2::new_unchecked_with_radius
                };
                CurveGeometry2::CircularArc(constructor(
                    sub_start,
                    sub_end,
                    curve.center().clone(),
                    curve.radius_squared(),
                    curve.is_clockwise(),
                    None,
                ))
            }
            Some(CurveGeometry2::QuadraticBezier(curve)) => CurveGeometry2::QuadraticBezier(
                curve
                    .subcurve_between_exact(&start, &end, policy)
                    .map_err(|cause| self.subdivision_error(cause))?,
            ),
            Some(CurveGeometry2::CubicBezier(curve)) => CurveGeometry2::CubicBezier(
                curve
                    .subcurve_between_exact(&start, &end, policy)
                    .map_err(|cause| self.subdivision_error(cause))?,
            ),
            Some(CurveGeometry2::RationalQuadraticBezier(curve)) => CurveGeometry2::from_bezier(
                match curve
                    .subcurve_between_exact_native(&start, &end, policy)
                    .map_err(|cause| self.subdivision_error(cause))?
                {
                    Classification::Decided(curve) => curve,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Subdivision,
                            self.family(),
                            reason,
                        ));
                    }
                },
            ),
            Some(CurveGeometry2::RationalBezier(curve)) => CurveGeometry2::RationalBezier(
                match curve
                    .subcurve_between_exact(&start, &end, policy)
                    .map_err(|cause| self.subdivision_error(cause))?
                {
                    Classification::Decided(curve) => curve,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Subdivision,
                            self.family(),
                            reason,
                        ));
                    }
                },
            ),
            Some(CurveGeometry2::PolynomialBSpline(curve)) => CurveGeometry2::PolynomialBSpline(
                curve
                    .subcurve_raw(start, end, policy)
                    .map_err(|error| remap_operation(error, CurveOperation2::Subdivision))?,
            ),
            Some(CurveGeometry2::Nurbs(curve)) => CurveGeometry2::Nurbs(
                curve
                    .subcurve_raw(start, end, policy)
                    .map_err(|error| remap_operation(error, CurveOperation2::Subdivision))?,
            ),
        };
        Ok(Self::from_geometry_with_lineage(geometry, lineage))
    }

    fn from_geometry_with_lineage(
        geometry: CurveGeometry2,
        lineage: CurveParameterLineage2,
    ) -> Self {
        Self {
            data: Arc::new(CurveData2 {
                carrier: CurveCarrier2::Native(geometry),
                lineage: Some(lineage),
                parameter_domain: OnceLock::new(),
                native_bezier_fragments: PolicyEvaluationCache::new(),
                rational_evaluators: PolicyEvaluationCache::new(),
                bounds: OnceLock::new(),
            }),
        }
    }

    fn lineage_subrange(
        &self,
        start: &Real,
        end: &Real,
    ) -> ExactCurveResult<CurveParameterLineage2> {
        Ok(CurveParameterLineage2 {
            root: Arc::clone(&self.data.lineage.as_ref().expect("native lineage").root),
            range: ParamRange::new(
                self.lineage_parameter_at(start)?,
                self.lineage_parameter_at(end)?,
            ),
            image_transform: self
                .data
                .lineage
                .as_ref()
                .expect("native lineage")
                .image_transform
                .clone(),
        })
    }

    pub(crate) fn lineage_parameter_at(&self, parameter: &Real) -> ExactCurveResult<Real> {
        let domain = self.native_parameter_domain()?;
        let local =
            ((parameter - domain.start()) / (domain.end() - domain.start())).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Subdivision, self.family(), cause.into())
            })?;
        Ok(self
            .data
            .lineage
            .as_ref()
            .expect("native lineage")
            .range
            .start()
            + &local
                * (self
                    .data
                    .lineage
                    .as_ref()
                    .expect("native lineage")
                    .range
                    .end()
                    - self
                        .data
                        .lineage
                        .as_ref()
                        .expect("native lineage")
                        .range
                        .start()))
    }

    fn retain_root_image_injectivity(&self, policy: &CurveContext) {
        let root = &self.data.lineage.as_ref().expect("native lineage").root;
        if root.image_is_injective.get().is_some()
            || !matches!(
                self.family(),
                CurveFamily2::QuadraticBezier | CurveFamily2::CubicBezier
            )
        {
            return;
        }
        let range = &self.data.lineage.as_ref().expect("native lineage").range;
        let certified = policy.strict_predicate_pass(|| {
            let covers_root_domain =
                (crate::classify::compare_reals(range.start(), root.domain.start(), policy)
                    == Some(std::cmp::Ordering::Equal)
                    && crate::classify::compare_reals(range.end(), root.domain.end(), policy)
                        == Some(std::cmp::Ordering::Equal))
                    || (crate::classify::compare_reals(range.start(), root.domain.end(), policy)
                        == Some(std::cmp::Ordering::Equal)
                        && crate::classify::compare_reals(
                            range.end(),
                            root.domain.start(),
                            policy,
                        ) == Some(std::cmp::Ordering::Equal));
            if !covers_root_domain {
                return false;
            }
            let Ok(Classification::Decided(evaluators)) =
                self.rational_evaluators_with_policy(policy)
            else {
                return false;
            };
            evaluators.len() == 1 && evaluators[0].has_certified_injective_axis(policy)
        });
        if certified {
            let _ = root.image_is_injective.set(true);
        }
    }

    pub(crate) fn shares_certified_parameter_lineage(&self, other: &Self) -> bool {
        match (&self.data.lineage, &other.data.lineage) {
            (Some(first), Some(second)) => {
                Arc::ptr_eq(&first.root, &second.root)
                    && first.image_transform == second.image_transform
                    && first.root.image_is_injective.get() == Some(&true)
            }
            _ => false,
        }
    }

    fn subdivision_error(&self, cause: CurveError) -> ExactCurveError {
        ExactCurveError::invalid(CurveOperation2::Subdivision, self.family(), cause)
    }

    fn transform_error(&self, cause: CurveError) -> ExactCurveError {
        ExactCurveError::invalid(CurveOperation2::Transformation, self.family(), cause)
    }

    /// Evaluates this curve at an exact parameter.
    ///
    /// Native line, arc, and Bezier parameters use `[0, 1]`. Arc parameters
    /// traverse exact rational quadratic spans in sweep order. Spline
    /// parameters use their authored knot domain. Generated curves use their
    /// retained source chart, including selected parameters; reversing such a
    /// curve changes traversal without changing that chart. The returned point
    /// retains its exact evidence even when it has no scalar coordinate view.
    pub fn point_at(&self, parameter: &CurveParameter2) -> crate::ExactCurveResult<CurvePoint2> {
        self.point_at_with_policy(parameter, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::point_at`] under an explicit predicate policy.
    pub(crate) fn point_at_with_policy(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurvePoint2>> {
        self.point_at_side_with_policy(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates an exact point with explicit spline-knot side policy.
    pub fn point_at_side(
        &self,
        parameter: &CurveParameter2,
        side: CurveParameterSide2,
    ) -> crate::ExactCurveResult<CurvePoint2> {
        self.point_at_side_with_policy(parameter, side, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::point_at_side`] under an explicit predicate policy.
    pub(crate) fn point_at_side_with_policy(
        &self,
        parameter: &CurveParameter2,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurvePoint2>> {
        resolve_certified_operation(policy, |attempt| {
            self.point_at_parameter_with_policy(parameter, side, attempt)
        })
    }

    pub(crate) fn point_at_side_raw(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        match self.geometry() {
            Some(CurveGeometry2::PolynomialBSpline(curve)) => {
                curve.point_at_side_raw(parameter, side, policy)
            }
            Some(CurveGeometry2::Nurbs(curve)) => curve.point_at_side_raw(parameter, side, policy),
            Some(geometry) => {
                let location = validate_unit_parameter(parameter, geometry.family(), policy)?;
                if let Some(endpoint) = retained_native_endpoint(geometry, location, policy) {
                    return Ok(endpoint);
                }
                match geometry {
                    CurveGeometry2::Line(curve) => Ok(curve.point_at(parameter.clone())),
                    CurveGeometry2::CircularArc(_) => {
                        let fragments = match self.native_bezier_fragments_raw(policy)? {
                            Classification::Decided(fragments) => fragments,
                            Classification::Uncertain(reason) => {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Evaluation,
                                    CurveFamily2::CircularArc,
                                    reason,
                                ));
                            }
                        };
                        evaluate_promoted_arc(fragments, parameter, policy)
                    }
                    CurveGeometry2::QuadraticBezier(curve) => Ok(curve.point_at(parameter.clone())),
                    CurveGeometry2::CubicBezier(curve) => Ok(curve.point_at(parameter.clone())),
                    CurveGeometry2::RationalQuadraticBezier(curve) => {
                        match curve.point_at(parameter.clone(), policy) {
                            Classification::Decided(point) => Ok(point),
                            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                                CurveOperation2::Evaluation,
                                CurveFamily2::RationalQuadraticBezier,
                                reason,
                            )),
                        }
                    }
                    CurveGeometry2::RationalBezier(curve) => {
                        match curve.point_at_classified(parameter, policy) {
                            Classification::Decided(point) => Ok(point),
                            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                                CurveOperation2::Evaluation,
                                CurveFamily2::RationalBezier,
                                reason,
                            )),
                        }
                    }
                    CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_) => {
                        unreachable!("spline evaluation handled before native parameter dispatch")
                    }
                }
            }
            None => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                self.family(),
                crate::UncertaintyReason::Unsupported,
            )),
        }
    }

    /// Evaluates an explicitly periodic spline at any exactly wrappable parameter.
    pub fn point_at_wrapped(&self, parameter: &Real) -> crate::ExactCurveResult<Point2> {
        self.point_at_wrapped_with_policy(parameter, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::point_at_wrapped`] under an explicit predicate policy.
    pub(crate) fn point_at_wrapped_with_policy(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        self.point_at_wrapped_side_with_policy(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates a periodic spline with explicit side selection at wrapped seams.
    pub fn point_at_wrapped_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> crate::ExactCurveResult<Point2> {
        self.point_at_wrapped_side_with_policy(parameter, side, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::point_at_wrapped_side`] under an explicit predicate policy.
    pub(crate) fn point_at_wrapped_side_with_policy(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        resolve_certified_operation(policy, |attempt| {
            self.point_at_wrapped_side_raw(parameter, side, attempt)
        })
    }

    pub(crate) fn point_at_wrapped_side_raw(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        match self.geometry() {
            Some(CurveGeometry2::PolynomialBSpline(curve)) => {
                curve.point_at_wrapped_side_raw(parameter, side, policy)
            }
            Some(CurveGeometry2::Nurbs(curve)) => {
                curve.point_at_wrapped_side_raw(parameter, side, policy)
            }
            _ => Err(ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                self.family(),
                CurveError::CurveIsNotPeriodic,
            )),
        }
    }

    /// Evaluates the exact first derivative in this curve's public parameter.
    ///
    /// Native curves use `[0, 1]`; spline curves use their authored knot
    /// domain. Promoted rational evaluators are built once per shared curve and
    /// preserve source-span parameter scaling.
    ///
    /// Selected algebraic parameters on authored spans return selected-field
    /// vectors without rounding the parameter.
    pub fn derivative_at(
        &self,
        parameter: &CurveParameter2,
    ) -> crate::ExactCurveResult<CurveVector2> {
        self.derivative_at_with_policy(parameter, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::derivative_at`] under an explicit predicate policy.
    pub(crate) fn derivative_at_with_policy(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveVector2>> {
        self.derivative_at_side_with_policy(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates an exact first derivative with explicit knot-boundary side policy.
    pub fn derivative_at_side(
        &self,
        parameter: &CurveParameter2,
        side: CurveParameterSide2,
    ) -> crate::ExactCurveResult<CurveVector2> {
        self.derivative_at_side_with_policy(parameter, side, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::derivative_at_side`] under an explicit predicate policy.
    pub(crate) fn derivative_at_side_with_policy(
        &self,
        parameter: &CurveParameter2,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveVector2>> {
        resolve_certified_operation(policy, |attempt| {
            let mut derivatives = self.general_derivatives_at_side(parameter, 1, side, attempt)?;
            Ok(derivatives.pop().expect("one derivative requested"))
        })
    }

    /// Evaluates the first periodic derivative at any wrappable parameter.
    pub fn derivative_at_wrapped(&self, parameter: &Real) -> crate::ExactCurveResult<CurveVector2> {
        self.derivative_at_wrapped_with_policy(parameter, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::derivative_at_wrapped`] under an explicit predicate policy.
    pub(crate) fn derivative_at_wrapped_with_policy(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveVector2>> {
        self.derivative_at_wrapped_side_with_policy(
            parameter,
            CurveParameterSide2::Automatic,
            policy,
        )
    }

    /// Evaluates the first periodic derivative with explicit seam-side selection.
    pub fn derivative_at_wrapped_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> crate::ExactCurveResult<CurveVector2> {
        self.derivative_at_wrapped_side_with_policy(
            parameter,
            side,
            &crate::policy::principal_context(),
        )
        .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::derivative_at_wrapped_side`] under an explicit predicate policy.
    pub(crate) fn derivative_at_wrapped_side_with_policy(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveVector2>> {
        resolve_certified_operation(policy, |attempt| {
            let mut derivatives =
                self.derivatives_at_wrapped_side_raw(parameter, 1, side, attempt)?;
            Ok(CurveVector2::represented(
                derivatives.pop().expect("one derivative requested"),
            ))
        })
    }

    /// Evaluates exact derivatives through `max_order` in the public parameter.
    ///
    /// The returned vector stores orders `1..=max_order`. Native curves use
    /// `[0, 1]`; spline curves use their authored knot domain.
    pub fn derivatives_at(
        &self,
        parameter: &CurveParameter2,
        max_order: usize,
    ) -> crate::ExactCurveResult<Vec<CurveVector2>> {
        self.derivatives_at_with_policy(parameter, max_order, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::derivatives_at`] under an explicit predicate policy.
    pub(crate) fn derivatives_at_with_policy(
        &self,
        parameter: &CurveParameter2,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveVector2>>> {
        self.derivatives_at_side_with_policy(
            parameter,
            max_order,
            CurveParameterSide2::Automatic,
            policy,
        )
    }

    /// Evaluates exact derivatives with explicit retained-fragment side policy.
    pub fn derivatives_at_side(
        &self,
        parameter: &CurveParameter2,
        max_order: usize,
        side: CurveParameterSide2,
    ) -> crate::ExactCurveResult<Vec<CurveVector2>> {
        self.derivatives_at_side_with_policy(
            parameter,
            max_order,
            side,
            &crate::policy::principal_context(),
        )
        .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::derivatives_at_side`] under an explicit predicate policy.
    pub(crate) fn derivatives_at_side_with_policy(
        &self,
        parameter: &CurveParameter2,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveVector2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.general_derivatives_at_side(parameter, max_order, side, attempt)
        })
    }

    /// Dispatches represented parameters to the scalar kernel and selected
    /// parameters to the authored span's algebraic derivative images.
    pub(super) fn general_derivatives_at_side(
        &self,
        parameter: &CurveParameter2,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveVector2>> {
        if self.source_range().is_some() {
            return self.source_range_derivatives_at(parameter, max_order, side, policy);
        }
        if self.geometry().is_none() {
            return self.retained_derivatives_at(parameter, max_order, policy);
        }
        if let Some(scalar) = parameter.scalar() {
            return Ok(self
                .derivatives_at_side_raw(scalar, max_order, side, policy)?
                .into_iter()
                .map(CurveVector2::represented)
                .collect());
        }
        self.selected_derivatives_at(parameter, max_order, side, policy)
    }

    fn selected_derivatives_at(
        &self,
        parameter: &CurveParameter2,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveVector2>> {
        let family = self.family();
        let invalid = |cause| ExactCurveError::invalid(CurveOperation2::Evaluation, family, cause);
        let blocked =
            |reason| ExactCurveError::blocked(CurveOperation2::Evaluation, family, reason);
        fn decided<T>(value: Classification<T>, family: CurveFamily2) -> ExactCurveResult<T> {
            match value {
                Classification::Decided(value) => Ok(value),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    family,
                    reason,
                )),
            }
        }
        let fragments =
            self.native_bezier_fragments_for_operation(policy, CurveOperation2::Evaluation)?;
        let evaluators =
            self.rational_evaluators_for_operation(policy, CurveOperation2::Evaluation)?;
        for (fragment, evaluator) in fragments.iter().zip(evaluators) {
            let (start, end) = fragment.parameter_range();
            let start_order = decided(
                parameter
                    .cmp_by_refinement(&CurveParameter2::from(start.clone()), policy)
                    .map_err(invalid)?,
                family,
            )?;
            let end_order = decided(
                parameter
                    .cmp_by_refinement(&CurveParameter2::from(end.clone()), policy)
                    .map_err(invalid)?,
                family,
            )?;
            // A selected root equal to a knot is that represented knot.
            if start_order.is_eq() {
                return self.general_derivatives_at_side(
                    &CurveParameter2::from(start.clone()),
                    max_order,
                    side,
                    policy,
                );
            }
            if end_order.is_eq() {
                return self.general_derivatives_at_side(
                    &CurveParameter2::from(end.clone()),
                    max_order,
                    side,
                    policy,
                );
            }
            if !(start_order.is_gt() && end_order.is_lt()) {
                continue;
            }
            let width = end - start;
            let scale = (Real::one() / &width).map_err(|cause| invalid(cause.into()))?;
            let offset = -(start * &scale);
            let local = decided(
                parameter
                    .affine_image_unbounded(&scale, &offset, policy)
                    .map_err(invalid)?,
                family,
            )?;
            let Some(crate::BezierParameter2::Algebraic(local)) = local.as_bezier_parameter()
            else {
                return Err(blocked(crate::UncertaintyReason::Unsupported));
            };
            let images = decided(
                evaluator
                    .derivatives_at_algebraic_parameter(local, max_order, policy)
                    .map_err(invalid)?,
                family,
            )?;
            // Chain rule: order k scales by (1 / width)^k.
            let mut chart_factor = Real::one();
            let mut vectors = Vec::with_capacity(images.len());
            for image in &images {
                chart_factor = &chart_factor * &scale;
                vectors.push(CurveVector2 {
                    data: CurveVectorData2::Selected {
                        vector: crate::BezierAlgebraicTangentVector2::from_image(image),
                        chart_factor: chart_factor.clone(),
                    },
                });
            }
            return Ok(vectors);
        }
        Err(invalid(CurveError::InvalidBezierParameter))
    }

    pub(crate) fn derivatives_at_side_raw(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        match self.geometry() {
            Some(CurveGeometry2::PolynomialBSpline(curve)) => {
                return curve.derivatives_at_side_raw(parameter, max_order, side, policy);
            }
            Some(CurveGeometry2::Nurbs(curve)) => {
                return curve.derivatives_at_side_raw(parameter, max_order, side, policy);
            }
            _ => {}
        }
        let fragments = match self.native_bezier_fragments_raw(policy)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    self.family(),
                    reason,
                ));
            }
        };
        let (first, last) = select_native_fragments(fragments, parameter, self.family(), policy)?;
        let first_derivatives =
            self.derivatives_on_native_fragment(first, parameter, max_order, policy)?;
        if first == last || side == CurveParameterSide2::Left {
            return Ok(first_derivatives);
        }
        let last_derivatives =
            self.derivatives_on_native_fragment(last, parameter, max_order, policy)?;
        if side == CurveParameterSide2::Right {
            return Ok(last_derivatives);
        }
        certify_matching_derivatives(first_derivatives, last_derivatives, self.family(), policy)
    }

    /// Evaluates periodic derivatives through `max_order` at any wrappable parameter.
    pub fn derivatives_at_wrapped(
        &self,
        parameter: &Real,
        max_order: usize,
    ) -> crate::ExactCurveResult<Vec<CurveVector2>> {
        self.derivatives_at_wrapped_with_policy(
            parameter,
            max_order,
            &crate::policy::principal_context(),
        )
        .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::derivatives_at_wrapped`] under an explicit predicate policy.
    pub(crate) fn derivatives_at_wrapped_with_policy(
        &self,
        parameter: &Real,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveVector2>>> {
        self.derivatives_at_wrapped_side_with_policy(
            parameter,
            max_order,
            CurveParameterSide2::Automatic,
            policy,
        )
    }

    /// Evaluates periodic derivatives with explicit side selection at wrapped seams.
    pub fn derivatives_at_wrapped_side(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
    ) -> crate::ExactCurveResult<Vec<CurveVector2>> {
        self.derivatives_at_wrapped_side_with_policy(
            parameter,
            max_order,
            side,
            &crate::policy::principal_context(),
        )
        .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::derivatives_at_wrapped_side`] under an explicit predicate policy.
    pub(crate) fn derivatives_at_wrapped_side_with_policy(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveVector2>>> {
        resolve_certified_operation(policy, |attempt| {
            Ok(self
                .derivatives_at_wrapped_side_raw(parameter, max_order, side, attempt)?
                .into_iter()
                .map(CurveVector2::represented)
                .collect())
        })
    }

    pub(crate) fn derivatives_at_wrapped_side_raw(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        match self.geometry() {
            Some(CurveGeometry2::PolynomialBSpline(curve)) => {
                curve.derivatives_at_wrapped_side_raw(parameter, max_order, side, policy)
            }
            Some(CurveGeometry2::Nurbs(curve)) => {
                curve.derivatives_at_wrapped_side_raw(parameter, max_order, side, policy)
            }
            _ => Err(ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                self.family(),
                CurveError::CurveIsNotPeriodic,
            )),
        }
    }

    fn derivatives_on_native_fragment(
        &self,
        fragment_index: usize,
        parameter: &Real,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        let fragments = match self.native_bezier_fragments_raw(policy)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    self.family(),
                    reason,
                ));
            }
        };
        let (start, end) = fragments[fragment_index].parameter_range();
        let width = end - start;
        let local = ((parameter - start) / &width).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Evaluation, self.family(), cause.into())
        })?;
        let evaluators = match self.rational_evaluators_with_policy(policy)? {
            Classification::Decided(evaluators) => evaluators,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    self.family(),
                    reason,
                ));
            }
        };
        let evaluator = &evaluators[fragment_index];
        let local_derivatives = match if max_order == 1 {
            evaluator
                .derivative_at_classified(&local, policy)
                .map(|derivative| vec![derivative])
        } else {
            evaluator.derivatives_at_classified(&local, max_order, policy)
        } {
            Classification::Decided(derivatives) => derivatives,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    self.family(),
                    reason,
                ));
            }
        };
        let inverse_width = (Real::one() / width).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Evaluation, self.family(), cause.into())
        })?;
        let mut scale = Real::one();
        Ok(local_derivatives
            .into_iter()
            .map(|derivative| {
                scale *= &inverse_width;
                derivative.scaled(&scale)
            })
            .collect())
    }

    /// Borrows conservative exact bounds computed once for this shared curve.
    pub fn bounds(&self) -> ExactCurveResult<&Aabb2> {
        match self.data.bounds.get_or_init(|| compute_curve_bounds(self)) {
            Ok(bounds) => Ok(bounds),
            Err(error) => Err(error.clone()),
        }
    }

    /// Returns retained exact native Bezier fragments for topology ingestion.
    ///
    /// Promotion runs once per shared curve object. Circular-arc, polynomial
    /// spline, and native NURBS spans preserve their source span index and
    /// exact parameter interval. The returned [`CurveOutcome`] records whether
    /// promotion consumed the `APPROXIMATE_512` terminal.
    #[inline(always)]
    pub fn native_bezier_fragments(&self) -> crate::ExactCurveResult<&[NativeBezierFragment2]> {
        self.native_bezier_fragments_with_policy(&crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::native_bezier_fragments`] under an explicit predicate policy.
    pub(crate) fn native_bezier_fragments_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<&[NativeBezierFragment2]>> {
        resolve_certified_operation(policy, |attempt| {
            self.native_bezier_fragments_for_operation(attempt, CurveOperation2::NativeTopology)
        })
    }

    #[inline]
    pub(crate) fn native_bezier_fragments_raw(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<&[NativeBezierFragment2]>> {
        Ok(
            match resolve_cached_evaluation(
                &self.data.native_bezier_fragments,
                policy,
                |attempt| promote_native_bezier_fragments(self, attempt),
            )? {
                Classification::Decided(fragments) => Classification::Decided(fragments.as_slice()),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    #[inline]
    pub(crate) fn native_bezier_fragments_for_operation(
        &self,
        policy: &CurveContext,
        operation: CurveOperation2,
    ) -> ExactCurveResult<&[NativeBezierFragment2]> {
        match self
            .native_bezier_fragments_raw(policy)
            .map_err(|error| error.with_operation(operation))?
        {
            Classification::Decided(fragments) => Ok(fragments),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, self.family(), reason))
            }
        }
    }

    pub(crate) fn rational_evaluators_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<&[RationalBezier2]>> {
        Ok(
            match resolve_cached_evaluation(&self.data.rational_evaluators, policy, |attempt| {
                let fragments = match self.native_bezier_fragments_raw(attempt)? {
                    Classification::Decided(fragments) => fragments,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                fragments
                    .iter()
                    .map(|fragment| rationalize_subcurve(fragment.native_curve(), self.family()))
                    .collect::<ExactCurveResult<Vec<_>>>()
                    .map(Classification::Decided)
            })? {
                Classification::Decided(evaluators) => {
                    Classification::Decided(evaluators.as_slice())
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    pub(crate) fn rational_evaluators_for_operation(
        &self,
        policy: &CurveContext,
        operation: CurveOperation2,
    ) -> ExactCurveResult<&[RationalBezier2]> {
        match self
            .rational_evaluators_with_policy(policy)
            .map_err(|error| error.with_operation(operation))?
        {
            Classification::Decided(evaluators) => Ok(evaluators),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, self.family(), reason))
            }
        }
    }
}

impl PartialEq for Curve2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data) || self.data.carrier == other.data.carrier
    }
}

impl CurvePath2 {
    fn from_connected_curves(
        curves: Vec<Curve2>,
        connectivity_policy: Option<CurveContext>,
        closure_policy: Option<CurveContext>,
    ) -> Self {
        Self {
            data: Arc::new(CurvePathData2 {
                curves,
                connectivity_policy,
                closure_policy,
                native_bezier_fragments: PolicyEvaluationCache::new(),
                boundary_loop: PolicyEvaluationCache::new(),
                bounds: OnceLock::new(),
            }),
        }
    }

    pub(crate) fn from_certified_closed_curves(curves: Vec<Curve2>, policy: CurveContext) -> Self {
        debug_assert!(!curves.is_empty());
        Self::from_connected_curves(curves, Some(policy), Some(policy))
    }

    pub(crate) fn from_structurally_closed_curves(curves: Vec<Curve2>) -> Self {
        debug_assert!(!curves.is_empty());
        debug_assert!(
            curves
                .iter()
                .zip(curves.iter().cycle().skip(1))
                .take(curves.len())
                .all(|(left, right)| left.end() == right.start())
        );
        Self::from_connected_curves(
            curves,
            Some(CurveContext::STRICT),
            Some(CurveContext::STRICT),
        )
    }

    /// Constructs a nonempty ordered path with exactly connected endpoints.
    pub fn try_new(curves: Vec<Curve2>) -> ExactCurveResult<Self> {
        Self::try_new_with_policy(curves, &crate::policy::principal_context())
            .map(CurveOutcome::into_value)
    }

    /// [`Self::try_new`] under an explicit endpoint policy.
    ///
    /// The outcome reports when connectivity consumed the authorized 512-bit
    /// terminal. No approximate coordinate replacement is performed.
    pub(crate) fn try_new_with_policy(
        curves: Vec<Curve2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| Self::try_new_raw(curves, attempt))
    }

    pub(crate) fn try_new_raw(
        curves: Vec<Curve2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if curves.is_empty() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                CurveFamily2::Line,
                CurveError::EmptyCurvePath,
            ));
        }
        let strict_closure_certified =
            curves.last().expect("nonempty path").end() == curves[0].start();
        let mut strict_connectivity_certified = true;
        for adjacent in curves.windows(2) {
            if adjacent[0].end() == adjacent[1].start() {
                continue;
            }
            let equality = adjacent[0]
                .end()
                .coincides_with(&adjacent[1].start(), policy);
            strict_connectivity_certified &= equality.certainty == crate::CurveCertainty::Certified;
            match equality.into_value() {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Construction,
                        adjacent[1].family(),
                        CurveError::DisconnectedCurvePath,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Construction,
                        adjacent[1].family(),
                        reason,
                    ));
                }
            }
        }
        Ok(Self::from_connected_curves(
            curves,
            Some(if strict_connectivity_certified {
                policy.strict_counterpart()
            } else {
                policy.retained_object_policy()
            }),
            strict_closure_certified.then_some(CurveContext::STRICT),
        ))
    }

    /// Returns curves in traversal order.
    pub fn curves(&self) -> &[Curve2] {
        &self.data.curves
    }

    /// Returns the exact path start point.
    pub fn start(&self) -> CurvePoint2 {
        self.data.curves[0].start()
    }

    /// Returns the exact path end point.
    pub fn end(&self) -> CurvePoint2 {
        self.data
            .curves
            .last()
            .expect("validated path is nonempty")
            .end()
    }

    /// Returns the same connected path with traversal direction reversed.
    pub fn reversed(&self) -> crate::ExactCurveResult<Self> {
        self.reversed_with_policy(&crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::reversed`] under an explicit predicate policy.
    pub(crate) fn reversed_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| self.reversed_raw(attempt))
    }

    pub(crate) fn reversed_raw(&self, policy: &CurveContext) -> ExactCurveResult<Self> {
        let curves = self
            .curves()
            .iter()
            .rev()
            .map(|curve| curve.reversed_raw(policy))
            .collect::<ExactCurveResult<Vec<_>>>()?;
        Ok(Self::from_connected_curves(
            curves,
            self.data.connectivity_policy,
            self.data.closure_policy,
        ))
    }

    /// Applies an exact planar similarity to every curve in the connected path.
    pub fn transform_similarity(&self, transform: &Similarity2) -> crate::ExactCurveResult<Self> {
        self.transform_similarity_with_policy(transform, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::transform_similarity`] under an explicit predicate policy.
    pub(crate) fn transform_similarity_with_policy(
        &self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            self.transform_similarity_raw(transform, attempt)
        })
    }

    pub(crate) fn transform_similarity_raw(
        &self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let curves = self
            .curves()
            .iter()
            .map(|curve| curve.transform_similarity_raw(transform, policy))
            .collect::<ExactCurveResult<Vec<_>>>()?;
        Ok(Self::from_connected_curves(
            curves,
            self.data.connectivity_policy,
            self.data.closure_policy,
        ))
    }

    /// Solves and applies an exact chord-setback chamfer at one path vertex.
    ///
    /// Each nonnegative setback is the Euclidean chord distance from the
    /// original corner along its incident curve. All selected cuts and their
    /// point witnesses remain exact in the returned path and can be reused by
    /// later operations. Finite trimming searches the complete authored
    /// domain, including selected major arcs and every spline span. Internal
    /// chart boundaries do not become trim limits or duplicate contacts;
    /// reconstruction retains the chart on the surviving side of each cut.
    ///
    /// [`CurveCornerMode2::TrimOrExtend`] includes incident support extensions,
    /// with rational extensions stopping at the first pole. Circular
    /// extensions exclude every closed authored sweep on the same support,
    /// including its endpoints and contacts on other spline charts. Exact
    /// circle contacts retain their point evidence through chart inversion
    /// and reconstruction. Candidates are returned in deterministic order.
    /// Reconstruction shares the exact chain
    /// machinery used by regions, with a native scalar specialization where
    /// its coordinates and parameters are already available.
    pub fn chamfer_vertex_by_setbacks(
        &self,
        vertex_index: usize,
        previous_setback: Real,
        next_setback: Real,
        mode: CurveCornerMode2,
    ) -> crate::ExactCurveResult<CurveCornerSolutions2<Self>> {
        self.chamfer_vertex_by_setbacks_with_policy(
            vertex_index,
            previous_setback,
            next_setback,
            mode,
            &crate::policy::principal_context(),
        )
        .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::chamfer_vertex_by_setbacks`] under an explicit predicate policy.
    pub(crate) fn chamfer_vertex_by_setbacks_with_policy(
        &self,
        vertex_index: usize,
        previous_setback: Real,
        next_setback: Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveCornerSolutions2<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            self.chamfer_vertex_by_setbacks_raw(
                vertex_index,
                previous_setback,
                next_setback,
                mode,
                attempt,
            )
        })
    }

    pub(crate) fn chamfer_vertex_by_setbacks_raw(
        &self,
        vertex_index: usize,
        previous_setback: Real,
        next_setback: Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveCornerSolutions2<Self>> {
        let (previous_index, next_index) =
            self.corner_curve_indices(vertex_index, CurveOperation2::Chamfer, policy)?;
        let previous = &self.data.curves[previous_index];
        let next = &self.data.curves[next_index];
        let previous_sign = validate_corner_design_value(
            &previous_setback,
            CurveOperation2::Chamfer,
            previous.family(),
            policy,
        )?;
        let next_sign = validate_corner_design_value(
            &next_setback,
            CurveOperation2::Chamfer,
            next.family(),
            policy,
        )?;
        if previous_sign == RealSign::Zero && next_sign == RealSign::Zero {
            return Ok(CurveCornerSolutions2::NoSolution(
                CurveCornerNoSolution2::ZeroDesignValue,
            ));
        }
        let mut previous_source =
            crate::bezier_region::CornerCarrierPreparation2::from_curve(previous, true);
        let mut next_source =
            crate::bezier_region::CornerCarrierPreparation2::from_curve(next, false);
        previous_source.prepare(CurveOperation2::Chamfer, policy)?;
        next_source.prepare(CurveOperation2::Chamfer, policy)?;
        let previous_carrier =
            previous_source.exact_carrier(true, CurveOperation2::Chamfer, policy)?;
        let next_carrier = next_source.exact_carrier(false, CurveOperation2::Chamfer, policy)?;
        let previous_retained_arc = previous_carrier.retained_rational_arc().cloned();
        let next_retained_arc = next_carrier.retained_rational_arc().cloned();
        let previous_cuts = previous.chamfer_cuts_in_authored_domain(
            previous_carrier,
            previous_source.source_chart(),
            &previous_setback,
            previous_sign,
            true,
            mode,
            policy,
        );
        if matches!(&previous_cuts, Ok(cuts) if cuts.is_empty()) {
            return Ok(CurveCornerSolutions2::NoSolution(
                CurveCornerNoSolution2::OutsideTrimDomain,
            ));
        }
        let next_cuts = next.chamfer_cuts_in_authored_domain(
            next_carrier,
            next_source.source_chart(),
            &next_setback,
            next_sign,
            false,
            mode,
            policy,
        );
        let solutions = combine_chamfer_cuts(previous_cuts, next_cuts, previous.family(), policy)?;
        let solutions = try_map_corner_solutions(solutions, |solution| {
            if previous_index == next_index
                && !previous.corner_cuts_leave_authored_interval(
                    &solution.previous,
                    &solution.next,
                    CurveOperation2::Chamfer,
                    policy,
                )?
            {
                return Ok(None);
            }
            if !corner_has_native_reconstruction(
                previous,
                &solution.previous,
                previous_retained_arc.as_deref(),
            ) || !corner_has_native_reconstruction(
                next,
                &solution.next,
                next_retained_arc.as_deref(),
            ) {
                return self.reconstruct_selected_chamfer(
                    previous_index,
                    next_index,
                    solution,
                    previous_retained_arc.as_deref(),
                    next_retained_arc.as_deref(),
                    policy,
                );
            }
            let previous_point = solution.previous.exact_point().cloned().ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Chamfer,
                    previous.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })?;
            let next_point = solution.next.exact_point().cloned().ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Chamfer,
                    next.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })?;
            let chamfer = Curve2::from(LineSeg2::try_new(previous_point, next_point).map_err(
                |cause| {
                    ExactCurveError::invalid(CurveOperation2::Chamfer, previous.family(), cause)
                },
            )?);
            if previous_index == next_index {
                return self
                    .with_single_curve_corner_replaced(
                        previous_index,
                        &solution.previous,
                        &solution.next,
                        chamfer,
                        CurveOperation2::Chamfer,
                        policy,
                    )
                    .map(Some);
            }
            let previous_trim = materialize_corner_side(
                previous,
                &solution.previous,
                true,
                CurveOperation2::Chamfer,
                policy,
            )?;
            let next_trim = materialize_corner_side(
                next,
                &solution.next,
                false,
                CurveOperation2::Chamfer,
                policy,
            )?;
            self.with_corner_replaced(
                vertex_index,
                previous_index,
                next_index,
                previous_trim,
                chamfer,
                next_trim,
                CurveOperation2::Chamfer,
                policy,
            )
            .map(Some)
        })?;
        Ok(compact_optional_corner_solutions(solutions))
    }

    /// Solves and applies an exact circular fillet of the requested radius.
    ///
    /// Fillet centers are intersections of equal signed-radius offsets of the
    /// two incident curves. Both modes search every finite rational chart of
    /// selected source restrictions, polynomial B-splines and NURBS. Authored
    /// endpoints remain open; an internal seam belongs to the side surviving
    /// the trim, including its one-sided tangent. Distinct source locations
    /// remain distinct candidates, and two cuts on one closed authored curve
    /// must leave a nonempty interval.
    ///
    /// Lines, circular arcs and retained exact line/circle images preserve
    /// their direct support kernels. General Bezier pairs use certified
    /// analytic-parallel incidence and retain selected contact evidence for
    /// subsequent operations. A radius leaving a continuous family requires
    /// additional exact center or contact constraints in [`CurveFillet2`].
    /// Contact parameters use the incident input curves' charts. Stationary
    /// contacts retain the one-sided tangent of the source that survives the
    /// cut; a vanishing authored derivative does not discard that contact.
    ///
    /// `TrimOrExtend` additionally extends each incident boundary chart;
    /// other charts keep their finite domains. Bezier extensions search the
    /// endpoint-adjacent regular cells, stopping at poles or source-speed
    /// zeros. A circular contact already owned by the closed authored sweep
    /// cannot be republished as an extension. Certified circular charts use
    /// their full projective continuation even with a noncircular partner.
    /// A selected incident restriction whose carrier does not retain that
    /// circular support still uses its analytic incident domain.
    pub fn fillet_vertex(
        &self,
        vertex_index: usize,
        request: &CurveFillet2,
        mode: CurveCornerMode2,
    ) -> crate::ExactCurveResult<CurveCornerSolutions2<Self>> {
        self.fillet_vertex_with_policy(
            vertex_index,
            request,
            mode,
            &crate::policy::principal_context(),
        )
        .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::fillet_vertex`] under an explicit predicate policy.
    pub(crate) fn fillet_vertex_with_policy(
        &self,
        vertex_index: usize,
        request: &CurveFillet2,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveCornerSolutions2<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            self.fillet_vertex_raw(vertex_index, request, mode, attempt)
        })
    }

    pub(crate) fn fillet_vertex_raw(
        &self,
        vertex_index: usize,
        request: &CurveFillet2,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveCornerSolutions2<Self>> {
        let radius = &request.radius;
        let (previous_index, next_index) =
            self.corner_curve_indices(vertex_index, CurveOperation2::Fillet, policy)?;
        let previous = &self.data.curves[previous_index];
        let next = &self.data.curves[next_index];
        let radius_sign = validate_corner_design_value(
            radius,
            CurveOperation2::Fillet,
            previous.family(),
            policy,
        )?;
        if radius_sign == RealSign::Zero {
            return Ok(CurveCornerSolutions2::NoSolution(
                CurveCornerNoSolution2::ZeroDesignValue,
            ));
        }
        if let Some(solutions) = self.fillets_in_authored_domain(
            vertex_index,
            previous_index,
            next_index,
            request,
            mode,
            policy,
        )? {
            return Ok(solutions);
        }
        let mut previous_source =
            crate::bezier_region::CornerCarrierPreparation2::from_curve(previous, true);
        let mut next_source =
            crate::bezier_region::CornerCarrierPreparation2::from_curve(next, false);
        previous_source.prepare(CurveOperation2::Fillet, policy)?;
        next_source.prepare(CurveOperation2::Fillet, policy)?;
        let previous_carrier =
            previous_source.exact_carrier(true, CurveOperation2::Fillet, policy)?;
        let next_carrier = next_source.exact_carrier(false, CurveOperation2::Fillet, policy)?;
        let previous_retained_arc = previous_carrier.retained_rational_arc().cloned();
        let next_retained_arc = next_carrier.retained_rational_arc().cloned();
        let placement = curve_fillet::PathFilletPlacement2::new(
            self,
            vertex_index,
            [previous_index, next_index],
            [previous_source.source_chart(), next_source.source_chart()],
            [None, None],
            [previous_retained_arc, next_retained_arc],
            [
                previous_source.promoted_parallel(),
                next_source.promoted_parallel(),
            ],
            [None, None],
        );
        let binding = placement.constraints(request);
        let solutions = solve_exact_fillet_corner(
            previous_carrier,
            next_carrier,
            radius,
            radius_sign,
            mode,
            false,
            previous.family(),
            next.family(),
            Some(&binding),
            policy,
        )?;
        Ok(compact_optional_corner_solutions(try_map_corner_solutions(
            solutions,
            |solution| placement.publish(solution, radius, policy),
        )?))
    }

    fn corner_curve_indices(
        &self,
        vertex_index: usize,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<(usize, usize)> {
        let curve_count = self.data.curves.len();
        if vertex_index >= curve_count {
            return Err(ExactCurveError::invalid(
                operation,
                self.data.curves[0].family(),
                CurveError::InvalidCurveRange,
            ));
        }
        if vertex_index == 0 {
            certify_closed_path(self, operation, policy)?;
            return Ok((curve_count - 1, 0));
        }
        Ok((vertex_index - 1, vertex_index))
    }

    fn with_single_curve_corner_replaced(
        &self,
        curve_index: usize,
        previous_cut: &CornerCut2,
        next_cut: &CornerCut2,
        inserted: Curve2,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let body = materialize_single_curve_corner_body(
            &self.data.curves[curve_index],
            previous_cut,
            next_cut,
            operation,
            policy,
        )?;
        let mut curves = Vec::with_capacity(body.curve_count() + 1);
        curves.push(inserted);
        body.append_to(&mut curves);
        Self::try_new_raw(curves, policy).map_err(|error| remap_operation(error, operation))
    }

    #[allow(clippy::too_many_arguments)]
    fn with_corner_replaced(
        &self,
        vertex_index: usize,
        previous_index: usize,
        next_index: usize,
        previous_trim: MaterializedCornerSide2,
        inserted: Curve2,
        next_trim: MaterializedCornerSide2,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let mut curves = Vec::with_capacity(
            self.data.curves.len()
                + previous_trim.extra_curve_count()
                + next_trim.extra_curve_count()
                + 1,
        );
        if vertex_index == 0 {
            curves.push(inserted);
            debug_assert_ne!(previous_index, next_index);
            next_trim.append_to(&mut curves);
            if next_index + 1 < previous_index {
                curves.extend(
                    self.data.curves[next_index + 1..previous_index]
                        .iter()
                        .cloned(),
                );
            }
            previous_trim.append_to(&mut curves);
        } else {
            curves.extend(self.data.curves[..previous_index].iter().cloned());
            previous_trim.append_to(&mut curves);
            curves.push(inserted);
            next_trim.append_to(&mut curves);
            curves.extend(self.data.curves[next_index + 1..].iter().cloned());
        }
        Self::try_new_raw(curves, policy).map_err(|error| remap_operation(error, operation))
    }

    /// Borrows conservative exact bounds computed once across all path curves.
    pub fn bounds(&self) -> ExactCurveResult<&Aabb2> {
        match self.data.bounds.get_or_init(|| {
            let mut bounds = self.data.curves[0].bounds()?.clone();
            for curve in &self.data.curves[1..] {
                bounds = decided_bounds(bounds.union(curve.bounds()?), curve.family())?;
            }
            Ok(bounds)
        }) {
            Ok(bounds) => Ok(bounds),
            Err(error) => Err(error.clone()),
        }
    }

    /// Classifies an exact point against this closed path.
    ///
    /// Native full circles use their radial predicate directly. Other paths
    /// reuse the retained exact Bezier boundary classifier. The returned
    /// [`CurveOutcome`] records whether the complete classification consumed
    /// the `APPROXIMATE_512` terminal.
    pub fn classify_point(
        &self,
        point: &CurvePoint2,
    ) -> crate::ExactCurveResult<ContourPointLocation> {
        self.classify_point_with_policy(point, &crate::policy::principal_context())
            .and_then(|outcome| {
                crate::ExactCurveError::decided(
                    crate::CurveOperation2::Classification,
                    outcome.into_value(),
                )
            })
    }

    /// [`Self::classify_point`] under an explicit predicate policy.
    pub(crate) fn classify_point_with_policy(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<ContourPointLocation>>> {
        resolve_certified_operation(policy, |attempt| {
            if let Some(coordinates) = point.coordinates() {
                return self.classify_point_raw(coordinates, attempt);
            }
            // A path retains its trace, including zero-area retracing. Use its
            // raw loop and parity semantics; region regularization could erase it.
            let raw = match crate::CurveRegion2::try_from_boundary_paths_raw(
                std::slice::from_ref(self),
                attempt,
            ) {
                Ok(raw) => raw,
                Err(ExactCurveError::Blocked(blocker)) => {
                    return Ok(Classification::Uncertain(blocker.reason()));
                }
                Err(error) => {
                    return Err(remap_operation(error, CurveOperation2::Classification));
                }
            };
            crate::bezier_region::classify_point_evidence_against_retained_loop(
                &raw, 0, point, attempt,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Classification,
                    self.curves()[0].family(),
                    cause,
                )
            })
        })
    }

    pub(crate) fn classify_point_raw(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<ContourPointLocation>> {
        if let [curve] = self.curves()
            && let Some(CurveGeometry2::CircularArc(arc)) = curve.geometry()
            && crate::classify::is_zero(&arc.start().distance_squared(arc.end()), policy)
                == Some(true)
        {
            let radial_delta = point.distance_squared(arc.center()) - arc.radius_squared_ref();
            return Ok(match crate::classify::real_sign(&radial_delta, policy) {
                Some(hyperreal::RealSign::Negative) => {
                    Classification::Decided(ContourPointLocation::Inside)
                }
                Some(hyperreal::RealSign::Zero) => {
                    Classification::Decided(ContourPointLocation::Boundary)
                }
                Some(hyperreal::RealSign::Positive) => {
                    Classification::Decided(ContourPointLocation::Outside)
                }
                None => Classification::Uncertain(crate::UncertaintyReason::RealSign),
            });
        }
        if let Some((arc_curve, arc, chord)) = native_arc_chord_path(self) {
            match classify_native_arc_chord_path(arc_curve, arc, chord, point, policy)? {
                Classification::Decided(location) => {
                    return Ok(Classification::Decided(location));
                }
                Classification::Uncertain(_) => {}
            }
        }

        let boundary = match self
            .boundary_loop_raw(policy)
            .map_err(|error| remap_operation(error, CurveOperation2::Classification))?
        {
            Classification::Decided(boundary) => boundary,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        boundary.classify_point_raw(point, policy).map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Classification,
                self.curves()[0].family(),
                cause,
            )
        })
    }

    /// Promotes this path once and borrows exact native Bezier fragments in traversal order.
    ///
    /// The returned [`CurveOutcome`] records whether promotion consumed the
    /// `APPROXIMATE_512` terminal.
    #[inline(always)]
    pub fn native_bezier_fragments(&self) -> crate::ExactCurveResult<&[NativeBezierFragment2]> {
        self.native_bezier_fragments_with_policy(&crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::native_bezier_fragments`] under an explicit predicate policy.
    pub(crate) fn native_bezier_fragments_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<&[NativeBezierFragment2]>> {
        resolve_certified_operation(policy, |attempt| {
            match self.native_bezier_fragments_raw(attempt)? {
                Classification::Decided(fragments) => Ok(fragments),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::NativeTopology,
                    self.data.curves[0].family(),
                    reason,
                )),
            }
        })
    }

    #[inline]
    pub(crate) fn native_bezier_fragments_raw(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<&[NativeBezierFragment2]>> {
        Ok(
            match resolve_cached_evaluation(
                &self.data.native_bezier_fragments,
                policy,
                |attempt| {
                    let mut capacity = 0_usize;
                    for curve in &self.data.curves {
                        let native = match curve.native_bezier_fragments_raw(attempt)? {
                            Classification::Decided(native) => native,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        capacity += native.len();
                    }
                    let mut fragments = Vec::with_capacity(capacity);
                    for curve in &self.data.curves {
                        let Classification::Decided(native) =
                            curve.native_bezier_fragments_raw(attempt)?
                        else {
                            unreachable!("the capacity pass decided every shared curve promotion");
                        };
                        fragments.extend_from_slice(native);
                    }
                    Ok(Classification::Decided(fragments))
                },
            )? {
                Classification::Decided(fragments) => Classification::Decided(fragments.as_slice()),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    /// Borrows the cached exact boundary of this closed path.
    ///
    /// Authored spans and generated selected curves retain their exact support,
    /// parameter and endpoint evidence. The outcome records any consumption of
    /// the `APPROXIMATE_512` terminal while validating the closed chain.
    pub fn boundary_loop(&self) -> crate::ExactCurveResult<&CurveRegionBoundaryLoop2> {
        self.boundary_loop_with_policy(&crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::boundary_loop`] under an explicit predicate policy.
    pub(crate) fn boundary_loop_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<&CurveRegionBoundaryLoop2>> {
        resolve_certified_operation(policy, |attempt| match self.boundary_loop_raw(attempt)? {
            Classification::Decided(boundary) => Ok(boundary),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Arrangement,
                self.data.curves[0].family(),
                reason,
            )),
        })
    }

    pub(crate) fn boundary_loop_raw(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<&CurveRegionBoundaryLoop2>> {
        resolve_cached_evaluation(&self.data.boundary_loop, policy, |attempt| {
            CurveRegionBoundaryLoop2::from_path(self, attempt)
        })
        .map_err(|error| error.with_operation(CurveOperation2::Arrangement))
    }
}

fn replay_path_certificate(retained: Option<CurveContext>, policy: &CurveContext) -> bool {
    let Some(retained) = retained.filter(|retained| policy.accepts_retained_policy(*retained))
    else {
        return false;
    };
    if retained.permits_approximate_512() {
        policy.observe_approximate_512();
    }
    true
}

pub(crate) fn validate_closed_curve_path_connectivity(
    path: &CurvePath2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<()>> {
    match validate_curve_path_connectivity(path, policy)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    match curve_path_is_closed(path, policy) {
        Classification::Decided(true) => Ok(Classification::Decided(())),
        Classification::Decided(false) => Err(ExactCurveError::invalid(
            CurveOperation2::Arrangement,
            path.curves()[0].family(),
            CurveError::OpenCurvePath,
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn validate_curve_path_connectivity(
    path: &CurvePath2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<()>> {
    if !replay_path_certificate(path.data.connectivity_policy, policy) {
        for adjacent in path.curves().windows(2) {
            match curve_path_points_equal(adjacent[0].end(), adjacent[1].start(), policy) {
                Some(true) => {}
                Some(false) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Arrangement,
                        adjacent[1].family(),
                        CurveError::DisconnectedCurvePath,
                    ));
                }
                None => {
                    return Ok(Classification::Uncertain(
                        crate::UncertaintyReason::RealSign,
                    ));
                }
            }
        }
    }
    Ok(Classification::Decided(()))
}

pub(crate) fn curve_path_is_closed(
    path: &CurvePath2,
    policy: &CurveContext,
) -> Classification<bool> {
    if replay_path_certificate(path.data.closure_policy, policy) {
        return Classification::Decided(true);
    }
    match curve_path_points_equal(path.end(), path.start(), policy) {
        Some(equal) => Classification::Decided(equal),
        None => Classification::Uncertain(crate::UncertaintyReason::RealSign),
    }
}

fn curve_path_points_equal(
    left: CurvePoint2,
    right: CurvePoint2,
    policy: &CurveContext,
) -> Option<bool> {
    match left.same_point(&right, policy) {
        Classification::Decided(equal) => Some(equal),
        Classification::Uncertain(_) => None,
    }
}

fn native_arc_chord_path(path: &CurvePath2) -> Option<(&Curve2, &CircularArc2, &LineSeg2)> {
    if path.start() != path.end() {
        return None;
    }
    match path.curves() {
        [first, second] => match (first.geometry(), second.geometry()) {
            (Some(CurveGeometry2::CircularArc(arc)), Some(CurveGeometry2::Line(chord))) => {
                Some((first, arc, chord))
            }
            (Some(CurveGeometry2::Line(chord)), Some(CurveGeometry2::CircularArc(arc))) => {
                Some((second, arc, chord))
            }
            _ => None,
        },
        _ => None,
    }
}

fn classify_native_arc_chord_path(
    arc_curve: &Curve2,
    arc: &CircularArc2,
    chord: &LineSeg2,
    point: &Point2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<ContourPointLocation>> {
    let radial_delta = point.distance_squared(arc.center()) - arc.radius_squared_ref();
    match crate::classify::real_sign(&radial_delta, policy) {
        Some(RealSign::Positive) => {
            return Ok(Classification::Decided(ContourPointLocation::Outside));
        }
        Some(RealSign::Zero) => {
            return Ok(match arc.contains_sweep_point(point, policy) {
                Classification::Decided(true) => {
                    Classification::Decided(ContourPointLocation::Boundary)
                }
                Classification::Decided(false) => {
                    Classification::Decided(ContourPointLocation::Outside)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        Some(RealSign::Negative) => {}
        None => {
            return Ok(Classification::Uncertain(
                crate::UncertaintyReason::RealSign,
            ));
        }
    }

    let point_side = match chord.classify_point(point, policy) {
        Classification::Decided(LineSide::On) => {
            return Ok(Classification::Decided(ContourPointLocation::Boundary));
        }
        Classification::Decided(side) => side,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let representative = match arc.representative_point(policy).map_err(|cause| {
        ExactCurveError::invalid(CurveOperation2::NativeTopology, arc_curve.family(), cause)
    })? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let arc_side = match chord.classify_point(&representative, policy) {
        Classification::Decided(LineSide::On) => {
            return Err(ExactCurveError::invalid(
                CurveOperation2::NativeTopology,
                arc_curve.family(),
                CurveError::Topology(
                    "circular-segment arc representative lies on its chord".into(),
                ),
            ));
        }
        Classification::Decided(side) => side,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(if point_side == arc_side {
        ContourPointLocation::Inside
    } else {
        ContourPointLocation::Outside
    }))
}

impl PartialEq for CurvePath2 {
    fn eq(&self, other: &Self) -> bool {
        self.data.curves == other.data.curves
    }
}

impl From<LineSeg2> for Curve2 {
    fn from(value: LineSeg2) -> Self {
        Self::new(CurveGeometry2::Line(value))
    }
}

impl From<CircularArc2> for Curve2 {
    fn from(value: CircularArc2) -> Self {
        Self::new(CurveGeometry2::CircularArc(value))
    }
}

impl From<QuadraticBezier2> for Curve2 {
    fn from(value: QuadraticBezier2) -> Self {
        Self::new(CurveGeometry2::QuadraticBezier(value))
    }
}

impl From<CubicBezier2> for Curve2 {
    fn from(value: CubicBezier2) -> Self {
        Self::new(CurveGeometry2::CubicBezier(value))
    }
}

impl From<RationalQuadraticBezier2> for Curve2 {
    fn from(value: RationalQuadraticBezier2) -> Self {
        Self::new(CurveGeometry2::RationalQuadraticBezier(value))
    }
}

impl From<CurveGeometry2> for Curve2 {
    fn from(value: CurveGeometry2) -> Self {
        Self::new(value)
    }
}

impl From<BezierSubcurve2> for Curve2 {
    fn from(value: BezierSubcurve2) -> Self {
        Self::new(CurveGeometry2::from_bezier(value))
    }
}

impl From<RationalBezier2> for Curve2 {
    fn from(value: RationalBezier2) -> Self {
        Self::new(CurveGeometry2::RationalBezier(value))
    }
}

impl From<PolynomialSplineCurve2> for Curve2 {
    fn from(value: PolynomialSplineCurve2) -> Self {
        Self::new(CurveGeometry2::PolynomialBSpline(value))
    }
}

impl From<NurbsCurve2> for Curve2 {
    fn from(value: NurbsCurve2) -> Self {
        Self::new(CurveGeometry2::Nurbs(value))
    }
}

impl From<crate::BezierAlgebraicChord2> for Curve2 {
    fn from(value: crate::BezierAlgebraicChord2) -> Self {
        Self::from_retained_fragment(crate::BezierSplitFragment2::AlgebraicChord(value))
    }
}

impl From<crate::BezierParallelFragment2> for Curve2 {
    fn from(value: crate::BezierParallelFragment2) -> Self {
        Self::from_retained_fragment(crate::BezierSplitFragment2::AnalyticParallel(value))
    }
}

impl From<crate::BezierAlgebraicCuspSemicircleFragment2> for Curve2 {
    fn from(value: crate::BezierAlgebraicCuspSemicircleFragment2) -> Self {
        Self::from_retained_fragment(crate::BezierSplitFragment2::AlgebraicCuspSemicircle(value))
    }
}

impl From<crate::bezier_split::BezierSelectedFiberFragment2> for Curve2 {
    fn from(value: crate::bezier_split::BezierSelectedFiberFragment2) -> Self {
        Self::from_retained_fragment(crate::BezierSplitFragment2::SelectedFiber(value))
    }
}

impl CurveSpanRange2 {
    pub(crate) fn from_affine_chart(scale: &Real, offset: &Real) -> Self {
        Self {
            start: offset.clone(),
            end: offset + scale,
        }
    }

    /// Returns the public parameter images of local zero and one, in that order.
    pub fn endpoints(&self) -> (&Real, &Real) {
        (&self.start, &self.end)
    }
}

impl NativeBezierFragment2 {
    /// Returns the promoted exact native curve geometry.
    pub fn curve(&self) -> CurveGeometry2 {
        CurveGeometry2::from_bezier(self.curve.clone())
    }

    pub(crate) const fn native_curve(&self) -> &BezierSubcurve2 {
        &self.curve
    }

    /// Returns this span's exact interval in the top-level curve parameterization.
    pub fn parameter_range(&self) -> (&Real, &Real) {
        self.span_range.endpoints()
    }

    /// Returns this span's exact public parameter interval.
    pub const fn span_range(&self) -> &CurveSpanRange2 {
        &self.span_range
    }

    /// Returns whether one exact coordinate certifies this fragment's image as
    /// injective on its complete local parameter interval.
    ///
    /// A `false` result is deliberately only a missing certificate; callers
    /// that require simple-path topology must retain that distinction instead
    /// of assuming the fragment self-intersects.
    pub fn has_certified_injective_axis(&self, policy: &CurveContext) -> ExactCurveResult<bool> {
        Ok(
            rationalize_subcurve(&self.curve, CurveFamily2::RationalBezier)?
                .has_certified_injective_axis(policy),
        )
    }

    /// Publishes the prepared span as an exact curve over `[0, 1]`.
    ///
    /// The result preserves the span's native geometry and original parameter
    /// lineage. It shares the control net and root certificates, without
    /// reconstructing a spline or retaining its former owner.
    pub fn into_curve(self) -> Curve2 {
        Curve2::from_geometry_with_lineage(CurveGeometry2::from_bezier(self.curve), self.lineage)
    }
}

fn compute_curve_bounds(curve: &Curve2) -> ExactCurveResult<Aabb2> {
    let policy = crate::CurveContext::STRICT;
    if let Some(spans) = curve.restricted_source_spans(&policy, CurveOperation2::NativeTopology)? {
        let mut bounds = decided_bounds(
            crate::bezier_region::retained_fragment_query_bounds(&spans[0].fragment, &policy),
            curve.family(),
        )?;
        for span in &spans[1..] {
            let next = decided_bounds(
                crate::bezier_region::retained_fragment_query_bounds(&span.fragment, &policy),
                curve.family(),
            )?;
            bounds = decided_bounds(bounds.union(&next), curve.family())?;
        }
        return Ok(bounds);
    }
    if let Some(fragment) = curve.retained_fragment() {
        return decided_bounds(
            crate::bezier_region::retained_fragment_query_bounds(fragment, &policy),
            curve.family(),
        );
    }
    match curve.geometry() {
        Some(CurveGeometry2::Line(line)) => decided_bounds(Aabb2::from_line(line), curve.family()),
        Some(CurveGeometry2::CircularArc(arc)) => decided_bounds(
            Aabb2::from_arc(arc).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::NativeTopology, curve.family(), cause)
            })?,
            curve.family(),
        ),
        _ => {
            let fragments = curve
                .native_bezier_fragments_for_operation(&policy, CurveOperation2::NativeTopology)?;
            let mut bounds = decided_subcurve_bounds(fragments[0].native_curve(), curve.family())?;
            for fragment in &fragments[1..] {
                let fragment_bounds =
                    decided_subcurve_bounds(fragment.native_curve(), curve.family())?;
                bounds = decided_bounds(bounds.union(&fragment_bounds), curve.family())?;
            }
            Ok(bounds)
        }
    }
}

fn decided_subcurve_bounds(
    curve: &BezierSubcurve2,
    family: CurveFamily2,
) -> ExactCurveResult<Aabb2> {
    let bounds = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.control_hull_box(),
        BezierSubcurve2::Cubic(curve) => curve.control_hull_box(),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(),
    };
    decided_bounds(bounds, family)
}

fn decided_bounds(bounds: Classification<Aabb2>, family: CurveFamily2) -> ExactCurveResult<Aabb2> {
    match bounds {
        Classification::Decided(bounds) => Ok(bounds),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::NativeTopology,
            family,
            reason,
        )),
    }
}

fn select_native_fragments(
    fragments: &[NativeBezierFragment2],
    parameter: &Real,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<(usize, usize)> {
    let mut first = None;
    let mut last = None;
    for (index, fragment) in fragments.iter().enumerate() {
        let (start, end) = fragment.parameter_range();
        match (
            crate::classify::compare_reals(start, parameter, policy),
            crate::classify::compare_reals(parameter, end, policy),
        ) {
            (
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            ) => {
                first.get_or_insert(index);
                last = Some(index);
            }
            (Some(_), Some(_)) => {}
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    family,
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
    }
    first.zip(last).ok_or_else(|| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            family,
            CurveError::InvalidCurveParameter,
        )
    })
}

fn certify_matching_derivatives(
    first: Vec<CurveDerivative2>,
    second: Vec<CurveDerivative2>,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<CurveDerivative2>> {
    debug_assert_eq!(first.len(), second.len());
    for (first_derivative, second_derivative) in first.iter().zip(&second) {
        match (
            crate::classify::compare_reals(first_derivative.dx(), second_derivative.dx(), policy),
            crate::classify::compare_reals(first_derivative.dy(), second_derivative.dy(), policy),
        ) {
            (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => {}
            (Some(_), Some(_)) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    family,
                    crate::UncertaintyReason::Boundary,
                ));
            }
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    family,
                    crate::UncertaintyReason::RealSign,
                ));
            }
        }
    }
    Ok(first)
}

fn remap_operation(error: ExactCurveError, operation: CurveOperation2) -> ExactCurveError {
    error.with_operation(operation)
}

fn geometry_parameter_range(geometry: &CurveGeometry2) -> ParamRange {
    let (start, end) = match geometry {
        CurveGeometry2::PolynomialBSpline(curve) => curve.parameter_domain(),
        CurveGeometry2::Nurbs(curve) => curve.parameter_domain(),
        _ => return ParamRange::new(Real::zero(), Real::one()),
    };
    ParamRange::new(start.clone(), end.clone())
}

fn rationalize_subcurve(
    curve: &BezierSubcurve2,
    family: CurveFamily2,
) -> ExactCurveResult<RationalBezier2> {
    RationalBezier2::try_from_subcurve(curve)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::NativeTopology, family, cause))
}

fn promote_native_bezier_fragments(
    curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<Vec<NativeBezierFragment2>>> {
    let native = |native_curve, parameter_start: Real, parameter_end: Real| {
        Ok(NativeBezierFragment2 {
            lineage: curve
                .lineage_subrange(&parameter_start, &parameter_end)
                .map_err(|error| error.with_operation(CurveOperation2::NativeTopology))?,
            curve: native_curve,
            span_range: CurveSpanRange2 {
                start: parameter_start,
                end: parameter_end,
            },
        })
    };
    let unit = || (Real::zero(), Real::one());
    match curve.geometry() {
        None => Ok(Classification::Uncertain(
            crate::UncertaintyReason::Unsupported,
        )),
        Some(CurveGeometry2::Line(line)) => {
            let (start, end) = unit();
            Ok(Classification::Decided(vec![native(
                BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(line.clone())),
                start,
                end,
            )?]))
        }
        Some(CurveGeometry2::CircularArc(value)) => {
            let decomposition = match decompose_circular_arc(value, policy)? {
                Classification::Decided(decomposition) => decomposition,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(Classification::Decided(
                decomposition
                    .spans()
                    .iter()
                    .map(|span| {
                        let (start, end) = span.parameter_range();
                        native(
                            BezierSubcurve2::RationalQuadratic(span.curve().clone()),
                            start.clone(),
                            end.clone(),
                        )
                    })
                    .collect::<ExactCurveResult<_>>()?,
            ))
        }
        Some(CurveGeometry2::QuadraticBezier(value)) => {
            let (start, end) = unit();
            Ok(Classification::Decided(vec![native(
                BezierSubcurve2::Quadratic(value.clone()),
                start,
                end,
            )?]))
        }
        Some(CurveGeometry2::CubicBezier(value)) => {
            let (start, end) = unit();
            Ok(Classification::Decided(vec![native(
                BezierSubcurve2::Cubic(value.clone()),
                start,
                end,
            )?]))
        }
        Some(CurveGeometry2::RationalQuadraticBezier(value)) => {
            let (start, end) = unit();
            Ok(Classification::Decided(vec![native(
                BezierSubcurve2::RationalQuadratic(value.clone()),
                start,
                end,
            )?]))
        }
        Some(CurveGeometry2::RationalBezier(value)) => {
            let (start, end) = unit();
            Ok(Classification::Decided(vec![native(
                BezierSubcurve2::Rational(value.clone()),
                start,
                end,
            )?]))
        }
        Some(CurveGeometry2::PolynomialBSpline(value)) => {
            let decomposition = match value.bezier_decomposition_with_policy(policy)? {
                Classification::Decided(decomposition) => decomposition,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(Classification::Decided(
                decomposition
                    .native_spans()
                    .iter()
                    .zip(decomposition.intervals())
                    .map(|(curve, (start, end))| native(curve.clone(), start.clone(), end.clone()))
                    .collect::<ExactCurveResult<_>>()?,
            ))
        }
        Some(CurveGeometry2::Nurbs(value)) => {
            let decomposition = match value.bezier_decomposition_with_policy(policy)? {
                Classification::Decided(decomposition) => decomposition,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let subcurves = match value.native_subcurves_raw(policy)? {
                Classification::Decided(subcurves) => subcurves,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            debug_assert_eq!(decomposition.spans().len(), subcurves.len());
            Ok(Classification::Decided(
                decomposition
                    .spans()
                    .iter()
                    .zip(subcurves)
                    .map(|(span, curve)| {
                        let (start, end) = span.knot_interval();
                        native(curve.clone(), start.clone(), end.clone())
                    })
                    .collect::<ExactCurveResult<_>>()?,
            ))
        }
    }
}

fn evaluate_promoted_arc(
    fragments: &[NativeBezierFragment2],
    parameter: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<Point2> {
    for fragment in fragments {
        let (start, end) = fragment.parameter_range();
        let lower = crate::classify::compare_reals(start, parameter, policy);
        let upper = crate::classify::compare_reals(parameter, end, policy);
        match (lower, upper) {
            (
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            ) => {
                let local = ((parameter - start) / (end - start)).map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Evaluation,
                        CurveFamily2::CircularArc,
                        cause.into(),
                    )
                })?;
                let BezierSubcurve2::RationalQuadratic(curve) = fragment.native_curve() else {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Evaluation,
                        CurveFamily2::CircularArc,
                        CurveError::Topology(
                            "circular arc promoted to a non-rational-quadratic span".into(),
                        ),
                    ));
                };
                return match curve.point_at(local, policy) {
                    Classification::Decided(point) => Ok(point),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Evaluation,
                        CurveFamily2::CircularArc,
                        reason,
                    )),
                };
            }
            (Some(_), Some(_)) => {}
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::CircularArc,
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
    }
    Err(ExactCurveError::invalid(
        CurveOperation2::Evaluation,
        CurveFamily2::CircularArc,
        CurveError::InvalidCurveParameter,
    ))
}

fn validate_unit_parameter(
    parameter: &Real,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<crate::classify::ClosedUnitIntervalLocation> {
    use crate::classify::ClosedUnitIntervalLocation;

    match crate::classify::closed_unit_interval_location(parameter, policy) {
        Some(ClosedUnitIntervalLocation::Outside) => Err(ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            family,
            CurveError::InvalidCurveParameter,
        )),
        Some(location) => Ok(location),
        None => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            family,
            crate::UncertaintyReason::Ordering,
        )),
    }
}

fn retained_native_endpoint(
    geometry: &CurveGeometry2,
    location: crate::classify::ClosedUnitIntervalLocation,
    policy: &CurveContext,
) -> Option<Point2> {
    use crate::classify::ClosedUnitIntervalLocation;

    let (endpoint, weight) = match (geometry, location) {
        (_, ClosedUnitIntervalLocation::Outside | ClosedUnitIntervalLocation::Interior) => {
            return None;
        }
        (CurveGeometry2::RationalQuadraticBezier(curve), ClosedUnitIntervalLocation::Start) => {
            (curve.start(), Some(curve.start_weight()))
        }
        (CurveGeometry2::RationalQuadraticBezier(curve), ClosedUnitIntervalLocation::End) => {
            (curve.end(), Some(curve.end_weight()))
        }
        (CurveGeometry2::RationalBezier(curve), ClosedUnitIntervalLocation::Start) => {
            (curve.start(), curve.weights().first())
        }
        (CurveGeometry2::RationalBezier(curve), ClosedUnitIntervalLocation::End) => {
            (curve.end(), curve.weights().last())
        }
        (geometry, ClosedUnitIntervalLocation::Start) => (geometry.start(), None),
        (geometry, ClosedUnitIntervalLocation::End) => (geometry.end(), None),
    };
    if weight.is_none_or(|weight| crate::classify::is_zero(weight, policy) == Some(false)) {
        Some(endpoint.clone())
    } else {
        None
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CornerPlacement2 {
    Trim,
    Corner,
    Extension,
}

fn exact_corner_parameter(parameter: Real) -> Option<CurveParameter2> {
    Some(CurveParameter2::from(BezierParameter2::Exact(parameter)))
}

#[derive(Clone, Debug)]
pub(crate) struct CornerCut2 {
    /// Canonical carrier-local parameter when the consuming representation
    /// needs it. Native fillet arcs may defer their sweep parameter because
    /// exact Cartesian incidence is sufficient for reconstruction.
    parameter: Option<CurveParameter2>,
    point: CurvePoint2,
    placement: CornerPlacement2,
}

impl CornerCut2 {
    fn map_source_parameter(
        &mut self,
        chart: Option<(&Real, &Real)>,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<()> {
        if let (Some((scale, offset)), Some(parameter)) = (chart, &self.parameter) {
            self.parameter = Some(
                match parameter
                    .affine_image_unbounded(scale, offset, policy)
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
                {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(operation, family, reason));
                    }
                },
            );
        }
        Ok(())
    }

    fn exact_point(&self) -> Option<&Point2> {
        self.point.coordinates()
    }

    pub(crate) fn into_retained_evidence(self) -> Option<CornerTrimCut2> {
        let parameter = self.parameter?;
        Some(CornerTrimCut2 {
            parameter,
            point: self.point,
            placement: self.placement,
            replacement: None,
        })
    }

    fn exact_parameter(&self) -> Option<&Real> {
        self.parameter.as_ref()?.scalar()
    }
}

#[derive(Clone, Debug)]
pub(crate) struct CornerTrimCut2 {
    pub(crate) parameter: CurveParameter2,
    pub(crate) point: CurvePoint2,
    pub(crate) placement: CornerPlacement2,
    /// The exact replacement span, in the same chart as this cut.
    pub(crate) replacement: Option<Arc<crate::BezierSplitFragment2>>,
}

impl CornerTrimCut2 {
    pub(crate) fn replacement_curve(&self) -> Option<&BezierSubcurve2> {
        match self.replacement.as_deref()? {
            crate::BezierSplitFragment2::Materialized { curve, .. } => Some(curve),
            _ => None,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct ChamferCorner2 {
    previous: CornerCut2,
    next: CornerCut2,
}

impl ChamferCorner2 {
    pub(crate) fn into_retained_cut_evidence(self) -> Option<(CornerTrimCut2, CornerTrimCut2)> {
        Some((
            self.previous.into_retained_evidence()?,
            self.next.into_retained_evidence()?,
        ))
    }
}

#[derive(Clone, Debug)]
pub(crate) struct FilletCorner2 {
    previous: CornerCut2,
    next: CornerCut2,
    center: CurvePoint2,
    clockwise: bool,
    retained_frame: Option<RetainedFilletFrame2>,
}

#[derive(Clone, Debug)]
pub(crate) struct RetainedFilletFrame2 {
    pub(crate) anchor_is_previous: bool,
    pub(crate) radial_frame: RetainedFilletRadialFrame2,
    pub(crate) radial_distance: Real,
    pub(crate) anchor_evidence: Option<RetainedFilletAnchorEvidence2>,
}

#[derive(Clone, Debug)]
pub(crate) enum RetainedFilletRadialFrame2 {
    RepresentedUnitNormal((Real, Real)),
    /// The fillet center is arbitrary retained evidence and the local radial
    /// is one algebraic chord's exact unit left normal. This is the general
    /// projective line/line frame when neither direction is a represented
    /// `Real` vector.
    ChordNormal {
        anchor: crate::BezierAlgebraicChord2,
        policy: CurveContext,
    },
    ConcentricArc {
        support_center: Point2,
        normal_denominator: Real,
    },
    /// The fillet center is a retained contact on `support`; its start radial
    /// direction is the selected support radius divided by
    /// `normal_denominator`. This keeps an independent two-field circle-pair
    /// contact compact instead of adjoining or flattening its coordinates.
    SelectedConcentric {
        support: crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
        center_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
        normal_denominator: Real,
    },
    /// The center lies on `center_support` at `center_parameter`; its source
    /// unit left normal is the fillet's start radial direction. This retains a
    /// general non-PH frame without adjoining the selected speed square root.
    ParallelNormal {
        center_support: BezierParallel2,
        center_parameter: CurveParameter2,
        policy: CurveContext,
    },
}

#[derive(Clone, Debug)]
pub(crate) struct RetainedFilletCenterParallel2 {
    pub(crate) support: BezierParallel2,
    /// The exact construction parameter on the analytic center support.
    /// Selected fibers remain local here; reconstruction must never require
    /// their degree-multiplied global projection merely to recover a tangent
    /// frame that the center solve already certified.
    pub(crate) parameter: Option<CurveParameter2>,
}

#[derive(Clone, Debug)]
pub(crate) struct RetainedFilletAnchorEvidence2 {
    pub(crate) cross: Option<RealSign>,
    pub(crate) dot: Option<RealSign>,
    /// Optional analytic center frame supplied by a represented carrier that
    /// deliberately entered the shared analytic intersection authority.
    pub(crate) center_parallel: Option<RetainedFilletCenterParallel2>,
    /// Traversal orientation of a selected analytic anchor relative to its
    /// homogeneous source tangent.
    pub(crate) source_direction: Option<RealSign>,
    pub(crate) canonical_anchor_curve: Option<RationalBezier2>,
    /// A direct arc/Bezier center whose circular cut must be recovered in the
    /// selected center fiber after the retained fillet circle is built.
    pub(crate) deferred_arc_contact: Option<RetainedDeferredArcFilletContact2>,
}

#[derive(Clone, Debug)]
pub(crate) struct RetainedDeferredArcFilletContact2 {
    pub(crate) source: ExactCornerArc2,
    pub(crate) source_radius: Real,
    pub(crate) signed_center_radius: Real,
    pub(crate) arc_is_previous: bool,
    /// Owns extension permission and whether the cut already names an
    /// authored source chart rather than a deferred canonical circle cell.
    pub(crate) domain: FilletContactDomain2,
    /// The exact full-circle center parameter selected by the common
    /// circle-pair authority. When present, retained fillet construction
    /// reuses this pair field as its radial frame instead of reconstructing
    /// the same center through a second circular-tangent solve.
    pub(crate) selected_center:
        Option<crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2>,
    /// Exact source-circle chart parameter inherited from the affine radial
    /// offset map. `None` retains the older deferred-search path used when the
    /// center solver did not traverse the circular carrier itself.
    pub(crate) contact_seed: Option<RetainedArcFilletContactSeed2>,
}

#[derive(Clone, Debug)]
pub(crate) struct RetainedArcFilletContactSeed2 {
    pub(crate) cell: RetainedArcFilletContactCell2,
    pub(crate) parameter: CurveParameter2,
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum RetainedArcFilletContactCell2 {
    /// One canonical projective cell of the authored circular sweep.
    Authored(usize),
    /// One canonical projective cell of the complementary sweep.
    Complement(usize),
}

impl FilletCorner2 {
    pub(crate) fn into_retained_cut_evidence(
        self,
    ) -> Option<(
        CornerTrimCut2,
        CornerTrimCut2,
        CurvePoint2,
        bool,
        Option<RetainedFilletFrame2>,
    )> {
        Some((
            self.previous.into_retained_evidence()?,
            self.next.into_retained_evidence()?,
            self.center,
            self.clockwise,
            self.retained_frame,
        ))
    }
}

enum CornerSolutionAccumulator<T> {
    Empty,
    One(T),
    Multiple(Vec<T>),
}

impl<T> CornerSolutionAccumulator<T> {
    fn push(&mut self, candidate: T) {
        *self = match std::mem::replace(self, Self::Empty) {
            Self::Empty => Self::One(candidate),
            Self::One(first) => Self::Multiple(vec![first, candidate]),
            Self::Multiple(mut candidates) => {
                candidates.push(candidate);
                Self::Multiple(candidates)
            }
        };
    }

    fn finish(self, empty_reason: CurveCornerNoSolution2) -> CurveCornerSolutions2<T> {
        match self {
            Self::Empty => CurveCornerSolutions2::NoSolution(empty_reason),
            Self::One(candidate) => CurveCornerSolutions2::Unique(candidate),
            Self::Multiple(candidates) => CurveCornerSolutions2::Multiple(candidates),
        }
    }

    fn append(&mut self, solutions: CurveCornerSolutions2<T>) -> Option<CurveCornerNoSolution2> {
        match solutions {
            CurveCornerSolutions2::NoSolution(reason) => return Some(reason),
            CurveCornerSolutions2::Unique(candidate) => self.push(candidate),
            CurveCornerSolutions2::Multiple(candidates) => {
                for candidate in candidates {
                    self.push(candidate);
                }
            }
        }
        None
    }
}

#[derive(Default)]
struct CornerCuts2 {
    first: Option<CornerCut2>,
    second: Option<CornerCut2>,
    overflow: Vec<CornerCut2>,
}

impl CornerCuts2 {
    fn push(&mut self, cut: CornerCut2) {
        if self.first.is_none() {
            self.first = Some(cut);
        } else if self.second.is_none() {
            self.second = Some(cut);
        } else {
            self.overflow.push(cut);
        }
    }

    fn iter(&self) -> impl Iterator<Item = &CornerCut2> {
        self.first
            .iter()
            .chain(self.second.iter())
            .chain(self.overflow.iter())
    }

    fn iter_mut(&mut self) -> impl Iterator<Item = &mut CornerCut2> {
        self.first
            .iter_mut()
            .chain(self.second.iter_mut())
            .chain(self.overflow.iter_mut())
    }

    fn is_empty(&self) -> bool {
        self.first.is_none() && self.second.is_none() && self.overflow.is_empty()
    }
}

fn exact_linear_corner_line(curve: &Curve2) -> Option<&LineSeg2> {
    match curve.geometry() {
        None => None,
        Some(CurveGeometry2::Line(line)) => Some(line),
        Some(CurveGeometry2::QuadraticBezier(curve)) => curve.retained_exact_line_image(),
        Some(CurveGeometry2::CircularArc(_))
        | Some(CurveGeometry2::CubicBezier(_))
        | Some(CurveGeometry2::RationalQuadraticBezier(_))
        | Some(CurveGeometry2::RationalBezier(_))
        | Some(CurveGeometry2::PolynomialBSpline(_))
        | Some(CurveGeometry2::Nurbs(_)) => None,
    }
}

pub(crate) enum ExactCornerCarrier2<'a> {
    Line(&'a LineSeg2),
    PromotedLine(&'a QuadraticBezier2),
    Arc(&'a CircularArc2),
    RetainedRationalArc(Arc<RetainedRationalCornerArc2>),
    Bezier(&'a Curve2),
    NativeBezierSpan(&'a NativeBezierFragment2),
    AlgebraicChord(&'a crate::BezierAlgebraicChord2),
    AnalyticParallel(&'a crate::BezierParallelFragment2),
    SelectedFiber(&'a crate::bezier_split::BezierSelectedFiberFragment2),
    AlgebraicCusp(&'a crate::BezierAlgebraicCuspSemicircleFragment2),
}

#[derive(Clone, Copy, Debug)]
enum ExactCornerBezier2<'a> {
    Direct(&'a Curve2),
    NativeSpan(&'a NativeBezierFragment2),
}

#[derive(Clone, Debug)]
pub(crate) enum ExactCornerArc2 {
    Native(CircularArc2),
    RetainedRational(Arc<RetainedRationalCornerArc2>),
}

/// A certified circular parent together with its actual exact interval.
/// The same range owner represents ordinary, spline, and selected charts;
/// restricting a circle never changes its full circular continuation.
#[derive(Clone, Debug)]
pub(crate) struct RetainedRationalCornerArc2 {
    pub(crate) fragment: crate::bezier_split::BezierSelectedFiberFragment2,
    support: CircularArc2,
    parameter_map: (Real, Real),
}

impl RetainedRationalCornerArc2 {
    fn prepare_evaluator(
        evaluator: RationalBezier2,
        support: &CircularArc2,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<RationalBezier2> {
        let evaluator = if evaluator.retained_circular_conic().is_some()
            || matches!(
                evaluator.control_weight_sign(),
                Classification::Decided(RealSign::Positive | RealSign::Negative)
            ) {
            evaluator
        } else {
            let (implicit_conic, circular_conic) = circular_conic_provenance(support);
            evaluator.with_implicit_quadratic_conic(implicit_conic, Some(circular_conic))
        };
        // Collapse degree elevation once, before either contact enumeration
        // or publication asks for the inverse of this parameterized circle.
        match evaluator
            .materialized_quadratic_representative(policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(Some(quadratic)) => Ok(quadratic.into()),
            Classification::Decided(None) => Ok(evaluator),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, family, reason))
            }
        }
    }

    fn from_source(
        source: ExactCornerBezier2<'_>,
        support: CircularArc2,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let evaluator = match source {
            ExactCornerBezier2::Direct(source) => {
                let [evaluator] = source.rational_evaluators_for_operation(policy, operation)?
                else {
                    return Err(ExactCurveError::invalid(
                        operation,
                        family,
                        CurveError::Topology(
                            "a circular chart must have one rational evaluator".into(),
                        ),
                    ));
                };
                evaluator.clone()
            }
            ExactCornerBezier2::NativeSpan(fragment) => {
                RationalBezier2::try_from_subcurve(fragment.native_curve())
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
            }
        };
        let evaluator = Self::prepare_evaluator(evaluator, &support, operation, family, policy)?;
        let fragment = crate::bezier_split::BezierSelectedFiberFragment2::new(
            crate::bezier_split::BezierSelectedFiberSource2::Rational(evaluator),
            CurveParameterRange2::new_validated(Real::zero().into(), Real::one().into()),
            support.start().clone().into(),
            support.end().clone().into(),
        );
        let (start, end) = source.parameter_range();
        Ok(Self {
            fragment,
            support,
            parameter_map: (end - start, start.clone()),
        })
    }

    pub(crate) fn from_fragment(
        fragment: &crate::BezierSplitFragment2,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Arc<Self>>> {
        match fragment {
            crate::BezierSplitFragment2::SelectedFiber(fragment) => {
                Self::from_selected(fragment, operation, policy)
            }
            crate::BezierSplitFragment2::AnalyticParallel(fragment) => {
                let family = CurveFamily2::AnalyticParallel;
                let curve = match policy
                    .strict_predicate_pass(|| {
                        fragment
                            .parallel()
                            .exact_circular_parallel_component(policy)
                    })
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
                {
                    Classification::Decided(Some(curve)) => curve,
                    Classification::Decided(None) | Classification::Uncertain(_) => {
                        return Ok(None);
                    }
                };
                let range = CurveParameterRange2::from_bezier_range(fragment.range().clone());
                let point = |parameter: &BezierParameter2| match fragment
                    .parallel()
                    .point_evidence_on_regular_range(&parameter.clone().into(), &range, policy)
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
                {
                    Classification::Decided(point) => Ok(point),
                    Classification::Uncertain(reason) => {
                        Err(ExactCurveError::blocked(operation, family, reason))
                    }
                };
                let start = point(fragment.range().start())?;
                let end = point(fragment.range().end())?;
                let selected = crate::bezier_split::BezierSelectedFiberFragment2::new(
                    crate::bezier_split::BezierSelectedFiberSource2::Rational(curve),
                    range,
                    start,
                    end,
                );
                Self::from_selected(
                    &if fragment.is_reversed() {
                        selected.reversed()
                    } else {
                        selected
                    },
                    operation,
                    policy,
                )
            }
            _ => Ok(None),
        }
    }

    fn from_selected(
        fragment: &crate::bezier_split::BezierSelectedFiberFragment2,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Arc<Self>>> {
        let source = match fragment.source() {
            crate::bezier_split::BezierSelectedFiberSource2::Rational(source) => {
                std::borrow::Cow::Borrowed(source)
            }
            crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
                match policy
                    .strict_predicate_pass(|| parallel.exact_circular_parallel_component(policy))
                    .map_err(|cause| {
                        ExactCurveError::invalid(operation, CurveFamily2::AnalyticParallel, cause)
                    })? {
                    Classification::Decided(Some(source)) => std::borrow::Cow::Owned(source),
                    Classification::Decided(None) | Classification::Uncertain(_) => {
                        return Ok(None);
                    }
                }
            }
        };
        let family = CurveFamily2::RationalBezier;
        let mut support = match rational_bezier_circular_arc(&source, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(Some(support)) => support,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        let mut contained = true;
        for (parameter, boundary, outside) in [
            (
                fragment.range().start(),
                Real::zero(),
                std::cmp::Ordering::Less,
            ),
            (
                fragment.range().end(),
                Real::one(),
                std::cmp::Ordering::Greater,
            ),
        ] {
            match policy
                .strict_predicate_pass(|| parameter.cmp_by_refinement(&boundary.into(), policy))
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
            {
                Classification::Decided(order) if order != outside => {}
                Classification::Decided(_) => {
                    contained = false;
                    break;
                }
                Classification::Uncertain(_) => return Ok(None),
            }
        }
        let mut evaluator =
            Self::prepare_evaluator(source.into_owned(), &support, operation, family, policy)?;
        let mut range = fragment.range().clone();
        let mut parameter_map = (Real::one(), Real::zero());
        if !contained {
            // A finite outer interval schedules one pole-free parent chart;
            // its exact selected endpoints still decide every cut. Mapping
            // the result back preserves the authored source parameter even
            // when the surviving range lies beyond either original endpoint.
            let (lower, upper) = match crate::bezier_split::CurveParameterDomain2::new(&range, None)
                .finite_envelope(policy)
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
            {
                Classification::Decided((_, [lower, upper])) => (lower.clone(), upper.clone()),
                Classification::Uncertain(_) => return Ok(None),
            };
            let scale = &upper - &lower;
            evaluator = match policy
                .strict_predicate_pass(|| {
                    evaluator.subcurve_between_affine_exact(&lower, &upper, policy)
                })
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
            {
                Classification::Decided(evaluator) => evaluator,
                Classification::Uncertain(_) => return Ok(None),
            };
            support = match rational_bezier_circular_arc(&evaluator, policy)
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
            {
                Classification::Decided(Some(support)) => support,
                Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
            };
            evaluator = Self::prepare_evaluator(evaluator, &support, operation, family, policy)?;
            let inverse_scale = (Real::one() / &scale)
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))?;
            let inverse_offset = -(&lower * &inverse_scale);
            let map = |parameter: &CurveParameter2| match parameter
                .affine_image_unbounded(&inverse_scale, &inverse_offset, policy)
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
            {
                Classification::Decided(parameter) => Ok(parameter),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            };
            range = CurveParameterRange2::new_validated(map(range.start())?, map(range.end())?);
            parameter_map = (scale, lower);
        }
        let mut retained = crate::bezier_split::BezierSelectedFiberFragment2::new(
            crate::bezier_split::BezierSelectedFiberSource2::Rational(evaluator),
            range,
            if fragment.is_reversed() {
                fragment.end_point()
            } else {
                fragment.start_point()
            }
            .clone(),
            if fragment.is_reversed() {
                fragment.start_point()
            } else {
                fragment.end_point()
            }
            .clone(),
        );
        let support = if fragment.is_reversed() {
            retained = retained.reversed();
            support.reversed()
        } else {
            support
        };
        Ok(Some(Arc::new(Self {
            fragment: retained,
            support,
            parameter_map,
        })))
    }

    pub(crate) fn support(&self) -> &CircularArc2 {
        &self.support
    }

    fn corner_parameter(&self, previous: bool) -> &CurveParameter2 {
        if previous != self.fragment.is_reversed() {
            self.fragment.range().end()
        } else {
            self.fragment.range().start()
        }
    }

    fn curve_parameter(
        &self,
        parameter: &CurveParameter2,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveParameter2> {
        let (scale, offset) = &self.parameter_map;
        if scale == &Real::one() && offset == &Real::zero() {
            return Ok(parameter.clone());
        }
        if let Some(parameter) = parameter.scalar() {
            return Ok((offset + scale * parameter).into());
        }
        match parameter
            .affine_image_unbounded(scale, offset, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(parameter) => Ok(parameter),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, family, reason))
            }
        }
    }

    pub(crate) fn parameter_at_incident_point(
        source: &RationalBezier2,
        point: &CurvePoint2,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CurveParameter2>> {
        let result = if let Some(point) = point.coordinates() {
            match source
                .retained_circle_point_parameters(point, policy)
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
            {
                Classification::Decided(parameters) => {
                    let parameter = match parameters.as_slice() {
                        [] => None,
                        [parameter] => Some(parameter.clone().into()),
                        _ => {
                            return Err(ExactCurveError::blocked(
                                operation,
                                family,
                                crate::UncertaintyReason::Boundary,
                            ));
                        }
                    };
                    Classification::Decided(parameter)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        } else {
            crate::bezier_offset::quadratic_conic_parameter_at_incident_point(point, source, policy)
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        };
        match result {
            Classification::Decided(parameter) => Ok(parameter),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, family, reason))
            }
        }
    }

    fn cut_at_incident_point(
        &self,
        point: CurvePoint2,
        previous: bool,
        mode: CurveCornerMode2,
        include_corner: bool,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CornerCut2>> {
        let source = self
            .fragment
            .rational_curve()
            .expect("a rational circular chart");
        if let Some(parameter) =
            Self::parameter_at_incident_point(source, &point, operation, family, policy)?
        {
            let compare = |boundary: &CurveParameter2| match parameter
                .cmp_by_refinement(boundary, policy)
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
            {
                Classification::Decided(order) => Ok(order),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            };
            let start_order = compare(self.fragment.range().start())?;
            let end_order = compare(self.fragment.range().end())?;
            if start_order.is_eq() || end_order.is_eq() {
                if !include_corner || !compare(self.corner_parameter(previous))?.is_eq() {
                    return Ok(None);
                }
            } else if !start_order.is_gt() || !end_order.is_lt() {
                if mode != CurveCornerMode2::TrimOrExtend {
                    return Ok(None);
                }
                return Ok(Some(CornerCut2 {
                    point,
                    parameter: Some(self.curve_parameter(&parameter, operation, family, policy)?),
                    placement: CornerPlacement2::Extension,
                }));
            }
            return Ok(Some(CornerCut2 {
                point,
                parameter: Some(self.curve_parameter(&parameter, operation, family, policy)?),
                placement: CornerPlacement2::Trim,
            }));
        }
        if mode != CurveCornerMode2::TrimOrExtend {
            return Ok(None);
        }
        Ok(Some(CornerCut2 {
            point,
            parameter: Some(self.curve_parameter(
                self.corner_parameter(previous),
                operation,
                family,
                policy,
            )?),
            placement: CornerPlacement2::Extension,
        }))
    }
}

/// Decomposes the complementary sweep of one certified circular support into
/// exact projective cells. Quarter turns avoid the midpoint radical required
/// by a generic major-arc construction and give both center solving and
/// retained publication the same full-circle domain authority.
pub(crate) fn retained_arc_complement_projective_spans(
    support: &CircularArc2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<RationalQuadraticBezier2>> {
    use crate::segment::ArcSweepPointLocation2;

    let mut current = support.end().clone();
    let target = support.start();
    let mut spans = Vec::with_capacity(4);
    for _ in 0..4 {
        if &current == target {
            return Ok(spans);
        }
        let radial = current.delta_from(support.center());
        let next_radial = if support.is_clockwise() {
            (radial.1, -radial.0)
        } else {
            (-radial.1, radial.0)
        };
        let next = support.center().translated(next_radial.0, next_radial.1);
        let quarter = CircularArc2::new_with_certified_radius(
            current.clone(),
            next.clone(),
            support.center().clone(),
            support.radius_squared(),
            support.is_clockwise(),
            None,
        );
        let reaches_target = match quarter.strict_sweep_point_location(target, policy) {
            Classification::Decided(
                ArcSweepPointLocation2::Interior | ArcSweepPointLocation2::Endpoint,
            ) => true,
            Classification::Decided(ArcSweepPointLocation2::Outside) => false,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, family, reason));
            }
        };
        let cell = if reaches_target {
            CircularArc2::new_with_certified_radius(
                current,
                target.clone(),
                support.center().clone(),
                support.radius_squared(),
                support.is_clockwise(),
                None,
            )
        } else {
            quarter
        };
        let decomposition = match decompose_circular_arc(&cell, policy)
            .map_err(|error| error.with_operation(operation))?
        {
            Classification::Decided(decomposition) => decomposition,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, family, reason));
            }
        };
        spans.extend(
            decomposition
                .spans()
                .iter()
                .map(|span| span.curve().clone()),
        );
        if reaches_target {
            return Ok(spans);
        }
        current = next;
    }
    Err(ExactCurveError::invalid(
        operation,
        family,
        CurveError::Topology(
            "an exact circular complement did not close within one revolution".into(),
        ),
    ))
}

/// Builds the one authoritative projective-cell cover used by both arc
/// center solving and retained fillet publication. Shared authored-cell
/// boundaries belong to the earlier cell; complementary endpoints are
/// excluded because the authored sweep already owns them.
fn retained_arc_fillet_projective_cells(
    support: &CircularArc2,
    mode: CurveCornerMode2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<(RationalBezier2, bool, bool, RetainedArcFilletContactCell2)>> {
    let authored = match support
        .rational_bezier_decomposition_with_policy(policy)
        .map_err(|error| error.with_operation(CurveOperation2::Fillet))?
    {
        Classification::Decided(decomposition) => decomposition,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            ));
        }
    };
    let complement = if mode == CurveCornerMode2::TrimOrExtend {
        retained_arc_complement_projective_spans(support, CurveOperation2::Fillet, family, policy)?
    } else {
        Vec::new()
    };
    let complement_count = complement.len();
    let mut cells = Vec::with_capacity(authored.spans().len() + complement_count);
    cells.extend(authored.spans().iter().enumerate().map(|(index, span)| {
        (
            RationalBezier2::from(span.curve().clone()),
            index == 0,
            true,
            RetainedArcFilletContactCell2::Authored(index),
        )
    }));
    cells.extend(complement.into_iter().enumerate().map(|(index, span)| {
        (
            RationalBezier2::from(span),
            false,
            index + 1 != complement_count,
            RetainedArcFilletContactCell2::Complement(index),
        )
    }));
    Ok(cells)
}

/// Transports one already-selected center contact radially onto the authored
/// arc circle and recovers its exact projective cell parameter. This is an
/// inverse parameter map, not a new tangent solve: the simple line/circle root
/// remains the sole construction root and the conic inverse preserves it in
/// the retained recursive quadratic field.
#[allow(clippy::too_many_arguments)]
fn retained_arc_fillet_contact_seed(
    support: &CircularArc2,
    offset_half: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    offset_parameter: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    source_radius: &Real,
    signed_center_radius: &Real,
    mode: CurveCornerMode2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<RetainedArcFilletContactSeed2>> {
    let radial_scale = (source_radius / signed_center_radius)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause.into()))?;
    let target_circle = match offset_half
        .scaled_radial_distance(&radial_scale, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(Some(circle)) => circle,
        Classification::Decided(None) => {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Fillet,
                family,
                CurveError::Topology(
                    "a nonzero concentric conic transport collapsed its circle".into(),
                ),
            ));
        }
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            ));
        }
    };
    let point = match offset_parameter
        .concentric_offset_point_evidence(offset_half, &target_circle, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(Some(point)) => point,
        Classification::Decided(None) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                crate::UncertaintyReason::Unsupported,
            ));
        }
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            ));
        }
    };
    let cells = retained_arc_fillet_projective_cells(support, mode, family, policy)?;
    let mut retained = None;
    let mut unresolved = None;
    for (curve, include_start, include_end, cell) in cells {
        let parameter = match crate::bezier_offset::quadratic_conic_parameter_at_incident_point(
            &point, &curve, policy,
        )
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(Some(parameter)) => parameter,
            Classification::Decided(None) => continue,
            Classification::Uncertain(reason) => {
                unresolved.get_or_insert(reason);
                continue;
            }
        };
        let boundary_order = |boundary: Real| {
            parameter
                .cmp_by_refinement(
                    &CurveParameter2::from(BezierParameter2::Exact(boundary)),
                    policy,
                )
                .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))
                .and_then(|order| match order {
                    Classification::Decided(order) => Ok(order),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        reason,
                    )),
                })
        };
        if (!include_start && boundary_order(Real::zero())?.is_eq())
            || (!include_end && boundary_order(Real::one())?.is_eq())
        {
            continue;
        }
        if retained.is_some() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Fillet,
                family,
                CurveError::Topology(
                    "one selected center contact mapped to multiple arc projective cells".into(),
                ),
            ));
        }
        retained = Some(RetainedArcFilletContactSeed2 { cell, parameter });
    }
    match (retained, unresolved) {
        (Some(retained), _) => Ok(Some(retained)),
        (None, Some(reason)) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
        (None, None) => Ok(None),
    }
}

impl ExactCornerArc2 {
    pub(crate) fn support(&self) -> &CircularArc2 {
        match self {
            Self::Native(arc) => arc,
            Self::RetainedRational(retained) => &retained.support,
        }
    }

    pub(crate) fn retained_rational_arc(&self) -> Option<&Arc<RetainedRationalCornerArc2>> {
        match self {
            Self::Native(_) => None,
            Self::RetainedRational(arc) => Some(arc),
        }
    }

    fn corner_parameter(
        &self,
        previous: bool,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveParameter2> {
        match self {
            Self::Native(_) => Ok(if previous { Real::one() } else { Real::zero() }.into()),
            Self::RetainedRational(arc) => {
                arc.curve_parameter(arc.corner_parameter(previous), operation, family, policy)
            }
        }
    }

    /// Returns the exact concentric support image of this certified arc.
    ///
    /// The authoritative circle kernels need only the transformed support;
    /// mapping a rational evaluator's controls and weights would allocate a
    /// duplicate carrier that is immediately decomposed back into canonical
    /// projective circle cells.
    fn concentric_offset_support(
        &self,
        source_radius: &Real,
        signed_radius: &Real,
        operation: CurveOperation2,
        family: CurveFamily2,
    ) -> ExactCurveResult<CircularArc2> {
        let scale = (signed_radius / source_radius)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))?;
        let center = self.support().center();
        let transform = |point: &Point2| {
            let radial = point.delta_from(center);
            center.translated(&radial.0 * &scale, &radial.1 * &scale)
        };
        Ok(CircularArc2::new_with_certified_radius(
            transform(self.support().start()),
            transform(self.support().end()),
            center.clone(),
            signed_radius * signed_radius,
            self.support().is_clockwise(),
            None,
        ))
    }
}

impl<'a> ExactCornerCarrier2<'a> {
    fn line_source(&self) -> Option<&'a LineSeg2> {
        match self {
            Self::Line(source) => Some(source),
            Self::PromotedLine(source) => source.retained_exact_line_image(),
            _ => None,
        }
    }

    pub(crate) fn retained_rational_arc(&self) -> Option<&Arc<RetainedRationalCornerArc2>> {
        match self {
            Self::RetainedRationalArc(arc) => Some(arc),
            _ => None,
        }
    }
}

fn retained_rational_arc_support(
    curve: &Curve2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CircularArc2>> {
    let support = match curve.geometry() {
        Some(CurveGeometry2::RationalQuadraticBezier(conic)) => {
            rational_quadratic_circular_arc(conic, policy)
        }
        Some(CurveGeometry2::RationalBezier(rational)) => {
            rational_bezier_circular_arc(rational, policy)
        }
        _ => return Ok(None),
    }
    .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?;
    Ok(match support {
        Classification::Decided(support) => support,
        Classification::Uncertain(_) => None,
    })
}

fn native_span_circular_arc(
    fragment: &NativeBezierFragment2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CircularArc2>> {
    let support = match fragment.native_curve() {
        BezierSubcurve2::RationalQuadratic(curve) => rational_quadratic_circular_arc(curve, policy),
        BezierSubcurve2::Rational(curve) => rational_bezier_circular_arc(curve, policy),
        BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => return Ok(None),
    }
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    match support {
        Classification::Decided(support) => Ok(support),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, family, reason))
        }
    }
}

pub(crate) fn exact_corner_carrier<'a>(
    curve: &'a Curve2,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<ExactCornerCarrier2<'a>>> {
    match curve.geometry() {
        Some(CurveGeometry2::Line(line)) => return Ok(Some(ExactCornerCarrier2::Line(line))),
        Some(CurveGeometry2::QuadraticBezier(source))
            if source.retained_exact_line_image().is_some() =>
        {
            return Ok(Some(ExactCornerCarrier2::PromotedLine(source)));
        }
        _ => {}
    }
    let retained = |support: CircularArc2| -> ExactCurveResult<ExactCornerCarrier2<'a>> {
        Ok(ExactCornerCarrier2::RetainedRationalArc(Arc::new(
            RetainedRationalCornerArc2::from_source(
                ExactCornerBezier2::Direct(curve),
                support,
                operation,
                curve.family(),
                policy,
            )?,
        )))
    };
    let bezier = || ExactCornerCarrier2::Bezier(curve);
    Ok(match curve.geometry() {
        None => {
            let fragment = curve.retained_fragment().expect("restricted carrier");
            if let Some(arc) =
                RetainedRationalCornerArc2::from_fragment(fragment, operation, policy)?
            {
                return Ok(Some(ExactCornerCarrier2::RetainedRationalArc(arc)));
            }
            match fragment {
                crate::BezierSplitFragment2::AlgebraicChord(chord) => {
                    Some(ExactCornerCarrier2::AlgebraicChord(chord))
                }
                crate::BezierSplitFragment2::AnalyticParallel(fragment) => {
                    Some(ExactCornerCarrier2::AnalyticParallel(fragment))
                }
                crate::BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => {
                    Some(ExactCornerCarrier2::AlgebraicCusp(fragment))
                }
                crate::BezierSplitFragment2::SelectedFiber(fragment) => {
                    Some(ExactCornerCarrier2::SelectedFiber(fragment))
                }
                crate::BezierSplitFragment2::RetainedBezier { .. }
                | crate::BezierSplitFragment2::Materialized { .. } => None,
            }
        }
        Some(CurveGeometry2::CircularArc(arc)) => Some(ExactCornerCarrier2::Arc(arc)),
        Some(CurveGeometry2::RationalQuadraticBezier(_))
        | Some(CurveGeometry2::RationalBezier(_)) => Some(
            match retained_rational_arc_support(curve, operation, policy)? {
                Some(support) => retained(support)?,
                None => bezier(),
            },
        ),
        Some(CurveGeometry2::QuadraticBezier(_)) | Some(CurveGeometry2::CubicBezier(_)) => {
            Some(bezier())
        }
        Some(CurveGeometry2::PolynomialBSpline(_)) | Some(CurveGeometry2::Nurbs(_)) => {
            let fragments = match curve
                .native_bezier_fragments_raw(policy)
                .map_err(|error| error.with_operation(operation))?
            {
                Classification::Decided(fragments) => fragments,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(operation, curve.family(), reason));
                }
            };
            let fragment = if previous {
                fragments.last()
            } else {
                fragments.first()
            }
            .ok_or_else(|| {
                ExactCurveError::invalid(
                    operation,
                    curve.family(),
                    CurveError::Topology(
                        "spline corner carrier did not promote an incident native span".into(),
                    ),
                )
            })?;
            Some(
                match native_span_circular_arc(fragment, operation, curve.family(), policy)? {
                    Some(support) => ExactCornerCarrier2::RetainedRationalArc(Arc::new(
                        RetainedRationalCornerArc2::from_source(
                            ExactCornerBezier2::NativeSpan(fragment),
                            support,
                            operation,
                            curve.family(),
                            policy,
                        )?,
                    )),
                    None => ExactCornerCarrier2::NativeBezierSpan(fragment),
                },
            )
        }
        Some(CurveGeometry2::Line(_)) => None,
    })
}

fn exact_corner_bezier_parallel(
    source: ExactCornerBezier2<'_>,
    distance: Real,
    operation: CurveOperation2,
    family: CurveFamily2,
) -> ExactCurveResult<BezierParallel2> {
    let parallel = match source {
        ExactCornerBezier2::Direct(source) => match source.geometry() {
            None => unreachable!("direct Bezier corner requires its native definition"),
            Some(CurveGeometry2::QuadraticBezier(source)) => source.parallel_left(distance),
            Some(CurveGeometry2::CubicBezier(source)) => source.parallel_left(distance),
            Some(CurveGeometry2::RationalQuadraticBezier(source)) => source.parallel_left(distance),
            Some(CurveGeometry2::RationalBezier(source)) => source.parallel_left(distance),
            Some(CurveGeometry2::Line(_))
            | Some(CurveGeometry2::CircularArc(_))
            | Some(CurveGeometry2::PolynomialBSpline(_))
            | Some(CurveGeometry2::Nurbs(_)) => {
                unreachable!("only direct Bezier corner carriers request an analytic parallel")
            }
        },
        ExactCornerBezier2::NativeSpan(fragment) => match fragment.native_curve() {
            BezierSubcurve2::Quadratic(source) => source.parallel_left(distance),
            BezierSubcurve2::Cubic(source) => source.parallel_left(distance),
            BezierSubcurve2::RationalQuadratic(source) => source.parallel_left(distance),
            BezierSubcurve2::Rational(source) => source.parallel_left(distance),
        },
    };
    parallel.map_err(|cause| ExactCurveError::invalid(operation, family, cause))
}

impl<'a> ExactCornerBezier2<'a> {
    fn parameter_range(self) -> (&'a Real, &'a Real) {
        match self {
            Self::Direct(source) => source
                .parameter_domain()
                .scalar_endpoints()
                .expect("direct native Bezier domain"),
            Self::NativeSpan(fragment) => fragment.parameter_range(),
        }
    }

    fn corner(self, previous: bool) -> &'a Point2 {
        match self {
            Self::Direct(source) => {
                let geometry = source.geometry().expect("direct native Bezier corner");
                if previous {
                    geometry.end()
                } else {
                    geometry.start()
                }
            }
            Self::NativeSpan(fragment) => {
                if previous {
                    fragment.native_curve().end()
                } else {
                    fragment.native_curve().start()
                }
            }
        }
    }

    fn curve_parameter(
        self,
        parameter: &CurveParameter2,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveParameter2> {
        let (start, end) = self.parameter_range();
        let scale = end - start;
        if let Some(parameter) = parameter.scalar() {
            return Ok((start + scale * parameter).into());
        }
        // Selected cuts name the same authored chart as stored scalar cuts.
        // Keep their local field while transporting the span's affine map;
        // a spline's knot interval is not generally the unit interval.
        match parameter
            .affine_image_unbounded(&scale, start, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(parameter) => Ok(parameter),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, family, reason))
            }
        }
    }
}

pub(crate) fn compact_optional_corner_solutions<T>(
    solutions: CurveCornerSolutions2<Option<T>>,
) -> CurveCornerSolutions2<T> {
    let mut candidates = match solutions {
        CurveCornerSolutions2::NoSolution(reason) => {
            return CurveCornerSolutions2::NoSolution(reason);
        }
        CurveCornerSolutions2::Unique(Some(candidate)) => {
            return CurveCornerSolutions2::Unique(candidate);
        }
        CurveCornerSolutions2::Unique(None) => {
            return CurveCornerSolutions2::NoSolution(
                crate::CurveCornerNoSolution2::OutsideTrimDomain,
            );
        }
        CurveCornerSolutions2::Multiple(candidates) => {
            candidates.into_iter().flatten().collect::<Vec<_>>()
        }
    };
    match candidates.len() {
        0 => CurveCornerSolutions2::NoSolution(crate::CurveCornerNoSolution2::OutsideTrimDomain),
        1 => CurveCornerSolutions2::Unique(
            candidates
                .pop()
                .expect("one retained corner candidate remains"),
        ),
        _ => CurveCornerSolutions2::Multiple(candidates),
    }
}

pub(crate) fn try_map_corner_solutions<T, U>(
    solutions: CurveCornerSolutions2<T>,
    mut map: impl FnMut(T) -> ExactCurveResult<U>,
) -> ExactCurveResult<CurveCornerSolutions2<U>> {
    match solutions {
        CurveCornerSolutions2::NoSolution(reason) => Ok(CurveCornerSolutions2::NoSolution(reason)),
        CurveCornerSolutions2::Unique(candidate) => {
            map(candidate).map(CurveCornerSolutions2::Unique)
        }
        CurveCornerSolutions2::Multiple(candidates) => candidates
            .into_iter()
            .map(map)
            .collect::<ExactCurveResult<Vec<_>>>()
            .map(CurveCornerSolutions2::Multiple),
    }
}

pub(crate) fn validate_corner_design_value(
    value: &Real,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<RealSign> {
    match crate::classify::real_sign(value, policy) {
        Some(sign @ (RealSign::Zero | RealSign::Positive)) => Ok(sign),
        Some(RealSign::Negative) => Err(ExactCurveError::invalid(
            operation,
            family,
            CurveError::InvalidCornerOptions,
        )),
        None => Err(ExactCurveError::blocked(
            operation,
            family,
            crate::UncertaintyReason::RealSign,
        )),
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn solve_exact_chamfer_corner(
    previous: ExactCornerCarrier2<'_>,
    next: ExactCornerCarrier2<'_>,
    previous_setback: &Real,
    next_setback: &Real,
    previous_sign: RealSign,
    next_sign: RealSign,
    mode: CurveCornerMode2,
    previous_logical_run: bool,
    next_logical_run: bool,
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<ChamferCorner2>> {
    if previous_sign == RealSign::Zero && next_sign == RealSign::Zero {
        return Ok(CurveCornerSolutions2::NoSolution(
            CurveCornerNoSolution2::ZeroDesignValue,
        ));
    }
    let previous_cuts = corner_chamfer_cuts(
        previous,
        previous_setback,
        previous_sign,
        true,
        mode,
        previous_logical_run,
        CurveOperation2::Chamfer,
        previous_family,
        policy,
    );
    if matches!(&previous_cuts, Ok(cuts) if cuts.is_empty()) {
        return Ok(CurveCornerSolutions2::NoSolution(
            CurveCornerNoSolution2::OutsideTrimDomain,
        ));
    }
    let next_cuts = corner_chamfer_cuts(
        next,
        next_setback,
        next_sign,
        false,
        mode,
        next_logical_run,
        CurveOperation2::Chamfer,
        next_family,
        policy,
    );
    combine_chamfer_cuts(previous_cuts, next_cuts, previous_family, policy)
}

fn combine_chamfer_cuts(
    previous_cuts: ExactCurveResult<CornerCuts2>,
    next_cuts: ExactCurveResult<CornerCuts2>,
    previous_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<ChamferCorner2>> {
    if matches!(&previous_cuts, Ok(cuts) if cuts.is_empty()) {
        return Ok(CurveCornerSolutions2::NoSolution(
            CurveCornerNoSolution2::OutsideTrimDomain,
        ));
    }
    if matches!(&next_cuts, Ok(cuts) if cuts.is_empty()) {
        return Ok(CurveCornerSolutions2::NoSolution(
            CurveCornerNoSolution2::OutsideTrimDomain,
        ));
    }
    let previous_cuts = previous_cuts?;
    let next_cuts = next_cuts?;
    let mut candidates = CornerSolutionAccumulator::Empty;
    for previous in previous_cuts.iter() {
        for next in next_cuts.iter() {
            match previous.point.same_point(&next.point, policy) {
                Classification::Decided(true) => continue,
                Classification::Decided(false) => candidates.push(ChamferCorner2 {
                    previous: previous.clone(),
                    next: next.clone(),
                }),
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Chamfer,
                        previous_family,
                        reason,
                    ));
                }
            }
        }
    }
    Ok(candidates.finish(CurveCornerNoSolution2::DegenerateCandidate))
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn solve_exact_fillet_corner(
    previous: ExactCornerCarrier2<'_>,
    next: ExactCornerCarrier2<'_>,
    radius: &Real,
    radius_sign: RealSign,
    mode: CurveCornerMode2,
    retain_selected_circle_endpoints: bool,
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    constraints: Option<&FilletConstraintBinding2<'_>>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    match radius_sign {
        RealSign::Zero => {
            return Ok(CurveCornerSolutions2::NoSolution(
                CurveCornerNoSolution2::ZeroDesignValue,
            ));
        }
        RealSign::Positive => {}
        RealSign::Negative => unreachable!("negative corner values are rejected"),
    }
    if let (Some(previous_line), Some(next_line)) = (previous.line_source(), next.line_source()) {
        return solve_line_fillet_corner(
            previous_line,
            next_line,
            radius,
            mode,
            previous_family,
            next_family,
            constraints,
            policy,
        );
    }
    solve_carrier_fillet_corner(
        previous,
        next,
        radius,
        retain_selected_circle_endpoints,
        [FilletContactDomain2::AuthoredCurve(mode); 2],
        previous_family,
        next_family,
        constraints,
        policy,
    )
}

#[derive(Clone, Copy)]
enum FilletLinearSource2<'a> {
    Native {
        source: &'a LineSeg2,
        parameterization: Option<&'a QuadraticBezier2>,
        parallel_tangent_contacts: &'a [crate::bezier::BezierParallelLineTangentContact2],
    },
    AlgebraicChord(&'a crate::BezierAlgebraicChord2),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distinct_parallel_sources_keep_a_shared_fillet_center_family() {
        let q = |n: i64, d: i64| (Real::from(n) / Real::from(d)).unwrap();
        // P(t)=(3t/8,9t^2/64). The reversed offsets 41/64 and 5/8
        // join at (0,41/64). Their common clockwise radius-1/128 center
        // support is the offset 81/128. At t=5/9 the two original
        // derivative scales are -17/2197 and 37/2197, respectively.
        let source = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(q(3, 16), Real::zero()),
            Point2::new(q(3, 8), q(9, 64)),
        )
        .parallel_left(Real::zero())
        .unwrap();
        let original = [
            source.with_distance(q(41, 64)),
            source.with_distance(q(5, 8)),
        ];
        let radius = q(1, 128);
        let center_distance = q(81, 128);
        let parameter = CurveParameter2::from(q(5, 9));
        let range = CurveParameterRange2::new_validated(
            CurveParameter2::from(q(5, 9) - q(1, 10_000)),
            CurveParameter2::from(q(5, 9) + q(1, 10_000)),
        );
        let contacts = [
            Point2::new(q(-95, 2496), q(4753, 7488)),
            Point2::new(q(-5, 156), q(4645, 7488)),
        ];
        let center = Point2::new(q(-175, 4992), q(4699, 7488));
        for point in &contacts {
            let dx = point.x() - center.x();
            let dy = point.y() - center.y();
            assert_eq!(&dx * &dx + &dy * &dy, &radius * &radius);
        }
        assert_ne!(contacts[0], contacts[1]);
        let domains = [FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOnly); 2];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let point_at = |parallel: &BezierParallel2, parameter: &CurveParameter2| {
                analytic_parallel_point_evidence(
                    parallel,
                    parameter,
                    CurveOperation2::Fillet,
                    CurveFamily2::AnalyticParallel,
                    &policy,
                )
                .unwrap()
            };
            for (parallel, expected) in original.iter().zip(&contacts) {
                assert_eq!(
                    point_at(parallel, &parameter)
                        .same_point(&CurvePoint2::from(expected.clone()), &policy,),
                    Classification::Decided(true)
                );
            }
            assert_eq!(
                point_at(&source.with_distance(center_distance.clone()), &parameter)
                    .same_point(&CurvePoint2::from(center.clone()), &policy),
                Classification::Decided(true)
            );
            let selected = original.each_ref().map(|parallel| {
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(
                        parallel.clone(),
                    ),
                    range.clone(),
                    point_at(parallel, range.start()),
                    point_at(parallel, range.end()),
                )
                .reversed()
            });
            for reversed in [false, true] {
                let curves = if reversed {
                    [selected[1].reversed(), selected[0].reversed()]
                } else {
                    selected.clone()
                };
                let prepared = curves.each_ref().map(|curve| {
                    PreparedFilletCarrier2::new(
                        ExactCornerCarrier2::SelectedFiber(curve),
                        CurveFamily2::AnalyticParallel,
                        domains[0],
                        &policy,
                    )
                    .unwrap()
                });
                let signed_radius = if reversed {
                    radius.clone()
                } else {
                    -radius.clone()
                };
                let previous_offsets = prepared[0]
                    .offsets(&signed_radius, CurveFamily2::AnalyticParallel, &policy)
                    .unwrap();
                let next_offsets = prepared[1]
                    .offsets(&signed_radius, CurveFamily2::AnalyticParallel, &policy)
                    .unwrap();
                let is_shared = |offset: &&FilletOffsetCarrier2<'_, '_>| {
                    matches!(
                        offset, FilletOffsetCarrier2::Parallel { support, .. }
                        if support.distance() == &center_distance
                    )
                };
                let previous = previous_offsets
                    .iter()
                    .flatten()
                    .find(is_shared)
                    .expect("the previous normal sheet reaches the rational center");
                let next = next_offsets
                    .iter()
                    .flatten()
                    .find(is_shared)
                    .expect("the next normal sheet reaches the same rational center");
                let accepted = |parameter: &CurveParameter2| {
                    [previous, next].into_iter().zip(&prepared).enumerate().all(
                        |(axis, (offset, prepared))| {
                            prepared
                                .accepts_offset_contact(
                                    offset,
                                    Some(parameter),
                                    axis == 0,
                                    &signed_radius,
                                    CurveFamily2::AnalyticParallel,
                                    &policy,
                                )
                                .unwrap()
                        },
                    )
                };
                assert!(accepted(&parameter));
                // Coincident centers alone do not establish original tangent
                // orientation. Outside the opposite-sign cusp band these
                // same center supports are incompatible normal sheets.
                assert!(!accepted(&CurveParameter2::from(Real::zero())));
                assert!(!accepted(&CurveParameter2::from(Real::one())));
                let constraints = [
                    prepared[0]
                        .component_normal_constraint(
                            previous,
                            &signed_radius,
                            hypersolve::CurveResultantParameter::First,
                            CurveFamily2::AnalyticParallel,
                        )
                        .unwrap()
                        .unwrap(),
                    prepared[1]
                        .component_normal_constraint(
                            next,
                            &signed_radius,
                            hypersolve::CurveResultantParameter::Second,
                            CurveFamily2::AnalyticParallel,
                        )
                        .unwrap()
                        .unwrap(),
                ];
                let centers = fillet_offset_centers(
                    previous,
                    next,
                    domains,
                    CurveFamily2::AnalyticParallel,
                    CurveFamily2::AnalyticParallel,
                    Some(&constraints),
                    &policy,
                )
                .expect("the exact common center support must classify");
                assert!(
                    !centers.components.is_empty(),
                    "different original offsets have a noncollapsed diagonal center family"
                );
                assert!(centers.components.iter().any(|component| {
                    component
                        .contains_pair(&parameter, &parameter, &policy)
                        .unwrap()
                        == Classification::Decided(true)
                }));
                for outside in [Real::zero(), Real::one()] {
                    let outside = CurveParameter2::from(outside);
                    for component in &centers.components {
                        assert_eq!(
                            component
                                .contains_pair(&outside, &outside, &policy)
                                .unwrap(),
                            Classification::Decided(false)
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn selected_parallel_fillet_keeps_contacts_beyond_interior_cusps() {
        let q = |n: i64, d: i64| (Real::from(n) / Real::from(d)).unwrap();
        // P(u)=(x,x^2), x=(12u-6)/5, with left offset 1. Its
        // midpoint traverses against P, but u=7/9 (x=2/3) agrees with P.
        let source = QuadraticBezier2::new(
            Point2::new(q(-6, 5), q(36, 25)),
            Point2::new(Real::zero(), q(-36, 25)),
            Point2::new(q(6, 5), q(36, 25)),
        )
        .parallel_left(Real::one())
        .unwrap();
        let start = Point2::new(q(-18, 65), q(593, 325));
        let end = Point2::new(q(18, 65), q(593, 325));
        let line =
            LineSeg2::try_new(end.clone(), end.translated(Real::zero(), Real::one())).unwrap();
        let selected = crate::bezier_split::BezierSelectedFiberFragment2::new(
            crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(source),
            CurveParameterRange2::unit(),
            start.into(),
            end.into(),
        );
        // The tangent at x=2/3 is (3,4)/5. A circle of radius 80/39
        // centered at (-346/195,1331/585) is tangent there and to the
        // upward line through the source endpoint. Both contacts are strict
        // trims: u=7/9 and v=1318/2925. All design values are rational.
        let center = CurvePoint2::from(Point2::new(q(-346, 195), q(1331, 585)));
        let source_parameter = CurveParameter2::from(q(7, 9));
        let line_parameter = q(1318, 2925);
        let radius = q(80, 39);
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let reversed_selected = selected.reversed();
                let reversed_line = line.reversed();
                let (previous, next, previous_family, next_family) = if reversed {
                    (
                        ExactCornerCarrier2::Line(&reversed_line),
                        ExactCornerCarrier2::SelectedFiber(&reversed_selected),
                        CurveFamily2::Line,
                        CurveFamily2::AnalyticParallel,
                    )
                } else {
                    (
                        ExactCornerCarrier2::SelectedFiber(&selected),
                        ExactCornerCarrier2::Line(&line),
                        CurveFamily2::AnalyticParallel,
                        CurveFamily2::Line,
                    )
                };
                let solutions = solve_carrier_fillet_corner(
                    previous,
                    next,
                    &radius,
                    true,
                    [FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOnly); 2],
                    previous_family,
                    next_family,
                    None,
                    &policy,
                )
                .unwrap();
                let candidates = solutions.into_solutions();
                let expected_line = CurveParameter2::from(if reversed {
                    Real::one() - &line_parameter
                } else {
                    line_parameter.clone()
                });
                let candidate = candidates.iter().find(|candidate| {
                    let (source_cut, line_cut) = if reversed {
                        (&candidate.next, &candidate.previous)
                    } else {
                        (&candidate.previous, &candidate.next)
                    };
                    source_cut.parameter.as_ref().is_some_and(|parameter| {
                        parameter.same_value(&source_parameter, &policy).unwrap()
                            == Classification::Decided(true)
                    }) && line_cut.parameter.as_ref().is_some_and(|parameter| {
                        parameter.same_value(&expected_line, &policy).unwrap()
                            == Classification::Decided(true)
                    })
                });
                let candidate = candidate
                    .expect("the exact tangent circle on the outer source branch must be retained");
                assert_eq!(candidate.clockwise, reversed);
                assert_eq!(
                    candidate.center.same_point(&center, &policy),
                    Classification::Decided(true)
                );
            }
        }
    }

    #[test]
    fn parallel_fillet_extension_keeps_contacts_across_source_cusps() {
        let q = |n: i64, d: i64| (Real::from(n) / Real::from(d)).unwrap();
        let source = QuadraticBezier2::new(
            Point2::new(q(-6, 5), q(36, 25)),
            Point2::new(Real::zero(), q(-36, 25)),
            Point2::new(q(6, 5), q(36, 25)),
        )
        .parallel_left(Real::one())
        .unwrap();
        // The finite source x-range [0,3/8] has negative derivative scale.
        // Extending to x=2/3 crosses its cusp and reaches positive scale.
        let start = Point2::from_values(0, 1);
        let end = Point2::new(q(-9, 40), q(301, 320));
        let range = BezierParameterRange2::new_validated(
            BezierParameter2::Exact(q(1, 2)),
            BezierParameter2::Exact(q(21, 32)),
        );
        let selected = crate::bezier_split::BezierSelectedFiberFragment2::new(
            crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(source.clone()),
            CurveParameterRange2::from_bezier_range(range.clone()),
            start.into(),
            end.clone().into(),
        );
        let line =
            LineSeg2::try_new(end.clone(), end.translated(Real::zero(), -Real::one())).unwrap();
        // Radius 11/216, center (-47/270,43/40), contacts u=7/9 and
        // v=-43/320. Both contacts extend their respective authored curves.
        let center = CurvePoint2::from(Point2::new(q(-47, 270), q(43, 40)));
        let source_parameter = CurveParameter2::from(q(7, 9));
        let line_parameter = q(-43, 320);
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(retained) =
                crate::BezierParallelFragment2::try_new(source.clone(), range.clone(), &policy)
                    .unwrap()
            else {
                panic!("the finite source fragment is regular and cusp-free")
            };
            for is_selected in [false, true] {
                for reversed in [false, true] {
                    let reversed_selected = selected.reversed();
                    let reversed_retained = retained.reversed();
                    let reversed_line = line.reversed();
                    for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                        let carrier = match (is_selected, reversed) {
                            (false, false) => ExactCornerCarrier2::AnalyticParallel(&retained),
                            (false, true) => {
                                ExactCornerCarrier2::AnalyticParallel(&reversed_retained)
                            }
                            (true, false) => ExactCornerCarrier2::SelectedFiber(&selected),
                            (true, true) => ExactCornerCarrier2::SelectedFiber(&reversed_selected),
                        };
                        let (previous, next, previous_family, next_family) = if reversed {
                            (
                                ExactCornerCarrier2::Line(&reversed_line),
                                carrier,
                                CurveFamily2::Line,
                                CurveFamily2::AnalyticParallel,
                            )
                        } else {
                            (
                                carrier,
                                ExactCornerCarrier2::Line(&line),
                                CurveFamily2::AnalyticParallel,
                                CurveFamily2::Line,
                            )
                        };
                        let solutions = solve_carrier_fillet_corner(
                            previous,
                            next,
                            &q(11, 216),
                            true,
                            [FilletContactDomain2::AuthoredCurve(mode); 2],
                            previous_family,
                            next_family,
                            None,
                            &policy,
                        )
                        .unwrap();
                        let candidates = solutions.into_solutions();
                        let expected_line = CurveParameter2::from(if reversed {
                            Real::one() - &line_parameter
                        } else {
                            line_parameter.clone()
                        });
                        let candidate = candidates.iter().find(|candidate| {
                            let (source_cut, line_cut) = if reversed {
                                (&candidate.next, &candidate.previous)
                            } else {
                                (&candidate.previous, &candidate.next)
                            };
                            source_cut.parameter.as_ref().is_some_and(|parameter| {
                                parameter.same_value(&source_parameter, &policy).unwrap()
                                    == Classification::Decided(true)
                            }) && line_cut.parameter.as_ref().is_some_and(|parameter| {
                                parameter.same_value(&expected_line, &policy).unwrap()
                                    == Classification::Decided(true)
                            })
                        });
                        assert_eq!(
                            candidate.is_some(),
                            mode == CurveCornerMode2::TrimOrExtend,
                            "a cusp changes orientation, not the authorized extension domain"
                        );
                        if let Some(candidate) = candidate {
                            assert_eq!(candidate.clockwise, reversed);
                            assert_eq!(candidate.previous.placement, CornerPlacement2::Extension);
                            assert_eq!(candidate.next.placement, CornerPlacement2::Extension);
                            assert_eq!(
                                candidate.center.same_point(&center, &policy),
                                Classification::Decided(true)
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn fillet_center_contacts_keep_source_orientation_across_support_cusps() {
        let p = Point2::from_values;
        let half = (Real::one() / Real::from(2)).unwrap();
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let curve = QuadraticBezier2::new(p(4, 0), p(3, 4), p(2, 0));
        let authored = Curve2::from(curve.clone());
        let line = LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let prepared = PreparedFilletCarrier2::new(
                ExactCornerCarrier2::Line(&line),
                CurveFamily2::Line,
                FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOnly),
                &policy,
            )
            .unwrap();
            let [Some(previous), None] = prepared
                .offsets(&half, CurveFamily2::Line, &policy)
                .unwrap()
            else {
                panic!("a line has one oriented center support");
            };
            let parallel = curve.parallel_left(Real::zero()).unwrap();
            let Classification::Decided(retained) = crate::BezierParallelFragment2::try_new(
                parallel.clone(),
                BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
                &policy,
            )
            .unwrap() else {
                panic!("the zero-distance cap is regular");
            };
            let selected = crate::bezier_split::BezierSelectedFiberFragment2::new(
                crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(parallel.clone()),
                CurveParameterRange2::unit(),
                p(4, 0).into(),
                p(2, 0).into(),
            );
            let reversed_retained = retained.reversed();
            let reversed_selected = selected.reversed();
            for (source, reversed) in [
                (
                    FilletParallelSource2::Direct(ExactCornerBezier2::Direct(&authored)),
                    false,
                ),
                (FilletParallelSource2::Retained(&retained), false),
                (FilletParallelSource2::Selected(&selected), false),
                (FilletParallelSource2::Retained(&reversed_retained), true),
                (FilletParallelSource2::Selected(&reversed_selected), true),
            ] {
                let next = FilletOffsetCarrier2::Parallel {
                    source,
                    support: parallel.with_distance(half.clone()),
                };
                let centers = fillet_offset_centers(
                    &previous,
                    &next,
                    [FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOnly); 2],
                    CurveFamily2::Line,
                    CurveFamily2::QuadraticBezier,
                    None,
                    &policy,
                )
                .unwrap();
                assert_eq!(centers.iter().count(), 2);
                for center in centers.iter() {
                    let parameter = center.next_parameter.as_ref().unwrap();
                    let Classification::Decided(side) = parameter
                        .polynomial_sign(&[-half.clone(), Real::one()], &policy)
                        .unwrap()
                    else {
                        panic!("each contact lies on one side of the cap");
                    };
                    let boundary = if side == RealSign::Negative {
                        quarter.clone()
                    } else {
                        Real::one() - &quarter
                    };
                    assert_eq!(
                        parameter
                            .polynomial_sign(&[-boundary, Real::one()], &policy)
                            .unwrap(),
                        Classification::Decided(side)
                    );
                    // P'=(-2,8-16t): its dot with the positive x-axis is
                    // negative, and P' x (1,0) has the sign of t-1/2.
                    // At these outer contacts the offset scale is positive;
                    // at the unrelated midpoint it is negative (radius 1/4).
                    let evidence = center.retained_anchor_evidence.as_ref().unwrap();
                    assert_eq!(
                        evidence.dot,
                        Some(if reversed {
                            RealSign::Positive
                        } else {
                            RealSign::Negative
                        })
                    );
                    assert_eq!(
                        evidence.cross,
                        Some(if reversed {
                            reverse_fillet_sign(side)
                        } else {
                            side
                        })
                    );
                    assert_eq!(
                        evidence.source_direction,
                        Some(if reversed {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        })
                    );
                }
            }
        }
    }
    #[test]
    fn parallel_derivative_orientation_replays_unprojectable_selected_fibers() {
        let p = Point2::from_values;
        let half = (Real::one() / Real::from(2)).unwrap();
        let parallel = QuadraticBezier2::new(p(4, 0), p(3, 4), p(2, 0))
            .parallel_left(half.clone())
            .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // u^135 = 1/(2*scale^9). The first root lies in (0.99,1),
            // the second in (0.49,0.5), on opposite sides of a support cusp.
            for (scale, expected) in [(1, RealSign::Positive), (32_768, RealSign::Negative)] {
                let selected = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
                    half.clone(),
                    scale,
                    &policy,
                );
                assert!(matches!(
                    selected.promoted_bezier_parameter(&policy).unwrap(),
                    Classification::Uncertain(_)
                ));
                let parameter = CurveParameter2::from_selected_fiber(selected);
                assert_eq!(
                    parallel
                        .parallel_derivative_scale_sign(&parameter, &policy)
                        .unwrap(),
                    Classification::Decided(expected)
                );
            }
        }
    }

    #[test]
    fn fillet_domains_control_each_circular_contact_independently() {
        let line =
            LineSeg2::try_new(Point2::from_values(-3, 0), Point2::from_values(0, 0)).unwrap();
        let arc = CircularArc2::try_from_center(
            Point2::from_values(1, 0),
            Point2::from_values(0, -1),
            Point2::from_values(0, 0),
            true,
        )
        .unwrap();
        let radius = (Real::one() / Real::from(4)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // These finite charts need not meet at the authored vertex. Each
            // extension permission belongs to its own source and is applied
            // both to center enumeration and to contact publication.
            for (previous_mode, next_mode, count) in [
                (CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOnly, 0),
                (
                    CurveCornerMode2::TrimOrExtend,
                    CurveCornerMode2::TrimOnly,
                    1,
                ),
                (
                    CurveCornerMode2::TrimOnly,
                    CurveCornerMode2::TrimOrExtend,
                    2,
                ),
                (
                    CurveCornerMode2::TrimOrExtend,
                    CurveCornerMode2::TrimOrExtend,
                    4,
                ),
            ] {
                let solutions = solve_carrier_fillet_corner(
                    ExactCornerCarrier2::Line(&line),
                    ExactCornerCarrier2::Arc(&arc),
                    &radius,
                    false,
                    [
                        FilletContactDomain2::AuthoredCurve(previous_mode),
                        FilletContactDomain2::AuthoredCurve(next_mode),
                    ],
                    CurveFamily2::Line,
                    CurveFamily2::CircularArc,
                    None,
                    &policy,
                )
                .unwrap();
                assert_eq!(
                    solutions.solutions().len(),
                    count,
                    "previous={previous_mode:?}, next={next_mode:?}, policy={policy:?}"
                );
            }
        }
    }

    #[test]
    fn fillet_parallel_domains_expand_only_the_authorized_source() {
        let half = (Real::one() / Real::from(2)).unwrap();
        let first = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(half.clone(), Real::zero()),
            Point2::from_values(1, 0),
        );
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for exterior_is_previous in [false, true] {
                let x = if exterior_is_previous {
                    Real::from(2) + &half
                } else {
                    Real::one() + &half
                };
                let y = if exterior_is_previous {
                    Real::zero()
                } else {
                    Real::from(2)
                };
                let second = QuadraticBezier2::new(
                    Point2::new(x.clone(), y.clone()),
                    Point2::new(x.clone(), &y + Real::one()),
                    Point2::new(x, &y + Real::from(2)),
                );
                let first_curve = Curve2::from(first.clone());
                let second_curve = Curve2::from(second.clone());
                let previous = FilletOffsetCarrier2::Parallel {
                    source: FilletParallelSource2::Direct(ExactCornerBezier2::Direct(&first_curve)),
                    support: first.parallel_left(Real::one()).unwrap(),
                };
                let next = FilletOffsetCarrier2::Parallel {
                    source: FilletParallelSource2::Direct(ExactCornerBezier2::Direct(
                        &second_curve,
                    )),
                    support: second.parallel_left(Real::one()).unwrap(),
                };
                for previous_mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    for next_mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                        let centers = fillet_offset_centers(
                            &previous,
                            &next,
                            [
                                FilletContactDomain2::AuthoredCurve(previous_mode),
                                FilletContactDomain2::AuthoredCurve(next_mode),
                            ],
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::QuadraticBezier,
                            None,
                            &policy,
                        )
                        .unwrap();
                        let expected = if exterior_is_previous {
                            previous_mode
                        } else {
                            next_mode
                        } == CurveCornerMode2::TrimOrExtend;
                        assert!(centers.coincident.is_none());
                        assert_eq!(
                            centers.iter().count(),
                            usize::from(expected),
                            "exterior_previous={exterior_is_previous}, previous={previous_mode:?}, next={next_mode:?}, policy={policy:?}"
                        );
                        if let Some(center) = centers.iter().next() {
                            let expected_parameters = if exterior_is_previous {
                                [Real::one() + &half, half.clone()]
                            } else {
                                [half.clone(), -&half]
                            };
                            for (previous, expected) in
                                [true, false].into_iter().zip(expected_parameters)
                            {
                                assert_eq!(
                                    center
                                        .parameter(previous)
                                        .unwrap()
                                        .cmp_by_refinement(&expected.into(), &policy)
                                        .unwrap(),
                                    Classification::Decided(std::cmp::Ordering::Equal)
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    fn corner_splines_in_shifted_knot_domain(policy: &CurveContext) -> [Curve2; 2] {
        let controls = vec![
            Point2::from_values(0, 0),
            Point2::from_values(0, 1),
            Point2::from_values(1, 2),
        ];
        let knots = vec![3, 3, 3, 7, 7, 7]
            .into_iter()
            .map(Real::from)
            .collect::<Vec<_>>();
        [
            Curve2::try_polynomial_bspline_with_policy(2, controls.clone(), knots.clone(), policy)
                .unwrap()
                .value,
            Curve2::try_nurbs_with_policy(2, controls, vec![Real::one(); 3], knots, policy)
                .unwrap()
                .value,
        ]
    }

    fn assert_spline_cut_replays_in_authored_chart(
        curve: &Curve2,
        cut: &CornerCut2,
        policy: &CurveContext,
    ) {
        let parameter = cut
            .parameter
            .as_ref()
            .expect("a spline cut retains its parameter");
        assert!(
            parameter.scalar().is_none(),
            "the fixture must exercise selected transport"
        );
        let point = curve
            .point_at_with_policy(parameter, policy)
            .expect("selected cut lies in the authored knot domain");
        assert_eq!(point.certainty, crate::CurveCertainty::Certified);
        let equal = point.value.coincides_with(&cut.point, policy);
        assert_eq!(equal.certainty, crate::CurveCertainty::Certified);
        assert_eq!(equal.value, Classification::Decided(true));
    }

    #[test]
    fn selected_spline_chamfer_cuts_reenter_the_authored_parameter_domain() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for curve in corner_splines_in_shifted_knot_domain(&policy) {
                for previous in [false, true] {
                    let carrier =
                        exact_corner_carrier(&curve, previous, CurveOperation2::Chamfer, &policy)
                            .unwrap()
                            .unwrap();
                    let cuts = corner_chamfer_cuts(
                        carrier,
                        &Real::one(),
                        RealSign::Positive,
                        previous,
                        CurveCornerMode2::TrimOnly,
                        false,
                        CurveOperation2::Chamfer,
                        curve.family(),
                        &policy,
                    )
                    .unwrap();
                    assert!(!cuts.is_empty());
                    for cut in cuts.iter() {
                        assert_spline_cut_replays_in_authored_chart(&curve, cut, &policy);
                    }
                }
            }
        }
    }

    #[test]
    fn selected_spline_fillet_cuts_reenter_the_authored_parameter_domain() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for curve in corner_splines_in_shifted_knot_domain(&policy) {
                let line = Curve2::from(
                    LineSeg2::try_new(Point2::from_values(-4, 0), Point2::from_values(0, 0))
                        .unwrap(),
                );
                let solutions = solve_exact_fillet_corner(
                    exact_corner_carrier(&line, true, CurveOperation2::Fillet, &policy)
                        .unwrap()
                        .unwrap(),
                    exact_corner_carrier(&curve, false, CurveOperation2::Fillet, &policy)
                        .unwrap()
                        .unwrap(),
                    &Real::one(),
                    RealSign::Positive,
                    CurveCornerMode2::TrimOnly,
                    false,
                    line.family(),
                    curve.family(),
                    None,
                    &policy,
                )
                .unwrap();
                let solutions = solutions.into_solutions();
                assert!(!solutions.is_empty());
                let cuts: Vec<_> = solutions
                    .into_iter()
                    .map(|solution| solution.next)
                    .collect();
                for cut in &cuts {
                    assert_spline_cut_replays_in_authored_chart(&curve, cut, &policy);
                }
            }
        }
    }

    #[test]
    fn retained_endpoints_share_projections_without_retaining_the_curve_owner() {
        let parallel = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(2, 2),
            Point2::from_values(4, 0),
        )
        .parallel_left(Real::one())
        .unwrap();
        let range = BezierParameterRange2::new_validated(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        );
        let Classification::Decided(fragment) =
            crate::BezierParallelFragment2::try_new(parallel, range, &CurveContext::STRICT)
                .unwrap()
        else {
            panic!("regular analytic parallel")
        };
        let curve =
            Curve2::from_retained_fragment(crate::BezierSplitFragment2::AnalyticParallel(fragment));
        let owner = Arc::downgrade(&curve.data);
        let start = curve.start();
        let end = curve.end();
        for _ in 0..16 {
            assert!(start.shares_storage(&curve.start()));
            assert!(end.shares_storage(&curve.end()));
        }
        assert!(!start.shares_storage(&end));
        drop(curve);
        assert!(owner.upgrade().is_none());
        for point in [start, end] {
            let bounds = point.bounds(&CurveContext::STRICT);
            assert_eq!(bounds.certainty, crate::CurveCertainty::Certified);
            assert!(matches!(bounds.value, Classification::Decided(_)));
        }
    }

    #[test]
    fn polynomial_exterior_subcurve_can_end_at_parameter_zero() {
        let source = CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 2),
            Point2::from_values(3, 2),
            Point2::from_values(4, 0),
        );
        let start = -Real::one();
        let middle = (-Real::one() / Real::from(2_i8)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let exterior = source
                .subcurve_between_affine_exact(&start, &Real::zero(), &policy)
                .expect("a finite exterior interval ending at zero must materialize");
            assert_eq!(exterior.start(), &source.point_at(start.clone()));
            assert_eq!(exterior.end(), source.start());
            assert_eq!(
                exterior.point_at((Real::one() / Real::from(2_i8)).unwrap()),
                source.point_at(middle.clone())
            );
        }
    }

    #[test]
    fn represented_single_curve_seam_materializes_one_direct_bezier_interval() {
        let cubic = CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(2, 0),
            Point2::from_values(0, 2),
            Point2::from_values(0, 0),
        );
        let source = Curve2::from(cubic.clone());
        let next_parameter = (Real::one() / Real::from(4_i8)).unwrap();
        let previous_parameter = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
        let next_point = cubic.point_at(next_parameter.clone());
        let previous_point = cubic.point_at(previous_parameter.clone());
        let next = CornerCut2 {
            parameter: exact_corner_parameter(next_parameter),
            point: next_point.clone().into(),
            placement: CornerPlacement2::Trim,
        };
        let previous = CornerCut2 {
            parameter: exact_corner_parameter(previous_parameter),
            point: previous_point.clone().into(),
            placement: CornerPlacement2::Trim,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let MaterializedCornerBody2::One(body) = materialize_single_curve_corner_body(
                &source,
                &previous,
                &next,
                CurveOperation2::Chamfer,
                &policy,
            )
            .expect("one represented seam interval must materialize once") else {
                panic!("one direct Bezier seam must remain one exact interval");
            };
            assert_eq!(body.family(), CurveFamily2::CubicBezier);
            assert_eq!(body.start(), crate::CurvePoint2::from(next_point.clone()));
            assert_eq!(body.end(), crate::CurvePoint2::from(previous_point.clone()));
            let Some(CurveGeometry2::CubicBezier(body)) = body.geometry() else {
                unreachable!();
            };
            assert_eq!(
                body.point_at((Real::one() / Real::from(2_i8)).unwrap()),
                cubic.point_at((Real::one() / Real::from(2_i8)).unwrap())
            );
        }

        let next_parameter = (-Real::one() / Real::from(4_i8)).unwrap();
        let previous_parameter = (Real::from(5_i8) / Real::from(4_i8)).unwrap();
        let next_point = cubic.point_at(next_parameter.clone());
        let previous_point = cubic.point_at(previous_parameter.clone());
        let next = CornerCut2 {
            parameter: exact_corner_parameter(next_parameter),
            point: next_point.clone().into(),
            placement: CornerPlacement2::Extension,
        };
        let previous = CornerCut2 {
            parameter: exact_corner_parameter(previous_parameter),
            point: previous_point.clone().into(),
            placement: CornerPlacement2::Extension,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let MaterializedCornerBody2::One(body) = materialize_single_curve_corner_body(
                &source,
                &previous,
                &next,
                CurveOperation2::Chamfer,
                &policy,
            )
            .expect("one direct Bezier exterior seam interval must materialize") else {
                panic!("one direct Bezier exterior seam must remain one exact interval");
            };
            assert_eq!(body.start(), crate::CurvePoint2::from(next_point.clone()));
            assert_eq!(body.end(), crate::CurvePoint2::from(previous_point.clone()));
            let Some(CurveGeometry2::CubicBezier(body)) = body.geometry() else {
                unreachable!();
            };
            assert_eq!(
                body.point_at((Real::one() / Real::from(2_i8)).unwrap()),
                cubic.point_at((Real::one() / Real::from(2_i8)).unwrap())
            );
        }
    }

    fn selected_inverse_square_parameter(
        denominator: i8,
        policy: &CurveContext,
    ) -> BezierParameter2 {
        let polynomial = match crate::BezierParameterPolynomial::try_new_power_basis(
            vec![-Real::one(), Real::zero(), Real::from(denominator)],
            policy,
        )
        .expect("the selected inverse-square polynomial is valid")
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                panic!("the selected inverse-square polynomial must decide: {reason:?}")
            }
        };
        let roots = match polynomial
            .isolate_unit_interval_roots(policy)
            .expect("the selected inverse-square roots isolate")
        {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => {
                panic!("the selected inverse-square root must decide: {reason:?}")
            }
        };
        let [parameter] = roots.as_slice() else {
            panic!("one positive inverse-square root must lie in the unit interval")
        };
        parameter.clone()
    }

    #[test]
    fn native_arc_and_nonrepresented_chord_share_extension_projection() {
        let radius = (Real::one() / Real::from(100_i16)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let selected_parameter = selected_inverse_square_parameter(2, &policy);
            let diagonal = RationalBezier2::try_new(
                vec![Point2::from_values(0, 0), Point2::from_values(1, 1)],
                vec![Real::one(); 2],
            )
            .unwrap();
            let corner = crate::tests::decided(
                crate::rational_bezier_general::exact_contact_point_evidence(
                    &diagonal,
                    &selected_parameter,
                    &policy,
                )
                .unwrap(),
            );
            let translated = |point: &CurvePoint2, x, y| {
                match crate::BezierAlgebraicChord2::translated_endpoint(
                    point,
                    &Real::from(x),
                    &Real::from(y),
                    &policy,
                )
                .unwrap()
                {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        panic!("the selected translation must decide: {reason:?}")
                    }
                }
            };
            let previous = crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
                translated(&corner, 0, -4),
                corner,
                crate::bezier_offset::BezierAlgebraicChordAxisDirection2::PositiveY,
                &policy,
            );
            let half = (Real::one() / Real::from(2_i8)).unwrap();
            let selected_coordinate = half.sqrt().unwrap();
            let arc_start = Point2::new(selected_coordinate.clone(), selected_coordinate.clone());
            let arc_end = Point2::new(
                &selected_coordinate + Real::one(),
                &selected_coordinate + Real::one(),
            );
            let arc_center = Point2::new(
                selected_coordinate.clone(),
                &selected_coordinate + Real::one(),
            );
            let arc = CircularArc2::try_from_center(arc_start, arc_end.clone(), arc_center, false)
                .expect("the native quarter circle is valid");
            let closing = match crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(arc_end),
                previous.start().clone(),
                &policy,
            )
            .unwrap()
            {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    panic!("the closing selected chord must decide: {reason:?}")
                }
            };
            assert!(closing.exact_line().is_none());
            assert!(closing.strict_provenance_support_line(&policy).is_none());
            assert!(closing.certified_unit_tangent().is_none());

            for reversed in [false, true] {
                let constraint_reversed_closing = closing.reversed();
                let constraint_reversed_arc = arc.reversed();
                let solve = |mode| {
                    if reversed {
                        solve_exact_fillet_corner(
                            ExactCornerCarrier2::AlgebraicChord(&constraint_reversed_closing),
                            ExactCornerCarrier2::Arc(&constraint_reversed_arc),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::RationalBezier,
                            CurveFamily2::CircularArc,
                            None,
                            &policy,
                        )
                    } else {
                        solve_exact_fillet_corner(
                            ExactCornerCarrier2::Arc(&arc),
                            ExactCornerCarrier2::AlgebraicChord(&closing),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::CircularArc,
                            CurveFamily2::RationalBezier,
                            None,
                            &policy,
                        )
                    }
                };
                let trim = solve(CurveCornerMode2::TrimOnly)
                    .expect("the native arc/chord trim solve must complete");
                let extended = solve(CurveCornerMode2::TrimOrExtend)
                    .expect("the native arc/chord extension solve must complete");
                assert!(!trim.solutions().is_empty());
                assert!(extended.solutions().len() > trim.solutions().len());
                let retains_recursive_parameter = |corner: &FilletCorner2| {
                    corner
                        .retained_frame
                        .as_ref()
                        .and_then(|frame| frame.anchor_evidence.as_ref())
                        .and_then(|evidence| evidence.deferred_arc_contact.as_ref())
                        .and_then(|deferred| deferred.contact_seed.as_ref())
                        .is_some_and(|seed| seed.parameter.as_recursive_projective().is_some())
                };
                assert!(extended.solutions().iter().any(retains_recursive_parameter));
            }
        }
    }

    #[test]
    fn represented_rational_corner_extension_materializes_before_its_pole() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let source = Curve2::from(
            RationalBezier2::try_new(
                vec![
                    Point2::from_values(0, 0),
                    Point2::new(half, Real::zero()),
                    Point2::from_values(1, 1),
                ],
                vec![
                    Real::one(),
                    (Real::one() / Real::from(2_i8)).unwrap(),
                    quarter,
                ],
            )
            .unwrap(),
        );
        let parameter = (Real::from(3_i8) / Real::from(2_i8)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let extended = materialize_corner_cut(
                &source,
                &CornerCut2 {
                    parameter: exact_corner_parameter(parameter.clone()),
                    point: Point2::from_values(3, 9).into(),
                    placement: CornerPlacement2::Extension,
                },
                true,
                CurveOperation2::Chamfer,
                &policy,
            )
            .expect("the represented pre-pole interval must materialize");
            assert_eq!(
                extended.start(),
                crate::CurvePoint2::from(Point2::from_values(0, 0).clone())
            );
            assert_eq!(
                extended.end(),
                crate::CurvePoint2::from(Point2::from_values(3, 9).clone())
            );
        }
    }

    #[test]
    fn rational_chamfer_solver_retains_pre_pole_extension() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let previous = Curve2::from(
            RationalBezier2::try_new(
                vec![
                    Point2::from_values(0, 0),
                    Point2::new(half.clone(), Real::zero()),
                    Point2::from_values(1, 1),
                ],
                vec![Real::one(), half, quarter],
            )
            .unwrap(),
        );
        let next = Curve2::from(
            LineSeg2::try_new(Point2::from_values(1, 1), Point2::from_values(1, 12)).unwrap(),
        );
        let setback = Real::from(68_i8).sqrt().unwrap();
        let expected_parameter = (Real::from(3_i8) / Real::from(2_i8)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let previous_carrier =
                exact_corner_carrier(&previous, true, CurveOperation2::Chamfer, &policy)
                    .unwrap()
                    .unwrap();
            let next_carrier =
                exact_corner_carrier(&next, false, CurveOperation2::Chamfer, &policy)
                    .unwrap()
                    .unwrap();
            let solution = solve_exact_chamfer_corner(
                previous_carrier,
                next_carrier,
                &setback,
                &Real::zero(),
                RealSign::Positive,
                RealSign::Zero,
                CurveCornerMode2::TrimOrExtend,
                false,
                false,
                previous.family(),
                next.family(),
                &policy,
            )
            .expect("the shared solver must retain the pre-pole rational contact");
            let CurveCornerSolutions2::Unique(solution) = solution else {
                panic!("the pre-pole rational contact must be unique");
            };
            assert_eq!(
                solution.previous.exact_parameter(),
                Some(&expected_parameter)
            );
            assert_eq!(
                solution.previous.exact_point(),
                Some(&Point2::from_values(3, 9))
            );
        }
    }

    #[test]
    fn parallel_fillet_frame_retains_selected_normal_and_radial_distance() {
        let source = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::from_values(2, 1),
        );
        let authored = Curve2::from(source.clone());
        let support = source.parallel_left(Real::from(2_i8)).unwrap();
        let center_parameter = BezierParameter2::Exact(
            (Real::one() / Real::from(2_i8)).expect("one half is represented"),
        );
        let parameter = CurveParameter2::from(center_parameter.clone());
        let carrier = FilletOffsetCarrier2::Parallel {
            source: FilletParallelSource2::Direct(ExactCornerBezier2::Direct(&authored)),
            support: support.clone(),
        };
        let frame = carrier
            .retained_fillet_frame(
                true,
                Some(&parameter),
                Some(RetainedFilletAnchorEvidence2 {
                    cross: Some(RealSign::Positive),
                    dot: Some(RealSign::Zero),
                    center_parallel: None,
                    source_direction: None,
                    canonical_anchor_curve: None,
                    deferred_arc_contact: None,
                }),
                false,
                CurveFamily2::QuadraticBezier,
                &CurveContext::STRICT,
            )
            .unwrap()
            .expect("a general parallel retains one radial frame");
        assert_eq!(frame.radial_distance, Real::from(-2_i8));
        assert_eq!(
            frame.anchor_evidence.as_ref().and_then(|value| value.cross),
            Some(RealSign::Positive)
        );
        match frame.radial_frame {
            RetainedFilletRadialFrame2::ParallelNormal {
                center_support,
                center_parameter: retained_parameter,
                policy,
            } => {
                assert_eq!(center_support, support);
                assert_eq!(retained_parameter, center_parameter.into());
                assert_eq!(policy, CurveContext::STRICT);
            }
            other => panic!("expected a selected parallel-normal frame, got {other:?}"),
        }
    }

    #[test]
    fn resource_blocked_selected_parallel_parameter_retains_local_fillet_frame() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let selected = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
                half.clone(),
                32_768,
                &policy,
            );
            assert!(matches!(
                selected.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
            // In Bernstein form `t^15` has only its last coefficient.
            // Therefore this degree-15 exact line evaluates to
            // `(3/5 + 32768*t^15, 4/5)`.  Its selected contact with the unit
            // circle about `(alpha, 0)` is transverse and has global
            // parameter degree 135.
            let three_fifths = (Real::from(3_i8) / Real::from(5_i8)).unwrap();
            let four_fifths = (Real::from(4_i8) / Real::from(5_i8)).unwrap();
            let mut control_points =
                vec![Point2::new(three_fifths.clone(), four_fifths.clone()); 15];
            control_points.push(Point2::new(
                &three_fifths + Real::from(32_768_i32),
                four_fifths,
            ));
            let target = RationalBezier2::try_new(control_points, vec![Real::one(); 16])
                .expect("the degree-15 selected-contact carrier is rational");
            let parallel = target.parallel_left(Real::zero()).unwrap();
            let authored = Curve2::from(target);
            let analytic = FilletOffsetCarrier2::Parallel {
                source: FilletParallelSource2::Direct(ExactCornerBezier2::Direct(&authored)),
                support: parallel.clone(),
            };
            let parameter = CurveParameter2::from_selected_fiber(selected);
            let frame = analytic
                .retained_fillet_frame(
                    true,
                    Some(&parameter),
                    Some(RetainedFilletAnchorEvidence2 {
                        cross: Some(RealSign::Negative),
                        dot: Some(RealSign::Positive),
                        center_parallel: Some(RetainedFilletCenterParallel2 {
                            support: parallel,
                            parameter: Some(parameter.clone()),
                        }),
                        source_direction: Some(RealSign::Positive),
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    }),
                    false,
                    CurveFamily2::QuadraticBezier,
                    &policy,
                )
                .unwrap()
                .expect("the selected fillet retains one exact radial frame");
            let RetainedFilletRadialFrame2::ParallelNormal {
                center_parameter, ..
            } = &frame.radial_frame
            else {
                panic!("a local root retains the same parallel-normal frame as a native root")
            };
            assert!(
                center_parameter == &parameter,
                "the original selected center must survive"
            );
            let retained = frame
                .anchor_evidence
                .as_ref()
                .and_then(|evidence| evidence.center_parallel.as_ref())
                .and_then(|center| center.parameter.as_ref())
                .and_then(CurveParameter2::as_selected_fiber)
                .expect("the fillet frame retains its local selected parameter");
            assert!(matches!(
                retained.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
        }
    }

    #[test]
    fn selected_parallel_fillet_clips_a_positive_dimensional_center_component_locally() {
        let line = |height: i8| {
            QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(
                    Point2::new(Real::zero(), Real::from(height)),
                    Point2::new(Real::one(), Real::from(height)),
                )
                .unwrap(),
            )
        };
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let selected = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
                half.clone(),
                32_768,
                &policy,
            );
            assert!(matches!(
                selected.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
            let range = crate::CurveParameterRange2::new_validated(
                CurveParameter2::from(BezierParameter2::Exact(Real::zero())),
                CurveParameter2::from_selected_fiber(selected.clone()),
            );
            let source_fragment = |height: i8| {
                let source = line(height).parallel_left(Real::zero()).unwrap();
                let end =
                    CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                        source.clone(),
                        selected.clone(),
                        &policy,
                    ));
                (
                    crate::bezier_split::BezierSelectedFiberFragment2::new(
                        crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(
                            source.clone(),
                        ),
                        range.clone(),
                        CurvePoint2::from(Point2::new(Real::zero(), Real::from(height))),
                        end,
                    ),
                    source,
                )
            };
            let (first_source, first_parallel) = source_fragment(0);
            let (second_source, second_parallel) = source_fragment(2);
            let first_support = first_parallel.with_distance(Real::one());
            let second_support = second_parallel.with_distance(Real::from(-1_i8));
            let previous = FilletOffsetCarrier2::Parallel {
                source: FilletParallelSource2::Selected(&first_source),
                support: first_support,
            };
            let next = FilletOffsetCarrier2::Parallel {
                source: FilletParallelSource2::Selected(&second_source),
                support: second_support,
            };

            let centers = fillet_offset_centers(
                &previous,
                &next,
                [FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOnly); 2],
                CurveFamily2::QuadraticBezier,
                CurveFamily2::QuadraticBezier,
                None,
                &policy,
            )
            .expect("a selected positive-dimensional center component must clip locally");
            assert!(!centers.components.is_empty());
            for parameter in [range.start(), range.end()] {
                assert!(centers.components.iter().any(|component| {
                    component
                        .contains_pair(parameter, parameter, &policy)
                        .unwrap()
                        == Classification::Decided(true)
                }));
            }
            assert!(centers.iter().next().is_none());
        }
    }

    #[test]
    fn selected_cusp_rational_contact_retains_mapped_point() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        // P(t)=(t^2, t^3-t/2) keeps the center parameter and radial frame
        // genuinely algebraic while P(sqrt(1/2))=(1/2, 0). The short x-axis
        // cutter therefore has one represented fiber root at the leftmost
        // circle point without collapsing the selected frame itself.
        let one_third = (Real::one() / Real::from(3_i8)).unwrap();
        let one_sixth = (Real::one() / Real::from(6_i8)).unwrap();
        let support = CubicBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(Real::zero(), -one_sixth),
            Point2::new(one_third.clone(), -one_third),
            Point2::new(Real::one(), half.clone()),
        )
        .parallel_left(Real::zero())
        .unwrap();
        let rational = RationalBezier2::try_new(
            vec![Point2::from_values(-1, 0), Point2::from_values(0, 0)],
            vec![Real::one(); 2],
        )
        .unwrap();

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let center_parameter = selected_inverse_square_parameter(2, &policy);
            let Classification::Decided(Some(circle)) =
                crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                    support.clone(),
                    center_parameter.into(),
                    Real::one(),
                    false,
                    &policy,
                )
                .unwrap()
            else {
                panic!("the selected circle must construct");
            };

            let Classification::Decided((
                crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber { contacts, overlaps: unexpected_overlaps },
                _,
            )) = circle
                .rational_intersections_with_parameter_map(&rational, &crate::CurveParameterRange2::unit(), &policy)
                .expect("the selected-circle kernel must retain the rational contact locally")
            else {
                panic!("the selected half must publish its local rational-contact fiber");
            };
            assert!(unexpected_overlaps.is_empty(), "unexpected overlaps");
            let [contact] = contacts.as_slice() else {
                panic!("the selected half must retain exactly its left-axis contact");
            };
            let parameter = contact.other_parameter();
            // This small fixture can project globally, but the fillet path has
            // no reason to pay for it. Nearby degree-135 fixtures prove that
            // imposing the same projection universally is also incomplete.
            assert!(matches!(
                parameter.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Decided(_)
            ));
            assert!(matches!(
                contact.point_evidence(),
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            ));
            let Classification::Decided(bounds) =
                crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(
                    &contact.point_evidence(),
                    8,
                    &policy,
                )
            else {
                panic!("the retained selected contact point must refine without promotion");
            };
            let expected = Point2::new(-half.clone(), Real::zero());
            assert_eq!(bounds.min(), &expected);
            assert_eq!(bounds.max(), &expected);
        }
    }

    #[test]
    fn resource_blocked_selected_parameter_has_exact_cold_promotion() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let selected = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
                half.clone(),
                32_768,
                &policy,
            );
            assert!(matches!(
                selected.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
            let Classification::Decided(promoted) = selected
                .promoted_bezier_parameter_complete(&policy)
                .unwrap()
            else {
                panic!("the cold exact resultant must remove only the scheduling cap");
            };
            assert_eq!(
                selected.cmp_bezier_parameter(&promoted, &policy).unwrap(),
                Classification::Decided(std::cmp::Ordering::Equal)
            );
        }
    }

    fn rationalizable_selected_semicircle(
        policy: &CurveContext,
    ) -> crate::bezier_offset::BezierAlgebraicCuspSemicircle2 {
        let polynomial = crate::BezierParameterPolynomial::try_new_power_basis(
            vec![Real::from(-1_i8), Real::zero(), Real::from(2_i8)],
            policy,
        )
        .expect("the selected quadratic is valid");
        let Classification::Decided(polynomial) = polynomial else {
            panic!("the selected quadratic must be decided");
        };
        let interval = crate::BezierParameterInterval::try_new(
            (Real::from(2_i8) / Real::from(3_i8)).unwrap(),
            (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
            policy,
        )
        .expect("the selected interval is valid");
        let Classification::Decided(interval) = interval else {
            panic!("the selected interval must be decided");
        };
        let parameter = crate::BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)
            .expect("the selected root is isolated");
        let Classification::Decided(parameter) = parameter else {
            panic!("the selected root must be decided");
        };
        let center_source = RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::one(), Real::zero()),
            ],
            vec![Real::one(), Real::one(), Real::one()],
        )
        .expect("the selected center source is valid");
        let center = CurvePoint2::from(crate::tests::decided(
            center_source
                .point_at_algebraic_parameter(&parameter, policy)
                .expect("the selected center retains its exact image"),
        ));
        let support =
            crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_axis_aligned_center(
                &center,
                (1, 0),
                Real::one(),
                true,
                policy,
            )
            .expect("the selected center defines a semicircle");
        let Classification::Decided(Some(support)) = support else {
            panic!("the nonzero selected semicircle must be decided");
        };
        support
    }

    #[test]
    fn retained_parallel_and_direct_bezier_share_projective_fillet_extension() {
        let first = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::from_values(2, 0),
        );
        let second = QuadraticBezier2::new(
            Point2::from_values(2, 0),
            Point2::from_values(2, 1),
            Point2::from_values(2, 2),
        );
        let first_curve = Curve2::from(first.clone());
        let second_curve = Curve2::from(second.clone());
        let reversed_first_curve = Curve2::from(first.reversed_with_retained_provenance().unwrap());
        let reversed_second_curve =
            Curve2::from(second.reversed_with_retained_provenance().unwrap());
        let retained = crate::BezierParallelFragment2::from_certified_range(
            first.parallel_left(Real::zero()).unwrap(),
            BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            ),
            false,
        );
        let radius = (Real::one() / Real::from(2_i8)).unwrap();

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let constraint_reversed_retained = retained.reversed();
                let solve = |retained_source: bool, mode| {
                    match (retained_source, reversed) {
                        (false, false) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::Bezier(&first_curve),
                            ExactCornerCarrier2::Bezier(&second_curve),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::QuadraticBezier,
                            None,
                            &policy,
                        ),
                        (false, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::Bezier(&reversed_second_curve),
                            ExactCornerCarrier2::Bezier(&reversed_first_curve),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::QuadraticBezier,
                            None,
                            &policy,
                        ),
                        (true, false) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::AnalyticParallel(&retained),
                            ExactCornerCarrier2::Bezier(&second_curve),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::QuadraticBezier,
                            None,
                            &policy,
                        ),
                        (true, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::Bezier(&reversed_second_curve),
                            ExactCornerCarrier2::AnalyticParallel(&constraint_reversed_retained),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::QuadraticBezier,
                            None,
                            &policy,
                        ),
                    }
                    .expect("the shared parallel-pair extension kernel must decide")
                };
                let direct_extension = solve(false, CurveCornerMode2::TrimOrExtend);
                let retained_extension = solve(true, CurveCornerMode2::TrimOrExtend);
                assert_eq!(
                    retained_extension.solutions().len(),
                    direct_extension.solutions().len()
                );
                assert!(!direct_extension.solutions().is_empty());
            }
        }
    }

    #[test]
    fn selected_circle_and_direct_bezier_share_projective_fillet_extension() {
        let source = QuadraticBezier2::new(
            Point2::new((-Real::one() / Real::from(2_i8)).unwrap(), Real::zero()),
            Point2::new(
                (-Real::from(3_i8) / Real::from(2_i8)).unwrap(),
                Real::zero(),
            ),
            Point2::new(
                (-Real::from(5_i8) / Real::from(2_i8)).unwrap(),
                Real::zero(),
            ),
        );
        let direct = Curve2::from(source.clone());
        let reversed_direct = Curve2::from(source.reversed_with_retained_provenance().unwrap());
        let retained = crate::BezierParallelFragment2::from_certified_range(
            source.parallel_left(Real::zero()).unwrap(),
            BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            ),
            false,
        );
        let radius = (Real::one() / Real::from(4_i8)).unwrap();

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let semicircle = rationalizable_selected_semicircle(&policy);
            let fragment = match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                semicircle,
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one()),
                false,
                &policy,
            )
            .unwrap()
            {
                Classification::Decided(fragment) => fragment,
                Classification::Uncertain(reason) => {
                    panic!("the selected source half must decide: {reason:?}")
                }
            };
            for reversed in [false, true] {
                let constraint_reversed_fragment = fragment.reversed();
                let constraint_reversed_retained = retained.reversed();
                let solve = |retained_source: bool, mode| {
                    match (retained_source, reversed) {
                        (false, false) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::AlgebraicCusp(&fragment),
                            ExactCornerCarrier2::Bezier(&direct),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::RationalBezier,
                            CurveFamily2::QuadraticBezier,
                            None,
                            &policy,
                        ),
                        (false, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::Bezier(&reversed_direct),
                            ExactCornerCarrier2::AlgebraicCusp(&constraint_reversed_fragment),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::RationalBezier,
                            None,
                            &policy,
                        ),
                        (true, false) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::AlgebraicCusp(&fragment),
                            ExactCornerCarrier2::AnalyticParallel(&retained),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::RationalBezier,
                            CurveFamily2::QuadraticBezier,
                            None,
                            &policy,
                        ),
                        (true, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::AnalyticParallel(&constraint_reversed_retained),
                            ExactCornerCarrier2::AlgebraicCusp(&constraint_reversed_fragment),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::RationalBezier,
                            None,
                            &policy,
                        ),
                    }
                    .expect("the selected-circle/parallel extension kernel must decide")
                };
                let direct_extension = solve(false, CurveCornerMode2::TrimOrExtend);
                let retained_extension = solve(true, CurveCornerMode2::TrimOrExtend);
                assert_eq!(
                    direct_extension.solutions().len(),
                    retained_extension.solutions().len()
                );
                assert!(!direct_extension.solutions().is_empty());
            }
        }
    }

    #[test]
    fn selected_circle_and_native_line_share_projective_fillet_extension() {
        let start = Point2::new((-Real::one() / Real::from(2_i8)).unwrap(), Real::zero());
        let end = Point2::new(
            (-Real::from(5_i8) / Real::from(2_i8)).unwrap(),
            Real::zero(),
        );
        let line = LineSeg2::try_new(start.clone(), end.clone()).unwrap();
        let reversed_line = line.reversed();
        let direct = Curve2::from(QuadraticBezier2::new(
            start,
            Point2::new(
                (-Real::from(3_i8) / Real::from(2_i8)).unwrap(),
                Real::zero(),
            ),
            end,
        ));
        let reversed_direct = Curve2::from(match direct.geometry() {
            Some(CurveGeometry2::QuadraticBezier(source)) => {
                source.reversed_with_retained_provenance().unwrap()
            }
            _ => unreachable!(),
        });
        let radius = (Real::one() / Real::from(4_i8)).unwrap();

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let semicircle = rationalizable_selected_semicircle(&policy);
            let fragment = match crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                semicircle,
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one()),
                false,
                &policy,
            )
            .unwrap()
            {
                Classification::Decided(fragment) => fragment,
                Classification::Uncertain(reason) => {
                    panic!("the selected source half must decide: {reason:?}")
                }
            };
            for reversed in [false, true] {
                let constraint_reversed_fragment = fragment.reversed();
                let solve = |native: bool, mode| {
                    match (native, reversed) {
                        (true, false) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::AlgebraicCusp(&fragment),
                            ExactCornerCarrier2::Line(&line),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::RationalBezier,
                            CurveFamily2::Line,
                            None,
                            &policy,
                        ),
                        (true, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::Line(&reversed_line),
                            ExactCornerCarrier2::AlgebraicCusp(&constraint_reversed_fragment),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::Line,
                            CurveFamily2::RationalBezier,
                            None,
                            &policy,
                        ),
                        (false, false) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::AlgebraicCusp(&fragment),
                            ExactCornerCarrier2::Bezier(&direct),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::RationalBezier,
                            CurveFamily2::QuadraticBezier,
                            None,
                            &policy,
                        ),
                        (false, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::Bezier(&reversed_direct),
                            ExactCornerCarrier2::AlgebraicCusp(&constraint_reversed_fragment),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::RationalBezier,
                            None,
                            &policy,
                        ),
                    }
                    .unwrap_or_else(|error| {
                        panic!(
                            "the selected-circle/line extension kernel must decide: policy={policy:?}, reversed={reversed}, native={native}, mode={mode:?}, error={error:?}"
                        )
                    })
                };
                let direct_trim = solve(false, CurveCornerMode2::TrimOnly);
                let direct_extension = solve(false, CurveCornerMode2::TrimOrExtend);
                let native_trim = solve(true, CurveCornerMode2::TrimOnly);
                let native_extension = solve(true, CurveCornerMode2::TrimOrExtend);
                assert_eq!(native_trim.solutions().len(), direct_trim.solutions().len());
                assert_eq!(
                    native_extension.solutions().len(),
                    direct_extension.solutions().len(),
                    "the native fast path must enumerate both selected-circle charts",
                );
                assert!(native_extension.solutions().len() > native_trim.solutions().len());
            }
        }
    }

    #[test]
    #[cfg(target_pointer_width = "64")]
    fn curve_path_carrier_keeps_compact_policy_aware_boundary_storage() {
        assert_eq!(core::mem::size_of::<CurvePath2>(), 8);
        assert_eq!(core::mem::size_of::<ExactCornerCarrier2<'_>>(), 16);
    }

    #[test]
    fn selected_circular_fillet_overlap_is_clipped_to_the_finite_fragment() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let support = rationalizable_selected_semicircle(&policy);
            let intersections = support
                .pair_intersections(&support, &policy)
                .expect("an identical selected support must intersect exactly");
            let Classification::Decided(
                crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(
                    overlap,
                ),
            ) = intersections
            else {
                panic!("an identical selected support must publish one pair overlap");
            };
            let eighth = (Real::one() / Real::from(8_i8)).unwrap();
            let quarter = (Real::one() / Real::from(4_i8)).unwrap();
            let half = (Real::one() / Real::from(2_i8)).unwrap();
            let three_quarters = &half + &quarter;
            let fragment = |start: Real, end: Real| {
                let fragment = crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                    support.clone(),
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(start),
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(end),
                    false,
                    &policy,
                )
                .expect("the selected fragment range is valid");
                let Classification::Decided(fragment) = fragment else {
                    panic!("the selected fragment must be decided");
                };
                fragment
            };
            let first = fragment(Real::zero(), quarter.clone());
            let disjoint = fragment(half, three_quarters);
            assert!(
                !retained_fillet_cusp_pair_overlap_is_positive(
                    &first,
                    &disjoint,
                    &overlap,
                    CurveFamily2::RationalBezier,
                    &policy,
                )
                .expect("disjoint finite overlap clipping must classify")
            );

            let overlapping = fragment(eighth.clone(), &quarter + &eighth);
            assert!(
                retained_fillet_cusp_pair_overlap_is_positive(
                    &first,
                    &overlapping,
                    &overlap,
                    CurveFamily2::RationalBezier,
                    &policy,
                )
                .expect("positive finite overlap clipping must classify")
            );
        }
    }

    #[test]
    fn retained_arc_selected_circle_overlap_clips_both_authored_sweeps() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let support = rationalizable_selected_semicircle(&policy);
            let center = support
                .center_point_image(&policy)
                .unwrap()
                .exact_point(&CurveContext::STRICT)
                .expect("the fixture center is exactly rational");
            let start = support
                .start_point_image(&policy)
                .unwrap()
                .exact_point(&CurveContext::STRICT)
                .expect("the fixture start is exactly rational");
            let end = support
                .end_point_image(&policy)
                .unwrap()
                .exact_point(&CurveContext::STRICT)
                .expect("the fixture end is exactly rational");
            let authored = CircularArc2::try_from_center(
                start.clone(),
                end.clone(),
                center.clone(),
                support.is_clockwise(),
            )
            .expect("the authored selected half has one native arc chart");
            let complementary =
                CircularArc2::try_from_center(end, start, center, support.is_clockwise())
                    .expect("the opposite half has one native arc chart");
            let eighth = (Real::one() / Real::from(8_i8)).unwrap();
            let three_eighths = (Real::from(3_i8) / Real::from(8_i8)).unwrap();
            let fragment = crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                support,
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(eighth),
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(three_eighths),
                false,
                &policy,
            )
            .expect("the selected finite range is valid");
            let Classification::Decided(fragment) = fragment else {
                panic!("the selected finite range must decide");
            };
            assert!(
                retained_fillet_arc_cusp_overlap_is_positive(
                    &authored,
                    &fragment,
                    CurveFamily2::CircularArc,
                    CurveFamily2::RationalBezier,
                    &policy,
                )
                .expect("the shared authored sweep must classify")
            );
            assert!(
                !retained_fillet_arc_cusp_overlap_is_positive(
                    &complementary,
                    &fragment,
                    CurveFamily2::CircularArc,
                    CurveFamily2::RationalBezier,
                    &policy,
                )
                .expect("the disjoint authored sweeps must classify")
            );
        }
    }

    #[test]
    fn retained_arc_selected_circle_endpoint_contact_uses_exact_fallback_frame() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let one = Real::one();
            let arc = CircularArc2::try_from_center(
                Point2::from_values(1, 0),
                Point2::from_values(0, 1),
                Point2::from_values(0, 0),
                false,
            )
            .expect("the source quarter circle is valid");
            let source_arc = ExactCornerArc2::Native(arc.clone());
            let arc_carrier = FilletOffsetCarrier2::Arc {
                source: &source_arc,
                source_radius: &one,
                signed_radius: one.clone(),
            };
            let cusp_center = Point2::from_values(2, 0);
            let cusp_axis = QuadraticBezier2::new(
                cusp_center.clone(),
                cusp_center.translated(Real::zero(), Real::from(-1_i8)),
                cusp_center.translated(Real::zero(), Real::from(-2_i8)),
            );
            let cusp_circle = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                cusp_axis.parallel_left(Real::zero()).unwrap(),
                BezierParameter2::Exact(Real::zero()).into(),
                one.clone(),
                false,
                &policy,
            )
            .unwrap() {
                Classification::Decided(Some(circle)) => circle,
                other => panic!("the exact selected circle must construct: {other:?}"),
            };
            let cusp_source =
                crate::BezierAlgebraicCuspSemicircleFragment2::full(cusp_circle.clone(), &policy);
            let cusp_carrier = FilletOffsetCarrier2::AlgebraicCusp {
                source: &cusp_source,
                support: cusp_source.clone(),
            };
            let centers = fillet_offset_centers(
                &arc_carrier,
                &cusp_carrier,
                [FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOrExtend); 2],
                CurveFamily2::CircularArc,
                CurveFamily2::RationalBezier,
                None,
                &policy,
            )
            .expect("the endpoint-only circle pair must solve exactly");
            assert!(centers.coincident.is_none());
            let mut retained = centers.iter();
            let center = retained
                .next()
                .expect("the externally tangent circles must have one center contact");
            assert!(
                retained.next().is_none(),
                "the externally tangent circles must have only one center contact"
            );
            let expected = Point2::from_values(1, 0);
            assert_eq!(center.point.coordinates(), Some(&expected));
            let evidence = center
                .retained_anchor_evidence
                .clone()
                .expect("the endpoint contact must retain its tangent relation");
            assert_eq!(evidence.cross, Some(RealSign::Zero));
            assert!(evidence.dot.is_some());
            assert!(
                evidence
                    .deferred_arc_contact
                    .as_ref()
                    .is_some_and(|deferred| deferred.selected_center.is_none()),
                "an exact diameter endpoint must not masquerade as a mapped pair center",
            );
            let frame = arc_carrier
                .retained_fillet_frame(
                    true,
                    None,
                    Some(evidence),
                    false,
                    CurveFamily2::CircularArc,
                    &policy,
                )
                .expect("the exact endpoint frame must construct")
                .expect("the arc must retain a reconstruction frame");
            assert!(matches!(
                frame.radial_frame,
                RetainedFilletRadialFrame2::ConcentricArc { .. }
            ));
        }
    }

    #[test]
    fn boundary_cache_revalidates_approximate_internal_path_joins() {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let unresolved_zero = &sine * &sine + &cosine * &cosine - Real::one();
        let left_x = Real::from(4_i8);
        let right_x = left_x.clone() + unresolved_zero;
        let lower_left = Point2::new(Real::zero(), Real::zero());
        let upper_left = Point2::new(Real::zero(), Real::from(2_i8));
        let lower_right_left_form = Point2::new(left_x, Real::zero());
        let lower_right_right_form = Point2::new(right_x.clone(), Real::zero());
        let upper_right = Point2::new(right_x, Real::from(2_i8));
        let curves = vec![
            Curve2::from(LineSeg2::try_new(lower_left.clone(), lower_right_left_form).unwrap()),
            Curve2::from(LineSeg2::try_new(lower_right_right_form, upper_right.clone()).unwrap()),
            Curve2::from(LineSeg2::try_new(upper_right, upper_left.clone()).unwrap()),
            Curve2::from(LineSeg2::try_new(upper_left, lower_left).unwrap()),
        ];
        let constructed = resolve_certified_operation(&CurveContext::APPROXIMATE_512, |attempt| {
            CurvePath2::try_new_raw(curves, attempt)
        })
        .expect("the terminal policy must construct the symbolically connected path");
        assert_eq!(
            constructed.certainty,
            crate::CurveCertainty::Approximate512Consumed
        );
        let path = constructed.value;

        let boundary = path
            .boundary_loop_with_policy(&CurveContext::APPROXIMATE_512)
            .expect("the terminal policy must validate every path join");
        assert_eq!(
            boundary.certainty,
            crate::CurveCertainty::Approximate512Consumed
        );
        assert_eq!(boundary.value.len(), 4);

        let strict_boundary = path
            .boundary_loop_with_policy(&CurveContext::STRICT)
            .unwrap_err();
        assert!(matches!(
            strict_boundary,
            ExactCurveError::Blocked(blocker)
                if blocker.operation() == CurveOperation2::Arrangement
                    && blocker.reason() == crate::UncertaintyReason::RealSign
        ));

        let strict_region = crate::CurveRegion2::try_from_boundary_paths_with_policy(
            std::slice::from_ref(&path),
            crate::FillRule::EvenOdd,
            &CurveContext::STRICT,
        )
        .unwrap_err();
        assert!(matches!(
            strict_region,
            ExactCurveError::Blocked(blocker)
                if blocker.operation() == CurveOperation2::Construction
                    && blocker.reason() == crate::UncertaintyReason::RealSign
        ));
        let approximate_region = crate::CurveRegion2::try_from_boundary_paths_with_policy(
            std::slice::from_ref(&path),
            crate::FillRule::EvenOdd,
            &CurveContext::APPROXIMATE_512,
        )
        .expect("region construction must revalidate the terminal internal join");
        assert_eq!(
            approximate_region.certainty,
            crate::CurveCertainty::Approximate512Consumed
        );

        let coordinates = Point2::from_values(1, 1);
        let endpoint = CurvePoint2::from_endpoint(
            std::sync::Arc::new(crate::BezierSplitFragment2::Materialized {
                start: crate::BezierParameter2::Exact(Real::zero()),
                end: crate::BezierParameter2::Exact(Real::one()),
                curve: crate::BezierSubcurve2::Quadratic(crate::QuadraticBezier2::new(
                    coordinates.clone(),
                    Point2::from_values(2, 1),
                    Point2::from_values(3, 1),
                )),
            }),
            true,
        );
        assert!(endpoint.coordinates().is_none());
        for point in [coordinates.into(), endpoint] {
            let approximate = path
                .classify_point_with_policy(&point, &CurveContext::APPROXIMATE_512)
                .expect("the terminal policy must classify through the retained boundary");
            assert_eq!(
                approximate.certainty,
                crate::CurveCertainty::Approximate512Consumed
            );
            assert_eq!(
                approximate.value,
                Classification::Decided(ContourPointLocation::Inside)
            );

            let strict = path
                .classify_point_with_policy(&point, &CurveContext::STRICT)
                .expect("strict classification preserves uncertainty as query evidence");
            assert_eq!(strict.certainty, crate::CurveCertainty::Certified);
            assert_eq!(
                strict.value,
                Classification::Uncertain(crate::UncertaintyReason::RealSign)
            );
        }
    }
}
