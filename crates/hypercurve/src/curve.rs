//! Top-level exact curve carriers.

#[path = "curve_evaluation.rs"]
mod curve_evaluation;

#[path = "curve_subdivision.rs"]
mod curve_subdivision;
use curve_subdivision::CurveSourceRange2;

#[path = "curve_corner_reconstruction.rs"]
mod curve_corner_reconstruction;
use curve_corner_reconstruction::corner_has_native_reconstruction;

#[path = "curve_corner_domain.rs"]
mod curve_corner_domain;

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
use crate::rational_bezier_general::RationalBezierOverlapParameterCorrespondence2;
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
    /// Every candidate collapses the inserted corner carrier.
    DegenerateCandidate,
}

/// Complete exact solutions for one corner-edit request.
///
/// Candidate order is deterministic. Fillets order the left-side center before
/// the right-side center. Chamfers order trim/trim, trim/extension,
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

/// Exact public parameter interval for one promoted native span.
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
    fn from_bezier(curve: BezierSubcurve2) -> Self {
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

    /// Constructs an exact polynomial B-spline carrier under `policy`.
    pub fn try_polynomial_bspline(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        PolynomialSplineCurve2::try_new(degree, control_points, knots, policy)
            .map(|outcome| outcome.map(|curve| Self::new(CurveGeometry2::PolynomialBSpline(curve))))
    }

    /// Constructs an exact NURBS carrier under `policy`.
    pub fn try_nurbs(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        NurbsCurve2::try_new(degree, control_points, weights, knots, policy)
            .map(|outcome| outcome.map(|curve| Self::new(CurveGeometry2::Nurbs(curve))))
    }

    /// Constructs a periodic polynomial B-spline from one period under `policy`.
    pub fn try_periodic_polynomial_bspline(
        degree: usize,
        control_points: Vec<Point2>,
        period_knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        PolynomialSplineCurve2::try_new_periodic(degree, control_points, period_knots, policy)
            .map(|outcome| outcome.map(|curve| Self::new(CurveGeometry2::PolynomialBSpline(curve))))
    }

    /// Constructs a periodic NURBS from one period under `policy`.
    pub fn try_periodic_nurbs(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        period_knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        NurbsCurve2::try_new_periodic(degree, control_points, weights, period_knots, policy)
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
            | crate::BezierSplitFragment2::AlgebraicEndpointImages {
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
    pub fn reversed(&self, policy: &CurveContext) -> ExactCurveResult<CurveOutcome<Self>> {
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
    pub fn transform_similarity(
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
        let transform_points = |points: &[Point2]| {
            points
                .iter()
                .map(|point| transform.transform_point(point))
                .collect::<Vec<_>>()
        };
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
            Some(CurveGeometry2::RationalBezier(curve)) => CurveGeometry2::RationalBezier(
                RationalBezier2::try_new(
                    transform_points(curve.control_points()),
                    curve.weights().to_vec(),
                )
                .map_err(|cause| self.transform_error(cause))?,
            ),
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
    pub fn split_at(
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
                self.subcurve_with_policy(domain.start().clone(), parameter.clone(), policy)?,
                self.subcurve_with_policy(parameter, domain.end().clone(), policy)?,
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

    pub(crate) fn subcurve_with_policy(
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
                    .point_at_side_with_policy(&start, CurveParameterSide2::Automatic, policy)
                    .map_err(|error| remap_operation(error, CurveOperation2::Subdivision))?;
                let sub_end = self
                    .point_at_side_with_policy(&end, CurveParameterSide2::Automatic, policy)
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
            Some(CurveGeometry2::RationalQuadraticBezier(curve)) => {
                CurveGeometry2::RationalQuadraticBezier(
                    curve
                        .subcurve_between_exact(&start, &end, policy)
                        .map_err(|cause| self.subdivision_error(cause))?,
                )
            }
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
    pub fn point_at(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurvePoint2>> {
        self.point_at_side(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates an exact point with explicit spline-knot side policy.
    pub fn point_at_side(
        &self,
        parameter: &CurveParameter2,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurvePoint2>> {
        resolve_certified_operation(policy, |attempt| {
            self.point_at_parameter_with_policy(parameter, side, attempt)
        })
    }

    pub(crate) fn point_at_side_with_policy(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        match self.geometry() {
            Some(CurveGeometry2::PolynomialBSpline(curve)) => {
                curve.point_at_side_with_policy(parameter, side, policy)
            }
            Some(CurveGeometry2::Nurbs(curve)) => {
                curve.point_at_side_with_policy(parameter, side, policy)
            }
            Some(geometry) => {
                let location = validate_unit_parameter(parameter, geometry.family(), policy)?;
                if let Some(endpoint) = retained_native_endpoint(geometry, location, policy) {
                    return Ok(endpoint);
                }
                match geometry {
                    CurveGeometry2::Line(curve) => Ok(curve.point_at(parameter.clone())),
                    CurveGeometry2::CircularArc(_) => {
                        let fragments = match self.native_bezier_fragments_with_policy(policy)? {
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
    pub fn point_at_wrapped(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        self.point_at_wrapped_side(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates a periodic spline with explicit side selection at wrapped seams.
    pub fn point_at_wrapped_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        resolve_certified_operation(policy, |attempt| {
            self.point_at_wrapped_side_with_policy(parameter, side, attempt)
        })
    }

    pub(crate) fn point_at_wrapped_side_with_policy(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        match self.geometry() {
            Some(CurveGeometry2::PolynomialBSpline(curve)) => {
                curve.point_at_wrapped_side_with_policy(parameter, side, policy)
            }
            Some(CurveGeometry2::Nurbs(curve)) => {
                curve.point_at_wrapped_side_with_policy(parameter, side, policy)
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
    pub fn derivative_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        self.derivative_at_side(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates an exact first derivative with explicit knot-boundary side policy.
    pub fn derivative_at_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        resolve_certified_operation(policy, |attempt| {
            self.derivative_at_side_with_policy(parameter, side, attempt)
        })
    }

    pub(crate) fn derivative_at_side_with_policy(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveDerivative2> {
        let mut derivatives = self.derivatives_at_side_with_policy(parameter, 1, side, policy)?;
        Ok(derivatives.pop().expect("one derivative requested"))
    }

    /// Evaluates the first periodic derivative at any wrappable parameter.
    pub fn derivative_at_wrapped(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        self.derivative_at_wrapped_side(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates the first periodic derivative with explicit seam-side selection.
    pub fn derivative_at_wrapped_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        resolve_certified_operation(policy, |attempt| {
            let mut derivatives =
                self.derivatives_at_wrapped_side_with_policy(parameter, 1, side, attempt)?;
            Ok(derivatives.pop().expect("one derivative requested"))
        })
    }

    /// Evaluates exact derivatives through `max_order` in the public parameter.
    ///
    /// The returned vector stores orders `1..=max_order`. Native curves use
    /// `[0, 1]`; spline curves use their authored knot domain.
    pub fn derivatives_at(
        &self,
        parameter: &Real,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        self.derivatives_at_side(parameter, max_order, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates exact derivatives with explicit retained-fragment side policy.
    pub fn derivatives_at_side(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.derivatives_at_side_with_policy(parameter, max_order, side, attempt)
        })
    }

    pub(crate) fn derivatives_at_side_with_policy(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        match self.geometry() {
            Some(CurveGeometry2::PolynomialBSpline(curve)) => {
                return curve.derivatives_at_side_with_policy(parameter, max_order, side, policy);
            }
            Some(CurveGeometry2::Nurbs(curve)) => {
                return curve.derivatives_at_side_with_policy(parameter, max_order, side, policy);
            }
            _ => {}
        }
        let fragments = match self.native_bezier_fragments_with_policy(policy)? {
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
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        self.derivatives_at_wrapped_side(
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
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.derivatives_at_wrapped_side_with_policy(parameter, max_order, side, attempt)
        })
    }

    pub(crate) fn derivatives_at_wrapped_side_with_policy(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        match self.geometry() {
            Some(CurveGeometry2::PolynomialBSpline(curve)) => {
                curve.derivatives_at_wrapped_side_with_policy(parameter, max_order, side, policy)
            }
            Some(CurveGeometry2::Nurbs(curve)) => {
                curve.derivatives_at_wrapped_side_with_policy(parameter, max_order, side, policy)
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
        let fragments = match self.native_bezier_fragments_with_policy(policy)? {
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
    pub fn native_bezier_fragments(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<&[NativeBezierFragment2]>> {
        resolve_certified_operation(policy, |attempt| {
            self.native_bezier_fragments_for_operation(attempt, CurveOperation2::NativeTopology)
        })
    }

    #[inline]
    pub(crate) fn native_bezier_fragments_with_policy(
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
            .native_bezier_fragments_with_policy(policy)
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
                let fragments = match self.native_bezier_fragments_with_policy(attempt)? {
                    Classification::Decided(fragments) => fragments,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                fragments
                    .iter()
                    .map(|fragment| rationalize_subcurve(fragment.curve(), self.family()))
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
        Self::try_new_raw(curves, &CurveContext::STRICT)
    }

    /// Constructs a nonempty ordered path under the selected endpoint policy.
    ///
    /// The outcome reports when connectivity consumed the authorized 512-bit
    /// terminal. No approximate coordinate replacement is performed.
    pub fn try_new_with_policy(
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
    pub fn reversed(&self, policy: &CurveContext) -> ExactCurveResult<CurveOutcome<Self>> {
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
    pub fn transform_similarity(
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
    /// with rational extensions stopping at the first pole. Candidates are
    /// returned in deterministic order. Reconstruction shares the exact chain
    /// machinery used by regions, with a native scalar specialization where
    /// its coordinates and parameters are already available.
    pub fn chamfer_vertex_by_setbacks(
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
        let previous_retained_arc = previous_carrier.retained_rational_arc_support().cloned();
        let next_retained_arc = next_carrier.retained_rational_arc_support().cloned();
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
            if !corner_has_native_reconstruction(previous, &solution.previous)
                || !corner_has_native_reconstruction(next, &solution.next)
            {
                return self.reconstruct_selected_chamfer(
                    previous_index,
                    next_index,
                    solution,
                    previous_retained_arc.as_ref(),
                    next_retained_arc.as_ref(),
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
    /// two incident curves. `TrimOnly` searches every finite rational chart of
    /// selected source restrictions, polynomial B-splines and NURBS. Authored
    /// endpoints remain open; an internal seam belongs to the side surviving
    /// the trim, including its one-sided tangent. Distinct source locations
    /// remain distinct candidates, and two cuts on one closed authored curve
    /// must leave a nonempty interval.
    ///
    /// Lines, circular arcs and retained exact line/circle images preserve
    /// their direct support kernels. General Bezier pairs use certified
    /// analytic-parallel incidence and retain selected contact evidence for
    /// subsequent operations. Positive-dimensional center/contact families
    /// do not produce an arbitrary isolated fillet.
    ///
    /// `TrimOrExtend` currently prepares the incident native span of spline
    /// and selected source restrictions. Direct Bezier extensions search the
    /// endpoint-adjacent regular cells, stopping at poles or source-speed
    /// zeros. Complete authored-domain enumeration in this mode remains an
    /// implementation limitation. Extended rational circles preserve their
    /// certified support and materialize the extended fragment as an arc.
    pub fn fillet_vertex_by_radius(
        &self,
        vertex_index: usize,
        radius: Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveCornerSolutions2<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            self.fillet_vertex_by_radius_raw(vertex_index, radius, mode, attempt)
        })
    }

    pub(crate) fn fillet_vertex_by_radius_raw(
        &self,
        vertex_index: usize,
        radius: Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveCornerSolutions2<Self>> {
        let (previous_index, next_index) =
            self.corner_curve_indices(vertex_index, CurveOperation2::Fillet, policy)?;
        let previous = &self.data.curves[previous_index];
        let next = &self.data.curves[next_index];
        let radius_sign = validate_corner_design_value(
            &radius,
            CurveOperation2::Fillet,
            previous.family(),
            policy,
        )?;
        if radius_sign == RealSign::Zero {
            return Ok(CurveCornerSolutions2::NoSolution(
                CurveCornerNoSolution2::ZeroDesignValue,
            ));
        }
        if mode == CurveCornerMode2::TrimOnly
            && let Some(solutions) = self.fillets_in_authored_domain(
                vertex_index,
                previous_index,
                next_index,
                &radius,
                policy,
            )?
        {
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
        let previous_retained_arc = previous_carrier.retained_rational_arc_support().cloned();
        let next_retained_arc = next_carrier.retained_rational_arc_support().cloned();
        let solutions = solve_exact_fillet_corner(
            previous_carrier,
            next_carrier,
            &radius,
            radius_sign,
            mode,
            false,
            previous.family(),
            next.family(),
            policy,
        )?;
        let solutions = try_map_corner_solutions(solutions, |mut solution| {
            solution.previous.map_source_parameter(
                previous_source.source_chart(),
                CurveOperation2::Fillet,
                previous.family(),
                policy,
            )?;
            solution.next.map_source_parameter(
                next_source.source_chart(),
                CurveOperation2::Fillet,
                next.family(),
                policy,
            )?;
            self.publish_fillet_corner(
                vertex_index,
                previous_index,
                next_index,
                solution,
                &radius,
                mode,
                [previous_retained_arc.as_ref(), next_retained_arc.as_ref()],
                [
                    previous_source.promoted_parallel(),
                    next_source.promoted_parallel(),
                ],
                policy,
            )
        })?;
        Ok(compact_optional_corner_solutions(solutions))
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
            let policy = crate::CurveContext::STRICT;
            for curve in &self.data.curves[1..] {
                bounds = decided_bounds(bounds.union(curve.bounds()?, &policy), curve.family())?;
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
        point: &Point2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Classification<ContourPointLocation>>> {
        resolve_certified_operation(policy, |attempt| self.classify_point_raw(point, attempt))
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
            .boundary_loop_with_policy(policy)
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
    pub fn native_bezier_fragments(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<&[NativeBezierFragment2]>> {
        resolve_certified_operation(policy, |attempt| {
            match self.native_bezier_fragments_with_policy(attempt)? {
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
    pub(crate) fn native_bezier_fragments_with_policy(
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
                        let native = match curve.native_bezier_fragments_with_policy(attempt)? {
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
                            curve.native_bezier_fragments_with_policy(attempt)?
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
    pub fn boundary_loop(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<&CurveRegionBoundaryLoop2>> {
        resolve_certified_operation(policy, |attempt| {
            match self.boundary_loop_with_policy(attempt)? {
                Classification::Decided(boundary) => Ok(boundary),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::Arrangement,
                    self.data.curves[0].family(),
                    reason,
                )),
            }
        })
    }

    pub(crate) fn boundary_loop_with_policy(
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

impl CurveSpanRange2 {
    /// Returns the exact interval in the top-level curve parameterization.
    pub fn endpoints(&self) -> (&Real, &Real) {
        (&self.start, &self.end)
    }
}

impl NativeBezierFragment2 {
    /// Returns the promoted exact native curve.
    pub const fn curve(&self) -> &BezierSubcurve2 {
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
            bounds = decided_bounds(bounds.union(&next, &policy), curve.family())?;
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
        Some(CurveGeometry2::Line(line)) => {
            decided_bounds(Aabb2::from_line(line, &policy), curve.family())
        }
        Some(CurveGeometry2::CircularArc(arc)) => decided_bounds(
            Aabb2::from_arc(arc, &policy).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::NativeTopology, curve.family(), cause)
            })?,
            curve.family(),
        ),
        _ => {
            let fragments = curve
                .native_bezier_fragments_for_operation(&policy, CurveOperation2::NativeTopology)?;
            let mut bounds =
                decided_subcurve_bounds(fragments[0].curve(), curve.family(), &policy)?;
            for fragment in &fragments[1..] {
                let fragment_bounds =
                    decided_subcurve_bounds(fragment.curve(), curve.family(), &policy)?;
                bounds = decided_bounds(bounds.union(&fragment_bounds, &policy), curve.family())?;
            }
            Ok(bounds)
        }
    }
}

fn decided_subcurve_bounds(
    curve: &BezierSubcurve2,
    family: CurveFamily2,
    policy: &crate::CurveContext,
) -> ExactCurveResult<Aabb2> {
    let bounds = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.control_hull_box(policy),
        BezierSubcurve2::Cubic(curve) => curve.control_hull_box(policy),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(policy),
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
                    .spans()
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
            let subcurves = match value.native_subcurves_with_policy(policy)? {
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
                let BezierSubcurve2::RationalQuadratic(curve) = fragment.curve() else {
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

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum CornerReplacement2 {
    Curve(BezierSubcurve2),
    AnalyticParallel {
        fragment: crate::BezierParallelFragment2,
        /// Maps the replacement parameter back to the authored source:
        /// `source = scale * replacement + offset`.
        source_scale: Real,
        source_offset: Real,
    },
    /// A finite rational envelope whose exact cut boundaries remain selected
    /// in local fibers. The replacement fragment owns the reparameterized
    /// carrier and both endpoint point witnesses, so reconstruction never
    /// needs the degree-multiplied global parameter projections.
    SelectedFiber {
        fragment: Arc<crate::bezier_split::BezierSelectedFiberFragment2>,
        /// Maps the replacement parameter back to the authored source:
        /// `source = scale * replacement + offset`.
        source_scale: Real,
        source_offset: Real,
    },
}

impl CornerReplacement2 {
    pub(crate) const fn as_curve(&self) -> Option<&BezierSubcurve2> {
        match self {
            Self::Curve(curve) => Some(curve),
            Self::AnalyticParallel { .. } | Self::SelectedFiber { .. } => None,
        }
    }

    pub(crate) const fn as_parallel_fragment(&self) -> Option<&crate::BezierParallelFragment2> {
        match self {
            Self::AnalyticParallel { fragment, .. } => Some(fragment),
            Self::Curve(_) | Self::SelectedFiber { .. } => None,
        }
    }

    pub(crate) fn parallel_source_parameter_map(&self) -> Option<(&Real, &Real)> {
        match self {
            Self::AnalyticParallel {
                source_scale,
                source_offset,
                ..
            } => Some((source_scale, source_offset)),
            Self::SelectedFiber {
                source_scale,
                source_offset,
                ..
            } => Some((source_scale, source_offset)),
            Self::Curve(_) => None,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct CornerTrimCut2 {
    pub(crate) parameter: CurveParameter2,
    pub(crate) point: CurvePoint2,
    pub(crate) placement: CornerPlacement2,
    pub(crate) replacement: Option<CornerReplacement2>,
}

impl CornerTrimCut2 {
    pub(crate) fn replacement_curve(&self) -> Option<&BezierSubcurve2> {
        self.replacement
            .as_ref()
            .and_then(CornerReplacement2::as_curve)
    }

    pub(crate) fn replacement_parallel_fragment(&self) -> Option<&crate::BezierParallelFragment2> {
        self.replacement
            .as_ref()
            .and_then(CornerReplacement2::as_parallel_fragment)
    }

    pub(crate) fn replacement_parallel_source_parameter_map(&self) -> Option<(&Real, &Real)> {
        self.replacement
            .as_ref()
            .and_then(CornerReplacement2::parallel_source_parameter_map)
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
        center_parameter: BezierParameter2,
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
    pub(crate) support: CircularArc2,
    pub(crate) source_radius: Real,
    pub(crate) signed_center_radius: Real,
    pub(crate) arc_is_previous: bool,
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
    RetainedRationalArc(Box<RetainedRationalCornerArc2<'a>>),
    Bezier(&'a Curve2),
    NativeBezierSpan(&'a NativeBezierFragment2),
    AlgebraicChord(&'a crate::BezierAlgebraicChord2),
    AnalyticParallel(&'a crate::BezierParallelFragment2),
    SelectedFiber(&'a crate::bezier_split::BezierSelectedFiberFragment2),
    AlgebraicCusp(&'a crate::BezierAlgebraicCuspSemicircleFragment2),
}

#[derive(Clone, Copy)]
enum ExactCornerBezier2<'a> {
    Direct(&'a Curve2),
    NativeSpan(&'a NativeBezierFragment2),
}

pub(crate) enum ExactCornerArc2<'a> {
    Native(&'a CircularArc2),
    RetainedRational(Box<RetainedRationalCornerArc2<'a>>),
}

pub(crate) struct RetainedRationalCornerArc2<'a> {
    source: &'a Curve2,
    support: CircularArc2,
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
    let cells = retained_arc_fillet_projective_cells(support, mode, family, policy)?;
    let mut retained = None;
    let mut unresolved = None;
    for (curve, include_start, include_end, cell) in cells {
        let parameter = match offset_parameter
            .concentric_quadratic_conic_parameter(offset_half, &radial_scale, &curve, policy)
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

impl ExactCornerArc2<'_> {
    fn support(&self) -> &CircularArc2 {
        match self {
            Self::Native(arc) => arc,
            Self::RetainedRational(retained) => &retained.support,
        }
    }

    /// Returns the authored rational circle in its original low-degree
    /// parameterization and attaches the already-certified circular support.
    ///
    /// Preserving mixed weights is important for a regular major quadratic:
    /// its denominator is pole-free even though the middle Bernstein weight
    /// has the opposite sign. Keeping degree two lets the shared incidence
    /// kernels reuse the authored parameter without a larger elimination.
    fn retained_rational_evaluator(
        &self,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<RationalBezier2>> {
        let Self::RetainedRational(retained) = self else {
            return Ok(None);
        };
        let [evaluator] = retained
            .source
            .rational_evaluators_for_operation(policy, operation)?
        else {
            return Err(ExactCurveError::invalid(
                operation,
                family,
                CurveError::Topology(
                    "retained circular carrier did not promote to one rational evaluator".into(),
                ),
            ));
        };
        let evaluator = if evaluator.retained_circular_conic().is_some()
            || matches!(
                evaluator.common_weight_sign(policy),
                Classification::Decided(RealSign::Positive | RealSign::Negative)
            ) {
            evaluator.clone()
        } else {
            let (implicit_conic, circular_conic) = circular_conic_provenance(self.support());
            RationalBezier2::try_new_with_implicit_quadratic_conic(
                evaluator.control_points().to_vec(),
                evaluator.weights().to_vec(),
                implicit_conic,
                Some(circular_conic),
            )
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        };
        // Degree elevation preserves the parameter but multiplies every
        // circle-incidence equation by structural zero factors. Collapse it
        // before root isolation so one geometric contact has one parameter,
        // independent of the authored degree. The retained quadratic frame is
        // exact and keeps mixed-weight major charts pole-free.
        match evaluator
            .retained_quadratic_representative(policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(Some(quadratic)) => Ok(Some(RationalBezier2::from(quadratic))),
            Classification::Decided(None) => Ok(Some(evaluator)),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, family, reason))
            }
        }
    }

    fn source_parameter_at_point(
        &self,
        point: &Point2,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<BezierParameter2>> {
        let Self::RetainedRational(retained) = self else {
            return Ok(None);
        };
        let evaluator = self
            .retained_rational_evaluator(operation, family, policy)?
            .expect("a retained rational arc supplies its authored evaluator");
        let parameters = match evaluator
            .retained_circle_point_parameters(point, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, family, reason));
            }
        };
        let [parameter] = parameters.as_slice() else {
            return Err(if parameters.is_empty() {
                ExactCurveError::invalid(
                    operation,
                    family,
                    CurveError::Topology(
                        "retained circular carrier omitted a certified support contact".into(),
                    ),
                )
            } else {
                ExactCurveError::blocked(operation, family, crate::UncertaintyReason::Boundary)
            });
        };
        let domain = retained.source.native_parameter_domain()?;
        if let Some(parameter) = parameter.scalar() {
            // Preserve the established source-domain expression order on the
            // represented hot path. Besides avoiding an algebraic-map setup,
            // this keeps retained circle/circle construction witnesses
            // structurally identical to their authored corner parameters.
            return Ok(Some(BezierParameter2::Exact(
                domain.start() + (domain.end() - domain.start()) * parameter,
            )));
        }
        match parameter
            .affine_image_unbounded(&(domain.end() - domain.start()), domain.start(), policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(parameter) => Ok(Some(parameter)),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, family, reason))
            }
        }
    }

    fn corner_parameter(&self, previous: bool) -> Real {
        match self {
            Self::Native(_) => {
                if previous {
                    Real::one()
                } else {
                    Real::zero()
                }
            }
            Self::RetainedRational(retained) => {
                if previous {
                    retained
                        .source
                        .parameter_domain()
                        .end()
                        .scalar()
                        .expect("native circular parameter")
                        .clone()
                } else {
                    retained
                        .source
                        .parameter_domain()
                        .start()
                        .scalar()
                        .expect("native circular parameter")
                        .clone()
                }
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

    pub(crate) fn retained_rational_arc_support(&self) -> Option<&CircularArc2> {
        match self {
            Self::RetainedRationalArc(arc) => Some(&arc.support),
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
    let retained = |support: CircularArc2| -> ExactCornerCarrier2<'a> {
        ExactCornerCarrier2::RetainedRationalArc(Box::new(RetainedRationalCornerArc2 {
            source: curve,
            support,
        }))
    };
    let bezier = || ExactCornerCarrier2::Bezier(curve);
    Ok(match curve.geometry() {
        None => match curve.retained_fragment().expect("restricted carrier") {
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
            crate::BezierSplitFragment2::AlgebraicEndpointImages { .. }
            | crate::BezierSplitFragment2::Materialized { .. } => None,
        },
        Some(CurveGeometry2::CircularArc(arc)) => Some(ExactCornerCarrier2::Arc(arc)),
        Some(CurveGeometry2::RationalQuadraticBezier(_))
        | Some(CurveGeometry2::RationalBezier(_)) => Some(
            match retained_rational_arc_support(curve, operation, policy)? {
                Some(support) => retained(support),
                None => bezier(),
            },
        ),
        Some(CurveGeometry2::QuadraticBezier(_)) | Some(CurveGeometry2::CubicBezier(_)) => {
            Some(bezier())
        }
        Some(CurveGeometry2::PolynomialBSpline(_)) | Some(CurveGeometry2::Nurbs(_)) => {
            let fragments = match curve
                .native_bezier_fragments_with_policy(policy)
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
            Some(ExactCornerCarrier2::NativeBezierSpan(fragment))
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
        ExactCornerBezier2::NativeSpan(fragment) => match fragment.curve() {
            BezierSubcurve2::Quadratic(source) => source.parallel_left(distance),
            BezierSubcurve2::Cubic(source) => source.parallel_left(distance),
            BezierSubcurve2::RationalQuadratic(source) => source.parallel_left(distance),
            BezierSubcurve2::Rational(source) => source.parallel_left(distance),
        },
    };
    parallel.map_err(|cause| ExactCurveError::invalid(operation, family, cause))
}

impl<'a> ExactCornerBezier2<'a> {
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
                    fragment.curve().end()
                } else {
                    fragment.curve().start()
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
        let (start, end) = match self {
            Self::Direct(source) => {
                let domain = source.parameter_domain();
                domain
                    .scalar_endpoints()
                    .expect("direct native Bezier domain")
            }
            Self::NativeSpan(fragment) => fragment.parameter_range(),
        };
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
            policy,
        );
    }
    solve_carrier_fillet_corner(
        previous,
        next,
        radius,
        mode,
        retain_selected_circle_endpoints,
        [FilletContactDomain2::OpenCurve; 2],
        previous_family,
        next_family,
        policy,
    )
}

#[derive(Clone, Copy)]
enum FilletLinearSource2<'a> {
    Native {
        source: &'a LineSeg2,
        parallel_tangent_contacts: &'a [crate::bezier::BezierParallelLineTangentContact2],
    },
    AlgebraicChord(&'a crate::BezierAlgebraicChord2),
}

impl FilletLinearSource2<'_> {
    const fn native_line(&self) -> Option<&LineSeg2> {
        match self {
            Self::Native { source, .. } => Some(source),
            Self::AlgebraicChord(_) => None,
        }
    }

    const fn algebraic_chord(&self) -> Option<&crate::BezierAlgebraicChord2> {
        match self {
            Self::Native { .. } => None,
            Self::AlgebraicChord(source) => Some(source),
        }
    }

    fn parallel_tangent_contacts(&self) -> &[crate::bezier::BezierParallelLineTangentContact2] {
        match self {
            Self::Native {
                parallel_tangent_contacts,
                ..
            } => parallel_tangent_contacts,
            Self::AlgebraicChord(source) => source.parallel_tangent_contacts(),
        }
    }
}

#[derive(Clone, Copy)]
enum FilletParallelSource2<'a> {
    Direct(ExactCornerBezier2<'a>),
    Retained(&'a crate::BezierParallelFragment2),
    Selected(&'a crate::bezier_split::BezierSelectedFiberFragment2),
}

impl FilletParallelSource2<'_> {
    const fn retained(&self) -> Option<&crate::BezierParallelFragment2> {
        match self {
            Self::Direct(_) => None,
            Self::Retained(source) => Some(source),
            Self::Selected(_) => None,
        }
    }

    fn parameter_range(&self) -> Option<BezierParameterRange2> {
        match self {
            Self::Direct(_) => Some(BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            )),
            Self::Retained(source) => Some(source.range().clone()),
            Self::Selected(_) => None,
        }
    }

    fn curve_parameter_range(&self) -> crate::CurveParameterRange2 {
        match self {
            Self::Direct(_) => crate::CurveParameterRange2::from_bezier_range(
                BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
            ),
            Self::Retained(source) => {
                crate::CurveParameterRange2::from_bezier_range(source.range().clone())
            }
            Self::Selected(source) => source.range().clone(),
        }
    }

    /// Returns a finite rational envelope for intersection enumeration.
    ///
    /// Selected endpoints remain authoritative for final admissibility. Their
    /// exact isolating bounds merely enlarge the solve interval, so no bound
    /// can become a construction parameter or discard an authored contact.
    fn intersection_parameter_range(
        &self,
        family: CurveFamily2,
    ) -> ExactCurveResult<BezierParameterRange2> {
        if let Some(range) = self.parameter_range() {
            return Ok(range);
        }
        let range = self.curve_parameter_range();
        let (Some((start, _)), Some((_, end))) = (
            range.start().finite_envelope_bounds(),
            range.end().finite_envelope_bounds(),
        ) else {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                crate::UncertaintyReason::Unsupported,
            ));
        };
        Ok(BezierParameterRange2::new_validated(
            BezierParameter2::Exact(start.clone()),
            BezierParameter2::Exact(end.clone()),
        ))
    }

    fn incident_domain(
        &self,
        support: &BezierParallel2,
        previous: bool,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<crate::bezier_offset::BezierParallelIncidentDomain2> {
        let source_reversed = match self {
            Self::Direct(_) => false,
            Self::Retained(source) => source.is_reversed(),
            Self::Selected(source) => source.is_reversed(),
        };
        let extends_toward_higher_parameter = previous != source_reversed;
        let direction = if extends_toward_higher_parameter {
            crate::BezierParameterRayDirection2::Increasing
        } else {
            crate::BezierParameterRayDirection2::Decreasing
        };
        let domain = match self {
            Self::Direct(_) | Self::Retained(_) => {
                let range = self
                    .parameter_range()
                    .expect("an ordinary parallel source has a Bezier range");
                let endpoint = if extends_toward_higher_parameter {
                    range.end()
                } else {
                    range.start()
                };
                support.incident_domain_from_parameter(endpoint, direction, policy)
            }
            Self::Selected(source) => {
                let endpoint = if extends_toward_higher_parameter {
                    source.range().end()
                } else {
                    source.range().start()
                };
                if let Some(endpoint) = endpoint.as_bezier_parameter() {
                    support.incident_domain_from_parameter(endpoint, direction, policy)
                } else if endpoint.is_retained_scalar() {
                    support.incident_domain_from_retained_parameter(endpoint, direction, policy)
                } else {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::Unsupported,
                    ));
                }
            }
        };
        match domain
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(domain) => Ok(domain),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            )),
        }
    }

    fn parameter_is_in_open_range(
        &self,
        parameter: &BezierParameter2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        match self {
            Self::Direct(_) => bezier_trim_parameter_is_interior(
                parameter,
                CurveOperation2::Fillet,
                family,
                policy,
            ),
            Self::Retained(source) => retained_fillet_parameter_is_in_open_range(
                parameter,
                source.range(),
                family,
                policy,
            ),
            Self::Selected(source) => retained_selected_fillet_parameter_is_in_open_range(
                parameter,
                source.range(),
                family,
                policy,
            ),
        }
    }

    fn parameter_placement(
        &self,
        parameter: &CurveParameter2,
        previous: bool,
        mode: CurveCornerMode2,
        domain: FilletContactDomain2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CornerPlacement2>> {
        let placement = match self {
            Self::Direct(_) => curve_region_corner_parameter_placement(
                parameter,
                previous,
                mode,
                CurveOperation2::Fillet,
                family,
                policy,
            )?,
            Self::Retained(source) => retained_parallel_corner_parameter_placement(
                parameter,
                source,
                previous,
                mode,
                CurveOperation2::Fillet,
                family,
                policy,
            )?,
            Self::Selected(source) => selected_fiber_corner_parameter_placement(
                parameter,
                source,
                previous,
                mode,
                CurveOperation2::Fillet,
                family,
                policy,
            )?,
        };
        domain.with_boundary_contact(
            placement,
            parameter,
            || {
                let reversed = match self {
                    Self::Direct(_) => false,
                    Self::Retained(source) => source.is_reversed(),
                    Self::Selected(source) => source.is_reversed(),
                };
                let range = self.curve_parameter_range();
                if previous != reversed {
                    range.end().clone()
                } else {
                    range.start().clone()
                }
            },
            family,
            policy,
        )
    }

    fn parameter_is_admissible(
        &self,
        parameter: &CurveParameter2,
        previous: bool,
        mode: CurveCornerMode2,
        domain: FilletContactDomain2,
        incident_domain: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let placement =
            self.parameter_placement(parameter, previous, mode, domain, family, policy)?;
        match placement {
            Some(CornerPlacement2::Trim | CornerPlacement2::Corner) => Ok(true),
            Some(CornerPlacement2::Extension) => {
                let domain = incident_domain.ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::Unsupported,
                    )
                })?;
                match domain
                    .contains_extension_parameter(parameter, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })? {
                    Classification::Decided(inside) => Ok(inside),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        reason,
                    )),
                }
            }
            None => Ok(false),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn bezier_parameter_is_admissible(
        &self,
        parameter: &BezierParameter2,
        previous: bool,
        mode: CurveCornerMode2,
        domain: FilletContactDomain2,
        incident_domain: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        self.parameter_is_admissible(
            &CurveParameter2::from(parameter.clone()),
            previous,
            mode,
            domain,
            incident_domain,
            family,
            policy,
        )
    }

    fn support_reverses_source(
        &self,
        support: &BezierParallel2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        match self {
            Self::Retained(source) => {
                retained_fillet_parallel_support_reverses_source(source, support, family, policy)
            }
            Self::Selected(source) => {
                let source_parallel = source.parallel_carrier();
                let source_scale = selected_fiber_parallel_derivative_scale_sign(
                    &source_parallel,
                    source,
                    family,
                    policy,
                )?;
                let support_scale =
                    selected_fiber_parallel_derivative_scale_sign(support, source, family, policy)?;
                Ok((source_scale != support_scale) != source.is_reversed())
            }
            Self::Direct(_) => {
                let range = BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                );
                match support
                    .regular_fragment_derivative_scale_sign(&range, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })? {
                    Classification::Decided(RealSign::Positive) => Ok(false),
                    Classification::Decided(RealSign::Negative) => Ok(true),
                    Classification::Decided(RealSign::Zero) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::Boundary,
                    )),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        reason,
                    )),
                }
            }
        }
    }

    fn parallel_distance(&self) -> Real {
        match self {
            Self::Direct(_) => Real::zero(),
            Self::Retained(source) => source.parallel().distance().clone(),
            Self::Selected(source) => source.parallel_carrier().distance().clone(),
        }
    }
}

/// Authored trim boundaries are open. An internal partition belongs to the
/// chart that survives the cut, including its one-sided tangent. The complete
/// authored domain places that contact after transport to the source chart.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum FilletContactDomain2 {
    OpenCurve,
    TrimChart,
}

impl FilletContactDomain2 {
    fn with_boundary_contact(
        self,
        placement: Option<CornerPlacement2>,
        parameter: &CurveParameter2,
        endpoint: impl FnOnce() -> CurveParameter2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CornerPlacement2>> {
        if self == Self::OpenCurve || placement.is_some() {
            return Ok(placement);
        }
        match parameter
            .cmp_by_refinement(&endpoint(), policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(order) => Ok(order.is_eq().then_some(CornerPlacement2::Trim)),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            )),
        }
    }
}

enum PreparedFilletCarrier2<'a> {
    Line {
        source: FilletLinearSource2<'a>,
        /// Only retained algebraic chords need an owned represented support;
        /// native line sources are already their own support.
        chord_support: Option<LineSeg2>,
        unit_x: Real,
        unit_y: Real,
    },
    Arc {
        source: ExactCornerArc2<'a>,
        radius: Real,
    },
    Bezier {
        source: ExactCornerBezier2<'a>,
    },
    AlgebraicCusp {
        source: &'a crate::BezierAlgebraicCuspSemicircleFragment2,
    },
    AlgebraicChord {
        source: &'a crate::BezierAlgebraicChord2,
    },
    AnalyticParallel {
        source: &'a crate::BezierParallelFragment2,
    },
    SelectedFiber {
        source: &'a crate::bezier_split::BezierSelectedFiberFragment2,
        parallel: BezierParallel2,
    },
}

impl<'a> PreparedFilletCarrier2<'a> {
    fn new(
        carrier: ExactCornerCarrier2<'a>,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        match carrier {
            ExactCornerCarrier2::Line(source) => {
                let (dx, dy) = source.delta();
                let (unit_x, unit_y, _) =
                    line_unit_direction(&dx, &dy, CurveOperation2::Fillet, family, policy)?;
                Ok(Self::Line {
                    source: FilletLinearSource2::Native {
                        source,
                        parallel_tangent_contacts: &[],
                    },
                    chord_support: None,
                    unit_x,
                    unit_y,
                })
            }
            ExactCornerCarrier2::PromotedLine(curve) => {
                let source = curve
                    .retained_exact_line_image()
                    .expect("a promoted-line carrier retains its exact line image");
                let (dx, dy) = source.delta();
                let (unit_x, unit_y, _) =
                    line_unit_direction(&dx, &dy, CurveOperation2::Fillet, family, policy)?;
                Ok(Self::Line {
                    source: FilletLinearSource2::Native {
                        source,
                        parallel_tangent_contacts: curve.retained_parallel_line_tangent_contacts(),
                    },
                    chord_support: None,
                    unit_x,
                    unit_y,
                })
            }
            ExactCornerCarrier2::Arc(source) => {
                let radius =
                    exact_corner_arc_radius(source, CurveOperation2::Fillet, family, policy)?;
                Ok(Self::Arc {
                    source: ExactCornerArc2::Native(source),
                    radius,
                })
            }
            ExactCornerCarrier2::RetainedRationalArc(source) => {
                let source = ExactCornerArc2::RetainedRational(source);
                let radius = exact_corner_arc_radius(
                    source.support(),
                    CurveOperation2::Fillet,
                    family,
                    policy,
                )?;
                Ok(Self::Arc { source, radius })
            }
            ExactCornerCarrier2::Bezier(source) => Ok(Self::Bezier {
                source: ExactCornerBezier2::Direct(source),
            }),
            ExactCornerCarrier2::NativeBezierSpan(fragment) => Ok(Self::Bezier {
                source: ExactCornerBezier2::NativeSpan(fragment),
            }),
            ExactCornerCarrier2::AlgebraicCusp(source) => Ok(Self::AlgebraicCusp { source }),
            ExactCornerCarrier2::AlgebraicChord(source) => {
                let canonical_axis_support = match (
                    source.certified_axis_direction(),
                    source
                        .exact_axis_support_coordinate(policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })?,
                ) {
                    (Some(direction), Some(coordinate)) => {
                        let (unit_x, unit_y) = direction.unit_tangent();
                        let start = match direction.axis() {
                            crate::Axis2::X => Point2::new(Real::zero(), coordinate),
                            crate::Axis2::Y => Point2::new(coordinate, Real::zero()),
                        };
                        Some(LineSeg2::new_unchecked(
                            start.clone(),
                            start.translated(unit_x, unit_y),
                        ))
                    }
                    _ => None,
                };
                let Some(support) = canonical_axis_support
                    .or_else(|| source.exact_line())
                    .or_else(|| source.strict_provenance_support_line(policy))
                else {
                    return Ok(Self::AlgebraicChord { source });
                };
                let (unit_x, unit_y) = if let Some(unit) = source.certified_unit_tangent() {
                    unit
                } else {
                    let (dx, dy) = support.delta();
                    let (unit_x, unit_y, _) =
                        line_unit_direction(&dx, &dy, CurveOperation2::Fillet, family, policy)?;
                    (unit_x, unit_y)
                };
                Ok(Self::Line {
                    source: FilletLinearSource2::AlgebraicChord(source),
                    chord_support: Some(support),
                    unit_x,
                    unit_y,
                })
            }
            ExactCornerCarrier2::AnalyticParallel(source) => Ok(Self::AnalyticParallel { source }),
            ExactCornerCarrier2::SelectedFiber(source) => Ok(Self::SelectedFiber {
                source,
                parallel: source.parallel_carrier(),
            }),
        }
    }

    fn offset<'b>(
        &'b self,
        signed_distance: &Real,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<FilletOffsetCarrier2<'a, 'b>> {
        match self {
            Self::Line {
                source,
                chord_support,
                unit_x,
                unit_y,
            } => {
                let source_support = source
                    .native_line()
                    .or(chord_support.as_ref())
                    .expect("a prepared linear fillet carrier retains one support");
                let offset_x = -unit_y * signed_distance;
                let offset_y = unit_x * signed_distance;
                // Translation preserves the already-validated nonzero source
                // direction, so rebuilding an endpoint-distance proof would
                // only allocate an algebraically identical norm.
                let support = LineSeg2::new_unchecked(
                    source_support
                        .start()
                        .translated(offset_x.clone(), offset_y.clone()),
                    source_support.end().translated(offset_x, offset_y),
                );
                Ok(FilletOffsetCarrier2::Line {
                    source: *source,
                    support,
                    unit_x,
                    unit_y,
                    signed_distance: signed_distance.clone(),
                })
            }
            Self::Arc { source, radius } => {
                let support = source.support();
                let signed_radius = if support.is_clockwise() {
                    radius + signed_distance
                } else {
                    radius - signed_distance
                };
                match crate::classify::real_sign(&signed_radius, policy) {
                    Some(RealSign::Zero) => Ok(FilletOffsetCarrier2::Point {
                        point: CurvePoint2::from(support.center().clone()),
                    }),
                    Some(RealSign::Positive | RealSign::Negative) => {
                        Ok(FilletOffsetCarrier2::Arc {
                            source,
                            source_radius: radius,
                            signed_radius,
                        })
                    }
                    None => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::RealSign,
                    )),
                }
            }
            Self::Bezier { source } => Ok(FilletOffsetCarrier2::Parallel {
                source: FilletParallelSource2::Direct(*source),
                support: exact_corner_bezier_parallel(
                    *source,
                    signed_distance.clone(),
                    CurveOperation2::Fillet,
                    family,
                )?,
            }),
            Self::AlgebraicCusp { source } => {
                let support =
                    match source
                        .offset_left(signed_distance, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })? {
                        Classification::Decided(Some(support)) => support,
                        Classification::Decided(None) => {
                            let point = match source
                                .semicircle()
                                .center_point_evidence(policy)
                                .map_err(|cause| {
                                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                                })? {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        family,
                                        reason,
                                    ));
                                }
                            };
                            return Ok(FilletOffsetCarrier2::Point { point });
                        }
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                reason,
                            ));
                        }
                    };
                Ok(FilletOffsetCarrier2::AlgebraicCusp { source, support })
            }
            Self::AlgebraicChord { source } => {
                let support = source
                    .parallel_left_retained(signed_distance.clone(), policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                Ok(FilletOffsetCarrier2::AlgebraicChord {
                    source,
                    support,
                    signed_distance: signed_distance.clone(),
                    finite_source_domain: true,
                })
            }
            Self::AnalyticParallel { source } => {
                let source_scale = match source
                    .parallel()
                    .regular_fragment_derivative_scale_sign(source.range(), policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })? {
                    Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => {
                        sign
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            family,
                            crate::UncertaintyReason::Boundary,
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
                let traversal_agrees_with_source =
                    (source_scale == RealSign::Positive) != source.is_reversed();
                let distance = if traversal_agrees_with_source {
                    source.parallel().distance() + signed_distance
                } else {
                    source.parallel().distance() - signed_distance
                };
                Ok(FilletOffsetCarrier2::Parallel {
                    source: FilletParallelSource2::Retained(source),
                    support: source.parallel().with_distance(distance),
                })
            }
            Self::SelectedFiber { source, parallel } => {
                let source_scale = selected_fiber_parallel_derivative_scale_sign(
                    parallel, source, family, policy,
                )?;
                let traversal_agrees_with_source =
                    (source_scale == RealSign::Positive) != source.is_reversed();
                let distance = if traversal_agrees_with_source {
                    parallel.distance() + signed_distance
                } else {
                    parallel.distance() - signed_distance
                };
                Ok(FilletOffsetCarrier2::Parallel {
                    source: FilletParallelSource2::Selected(source),
                    support: parallel.with_distance(distance),
                })
            }
        }
    }
}

enum FilletOffsetCarrier2<'a, 'b> {
    Line {
        source: FilletLinearSource2<'a>,
        support: LineSeg2,
        unit_x: &'b Real,
        unit_y: &'b Real,
        signed_distance: Real,
    },
    Arc {
        source: &'b ExactCornerArc2<'a>,
        source_radius: &'b Real,
        signed_radius: Real,
    },
    Point {
        point: CurvePoint2,
    },
    Parallel {
        source: FilletParallelSource2<'a>,
        support: BezierParallel2,
    },
    AlgebraicCusp {
        source: &'a crate::BezierAlgebraicCuspSemicircleFragment2,
        support: crate::BezierAlgebraicCuspSemicircleFragment2,
    },
    AlgebraicChord {
        source: &'a crate::BezierAlgebraicChord2,
        support: crate::BezierAlgebraicChord2,
        signed_distance: Real,
        /// Whether the support witness endpoints are the authored finite
        /// domain. A canonical line witness can name only the infinite
        /// support; its source chord owns final trim/extension classification.
        finite_source_domain: bool,
    },
}

impl FilletOffsetCarrier2<'_, '_> {
    fn retained_fillet_frame(
        &self,
        anchor_is_previous: bool,
        anchor_parameter: Option<&CurveParameter2>,
        mut anchor_evidence: Option<RetainedFilletAnchorEvidence2>,
        force_chord_normal: bool,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<RetainedFilletFrame2>> {
        let (radial_frame, radial_distance) = match self {
            Self::Line {
                source,
                support,
                unit_x,
                unit_y,
                signed_distance,
                ..
            } => {
                let radial_frame = if force_chord_normal {
                    let anchor = if let Some(anchor) = source.algebraic_chord() {
                        anchor.clone()
                    } else {
                        algebraic_chord_from_line_support(
                            support,
                            CurveOperation2::Fillet,
                            family,
                            policy,
                        )?
                    };
                    #[cfg(feature = "dispatch-trace")]
                    if let Some((anchor_x, anchor_y)) = anchor.certified_unit_tangent() {
                        let dot = &anchor_x * *unit_x + &anchor_y * *unit_y;
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-fillet-chord-frame-orientation",
                            match crate::classify::real_sign(&dot, policy) {
                                Some(RealSign::Positive) => "agrees-with-line",
                                Some(RealSign::Negative) => "reverses-line",
                                Some(RealSign::Zero) => "orthogonal-to-line",
                                None => "uncertain",
                            },
                        );
                    }
                    RetainedFilletRadialFrame2::ChordNormal {
                        anchor,
                        policy: *policy,
                    }
                } else if let Some(center_frame) = anchor_evidence
                    .as_ref()
                    .and_then(|evidence| evidence.center_parallel.clone())
                {
                    let Some(center_parameter) = center_frame
                        .parameter
                        .as_ref()
                        .and_then(CurveParameter2::as_bezier_parameter)
                        .cloned()
                        .or_else(|| {
                            anchor_parameter
                                .and_then(CurveParameter2::as_bezier_parameter)
                                .cloned()
                        })
                    else {
                        return Ok(None);
                    };
                    RetainedFilletRadialFrame2::ParallelNormal {
                        center_support: center_frame.support,
                        center_parameter,
                        policy: *policy,
                    }
                } else {
                    RetainedFilletRadialFrame2::RepresentedUnitNormal((
                        -(*unit_y).clone(),
                        (*unit_x).clone(),
                    ))
                };
                (radial_frame, -signed_distance.clone())
            }
            Self::Arc {
                source,
                source_radius,
                signed_radius,
            } => {
                let support = source.support();
                let (normal_denominator, radial_distance) = if support.is_clockwise() {
                    (signed_radius.clone(), *source_radius - signed_radius)
                } else {
                    (-signed_radius.clone(), signed_radius - *source_radius)
                };
                let signed_radius_sign =
                    crate::classify::real_sign(signed_radius, &CurveContext::STRICT);
                let selected_center = anchor_evidence
                    .as_mut()
                    .and_then(|evidence| evidence.deferred_arc_contact.as_mut())
                    .and_then(|deferred| deferred.selected_center.take());
                let radial_frame = if let Some(selected_center) = selected_center.as_ref() {
                    RetainedFilletRadialFrame2::SelectedConcentric {
                        support: selected_center
                            .mapped_semicircle_carrier()
                            .expect("a retained selected arc center is mapped")
                            .clone(),
                        center_parameter: selected_center.clone(),
                        normal_denominator: normal_denominator.clone(),
                    }
                } else if let (Some(center_frame), Some(center_parameter)) = (
                    anchor_evidence
                        .as_ref()
                        .and_then(|evidence| evidence.center_parallel.as_ref()),
                    anchor_parameter.filter(|parameter| parameter.is_retained_scalar()),
                ) {
                    let anchor = match crate::BezierAlgebraicChord2::from_certified_retained_parallel_unit_tangent(
                        center_frame.support.clone(),
                        center_parameter,
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })? {
                        Classification::Decided(anchor) => anchor,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                reason,
                            ));
                        }
                    };
                    // Positive concentric scaling preserves the offset cell's
                    // left normal. Past-center scaling reverses it. Never
                    // choose an orientation when that nonzero sign was not
                    // proved by the center construction.
                    let source_direction = anchor_evidence
                        .as_ref()
                        .and_then(|evidence| evidence.source_direction)
                        .or(signed_radius_sign);
                    let anchor = match source_direction {
                        Some(RealSign::Positive) => anchor,
                        Some(RealSign::Negative) => anchor.reversed(),
                        Some(RealSign::Zero) => {
                            return Err(ExactCurveError::invalid(
                                CurveOperation2::Fillet,
                                family,
                                CurveError::Topology(
                                    "a retained arc fillet frame had zero center radius".into(),
                                ),
                            ));
                        }
                        None => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                crate::UncertaintyReason::RealSign,
                            ));
                        }
                    };
                    RetainedFilletRadialFrame2::ChordNormal {
                        anchor,
                        policy: *policy,
                    }
                } else if signed_radius_sign == Some(RealSign::Positive) {
                    match (
                        anchor_evidence
                            .as_ref()
                            .and_then(|evidence| evidence.center_parallel.as_ref())
                            .map(|frame| frame.support.clone()),
                        anchor_parameter
                            .and_then(CurveParameter2::as_bezier_parameter)
                            .cloned(),
                    ) {
                        (Some(center_support), Some(center_parameter)) => {
                            RetainedFilletRadialFrame2::ParallelNormal {
                                center_support,
                                center_parameter,
                                policy: *policy,
                            }
                        }
                        _ => RetainedFilletRadialFrame2::ConcentricArc {
                            support_center: support.center().clone(),
                            normal_denominator,
                        },
                    }
                } else {
                    // A past-center concentric image reverses its rational
                    // tangent. Keep the orientation-independent radial
                    // frame unless that reversal is proved and retained by
                    // a future mapped-contact authority.
                    RetainedFilletRadialFrame2::ConcentricArc {
                        support_center: support.center().clone(),
                        normal_denominator,
                    }
                };
                (radial_frame, radial_distance)
            }
            Self::AlgebraicCusp { source, support } => {
                let complementary =
                    anchor_parameter.is_some_and(CurveParameter2::is_algebraic_cusp_complement);
                let support_circle = if complementary {
                    support.semicircle().complementary_half()
                } else {
                    support.semicircle().clone()
                };
                let source_circle = if complementary {
                    source.semicircle().complementary_half()
                } else {
                    source.semicircle().clone()
                };
                let center =
                    match support_circle
                        .center_point_evidence(policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })? {
                        Classification::Decided(center) => center,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                reason,
                            ));
                        }
                    };
                let support_radius = support_circle.radial_distance();
                let source_radius = source_circle.radial_distance();
                let clockwise = support_circle.is_clockwise() != support.is_reversed();
                let (normal_denominator, radial_distance) = if clockwise {
                    (support_radius.clone(), source_radius - support_radius)
                } else {
                    (-support_radius.clone(), support_radius - source_radius)
                };
                let support_center = match &center {
                    CurvePoint2(CurvePointData2::Exact(point)) => Some(point.clone()),
                    CurvePoint2(CurvePointData2::Algebraic(image)) => {
                        image.exact_point(&CurveContext::STRICT)
                    }
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                    | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                    | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                    | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                    | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                    | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                        None
                    }
                };
                let selected_center_parameter = anchor_parameter
                    .and_then(CurveParameter2::as_algebraic_cusp)
                    .cloned();
                let retain_pair_frame = selected_center_parameter
                    .as_ref()
                    .is_some_and(|parameter| parameter.retains_pair_contact());
                let radial_frame =
                    if let Some(support_center) = support_center.filter(|_| !retain_pair_frame) {
                        RetainedFilletRadialFrame2::ConcentricArc {
                            support_center,
                            normal_denominator,
                        }
                    } else {
                        let Some(center_parameter) = selected_center_parameter else {
                            return Ok(None);
                        };
                        RetainedFilletRadialFrame2::SelectedConcentric {
                            support: support_circle,
                            center_parameter,
                            normal_denominator,
                        }
                    };
                (radial_frame, radial_distance)
            }
            Self::Parallel { source, support } => {
                let radial_frame = if let Some(center_parameter) = anchor_parameter
                    .and_then(CurveParameter2::as_bezier_parameter)
                    .cloned()
                {
                    RetainedFilletRadialFrame2::ParallelNormal {
                        center_support: support.clone(),
                        center_parameter,
                        policy: *policy,
                    }
                } else if let Some(center_parameter) =
                    anchor_parameter.filter(|parameter| parameter.is_retained_scalar())
                {
                    let anchor = match crate::BezierAlgebraicChord2::from_certified_retained_parallel_unit_tangent(
                        support.clone(),
                        center_parameter,
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })? {
                        Classification::Decided(anchor) => anchor,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                reason,
                            ));
                        }
                    };
                    RetainedFilletRadialFrame2::ChordNormal {
                        anchor,
                        policy: *policy,
                    }
                } else {
                    return Ok(None);
                };
                (
                    radial_frame,
                    source.parallel_distance() - support.distance(),
                )
            }
            Self::AlgebraicChord {
                source,
                signed_distance,
                ..
            } => {
                let radial_frame = RetainedFilletRadialFrame2::ChordNormal {
                    anchor: (*source).clone(),
                    policy: *policy,
                };
                (radial_frame, -signed_distance.clone())
            }
            _ => return Ok(None),
        };
        Ok(Some(RetainedFilletFrame2 {
            anchor_is_previous,
            radial_frame,
            radial_distance,
            anchor_evidence,
        }))
    }
}

struct FilletCenterWitness2 {
    point: CurvePoint2,
    previous_parameter: Option<CurveParameter2>,
    next_parameter: Option<CurveParameter2>,
    retained_anchor_evidence: Option<RetainedFilletAnchorEvidence2>,
}

const fn reverse_fillet_sign(sign: RealSign) -> RealSign {
    match sign {
        RealSign::Positive => RealSign::Negative,
        RealSign::Negative => RealSign::Positive,
        RealSign::Zero => RealSign::Zero,
    }
}

fn retained_fillet_cusp_support_reverses_source(
    source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    support: &crate::BezierAlgebraicCuspSemicircleFragment2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let nonzero_radius_sign = |radius: &Real| match crate::classify::real_sign(radius, policy) {
        Some(sign @ (RealSign::Positive | RealSign::Negative)) => Ok(sign),
        Some(RealSign::Zero) => Err(ExactCurveError::invalid(
            CurveOperation2::Fillet,
            family,
            CurveError::Topology(
                "a retained selected-circle fillet support had zero radius".into(),
            ),
        )),
        None => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            crate::UncertaintyReason::RealSign,
        )),
    };
    Ok((nonzero_radius_sign(source.semicircle().radial_distance())?
        != nonzero_radius_sign(support.semicircle().radial_distance())?)
        != source.is_reversed())
}

impl FilletCenterWitness2 {
    fn parameter(&self, previous: bool) -> Option<&CurveParameter2> {
        if previous {
            self.previous_parameter.as_ref()
        } else {
            self.next_parameter.as_ref()
        }
    }
}

#[derive(Default)]
struct FilletCenters2 {
    first: Option<FilletCenterWitness2>,
    second: Option<FilletCenterWitness2>,
    overflow: Vec<FilletCenterWitness2>,
    coincident: bool,
    outside_domain: bool,
}

impl FilletCenters2 {
    fn push(&mut self, witness: FilletCenterWitness2) {
        if self.first.is_none() {
            self.first = Some(witness);
        } else if self.second.is_none() {
            self.second = Some(witness);
        } else {
            self.overflow.push(witness);
        }
    }

    fn iter(&self) -> impl Iterator<Item = &FilletCenterWitness2> {
        self.first
            .iter()
            .chain(self.second.iter())
            .chain(self.overflow.iter())
    }

    fn iter_mut(&mut self) -> impl Iterator<Item = &mut FilletCenterWitness2> {
        self.first
            .iter_mut()
            .chain(self.second.iter_mut())
            .chain(self.overflow.iter_mut())
    }
}

#[allow(clippy::too_many_arguments)]
fn solve_carrier_fillet_corner(
    previous: ExactCornerCarrier2<'_>,
    next: ExactCornerCarrier2<'_>,
    radius: &Real,
    mode: CurveCornerMode2,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    // Primitive line/circle pairs retain represented Cartesian centers and
    // recover the original rational parameter directly. Other pairs keep the
    // authored rational evaluator: a deferred circle decomposition would name
    // a different projective chart and discard the source location.
    let represented_centers = |carrier: &ExactCornerCarrier2<'_>| {
        matches!(
            carrier,
            ExactCornerCarrier2::Line(_)
                | ExactCornerCarrier2::PromotedLine(_)
                | ExactCornerCarrier2::Arc(_)
                | ExactCornerCarrier2::RetainedRationalArc(_)
        )
    };
    let primitive_pair = represented_centers(&previous) && represented_centers(&next);
    let chart_carrier = |carrier, domain| match (carrier, domain, primitive_pair) {
        (ExactCornerCarrier2::RetainedRationalArc(arc), FilletContactDomain2::TrimChart, false) => {
            ExactCornerCarrier2::Bezier(arc.source)
        }
        (carrier, _, _) => carrier,
    };
    let previous =
        PreparedFilletCarrier2::new(chart_carrier(previous, domains[0]), previous_family, policy)?;
    let next = PreparedFilletCarrier2::new(chart_carrier(next, domains[1]), next_family, policy)?;
    let mut candidates = CornerSolutionAccumulator::Empty;
    let mut saw_outside_domain = false;
    let mut saw_degenerate = false;

    // Positive signed distance is the common left offset and therefore gives
    // a counterclockwise fillet. Preserve that documented candidate order.
    for clockwise in [false, true] {
        let signed_distance = if clockwise {
            -radius.clone()
        } else {
            radius.clone()
        };
        let previous_offset = previous.offset(&signed_distance, previous_family, policy)?;
        let next_offset = next.offset(&signed_distance, next_family, policy)?;
        let centers = fillet_offset_centers(
            &previous_offset,
            &next_offset,
            mode,
            domains,
            previous_family,
            next_family,
            policy,
        )?;
        saw_outside_domain |= centers.outside_domain;
        if centers.coincident {
            saw_degenerate = true;
            continue;
        }
        for center in centers.iter() {
            let deferred_arc_is_previous = center
                .retained_anchor_evidence
                .as_ref()
                .and_then(|evidence| evidence.deferred_arc_contact.as_ref())
                .map(|deferred| deferred.arc_is_previous);
            let Some(previous_cut) = fillet_cut_from_center(
                &previous_offset,
                &center.point,
                center.parameter(true),
                deferred_arc_is_previous == Some(true),
                true,
                mode,
                retain_selected_circle_endpoints,
                domains[0],
                previous_family,
                policy,
            )?
            else {
                saw_outside_domain = true;
                continue;
            };
            let Some(next_cut) = fillet_cut_from_center(
                &next_offset,
                &center.point,
                center.parameter(false),
                deferred_arc_is_previous == Some(false),
                false,
                mode,
                retain_selected_circle_endpoints,
                domains[1],
                next_family,
                policy,
            )?
            else {
                saw_outside_domain = true;
                continue;
            };
            let cut_point_relation = if center
                .retained_anchor_evidence
                .as_ref()
                .and_then(|evidence| evidence.cross)
                .is_some_and(|cross| matches!(cross, RealSign::Positive | RealSign::Negative))
            {
                // Two contacts on one nonzero-radius circle cannot occupy the
                // same point with nonparallel tangents: both tangents would be
                // perpendicular to the same radial vector. The pair replay's
                // exact nonzero tangent cross is therefore also a constant-
                // time distinct-cut certificate and avoids constructing a
                // potentially high-degree Cartesian compositum solely for
                // this degeneracy test.
                Classification::Decided(false)
            } else if center.point.coordinates().is_none()
                && center
                    .retained_anchor_evidence
                    .as_ref()
                    .is_some_and(|evidence| evidence.deferred_arc_contact.is_some())
            {
                // One circular cut is only a transient marker until the
                // retained fillet circle is intersected with the authored arc.
                // Its marker stores the center, so it cannot participate in
                // the ordinary two-contact degeneracy predicate.
                Classification::Decided(false)
            } else {
                previous_cut.point.same_point(&next_cut.point, policy)
            };
            match cut_point_relation {
                Classification::Decided(true) => saw_degenerate = true,
                Classification::Decided(false) => {
                    let previous_is_cusp =
                        matches!(previous_offset, FilletOffsetCarrier2::AlgebraicCusp { .. });
                    let next_is_cusp =
                        matches!(next_offset, FilletOffsetCarrier2::AlgebraicCusp { .. });
                    let previous_chord_anchors_on_next_arc = matches!(
                        (&previous_offset, &next_offset),
                        (
                            FilletOffsetCarrier2::AlgebraicChord { .. },
                            FilletOffsetCarrier2::Arc { .. }
                        )
                    );
                    let cusp_and_line = matches!(
                        (&previous_offset, &next_offset),
                        (
                            FilletOffsetCarrier2::AlgebraicCusp { .. },
                            FilletOffsetCarrier2::Line { .. }
                        )
                    ) || matches!(
                        (&previous_offset, &next_offset),
                        (
                            FilletOffsetCarrier2::Line { .. },
                            FilletOffsetCarrier2::AlgebraicCusp { .. }
                        )
                    );
                    let line_precedes_chord = matches!(
                        (&previous_offset, &next_offset),
                        (
                            FilletOffsetCarrier2::Line { .. },
                            FilletOffsetCarrier2::AlgebraicChord { .. }
                        )
                    );
                    let (first, first_is_previous, first_family, second, second_family) =
                        if cusp_and_line {
                            // The line is transiently lowered to a chord for
                            // center incidence. Keep the selected circle as
                            // the reconstruction anchor so its mapped radial
                            // field remains authoritative.
                            if previous_is_cusp {
                                (
                                    &previous_offset,
                                    true,
                                    previous_family,
                                    &next_offset,
                                    next_family,
                                )
                            } else {
                                (
                                    &next_offset,
                                    false,
                                    next_family,
                                    &previous_offset,
                                    previous_family,
                                )
                            }
                        } else if (previous_is_cusp && !next_is_cusp)
                            || previous_chord_anchors_on_next_arc
                            || line_precedes_chord
                        {
                            (
                                &next_offset,
                                false,
                                next_family,
                                &previous_offset,
                                previous_family,
                            )
                        } else {
                            (
                                &previous_offset,
                                true,
                                previous_family,
                                &next_offset,
                                next_family,
                            )
                        };
                    let deferred_arc_frame = center
                        .retained_anchor_evidence
                        .as_ref()
                        .and_then(|evidence| evidence.deferred_arc_contact.as_ref())
                        .map(|deferred| {
                            (deferred.arc_is_previous, deferred.contact_seed.is_some())
                        });
                    let prefer_parallel_frame = center
                        .retained_anchor_evidence
                        .as_ref()
                        .is_some_and(|evidence| {
                            evidence.center_parallel.is_some()
                                || evidence.source_direction.is_some()
                        });
                    let force_chord_normal = matches!(
                        (&previous_offset, &next_offset),
                        (
                            FilletOffsetCarrier2::AlgebraicChord { .. },
                            FilletOffsetCarrier2::Line { .. }
                        ) | (
                            FilletOffsetCarrier2::Line { .. },
                            FilletOffsetCarrier2::AlgebraicChord { .. }
                        )
                    );
                    #[cfg(feature = "dispatch-trace")]
                    {
                        let carrier_kind = |carrier: &FilletOffsetCarrier2<'_, '_>| match carrier {
                            FilletOffsetCarrier2::Line {
                                source: FilletLinearSource2::Native { .. },
                                ..
                            } => "line-native",
                            FilletOffsetCarrier2::Line {
                                source: FilletLinearSource2::AlgebraicChord(_),
                                ..
                            } => "line-chord",
                            FilletOffsetCarrier2::Arc { .. } => "arc",
                            FilletOffsetCarrier2::Point { .. } => "point",
                            FilletOffsetCarrier2::Parallel { .. } => "parallel",
                            FilletOffsetCarrier2::AlgebraicCusp { .. } => "algebraic-cusp",
                            FilletOffsetCarrier2::AlgebraicChord { .. } => "algebraic-chord",
                        };
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-fillet-previous-carrier",
                            carrier_kind(&previous_offset),
                        );
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-fillet-next-carrier",
                            carrier_kind(&next_offset),
                        );
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-fillet-force-chord-normal",
                            if force_chord_normal { "yes" } else { "no" },
                        );
                    }
                    let first_frame = first.retained_fillet_frame(
                        first_is_previous,
                        center.parameter(first_is_previous),
                        center.retained_anchor_evidence.clone(),
                        force_chord_normal,
                        first_family,
                        policy,
                    )?;
                    let frame_is_preferred = |frame: &RetainedFilletFrame2| {
                        if let Some((arc_is_previous, contact_is_preselected)) = deferred_arc_frame
                        {
                            return (frame.anchor_is_previous == arc_is_previous)
                                == contact_is_preselected;
                        }
                        if force_chord_normal {
                            return matches!(
                                &frame.radial_frame,
                                RetainedFilletRadialFrame2::ChordNormal { .. }
                            );
                        }
                        if matches!(
                            &frame.radial_frame,
                            RetainedFilletRadialFrame2::ChordNormal { .. }
                        ) {
                            return prefer_parallel_frame
                                && frame
                                    .anchor_evidence
                                    .as_ref()
                                    .and_then(|evidence| evidence.center_parallel.as_ref())
                                    .and_then(|center| center.parameter.as_ref())
                                    .is_some_and(CurveParameter2::is_retained_scalar);
                        }
                        matches!(
                            &frame.radial_frame,
                            RetainedFilletRadialFrame2::ParallelNormal { .. }
                        ) == prefer_parallel_frame
                    };
                    let retained_frame = if first_frame.as_ref().is_some_and(frame_is_preferred) {
                        first_frame
                    } else {
                        let second_frame = second.retained_fillet_frame(
                            !first_is_previous,
                            center.parameter(!first_is_previous),
                            center.retained_anchor_evidence.clone(),
                            force_chord_normal,
                            second_family,
                            policy,
                        )?;
                        if second_frame.as_ref().is_some_and(frame_is_preferred) {
                            second_frame
                        } else {
                            first_frame.or(second_frame)
                        }
                    };
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-fillet-retained-frame",
                        match retained_frame.as_ref().map(|frame| &frame.radial_frame) {
                            Some(RetainedFilletRadialFrame2::RepresentedUnitNormal(_)) => {
                                "represented-unit-normal"
                            }
                            Some(RetainedFilletRadialFrame2::ChordNormal { .. }) => "chord-normal",
                            Some(RetainedFilletRadialFrame2::ConcentricArc { .. }) => {
                                "concentric-arc"
                            }
                            Some(RetainedFilletRadialFrame2::SelectedConcentric { .. }) => {
                                "selected-concentric"
                            }
                            Some(RetainedFilletRadialFrame2::ParallelNormal { .. }) => {
                                "parallel-normal"
                            }
                            None => "none",
                        },
                    );
                    candidates.push(FilletCorner2 {
                        previous: previous_cut,
                        next: next_cut,
                        center: center.point.clone(),
                        clockwise,
                        retained_frame,
                    });
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        reason,
                    ));
                }
            }
        }
    }

    // An in-domain candidate that collapses is more specific than unrelated
    // support intersections outside the authored trims. In particular, it
    // must not be relabeled as an extendable trim-domain miss.
    let empty_reason = if saw_degenerate {
        CurveCornerNoSolution2::DegenerateCandidate
    } else if saw_outside_domain {
        CurveCornerNoSolution2::OutsideTrimDomain
    } else {
        CurveCornerNoSolution2::NoTangentCircle
    };
    Ok(candidates.finish(empty_reason))
}

fn retained_fillet_cusp_fragment_range(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
) -> crate::CurveParameterRange2 {
    crate::CurveParameterRange2::new_validated(
        CurveParameter2::from_algebraic_cusp(fragment.start_parameter().clone()),
        CurveParameter2::from_algebraic_cusp(fragment.end_parameter().clone()),
    )
}

fn retained_fillet_positive_overlap(
    result: crate::CurveResult<Classification<bool>>,
    family: CurveFamily2,
) -> ExactCurveResult<bool> {
    match result
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(positive) => Ok(positive),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

fn retained_fillet_cusp_mapped_overlap_is_positive(
    cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
    overlap: &crate::bezier_offset::BezierAlgebraicCuspSemicircleMappedOverlap2,
    other_range: &crate::CurveParameterRange2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    retained_fillet_positive_overlap(
        overlap.has_positive_overlap(
            &retained_fillet_cusp_fragment_range(cusp),
            other_range,
            policy,
        ),
        family,
    )
}

fn retained_fillet_cusp_pair_overlap_is_positive(
    first: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second: &crate::BezierAlgebraicCuspSemicircleFragment2,
    overlap: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairOverlap2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    retained_fillet_positive_overlap(
        overlap.has_positive_overlap(
            &retained_fillet_cusp_fragment_range(first),
            &retained_fillet_cusp_fragment_range(second),
            policy,
        ),
        family,
    )
}

/// Publishes a compact one-field center when one side of a direct circle pair
/// already has an exact rational image. Contact selection remains wholly owned
/// by the pair map; the temporary conic spans only encode the chosen point for
/// retained curve storage.
fn retained_fillet_pair_contact_rational_point(
    support: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
    parameter_map: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairParameterMap2,
    contact: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairContact2,
    first: bool,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurvePoint2>> {
    if !support.has_rational_frame() {
        return Ok(None);
    }
    let center = support
        .center_point_image(policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    let start = support
        .start_point_image(policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    let end = support
        .end_point_image(policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    let (Some(center), Some(start), Some(end)) = (
        center.exact_point(&CurveContext::STRICT),
        start.exact_point(&CurveContext::STRICT),
        end.exact_point(&CurveContext::STRICT),
    ) else {
        return Ok(None);
    };
    let arc = CircularArc2::try_from_center(start, end, center, support.is_clockwise())
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    retained_fillet_pair_contact_rational_point_on_arc(
        &arc,
        parameter_map,
        contact,
        first,
        family,
        policy,
    )
}

/// Publishes a selected-circle pair contact through an already-certified
/// rational chart of one supporting half circle. The circle-pair map remains
/// the sole root authority; this inverse chart only chooses the compact
/// one-field point carrier used by the retained CurveRegion boundary.
fn retained_fillet_pair_contact_rational_point_on_arc(
    arc: &CircularArc2,
    parameter_map: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairParameterMap2,
    contact: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairContact2,
    first: bool,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurvePoint2>> {
    let decomposition = match arc
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
    for span in decomposition.spans() {
        let curve: RationalBezier2 = span.curve().clone().into();
        let points = match parameter_map
            .rational_point_evidence_for_contact(contact, first, &curve, false, policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(points) => points,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    reason,
                ));
            }
        };
        if let Some(point) = points.into_iter().next() {
            return Ok(Some(point));
        }
    }
    Ok(None)
}

fn retained_fillet_incident_overlap_range(
    overlap: &crate::CurveParameterRange2,
    domain: &crate::bezier_offset::BezierParallelIncidentDomain2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<crate::CurveParameterRange2>> {
    let barrier = || {
        domain
            .barrier()
            .map(|parameter| CurveParameter2::from(parameter.clone()))
    };
    let (start, end) = match domain.direction() {
        crate::BezierParameterRayDirection2::Decreasing => (
            barrier().unwrap_or_else(|| overlap.start().clone()),
            domain.endpoint().clone(),
        ),
        crate::BezierParameterRayDirection2::Increasing => (
            domain.endpoint().clone(),
            barrier().unwrap_or_else(|| overlap.end().clone()),
        ),
    };
    Ok(
        match retained_fillet_curve_region_parameter_order(&start, &end, family, policy)? {
            std::cmp::Ordering::Equal => None,
            std::cmp::Ordering::Less | std::cmp::Ordering::Greater => {
                Some(crate::CurveParameterRange2::new_validated(start, end))
            }
        },
    )
}

fn retained_fillet_curve_region_parameter_order(
    first: &CurveParameter2,
    second: &CurveParameter2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<std::cmp::Ordering> {
    match first
        .cmp_by_refinement(second, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(order) => Ok(order),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

fn retained_fillet_corresponding_overlap_is_positive(
    first_curve: &RationalBezier2,
    second_curve: &RationalBezier2,
    overlap: &crate::RationalBezierIntersectionOverlap2,
    first_fragment: &crate::CurveParameterRange2,
    second_fragment: &crate::CurveParameterRange2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let correspondence = RationalBezierOverlapParameterCorrespondence2::for_overlap(
        first_curve,
        second_curve,
        overlap,
        policy,
    );
    match correspondence
        .clipped_ranges(
            overlap.first_range(),
            overlap.second_range(),
            first_fragment,
            second_fragment,
            policy,
        )
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(ranges) => Ok(ranges.is_some()),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

fn retained_fillet_parameter_component_overlap_is_positive(
    overlap: &crate::bezier_offset::BezierParameterComponentOverlap2,
    first_fragment: &crate::CurveParameterRange2,
    second_fragment: &crate::CurveParameterRange2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    match overlap
        .clipped_ranges(first_fragment, second_fragment, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(ranges) => Ok(ranges.is_some()),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

/// Clips a positive-dimensional mixed arc/selected-circle center component to
/// the two authored finite domains. Isolated centers are owned by the common
/// selected-circle pair kernel; rational arc cells survive here only as an
/// exact inverse-domain adapter for coincident supporting circles.
fn retained_fillet_arc_cusp_overlap_is_positive(
    arc_support: &CircularArc2,
    cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
    arc_family: CurveFamily2,
    cusp_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let range =
        crate::CurveParameterRange2::from_bezier_range(BezierParameterRange2::new_validated(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        ));
    for (cell, _, _, _) in retained_arc_fillet_projective_cells(
        arc_support,
        CurveCornerMode2::TrimOnly,
        arc_family,
        policy,
    )? {
        let (intersections, _) = match cusp
            .semicircle()
            .rational_intersections_with_parameter_map(&cell, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
            })? {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    cusp_family,
                    reason,
                ));
            }
        };
        match intersections {
            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiberOverlaps(overlaps) => {
                for overlap in overlaps {
                    if retained_selected_fillet_overlap_is_positive(
                        &overlap,
                        &range,
                        None,
                        cusp,
                        arc_family,
                        cusp_family,
                        policy,
                    )? {
                        return Ok(true);
                    }
                }
            }
            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Overlaps(overlaps) => {
                for overlap in overlaps {
                    let cell_overlap = crate::CurveParameterRange2::from_bezier_range(
                        overlap.other_range().clone(),
                    );
                    if retained_fillet_cusp_mapped_overlap_is_positive(
                        cusp,
                        &overlap,
                        &cell_overlap,
                        cusp_family,
                        policy,
                    )? {
                        return Ok(true);
                    }
                }
            }
            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Contacts(_)
            | crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiberContacts(_) => {}
            crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    cusp_family,
                    crate::UncertaintyReason::Unsupported,
                ));
            }
        }
    }
    Ok(false)
}

fn retained_fillet_parallel_support_reverses_source(
    source: &crate::BezierParallelFragment2,
    support: &BezierParallel2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let derivative_scale = |parallel: &BezierParallel2| match parallel
        .regular_fragment_derivative_scale_sign(source.range(), policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => Ok(sign),
        Classification::Decided(RealSign::Zero) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            crate::UncertaintyReason::Boundary,
        )),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    };
    Ok(
        (derivative_scale(source.parallel())? != derivative_scale(support)?)
            != source.is_reversed(),
    )
}

fn selected_fiber_parallel_derivative_scale_sign(
    parallel: &BezierParallel2,
    source: &crate::bezier_split::BezierSelectedFiberFragment2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<RealSign> {
    if parallel.has_exact_affine_line_parameterization() {
        return Ok(RealSign::Positive);
    }
    let parameter = match source
        .range()
        .start()
        .strict_scalar_between_ordered(source.range().end(), policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(parameter) => BezierParameter2::Exact(parameter),
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            ));
        }
    };
    match parallel
        .parallel_derivative_scale_sign(&parameter, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => Ok(sign),
        Classification::Decided(RealSign::Zero) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            crate::UncertaintyReason::Boundary,
        )),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

fn retained_fillet_parameter_is_in_open_range(
    parameter: &BezierParameter2,
    range: &BezierParameterRange2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    match crate::bezier_offset::overlap_parameter_is_in_range(parameter, range, false, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(inside) => Ok(inside),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

fn retained_selected_fillet_parameter_is_in_open_range(
    parameter: &BezierParameter2,
    range: &crate::bezier_split::CurveParameterRange2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let parameter = CurveParameter2::from(parameter.clone());
    let order = |boundary: &CurveParameter2| match parameter
        .cmp_by_refinement(boundary, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(order) => Ok(order),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    };
    Ok(order(range.start())?.is_gt() && order(range.end())?.is_lt())
}

fn retained_selected_fillet_overlap_is_positive(
    overlap: &crate::bezier_offset::BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2,
    analytic_range: &crate::CurveParameterRange2,
    incident_domain: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
    cusp_source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    analytic_family: CurveFamily2,
    cusp_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let cusp_range = retained_fillet_cusp_fragment_range(cusp_source);
    let overlaps_authored = retained_fillet_positive_overlap(
        overlap.has_positive_overlap(&cusp_range, analytic_range, policy),
        cusp_family,
    )?;
    if overlaps_authored {
        return Ok(true);
    }
    let Some(domain) = incident_domain else {
        return Ok(false);
    };
    let other_overlap = crate::CurveParameterRange2::new_validated(
        CurveParameter2::from_selected_fiber(overlap.other_start_parameter()),
        CurveParameter2::from_selected_fiber(overlap.other_end_parameter()),
    );
    let Some(incident_range) =
        retained_fillet_incident_overlap_range(&other_overlap, domain, analytic_family, policy)?
    else {
        return Ok(false);
    };
    retained_fillet_positive_overlap(
        overlap.has_positive_overlap(&cusp_range, &incident_range, policy),
        cusp_family,
    )
}

#[allow(clippy::too_many_arguments)]
fn retain_cusp_parallel_fillet_contact(
    centers: &mut FilletCenters2,
    cusp_source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    parallel_source: FilletParallelSource2<'_>,
    analytic_support: &BezierParallel2,
    cusp_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    analytic_parameter: CurveParameter2,
    point: CurvePoint2,
    location: crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2,
    mut cross: RealSign,
    mut dot: RealSign,
    complementary: bool,
    cusp_support_reverses_source: bool,
    analytic_support_reverses_source: bool,
    cusp_is_previous: bool,
    mode: CurveCornerMode2,
    analytic_domain: FilletContactDomain2,
    incident_domain: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
    cusp_family: CurveFamily2,
    analytic_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    if complementary
        && location != crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior
    {
        return Ok(());
    }
    if mode != CurveCornerMode2::TrimOrExtend {
        match cusp_source
            .contains_parameter(&cusp_parameter, false, false, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
            })? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(()),
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    cusp_family,
                    reason,
                ));
            }
        }
    }
    if !parallel_source.parameter_is_admissible(
        &analytic_parameter,
        !cusp_is_previous,
        mode,
        analytic_domain,
        incident_domain,
        analytic_family,
        policy,
    )? {
        return Ok(());
    }
    if cusp_support_reverses_source {
        cross = reverse_fillet_sign(cross);
        dot = reverse_fillet_sign(dot);
    }
    if analytic_support_reverses_source {
        cross = reverse_fillet_sign(cross);
        dot = reverse_fillet_sign(dot);
    }
    let cusp_parameter = if complementary {
        CurveParameter2::from_algebraic_cusp_complement(cusp_parameter)
    } else {
        CurveParameter2::from_algebraic_cusp(cusp_parameter)
    };
    let (previous_parameter, next_parameter) = if cusp_is_previous {
        (Some(cusp_parameter), Some(analytic_parameter.clone()))
    } else {
        (Some(analytic_parameter.clone()), Some(cusp_parameter))
    };
    centers.push(FilletCenterWitness2 {
        point,
        previous_parameter,
        next_parameter,
        // The retained frame is the analytic carrier, so store analytic x
        // cusp rather than the direct kernel's cusp x analytic relation.
        retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
            cross: Some(reverse_fillet_sign(cross)),
            dot: Some(dot),
            center_parallel: Some(RetainedFilletCenterParallel2 {
                support: analytic_support.clone(),
                parameter: Some(analytic_parameter),
            }),
            source_direction: Some(if analytic_support_reverses_source {
                RealSign::Negative
            } else {
                RealSign::Positive
            }),
            canonical_anchor_curve: None,
            deferred_arc_contact: None,
        }),
    });
    Ok(())
}

fn fillet_offset_centers(
    previous: &FilletOffsetCarrier2<'_, '_>,
    next: &FilletOffsetCarrier2<'_, '_>,
    mode: CurveCornerMode2,
    domains: [FilletContactDomain2; 2],
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<FilletCenters2> {
    let exact_parameter = |parameter| CurveParameter2::from(BezierParameter2::Exact(parameter));
    let mut centers = FilletCenters2::default();
    match (previous, next) {
        (FilletOffsetCarrier2::Point { .. }, _) | (_, FilletOffsetCarrier2::Point { .. }) => {
            let (point, other) = match (previous, next) {
                (FilletOffsetCarrier2::Point { point }, other) => (point, other),
                (other, FilletOffsetCarrier2::Point { point }) => (point, other),
                _ => unreachable!(),
            };
            let other_family = if matches!(previous, FilletOffsetCarrier2::Point { .. }) {
                next_family
            } else {
                previous_family
            };
            if point_on_fillet_offset(point, other, other_family, policy)? {
                // The center is isolated, but tangency on the collapsed source
                // offset is not. Do not manufacture one contact from a
                // continuum of equally valid source-circle contacts.
                centers.coincident = true;
            }
        }
        (FilletOffsetCarrier2::Line { .. }, FilletOffsetCarrier2::Arc { .. })
        | (FilletOffsetCarrier2::Arc { .. }, FilletOffsetCarrier2::Line { .. }) => {
            let (support, line_source, source, signed_radius, line_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::Line {
                            source: line_source,
                            support,
                            ..
                        },
                        FilletOffsetCarrier2::Arc {
                            source,
                            signed_radius,
                            ..
                        },
                    ) => (support, line_source, source, signed_radius, true),
                    (
                        FilletOffsetCarrier2::Arc {
                            source,
                            signed_radius,
                            ..
                        },
                        FilletOffsetCarrier2::Line {
                            source: line_source,
                            support,
                            ..
                        },
                    ) => (support, line_source, source, signed_radius, false),
                    _ => unreachable!(),
                };
            let relation = crate::intersect::line_circle_relation_from_supports(
                support,
                source.support().center(),
                &(signed_radius * signed_radius),
                policy,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
            })?;
            let mut push = |point: Point2, parameter: Real| {
                let parameter = line_source
                    .native_line()
                    .is_some()
                    .then(|| exact_parameter(parameter));
                let (previous_parameter, next_parameter) = if line_is_previous {
                    (parameter, None)
                } else {
                    (None, parameter)
                };
                centers.push(FilletCenterWitness2 {
                    point: point.into(),
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence: None,
                });
            };
            match relation {
                crate::LineCircleRelation::Disjoint => {}
                crate::LineCircleRelation::Tangent { point, line_param } => {
                    push(point, line_param);
                }
                crate::LineCircleRelation::Secant {
                    first_point,
                    first_param,
                    second_point,
                    second_param,
                } => {
                    push(first_point, first_param);
                    push(second_point, second_param);
                }
                crate::LineCircleRelation::Uncertain { reason } => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        reason,
                    ));
                }
            }
        }
        (
            FilletOffsetCarrier2::Arc {
                source: previous,
                signed_radius: previous_radius,
                ..
            },
            FilletOffsetCarrier2::Arc {
                source: next,
                signed_radius: next_radius,
                ..
            },
        ) => match crate::intersect::circle_relation_from_supports(
            previous.support().center(),
            &(previous_radius * previous_radius),
            next.support().center(),
            &(next_radius * next_radius),
            policy,
        )
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
        })? {
            crate::CircleCircleRelation::Disjoint => {}
            crate::CircleCircleRelation::Tangent { point } => {
                centers.push(FilletCenterWitness2 {
                    point: point.into(),
                    previous_parameter: None,
                    next_parameter: None,
                    retained_anchor_evidence: None,
                });
            }
            crate::CircleCircleRelation::Secant {
                first_point,
                second_point,
            } => {
                centers.push(FilletCenterWitness2 {
                    point: first_point.into(),
                    previous_parameter: None,
                    next_parameter: None,
                    retained_anchor_evidence: None,
                });
                centers.push(FilletCenterWitness2 {
                    point: second_point.into(),
                    previous_parameter: None,
                    next_parameter: None,
                    retained_anchor_evidence: None,
                });
            }
            crate::CircleCircleRelation::Coincident => centers.coincident = true,
            crate::CircleCircleRelation::Uncertain { reason } => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    previous_family,
                    reason,
                ));
            }
        },
        (
            FilletOffsetCarrier2::Arc { .. },
            FilletOffsetCarrier2::Parallel {
                source: parallel_source,
                ..
            },
        )
        | (
            FilletOffsetCarrier2::Parallel {
                source: parallel_source,
                ..
            },
            FilletOffsetCarrier2::Arc { .. },
        ) => {
            let (arc, source_radius, signed_radius, bezier, bezier_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::Arc {
                            source,
                            source_radius,
                            signed_radius,
                        },
                        FilletOffsetCarrier2::Parallel {
                            support: bezier, ..
                        },
                    ) => (source, source_radius, signed_radius, bezier, false),
                    (
                        FilletOffsetCarrier2::Parallel {
                            support: bezier, ..
                        },
                        FilletOffsetCarrier2::Arc {
                            source,
                            source_radius,
                            signed_radius,
                        },
                    ) => (source, source_radius, signed_radius, bezier, true),
                    _ => unreachable!(),
                };
            let bezier_family = if bezier_is_previous {
                previous_family
            } else {
                next_family
            };
            let incident_domain = if mode == CurveCornerMode2::TrimOrExtend {
                Some(parallel_source.incident_domain(
                    bezier,
                    bezier_is_previous,
                    bezier_family,
                    policy,
                )?)
            } else {
                None
            };
            let mut parameters = match bezier
                .circle_incidence(
                    arc.support().center(),
                    &(signed_radius * signed_radius),
                    &[],
                    policy,
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, bezier_family, cause)
                })? {
                Classification::Decided(parameters) => parameters,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        bezier_family,
                        reason,
                    ));
                }
            };
            if let Some(domain) = incident_domain.as_ref() {
                match bezier
                    .circle_incidence_on_incident_ray(
                        arc.support().center(),
                        &(signed_radius * signed_radius),
                        domain,
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, bezier_family, cause)
                    })? {
                    Classification::Decided(exterior) => parameters.extend(exterior),
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            bezier_family,
                            reason,
                        ));
                    }
                }
            }
            for (parameter, _) in parameters {
                if !parallel_source.bezier_parameter_is_admissible(
                    &parameter,
                    bezier_is_previous,
                    mode,
                    domains[usize::from(!bezier_is_previous)],
                    incident_domain.as_ref(),
                    bezier_family,
                    policy,
                )? {
                    continue;
                }
                let point = analytic_parallel_point_evidence(
                    bezier,
                    &parameter.clone().into(),
                    CurveOperation2::Fillet,
                    bezier_family,
                    policy,
                )?;
                let retained_anchor_evidence = Some(RetainedFilletAnchorEvidence2 {
                    cross: None,
                    dot: None,
                    center_parallel: None,
                    source_direction: None,
                    canonical_anchor_curve: None,
                    deferred_arc_contact: Some(RetainedDeferredArcFilletContact2 {
                        support: arc.support().clone(),
                        source_radius: (*source_radius).clone(),
                        signed_center_radius: signed_radius.clone(),
                        arc_is_previous: !bezier_is_previous,
                        selected_center: None,
                        contact_seed: None,
                    }),
                });
                let bezier_parameter = CurveParameter2::from(parameter);
                centers.push(FilletCenterWitness2 {
                    point,
                    previous_parameter: bezier_is_previous.then(|| bezier_parameter.clone()),
                    next_parameter: (!bezier_is_previous).then_some(bezier_parameter),
                    retained_anchor_evidence,
                });
            }
        }
        (
            FilletOffsetCarrier2::Parallel {
                source: previous_source,
                support: previous,
            },
            FilletOffsetCarrier2::Parallel {
                source: next_source,
                support: next,
            },
        ) => {
            let identical_supports = previous == next;
            let use_incident_rays = mode == CurveCornerMode2::TrimOrExtend;
            let previous_incident_domain = if use_incident_rays {
                Some(previous_source.incident_domain(previous, true, previous_family, policy)?)
            } else {
                None
            };
            let next_incident_domain = if use_incident_rays {
                Some(next_source.incident_domain(next, false, next_family, policy)?)
            } else {
                None
            };
            let (intersections, positive_dimensional_incident_seam) = if use_incident_rays {
                let extensions = [
                    previous_incident_domain.as_ref(),
                    next_incident_domain.as_ref(),
                ]
                .map(|domain| domain.map(|domain| domain.parameter_ray()));
                let incident = match (if identical_supports {
                    previous.ordered_self_intersections_in_domain(extensions, policy)
                } else {
                    previous.parallel_intersections_in_domain(next, extensions, policy)
                })
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
                })? {
                    Classification::Decided(intersections) => intersections,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            previous_family,
                            reason,
                        ));
                    }
                };
                incident.into_parts()
            } else {
                let intersections = match (if identical_supports {
                    previous.self_intersections(policy)
                } else {
                    previous.parallel_intersections(next, policy)
                })
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
                })? {
                    Classification::Decided(intersections) => intersections,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            previous_family,
                            reason,
                        ));
                    }
                };
                (intersections, false)
            };
            if !intersections.is_complete() {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    previous_family,
                    crate::UncertaintyReason::Predicate,
                ));
            }
            centers.coincident |= positive_dimensional_incident_seam;
            let previous_curve_range = previous_source.curve_parameter_range();
            let next_curve_range = next_source.curve_parameter_range();
            let has_selected_range = previous_source.parameter_range().is_none()
                || next_source.parameter_range().is_none();
            if !intersections.overlaps().is_empty() {
                let mut rational_sources = None;
                for overlap in intersections.overlaps() {
                    let mut correspondence_error = None;
                    if has_selected_range {
                        let corresponding = (|| {
                            if rational_sources.is_none() {
                                rational_sources = Some((
                                    bezier_parallel_rational_source(
                                        previous,
                                        CurveOperation2::Fillet,
                                        previous_family,
                                    )?,
                                    bezier_parallel_rational_source(
                                        next,
                                        CurveOperation2::Fillet,
                                        next_family,
                                    )?,
                                ));
                            }
                            let (previous_curve, next_curve) = rational_sources
                                .as_ref()
                                .expect("fillet rational sources were initialized");
                            retained_fillet_corresponding_overlap_is_positive(
                                previous_curve,
                                next_curve,
                                overlap,
                                &previous_curve_range,
                                &next_curve_range,
                                previous_family,
                                policy,
                            )
                        })();
                        match corresponding {
                            Ok(positive) => {
                                centers.coincident |= positive;
                                continue;
                            }
                            Err(error) => correspondence_error = Some(error),
                        }
                    }
                    let mut has_component = false;
                    let mut positive = false;
                    for component in intersections
                        .component_overlaps()
                        .iter()
                        .filter(|source| source.overlap() == overlap)
                    {
                        has_component = true;
                        if retained_fillet_parameter_component_overlap_is_positive(
                            component,
                            &previous_curve_range,
                            &next_curve_range,
                            previous_family,
                            policy,
                        )? {
                            positive = true;
                            break;
                        }
                    }
                    if !has_component {
                        if let Some(error) = correspondence_error {
                            return Err(error);
                        }
                        if rational_sources.is_none() {
                            rational_sources = Some((
                                bezier_parallel_rational_source(
                                    previous,
                                    CurveOperation2::Fillet,
                                    previous_family,
                                )?,
                                bezier_parallel_rational_source(
                                    next,
                                    CurveOperation2::Fillet,
                                    next_family,
                                )?,
                            ));
                        }
                        let (previous_curve, next_curve) = rational_sources
                            .as_ref()
                            .expect("fillet rational sources were initialized");
                        positive = retained_fillet_corresponding_overlap_is_positive(
                            previous_curve,
                            next_curve,
                            overlap,
                            &previous_curve_range,
                            &next_curve_range,
                            previous_family,
                            policy,
                        )?;
                    }
                    if positive {
                        centers.coincident = true;
                        break;
                    }
                }
            }
            for component in intersections.parameter_components() {
                let previous_inside = match component.first_parameter() {
                    Some(parameter) => previous_source.parameter_is_in_open_range(
                        parameter,
                        previous_family,
                        policy,
                    )?,
                    None => true,
                };
                let next_inside = match component.second_parameter() {
                    Some(parameter) => {
                        next_source.parameter_is_in_open_range(parameter, next_family, policy)?
                    }
                    None => true,
                };
                if previous_inside && next_inside {
                    centers.coincident = true;
                    break;
                }
            }
            let previous_support_reverses_source =
                previous_source.support_reverses_source(previous, previous_family, policy)?;
            let next_support_reverses_source =
                next_source.support_reverses_source(next, next_family, policy)?;
            let reverse_tangent_relation =
                previous_support_reverses_source != next_support_reverses_source;
            for contact in intersections.contacts() {
                let point = analytic_parallel_point_evidence(
                    previous,
                    &contact.first_parameter().clone().into(),
                    CurveOperation2::Fillet,
                    previous_family,
                    policy,
                )?;
                // Self-contact publication is unordered. Both parameter-role
                // assignments are mathematically distinct at a closed seam;
                // the retained interval authority subsequently keeps only the
                // assignment whose cuts bound disjoint seam-side intervals.
                for swapped in 0..=usize::from(identical_supports) {
                    let swapped = swapped != 0;
                    let (previous_parameter, next_parameter) = if swapped {
                        (contact.second_parameter(), contact.first_parameter())
                    } else {
                        (contact.first_parameter(), contact.second_parameter())
                    };
                    if !previous_source.bezier_parameter_is_admissible(
                        previous_parameter,
                        true,
                        mode,
                        domains[0],
                        previous_incident_domain.as_ref(),
                        previous_family,
                        policy,
                    )? || !next_source.bezier_parameter_is_admissible(
                        next_parameter,
                        false,
                        mode,
                        domains[1],
                        next_incident_domain.as_ref(),
                        next_family,
                        policy,
                    )? {
                        continue;
                    }
                    let orient = |sign| {
                        if swapped != reverse_tangent_relation {
                            reverse_fillet_sign(sign)
                        } else {
                            sign
                        }
                    };
                    centers.push(FilletCenterWitness2 {
                        point: point.clone(),
                        previous_parameter: Some(CurveParameter2::from(previous_parameter.clone())),
                        next_parameter: Some(CurveParameter2::from(next_parameter.clone())),
                        retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                            cross: contact.tangent_cross_sign().map(orient),
                            dot: contact.tangent_dot_sign().map(|sign| {
                                if reverse_tangent_relation {
                                    reverse_fillet_sign(sign)
                                } else {
                                    sign
                                }
                            }),
                            center_parallel: None,
                            source_direction: None,
                            canonical_anchor_curve: None,
                            deferred_arc_contact: None,
                        }),
                    });
                }
            }
        }
        (FilletOffsetCarrier2::Line { .. }, FilletOffsetCarrier2::Parallel { .. })
        | (FilletOffsetCarrier2::Parallel { .. }, FilletOffsetCarrier2::Line { .. }) => {
            let (line, line_source, line_unit_x, line_unit_y, parallel, line_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::Line {
                            source,
                            support,
                            unit_x,
                            unit_y,
                            ..
                        },
                        parallel @ FilletOffsetCarrier2::Parallel { .. },
                    ) => (support, source, *unit_x, *unit_y, parallel, true),
                    (
                        parallel @ FilletOffsetCarrier2::Parallel { .. },
                        FilletOffsetCarrier2::Line {
                            source,
                            support,
                            unit_x,
                            unit_y,
                            ..
                        },
                    ) => (support, source, *unit_x, *unit_y, parallel, false),
                    _ => unreachable!(),
                };
            let FilletOffsetCarrier2::Parallel { source, support } = parallel else {
                unreachable!()
            };
            let parallel_family = if line_is_previous {
                next_family
            } else {
                previous_family
            };
            let line_endpoint = if line_is_previous {
                BezierEndpoint::End
            } else {
                BezierEndpoint::Start
            };
            let certified_tangency = source.retained().and_then(|source| {
                let corner_parameter = if line_is_previous == source.is_reversed() {
                    source.range().end()
                } else {
                    source.range().start()
                };
                corner_parameter.scalar().and_then(|parameter| {
                    line_source
                        .parallel_tangent_contacts()
                        .iter()
                        .find(|contact| {
                            contact.line_endpoint() == line_endpoint
                                && contact.parallel() == source.parallel()
                                && contact.parallel_fragment_reversed() == source.is_reversed()
                                && contact.parameter() == parameter
                        })
                })
            });
            let certified_tangencies = certified_tangency
                .map(|contact| std::slice::from_ref(contact.parameter()))
                .unwrap_or_default();
            let parallel_is_previous = !line_is_previous;
            let incident_domain = if mode == CurveCornerMode2::TrimOrExtend {
                Some(source.incident_domain(
                    support,
                    parallel_is_previous,
                    parallel_family,
                    policy,
                )?)
            } else {
                None
            };
            let mut parameters = match support
                .supporting_line_incidence_with_direction(
                    line,
                    line_unit_x,
                    line_unit_y,
                    certified_tangencies,
                    policy,
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, parallel_family, cause)
                })? {
                Classification::Decided(crate::BezierParallelIncidence2::EntireCurve) => {
                    centers.coincident = true;
                    Vec::new()
                }
                Classification::Decided(crate::BezierParallelIncidence2::Parameters(
                    parameters,
                )) => parameters,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        parallel_family,
                        reason,
                    ));
                }
            };
            if let Some(domain) = incident_domain.as_ref() {
                match support
                    .supporting_line_incidence_on_incident_ray_with_direction(
                        line,
                        line_unit_x,
                        line_unit_y,
                        domain,
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, parallel_family, cause)
                    })? {
                    Classification::Decided(crate::BezierParallelIncidence2::EntireCurve) => {
                        centers.coincident = true;
                    }
                    Classification::Decided(crate::BezierParallelIncidence2::Parameters(
                        exterior,
                    )) => parameters.extend(exterior),
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            parallel_family,
                            reason,
                        ));
                    }
                }
            }
            for parameter in parameters {
                if !source.bezier_parameter_is_admissible(
                    &parameter,
                    parallel_is_previous,
                    mode,
                    domains[usize::from(!parallel_is_previous)],
                    incident_domain.as_ref(),
                    parallel_family,
                    policy,
                )? {
                    continue;
                }
                // A selected center already owns its exact point and normal.
                // Recover the line cut from that evidence in either mode;
                // rebuilding its scalar image can force an unnecessary
                // resultant over nonrational source coefficients.
                let procedural_affine_contact = parameter.scalar().is_none();
                let line_parameter = if procedural_affine_contact {
                    None
                } else {
                    let contact = if mode == CurveCornerMode2::TrimOrExtend {
                        support.supporting_line_contact_evidence_affine(line, &parameter, policy)
                    } else {
                        support.supporting_line_contact_evidence(line, &parameter, policy)
                    };
                    let (_, line_parameter) = match contact.map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, parallel_family, cause)
                    })? {
                        Classification::Decided(contact) => contact,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                parallel_family,
                                reason,
                            ));
                        }
                    };
                    line_parameter
                };
                // The selected center is natively one point of this analytic
                // parallel. Keep that one-parameter authority and use the
                // line solve only for its affine cut parameter; publishing an
                // independent Cartesian algebraic image here would force
                // later circle/chord replay to prove equality across two
                // avoidable coordinate constructions.
                let point = analytic_parallel_point_evidence(
                    support,
                    &parameter.clone().into(),
                    CurveOperation2::Fillet,
                    parallel_family,
                    policy,
                )?;
                let line_parameter =
                    if line_source.algebraic_chord().is_some() || procedural_affine_contact {
                        None
                    } else {
                        let Some(parameter) = line_parameter else {
                            continue;
                        };
                        Some(CurveParameter2::from(parameter))
                    };
                let retained_anchor_evidence = {
                    let (mut cross, mut dot) = match support
                        .vector_tangent_cross_and_dot_signs(
                            &parameter,
                            line_unit_x,
                            line_unit_y,
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Fillet,
                                parallel_family,
                                cause,
                            )
                        })? {
                        Classification::Decided(signs) => signs,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                parallel_family,
                                reason,
                            ));
                        }
                    };
                    let support_reverses_source =
                        source.support_reverses_source(support, parallel_family, policy)?;
                    if support_reverses_source {
                        cross = reverse_fillet_sign(cross);
                        dot = reverse_fillet_sign(dot);
                    }
                    Some(RetainedFilletAnchorEvidence2 {
                        // The selected analytic circle frame is the anchor;
                        // the predicate above reports line x analytic.
                        cross: Some(reverse_fillet_sign(cross)),
                        dot: Some(dot),
                        center_parallel: None,
                        source_direction: Some(if support_reverses_source {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        }),
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    })
                };
                let parallel_parameter = Some(CurveParameter2::from(parameter));
                let (previous_parameter, next_parameter) = if line_is_previous {
                    (line_parameter, parallel_parameter)
                } else {
                    (parallel_parameter, line_parameter)
                };
                centers.push(FilletCenterWitness2 {
                    point,
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence,
                });
            }
        }
        (FilletOffsetCarrier2::AlgebraicCusp { .. }, FilletOffsetCarrier2::Parallel { .. })
        | (FilletOffsetCarrier2::Parallel { .. }, FilletOffsetCarrier2::AlgebraicCusp { .. }) => {
            let (cusp_source, cusp_support, parallel_source, analytic_support, cusp_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::AlgebraicCusp { source, support },
                        FilletOffsetCarrier2::Parallel {
                            source: analytic,
                            support: analytic_support,
                        },
                    ) => (source, support, analytic, analytic_support, true),
                    (
                        FilletOffsetCarrier2::Parallel {
                            source: analytic,
                            support: analytic_support,
                        },
                        FilletOffsetCarrier2::AlgebraicCusp { source, support },
                    ) => (source, support, analytic, analytic_support, false),
                    _ => unreachable!(),
                };
            let cusp_family = if cusp_is_previous {
                previous_family
            } else {
                next_family
            };
            let analytic_family = if cusp_is_previous {
                next_family
            } else {
                previous_family
            };
            let analytic_authored_range = parallel_source.curve_parameter_range();
            let analytic_range = parallel_source.intersection_parameter_range(analytic_family)?;
            let cusp_support_reverses_source = retained_fillet_cusp_support_reverses_source(
                cusp_source,
                cusp_support,
                cusp_family,
                policy,
            )?;
            let analytic_support_reverses_source = parallel_source.support_reverses_source(
                analytic_support,
                analytic_family,
                policy,
            )?;
            let incident_domain = if mode == CurveCornerMode2::TrimOrExtend {
                Some(parallel_source.incident_domain(
                    analytic_support,
                    !cusp_is_previous,
                    analytic_family,
                    policy,
                )?)
            } else {
                None
            };
            let projected_range = incident_domain.as_ref().map_or_else(
                || analytic_range.clone(),
                |domain| domain.expanded_range(&analytic_range),
            );
            let complementary_support = (mode == CurveCornerMode2::TrimOrExtend)
                .then(|| cusp_support.semicircle().complementary_half());
            // Every selected-circle frame enters the same complete
            // circle/parallel authority. Extension changes only the chart
            // domains: the authored half owns both diameter endpoints, its
            // complement owns neither, and the analytic carrier contributes
            // only its regular incident ray beyond the authored range.
            for (cusp_circle, complementary) in std::iter::once((cusp_support.semicircle(), false))
                .chain(complementary_support.as_ref().map(|circle| (circle, true)))
            {
                let result = if let Some(domain) = incident_domain.as_ref() {
                    cusp_circle.parallel_intersections_with_incident_ray(
                        analytic_support,
                        &projected_range,
                        domain,
                        policy,
                    )
                } else {
                    cusp_circle.parallel_intersections_in_range(
                        analytic_support,
                        &analytic_range,
                        policy,
                    )
                };
                let intersections = match result.map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
                })? {
                    Classification::Decided(intersections) => intersections,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            cusp_family,
                            reason,
                        ));
                    }
                };
                match intersections {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiberContacts(contacts) => {
                        for contact in contacts {
                            let dot = match contact
                                .tangent_dot_sign(policy)
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        analytic_family,
                                        cause,
                                    )
                                })? {
                                Classification::Decided(sign) => sign,
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        analytic_family,
                                        reason,
                                    ));
                                }
                            };
                            retain_cusp_parallel_fillet_contact(
                                &mut centers,
                                cusp_source,
                                *parallel_source,
                                analytic_support,
                                contact.cusp_parameter(),
                                CurveParameter2::from_selected_fiber(
                                    contact.other_parameter().clone(),
                                ),
                                contact.point_evidence(),
                                contact.location(),
                                contact.tangent_cross_sign(),
                                dot,
                                complementary,
                                cusp_support_reverses_source,
                                analytic_support_reverses_source,
                                cusp_is_previous,
                                mode,
                                domains[usize::from(cusp_is_previous)],
                                incident_domain.as_ref(),
                                cusp_family,
                                analytic_family,
                                policy,
                            )?;
                        }
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(contacts) => {
                        for contact in contacts {
                            retain_cusp_parallel_fillet_contact(
                                &mut centers,
                                cusp_source,
                                *parallel_source,
                                analytic_support,
                                contact.cusp_parameter(),
                                contact.other_parameter().clone(),
                                contact.point_evidence(),
                                contact.location(),
                                contact.tangent_cross_sign(),
                                contact.tangent_dot_sign(),
                                complementary,
                                cusp_support_reverses_source,
                                analytic_support_reverses_source,
                                cusp_is_previous,
                                mode,
                                domains[usize::from(cusp_is_previous)],
                                incident_domain.as_ref(),
                                cusp_family,
                                analytic_family,
                                policy,
                            )?;
                        }
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiberOverlaps(overlaps) => {
                        for overlap in overlaps {
                            if retained_selected_fillet_overlap_is_positive(
                                &overlap,
                                &analytic_authored_range,
                                incident_domain.as_ref(),
                                cusp_source,
                                analytic_family,
                                cusp_family,
                                policy,
                            )? {
                                centers.coincident = true;
                                break;
                            }
                        }
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::Contacts(contacts) => {
                        let parameter_map = match cusp_circle
                            .parallel_parameter_map(analytic_support, policy)
                            .map_err(|cause| {
                                ExactCurveError::invalid(
                                    CurveOperation2::Fillet,
                                    cusp_family,
                                    cause,
                                )
                            })? {
                            Classification::Decided(map) => map,
                            Classification::Uncertain(reason) => {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    cusp_family,
                                    reason,
                                ));
                            }
                        };
                        for contact in contacts {
                            if (complementary
                                && contact.location
                                    != crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior)
                                || !parallel_source.bezier_parameter_is_admissible(
                                &contact.parallel_parameter,
                                !cusp_is_previous,
                                mode,
                                domains[usize::from(cusp_is_previous)],
                                incident_domain.as_ref(),
                                analytic_family,
                                policy,
                            )? {
                                continue;
                            }
                            let cusp_parameter = parameter_map.contact_parameter(&contact);
                            if mode != CurveCornerMode2::TrimOrExtend {
                                match cusp_source
                                    .contains_parameter(&cusp_parameter, false, false, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            cusp_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(true) => {}
                                    Classification::Decided(false) => continue,
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            cusp_family,
                                            reason,
                                        ));
                                    }
                                }
                            }
                            let mut cross = contact.tangent_cross_sign.ok_or_else(|| {
                                ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    analytic_family,
                                    crate::UncertaintyReason::Predicate,
                                )
                            })?;
                            let mut dot = match cusp_circle
                                .parallel_contact_tangent_dot_sign(
                                    analytic_support,
                                    &contact,
                                    policy,
                                )
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        analytic_family,
                                        cause,
                                    )
                                })? {
                                Classification::Decided(sign) => sign,
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        analytic_family,
                                        reason,
                                    ));
                                }
                            };
                            if cusp_support_reverses_source {
                                cross = reverse_fillet_sign(cross);
                                dot = reverse_fillet_sign(dot);
                            }
                            if analytic_support_reverses_source {
                                cross = reverse_fillet_sign(cross);
                                dot = reverse_fillet_sign(dot);
                            }
                            let point = match cusp_parameter
                                .coincident_point_evidence(cusp_circle, policy)
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        cusp_family,
                                        cause,
                                    )
                                })? {
                                Classification::Decided(Some(point)) => point,
                                Classification::Decided(None) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        cusp_family,
                                        crate::UncertaintyReason::Unsupported,
                                    ));
                                }
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        cusp_family,
                                        reason,
                                    ));
                                }
                            };
                            let cusp_parameter = if complementary {
                                CurveParameter2::from_algebraic_cusp_complement(
                                    cusp_parameter,
                                )
                            } else {
                                CurveParameter2::from_algebraic_cusp(cusp_parameter)
                            };
                            let analytic_parameter = CurveParameter2::from(
                                contact.parallel_parameter,
                            );
                            let (previous_parameter, next_parameter) = if cusp_is_previous {
                                (Some(cusp_parameter), Some(analytic_parameter.clone()))
                            } else {
                                (Some(analytic_parameter.clone()), Some(cusp_parameter))
                            };
                            centers.push(FilletCenterWitness2 {
                                point,
                                previous_parameter,
                                next_parameter,
                                retained_anchor_evidence: Some(
                                    RetainedFilletAnchorEvidence2 {
                                        cross: Some(reverse_fillet_sign(cross)),
                                        dot: Some(dot),
                                        center_parallel: Some(RetainedFilletCenterParallel2 {
                                            support: analytic_support.clone(),
                                            parameter: Some(analytic_parameter),
                                        }),
                                        source_direction: Some(
                                            if analytic_support_reverses_source {
                                                RealSign::Negative
                                            } else {
                                                RealSign::Positive
                                            },
                                        ),
                                        canonical_anchor_curve: None,
                                        deferred_arc_contact: None,
                                    },
                                ),
                            });
                        }
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::Overlaps(overlaps) => {
                        for overlap in overlaps {
                            let overlaps_authored = retained_fillet_cusp_mapped_overlap_is_positive(
                                cusp_source,
                                &overlap,
                                &analytic_authored_range,
                                cusp_family,
                                policy,
                            )?;
                            let overlaps_incident = if let Some(domain) = incident_domain.as_ref() {
                                let other_overlap = crate::CurveParameterRange2::from_bezier_range(
                                    overlap.other_range().clone(),
                                );
                                let incident_range = retained_fillet_incident_overlap_range(
                                    &other_overlap,
                                    domain,
                                    analytic_family,
                                    policy,
                                )?;
                                match incident_range {
                                    Some(incident_range) => retained_fillet_cusp_mapped_overlap_is_positive(
                                        cusp_source,
                                        &overlap,
                                        &incident_range,
                                        cusp_family,
                                        policy,
                                    )?,
                                    None => false,
                                }
                            } else {
                                false
                            };
                            if overlaps_authored || overlaps_incident {
                                centers.coincident = true;
                                break;
                            }
                        }
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent => {
                        centers.coincident = true;
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            cusp_family,
                            crate::UncertaintyReason::Predicate,
                        ));
                    }
                }
            }
            return Ok(centers);
        }
        (FilletOffsetCarrier2::Arc { .. }, FilletOffsetCarrier2::AlgebraicCusp { .. })
        | (FilletOffsetCarrier2::AlgebraicCusp { .. }, FilletOffsetCarrier2::Arc { .. }) => {
            let (arc, source_radius, signed_radius, cusp_source, cusp, arc_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::Arc {
                            source,
                            source_radius,
                            signed_radius,
                        },
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source: cusp_source,
                            support,
                        },
                    ) => (
                        (*source),
                        *source_radius,
                        signed_radius,
                        *cusp_source,
                        support,
                        true,
                    ),
                    (
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source: cusp_source,
                            support,
                        },
                        FilletOffsetCarrier2::Arc {
                            source,
                            source_radius,
                            signed_radius,
                        },
                    ) => (
                        (*source),
                        *source_radius,
                        signed_radius,
                        *cusp_source,
                        support,
                        false,
                    ),
                    _ => unreachable!(),
                };
            let arc_family = if arc_is_previous {
                previous_family
            } else {
                next_family
            };
            let cusp_family = if arc_is_previous {
                next_family
            } else {
                previous_family
            };
            let offset_support = arc.concentric_offset_support(
                source_radius,
                signed_radius,
                CurveOperation2::Fillet,
                arc_family,
            )?;
            let signed_radius_sign = match crate::classify::real_sign(signed_radius, policy) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => unreachable!("collapsed arc offsets use the point carrier"),
                None => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        arc_family,
                        crate::UncertaintyReason::RealSign,
                    ));
                }
            };
            let cusp_support_reverses_source = retained_fillet_cusp_support_reverses_source(
                cusp_source,
                cusp,
                cusp_family,
                policy,
            )?;
            // A concentric arc offset is one exact circle, independent of the
            // authored rational parameterization. Give it a fixed +x radial
            // chart and solve both half charts against the selected cusp in
            // the same rank-independent circle-pair authority used by every
            // selected/selected carrier combination. The rational arc cells
            // below survive only as deferred inverse-domain maps.
            let center = arc.support().center();
            let axis_source = QuadraticBezier2::new(
                center.clone(),
                center.translated(Real::zero(), Real::from(-1_i8)),
                center.translated(Real::zero(), Real::from(-2_i8)),
            );
            let axis_parallel = axis_source.parallel_left(Real::zero()).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
            })?;
            let offset_circle = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                axis_parallel,
                BezierParameter2::Exact(Real::zero()),
                signed_radius.clone(),
                arc.support().is_clockwise(),
                policy,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
            })? {
                Classification::Decided(Some(circle)) => circle,
                Classification::Decided(None) => {
                    unreachable!("the nonzero concentric arc offset defines a circle")
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        arc_family,
                        reason,
                    ));
                }
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-arc-algebraic-cusp",
                "selected-circle-pair-authority",
            );

            let offset_complement = offset_circle.complementary_half();
            let cusp_complement = (mode == CurveCornerMode2::TrimOrExtend)
                .then(|| cusp.semicircle().complementary_half());
            let cusp_charts = [
                Some((cusp.semicircle(), false)),
                cusp_complement.as_ref().map(|circle| (circle, true)),
            ];
            let chart_owns_endpoint = |complementary: bool, location| {
                !complementary
                    || location
                        == crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            let endpoint_parameter = |location| match location {
                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Start => {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                        Real::zero(),
                    )
                }
                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End => {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one())
                }
                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
                    unreachable!("endpoint-only pair contact was interior")
                }
            };
            let reverse_pair_relation =
                cusp_support_reverses_source != (signed_radius_sign == RealSign::Negative);
            for (arc_circle, arc_complementary) in
                [(&offset_circle, false), (&offset_complement, true)]
            {
                let radial = arc_circle.radial_distance();
                let rational_half = CircularArc2::try_from_center(
                    center.translated(radial.clone(), Real::zero()),
                    center.translated(-radial.clone(), Real::zero()),
                    center.clone(),
                    arc_circle.is_clockwise(),
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
                })?;
                for (cusp_circle, cusp_complementary) in cusp_charts.iter().flatten().copied() {
                    let intersections = match arc_circle
                        .pair_intersections(cusp_circle, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
                        })? {
                        Classification::Decided(intersections) => intersections,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                arc_family,
                                reason,
                            ));
                        }
                    };
                    let mut chart_contacts = Vec::with_capacity(2);
                    match intersections {
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts => {}
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                            contacts,
                            parameter_map,
                        } => {
                            for contact in contacts {
                                if !chart_owns_endpoint(
                                    arc_complementary,
                                    contact.first_location,
                                ) || !chart_owns_endpoint(
                                    cusp_complementary,
                                    contact.second_location,
                                ) {
                                    continue;
                                }
                                let tangent_cross = contact.tangent_cross_sign;
                                let tangent_dot = if tangent_cross == RealSign::Zero {
                                    Some(match parameter_map
                                        .tangent_dot_sign(&contact, policy)
                                        .map_err(|cause| {
                                            ExactCurveError::invalid(
                                                CurveOperation2::Fillet,
                                                arc_family,
                                                cause,
                                            )
                                        })? {
                                        Classification::Decided(sign) => sign,
                                        Classification::Uncertain(reason) => {
                                            return Err(ExactCurveError::blocked(
                                                CurveOperation2::Fillet,
                                                arc_family,
                                                reason,
                                            ));
                                        }
                                    })
                                } else {
                                    None
                                };
                                let arc_parameter =
                                    parameter_map.first_contact_parameter(&contact);
                                let cusp_parameter =
                                    parameter_map.second_contact_parameter(&contact);
                                let direct_point = match arc_parameter
                                    .coincident_point_evidence(arc_circle, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(Some(point)) => point,
                                    Classification::Decided(None) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            crate::UncertaintyReason::Unsupported,
                                        ));
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            reason,
                                        ));
                                    }
                                };
                                let point = retained_fillet_pair_contact_rational_point_on_arc(
                                    &rational_half,
                                    &parameter_map,
                                    &contact,
                                    true,
                                    arc_family,
                                    policy,
                                )?
                                .unwrap_or(direct_point);
                                chart_contacts.push((
                                    arc_parameter,
                                    cusp_parameter,
                                    tangent_cross,
                                    tangent_dot,
                                    point,
                                ));
                            }
                        }
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(contacts) => {
                            for contact in contacts {
                                if !chart_owns_endpoint(
                                    arc_complementary,
                                    contact.first_location,
                                ) || !chart_owns_endpoint(
                                    cusp_complementary,
                                    contact.second_location,
                                ) {
                                    continue;
                                }
                                let arc_parameter = endpoint_parameter(contact.first_location);
                                let cusp_parameter = endpoint_parameter(contact.second_location);
                                let point = match arc_parameter
                                    .coincident_point_evidence(arc_circle, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(Some(point)) => point,
                                    Classification::Decided(None) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            crate::UncertaintyReason::Unsupported,
                                        ));
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            reason,
                                        ));
                                    }
                                };
                                let tangent_cross = contact.tangent_cross_sign;
                                let tangent_dot = if tangent_cross == RealSign::Zero {
                                    let endpoint_tangent = |circle: &crate::bezier_offset::BezierAlgebraicCuspSemicircle2,
                                                            location|
                                     -> ExactCurveResult<crate::BezierAlgebraicChord2> {
                                        let start = match location {
                                            crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Start => true,
                                            crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End => false,
                                            crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior => unreachable!("endpoint-only pair contact was interior"),
                                        };
                                        match crate::BezierAlgebraicCuspSemicircleFragment2::full(
                                            circle.clone(),
                                            policy,
                                        )
                                            .endpoint_tangent_chord(start, policy)
                                            .map_err(|cause| {
                                                ExactCurveError::invalid(
                                                    CurveOperation2::Fillet,
                                                    arc_family,
                                                    cause,
                                                )
                                            })? {
                                            Classification::Decided(Some(chord)) => Ok(chord),
                                            Classification::Decided(None) => Err(
                                                ExactCurveError::blocked(
                                                    CurveOperation2::Fillet,
                                                    arc_family,
                                                    crate::UncertaintyReason::Unsupported,
                                                ),
                                            ),
                                            Classification::Uncertain(reason) => Err(
                                                ExactCurveError::blocked(
                                                    CurveOperation2::Fillet,
                                                    arc_family,
                                                    reason,
                                                ),
                                            ),
                                        }
                                    };
                                    let first = endpoint_tangent(
                                        arc_circle,
                                        contact.first_location,
                                    )?;
                                    let second = endpoint_tangent(
                                        cusp_circle,
                                        contact.second_location,
                                    )?;
                                    Some(match first
                                        .tangent_dot_sign(&second, policy)
                                        .map_err(|cause| {
                                            ExactCurveError::invalid(
                                                CurveOperation2::Fillet,
                                                arc_family,
                                                cause,
                                            )
                                        })? {
                                        Classification::Decided(sign) => sign,
                                        Classification::Uncertain(reason) => {
                                            return Err(ExactCurveError::blocked(
                                                CurveOperation2::Fillet,
                                                arc_family,
                                                reason,
                                            ));
                                        }
                                    })
                                } else {
                                    None
                                };
                                chart_contacts.push((
                                    arc_parameter,
                                    cusp_parameter,
                                    tangent_cross,
                                    tangent_dot,
                                    point,
                                ));
                            }
                        }
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(_) => {
                            centers.coincident = mode == CurveCornerMode2::TrimOrExtend
                                || retained_fillet_arc_cusp_overlap_is_positive(
                                    &offset_support,
                                    cusp,
                                    arc_family,
                                    cusp_family,
                                    policy,
                                )?;
                            return Ok(centers);
                        }
                    }
                    for (
                        arc_parameter,
                        cusp_parameter,
                        mut tangent_cross,
                        mut tangent_dot,
                        point,
                    ) in chart_contacts
                    {
                        if mode == CurveCornerMode2::TrimOnly {
                            match cusp
                                .contains_parameter(&cusp_parameter, false, false, policy)
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        cusp_family,
                                        cause,
                                    )
                                })? {
                                Classification::Decided(true) => {}
                                Classification::Decided(false) => {
                                    centers.outside_domain = true;
                                    continue;
                                }
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        cusp_family,
                                        reason,
                                    ));
                                }
                            }
                        }
                        let Some(contact_seed) = retained_arc_fillet_contact_seed(
                            arc.support(),
                            arc_circle,
                            &arc_parameter,
                            source_radius,
                            signed_radius,
                            mode,
                            arc_family,
                            policy,
                        )?
                        else {
                            centers.outside_domain = true;
                            continue;
                        };
                        if reverse_pair_relation {
                            tangent_cross = reverse_fillet_sign(tangent_cross);
                            tangent_dot = tangent_dot.map(reverse_fillet_sign);
                        }
                        let cusp_parameter = if cusp_complementary {
                            CurveParameter2::from_algebraic_cusp_complement(cusp_parameter)
                        } else {
                            CurveParameter2::from_algebraic_cusp(cusp_parameter)
                        };
                        let (previous_parameter, next_parameter) = if arc_is_previous {
                            (None, Some(cusp_parameter))
                        } else {
                            (Some(cusp_parameter), None)
                        };
                        centers.push(FilletCenterWitness2 {
                            point,
                            previous_parameter,
                            next_parameter,
                            retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                                // The common pair authority reports offset
                                // arc x selected cusp. The two exact source
                                // reversal factors above map that relation to
                                // the authored arc anchor and cusp traversal.
                                cross: Some(tangent_cross),
                                dot: tangent_dot,
                                center_parallel: None,
                                source_direction: None,
                                canonical_anchor_curve: None,
                                deferred_arc_contact: Some(RetainedDeferredArcFilletContact2 {
                                    support: arc.support().clone(),
                                    source_radius: source_radius.clone(),
                                    signed_center_radius: signed_radius.clone(),
                                    arc_is_previous,
                                    selected_center: matches!(
                                        &arc_parameter,
                                        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Mapped(_)
                                    )
                                    .then_some(arc_parameter),
                                    contact_seed: Some(contact_seed),
                                }),
                            }),
                        });
                    }
                }
            }
        }
        (FilletOffsetCarrier2::Line { .. }, FilletOffsetCarrier2::AlgebraicCusp { .. })
        | (FilletOffsetCarrier2::AlgebraicCusp { .. }, FilletOffsetCarrier2::Line { .. }) => {
            let (line_source, line_support, line_signed_distance, line_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::Line {
                            source,
                            support,
                            signed_distance,
                            ..
                        },
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source: _,
                            support: _,
                        },
                    ) => (source, support, signed_distance, true),
                    (
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source: _,
                            support: _,
                        },
                        FilletOffsetCarrier2::Line {
                            source,
                            support,
                            signed_distance,
                            ..
                        },
                    ) => (source, support, signed_distance, false),
                    _ => unreachable!(),
                };
            // Lower every represented line to the authoritative chord cell.
            // Native-line witnesses retain their authored finite domain for
            // TrimOnly. A canonical witness for an algebraic chord names only
            // its complete affine support; final cut publication reclassifies
            // the contact on the authored source chord. TrimOrExtend selects
            // both selected-circle charts and complete support in either case.
            let line_family = if line_is_previous {
                previous_family
            } else {
                next_family
            };
            let source_chord = match line_source.algebraic_chord() {
                Some(source) => source.clone(),
                None => algebraic_chord_from_line_support(
                    line_source
                        .native_line()
                        .expect("a represented line carrier retains one source line"),
                    CurveOperation2::Fillet,
                    line_family,
                    policy,
                )?,
            };
            let retained_support = source_chord
                .parallel_left_retained(line_signed_distance.clone(), policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                })?;
            let represented_line_names_finite_domain = line_source.native_line().is_some()
                || algebraic_chord_domain_matches_line_witness(
                    &retained_support,
                    line_support,
                    line_family,
                    policy,
                )?;
            // Keep canonical-Real line endpoints as the intersection hot
            // path while retaining the procedural normal-offset support as
            // ancestry. The former selects the compact selected-fiber line
            // kernel; the latter is the exact tangent relation needed when a
            // recursively selected terminal circle is reconstructed.
            let (support_chord, finite_source_domain) =
                if mode == CurveCornerMode2::TrimOnly && !represented_line_names_finite_domain {
                    // A canonical represented line names only the infinite
                    // support of this algebraic chord. For TrimOnly, intersect
                    // the translated authored chord itself so the common kernel
                    // owns and retains both exact finite-boundary inequalities.
                    (retained_support.clone(), true)
                } else {
                    let support = retained_support
                        .chord_between_certified_ordered_support_points(
                            CurvePoint2::from(line_support.start().clone()),
                            CurvePoint2::from(line_support.end().clone()),
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                        })?;
                    (support, represented_line_names_finite_domain)
                };
            let promoted = FilletOffsetCarrier2::AlgebraicChord {
                source: &source_chord,
                support: support_chord,
                signed_distance: line_signed_distance.clone(),
                finite_source_domain,
            };
            let mut promoted_centers = if line_is_previous {
                fillet_offset_centers(
                    &promoted,
                    next,
                    mode,
                    domains,
                    previous_family,
                    next_family,
                    policy,
                )
            } else {
                fillet_offset_centers(
                    previous,
                    &promoted,
                    mode,
                    domains,
                    previous_family,
                    next_family,
                    policy,
                )
            }?;
            // Retain the promoted contact parameter needed by the original
            // carrier. Native lines reuse the recursive quadratic scalar;
            // algebraic chords retain contact provenance while their final
            // cut is classified on the authored source domain.
            for center in promoted_centers.iter_mut() {
                let line_parameter = if line_is_previous {
                    center.previous_parameter.as_ref()
                } else {
                    center.next_parameter.as_ref()
                };
                let recursive_line_parameter = line_parameter
                    .and_then(CurveParameter2::as_algebraic_chord)
                    .and_then(|parameter| parameter.point().as_algebraic_cusp_chord())
                    .and_then(|point| point.recursive_quadratic_line_parameter())
                    .map(CurveParameter2::from_recursive_projective);
                let retained_line_parameter = finite_source_domain
                    .then(|| {
                        recursive_line_parameter.or_else(|| {
                            line_source
                                .algebraic_chord()
                                .and_then(|_| line_parameter.cloned())
                        })
                    })
                    .flatten();
                if line_is_previous {
                    center.previous_parameter = retained_line_parameter;
                } else {
                    center.next_parameter = retained_line_parameter;
                }
            }
            return Ok(promoted_centers);
        }
        (
            FilletOffsetCarrier2::AlgebraicCusp {
                source: previous_source,
                support: previous_support,
            },
            FilletOffsetCarrier2::AlgebraicCusp {
                source: next_source,
                support: next_support,
            },
        ) => {
            let previous_support_reverses_source = retained_fillet_cusp_support_reverses_source(
                previous_source,
                previous_support,
                previous_family,
                policy,
            )?;
            let next_support_reverses_source = retained_fillet_cusp_support_reverses_source(
                next_source,
                next_support,
                next_family,
                policy,
            )?;
            let reverse_pair_relation =
                previous_support_reverses_source != next_support_reverses_source;
            let witness = |previous_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
                           previous_complementary: bool,
                           next_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
                           next_complementary: bool,
                           point: CurvePoint2,
                           retained_anchor_evidence: Option<RetainedFilletAnchorEvidence2>|
             -> ExactCurveResult<Option<FilletCenterWitness2>> {
                if mode == CurveCornerMode2::TrimOnly {
                    for (source, parameter, family) in [
                        (previous_source, &previous_parameter, previous_family),
                        (next_source, &next_parameter, next_family),
                    ] {
                        match source
                            .contains_parameter(parameter, false, false, policy)
                            .map_err(|cause| {
                                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                            })? {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => return Ok(None),
                            // This is only a conservative fast rejection. The
                            // authoritative cut classifier below owns every
                            // case this cheaper parameter predicate cannot
                            // decide exactly.
                            Classification::Uncertain(_) => {}
                        }
                    }
                }
                let previous_parameter = if previous_complementary {
                    CurveParameter2::from_algebraic_cusp_complement(previous_parameter)
                } else {
                    CurveParameter2::from_algebraic_cusp(previous_parameter)
                };
                let next_parameter = if next_complementary {
                    CurveParameter2::from_algebraic_cusp_complement(next_parameter)
                } else {
                    CurveParameter2::from_algebraic_cusp(next_parameter)
                };
                Ok(Some(FilletCenterWitness2 {
                    point,
                    previous_parameter: Some(previous_parameter),
                    next_parameter: Some(next_parameter),
                    retained_anchor_evidence,
                }))
            };
            let previous_complement = (mode == CurveCornerMode2::TrimOrExtend)
                .then(|| previous_support.semicircle().complementary_half());
            let next_complement = (mode == CurveCornerMode2::TrimOrExtend)
                .then(|| next_support.semicircle().complementary_half());
            let previous_charts = [
                Some((previous_support.semicircle(), false)),
                previous_complement.as_ref().map(|circle| (circle, true)),
            ];
            let next_charts = [
                Some((next_support.semicircle(), false)),
                next_complement.as_ref().map(|circle| (circle, true)),
            ];
            let chart_owns_endpoint = |complementary: bool, location| {
                !complementary
                    || location
                        == crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior
            };
            for (previous_circle, previous_complementary) in
                previous_charts.iter().flatten().copied()
            {
                for (next_circle, next_complementary) in next_charts.iter().flatten().copied() {
                    let intersections = match previous_circle
                        .pair_intersections(next_circle, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Fillet,
                                previous_family,
                                cause,
                            )
                        })? {
                        Classification::Decided(intersections) => intersections,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                previous_family,
                                reason,
                            ));
                        }
                    };
                    match intersections {
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts => {}
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                            contacts,
                            parameter_map,
                        } => {
                            for contact in contacts {
                                if !chart_owns_endpoint(
                                    previous_complementary,
                                    contact.first_location,
                                ) || !chart_owns_endpoint(
                                    next_complementary,
                                    contact.second_location,
                                ) {
                                    continue;
                                }
                                let mut tangent_cross = contact.tangent_cross_sign;
                                // A transverse contact's oriented cross sign
                                // alone selects the one- or two-half fillet
                                // sweep. Do not ask the represented pair for
                                // an independent high-degree dot predicate
                                // unless the tangents are parallel.
                                let mut tangent_dot = if tangent_cross == RealSign::Zero {
                                    Some(
                                        match parameter_map
                                            .tangent_dot_sign(&contact, policy)
                                            .map_err(|cause| {
                                                ExactCurveError::invalid(
                                                    CurveOperation2::Fillet,
                                                    previous_family,
                                                    cause,
                                                )
                                            })? {
                                            Classification::Decided(sign) => sign,
                                            Classification::Uncertain(reason) => {
                                                return Err(ExactCurveError::blocked(
                                                    CurveOperation2::Fillet,
                                                    previous_family,
                                                    reason,
                                                ));
                                            }
                                        },
                                    )
                                } else {
                                    None
                                };
                                if reverse_pair_relation {
                                    tangent_cross = reverse_fillet_sign(tangent_cross);
                                    tangent_dot = tangent_dot.map(reverse_fillet_sign);
                                }
                                let previous_parameter =
                                    parameter_map.first_contact_parameter(&contact);
                                let next_parameter =
                                    parameter_map.second_contact_parameter(&contact);
                                let direct_point = match previous_parameter
                                    .coincident_point_evidence(previous_circle, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(Some(point)) => point,
                                    Classification::Decided(None) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            crate::UncertaintyReason::Unsupported,
                                        ));
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            reason,
                                        ));
                                    }
                                };
                                let point = retained_fillet_pair_contact_rational_point(
                                    previous_circle,
                                    &parameter_map,
                                    &contact,
                                    true,
                                    previous_family,
                                    policy,
                                )?
                                .unwrap_or(direct_point);
                                if let Some(witness) = witness(
                                    previous_parameter,
                                    previous_complementary,
                                    next_parameter,
                                    next_complementary,
                                    point,
                                    Some(RetainedFilletAnchorEvidence2 {
                                        cross: Some(tangent_cross),
                                        dot: tangent_dot,
                                        center_parallel: None,
                                        source_direction: None,
                                        canonical_anchor_curve: None,
                                        deferred_arc_contact: None,
                                    }),
                                )? {
                                    centers.push(witness);
                                } else {
                                    centers.outside_domain = true;
                                }
                            }
                        }
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(contacts) => {
                            let endpoint = |location| match location {
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Start => crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End => crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one()),
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior => unreachable!("endpoint-only pair contact was interior"),
                            };
                            for contact in contacts {
                                if !chart_owns_endpoint(
                                    previous_complementary,
                                    contact.first_location,
                                ) || !chart_owns_endpoint(
                                    next_complementary,
                                    contact.second_location,
                                ) {
                                    continue;
                                }
                                let previous_parameter = endpoint(contact.first_location);
                                let next_parameter = endpoint(contact.second_location);
                                let point = match previous_parameter
                                    .coincident_point_evidence(previous_circle, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(Some(point)) => point,
                                    Classification::Decided(None) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            crate::UncertaintyReason::Unsupported,
                                        ));
                                    }
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            reason,
                                        ));
                                    }
                                };
                                if let Some(witness) = witness(
                                    previous_parameter,
                                    previous_complementary,
                                    next_parameter,
                                    next_complementary,
                                    point,
                                    None,
                                )? {
                                    centers.push(witness);
                                } else {
                                    centers.outside_domain = true;
                                }
                            }
                        }
                        crate::bezier_offset::BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(overlap) => {
                            if mode == CurveCornerMode2::TrimOrExtend {
                                centers.coincident = true;
                                return Ok(centers);
                            }
                            centers.coincident = retained_fillet_cusp_pair_overlap_is_positive(
                                previous_source,
                                next_source,
                                &overlap,
                                previous_family,
                                policy,
                            )?;
                        }
                    }
                }
            }
        }
        (
            FilletOffsetCarrier2::AlgebraicChord {
                source: previous_source,
                support: previous_support,
                signed_distance: previous_distance,
                ..
            },
            FilletOffsetCarrier2::AlgebraicChord {
                source: next_source,
                support: next_support,
                signed_distance: next_distance,
                ..
            },
        ) => {
            let tangent_relation = |cross| {
                let relation = if cross {
                    previous_support.tangent_cross_sign(next_support, policy)
                } else {
                    previous_support.tangent_dot_sign(next_support, policy)
                }
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
                })?;
                match relation {
                    Classification::Decided(sign) => Ok(sign),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        reason,
                    )),
                }
            };
            let common_corner = previous_source.end();
            let shares_corner = common_corner.shares_storage(next_source.start())
                || common_corner.same_point(next_source.start(), policy)
                    == Classification::Decided(true);
            let common_corner_center = if shares_corner {
                match (
                    previous_source.certified_unit_tangent(),
                    next_source.certified_unit_tangent(),
                ) {
                    (Some(previous_tangent), Some(next_tangent)) => {
                        let offset_origin = |tangent: &(Real, Real), distance: &Real| {
                            Point2::new(-(&tangent.1 * distance), &tangent.0 * distance)
                        };
                        let previous_origin = offset_origin(&previous_tangent, previous_distance);
                        let next_origin = offset_origin(&next_tangent, next_distance);
                        let previous_line = LineSeg2::new_unchecked(
                            previous_origin.clone(),
                            previous_origin.translated(previous_tangent.0, previous_tangent.1),
                        );
                        let next_line = LineSeg2::new_unchecked(
                            next_origin.clone(),
                            next_origin.translated(next_tangent.0, next_tangent.1),
                        );
                        match crate::offset::line_support_intersection(
                            &previous_line,
                            &next_line,
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Fillet,
                                previous_family,
                                cause,
                            )
                        })? {
                            Classification::Decided(Some(delta)) => {
                                match crate::BezierAlgebraicChord2::translated_endpoint(
                                    common_corner,
                                    delta.x(),
                                    delta.y(),
                                    policy,
                                )
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        previous_family,
                                        cause,
                                    )
                                })? {
                                    Classification::Decided(point) => Some((
                                        point,
                                        tangent_relation(true)?,
                                        tangent_relation(false)?,
                                    )),
                                    Classification::Uncertain(reason) => {
                                        return Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            previous_family,
                                            reason,
                                        ));
                                    }
                                }
                            }
                            Classification::Decided(None) => None,
                            Classification::Uncertain(reason) => {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    previous_family,
                                    reason,
                                ));
                            }
                        }
                    }
                    _ => None,
                }
            } else {
                None
            };
            if let Some((point, tangent_cross, tangent_dot)) = common_corner_center {
                centers.push(FilletCenterWitness2 {
                    point,
                    previous_parameter: None,
                    next_parameter: None,
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        cross: Some(tangent_cross),
                        dot: Some(tangent_dot),
                        center_parallel: None,
                        source_direction: None,
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    }),
                });
                return Ok(centers);
            }
            let tangent_cross = tangent_relation(true)?;
            if tangent_cross == RealSign::Zero {
                let side = match previous_support
                    .oriented_support_side(next_support.start(), policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
                    })? {
                    Classification::Decided(side) => side,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            previous_family,
                            reason,
                        ));
                    }
                };
                centers.coincident = side == crate::classify::LineSide::On;
                return Ok(centers);
            }
            let point = match previous_support
                .supporting_line_intersection(next_support, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
                })? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Fillet,
                        previous_family,
                        CurveError::Topology(
                            "nonparallel retained fillet supports omitted their intersection"
                                .into(),
                        ),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        reason,
                    ));
                }
            };
            centers.push(FilletCenterWitness2 {
                point,
                previous_parameter: None,
                next_parameter: None,
                retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                    cross: Some(tangent_cross),
                    dot: Some(tangent_relation(false)?),
                    center_parallel: None,
                    source_direction: None,
                    canonical_anchor_curve: None,
                    deferred_arc_contact: None,
                }),
            });
        }
        (
            FilletOffsetCarrier2::AlgebraicChord { .. },
            FilletOffsetCarrier2::AlgebraicCusp { .. },
        )
        | (
            FilletOffsetCarrier2::AlgebraicCusp { .. },
            FilletOffsetCarrier2::AlgebraicChord { .. },
        ) => {
            let (chord_support, cusp_source, cusp_support, chord_is_previous, finite_source_domain) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::AlgebraicChord {
                            support,
                            finite_source_domain,
                            ..
                        },
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source,
                            support: cusp,
                        },
                    ) => (support, source, cusp, true, *finite_source_domain),
                    (
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source,
                            support: cusp,
                        },
                        FilletOffsetCarrier2::AlgebraicChord {
                            support,
                            finite_source_domain,
                            ..
                        },
                    ) => (support, source, cusp, false, *finite_source_domain),
                    _ => unreachable!(),
                };
            let chord_family = if chord_is_previous {
                previous_family
            } else {
                next_family
            };
            let cusp_family = if chord_is_previous {
                next_family
            } else {
                previous_family
            };
            let intersections = cusp_support
                .semicircle()
                .chord_intersections_prefer_exact_line(
                    chord_support,
                    finite_source_domain && mode != CurveCornerMode2::TrimOrExtend,
                    policy,
                );
            let base_contacts = match intersections
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
                })? {
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordIntersections2::NoContacts,
                ) => Vec::new(),
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordIntersections2::Contacts(contacts),
                ) => contacts,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        cusp_family,
                        reason,
                    ));
                }
            };
            let mut contacts = base_contacts
                .into_iter()
                .map(|contact| (contact, false))
                .collect::<Vec<_>>();
            if mode == CurveCornerMode2::TrimOrExtend {
                let complementary = cusp_support.semicircle().complementary_half();
                let complementary_contacts = match complementary
                    .chord_support_intersections(chord_support, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
                    })? {
                    Classification::Decided(
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordIntersections2::NoContacts,
                    ) => Vec::new(),
                    Classification::Decided(
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordIntersections2::Contacts(contacts),
                    ) => contacts,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            cusp_family,
                            reason,
                        ));
                    }
                };
                for contact in complementary_contacts {
                    // Both half charts name their common diameter endpoints.
                    // Keep the authored-half copy so one geometric center is
                    // never emitted twice.
                    let at_start = match contact
                        .cusp_parameter
                        .order_to_real(&Real::zero(), policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
                        })? {
                        Classification::Decided(order) => order == std::cmp::Ordering::Equal,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                cusp_family,
                                reason,
                            ));
                        }
                    };
                    let at_end = match contact
                        .cusp_parameter
                        .order_to_real(&Real::one(), policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
                        })? {
                        Classification::Decided(order) => order == std::cmp::Ordering::Equal,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                cusp_family,
                                reason,
                            ));
                        }
                    };
                    if !at_start && !at_end {
                        contacts.push((contact, true));
                    }
                }
            }
            let cusp_support_reverses_source = retained_fillet_cusp_support_reverses_source(
                cusp_source,
                cusp_support,
                cusp_family,
                policy,
            )?;
            for (contact, complementary) in contacts {
                let mut tangent_dot = match contact
                    .tangent_dot_sign(cusp_support.semicircle(), chord_support, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                    })? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            chord_family,
                            reason,
                        ));
                    }
                };
                if cusp_support_reverses_source {
                    tangent_dot = reverse_fillet_sign(tangent_dot);
                }
                let tangent_cross = if cusp_support_reverses_source {
                    reverse_fillet_sign(contact.tangent_cross_sign)
                } else {
                    contact.tangent_cross_sign
                };
                let cusp_parameter = if complementary {
                    CurveParameter2::from_algebraic_cusp_complement(contact.cusp_parameter)
                } else {
                    CurveParameter2::from_algebraic_cusp(contact.cusp_parameter)
                };
                let chord_parameter =
                    CurveParameter2::from_algebraic_chord(contact.chord_parameter);
                let (previous_parameter, next_parameter) = if chord_is_previous {
                    (Some(chord_parameter), Some(cusp_parameter))
                } else {
                    (Some(cusp_parameter), Some(chord_parameter))
                };
                centers.push(FilletCenterWitness2 {
                    point: contact.point,
                    previous_parameter,
                    next_parameter,
                    // The selected circle is the reconstruction anchor. The
                    // lower kernel reports exactly circle x chord.
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        cross: Some(tangent_cross),
                        dot: Some(tangent_dot),
                        center_parallel: None,
                        source_direction: None,
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    }),
                });
            }
        }
        (FilletOffsetCarrier2::AlgebraicChord { .. }, FilletOffsetCarrier2::Arc { .. })
        | (FilletOffsetCarrier2::Arc { .. }, FilletOffsetCarrier2::AlgebraicChord { .. }) => {
            let (
                chord_source,
                chord_support,
                chord_signed_distance,
                arc,
                source_radius,
                signed_radius,
                chord_is_previous,
            ) = match (previous, next) {
                (
                    FilletOffsetCarrier2::AlgebraicChord {
                        source: chord_source,
                        support,
                        signed_distance,
                        ..
                    },
                    FilletOffsetCarrier2::Arc {
                        source: arc_source,
                        source_radius,
                        signed_radius,
                    },
                ) => (
                    *chord_source,
                    support,
                    signed_distance,
                    *arc_source,
                    *source_radius,
                    signed_radius,
                    true,
                ),
                (
                    FilletOffsetCarrier2::Arc {
                        source,
                        source_radius,
                        signed_radius,
                    },
                    FilletOffsetCarrier2::AlgebraicChord {
                        source: chord_source,
                        support,
                        signed_distance,
                        ..
                    },
                ) => (
                    *chord_source,
                    support,
                    signed_distance,
                    *source,
                    *source_radius,
                    signed_radius,
                    false,
                ),
                _ => unreachable!(),
            };
            let chord_family = if chord_is_previous {
                previous_family
            } else {
                next_family
            };
            let arc_family = if chord_is_previous {
                next_family
            } else {
                previous_family
            };

            // Adjacent retained carriers sometimes name their common vertex
            // in independent exact fields. When STRICT can replay that
            // identity and the chord already owns a represented unit tangent,
            // the common arc endpoint is an exact represented point on the
            // chord support. Translate that point by the authored chord
            // offset and use the primitive line/circle relation. This is the
            // same authoritative geometry as the general selected-axis
            // projection below, but it avoids constructing a three-parameter
            // compositum for the overwhelmingly common incident-corner case.
            let arc_corner = if chord_is_previous {
                arc.support().start()
            } else {
                arc.support().end()
            };
            let chord_corner = if chord_is_previous {
                chord_source.end()
            } else {
                chord_source.start()
            };
            let arc_corner_evidence = CurvePoint2::from(arc_corner.clone());
            let shares_strict_corner = chord_corner.shares_storage(&arc_corner_evidence)
                || chord_corner.same_point(&arc_corner_evidence, &CurveContext::STRICT)
                    == Classification::Decided(true);
            if shares_strict_corner
                && let Some((tangent_x, tangent_y)) = chord_source.certified_unit_tangent()
            {
                let line_start = arc_corner.translated(
                    -(&tangent_y * chord_signed_distance),
                    &tangent_x * chord_signed_distance,
                );
                let line_support = LineSeg2::new_unchecked(
                    line_start.clone(),
                    line_start.translated(tangent_x, tangent_y),
                );
                let relation = crate::intersect::line_circle_relation_from_supports(
                    &line_support,
                    arc.support().center(),
                    &(signed_radius * signed_radius),
                    policy,
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                })?;
                let mut push = |point: Point2| {
                    centers.push(FilletCenterWitness2 {
                        point: point.into(),
                        previous_parameter: None,
                        next_parameter: None,
                        retained_anchor_evidence: None,
                    });
                };
                match relation {
                    crate::LineCircleRelation::Disjoint => {}
                    crate::LineCircleRelation::Tangent { point, .. } => push(point),
                    crate::LineCircleRelation::Secant {
                        first_point,
                        second_point,
                        ..
                    } => {
                        push(first_point);
                        push(second_point);
                    }
                    crate::LineCircleRelation::Uncertain { reason } => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            chord_family,
                            reason,
                        ));
                    }
                }
                return Ok(centers);
            }

            let canonical_chord;
            let (intersection_chord, chord_tangent_reversed) = if chord_support.is_reversed() {
                canonical_chord = chord_support.reversed();
                (&canonical_chord, true)
            } else {
                (chord_support, false)
            };
            let arc_is_previous = !chord_is_previous;
            let signed_radius_sign = match crate::classify::real_sign(signed_radius, policy) {
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Some(RealSign::Zero) => {
                    unreachable!("collapsed arc offsets use the point carrier")
                }
                None => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        arc_family,
                        crate::UncertaintyReason::RealSign,
                    ));
                }
            };

            // A concentric arc offset is still exactly one circle. Solve its
            // complete carrier against the retained chord in the common
            // selected-circle/chord kernel instead of projecting the same
            // quadratic incidence independently through every rational arc
            // cell. The chord supplies a compact orthonormal chart; both half
            // charts enumerate the full circle, while the deferred tangent
            // replay below owns the authored/extension arc domain.
            let offset_circle = match crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_retained_center_and_chord_normal(
                CurvePoint2::from(
                    arc.support().center().clone(),
                ),
                intersection_chord.clone(),
                signed_radius.clone(),
                arc.support().is_clockwise(),
                policy,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
            })? {
                Classification::Decided(Some(circle)) => circle,
                Classification::Decided(None) => {
                    unreachable!("the nonzero concentric arc offset defines a circle")
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        arc_family,
                        reason,
                    ));
                }
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-algebraic-chord-arc",
                "selected-circle-authority",
            );
            let mut contacts = Vec::new();
            for (circle, complementary) in [
                (offset_circle.clone(), false),
                (offset_circle.complementary_half(), true),
            ] {
                let intersections = if mode == CurveCornerMode2::TrimOrExtend {
                    circle.chord_support_intersections(intersection_chord, policy)
                } else {
                    circle.chord_intersections(intersection_chord, policy)
                }
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                })?;
                let circle_contacts = match intersections {
                    Classification::Decided(
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordIntersections2::NoContacts,
                    ) => Vec::new(),
                    Classification::Decided(
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordIntersections2::Contacts(contacts),
                    ) => contacts,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            chord_family,
                            reason,
                        ));
                    }
                };
                for contact in circle_contacts {
                    if complementary {
                        let at_diameter_endpoint = [Real::zero(), Real::one()]
                            .into_iter()
                            .try_fold(false, |at_endpoint, endpoint| {
                                if at_endpoint {
                                    return Ok(true);
                                }
                                match contact
                                    .cusp_parameter
                                    .order_to_real(&endpoint, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            cause,
                                        )
                                    })? {
                                    Classification::Decided(order) => {
                                        Ok(order == std::cmp::Ordering::Equal)
                                    }
                                    Classification::Uncertain(reason) => {
                                        Err(ExactCurveError::blocked(
                                            CurveOperation2::Fillet,
                                            arc_family,
                                            reason,
                                        ))
                                    }
                                }
                            })?;
                        if at_diameter_endpoint {
                            continue;
                        }
                    }
                    contacts.push((contact, circle.clone()));
                }
            }
            let mut dot = match offset_circle
                .chord_tangent_dot_sign(intersection_chord, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                })? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        chord_family,
                        reason,
                    ));
                }
            };
            if chord_tangent_reversed {
                dot = reverse_fillet_sign(dot);
            }
            if signed_radius_sign == RealSign::Negative {
                dot = reverse_fillet_sign(dot);
            }
            for (contact, circle) in contacts {
                let Some(contact_seed) = retained_arc_fillet_contact_seed(
                    arc.support(),
                    &circle,
                    &contact.cusp_parameter,
                    source_radius,
                    signed_radius,
                    mode,
                    arc_family,
                    policy,
                )?
                else {
                    continue;
                };
                let mut cross = contact.tangent_cross_sign;
                if chord_tangent_reversed {
                    cross = reverse_fillet_sign(cross);
                }
                if signed_radius_sign == RealSign::Negative {
                    cross = reverse_fillet_sign(cross);
                }
                let chord_parameter =
                    CurveParameter2::from_algebraic_chord(contact.chord_parameter);
                let (previous_parameter, next_parameter) = if chord_is_previous {
                    (Some(chord_parameter), None)
                } else {
                    (None, Some(chord_parameter))
                };
                centers.push(FilletCenterWitness2 {
                    point: contact.point,
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        // The selected circle reports offset-arc x chord.
                        // Signed-radius reversal above maps this back to the
                        // authored arc tangent used to select the fillet sweep.
                        cross: Some(cross),
                        dot: Some(dot),
                        center_parallel: None,
                        source_direction: None,
                        canonical_anchor_curve: None,
                        deferred_arc_contact: Some(RetainedDeferredArcFilletContact2 {
                            support: arc.support().clone(),
                            source_radius: source_radius.clone(),
                            signed_center_radius: signed_radius.clone(),
                            arc_is_previous,
                            selected_center: None,
                            contact_seed: Some(contact_seed),
                        }),
                    }),
                });
            }
        }
        (FilletOffsetCarrier2::AlgebraicChord { .. }, FilletOffsetCarrier2::Parallel { .. })
        | (FilletOffsetCarrier2::Parallel { .. }, FilletOffsetCarrier2::AlgebraicChord { .. }) => {
            let (chord_support, parallel_source, analytic_support, chord_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::AlgebraicChord { support, .. },
                        FilletOffsetCarrier2::Parallel {
                            source,
                            support: analytic,
                        },
                    ) => (support, source, analytic, true),
                    (
                        FilletOffsetCarrier2::Parallel {
                            source,
                            support: analytic,
                        },
                        FilletOffsetCarrier2::AlgebraicChord { support, .. },
                    ) => (support, source, analytic, false),
                    _ => unreachable!(),
                };
            let chord_family = if chord_is_previous {
                previous_family
            } else {
                next_family
            };
            let analytic_family = if chord_is_previous {
                next_family
            } else {
                previous_family
            };
            let analytic_is_previous = !chord_is_previous;
            let incident_domain = if mode == CurveCornerMode2::TrimOrExtend {
                Some(parallel_source.incident_domain(
                    analytic_support,
                    analytic_is_previous,
                    analytic_family,
                    policy,
                )?)
            } else {
                None
            };
            let intersection_result = if let Some(domain) = incident_domain.as_ref() {
                chord_support.parallel_intersections_with_incident_ray(
                    analytic_support,
                    domain,
                    policy,
                )
            } else {
                chord_support.parallel_intersections(analytic_support, policy)
            };
            let intersections = match intersection_result.map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                })? {
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::Contacts(
                        contacts,
                    ),
                ) => contacts,
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent
                    | crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                ) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        chord_family,
                        crate::UncertaintyReason::Boundary,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        chord_family,
                        reason,
                    ));
                }
            };
            let analytic_support_reverses_source = parallel_source.support_reverses_source(
                analytic_support,
                analytic_family,
                policy,
            )?;
            for contact in intersections {
                if !parallel_source.bezier_parameter_is_admissible(
                    contact.parallel_parameter(),
                    analytic_is_previous,
                    mode,
                    domains[usize::from(!analytic_is_previous)],
                    incident_domain.as_ref(),
                    analytic_family,
                    policy,
                )? {
                    continue;
                }
                let mut cross = contact.tangent_cross_sign();
                let mut dot = contact.tangent_dot_sign();
                if analytic_support_reverses_source {
                    cross = reverse_fillet_sign(cross);
                    dot = reverse_fillet_sign(dot);
                }
                let analytic_parameter =
                    CurveParameter2::from(contact.parallel_parameter().clone());
                let (previous_parameter, next_parameter) = if chord_is_previous {
                    (None, Some(analytic_parameter))
                } else {
                    (Some(analytic_parameter), None)
                };
                centers.push(FilletCenterWitness2 {
                    point: contact.point().clone(),
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        // The retained circle frame is analytic, while the
                        // intersection kernel reports chord x analytic.
                        cross: Some(reverse_fillet_sign(cross)),
                        dot: Some(dot),
                        center_parallel: None,
                        source_direction: Some(if analytic_support_reverses_source {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        }),
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    }),
                });
            }
        }
        (FilletOffsetCarrier2::AlgebraicChord { .. }, FilletOffsetCarrier2::Line { .. })
        | (FilletOffsetCarrier2::Line { .. }, FilletOffsetCarrier2::AlgebraicChord { .. }) => {
            let (chord_support, line_support, chord_is_previous) = match (previous, next) {
                (
                    FilletOffsetCarrier2::AlgebraicChord { support, .. },
                    FilletOffsetCarrier2::Line { support: line, .. },
                ) => (support, line, true),
                (
                    FilletOffsetCarrier2::Line { support: line, .. },
                    FilletOffsetCarrier2::AlgebraicChord { support, .. },
                ) => (support, line, false),
                _ => unreachable!(),
            };
            let line_family = if chord_is_previous {
                next_family
            } else {
                previous_family
            };
            let line_chord = algebraic_chord_from_line_support(
                line_support,
                CurveOperation2::Fillet,
                line_family,
                policy,
            )?;
            let tangent_relation = |cross| {
                let relation = if cross {
                    line_chord.tangent_cross_sign(chord_support, policy)
                } else {
                    line_chord.tangent_dot_sign(chord_support, policy)
                }
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                })?;
                match relation {
                    Classification::Decided(sign) => Ok(sign),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        line_family,
                        reason,
                    )),
                }
            };
            let tangent_cross = tangent_relation(true)?;
            if tangent_cross == RealSign::Zero {
                let side = match line_chord
                    .oriented_support_side(chord_support.start(), policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                    })? {
                    Classification::Decided(side) => side,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            line_family,
                            reason,
                        ));
                    }
                };
                centers.coincident = side == crate::classify::LineSide::On;
                return Ok(centers);
            }
            let point = match line_chord
                .supporting_line_intersection(chord_support, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                })? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Fillet,
                        line_family,
                        CurveError::Topology(
                            "nonparallel retained line/chord fillet supports omitted their intersection"
                                .into(),
                        ),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        line_family,
                        reason,
                    ));
                }
            };
            centers.push(FilletCenterWitness2 {
                point,
                previous_parameter: None,
                next_parameter: None,
                retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                    // The algebraic chord's exact center frame is shared by
                    // trim and extension; cut classifiers own finite domains.
                    cross: Some(reverse_fillet_sign(tangent_cross)),
                    dot: Some(tangent_relation(false)?),
                    center_parallel: None,
                    source_direction: None,
                    canonical_anchor_curve: None,
                    deferred_arc_contact: None,
                }),
            });
            return Ok(centers);
        }
        (
            FilletOffsetCarrier2::Line {
                source: previous_source,
                support: previous_support,
                ..
            },
            FilletOffsetCarrier2::Line {
                source: next_source,
                support: next_support,
                ..
            },
        ) => {
            // Distinct finite charts need not meet at the authored corner.
            // They share the support intersection and exact affine cut map
            // with retained lines; only connected inputs take the fast path.
            let point = match crate::offset::line_support_intersection(
                previous_support,
                next_support,
                policy,
            )
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause)
            })? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => return Ok(centers),
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous_family,
                        reason,
                    ));
                }
            };
            let retained_parameter = |source: &FilletLinearSource2<'_>,
                                      support: &LineSeg2,
                                      family: CurveFamily2|
             -> ExactCurveResult<Option<CurveParameter2>> {
                source
                    .native_line()
                    .map(|_| {
                        line_parameter_at_point(support, &point, CurveOperation2::Fillet, family)
                            .map(exact_parameter)
                    })
                    .transpose()
            };
            centers.push(FilletCenterWitness2 {
                previous_parameter: retained_parameter(
                    previous_source,
                    previous_support,
                    previous_family,
                )?,
                next_parameter: retained_parameter(next_source, next_support, next_family)?,
                point: point.into(),
                retained_anchor_evidence: None,
            });
        }
    }
    Ok(centers)
}

fn point_on_fillet_offset(
    point: &CurvePoint2,
    support: &FilletOffsetCarrier2<'_, '_>,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let decided = |classification| match classification {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    };
    match support {
        FilletOffsetCarrier2::Point { point: other } => decided(point.same_point(other, policy)),
        FilletOffsetCarrier2::Arc {
            source,
            signed_radius,
            ..
        } => match crate::bezier_offset::retained_point_circle_incidence_sign(
            point,
            source.support().center(),
            &(signed_radius * signed_radius),
            policy,
        )
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(RealSign::Zero) => Ok(true),
            Classification::Decided(RealSign::Positive | RealSign::Negative) => Ok(false),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            )),
        },
        FilletOffsetCarrier2::Line { support, .. } => {
            if let Some(point) = point.coordinates() {
                let (dx, dy) = support.delta();
                let from_start = point.delta_from(support.start());
                return crate::classify::is_zero(
                    &(&dx * &from_start.1 - &dy * &from_start.0),
                    policy,
                )
                .ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::RealSign,
                    )
                });
            }
            let chord = match crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(support.start().clone()),
                CurvePoint2::from(support.end().clone()),
                policy,
            )
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
            {
                Classification::Decided(chord) => chord,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        reason,
                    ));
                }
            };
            let side = chord
                .oriented_support_side(point, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })?;
            decided(side.map(|side| side == LineSide::On))
        }
        FilletOffsetCarrier2::Parallel { support, .. } => decided(
            support
                .contains_point_evidence(point, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })?,
        ),
        FilletOffsetCarrier2::AlgebraicChord { support, .. } => decided(
            support
                .contains_point_evidence(point, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })?,
        ),
        FilletOffsetCarrier2::AlgebraicCusp { support, .. } => decided(
            support
                .contains_point_evidence(point, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })?,
        ),
    }
}

#[allow(clippy::too_many_arguments)]
fn fillet_cut_from_center(
    offset: &FilletOffsetCarrier2<'_, '_>,
    center: &CurvePoint2,
    retained_parameter: Option<&CurveParameter2>,
    deferred_arc_contact: bool,
    previous: bool,
    mode: CurveCornerMode2,
    retain_selected_circle_endpoints: bool,
    domain: FilletContactDomain2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    match offset {
        FilletOffsetCarrier2::Line {
            source,
            unit_x,
            unit_y,
            signed_distance,
            ..
        } => {
            if let Some(source) = source.algebraic_chord() {
                return algebraic_chord_fillet_cut_from_center(
                    source,
                    center,
                    retained_parameter,
                    signed_distance,
                    previous,
                    mode,
                    family,
                    policy,
                );
            }
            let source = source
                .native_line()
                .expect("a non-chord linear fillet carrier retains its native line");
            if retained_parameter.is_none() {
                let support = algebraic_chord_from_line_support(
                    source,
                    CurveOperation2::Fillet,
                    family,
                    policy,
                )?;
                let point = support
                    .normal_displaced_point_evidence(
                        center.clone(),
                        -signed_distance.clone(),
                        policy,
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                if domain == FilletContactDomain2::OpenCurve {
                    // An unpartitioned line can retain its geometric chord
                    // coordinate. Only transport into an authored multi-chart
                    // source needs the normalized affine scalar below.
                    return algebraic_chord_corner_cut_from_support_point(
                        &support,
                        point,
                        previous,
                        mode,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    );
                }
                let parameter = support
                    .parameter_at_certified_support_point(point.clone(), policy)
                    .and_then(|parameter| parameter.exact_line_curve_parameter(policy))
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                let parameter = match parameter {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            family,
                            reason,
                        ));
                    }
                };
                let placement = curve_region_corner_parameter_placement(
                    &parameter,
                    previous,
                    mode,
                    CurveOperation2::Fillet,
                    family,
                    policy,
                )?;
                let placement = domain.with_boundary_contact(
                    placement,
                    &parameter,
                    || if previous { Real::one() } else { Real::zero() }.into(),
                    family,
                    policy,
                )?;
                return Ok(placement.map(|placement| CornerCut2 {
                    point,
                    parameter: Some(parameter),
                    placement,
                }));
            }
            let parameter = retained_parameter
                .expect("a line offset intersection retains its affine parameter")
                .clone();
            let placement = curve_region_corner_parameter_placement(
                &parameter,
                previous,
                mode,
                CurveOperation2::Fillet,
                family,
                policy,
            )?;
            let placement = domain.with_boundary_contact(
                placement,
                &parameter,
                || if previous { Real::one() } else { Real::zero() }.into(),
                family,
                policy,
            )?;
            let Some(placement) = placement else {
                return Ok(None);
            };
            let point = if let Some(parameter) = parameter.scalar() {
                source.point_at(parameter.clone()).into()
            } else {
                {
                    {
                        match crate::BezierAlgebraicChord2::translated_endpoint(
                            center,
                            &(signed_distance * *unit_y),
                            &(-(signed_distance * *unit_x)),
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })? {
                            Classification::Decided(point) => point,
                            Classification::Uncertain(reason) => {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    family,
                                    reason,
                                ));
                            }
                        }
                    }
                }
            };
            Ok(Some(CornerCut2 {
                point,
                parameter: Some(parameter),
                placement,
            }))
        }
        FilletOffsetCarrier2::Arc {
            source,
            source_radius,
            signed_radius,
        } => {
            let Some(center) = center.coordinates() else {
                let radial_scale = (*source_radius / signed_radius).map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause.into())
                })?;
                let point = match crate::BezierAlgebraicChord2::scaled_about_point_endpoint(
                    center,
                    source.support().center(),
                    &radial_scale,
                    policy,
                )
                .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
                {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            family,
                            reason,
                        ));
                    }
                };
                if retained_parameter.is_none() {
                    // Keep the actual radial contact even while its source
                    // chart and placement await deferred replay. A center
                    // used as a point placeholder could become materializable
                    // later and falsely certify an off-circle fillet endpoint.
                    return Ok(Some(CornerCut2 {
                        point,
                        parameter: exact_corner_parameter(source.corner_parameter(previous)),
                        placement: CornerPlacement2::Corner,
                    }));
                }
                {
                    let parameter = retained_parameter
                        .expect("a retained arc offset intersection keeps its parameter")
                        .clone();
                    let Some(placement) = curve_region_corner_parameter_placement(
                        &parameter,
                        previous,
                        mode,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?
                    else {
                        return Ok(None);
                    };
                    return Ok(Some(CornerCut2 {
                        point,
                        parameter: Some(parameter),
                        placement,
                    }));
                }
            };
            let scale = (*source_radius / signed_radius).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause.into())
            })?;
            let support = source.support();
            let radial = center.delta_from(support.center());
            let point = source
                .support()
                .center()
                .translated(&radial.0 * &scale, &radial.1 * scale);
            arc_fillet_cut_from_incident_point(
                source,
                point,
                deferred_arc_contact,
                previous,
                mode,
                domain,
                family,
                policy,
            )
        }
        FilletOffsetCarrier2::Point { .. } => {
            unreachable!("a collapsed arc offset has no isolated tangency contact")
        }
        FilletOffsetCarrier2::Parallel { source, support } => {
            let parameter = retained_parameter
                .expect("a parallel offset intersection retains its parameter")
                .clone();
            let Some(placement) =
                source.parameter_placement(&parameter, previous, mode, domain, family, policy)?
            else {
                return Ok(None);
            };
            let (point, parameter) = match source {
                FilletParallelSource2::Direct(source) => (
                    curve_region_parallel_point_evidence(
                        support,
                        &parameter,
                        true,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?,
                    source.curve_parameter(&parameter, CurveOperation2::Fillet, family, policy)?,
                ),
                FilletParallelSource2::Retained(source) => (
                    curve_region_parallel_point_evidence(
                        source.parallel(),
                        &parameter,
                        false,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?,
                    parameter,
                ),
                FilletParallelSource2::Selected(source) => (
                    curve_region_parallel_point_evidence(
                        &source.parallel_carrier(),
                        &parameter,
                        false,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?,
                    parameter,
                ),
            };
            Ok(Some(CornerCut2 {
                point,
                parameter: Some(parameter),
                placement,
            }))
        }
        FilletOffsetCarrier2::AlgebraicChord {
            source,
            signed_distance,
            ..
        } => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-algebraic-chord-cut",
                match retained_parameter {
                    Some(parameter) if parameter.is_algebraic_chord() => "algebraic-chord",
                    Some(_) => "other",
                    None => "missing",
                },
            );
            algebraic_chord_fillet_cut_from_center(
                source,
                center,
                retained_parameter,
                signed_distance,
                previous,
                mode,
                family,
                policy,
            )
        }
        FilletOffsetCarrier2::AlgebraicCusp { source, support } => {
            let retained_parameter = retained_parameter
                .expect("a selected-circle offset contact retains its local parameter");
            let complementary = retained_parameter.is_algebraic_cusp_complement();
            let parameter = retained_parameter
                .as_algebraic_cusp()
                .cloned()
                .ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::Unsupported,
                    )
                })?;
            let strict_support_interior = if complementary {
                Classification::Decided(false)
            } else {
                let translated_pair_interior = source
                    .translated_pair_parameter_is_strict_interior(&parameter, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                match translated_pair_interior {
                    Some(Classification::Decided(interior)) => Classification::Decided(interior),
                    Some(Classification::Uncertain(_)) | None => {
                        use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::Interior;
                        match policy
                            .strict_predicate_pass(|| {
                                support.certified_incident_point_evidence_location(
                                    &parameter, center, policy,
                                )
                            })
                            .map_err(|cause| {
                                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                            })? {
                            Classification::Decided(location) => {
                                Classification::Decided(location == Interior)
                            }
                            Classification::Uncertain(_) => support
                                .certified_incident_point_evidence_is_strict_interior(
                                    center, policy,
                                )
                                .map_err(|cause| {
                                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                                })?,
                        }
                    }
                }
            };
            let placement = if complementary {
                if mode == CurveCornerMode2::TrimOrExtend {
                    CornerPlacement2::Extension
                } else {
                    return Ok(None);
                }
            } else if strict_support_interior == Classification::Decided(true) {
                CornerPlacement2::Trim
            } else if strict_support_interior == Classification::Decided(false)
                && mode != CurveCornerMode2::TrimOrExtend
            {
                // `support` is the concentric offset image of `source` over
                // the identical angular range. The center constructor has
                // already certified circle incidence, so a decided failure
                // of strict support-fragment interior proves that this
                // candidate is either an endpoint or exterior on the source
                // as well. TrimOnly rejects both; comparing independently
                // represented angular parameters cannot add information.
                return Ok(None);
            } else {
                use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::{
                    End, Exterior, Interior, Start,
                };
                // Incidence on the offset circle is already certified by the
                // intersection constructor. Its endpoint chord is therefore
                // the cheapest exact finite-domain authority and avoids
                // rebuilding two unrelated dense angular fields for a
                // retained smooth run. Parameter comparison remains the
                // complete fallback when that structural certificate cannot
                // decide.
                let incident_location = if retain_selected_circle_endpoints {
                    Some(
                        support
                            .certified_incident_point_evidence_location(&parameter, center, policy)
                            .map_err(|cause| {
                                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                            })?,
                    )
                } else {
                    None
                };
                match incident_location {
                    Some(Classification::Decided(Interior)) => CornerPlacement2::Trim,
                    Some(Classification::Decided(Start | End | Exterior))
                        if mode == CurveCornerMode2::TrimOrExtend =>
                    {
                        CornerPlacement2::Extension
                    }
                    Some(Classification::Decided(Start | End | Exterior)) => return Ok(None),
                    Some(Classification::Uncertain(_))
                        if mode == CurveCornerMode2::TrimOrExtend =>
                    {
                        // A logical selected-circle run is solved on one
                        // ancestral full support, then rebound to its authored
                        // fragments by the retained coincident-circle overlap
                        // map. Hand off an unresolved finite-domain candidate
                        // immediately: the run rebinder is its authoritative
                        // exact domain owner, while dense angular refinement
                        // here would duplicate that decision.
                        CornerPlacement2::Extension
                    }
                    Some(Classification::Uncertain(_)) | None => {
                        let source_contains = source
                            .contains_parameter(&parameter, false, false, policy)
                            .map_err(|cause| {
                                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                            })?;
                        match source_contains {
                        Classification::Decided(true) => CornerPlacement2::Trim,
                        Classification::Decided(false)
                            if retain_selected_circle_endpoints
                                && mode == CurveCornerMode2::TrimOrExtend =>
                        {
                            CornerPlacement2::Extension
                        }
                        Classification::Decided(false) => {
                            let endpoint_order = |endpoint| {
                                parameter
                                    .cmp_by_refinement(endpoint, policy)
                                    .map_err(|cause| {
                                        ExactCurveError::invalid(
                                            CurveOperation2::Fillet,
                                            family,
                                            cause,
                                        )
                                    })
                                    .and_then(|order| match order {
                                        Classification::Decided(order) => Ok(order),
                                        Classification::Uncertain(reason) => {
                                            Err(ExactCurveError::blocked(
                                                CurveOperation2::Fillet,
                                                family,
                                                reason,
                                            ))
                                        }
                                    })
                            };
                            let start_order = endpoint_order(source.start_parameter())?;
                            let end_order = endpoint_order(source.end_parameter())?;
                            if start_order == std::cmp::Ordering::Equal
                                || end_order == std::cmp::Ordering::Equal
                            {
                                return Ok(None);
                            }
                            if mode == CurveCornerMode2::TrimOrExtend {
                                CornerPlacement2::Extension
                            } else {
                                return Ok(None);
                            }
                        }
                        Classification::Uncertain(parameter_reason)
                            if retain_selected_circle_endpoints =>
                        {
                            match incident_location.expect(
                                "selected-circle endpoint retention computed its incident location",
                            ) {
                                Classification::Decided(Interior) => CornerPlacement2::Trim,
                                Classification::Decided(Start | End | Exterior)
                                    if mode == CurveCornerMode2::TrimOrExtend =>
                                {
                                    CornerPlacement2::Extension
                                }
                                Classification::Decided(Start | End | Exterior) => return Ok(None),
                                Classification::Uncertain(_) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        family,
                                        parameter_reason,
                                    ));
                                }
                            }
                        }
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                reason,
                            ));
                        }
                    }
                    }
                }
            };
            let complementary_support;
            let complementary_source;
            let (support_semicircle, source_semicircle) = if complementary {
                complementary_support = support.semicircle().complementary_half();
                complementary_source = source.semicircle().complementary_half();
                (&complementary_support, &complementary_source)
            } else {
                (support.semicircle(), source.semicircle())
            };
            let point = if let (Some(center), Some(support_center)) = (
                center.coordinates(),
                support_semicircle.exact_center(policy).map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })?,
            ) {
                // The selected offset contact and circle center already live
                // in canonical Real. Replay the concentric radial map there
                // instead of wrapping the same coordinates in a selected
                // point image that reconstruction would immediately have to
                // eliminate again.
                let radial_scale = (source_semicircle.radial_distance()
                    / support_semicircle.radial_distance())
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause.into())
                })?;
                let radial = center.delta_from(&support_center);
                CurvePoint2::from(
                    support_center.translated(&radial.0 * &radial_scale, &radial.1 * radial_scale),
                )
            } else {
                let point = parameter
                    .concentric_offset_point_evidence(support_semicircle, source_semicircle, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })?;
                match point {
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
                }
            };
            Ok(Some(CornerCut2 {
                point,
                parameter: Some(if complementary {
                    CurveParameter2::from_algebraic_cusp_complement(parameter)
                } else {
                    CurveParameter2::from_algebraic_cusp(parameter)
                }),
                placement,
            }))
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn algebraic_chord_fillet_cut_from_center(
    source: &crate::BezierAlgebraicChord2,
    center: &CurvePoint2,
    retained_parameter: Option<&CurveParameter2>,
    signed_distance: &Real,
    previous: bool,
    mode: CurveCornerMode2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    let point = source
        .normal_displaced_point_evidence(center.clone(), -signed_distance.clone(), policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    if let Some(retained_parameter) = retained_parameter {
        let placement = if retained_parameter.is_retained_scalar() {
            curve_region_corner_parameter_placement(
                retained_parameter,
                previous,
                mode,
                CurveOperation2::Fillet,
                family,
                policy,
            )?
        } else if let Some(retained_parameter) = retained_parameter.as_algebraic_chord() {
            algebraic_chord_parallel_parameter_placement(
                source,
                retained_parameter,
                previous,
                mode,
                family,
                policy,
            )?
        } else {
            None
        };
        if let Some(placement) = placement {
            let parameter = source
                .parameter_at_certified_support_point(point.clone(), policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })?;
            return Ok(Some(CornerCut2 {
                point,
                parameter: Some(CurveParameter2::from_algebraic_chord(parameter)),
                placement,
            }));
        }
        return Ok(None);
    }
    algebraic_chord_corner_cut_from_support_point(
        source,
        point,
        previous,
        mode,
        CurveOperation2::Fillet,
        family,
        policy,
    )
}

fn algebraic_chord_parallel_parameter_placement(
    source: &crate::BezierAlgebraicChord2,
    parameter: &crate::bezier_offset::BezierAlgebraicChordParameter2,
    previous: bool,
    mode: CurveCornerMode2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let chord = parameter.chord();
    let reversed = chord
        .retained_normal_offset_tangent_reversal_to(source)
        .ok_or_else(|| {
            ExactCurveError::invalid(
                CurveOperation2::Fillet,
                family,
                CurveError::Topology(
                    "a retained fillet contact did not descend from its source chord".into(),
                ),
            )
        })?;
    if parameter.has_certified_strict_interior_location() {
        return Ok(Some(CornerPlacement2::Trim));
    }
    let compare = |boundary| {
        parameter
            .cmp_by_refinement(boundary, policy)
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
    let start = chord.start_parameter();
    let end = chord.end_parameter();
    let start_order = compare(&start)?;
    let end_order = compare(&end)?;
    if start_order.is_gt() && end_order.is_lt() {
        return Ok(Some(CornerPlacement2::Trim));
    }
    let extends_toward_end = previous != reversed;
    Ok((mode == CurveCornerMode2::TrimOrExtend
        && ((extends_toward_end && end_order.is_gt())
            || (!extends_toward_end && start_order.is_lt())))
    .then_some(CornerPlacement2::Extension))
}

#[allow(clippy::too_many_arguments)]
fn algebraic_chord_corner_cut_from_support_point(
    source: &crate::BezierAlgebraicChord2,
    point: CurvePoint2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    let parameter = source
        .parameter_at_certified_support_point(point.clone(), policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    let start = source.start_parameter();
    let end = source.end_parameter();
    let compare = |boundary| {
        parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|order| match order {
                Classification::Decided(order) => Ok(order),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let start_order = compare(&start)?;
    let end_order = compare(&end)?;
    let placement = if start_order.is_gt() && end_order.is_lt() {
        CornerPlacement2::Trim
    } else if mode == CurveCornerMode2::TrimOrExtend
        && ((previous && end_order.is_gt()) || (!previous && start_order.is_lt()))
    {
        CornerPlacement2::Extension
    } else {
        return Ok(None);
    };
    Ok(Some(CornerCut2 {
        point,
        parameter: Some(CurveParameter2::from_algebraic_chord(parameter)),
        placement,
    }))
}

fn line_parameter_at_point(
    line: &LineSeg2,
    point: &Point2,
    operation: CurveOperation2,
    family: CurveFamily2,
) -> ExactCurveResult<Real> {
    let delta = line.delta();
    let from_start = point.delta_from(line.start());
    let numerator = &from_start.0 * &delta.0 + &from_start.1 * &delta.1;
    let denominator = &delta.0 * &delta.0 + &delta.1 * &delta.1;
    (numerator / denominator)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))
}

/// Proves that an exact line witness carries the same finite parameter domain
/// as an algebraic chord. Canonical unit witnesses otherwise name only the
/// infinite support and must not classify trim/extension against `[0, 1]`.
fn algebraic_chord_domain_matches_line_witness(
    source: &crate::BezierAlgebraicChord2,
    witness: &LineSeg2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    if let Some(line) = source.exact_line() {
        return Ok(
            line == *witness || (line.start() == witness.end() && line.end() == witness.start())
        );
    }
    let Some(direction) = source.certified_axis_direction() else {
        return Ok(false);
    };
    let axis = direction.axis();
    let coordinate = |point: &Point2| match axis {
        crate::Axis2::X => point.x().clone(),
        crate::Axis2::Y => point.y().clone(),
    };
    // Prepared algebraic axis chords use a canonical unit witness solely to
    // name their affine support.  Its active coordinate is structurally
    // `[0, +/-1]`; final cut publication still classifies every center on the
    // authored algebraic chord.  Treat that witness as unbounded immediately
    // instead of trying to rediscover two potentially recursive endpoint
    // equalities through the terminal predicate schedule.
    let witness_start = coordinate(witness.start());
    let witness_end = coordinate(witness.end());
    if witness_start.zero_status() == hyperreal::ZeroKnowledge::Zero
        && ((&witness_end - &witness_start).abs() - Real::one()).zero_status()
            == hyperreal::ZeroKnowledge::Zero
    {
        return Ok(false);
    }
    let equal = |point, value| {
        policy
            .strict_predicate_pass(|| {
                crate::BezierAlgebraicChord2::point_axis_order_to_real(point, axis, value, policy)
            })
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))
            .map(|order| order == Classification::Decided(std::cmp::Ordering::Equal))
    };
    let forward = equal(source.start(), &witness_start)? && equal(source.end(), &witness_end)?;
    if forward {
        return Ok(true);
    }
    Ok(equal(source.start(), &witness_end)? && equal(source.end(), &witness_start)?)
}

fn algebraic_chord_from_line_support(
    line: &LineSeg2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<crate::BezierAlgebraicChord2> {
    match crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
        CurvePoint2::from(line.start().clone()),
        CurvePoint2::from(line.end().clone()),
        policy,
    )
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(chord) => Ok(chord),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, family, reason))
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn solve_line_fillet_corner(
    previous: &LineSeg2,
    next: &LineSeg2,
    radius: &Real,
    mode: CurveCornerMode2,
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    let previous_delta = previous.delta();
    let next_delta = next.delta();
    let previous_unit = line_unit_direction(
        &previous_delta.0,
        &previous_delta.1,
        CurveOperation2::Fillet,
        previous_family,
        policy,
    )?;
    let next_unit = line_unit_direction(
        &next_delta.0,
        &next_delta.1,
        CurveOperation2::Fillet,
        next_family,
        policy,
    )?;
    let denominator = &previous_delta.0 * &next_delta.1 - &previous_delta.1 * &next_delta.0;
    let denominator_sign = match crate::classify::real_sign(&denominator, policy) {
        Some(RealSign::Zero) => {
            return Ok(CurveCornerSolutions2::NoSolution(
                CurveCornerNoSolution2::ParallelTangents,
            ));
        }
        Some(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        None => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                previous_family,
                crate::UncertaintyReason::RealSign,
            ));
        }
    };
    let denominator_reciprocal = denominator
        .inverse_ref_assuming_nonzero()
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause.into())
        })?;

    let mut candidates = CornerSolutionAccumulator::Empty;
    // For connected incoming/outgoing lines, only the offset side matching the
    // turn can have both contacts in the open trim domains. Extension mode must
    // retain both exact carrier solutions.
    let sides: &[(bool, bool)] = match (mode, denominator_sign) {
        (CurveCornerMode2::TrimOnly, RealSign::Positive) => &[(true, false)],
        (CurveCornerMode2::TrimOnly, RealSign::Negative) => &[(false, true)],
        (CurveCornerMode2::TrimOrExtend, _) => &[(true, false), (false, true)],
        (_, RealSign::Zero) => unreachable!("parallel line directions return before solving"),
    };
    for &(positive_radius, clockwise) in sides {
        let signed_radius = if positive_radius {
            radius.clone()
        } else {
            -radius.clone()
        };
        let previous_offset_start = previous.start().translated(
            -&previous_unit.1 * &signed_radius,
            &previous_unit.0 * &signed_radius,
        );
        let next_offset_start = next.start().translated(
            -&next_unit.1 * &signed_radius,
            &next_unit.0 * &signed_radius,
        );
        let between_offsets = next_offset_start.delta_from(&previous_offset_start);
        let previous_numerator =
            &between_offsets.0 * &next_delta.1 - &between_offsets.1 * &next_delta.0;
        let next_numerator =
            &between_offsets.0 * &previous_delta.1 - &between_offsets.1 * &previous_delta.0;
        let previous_parameter = previous_numerator * &denominator_reciprocal;
        let next_parameter = next_numerator * &denominator_reciprocal;
        let Some(previous_placement) = corner_parameter_placement(
            &previous_parameter,
            true,
            mode,
            CurveOperation2::Fillet,
            previous_family,
            policy,
        )?
        else {
            continue;
        };
        let Some(next_placement) = corner_parameter_placement(
            &next_parameter,
            false,
            mode,
            CurveOperation2::Fillet,
            next_family,
            policy,
        )?
        else {
            continue;
        };
        let previous_point = previous.point_at(previous_parameter.clone());
        let next_point = next.point_at(next_parameter.clone());
        let center = previous_offset_start.translated(
            &previous_delta.0 * &previous_parameter,
            &previous_delta.1 * &previous_parameter,
        );
        match crate::classify::is_zero(&previous_point.distance_squared(&next_point), policy) {
            Some(true) => continue,
            Some(false) => candidates.push(FilletCorner2 {
                previous: CornerCut2 {
                    parameter: exact_corner_parameter(previous_parameter),
                    point: previous_point.into(),
                    placement: previous_placement,
                },
                next: CornerCut2 {
                    parameter: exact_corner_parameter(next_parameter),
                    point: next_point.into(),
                    placement: next_placement,
                },
                center: center.into(),
                clockwise,
                retained_frame: None,
            }),
            None => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    previous_family,
                    crate::UncertaintyReason::RealSign,
                ));
            }
        }
    }
    Ok(candidates.finish(CurveCornerNoSolution2::OutsideTrimDomain))
}

fn line_unit_direction(
    dx: &Real,
    dy: &Real,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<(Real, Real, Real)> {
    // Axis-aligned edges are common and their exact norm is already one
    // coordinate; avoid constructing and reducing a redundant square root.
    match (dx.structural_facts().sign, dy.structural_facts().sign) {
        (Some(RealSign::Zero), Some(RealSign::Positive)) => {
            return Ok((Real::zero(), Real::one(), dy.clone()));
        }
        (Some(RealSign::Zero), Some(RealSign::Negative)) => {
            return Ok((Real::zero(), -Real::one(), -dy.clone()));
        }
        (Some(RealSign::Positive), Some(RealSign::Zero)) => {
            return Ok((Real::one(), Real::zero(), dx.clone()));
        }
        (Some(RealSign::Negative), Some(RealSign::Zero)) => {
            return Ok((-Real::one(), Real::zero(), -dx.clone()));
        }
        _ => {}
    }
    let length_squared = dx * dx + dy * dy;
    match crate::classify::real_sign(&length_squared, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Err(ExactCurveError::invalid(
                operation,
                family,
                CurveError::ZeroLengthLine,
            ));
        }
        None => {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::RealSign,
            ));
        }
    }
    let length = length_squared
        .sqrt()
        .map_err(|cause| ExactCurveError::invalid(operation, family, CurveError::from(cause)))?;
    let unit_x = (dx / &length)
        .map_err(|cause| ExactCurveError::invalid(operation, family, CurveError::from(cause)))?;
    let unit_y = (dy / &length)
        .map_err(|cause| ExactCurveError::invalid(operation, family, CurveError::from(cause)))?;
    Ok((unit_x, unit_y, length))
}

fn compare_corner_parameter(
    left: &Real,
    right: &Real,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<std::cmp::Ordering> {
    crate::classify::compare_reals(left, right, policy).ok_or_else(|| {
        ExactCurveError::blocked(operation, family, crate::UncertaintyReason::Ordering)
    })
}

fn corner_parameter_placement(
    parameter: &Real,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let zero_order = compare_corner_parameter(parameter, &Real::zero(), operation, family, policy)?;
    let one_order = compare_corner_parameter(parameter, &Real::one(), operation, family, policy)?;
    if zero_order == std::cmp::Ordering::Greater && one_order == std::cmp::Ordering::Less {
        return Ok(Some(CornerPlacement2::Trim));
    }
    if mode == CurveCornerMode2::TrimOrExtend
        && ((previous && one_order == std::cmp::Ordering::Greater)
            || (!previous && zero_order == std::cmp::Ordering::Less))
    {
        return Ok(Some(CornerPlacement2::Extension));
    }
    Ok(None)
}

fn bezier_trim_parameter_is_interior(
    parameter: &BezierParameter2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    Ok(bezier_corner_parameter_placement(
        parameter,
        false,
        CurveCornerMode2::TrimOnly,
        operation,
        family,
        policy,
    )? == Some(CornerPlacement2::Trim))
}

fn bezier_corner_parameter_placement(
    parameter: &BezierParameter2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    let compare = |boundary: &BezierParameter2| {
        parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|result| match result {
                Classification::Decided(order) => Ok(order),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let zero_order = compare(&zero)?;
    let one_order = compare(&one)?;
    if zero_order == std::cmp::Ordering::Greater && one_order == std::cmp::Ordering::Less {
        return Ok(Some(CornerPlacement2::Trim));
    }
    if mode == CurveCornerMode2::TrimOrExtend
        && ((previous && one_order == std::cmp::Ordering::Greater)
            || (!previous && zero_order == std::cmp::Ordering::Less))
    {
        return Ok(Some(CornerPlacement2::Extension));
    }
    Ok(None)
}

fn curve_region_corner_parameter_placement(
    parameter: &CurveParameter2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let zero = CurveParameter2::from(BezierParameter2::Exact(Real::zero()));
    let one = CurveParameter2::from(BezierParameter2::Exact(Real::one()));
    let compare = |boundary: &CurveParameter2| {
        parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|result| match result {
                Classification::Decided(order) => Ok(order),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let zero_order = compare(&zero)?;
    let one_order = compare(&one)?;
    if zero_order == std::cmp::Ordering::Greater && one_order == std::cmp::Ordering::Less {
        return Ok(Some(CornerPlacement2::Trim));
    }
    if mode == CurveCornerMode2::TrimOrExtend
        && ((previous && one_order == std::cmp::Ordering::Greater)
            || (!previous && zero_order == std::cmp::Ordering::Less))
    {
        return Ok(Some(CornerPlacement2::Extension));
    }
    Ok(None)
}

fn decided_parallel_point(
    parallel: &BezierParallel2,
    parameter: &Real,
    source_point: bool,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Point2> {
    let point = if source_point {
        parallel.source_point_at(parameter, policy)
    } else {
        parallel.point_at_affine(parameter, policy)
    }
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    match point {
        Classification::Decided(point) => Ok(point),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, family, reason))
        }
    }
}

fn bezier_parallel_source_point_evidence(
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurvePoint2> {
    if let Some(parameter) = parameter.scalar() {
        return match parallel.source_point_at_unchecked(parameter, policy) {
            Classification::Decided(point) => Ok(point.into()),
            Classification::Uncertain(reason) => {
                Err(ExactCurveError::blocked(operation, family, reason))
            }
        };
    }
    let rational_source = bezier_parallel_rational_source(parallel, operation, family)?;
    if let Some(point) = crate::rational_bezier_general::exact_contact_point_evidence(
        &rational_source,
        parameter,
        policy,
    )
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        return Ok(point);
    }
    {
        // A selected fiber can have non-rational coefficients even though the
        // source curve is rational. Keep the source point in the same procedural
        // normalized-frame carrier used by analytic parallels instead of rejecting
        // an exact parameter merely because a one-field coordinate image was not
        // profitable to materialize.
        Ok(CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new(
            parallel.with_distance(Real::zero()),
            parameter.clone(),
            policy,
        )))
    }
}

fn curve_region_parallel_point_evidence(
    parallel: &BezierParallel2,
    parameter: &CurveParameter2,
    source_point: bool,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurvePoint2> {
    if source_point {
        if let Some(parameter) = parameter.as_bezier_parameter() {
            return bezier_parallel_source_point_evidence(
                parallel, parameter, operation, family, policy,
            );
        }
        return analytic_parallel_point_evidence(
            &parallel.with_distance(Real::zero()),
            parameter,
            operation,
            family,
            policy,
        );
    }
    analytic_parallel_point_evidence(parallel, parameter, operation, family, policy)
}

fn bezier_parallel_rational_source(
    parallel: &BezierParallel2,
    operation: CurveOperation2,
    family: CurveFamily2,
) -> ExactCurveResult<RationalBezier2> {
    match parallel.source() {
        crate::BezierParallelSource2::Quadratic(curve) => {
            RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(curve.clone()))
        }
        crate::BezierParallelSource2::Cubic(curve) => {
            RationalBezier2::try_from_subcurve(&BezierSubcurve2::Cubic(curve.clone()))
        }
        crate::BezierParallelSource2::Rational(curve) => Ok(curve.clone()),
    }
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
}

#[allow(clippy::too_many_arguments)]
fn corner_chamfer_cuts(
    carrier: ExactCornerCarrier2<'_>,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    logical_run: bool,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    match carrier {
        ExactCornerCarrier2::Line(source) => line_chamfer_cuts(
            source,
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::PromotedLine(curve) => line_chamfer_cuts(
            curve
                .retained_exact_line_image()
                .expect("a promoted-line carrier retains its exact line image"),
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::Arc(arc) => arc_chamfer_cuts(
            ExactCornerArc2::Native(arc),
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::RetainedRationalArc(arc) => arc_chamfer_cuts(
            ExactCornerArc2::RetainedRational(arc),
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::Bezier(source) => bezier_chamfer_cuts(
            ExactCornerBezier2::Direct(source),
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::NativeBezierSpan(fragment) => bezier_chamfer_cuts(
            ExactCornerBezier2::NativeSpan(fragment),
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::AlgebraicChord(chord) => algebraic_chord_chamfer_cuts(
            chord,
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::AnalyticParallel(fragment) => analytic_parallel_chamfer_cuts(
            fragment,
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::SelectedFiber(fragment) => selected_fiber_chamfer_cuts(
            fragment,
            setback,
            setback_sign,
            previous,
            mode,
            operation,
            family,
            policy,
        ),
        ExactCornerCarrier2::AlgebraicCusp(fragment) => algebraic_cusp_chamfer_cuts(
            fragment,
            setback,
            setback_sign,
            previous,
            mode,
            logical_run,
            operation,
            family,
            policy,
        ),
    }
}

#[allow(clippy::too_many_arguments)]
fn algebraic_cusp_chamfer_cuts(
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    logical_run: bool,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    fragment
        .validate_policy(policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    let start_endpoint = !previous;
    let corner_parameter = fragment.endpoint_parameter(start_endpoint).clone();
    let corner = match fragment
        .endpoint_point_evidence(start_endpoint, policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(Some(point)) => point,
        Classification::Decided(None) => {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::Unsupported,
            ));
        }
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    };
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(CurveParameter2::from_algebraic_cusp(corner_parameter)),
                point: corner,
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let mut cuts = CornerCuts2::default();
    for (outward, placement) in [
        (false, CornerPlacement2::Trim),
        (true, CornerPlacement2::Extension),
    ] {
        if outward && mode != CurveCornerMode2::TrimOrExtend {
            continue;
        }
        let cut = match fragment
            .endpoint_chord_setback_cut(start_endpoint, setback, outward, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(Some(cut)) => Some(cut),
            Classification::Decided(None) if logical_run && !outward => {
                match fragment
                    .endpoint_chord_setback_support_cut(start_endpoint, setback, policy)
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
                {
                    Classification::Decided(cut) => cut,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(operation, family, reason));
                    }
                }
            }
            Classification::Decided(None) => None,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, family, reason));
            }
        };
        let Some((parameter, point, complementary)) = cut else {
            continue;
        };
        let parameter = if complementary {
            CurveParameter2::from_algebraic_cusp_complement(parameter)
        } else {
            CurveParameter2::from_algebraic_cusp(parameter)
        };
        cuts.push(CornerCut2 {
            parameter: Some(parameter),
            point,
            placement,
        });
    }
    Ok(cuts)
}

#[allow(clippy::too_many_arguments)]
fn algebraic_chord_chamfer_cuts(
    chord: &crate::BezierAlgebraicChord2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    chord
        .validate_policy(policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    let corner_parameter = if previous {
        chord.end_parameter()
    } else {
        chord.start_parameter()
    };
    let corner = if previous { chord.end() } else { chord.start() };
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(CurveParameter2::from_algebraic_chord(corner_parameter)),
                point: corner.clone(),
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let interior_distance = if previous {
        -setback.clone()
    } else {
        setback.clone()
    };
    let mut cuts = CornerCuts2::default();
    let distances = if mode == CurveCornerMode2::TrimOrExtend {
        [Some(interior_distance.clone()), Some(-interior_distance)]
    } else {
        [Some(interior_distance), None]
    };
    for distance in distances.into_iter().flatten() {
        // Unit-tangent displacement is construction evidence for support
        // incidence. General endpoint fields remain separate behind one lazy
        // normalized expression; only finite-domain placement is a predicate.
        let point = match chord
            .endpoint_at_signed_tangent_distance(previous, distance, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, family, reason));
            }
        };
        if let Some(cut) = algebraic_chord_corner_cut_from_support_point(
            chord, point, previous, mode, operation, family, policy,
        )? {
            cuts.push(cut);
        }
    }
    Ok(cuts)
}

fn analytic_parallel_point_evidence(
    parallel: &BezierParallel2,
    parameter: &CurveParameter2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurvePoint2> {
    if let Some(parameter) = parameter
        .as_bezier_parameter()
        .and_then(BezierParameter2::scalar)
    {
        return decided_parallel_point(parallel, parameter, false, operation, family, policy)
            .map(Into::into);
    }
    crate::BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
        parallel.clone(),
        parameter,
        Real::zero(),
        policy,
    )
    .map(CurvePoint2::from)
    .ok_or_else(|| {
        ExactCurveError::blocked(operation, family, crate::UncertaintyReason::Unsupported)
    })
}

fn retained_parallel_corner_parameter_placement(
    parameter: &CurveParameter2,
    fragment: &crate::BezierParallelFragment2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let compare = |boundary: &BezierParameter2| {
        parameter
            .cmp_by_refinement(&CurveParameter2::from(boundary.clone()), policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|ordering| match ordering {
                Classification::Decided(ordering) => Ok(ordering),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let start_order = compare(fragment.range().start())?;
    let end_order = compare(fragment.range().end())?;
    Ok(retained_parallel_corner_orders_placement(
        start_order,
        end_order,
        fragment,
        previous,
        mode,
    ))
}

fn retained_parallel_corner_orders_placement(
    start_order: std::cmp::Ordering,
    end_order: std::cmp::Ordering,
    fragment: &crate::BezierParallelFragment2,
    previous: bool,
    mode: CurveCornerMode2,
) -> Option<CornerPlacement2> {
    if start_order.is_gt() && end_order.is_lt() {
        return Some(CornerPlacement2::Trim);
    }
    if mode != CurveCornerMode2::TrimOrExtend {
        return None;
    }
    let extends_toward_higher_parameter = previous != fragment.is_reversed();
    ((extends_toward_higher_parameter && end_order.is_gt())
        || (!extends_toward_higher_parameter && start_order.is_lt()))
    .then_some(CornerPlacement2::Extension)
}

#[allow(clippy::too_many_arguments)]
fn selected_fiber_corner_parameter_placement(
    parameter: &CurveParameter2,
    fragment: &crate::bezier_split::BezierSelectedFiberFragment2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let compare = |boundary: &CurveParameter2| {
        parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|ordering| match ordering {
                Classification::Decided(ordering) => Ok(ordering),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let start_order = compare(fragment.range().start())?;
    let end_order = compare(fragment.range().end())?;
    if start_order.is_gt() && end_order.is_lt() {
        return Ok(Some(CornerPlacement2::Trim));
    }
    if mode != CurveCornerMode2::TrimOrExtend {
        return Ok(None);
    }
    let extends_toward_higher_parameter = previous != fragment.is_reversed();
    Ok(((extends_toward_higher_parameter && end_order.is_gt())
        || (!extends_toward_higher_parameter && start_order.is_lt()))
    .then_some(CornerPlacement2::Extension))
}

#[allow(clippy::too_many_arguments)]
fn selected_fiber_chamfer_cuts(
    fragment: &crate::bezier_split::BezierSelectedFiberFragment2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    let corner_parameter = if previous != fragment.is_reversed() {
        fragment.range().end()
    } else {
        fragment.range().start()
    };
    let corner = if previous {
        fragment.end_point().clone()
    } else {
        fragment.start_point().clone()
    };
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(corner_parameter.clone()),
                point: corner,
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let parallel = fragment.parallel_carrier();
    let direction = if previous != fragment.is_reversed() {
        crate::BezierParameterRayDirection2::Increasing
    } else {
        crate::BezierParameterRayDirection2::Decreasing
    };
    let parameters = match parallel
        .fixed_distance_incidence(
            &parallel,
            corner_parameter,
            setback,
            fragment.range(),
            (mode == CurveCornerMode2::TrimOrExtend).then_some(direction),
            policy,
        )
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    };
    let mut cuts = CornerCuts2::default();
    for parameter in parameters {
        let Some(placement) = selected_fiber_corner_parameter_placement(
            &parameter, fragment, previous, mode, operation, family, policy,
        )?
        else {
            continue;
        };
        let point =
            analytic_parallel_point_evidence(&parallel, &parameter, operation, family, policy)?;
        cuts.push(CornerCut2 {
            parameter: Some(parameter),
            point,
            placement,
        });
    }
    Ok(cuts)
}

#[allow(clippy::too_many_arguments)]
fn analytic_parallel_chamfer_cuts(
    fragment: &crate::BezierParallelFragment2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    let corner_parameter = match (previous, fragment.is_reversed()) {
        (true, false) | (false, true) => fragment.range().end(),
        (true, true) | (false, false) => fragment.range().start(),
    };
    let corner = analytic_parallel_point_evidence(
        fragment.parallel(),
        &corner_parameter.clone().into(),
        operation,
        family,
        policy,
    )?;
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(CurveParameter2::from(corner_parameter.clone())),
                point: corner,
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let direction = if previous != fragment.is_reversed() {
        crate::BezierParameterRayDirection2::Increasing
    } else {
        crate::BezierParameterRayDirection2::Decreasing
    };
    let parameters = match fragment
        .parallel()
        .fixed_distance_incidence(
            fragment.parallel(),
            &CurveParameter2::from(corner_parameter.clone()),
            setback,
            &crate::CurveParameterRange2::new_validated(
                fragment.range().start().clone().into(),
                fragment.range().end().clone().into(),
            ),
            (mode == CurveCornerMode2::TrimOrExtend).then_some(direction),
            policy,
        )
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    };
    let mut cuts = CornerCuts2::default();
    for parameter in parameters {
        let Some(placement) = retained_parallel_corner_parameter_placement(
            &parameter, fragment, previous, mode, operation, family, policy,
        )?
        else {
            continue;
        };
        let point = analytic_parallel_point_evidence(
            fragment.parallel(),
            &parameter,
            operation,
            family,
            policy,
        )?;
        cuts.push(CornerCut2 {
            parameter: Some(parameter),
            point,
            placement,
        });
    }
    Ok(cuts)
}

#[allow(clippy::too_many_arguments)]
fn bezier_chamfer_cuts(
    source: ExactCornerBezier2<'_>,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    let corner = source.corner(previous);
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(source.curve_parameter(
                    &if previous { Real::one() } else { Real::zero() }.into(),
                    operation,
                    family,
                    policy,
                )?),
                point: corner.clone().into(),
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }

    let radius_squared = setback * setback;
    let parallel = exact_corner_bezier_parallel(source, Real::zero(), operation, family)?;
    let mut parameters = match parallel
        .source_circle_incidence(corner, &radius_squared, policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    };
    if mode == CurveCornerMode2::TrimOrExtend {
        let (anchor, direction) = if previous {
            (Real::one(), crate::BezierParameterRayDirection2::Increasing)
        } else {
            (
                Real::zero(),
                crate::BezierParameterRayDirection2::Decreasing,
            )
        };
        let exterior = match parallel
            .source_circle_incidence_on_incident_ray(
                corner,
                &radius_squared,
                &anchor,
                direction,
                policy,
            )
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
        {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, family, reason));
            }
        };
        parameters.extend(exterior);
    }
    let mut cuts = CornerCuts2::default();
    for parameter in parameters {
        let Some(placement) = bezier_corner_parameter_placement(
            &parameter, previous, mode, operation, family, policy,
        )?
        else {
            continue;
        };
        let point = bezier_parallel_source_point_evidence(
            &parallel, &parameter, operation, family, policy,
        )?;
        let parameter =
            Some(source.curve_parameter(&parameter.into(), operation, family, policy)?);
        cuts.push(CornerCut2 {
            parameter,
            point,
            placement,
        });
    }
    Ok(cuts)
}

#[allow(clippy::too_many_arguments)]
fn arc_chamfer_cuts(
    arc: ExactCornerArc2<'_>,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    let support = arc.support();
    validate_exact_corner_arc_support(support, operation, family, policy)?;
    let corner = if previous {
        support.end()
    } else {
        support.start()
    };
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: exact_corner_parameter(arc.corner_parameter(previous)),
                point: corner.clone().into(),
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }

    let setback_squared = setback * setback;
    let relation = crate::intersect::circle_relation_from_supports(
        support.center(),
        support.radius_squared_ref(),
        corner,
        &setback_squared,
        policy,
    )
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    let mut cuts = CornerCuts2::default();
    match relation {
        crate::CircleCircleRelation::Disjoint => {}
        crate::CircleCircleRelation::Tangent { point } => arc_chamfer_cut_candidate(
            &arc, point, previous, mode, operation, family, policy, &mut cuts,
        )?,
        crate::CircleCircleRelation::Secant {
            first_point,
            second_point,
        } => {
            arc_chamfer_cut_candidate(
                &arc,
                first_point,
                previous,
                mode,
                operation,
                family,
                policy,
                &mut cuts,
            )?;
            arc_chamfer_cut_candidate(
                &arc,
                second_point,
                previous,
                mode,
                operation,
                family,
                policy,
                &mut cuts,
            )?;
        }
        crate::CircleCircleRelation::Coincident => {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::Unsupported,
            ));
        }
        crate::CircleCircleRelation::Uncertain { reason } => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    }
    Ok(cuts)
}

#[allow(clippy::too_many_arguments)]
fn arc_chamfer_cut_candidate(
    arc: &ExactCornerArc2<'_>,
    point: Point2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
    cuts: &mut CornerCuts2,
) -> ExactCurveResult<()> {
    if let Some(cut) =
        arc_corner_cut_from_incident_point(arc, point, previous, mode, operation, family, policy)?
    {
        cuts.push(cut);
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn arc_corner_cut_from_incident_point(
    arc: &ExactCornerArc2<'_>,
    point: Point2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    // The chamfer circle relation or fillet offset/contact construction has
    // already certified source-support incidence. Re-expanding the radical
    // construction through `contains_point` would ask Hyperreal to rediscover
    // that equality and can block STRICT on an otherwise exact square-root
    // representation. Only sweep membership is a new predicate here.
    let support = arc.support();
    match support.contains_sweep_point(&point, policy) {
        Classification::Decided(true) => {
            let sweep_fraction = match support
                .sweep_fraction_for_incident_point(&point, policy)
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(operation, family, reason));
                }
            };
            if corner_parameter_placement(
                &sweep_fraction,
                previous,
                CurveCornerMode2::TrimOnly,
                operation,
                family,
                policy,
            )? == Some(CornerPlacement2::Trim)
            {
                return Ok(Some(CornerCut2 {
                    parameter: match arc
                        .source_parameter_at_point(&point, operation, family, policy)?
                    {
                        Some(parameter) => Some(CurveParameter2::from(parameter)),
                        None => exact_corner_parameter(sweep_fraction),
                    },
                    point: point.into(),
                    placement: CornerPlacement2::Trim,
                }));
            }
        }
        Classification::Decided(false) if mode == CurveCornerMode2::TrimOrExtend => {
            if arc_extension_contains_corner(support, &point, previous, operation, family, policy)?
            {
                return Ok(Some(CornerCut2 {
                    parameter: exact_corner_parameter(arc.corner_parameter(previous)),
                    point: point.into(),
                    placement: CornerPlacement2::Extension,
                }));
            }
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    }
    Ok(None)
}

pub(crate) fn arc_fillet_cut_from_incident_point(
    arc: &ExactCornerArc2<'_>,
    point: Point2,
    deferred_arc_contact: bool,
    previous: bool,
    mode: CurveCornerMode2,
    domain: FilletContactDomain2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    use crate::segment::ArcSweepPointLocation2;

    let support = arc.support();
    match support.strict_sweep_point_location(&point, policy) {
        Classification::Decided(ArcSweepPointLocation2::Interior) => {
            let parameter = if deferred_arc_contact {
                exact_corner_parameter(arc.corner_parameter(previous))
            } else {
                arc.source_parameter_at_point(&point, CurveOperation2::Fillet, family, policy)?
                    .map(CurveParameter2::from)
            };
            Ok(Some(CornerCut2 {
                parameter,
                point: point.into(),
                placement: CornerPlacement2::Trim,
            }))
        }
        Classification::Decided(ArcSweepPointLocation2::Endpoint) => {
            if domain == FilletContactDomain2::OpenCurve {
                return Ok(None);
            }
            let parameter = arc
                .source_parameter_at_point(&point, CurveOperation2::Fillet, family, policy)?
                .map(CurveParameter2::from)
                .expect("a finite circular chart retains its rational parameter");
            let placement = domain.with_boundary_contact(
                None,
                &parameter,
                || arc.corner_parameter(previous).into(),
                family,
                policy,
            )?;
            Ok(placement.map(|placement| CornerCut2 {
                point: point.into(),
                parameter: Some(parameter),
                placement,
            }))
        }
        Classification::Decided(ArcSweepPointLocation2::Outside)
            if mode == CurveCornerMode2::TrimOrExtend =>
        {
            if arc_extension_contains_corner(
                support,
                &point,
                previous,
                CurveOperation2::Fillet,
                family,
                policy,
            )? {
                Ok(Some(CornerCut2 {
                    // Retained CurveRegion reconstruction replaces this
                    // endpoint marker from exact circular-contact evidence.
                    // Native CurvePath materialization uses `point` directly.
                    parameter: exact_corner_parameter(arc.corner_parameter(previous)),
                    point: point.into(),
                    placement: CornerPlacement2::Extension,
                }))
            } else {
                Ok(None)
            }
        }
        Classification::Decided(ArcSweepPointLocation2::Outside) => Ok(None),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

fn validate_exact_corner_arc_support(
    arc: &CircularArc2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    match crate::classify::real_sign(arc.radius_squared_ref(), policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero) => {
            return Err(ExactCurveError::invalid(
                operation,
                family,
                CurveError::ZeroRadiusArc,
            ));
        }
        Some(RealSign::Negative) => {
            return Err(ExactCurveError::invalid(
                operation,
                family,
                CurveError::RadiusMismatch,
            ));
        }
        None => {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::RealSign,
            ));
        }
    }
    if !arc.endpoints_on_stored_circle_are_certified() {
        for endpoint in [arc.start(), arc.end()] {
            let radius_delta = endpoint.distance_squared(arc.center()) - arc.radius_squared_ref();
            match crate::classify::is_zero(&radius_delta, policy) {
                Some(true) => {}
                Some(false) => {
                    return Err(ExactCurveError::invalid(
                        operation,
                        family,
                        CurveError::RadiusMismatch,
                    ));
                }
                None => {
                    return Err(ExactCurveError::blocked(
                        operation,
                        family,
                        crate::UncertaintyReason::RealSign,
                    ));
                }
            }
        }
    }
    Ok(())
}

fn exact_corner_arc_radius(
    arc: &CircularArc2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Real> {
    validate_exact_corner_arc_support(arc, operation, family, policy)?;
    arc.radius_squared()
        .sqrt()
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))
}

fn arc_extension_contains_corner(
    arc: &CircularArc2,
    point: &Point2,
    previous: bool,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let extended = if previous {
        CircularArc2::new_with_certified_radius(
            arc.start().clone(),
            point.clone(),
            arc.center().clone(),
            arc.radius_squared(),
            arc.is_clockwise(),
            None,
        )
    } else {
        CircularArc2::new_with_certified_radius(
            point.clone(),
            arc.end().clone(),
            arc.center().clone(),
            arc.radius_squared(),
            arc.is_clockwise(),
            None,
        )
    };
    let retained_corner = if previous { arc.end() } else { arc.start() };
    match extended.contains_sweep_point(retained_corner, policy) {
        Classification::Decided(contains) => Ok(contains),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, family, reason))
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn line_chamfer_cuts(
    line: &LineSeg2,
    setback: &Real,
    setback_sign: RealSign,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CornerCuts2> {
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: exact_corner_parameter(if previous {
                    Real::one()
                } else {
                    Real::zero()
                }),
                point: if previous {
                    line.end().clone()
                } else {
                    line.start().clone()
                }
                .into(),
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let (dx, dy) = line.delta();
    let (_, _, length) = line_unit_direction(&dx, &dy, operation, family, policy)?;
    let ratio = (setback / &length)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))?;
    let interior_parameter = if previous {
        Real::one() - &ratio
    } else {
        ratio.clone()
    };
    let mut cuts = CornerCuts2::default();
    let interior_after_zero = compare_corner_parameter(
        &interior_parameter,
        &Real::zero(),
        operation,
        family,
        policy,
    )?;
    let interior_before_one =
        compare_corner_parameter(&interior_parameter, &Real::one(), operation, family, policy)?;
    if interior_after_zero == std::cmp::Ordering::Greater
        && interior_before_one == std::cmp::Ordering::Less
    {
        cuts.push(CornerCut2 {
            point: line.point_at(interior_parameter.clone()).into(),
            parameter: exact_corner_parameter(interior_parameter),
            placement: CornerPlacement2::Trim,
        });
    }
    if mode == CurveCornerMode2::TrimOrExtend {
        let extension_parameter = if previous {
            Real::one() + ratio
        } else {
            -ratio
        };
        cuts.push(CornerCut2 {
            point: line.point_at(extension_parameter.clone()).into(),
            parameter: exact_corner_parameter(extension_parameter),
            placement: CornerPlacement2::Extension,
        });
    }
    Ok(cuts)
}

enum MaterializedCornerSide2 {
    One(Curve2),
    SplineExtension {
        source: Curve2,
        extension: Curve2,
        previous: bool,
    },
}

impl MaterializedCornerSide2 {
    const fn extra_curve_count(&self) -> usize {
        match self {
            Self::One(_) => 0,
            Self::SplineExtension { .. } => 1,
        }
    }

    fn append_to(self, curves: &mut Vec<Curve2>) {
        match self {
            Self::One(curve) => curves.push(curve),
            Self::SplineExtension {
                source,
                extension,
                previous: true,
            } => curves.extend([source, extension]),
            Self::SplineExtension {
                source,
                extension,
                previous: false,
            } => curves.extend([extension, source]),
        }
    }
}

enum MaterializedCornerBody2 {
    One(Curve2),
    Two(Curve2, Curve2),
    Three(Curve2, Curve2, Curve2),
}

impl MaterializedCornerBody2 {
    fn from_spline_sides(next: MaterializedCornerSide2, previous: MaterializedCornerSide2) -> Self {
        match (next, previous) {
            (
                MaterializedCornerSide2::SplineExtension {
                    extension,
                    previous: false,
                    ..
                },
                MaterializedCornerSide2::One(previous),
            ) => Self::Two(extension, previous),
            (
                MaterializedCornerSide2::One(next),
                MaterializedCornerSide2::SplineExtension {
                    extension,
                    previous: true,
                    ..
                },
            ) => Self::Two(next, extension),
            (
                MaterializedCornerSide2::SplineExtension {
                    source,
                    extension: next_extension,
                    previous: false,
                },
                MaterializedCornerSide2::SplineExtension {
                    extension: previous_extension,
                    previous: true,
                    ..
                },
            ) => Self::Three(next_extension, source, previous_extension),
            _ => unreachable!("corner sides retain their incident traversal direction"),
        }
    }

    const fn curve_count(&self) -> usize {
        match self {
            Self::One(_) => 1,
            Self::Two(_, _) => 2,
            Self::Three(_, _, _) => 3,
        }
    }

    fn append_to(self, curves: &mut Vec<Curve2>) {
        match self {
            Self::One(curve) => curves.push(curve),
            Self::Two(first, second) => curves.extend([first, second]),
            Self::Three(first, second, third) => curves.extend([first, second, third]),
        }
    }
}

fn materialize_single_curve_corner_body(
    curve: &Curve2,
    previous: &CornerCut2,
    next: &CornerCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<MaterializedCornerBody2> {
    if previous.placement == CornerPlacement2::Extension
        || next.placement == CornerPlacement2::Extension
    {
        if matches!(
            curve.geometry(),
            Some(CurveGeometry2::PolynomialBSpline(_)) | Some(CurveGeometry2::Nurbs(_))
        ) {
            return Ok(MaterializedCornerBody2::from_spline_sides(
                materialize_corner_side(curve, next, false, operation, policy)?,
                materialize_corner_side(curve, previous, true, operation, policy)?,
            ));
        }
        let start = next.exact_parameter().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                curve.family(),
                crate::UncertaintyReason::Unsupported,
            )
        })?;
        let end = previous.exact_parameter().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                curve.family(),
                crate::UncertaintyReason::Unsupported,
            )
        })?;
        return materialize_affine_corner_subcurve(curve, start, end, operation, policy)
            .map(MaterializedCornerBody2::One);
    }

    let parameter = |cut: &CornerCut2| {
        cut.exact_parameter().cloned().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                curve.family(),
                crate::UncertaintyReason::Unsupported,
            )
        })
    };
    let (start, end) = if let Some(CurveGeometry2::CircularArc(arc)) = curve.geometry() {
        (
            materialized_arc_cut_parameter(curve, arc, next, operation, policy)?,
            materialized_arc_cut_parameter(curve, arc, previous, operation, policy)?,
        )
    } else {
        (parameter(next)?, parameter(previous)?)
    };
    curve
        .subcurve_with_policy(start, end, policy)
        .map_err(|error| remap_operation(error, operation))
        .map(MaterializedCornerBody2::One)
}

fn materialized_arc_cut_parameter(
    curve: &Curve2,
    arc: &CircularArc2,
    cut: &CornerCut2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Real> {
    let point = cut.exact_point().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            curve.family(),
            crate::UncertaintyReason::Unsupported,
        )
    })?;
    // Deferred circle contacts carry endpoint markers, not source parameters.
    // The certified point supplies the actual chart coordinate for lineage.
    match arc
        .parameter_at_incident_point(point, policy)
        .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?
    {
        Classification::Decided(parameter) => Ok(parameter),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, curve.family(), reason))
        }
    }
}

fn materialize_corner_side(
    curve: &Curve2,
    cut: &CornerCut2,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<MaterializedCornerSide2> {
    if cut.placement != CornerPlacement2::Extension
        || !matches!(
            curve.geometry(),
            Some(CurveGeometry2::PolynomialBSpline(_)) | Some(CurveGeometry2::Nurbs(_))
        )
    {
        return materialize_corner_cut(curve, cut, previous, operation, policy)
            .map(MaterializedCornerSide2::One);
    }

    let parameter = cut.exact_parameter().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            curve.family(),
            crate::UncertaintyReason::Unsupported,
        )
    })?;
    cut.exact_point().ok_or_else(|| {
        ExactCurveError::blocked(
            operation,
            curve.family(),
            crate::UncertaintyReason::Unsupported,
        )
    })?;
    let fragments = curve.native_bezier_fragments_for_operation(policy, operation)?;
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
                "spline corner extension did not retain an incident native span".into(),
            ),
        )
    })?;
    let (span_start, span_end) = fragment.parameter_range();
    let local_parameter = ((parameter - span_start) / (span_end - span_start))
        .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause.into()))?;
    let (start, end) = if previous {
        (Real::one(), local_parameter)
    } else {
        (local_parameter, Real::zero())
    };
    let extension = match fragment
        .curve()
        .subcurve_between_affine_exact(&start, &end, policy)
        .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?
    {
        Classification::Decided(extension) => Curve2::from(extension),
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, curve.family(), reason));
        }
    };
    Ok(MaterializedCornerSide2::SplineExtension {
        source: curve.clone(),
        extension,
        previous,
    })
}

fn materialize_affine_corner_subcurve(
    curve: &Curve2,
    start: &Real,
    end: &Real,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Curve2> {
    let source = match curve.geometry() {
        Some(CurveGeometry2::QuadraticBezier(source)) => BezierSubcurve2::Quadratic(source.clone()),
        Some(CurveGeometry2::CubicBezier(source)) => BezierSubcurve2::Cubic(source.clone()),
        Some(CurveGeometry2::RationalQuadraticBezier(source)) => {
            BezierSubcurve2::RationalQuadratic(source.clone())
        }
        Some(CurveGeometry2::RationalBezier(source)) => BezierSubcurve2::Rational(source.clone()),
        _ => {
            return Err(ExactCurveError::blocked(
                operation,
                curve.family(),
                crate::UncertaintyReason::Unsupported,
            ));
        }
    };
    match source
        .subcurve_between_affine_exact(start, end, policy)
        .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?
    {
        Classification::Decided(curve) => Ok(Curve2::from(curve)),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, curve.family(), reason))
        }
    }
}

fn materialize_corner_cut(
    curve: &Curve2,
    cut: &CornerCut2,
    previous: bool,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Curve2> {
    match cut.placement {
        CornerPlacement2::Trim => {
            if let Some(CurveGeometry2::CircularArc(arc)) = curve.geometry() {
                let point = cut.exact_point().ok_or_else(|| {
                    ExactCurveError::blocked(
                        operation,
                        curve.family(),
                        crate::UncertaintyReason::Unsupported,
                    )
                })?;
                // The carrier solver already certified the strict parameter
                // placement and the circle kernel certified `cut.point` on
                // the support. Retain that exact point as the fragment
                // endpoint instead of evaluating an algebraically equivalent
                // rational parameter and then asking path connectivity to
                // rediscover the equality.
                let parameter = materialized_arc_cut_parameter(curve, arc, cut, operation, policy)?;
                let domain = curve.native_parameter_domain()?;
                let (start, end) = if previous {
                    (domain.start().clone(), parameter)
                } else {
                    (parameter, domain.end().clone())
                };
                let lineage = curve
                    .lineage_subrange(&start, &end)
                    .map_err(|error| remap_operation(error, operation))?;
                let constructor = if arc.endpoints_on_stored_circle_are_certified() {
                    CircularArc2::new_with_certified_radius
                } else {
                    CircularArc2::new_unchecked_with_radius
                };
                let trimmed = if previous {
                    constructor(
                        arc.start().clone(),
                        point.clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                } else {
                    constructor(
                        point.clone(),
                        arc.end().clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                };
                Ok(Curve2::from_geometry_with_lineage(
                    CurveGeometry2::CircularArc(trimmed),
                    lineage,
                ))
            } else {
                let parameter = cut.exact_parameter().ok_or_else(|| {
                    ExactCurveError::blocked(
                        operation,
                        curve.family(),
                        crate::UncertaintyReason::Unsupported,
                    )
                })?;
                let domain = curve.native_parameter_domain()?;
                let (start, end) = if previous {
                    (domain.start().clone(), parameter.clone())
                } else {
                    (parameter.clone(), domain.end().clone())
                };
                curve
                    .subcurve_with_policy(start, end, policy)
                    .map_err(|error| remap_operation(error, operation))
            }
        }
        CornerPlacement2::Corner => Ok(curve.clone()),
        CornerPlacement2::Extension => {
            let point = cut.exact_point().ok_or_else(|| {
                ExactCurveError::blocked(
                    operation,
                    curve.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })?;
            if let Some(line) = exact_linear_corner_line(curve) {
                let extended = if previous {
                    LineSeg2::try_new(line.start().clone(), point.clone())
                } else {
                    LineSeg2::try_new(point.clone(), line.end().clone())
                }
                .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?;
                Ok(match curve.geometry() {
                    Some(CurveGeometry2::QuadraticBezier(source))
                        if source.retained_exact_line_image().is_some() =>
                    {
                        Curve2::from(QuadraticBezier2::from_line_segment(extended))
                    }
                    _ => Curve2::from(extended),
                })
            } else if let Some(CurveGeometry2::CircularArc(arc)) = curve.geometry() {
                Ok(Curve2::from(if previous {
                    CircularArc2::new_with_certified_radius(
                        arc.start().clone(),
                        point.clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                } else {
                    CircularArc2::new_with_certified_radius(
                        point.clone(),
                        arc.end().clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                }))
            } else if let Some(arc) = retained_rational_arc_support(curve, operation, policy)? {
                Ok(Curve2::from(if previous {
                    CircularArc2::new_with_certified_radius(
                        arc.start().clone(),
                        point.clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                } else {
                    CircularArc2::new_with_certified_radius(
                        point.clone(),
                        arc.end().clone(),
                        arc.center().clone(),
                        arc.radius_squared(),
                        arc.is_clockwise(),
                        None,
                    )
                }))
            } else if let Some(parameter) = cut.exact_parameter() {
                let domain = curve.native_parameter_domain()?;
                let (start, end) = if previous {
                    (domain.start(), parameter)
                } else {
                    (parameter, domain.end())
                };
                materialize_affine_corner_subcurve(curve, start, end, operation, policy)
            } else {
                Err(ExactCurveError::blocked(
                    operation,
                    curve.family(),
                    crate::UncertaintyReason::Unsupported,
                ))
            }
        }
    }
}

fn certify_closed_path(
    path: &CurvePath2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    match validate_closed_curve_path_connectivity(path, policy)
        .map_err(|error| remap_operation(error, operation))?
    {
        Classification::Decided(()) => Ok(()),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            path.data.curves[0].family(),
            reason,
        )),
    }
}

fn validate_strict_split_parameter(
    domain_start: &Real,
    parameter: &Real,
    domain_end: &Real,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    match (
        crate::classify::compare_reals(domain_start, parameter, policy),
        crate::classify::compare_reals(parameter, domain_end, policy),
    ) {
        (Some(std::cmp::Ordering::Less), Some(std::cmp::Ordering::Less)) => Ok(()),
        (Some(_), Some(_)) => Err(ExactCurveError::invalid(
            CurveOperation2::Subdivision,
            family,
            CurveError::InvalidCurveParameter,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            family,
            crate::UncertaintyReason::Ordering,
        )),
    }
}

fn validate_subcurve_range(
    domain_start: &Real,
    start: &Real,
    end: &Real,
    domain_end: &Real,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    match (
        crate::classify::compare_reals(domain_start, start, policy),
        crate::classify::compare_reals(start, end, policy),
        crate::classify::compare_reals(end, domain_end, policy),
    ) {
        (
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            Some(std::cmp::Ordering::Less),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
        ) => Ok(()),
        (Some(_), Some(_), Some(_)) => Err(ExactCurveError::invalid(
            CurveOperation2::Subdivision,
            family,
            CurveError::InvalidCurveParameter,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            family,
            crate::UncertaintyReason::Ordering,
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
            Curve2::try_polynomial_bspline(2, controls.clone(), knots.clone(), policy)
                .unwrap()
                .value,
            Curve2::try_nurbs(2, controls, vec![Real::one(); 3], knots, policy)
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
            .point_at(parameter, policy)
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
                    &policy,
                )
                .unwrap();
                let cuts = match solutions {
                    CurveCornerSolutions2::Unique(solution) => vec![solution.next],
                    CurveCornerSolutions2::Multiple(solutions) => solutions
                        .into_iter()
                        .map(|solution| solution.next)
                        .collect(),
                    CurveCornerSolutions2::NoSolution(reason) => {
                        panic!("the spline corner has a fillet: {reason:?}")
                    }
                };
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
            let corner = crate::rational_bezier_general::exact_contact_point_evidence(
                &diagonal,
                &selected_parameter,
                &policy,
            )
            .unwrap()
            .expect("the selected corner retains exact evidence");
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
                let solve = |mode| {
                    if reversed {
                        solve_exact_fillet_corner(
                            ExactCornerCarrier2::AlgebraicChord(&closing.reversed()),
                            ExactCornerCarrier2::Arc(&arc.reversed()),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::RationalBezier,
                            CurveFamily2::CircularArc,
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
                            &policy,
                        )
                    }
                };
                let trim = solve(CurveCornerMode2::TrimOnly)
                    .expect("the native arc/chord trim solve must complete");
                let extended = solve(CurveCornerMode2::TrimOrExtend)
                    .expect("the native arc/chord extension solve must complete");
                assert!(trim.candidate_count() > 0);
                assert!(extended.candidate_count() > trim.candidate_count());
                let retains_recursive_parameter = |corner: &FilletCorner2| {
                    corner
                        .retained_frame
                        .as_ref()
                        .and_then(|frame| frame.anchor_evidence.as_ref())
                        .and_then(|evidence| evidence.deferred_arc_contact.as_ref())
                        .and_then(|deferred| deferred.contact_seed.as_ref())
                        .is_some_and(|seed| seed.parameter.as_recursive_projective().is_some())
                };
                assert!(match &extended {
                    CurveCornerSolutions2::Unique(corner) => retains_recursive_parameter(corner),
                    CurveCornerSolutions2::Multiple(corners) => {
                        corners.iter().any(retains_recursive_parameter)
                    }
                    CurveCornerSolutions2::NoSolution(_) => false,
                });
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
                assert_eq!(retained_parameter, center_parameter);
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
            assert!(matches!(
                frame.radial_frame,
                RetainedFilletRadialFrame2::ChordNormal { .. }
            ));
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
                CurveCornerMode2::TrimOnly,
                [FilletContactDomain2::OpenCurve; 2],
                CurveFamily2::QuadraticBezier,
                CurveFamily2::QuadraticBezier,
                &policy,
            )
            .expect("a selected positive-dimensional center component must clip locally");
            assert!(centers.coincident);
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
                    center_parameter,
                    Real::one(),
                    false,
                    &policy,
                )
                .unwrap()
            else {
                panic!("the selected circle must construct");
            };

            let Classification::Decided((
                crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiberContacts(contacts),
                _,
            )) = circle
                .rational_intersections_with_parameter_map(&rational, &policy)
                .expect("the selected-circle kernel must retain the rational contact locally")
            else {
                panic!("the selected half must publish its local rational-contact fiber");
            };
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
        let center = CurvePoint2::from(
            center_source
                .point_at_algebraic_parameter(&parameter, policy)
                .expect("the selected center retains its exact image"),
        );
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
                            &policy,
                        ),
                        (true, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::Bezier(&reversed_second_curve),
                            ExactCornerCarrier2::AnalyticParallel(&retained.reversed()),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::QuadraticBezier,
                            &policy,
                        ),
                    }
                    .expect("the shared parallel-pair extension kernel must decide")
                };
                let direct_extension = solve(false, CurveCornerMode2::TrimOrExtend);
                let retained_extension = solve(true, CurveCornerMode2::TrimOrExtend);
                assert_eq!(
                    retained_extension.candidate_count(),
                    direct_extension.candidate_count()
                );
                assert!(direct_extension.candidate_count() > 0);
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
                            &policy,
                        ),
                        (false, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::Bezier(&reversed_direct),
                            ExactCornerCarrier2::AlgebraicCusp(&fragment.reversed()),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::RationalBezier,
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
                            &policy,
                        ),
                        (true, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::AnalyticParallel(&retained.reversed()),
                            ExactCornerCarrier2::AlgebraicCusp(&fragment.reversed()),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::RationalBezier,
                            &policy,
                        ),
                    }
                    .expect("the selected-circle/parallel extension kernel must decide")
                };
                let direct_extension = solve(false, CurveCornerMode2::TrimOrExtend);
                let retained_extension = solve(true, CurveCornerMode2::TrimOrExtend);
                assert_eq!(
                    direct_extension.candidate_count(),
                    retained_extension.candidate_count()
                );
                assert!(direct_extension.candidate_count() > 0);
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
                            &policy,
                        ),
                        (true, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::Line(&reversed_line),
                            ExactCornerCarrier2::AlgebraicCusp(&fragment.reversed()),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::Line,
                            CurveFamily2::RationalBezier,
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
                            &policy,
                        ),
                        (false, true) => solve_exact_fillet_corner(
                            ExactCornerCarrier2::Bezier(&reversed_direct),
                            ExactCornerCarrier2::AlgebraicCusp(&fragment.reversed()),
                            &radius,
                            RealSign::Positive,
                            mode,
                            false,
                            CurveFamily2::QuadraticBezier,
                            CurveFamily2::RationalBezier,
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
                assert_eq!(native_trim.candidate_count(), direct_trim.candidate_count());
                assert_eq!(
                    native_extension.candidate_count(),
                    direct_extension.candidate_count(),
                    "the native fast path must enumerate both selected-circle charts",
                );
                assert!(native_extension.candidate_count() > native_trim.candidate_count());
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
            let source_arc = ExactCornerArc2::Native(&arc);
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
                BezierParameter2::Exact(Real::zero()),
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
                CurveCornerMode2::TrimOrExtend,
                [FilletContactDomain2::OpenCurve; 2],
                CurveFamily2::CircularArc,
                CurveFamily2::RationalBezier,
                &policy,
            )
            .expect("the endpoint-only circle pair must solve exactly");
            assert!(!centers.coincident);
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
            .boundary_loop(&CurveContext::APPROXIMATE_512)
            .expect("the terminal policy must validate every path join");
        assert_eq!(
            boundary.certainty,
            crate::CurveCertainty::Approximate512Consumed
        );
        assert_eq!(boundary.value.len(), 4);

        let strict_boundary = path.boundary_loop(&CurveContext::STRICT).unwrap_err();
        assert!(matches!(
            strict_boundary,
            ExactCurveError::Blocked(blocker)
                if blocker.operation() == CurveOperation2::Arrangement
                    && blocker.reason() == crate::UncertaintyReason::RealSign
        ));

        let strict_region = crate::CurveRegion2::try_from_boundary_paths(
            std::slice::from_ref(&path),
            &CurveContext::STRICT,
        )
        .unwrap_err();
        assert!(matches!(
            strict_region,
            ExactCurveError::Blocked(blocker)
                if blocker.operation() == CurveOperation2::Construction
                    && blocker.reason() == crate::UncertaintyReason::RealSign
        ));
        let approximate_region = crate::CurveRegion2::try_from_boundary_paths(
            std::slice::from_ref(&path),
            &CurveContext::APPROXIMATE_512,
        )
        .expect("region construction must revalidate the terminal internal join");
        assert_eq!(
            approximate_region.certainty,
            crate::CurveCertainty::Approximate512Consumed
        );

        let approximate = path
            .classify_point(
                &Point2::new(Real::one(), Real::one()),
                &CurveContext::APPROXIMATE_512,
            )
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
            .classify_point(
                &Point2::new(Real::one(), Real::one()),
                &CurveContext::STRICT,
            )
            .expect("strict classification preserves uncertainty as query evidence");
        assert_eq!(strict.certainty, crate::CurveCertainty::Certified);
        assert_eq!(
            strict.value,
            Classification::Uncertain(crate::UncertaintyReason::RealSign)
        );
    }
}
