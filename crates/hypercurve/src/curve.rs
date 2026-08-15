//! Top-level owned and borrowed exact curve carriers.

use std::sync::Arc;
use std::sync::OnceLock;

use hyperreal::RealSign;

use crate::arc_bezier::{
    decompose_circular_arc, rational_bezier_circular_arc, rational_quadratic_circular_arc,
};
use crate::policy::{
    PolicyEvaluationCache, resolve_cached_evaluation, resolve_certified_operation,
};
use crate::rational_bezier_general::RationalBezierOverlapParameterCorrespondence2;
use crate::{
    Aabb2, BezierBoundaryLoop2, BezierParallel2, BezierParameter2, BezierSubcurve2, CircularArc2,
    Classification, ContourPointLocation, CubicBezier2, CurveContext, CurveError, CurveOperation2,
    CurveOutcome, CurveRegionParameter2, ExactCurveError, ExactCurveResult, LineSeg2, LineSide,
    NurbsCurve2, ParamRange, Point2, PolynomialSplineCurve2, QuadraticBezier2, RationalBezier2,
    RationalBezierIntersectionPointEvidence2, RationalQuadraticBezier2, Real, Similarity2,
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
}

/// Exact derivative vector of a planar curve with respect to its public parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveDerivative2 {
    dx: Real,
    dy: Real,
    zero_status: hyperreal::ZeroKnowledge,
}

/// Exact closed public parameter domain of one top-level curve.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveParameterDomain2 {
    start: Real,
    end: Real,
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

impl CurveParameterDomain2 {
    /// Returns the inclusive domain start.
    pub const fn start(&self) -> &Real {
        &self.start
    }

    /// Returns the inclusive domain end.
    pub const fn end(&self) -> &Real {
        &self.end
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
    geometry: CurveGeometry2,
    lineage: CurveParameterLineage2,
    parameter_domain: OnceLock<CurveParameterDomain2>,
    native_bezier_fragments: PolicyEvaluationCache<Vec<NativeBezierFragment2>>,
    rational_evaluators: PolicyEvaluationCache<Vec<RationalBezier2>>,
    bounds: OnceLock<ExactCurveResult<Aabb2>>,
}

#[derive(Clone, Debug)]
struct CurveParameterLineage2 {
    root: Arc<CurveParameterLineageRoot2>,
    range: ParamRange,
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
        }
    }

    fn reversed(&self) -> Self {
        Self {
            root: Arc::clone(&self.root),
            range: ParamRange::new(self.range.end().clone(), self.range.start().clone()),
        }
    }
}

/// Immutable top-level exact planar curve.
///
/// Clones share the exact carrier and its retained calculations. Use
/// [`Curve2::as_view`] for borrowed algorithms.
#[derive(Clone, Debug)]
pub struct Curve2 {
    data: Arc<CurveData2>,
}

/// Borrowed view of one top-level exact curve.
#[derive(Clone, Copy, Debug)]
pub struct CurveView2<'a> {
    curve: &'a Curve2,
}

/// Ordered connected sequence of exact curves.
#[derive(Clone, Debug)]
pub struct CurvePath2 {
    data: Arc<CurvePathData2>,
}

#[derive(Debug)]
struct CurvePathData2 {
    curves: Vec<Curve2>,
    strict_connectivity_certified: bool,
    strict_closure_certified: bool,
    native_bezier_fragments: PolicyEvaluationCache<Vec<NativeBezierFragment2>>,
    bezier_boundary_loop: PolicyEvaluationCache<NativeBezierBoundaryLoop2>,
    bounds: OnceLock<ExactCurveResult<Aabb2>>,
}

/// Borrowed view of an ordered exact curve path.
#[derive(Clone, Copy, Debug)]
pub struct CurvePathView2<'a> {
    curves: &'a [Curve2],
}

/// Exact public parameter interval for one promoted native span.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveSpanRange2 {
    start: Real,
    end: Real,
}

/// Exact native Bezier/conic fragment and its public parameter interval.
#[derive(Clone, Debug, PartialEq)]
pub struct NativeBezierFragment2 {
    curve: BezierSubcurve2,
    span_range: CurveSpanRange2,
}

/// Validated native Bezier boundary derived from a path's retained promotion.
#[derive(Clone, Debug, PartialEq)]
pub struct NativeBezierBoundaryLoop2 {
    boundary_loop: BezierBoundaryLoop2,
}

impl CurveGeometry2 {
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
        Self {
            data: Arc::new(CurveData2 {
                geometry,
                lineage,
                parameter_domain: OnceLock::new(),
                native_bezier_fragments: PolicyEvaluationCache::new(),
                rational_evaluators: PolicyEvaluationCache::new(),
                bounds: OnceLock::new(),
            }),
        }
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

    /// Returns a borrowed view without cloning geometry.
    pub const fn as_view(&self) -> CurveView2<'_> {
        CurveView2 { curve: self }
    }

    /// Returns the exact geometry carrier.
    pub fn geometry(&self) -> &CurveGeometry2 {
        &self.data.geometry
    }

    /// Returns the curve family.
    pub fn family(&self) -> CurveFamily2 {
        self.data.geometry.family()
    }

    /// Returns the exact start point.
    pub fn start(&self) -> &Point2 {
        self.data.geometry.start()
    }

    /// Returns the exact end point.
    pub fn end(&self) -> &Point2 {
        self.data.geometry.end()
    }

    /// Returns the clone-shared exact public parameter domain.
    pub fn parameter_domain(&self) -> &CurveParameterDomain2 {
        self.data.parameter_domain.get_or_init(|| {
            let (start, end) = match self.geometry() {
                CurveGeometry2::PolynomialBSpline(curve) => curve.parameter_domain(),
                CurveGeometry2::Nurbs(curve) => curve.parameter_domain(),
                _ => {
                    return CurveParameterDomain2 {
                        start: Real::zero(),
                        end: Real::one(),
                    };
                }
            };
            CurveParameterDomain2 {
                start: start.clone(),
                end: end.clone(),
            }
        })
    }

    /// Returns the exact period when this top-level curve is periodic.
    pub fn period(&self) -> Option<&Real> {
        match self.geometry() {
            CurveGeometry2::PolynomialBSpline(curve) => curve.period(),
            CurveGeometry2::Nurbs(curve) => curve.period(),
            _ => None,
        }
    }

    /// Returns whether this curve carries explicit periodic semantics.
    pub fn is_periodic(&self) -> bool {
        self.period().is_some()
    }

    /// Returns the same exact curve image with traversal direction reversed.
    ///
    /// The public parameter mapping is retained.
    /// Parameters map as `u -> start + end - u`.
    pub fn reversed(&self, policy: &CurveContext) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| self.reversed_raw(attempt))
    }

    pub(crate) fn reversed_raw(&self, policy: &CurveContext) -> ExactCurveResult<Self> {
        let geometry = match self.geometry() {
            CurveGeometry2::Line(curve) => CurveGeometry2::Line(curve.reversed()),
            CurveGeometry2::CircularArc(curve) => CurveGeometry2::CircularArc(curve.reversed()),
            CurveGeometry2::QuadraticBezier(curve) => CurveGeometry2::QuadraticBezier(
                curve.reversed_with_retained_provenance().map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Reversal,
                        CurveFamily2::QuadraticBezier,
                        cause,
                    )
                })?,
            ),
            CurveGeometry2::CubicBezier(curve) => CurveGeometry2::CubicBezier(CubicBezier2::new(
                curve.end().clone(),
                curve.control2().clone(),
                curve.control1().clone(),
                curve.start().clone(),
            )),
            CurveGeometry2::RationalQuadraticBezier(curve) => {
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
            CurveGeometry2::RationalBezier(curve) => {
                CurveGeometry2::RationalBezier(curve.reversed())
            }
            CurveGeometry2::PolynomialBSpline(curve) => {
                CurveGeometry2::PolynomialBSpline(curve.reversed_raw(policy)?)
            }
            CurveGeometry2::Nurbs(curve) => CurveGeometry2::Nurbs(curve.reversed_raw(policy)?),
        };
        self.with_lineage(geometry, self.data.lineage.reversed())
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
        let transform_points = |points: &[Point2]| {
            points
                .iter()
                .map(|point| transform.transform_point(point))
                .collect::<Vec<_>>()
        };
        let geometry = match self.geometry() {
            CurveGeometry2::Line(curve) => CurveGeometry2::Line(
                curve
                    .transform_similarity(transform)
                    .map_err(|cause| self.transform_error(cause))?,
            ),
            CurveGeometry2::CircularArc(curve) => CurveGeometry2::CircularArc(
                curve
                    .transform_similarity(transform)
                    .map_err(|cause| self.transform_error(cause))?,
            ),
            CurveGeometry2::QuadraticBezier(curve) => CurveGeometry2::QuadraticBezier(
                curve
                    .transform_similarity_with_retained_provenance(transform)
                    .map_err(|cause| self.transform_error(cause))?,
            ),
            CurveGeometry2::CubicBezier(curve) => {
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
            CurveGeometry2::RationalQuadraticBezier(curve) => {
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
            CurveGeometry2::RationalBezier(curve) => CurveGeometry2::RationalBezier(
                RationalBezier2::try_new(
                    transform_points(curve.control_points()),
                    curve.weights().to_vec(),
                )
                .map_err(|cause| self.transform_error(cause))?,
            ),
            CurveGeometry2::PolynomialBSpline(curve) => CurveGeometry2::PolynomialBSpline(
                curve.transform_similarity_raw(transform, policy)?,
            ),
            CurveGeometry2::Nurbs(curve) => {
                CurveGeometry2::Nurbs(curve.transform_similarity_raw(transform, policy)?)
            }
        };
        self.with_lineage(geometry, self.data.lineage.clone())
    }

    /// Splits this curve exactly at a strict interior public parameter.
    ///
    /// Native result curves use their usual `[0, 1]` parameter domain. Spline
    /// results retain the two corresponding authored knot-domain intervals.
    /// Curve family and public parameter mapping are preserved. The returned
    /// [`CurveOutcome`] covers the complete split and carrier reconstruction.
    #[inline(always)]
    pub fn split_at(
        &self,
        parameter: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<(Self, Self)>> {
        resolve_certified_operation(policy, |attempt| self.split_at_raw(parameter, attempt))
    }

    pub(crate) fn split_at_raw(
        &self,
        parameter: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<(Self, Self)> {
        let domain = self.parameter_domain();
        validate_strict_split_parameter(
            domain.start(),
            &parameter,
            domain.end(),
            self.family(),
            policy,
        )?;
        match self.geometry() {
            CurveGeometry2::PolynomialBSpline(curve) => {
                let (left, right) = curve.split_at_raw(parameter.clone(), policy)?;
                let left_lineage = self.lineage_subrange(domain.start(), &parameter)?;
                let right_lineage = self.lineage_subrange(&parameter, domain.end())?;
                Ok((
                    self.with_lineage(CurveGeometry2::PolynomialBSpline(left), left_lineage)?,
                    self.with_lineage(CurveGeometry2::PolynomialBSpline(right), right_lineage)?,
                ))
            }
            CurveGeometry2::Nurbs(curve) => {
                let (left, right) = curve.split_at_raw(parameter.clone(), policy)?;
                let left_lineage = self.lineage_subrange(domain.start(), &parameter)?;
                let right_lineage = self.lineage_subrange(&parameter, domain.end())?;
                Ok((
                    self.with_lineage(CurveGeometry2::Nurbs(left), left_lineage)?,
                    self.with_lineage(CurveGeometry2::Nurbs(right), right_lineage)?,
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
    /// A full-domain request returns a clone sharing retained facts. Native
    /// result curves are reparameterized to `[0, 1]`; spline results retain the
    /// requested authored knot range. Curve family and source are preserved.
    /// The returned [`CurveOutcome`] records any terminal decision consumed by
    /// the complete exact range extraction.
    #[inline(always)]
    pub fn subcurve(
        &self,
        start: Real,
        end: Real,
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
            self.subcurve_with_policy(start, end, attempt)
        })
    }

    pub(crate) fn subcurve_with_policy(
        &self,
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let domain = self.parameter_domain();
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
            CurveGeometry2::Line(curve) => CurveGeometry2::Line(
                LineSeg2::try_new(curve.point_at(start), curve.point_at(end))
                    .map_err(|cause| self.subdivision_error(cause))?,
            ),
            CurveGeometry2::CircularArc(curve) => {
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
            CurveGeometry2::QuadraticBezier(curve) => CurveGeometry2::QuadraticBezier(
                curve
                    .subcurve_between_exact(&start, &end, policy)
                    .map_err(|cause| self.subdivision_error(cause))?,
            ),
            CurveGeometry2::CubicBezier(curve) => CurveGeometry2::CubicBezier(
                curve
                    .subcurve_between_exact(&start, &end, policy)
                    .map_err(|cause| self.subdivision_error(cause))?,
            ),
            CurveGeometry2::RationalQuadraticBezier(curve) => {
                CurveGeometry2::RationalQuadraticBezier(
                    curve
                        .subcurve_between_exact(&start, &end, policy)
                        .map_err(|cause| self.subdivision_error(cause))?,
                )
            }
            CurveGeometry2::RationalBezier(curve) => CurveGeometry2::RationalBezier(
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
            CurveGeometry2::PolynomialBSpline(curve) => CurveGeometry2::PolynomialBSpline(
                curve
                    .subcurve_raw(start, end, policy)
                    .map_err(|error| remap_operation(error, CurveOperation2::Subdivision))?,
            ),
            CurveGeometry2::Nurbs(curve) => CurveGeometry2::Nurbs(
                curve
                    .subcurve_raw(start, end, policy)
                    .map_err(|error| remap_operation(error, CurveOperation2::Subdivision))?,
            ),
        };
        self.with_lineage(geometry, lineage)
    }

    /// Returns an exact finite subcurve suitable for clamped topology carriers.
    ///
    /// Spline families preserve their authored parameter interval and exact
    /// image in clamped piecewise-Bézier form. Other families use their native
    /// exact subdivision. One [`CurveOutcome`] covers the complete operation.
    #[inline(always)]
    pub fn clamped_subcurve(
        &self,
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            self.clamped_subcurve_raw(start, end, attempt)
        })
    }

    pub(crate) fn clamped_subcurve_raw(
        &self,
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let domain = self.parameter_domain();
        validate_subcurve_range(
            domain.start(),
            &start,
            &end,
            domain.end(),
            self.family(),
            policy,
        )?;
        let lineage = self.lineage_subrange(&start, &end)?;
        match self.geometry() {
            CurveGeometry2::PolynomialBSpline(curve) => self.with_lineage(
                CurveGeometry2::PolynomialBSpline(curve.clamped_subcurve_raw(start, end, policy)?),
                lineage,
            ),
            CurveGeometry2::Nurbs(curve) => self.with_lineage(
                CurveGeometry2::Nurbs(curve.clamped_subcurve_raw(start, end, policy)?),
                lineage,
            ),
            _ => self.subcurve_with_policy(start, end, policy),
        }
    }

    fn with_lineage(
        &self,
        geometry: CurveGeometry2,
        lineage: CurveParameterLineage2,
    ) -> ExactCurveResult<Self> {
        Ok(Self {
            data: Arc::new(CurveData2 {
                geometry,
                lineage,
                parameter_domain: OnceLock::new(),
                native_bezier_fragments: PolicyEvaluationCache::new(),
                rational_evaluators: PolicyEvaluationCache::new(),
                bounds: OnceLock::new(),
            }),
        })
    }

    fn lineage_subrange(
        &self,
        start: &Real,
        end: &Real,
    ) -> ExactCurveResult<CurveParameterLineage2> {
        Ok(CurveParameterLineage2 {
            root: Arc::clone(&self.data.lineage.root),
            range: ParamRange::new(
                self.lineage_parameter_at(start)?,
                self.lineage_parameter_at(end)?,
            ),
        })
    }

    pub(crate) fn lineage_parameter_at(&self, parameter: &Real) -> ExactCurveResult<Real> {
        let domain = self.parameter_domain();
        let local =
            ((parameter - domain.start()) / (domain.end() - domain.start())).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Subdivision, self.family(), cause.into())
            })?;
        Ok(self.data.lineage.range.start()
            + &local * (self.data.lineage.range.end() - self.data.lineage.range.start()))
    }

    fn retain_root_image_injectivity(&self, policy: &CurveContext) {
        let root = &self.data.lineage.root;
        if root.image_is_injective.get().is_some()
            || !matches!(
                self.family(),
                CurveFamily2::QuadraticBezier | CurveFamily2::CubicBezier
            )
        {
            return;
        }
        let range = &self.data.lineage.range;
        let covers_root_domain =
            (crate::classify::compare_reals(range.start(), root.domain.start(), policy)
                == Some(std::cmp::Ordering::Equal)
                && crate::classify::compare_reals(range.end(), root.domain.end(), policy)
                    == Some(std::cmp::Ordering::Equal))
                || (crate::classify::compare_reals(range.start(), root.domain.end(), policy)
                    == Some(std::cmp::Ordering::Equal)
                    && crate::classify::compare_reals(range.end(), root.domain.start(), policy)
                        == Some(std::cmp::Ordering::Equal));
        if !covers_root_domain {
            return;
        }
        let Ok(Classification::Decided(evaluators)) = self.rational_evaluators_with_policy(policy)
        else {
            return;
        };
        if evaluators.len() == 1 && evaluators[0].has_certified_injective_axis(policy) {
            let _ = root.image_is_injective.set(true);
        }
    }

    pub(crate) fn shares_certified_parameter_lineage(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data.lineage.root, &other.data.lineage.root)
            && self.data.lineage.root.image_is_injective.get() == Some(&true)
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
    /// parameters use their authored knot domain.
    pub fn point_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        self.point_at_side(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates an exact point with explicit spline-knot side policy.
    pub fn point_at_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        resolve_certified_operation(policy, |attempt| {
            self.point_at_side_with_policy(parameter, side, attempt)
        })
    }

    pub(crate) fn point_at_side_with_policy(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        match self.geometry() {
            CurveGeometry2::PolynomialBSpline(curve) => {
                curve.point_at_side_with_policy(parameter, side, policy)
            }
            CurveGeometry2::Nurbs(curve) => {
                curve.point_at_side_with_policy(parameter, side, policy)
            }
            geometry => {
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
            CurveGeometry2::PolynomialBSpline(curve) => {
                curve.point_at_wrapped_side_with_policy(parameter, side, policy)
            }
            CurveGeometry2::Nurbs(curve) => {
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
            CurveGeometry2::PolynomialBSpline(curve) => {
                return curve.derivatives_at_side_with_policy(parameter, max_order, side, policy);
            }
            CurveGeometry2::Nurbs(curve) => {
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
            CurveGeometry2::PolynomialBSpline(curve) => {
                curve.derivatives_at_wrapped_side_with_policy(parameter, max_order, side, policy)
            }
            CurveGeometry2::Nurbs(curve) => {
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
        self.data.geometry == other.data.geometry
    }
}

impl<'a> CurveView2<'a> {
    /// Returns the owned curve backing this view.
    pub const fn curve(self) -> &'a Curve2 {
        self.curve
    }

    /// Returns the exact geometry carrier.
    pub fn geometry(self) -> &'a CurveGeometry2 {
        self.curve.geometry()
    }

    /// Returns the curve family.
    pub fn family(self) -> CurveFamily2 {
        self.curve.family()
    }

    /// Returns the exact start point.
    pub fn start(self) -> &'a Point2 {
        self.curve.start()
    }

    /// Returns the exact end point.
    pub fn end(self) -> &'a Point2 {
        self.curve.end()
    }

    /// Returns the clone-shared exact public parameter domain.
    pub fn parameter_domain(self) -> &'a CurveParameterDomain2 {
        self.curve.parameter_domain()
    }

    /// Returns the exact period when this curve is explicitly periodic.
    pub fn period(self) -> Option<&'a Real> {
        self.curve.period()
    }

    /// Returns whether this curve carries explicit periodic semantics.
    pub fn is_periodic(self) -> bool {
        self.curve.is_periodic()
    }

    /// Returns an owned curve with traversal direction reversed.
    pub fn reversed(self, policy: &CurveContext) -> ExactCurveResult<CurveOutcome<Curve2>> {
        self.curve.reversed(policy)
    }

    /// Applies an exact planar similarity without cloning the source carrier first.
    pub fn transform_similarity(
        self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Curve2>> {
        self.curve.transform_similarity(transform, policy)
    }

    /// Splits this curve exactly at a strict interior public parameter.
    #[inline(always)]
    pub fn split_at(
        self,
        parameter: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<(Curve2, Curve2)>> {
        self.curve.split_at(parameter, policy)
    }

    /// Returns the exact curve image over a strictly ordered public range.
    #[inline(always)]
    pub fn subcurve(
        self,
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Curve2>> {
        self.curve.subcurve(start, end, policy)
    }

    /// Returns a finite exact subcurve in clamped topology-ingestion form.
    #[inline(always)]
    pub fn clamped_subcurve(
        self,
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Curve2>> {
        self.curve.clamped_subcurve(start, end, policy)
    }

    /// Evaluates this borrowed curve without cloning its retained carrier.
    pub fn point_at(
        self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        self.curve.point_at(parameter, policy)
    }

    /// Evaluates an exact point with explicit spline-knot side policy.
    pub fn point_at_side(
        self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        self.curve.point_at_side(parameter, side, policy)
    }

    /// Evaluates an explicitly periodic spline at any wrappable parameter.
    pub fn point_at_wrapped(
        self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        self.curve.point_at_wrapped(parameter, policy)
    }

    /// Evaluates a periodic spline with explicit side selection at wrapped seams.
    pub fn point_at_wrapped_side(
        self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        self.curve.point_at_wrapped_side(parameter, side, policy)
    }

    /// Evaluates the exact first derivative without cloning the curve.
    pub fn derivative_at(
        self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        self.curve.derivative_at(parameter, policy)
    }

    /// Evaluates an exact one-sided or certified two-sided first derivative.
    pub fn derivative_at_side(
        self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        self.curve.derivative_at_side(parameter, side, policy)
    }

    /// Evaluates the first periodic derivative at any wrappable parameter.
    pub fn derivative_at_wrapped(
        self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        self.curve.derivative_at_wrapped(parameter, policy)
    }

    /// Evaluates the first periodic derivative with explicit seam-side selection.
    pub fn derivative_at_wrapped_side(
        self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        self.curve
            .derivative_at_wrapped_side(parameter, side, policy)
    }

    /// Evaluates exact derivatives through `max_order` without cloning the curve.
    pub fn derivatives_at(
        self,
        parameter: &Real,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        self.curve.derivatives_at(parameter, max_order, policy)
    }

    /// Evaluates exact derivatives with explicit retained-fragment side policy.
    pub fn derivatives_at_side(
        self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        self.curve
            .derivatives_at_side(parameter, max_order, side, policy)
    }

    /// Evaluates periodic derivatives through `max_order` at any wrappable parameter.
    pub fn derivatives_at_wrapped(
        self,
        parameter: &Real,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        self.curve
            .derivatives_at_wrapped(parameter, max_order, policy)
    }

    /// Evaluates periodic derivatives with explicit side selection at wrapped seams.
    pub fn derivatives_at_wrapped_side(
        self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        self.curve
            .derivatives_at_wrapped_side(parameter, max_order, side, policy)
    }
}

impl CurvePath2 {
    fn from_connected_curves(
        curves: Vec<Curve2>,
        strict_connectivity_certified: bool,
        strict_closure_certified: bool,
    ) -> Self {
        Self {
            data: Arc::new(CurvePathData2 {
                curves,
                strict_connectivity_certified,
                strict_closure_certified,
                native_bezier_fragments: PolicyEvaluationCache::new(),
                bezier_boundary_loop: PolicyEvaluationCache::new(),
                bounds: OnceLock::new(),
            }),
        }
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
        Self::from_connected_curves(curves, true, true)
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
            match crate::classify::is_zero(
                &adjacent[0].end().distance_squared(adjacent[1].start()),
                policy,
            ) {
                Some(true) => {
                    strict_connectivity_certified &= !policy.permits_approximate_512();
                }
                Some(false) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Construction,
                        adjacent[1].family(),
                        CurveError::DisconnectedCurvePath,
                    ));
                }
                None => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Construction,
                        adjacent[1].family(),
                        crate::UncertaintyReason::RealSign,
                    ));
                }
            }
        }
        Ok(Self::from_connected_curves(
            curves,
            strict_connectivity_certified,
            strict_closure_certified,
        ))
    }

    /// Returns a borrowed path view.
    pub fn as_view(&self) -> CurvePathView2<'_> {
        CurvePathView2 {
            curves: &self.data.curves,
        }
    }

    /// Returns curves in traversal order.
    pub fn curves(&self) -> &[Curve2] {
        &self.data.curves
    }

    /// Returns the exact path start point.
    pub fn start(&self) -> &Point2 {
        self.data.curves[0].start()
    }

    /// Returns the exact path end point.
    pub fn end(&self) -> &Point2 {
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
            self.data.strict_connectivity_certified,
            self.data.strict_closure_certified,
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
            self.data.strict_connectivity_certified,
            self.data.strict_closure_certified,
        ))
    }

    /// Solves and applies an exact chord-setback chamfer at one path vertex.
    ///
    /// Each nonnegative setback is the Euclidean chord distance from the
    /// original corner along its incident curve image. The shared exact carrier
    /// kernel handles native lines and circular arcs, retained degree-elevated
    /// line images, exact circular rational carriers, and direct polynomial or
    /// rational Bezier carriers through complete circle incidence. Polynomial
    /// spline and NURBS endpoints reuse only their incident retained native
    /// span, then map every represented contact back to the authored knot
    /// interval before subdivision. An exterior contact preserves the authored
    /// spline and appends or prepends only that span's exact continuation. A
    /// nonendpoint algebraic trim remains an
    /// explicit `Unsupported` blocker until public curve subdivision can retain
    /// that parameter exactly.
    /// [`CurveCornerMode2::TrimOrExtend`] returns every exact native-support
    /// candidate in deterministic order. Direct polynomial Beziers use their
    /// complete affine incident ray; represented exterior roots materialize as
    /// polynomial subcurves, while a public path explicitly blocks an
    /// algebraic endpoint that only retained-region topology can store. The
    /// retained-region caller also uses this kernel for affine algebraic
    /// chords. General rational Beziers extend only through the finite incident
    /// projective cell before their first pole; exact homogeneous subdivision
    /// materializes that cell without weakening any predicate. A one-curve
    /// closed path materializes its shared retained body exactly once.
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
        let previous_carrier =
            exact_corner_carrier(previous, true, CurveOperation2::Chamfer, policy)?.ok_or_else(
                || {
                    ExactCurveError::blocked(
                        CurveOperation2::Chamfer,
                        previous.family(),
                        crate::UncertaintyReason::Unsupported,
                    )
                },
            )?;
        let next_carrier = exact_corner_carrier(next, false, CurveOperation2::Chamfer, policy)?
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Chamfer,
                    next.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })?;
        if mode == CurveCornerMode2::TrimOrExtend
            && (!previous_carrier.supports_extension(CurveOperation2::Chamfer)
                || !next_carrier.supports_extension(CurveOperation2::Chamfer))
        {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Chamfer,
                if !previous_carrier.supports_extension(CurveOperation2::Chamfer) {
                    previous.family()
                } else {
                    next.family()
                },
                crate::UncertaintyReason::Unsupported,
            ));
        }

        let solutions = solve_exact_chamfer_corner(
            previous_carrier,
            next_carrier,
            &previous_setback,
            &next_setback,
            previous_sign,
            next_sign,
            mode,
            false,
            false,
            previous.family(),
            next.family(),
            policy,
        )?;
        try_map_corner_solutions(solutions, |solution| {
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
                return self.with_single_curve_corner_replaced(
                    previous_index,
                    &solution.previous,
                    &solution.next,
                    chamfer,
                    CurveOperation2::Chamfer,
                    policy,
                );
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
        })
    }

    /// Solves and applies an exact circular fillet of the requested radius.
    ///
    /// Fillet centers are intersections of equal signed-radius offsets of the
    /// two incident carriers. The shared kernel handles lines, circular arcs,
    /// retained exact line/circle images, and direct polynomial or rational
    /// Beziers, including the one incident retained native span of a polynomial
    /// spline or NURBS carrier. Retained algebraic chords with a certified
    /// represented support share the line carrier and preserve their canonical
    /// finite or exterior affine-support parameters. Mixed line/Bezier and
    /// arc/Bezier pairs use complete
    /// analytic incidence. Bezier/Bezier pairs share the complete selected-
    /// branch parallel-intersection kernel. For two direct Beziers,
    /// `TrimOrExtend` enlarges both parameter projections through the regular
    /// endpoint-adjacent cells, stopping before the first pole or source-speed
    /// zero. A positive-dimensional exterior correspondence and any algebraic
    /// center or trim that a public path cannot retain remain explicit
    /// blockers for this API and are delegated to retained-region
    /// reconstruction. Spline and NURBS endpoint extensions preserve the
    /// authored carrier and materialize only the selected incident Bezier cell;
    /// a one-curve closed path materializes its shared retained body exactly
    /// once. Retained rational
    /// circles use their certified native support for extension and collapse
    /// only the extended source fragment to [`CircularArc2`].
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
        let previous_carrier =
            exact_corner_carrier(previous, true, CurveOperation2::Fillet, policy)?.ok_or_else(
                || {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        previous.family(),
                        crate::UncertaintyReason::Unsupported,
                    )
                },
            )?;
        let next_carrier = exact_corner_carrier(next, false, CurveOperation2::Fillet, policy)?
            .ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    next.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })?;
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
        try_map_corner_solutions(solutions, |solution| {
            let previous_point = solution.previous.exact_point().cloned().ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    previous.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })?;
            let next_point = solution.next.exact_point().cloned().ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    next.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })?;
            let center = solution.center.as_exact().cloned().ok_or_else(|| {
                ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    previous.family(),
                    crate::UncertaintyReason::Unsupported,
                )
            })?;
            let fillet = Curve2::from(CircularArc2::new_with_certified_radius(
                previous_point,
                next_point,
                center,
                &radius * &radius,
                solution.clockwise,
                None,
            ));
            if previous_index == next_index {
                return self.with_single_curve_corner_replaced(
                    previous_index,
                    &solution.previous,
                    &solution.next,
                    fillet,
                    CurveOperation2::Fillet,
                    policy,
                );
            }
            let previous_trim = materialize_corner_side(
                previous,
                &solution.previous,
                true,
                CurveOperation2::Fillet,
                policy,
            )?;
            let next_trim = materialize_corner_side(
                next,
                &solution.next,
                false,
                CurveOperation2::Fillet,
                policy,
            )?;
            self.with_corner_replaced(
                vertex_index,
                previous_index,
                next_index,
                previous_trim,
                fillet,
                next_trim,
                CurveOperation2::Fillet,
                policy,
            )
        })
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
            && let CurveGeometry2::CircularArc(arc) = curve.geometry()
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
            .bezier_boundary_loop_with_policy(policy)
            .map_err(|error| remap_operation(error, CurveOperation2::Classification))?
        {
            Classification::Decided(boundary) => boundary,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        boundary
            .boundary_loop()
            .classify_point(point, policy)
            .map_err(|cause| {
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

    /// Builds a closed native Bezier boundary once and borrows the retained result.
    ///
    /// The returned [`CurveOutcome`] records whether validating every path and
    /// promoted-fragment join consumed the `APPROXIMATE_512` terminal.
    pub fn bezier_boundary_loop(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<&NativeBezierBoundaryLoop2>> {
        resolve_certified_operation(policy, |attempt| {
            match self.bezier_boundary_loop_with_policy(attempt)? {
                Classification::Decided(boundary) => Ok(boundary),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::Arrangement,
                    self.data.curves[0].family(),
                    reason,
                )),
            }
        })
    }

    pub(crate) fn bezier_boundary_loop_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<&NativeBezierBoundaryLoop2>> {
        Ok(
            match resolve_cached_evaluation(&self.data.bezier_boundary_loop, policy, |attempt| {
                match validate_closed_curve_path_connectivity(self, attempt)? {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                let fragments = match self.native_bezier_fragments_with_policy(attempt)? {
                    Classification::Decided(fragments) => fragments,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match validate_native_fragment_cycle(
                    fragments,
                    self.data.curves[0].family(),
                    attempt,
                )? {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                Ok(Classification::Decided(NativeBezierBoundaryLoop2 {
                    boundary_loop: BezierBoundaryLoop2::from_policy_validated_fragments(
                        fragments
                            .iter()
                            .map(|fragment| fragment.curve().clone())
                            .collect(),
                    ),
                }))
            })
            .map_err(|error| remap_operation(error, CurveOperation2::Arrangement))?
            {
                Classification::Decided(boundary) => Classification::Decided(boundary),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }
}

pub(crate) fn validate_closed_curve_path_connectivity(
    path: &CurvePath2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<()>> {
    if !path.data.strict_connectivity_certified {
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
    if path.data.strict_closure_certified {
        return Ok(Classification::Decided(()));
    }
    match curve_path_points_equal(path.end(), path.start(), policy) {
        Some(true) => Ok(Classification::Decided(())),
        Some(false) => Err(ExactCurveError::invalid(
            CurveOperation2::Arrangement,
            path.curves()[0].family(),
            CurveError::OpenCurvePath,
        )),
        None => Ok(Classification::Uncertain(
            crate::UncertaintyReason::RealSign,
        )),
    }
}

fn validate_native_fragment_cycle(
    fragments: &[NativeBezierFragment2],
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<()>> {
    if fragments.is_empty() {
        return Err(ExactCurveError::invalid(
            CurveOperation2::Arrangement,
            family,
            CurveError::Topology("native Bezier boundary requires nonempty fragments".into()),
        ));
    }
    for (left, right) in fragments
        .iter()
        .zip(fragments.iter().cycle().skip(1))
        .take(fragments.len())
    {
        let (_, left_end) = left.curve().endpoint_refs();
        let (right_start, _) = right.curve().endpoint_refs();
        match curve_path_points_equal(left_end, right_start, policy) {
            Some(true) => {}
            Some(false) => {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Arrangement,
                    family,
                    CurveError::Topology(
                        "native Bezier boundary fragments must be endpoint-connected and closed"
                            .into(),
                    ),
                ));
            }
            None => {
                return Ok(Classification::Uncertain(
                    crate::UncertaintyReason::RealSign,
                ));
            }
        }
    }
    Ok(Classification::Decided(()))
}

fn curve_path_points_equal(left: &Point2, right: &Point2, policy: &CurveContext) -> Option<bool> {
    if left == right {
        Some(true)
    } else {
        crate::classify::is_zero(&left.distance_squared(right), policy)
    }
}

fn native_arc_chord_path(path: &CurvePath2) -> Option<(&Curve2, &CircularArc2, &LineSeg2)> {
    if path.start() != path.end() {
        return None;
    }
    match path.curves() {
        [first, second] => match (first.geometry(), second.geometry()) {
            (CurveGeometry2::CircularArc(arc), CurveGeometry2::Line(chord)) => {
                Some((first, arc, chord))
            }
            (CurveGeometry2::Line(chord), CurveGeometry2::CircularArc(arc)) => {
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

impl<'a> CurvePathView2<'a> {
    /// Returns the borrowed owned-curve slice.
    pub const fn curves(self) -> &'a [Curve2] {
        self.curves
    }

    /// Iterates borrowed curve views without allocation.
    pub fn iter(self) -> impl ExactSizeIterator<Item = CurveView2<'a>> {
        self.curves.iter().map(Curve2::as_view)
    }

    /// Returns the exact path start point.
    pub fn start(self) -> &'a Point2 {
        self.curves[0].start()
    }

    /// Returns the exact path end point.
    pub fn end(self) -> &'a Point2 {
        self.curves
            .last()
            .expect("validated path view is nonempty")
            .end()
    }

    /// Returns an owned path with traversal direction reversed.
    pub fn reversed(self, policy: &CurveContext) -> ExactCurveResult<CurveOutcome<CurvePath2>> {
        resolve_certified_operation(policy, |attempt| self.reversed_raw(attempt))
    }

    fn reversed_raw(self, policy: &CurveContext) -> ExactCurveResult<CurvePath2> {
        let strict_closure_certified = self.end() == self.start();
        let curves = self
            .curves
            .iter()
            .rev()
            .map(|curve| curve.reversed_raw(policy))
            .collect::<ExactCurveResult<Vec<_>>>()?;
        Ok(CurvePath2::from_connected_curves(
            curves,
            false,
            strict_closure_certified,
        ))
    }

    /// Applies an exact planar similarity to the borrowed connected path.
    pub fn transform_similarity(
        self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurvePath2>> {
        resolve_certified_operation(policy, |attempt| {
            self.transform_similarity_raw(transform, attempt)
        })
    }

    fn transform_similarity_raw(
        self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePath2> {
        let strict_closure_certified = self.end() == self.start();
        let curves = self
            .curves
            .iter()
            .map(|curve| curve.transform_similarity_raw(transform, policy))
            .collect::<ExactCurveResult<Vec<_>>>()?;
        Ok(CurvePath2::from_connected_curves(
            curves,
            false,
            strict_closure_certified,
        ))
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
        match value {
            BezierSubcurve2::Quadratic(curve) => curve.into(),
            BezierSubcurve2::Cubic(curve) => curve.into(),
            BezierSubcurve2::RationalQuadratic(curve) => curve.into(),
            BezierSubcurve2::Rational(curve) => curve.into(),
        }
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

    /// Consumes this fragment and returns its native curve.
    pub fn into_curve(self) -> BezierSubcurve2 {
        self.curve
    }
}

impl NativeBezierBoundaryLoop2 {
    /// Returns the validated native Bezier boundary used by arrangement code.
    pub const fn boundary_loop(&self) -> &BezierBoundaryLoop2 {
        &self.boundary_loop
    }

    /// Returns the number of native boundary curves.
    pub fn len(&self) -> usize {
        self.boundary_loop.len()
    }

    /// Returns whether the validated boundary contains no curves.
    pub fn is_empty(&self) -> bool {
        self.boundary_loop.is_empty()
    }

    /// Consumes the retained result into its validated native boundary.
    pub fn into_boundary_loop(self) -> BezierBoundaryLoop2 {
        self.boundary_loop
    }
}

fn compute_curve_bounds(curve: &Curve2) -> ExactCurveResult<Aabb2> {
    let policy = crate::CurveContext::STRICT;
    match curve.geometry() {
        CurveGeometry2::Line(line) => {
            decided_bounds(Aabb2::from_line(line, &policy), curve.family())
        }
        CurveGeometry2::CircularArc(arc) => decided_bounds(
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
    let native = |native_curve, parameter_start: Real, parameter_end: Real| NativeBezierFragment2 {
        curve: native_curve,
        span_range: CurveSpanRange2 {
            start: parameter_start,
            end: parameter_end,
        },
    };
    let unit = || (Real::zero(), Real::one());
    match curve.geometry() {
        CurveGeometry2::Line(line) => {
            let (start, end) = unit();
            Ok(Classification::Decided(vec![native(
                BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(line.clone())),
                start,
                end,
            )]))
        }
        CurveGeometry2::CircularArc(value) => {
            Ok(decompose_circular_arc(value, policy)?.map(|decomposition| {
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
                    .collect()
            }))
        }
        CurveGeometry2::QuadraticBezier(value) => {
            let (start, end) = unit();
            Ok(Classification::Decided(vec![native(
                BezierSubcurve2::Quadratic(value.clone()),
                start,
                end,
            )]))
        }
        CurveGeometry2::CubicBezier(value) => {
            let (start, end) = unit();
            Ok(Classification::Decided(vec![native(
                BezierSubcurve2::Cubic(value.clone()),
                start,
                end,
            )]))
        }
        CurveGeometry2::RationalQuadraticBezier(value) => {
            let (start, end) = unit();
            Ok(Classification::Decided(vec![native(
                BezierSubcurve2::RationalQuadratic(value.clone()),
                start,
                end,
            )]))
        }
        CurveGeometry2::RationalBezier(value) => {
            let (start, end) = unit();
            Ok(Classification::Decided(vec![native(
                BezierSubcurve2::Rational(value.clone()),
                start,
                end,
            )]))
        }
        CurveGeometry2::PolynomialBSpline(value) => {
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
                    .collect(),
            ))
        }
        CurveGeometry2::Nurbs(value) => {
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
                    .collect(),
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

fn exact_corner_parameter(parameter: Real) -> Option<CurveRegionParameter2> {
    Some(CurveRegionParameter2::from_bezier(BezierParameter2::Exact(
        parameter,
    )))
}

#[derive(Clone, Debug)]
struct CornerCut2 {
    /// Canonical carrier-local parameter when the consuming representation
    /// needs it. Native fillet arcs may defer their sweep parameter because
    /// exact Cartesian incidence is sufficient for reconstruction.
    parameter: Option<CurveRegionParameter2>,
    point: RationalBezierIntersectionPointEvidence2,
    placement: CornerPlacement2,
}

impl CornerCut2 {
    fn exact_point(&self) -> Option<&Point2> {
        self.point.as_exact()
    }

    fn into_retained_evidence(self) -> Option<CornerTrimCut2> {
        let parameter = self.parameter?;
        Some(CornerTrimCut2 {
            parameter,
            point: self.point,
            placement: self.placement,
            replacement: None,
        })
    }

    fn exact_parameter(&self) -> Option<&Real> {
        self.parameter.as_ref()?.as_exact()
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
}

impl CornerReplacement2 {
    pub(crate) const fn as_curve(&self) -> Option<&BezierSubcurve2> {
        match self {
            Self::Curve(curve) => Some(curve),
            Self::AnalyticParallel { .. } => None,
        }
    }

    pub(crate) const fn as_parallel_fragment(&self) -> Option<&crate::BezierParallelFragment2> {
        match self {
            Self::AnalyticParallel { fragment, .. } => Some(fragment),
            Self::Curve(_) => None,
        }
    }

    pub(crate) fn parallel_source_parameter_map(&self) -> Option<(&Real, &Real)> {
        match self {
            Self::AnalyticParallel {
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
    pub(crate) parameter: CurveRegionParameter2,
    pub(crate) point: RationalBezierIntersectionPointEvidence2,
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
    center: RationalBezierIntersectionPointEvidence2,
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
    /// A promoted construction parameter is retained only when the source
    /// trim parameter belongs to a compact selected fiber.
    pub(crate) parameter: Option<BezierParameter2>,
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
    /// Exact source-circle chart parameter inherited from the affine radial
    /// offset map. `None` retains the older deferred-search path used when the
    /// center solver did not traverse the circular carrier itself.
    pub(crate) contact_seed: Option<RetainedArcFilletContactSeed2>,
}

#[derive(Clone, Debug)]
pub(crate) struct RetainedArcFilletContactSeed2 {
    /// `None` names the authored circular span; `Some(i)` names complement
    /// projective cell `i` in traversal order.
    pub(crate) complement_span: Option<usize>,
    pub(crate) parameter: CurveRegionParameter2,
}

impl FilletCorner2 {
    pub(crate) fn into_retained_cut_evidence(
        self,
    ) -> Option<(
        CornerTrimCut2,
        CornerTrimCut2,
        RationalBezierIntersectionPointEvidence2,
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

    fn is_empty(&self) -> bool {
        self.first.is_none() && self.second.is_none() && self.overflow.is_empty()
    }
}

fn exact_linear_corner_line(curve: &Curve2) -> Option<&LineSeg2> {
    match curve.geometry() {
        CurveGeometry2::Line(line) => Some(line),
        CurveGeometry2::QuadraticBezier(curve) => curve.retained_exact_line_image(),
        CurveGeometry2::CircularArc(_)
        | CurveGeometry2::CubicBezier(_)
        | CurveGeometry2::RationalQuadraticBezier(_)
        | CurveGeometry2::RationalBezier(_)
        | CurveGeometry2::PolynomialBSpline(_)
        | CurveGeometry2::Nurbs(_) => None,
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

struct RationalArcOffsetEvaluator2 {
    offset: RationalBezier2,
    canonical_source: RationalBezier2,
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

impl ExactCornerArc2<'_> {
    fn support(&self) -> &CircularArc2 {
        match self {
            Self::Native(arc) => arc,
            Self::RetainedRational(retained) => &retained.support,
        }
    }

    fn source_parameter_at_point(
        &self,
        point: &Point2,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Real>> {
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
        let Some(parameter) = parameter.as_exact() else {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::Unsupported,
            ));
        };
        let domain = retained.source.parameter_domain();
        Ok(Some(
            domain.start() + (domain.end() - domain.start()) * parameter,
        ))
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
                    retained.source.parameter_domain().end().clone()
                } else {
                    retained.source.parameter_domain().start().clone()
                }
            }
        }
    }

    /// Materializes the exact concentric image of any certified circular arc.
    ///
    /// An affine radial scale maps every rational Bezier control point while
    /// preserving its homogeneous weight and local parameter. This gives the
    /// analytic-parallel/rational intersection authority a finite exact arc
    /// offset carrier without adding a separate arc/parallel solver.
    fn rational_offset_evaluator(
        &self,
        source_radius: &Real,
        signed_radius: &Real,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<RationalArcOffsetEvaluator2> {
        let decomposition = match self
            .support()
            .rational_bezier_decomposition_with_policy(policy)
            .map_err(|error| error.with_operation(operation))?
        {
            Classification::Decided(decomposition) => decomposition,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, family, reason));
            }
        };
        let [span] = decomposition.spans() else {
            return Err(ExactCurveError::blocked(
                operation,
                family,
                crate::UncertaintyReason::Unsupported,
            ));
        };
        let canonical_source: RationalBezier2 = span.curve().clone().into();
        let scale = (signed_radius / source_radius)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))?;
        let center = self.support().center();
        let control_points = canonical_source
            .control_points()
            .iter()
            .map(|control| {
                let radial = control.delta_from(center);
                center.translated(&radial.0 * &scale, &radial.1 * &scale)
            })
            .collect();
        let offset = RationalBezier2::try_new(control_points, canonical_source.weights().to_vec())
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
        let support = CircularArc2::new_with_certified_radius(
            offset.start().clone(),
            offset.end().clone(),
            center.clone(),
            signed_radius * signed_radius,
            self.support().is_clockwise(),
            None,
        );
        Ok(RationalArcOffsetEvaluator2 {
            offset,
            canonical_source,
            support,
        })
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

    pub(crate) fn supports_extension(&self, operation: CurveOperation2) -> bool {
        match self {
            Self::Line(_) | Self::PromotedLine(_) | Self::Arc(_) => true,
            Self::Bezier(curve) => {
                operation == CurveOperation2::Chamfer
                    && matches!(
                        curve.geometry(),
                        CurveGeometry2::QuadraticBezier(_)
                            | CurveGeometry2::CubicBezier(_)
                            | CurveGeometry2::RationalQuadraticBezier(_)
                            | CurveGeometry2::RationalBezier(_)
                    )
            }
            Self::RetainedRationalArc(_) => operation == CurveOperation2::Chamfer,
            Self::NativeBezierSpan(_) => operation == CurveOperation2::Chamfer,
            Self::AlgebraicChord(_) => true,
            Self::AnalyticParallel(_) => matches!(
                operation,
                CurveOperation2::Chamfer | CurveOperation2::Fillet
            ),
            Self::AlgebraicCusp(_) => operation == CurveOperation2::Chamfer,
        }
    }
}

fn retained_rational_arc_support(
    curve: &Curve2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CircularArc2>> {
    let support = match curve.geometry() {
        CurveGeometry2::RationalQuadraticBezier(conic) => {
            rational_quadratic_circular_arc(conic, policy)
        }
        CurveGeometry2::RationalBezier(rational) => rational_bezier_circular_arc(rational, policy),
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
        CurveGeometry2::Line(line) => return Ok(Some(ExactCornerCarrier2::Line(line))),
        CurveGeometry2::QuadraticBezier(source) if source.retained_exact_line_image().is_some() => {
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
        CurveGeometry2::CircularArc(arc) => Some(ExactCornerCarrier2::Arc(arc)),
        CurveGeometry2::RationalQuadraticBezier(_) | CurveGeometry2::RationalBezier(_) => Some(
            match retained_rational_arc_support(curve, operation, policy)? {
                Some(support) => retained(support),
                None => bezier(),
            },
        ),
        CurveGeometry2::QuadraticBezier(_) | CurveGeometry2::CubicBezier(_) => Some(bezier()),
        CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_) => {
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
        CurveGeometry2::Line(_) => None,
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
            CurveGeometry2::QuadraticBezier(source) => source.parallel_left(distance),
            CurveGeometry2::CubicBezier(source) => source.parallel_left(distance),
            CurveGeometry2::RationalQuadraticBezier(source) => source.parallel_left(distance),
            CurveGeometry2::RationalBezier(source) => source.parallel_left(distance),
            CurveGeometry2::Line(_)
            | CurveGeometry2::CircularArc(_)
            | CurveGeometry2::PolynomialBSpline(_)
            | CurveGeometry2::Nurbs(_) => {
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
                if previous {
                    source.end()
                } else {
                    source.start()
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

    fn public_parameter(self, parameter: &Real) -> Real {
        let (start, end) = match self {
            Self::Direct(source) => {
                let domain = source.parameter_domain();
                (domain.start(), domain.end())
            }
            Self::NativeSpan(fragment) => fragment.parameter_range(),
        };
        start + (end - start) * parameter
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
    if mode == CurveCornerMode2::TrimOrExtend
        && !fillet_carrier_pair_supports_extension(&previous, &next)
    {
        return Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            if !previous.supports_extension(CurveOperation2::Fillet) {
                previous_family
            } else {
                next_family
            },
            crate::UncertaintyReason::Unsupported,
        ));
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
        previous_family,
        next_family,
        policy,
    )
}

fn fillet_carrier_pair_supports_extension(
    previous: &ExactCornerCarrier2<'_>,
    next: &ExactCornerCarrier2<'_>,
) -> bool {
    if previous.supports_extension(CurveOperation2::Fillet)
        && next.supports_extension(CurveOperation2::Fillet)
    {
        return true;
    }
    let is_affine_line = |carrier: &ExactCornerCarrier2<'_>| {
        matches!(
            carrier,
            ExactCornerCarrier2::Line(_)
                | ExactCornerCarrier2::PromotedLine(_)
                | ExactCornerCarrier2::AlgebraicChord(_)
        )
    };
    let is_arc = |carrier: &ExactCornerCarrier2<'_>| {
        matches!(
            carrier,
            ExactCornerCarrier2::Arc(_) | ExactCornerCarrier2::RetainedRationalArc(_)
        )
    };
    let is_selected_circle = |carrier: &ExactCornerCarrier2<'_>| {
        matches!(carrier, ExactCornerCarrier2::AlgebraicCusp(_))
    };
    let is_analytic_parallel = |carrier: &ExactCornerCarrier2<'_>| {
        matches!(carrier, ExactCornerCarrier2::AnalyticParallel(_))
    };
    let is_bezier = |carrier: &ExactCornerCarrier2<'_>| {
        matches!(
            carrier,
            ExactCornerCarrier2::Bezier(_) | ExactCornerCarrier2::NativeBezierSpan(_)
        )
    };
    (is_affine_line(previous) && is_arc(next))
        || (is_arc(previous) && is_affine_line(next))
        || (is_arc(previous) && is_arc(next))
        || (is_arc(previous) && is_selected_circle(next))
        || (is_selected_circle(previous) && is_arc(next))
        || (is_arc(previous) && is_analytic_parallel(next))
        || (is_analytic_parallel(previous) && is_arc(next))
        || (is_affine_line(previous) && is_selected_circle(next))
        || (is_selected_circle(previous) && is_affine_line(next))
        || (is_selected_circle(previous) && is_selected_circle(next))
        || (is_selected_circle(previous) && is_analytic_parallel(next))
        || (is_analytic_parallel(previous) && is_selected_circle(next))
        || (is_affine_line(previous) && is_bezier(next))
        || (is_bezier(previous) && is_affine_line(next))
        || (is_arc(previous) && is_bezier(next))
        || (is_bezier(previous) && is_arc(next))
        || (is_selected_circle(previous) && is_bezier(next))
        || (is_bezier(previous) && is_selected_circle(next))
        || (is_analytic_parallel(previous) && is_bezier(next))
        || (is_bezier(previous) && is_analytic_parallel(next))
        || (is_bezier(previous) && is_bezier(next))
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
}

impl FilletParallelSource2<'_> {
    const fn retained(&self) -> Option<&crate::BezierParallelFragment2> {
        match self {
            Self::Direct(_) => None,
            Self::Retained(source) => Some(source),
        }
    }

    fn parameter_range(&self) -> BezierParameterRange2 {
        match self {
            Self::Direct(_) => BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            ),
            Self::Retained(source) => source.range().clone(),
        }
    }

    fn incident_domain(
        &self,
        support: &BezierParallel2,
        previous: bool,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<crate::bezier_offset::BezierParallelIncidentDomain2> {
        let range = self.parameter_range();
        let source_reversed = self.retained().is_some_and(|source| source.is_reversed());
        let extends_toward_higher_parameter = previous != source_reversed;
        let (endpoint, direction) = if extends_toward_higher_parameter {
            (range.end(), crate::BezierParameterRayDirection2::Increasing)
        } else {
            (
                range.start(),
                crate::BezierParameterRayDirection2::Decreasing,
            )
        };
        match support
            .incident_domain_from_parameter(endpoint, direction, policy)
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
        }
    }

    fn parameter_is_admissible(
        &self,
        parameter: &BezierParameter2,
        previous: bool,
        mode: CurveCornerMode2,
        incident_domain: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let placement = match self {
            Self::Direct(_) => bezier_corner_parameter_placement(
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
        };
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

    fn support_reverses_source(
        &self,
        support: &BezierParallel2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let Self::Direct(_) = self else {
            return retained_fillet_parallel_support_reverses_source(
                self.retained()
                    .expect("the non-direct parallel source is retained"),
                support,
                family,
                policy,
            );
        };
        let range = BezierParameterRange2::new_validated(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        );
        match support
            .regular_fragment_derivative_scale_sign(&range, policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
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

    fn parallel_distance(&self) -> Real {
        match self {
            Self::Direct(_) => Real::zero(),
            Self::Retained(source) => source.parallel().distance().clone(),
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
                        point: RationalBezierIntersectionPointEvidence2::Exact(
                            support.center().clone(),
                        ),
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
        point: RationalBezierIntersectionPointEvidence2,
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
    },
}

impl FilletOffsetCarrier2<'_, '_> {
    fn retained_fillet_frame(
        &self,
        anchor_is_previous: bool,
        anchor_parameter: Option<&CurveRegionParameter2>,
        anchor_evidence: Option<RetainedFilletAnchorEvidence2>,
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
                    let Some(center_parameter) = center_frame.parameter.or_else(|| {
                        anchor_parameter
                            .and_then(CurveRegionParameter2::as_bezier_parameter)
                            .cloned()
                    }) else {
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
                let radial_frame = if matches!(
                    crate::classify::real_sign(signed_radius, &CurveContext::STRICT),
                    Some(RealSign::Positive)
                ) {
                    match (
                        anchor_evidence
                            .as_ref()
                            .and_then(|evidence| evidence.center_parallel.as_ref())
                            .map(|frame| frame.support.clone()),
                        anchor_parameter
                            .and_then(CurveRegionParameter2::as_bezier_parameter)
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
                let complementary = anchor_parameter
                    .is_some_and(CurveRegionParameter2::is_algebraic_cusp_complement);
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
                    RationalBezierIntersectionPointEvidence2::Exact(point) => Some(point.clone()),
                    RationalBezierIntersectionPointEvidence2::Algebraic(image) => {
                        image.exact_rational_point(&CurveContext::STRICT)
                    }
                    RationalBezierIntersectionPointEvidence2::AlgebraicChordPair(_)
                    | RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_)
                    | RationalBezierIntersectionPointEvidence2::AlgebraicCuspChordDerived(_)
                    | RationalBezierIntersectionPointEvidence2::AlgebraicChordParallel(_)
                    | RationalBezierIntersectionPointEvidence2::AnalyticParallel(_)
                    | RationalBezierIntersectionPointEvidence2::Similarity(_) => None,
                };
                let selected_center_parameter = anchor_parameter
                    .and_then(CurveRegionParameter2::as_algebraic_cusp)
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
                let Some(center_parameter) = anchor_parameter
                    .and_then(CurveRegionParameter2::as_bezier_parameter)
                    .cloned()
                else {
                    return Ok(None);
                };
                (
                    RetainedFilletRadialFrame2::ParallelNormal {
                        center_support: support.clone(),
                        center_parameter,
                        policy: *policy,
                    },
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
    point: RationalBezierIntersectionPointEvidence2,
    previous_parameter: Option<CurveRegionParameter2>,
    next_parameter: Option<CurveRegionParameter2>,
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
    fn parameter(&self, previous: bool) -> Option<&CurveRegionParameter2> {
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
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    let previous = PreparedFilletCarrier2::new(previous, previous_family, policy)?;
    let next = PreparedFilletCarrier2::new(next, next_family, policy)?;
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
            } else if center.point.as_exact().is_none()
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
                    let extension_line_precedes_chord = mode == CurveCornerMode2::TrimOrExtend
                        && matches!(
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
                            || extension_line_precedes_chord
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
                    let force_chord_normal = mode == CurveCornerMode2::TrimOrExtend
                        && matches!(
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
                        matches!(
                            &frame.radial_frame,
                            RetainedFilletRadialFrame2::ParallelNormal { .. }
                        ) == prefer_parallel_frame
                            && !matches!(
                                &frame.radial_frame,
                                RetainedFilletRadialFrame2::ChordNormal { .. }
                            )
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

struct RetainedFilletCuspRationalContact2 {
    other_parameter: CurveRegionParameter2,
    cusp_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    point: RationalBezierIntersectionPointEvidence2,
}

struct RetainedFilletCuspRationalContacts2 {
    contacts: Vec<RetainedFilletCuspRationalContact2>,
    coincident: bool,
}

fn retained_fillet_cusp_parameter_order(
    first: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
    second: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
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

fn retained_fillet_cusp_overlap_range(
    cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
    overlap: &crate::bezier_offset::BezierAlgebraicCuspSemicircleMappedOverlap2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<BezierParameterRange2>> {
    let overlap_start = overlap.cusp_start_parameter();
    let overlap_end = overlap.cusp_end_parameter();
    let clipped_start = if retained_fillet_cusp_parameter_order(
        &overlap_start,
        cusp.start_parameter(),
        family,
        policy,
    )?
    .is_lt()
    {
        cusp.start_parameter().clone()
    } else {
        overlap_start
    };
    let clipped_end = if retained_fillet_cusp_parameter_order(
        &overlap_end,
        cusp.end_parameter(),
        family,
        policy,
    )?
    .is_gt()
    {
        cusp.end_parameter().clone()
    } else {
        overlap_end
    };
    if !retained_fillet_cusp_parameter_order(&clipped_start, &clipped_end, family, policy)?.is_lt()
    {
        return Ok(None);
    }
    let map_endpoint = |parameter| match overlap
        .other_parameter_for_cusp(parameter, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(parameter) => Ok(parameter),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    };
    let first = map_endpoint(&clipped_start)?;
    let second = map_endpoint(&clipped_end)?;
    let order = match first
        .cmp_by_refinement(&second, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            ));
        }
    };
    Ok(match order {
        std::cmp::Ordering::Less => Some(BezierParameterRange2::new_validated(first, second)),
        std::cmp::Ordering::Greater => Some(BezierParameterRange2::new_validated(second, first)),
        std::cmp::Ordering::Equal => {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Fillet,
                family,
                CurveError::DegenerateOverlapRange,
            ));
        }
    })
}

fn retained_fillet_cusp_pair_overlap_is_positive(
    first: &crate::BezierAlgebraicCuspSemicircleFragment2,
    second: &crate::BezierAlgebraicCuspSemicircleFragment2,
    overlap: &crate::bezier_offset::BezierAlgebraicCuspSemicirclePairOverlap2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let maximum = |parameters: [&crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2;
                       2]| {
        if retained_fillet_cusp_parameter_order(parameters[0], parameters[1], family, policy)?
            .is_lt()
        {
            Ok(parameters[1].clone())
        } else {
            Ok(parameters[0].clone())
        }
    };
    let minimum = |parameters: [&crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2;
                       2]| {
        if retained_fillet_cusp_parameter_order(parameters[0], parameters[1], family, policy)?
            .is_gt()
        {
            Ok(parameters[1].clone())
        } else {
            Ok(parameters[0].clone())
        }
    };
    let first_overlap_start = overlap.first_start_parameter();
    let first_overlap_end = overlap.first_end_parameter();
    let first_start = maximum([&first_overlap_start, first.start_parameter()])?;
    let first_end = minimum([&first_overlap_end, first.end_parameter()])?;
    if !retained_fillet_cusp_parameter_order(&first_start, &first_end, family, policy)?.is_lt() {
        return Ok(false);
    }

    let mapped_first = overlap.map_parameter(&first_start, true);
    let mapped_second = overlap.map_parameter(&first_end, true);
    let (mapped_low, mapped_high) =
        if retained_fillet_cusp_parameter_order(&mapped_first, &mapped_second, family, policy)?
            .is_lt()
        {
            (mapped_first, mapped_second)
        } else {
            (mapped_second, mapped_first)
        };
    let second_overlap_start = overlap.second_start_parameter();
    let second_overlap_end = overlap.second_end_parameter();
    let second_start = maximum([&mapped_low, &second_overlap_start])?;
    let second_start = maximum([&second_start, second.start_parameter()])?;
    let second_end = minimum([&mapped_high, &second_overlap_end])?;
    let second_end = minimum([&second_end, second.end_parameter()])?;
    Ok(retained_fillet_cusp_parameter_order(&second_start, &second_end, family, policy)?.is_lt())
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
) -> ExactCurveResult<Option<RationalBezierIntersectionPointEvidence2>> {
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
        center.exact_rational_point(&CurveContext::STRICT),
        start.exact_rational_point(&CurveContext::STRICT),
        end.exact_rational_point(&CurveContext::STRICT),
    ) else {
        return Ok(None);
    };
    let arc = CircularArc2::try_from_center(start, end, center, support.is_clockwise())
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
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

fn retained_fillet_ranges_overlap(
    first: &BezierParameterRange2,
    second: &BezierParameterRange2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    Ok(
        retained_fillet_bezier_parameter_order(first.end(), second.start(), family, policy)?
            .is_gt()
            && retained_fillet_bezier_parameter_order(second.end(), first.start(), family, policy)?
                .is_gt(),
    )
}

fn retained_fillet_range_overlaps_incident_domain(
    range: &BezierParameterRange2,
    domain: &crate::bezier_offset::BezierParallelIncidentDomain2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    Ok(match domain.direction() {
        crate::BezierParameterRayDirection2::Decreasing => {
            retained_fillet_bezier_parameter_order(
                range.start(),
                domain.endpoint(),
                family,
                policy,
            )?
            .is_lt()
                && match domain.barrier() {
                    Some(barrier) => retained_fillet_bezier_parameter_order(
                        range.end(),
                        barrier,
                        family,
                        policy,
                    )?
                    .is_gt(),
                    None => true,
                }
        }
        crate::BezierParameterRayDirection2::Increasing => {
            retained_fillet_bezier_parameter_order(range.end(), domain.endpoint(), family, policy)?
                .is_gt()
                && match domain.barrier() {
                    Some(barrier) => retained_fillet_bezier_parameter_order(
                        range.start(),
                        barrier,
                        family,
                        policy,
                    )?
                    .is_lt(),
                    None => true,
                }
        }
    })
}

fn retained_fillet_bezier_parameter_order(
    first: &BezierParameter2,
    second: &BezierParameter2,
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
    first_fragment: &BezierParameterRange2,
    second_fragment: &BezierParameterRange2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let ascending = |range: &BezierParameterRange2| {
        Ok(
            match retained_fillet_bezier_parameter_order(
                range.start(),
                range.end(),
                family,
                policy,
            )? {
                std::cmp::Ordering::Less => (range.start().clone(), range.end().clone()),
                std::cmp::Ordering::Greater => (range.end().clone(), range.start().clone()),
                std::cmp::Ordering::Equal => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Fillet,
                        family,
                        CurveError::DegenerateOverlapRange,
                    ));
                }
            },
        )
    };
    let maximum = |parameters: [&BezierParameter2; 3]| {
        let mut selected = parameters[0];
        for parameter in &parameters[1..] {
            if retained_fillet_bezier_parameter_order(selected, parameter, family, policy)?.is_lt()
            {
                selected = parameter;
            }
        }
        Ok(selected.clone())
    };
    let minimum = |parameters: [&BezierParameter2; 3]| {
        let mut selected = parameters[0];
        for parameter in &parameters[1..] {
            if retained_fillet_bezier_parameter_order(selected, parameter, family, policy)?.is_gt()
            {
                selected = parameter;
            }
        }
        Ok(selected.clone())
    };

    let (first_overlap_start, first_overlap_end) = ascending(overlap.first_range())?;
    let (first_fragment_start, first_fragment_end) = ascending(first_fragment)?;
    let first_start = if retained_fillet_bezier_parameter_order(
        &first_overlap_start,
        &first_fragment_start,
        family,
        policy,
    )?
    .is_lt()
    {
        first_fragment_start.clone()
    } else {
        first_overlap_start.clone()
    };
    let first_end = if retained_fillet_bezier_parameter_order(
        &first_overlap_end,
        &first_fragment_end,
        family,
        policy,
    )?
    .is_gt()
    {
        first_fragment_end.clone()
    } else {
        first_overlap_end.clone()
    };
    if !retained_fillet_bezier_parameter_order(&first_start, &first_end, family, policy)?.is_lt() {
        return Ok(false);
    }

    let correspondence = RationalBezierOverlapParameterCorrespondence2::for_overlap(
        first_curve,
        second_curve,
        overlap,
        policy,
    );
    let map = |parameter| match correspondence
        .map_first_to_second(
            parameter,
            overlap.first_range(),
            overlap.second_range(),
            policy,
        )
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(Some(parameter)) => Ok(parameter),
        Classification::Decided(None) => Err(ExactCurveError::invalid(
            CurveOperation2::Fillet,
            family,
            CurveError::Topology(
                "a certified fillet overlap omitted its parameter correspondence".into(),
            ),
        )),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    };
    let mapped_first = map(&first_start)?;
    let mapped_second = map(&first_end)?;
    let (mapped_low, mapped_high) = match retained_fillet_bezier_parameter_order(
        &mapped_first,
        &mapped_second,
        family,
        policy,
    )? {
        std::cmp::Ordering::Less => (&mapped_first, &mapped_second),
        std::cmp::Ordering::Greater => (&mapped_second, &mapped_first),
        std::cmp::Ordering::Equal => return Ok(false),
    };
    let (second_overlap_start, second_overlap_end) = ascending(overlap.second_range())?;
    let (second_fragment_start, second_fragment_end) = ascending(second_fragment)?;
    let second_start = maximum([mapped_low, &second_overlap_start, &second_fragment_start])?;
    let second_end = minimum([mapped_high, &second_overlap_end, &second_fragment_end])?;
    Ok(retained_fillet_bezier_parameter_order(&second_start, &second_end, family, policy)?.is_lt())
}

fn retained_fillet_parameter_component_overlap_is_positive(
    overlap: &crate::bezier_offset::BezierParameterComponentOverlap2,
    first_fragment: &BezierParameterRange2,
    second_fragment: &BezierParameterRange2,
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

fn retained_fillet_cusp_rational_contacts(
    cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
    rational: &RationalBezier2,
    include_cusp_start: bool,
    include_cusp_end: bool,
    include_rational_start: bool,
    include_rational_end: bool,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<RetainedFilletCuspRationalContacts2> {
    let (intersections, parameter_map) = match cusp
        .semicircle()
        .rational_intersections_with_parameter_map(rational, policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(result) => result,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            ));
        }
    };
    let zero = CurveRegionParameter2::from_bezier(BezierParameter2::Exact(Real::zero()));
    let one = CurveRegionParameter2::from_bezier(BezierParameter2::Exact(Real::one()));
    let mut retained = Vec::new();
    let mut retain_contact =
        |other_parameter: CurveRegionParameter2,
         cusp_parameter: crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
         point: RationalBezierIntersectionPointEvidence2| {
            let other_at_boundary = |boundary: &CurveRegionParameter2| {
                other_parameter
                    .cmp_by_refinement(boundary, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })
                    .and_then(|order| match order {
                        Classification::Decided(order) => Ok(order == std::cmp::Ordering::Equal),
                        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            family,
                            reason,
                        )),
                    })
            };
            if (!include_rational_start && other_at_boundary(&zero)?)
                || (!include_rational_end && other_at_boundary(&one)?)
            {
                return Ok(());
            }
            match cusp
                .contains_parameter(
                    &cusp_parameter,
                    include_cusp_start,
                    include_cusp_end,
                    policy,
                )
                .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
            {
                Classification::Decided(true) => {
                    retained.push(RetainedFilletCuspRationalContact2 {
                        other_parameter,
                        cusp_parameter,
                        point,
                    });
                    Ok(())
                }
                Classification::Decided(false) => Ok(()),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    reason,
                )),
            }
        };
    match intersections {
        crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Contacts(
            contacts,
        ) => {
            for contact in contacts {
                let cusp_parameter = match contact.location {
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Start => {
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                            Real::zero(),
                        )
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::End => {
                        crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(
                            Real::one(),
                        )
                    }
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
                        parameter_map
                            .as_ref()
                            .ok_or_else(|| {
                                ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    family,
                                    crate::UncertaintyReason::Unsupported,
                                )
                            })?
                            .contact_parameter(&contact)
                    }
                };
                retain_contact(
                    CurveRegionParameter2::from_bezier(contact.other_parameter),
                    cusp_parameter,
                    contact.point,
                )?;
            }
        }
        crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiberContacts(
            contacts,
        ) => {
            for contact in contacts {
                retain_contact(
                    CurveRegionParameter2::from_selected_fiber(
                        contact.other_parameter().clone(),
                    ),
                    contact.cusp_parameter(),
                    contact.point_evidence(),
                )?;
            }
        }
        crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiberOverlaps(
            overlaps,
        ) => {
            let range = BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            );
            for overlap in overlaps {
                if retained_selected_fillet_overlap_is_positive(
                    &overlap, &range, None, cusp, family, family, policy,
                )? {
                    return Ok(RetainedFilletCuspRationalContacts2 {
                        contacts: Vec::new(),
                        coincident: true,
                    });
                }
            }
            return Ok(RetainedFilletCuspRationalContacts2 {
                contacts: Vec::new(),
                coincident: false,
            });
        }
        crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::Overlaps(
            overlaps,
        ) => {
            for overlap in overlaps {
                if retained_fillet_cusp_overlap_range(cusp, &overlap, family, policy)?.is_some() {
                    return Ok(RetainedFilletCuspRationalContacts2 {
                        contacts: Vec::new(),
                        coincident: true,
                    });
                }
            }
            return Ok(RetainedFilletCuspRationalContacts2 {
                contacts: Vec::new(),
                coincident: false,
            });
        }
        crate::bezier_offset::BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                crate::UncertaintyReason::Unsupported,
            ));
        }
    }
    // A selected-fiber scalar stays compact for source trimming. Constructing
    // a new fillet circle is the one genuine carrier switch in this path, so
    // promote only accepted center contacts to a standalone exact algebraic
    // point. The selected authority chooses the identical resultant root; no
    // isolating bound or terminal approximation becomes construction data.
    for contact in &mut retained {
        let Some(parameter) = contact.other_parameter.as_selected_fiber() else {
            continue;
        };
        let parameter = match parameter
            .promoted_bezier_parameter(policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    reason,
                ));
            }
        };
        contact.point = crate::rational_bezier_general::exact_contact_point_evidence(
            rational, &parameter, policy,
        )
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        .ok_or_else(|| {
            ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                crate::UncertaintyReason::Unsupported,
            )
        })?;
    }
    Ok(RetainedFilletCuspRationalContacts2 {
        contacts: retained,
        coincident: false,
    })
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

fn retained_selected_fillet_overlap_is_positive(
    overlap: &crate::bezier_offset::BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2,
    analytic_range: &BezierParameterRange2,
    incident_domain: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
    cusp_source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    analytic_family: CurveFamily2,
    cusp_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let selected_order =
        |parameter: &crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2,
         boundary: &BezierParameter2| {
            parameter
                .cmp_bezier_parameter(boundary, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, analytic_family, cause)
                })
                .and_then(|order| match order {
                    Classification::Decided(order) => Ok(order),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        analytic_family,
                        reason,
                    )),
                })
        };
    let other_start = overlap.other_start_parameter();
    let other_end = overlap.other_end_parameter();
    let overlaps_authored = selected_order(&other_end, analytic_range.start())?.is_gt()
        && selected_order(&other_start, analytic_range.end())?.is_lt();
    let overlaps_extension = if let Some(domain) = incident_domain {
        match domain.direction() {
            crate::BezierParameterRayDirection2::Decreasing => {
                selected_order(&other_start, domain.endpoint())?.is_lt()
                    && match domain.barrier() {
                        Some(barrier) => selected_order(&other_end, barrier)?.is_gt(),
                        None => true,
                    }
            }
            crate::BezierParameterRayDirection2::Increasing => {
                selected_order(&other_end, domain.endpoint())?.is_gt()
                    && match domain.barrier() {
                        Some(barrier) => selected_order(&other_start, barrier)?.is_lt(),
                        None => true,
                    }
            }
        }
    } else {
        false
    };
    if !overlaps_authored && !overlaps_extension {
        return Ok(false);
    }
    let cusp_order =
        |parameter: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2,
         boundary: &crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2| {
            parameter
                .cmp_by_refinement(boundary, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, cusp_family, cause)
                })
                .and_then(|order| match order {
                    Classification::Decided(order) => Ok(order),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        cusp_family,
                        reason,
                    )),
                })
        };
    Ok(
        cusp_order(&overlap.cusp_end_parameter(), cusp_source.start_parameter())?
            == std::cmp::Ordering::Greater
            && cusp_order(&overlap.cusp_start_parameter(), cusp_source.end_parameter())?
                == std::cmp::Ordering::Less,
    )
}

fn fillet_offset_centers(
    previous: &FilletOffsetCarrier2<'_, '_>,
    next: &FilletOffsetCarrier2<'_, '_>,
    mode: CurveCornerMode2,
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<FilletCenters2> {
    let exact_parameter =
        |parameter| CurveRegionParameter2::from_bezier(BezierParameter2::Exact(parameter));
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
                if !parallel_source.parameter_is_admissible(
                    &parameter,
                    bezier_is_previous,
                    mode,
                    incident_domain.as_ref(),
                    bezier_family,
                    policy,
                )? {
                    continue;
                }
                let point = analytic_parallel_point_evidence(
                    bezier,
                    &parameter,
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
                        contact_seed: None,
                    }),
                });
                let bezier_parameter = CurveRegionParameter2::from_bezier(parameter);
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
                let first_domain = previous_incident_domain
                    .as_ref()
                    .expect("previous incident domain was built");
                let second_domain = next_incident_domain
                    .as_ref()
                    .expect("next incident domain was built");
                let incident = match (if identical_supports {
                    previous.self_intersections_with_incident_rays(
                        first_domain,
                        second_domain,
                        policy,
                    )
                } else {
                    previous.parallel_intersections_with_incident_rays(
                        next,
                        first_domain,
                        second_domain,
                        policy,
                    )
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
            let previous_range = previous_source.parameter_range();
            let next_range = next_source.parameter_range();
            if !intersections.overlaps().is_empty() {
                let mut rational_sources = None;
                for overlap in intersections.overlaps() {
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
                            &previous_range,
                            &next_range,
                            previous_family,
                            policy,
                        )? {
                            positive = true;
                            break;
                        }
                    }
                    if !has_component {
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
                            &previous_range,
                            &next_range,
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
                    contact.first_parameter(),
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
                    if !previous_source.parameter_is_admissible(
                        previous_parameter,
                        true,
                        mode,
                        previous_incident_domain.as_ref(),
                        previous_family,
                        policy,
                    )? || !next_source.parameter_is_admissible(
                        next_parameter,
                        false,
                        mode,
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
                        previous_parameter: Some(CurveRegionParameter2::from_bezier(
                            previous_parameter.clone(),
                        )),
                        next_parameter: Some(CurveRegionParameter2::from_bezier(
                            next_parameter.clone(),
                        )),
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
                corner_parameter.as_exact().and_then(|parameter| {
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
                if !source.parameter_is_admissible(
                    &parameter,
                    parallel_is_previous,
                    mode,
                    incident_domain.as_ref(),
                    parallel_family,
                    policy,
                )? {
                    continue;
                }
                let procedural_affine_contact =
                    mode == CurveCornerMode2::TrimOrExtend && parameter.as_exact().is_none();
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
                    &parameter,
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
                        Some(CurveRegionParameter2::from_bezier(parameter))
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
                let parallel_parameter = Some(CurveRegionParameter2::from_bezier(parameter));
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
            let analytic_range = parallel_source.retained().map_or_else(
                || {
                    BezierParameterRange2::new_validated(
                        BezierParameter2::Exact(Real::zero()),
                        BezierParameter2::Exact(Real::one()),
                    )
                },
                |source| source.range().clone(),
            );
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
                            if complementary
                                && contact.location()
                                    != crate::bezier_offset::BezierAlgebraicCuspSemicircleContactLocation2::Interior
                            {
                                continue;
                            }
                            let cusp_parameter = contact.cusp_parameter();
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
                            let mut cross = contact.tangent_cross_sign();
                            let mut dot = match contact
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
                            if cusp_support_reverses_source {
                                cross = reverse_fillet_sign(cross);
                                dot = reverse_fillet_sign(dot);
                            }
                            if analytic_support_reverses_source {
                                cross = reverse_fillet_sign(cross);
                                dot = reverse_fillet_sign(dot);
                            }
                            let analytic_parameter = match contact
                                .other_parameter()
                                .promoted_bezier_parameter(policy)
                                .map_err(|cause| {
                                    ExactCurveError::invalid(
                                        CurveOperation2::Fillet,
                                        analytic_family,
                                        cause,
                                    )
                                })? {
                                Classification::Decided(parameter) => parameter,
                                Classification::Uncertain(reason) => {
                                    return Err(ExactCurveError::blocked(
                                        CurveOperation2::Fillet,
                                        analytic_family,
                                        reason,
                                    ));
                                }
                            };
                            if !parallel_source.parameter_is_admissible(
                                &analytic_parameter,
                                !cusp_is_previous,
                                mode,
                                incident_domain.as_ref(),
                                analytic_family,
                                policy,
                            )? {
                                continue;
                            }
                            let cusp_parameter = if complementary {
                                CurveRegionParameter2::from_algebraic_cusp_complement(
                                    cusp_parameter,
                                )
                            } else {
                                CurveRegionParameter2::from_algebraic_cusp(cusp_parameter)
                            };
                            let analytic_parameter =
                                CurveRegionParameter2::from_bezier(analytic_parameter);
                            let (previous_parameter, next_parameter) = if cusp_is_previous {
                                (Some(cusp_parameter), Some(analytic_parameter))
                            } else {
                                (Some(analytic_parameter), Some(cusp_parameter))
                            };
                            centers.push(FilletCenterWitness2 {
                                point: contact.point_evidence(),
                                previous_parameter,
                                next_parameter,
                                // The retained frame is the analytic carrier,
                                // so store analytic x cusp rather than the
                                // direct kernel's cusp x analytic relation.
                                retained_anchor_evidence: Some(
                                    RetainedFilletAnchorEvidence2 {
                                        cross: Some(reverse_fillet_sign(cross)),
                                        dot: Some(dot),
                                        center_parallel: None,
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
                    crate::bezier_offset::BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiberOverlaps(overlaps) => {
                        for overlap in overlaps {
                            if retained_selected_fillet_overlap_is_positive(
                                &overlap,
                                &analytic_range,
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
                                || !parallel_source.parameter_is_admissible(
                                &contact.parallel_parameter,
                                !cusp_is_previous,
                                mode,
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
                                CurveRegionParameter2::from_algebraic_cusp_complement(
                                    cusp_parameter,
                                )
                            } else {
                                CurveRegionParameter2::from_algebraic_cusp(cusp_parameter)
                            };
                            let analytic_parameter = CurveRegionParameter2::from_bezier(
                                contact.parallel_parameter,
                            );
                            let (previous_parameter, next_parameter) = if cusp_is_previous {
                                (Some(cusp_parameter), Some(analytic_parameter))
                            } else {
                                (Some(analytic_parameter), Some(cusp_parameter))
                            };
                            centers.push(FilletCenterWitness2 {
                                point,
                                previous_parameter,
                                next_parameter,
                                retained_anchor_evidence: Some(
                                    RetainedFilletAnchorEvidence2 {
                                        cross: Some(reverse_fillet_sign(cross)),
                                        dot: Some(dot),
                                        center_parallel: None,
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
                            let Some(mapped_range) = retained_fillet_cusp_overlap_range(
                                cusp_source,
                                &overlap,
                                cusp_family,
                                policy,
                            )? else {
                                continue;
                            };
                            let overlaps_authored = retained_fillet_ranges_overlap(
                                &mapped_range,
                                &analytic_range,
                                analytic_family,
                                policy,
                            )?;
                            let overlaps_incident = if let Some(domain) = incident_domain.as_ref() {
                                retained_fillet_range_overlaps_incident_domain(
                                    &mapped_range,
                                    domain,
                                    analytic_family,
                                    policy,
                                )
                                ?
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
            let (arc, source_radius, signed_radius, cusp, arc_is_previous) = match (previous, next)
            {
                (
                    FilletOffsetCarrier2::Arc {
                        source,
                        source_radius,
                        signed_radius,
                    },
                    FilletOffsetCarrier2::AlgebraicCusp { support, .. },
                ) => ((*source), *source_radius, signed_radius, support, true),
                (
                    FilletOffsetCarrier2::AlgebraicCusp { support, .. },
                    FilletOffsetCarrier2::Arc {
                        source,
                        source_radius,
                        signed_radius,
                    },
                ) => ((*source), *source_radius, signed_radius, support, false),
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
            let offset_arc = arc.rational_offset_evaluator(
                source_radius,
                signed_radius,
                CurveOperation2::Fillet,
                arc_family,
                policy,
            )?;
            if mode == CurveCornerMode2::TrimOrExtend {
                let complementary_cusp = cusp.semicircle().complementary_half();
                let cusp_charts = [
                    (
                        crate::BezierAlgebraicCuspSemicircleFragment2::full(
                            cusp.semicircle().clone(),
                            policy,
                        ),
                        false,
                        true,
                        true,
                    ),
                    (
                        crate::BezierAlgebraicCuspSemicircleFragment2::full(
                            complementary_cusp,
                            policy,
                        ),
                        true,
                        false,
                        false,
                    ),
                ];
                let complement_spans = retained_arc_complement_projective_spans(
                    &offset_arc.support,
                    CurveOperation2::Fillet,
                    arc_family,
                    policy,
                )?;
                let mut arc_cells = Vec::with_capacity(complement_spans.len() + 1);
                arc_cells.push((offset_arc.offset.clone(), true, true, None));
                let complement_count = complement_spans.len();
                arc_cells.extend(
                    complement_spans
                        .into_iter()
                        .enumerate()
                        .map(|(index, span)| {
                            (
                                RationalBezier2::from(span),
                                false,
                                index + 1 != complement_count,
                                Some(index),
                            )
                        }),
                );
                for (cusp_chart, complementary, include_cusp_start, include_cusp_end) in
                    &cusp_charts
                {
                    for (arc_cell, include_arc_start, include_arc_end, complement_span) in
                        &arc_cells
                    {
                        let contacts = retained_fillet_cusp_rational_contacts(
                            cusp_chart,
                            arc_cell,
                            *include_cusp_start,
                            *include_cusp_end,
                            *include_arc_start,
                            *include_arc_end,
                            cusp_family,
                            policy,
                        )?;
                        if contacts.coincident {
                            centers.coincident = true;
                            return Ok(centers);
                        }
                        for contact in contacts.contacts {
                            let cusp_parameter = if *complementary {
                                CurveRegionParameter2::from_algebraic_cusp_complement(
                                    contact.cusp_parameter,
                                )
                            } else {
                                CurveRegionParameter2::from_algebraic_cusp(contact.cusp_parameter)
                            };
                            let (previous_parameter, next_parameter) = if arc_is_previous {
                                (None, Some(cusp_parameter))
                            } else {
                                (Some(cusp_parameter), None)
                            };
                            centers.push(FilletCenterWitness2 {
                                point: contact.point,
                                previous_parameter,
                                next_parameter,
                                retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                                    cross: None,
                                    dot: None,
                                    center_parallel: None,
                                    source_direction: None,
                                    canonical_anchor_curve: None,
                                    deferred_arc_contact: Some(RetainedDeferredArcFilletContact2 {
                                        support: arc.support().clone(),
                                        source_radius: source_radius.clone(),
                                        signed_center_radius: signed_radius.clone(),
                                        arc_is_previous,
                                        contact_seed: Some(RetainedArcFilletContactSeed2 {
                                            complement_span: *complement_span,
                                            parameter: contact.other_parameter,
                                        }),
                                    }),
                                }),
                            });
                        }
                    }
                }
                return Ok(centers);
            }
            let contacts = retained_fillet_cusp_rational_contacts(
                cusp,
                &offset_arc.offset,
                false,
                false,
                true,
                true,
                cusp_family,
                policy,
            )?;
            centers.coincident |= contacts.coincident;
            for contact in contacts.contacts {
                let arc_parameter = contact.other_parameter;
                let cusp_parameter =
                    CurveRegionParameter2::from_algebraic_cusp(contact.cusp_parameter);
                let (previous_parameter, next_parameter) = if arc_is_previous {
                    (Some(arc_parameter), Some(cusp_parameter))
                } else {
                    (Some(cusp_parameter), Some(arc_parameter))
                };
                centers.push(FilletCenterWitness2 {
                    point: contact.point,
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        cross: None,
                        dot: None,
                        center_parallel: None,
                        source_direction: None,
                        canonical_anchor_curve: Some(offset_arc.canonical_source.clone()),
                        deferred_arc_contact: None,
                    }),
                });
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
            // TrimOnly selects its finite domain; TrimOrExtend selects both
            // selected-circle charts and the complete affine support. Keeping
            // those policies in one cell prevents the old rational-line fast
            // path from reaching a different exactness decision. Final cut
            // publication still uses the original line carrier.
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
            // Keep canonical-Real line endpoints as the intersection hot
            // path while retaining the procedural normal-offset support as
            // ancestry. The former selects the compact selected-fiber line
            // kernel; the latter is the exact tangent relation needed when a
            // recursively selected terminal circle is reconstructed.
            let support_chord = retained_support
                .chord_between_certified_ordered_support_points(
                    RationalBezierIntersectionPointEvidence2::Exact(line_support.start().clone()),
                    RationalBezierIntersectionPointEvidence2::Exact(line_support.end().clone()),
                    policy,
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
                })?;
            let promoted = FilletOffsetCarrier2::AlgebraicChord {
                source: &source_chord,
                support: support_chord,
                signed_distance: line_signed_distance.clone(),
            };
            let mut promoted_centers = if line_is_previous {
                fillet_offset_centers(&promoted, next, mode, previous_family, next_family, policy)
            } else {
                fillet_offset_centers(
                    previous,
                    &promoted,
                    mode,
                    previous_family,
                    next_family,
                    policy,
                )
            }?;
            // An algebraic-chord source can transfer the transient offset
            // chord's certified order directly back to its authored support.
            // A native line instead retains the selected-fiber scalar used by
            // its public parameter domain. In both cases parallel translation
            // preserves the affine coordinate without a new point-equality
            // solve.
            for center in promoted_centers.iter_mut() {
                let line_parameter = if line_is_previous {
                    center.previous_parameter.as_ref()
                } else {
                    center.next_parameter.as_ref()
                };
                let selected_line_parameter = line_parameter
                    .and_then(CurveRegionParameter2::as_algebraic_chord)
                    .and_then(|parameter| parameter.point().as_algebraic_cusp_chord())
                    .and_then(|point| point.selected_fiber_line_parameter())
                    .map(CurveRegionParameter2::from_selected_fiber);
                let retained_line_parameter = selected_line_parameter.or_else(|| {
                    line_source
                        .algebraic_chord()
                        .and_then(|_| line_parameter.cloned())
                });
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
                           point: RationalBezierIntersectionPointEvidence2,
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
                    CurveRegionParameter2::from_algebraic_cusp_complement(previous_parameter)
                } else {
                    CurveRegionParameter2::from_algebraic_cusp(previous_parameter)
                };
                let next_parameter = if next_complementary {
                    CurveRegionParameter2::from_algebraic_cusp_complement(next_parameter)
                } else {
                    CurveRegionParameter2::from_algebraic_cusp(next_parameter)
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
            },
            FilletOffsetCarrier2::AlgebraicChord {
                source: next_source,
                support: next_support,
                signed_distance: next_distance,
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
            let (chord_support, cusp_source, cusp_support, chord_is_previous) =
                match (previous, next) {
                    (
                        FilletOffsetCarrier2::AlgebraicChord { support, .. },
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source,
                            support: cusp,
                        },
                    ) => (support, source, cusp, true),
                    (
                        FilletOffsetCarrier2::AlgebraicCusp {
                            source,
                            support: cusp,
                        },
                        FilletOffsetCarrier2::AlgebraicChord { support, .. },
                    ) => (support, source, cusp, false),
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
            let intersections = if mode == CurveCornerMode2::TrimOrExtend {
                cusp_support
                    .semicircle()
                    .chord_support_intersections(chord_support, policy)
            } else {
                cusp_support
                    .semicircle()
                    .chord_intersections(chord_support, policy)
            };
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
            let mut tangent_dot = match cusp_support
                .semicircle()
                .chord_tangent_dot_sign(chord_support, policy)
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
            for (contact, complementary) in contacts {
                let tangent_cross = if cusp_support_reverses_source {
                    reverse_fillet_sign(contact.tangent_cross_sign)
                } else {
                    contact.tangent_cross_sign
                };
                let cusp_parameter = if complementary {
                    CurveRegionParameter2::from_algebraic_cusp_complement(contact.cusp_parameter)
                } else {
                    CurveRegionParameter2::from_algebraic_cusp(contact.cusp_parameter)
                };
                let chord_parameter =
                    CurveRegionParameter2::from_algebraic_chord(contact.chord_parameter);
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
            let arc_corner_evidence =
                RationalBezierIntersectionPointEvidence2::Exact(arc_corner.clone());
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

            let offset_arc = arc.rational_offset_evaluator(
                source_radius,
                signed_radius,
                CurveOperation2::Fillet,
                arc_family,
                policy,
            )?;
            let center_parallel =
                offset_arc
                    .offset
                    .parallel_left(Real::zero())
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
                    })?;
            let canonical_chord;
            let (intersection_chord, chord_tangent_reversed) = if chord_support.is_reversed() {
                canonical_chord = chord_support.reversed();
                (&canonical_chord, true)
            } else {
                (chord_support, false)
            };
            let arc_is_previous = !chord_is_previous;
            let incident_domain = if mode == CurveCornerMode2::TrimOrExtend {
                let (endpoint, direction) = if arc_is_previous {
                    (Real::one(), crate::BezierParameterRayDirection2::Increasing)
                } else {
                    (
                        Real::zero(),
                        crate::BezierParameterRayDirection2::Decreasing,
                    )
                };
                let endpoint = BezierParameter2::Exact(endpoint);
                Some(
                    match center_parallel
                        .incident_domain_from_parameter(&endpoint, direction, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, arc_family, cause)
                        })? {
                        Classification::Decided(domain) => domain,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                arc_family,
                                reason,
                            ));
                        }
                    },
                )
            } else {
                None
            };
            let intersection_result = if let Some(domain) = incident_domain.as_ref() {
                intersection_chord.parallel_intersections_with_incident_ray(
                    &center_parallel,
                    domain,
                    policy,
                )
            } else {
                intersection_chord.parallel_intersections(&center_parallel, policy)
            };
            let contacts = match intersection_result.map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                })? {
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::Contacts(
                        contacts,
                    ),
                ) => contacts,
                Classification::Decided(
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
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
            for contact in contacts {
                let mut cross = contact.tangent_cross_sign();
                let mut dot = contact.tangent_dot_sign();
                if chord_tangent_reversed {
                    cross = reverse_fillet_sign(cross);
                    dot = reverse_fillet_sign(dot);
                }
                if signed_radius_sign == RealSign::Negative {
                    cross = reverse_fillet_sign(cross);
                    dot = reverse_fillet_sign(dot);
                }
                let arc_parameter =
                    CurveRegionParameter2::from_bezier(contact.parallel_parameter().clone());
                let (previous_parameter, next_parameter) = if chord_is_previous {
                    (None, Some(arc_parameter))
                } else {
                    (Some(arc_parameter), None)
                };
                centers.push(FilletCenterWitness2 {
                    point: contact.point().clone(),
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        // The retained concentric arc supplies the fillet
                        // frame. The intersection kernel reports chord x arc.
                        cross: Some(reverse_fillet_sign(cross)),
                        dot: Some(dot),
                        center_parallel: Some(RetainedFilletCenterParallel2 {
                            support: center_parallel.clone(),
                            parameter: None,
                        }),
                        source_direction: None,
                        canonical_anchor_curve: Some(offset_arc.canonical_source.clone()),
                        deferred_arc_contact: None,
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
                    crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
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
                if !parallel_source.parameter_is_admissible(
                    contact.parallel_parameter(),
                    analytic_is_previous,
                    mode,
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
                    CurveRegionParameter2::from_bezier(contact.parallel_parameter().clone());
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
            if mode == CurveCornerMode2::TrimOrExtend {
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
                        // Extension reconstruction uses the algebraic chord's
                        // general center frame, so retain anchor chord x line.
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
            let chord_family = if chord_is_previous {
                previous_family
            } else {
                next_family
            };
            let line_family = if chord_is_previous {
                next_family
            } else {
                previous_family
            };
            let line = RationalBezier2::try_new_with_exact_line_image(
                vec![line_support.start().clone(), line_support.end().clone()],
                vec![Real::one(), Real::one()],
                line_support.clone(),
            )
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
            })?;
            let center_parallel = line.parallel_left(Real::zero()).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, line_family, cause)
            })?;
            let intersections = match chord_support
                .parallel_intersections(&center_parallel, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, chord_family, cause)
                })? {
                Classification::Decided(intersections) => intersections,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        chord_family,
                        reason,
                    ));
                }
            };
            let contacts = match intersections {
                crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::Contacts(
                    contacts,
                ) => contacts,
                crate::bezier_offset::BezierAlgebraicChordParallelIntersections2::DegenerateProjection => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        chord_family,
                        crate::UncertaintyReason::Boundary,
                    ));
                }
            };
            for contact in contacts {
                let line_parameter =
                    CurveRegionParameter2::from_bezier(contact.parallel_parameter().clone());
                let (previous_parameter, next_parameter) = if chord_is_previous {
                    (None, Some(line_parameter))
                } else {
                    (Some(line_parameter), None)
                };
                centers.push(FilletCenterWitness2 {
                    point: contact.point().clone(),
                    previous_parameter,
                    next_parameter,
                    retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                        // The represented line supplies the circle frame, so
                        // retain line x chord rather than the kernel's chord x
                        // line relation.
                        cross: Some(reverse_fillet_sign(contact.tangent_cross_sign())),
                        dot: Some(contact.tangent_dot_sign()),
                        center_parallel: Some(RetainedFilletCenterParallel2 {
                            support: center_parallel.clone(),
                            parameter: None,
                        }),
                        source_direction: None,
                        canonical_anchor_curve: None,
                        deferred_arc_contact: None,
                    }),
                });
            }
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
            if previous_source.algebraic_chord().is_none()
                && next_source.algebraic_chord().is_none()
            {
                unreachable!("native line/line fillets use the specialized exact fast path");
            }
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
            let retained_parameter =
                |source: &FilletLinearSource2<'_>,
                 support: &LineSeg2,
                 family: CurveFamily2|
                 -> ExactCurveResult<Option<CurveRegionParameter2>> {
                    source
                        .native_line()
                        .map(|_| {
                            line_parameter_at_point(
                                support,
                                &point,
                                CurveOperation2::Fillet,
                                family,
                            )
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
    point: &RationalBezierIntersectionPointEvidence2,
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
            if let Some(point) = point.as_exact() {
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
                RationalBezierIntersectionPointEvidence2::Exact(support.start().clone()),
                RationalBezierIntersectionPointEvidence2::Exact(support.end().clone()),
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
    center: &RationalBezierIntersectionPointEvidence2,
    retained_parameter: Option<&CurveRegionParameter2>,
    deferred_arc_contact: bool,
    previous: bool,
    mode: CurveCornerMode2,
    retain_selected_circle_endpoints: bool,
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
            let Some(placement) = placement else {
                return Ok(None);
            };
            let point = if let Some(parameter) = parameter.as_exact() {
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
            let Some(center) = center.as_exact() else {
                if retained_parameter.is_none() {
                    // Direct arc/Bezier incidence retains the center in the
                    // Bezier parallel's selected normal field. The retained
                    // CurveRegion reconstruction intersects the resulting
                    // exact fillet circle with the authored arc (or its
                    // complement) and replaces this transient corner marker
                    // with the paired circular cut parameter and point.
                    return Ok(Some(CornerCut2 {
                        point: center.clone(),
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
                    let radial_scale = (*source_radius / signed_radius).map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause.into())
                    })?;
                    let point = match crate::BezierAlgebraicChord2::scaled_about_point_endpoint(
                        center,
                        source.support().center(),
                        &radial_scale,
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
                family,
                policy,
            )
        }
        FilletOffsetCarrier2::Point { .. } => {
            unreachable!("a collapsed arc offset has no isolated tangency contact")
        }
        FilletOffsetCarrier2::Parallel { source, support } => {
            let parameter = retained_parameter
                .expect("a parallel offset intersection retains its source parameter")
                .as_bezier_parameter()
                .cloned()
                .ok_or_else(|| {
                    ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        crate::UncertaintyReason::Unsupported,
                    )
                })?;
            match source {
                FilletParallelSource2::Direct(source) => {
                    let Some(placement) = bezier_corner_parameter_placement(
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
                    let point = bezier_parallel_source_point_evidence(
                        support,
                        &parameter,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?;
                    let parameter = match parameter {
                        BezierParameter2::Exact(parameter) => {
                            exact_corner_parameter(source.public_parameter(&parameter))
                        }
                        parameter @ BezierParameter2::Algebraic(_) => {
                            Some(CurveRegionParameter2::from_bezier(parameter))
                        }
                    };
                    Ok(Some(CornerCut2 {
                        point,
                        parameter,
                        placement,
                    }))
                }
                FilletParallelSource2::Retained(source) => {
                    let Some(placement) = retained_parallel_corner_parameter_placement(
                        &parameter,
                        source,
                        previous,
                        mode,
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?
                    else {
                        return Ok(None);
                    };
                    Ok(Some(CornerCut2 {
                        point: analytic_parallel_point_evidence(
                            source.parallel(),
                            &parameter,
                            CurveOperation2::Fillet,
                            family,
                            policy,
                        )?,
                        parameter: Some(CurveRegionParameter2::from_bezier(parameter)),
                        placement,
                    }))
                }
            }
        }
        FilletOffsetCarrier2::AlgebraicChord {
            source,
            signed_distance,
            ..
        } => {
            let point = source
                .normal_displaced_point_evidence(center.clone(), -signed_distance.clone(), policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })?;
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
                    Some(Classification::Uncertain(_)) | None => support
                        .certified_incident_point_evidence_is_strict_interior(center, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                        })?,
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
            } else {
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
                                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
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
                    Classification::Uncertain(reason) if retain_selected_circle_endpoints => {
                        use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::{
                            End, Exterior, Interior, Start,
                        };
                        match support
                            .certified_incident_point_evidence_location(&parameter, center, policy)
                            .map_err(|cause| {
                                ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                            })? {
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
                                    reason,
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
                center.as_exact(),
                support_semicircle
                    .exact_rational_center(policy)
                    .map_err(|cause| {
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
                RationalBezierIntersectionPointEvidence2::Exact(
                    support_center.translated(&radial.0 * &radial_scale, &radial.1 * radial_scale),
                )
            } else {
                match parameter
                    .concentric_offset_point_evidence(support_semicircle, source_semicircle, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })? {
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
                    CurveRegionParameter2::from_algebraic_cusp_complement(parameter)
                } else {
                    CurveRegionParameter2::from_algebraic_cusp(parameter)
                }),
                placement,
            }))
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn algebraic_chord_fillet_cut_from_center(
    source: &crate::BezierAlgebraicChord2,
    center: &RationalBezierIntersectionPointEvidence2,
    retained_parameter: Option<&CurveRegionParameter2>,
    signed_distance: &Real,
    previous: bool,
    mode: CurveCornerMode2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    let point = source
        .normal_displaced_point_evidence(center.clone(), -signed_distance.clone(), policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    if let Some(retained_parameter) = retained_parameter
        && retained_parameter.is_selected_fiber()
    {
        let Some(placement) = curve_region_corner_parameter_placement(
            retained_parameter,
            previous,
            mode,
            CurveOperation2::Fillet,
            family,
            policy,
        )?
        else {
            return Ok(None);
        };
        let parameter = source
            .parameter_at_certified_support_point(point.clone(), policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
        return Ok(Some(CornerCut2 {
            point,
            parameter: Some(CurveRegionParameter2::from_algebraic_chord(parameter)),
            placement,
        }));
    }
    if let Some(retained_parameter) =
        retained_parameter.and_then(CurveRegionParameter2::as_algebraic_chord)
    {
        return algebraic_chord_corner_cut_from_parallel_parameter(
            source,
            point,
            retained_parameter,
            previous,
            mode,
            CurveOperation2::Fillet,
            family,
            policy,
        );
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

#[allow(clippy::too_many_arguments)]
fn algebraic_chord_corner_cut_from_parallel_parameter(
    source: &crate::BezierAlgebraicChord2,
    point: RationalBezierIntersectionPointEvidence2,
    retained_parameter: &crate::bezier_offset::BezierAlgebraicChordParameter2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    let retained_chord = retained_parameter.chord();
    let reversed = retained_chord
        .retained_normal_offset_tangent_reversal_to(source)
        .ok_or_else(|| {
            ExactCurveError::invalid(
                operation,
                family,
                CurveError::Topology(
                    "a retained fillet contact did not descend from its source chord".into(),
                ),
            )
        })?;
    let compare = |boundary| {
        retained_parameter
            .cmp_by_refinement(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|order| match order {
                Classification::Decided(order) => Ok(order),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    let start = retained_chord.start_parameter();
    let end = retained_chord.end_parameter();
    let start_order = compare(&start)?;
    let end_order = compare(&end)?;
    let extends_toward_higher_parameter = previous != reversed;
    let placement = if start_order.is_gt() && end_order.is_lt() {
        CornerPlacement2::Trim
    } else if mode == CurveCornerMode2::TrimOrExtend
        && ((extends_toward_higher_parameter && end_order.is_gt())
            || (!extends_toward_higher_parameter && start_order.is_lt()))
    {
        CornerPlacement2::Extension
    } else {
        return Ok(None);
    };
    let parameter = source
        .parameter_at_certified_support_point(point.clone(), policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
    Ok(Some(CornerCut2 {
        point,
        parameter: Some(CurveRegionParameter2::from_algebraic_chord(parameter)),
        placement,
    }))
}

#[allow(clippy::too_many_arguments)]
fn algebraic_chord_corner_cut_from_support_point(
    source: &crate::BezierAlgebraicChord2,
    point: RationalBezierIntersectionPointEvidence2,
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
        parameter: Some(CurveRegionParameter2::from_algebraic_chord(parameter)),
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

fn algebraic_chord_from_line_support(
    line: &LineSeg2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<crate::BezierAlgebraicChord2> {
    match crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
        RationalBezierIntersectionPointEvidence2::Exact(line.start().clone()),
        RationalBezierIntersectionPointEvidence2::Exact(line.end().clone()),
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
        let previous_parameter = (previous_numerator / &denominator).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Fillet, previous_family, cause.into())
        })?;
        let next_parameter = (next_numerator / &denominator).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Fillet, next_family, cause.into())
        })?;
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
    parameter: &CurveRegionParameter2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let zero = CurveRegionParameter2::from_bezier(BezierParameter2::Exact(Real::zero()));
    let one = CurveRegionParameter2::from_bezier(BezierParameter2::Exact(Real::one()));
    let compare = |boundary: &CurveRegionParameter2| {
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
) -> ExactCurveResult<RationalBezierIntersectionPointEvidence2> {
    if let Some(parameter) = parameter.as_exact() {
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
        Ok(RationalBezierIntersectionPointEvidence2::AnalyticParallel(
            crate::BezierAnalyticParallelPoint2::new(
                parallel.with_distance(Real::zero()),
                parameter.clone(),
                policy,
            ),
        ))
    }
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
                parameter: Some(CurveRegionParameter2::from_algebraic_cusp(corner_parameter)),
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
            CurveRegionParameter2::from_algebraic_cusp_complement(parameter)
        } else {
            CurveRegionParameter2::from_algebraic_cusp(parameter)
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
                parameter: Some(CurveRegionParameter2::from_algebraic_chord(
                    corner_parameter,
                )),
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
    parameter: &BezierParameter2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<RationalBezierIntersectionPointEvidence2> {
    if let Some(parameter) = parameter.as_exact() {
        return decided_parallel_point(parallel, parameter, false, operation, family, policy)
            .map(Into::into);
    }
    Ok(RationalBezierIntersectionPointEvidence2::AnalyticParallel(
        crate::BezierAnalyticParallelPoint2::new(parallel.clone(), parameter.clone(), policy),
    ))
}

fn retained_parallel_corner_parameter_placement(
    parameter: &BezierParameter2,
    fragment: &crate::BezierParallelFragment2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let compare = |boundary: &BezierParameter2| {
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

fn retained_parallel_selected_corner_parameter_placement(
    parameter: &crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2,
    fragment: &crate::BezierParallelFragment2,
    previous: bool,
    mode: CurveCornerMode2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerPlacement2>> {
    let compare = |boundary: &BezierParameter2| {
        parameter
            .cmp_bezier_parameter(boundary, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))
            .and_then(|ordering| match ordering {
                Classification::Decided(ordering) => Ok(ordering),
                Classification::Uncertain(reason) => {
                    Err(ExactCurveError::blocked(operation, family, reason))
                }
            })
    };
    Ok(retained_parallel_corner_orders_placement(
        compare(fragment.range().start())?,
        compare(fragment.range().end())?,
        fragment,
        previous,
        mode,
    ))
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
        corner_parameter,
        operation,
        family,
        policy,
    )?;
    if setback_sign == RealSign::Zero {
        return Ok(CornerCuts2 {
            first: Some(CornerCut2 {
                parameter: Some(CurveRegionParameter2::from_bezier(corner_parameter.clone())),
                point: corner,
                placement: CornerPlacement2::Corner,
            }),
            second: None,
            overflow: Vec::new(),
        });
    }
    let radius_squared = setback * setback;
    let direction = if previous != fragment.is_reversed() {
        crate::BezierParameterRayDirection2::Increasing
    } else {
        crate::BezierParameterRayDirection2::Decreasing
    };
    let parameters = match (if mode == CurveCornerMode2::TrimOrExtend {
        fragment
            .parallel()
            .fixed_distance_incidence_from_parameter_with_incident_ray(
                corner_parameter,
                &radius_squared,
                fragment.range(),
                direction,
                policy,
            )
    } else {
        fragment.parallel().fixed_distance_incidence_from_parameter(
            corner_parameter,
            &radius_squared,
            fragment.range(),
            policy,
        )
    })
    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(operation, family, reason));
        }
    };
    let mut cuts = CornerCuts2::default();
    for parameter in parameters {
        let (parameter, point, placement) = match parameter {
            crate::bezier_offset::BezierParallelFixedDistanceParameter2::Bezier(parameter) => {
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
                (
                    CurveRegionParameter2::from_bezier(parameter),
                    point,
                    placement,
                )
            }
            crate::bezier_offset::BezierParallelFixedDistanceParameter2::SelectedFiber(
                parameter,
            ) => {
                let Some(placement) = retained_parallel_selected_corner_parameter_placement(
                    &parameter, fragment, previous, mode, operation, family, policy,
                )?
                else {
                    continue;
                };
                let point = RationalBezierIntersectionPointEvidence2::AnalyticParallel(
                    crate::BezierAnalyticParallelPoint2::new_selected_fiber(
                        fragment.parallel().clone(),
                        parameter.clone(),
                        policy,
                    ),
                );
                (
                    CurveRegionParameter2::from_selected_fiber(parameter),
                    point,
                    placement,
                )
            }
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
                parameter: exact_corner_parameter(if previous {
                    source.public_parameter(&Real::one())
                } else {
                    source.public_parameter(&Real::zero())
                }),
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
        let parameter = match parameter {
            BezierParameter2::Exact(parameter) => {
                exact_corner_parameter(source.public_parameter(&parameter))
            }
            parameter @ BezierParameter2::Algebraic(_) => {
                Some(CurveRegionParameter2::from_bezier(parameter))
            }
        };
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
                        Some(parameter) => exact_corner_parameter(parameter),
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

fn arc_fillet_cut_from_incident_point(
    arc: &ExactCornerArc2<'_>,
    point: Point2,
    deferred_arc_contact: bool,
    previous: bool,
    mode: CurveCornerMode2,
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
                    .and_then(exact_corner_parameter)
            };
            Ok(Some(CornerCut2 {
                parameter,
                point: point.into(),
                placement: CornerPlacement2::Trim,
            }))
        }
        Classification::Decided(ArcSweepPointLocation2::Endpoint) => Ok(None),
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
            CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_)
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
    let (start, end) = if let CurveGeometry2::CircularArc(arc) = curve.geometry() {
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
    let sweep_fraction = if let Some(parameter) = cut.exact_parameter() {
        parameter.clone()
    } else {
        let point = cut.exact_point().ok_or_else(|| {
            ExactCurveError::blocked(
                operation,
                curve.family(),
                crate::UncertaintyReason::Unsupported,
            )
        })?;
        match arc
            .sweep_fraction_for_incident_point(point, policy)
            .map_err(|cause| ExactCurveError::invalid(operation, curve.family(), cause))?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(operation, curve.family(), reason));
            }
        }
    };
    match arc
        .parameter_at_sweep_fraction(&sweep_fraction, policy)
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
            CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_)
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
        CurveGeometry2::QuadraticBezier(source) => BezierSubcurve2::Quadratic(source.clone()),
        CurveGeometry2::CubicBezier(source) => BezierSubcurve2::Cubic(source.clone()),
        CurveGeometry2::RationalQuadraticBezier(source) => {
            BezierSubcurve2::RationalQuadratic(source.clone())
        }
        CurveGeometry2::RationalBezier(source) => BezierSubcurve2::Rational(source.clone()),
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
            if let CurveGeometry2::CircularArc(arc) = curve.geometry() {
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
                let domain = curve.parameter_domain();
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
                curve
                    .with_lineage(CurveGeometry2::CircularArc(trimmed), lineage)
                    .map_err(|error| remap_operation(error, operation))
            } else {
                let parameter = cut.exact_parameter().ok_or_else(|| {
                    ExactCurveError::blocked(
                        operation,
                        curve.family(),
                        crate::UncertaintyReason::Unsupported,
                    )
                })?;
                let domain = curve.parameter_domain();
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
                    CurveGeometry2::QuadraticBezier(source)
                        if source.retained_exact_line_image().is_some() =>
                    {
                        Curve2::from(QuadraticBezier2::from_line_segment(extended))
                    }
                    _ => Curve2::from(extended),
                })
            } else if let CurveGeometry2::CircularArc(arc) = curve.geometry() {
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
                let domain = curve.parameter_domain();
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
            assert_eq!(body.start(), &next_point);
            assert_eq!(body.end(), &previous_point);
            let CurveGeometry2::CubicBezier(body) = body.geometry() else {
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
            assert_eq!(body.start(), &next_point);
            assert_eq!(body.end(), &previous_point);
            let CurveGeometry2::CubicBezier(body) = body.geometry() else {
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
            let translated = |point: &RationalBezierIntersectionPointEvidence2, x, y| {
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
                RationalBezierIntersectionPointEvidence2::Exact(arc_end),
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
            assert_eq!(extended.start(), &Point2::from_values(0, 0));
            assert_eq!(extended.end(), &Point2::from_values(3, 9));
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
        let parameter = CurveRegionParameter2::from_bezier(center_parameter.clone());
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
        let center = RationalBezierIntersectionPointEvidence2::Algebraic(
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
            CurveGeometry2::QuadraticBezier(source) => {
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
        assert_eq!(core::mem::size_of::<CurvePathData2>(), 160);
        assert_eq!(core::mem::size_of::<NativeBezierBoundaryLoop2>(), 24);
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
    fn boundary_cache_revalidates_approximate_internal_path_joins() {
        let left_x = Real::pi() + Real::e();
        let right_x = Real::e() + Real::pi();
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
            .bezier_boundary_loop(&CurveContext::APPROXIMATE_512)
            .expect("the terminal policy must validate every path join");
        assert_eq!(
            boundary.certainty,
            crate::CurveCertainty::Approximate512Consumed
        );
        assert_eq!(boundary.value.len(), 4);

        let strict_boundary = path
            .bezier_boundary_loop(&CurveContext::STRICT)
            .unwrap_err();
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
