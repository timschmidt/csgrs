//! Top-level owned and borrowed exact curve carriers.

use std::cell::OnceCell;
use std::rc::Rc;

use hyperreal::{RealSign, ZeroKnowledge};

use crate::arc_bezier::decompose_circular_arc;
use crate::{
    Aabb2, BezierBoundaryLoop2, BezierSubcurve2, CircularArc2, Classification,
    ContourPointLocation, CubicBezier2, CurveError, CurveOperation2, CurvePolicy, ExactCurveError,
    ExactCurveResult, LineSeg2, LineSide, NurbsCurve2, ParamRange, Point2, PolynomialSplineCurve2,
    QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2, Real, Similarity2,
};

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

/// Stable application-owned source identity for an exact curve.
///
/// Hypercurve treats `id` as opaque. Importers can use it for a STEP entity,
/// DXF handle, database row, or another stable source key. `version` makes
/// retained calculations auditable after source edits.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct CurveSource2 {
    id: u64,
    version: u64,
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
    source: Option<CurveSource2>,
    lineage: CurveParameterLineage2,
    parameter_domain: OnceCell<CurveParameterDomain2>,
    native_bezier_fragments: OnceCell<ExactCurveResult<Vec<NativeBezierFragment2>>>,
    rational_evaluators: OnceCell<ExactCurveResult<Vec<RationalBezier2>>>,
    bounds: OnceCell<ExactCurveResult<Aabb2>>,
}

#[derive(Clone, Debug)]
struct CurveParameterLineage2 {
    root: Rc<CurveParameterLineageRoot2>,
    range: ParamRange,
}

#[derive(Debug)]
struct CurveParameterLineageRoot2 {
    domain: ParamRange,
    image_is_injective: OnceCell<bool>,
}

impl CurveParameterLineage2 {
    fn new(range: ParamRange) -> Self {
        Self {
            root: Rc::new(CurveParameterLineageRoot2 {
                domain: range.clone(),
                image_is_injective: OnceCell::new(),
            }),
            range,
        }
    }

    fn reversed(&self) -> Self {
        Self {
            root: Rc::clone(&self.root),
            range: ParamRange::new(self.range.end().clone(), self.range.start().clone()),
        }
    }
}

/// Immutable top-level exact planar curve.
///
/// Clones share the exact carrier and its retained calculations. Use
/// [`Curve2::as_view`] for borrowed algorithms and [`Curve2::source`] for
/// stable application provenance.
#[derive(Clone, Debug)]
pub struct Curve2 {
    data: Rc<CurveData2>,
}

/// Borrowed view of one top-level exact curve.
#[derive(Clone, Copy, Debug)]
pub struct CurveView2<'a> {
    curve: &'a Curve2,
}

/// Ordered connected sequence of exact curves.
#[derive(Clone, Debug)]
pub struct CurvePath2 {
    data: Rc<CurvePathData2>,
}

#[derive(Debug)]
struct CurvePathData2 {
    curves: Vec<Curve2>,
    native_bezier_fragments: OnceCell<ExactCurveResult<Vec<NativeBezierFragment2>>>,
    bezier_boundary_loop: OnceCell<ExactCurveResult<NativeBezierBoundaryLoop2>>,
    bounds: OnceCell<ExactCurveResult<Aabb2>>,
}

/// Borrowed view of an ordered exact curve path.
#[derive(Clone, Copy, Debug)]
pub struct CurvePathView2<'a> {
    curves: &'a [Curve2],
}

/// Source identity and parameter range for one promoted Bezier fragment.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveSpanProvenance2 {
    source: Option<CurveSource2>,
    source_span_index: Option<usize>,
    parameter_start: Real,
    parameter_end: Real,
    source_parameter_start: Real,
    source_parameter_end: Real,
}

/// Exact native Bezier/conic fragment with retained source provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct NativeBezierFragment2 {
    curve: BezierSubcurve2,
    provenance: CurveSpanProvenance2,
}

/// Validated native Bezier boundary with one-to-one source provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct NativeBezierBoundaryLoop2 {
    boundary_loop: BezierBoundaryLoop2,
    fragments: Vec<NativeBezierFragment2>,
}

impl CurveSource2 {
    /// Constructs source identity at version zero.
    pub const fn new(id: u64) -> Self {
        Self::with_version(id, 0)
    }

    /// Constructs source identity with an explicit version.
    pub const fn with_version(id: u64, version: u64) -> Self {
        Self { id, version }
    }

    /// Returns the opaque source id.
    pub const fn id(self) -> u64 {
        self.id
    }

    /// Returns the source version used by retained calculations.
    pub const fn version(self) -> u64 {
        self.version
    }
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

    fn retained_source(&self) -> Option<CurveSource2> {
        match self {
            Self::PolynomialBSpline(curve) => curve.source(),
            Self::Nurbs(curve) => curve.source(),
            _ => None,
        }
    }
}

impl Curve2 {
    /// Wraps exact geometry without application source provenance.
    pub fn new(geometry: CurveGeometry2) -> Self {
        let source = geometry.retained_source();
        let lineage = CurveParameterLineage2::new(geometry_parameter_range(&geometry));
        Self {
            data: Rc::new(CurveData2 {
                geometry,
                source,
                lineage,
                parameter_domain: OnceCell::new(),
                native_bezier_fragments: OnceCell::new(),
                rational_evaluators: OnceCell::new(),
                bounds: OnceCell::new(),
            }),
        }
    }

    /// Wraps exact geometry with stable application source provenance.
    pub fn with_source(geometry: CurveGeometry2, source: CurveSource2) -> ExactCurveResult<Self> {
        if geometry
            .retained_source()
            .is_some_and(|retained| retained != source)
        {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                geometry.family(),
                Some(source),
                CurveError::ConflictingCurveSource,
            ));
        }
        let lineage = CurveParameterLineage2::new(geometry_parameter_range(&geometry));
        Ok(Self {
            data: Rc::new(CurveData2 {
                geometry,
                source: Some(source),
                lineage,
                parameter_domain: OnceCell::new(),
                native_bezier_fragments: OnceCell::new(),
                rational_evaluators: OnceCell::new(),
                bounds: OnceCell::new(),
            }),
        })
    }

    /// Constructs a policy-free exact polynomial B-spline carrier.
    pub fn try_polynomial_bspline(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        source: Option<CurveSource2>,
    ) -> ExactCurveResult<Self> {
        let curve = match source {
            Some(source) => {
                PolynomialSplineCurve2::try_new_with_source(degree, control_points, knots, source)?
            }
            None => PolynomialSplineCurve2::try_new(degree, control_points, knots)?,
        };
        Ok(Self::new(CurveGeometry2::PolynomialBSpline(curve)))
    }

    /// Constructs a policy-free exact NURBS carrier.
    pub fn try_nurbs(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        source: Option<CurveSource2>,
    ) -> ExactCurveResult<Self> {
        let curve = match source {
            Some(source) => {
                NurbsCurve2::try_new_with_source(degree, control_points, weights, knots, source)?
            }
            None => NurbsCurve2::try_new(degree, control_points, weights, knots)?,
        };
        Ok(Self::new(CurveGeometry2::Nurbs(curve)))
    }

    /// Constructs a policy-free periodic polynomial B-spline from one period.
    pub fn try_periodic_polynomial_bspline(
        degree: usize,
        control_points: Vec<Point2>,
        period_knots: Vec<Real>,
        source: Option<CurveSource2>,
    ) -> ExactCurveResult<Self> {
        let curve = match source {
            Some(source) => PolynomialSplineCurve2::try_new_periodic_with_source(
                degree,
                control_points,
                period_knots,
                source,
            )?,
            None => PolynomialSplineCurve2::try_new_periodic(degree, control_points, period_knots)?,
        };
        Ok(Self::new(CurveGeometry2::PolynomialBSpline(curve)))
    }

    /// Constructs a policy-free periodic NURBS from one period.
    pub fn try_periodic_nurbs(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        period_knots: Vec<Real>,
        source: Option<CurveSource2>,
    ) -> ExactCurveResult<Self> {
        let curve = match source {
            Some(source) => NurbsCurve2::try_new_periodic_with_source(
                degree,
                control_points,
                weights,
                period_knots,
                source,
            )?,
            None => NurbsCurve2::try_new_periodic(degree, control_points, weights, period_knots)?,
        };
        Ok(Self::new(CurveGeometry2::Nurbs(curve)))
    }

    /// Returns a borrowed view without cloning geometry or provenance.
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

    /// Returns stable source identity when retained.
    pub fn source(&self) -> Option<CurveSource2> {
        self.data.source
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
    /// Stable source provenance and the public parameter domain are retained.
    /// Parameters map as `u -> start + end - u`.
    pub fn reversed(&self) -> ExactCurveResult<Self> {
        let geometry = match self.geometry() {
            CurveGeometry2::Line(curve) => CurveGeometry2::Line(curve.reversed()),
            CurveGeometry2::CircularArc(curve) => CurveGeometry2::CircularArc(curve.reversed()),
            CurveGeometry2::QuadraticBezier(curve) => {
                CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(
                    curve.end().clone(),
                    curve.control().clone(),
                    curve.start().clone(),
                ))
            }
            CurveGeometry2::CubicBezier(curve) => CurveGeometry2::CubicBezier(CubicBezier2::new(
                curve.end().clone(),
                curve.control2().clone(),
                curve.control1().clone(),
                curve.start().clone(),
            )),
            CurveGeometry2::RationalQuadraticBezier(curve) => {
                CurveGeometry2::RationalQuadraticBezier(
                    RationalQuadraticBezier2::try_new(
                        curve.end().clone(),
                        curve.control().clone(),
                        curve.start().clone(),
                        curve.end_weight().clone(),
                        curve.control_weight().clone(),
                        curve.start_weight().clone(),
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(
                            CurveOperation2::Reversal,
                            CurveFamily2::RationalQuadraticBezier,
                            self.source(),
                            cause,
                        )
                    })?,
                )
            }
            CurveGeometry2::RationalBezier(curve) => {
                CurveGeometry2::RationalBezier(curve.reversed())
            }
            CurveGeometry2::PolynomialBSpline(curve) => {
                CurveGeometry2::PolynomialBSpline(curve.reversed()?)
            }
            CurveGeometry2::Nurbs(curve) => CurveGeometry2::Nurbs(curve.reversed()?),
        };
        self.with_lineage(geometry, self.data.lineage.reversed())
    }

    /// Applies an exact planar similarity while preserving curve family and source.
    pub fn transform_similarity(&self, transform: &Similarity2) -> ExactCurveResult<Self> {
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
            CurveGeometry2::QuadraticBezier(curve) => {
                let points = curve
                    .control_points()
                    .map(|point| transform.transform_point(point));
                CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(
                    points[0].clone(),
                    points[1].clone(),
                    points[2].clone(),
                ))
            }
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
            CurveGeometry2::PolynomialBSpline(curve) => {
                CurveGeometry2::PolynomialBSpline(curve.transform_similarity(transform)?)
            }
            CurveGeometry2::Nurbs(curve) => {
                CurveGeometry2::Nurbs(curve.transform_similarity(transform)?)
            }
        };
        self.with_lineage(geometry, self.data.lineage.clone())
    }

    /// Splits this curve exactly at a strict interior public parameter.
    ///
    /// Native result curves use their usual `[0, 1]` parameter domain. Spline
    /// results retain the two corresponding authored knot-domain intervals.
    /// Curve family and stable source provenance are preserved.
    pub fn split_at(&self, parameter: Real) -> ExactCurveResult<(Self, Self)> {
        let domain = self.parameter_domain();
        validate_strict_split_parameter(
            domain.start(),
            &parameter,
            domain.end(),
            self.family(),
            self.source(),
        )?;
        match self.geometry() {
            CurveGeometry2::PolynomialBSpline(curve) => {
                let (left, right) = curve.split_at(parameter.clone())?;
                let left_lineage = self.lineage_subrange(domain.start(), &parameter)?;
                let right_lineage = self.lineage_subrange(&parameter, domain.end())?;
                Ok((
                    self.with_lineage(CurveGeometry2::PolynomialBSpline(left), left_lineage)?,
                    self.with_lineage(CurveGeometry2::PolynomialBSpline(right), right_lineage)?,
                ))
            }
            CurveGeometry2::Nurbs(curve) => {
                let (left, right) = curve.split_at(parameter.clone())?;
                let left_lineage = self.lineage_subrange(domain.start(), &parameter)?;
                let right_lineage = self.lineage_subrange(&parameter, domain.end())?;
                Ok((
                    self.with_lineage(CurveGeometry2::Nurbs(left), left_lineage)?,
                    self.with_lineage(CurveGeometry2::Nurbs(right), right_lineage)?,
                ))
            }
            _ => Ok((
                self.subcurve(domain.start().clone(), parameter.clone())?,
                self.subcurve(parameter, domain.end().clone())?,
            )),
        }
    }

    /// Returns the exact curve image over a strictly ordered public range.
    ///
    /// A full-domain request returns a clone sharing retained facts. Native
    /// result curves are reparameterized to `[0, 1]`; spline results retain the
    /// requested authored knot range. Curve family and source are preserved.
    pub fn subcurve(&self, start: Real, end: Real) -> ExactCurveResult<Self> {
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
            self.source(),
        )?;
        let policy = CurvePolicy::certified();
        if crate::classify::compare_reals(&start, domain.start(), &policy)
            == Some(std::cmp::Ordering::Equal)
            && crate::classify::compare_reals(&end, domain.end(), &policy)
                == Some(std::cmp::Ordering::Equal)
        {
            return Ok(self.clone());
        }
        self.retain_root_image_injectivity(&policy);
        let lineage = self.lineage_subrange(&start, &end)?;
        let geometry = match self.geometry() {
            CurveGeometry2::Line(curve) => CurveGeometry2::Line(
                LineSeg2::try_new(curve.point_at(start), curve.point_at(end))
                    .map_err(|cause| self.subdivision_error(cause))?,
            ),
            CurveGeometry2::CircularArc(curve) => {
                let sub_start = self
                    .point_at(&start)
                    .map_err(|error| remap_operation(error, CurveOperation2::Subdivision))?;
                let sub_end = self
                    .point_at(&end)
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
                    .subcurve_between_exact(&start, &end, &policy)
                    .map_err(|cause| self.subdivision_error(cause))?,
            ),
            CurveGeometry2::CubicBezier(curve) => CurveGeometry2::CubicBezier(
                curve
                    .subcurve_between_exact(&start, &end, &policy)
                    .map_err(|cause| self.subdivision_error(cause))?,
            ),
            CurveGeometry2::RationalQuadraticBezier(curve) => {
                CurveGeometry2::RationalQuadraticBezier(
                    curve
                        .subcurve_between_exact(&start, &end, &policy)
                        .map_err(|cause| self.subdivision_error(cause))?,
                )
            }
            CurveGeometry2::RationalBezier(curve) => CurveGeometry2::RationalBezier(
                match curve
                    .subcurve_between_exact(&start, &end, &policy)
                    .map_err(|cause| self.subdivision_error(cause))?
                {
                    Classification::Decided(curve) => curve,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Subdivision,
                            self.family(),
                            self.source(),
                            reason,
                        ));
                    }
                },
            ),
            CurveGeometry2::PolynomialBSpline(curve) => CurveGeometry2::PolynomialBSpline(
                curve
                    .subcurve(start, end)
                    .map_err(|error| remap_operation(error, CurveOperation2::Subdivision))?,
            ),
            CurveGeometry2::Nurbs(curve) => CurveGeometry2::Nurbs(
                curve
                    .subcurve(start, end)
                    .map_err(|error| remap_operation(error, CurveOperation2::Subdivision))?,
            ),
        };
        self.with_lineage(geometry, lineage)
    }

    fn with_lineage(
        &self,
        geometry: CurveGeometry2,
        lineage: CurveParameterLineage2,
    ) -> ExactCurveResult<Self> {
        if geometry
            .retained_source()
            .is_some_and(|retained| Some(retained) != self.source())
        {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                geometry.family(),
                self.source(),
                CurveError::ConflictingCurveSource,
            ));
        }
        Ok(Self {
            data: Rc::new(CurveData2 {
                geometry,
                source: self.source(),
                lineage,
                parameter_domain: OnceCell::new(),
                native_bezier_fragments: OnceCell::new(),
                rational_evaluators: OnceCell::new(),
                bounds: OnceCell::new(),
            }),
        })
    }

    fn lineage_subrange(
        &self,
        start: &Real,
        end: &Real,
    ) -> ExactCurveResult<CurveParameterLineage2> {
        Ok(CurveParameterLineage2 {
            root: Rc::clone(&self.data.lineage.root),
            range: ParamRange::new(
                self.lineage_parameter_at(start)?,
                self.lineage_parameter_at(end)?,
            ),
        })
    }

    fn lineage_parameter_at(&self, parameter: &Real) -> ExactCurveResult<Real> {
        let domain = self.parameter_domain();
        let local =
            ((parameter - domain.start()) / (domain.end() - domain.start())).map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Subdivision,
                    self.family(),
                    self.source(),
                    cause.into(),
                )
            })?;
        Ok(self.data.lineage.range.start()
            + &local * (self.data.lineage.range.end() - self.data.lineage.range.start()))
    }

    fn retain_root_image_injectivity(&self, policy: &CurvePolicy) {
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
        let Ok(evaluators) = self.rational_evaluators() else {
            return;
        };
        if evaluators.len() == 1 && evaluators[0].has_certified_injective_axis(policy) {
            let _ = root.image_is_injective.set(true);
        }
    }

    pub(crate) fn shares_certified_parameter_lineage(&self, other: &Self) -> bool {
        Rc::ptr_eq(&self.data.lineage.root, &other.data.lineage.root)
            && self.data.lineage.root.image_is_injective.get() == Some(&true)
    }

    fn subdivision_error(&self, cause: CurveError) -> ExactCurveError {
        ExactCurveError::invalid(
            CurveOperation2::Subdivision,
            self.family(),
            self.source(),
            cause,
        )
    }

    fn transform_error(&self, cause: CurveError) -> ExactCurveError {
        ExactCurveError::invalid(
            CurveOperation2::Transformation,
            self.family(),
            self.source(),
            cause,
        )
    }

    /// Evaluates this curve at an exact parameter.
    ///
    /// Native line, arc, and Bezier parameters use `[0, 1]`. Arc parameters
    /// traverse exact rational quadratic spans in sweep order. Spline
    /// parameters use their authored knot domain.
    pub fn point_at(&self, parameter: &Real) -> ExactCurveResult<Point2> {
        self.point_at_side(parameter, CurveParameterSide2::Automatic)
    }

    /// Evaluates an exact point with explicit spline-knot side policy.
    pub fn point_at_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Point2> {
        match self.geometry() {
            CurveGeometry2::PolynomialBSpline(curve) => curve.point_at_side(parameter, side),
            CurveGeometry2::Nurbs(curve) => curve.point_at_side(parameter, side),
            geometry => {
                validate_unit_parameter(parameter, geometry.family(), self.source())?;
                match geometry {
                    CurveGeometry2::Line(curve) => Ok(curve.point_at(parameter.clone())),
                    CurveGeometry2::CircularArc(_) => evaluate_promoted_arc(
                        self.native_bezier_fragments()?,
                        parameter,
                        self.source(),
                    ),
                    CurveGeometry2::QuadraticBezier(curve) => Ok(curve.point_at(parameter.clone())),
                    CurveGeometry2::CubicBezier(curve) => Ok(curve.point_at(parameter.clone())),
                    CurveGeometry2::RationalQuadraticBezier(curve) => {
                        match curve.point_at(parameter.clone(), &crate::CurvePolicy::certified()) {
                            Classification::Decided(point) => Ok(point),
                            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                                CurveOperation2::Evaluation,
                                CurveFamily2::RationalQuadraticBezier,
                                self.source(),
                                reason,
                            )),
                        }
                    }
                    CurveGeometry2::RationalBezier(curve) => {
                        match curve.point_at_classified(parameter, &crate::CurvePolicy::certified())
                        {
                            Classification::Decided(point) => Ok(point),
                            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                                CurveOperation2::Evaluation,
                                CurveFamily2::RationalBezier,
                                self.source(),
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
    pub fn point_at_wrapped(&self, parameter: &Real) -> ExactCurveResult<Point2> {
        self.point_at_wrapped_side(parameter, CurveParameterSide2::Automatic)
    }

    /// Evaluates a periodic spline with explicit side selection at wrapped seams.
    pub fn point_at_wrapped_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Point2> {
        match self.geometry() {
            CurveGeometry2::PolynomialBSpline(curve) => {
                curve.point_at_wrapped_side(parameter, side)
            }
            CurveGeometry2::Nurbs(curve) => curve.point_at_wrapped_side(parameter, side),
            _ => Err(ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                self.family(),
                self.source(),
                CurveError::CurveIsNotPeriodic,
            )),
        }
    }

    /// Evaluates the exact first derivative in this curve's public parameter.
    ///
    /// Native curves use `[0, 1]`; spline curves use their authored knot
    /// domain. Promoted rational evaluators are built once per shared curve and
    /// preserve source-span parameter scaling.
    pub fn derivative_at(&self, parameter: &Real) -> ExactCurveResult<CurveDerivative2> {
        self.derivative_at_side(parameter, CurveParameterSide2::Automatic)
    }

    /// Evaluates an exact first derivative with explicit knot-boundary side policy.
    pub fn derivative_at_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<CurveDerivative2> {
        let mut derivatives = self.derivatives_at_side(parameter, 1, side)?;
        Ok(derivatives.pop().expect("one derivative requested"))
    }

    /// Evaluates the first periodic derivative at any wrappable parameter.
    pub fn derivative_at_wrapped(&self, parameter: &Real) -> ExactCurveResult<CurveDerivative2> {
        self.derivative_at_wrapped_side(parameter, CurveParameterSide2::Automatic)
    }

    /// Evaluates the first periodic derivative with explicit seam-side selection.
    pub fn derivative_at_wrapped_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<CurveDerivative2> {
        let mut derivatives = self.derivatives_at_wrapped_side(parameter, 1, side)?;
        Ok(derivatives.pop().expect("one derivative requested"))
    }

    /// Evaluates exact derivatives through `max_order` in the public parameter.
    ///
    /// The returned vector stores orders `1..=max_order`. Native curves use
    /// `[0, 1]`; spline curves use their authored knot domain.
    pub fn derivatives_at(
        &self,
        parameter: &Real,
        max_order: usize,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        self.derivatives_at_side(parameter, max_order, CurveParameterSide2::Automatic)
    }

    /// Evaluates exact derivatives with explicit retained-fragment side policy.
    pub fn derivatives_at_side(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        match self.geometry() {
            CurveGeometry2::PolynomialBSpline(curve) => {
                return curve.derivatives_at_side(parameter, max_order, side);
            }
            CurveGeometry2::Nurbs(curve) => {
                return curve.derivatives_at_side(parameter, max_order, side);
            }
            _ => {}
        }
        let fragments = self.native_bezier_fragments()?;
        let (first, last) =
            select_native_fragments(fragments, parameter, self.family(), self.source())?;
        let first_derivatives = self.derivatives_on_native_fragment(first, parameter, max_order)?;
        if first == last || side == CurveParameterSide2::Left {
            return Ok(first_derivatives);
        }
        let last_derivatives = self.derivatives_on_native_fragment(last, parameter, max_order)?;
        if side == CurveParameterSide2::Right {
            return Ok(last_derivatives);
        }
        certify_matching_derivatives(
            first_derivatives,
            last_derivatives,
            self.family(),
            self.source(),
        )
    }

    /// Evaluates periodic derivatives through `max_order` at any wrappable parameter.
    pub fn derivatives_at_wrapped(
        &self,
        parameter: &Real,
        max_order: usize,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        self.derivatives_at_wrapped_side(parameter, max_order, CurveParameterSide2::Automatic)
    }

    /// Evaluates periodic derivatives with explicit side selection at wrapped seams.
    pub fn derivatives_at_wrapped_side(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        match self.geometry() {
            CurveGeometry2::PolynomialBSpline(curve) => {
                curve.derivatives_at_wrapped_side(parameter, max_order, side)
            }
            CurveGeometry2::Nurbs(curve) => {
                curve.derivatives_at_wrapped_side(parameter, max_order, side)
            }
            _ => Err(ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                self.family(),
                self.source(),
                CurveError::CurveIsNotPeriodic,
            )),
        }
    }

    fn derivatives_on_native_fragment(
        &self,
        fragment_index: usize,
        parameter: &Real,
        max_order: usize,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        let fragments = self.native_bezier_fragments()?;
        let provenance = fragments[fragment_index].provenance();
        let (start, end) = provenance.parameter_range();
        let width = end - start;
        let local = ((parameter - start) / &width).map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                self.family(),
                self.source(),
                cause.into(),
            )
        })?;
        let evaluator = &self.rational_evaluators()?[fragment_index];
        let local_derivatives = match if max_order == 1 {
            evaluator
                .derivative_at_classified(&local, &crate::CurvePolicy::certified())
                .map(|derivative| vec![derivative])
        } else {
            evaluator.derivatives_at_classified(&local, max_order, &crate::CurvePolicy::certified())
        } {
            Classification::Decided(derivatives) => derivatives,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    self.family(),
                    self.source(),
                    reason,
                ));
            }
        };
        let inverse_width = (Real::one() / width).map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                self.family(),
                self.source(),
                cause.into(),
            )
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

    /// Returns whether reusable promoted rational evaluators have been retained.
    pub fn is_rational_evaluator_cache_cached(&self) -> bool {
        self.data.rational_evaluators.get().is_some()
    }

    /// Returns whether conservative exact bounds have already been retained.
    pub fn is_bounds_cached(&self) -> bool {
        self.data.bounds.get().is_some()
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
    /// exact parameter interval.
    pub fn native_bezier_fragments(&self) -> ExactCurveResult<&[NativeBezierFragment2]> {
        match self
            .data
            .native_bezier_fragments
            .get_or_init(|| promote_native_bezier_fragments(self))
        {
            Ok(fragments) => Ok(fragments),
            Err(error) => Err(error.clone()),
        }
    }

    pub(crate) fn rational_evaluators(&self) -> ExactCurveResult<&[RationalBezier2]> {
        match self.data.rational_evaluators.get_or_init(|| {
            self.native_bezier_fragments()?
                .iter()
                .map(|fragment| {
                    rationalize_subcurve(fragment.curve(), self.family(), self.source())
                })
                .collect()
        }) {
            Ok(evaluators) => Ok(evaluators),
            Err(error) => Err(error.clone()),
        }
    }
}

impl PartialEq for Curve2 {
    fn eq(&self, other: &Self) -> bool {
        self.data.geometry == other.data.geometry && self.data.source == other.data.source
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

    /// Returns stable source identity when retained.
    pub fn source(self) -> Option<CurveSource2> {
        self.curve.source()
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
    pub fn reversed(self) -> ExactCurveResult<Curve2> {
        self.curve.reversed()
    }

    /// Applies an exact planar similarity without cloning the source carrier first.
    pub fn transform_similarity(self, transform: &Similarity2) -> ExactCurveResult<Curve2> {
        self.curve.transform_similarity(transform)
    }

    /// Splits this curve exactly at a strict interior public parameter.
    pub fn split_at(self, parameter: Real) -> ExactCurveResult<(Curve2, Curve2)> {
        self.curve.split_at(parameter)
    }

    /// Returns the exact curve image over a strictly ordered public range.
    pub fn subcurve(self, start: Real, end: Real) -> ExactCurveResult<Curve2> {
        self.curve.subcurve(start, end)
    }

    /// Evaluates this borrowed curve without cloning its retained carrier.
    pub fn point_at(self, parameter: &Real) -> ExactCurveResult<Point2> {
        self.curve.point_at(parameter)
    }

    /// Evaluates an exact point with explicit spline-knot side policy.
    pub fn point_at_side(
        self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Point2> {
        self.curve.point_at_side(parameter, side)
    }

    /// Evaluates an explicitly periodic spline at any wrappable parameter.
    pub fn point_at_wrapped(self, parameter: &Real) -> ExactCurveResult<Point2> {
        self.curve.point_at_wrapped(parameter)
    }

    /// Evaluates a periodic spline with explicit side selection at wrapped seams.
    pub fn point_at_wrapped_side(
        self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Point2> {
        self.curve.point_at_wrapped_side(parameter, side)
    }

    /// Evaluates the exact first derivative without cloning the curve.
    pub fn derivative_at(self, parameter: &Real) -> ExactCurveResult<CurveDerivative2> {
        self.curve.derivative_at(parameter)
    }

    /// Evaluates an exact one-sided or certified two-sided first derivative.
    pub fn derivative_at_side(
        self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<CurveDerivative2> {
        self.curve.derivative_at_side(parameter, side)
    }

    /// Evaluates the first periodic derivative at any wrappable parameter.
    pub fn derivative_at_wrapped(self, parameter: &Real) -> ExactCurveResult<CurveDerivative2> {
        self.curve.derivative_at_wrapped(parameter)
    }

    /// Evaluates the first periodic derivative with explicit seam-side selection.
    pub fn derivative_at_wrapped_side(
        self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<CurveDerivative2> {
        self.curve.derivative_at_wrapped_side(parameter, side)
    }

    /// Evaluates exact derivatives through `max_order` without cloning the curve.
    pub fn derivatives_at(
        self,
        parameter: &Real,
        max_order: usize,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        self.curve.derivatives_at(parameter, max_order)
    }

    /// Evaluates exact derivatives with explicit retained-fragment side policy.
    pub fn derivatives_at_side(
        self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        self.curve.derivatives_at_side(parameter, max_order, side)
    }

    /// Evaluates periodic derivatives through `max_order` at any wrappable parameter.
    pub fn derivatives_at_wrapped(
        self,
        parameter: &Real,
        max_order: usize,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        self.curve.derivatives_at_wrapped(parameter, max_order)
    }

    /// Evaluates periodic derivatives with explicit side selection at wrapped seams.
    pub fn derivatives_at_wrapped_side(
        self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        self.curve
            .derivatives_at_wrapped_side(parameter, max_order, side)
    }
}

impl CurvePath2 {
    /// Constructs a nonempty ordered path with exactly connected endpoints.
    pub fn try_new(curves: Vec<Curve2>) -> ExactCurveResult<Self> {
        if curves.is_empty() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                CurveFamily2::Line,
                None,
                CurveError::EmptyCurvePath,
            ));
        }
        for adjacent in curves.windows(2) {
            if adjacent[0].end() == adjacent[1].start() {
                continue;
            }
            match adjacent[0]
                .end()
                .distance_squared(adjacent[1].start())
                .zero_status()
            {
                ZeroKnowledge::Zero => {}
                ZeroKnowledge::NonZero => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Construction,
                        adjacent[1].family(),
                        adjacent[1].source(),
                        CurveError::DisconnectedCurvePath,
                    ));
                }
                ZeroKnowledge::Unknown => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Construction,
                        adjacent[1].family(),
                        adjacent[1].source(),
                        crate::UncertaintyReason::RealSign,
                    ));
                }
            }
        }
        Ok(Self {
            data: Rc::new(CurvePathData2 {
                curves,
                native_bezier_fragments: OnceCell::new(),
                bezier_boundary_loop: OnceCell::new(),
                bounds: OnceCell::new(),
            }),
        })
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
    pub fn reversed(&self) -> ExactCurveResult<Self> {
        let curves = self
            .curves()
            .iter()
            .rev()
            .map(Curve2::reversed)
            .collect::<ExactCurveResult<Vec<_>>>()?;
        Self::try_new(curves).map_err(|error| remap_operation(error, CurveOperation2::Reversal))
    }

    /// Applies an exact planar similarity to every curve in the connected path.
    pub fn transform_similarity(&self, transform: &Similarity2) -> ExactCurveResult<Self> {
        let curves = self
            .curves()
            .iter()
            .map(|curve| curve.transform_similarity(transform))
            .collect::<ExactCurveResult<Vec<_>>>()?;
        Self::try_new(curves)
            .map_err(|error| remap_operation(error, CurveOperation2::Transformation))
    }

    /// Replaces one path vertex with an exact line chamfer.
    ///
    /// `vertex_index` identifies the next curve at the vertex. Interior
    /// vertices therefore use `1..curves().len()`. Index zero addresses the
    /// start/end seam of an exactly closed path. Both parameters must be
    /// strictly interior to their adjacent curves' public parameter domains.
    /// Every retained source curve keeps its family, source identity, and
    /// authored parameter lineage; only the inserted chamfer is a new line.
    pub fn chamfer_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_parameter: Real,
        next_parameter: Real,
    ) -> ExactCurveResult<Self> {
        let (previous_index, next_index) =
            self.corner_curve_indices(vertex_index, CurveOperation2::Chamfer)?;
        let previous = &self.data.curves[previous_index];
        let next = &self.data.curves[next_index];
        validate_corner_parameter(previous, &previous_parameter, CurveOperation2::Chamfer)?;
        validate_corner_parameter(next, &next_parameter, CurveOperation2::Chamfer)?;

        let previous_cut = previous
            .point_at_side(&previous_parameter, CurveParameterSide2::Left)
            .map_err(|error| remap_operation(error, CurveOperation2::Chamfer))?;
        let next_cut = next
            .point_at_side(&next_parameter, CurveParameterSide2::Right)
            .map_err(|error| remap_operation(error, CurveOperation2::Chamfer))?;
        let previous_trim = previous
            .subcurve(
                previous.parameter_domain().start().clone(),
                previous_parameter,
            )
            .map_err(|error| remap_operation(error, CurveOperation2::Chamfer))?;
        let next_trim = next
            .subcurve(next_parameter, next.parameter_domain().end().clone())
            .map_err(|error| remap_operation(error, CurveOperation2::Chamfer))?;
        let chamfer = LineSeg2::try_new(previous_cut, next_cut)
            .map(Curve2::from)
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Chamfer,
                    previous.family(),
                    previous.source(),
                    cause,
                )
            })?;

        self.with_corner_replaced(
            vertex_index,
            previous_index,
            next_index,
            previous_trim,
            chamfer,
            next_trim,
            CurveOperation2::Chamfer,
        )
    }

    /// Replaces one path vertex with an exact tangent circular fillet.
    ///
    /// The two parameters identify tangent points on the adjacent curves and
    /// must be strictly interior to their public domains. `center` and
    /// `clockwise` define the inserted circular arc. Hypercurve certifies a
    /// nonzero common radius, tangency, and traversal-direction agreement using
    /// [`Real`] predicates before materializing the result. Index zero edits
    /// the seam of an exactly closed path.
    pub fn fillet_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_parameter: Real,
        next_parameter: Real,
        center: &Point2,
        clockwise: bool,
    ) -> ExactCurveResult<Self> {
        let (previous_index, next_index) =
            self.corner_curve_indices(vertex_index, CurveOperation2::Fillet)?;
        let previous = &self.data.curves[previous_index];
        let next = &self.data.curves[next_index];
        validate_corner_parameter(previous, &previous_parameter, CurveOperation2::Fillet)?;
        validate_corner_parameter(next, &next_parameter, CurveOperation2::Fillet)?;

        let previous_point = previous
            .point_at_side(&previous_parameter, CurveParameterSide2::Left)
            .map_err(|error| remap_operation(error, CurveOperation2::Fillet))?;
        let next_point = next
            .point_at_side(&next_parameter, CurveParameterSide2::Right)
            .map_err(|error| remap_operation(error, CurveOperation2::Fillet))?;
        let radius_squared =
            validate_fillet_radius(previous, &previous_point, &next_point, center)?;
        validate_curve_fillet_tangent(
            previous,
            &previous_parameter,
            CurveParameterSide2::Left,
            &previous_point,
            center,
            clockwise,
        )?;
        validate_curve_fillet_tangent(
            next,
            &next_parameter,
            CurveParameterSide2::Right,
            &next_point,
            center,
            clockwise,
        )?;

        let previous_trim = previous
            .subcurve(
                previous.parameter_domain().start().clone(),
                previous_parameter,
            )
            .map_err(|error| remap_operation(error, CurveOperation2::Fillet))?;
        let next_trim = next
            .subcurve(next_parameter, next.parameter_domain().end().clone())
            .map_err(|error| remap_operation(error, CurveOperation2::Fillet))?;
        let fillet = Curve2::from(CircularArc2::new_with_certified_radius(
            previous_point,
            next_point,
            center.clone(),
            radius_squared,
            clockwise,
            None,
        ));

        self.with_corner_replaced(
            vertex_index,
            previous_index,
            next_index,
            previous_trim,
            fillet,
            next_trim,
            CurveOperation2::Fillet,
        )
    }

    fn corner_curve_indices(
        &self,
        vertex_index: usize,
        operation: CurveOperation2,
    ) -> ExactCurveResult<(usize, usize)> {
        let curve_count = self.data.curves.len();
        if vertex_index >= curve_count {
            return Err(ExactCurveError::invalid(
                operation,
                self.data.curves[0].family(),
                self.data.curves[0].source(),
                CurveError::InvalidCurveRange,
            ));
        }
        if vertex_index == 0 {
            certify_closed_path(self, operation)?;
            return Ok((curve_count - 1, 0));
        }
        Ok((vertex_index - 1, vertex_index))
    }

    #[allow(clippy::too_many_arguments)]
    fn with_corner_replaced(
        &self,
        vertex_index: usize,
        previous_index: usize,
        next_index: usize,
        previous_trim: Curve2,
        inserted: Curve2,
        next_trim: Curve2,
        operation: CurveOperation2,
    ) -> ExactCurveResult<Self> {
        let mut curves = Vec::with_capacity(self.data.curves.len() + 1);
        if vertex_index == 0 {
            curves.push(inserted);
            curves.push(next_trim);
            if next_index + 1 < previous_index {
                curves.extend(
                    self.data.curves[next_index + 1..previous_index]
                        .iter()
                        .cloned(),
                );
            }
            curves.push(previous_trim);
        } else {
            curves.extend(self.data.curves[..previous_index].iter().cloned());
            curves.push(previous_trim);
            curves.push(inserted);
            curves.push(next_trim);
            curves.extend(self.data.curves[next_index + 1..].iter().cloned());
        }
        Self::try_new(curves).map_err(|error| remap_operation(error, operation))
    }

    /// Returns whether aggregate path bounds have already been retained.
    pub fn is_bounds_cached(&self) -> bool {
        self.data.bounds.get().is_some()
    }

    /// Borrows conservative exact bounds computed once across all path curves.
    pub fn bounds(&self) -> ExactCurveResult<&Aabb2> {
        match self.data.bounds.get_or_init(|| {
            let mut bounds = self.data.curves[0].bounds()?.clone();
            let policy = crate::CurvePolicy::certified();
            for curve in &self.data.curves[1..] {
                bounds = decided_bounds(
                    bounds.union(curve.bounds()?, &policy),
                    curve.family(),
                    curve.source(),
                )?;
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
    /// reuse the retained exact Bezier boundary classifier.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
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
            return classify_native_arc_chord_path(arc_curve, arc, chord, point, policy);
        }

        self.bezier_boundary_loop()?
            .boundary_loop()
            .classify_point(point, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::NativeTopology,
                    self.curves()[0].family(),
                    self.curves()[0].source(),
                    cause,
                )
            })
    }

    /// Returns whether exact native promotion has already been retained.
    pub fn is_native_bezier_fragments_cached(&self) -> bool {
        self.data.native_bezier_fragments.get().is_some()
    }

    /// Promotes this path once and borrows exact native Bezier fragments in traversal order.
    pub fn native_bezier_fragments(&self) -> ExactCurveResult<&[NativeBezierFragment2]> {
        match self.data.native_bezier_fragments.get_or_init(|| {
            let capacity = self.data.curves.iter().try_fold(0_usize, |count, curve| {
                curve
                    .native_bezier_fragments()
                    .map(|fragments| count + fragments.len())
            })?;
            let mut fragments = Vec::with_capacity(capacity);
            for curve in &self.data.curves {
                fragments.extend_from_slice(curve.native_bezier_fragments()?);
            }
            Ok(fragments)
        }) {
            Ok(fragments) => Ok(fragments),
            Err(error) => Err(error.clone()),
        }
    }

    /// Returns whether closed boundary construction has already been retained.
    pub fn is_bezier_boundary_loop_cached(&self) -> bool {
        self.data.bezier_boundary_loop.get().is_some()
    }

    /// Builds a closed native Bezier boundary once and borrows the retained result.
    pub fn bezier_boundary_loop(&self) -> ExactCurveResult<&NativeBezierBoundaryLoop2> {
        match self.data.bezier_boundary_loop.get_or_init(|| {
            if self.start() != self.end() {
                match self.start().distance_squared(self.end()).zero_status() {
                    ZeroKnowledge::Zero => {}
                    ZeroKnowledge::NonZero => {
                        return Err(ExactCurveError::invalid(
                            CurveOperation2::Arrangement,
                            self.data.curves[0].family(),
                            self.data.curves[0].source(),
                            CurveError::OpenCurvePath,
                        ));
                    }
                    ZeroKnowledge::Unknown => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Arrangement,
                            self.data.curves[0].family(),
                            self.data.curves[0].source(),
                            crate::UncertaintyReason::RealSign,
                        ));
                    }
                }
            }
            let fragments = self.native_bezier_fragments()?.to_vec();
            let boundary_loop = BezierBoundaryLoop2::new(
                fragments
                    .iter()
                    .map(|fragment| fragment.curve().clone())
                    .collect(),
            )
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Arrangement,
                    self.data.curves[0].family(),
                    self.data.curves[0].source(),
                    cause,
                )
            })?;
            Ok(NativeBezierBoundaryLoop2 {
                boundary_loop,
                fragments,
            })
        }) {
            Ok(boundary) => Ok(boundary),
            Err(error) => Err(error.clone()),
        }
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
    policy: &CurvePolicy,
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
        ExactCurveError::invalid(
            CurveOperation2::NativeTopology,
            arc_curve.family(),
            arc_curve.source(),
            cause,
        )
    })? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let arc_side = match chord.classify_point(&representative, policy) {
        Classification::Decided(LineSide::On) => {
            return Err(ExactCurveError::invalid(
                CurveOperation2::NativeTopology,
                arc_curve.family(),
                arc_curve.source(),
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
    pub fn reversed(self) -> ExactCurveResult<CurvePath2> {
        let curves = self
            .curves
            .iter()
            .rev()
            .map(Curve2::reversed)
            .collect::<ExactCurveResult<Vec<_>>>()?;
        CurvePath2::try_new(curves)
            .map_err(|error| remap_operation(error, CurveOperation2::Reversal))
    }

    /// Applies an exact planar similarity to the borrowed connected path.
    pub fn transform_similarity(self, transform: &Similarity2) -> ExactCurveResult<CurvePath2> {
        let curves = self
            .curves
            .iter()
            .map(|curve| curve.transform_similarity(transform))
            .collect::<ExactCurveResult<Vec<_>>>()?;
        CurvePath2::try_new(curves)
            .map_err(|error| remap_operation(error, CurveOperation2::Transformation))
    }

    /// Replaces one borrowed path vertex with an exact line chamfer.
    pub fn chamfer_vertex_by_parameters(
        self,
        vertex_index: usize,
        previous_parameter: Real,
        next_parameter: Real,
    ) -> ExactCurveResult<CurvePath2> {
        CurvePath2::try_new(self.curves.to_vec())?.chamfer_vertex_by_parameters(
            vertex_index,
            previous_parameter,
            next_parameter,
        )
    }

    /// Replaces one borrowed path vertex with an exact tangent circular fillet.
    pub fn fillet_vertex_by_parameters(
        self,
        vertex_index: usize,
        previous_parameter: Real,
        next_parameter: Real,
        center: &Point2,
        clockwise: bool,
    ) -> ExactCurveResult<CurvePath2> {
        CurvePath2::try_new(self.curves.to_vec())?.fillet_vertex_by_parameters(
            vertex_index,
            previous_parameter,
            next_parameter,
            center,
            clockwise,
        )
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

impl CurveSpanProvenance2 {
    /// Returns the owning curve source identity, when supplied.
    pub const fn source(&self) -> Option<CurveSource2> {
        self.source
    }

    /// Returns the spline source-span index, or `None` for a native curve.
    pub const fn source_span_index(&self) -> Option<usize> {
        self.source_span_index
    }

    /// Returns the exact span range in the current curve's public parameterization.
    pub fn parameter_range(&self) -> (&Real, &Real) {
        (&self.parameter_start, &self.parameter_end)
    }

    /// Returns the exact range in the root source curve's parameterization.
    ///
    /// This differs from [`CurveSpanProvenance2::parameter_range`] after a
    /// top-level trim or reversal. The regular parameter range remains in the
    /// current curve's public domain; this range tracks the authored source.
    pub fn source_parameter_range(&self) -> (&Real, &Real) {
        (&self.source_parameter_start, &self.source_parameter_end)
    }
}

impl NativeBezierFragment2 {
    /// Returns the promoted exact native curve.
    pub const fn curve(&self) -> &BezierSubcurve2 {
        &self.curve
    }

    /// Returns source identity and exact source parameter range.
    pub const fn provenance(&self) -> &CurveSpanProvenance2 {
        &self.provenance
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

    /// Returns provenance-bearing fragments one-to-one with boundary curves.
    pub fn fragments(&self) -> &[NativeBezierFragment2] {
        &self.fragments
    }

    /// Consumes the result into its validated boundary and provenance records.
    pub fn into_parts(self) -> (BezierBoundaryLoop2, Vec<NativeBezierFragment2>) {
        (self.boundary_loop, self.fragments)
    }
}

fn compute_curve_bounds(curve: &Curve2) -> ExactCurveResult<Aabb2> {
    let policy = crate::CurvePolicy::certified();
    match curve.geometry() {
        CurveGeometry2::Line(line) => decided_bounds(
            Aabb2::from_line(line, &policy),
            curve.family(),
            curve.source(),
        ),
        CurveGeometry2::CircularArc(arc) => decided_bounds(
            Aabb2::from_arc(arc, &policy).map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::NativeTopology,
                    curve.family(),
                    curve.source(),
                    cause,
                )
            })?,
            curve.family(),
            curve.source(),
        ),
        _ => {
            let fragments = curve.native_bezier_fragments()?;
            let mut bounds = decided_subcurve_bounds(
                fragments[0].curve(),
                curve.family(),
                curve.source(),
                &policy,
            )?;
            for fragment in &fragments[1..] {
                let fragment_bounds = decided_subcurve_bounds(
                    fragment.curve(),
                    curve.family(),
                    curve.source(),
                    &policy,
                )?;
                bounds = decided_bounds(
                    bounds.union(&fragment_bounds, &policy),
                    curve.family(),
                    curve.source(),
                )?;
            }
            Ok(bounds)
        }
    }
}

fn decided_subcurve_bounds(
    curve: &BezierSubcurve2,
    family: CurveFamily2,
    source: Option<CurveSource2>,
    policy: &crate::CurvePolicy,
) -> ExactCurveResult<Aabb2> {
    let bounds = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.control_hull_box(policy),
        BezierSubcurve2::Cubic(curve) => curve.control_hull_box(policy),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(policy),
    };
    decided_bounds(bounds, family, source)
}

fn decided_bounds(
    bounds: Classification<Aabb2>,
    family: CurveFamily2,
    source: Option<CurveSource2>,
) -> ExactCurveResult<Aabb2> {
    match bounds {
        Classification::Decided(bounds) => Ok(bounds),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::NativeTopology,
            family,
            source,
            reason,
        )),
    }
}

fn select_native_fragments(
    fragments: &[NativeBezierFragment2],
    parameter: &Real,
    family: CurveFamily2,
    source: Option<CurveSource2>,
) -> ExactCurveResult<(usize, usize)> {
    let policy = crate::CurvePolicy::certified();
    let mut first = None;
    let mut last = None;
    for (index, fragment) in fragments.iter().enumerate() {
        let (start, end) = fragment.provenance().parameter_range();
        match (
            crate::classify::compare_reals(start, parameter, &policy),
            crate::classify::compare_reals(parameter, end, &policy),
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
                    source,
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
    }
    first.zip(last).ok_or_else(|| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            family,
            source,
            CurveError::InvalidCurveParameter,
        )
    })
}

fn certify_matching_derivatives(
    first: Vec<CurveDerivative2>,
    second: Vec<CurveDerivative2>,
    family: CurveFamily2,
    source: Option<CurveSource2>,
) -> ExactCurveResult<Vec<CurveDerivative2>> {
    debug_assert_eq!(first.len(), second.len());
    let policy = crate::CurvePolicy::certified();
    for (first_derivative, second_derivative) in first.iter().zip(&second) {
        match (
            crate::classify::compare_reals(first_derivative.dx(), second_derivative.dx(), &policy),
            crate::classify::compare_reals(first_derivative.dy(), second_derivative.dy(), &policy),
        ) {
            (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => {}
            (Some(_), Some(_)) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    family,
                    source,
                    crate::UncertaintyReason::Boundary,
                ));
            }
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    family,
                    source,
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
    source: Option<CurveSource2>,
) -> ExactCurveResult<RationalBezier2> {
    let (control_points, weights) = match curve {
        BezierSubcurve2::Quadratic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 3],
        ),
        BezierSubcurve2::Cubic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 4],
        ),
        BezierSubcurve2::RationalQuadratic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            curve.weights().into_iter().cloned().collect(),
        ),
        BezierSubcurve2::Rational(curve) => return Ok(curve.clone()),
    };
    RationalBezier2::try_new(control_points, weights).map_err(|cause| {
        ExactCurveError::invalid(CurveOperation2::NativeTopology, family, source, cause)
    })
}

fn promote_native_bezier_fragments(curve: &Curve2) -> ExactCurveResult<Vec<NativeBezierFragment2>> {
    let source = curve.source();
    let native = |native_curve, source_span_index, parameter_start: Real, parameter_end: Real| {
        let source_parameter_start = curve
            .lineage_parameter_at(&parameter_start)
            .map_err(|error| remap_operation(error, CurveOperation2::NativeTopology))?;
        let source_parameter_end = curve
            .lineage_parameter_at(&parameter_end)
            .map_err(|error| remap_operation(error, CurveOperation2::NativeTopology))?;
        Ok(NativeBezierFragment2 {
            curve: native_curve,
            provenance: CurveSpanProvenance2 {
                source,
                source_span_index,
                parameter_start,
                parameter_end,
                source_parameter_start,
                source_parameter_end,
            },
        })
    };
    let unit = || (Real::zero(), Real::one());
    match curve.geometry() {
        CurveGeometry2::Line(line) => {
            let (start, end) = unit();
            let midpoint = line.point_at(
                (Real::one() / Real::from(2_i8)).expect("two is a nonzero exact denominator"),
            );
            Ok(vec![native(
                BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                    line.start().clone(),
                    midpoint,
                    line.end().clone(),
                )),
                None,
                start,
                end,
            )?])
        }
        CurveGeometry2::CircularArc(value) => Ok(decompose_circular_arc(value, source)?
            .spans()
            .iter()
            .enumerate()
            .map(|(span_index, span)| {
                let (start, end) = span.parameter_range();
                native(
                    BezierSubcurve2::RationalQuadratic(span.curve().clone()),
                    Some(span_index),
                    start.clone(),
                    end.clone(),
                )
            })
            .collect::<ExactCurveResult<Vec<_>>>()?),
        CurveGeometry2::QuadraticBezier(value) => {
            let (start, end) = unit();
            Ok(vec![native(
                BezierSubcurve2::Quadratic(value.clone()),
                None,
                start,
                end,
            )?])
        }
        CurveGeometry2::CubicBezier(value) => {
            let (start, end) = unit();
            Ok(vec![native(
                BezierSubcurve2::Cubic(value.clone()),
                None,
                start,
                end,
            )?])
        }
        CurveGeometry2::RationalQuadraticBezier(value) => {
            let (start, end) = unit();
            Ok(vec![native(
                BezierSubcurve2::RationalQuadratic(value.clone()),
                None,
                start,
                end,
            )?])
        }
        CurveGeometry2::RationalBezier(value) => {
            let (start, end) = unit();
            Ok(vec![native(
                BezierSubcurve2::Rational(value.clone()),
                None,
                start,
                end,
            )?])
        }
        CurveGeometry2::PolynomialBSpline(value) => Ok(value
            .bezier_spans()?
            .map(|span| {
                let (start, end) = span.knot_interval();
                native(
                    span.curve().clone(),
                    Some(span.span_index()),
                    start.clone(),
                    end.clone(),
                )
            })
            .collect::<ExactCurveResult<Vec<_>>>()?),
        CurveGeometry2::Nurbs(value) => Ok(value
            .native_spans()?
            .map(|span| {
                let source_span = span.source_span();
                let (start, end) = source_span.knot_interval();
                native(
                    span.curve().clone(),
                    Some(source_span.span_index()),
                    start.clone(),
                    end.clone(),
                )
            })
            .collect::<ExactCurveResult<Vec<_>>>()?),
    }
}

fn evaluate_promoted_arc(
    fragments: &[NativeBezierFragment2],
    parameter: &Real,
    source: Option<CurveSource2>,
) -> ExactCurveResult<Point2> {
    let policy = crate::CurvePolicy::certified();
    for fragment in fragments {
        let (start, end) = fragment.provenance().parameter_range();
        let lower = crate::classify::compare_reals(start, parameter, &policy);
        let upper = crate::classify::compare_reals(parameter, end, &policy);
        match (lower, upper) {
            (
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            ) => {
                let local = ((parameter - start) / (end - start)).map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Evaluation,
                        CurveFamily2::CircularArc,
                        source,
                        cause.into(),
                    )
                })?;
                let BezierSubcurve2::RationalQuadratic(curve) = fragment.curve() else {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Evaluation,
                        CurveFamily2::CircularArc,
                        source,
                        CurveError::Topology(
                            "circular arc promoted to a non-rational-quadratic span".into(),
                        ),
                    ));
                };
                return match curve.point_at(local, &policy) {
                    Classification::Decided(point) => Ok(point),
                    Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                        CurveOperation2::Evaluation,
                        CurveFamily2::CircularArc,
                        source,
                        reason,
                    )),
                };
            }
            (Some(_), Some(_)) => {}
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::CircularArc,
                    source,
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
    }
    Err(ExactCurveError::invalid(
        CurveOperation2::Evaluation,
        CurveFamily2::CircularArc,
        source,
        CurveError::InvalidCurveParameter,
    ))
}

fn validate_unit_parameter(
    parameter: &Real,
    family: CurveFamily2,
    source: Option<CurveSource2>,
) -> ExactCurveResult<()> {
    match crate::classify::in_closed_unit_interval(parameter, &crate::CurvePolicy::certified()) {
        Some(true) => Ok(()),
        Some(false) => Err(ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            family,
            source,
            CurveError::InvalidCurveParameter,
        )),
        None => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            family,
            source,
            crate::UncertaintyReason::Ordering,
        )),
    }
}

fn validate_corner_parameter(
    curve: &Curve2,
    parameter: &Real,
    operation: CurveOperation2,
) -> ExactCurveResult<()> {
    validate_strict_split_parameter(
        curve.parameter_domain().start(),
        parameter,
        curve.parameter_domain().end(),
        curve.family(),
        curve.source(),
    )
    .map_err(|error| remap_operation(error, operation))
}

fn certify_closed_path(path: &CurvePath2, operation: CurveOperation2) -> ExactCurveResult<()> {
    if path.start() == path.end() {
        return Ok(());
    }
    let first = &path.data.curves[0];
    match crate::classify::is_zero(
        &path.start().distance_squared(path.end()),
        &CurvePolicy::certified(),
    ) {
        Some(true) => Ok(()),
        Some(false) => Err(ExactCurveError::invalid(
            operation,
            first.family(),
            first.source(),
            CurveError::OpenCurvePath,
        )),
        None => Err(ExactCurveError::blocked(
            operation,
            first.family(),
            first.source(),
            crate::UncertaintyReason::RealSign,
        )),
    }
}

fn validate_fillet_radius(
    context: &Curve2,
    previous_point: &Point2,
    next_point: &Point2,
    center: &Point2,
) -> ExactCurveResult<Real> {
    let policy = CurvePolicy::certified();
    let radius_squared = previous_point.distance_squared(center);
    match crate::classify::is_zero(&radius_squared, &policy) {
        Some(false) => {}
        Some(true) => {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Fillet,
                context.family(),
                context.source(),
                CurveError::ZeroRadiusArc,
            ));
        }
        None => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                context.family(),
                context.source(),
                crate::UncertaintyReason::RealSign,
            ));
        }
    }

    let radius_delta = &radius_squared - next_point.distance_squared(center);
    match crate::classify::is_zero(&radius_delta, &policy) {
        Some(true) => Ok(radius_squared),
        Some(false) => Err(ExactCurveError::invalid(
            CurveOperation2::Fillet,
            context.family(),
            context.source(),
            CurveError::RadiusMismatch,
        )),
        None => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            context.family(),
            context.source(),
            crate::UncertaintyReason::RealSign,
        )),
    }
}

fn validate_curve_fillet_tangent(
    curve: &Curve2,
    parameter: &Real,
    side: CurveParameterSide2,
    tangent_point: &Point2,
    center: &Point2,
    clockwise: bool,
) -> ExactCurveResult<()> {
    let (source_dx, source_dy, source_zero_status) = match curve.geometry() {
        CurveGeometry2::CircularArc(arc) => {
            let (radius_dx, radius_dy) = tangent_point.delta_from(arc.center());
            let (dx, dy) = if arc.is_clockwise() {
                (radius_dy, -radius_dx)
            } else {
                (-radius_dy, radius_dx)
            };
            let zero_status = (&dx * &dx + &dy * &dy).zero_status();
            (dx, dy, zero_status)
        }
        _ => {
            let derivative = curve
                .derivative_at_side(parameter, side)
                .map_err(|error| remap_operation(error, CurveOperation2::Fillet))?;
            (
                derivative.dx().clone(),
                derivative.dy().clone(),
                derivative.zero_status(),
            )
        }
    };
    match source_zero_status {
        ZeroKnowledge::NonZero => {}
        ZeroKnowledge::Zero => {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Fillet,
                curve.family(),
                curve.source(),
                CurveError::InvalidFilletTangency,
            ));
        }
        ZeroKnowledge::Unknown => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                curve.family(),
                curve.source(),
                crate::UncertaintyReason::RealSign,
            ));
        }
    }

    let (radius_dx, radius_dy) = tangent_point.delta_from(center);
    let (fillet_dx, fillet_dy) = if clockwise {
        (radius_dy, -radius_dx)
    } else {
        (-radius_dy, radius_dx)
    };
    let tangent_cross = &source_dx * &fillet_dy - &source_dy * &fillet_dx;
    let policy = CurvePolicy::certified();
    match crate::classify::is_zero(&tangent_cross, &policy) {
        Some(true) => {}
        Some(false) => {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Fillet,
                curve.family(),
                curve.source(),
                CurveError::InvalidFilletTangency,
            ));
        }
        None => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                curve.family(),
                curve.source(),
                crate::UncertaintyReason::RealSign,
            ));
        }
    }

    let direction_dot = &source_dx * &fillet_dx + &source_dy * &fillet_dy;
    match crate::classify::real_sign(&direction_dot, &policy) {
        Some(RealSign::Positive) => Ok(()),
        Some(RealSign::Zero | RealSign::Negative) => Err(ExactCurveError::invalid(
            CurveOperation2::Fillet,
            curve.family(),
            curve.source(),
            CurveError::InvalidFilletTangency,
        )),
        None => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            curve.family(),
            curve.source(),
            crate::UncertaintyReason::RealSign,
        )),
    }
}

fn validate_strict_split_parameter(
    domain_start: &Real,
    parameter: &Real,
    domain_end: &Real,
    family: CurveFamily2,
    source: Option<CurveSource2>,
) -> ExactCurveResult<()> {
    let policy = CurvePolicy::certified();
    match (
        crate::classify::compare_reals(domain_start, parameter, &policy),
        crate::classify::compare_reals(parameter, domain_end, &policy),
    ) {
        (Some(std::cmp::Ordering::Less), Some(std::cmp::Ordering::Less)) => Ok(()),
        (Some(_), Some(_)) => Err(ExactCurveError::invalid(
            CurveOperation2::Subdivision,
            family,
            source,
            CurveError::InvalidCurveParameter,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            family,
            source,
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
    source: Option<CurveSource2>,
) -> ExactCurveResult<()> {
    let policy = CurvePolicy::certified();
    match (
        crate::classify::compare_reals(domain_start, start, &policy),
        crate::classify::compare_reals(start, end, &policy),
        crate::classify::compare_reals(end, domain_end, &policy),
    ) {
        (
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            Some(std::cmp::Ordering::Less),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
        ) => Ok(()),
        (Some(_), Some(_), Some(_)) => Err(ExactCurveError::invalid(
            CurveOperation2::Subdivision,
            family,
            source,
            CurveError::InvalidCurveParameter,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            family,
            source,
            crate::UncertaintyReason::Ordering,
        )),
    }
}
