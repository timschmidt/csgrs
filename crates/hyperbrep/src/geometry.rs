//! Canonical exact 3D curves and parameterized surfaces.

use std::cmp::Ordering;
use std::sync::{Arc, OnceLock};

use hypercurve::{
    BezierParameter2, CircularArc2, Classification, Contour2, Curve2, CurveFamily2, CurveGeometry2,
    CurvePolicy, LineArcRegion2, LineSeg2, Point2 as CurvePoint2, RationalBezier2,
    RationalBezierPointIncidence2, Segment2,
};
use hyperlattice::{Aabb, Matrix4, Point2, Point3, Real, Vector2, Vector3};
use hyperlimit::{PredicateOutcome, compare_reals, point3_equal};

use crate::error::{GeometryError, GeometryResult};

/// Exact closed parameter interval.
#[derive(Clone, Debug)]
pub struct ParameterDomain {
    start: Real,
    end: Real,
}

impl ParameterDomain {
    /// Constructs a nonempty increasing closed parameter interval.
    pub fn new(start: Real, end: Real) -> GeometryResult<Self> {
        match compare_reals(&start, &end) {
            PredicateOutcome::Decided {
                value: Ordering::Less,
                ..
            } => Ok(Self { start, end }),
            PredicateOutcome::Decided { .. } => Err(GeometryError::InvalidParameterDomain),
            PredicateOutcome::Unknown { needed, stage } => {
                Err(GeometryError::PredicateUnresolved { needed, stage })
            }
        }
    }

    /// Returns the conventional unit interval.
    pub fn unit() -> Self {
        Self {
            start: Real::zero(),
            end: Real::one(),
        }
    }

    /// Returns the inclusive interval start.
    pub const fn start(&self) -> &Real {
        &self.start
    }

    /// Returns the inclusive interval end.
    pub const fn end(&self) -> &Real {
        &self.end
    }

    /// Certifies whether `parameter` belongs to this interval.
    pub fn contains(&self, parameter: &Real) -> GeometryResult<bool> {
        let after_start = decided_order(compare_reals(parameter, &self.start))?;
        let before_end = decided_order(compare_reals(parameter, &self.end))?;
        Ok(matches!(after_start, Ordering::Equal | Ordering::Greater)
            && matches!(before_end, Ordering::Equal | Ordering::Less))
    }
}

/// Supported exact spatial curve family.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Curve3Kind {
    /// Finite line segment.
    Line,
    /// Arbitrary-degree rational Bézier curve.
    RationalBezier,
    /// Non-periodic finite-domain NURBS curve.
    Nurbs,
    /// Exact circular arc.
    CircleArc,
    /// Exact elliptic arc.
    EllipseArc,
}

/// Exact first derivative of a spatial curve.
#[derive(Clone, Debug)]
pub struct CurveDerivative3 {
    vector: Vector3,
}

/// Complete represented parameter-location result for one spatial curve.
#[derive(Clone, Debug)]
pub enum CurveParameterLocation {
    /// The point is not on the finite curve.
    None,
    /// All represented exact parameters whose images equal the point.
    Parameters(Vec<Real>),
    /// Every parameter in the curve domain maps to the point.
    EntireDomain,
}

impl CurveDerivative3 {
    /// Returns the exact derivative vector.
    pub const fn vector(&self) -> &Vector3 {
        &self.vector
    }
}

/// Immutable exact face-local parameter-space curve.
///
/// Hypercurve owns the planar curve implementation; HyperBREP binds it to an
/// edge use and surface.
#[derive(Clone, Debug)]
pub struct Pcurve {
    curve: Curve2,
}

impl Pcurve {
    /// Binds an exact Hypercurve carrier for later use by BREP topology.
    pub fn new(curve: Curve2) -> Self {
        Self { curve }
    }

    /// Returns the underlying exact planar curve.
    pub const fn curve(&self) -> &Curve2 {
        &self.curve
    }

    /// Returns the planar curve family.
    pub fn kind(&self) -> CurveFamily2 {
        self.curve.family()
    }

    /// Returns the inclusive exact parameter-domain start.
    pub fn domain_start(&self) -> &Real {
        self.curve.parameter_domain().start()
    }

    /// Returns the inclusive exact parameter-domain end.
    pub fn domain_end(&self) -> &Real {
        self.curve.parameter_domain().end()
    }

    /// Evaluates an exact point in surface parameter space.
    pub fn point_at(&self, parameter: &Real) -> GeometryResult<Point2> {
        let point = self.curve.point_at(parameter)?;
        Ok(Point2::new(point.x().clone(), point.y().clone()))
    }

    /// Splits this pcurve exactly at a strict interior public parameter.
    pub fn split_at(&self, parameter: &Real) -> GeometryResult<(Self, Self)> {
        let (first, second) = self.curve.split_at(parameter.clone())?;
        Ok((Self::new(first), Self::new(second)))
    }

    /// Returns the same exact parameter-space image with reversed traversal.
    pub fn reversed(&self) -> GeometryResult<Self> {
        Ok(Self::new(self.curve.reversed()?))
    }

    pub(crate) fn endpoints(&self) -> GeometryResult<(Point2, Point2)> {
        Ok((
            self.point_at(self.domain_start())?,
            self.point_at(self.domain_end())?,
        ))
    }

    pub(crate) fn segment(&self) -> GeometryResult<Segment2> {
        match self.curve.geometry() {
            CurveGeometry2::Line(line) => Ok(Segment2::Line(line.clone())),
            CurveGeometry2::CircularArc(arc) => Ok(Segment2::Arc(arc.clone())),
            _ => Err(GeometryError::UnsupportedPcurveContour),
        }
    }

    pub(crate) fn circular_arc(&self) -> Option<&hypercurve::CircularArc2> {
        match self.curve.geometry() {
            CurveGeometry2::CircularArc(arc) => Some(arc),
            _ => None,
        }
    }

    pub(crate) fn line_segment(&self) -> Option<&hypercurve::LineSeg2> {
        match self.curve.geometry() {
            CurveGeometry2::Line(line) => Some(line),
            _ => None,
        }
    }

    pub(crate) fn reflected_and_reversed_x(&self, reflection_sum: Real) -> GeometryResult<Self> {
        let reflect = |point: &hypercurve::Point2| {
            hypercurve::Point2::new(&reflection_sum - point.x(), point.y().clone())
        };
        match self.curve.geometry() {
            CurveGeometry2::Line(line) => Ok(Self::new(Curve2::from(
                hypercurve::LineSeg2::try_new(reflect(line.end()), reflect(line.start()))?,
            ))),
            CurveGeometry2::CircularArc(arc) => Ok(Self::new(Curve2::from(
                hypercurve::CircularArc2::try_from_center(
                    reflect(arc.end()),
                    reflect(arc.start()),
                    reflect(arc.center()),
                    arc.is_clockwise(),
                )?,
            ))),
            _ => Err(GeometryError::UnsupportedTransform),
        }
    }
}

/// Immutable exact spatial curve.
#[derive(Clone, Debug)]
pub struct Curve3 {
    data: Arc<Curve3Data>,
}

#[derive(Debug)]
struct Curve3Data {
    geometry: CurveGeometry3,
    domain: ParameterDomain,
    bounds: OnceLock<Result<Aabb, GeometryError>>,
}

#[derive(Debug)]
enum CurveGeometry3 {
    Line(Line3),
    RationalBezier(RationalBezier3),
    Nurbs(NurbsCurve3),
    CircleArc(EllipseArc3),
    EllipseArc(EllipseArc3),
}

#[derive(Clone, Debug)]
pub(crate) enum Curve3ExactData {
    Line(Box<Line3ExactData>),
    RationalBezier {
        control_points: Vec<Point3>,
        weights: Vec<Real>,
    },
    Nurbs {
        degree: usize,
        control_points: Vec<Point3>,
        weights: Vec<Real>,
        knots: Vec<Real>,
    },
    EllipseArc(Box<EllipseArcExactData>),
}

#[derive(Clone, Debug)]
pub(crate) struct Line3ExactData {
    pub(crate) start: Point3,
    pub(crate) end: Point3,
}

#[derive(Clone, Debug)]
pub(crate) struct EllipseArcExactData {
    pub(crate) circle: bool,
    pub(crate) center: Point3,
    pub(crate) x: Vector3,
    pub(crate) y: Vector3,
    pub(crate) x_radius: Real,
    pub(crate) y_radius: Real,
    pub(crate) domain_start: Real,
    pub(crate) domain_end: Real,
    pub(crate) angle_at_start: Real,
    pub(crate) direction: i8,
}

#[derive(Debug)]
struct Line3 {
    start: Point3,
    end: Point3,
}

#[derive(Debug)]
struct RationalBezier3 {
    control_points: Vec<Point3>,
    weights: Vec<Real>,
    homogeneous_controls: OnceLock<Vec<HomogeneousPoint3>>,
}

#[derive(Debug)]
struct NurbsCurve3 {
    degree: usize,
    control_points: Vec<Point3>,
    weights: Vec<Real>,
    knots: Vec<Real>,
    homogeneous_controls: OnceLock<Vec<HomogeneousPoint3>>,
}

#[derive(Clone, Debug)]
struct EllipseArc3 {
    center: Point3,
    x: Vector3,
    y: Vector3,
    x_radius: Real,
    y_radius: Real,
    angle_at_start: Real,
    direction: i8,
}

#[derive(Clone, Debug)]
struct HomogeneousPoint3 {
    x: Real,
    y: Real,
    z: Real,
    w: Real,
}

type AffineControlGrid = (Vec<Vec<Point3>>, Vec<Vec<Real>>);
type HomogeneousNurbsSplit = (
    Vec<HomogeneousPoint3>,
    Vec<Real>,
    Vec<HomogeneousPoint3>,
    Vec<Real>,
);

impl Curve3 {
    /// Constructs a finite exact line segment.
    pub fn line(start: Point3, end: Point3) -> GeometryResult<Self> {
        match point3_equal(&start, &end) {
            PredicateOutcome::Decided { value: true, .. } => {
                return Err(GeometryError::DegenerateLine);
            }
            PredicateOutcome::Decided { value: false, .. } => {}
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(GeometryError::PredicateUnresolved { needed, stage });
            }
        }
        Ok(Self::from_parts(
            CurveGeometry3::Line(Line3 { start, end }),
            ParameterDomain::unit(),
        ))
    }

    /// Constructs an exact circular arc parameterized directly by angle.
    pub fn circle_arc(
        center: Point3,
        x: Vector3,
        y: Vector3,
        radius: Real,
        start_angle: Real,
        end_angle: Real,
    ) -> GeometryResult<Self> {
        require_positive(&radius, GeometryError::InvalidRadius)?;
        validate_arc_axes(&x, &y)?;
        let domain = validate_arc_domain(start_angle, end_angle)?;
        Ok(Self::from_parts(
            CurveGeometry3::CircleArc(EllipseArc3 {
                center,
                x,
                y,
                x_radius: radius.clone(),
                y_radius: radius,
                angle_at_start: domain.start().clone(),
                direction: 1,
            }),
            domain,
        ))
    }

    /// Constructs an exact elliptic arc parameterized directly by angle.
    pub fn ellipse_arc(
        center: Point3,
        x: Vector3,
        y: Vector3,
        x_radius: Real,
        y_radius: Real,
        start_angle: Real,
        end_angle: Real,
    ) -> GeometryResult<Self> {
        require_positive(&x_radius, GeometryError::InvalidEllipseRadii)?;
        require_positive(&y_radius, GeometryError::InvalidEllipseRadii)?;
        validate_arc_axes(&x, &y)?;
        let domain = validate_arc_domain(start_angle, end_angle)?;
        Ok(Self::from_parts(
            CurveGeometry3::EllipseArc(EllipseArc3 {
                center,
                x,
                y,
                x_radius,
                y_radius,
                angle_at_start: domain.start().clone(),
                direction: 1,
            }),
            domain,
        ))
    }

    /// Constructs an arbitrary-degree rational Bézier curve over `[0, 1]`.
    pub fn rational_bezier(
        control_points: Vec<Point3>,
        weights: Vec<Real>,
    ) -> GeometryResult<Self> {
        validate_control_net(&control_points, &weights)?;
        validate_positive_weights(&weights)?;
        Ok(Self::from_parts(
            CurveGeometry3::RationalBezier(RationalBezier3 {
                control_points,
                weights,
                homogeneous_controls: OnceLock::new(),
            }),
            ParameterDomain::unit(),
        ))
    }

    /// Constructs a finite, non-periodic exact NURBS curve.
    pub fn nurbs(
        degree: usize,
        control_points: Vec<Point3>,
        weights: Vec<Real>,
        knots: Vec<Real>,
    ) -> GeometryResult<Self> {
        validate_control_net(&control_points, &weights)?;
        validate_positive_weights(&weights)?;
        if degree == 0 || degree >= control_points.len() {
            return Err(GeometryError::InvalidDegree);
        }
        if knots.len() != control_points.len() + degree + 1 {
            return Err(GeometryError::InvalidKnotCount);
        }
        for adjacent in knots.windows(2) {
            if !matches!(
                decided_order(compare_reals(&adjacent[0], &adjacent[1]))?,
                Ordering::Less | Ordering::Equal
            ) {
                return Err(GeometryError::InvalidKnotOrder);
            }
        }
        validate_clamped_knot_multiplicities(degree, &knots)?;
        let domain =
            ParameterDomain::new(knots[degree].clone(), knots[control_points.len()].clone())?;
        Ok(Self::from_parts(
            CurveGeometry3::Nurbs(NurbsCurve3 {
                degree,
                control_points,
                weights,
                knots,
                homogeneous_controls: OnceLock::new(),
            }),
            domain,
        ))
    }

    fn from_parts(geometry: CurveGeometry3, domain: ParameterDomain) -> Self {
        Self {
            data: Arc::new(Curve3Data {
                geometry,
                domain,
                bounds: OnceLock::new(),
            }),
        }
    }

    /// Returns the curve family.
    pub fn kind(&self) -> Curve3Kind {
        match &self.data.geometry {
            CurveGeometry3::Line(_) => Curve3Kind::Line,
            CurveGeometry3::RationalBezier(_) => Curve3Kind::RationalBezier,
            CurveGeometry3::Nurbs(_) => Curve3Kind::Nurbs,
            CurveGeometry3::CircleArc(_) => Curve3Kind::CircleArc,
            CurveGeometry3::EllipseArc(_) => Curve3Kind::EllipseArc,
        }
    }

    pub(crate) fn canonical_line(&self) -> GeometryResult<Option<Self>> {
        match &self.data.geometry {
            CurveGeometry3::Line(_) => Ok(Some(self.clone())),
            CurveGeometry3::RationalBezier(curve) => {
                let Some(first_weight) = curve.weights.first() else {
                    return Ok(None);
                };
                for weight in &curve.weights {
                    if decided_order(compare_reals(weight, first_weight))? != Ordering::Equal {
                        return Ok(None);
                    }
                }
                let start = curve
                    .control_points
                    .first()
                    .expect("validated rational Bézier has controls");
                let end = curve
                    .control_points
                    .last()
                    .expect("validated rational Bézier has controls");
                let degree = curve.control_points.len() - 1;
                if degree == 0 {
                    return Ok(None);
                }
                for (index, point) in curve.control_points.iter().enumerate() {
                    let parameter = (Real::from(index as u64) / Real::from(degree as u64))
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    if !points_equal(point, &start.lerp(end, &parameter))? {
                        return Ok(None);
                    }
                }
                Ok(Some(Self::line(start.clone(), end.clone())?))
            }
            CurveGeometry3::Nurbs(_)
            | CurveGeometry3::CircleArc(_)
            | CurveGeometry3::EllipseArc(_) => Ok(None),
        }
    }

    /// Returns the exact public parameter domain.
    pub fn domain(&self) -> &ParameterDomain {
        &self.data.domain
    }

    /// Returns conservative exact model-space bounds.
    ///
    /// Positive-weight rational Bézier and NURBS curves lie in the convex hull
    /// of their affine control points, so this bound never samples or converts
    /// to floating point.
    pub fn bounds(&self) -> GeometryResult<Aabb> {
        self.data
            .bounds
            .get_or_init(|| match &self.data.geometry {
                CurveGeometry3::Line(line) => {
                    exact_point_bounds(&[line.start.clone(), line.end.clone()])
                }
                CurveGeometry3::RationalBezier(curve) => exact_point_bounds(&curve.control_points),
                CurveGeometry3::Nurbs(curve) => exact_point_bounds(&curve.control_points),
                CurveGeometry3::CircleArc(arc) | CurveGeometry3::EllipseArc(arc) => {
                    ellipse_arc_conservative_bounds(arc)
                }
            })
            .clone()
    }

    /// Evaluates an exact model-space point.
    pub fn point_at(&self, parameter: &Real) -> GeometryResult<Point3> {
        if !self.domain().contains(parameter)? {
            return Err(GeometryError::ParameterOutsideDomain);
        }
        match &self.data.geometry {
            CurveGeometry3::Line(line) => Ok(line.start.lerp(&line.end, parameter)),
            CurveGeometry3::RationalBezier(curve) => {
                evaluate_homogeneous_bezier(curve.homogeneous_controls(), parameter)
            }
            CurveGeometry3::Nurbs(curve) => {
                let span = find_span(
                    parameter,
                    self.domain(),
                    curve.degree,
                    curve.control_points.len(),
                    &curve.knots,
                )?;
                evaluate_homogeneous_de_boor(
                    curve.homogeneous_controls(),
                    &curve.knots,
                    curve.degree,
                    span,
                    parameter,
                )
            }
            CurveGeometry3::CircleArc(arc) | CurveGeometry3::EllipseArc(arc) => {
                Ok(evaluate_ellipse_arc(arc, self.domain(), parameter))
            }
        }
    }

    /// Evaluates one positive-order exact derivative.
    pub fn derivative_at(
        &self,
        parameter: &Real,
        order: usize,
    ) -> GeometryResult<CurveDerivative3> {
        if order == 0 {
            return Err(GeometryError::InvalidDerivativeOrder);
        }
        if !self.domain().contains(parameter)? {
            return Err(GeometryError::ParameterOutsideDomain);
        }
        let vector = match &self.data.geometry {
            CurveGeometry3::Line(line) => {
                if order == 1 {
                    &line.end - &line.start
                } else {
                    Vector3::zero()
                }
            }
            CurveGeometry3::RationalBezier(curve) => {
                evaluate_rational_bezier_derivative(curve, parameter, order)?
            }
            CurveGeometry3::Nurbs(curve) => {
                let span = find_span(
                    parameter,
                    self.domain(),
                    curve.degree,
                    curve.control_points.len(),
                    &curve.knots,
                )?;
                evaluate_homogeneous_de_boor_derivative(
                    curve.homogeneous_controls(),
                    &curve.knots,
                    curve.degree,
                    span,
                    parameter,
                    order,
                )?
            }
            CurveGeometry3::CircleArc(arc) | CurveGeometry3::EllipseArc(arc) => {
                evaluate_ellipse_arc_derivative(arc, self.domain(), parameter, order)
            }
        };
        Ok(CurveDerivative3 { vector })
    }

    /// Evaluates the curve at its exact domain start.
    pub fn start(&self) -> GeometryResult<Point3> {
        self.point_at(self.domain().start())
    }

    /// Evaluates the curve at its exact domain end.
    pub fn end(&self) -> GeometryResult<Point3> {
        self.point_at(self.domain().end())
    }

    pub(crate) fn line_endpoints(&self) -> Option<(&Point3, &Point3)> {
        match &self.data.geometry {
            CurveGeometry3::Line(line) => Some((&line.start, &line.end)),
            CurveGeometry3::RationalBezier(_)
            | CurveGeometry3::Nurbs(_)
            | CurveGeometry3::CircleArc(_)
            | CurveGeometry3::EllipseArc(_) => None,
        }
    }

    pub(crate) fn exact_data(&self) -> Curve3ExactData {
        match &self.data.geometry {
            CurveGeometry3::Line(line) => Curve3ExactData::Line(Box::new(Line3ExactData {
                start: line.start.clone(),
                end: line.end.clone(),
            })),
            CurveGeometry3::RationalBezier(curve) => Curve3ExactData::RationalBezier {
                control_points: curve.control_points.clone(),
                weights: curve.weights.clone(),
            },
            CurveGeometry3::Nurbs(curve) => Curve3ExactData::Nurbs {
                degree: curve.degree,
                control_points: curve.control_points.clone(),
                weights: curve.weights.clone(),
                knots: curve.knots.clone(),
            },
            CurveGeometry3::CircleArc(arc) | CurveGeometry3::EllipseArc(arc) => {
                Curve3ExactData::EllipseArc(Box::new(EllipseArcExactData {
                    circle: matches!(self.data.geometry, CurveGeometry3::CircleArc(_)),
                    center: arc.center.clone(),
                    x: arc.x.clone(),
                    y: arc.y.clone(),
                    x_radius: arc.x_radius.clone(),
                    y_radius: arc.y_radius.clone(),
                    domain_start: self.domain().start().clone(),
                    domain_end: self.domain().end().clone(),
                    angle_at_start: arc.angle_at_start.clone(),
                    direction: arc.direction,
                }))
            }
        }
    }

    pub(crate) fn from_exact_data(data: Curve3ExactData) -> GeometryResult<Self> {
        match data {
            Curve3ExactData::Line(data) => Self::line(data.start, data.end),
            Curve3ExactData::RationalBezier {
                control_points,
                weights,
            } => Self::rational_bezier(control_points, weights),
            Curve3ExactData::Nurbs {
                degree,
                control_points,
                weights,
                knots,
            } => Self::nurbs(degree, control_points, weights, knots),
            Curve3ExactData::EllipseArc(data) => {
                let EllipseArcExactData {
                    circle,
                    center,
                    x,
                    y,
                    x_radius,
                    y_radius,
                    domain_start,
                    domain_end,
                    angle_at_start,
                    direction,
                } = *data;
                require_positive(&x_radius, GeometryError::InvalidEllipseRadii)?;
                require_positive(&y_radius, GeometryError::InvalidEllipseRadii)?;
                validate_arc_axes(&x, &y)?;
                let domain = validate_arc_domain(domain_start, domain_end)?;
                if !matches!(direction, -1 | 1) {
                    return Err(GeometryError::InvalidParameterDomain);
                }
                if circle && decided_order(compare_reals(&x_radius, &y_radius))? != Ordering::Equal
                {
                    return Err(GeometryError::InvalidEllipseRadii);
                }
                let arc = EllipseArc3 {
                    center,
                    x,
                    y,
                    x_radius,
                    y_radius,
                    angle_at_start,
                    direction,
                };
                Ok(Self::from_parts(
                    if circle {
                        CurveGeometry3::CircleArc(arc)
                    } else {
                        CurveGeometry3::EllipseArc(arc)
                    },
                    domain,
                ))
            }
        }
    }

    /// Returns the same geometric image with reversed parameter direction.
    pub fn reversed(&self) -> GeometryResult<Self> {
        match &self.data.geometry {
            CurveGeometry3::Line(line) => Self::line(line.end.clone(), line.start.clone()),
            CurveGeometry3::RationalBezier(curve) => {
                let mut points = curve.control_points.clone();
                let mut weights = curve.weights.clone();
                points.reverse();
                weights.reverse();
                Self::rational_bezier(points, weights)
            }
            CurveGeometry3::Nurbs(curve) => {
                let mut points = curve.control_points.clone();
                let mut weights = curve.weights.clone();
                points.reverse();
                weights.reverse();
                let domain_sum = self.domain().start() + self.domain().end();
                let knots = curve
                    .knots
                    .iter()
                    .rev()
                    .map(|knot| &domain_sum - knot)
                    .collect();
                Self::nurbs(curve.degree, points, weights, knots)
            }
            CurveGeometry3::CircleArc(arc) => Ok(Self::from_parts(
                CurveGeometry3::CircleArc(reversed_ellipse_arc(arc, self.domain())),
                self.domain().clone(),
            )),
            CurveGeometry3::EllipseArc(arc) => Ok(Self::from_parts(
                CurveGeometry3::EllipseArc(reversed_ellipse_arc(arc, self.domain())),
                self.domain().clone(),
            )),
        }
    }

    /// Splits any supported finite curve at a certified interior parameter.
    ///
    /// Line and rational-Bézier halves use their conventional local unit
    /// interval; NURBS and conic halves retain their native source intervals.
    pub fn split_at(&self, parameter: &Real) -> GeometryResult<(Self, Self)> {
        require_interior_parameter(self.domain(), parameter)?;
        match &self.data.geometry {
            CurveGeometry3::Line(line) => {
                let split = line.start.lerp(&line.end, parameter);
                Ok((
                    Self::line(line.start.clone(), split.clone())?,
                    Self::line(split, line.end.clone())?,
                ))
            }
            CurveGeometry3::RationalBezier(curve) => {
                let (left, right) =
                    split_homogeneous_bezier(curve.homogeneous_controls(), parameter);
                Ok((
                    rational_bezier_from_homogeneous(left)?,
                    rational_bezier_from_homogeneous(right)?,
                ))
            }
            CurveGeometry3::Nurbs(curve) => split_nurbs_curve(curve, parameter),
            CurveGeometry3::CircleArc(arc) => {
                split_ellipse_arc(arc, self.domain(), parameter, Curve3Kind::CircleArc)
            }
            CurveGeometry3::EllipseArc(arc) => {
                split_ellipse_arc(arc, self.domain(), parameter, Curve3Kind::EllipseArc)
            }
        }
    }

    /// Returns the exact image over a strictly ordered source-parameter range.
    ///
    /// Line and rational-Bézier results use their conventional local unit
    /// domain. NURBS and named conic results retain the selected source
    /// interval because their native knot or angle parameterization is
    /// authoritative.
    pub fn subcurve(&self, start: &Real, end: &Real) -> GeometryResult<Self> {
        if decided_order(compare_reals(start, end))? != Ordering::Less
            || decided_order(compare_reals(start, self.domain().start()))? == Ordering::Less
            || decided_order(compare_reals(end, self.domain().end()))? == Ordering::Greater
        {
            return Err(GeometryError::InvalidParameterDomain);
        }
        if decided_order(compare_reals(start, self.domain().start()))? == Ordering::Equal
            && decided_order(compare_reals(end, self.domain().end()))? == Ordering::Equal
        {
            return Ok(self.clone());
        }
        match &self.data.geometry {
            CurveGeometry3::Line(_) => Self::line(self.point_at(start)?, self.point_at(end)?),
            CurveGeometry3::RationalBezier(curve) => {
                let controls = if decided_order(compare_reals(start, &Real::zero()))?
                    == Ordering::Equal
                {
                    split_homogeneous_bezier(curve.homogeneous_controls(), end).0
                } else {
                    let (_, right) = split_homogeneous_bezier(curve.homogeneous_controls(), start);
                    let relative_end = ((end - start) / (Real::one() - start))
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    split_homogeneous_bezier(&right, &relative_end).0
                };
                rational_bezier_from_homogeneous(controls)
            }
            CurveGeometry3::Nurbs(_) => {
                let mut selected = self.clone();
                if decided_order(compare_reals(end, selected.domain().end()))? == Ordering::Less {
                    selected = selected.split_at(end)?.0;
                }
                if decided_order(compare_reals(start, selected.domain().start()))?
                    == Ordering::Greater
                {
                    selected = selected.split_at(start)?.1;
                }
                Ok(selected)
            }
            CurveGeometry3::CircleArc(arc) | CurveGeometry3::EllipseArc(arc) => {
                let geometry = EllipseArc3 {
                    angle_at_start: ellipse_arc_angle(arc, self.domain(), start),
                    ..arc.clone()
                };
                Ok(Self::from_parts(
                    if matches!(self.data.geometry, CurveGeometry3::CircleArc(_)) {
                        CurveGeometry3::CircleArc(geometry)
                    } else {
                        CurveGeometry3::EllipseArc(geometry)
                    },
                    ParameterDomain::new(start.clone(), end.clone())?,
                ))
            }
        }
    }

    /// Locates every represented exact parameter whose image is `point`.
    ///
    /// Algebraic roots not representable by `Real` are an explicit error;
    /// self-intersections may return more than one parameter.
    pub fn parameters_of(&self, point: &Point3) -> GeometryResult<CurveParameterLocation> {
        match &self.data.geometry {
            CurveGeometry3::Line(line) => locate_line_parameter(self, line, point),
            CurveGeometry3::RationalBezier(curve) => {
                locate_rational_bezier_parameters(self, curve, point)
            }
            CurveGeometry3::CircleArc(arc) | CurveGeometry3::EllipseArc(arc) => {
                locate_ellipse_arc_parameters(self, arc, point)
            }
            CurveGeometry3::Nurbs(curve) => locate_nurbs_parameters(curve, point),
        }
    }

    pub(crate) fn transformed(&self, transform: &Matrix4) -> GeometryResult<Self> {
        match &self.data.geometry {
            CurveGeometry3::Line(line) => Self::line(
                transform_point(transform, &line.start)?,
                transform_point(transform, &line.end)?,
            ),
            CurveGeometry3::RationalBezier(curve) => Self::rational_bezier(
                transform_points(transform, &curve.control_points)?,
                curve.weights.clone(),
            ),
            CurveGeometry3::Nurbs(curve) => Self::nurbs(
                curve.degree,
                transform_points(transform, &curve.control_points)?,
                curve.weights.clone(),
                curve.knots.clone(),
            ),
            CurveGeometry3::CircleArc(arc) | CurveGeometry3::EllipseArc(arc) => {
                let transformed = EllipseArcExactData {
                    circle: matches!(self.data.geometry, CurveGeometry3::CircleArc(_)),
                    center: transform_point(transform, &arc.center)?,
                    x: transform.transform_direction3(&arc.x),
                    y: transform.transform_direction3(&arc.y),
                    x_radius: arc.x_radius.clone(),
                    y_radius: arc.y_radius.clone(),
                    domain_start: self.domain().start().clone(),
                    domain_end: self.domain().end().clone(),
                    angle_at_start: arc.angle_at_start.clone(),
                    direction: arc.direction,
                };
                Self::from_exact_data(Curve3ExactData::EllipseArc(Box::new(transformed)))
            }
        }
    }
}

impl RationalBezier3 {
    fn homogeneous_controls(&self) -> &[HomogeneousPoint3] {
        self.homogeneous_controls
            .get_or_init(|| weighted_controls(&self.control_points, &self.weights))
    }
}

impl NurbsCurve3 {
    fn homogeneous_controls(&self) -> &[HomogeneousPoint3] {
        self.homogeneous_controls
            .get_or_init(|| weighted_controls(&self.control_points, &self.weights))
    }
}

impl RationalBezierSurface {
    fn homogeneous_controls(&self) -> &[Vec<HomogeneousPoint3>] {
        self.homogeneous_controls.get_or_init(|| {
            self.control_points
                .iter()
                .zip(&self.weights)
                .map(|(points, weights)| weighted_controls(points, weights))
                .collect()
        })
    }
}

impl NurbsSurface {
    fn homogeneous_controls(&self) -> &[Vec<HomogeneousPoint3>] {
        self.homogeneous_controls.get_or_init(|| {
            self.control_points
                .iter()
                .zip(&self.weights)
                .map(|(points, weights)| weighted_controls(points, weights))
                .collect()
        })
    }
}

impl HomogeneousPoint3 {
    fn from_affine(point: &Point3, weight: &Real) -> Self {
        Self {
            x: &point.x * weight,
            y: &point.y * weight,
            z: &point.z * weight,
            w: weight.clone(),
        }
    }

    fn lerp(&self, other: &Self, parameter: &Real) -> Self {
        let one_minus = Real::one() - parameter;
        Self {
            x: &one_minus * &self.x + parameter * &other.x,
            y: &one_minus * &self.y + parameter * &other.y,
            z: &one_minus * &self.z + parameter * &other.z,
            w: &one_minus * &self.w + parameter * &other.w,
        }
    }

    fn project(&self) -> GeometryResult<Point3> {
        Ok(Point3::new(
            (&self.x / &self.w).map_err(|_| GeometryError::ProjectiveDivision)?,
            (&self.y / &self.w).map_err(|_| GeometryError::ProjectiveDivision)?,
            (&self.z / &self.w).map_err(|_| GeometryError::ProjectiveDivision)?,
        ))
    }
}

/// Supported exact surface family.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SurfaceKind {
    /// Unbounded parameterized plane.
    Plane,
    /// Circular cylinder.
    Cylinder,
    /// Sphere.
    Sphere,
    /// Right circular cone.
    Cone,
    /// Non-self-intersecting ring torus.
    Torus,
    /// Linear extrusion of a finite exact profile curve.
    Extrusion,
    /// Full revolution of a finite exact profile curve.
    Revolution,
    /// Tensor-product rational Bézier surface.
    RationalBezier,
    /// Finite non-periodic tensor-product NURBS surface.
    Nurbs,
}

/// Parameter direction retained by a tensor-product iso-curve.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SurfaceIsoAxis {
    /// Vary `u` while holding `v` constant.
    U,
    /// Vary `v` while holding `u` constant.
    V,
}

#[derive(Clone, Debug)]
pub(crate) enum SurfaceExactData {
    Plane {
        origin: Point3,
        u: Vector3,
        v: Vector3,
    },
    Cylinder {
        origin: Point3,
        x: Vector3,
        y: Vector3,
        axis: Vector3,
        radius: Real,
    },
    Sphere {
        center: Point3,
        x: Vector3,
        y: Vector3,
        axis: Vector3,
        radius: Real,
    },
    Cone {
        apex: Point3,
        x: Vector3,
        y: Vector3,
        axis: Vector3,
        semi_angle: Real,
    },
    Torus {
        center: Point3,
        x: Vector3,
        y: Vector3,
        axis: Vector3,
        major_radius: Real,
        minor_radius: Real,
    },
    Extrusion {
        profile: Box<Curve3ExactData>,
        direction: Vector3,
    },
    Revolution {
        profile: Box<Curve3ExactData>,
        axis_origin: Point3,
        axis: Vector3,
    },
    RationalBezier {
        control_points: Vec<Vec<Point3>>,
        weights: Vec<Vec<Real>>,
    },
    Nurbs {
        u_degree: usize,
        v_degree: usize,
        control_points: Vec<Vec<Point3>>,
        weights: Vec<Vec<Real>>,
        u_knots: Vec<Real>,
        v_knots: Vec<Real>,
    },
}

/// Conservative exact model-space extent of a surface.
#[derive(Clone, Debug)]
pub enum SurfaceBounds {
    /// The surface is unbounded in model space.
    Unbounded,
    /// The entire surface lies inside this exact axis-aligned box.
    Bounded(Box<Aabb>),
}

/// Canonical domain of one surface parameter axis.
#[derive(Clone, Debug)]
pub enum SurfaceParameterDomain {
    /// Every real parameter is accepted.
    Unbounded,
    /// A finite closed exact interval.
    Closed(ParameterDomain),
    /// A periodic parameter with canonical start and positive period.
    Periodic {
        /// Canonical period start.
        start: Real,
        /// Positive exact period.
        period: Real,
    },
    /// A closed lower bound with no upper bound.
    LowerBounded {
        /// Inclusive exact lower bound.
        start: Real,
    },
}

/// Canonical two-dimensional parameter domain of a surface.
#[derive(Clone, Debug)]
pub struct SurfaceDomain {
    u: SurfaceParameterDomain,
    v: SurfaceParameterDomain,
}

impl SurfaceDomain {
    /// Returns the u-axis domain.
    pub const fn u(&self) -> &SurfaceParameterDomain {
        &self.u
    }

    /// Returns the v-axis domain.
    pub const fn v(&self) -> &SurfaceParameterDomain {
        &self.v
    }
}

/// First partial derivatives of a parametric surface.
#[derive(Clone, Debug)]
pub struct SurfacePartials {
    u: Vector3,
    v: Vector3,
}

/// Exact intersection between a finite spatial curve and a surface.
#[derive(Clone, Debug)]
pub enum CurveSurfaceIntersection {
    /// The finite curve does not meet the surface.
    None,
    /// The finite curve meets the surface at isolated exact points.
    Points(Vec<CurveSurfacePoint>),
    /// A finite exact subinterval of the curve belongs to the surface.
    Overlap(ParameterDomain),
    /// The entire finite curve belongs to the surface.
    Contained,
}

/// One isolated exact curve/surface intersection.
#[derive(Clone, Debug)]
pub struct CurveSurfacePoint {
    /// Parameter on the spatial curve.
    pub parameter: Real,
    /// Exact model-space intersection point.
    pub point: Point3,
    /// Certified local intersection multiplicity.
    pub multiplicity: IntersectionMultiplicity,
}

/// Certified local multiplicity of an isolated intersection.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum IntersectionMultiplicity {
    /// The curve crosses the surface.
    Simple,
    /// The curve is tangent to the surface.
    Tangent,
}

/// Exact retained intersection between two supported parameterized surfaces.
#[derive(Clone, Debug)]
pub enum SurfaceSurfaceIntersection {
    /// The surfaces do not meet.
    None,
    /// The surfaces denote the same unbounded plane.
    Coincident,
    /// The surfaces touch at one exact point.
    Point(Box<Point3>),
    /// Two planes meet in an unbounded exact line.
    Line(Box<SurfaceIntersectionLine>),
    /// The surfaces meet in multiple unbounded exact lines.
    Lines(Vec<SurfaceIntersectionLine>),
    /// The surfaces meet in one exact lower-bounded ray.
    Ray(Box<SurfaceIntersectionRay>),
    /// The surfaces meet in multiple exact lower-bounded rays.
    Rays(Vec<SurfaceIntersectionRay>),
    /// The surfaces meet in one exact full circle.
    Circle(Curve3),
    /// The surfaces meet in multiple exact full circles.
    Circles(Vec<Curve3>),
    /// The surfaces meet in one exact full noncircular ellipse.
    Ellipse(Curve3),
    /// Two finite carriers meet in one exact curve with retained parameter
    /// images on both surfaces.
    Curve(Box<SurfaceIntersectionCurve>),
    /// Two finite carriers meet in multiple disjoint exact curves with
    /// retained parameter images on both surfaces.
    Curves(Vec<SurfaceIntersectionCurve>),
}

/// One exact finite surface/surface intersection curve and its two pcurves.
#[derive(Clone, Debug)]
pub struct SurfaceIntersectionCurve {
    curve: Curve3,
    first_pcurve: SurfaceIntersectionPcurve,
    second_pcurve: SurfaceIntersectionPcurve,
}

/// Selects which retained pcurve belongs to an operand of a surface
/// intersection.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SurfaceIntersectionOperand {
    /// Use the pcurve on the first surface passed to the intersection query.
    First,
    /// Use the pcurve on the second surface passed to the intersection query.
    Second,
}

/// Exact surface-parameter image of a retained intersection curve.
#[derive(Clone, Debug)]
pub struct SurfaceIntersectionPcurve {
    domain: ParameterDomain,
    source_scale: Real,
    source_offset: Real,
    mapping: SurfaceIntersectionPcurveMapping,
}

/// Materialized exact Hypercurve pcurve and its spatial-parameter map.
#[derive(Clone, Debug)]
pub struct MaterializedSurfacePcurve {
    curve: Curve2,
    correspondence: SurfacePcurveCorrespondence,
}

/// Exact correspondence from a materialized pcurve to its retained spatial
/// intersection curve.
#[derive(Clone, Debug)]
pub enum SurfacePcurveCorrespondence {
    /// `spatial_parameter = scale * pcurve_parameter + offset`.
    Affine {
        /// Nonzero exact scale.
        scale: Real,
        /// Exact offset.
        offset: Real,
    },
    /// The pcurve's directed angular sweep fraction spans the spatial curve
    /// domain.
    AngularSweep {
        /// Spatial parameter at the pcurve sweep start.
        spatial_start: Real,
        /// Spatial parameter at the pcurve sweep end.
        spatial_end: Real,
    },
}

impl SurfacePcurveCorrespondence {
    /// Returns affine coefficients, or `None` for angular sweep
    /// correspondence.
    pub const fn affine_coefficients(&self) -> Option<(&Real, &Real)> {
        match self {
            Self::Affine { scale, offset } => Some((scale, offset)),
            Self::AngularSweep { .. } => None,
        }
    }
}

#[derive(Clone, Debug)]
enum SurfaceIntersectionPcurveMapping {
    RetainedCurve {
        curve: Curve2,
    },
    PlaneProjection {
        spatial: Curve3,
        origin: Point3,
        u: Vector3,
        v: Vector3,
    },
    LinearPlaneSection {
        profile: Curve3,
        plane_origin: Point3,
        plane_normal: Vector3,
        denominator: Real,
        profile_axis: TensorAxis,
        coefficient_scale: Real,
        coefficient_offset: Real,
    },
    TensorIsoV {
        constant: Real,
    },
    TensorIsoU {
        constant: Real,
    },
}

#[derive(Clone, Copy, Debug)]
enum TensorAxis {
    U,
    V,
}

#[derive(Clone, Debug)]
pub(crate) struct SurfacePcurveClipCarrier {
    pub(crate) curve: Curve2,
    pub(crate) spatial_scale: Real,
    pub(crate) spatial_offset: Real,
}

impl SurfaceIntersectionCurve {
    pub(crate) fn new(
        curve: Curve3,
        first_pcurve: SurfaceIntersectionPcurve,
        second_pcurve: SurfaceIntersectionPcurve,
    ) -> Self {
        Self {
            curve,
            first_pcurve,
            second_pcurve,
        }
    }

    pub(crate) fn on_plane(curve: Curve3, surface: &Surface) -> GeometryResult<Self> {
        let SurfaceGeometry::Plane(plane) = &surface.data.geometry else {
            return Err(GeometryError::UnsupportedIntersection);
        };
        Ok(Self::new(
            curve.clone(),
            SurfaceIntersectionPcurve::plane_projection(curve.clone(), plane),
            SurfaceIntersectionPcurve::plane_projection(curve, plane),
        ))
    }

    pub(crate) fn from_exact_pcurves(
        curve: Curve3,
        first_pcurve: Curve2,
        second_pcurve: Curve2,
    ) -> GeometryResult<Self> {
        Ok(Self::new(
            curve,
            SurfaceIntersectionPcurve::retained_curve(first_pcurve)?,
            SurfaceIntersectionPcurve::retained_curve(second_pcurve)?,
        ))
    }

    pub(crate) fn from_iso_v_pcurves(
        curve: Curve3,
        first_constant: Real,
        second_constant: Real,
    ) -> Self {
        let domain = curve.domain().clone();
        Self::new(
            curve,
            SurfaceIntersectionPcurve::tensor_iso_v(domain.clone(), first_constant),
            SurfaceIntersectionPcurve::tensor_iso_v(domain, second_constant),
        )
    }

    fn swapped(mut self) -> Self {
        std::mem::swap(&mut self.first_pcurve, &mut self.second_pcurve);
        self
    }

    /// Returns the exact model-space intersection curve.
    pub const fn curve(&self) -> &Curve3 {
        &self.curve
    }

    /// Returns the exact pcurve on the first surface operand.
    pub const fn first_pcurve(&self) -> &SurfaceIntersectionPcurve {
        &self.first_pcurve
    }

    /// Returns the exact pcurve on the second surface operand.
    pub const fn second_pcurve(&self) -> &SurfaceIntersectionPcurve {
        &self.second_pcurve
    }

    /// Returns the retained pcurve for one surface operand.
    pub const fn pcurve(&self, operand: SurfaceIntersectionOperand) -> &SurfaceIntersectionPcurve {
        match operand {
            SurfaceIntersectionOperand::First => &self.first_pcurve,
            SurfaceIntersectionOperand::Second => &self.second_pcurve,
        }
    }

    /// Restricts the spatial curve and both exact pcurve mappings to one
    /// increasing represented parameter interval.
    pub fn subcurve(&self, start: &Real, end: &Real) -> GeometryResult<Self> {
        let curve = self.curve.subcurve(start, end)?;
        let first_pcurve =
            self.first_pcurve
                .reparameterized_subcurve(start, end, curve.domain())?;
        let second_pcurve =
            self.second_pcurve
                .reparameterized_subcurve(start, end, curve.domain())?;
        Ok(Self::new(curve, first_pcurve, second_pcurve))
    }

    /// Returns the same exact spatial and two-operand parameter-space images
    /// with one common reversed traversal.
    pub fn reversed(&self) -> GeometryResult<Self> {
        Ok(Self::new(
            self.curve.reversed()?,
            self.first_pcurve.reversed()?,
            self.second_pcurve.reversed()?,
        ))
    }
}

impl SurfaceIntersectionPcurve {
    fn retained_curve(curve: Curve2) -> GeometryResult<Self> {
        let curve_domain = curve.parameter_domain();
        Ok(Self {
            domain: ParameterDomain::new(curve_domain.start().clone(), curve_domain.end().clone())?,
            source_scale: Real::one(),
            source_offset: Real::zero(),
            mapping: SurfaceIntersectionPcurveMapping::RetainedCurve { curve },
        })
    }

    fn plane_projection(curve: Curve3, plane: &PlaneSurface) -> Self {
        Self {
            domain: curve.domain().clone(),
            source_scale: Real::one(),
            source_offset: Real::zero(),
            mapping: SurfaceIntersectionPcurveMapping::PlaneProjection {
                spatial: curve,
                origin: plane.origin.clone(),
                u: plane.u.clone(),
                v: plane.v.clone(),
            },
        }
    }

    fn linear_plane_section(
        profile: Curve3,
        plane: &PlaneSurface,
        denominator: Real,
        profile_axis: TensorAxis,
        coefficient_scale: Real,
        coefficient_offset: Real,
    ) -> Self {
        Self {
            domain: profile.domain().clone(),
            source_scale: Real::one(),
            source_offset: Real::zero(),
            mapping: SurfaceIntersectionPcurveMapping::LinearPlaneSection {
                profile,
                plane_origin: plane.origin.clone(),
                plane_normal: plane.u.cross(&plane.v),
                denominator,
                profile_axis,
                coefficient_scale,
                coefficient_offset,
            },
        }
    }

    fn tensor_iso_v(domain: ParameterDomain, constant: Real) -> Self {
        Self {
            domain,
            source_scale: Real::one(),
            source_offset: Real::zero(),
            mapping: SurfaceIntersectionPcurveMapping::TensorIsoV { constant },
        }
    }

    fn tensor_iso(
        domain: ParameterDomain,
        constant: Real,
        profile_axis: TensorAxis,
        coefficient_scale: Real,
        coefficient_offset: Real,
    ) -> Self {
        Self {
            domain,
            source_scale: coefficient_scale,
            source_offset: coefficient_offset,
            mapping: match profile_axis {
                TensorAxis::U => SurfaceIntersectionPcurveMapping::TensorIsoU { constant },
                TensorAxis::V => SurfaceIntersectionPcurveMapping::TensorIsoV { constant },
            },
        }
    }

    /// Returns the pcurve's parameter domain, shared exactly with the spatial
    /// intersection curve.
    pub const fn domain(&self) -> &ParameterDomain {
        &self.domain
    }

    /// Evaluates the exact surface parameter corresponding to `parameter` on
    /// the retained spatial curve.
    pub fn point_at(&self, parameter: &Real) -> GeometryResult<Point2> {
        if !self.domain.contains(parameter)? {
            return Err(GeometryError::ParameterOutsideDomain);
        }
        let source_parameter = &self.source_scale * parameter + &self.source_offset;
        match &self.mapping {
            SurfaceIntersectionPcurveMapping::RetainedCurve { curve } => {
                let point = curve.point_at(&source_parameter)?;
                Ok(Point2::new(point.x().clone(), point.y().clone()))
            }
            SurfaceIntersectionPcurveMapping::PlaneProjection {
                spatial,
                origin,
                u,
                v,
            } => project_point_to_plane_frame(origin, u, v, &spatial.point_at(&source_parameter)?),
            SurfaceIntersectionPcurveMapping::LinearPlaneSection {
                profile,
                plane_origin,
                plane_normal,
                denominator,
                profile_axis,
                coefficient_scale,
                coefficient_offset,
                ..
            } => {
                let profile_point = profile.point_at(&source_parameter)?;
                let normalized_coefficient = (plane_normal.dot(&(plane_origin - &profile_point))
                    / denominator)
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
                let coefficient = coefficient_offset + coefficient_scale * normalized_coefficient;
                Ok(match profile_axis {
                    TensorAxis::U => Point2::new(source_parameter, coefficient),
                    TensorAxis::V => Point2::new(coefficient, source_parameter),
                })
            }
            SurfaceIntersectionPcurveMapping::TensorIsoV { constant } => {
                Ok(Point2::new(source_parameter, constant.clone()))
            }
            SurfaceIntersectionPcurveMapping::TensorIsoU { constant } => {
                Ok(Point2::new(constant.clone(), source_parameter))
            }
        }
    }

    /// Restricts this exact surface-parameter mapping without changing its
    /// authoritative spatial-curve parameterization.
    pub fn subcurve(&self, start: &Real, end: &Real) -> GeometryResult<Self> {
        if !self.domain.contains(start)? || !self.domain.contains(end)? {
            return Err(GeometryError::ParameterOutsideDomain);
        }
        Ok(Self {
            domain: ParameterDomain::new(start.clone(), end.clone())?,
            source_scale: self.source_scale.clone(),
            source_offset: self.source_offset.clone(),
            mapping: self.mapping.clone(),
        })
    }

    /// Returns the same exact parameter-space image with reversed traversal.
    pub fn reversed(&self) -> GeometryResult<Self> {
        let domain_sum = self.domain.start() + self.domain.end();
        Ok(Self {
            domain: self.domain.clone(),
            source_scale: -self.source_scale.clone(),
            source_offset: &self.source_scale * domain_sum + &self.source_offset,
            mapping: self.mapping.clone(),
        })
    }

    /// Materializes this retained mapping as one exact Hypercurve carrier.
    ///
    /// The returned affine coefficients map
    /// `spatial_parameter = scale * pcurve_parameter + offset`. A mapping
    /// that requires more than one Hypercurve carrier remains explicitly
    /// unsupported.
    pub fn materialize(&self) -> GeometryResult<MaterializedSurfacePcurve> {
        let source_start = &self.source_scale * self.domain.start() + &self.source_offset;
        let source_end = &self.source_scale * self.domain.end() + &self.source_offset;
        let source_order = decided_order(compare_reals(&source_start, &source_end))?;
        if source_order == Ordering::Equal {
            return Err(GeometryError::InvalidParameterDomain);
        }
        let descending = source_order == Ordering::Greater;
        let (ordered_source_start, ordered_source_end) = if descending {
            (&source_end, &source_start)
        } else {
            (&source_start, &source_end)
        };
        let orient_curve = |curve: Curve2| -> GeometryResult<Curve2> {
            if descending {
                curve.reversed().map_err(GeometryError::from)
            } else {
                Ok(curve)
            }
        };
        match &self.mapping {
            SurfaceIntersectionPcurveMapping::RetainedCurve { curve } => {
                let domain = curve.parameter_domain();
                let restricted =
                    if decided_order(compare_reals(ordered_source_start, domain.start()))?
                        == Ordering::Equal
                        && decided_order(compare_reals(ordered_source_end, domain.end()))?
                            == Ordering::Equal
                    {
                        curve.clone()
                    } else {
                        curve.subcurve(ordered_source_start.clone(), ordered_source_end.clone())?
                    };
                materialized_surface_pcurve_from_matching_domains(
                    orient_curve(restricted)?,
                    &self.domain,
                )
            }
            SurfaceIntersectionPcurveMapping::PlaneProjection {
                spatial,
                origin,
                u,
                v,
            } => {
                let source_curve = spatial.subcurve(ordered_source_start, ordered_source_end)?;
                let curve = orient_curve(
                    project_curve_to_plane_frame(&source_curve, origin, u, v)?
                        .ok_or(GeometryError::UnsupportedIntersection)?,
                )?;
                if curve.family() == CurveFamily2::CircularArc {
                    Ok(MaterializedSurfacePcurve {
                        curve,
                        correspondence: SurfacePcurveCorrespondence::AngularSweep {
                            spatial_start: self.domain.start().clone(),
                            spatial_end: self.domain.end().clone(),
                        },
                    })
                } else {
                    materialized_surface_pcurve_from_matching_domains(curve, &self.domain)
                }
            }
            SurfaceIntersectionPcurveMapping::LinearPlaneSection {
                profile,
                plane_origin,
                plane_normal,
                denominator,
                profile_axis,
                coefficient_scale,
                coefficient_offset,
            } => {
                let carriers = linear_section_pcurve_carriers(
                    profile,
                    plane_origin,
                    plane_normal,
                    denominator,
                    *profile_axis,
                    coefficient_scale,
                    coefficient_offset,
                )?
                .ok_or(GeometryError::UnsupportedIntersection)?;
                let mut curves = Vec::new();
                let mut boundaries = Vec::new();
                for carrier in carriers {
                    let carrier_start = carrier.spatial_offset.clone();
                    let carrier_end = &carrier.spatial_offset + &carrier.spatial_scale;
                    let overlap_start =
                        if decided_order(compare_reals(ordered_source_start, &carrier_start))?
                            == Ordering::Greater
                        {
                            ordered_source_start.clone()
                        } else {
                            carrier_start
                        };
                    let overlap_end =
                        if decided_order(compare_reals(ordered_source_end, &carrier_end))?
                            == Ordering::Less
                        {
                            ordered_source_end.clone()
                        } else {
                            carrier_end
                        };
                    if decided_order(compare_reals(&overlap_start, &overlap_end))? != Ordering::Less
                    {
                        continue;
                    }
                    let start = ((&overlap_start - &carrier.spatial_offset)
                        / &carrier.spatial_scale)
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let end = ((&overlap_end - &carrier.spatial_offset) / &carrier.spatial_scale)
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    let domain = carrier.curve.parameter_domain();
                    let curve = if decided_order(compare_reals(&start, domain.start()))?
                        == Ordering::Equal
                        && decided_order(compare_reals(&end, domain.end()))? == Ordering::Equal
                    {
                        carrier.curve
                    } else {
                        carrier.curve.subcurve(start, end)?
                    };
                    if boundaries.is_empty() {
                        boundaries.push(overlap_start);
                    }
                    boundaries.push(overlap_end);
                    curves.push(curve);
                }
                match curves.as_slice() {
                    [] => Err(GeometryError::UnsupportedIntersection),
                    [curve] => materialized_surface_pcurve_from_matching_domains(
                        orient_curve(curve.clone())?,
                        &self.domain,
                    ),
                    _ => materialized_surface_pcurve_from_matching_domains(
                        orient_curve(concatenate_rational_bezier_spans_as_nurbs(
                            &curves,
                            &boundaries,
                        )?)?,
                        &self.domain,
                    ),
                }
            }
            SurfaceIntersectionPcurveMapping::TensorIsoV { constant } => {
                let curve = orient_curve(Curve2::from(LineSeg2::try_new(
                    CurvePoint2::new(ordered_source_start.clone(), constant.clone()),
                    CurvePoint2::new(ordered_source_end.clone(), constant.clone()),
                )?))?;
                materialized_surface_pcurve_from_matching_domains(curve, &self.domain)
            }
            SurfaceIntersectionPcurveMapping::TensorIsoU { constant } => {
                let curve = orient_curve(Curve2::from(LineSeg2::try_new(
                    CurvePoint2::new(constant.clone(), ordered_source_start.clone()),
                    CurvePoint2::new(constant.clone(), ordered_source_end.clone()),
                )?))?;
                materialized_surface_pcurve_from_matching_domains(curve, &self.domain)
            }
        }
    }

    fn reparameterized_subcurve(
        &self,
        start: &Real,
        end: &Real,
        target_domain: &ParameterDomain,
    ) -> GeometryResult<Self> {
        if !self.domain.contains(start)? || !self.domain.contains(end)? {
            return Err(GeometryError::ParameterOutsideDomain);
        }
        let source_start = &self.source_scale * start + &self.source_offset;
        let source_end = &self.source_scale * end + &self.source_offset;
        let target_span = target_domain.end() - target_domain.start();
        let source_scale = ((&source_end - &source_start) / target_span)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let source_offset = source_start - &source_scale * target_domain.start();
        Ok(Self {
            domain: target_domain.clone(),
            source_scale,
            source_offset,
            mapping: self.mapping.clone(),
        })
    }

    pub(crate) fn clipping_carriers(
        &self,
    ) -> GeometryResult<Option<Vec<SurfacePcurveClipCarrier>>> {
        let identity = |curve| SurfacePcurveClipCarrier {
            curve,
            spatial_scale: Real::one(),
            spatial_offset: Real::zero(),
        };
        match &self.mapping {
            SurfaceIntersectionPcurveMapping::RetainedCurve { curve } => {
                Ok(Some(vec![identity(curve.clone())]))
            }
            SurfaceIntersectionPcurveMapping::PlaneProjection {
                spatial,
                origin,
                u,
                v,
            } => {
                let source_start = &self.source_scale * self.domain.start() + &self.source_offset;
                let source_end = &self.source_scale * self.domain.end() + &self.source_offset;
                let descending =
                    decided_order(compare_reals(&source_start, &source_end))? == Ordering::Greater;
                let source_curve = if descending {
                    spatial.subcurve(&source_end, &source_start)?
                } else {
                    spatial.subcurve(&source_start, &source_end)?
                };
                let Some(mut curve) = project_curve_to_plane_frame(&source_curve, origin, u, v)?
                else {
                    return Ok(None);
                };
                if descending {
                    curve = curve.reversed()?;
                }
                let curve_domain = curve.parameter_domain();
                let curve_span = curve_domain.end() - curve_domain.start();
                let spatial_span = self.domain.end() - self.domain.start();
                let spatial_scale =
                    (&spatial_span / &curve_span).map_err(|_| GeometryError::ProjectiveDivision)?;
                let spatial_offset = self.domain.start() - &spatial_scale * curve_domain.start();
                Ok(Some(vec![SurfacePcurveClipCarrier {
                    curve,
                    spatial_scale,
                    spatial_offset,
                }]))
            }
            SurfaceIntersectionPcurveMapping::LinearPlaneSection {
                profile,
                plane_origin,
                plane_normal,
                denominator,
                profile_axis,
                coefficient_scale,
                coefficient_offset,
            } => linear_section_pcurve_carriers(
                profile,
                plane_origin,
                plane_normal,
                denominator,
                *profile_axis,
                coefficient_scale,
                coefficient_offset,
            ),
            SurfaceIntersectionPcurveMapping::TensorIsoV { constant } => {
                let span = self.domain.end() - self.domain.start();
                Ok(Some(vec![SurfacePcurveClipCarrier {
                    curve: Curve2::from(LineSeg2::try_new(
                        CurvePoint2::new(self.domain.start().clone(), constant.clone()),
                        CurvePoint2::new(self.domain.end().clone(), constant.clone()),
                    )?),
                    spatial_scale: span,
                    spatial_offset: self.domain.start().clone(),
                }]))
            }
            SurfaceIntersectionPcurveMapping::TensorIsoU { constant } => {
                let span = self.domain.end() - self.domain.start();
                Ok(Some(vec![SurfacePcurveClipCarrier {
                    curve: Curve2::from(LineSeg2::try_new(
                        CurvePoint2::new(constant.clone(), self.domain.start().clone()),
                        CurvePoint2::new(constant.clone(), self.domain.end().clone()),
                    )?),
                    spatial_scale: span,
                    spatial_offset: self.domain.start().clone(),
                }]))
            }
        }
    }
}

impl MaterializedSurfacePcurve {
    /// Returns the exact Hypercurve carrier.
    pub const fn curve(&self) -> &Curve2 {
        &self.curve
    }

    /// Returns the exact pcurve-to-spatial parameter correspondence.
    pub const fn correspondence(&self) -> &SurfacePcurveCorrespondence {
        &self.correspondence
    }

    /// Maps one pcurve parameter to the authoritative spatial-curve parameter.
    pub fn spatial_parameter_at(&self, parameter: &Real) -> GeometryResult<Real> {
        match &self.correspondence {
            SurfacePcurveCorrespondence::Affine { scale, offset } => Ok(scale * parameter + offset),
            SurfacePcurveCorrespondence::AngularSweep {
                spatial_start,
                spatial_end,
            } => {
                let CurveGeometry2::CircularArc(arc) = self.curve.geometry() else {
                    return Err(GeometryError::UnsupportedPcurveContour);
                };
                let point = self.curve.point_at(parameter)?;
                let point = CurvePoint2::new(point.x().clone(), point.y().clone());
                let fraction = match arc.sweep_fraction(&point, &CurvePolicy::certified())? {
                    Classification::Decided(fraction) => fraction,
                    Classification::Uncertain(reason) => {
                        return Err(GeometryError::PlanarClassificationUnresolved(reason));
                    }
                };
                Ok(spatial_start + (spatial_end - spatial_start) * fraction)
            }
        }
    }
}

fn materialized_surface_pcurve_from_matching_domains(
    curve: Curve2,
    target_domain: &ParameterDomain,
) -> GeometryResult<MaterializedSurfacePcurve> {
    let curve_domain = curve.parameter_domain();
    let curve_start = curve_domain.start().clone();
    let curve_span = curve_domain.end() - &curve_start;
    let target_span = target_domain.end() - target_domain.start();
    let spatial_scale =
        (target_span / curve_span).map_err(|_| GeometryError::ProjectiveDivision)?;
    let spatial_offset = target_domain.start() - &spatial_scale * curve_start;
    Ok(MaterializedSurfacePcurve {
        curve,
        correspondence: SurfacePcurveCorrespondence::Affine {
            scale: spatial_scale,
            offset: spatial_offset,
        },
    })
}

/// Exact unbounded line produced by a surface/surface intersection.
#[derive(Clone, Debug)]
pub struct SurfaceIntersectionLine {
    /// One exact point on the line.
    pub point: Point3,
    /// Exact nonzero line direction.
    pub direction: Vector3,
}

/// Exact lower-bounded ray produced by a surface/surface intersection.
#[derive(Clone, Debug)]
pub struct SurfaceIntersectionRay {
    /// Exact point at ray parameter zero.
    pub point: Point3,
    /// Exact nonzero direction for increasing parameters.
    pub direction: Vector3,
    /// Authoritative lower bound of the ray parameter.
    pub minimum: Real,
    first_pcurve: SurfaceIntersectionParameterRay,
    second_pcurve: SurfaceIntersectionParameterRay,
}

/// Exact affine surface-parameter image of an intersection ray.
#[derive(Clone, Debug)]
pub struct SurfaceIntersectionParameterRay {
    origin: Point2,
    direction: Vector2,
}

impl SurfaceIntersectionRay {
    fn new(
        point: Point3,
        direction: Vector3,
        minimum: Real,
        first_pcurve: SurfaceIntersectionParameterRay,
        second_pcurve: SurfaceIntersectionParameterRay,
    ) -> Self {
        Self {
            point,
            direction,
            minimum,
            first_pcurve,
            second_pcurve,
        }
    }

    fn swapped(mut self) -> Self {
        std::mem::swap(&mut self.first_pcurve, &mut self.second_pcurve);
        self
    }

    /// Returns the exact affine parameter ray on one surface operand.
    pub const fn pcurve(
        &self,
        operand: SurfaceIntersectionOperand,
    ) -> &SurfaceIntersectionParameterRay {
        match operand {
            SurfaceIntersectionOperand::First => &self.first_pcurve,
            SurfaceIntersectionOperand::Second => &self.second_pcurve,
        }
    }
}

impl SurfaceIntersectionParameterRay {
    fn new(origin: Point2, direction: Vector2) -> Self {
        Self { origin, direction }
    }

    /// Returns the surface parameter at one authoritative ray parameter.
    pub fn point_at(&self, parameter: &Real) -> Point2 {
        self.origin.clone() + self.direction.clone() * parameter
    }

    /// Returns the exact surface-parameter origin.
    pub const fn origin(&self) -> &Point2 {
        &self.origin
    }

    /// Returns the exact surface-parameter direction.
    pub const fn direction(&self) -> &Vector2 {
        &self.direction
    }
}

impl SurfacePartials {
    /// Returns the partial derivative in the surface `u` direction.
    pub const fn u(&self) -> &Vector3 {
        &self.u
    }

    /// Returns the partial derivative in the surface `v` direction.
    pub const fn v(&self) -> &Vector3 {
        &self.v
    }
}

/// Immutable exact parametric surface.
#[derive(Clone, Debug)]
pub struct Surface {
    data: Arc<SurfaceData>,
}

#[derive(Debug)]
struct SurfaceData {
    geometry: SurfaceGeometry,
    domain: SurfaceDomain,
    bounds: OnceLock<Result<SurfaceBounds, GeometryError>>,
}

#[derive(Debug)]
enum SurfaceGeometry {
    Plane(PlaneSurface),
    Cylinder(CylinderSurface),
    Sphere(SphereSurface),
    Cone(ConeSurface),
    Torus(TorusSurface),
    Extrusion(ExtrusionSurface),
    Revolution(RevolutionSurface),
    RationalBezier(RationalBezierSurface),
    Nurbs(NurbsSurface),
}

#[derive(Debug)]
struct PlaneSurface {
    origin: Point3,
    u: Vector3,
    v: Vector3,
}

#[derive(Debug)]
struct OrthonormalFrame3 {
    x: Vector3,
    y: Vector3,
    z: Vector3,
}

#[derive(Debug)]
struct CylinderSurface {
    origin: Point3,
    frame: OrthonormalFrame3,
    radius: Real,
}

#[derive(Debug)]
struct SphereSurface {
    center: Point3,
    frame: OrthonormalFrame3,
    radius: Real,
}

#[derive(Debug)]
struct ConeSurface {
    apex: Point3,
    frame: OrthonormalFrame3,
    semi_angle: Real,
}

#[derive(Debug)]
struct TorusSurface {
    center: Point3,
    frame: OrthonormalFrame3,
    major_radius: Real,
    minor_radius: Real,
}

#[derive(Debug)]
struct ExtrusionSurface {
    profile: Curve3,
    direction: Vector3,
}

#[derive(Debug)]
struct RevolutionSurface {
    profile: Curve3,
    axis_origin: Point3,
    axis: Vector3,
}

#[derive(Debug)]
struct RationalBezierSurface {
    control_points: Vec<Vec<Point3>>,
    weights: Vec<Vec<Real>>,
    homogeneous_controls: OnceLock<Vec<Vec<HomogeneousPoint3>>>,
}

#[derive(Debug)]
struct NurbsSurface {
    u_degree: usize,
    v_degree: usize,
    control_points: Vec<Vec<Point3>>,
    weights: Vec<Vec<Real>>,
    u_knots: Vec<Real>,
    v_knots: Vec<Real>,
    homogeneous_controls: OnceLock<Vec<Vec<HomogeneousPoint3>>>,
}

impl Surface {
    /// Constructs an unbounded exact plane with an authored parameter frame.
    pub fn plane(origin: Point3, u: Vector3, v: Vector3) -> GeometryResult<Self> {
        let normal_norm = u.cross(&v).norm_squared();
        match compare_reals(&normal_norm, &Real::zero()) {
            PredicateOutcome::Decided {
                value: Ordering::Greater,
                ..
            } => Ok(Self::from_parts(
                SurfaceGeometry::Plane(PlaneSurface { origin, u, v }),
                SurfaceDomain {
                    u: SurfaceParameterDomain::Unbounded,
                    v: SurfaceParameterDomain::Unbounded,
                },
            )),
            PredicateOutcome::Decided { .. } => Err(GeometryError::DegeneratePlaneBasis),
            PredicateOutcome::Unknown { needed, stage } => {
                Err(GeometryError::PredicateUnresolved { needed, stage })
            }
        }
    }

    /// Constructs a circular cylinder with periodic angle `u` and axial `v`.
    pub fn cylinder(
        origin: Point3,
        x: Vector3,
        y: Vector3,
        axis: Vector3,
        radius: Real,
    ) -> GeometryResult<Self> {
        let frame = validate_orthonormal_frame(x, y, axis)?;
        require_positive(&radius, GeometryError::InvalidRadius)?;
        Ok(Self::from_parts(
            SurfaceGeometry::Cylinder(CylinderSurface {
                origin,
                frame,
                radius,
            }),
            angle_unbounded_domain(),
        ))
    }

    /// Constructs a sphere with longitude `u` and latitude `v`.
    pub fn sphere(
        center: Point3,
        x: Vector3,
        y: Vector3,
        axis: Vector3,
        radius: Real,
    ) -> GeometryResult<Self> {
        let frame = validate_orthonormal_frame(x, y, axis)?;
        require_positive(&radius, GeometryError::InvalidRadius)?;
        let half_pi =
            (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
        Ok(Self::from_parts(
            SurfaceGeometry::Sphere(SphereSurface {
                center,
                frame,
                radius,
            }),
            SurfaceDomain {
                u: angle_domain(),
                v: SurfaceParameterDomain::Closed(ParameterDomain::new(-half_pi.clone(), half_pi)?),
            },
        ))
    }

    /// Constructs a right circular cone with nonnegative radial-height `v`.
    pub fn cone(
        apex: Point3,
        x: Vector3,
        y: Vector3,
        axis: Vector3,
        semi_angle: Real,
    ) -> GeometryResult<Self> {
        let frame = validate_orthonormal_frame(x, y, axis)?;
        let half_pi =
            (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)?;
        if decided_order(compare_reals(&semi_angle, &Real::zero()))? != Ordering::Greater
            || decided_order(compare_reals(&semi_angle, &half_pi))? != Ordering::Less
        {
            return Err(GeometryError::InvalidConeAngle);
        }
        Ok(Self::from_parts(
            SurfaceGeometry::Cone(ConeSurface {
                apex,
                frame,
                semi_angle,
            }),
            SurfaceDomain {
                u: angle_domain(),
                v: SurfaceParameterDomain::LowerBounded {
                    start: Real::zero(),
                },
            },
        ))
    }

    /// Constructs a non-self-intersecting ring torus.
    pub fn torus(
        center: Point3,
        x: Vector3,
        y: Vector3,
        axis: Vector3,
        major_radius: Real,
        minor_radius: Real,
    ) -> GeometryResult<Self> {
        let frame = validate_orthonormal_frame(x, y, axis)?;
        require_positive(&minor_radius, GeometryError::InvalidTorusRadii)?;
        if decided_order(compare_reals(&major_radius, &minor_radius))? != Ordering::Greater {
            return Err(GeometryError::InvalidTorusRadii);
        }
        Ok(Self::from_parts(
            SurfaceGeometry::Torus(TorusSurface {
                center,
                frame,
                major_radius,
                minor_radius,
            }),
            SurfaceDomain {
                u: angle_domain(),
                v: angle_domain(),
            },
        ))
    }

    /// Constructs the unbounded linear extrusion of a finite profile curve.
    ///
    /// `u` is the profile parameter and `v` is the signed coefficient of the
    /// authored direction: `S(u, v) = profile(u) + v * direction`.
    pub fn extrusion(profile: Curve3, direction: Vector3) -> GeometryResult<Self> {
        if decided_order(compare_reals(&direction.norm_squared(), &Real::zero()))?
            != Ordering::Greater
        {
            return Err(GeometryError::DegenerateExtrusionDirection);
        }
        let u_domain = profile.domain().clone();
        Ok(Self::from_parts(
            SurfaceGeometry::Extrusion(ExtrusionSurface { profile, direction }),
            SurfaceDomain {
                u: SurfaceParameterDomain::Closed(u_domain),
                v: SurfaceParameterDomain::Unbounded,
            },
        ))
    }

    /// Constructs the full revolution of a finite profile around a unit axis.
    ///
    /// `u` is the periodic revolution angle and `v` is the profile parameter.
    /// Profile points on the axis remain explicit parameter singularities.
    pub fn revolution(profile: Curve3, axis_origin: Point3, axis: Vector3) -> GeometryResult<Self> {
        if decided_order(compare_reals(&axis.norm_squared(), &Real::one()))? != Ordering::Equal {
            return Err(GeometryError::InvalidRevolutionAxis);
        }
        let v_domain = profile.domain().clone();
        Ok(Self::from_parts(
            SurfaceGeometry::Revolution(RevolutionSurface {
                profile,
                axis_origin,
                axis,
            }),
            SurfaceDomain {
                u: angle_domain(),
                v: SurfaceParameterDomain::Closed(v_domain),
            },
        ))
    }

    /// Constructs a tensor-product rational Bézier surface over `[0, 1]²`.
    ///
    /// The outer vectors are rows in increasing `v`; each row contains control
    /// points in increasing `u`. Both axes must contain at least two controls.
    pub fn rational_bezier(
        control_points: Vec<Vec<Point3>>,
        weights: Vec<Vec<Real>>,
    ) -> GeometryResult<Self> {
        let (u_count, _) = validate_surface_control_net(&control_points, &weights)?;
        validate_positive_surface_weights(&weights)?;
        debug_assert!(u_count >= 2);
        Ok(Self::from_parts(
            SurfaceGeometry::RationalBezier(RationalBezierSurface {
                control_points,
                weights,
                homogeneous_controls: OnceLock::new(),
            }),
            SurfaceDomain {
                u: SurfaceParameterDomain::Closed(ParameterDomain::unit()),
                v: SurfaceParameterDomain::Closed(ParameterDomain::unit()),
            },
        ))
    }

    /// Constructs a finite non-periodic tensor-product NURBS surface.
    ///
    /// The outer vectors are rows in increasing `v`; each row contains control
    /// points in increasing `u`. Knot vectors are clamped and nondecreasing.
    pub fn nurbs(
        u_degree: usize,
        v_degree: usize,
        control_points: Vec<Vec<Point3>>,
        weights: Vec<Vec<Real>>,
        u_knots: Vec<Real>,
        v_knots: Vec<Real>,
    ) -> GeometryResult<Self> {
        let (u_count, v_count) = validate_surface_control_net(&control_points, &weights)?;
        validate_positive_surface_weights(&weights)?;
        validate_nurbs_axis(u_degree, u_count, &u_knots)?;
        validate_nurbs_axis(v_degree, v_count, &v_knots)?;
        let u_domain = ParameterDomain::new(u_knots[u_degree].clone(), u_knots[u_count].clone())?;
        let v_domain = ParameterDomain::new(v_knots[v_degree].clone(), v_knots[v_count].clone())?;
        Ok(Self::from_parts(
            SurfaceGeometry::Nurbs(NurbsSurface {
                u_degree,
                v_degree,
                control_points,
                weights,
                u_knots,
                v_knots,
                homogeneous_controls: OnceLock::new(),
            }),
            SurfaceDomain {
                u: SurfaceParameterDomain::Closed(u_domain),
                v: SurfaceParameterDomain::Closed(v_domain),
            },
        ))
    }

    fn from_parts(geometry: SurfaceGeometry, domain: SurfaceDomain) -> Self {
        Self {
            data: Arc::new(SurfaceData {
                geometry,
                domain,
                bounds: OnceLock::new(),
            }),
        }
    }

    /// Returns the surface family.
    pub fn kind(&self) -> SurfaceKind {
        match &self.data.geometry {
            SurfaceGeometry::Plane(_) => SurfaceKind::Plane,
            SurfaceGeometry::Cylinder(_) => SurfaceKind::Cylinder,
            SurfaceGeometry::Sphere(_) => SurfaceKind::Sphere,
            SurfaceGeometry::Cone(_) => SurfaceKind::Cone,
            SurfaceGeometry::Torus(_) => SurfaceKind::Torus,
            SurfaceGeometry::Extrusion(_) => SurfaceKind::Extrusion,
            SurfaceGeometry::Revolution(_) => SurfaceKind::Revolution,
            SurfaceGeometry::RationalBezier(_) => SurfaceKind::RationalBezier,
            SurfaceGeometry::Nurbs(_) => SurfaceKind::Nurbs,
        }
    }

    pub(crate) fn canonical_plane(&self) -> GeometryResult<Option<Self>> {
        let controls = match &self.data.geometry {
            SurfaceGeometry::Plane(_) => return Ok(Some(self.clone())),
            SurfaceGeometry::RationalBezier(surface) => {
                surface.control_points.iter().flatten().collect::<Vec<_>>()
            }
            SurfaceGeometry::Nurbs(surface) => {
                surface.control_points.iter().flatten().collect::<Vec<_>>()
            }
            SurfaceGeometry::Sphere(_)
            | SurfaceGeometry::Cylinder(_)
            | SurfaceGeometry::Cone(_)
            | SurfaceGeometry::Torus(_)
            | SurfaceGeometry::Extrusion(_)
            | SurfaceGeometry::Revolution(_) => return Ok(None),
        };
        let Some(&origin) = controls.first() else {
            return Ok(None);
        };
        let mut u = None;
        for &point in controls.iter().skip(1) {
            let candidate = point - origin;
            if decided_order(compare_reals(&candidate.norm_squared(), &Real::zero()))?
                == Ordering::Greater
            {
                u = Some(candidate);
                break;
            }
        }
        let Some(u) = u else {
            return Ok(None);
        };
        let mut v = None;
        for &point in controls.iter().skip(1) {
            let candidate = point - origin;
            if decided_order(compare_reals(
                &u.cross(&candidate).norm_squared(),
                &Real::zero(),
            ))? == Ordering::Greater
            {
                v = Some(candidate);
                break;
            }
        }
        let Some(v) = v else {
            return Ok(None);
        };
        let normal = u.cross(&v);
        for &point in &controls {
            if decided_order(compare_reals(&normal.dot(&(point - origin)), &Real::zero()))?
                != Ordering::Equal
            {
                return Ok(None);
            }
        }
        Ok(Some(Self::plane(origin.clone(), u, v)?))
    }

    /// Extracts one exact tensor-product iso-curve.
    ///
    /// Rational Bézier surfaces collapse the orthogonal homogeneous control
    /// direction with de Casteljau. NURBS surfaces use homogeneous de Boor and
    /// retain the varying direction's authored degree and knot vector.
    pub fn iso_curve(&self, axis: SurfaceIsoAxis, constant: &Real) -> GeometryResult<Curve3> {
        let homogeneous_to_curve_data = |controls: Vec<HomogeneousPoint3>| {
            let mut points = Vec::with_capacity(controls.len());
            let mut weights = Vec::with_capacity(controls.len());
            for control in controls {
                weights.push(control.w.clone());
                points.push(control.project()?);
            }
            Ok::<_, GeometryError>((points, weights))
        };
        match &self.data.geometry {
            SurfaceGeometry::RationalBezier(surface) => {
                if !ParameterDomain::unit().contains(constant)? {
                    return Err(GeometryError::ParameterOutsideDomain);
                }
                let controls = surface.homogeneous_controls();
                let iso_controls = match axis {
                    SurfaceIsoAxis::U => (0..controls[0].len())
                        .map(|column| {
                            evaluate_homogeneous_bezier_value(
                                &controls
                                    .iter()
                                    .map(|row| row[column].clone())
                                    .collect::<Vec<_>>(),
                                constant,
                            )
                            .ok_or(GeometryError::InvalidControlNetShape)
                        })
                        .collect::<GeometryResult<Vec<_>>>()?,
                    SurfaceIsoAxis::V => controls
                        .iter()
                        .map(|row| {
                            evaluate_homogeneous_bezier_value(row, constant)
                                .ok_or(GeometryError::InvalidControlNetShape)
                        })
                        .collect::<GeometryResult<Vec<_>>>()?,
                };
                let (points, weights) = homogeneous_to_curve_data(iso_controls)?;
                Curve3::rational_bezier(points, weights)
            }
            SurfaceGeometry::Nurbs(surface) => {
                let (domain, degree, knots, control_count) = match axis {
                    SurfaceIsoAxis::U => (
                        match &self.data.domain.v {
                            SurfaceParameterDomain::Closed(domain) => domain,
                            _ => return Err(GeometryError::InvalidParameterDomain),
                        },
                        surface.v_degree,
                        &surface.v_knots,
                        surface.control_points.len(),
                    ),
                    SurfaceIsoAxis::V => (
                        match &self.data.domain.u {
                            SurfaceParameterDomain::Closed(domain) => domain,
                            _ => return Err(GeometryError::InvalidParameterDomain),
                        },
                        surface.u_degree,
                        &surface.u_knots,
                        surface.control_points[0].len(),
                    ),
                };
                if !domain.contains(constant)? {
                    return Err(GeometryError::ParameterOutsideDomain);
                }
                let span = find_span(constant, domain, degree, control_count, knots)?;
                let controls = surface.homogeneous_controls();
                let iso_controls = match axis {
                    SurfaceIsoAxis::U => (0..controls[0].len())
                        .map(|column| {
                            evaluate_homogeneous_de_boor_jet(
                                &controls
                                    .iter()
                                    .map(|row| row[column].clone())
                                    .collect::<Vec<_>>(),
                                &surface.v_knots,
                                surface.v_degree,
                                span,
                                constant,
                            )
                            .map(|(value, _)| value)
                        })
                        .collect::<GeometryResult<Vec<_>>>()?,
                    SurfaceIsoAxis::V => controls
                        .iter()
                        .map(|row| {
                            evaluate_homogeneous_de_boor_jet(
                                row,
                                &surface.u_knots,
                                surface.u_degree,
                                span,
                                constant,
                            )
                            .map(|(value, _)| value)
                        })
                        .collect::<GeometryResult<Vec<_>>>()?,
                };
                let (points, weights) = homogeneous_to_curve_data(iso_controls)?;
                match axis {
                    SurfaceIsoAxis::U => {
                        Curve3::nurbs(surface.u_degree, points, weights, surface.u_knots.clone())
                    }
                    SurfaceIsoAxis::V => {
                        Curve3::nurbs(surface.v_degree, points, weights, surface.v_knots.clone())
                    }
                }
            }
            _ => Err(GeometryError::UnsupportedSubdivision),
        }
    }

    /// Returns the canonical exact parameter domain.
    pub fn domain(&self) -> &SurfaceDomain {
        &self.data.domain
    }

    /// Returns conservative exact model-space bounds.
    ///
    /// Positive-weight tensor-product spline surfaces lie in the convex hull
    /// of their affine control net. Infinite analytic families are reported as
    /// unbounded; bounded analytic families use an exact enclosing cube.
    pub fn bounds(&self) -> GeometryResult<SurfaceBounds> {
        self.data
            .bounds
            .get_or_init(|| match &self.data.geometry {
                SurfaceGeometry::Plane(_)
                | SurfaceGeometry::Cylinder(_)
                | SurfaceGeometry::Cone(_)
                | SurfaceGeometry::Extrusion(_) => Ok(SurfaceBounds::Unbounded),
                SurfaceGeometry::Sphere(surface) => Ok(SurfaceBounds::Bounded(Box::new(
                    centered_cube_bounds(&surface.center, &surface.radius),
                ))),
                SurfaceGeometry::Torus(surface) => {
                    Ok(SurfaceBounds::Bounded(Box::new(centered_cube_bounds(
                        &surface.center,
                        &(&surface.major_radius + &surface.minor_radius),
                    ))))
                }
                SurfaceGeometry::Revolution(surface) => Ok(SurfaceBounds::Bounded(Box::new(
                    revolution_bounds(surface)?,
                ))),
                SurfaceGeometry::RationalBezier(surface) => Ok(SurfaceBounds::Bounded(Box::new(
                    exact_surface_control_bounds(&surface.control_points)?,
                ))),
                SurfaceGeometry::Nurbs(surface) => Ok(SurfaceBounds::Bounded(Box::new(
                    exact_surface_control_bounds(&surface.control_points)?,
                ))),
            })
            .clone()
    }

    /// Evaluates an exact model-space point.
    pub fn point_at(&self, parameter: &Point2) -> GeometryResult<Point3> {
        validate_surface_parameter(&self.data.domain, parameter)?;
        match &self.data.geometry {
            SurfaceGeometry::Plane(plane) => {
                let displacement = plane.u.clone() * &parameter.x + plane.v.clone() * &parameter.y;
                Ok(plane.origin.clone() + displacement)
            }
            SurfaceGeometry::Cylinder(surface) => {
                let (sin_u, cos_u) = (parameter.x.clone().sin(), parameter.x.clone().cos());
                Ok(surface.origin.clone()
                    + surface.frame.x.clone() * (&surface.radius * cos_u)
                    + surface.frame.y.clone() * (&surface.radius * sin_u)
                    + surface.frame.z.clone() * &parameter.y)
            }
            SurfaceGeometry::Sphere(surface) => {
                let (sin_u, cos_u) = (parameter.x.clone().sin(), parameter.x.clone().cos());
                let (sin_v, cos_v) = (parameter.y.clone().sin(), parameter.y.clone().cos());
                let radial = surface.frame.x.clone() * (&cos_v * cos_u)
                    + surface.frame.y.clone() * (&cos_v * sin_u)
                    + surface.frame.z.clone() * sin_v;
                Ok(surface.center.clone() + radial * &surface.radius)
            }
            SurfaceGeometry::Cone(surface) => {
                let (sin_u, cos_u) = (parameter.x.clone().sin(), parameter.x.clone().cos());
                let sin_angle = surface.semi_angle.clone().sin();
                let cos_angle = surface.semi_angle.clone().cos();
                let direction = surface.frame.x.clone() * (&sin_angle * cos_u)
                    + surface.frame.y.clone() * (&sin_angle * sin_u)
                    + surface.frame.z.clone() * cos_angle;
                Ok(surface.apex.clone() + direction * &parameter.y)
            }
            SurfaceGeometry::Torus(surface) => {
                let (sin_u, cos_u) = (parameter.x.clone().sin(), parameter.x.clone().cos());
                let (sin_v, cos_v) = (parameter.y.clone().sin(), parameter.y.clone().cos());
                let radial_distance = &surface.major_radius + &surface.minor_radius * &cos_v;
                Ok(surface.center.clone()
                    + surface.frame.x.clone() * (&radial_distance * cos_u)
                    + surface.frame.y.clone() * (&radial_distance * sin_u)
                    + surface.frame.z.clone() * (&surface.minor_radius * sin_v))
            }
            SurfaceGeometry::Extrusion(surface) => {
                Ok(surface.profile.point_at(&parameter.x)?
                    + surface.direction.clone() * &parameter.y)
            }
            SurfaceGeometry::Revolution(surface) => {
                let profile_point = surface.profile.point_at(&parameter.y)?;
                Ok(rotate_point_about_axis(
                    &profile_point,
                    &surface.axis_origin,
                    &surface.axis,
                    &parameter.x,
                ))
            }
            SurfaceGeometry::RationalBezier(surface) => {
                evaluate_tensor_bezier(surface.homogeneous_controls(), parameter)
            }
            SurfaceGeometry::Nurbs(surface) => {
                evaluate_tensor_nurbs(surface, &self.data.domain, parameter)
            }
        }
    }

    /// Returns exact first partial derivatives.
    pub fn partials_at(&self, parameter: &Point2) -> GeometryResult<SurfacePartials> {
        validate_surface_parameter(&self.data.domain, parameter)?;
        match &self.data.geometry {
            SurfaceGeometry::Plane(plane) => Ok(SurfacePartials {
                u: plane.u.clone(),
                v: plane.v.clone(),
            }),
            SurfaceGeometry::Cylinder(surface) => {
                let (sin_u, cos_u) = (parameter.x.clone().sin(), parameter.x.clone().cos());
                Ok(SurfacePartials {
                    u: surface.frame.x.clone() * (-&surface.radius * sin_u)
                        + surface.frame.y.clone() * (&surface.radius * cos_u),
                    v: surface.frame.z.clone(),
                })
            }
            SurfaceGeometry::Sphere(surface) => {
                let (sin_u, cos_u) = (parameter.x.clone().sin(), parameter.x.clone().cos());
                let (sin_v, cos_v) = (parameter.y.clone().sin(), parameter.y.clone().cos());
                Ok(SurfacePartials {
                    u: surface.frame.x.clone() * (-&surface.radius * &cos_v * &sin_u)
                        + surface.frame.y.clone() * (&surface.radius * &cos_v * &cos_u),
                    v: surface.frame.x.clone() * (-&surface.radius * &sin_v * &cos_u)
                        + surface.frame.y.clone() * (-&surface.radius * &sin_v * &sin_u)
                        + surface.frame.z.clone() * (&surface.radius * cos_v),
                })
            }
            SurfaceGeometry::Cone(surface) => {
                let (sin_u, cos_u) = (parameter.x.clone().sin(), parameter.x.clone().cos());
                let sin_angle = surface.semi_angle.clone().sin();
                let cos_angle = surface.semi_angle.clone().cos();
                Ok(SurfacePartials {
                    u: surface.frame.x.clone() * (-&parameter.y * &sin_angle * &sin_u)
                        + surface.frame.y.clone() * (&parameter.y * &sin_angle * &cos_u),
                    v: surface.frame.x.clone() * (&sin_angle * cos_u)
                        + surface.frame.y.clone() * (&sin_angle * sin_u)
                        + surface.frame.z.clone() * cos_angle,
                })
            }
            SurfaceGeometry::Torus(surface) => {
                let (sin_u, cos_u) = (parameter.x.clone().sin(), parameter.x.clone().cos());
                let (sin_v, cos_v) = (parameter.y.clone().sin(), parameter.y.clone().cos());
                let radial_distance = &surface.major_radius + &surface.minor_radius * &cos_v;
                Ok(SurfacePartials {
                    u: surface.frame.x.clone() * (-&radial_distance * &sin_u)
                        + surface.frame.y.clone() * (&radial_distance * &cos_u),
                    v: surface.frame.x.clone() * (-&surface.minor_radius * &sin_v * &cos_u)
                        + surface.frame.y.clone() * (-&surface.minor_radius * &sin_v * &sin_u)
                        + surface.frame.z.clone() * (&surface.minor_radius * cos_v),
                })
            }
            SurfaceGeometry::Extrusion(surface) => Ok(SurfacePartials {
                u: surface.profile.derivative_at(&parameter.x, 1)?.vector,
                v: surface.direction.clone(),
            }),
            SurfaceGeometry::Revolution(surface) => {
                let profile_point = surface.profile.point_at(&parameter.y)?;
                let profile_derivative = surface.profile.derivative_at(&parameter.y, 1)?.vector;
                let relative = &profile_point - &surface.axis_origin;
                let axial = surface.axis.clone() * surface.axis.dot(&relative);
                let radial = relative - axial;
                let sin = parameter.x.clone().sin();
                let cos = parameter.x.clone().cos();
                let u = radial.clone() * -sin + surface.axis.cross(&radial) * cos;
                let v = rotate_vector_about_axis(&profile_derivative, &surface.axis, &parameter.x);
                Ok(SurfacePartials { u, v })
            }
            SurfaceGeometry::RationalBezier(surface) => {
                evaluate_tensor_bezier_partials(surface.homogeneous_controls(), parameter)
            }
            SurfaceGeometry::Nurbs(surface) => {
                evaluate_tensor_nurbs_partials(surface, &self.data.domain, parameter)
            }
        }
    }

    /// Returns the exact unit normal induced by the authored parameter order.
    ///
    /// Poles, cone apices, revolution-axis contacts, and other rank-deficient
    /// parameters return [`GeometryError::SingularSurfaceParameter`].
    pub fn normal_at(&self, parameter: &Point2) -> GeometryResult<Vector3> {
        let partials = self.partials_at(parameter)?;
        let normal = partials.u.cross(&partials.v);
        if decided_order(compare_reals(&normal.norm_squared(), &Real::zero()))? == Ordering::Equal {
            return Err(GeometryError::SingularSurfaceParameter);
        }
        normal
            .normalize()
            .map_err(|_| GeometryError::ElementaryFunction)
    }

    /// Splits a supported finite tensor-product surface along interior `u`.
    pub fn split_u_at(&self, parameter: &Real) -> GeometryResult<(Self, Self)> {
        let SurfaceParameterDomain::Closed(domain) = self.domain().u() else {
            return Err(GeometryError::UnsupportedSubdivision);
        };
        require_interior_parameter(domain, parameter)?;
        match &self.data.geometry {
            SurfaceGeometry::RationalBezier(surface) => {
                split_rational_bezier_surface_u(surface, parameter)
            }
            SurfaceGeometry::Nurbs(surface) => split_nurbs_surface_u(surface, parameter),
            _ => Err(GeometryError::UnsupportedSubdivision),
        }
    }

    /// Splits a supported finite tensor-product surface along interior `v`.
    pub fn split_v_at(&self, parameter: &Real) -> GeometryResult<(Self, Self)> {
        let SurfaceParameterDomain::Closed(domain) = self.domain().v() else {
            return Err(GeometryError::UnsupportedSubdivision);
        };
        require_interior_parameter(domain, parameter)?;
        match &self.data.geometry {
            SurfaceGeometry::RationalBezier(surface) => {
                split_rational_bezier_surface_v(surface, parameter)
            }
            SurfaceGeometry::Nurbs(surface) => split_nurbs_surface_v(surface, parameter),
            _ => Err(GeometryError::UnsupportedSubdivision),
        }
    }

    /// Returns the authored plane origin when this is a plane.
    pub fn plane_origin(&self) -> Option<&Point3> {
        match &self.data.geometry {
            SurfaceGeometry::Plane(plane) => Some(&plane.origin),
            SurfaceGeometry::Cylinder(_)
            | SurfaceGeometry::Sphere(_)
            | SurfaceGeometry::Cone(_)
            | SurfaceGeometry::Torus(_)
            | SurfaceGeometry::Extrusion(_)
            | SurfaceGeometry::Revolution(_)
            | SurfaceGeometry::RationalBezier(_)
            | SurfaceGeometry::Nurbs(_) => None,
        }
    }

    /// Returns the authored plane parameter directions when this is a plane.
    pub fn plane_directions(&self) -> Option<(&Vector3, &Vector3)> {
        match &self.data.geometry {
            SurfaceGeometry::Plane(plane) => Some((&plane.u, &plane.v)),
            SurfaceGeometry::Cylinder(_)
            | SurfaceGeometry::Sphere(_)
            | SurfaceGeometry::Cone(_)
            | SurfaceGeometry::Torus(_)
            | SurfaceGeometry::Extrusion(_)
            | SurfaceGeometry::Revolution(_)
            | SurfaceGeometry::RationalBezier(_)
            | SurfaceGeometry::Nurbs(_) => None,
        }
    }

    pub(crate) fn exact_data(&self) -> SurfaceExactData {
        match &self.data.geometry {
            SurfaceGeometry::Plane(surface) => SurfaceExactData::Plane {
                origin: surface.origin.clone(),
                u: surface.u.clone(),
                v: surface.v.clone(),
            },
            SurfaceGeometry::Cylinder(surface) => SurfaceExactData::Cylinder {
                origin: surface.origin.clone(),
                x: surface.frame.x.clone(),
                y: surface.frame.y.clone(),
                axis: surface.frame.z.clone(),
                radius: surface.radius.clone(),
            },
            SurfaceGeometry::Sphere(surface) => SurfaceExactData::Sphere {
                center: surface.center.clone(),
                x: surface.frame.x.clone(),
                y: surface.frame.y.clone(),
                axis: surface.frame.z.clone(),
                radius: surface.radius.clone(),
            },
            SurfaceGeometry::Cone(surface) => SurfaceExactData::Cone {
                apex: surface.apex.clone(),
                x: surface.frame.x.clone(),
                y: surface.frame.y.clone(),
                axis: surface.frame.z.clone(),
                semi_angle: surface.semi_angle.clone(),
            },
            SurfaceGeometry::Torus(surface) => SurfaceExactData::Torus {
                center: surface.center.clone(),
                x: surface.frame.x.clone(),
                y: surface.frame.y.clone(),
                axis: surface.frame.z.clone(),
                major_radius: surface.major_radius.clone(),
                minor_radius: surface.minor_radius.clone(),
            },
            SurfaceGeometry::Extrusion(surface) => SurfaceExactData::Extrusion {
                profile: Box::new(surface.profile.exact_data()),
                direction: surface.direction.clone(),
            },
            SurfaceGeometry::Revolution(surface) => SurfaceExactData::Revolution {
                profile: Box::new(surface.profile.exact_data()),
                axis_origin: surface.axis_origin.clone(),
                axis: surface.axis.clone(),
            },
            SurfaceGeometry::RationalBezier(surface) => SurfaceExactData::RationalBezier {
                control_points: surface.control_points.clone(),
                weights: surface.weights.clone(),
            },
            SurfaceGeometry::Nurbs(surface) => SurfaceExactData::Nurbs {
                u_degree: surface.u_degree,
                v_degree: surface.v_degree,
                control_points: surface.control_points.clone(),
                weights: surface.weights.clone(),
                u_knots: surface.u_knots.clone(),
                v_knots: surface.v_knots.clone(),
            },
        }
    }

    pub(crate) fn extrusion_profile_and_direction(&self) -> Option<(&Curve3, &Vector3)> {
        match &self.data.geometry {
            SurfaceGeometry::Extrusion(surface) => Some((&surface.profile, &surface.direction)),
            _ => None,
        }
    }

    pub(crate) fn revolution_meridian_curve(&self, angle: &Real) -> GeometryResult<Curve3> {
        let SurfaceGeometry::Revolution(surface) = &self.data.geometry else {
            return Err(GeometryError::UnsupportedSubdivision);
        };
        let rotate_point = |point: &Point3| {
            rotate_point_about_axis(point, &surface.axis_origin, &surface.axis, angle)
        };
        let rotate_vector =
            |vector: &Vector3| rotate_vector_about_axis(vector, &surface.axis, angle);
        match surface.profile.exact_data() {
            Curve3ExactData::Line(line) => {
                Curve3::line(rotate_point(&line.start), rotate_point(&line.end))
            }
            Curve3ExactData::RationalBezier {
                control_points,
                weights,
            } => Curve3::rational_bezier(
                control_points.iter().map(rotate_point).collect::<Vec<_>>(),
                weights,
            ),
            Curve3ExactData::Nurbs {
                degree,
                control_points,
                weights,
                knots,
            } => Curve3::nurbs(
                degree,
                control_points.iter().map(rotate_point).collect::<Vec<_>>(),
                weights,
                knots,
            ),
            Curve3ExactData::EllipseArc(data) => Curve3::from_exact_data(
                Curve3ExactData::EllipseArc(Box::new(EllipseArcExactData {
                    circle: data.circle,
                    center: rotate_point(&data.center),
                    x: rotate_vector(&data.x),
                    y: rotate_vector(&data.y),
                    x_radius: data.x_radius,
                    y_radius: data.y_radius,
                    domain_start: data.domain_start,
                    domain_end: data.domain_end,
                    angle_at_start: data.angle_at_start,
                    direction: data.direction,
                })),
            ),
        }
    }

    pub(crate) fn from_exact_data(data: SurfaceExactData) -> GeometryResult<Self> {
        match data {
            SurfaceExactData::Plane { origin, u, v } => Self::plane(origin, u, v),
            SurfaceExactData::Cylinder {
                origin,
                x,
                y,
                axis,
                radius,
            } => Self::cylinder(origin, x, y, axis, radius),
            SurfaceExactData::Sphere {
                center,
                x,
                y,
                axis,
                radius,
            } => Self::sphere(center, x, y, axis, radius),
            SurfaceExactData::Cone {
                apex,
                x,
                y,
                axis,
                semi_angle,
            } => Self::cone(apex, x, y, axis, semi_angle),
            SurfaceExactData::Torus {
                center,
                x,
                y,
                axis,
                major_radius,
                minor_radius,
            } => Self::torus(center, x, y, axis, major_radius, minor_radius),
            SurfaceExactData::Extrusion { profile, direction } => {
                Self::extrusion(Curve3::from_exact_data(*profile)?, direction)
            }
            SurfaceExactData::Revolution {
                profile,
                axis_origin,
                axis,
            } => Self::revolution(Curve3::from_exact_data(*profile)?, axis_origin, axis),
            SurfaceExactData::RationalBezier {
                control_points,
                weights,
            } => Self::rational_bezier(control_points, weights),
            SurfaceExactData::Nurbs {
                u_degree,
                v_degree,
                control_points,
                weights,
                u_knots,
                v_knots,
            } => Self::nurbs(
                u_degree,
                v_degree,
                control_points,
                weights,
                u_knots,
                v_knots,
            ),
        }
    }

    /// Intersects this surface with a finite spatial curve.
    pub fn intersect_curve(&self, curve: &Curve3) -> GeometryResult<CurveSurfaceIntersection> {
        if let (
            SurfaceGeometry::Plane(plane),
            CurveGeometry3::CircleArc(arc) | CurveGeometry3::EllipseArc(arc),
        ) = (&self.data.geometry, &curve.data.geometry)
        {
            return intersect_ellipse_arc_plane(curve, arc, plane);
        }
        if let (SurfaceGeometry::Plane(plane), CurveGeometry3::RationalBezier(bezier)) =
            (&self.data.geometry, &curve.data.geometry)
        {
            return intersect_rational_bezier_plane(curve, bezier, plane);
        }
        if let (SurfaceGeometry::Plane(plane), CurveGeometry3::Nurbs(nurbs)) =
            (&self.data.geometry, &curve.data.geometry)
        {
            return intersect_nurbs_plane(nurbs, plane);
        }
        if let (SurfaceGeometry::Sphere(sphere), CurveGeometry3::CircleArc(arc)) =
            (&self.data.geometry, &curve.data.geometry)
        {
            return intersect_circle_arc_sphere(curve, arc, sphere);
        }
        if let (SurfaceGeometry::Cylinder(cylinder), CurveGeometry3::CircleArc(arc)) =
            (&self.data.geometry, &curve.data.geometry)
        {
            return intersect_transverse_circle_arc_cylinder(curve, arc, cylinder);
        }
        if let (SurfaceGeometry::Cone(cone), CurveGeometry3::CircleArc(arc)) =
            (&self.data.geometry, &curve.data.geometry)
        {
            return intersect_transverse_circle_arc_cone(curve, arc, cone);
        }
        if let (SurfaceGeometry::Torus(torus), CurveGeometry3::CircleArc(arc)) =
            (&self.data.geometry, &curve.data.geometry)
        {
            return intersect_transverse_circle_arc_torus(curve, arc, torus);
        }
        let Some((start, end)) = curve.line_endpoints() else {
            return Err(GeometryError::UnsupportedIntersection);
        };
        match &self.data.geometry {
            SurfaceGeometry::Plane(plane) => {
                let normal = plane.u.cross(&plane.v);
                let direction = end - start;
                let denominator = normal.dot(&direction);
                let start_value = normal.dot(&(start - &plane.origin));
                match decided_order(compare_reals(&denominator, &Real::zero()))? {
                    Ordering::Equal => {
                        if decided_order(compare_reals(&start_value, &Real::zero()))?
                            == Ordering::Equal
                        {
                            Ok(CurveSurfaceIntersection::Contained)
                        } else {
                            Ok(CurveSurfaceIntersection::None)
                        }
                    }
                    Ordering::Less | Ordering::Greater => {
                        let parameter = ((-start_value) / denominator)
                            .map_err(|_| GeometryError::ProjectiveDivision)?;
                        isolated_line_parameters(
                            curve,
                            [(parameter, IntersectionMultiplicity::Simple)],
                        )
                    }
                }
            }
            SurfaceGeometry::Sphere(surface) => {
                let offset = start - &surface.center;
                quadratic_line_intersection(curve, &offset, &(end - start), &surface.radius)
            }
            SurfaceGeometry::Cylinder(surface) => {
                let offset = start - &surface.origin;
                let direction = end - start;
                let radial_offset =
                    &offset - &(surface.frame.z.clone() * offset.dot(&surface.frame.z));
                let radial_direction =
                    &direction - &(surface.frame.z.clone() * direction.dot(&surface.frame.z));
                quadratic_line_intersection(
                    curve,
                    &radial_offset,
                    &radial_direction,
                    &surface.radius,
                )
            }
            SurfaceGeometry::Cone(surface) => intersect_line_cone(curve, start, end, surface),
            SurfaceGeometry::Torus(_)
            | SurfaceGeometry::Extrusion(_)
            | SurfaceGeometry::Revolution(_)
            | SurfaceGeometry::RationalBezier(_)
            | SurfaceGeometry::Nurbs(_) => Err(GeometryError::UnsupportedIntersection),
        }
    }

    /// Intersects this surface with another supported surface.
    pub fn intersect_surface(&self, other: &Self) -> GeometryResult<SurfaceSurfaceIntersection> {
        match (&self.data.geometry, &other.data.geometry) {
            (SurfaceGeometry::Plane(first), SurfaceGeometry::Plane(second)) => {
                intersect_planes(first, second)
            }
            (SurfaceGeometry::Plane(plane), SurfaceGeometry::Sphere(sphere)) => {
                intersect_plane_sphere(plane, sphere)
            }
            (SurfaceGeometry::Sphere(sphere), SurfaceGeometry::Plane(plane)) => {
                intersect_plane_sphere(plane, sphere).map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Plane(plane), SurfaceGeometry::Cylinder(cylinder)) => {
                intersect_plane_cylinder(plane, cylinder)
            }
            (SurfaceGeometry::Cylinder(cylinder), SurfaceGeometry::Plane(plane)) => {
                intersect_plane_cylinder(plane, cylinder).map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Plane(plane), SurfaceGeometry::Cone(cone)) => {
                intersect_plane_cone(plane, cone)
            }
            (SurfaceGeometry::Cone(cone), SurfaceGeometry::Plane(plane)) => {
                intersect_plane_cone(plane, cone).map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Plane(plane), SurfaceGeometry::Torus(torus)) => {
                intersect_plane_torus(plane, torus)
            }
            (SurfaceGeometry::Torus(torus), SurfaceGeometry::Plane(plane)) => {
                intersect_plane_torus(plane, torus).map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Plane(plane), SurfaceGeometry::Extrusion(surface)) => {
                intersect_plane_extrusion(plane, surface)
            }
            (SurfaceGeometry::Extrusion(surface), SurfaceGeometry::Plane(plane)) => {
                intersect_plane_extrusion(plane, surface).map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Plane(plane), SurfaceGeometry::Revolution(surface)) => {
                intersect_plane_revolution(plane, surface)
            }
            (SurfaceGeometry::Revolution(surface), SurfaceGeometry::Plane(plane)) => {
                intersect_plane_revolution(plane, surface).map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Plane(plane), SurfaceGeometry::RationalBezier(surface)) => {
                intersect_plane_rational_bezier_surface(plane, surface)
            }
            (SurfaceGeometry::RationalBezier(surface), SurfaceGeometry::Plane(plane)) => {
                intersect_plane_rational_bezier_surface(plane, surface)
                    .map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Plane(plane), SurfaceGeometry::Nurbs(surface)) => {
                intersect_plane_nurbs_surface(plane, surface)
            }
            (SurfaceGeometry::Nurbs(surface), SurfaceGeometry::Plane(plane)) => {
                intersect_plane_nurbs_surface(plane, surface).map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Cylinder(first), SurfaceGeometry::Cylinder(second)) => {
                intersect_parallel_cylinders(first, second)
            }
            (SurfaceGeometry::Cylinder(cylinder), SurfaceGeometry::Cone(cone)) => {
                intersect_coaxial_cylinder_cone(cylinder, cone)
            }
            (SurfaceGeometry::Cone(cone), SurfaceGeometry::Cylinder(cylinder)) => {
                intersect_coaxial_cylinder_cone(cylinder, cone).map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Sphere(sphere), SurfaceGeometry::Cylinder(cylinder)) => {
                intersect_coaxial_sphere_cylinder(sphere, cylinder)
            }
            (SurfaceGeometry::Cylinder(cylinder), SurfaceGeometry::Sphere(sphere)) => {
                intersect_coaxial_sphere_cylinder(sphere, cylinder).map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Sphere(sphere), SurfaceGeometry::Cone(cone)) => {
                intersect_coaxial_sphere_cone(sphere, cone)
            }
            (SurfaceGeometry::Cone(cone), SurfaceGeometry::Sphere(sphere)) => {
                intersect_coaxial_sphere_cone(sphere, cone).map(swapped_curve_intersection)
            }
            (SurfaceGeometry::Sphere(first), SurfaceGeometry::Sphere(second)) => {
                intersect_spheres(first, second)
            }
            _ => Err(GeometryError::UnsupportedIntersection),
        }
    }

    pub(crate) fn transformed(
        &self,
        transform: &Matrix4,
        reflect_parameters: bool,
    ) -> GeometryResult<Self> {
        match &self.data.geometry {
            SurfaceGeometry::Plane(plane) => {
                let mut u = transform.transform_direction3(&plane.u);
                if reflect_parameters {
                    u = -u;
                }
                Self::plane(
                    transform_point(transform, &plane.origin)?,
                    u,
                    transform.transform_direction3(&plane.v),
                )
            }
            SurfaceGeometry::Cylinder(surface) => {
                let x = transform.transform_direction3(&surface.frame.x);
                let mut y = transform.transform_direction3(&surface.frame.y);
                if reflect_parameters {
                    y = -y;
                }
                Self::cylinder(
                    transform_point(transform, &surface.origin)?,
                    x,
                    y,
                    transform.transform_direction3(&surface.frame.z),
                    surface.radius.clone(),
                )
            }
            SurfaceGeometry::Torus(surface) => {
                let x = transform.transform_direction3(&surface.frame.x);
                let mut y = transform.transform_direction3(&surface.frame.y);
                if reflect_parameters {
                    y = -y;
                }
                Self::torus(
                    transform_point(transform, &surface.center)?,
                    x,
                    y,
                    transform.transform_direction3(&surface.frame.z),
                    surface.major_radius.clone(),
                    surface.minor_radius.clone(),
                )
            }
            SurfaceGeometry::Cone(surface) => {
                let x = transform.transform_direction3(&surface.frame.x);
                let mut y = transform.transform_direction3(&surface.frame.y);
                if reflect_parameters {
                    y = -y;
                }
                Self::cone(
                    transform_point(transform, &surface.apex)?,
                    x,
                    y,
                    transform.transform_direction3(&surface.frame.z),
                    surface.semi_angle.clone(),
                )
            }
            SurfaceGeometry::Sphere(surface) => {
                let x = transform.transform_direction3(&surface.frame.x);
                let mut y = transform.transform_direction3(&surface.frame.y);
                if reflect_parameters {
                    y = -y;
                }
                Self::sphere(
                    transform_point(transform, &surface.center)?,
                    x,
                    y,
                    transform.transform_direction3(&surface.frame.z),
                    surface.radius.clone(),
                )
            }
            SurfaceGeometry::Revolution(surface) => Self::revolution(
                surface.profile.transformed(transform)?,
                transform_point(transform, &surface.axis_origin)?,
                transform.transform_direction3(&surface.axis),
            ),
            SurfaceGeometry::Extrusion(surface) => {
                let mut direction = transform.transform_direction3(&surface.direction);
                if reflect_parameters {
                    direction = -direction;
                }
                Self::extrusion(surface.profile.transformed(transform)?, direction)
            }
            SurfaceGeometry::RationalBezier(surface) => {
                let mut control_points =
                    transform_surface_control_points(transform, &surface.control_points)?;
                let mut weights = surface.weights.clone();
                if reflect_parameters {
                    reverse_surface_u(&mut control_points);
                    reverse_surface_u(&mut weights);
                }
                Self::rational_bezier(control_points, weights)
            }
            SurfaceGeometry::Nurbs(surface) => {
                let mut control_points =
                    transform_surface_control_points(transform, &surface.control_points)?;
                let mut weights = surface.weights.clone();
                let mut u_knots = surface.u_knots.clone();
                if reflect_parameters {
                    reverse_surface_u(&mut control_points);
                    reverse_surface_u(&mut weights);
                    reverse_knot_axis(&mut u_knots);
                }
                Self::nurbs(
                    surface.u_degree,
                    surface.v_degree,
                    control_points,
                    weights,
                    u_knots,
                    surface.v_knots.clone(),
                )
            }
        }
    }
}

fn intersect_ellipse_arc_plane(
    curve: &Curve3,
    arc: &EllipseArc3,
    plane: &PlaneSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let center_value = normal.dot(&(&arc.center - &plane.origin));
    let cosine_coefficient = normal.dot(&(arc.x.clone() * &arc.x_radius));
    let sine_coefficient = normal.dot(&(arc.y.clone() * &arc.y_radius));
    intersect_ellipse_arc_scalar_equation(
        curve,
        arc,
        cosine_coefficient,
        sine_coefficient,
        center_value,
    )
}

fn intersect_circle_arc_sphere(
    curve: &Curve3,
    arc: &EllipseArc3,
    sphere: &SphereSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let offset = &arc.center - &sphere.center;
    let twice_radius = Real::from(2) * &arc.x_radius;
    let cosine_coefficient = &twice_radius * arc.x.dot(&offset);
    let sine_coefficient = &twice_radius * arc.y.dot(&offset);
    let center_value =
        offset.norm_squared() + &arc.x_radius * &arc.x_radius - &sphere.radius * &sphere.radius;
    intersect_ellipse_arc_scalar_equation(
        curve,
        arc,
        cosine_coefficient,
        sine_coefficient,
        center_value,
    )
}

fn intersect_transverse_circle_arc_cylinder(
    curve: &Curve3,
    arc: &EllipseArc3,
    cylinder: &CylinderSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let circle_axis = arc.x.cross(&arc.y);
    if decided_order(compare_reals(
        &circle_axis.cross(&cylinder.frame.z).norm_squared(),
        &Real::zero(),
    ))? != Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }
    intersect_transverse_circle_arc_radius(
        curve,
        arc,
        &cylinder.origin,
        &cylinder.frame.z,
        &cylinder.radius,
    )
}

fn intersect_transverse_circle_arc_cone(
    curve: &Curve3,
    arc: &EllipseArc3,
    cone: &ConeSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let circle_axis = arc.x.cross(&arc.y);
    if decided_order(compare_reals(
        &circle_axis.cross(&cone.frame.z).norm_squared(),
        &Real::zero(),
    ))? != Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let center_offset = &arc.center - &cone.apex;
    let height = center_offset.dot(&cone.frame.z);
    if decided_order(compare_reals(&height, &Real::zero()))? == Ordering::Less {
        return Ok(CurveSurfaceIntersection::None);
    }
    let cone_radius = height
        * cone
            .semi_angle
            .clone()
            .tan()
            .map_err(|_| GeometryError::ElementaryFunction)?;
    intersect_transverse_circle_arc_radius(curve, arc, &cone.apex, &cone.frame.z, &cone_radius)
}

fn intersect_transverse_circle_arc_torus(
    curve: &Curve3,
    arc: &EllipseArc3,
    torus: &TorusSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let circle_axis = arc.x.cross(&arc.y);
    if decided_order(compare_reals(
        &circle_axis.cross(&torus.frame.z).norm_squared(),
        &Real::zero(),
    ))? != Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let center_offset = &arc.center - &torus.center;
    let height = center_offset.dot(&torus.frame.z);
    let height_squared = &height * &height;
    let minor_squared = &torus.minor_radius * &torus.minor_radius;
    if decided_order(compare_reals(&height_squared, &minor_squared))? == Ordering::Greater {
        return Ok(CurveSurfaceIntersection::None);
    }
    let radial_delta = (minor_squared - height_squared)
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let mut radii = vec![&torus.major_radius + &radial_delta];
    if decided_order(compare_reals(&radial_delta, &Real::zero()))? != Ordering::Equal {
        radii.push(&torus.major_radius - radial_delta);
    }
    let mut combined = Vec::<CurveSurfacePoint>::new();
    for radius in radii {
        match intersect_transverse_circle_arc_radius(
            curve,
            arc,
            &torus.center,
            &torus.frame.z,
            &radius,
        )? {
            CurveSurfaceIntersection::None => {}
            CurveSurfaceIntersection::Points(points) => {
                for point in points {
                    let mut duplicate = false;
                    for existing in &combined {
                        if decided_order(compare_reals(&existing.parameter, &point.parameter))?
                            == Ordering::Equal
                        {
                            duplicate = true;
                            break;
                        }
                    }
                    if !duplicate {
                        combined.push(point);
                    }
                }
            }
            CurveSurfaceIntersection::Contained => {
                return Ok(CurveSurfaceIntersection::Contained);
            }
            CurveSurfaceIntersection::Overlap(_) => {
                unreachable!("exact circle relation has no proper overlap")
            }
        }
    }
    for index in 1..combined.len() {
        let mut position = index;
        while position > 0
            && decided_order(compare_reals(
                &combined[position].parameter,
                &combined[position - 1].parameter,
            ))? == Ordering::Less
        {
            combined.swap(position, position - 1);
            position -= 1;
        }
    }
    if combined.is_empty() {
        Ok(CurveSurfaceIntersection::None)
    } else {
        Ok(CurveSurfaceIntersection::Points(combined))
    }
}

fn intersect_transverse_circle_arc_radius(
    curve: &Curve3,
    arc: &EllipseArc3,
    axis_origin: &Point3,
    axis: &Vector3,
    target_radius: &Real,
) -> GeometryResult<CurveSurfaceIntersection> {
    let center_offset = &arc.center - axis_origin;
    let radial_offset = &center_offset - &(axis.clone() * center_offset.dot(axis));
    let twice_radius = Real::from(2) * &arc.x_radius;
    let cosine_coefficient = &twice_radius * arc.x.dot(&radial_offset);
    let sine_coefficient = &twice_radius * arc.y.dot(&radial_offset);
    let center_value = radial_offset.norm_squared() + &arc.x_radius * &arc.x_radius
        - target_radius * target_radius;
    intersect_ellipse_arc_scalar_equation(
        curve,
        arc,
        cosine_coefficient,
        sine_coefficient,
        center_value,
    )
}

fn intersect_ellipse_arc_scalar_equation(
    curve: &Curve3,
    arc: &EllipseArc3,
    cosine_coefficient: Real,
    sine_coefficient: Real,
    center_value: Real,
) -> GeometryResult<CurveSurfaceIntersection> {
    let amplitude_squared =
        &cosine_coefficient * &cosine_coefficient + &sine_coefficient * &sine_coefficient;
    if decided_order(compare_reals(&amplitude_squared, &Real::zero()))? == Ordering::Equal {
        return if decided_order(compare_reals(&center_value, &Real::zero()))? == Ordering::Equal {
            Ok(CurveSurfaceIntersection::Contained)
        } else {
            Ok(CurveSurfaceIntersection::None)
        };
    }
    let center_squared = &center_value * &center_value;
    let relation = decided_order(compare_reals(&center_squared, &amplitude_squared))?;
    if relation == Ordering::Greater {
        return Ok(CurveSurfaceIntersection::None);
    }
    let amplitude = amplitude_squared
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let phase = certified_atan2(sine_coefficient, cosine_coefficient)?;
    let ratio = ((-center_value) / amplitude).map_err(|_| GeometryError::ProjectiveDivision)?;
    let offset = ratio
        .acos()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let angles = if relation == Ordering::Equal {
        vec![phase + offset]
    } else {
        vec![&phase + &offset, phase - offset]
    };
    let multiplicity = if relation == Ordering::Equal {
        IntersectionMultiplicity::Tangent
    } else {
        IntersectionMultiplicity::Simple
    };
    let mut points = Vec::<CurveSurfacePoint>::new();
    for angle in angles {
        let point = arc.center.clone()
            + arc.x.clone() * (&arc.x_radius * angle.clone().cos())
            + arc.y.clone() * (&arc.y_radius * angle.sin());
        let CurveParameterLocation::Parameters(parameters) =
            locate_ellipse_arc_parameters(curve, arc, &point)?
        else {
            continue;
        };
        for parameter in parameters {
            let mut duplicate = false;
            for existing in &points {
                if decided_order(compare_reals(&existing.parameter, &parameter))? == Ordering::Equal
                {
                    duplicate = true;
                    break;
                }
            }
            if duplicate {
                continue;
            }
            points.push(CurveSurfacePoint {
                parameter,
                point: point.clone(),
                multiplicity,
            });
        }
    }
    for index in 1..points.len() {
        let mut position = index;
        while position > 0
            && decided_order(compare_reals(
                &points[position].parameter,
                &points[position - 1].parameter,
            ))? == Ordering::Less
        {
            points.swap(position, position - 1);
            position -= 1;
        }
    }
    if points.is_empty() {
        Ok(CurveSurfaceIntersection::None)
    } else {
        Ok(CurveSurfaceIntersection::Points(points))
    }
}

fn angle_domain() -> SurfaceParameterDomain {
    SurfaceParameterDomain::Periodic {
        start: Real::zero(),
        period: Real::from(2) * Real::pi(),
    }
}

fn angle_unbounded_domain() -> SurfaceDomain {
    SurfaceDomain {
        u: angle_domain(),
        v: SurfaceParameterDomain::Unbounded,
    }
}

fn validate_surface_parameter(domain: &SurfaceDomain, parameter: &Point2) -> GeometryResult<()> {
    if surface_axis_contains(&domain.u, &parameter.x)?
        && surface_axis_contains(&domain.v, &parameter.y)?
    {
        Ok(())
    } else {
        Err(GeometryError::SurfaceParameterOutsideDomain)
    }
}

fn surface_axis_contains(
    domain: &SurfaceParameterDomain,
    parameter: &Real,
) -> GeometryResult<bool> {
    match domain {
        SurfaceParameterDomain::Unbounded | SurfaceParameterDomain::Periodic { .. } => Ok(true),
        SurfaceParameterDomain::Closed(domain) => domain.contains(parameter),
        SurfaceParameterDomain::LowerBounded { start } => Ok(matches!(
            decided_order(compare_reals(parameter, start))?,
            Ordering::Equal | Ordering::Greater
        )),
    }
}

fn require_positive(value: &Real, error: GeometryError) -> GeometryResult<()> {
    if decided_order(compare_reals(value, &Real::zero()))? == Ordering::Greater {
        Ok(())
    } else {
        Err(error)
    }
}

fn validate_orthonormal_frame(
    x: Vector3,
    y: Vector3,
    z: Vector3,
) -> GeometryResult<OrthonormalFrame3> {
    let zero = Real::zero();
    let one = Real::one();
    let scalar_checks = [
        (x.norm_squared(), &one),
        (y.norm_squared(), &one),
        (z.norm_squared(), &one),
        (x.dot(&y), &zero),
        (y.dot(&z), &zero),
        (z.dot(&x), &zero),
    ];
    for (actual, expected) in scalar_checks {
        if decided_order(compare_reals(&actual, expected))? != Ordering::Equal {
            return Err(GeometryError::InvalidSurfaceFrame);
        }
    }
    let handedness = x.cross(&y).dot(&z);
    if decided_order(compare_reals(&handedness, &one))? != Ordering::Equal {
        return Err(GeometryError::InvalidSurfaceFrame);
    }
    Ok(OrthonormalFrame3 { x, y, z })
}

pub(crate) fn affine_transform_orientation(transform: &Matrix4) -> GeometryResult<Ordering> {
    for (entry, expected) in
        transform.0[3]
            .iter()
            .zip([Real::zero(), Real::zero(), Real::zero(), Real::one()])
    {
        if decided_order(compare_reals(entry, &expected))? != Ordering::Equal {
            return Err(GeometryError::NonAffineTransform);
        }
    }
    let m = &transform.0;
    let determinant = &m[0][0] * (&m[1][1] * &m[2][2] - &m[1][2] * &m[2][1])
        - &m[0][1] * (&m[1][0] * &m[2][2] - &m[1][2] * &m[2][0])
        + &m[0][2] * (&m[1][0] * &m[2][1] - &m[1][1] * &m[2][0]);
    match decided_order(compare_reals(&determinant, &Real::zero()))? {
        Ordering::Greater => Ok(Ordering::Greater),
        Ordering::Equal => Err(GeometryError::SingularTransform),
        Ordering::Less => Ok(Ordering::Less),
    }
}

fn transform_point(transform: &Matrix4, point: &Point3) -> GeometryResult<Point3> {
    transform
        .transform_point3(point)
        .map_err(|_| GeometryError::TransformFailure)
}

fn transform_points(transform: &Matrix4, points: &[Point3]) -> GeometryResult<Vec<Point3>> {
    transform
        .transform_point3_batch(points)
        .map_err(|_| GeometryError::TransformFailure)
}

fn transform_surface_control_points(
    transform: &Matrix4,
    control_points: &[Vec<Point3>],
) -> GeometryResult<Vec<Vec<Point3>>> {
    control_points
        .iter()
        .map(|row| transform_points(transform, row))
        .collect()
}

fn reverse_surface_u<T>(rows: &mut [Vec<T>]) {
    for row in rows {
        row.reverse();
    }
}

fn reverse_knot_axis(knots: &mut [Real]) {
    let sum = &knots[0] + &knots[knots.len() - 1];
    knots.reverse();
    for knot in knots {
        *knot = &sum - &*knot;
    }
}

fn swapped_curve_intersection(
    intersection: SurfaceSurfaceIntersection,
) -> SurfaceSurfaceIntersection {
    match intersection {
        SurfaceSurfaceIntersection::Curve(curve) => {
            SurfaceSurfaceIntersection::Curve(Box::new((*curve).swapped()))
        }
        SurfaceSurfaceIntersection::Curves(curves) => SurfaceSurfaceIntersection::Curves(
            curves
                .into_iter()
                .map(SurfaceIntersectionCurve::swapped)
                .collect(),
        ),
        SurfaceSurfaceIntersection::Ray(ray) => {
            SurfaceSurfaceIntersection::Ray(Box::new((*ray).swapped()))
        }
        SurfaceSurfaceIntersection::Rays(rays) => SurfaceSurfaceIntersection::Rays(
            rays.into_iter()
                .map(SurfaceIntersectionRay::swapped)
                .collect(),
        ),
        other => other,
    }
}

fn project_point_to_plane_frame(
    origin: &Point3,
    u: &Vector3,
    v: &Vector3,
    point: &Point3,
) -> GeometryResult<Point2> {
    let displacement = point - origin;
    let uu = u.dot(u);
    let uv = u.dot(v);
    let vv = v.dot(v);
    let du = displacement.dot(u);
    let dv = displacement.dot(v);
    let determinant = &uu * &vv - &uv * &uv;
    Ok(Point2::new(
        ((&du * &vv - &dv * &uv) / &determinant).map_err(|_| GeometryError::ProjectiveDivision)?,
        ((&dv * &uu - &du * &uv) / determinant).map_err(|_| GeometryError::ProjectiveDivision)?,
    ))
}

pub(crate) fn project_curve_to_plane_frame(
    curve: &Curve3,
    origin: &Point3,
    u: &Vector3,
    v: &Vector3,
) -> GeometryResult<Option<Curve2>> {
    let project = |point: &Point3| {
        project_point_to_plane_frame(origin, u, v, point)
            .map(|point| CurvePoint2::new(point.x, point.y))
    };
    match curve.exact_data() {
        Curve3ExactData::Line(line) => Ok(Some(Curve2::from(LineSeg2::try_new(
            project(&line.start)?,
            project(&line.end)?,
        )?))),
        Curve3ExactData::RationalBezier {
            control_points,
            weights,
        } => Ok(Some(Curve2::from(RationalBezier2::try_new(
            control_points
                .iter()
                .map(project)
                .collect::<GeometryResult<Vec<_>>>()?,
            weights,
        )?))),
        Curve3ExactData::Nurbs {
            degree,
            control_points,
            weights,
            knots,
        } => Ok(Some(Curve2::try_nurbs(
            degree,
            control_points
                .iter()
                .map(project)
                .collect::<GeometryResult<Vec<_>>>()?,
            weights,
            knots,
        )?)),
        Curve3ExactData::EllipseArc(data) if data.circle => {
            let center = project(&data.center)?;
            let projected_x =
                project(&(data.center.clone() + data.x.clone() * data.x_radius.clone()))?;
            let projected_y =
                project(&(data.center.clone() + data.y.clone() * data.y_radius.clone()))?;
            let x = (projected_x.x() - center.x(), projected_x.y() - center.y());
            let y = (projected_y.x() - center.x(), projected_y.y() - center.y());
            let xy = &x.0 * &y.0 + &x.1 * &y.1;
            let x_squared = &x.0 * &x.0 + &x.1 * &x.1;
            let y_squared = &y.0 * &y.0 + &y.1 * &y.1;
            if decided_order(compare_reals(&xy, &Real::zero()))? != Ordering::Equal
                || decided_order(compare_reals(&x_squared, &y_squared))? != Ordering::Equal
            {
                return Ok(None);
            }
            let start = project(&curve.start()?)?;
            let end = project(&curve.end()?)?;
            let tangent_point =
                data.center.clone() + curve.derivative_at(curve.domain().start(), 1)?.vector();
            let projected_tangent_point = project(&tangent_point)?;
            let projected_tangent = (
                projected_tangent_point.x() - center.x(),
                projected_tangent_point.y() - center.y(),
            );
            let radial = (start.x() - center.x(), start.y() - center.y());
            let orientation = &radial.0 * &projected_tangent.1 - &radial.1 * &projected_tangent.0;
            let clockwise =
                decided_order(compare_reals(&orientation, &Real::zero()))? == Ordering::Less;
            Ok(Some(Curve2::from(CircularArc2::try_from_center(
                start, end, center, clockwise,
            )?)))
        }
        Curve3ExactData::EllipseArc(_) => Ok(None),
    }
}

fn linear_section_pcurve_carriers(
    profile: &Curve3,
    plane_origin: &Point3,
    plane_normal: &Vector3,
    denominator: &Real,
    profile_axis: TensorAxis,
    coefficient_scale: &Real,
    coefficient_offset: &Real,
) -> GeometryResult<Option<Vec<SurfacePcurveClipCarrier>>> {
    let coefficient = |point: &Point3| {
        let normalized = (plane_normal.dot(&(plane_origin - point)) / denominator)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        Ok(coefficient_offset + coefficient_scale * normalized)
    };
    let span = profile.domain().end() - profile.domain().start();
    match &profile.data.geometry {
        CurveGeometry3::Line(line) => Ok(Some(vec![SurfacePcurveClipCarrier {
            curve: Curve2::from(LineSeg2::try_new(
                section_parameter_point(
                    profile.domain().start().clone(),
                    coefficient(&line.start)?,
                    profile_axis,
                ),
                section_parameter_point(
                    profile.domain().end().clone(),
                    coefficient(&line.end)?,
                    profile_axis,
                ),
            )?),
            spatial_scale: span,
            spatial_offset: profile.domain().start().clone(),
        }])),
        CurveGeometry3::RationalBezier(curve) => Ok(Some(vec![rational_linear_section_carrier(
            curve,
            profile.domain().start(),
            &span,
            &coefficient,
            profile_axis,
        )?])),
        CurveGeometry3::Nurbs(nurbs) => {
            let mut carriers = Vec::new();
            for (segment, domain) in decompose_nurbs_into_bezier_segments(nurbs)? {
                let CurveGeometry3::RationalBezier(curve) = &segment.data.geometry else {
                    unreachable!("NURBS decomposition produces rational Bézier segments");
                };
                carriers.push(rational_linear_section_carrier(
                    curve,
                    domain.start(),
                    &(domain.end() - domain.start()),
                    &coefficient,
                    profile_axis,
                )?);
            }
            Ok(Some(carriers))
        }
        CurveGeometry3::CircleArc(_) | CurveGeometry3::EllipseArc(_) => Ok(None),
    }
}

pub(crate) fn concatenate_rational_bezier_spans_as_nurbs(
    curves: &[Curve2],
    boundaries: &[Real],
) -> GeometryResult<Curve2> {
    if curves.len() < 2 || boundaries.len() != curves.len() + 1 {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let CurveGeometry2::RationalBezier(first) = curves[0].geometry() else {
        return Err(GeometryError::UnsupportedIntersection);
    };
    let degree = first.degree();
    if degree == 0 {
        return Err(GeometryError::InvalidDegree);
    }
    let mut control_points = first.control_points().to_vec();
    let mut weights = first.weights().to_vec();
    for curve in &curves[1..] {
        let CurveGeometry2::RationalBezier(curve) = curve.geometry() else {
            return Err(GeometryError::UnsupportedIntersection);
        };
        if curve.degree() != degree
            || decided_order(compare_reals(
                control_points
                    .last()
                    .expect("first rational span has controls")
                    .x(),
                curve
                    .control_points()
                    .first()
                    .expect("rational span has controls")
                    .x(),
            ))? != Ordering::Equal
            || decided_order(compare_reals(
                control_points
                    .last()
                    .expect("first rational span has controls")
                    .y(),
                curve
                    .control_points()
                    .first()
                    .expect("rational span has controls")
                    .y(),
            ))? != Ordering::Equal
        {
            return Err(GeometryError::UnsupportedIntersection);
        }
        let scale = (weights
            .last()
            .expect("first rational span has weights")
            .clone()
            / &curve.weights()[0])
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        control_points.extend(curve.control_points().iter().skip(1).cloned());
        weights.extend(curve.weights().iter().skip(1).map(|weight| weight * &scale));
    }
    let mut knots = Vec::with_capacity(control_points.len() + degree + 1);
    knots.extend(std::iter::repeat_n(boundaries[0].clone(), degree + 1));
    for boundary in &boundaries[1..boundaries.len() - 1] {
        knots.extend(std::iter::repeat_n(boundary.clone(), degree));
    }
    knots.extend(std::iter::repeat_n(
        boundaries.last().expect("at least two boundaries").clone(),
        degree + 1,
    ));
    Curve2::try_nurbs(degree, control_points, weights, knots).map_err(GeometryError::from)
}

pub(crate) fn materialize_nurbs_parameter_graph(
    degree: usize,
    coefficients: &[Real],
    weights: &[Real],
    knots: &[Real],
    profile_axis: SurfaceIsoAxis,
) -> GeometryResult<Curve2> {
    if coefficients.len() != weights.len() {
        return Err(GeometryError::WeightCountMismatch);
    }
    let synthetic = Curve3::nurbs(
        degree,
        coefficients
            .iter()
            .map(|coefficient| Point3::new(coefficient.clone(), Real::zero(), Real::zero()))
            .collect(),
        weights.to_vec(),
        knots.to_vec(),
    )?;
    let CurveGeometry3::Nurbs(nurbs) = &synthetic.data.geometry else {
        unreachable!("synthetic NURBS graph source retains NURBS geometry");
    };
    let profile_axis = match profile_axis {
        SurfaceIsoAxis::U => TensorAxis::U,
        SurfaceIsoAxis::V => TensorAxis::V,
    };
    let mut curves = Vec::new();
    let mut boundaries = Vec::new();
    for (segment, domain) in decompose_nurbs_into_bezier_segments(nurbs)? {
        let CurveGeometry3::RationalBezier(curve) = &segment.data.geometry else {
            unreachable!("NURBS decomposition produces rational Bézier segments");
        };
        if boundaries.is_empty() {
            boundaries.push(domain.start().clone());
        }
        boundaries.push(domain.end().clone());
        curves.push(
            rational_linear_section_carrier(
                curve,
                domain.start(),
                &(domain.end() - domain.start()),
                &|point| Ok(point.x.clone()),
                profile_axis,
            )?
            .curve,
        );
    }
    match curves.as_slice() {
        [curve] => Ok(curve.clone()),
        _ => concatenate_rational_bezier_spans_as_nurbs(&curves, &boundaries),
    }
}

fn section_parameter_point(
    profile_parameter: Real,
    coefficient: Real,
    profile_axis: TensorAxis,
) -> CurvePoint2 {
    match profile_axis {
        TensorAxis::U => CurvePoint2::new(profile_parameter, coefficient),
        TensorAxis::V => CurvePoint2::new(coefficient, profile_parameter),
    }
}

fn rational_linear_section_carrier(
    curve: &RationalBezier3,
    parameter_start: &Real,
    parameter_span: &Real,
    coefficient: &impl Fn(&Point3) -> GeometryResult<Real>,
    profile_axis: TensorAxis,
) -> GeometryResult<SurfacePcurveClipCarrier> {
    let degree_plus_one = curve.weights.len();
    let denominator_degree =
        Real::from(i64::try_from(degree_plus_one).map_err(|_| GeometryError::InvalidDegree)?);
    let section_homogeneous = curve
        .control_points
        .iter()
        .zip(&curve.weights)
        .map(|(point, weight)| Ok::<_, GeometryError>(weight * coefficient(point)?))
        .collect::<Result<Vec<_>, _>>()?;
    let mut elevated_weights = Vec::with_capacity(degree_plus_one + 1);
    let mut elevated_points = Vec::with_capacity(degree_plus_one + 1);
    for index in 0..=degree_plus_one {
        let alpha = (Real::from(i64::try_from(index).map_err(|_| GeometryError::InvalidDegree)?)
            / &denominator_degree)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let one_minus_alpha = Real::one() - &alpha;
        let previous_weight = (index > 0).then(|| curve.weights[index - 1].clone());
        let next_weight = (index < degree_plus_one).then(|| curve.weights[index].clone());
        let elevated_weight = previous_weight
            .as_ref()
            .map_or_else(Real::zero, |weight| &alpha * weight)
            + next_weight
                .as_ref()
                .map_or_else(Real::zero, |weight| &one_minus_alpha * weight);
        let local_parameter_homogeneous = previous_weight
            .as_ref()
            .map_or_else(Real::zero, |weight| &alpha * weight);
        let x_homogeneous =
            parameter_start * &elevated_weight + parameter_span * local_parameter_homogeneous;
        let previous_y = if index > 0 {
            &alpha * &section_homogeneous[index - 1]
        } else {
            Real::zero()
        };
        let next_y = if index < degree_plus_one {
            &one_minus_alpha * &section_homogeneous[index]
        } else {
            Real::zero()
        };
        let y_homogeneous = previous_y + next_y;
        elevated_points.push(section_parameter_point(
            (x_homogeneous / &elevated_weight).map_err(|_| GeometryError::ProjectiveDivision)?,
            (y_homogeneous / &elevated_weight).map_err(|_| GeometryError::ProjectiveDivision)?,
            profile_axis,
        ));
        elevated_weights.push(elevated_weight);
    }
    Ok(SurfacePcurveClipCarrier {
        curve: Curve2::from(RationalBezier2::try_new(elevated_points, elevated_weights)?),
        spatial_scale: parameter_span.clone(),
        spatial_offset: parameter_start.clone(),
    })
}

fn intersect_planes(
    first: &PlaneSurface,
    second: &PlaneSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let first_normal = first.u.cross(&first.v);
    let second_normal = second.u.cross(&second.v);
    let direction = first_normal.cross(&second_normal);
    let direction_squared = direction.norm_squared();
    if decided_order(compare_reals(&direction_squared, &Real::zero()))? == Ordering::Equal {
        let separation = first_normal.dot(&(&second.origin - &first.origin));
        return if decided_order(compare_reals(&separation, &Real::zero()))? == Ordering::Equal {
            Ok(SurfaceSurfaceIntersection::Coincident)
        } else {
            Ok(SurfaceSurfaceIntersection::None)
        };
    }

    let first_constant = first_normal.dot(&Vector3::from(first.origin.clone()));
    let second_constant = second_normal.dot(&Vector3::from(second.origin.clone()));
    let numerator = (second_normal.clone() * first_constant
        - first_normal.clone() * second_constant)
        .cross(&direction);
    let point = Point3::from(Vector3::from_xyz(
        (numerator.0[0].clone() / &direction_squared)
            .map_err(|_| GeometryError::ProjectiveDivision)?,
        (numerator.0[1].clone() / &direction_squared)
            .map_err(|_| GeometryError::ProjectiveDivision)?,
        (numerator.0[2].clone() / direction_squared)
            .map_err(|_| GeometryError::ProjectiveDivision)?,
    ));
    Ok(SurfaceSurfaceIntersection::Line(Box::new(
        SurfaceIntersectionLine { point, direction },
    )))
}

fn intersect_plane_rational_bezier_surface(
    plane: &PlaneSurface,
    surface: &RationalBezierSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    if let Some(direction) = linear_tensor_u_direction(&surface.control_points, &surface.weights)? {
        let profile_controls = surface
            .control_points
            .iter()
            .map(|row| row[0].clone())
            .collect::<Vec<_>>();
        let profile_weights = surface
            .weights
            .iter()
            .map(|row| row[0].clone())
            .collect::<Vec<_>>();
        let profile = Curve3::rational_bezier(profile_controls.clone(), profile_weights)?;
        match intersect_plane_linear_tensor_graph(
            plane,
            profile,
            &profile_controls,
            direction,
            TensorAxis::V,
            Real::one(),
            Real::zero(),
        ) {
            Ok(intersection) => return Ok(intersection),
            Err(GeometryError::UnsupportedIntersection) => {}
            Err(error) => return Err(error),
        }
    }
    let Some(direction) = linear_tensor_v_direction(&surface.control_points, &surface.weights)?
    else {
        return Err(GeometryError::UnsupportedIntersection);
    };
    let profile = Curve3::rational_bezier(
        surface.control_points[0].clone(),
        surface.weights[0].clone(),
    )?;
    intersect_plane_v_linear_tensor_iso(
        plane,
        profile,
        &surface.control_points[0],
        direction,
        Real::one(),
        Real::zero(),
    )
}

fn intersect_plane_extrusion(
    plane: &PlaneSurface,
    surface: &ExtrusionSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let denominator = normal.dot(&surface.direction);
    if decided_order(compare_reals(&denominator, &Real::zero()))? == Ordering::Equal {
        let plane_surface = Surface::plane(plane.origin.clone(), plane.u.clone(), plane.v.clone())?;
        return match plane_surface.intersect_curve(&surface.profile)? {
            CurveSurfaceIntersection::None => Ok(SurfaceSurfaceIntersection::None),
            CurveSurfaceIntersection::Contained | CurveSurfaceIntersection::Overlap(_) => {
                Err(GeometryError::UnsupportedIntersection)
            }
            CurveSurfaceIntersection::Points(points) => {
                let mut lines = points
                    .into_iter()
                    .map(|point| SurfaceIntersectionLine {
                        point: point.point,
                        direction: surface.direction.clone(),
                    })
                    .collect::<Vec<_>>();
                if lines.len() == 1 {
                    Ok(SurfaceSurfaceIntersection::Line(Box::new(
                        lines.pop().expect("one retained extrusion line"),
                    )))
                } else {
                    Ok(SurfaceSurfaceIntersection::Lines(lines))
                }
            }
        };
    }
    if !matches!(
        surface.profile.kind(),
        Curve3Kind::Line | Curve3Kind::RationalBezier | Curve3Kind::Nurbs
    ) {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let curve = project_curve_onto_plane_along_direction(
        &surface.profile,
        plane,
        &surface.direction,
        &denominator,
    )?;
    Ok(SurfaceSurfaceIntersection::Curve(Box::new(
        SurfaceIntersectionCurve::new(
            curve.clone(),
            SurfaceIntersectionPcurve::plane_projection(curve, plane),
            SurfaceIntersectionPcurve::linear_plane_section(
                surface.profile.clone(),
                plane,
                denominator,
                TensorAxis::U,
                Real::one(),
                Real::zero(),
            ),
        ),
    )))
}

fn intersect_plane_revolution(
    plane: &PlaneSurface,
    surface: &RevolutionSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    if decided_order(compare_reals(
        &normal.cross(&surface.axis).norm_squared(),
        &Real::zero(),
    ))? != Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let plane_surface = Surface::plane(plane.origin.clone(), plane.u.clone(), plane.v.clone())?;
    let points = match plane_surface.intersect_curve(&surface.profile)? {
        CurveSurfaceIntersection::None => return Ok(SurfaceSurfaceIntersection::None),
        CurveSurfaceIntersection::Contained | CurveSurfaceIntersection::Overlap(_) => {
            return Err(GeometryError::UnsupportedIntersection);
        }
        CurveSurfaceIntersection::Points(points) => points,
    };
    let mut sections = Vec::with_capacity(points.len());
    let mut singular = None;
    let mut radii = Vec::<Real>::with_capacity(points.len());
    for point in points {
        let relative = &point.point - &surface.axis_origin;
        let axial_parameter = relative.dot(&surface.axis);
        let center = surface.axis_origin.clone() + surface.axis.clone() * axial_parameter.clone();
        let radial = relative - surface.axis.clone() * axial_parameter;
        let radius_squared = radial.norm_squared();
        if decided_order(compare_reals(&radius_squared, &Real::zero()))? == Ordering::Equal {
            if singular.replace(center).is_some() {
                return Err(GeometryError::UnsupportedIntersection);
            }
            continue;
        }
        let radius = radius_squared
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        for existing in &radii {
            if decided_order(compare_reals(existing, &radius))? == Ordering::Equal {
                return Err(GeometryError::UnsupportedIntersection);
            }
        }
        radii.push(radius.clone());
        let radial_direction = (radial / &radius).map_err(|_| GeometryError::ProjectiveDivision)?;
        let curve = Curve3::circle_arc(
            center,
            radial_direction.clone(),
            surface.axis.cross(&radial_direction),
            radius,
            Real::zero(),
            Real::tau(),
        )?;
        let domain = curve.domain().clone();
        sections.push(SurfaceIntersectionCurve::new(
            curve.clone(),
            SurfaceIntersectionPcurve::plane_projection(curve, plane),
            SurfaceIntersectionPcurve::tensor_iso_v(domain, point.parameter),
        ));
    }
    if let Some(point) = singular {
        if sections.is_empty() {
            return Ok(SurfaceSurfaceIntersection::Point(Box::new(point)));
        }
        return Err(GeometryError::UnsupportedIntersection);
    }
    Ok(match sections.len() {
        0 => SurfaceSurfaceIntersection::None,
        1 => SurfaceSurfaceIntersection::Curve(Box::new(
            sections.pop().expect("one revolution section"),
        )),
        _ => SurfaceSurfaceIntersection::Curves(sections),
    })
}

fn intersect_plane_nurbs_surface(
    plane: &PlaneSurface,
    surface: &NurbsSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    if surface.u_degree == 1
        && let Some(direction) =
            linear_tensor_u_direction(&surface.control_points, &surface.weights)?
    {
        let profile_controls = surface
            .control_points
            .iter()
            .map(|row| row[0].clone())
            .collect::<Vec<_>>();
        let profile_weights = surface
            .weights
            .iter()
            .map(|row| row[0].clone())
            .collect::<Vec<_>>();
        let profile = Curve3::nurbs(
            surface.v_degree,
            profile_controls.clone(),
            profile_weights,
            surface.v_knots.clone(),
        )?;
        let coefficient_offset = surface.u_knots[surface.u_degree].clone();
        let coefficient_scale =
            &surface.u_knots[surface.control_points[0].len()] - &coefficient_offset;
        match intersect_plane_linear_tensor_graph(
            plane,
            profile,
            &profile_controls,
            direction,
            TensorAxis::V,
            coefficient_scale,
            coefficient_offset,
        ) {
            Ok(intersection) => return Ok(intersection),
            Err(GeometryError::UnsupportedIntersection) => {}
            Err(error) => return Err(error),
        }
    }
    if surface.v_degree != 1 {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let Some(direction) = linear_tensor_v_direction(&surface.control_points, &surface.weights)?
    else {
        return Err(GeometryError::UnsupportedIntersection);
    };
    let profile = Curve3::nurbs(
        surface.u_degree,
        surface.control_points[0].clone(),
        surface.weights[0].clone(),
        surface.u_knots.clone(),
    )?;
    let coefficient_offset = surface.v_knots[surface.v_degree].clone();
    let coefficient_scale = &surface.v_knots[surface.control_points.len()] - &coefficient_offset;
    intersect_plane_v_linear_tensor_iso(
        plane,
        profile,
        &surface.control_points[0],
        direction,
        coefficient_scale,
        coefficient_offset,
    )
}

fn linear_tensor_u_direction(
    control_points: &[Vec<Point3>],
    weights: &[Vec<Real>],
) -> GeometryResult<Option<Vector3>> {
    if control_points.is_empty()
        || control_points.len() != weights.len()
        || control_points
            .iter()
            .zip(weights)
            .any(|(points, row_weights)| points.len() != 2 || row_weights.len() != 2)
    {
        return Ok(None);
    }
    for row in weights {
        if decided_order(compare_reals(&row[0], &row[1]))? != Ordering::Equal {
            return Ok(None);
        }
    }
    let direction = &control_points[0][1] - &control_points[0][0];
    if decided_order(compare_reals(&direction.norm_squared(), &Real::zero()))? != Ordering::Greater
    {
        return Ok(None);
    }
    for row in control_points {
        if !points_equal(&(row[0].clone() + direction.clone()), &row[1])? {
            return Ok(None);
        }
    }
    Ok(Some(direction))
}

fn linear_tensor_v_direction(
    control_points: &[Vec<Point3>],
    weights: &[Vec<Real>],
) -> GeometryResult<Option<Vector3>> {
    if control_points.len() != 2
        || weights.len() != 2
        || control_points[0].len() != control_points[1].len()
        || weights[0].len() != weights[1].len()
    {
        return Ok(None);
    }
    for (first, second) in weights[0].iter().zip(&weights[1]) {
        if decided_order(compare_reals(first, second))? != Ordering::Equal {
            return Ok(None);
        }
    }
    let direction = &control_points[1][0] - &control_points[0][0];
    if decided_order(compare_reals(&direction.norm_squared(), &Real::zero()))? != Ordering::Greater
    {
        return Ok(None);
    }
    for (first, second) in control_points[0].iter().zip(&control_points[1]) {
        if !points_equal(&(first.clone() + direction.clone()), second)? {
            return Ok(None);
        }
    }
    Ok(Some(direction))
}

fn intersect_plane_linear_tensor_graph(
    plane: &PlaneSurface,
    profile: Curve3,
    profile_controls: &[Point3],
    direction: Vector3,
    profile_axis: TensorAxis,
    coefficient_scale: Real,
    coefficient_offset: Real,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let signed_plane_value = |point: &Point3| normal.dot(&(point - &plane.origin));
    let values = profile_controls
        .iter()
        .map(signed_plane_value)
        .collect::<Vec<_>>();
    let denominator = normal.dot(&direction);
    if decided_order(compare_reals(&denominator, &Real::zero()))? == Ordering::Equal {
        if curve_is_strictly_on_one_plane_side(&profile, &signed_plane_value)? {
            return Ok(SurfaceSurfaceIntersection::None);
        }
        let plane_surface = Surface::plane(plane.origin.clone(), plane.u.clone(), plane.v.clone())?;
        return match plane_surface.intersect_curve(&profile)? {
            CurveSurfaceIntersection::None => Ok(SurfaceSurfaceIntersection::None),
            CurveSurfaceIntersection::Contained | CurveSurfaceIntersection::Overlap(_) => {
                Err(GeometryError::UnsupportedIntersection)
            }
            CurveSurfaceIntersection::Points(points) => {
                let mut curves = points
                    .into_iter()
                    .map(|point| {
                        let curve =
                            Curve3::line(point.point.clone(), point.point + direction.clone())?;
                        Ok(SurfaceIntersectionCurve::new(
                            curve.clone(),
                            SurfaceIntersectionPcurve::plane_projection(curve.clone(), plane),
                            SurfaceIntersectionPcurve::tensor_iso(
                                curve.domain().clone(),
                                point.parameter,
                                profile_axis,
                                coefficient_scale.clone(),
                                coefficient_offset.clone(),
                            ),
                        ))
                    })
                    .collect::<GeometryResult<Vec<_>>>()?;
                Ok(match curves.as_mut_slice() {
                    [curve] => SurfaceSurfaceIntersection::Curve(Box::new(curve.clone())),
                    _ => SurfaceSurfaceIntersection::Curves(curves),
                })
            }
        };
    }

    let coefficients = values
        .into_iter()
        .map(|value| ((-value) / &denominator).map_err(|_| GeometryError::ProjectiveDivision))
        .collect::<GeometryResult<Vec<_>>>()?;
    let coefficient_orders = coefficients
        .iter()
        .map(|coefficient| {
            Ok((
                decided_order(compare_reals(coefficient, &Real::zero()))?,
                decided_order(compare_reals(coefficient, &Real::one()))?,
            ))
        })
        .collect::<GeometryResult<Vec<_>>>()?;
    if coefficient_orders
        .iter()
        .all(|(zero, _)| *zero == Ordering::Less)
        || coefficient_orders
            .iter()
            .all(|(_, one)| *one == Ordering::Greater)
    {
        return Ok(SurfaceSurfaceIntersection::None);
    }
    let complete = coefficient_orders.iter().all(|(zero, one)| {
        matches!(zero, Ordering::Equal | Ordering::Greater)
            && matches!(one, Ordering::Equal | Ordering::Less)
    });

    let curve =
        project_curve_onto_plane_along_direction(&profile, plane, &direction, &denominator)?;
    let section = SurfaceIntersectionCurve::new(
        curve.clone(),
        SurfaceIntersectionPcurve::plane_projection(curve, plane),
        SurfaceIntersectionPcurve::linear_plane_section(
            profile,
            plane,
            denominator,
            profile_axis,
            coefficient_scale.clone(),
            coefficient_offset.clone(),
        ),
    );
    if complete {
        return Ok(SurfaceSurfaceIntersection::Curve(Box::new(section)));
    }
    clip_linear_tensor_section(
        &section,
        profile_axis,
        &coefficient_offset,
        &(&coefficient_offset + coefficient_scale),
    )
}

fn curve_is_strictly_on_one_plane_side(
    curve: &Curve3,
    signed_plane_value: &impl Fn(&Point3) -> Real,
) -> GeometryResult<bool> {
    let controls_are_strictly_one_sided = |controls: &[Point3]| -> GeometryResult<bool> {
        let orders = controls
            .iter()
            .map(|point| decided_order(compare_reals(&signed_plane_value(point), &Real::zero())))
            .collect::<GeometryResult<Vec<_>>>()?;
        let endpoints_are = |side| orders.first() == Some(&side) && orders.last() == Some(&side);
        Ok((endpoints_are(Ordering::Greater)
            && orders
                .iter()
                .all(|order| matches!(order, Ordering::Greater | Ordering::Equal)))
            || (endpoints_are(Ordering::Less)
                && orders
                    .iter()
                    .all(|order| matches!(order, Ordering::Less | Ordering::Equal))))
    };

    match &curve.data.geometry {
        CurveGeometry3::Line(line) => {
            controls_are_strictly_one_sided(&[line.start.clone(), line.end.clone()])
        }
        CurveGeometry3::RationalBezier(curve) => {
            controls_are_strictly_one_sided(&curve.control_points)
        }
        CurveGeometry3::Nurbs(curve) => {
            for (span, _) in decompose_nurbs_into_bezier_segments(curve)? {
                let CurveGeometry3::RationalBezier(span) = &span.data.geometry else {
                    unreachable!("NURBS decomposition produces rational Bézier spans");
                };
                if !controls_are_strictly_one_sided(&span.control_points)? {
                    return Ok(false);
                }
            }
            Ok(true)
        }
        CurveGeometry3::CircleArc(_) | CurveGeometry3::EllipseArc(_) => Ok(false),
    }
}

fn clip_linear_tensor_section(
    section: &SurfaceIntersectionCurve,
    profile_axis: TensorAxis,
    coefficient_start: &Real,
    coefficient_end: &Real,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let profile_start = section.curve.domain().start();
    let profile_end = section.curve.domain().end();
    let points = match profile_axis {
        TensorAxis::U => [
            CurvePoint2::new(profile_start.clone(), coefficient_start.clone()),
            CurvePoint2::new(profile_end.clone(), coefficient_start.clone()),
            CurvePoint2::new(profile_end.clone(), coefficient_end.clone()),
            CurvePoint2::new(profile_start.clone(), coefficient_end.clone()),
        ],
        TensorAxis::V => [
            CurvePoint2::new(coefficient_start.clone(), profile_start.clone()),
            CurvePoint2::new(coefficient_end.clone(), profile_start.clone()),
            CurvePoint2::new(coefficient_end.clone(), profile_end.clone()),
            CurvePoint2::new(coefficient_start.clone(), profile_end.clone()),
        ],
    };
    let contour = Contour2::try_new(
        (0..points.len())
            .map(|index| {
                LineSeg2::try_new(
                    points[index].clone(),
                    points[(index + 1) % points.len()].clone(),
                )
                .map(Segment2::Line)
            })
            .collect::<Result<Vec<_>, _>>()?,
    )?;
    let region = LineArcRegion2::from_material_contours(vec![contour]);
    let materialized = section.second_pcurve.materialize()?;
    let trimmed = materialized
        .curve()
        .trim_inside_region_with_parameters(&region, &CurvePolicy::certified())?;
    let mut intervals: Vec<(Real, Real)> = Vec::with_capacity(trimmed.len());
    for fragment in trimmed {
        let Some((pcurve_start, pcurve_end)) = fragment.represented_parameter_range() else {
            return Err(GeometryError::UnrepresentableParameter);
        };
        let spatial_start = materialized.spatial_parameter_at(&pcurve_start)?;
        let spatial_end = materialized.spatial_parameter_at(&pcurve_end)?;
        if let Some((_, previous_end)) = intervals.last_mut()
            && decided_order(compare_reals(previous_end, &spatial_start))? == Ordering::Equal
        {
            *previous_end = spatial_end;
        } else {
            intervals.push((spatial_start, spatial_end));
        }
    }
    let mut fragments = intervals
        .into_iter()
        .map(|(start, end)| section.subcurve(&start, &end))
        .collect::<GeometryResult<Vec<_>>>()?;
    match fragments.len() {
        0 => Err(GeometryError::UnsupportedIntersection),
        1 => Ok(SurfaceSurfaceIntersection::Curve(Box::new(
            fragments.pop().expect("one retained fragment"),
        ))),
        _ => Ok(SurfaceSurfaceIntersection::Curves(fragments)),
    }
}

fn project_curve_onto_plane_along_direction(
    curve: &Curve3,
    plane: &PlaneSurface,
    direction: &Vector3,
    denominator: &Real,
) -> GeometryResult<Curve3> {
    let normal = plane.u.cross(&plane.v);
    let constant = normal.dot(&Vector3::from(plane.origin.clone()));
    let ratio = |row: usize, column: usize| {
        ((&direction.0[row] * &normal.0[column]) / denominator)
            .map_err(|_| GeometryError::ProjectiveDivision)
    };
    let translation = |row: usize| {
        ((&direction.0[row] * &constant) / denominator)
            .map_err(|_| GeometryError::ProjectiveDivision)
    };
    let projection = Matrix4::from_row_major([
        Real::one() - ratio(0, 0)?,
        -ratio(0, 1)?,
        -ratio(0, 2)?,
        translation(0)?,
        -ratio(1, 0)?,
        Real::one() - ratio(1, 1)?,
        -ratio(1, 2)?,
        translation(1)?,
        -ratio(2, 0)?,
        -ratio(2, 1)?,
        Real::one() - ratio(2, 2)?,
        translation(2)?,
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
    ]);
    curve.transformed(&projection)
}

fn intersect_plane_v_linear_tensor_iso(
    plane: &PlaneSurface,
    profile: Curve3,
    profile_controls: &[Point3],
    direction: Vector3,
    coefficient_scale: Real,
    coefficient_offset: Real,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let plane_value = |point: &Point3| normal.dot(&(point - &plane.origin));
    let offset = plane_value(&profile_controls[0]);
    for control in profile_controls.iter().skip(1) {
        if decided_order(compare_reals(&plane_value(control), &offset))? != Ordering::Equal {
            return intersect_plane_linear_tensor_graph(
                plane,
                profile,
                profile_controls,
                direction,
                TensorAxis::U,
                coefficient_scale,
                coefficient_offset,
            );
        }
    }
    let denominator = normal.dot(&direction);
    if decided_order(compare_reals(&denominator, &Real::zero()))? == Ordering::Equal {
        return if decided_order(compare_reals(&offset, &Real::zero()))? == Ordering::Equal {
            Err(GeometryError::UnsupportedIntersection)
        } else {
            Ok(SurfaceSurfaceIntersection::None)
        };
    }
    let normalized_fraction =
        ((-offset) / denominator).map_err(|_| GeometryError::ProjectiveDivision)?;
    if decided_order(compare_reals(&normalized_fraction, &Real::zero()))? == Ordering::Less
        || decided_order(compare_reals(&normalized_fraction, &Real::one()))? == Ordering::Greater
    {
        return Ok(SurfaceSurfaceIntersection::None);
    }
    let translated = profile.transformed(&Matrix4::affine_translation([
        direction.0[0].clone() * &normalized_fraction,
        direction.0[1].clone() * &normalized_fraction,
        direction.0[2].clone() * &normalized_fraction,
    ]))?;
    let surface_parameter = coefficient_offset + coefficient_scale * normalized_fraction;
    Ok(SurfaceSurfaceIntersection::Curve(Box::new(
        SurfaceIntersectionCurve::new(
            translated.clone(),
            SurfaceIntersectionPcurve::plane_projection(translated, plane),
            SurfaceIntersectionPcurve::tensor_iso_v(profile.domain().clone(), surface_parameter),
        ),
    )))
}

fn intersect_plane_sphere(
    plane: &PlaneSurface,
    sphere: &SphereSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let normal_squared = normal.norm_squared();
    let axial = decided_order(compare_reals(
        &normal.cross(&sphere.frame.z).norm_squared(),
        &Real::zero(),
    ))? == Ordering::Equal;
    let separation = normal.dot(&(&sphere.center - &plane.origin));
    let distance_squared = ((&separation * &separation) / &normal_squared)
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    let radius_squared = &sphere.radius * &sphere.radius;
    match decided_order(compare_reals(&distance_squared, &radius_squared))? {
        Ordering::Greater => Ok(SurfaceSurfaceIntersection::None),
        Ordering::Equal | Ordering::Less => {
            let projection_scale =
                (&separation / &normal_squared).map_err(|_| GeometryError::ProjectiveDivision)?;
            let center = sphere.center.clone() - normal.clone() * projection_scale;
            if decided_order(compare_reals(&distance_squared, &radius_squared))? == Ordering::Equal
            {
                return Ok(SurfaceSurfaceIntersection::Point(Box::new(center)));
            }
            let radius = (radius_squared - distance_squared)
                .sqrt()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            if axial {
                let height = ((-&separation) / normal.dot(&sphere.frame.z))
                    .map_err(|_| GeometryError::ProjectiveDivision)?;
                let center = sphere.center.clone() + sphere.frame.z.clone() * &height;
                let curve = Curve3::circle_arc(
                    center,
                    sphere.frame.x.clone(),
                    sphere.frame.y.clone(),
                    radius,
                    Real::zero(),
                    Real::tau(),
                )?;
                let latitude = (height / &sphere.radius)
                    .map_err(|_| GeometryError::ProjectiveDivision)?
                    .asin()
                    .map_err(|_| GeometryError::ElementaryFunction)?;
                let domain = curve.domain().clone();
                return Ok(SurfaceSurfaceIntersection::Curve(Box::new(
                    SurfaceIntersectionCurve::new(
                        curve.clone(),
                        SurfaceIntersectionPcurve::plane_projection(curve, plane),
                        SurfaceIntersectionPcurve::tensor_iso_v(domain, latitude),
                    ),
                )));
            }
            let unit_normal = normal
                .normalize()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            let x = plane
                .u
                .normalize()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            let y = unit_normal.cross(&x);
            Ok(SurfaceSurfaceIntersection::Circle(Curve3::circle_arc(
                center,
                x,
                y,
                radius,
                Real::zero(),
                Real::from(2) * Real::pi(),
            )?))
        }
    }
}

fn intersect_plane_cylinder(
    plane: &PlaneSurface,
    cylinder: &CylinderSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let normal_squared = normal.norm_squared();
    let axis_cross_normal = cylinder.frame.z.cross(&normal);
    if decided_order(compare_reals(
        &axis_cross_normal.norm_squared(),
        &Real::zero(),
    ))? == Ordering::Equal
    {
        let denominator = normal.dot(&cylinder.frame.z);
        let numerator = normal.dot(&(&plane.origin - &cylinder.origin));
        let axial_parameter =
            (numerator / denominator).map_err(|_| GeometryError::ProjectiveDivision)?;
        let center = cylinder.origin.clone() + cylinder.frame.z.clone() * axial_parameter.clone();
        let curve = Curve3::circle_arc(
            center,
            cylinder.frame.x.clone(),
            cylinder.frame.y.clone(),
            cylinder.radius.clone(),
            Real::zero(),
            Real::from(2) * Real::pi(),
        )?;
        let domain = curve.domain().clone();
        return Ok(SurfaceSurfaceIntersection::Curve(Box::new(
            SurfaceIntersectionCurve::new(
                curve.clone(),
                SurfaceIntersectionPcurve::plane_projection(curve, plane),
                SurfaceIntersectionPcurve::tensor_iso_v(domain, axial_parameter),
            ),
        )));
    }

    let axis_dot_normal = cylinder.frame.z.dot(&normal);
    if decided_order(compare_reals(&axis_dot_normal, &Real::zero()))? != Ordering::Equal {
        let separation = normal.dot(&(&plane.origin - &cylinder.origin));
        let axial_center =
            (separation / &axis_dot_normal).map_err(|_| GeometryError::ProjectiveDivision)?;
        let center = cylinder.origin.clone() + cylinder.frame.z.clone() * axial_center;
        let radial_normal = normal.clone() - cylinder.frame.z.clone() * axis_dot_normal.clone();
        let radial_normal_length = radial_normal
            .norm_squared()
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        let radial_parallel = (radial_normal / &radial_normal_length)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let radial_perpendicular = cylinder.frame.z.cross(&radial_parallel);
        let slope = (&radial_normal_length / &axis_dot_normal)
            .map_err(|_| GeometryError::ProjectiveDivision)?;
        let sloped_axis = radial_parallel - cylinder.frame.z.clone() * slope;
        let sloped_length = sloped_axis
            .norm_squared()
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        let sloped_direction =
            (sloped_axis / &sloped_length).map_err(|_| GeometryError::ProjectiveDivision)?;
        return Ok(SurfaceSurfaceIntersection::Ellipse(Curve3::ellipse_arc(
            center,
            radial_perpendicular,
            sloped_direction,
            cylinder.radius.clone(),
            cylinder.radius.clone() * sloped_length,
            Real::zero(),
            Real::from(2) * Real::pi(),
        )?));
    }
    let separation = normal.dot(&(&plane.origin - &cylinder.origin));
    let distance_squared = ((&separation * &separation) / &normal_squared)
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    let radius_squared = &cylinder.radius * &cylinder.radius;
    match decided_order(compare_reals(&distance_squared, &radius_squared))? {
        Ordering::Greater => Ok(SurfaceSurfaceIntersection::None),
        Ordering::Equal | Ordering::Less => {
            let projection_scale =
                (separation / &normal_squared).map_err(|_| GeometryError::ProjectiveDivision)?;
            let radial_projection = normal.clone() * projection_scale;
            let line_offset_squared = &radius_squared - &distance_squared;
            let tangent = cylinder
                .frame
                .z
                .cross(&normal)
                .normalize()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            if decided_order(compare_reals(&line_offset_squared, &Real::zero()))? == Ordering::Equal
            {
                return Ok(SurfaceSurfaceIntersection::Line(Box::new(
                    SurfaceIntersectionLine {
                        point: cylinder.origin.clone() + radial_projection,
                        direction: cylinder.frame.z.clone(),
                    },
                )));
            }
            let line_offset = line_offset_squared
                .sqrt()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            Ok(SurfaceSurfaceIntersection::Lines(vec![
                SurfaceIntersectionLine {
                    point: cylinder.origin.clone()
                        + radial_projection.clone()
                        + tangent.clone() * &line_offset,
                    direction: cylinder.frame.z.clone(),
                },
                SurfaceIntersectionLine {
                    point: cylinder.origin.clone() + radial_projection - tangent * line_offset,
                    direction: cylinder.frame.z.clone(),
                },
            ]))
        }
    }
}

fn intersect_plane_cone(
    plane: &PlaneSurface,
    cone: &ConeSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let axis_cross_normal = decided_order(compare_reals(
        &cone.frame.z.cross(&normal).norm_squared(),
        &Real::zero(),
    ))?;
    if axis_cross_normal != Ordering::Equal {
        let axis_dot_normal = cone.frame.z.dot(&normal);
        let apex_separation = normal.dot(&(&cone.apex - &plane.origin));
        if decided_order(compare_reals(&axis_dot_normal, &Real::zero()))? != Ordering::Equal
            || decided_order(compare_reals(&apex_separation, &Real::zero()))? != Ordering::Equal
        {
            return Err(GeometryError::UnsupportedIntersection);
        }
        let radial = normal
            .cross(&cone.frame.z)
            .normalize()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        let sine = cone.semi_angle.clone().sin();
        let cosine = cone.semi_angle.clone().cos();
        let ray = |radial: Vector3| -> GeometryResult<SurfaceIntersectionRay> {
            let direction = radial.clone() * &sine + cone.frame.z.clone() * &cosine;
            let plane_origin =
                project_point_to_plane_frame(&plane.origin, &plane.u, &plane.v, &cone.apex)?;
            let plane_end = project_point_to_plane_frame(
                &plane.origin,
                &plane.u,
                &plane.v,
                &(cone.apex.clone() + direction.clone()),
            )?;
            let mut u = certified_atan2(radial.dot(&cone.frame.y), radial.dot(&cone.frame.x))?;
            if decided_order(compare_reals(&u, &Real::zero()))? == Ordering::Less {
                u += Real::tau();
            }
            Ok(SurfaceIntersectionRay::new(
                cone.apex.clone(),
                direction,
                Real::zero(),
                SurfaceIntersectionParameterRay::new(
                    plane_origin.clone(),
                    Vector2::from_xy(
                        &plane_end.x - &plane_origin.x,
                        &plane_end.y - &plane_origin.y,
                    ),
                ),
                SurfaceIntersectionParameterRay::new(Point2::new(u, Real::zero()), Vector2::y()),
            ))
        };
        return Ok(SurfaceSurfaceIntersection::Rays(vec![
            ray(radial.clone())?,
            ray(-radial)?,
        ]));
    }
    let axial_height = (normal.dot(&(&plane.origin - &cone.apex)) / normal.dot(&cone.frame.z))
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    match decided_order(compare_reals(&axial_height, &Real::zero()))? {
        Ordering::Less => Ok(SurfaceSurfaceIntersection::None),
        Ordering::Equal => Ok(SurfaceSurfaceIntersection::Point(Box::new(
            cone.apex.clone(),
        ))),
        Ordering::Greater => {
            let radius = axial_height.clone()
                * cone
                    .semi_angle
                    .clone()
                    .tan()
                    .map_err(|_| GeometryError::ElementaryFunction)?;
            let center = cone.apex.clone() + cone.frame.z.clone() * axial_height.clone();
            let curve = Curve3::circle_arc(
                center,
                cone.frame.x.clone(),
                cone.frame.y.clone(),
                radius,
                Real::zero(),
                Real::from(2) * Real::pi(),
            )?;
            let v = (axial_height / cone.semi_angle.clone().cos())
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            let domain = curve.domain().clone();
            Ok(SurfaceSurfaceIntersection::Curve(Box::new(
                SurfaceIntersectionCurve::new(
                    curve.clone(),
                    SurfaceIntersectionPcurve::plane_projection(curve, plane),
                    SurfaceIntersectionPcurve::tensor_iso_v(domain, v),
                ),
            )))
        }
    }
}

fn intersect_plane_torus(
    plane: &PlaneSurface,
    torus: &TorusSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let axis_cross_normal_order = decided_order(compare_reals(
        &torus.frame.z.cross(&normal).norm_squared(),
        &Real::zero(),
    ))?;
    if axis_cross_normal_order != Ordering::Equal {
        let axis_dot_normal = torus.frame.z.dot(&normal);
        let center_separation = normal.dot(&(&torus.center - &plane.origin));
        if decided_order(compare_reals(&axis_dot_normal, &Real::zero()))? != Ordering::Equal {
            return Err(GeometryError::UnsupportedIntersection);
        }
        if decided_order(compare_reals(&center_separation, &Real::zero()))? != Ordering::Equal {
            let distance_squared = ((&center_separation * &center_separation)
                / normal.norm_squared())
            .map_err(|_| GeometryError::ProjectiveDivision)?;
            let outer_radius = &torus.major_radius + &torus.minor_radius;
            match decided_order(compare_reals(
                &distance_squared,
                &(&outer_radius * &outer_radius),
            ))? {
                Ordering::Greater => return Ok(SurfaceSurfaceIntersection::None),
                Ordering::Equal => {
                    let projection_scale = (center_separation / normal.norm_squared())
                        .map_err(|_| GeometryError::ProjectiveDivision)?;
                    return Ok(SurfaceSurfaceIntersection::Point(Box::new(
                        torus.center.clone() - normal.clone() * projection_scale,
                    )));
                }
                Ordering::Less => {}
            }
            return Err(GeometryError::UnsupportedIntersection);
        }

        let radial = normal
            .cross(&torus.frame.z)
            .normalize()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        let normalize_angle = |angle: Real| -> GeometryResult<Real> {
            Ok(
                if decided_order(compare_reals(&angle, &Real::zero()))? == Ordering::Less {
                    angle + Real::tau()
                } else {
                    angle
                },
            )
        };
        let section = |radial: Vector3| -> GeometryResult<SurfaceIntersectionCurve> {
            let u = normalize_angle(certified_atan2(
                radial.dot(&torus.frame.y),
                radial.dot(&torus.frame.x),
            )?)?;
            let curve = Curve3::circle_arc(
                torus.center.clone() + radial.clone() * &torus.major_radius,
                radial,
                torus.frame.z.clone(),
                torus.minor_radius.clone(),
                Real::zero(),
                Real::tau(),
            )?;
            let domain = curve.domain().clone();
            Ok(SurfaceIntersectionCurve::new(
                curve.clone(),
                SurfaceIntersectionPcurve::plane_projection(curve, plane),
                SurfaceIntersectionPcurve::tensor_iso(
                    domain,
                    u,
                    TensorAxis::U,
                    Real::one(),
                    Real::zero(),
                ),
            ))
        };
        return Ok(SurfaceSurfaceIntersection::Curves(vec![
            section(radial.clone())?,
            section(-radial)?,
        ]));
    }
    let axial_height = (normal.dot(&(&plane.origin - &torus.center)) / normal.dot(&torus.frame.z))
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    let axial_squared = &axial_height * &axial_height;
    let minor_squared = &torus.minor_radius * &torus.minor_radius;
    match decided_order(compare_reals(&axial_squared, &minor_squared))? {
        Ordering::Greater => Ok(SurfaceSurfaceIntersection::None),
        Ordering::Equal | Ordering::Less => {
            let radial_offset = (minor_squared - axial_squared)
                .sqrt()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            let center = torus.center.clone() + torus.frame.z.clone() * axial_height.clone();
            let normalize_angle = |angle: Real| -> GeometryResult<Real> {
                Ok(
                    if decided_order(compare_reals(&angle, &Real::zero()))? == Ordering::Less {
                        angle + Real::tau()
                    } else {
                        angle
                    },
                )
            };
            let section = |radius: Real, v: Real| -> GeometryResult<SurfaceIntersectionCurve> {
                let curve = Curve3::circle_arc(
                    center.clone(),
                    torus.frame.x.clone(),
                    torus.frame.y.clone(),
                    radius,
                    Real::zero(),
                    Real::tau(),
                )?;
                let domain = curve.domain().clone();
                Ok(SurfaceIntersectionCurve::new(
                    curve.clone(),
                    SurfaceIntersectionPcurve::plane_projection(curve, plane),
                    SurfaceIntersectionPcurve::tensor_iso_v(domain, v),
                ))
            };
            if decided_order(compare_reals(&radial_offset, &Real::zero()))? == Ordering::Equal {
                let v = normalize_angle(certified_atan2(axial_height, radial_offset)?)?;
                return Ok(SurfaceSurfaceIntersection::Curve(Box::new(section(
                    torus.major_radius.clone(),
                    v,
                )?)));
            }
            let outer_v = normalize_angle(certified_atan2(
                axial_height.clone(),
                radial_offset.clone(),
            )?)?;
            let inner_v = normalize_angle(certified_atan2(axial_height, -radial_offset.clone())?)?;
            Ok(SurfaceSurfaceIntersection::Curves(vec![
                section(&torus.major_radius + &radial_offset, outer_v)?,
                section(&torus.major_radius - radial_offset, inner_v)?,
            ]))
        }
    }
}

fn intersect_parallel_cylinders(
    first: &CylinderSurface,
    second: &CylinderSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    if decided_order(compare_reals(
        &first.frame.z.cross(&second.frame.z).norm_squared(),
        &Real::zero(),
    ))? != Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let displacement = &second.origin - &first.origin;
    let radial_displacement =
        &displacement - &(first.frame.z.clone() * displacement.dot(&first.frame.z));
    let distance_squared = radial_displacement.norm_squared();
    if decided_order(compare_reals(&distance_squared, &Real::zero()))? == Ordering::Equal {
        return if decided_order(compare_reals(&first.radius, &second.radius))? == Ordering::Equal {
            Ok(SurfaceSurfaceIntersection::Coincident)
        } else {
            Ok(SurfaceSurfaceIntersection::None)
        };
    }
    let distance = distance_squared
        .clone()
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let radius_sum = &first.radius + &second.radius;
    let radius_difference = match decided_order(compare_reals(&first.radius, &second.radius))? {
        Ordering::Less => &second.radius - &first.radius,
        Ordering::Equal | Ordering::Greater => &first.radius - &second.radius,
    };
    if decided_order(compare_reals(&distance, &radius_sum))? == Ordering::Greater
        || decided_order(compare_reals(&distance, &radius_difference))? == Ordering::Less
    {
        return Ok(SurfaceSurfaceIntersection::None);
    }
    let along = ((&first.radius * &first.radius - &second.radius * &second.radius
        + &distance_squared)
        / (Real::from(2) * &distance))
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    let unit_displacement =
        (radial_displacement / &distance).map_err(|_| GeometryError::ProjectiveDivision)?;
    let base = first.origin.clone() + unit_displacement.clone() * &along;
    let height_squared = &first.radius * &first.radius - &along * &along;
    if decided_order(compare_reals(&height_squared, &Real::zero()))? == Ordering::Equal {
        return Ok(SurfaceSurfaceIntersection::Line(Box::new(
            SurfaceIntersectionLine {
                point: base,
                direction: first.frame.z.clone(),
            },
        )));
    }
    let height = height_squared
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let transverse = first.frame.z.cross(&unit_displacement);
    Ok(SurfaceSurfaceIntersection::Lines(vec![
        SurfaceIntersectionLine {
            point: base.clone() + transverse.clone() * &height,
            direction: first.frame.z.clone(),
        },
        SurfaceIntersectionLine {
            point: base - transverse * height,
            direction: first.frame.z.clone(),
        },
    ]))
}

fn intersect_coaxial_sphere_cylinder(
    sphere: &SphereSurface,
    cylinder: &CylinderSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let center_offset = &sphere.center - &cylinder.origin;
    let axial_offset = center_offset.dot(&cylinder.frame.z);
    let radial_offset = center_offset - cylinder.frame.z.clone() * &axial_offset;
    if decided_order(compare_reals(&radial_offset.norm_squared(), &Real::zero()))?
        != Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let sphere_radius_squared = &sphere.radius * &sphere.radius;
    let cylinder_radius_squared = &cylinder.radius * &cylinder.radius;
    let frames_match = orthonormal_frames_equal(&sphere.frame, &cylinder.frame)?;
    let retained_circle = |height: Real| -> GeometryResult<SurfaceIntersectionCurve> {
        let center = sphere.center.clone() + cylinder.frame.z.clone() * &height;
        let curve = Curve3::circle_arc(
            center,
            sphere.frame.x.clone(),
            sphere.frame.y.clone(),
            cylinder.radius.clone(),
            Real::zero(),
            Real::tau(),
        )?;
        let latitude = (height.clone() / &sphere.radius)
            .map_err(|_| GeometryError::ProjectiveDivision)?
            .asin()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        let cylinder_height = &axial_offset + height;
        let domain = curve.domain().clone();
        Ok(SurfaceIntersectionCurve::new(
            curve,
            SurfaceIntersectionPcurve::tensor_iso_v(domain.clone(), latitude),
            SurfaceIntersectionPcurve::tensor_iso_v(domain, cylinder_height),
        ))
    };
    match decided_order(compare_reals(
        &sphere_radius_squared,
        &cylinder_radius_squared,
    ))? {
        Ordering::Less => Ok(SurfaceSurfaceIntersection::None),
        Ordering::Equal => {
            if frames_match {
                Ok(SurfaceSurfaceIntersection::Curve(Box::new(
                    retained_circle(Real::zero())?,
                )))
            } else {
                Ok(SurfaceSurfaceIntersection::Circle(Curve3::circle_arc(
                    sphere.center.clone(),
                    cylinder.frame.x.clone(),
                    cylinder.frame.y.clone(),
                    cylinder.radius.clone(),
                    Real::zero(),
                    Real::tau(),
                )?))
            }
        }
        Ordering::Greater => {
            let axial_distance = (sphere_radius_squared - cylinder_radius_squared)
                .sqrt()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            if frames_match {
                return Ok(SurfaceSurfaceIntersection::Curves(vec![
                    retained_circle(axial_distance.clone())?,
                    retained_circle(-axial_distance)?,
                ]));
            }
            Ok(SurfaceSurfaceIntersection::Circles(vec![
                Curve3::circle_arc(
                    sphere.center.clone() + cylinder.frame.z.clone() * &axial_distance,
                    cylinder.frame.x.clone(),
                    cylinder.frame.y.clone(),
                    cylinder.radius.clone(),
                    Real::zero(),
                    Real::from(2) * Real::pi(),
                )?,
                Curve3::circle_arc(
                    sphere.center.clone() - cylinder.frame.z.clone() * axial_distance,
                    cylinder.frame.x.clone(),
                    cylinder.frame.y.clone(),
                    cylinder.radius.clone(),
                    Real::zero(),
                    Real::from(2) * Real::pi(),
                )?,
            ]))
        }
    }
}

fn intersect_coaxial_sphere_cone(
    sphere: &SphereSurface,
    cone: &ConeSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let center_offset = &sphere.center - &cone.apex;
    let axial_offset = center_offset.dot(&cone.frame.z);
    let radial_offset = center_offset - cone.frame.z.clone() * &axial_offset;
    if decided_order(compare_reals(&radial_offset.norm_squared(), &Real::zero()))?
        != Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }

    let sine = cone.semi_angle.clone().sin();
    let cosine = cone.semi_angle.clone().cos();
    let discriminant =
        &sphere.radius * &sphere.radius - &axial_offset * &axial_offset * &sine * &sine;
    let discriminant_order = decided_order(compare_reals(&discriminant, &Real::zero()))?;
    if discriminant_order == Ordering::Less {
        return Ok(SurfaceSurfaceIntersection::None);
    }
    let root_center = &axial_offset * &cosine;
    let root_parameters = if discriminant_order == Ordering::Equal {
        vec![root_center]
    } else {
        let root_offset = discriminant
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        vec![&root_center - &root_offset, root_center + root_offset]
    };
    let mut has_apex = false;
    let mut slant_parameters = Vec::with_capacity(root_parameters.len());
    for parameter in root_parameters {
        match decided_order(compare_reals(&parameter, &Real::zero()))? {
            Ordering::Less => {}
            Ordering::Equal => {
                has_apex = true;
            }
            Ordering::Greater => slant_parameters.push(parameter),
        }
    }
    if has_apex && !slant_parameters.is_empty() {
        return Err(GeometryError::UnsupportedIntersection);
    }
    if has_apex {
        return Ok(SurfaceSurfaceIntersection::Point(Box::new(
            cone.apex.clone(),
        )));
    }
    if slant_parameters.is_empty() {
        return Ok(SurfaceSurfaceIntersection::None);
    }

    let frames_match = orthonormal_frames_equal(&sphere.frame, &cone.frame)?;
    let mut circles = Vec::with_capacity(slant_parameters.len());
    let mut retained = Vec::with_capacity(slant_parameters.len());
    for parameter in slant_parameters {
        let axial_height = &parameter * &cosine;
        let radius = &parameter * &sine;
        let center = cone.apex.clone() + cone.frame.z.clone() * &axial_height;
        let curve = Curve3::circle_arc(
            center,
            cone.frame.x.clone(),
            cone.frame.y.clone(),
            radius,
            Real::zero(),
            Real::tau(),
        )?;
        if frames_match {
            let sphere_height = axial_height - &axial_offset;
            let latitude = (sphere_height / &sphere.radius)
                .map_err(|_| GeometryError::ProjectiveDivision)?
                .asin()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            let domain = curve.domain().clone();
            retained.push(SurfaceIntersectionCurve::new(
                curve,
                SurfaceIntersectionPcurve::tensor_iso_v(domain.clone(), latitude),
                SurfaceIntersectionPcurve::tensor_iso_v(domain, parameter),
            ));
        } else {
            circles.push(curve);
        }
    }
    if frames_match {
        Ok(match retained.len() {
            1 => SurfaceSurfaceIntersection::Curve(Box::new(
                retained.pop().expect("one retained sphere/cone circle"),
            )),
            _ => SurfaceSurfaceIntersection::Curves(retained),
        })
    } else {
        Ok(match circles.len() {
            1 => SurfaceSurfaceIntersection::Circle(circles.pop().expect("one sphere/cone circle")),
            _ => SurfaceSurfaceIntersection::Circles(circles),
        })
    }
}

fn intersect_coaxial_cylinder_cone(
    cylinder: &CylinderSurface,
    cone: &ConeSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    if decided_order(compare_reals(
        &cylinder.frame.z.cross(&cone.frame.z).norm_squared(),
        &Real::zero(),
    ))? != Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }
    let origin_offset = &cylinder.origin - &cone.apex;
    let radial_offset = &origin_offset - &(cone.frame.z.clone() * origin_offset.dot(&cone.frame.z));
    if decided_order(compare_reals(&radial_offset.norm_squared(), &Real::zero()))?
        != Ordering::Equal
    {
        return Err(GeometryError::UnsupportedIntersection);
    }

    let sine = cone.semi_angle.clone().sin();
    let cosine = cone.semi_angle.clone().cos();
    let slant_parameter =
        (&cylinder.radius / &sine).map_err(|_| GeometryError::ProjectiveDivision)?;
    let axial_height = &slant_parameter * cosine;
    let center = cone.apex.clone() + cone.frame.z.clone() * axial_height;
    let curve = Curve3::circle_arc(
        center.clone(),
        cone.frame.x.clone(),
        cone.frame.y.clone(),
        cylinder.radius.clone(),
        Real::zero(),
        Real::tau(),
    )?;
    if !orthonormal_frames_equal(&cylinder.frame, &cone.frame)? {
        return Ok(SurfaceSurfaceIntersection::Circle(curve));
    }
    let cylinder_height = (&center - &cylinder.origin).dot(&cylinder.frame.z);
    let domain = curve.domain().clone();
    Ok(SurfaceSurfaceIntersection::Curve(Box::new(
        SurfaceIntersectionCurve::new(
            curve,
            SurfaceIntersectionPcurve::tensor_iso_v(domain.clone(), cylinder_height),
            SurfaceIntersectionPcurve::tensor_iso_v(domain, slant_parameter),
        ),
    )))
}

fn orthonormal_frames_equal(
    first: &OrthonormalFrame3,
    second: &OrthonormalFrame3,
) -> GeometryResult<bool> {
    [&first.x, &first.y, &first.z]
        .into_iter()
        .zip([&second.x, &second.y, &second.z])
        .try_fold(true, |matches, (first_axis, second_axis)| {
            Ok(matches
                && decided_order(compare_reals(
                    &(first_axis - second_axis).norm_squared(),
                    &Real::zero(),
                ))? == Ordering::Equal)
        })
}

fn intersect_spheres(
    first: &SphereSurface,
    second: &SphereSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    match point3_equal(&first.center, &second.center) {
        PredicateOutcome::Decided { value: true, .. } => {
            return if decided_order(compare_reals(&first.radius, &second.radius))?
                == Ordering::Equal
            {
                Ok(SurfaceSurfaceIntersection::Coincident)
            } else {
                Ok(SurfaceSurfaceIntersection::None)
            };
        }
        PredicateOutcome::Decided { value: false, .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(GeometryError::PredicateUnresolved { needed, stage });
        }
    }
    let displacement = &second.center - &first.center;
    let distance_squared = displacement.norm_squared();
    let distance = distance_squared
        .clone()
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let radius_sum = &first.radius + &second.radius;
    let radius_difference = match decided_order(compare_reals(&first.radius, &second.radius))? {
        Ordering::Less => &second.radius - &first.radius,
        Ordering::Equal | Ordering::Greater => &first.radius - &second.radius,
    };
    if decided_order(compare_reals(&distance, &radius_sum))? == Ordering::Greater
        || decided_order(compare_reals(&distance, &radius_difference))? == Ordering::Less
    {
        return Ok(SurfaceSurfaceIntersection::None);
    }

    let denominator = Real::from(2) * &distance;
    let center_distance = ((&first.radius * &first.radius - &second.radius * &second.radius
        + &distance_squared)
        / denominator)
        .map_err(|_| GeometryError::ProjectiveDivision)?;
    let inverse_distance =
        (Real::one() / distance).map_err(|_| GeometryError::ProjectiveDivision)?;
    let normal = displacement * inverse_distance;
    let center = first.center.clone() + normal.clone() * &center_distance;
    let circle_radius_squared = &first.radius * &first.radius - &center_distance * &center_distance;
    match decided_order(compare_reals(&circle_radius_squared, &Real::zero()))? {
        Ordering::Less => Ok(SurfaceSurfaceIntersection::None),
        Ordering::Equal => Ok(SurfaceSurfaceIntersection::Point(Box::new(center))),
        Ordering::Greater => {
            let radius = circle_radius_squared
                .sqrt()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            let (x, y) = normal
                .orthonormal_basis_checked()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            Ok(SurfaceSurfaceIntersection::Circle(Curve3::circle_arc(
                center,
                x,
                y,
                radius,
                Real::zero(),
                Real::from(2) * Real::pi(),
            )?))
        }
    }
}

enum QuadraticRoots {
    None,
    All,
    Isolated(Vec<(Real, IntersectionMultiplicity)>),
}

fn quadratic_roots(a: Real, b: Real, c: Real) -> GeometryResult<QuadraticRoots> {
    if decided_order(compare_reals(&a, &Real::zero()))? == Ordering::Equal {
        if decided_order(compare_reals(&b, &Real::zero()))? == Ordering::Equal {
            return if decided_order(compare_reals(&c, &Real::zero()))? == Ordering::Equal {
                Ok(QuadraticRoots::All)
            } else {
                Ok(QuadraticRoots::None)
            };
        }
        let parameter = ((-c) / b).map_err(|_| GeometryError::ProjectiveDivision)?;
        return Ok(QuadraticRoots::Isolated(vec![(
            parameter,
            IntersectionMultiplicity::Simple,
        )]));
    }
    let discriminant = &b * &b - Real::from(4) * &a * &c;
    match decided_order(compare_reals(&discriminant, &Real::zero()))? {
        Ordering::Less => Ok(QuadraticRoots::None),
        Ordering::Equal => {
            let parameter =
                ((-b) / (Real::from(2) * a)).map_err(|_| GeometryError::ProjectiveDivision)?;
            Ok(QuadraticRoots::Isolated(vec![(
                parameter,
                IntersectionMultiplicity::Tangent,
            )]))
        }
        Ordering::Greater => {
            let root = discriminant
                .sqrt()
                .map_err(|_| GeometryError::ElementaryFunction)?;
            let denominator = Real::from(2) * a;
            let first =
                ((-&b - &root) / &denominator).map_err(|_| GeometryError::ProjectiveDivision)?;
            let second =
                ((-&b + root) / denominator).map_err(|_| GeometryError::ProjectiveDivision)?;
            Ok(QuadraticRoots::Isolated(vec![
                (first, IntersectionMultiplicity::Simple),
                (second, IntersectionMultiplicity::Simple),
            ]))
        }
    }
}

fn intersect_rational_bezier_plane(
    curve: &Curve3,
    bezier: &RationalBezier3,
    plane: &PlaneSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let normal = plane.u.cross(&plane.v);
    let coefficients = bezier
        .control_points
        .iter()
        .zip(&bezier.weights)
        .map(|(point, weight)| weight * normal.dot(&(point - &plane.origin)))
        .collect::<Vec<_>>();
    let roots = match coefficients.as_slice() {
        [start, end] => quadratic_roots(Real::zero(), end - start, start.clone())?,
        [start, middle, end] => quadratic_roots(
            start - Real::from(2) * middle + end,
            Real::from(2) * (middle - start),
            start.clone(),
        )?,
        _ => return Err(GeometryError::UnsupportedIntersection),
    };
    match roots {
        QuadraticRoots::None => Ok(CurveSurfaceIntersection::None),
        QuadraticRoots::All => Ok(CurveSurfaceIntersection::Contained),
        QuadraticRoots::Isolated(roots) => isolated_line_parameters(curve, roots),
    }
}

fn intersect_nurbs_plane(
    nurbs: &NurbsCurve3,
    plane: &PlaneSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let segments = decompose_nurbs_into_bezier_segments(nurbs)?;
    let segment_count = segments.len();
    let mut points: Vec<CurveSurfacePoint> = Vec::new();
    let mut contained_count = 0_usize;
    for (segment, domain) in segments {
        let CurveGeometry3::RationalBezier(bezier) = &segment.data.geometry else {
            unreachable!("NURBS decomposition produces rational Bézier segments");
        };
        match intersect_rational_bezier_plane(&segment, bezier, plane)? {
            CurveSurfaceIntersection::None => {}
            CurveSurfaceIntersection::Points(local_points) => {
                let span = domain.end() - domain.start();
                for local in local_points {
                    let parameter = domain.start() + &span * &local.parameter;
                    let mut duplicate = None;
                    for (index, existing) in points.iter().enumerate() {
                        if decided_order(compare_reals(&existing.parameter, &parameter))?
                            == Ordering::Equal
                        {
                            duplicate = Some(index);
                            break;
                        }
                    }
                    if let Some(index) = duplicate {
                        if local.multiplicity == IntersectionMultiplicity::Tangent {
                            points[index].multiplicity = IntersectionMultiplicity::Tangent;
                        }
                    } else {
                        points.push(CurveSurfacePoint {
                            parameter,
                            point: local.point,
                            multiplicity: local.multiplicity,
                        });
                    }
                }
            }
            CurveSurfaceIntersection::Contained => contained_count += 1,
            CurveSurfaceIntersection::Overlap(_) => {
                unreachable!("rational Bézier/plane dispatch returns contained spans")
            }
        }
    }
    if contained_count == segment_count {
        return Ok(CurveSurfaceIntersection::Contained);
    }
    if contained_count != 0 {
        return Err(GeometryError::UnsupportedIntersection);
    }
    if points.is_empty() {
        Ok(CurveSurfaceIntersection::None)
    } else {
        Ok(CurveSurfaceIntersection::Points(points))
    }
}

fn intersect_line_cone(
    curve: &Curve3,
    start: &Point3,
    end: &Point3,
    cone: &ConeSurface,
) -> GeometryResult<CurveSurfaceIntersection> {
    let offset = start - &cone.apex;
    let direction = end - start;
    let axial_offset = cone.frame.z.dot(&offset);
    let axial_direction = cone.frame.z.dot(&direction);
    let radial_offset = &offset - &(cone.frame.z.clone() * &axial_offset);
    let radial_direction = &direction - &(cone.frame.z.clone() * &axial_direction);
    let sin = cone.semi_angle.clone().sin();
    let cos = cone.semi_angle.clone().cos();
    let sin_squared = &sin * &sin;
    let cos_squared = &cos * &cos;
    let a = &cos_squared * radial_direction.norm_squared()
        - &sin_squared * &axial_direction * &axial_direction;
    let b = Real::from(2)
        * (&cos_squared * radial_offset.dot(&radial_direction)
            - &sin_squared * &axial_offset * &axial_direction);
    let c =
        &cos_squared * radial_offset.norm_squared() - &sin_squared * &axial_offset * &axial_offset;
    match quadratic_roots(a, b, c)? {
        QuadraticRoots::None => Ok(CurveSurfaceIntersection::None),
        QuadraticRoots::Isolated(roots) => {
            let mut upper_roots = Vec::new();
            for (parameter, multiplicity) in roots {
                let axial = &axial_offset + &parameter * &axial_direction;
                if decided_order(compare_reals(&axial, &Real::zero()))? != Ordering::Less {
                    upper_roots.push((parameter, multiplicity));
                }
            }
            isolated_line_parameters(curve, upper_roots)
        }
        QuadraticRoots::All => {
            let axial_start = axial_offset;
            let axial_end = &axial_start + &axial_direction;
            let start_order = decided_order(compare_reals(&axial_start, &Real::zero()))?;
            let end_order = decided_order(compare_reals(&axial_end, &Real::zero()))?;
            if start_order != Ordering::Less && end_order != Ordering::Less {
                return Ok(CurveSurfaceIntersection::Contained);
            }
            if start_order == Ordering::Less && end_order == Ordering::Less {
                return Ok(CurveSurfaceIntersection::None);
            }
            let boundary = ((-&axial_start) / &axial_direction)
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            if start_order == Ordering::Equal || end_order == Ordering::Equal {
                return isolated_line_parameters(
                    curve,
                    [(boundary, IntersectionMultiplicity::Simple)],
                );
            }
            let domain = if start_order == Ordering::Greater {
                ParameterDomain::new(curve.domain().start().clone(), boundary)?
            } else {
                ParameterDomain::new(boundary, curve.domain().end().clone())?
            };
            Ok(CurveSurfaceIntersection::Overlap(domain))
        }
    }
}

fn quadratic_line_intersection(
    curve: &Curve3,
    offset: &Vector3,
    direction: &Vector3,
    radius: &Real,
) -> GeometryResult<CurveSurfaceIntersection> {
    let a = direction.dot(direction);
    let b = Real::from(2) * offset.dot(direction);
    let c = offset.dot(offset) - radius * radius;
    match quadratic_roots(a, b, c)? {
        QuadraticRoots::None => Ok(CurveSurfaceIntersection::None),
        QuadraticRoots::All => Ok(CurveSurfaceIntersection::Contained),
        QuadraticRoots::Isolated(roots) => isolated_line_parameters(curve, roots),
    }
}

fn isolated_line_parameters(
    curve: &Curve3,
    parameters: impl IntoIterator<Item = (Real, IntersectionMultiplicity)>,
) -> GeometryResult<CurveSurfaceIntersection> {
    let mut points = Vec::new();
    for (parameter, multiplicity) in parameters {
        if curve.domain().contains(&parameter)? {
            points.push(CurveSurfacePoint {
                point: curve.point_at(&parameter)?,
                parameter,
                multiplicity,
            });
        }
    }
    if points.is_empty() {
        Ok(CurveSurfaceIntersection::None)
    } else {
        Ok(CurveSurfaceIntersection::Points(points))
    }
}

fn validate_control_net(control_points: &[Point3], weights: &[Real]) -> GeometryResult<()> {
    if control_points.len() < 2 {
        return Err(GeometryError::TooFewControlPoints);
    }
    if control_points.len() != weights.len() {
        return Err(GeometryError::WeightCountMismatch);
    }
    Ok(())
}

fn locate_line_parameter(
    curve: &Curve3,
    line: &Line3,
    point: &Point3,
) -> GeometryResult<CurveParameterLocation> {
    let direction = &line.end - &line.start;
    let offset = point - &line.start;
    let mut parameter = None;
    for axis in 0..3 {
        match decided_order(compare_reals(&direction.0[axis], &Real::zero()))? {
            Ordering::Equal => {}
            Ordering::Less | Ordering::Greater => {
                parameter = Some(
                    (&offset.0[axis] / &direction.0[axis])
                        .map_err(|_| GeometryError::ProjectiveDivision)?,
                );
                break;
            }
        }
    }
    let parameter = parameter.ok_or(GeometryError::DegenerateLine)?;
    if !curve.domain().contains(&parameter)? {
        return Ok(CurveParameterLocation::None);
    }
    if points_equal(&curve.point_at(&parameter)?, point)? {
        Ok(CurveParameterLocation::Parameters(vec![parameter]))
    } else {
        Ok(CurveParameterLocation::None)
    }
}

fn locate_rational_bezier_parameters(
    curve: &Curve3,
    rational: &RationalBezier3,
    point: &Point3,
) -> GeometryResult<CurveParameterLocation> {
    let mut projection = None;
    for axes in [(0, 1), (0, 2), (1, 2)] {
        if surface_projection_varies(&rational.control_points, axes)? {
            projection = Some(axes);
            break;
        }
    }
    let Some((first_axis, second_axis)) = projection else {
        return if points_equal(&rational.control_points[0], point)? {
            Ok(CurveParameterLocation::EntireDomain)
        } else {
            Ok(CurveParameterLocation::None)
        };
    };
    let controls = rational
        .control_points
        .iter()
        .map(|control| {
            CurvePoint2::new(
                control_coordinate(control, first_axis).clone(),
                control_coordinate(control, second_axis).clone(),
            )
        })
        .collect();
    let projected = RationalBezier2::try_new(controls, rational.weights.clone())?;
    let query = CurvePoint2::new(
        control_coordinate(point, first_axis).clone(),
        control_coordinate(point, second_axis).clone(),
    );
    match projected.point_incidence(&query, &CurvePolicy::certified())? {
        RationalBezierPointIncidence2::EntireCurve => {
            Err(GeometryError::UnsupportedParameterLocation)
        }
        RationalBezierPointIncidence2::Parameters(parameters) => {
            let mut represented = Vec::new();
            for parameter in parameters {
                let BezierParameter2::Exact(parameter) = parameter else {
                    return Err(GeometryError::UnrepresentableParameter);
                };
                if points_equal(&curve.point_at(&parameter)?, point)? {
                    represented.push(parameter);
                }
            }
            if represented.is_empty() {
                Ok(CurveParameterLocation::None)
            } else {
                Ok(CurveParameterLocation::Parameters(represented))
            }
        }
    }
}

fn locate_nurbs_parameters(
    nurbs: &NurbsCurve3,
    point: &Point3,
) -> GeometryResult<CurveParameterLocation> {
    let mut all_controls_equal = true;
    for control in &nurbs.control_points[1..] {
        if !points_equal(control, &nurbs.control_points[0])? {
            all_controls_equal = false;
            break;
        }
    }
    if all_controls_equal {
        return if points_equal(&nurbs.control_points[0], point)? {
            Ok(CurveParameterLocation::EntireDomain)
        } else {
            Ok(CurveParameterLocation::None)
        };
    }
    let segments = decompose_nurbs_into_bezier_segments(nurbs)?;
    let mut parameters = Vec::new();
    for (segment, domain) in segments {
        match segment.parameters_of(point)? {
            CurveParameterLocation::None => {}
            CurveParameterLocation::Parameters(local_parameters) => {
                let span = domain.end() - domain.start();
                for local in local_parameters {
                    let global = domain.start() + &span * local;
                    let mut duplicate = false;
                    for existing in &parameters {
                        if decided_order(compare_reals(existing, &global))? == Ordering::Equal {
                            duplicate = true;
                            break;
                        }
                    }
                    if !duplicate {
                        parameters.push(global);
                    }
                }
            }
            CurveParameterLocation::EntireDomain => {
                return Err(GeometryError::UnsupportedParameterLocation);
            }
        }
    }
    if parameters.is_empty() {
        Ok(CurveParameterLocation::None)
    } else {
        Ok(CurveParameterLocation::Parameters(parameters))
    }
}

fn decompose_nurbs_into_bezier_segments(
    nurbs: &NurbsCurve3,
) -> GeometryResult<Vec<(Curve3, ParameterDomain)>> {
    let mut breaks = vec![nurbs.knots[nurbs.degree].clone()];
    for knot in &nurbs.knots[(nurbs.degree + 1)..nurbs.control_points.len()] {
        if decided_order(compare_reals(knot, breaks.last().expect("start break")))?
            != Ordering::Equal
        {
            breaks.push(knot.clone());
        }
    }
    breaks.push(nurbs.knots[nurbs.control_points.len()].clone());

    let mut controls = nurbs.homogeneous_controls().to_vec();
    let mut knots = nurbs.knots.clone();
    for knot in &breaks[1..breaks.len() - 1] {
        let mut multiplicity = knot_multiplicity(&knots, knot)?;
        while multiplicity < nurbs.degree {
            let span = find_raw_span(knot, nurbs.degree, controls.len(), &knots)?;
            (controls, knots) =
                insert_nurbs_knot_once(&controls, &knots, nurbs.degree, span, multiplicity, knot)?;
            multiplicity += 1;
        }
    }
    let mut segments = Vec::with_capacity(breaks.len() - 1);
    for index in 0..breaks.len() - 1 {
        let control_start = index * nurbs.degree;
        let control_end = control_start + nurbs.degree;
        segments.push((
            rational_bezier_from_homogeneous(controls[control_start..=control_end].to_vec())?,
            ParameterDomain::new(breaks[index].clone(), breaks[index + 1].clone())?,
        ));
    }
    Ok(segments)
}

fn surface_projection_varies(
    control_points: &[Point3],
    axes: (usize, usize),
) -> GeometryResult<bool> {
    for point in &control_points[1..] {
        if decided_order(compare_reals(
            control_coordinate(point, axes.0),
            control_coordinate(&control_points[0], axes.0),
        ))? != Ordering::Equal
            || decided_order(compare_reals(
                control_coordinate(point, axes.1),
                control_coordinate(&control_points[0], axes.1),
            ))? != Ordering::Equal
        {
            return Ok(true);
        }
    }
    Ok(false)
}

fn control_coordinate(point: &Point3, axis: usize) -> &Real {
    match axis {
        0 => &point.x,
        1 => &point.y,
        2 => &point.z,
        _ => unreachable!("3D coordinate axis"),
    }
}

fn locate_ellipse_arc_parameters(
    curve: &Curve3,
    arc: &EllipseArc3,
    point: &Point3,
) -> GeometryResult<CurveParameterLocation> {
    let mut endpoints = Vec::new();
    if points_equal(&curve.start()?, point)? {
        endpoints.push(curve.domain().start().clone());
    }
    if points_equal(&curve.end()?, point)?
        && (endpoints.is_empty()
            || decided_order(compare_reals(&endpoints[0], curve.domain().end()))?
                != Ordering::Equal)
    {
        endpoints.push(curve.domain().end().clone());
    }
    if !endpoints.is_empty() {
        return Ok(CurveParameterLocation::Parameters(endpoints));
    }

    let relative = point - &arc.center;
    let x_coordinate = arc.x.dot(&relative);
    let y_coordinate = arc.y.dot(&relative);
    let represented = arc.x.clone() * &x_coordinate + arc.y.clone() * &y_coordinate;
    if decided_order(compare_reals(
        &(&relative - &represented).norm_squared(),
        &Real::zero(),
    ))? != Ordering::Equal
    {
        return Ok(CurveParameterLocation::None);
    }
    let normalized_x =
        (x_coordinate / &arc.x_radius).map_err(|_| GeometryError::ProjectiveDivision)?;
    let normalized_y =
        (y_coordinate / &arc.y_radius).map_err(|_| GeometryError::ProjectiveDivision)?;
    if decided_order(compare_reals(
        &(&normalized_x * &normalized_x + &normalized_y * &normalized_y),
        &Real::one(),
    ))? != Ordering::Equal
    {
        return Ok(CurveParameterLocation::None);
    }
    let start_sin = arc.angle_at_start.clone().sin();
    let start_cos = arc.angle_at_start.clone().cos();
    let cosine_delta = &start_cos * &normalized_x + &start_sin * &normalized_y;
    let mut sine_delta = &start_cos * &normalized_y - &start_sin * &normalized_x;
    if arc.direction < 0 {
        sine_delta = -sine_delta;
    }
    let mut delta = certified_atan2(sine_delta, cosine_delta)?;
    if decided_order(compare_reals(&delta, &Real::zero()))? == Ordering::Less {
        delta += Real::from(2) * Real::pi();
    }
    let parameter = curve.domain().start() + delta;
    if curve.domain().contains(&parameter)? && points_equal(&curve.point_at(&parameter)?, point)? {
        Ok(CurveParameterLocation::Parameters(vec![parameter]))
    } else {
        Ok(CurveParameterLocation::None)
    }
}

fn points_equal(left: &Point3, right: &Point3) -> GeometryResult<bool> {
    match point3_equal(left, right) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(GeometryError::PredicateUnresolved { needed, stage })
        }
    }
}

fn validate_surface_control_net(
    control_points: &[Vec<Point3>],
    weights: &[Vec<Real>],
) -> GeometryResult<(usize, usize)> {
    if control_points.len() < 2 {
        return Err(GeometryError::InvalidControlNetShape);
    }
    if control_points.len() != weights.len() {
        return Err(GeometryError::SurfaceWeightShapeMismatch);
    }
    let u_count = control_points[0].len();
    if u_count < 2 || control_points.iter().any(|row| row.len() != u_count) {
        return Err(GeometryError::InvalidControlNetShape);
    }
    if weights.iter().any(|row| row.len() != u_count) {
        return Err(GeometryError::SurfaceWeightShapeMismatch);
    }
    Ok((u_count, control_points.len()))
}

fn validate_positive_surface_weights(weights: &[Vec<Real>]) -> GeometryResult<()> {
    for row in weights {
        validate_positive_weights(row)?;
    }
    Ok(())
}

fn validate_nurbs_axis(degree: usize, control_count: usize, knots: &[Real]) -> GeometryResult<()> {
    if degree == 0 || degree >= control_count {
        return Err(GeometryError::InvalidDegree);
    }
    if knots.len() != control_count + degree + 1 {
        return Err(GeometryError::InvalidKnotCount);
    }
    for adjacent in knots.windows(2) {
        if !matches!(
            decided_order(compare_reals(&adjacent[0], &adjacent[1]))?,
            Ordering::Less | Ordering::Equal
        ) {
            return Err(GeometryError::InvalidKnotOrder);
        }
    }
    validate_clamped_knot_multiplicities(degree, knots)
}

fn validate_positive_weights(weights: &[Real]) -> GeometryResult<()> {
    for weight in weights {
        match compare_reals(weight, &Real::zero()) {
            PredicateOutcome::Decided {
                value: Ordering::Greater,
                ..
            } => {}
            PredicateOutcome::Decided { .. } => return Err(GeometryError::InvalidWeight),
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(GeometryError::PredicateUnresolved { needed, stage });
            }
        }
    }
    Ok(())
}

fn validate_clamped_knot_multiplicities(degree: usize, knots: &[Real]) -> GeometryResult<()> {
    let endpoint_count = degree + 1;
    for knot in &knots[1..endpoint_count] {
        if decided_order(compare_reals(knot, &knots[0]))? != Ordering::Equal {
            return Err(GeometryError::UnclampedNurbs);
        }
    }
    let last = knots.len() - 1;
    for knot in &knots[(last + 1 - endpoint_count)..last] {
        if decided_order(compare_reals(knot, &knots[last]))? != Ordering::Equal {
            return Err(GeometryError::UnclampedNurbs);
        }
    }
    let mut run = 1_usize;
    for index in endpoint_count..(knots.len() - endpoint_count) {
        if decided_order(compare_reals(&knots[index], &knots[index - 1]))? == Ordering::Equal {
            run += 1;
            if run > degree {
                return Err(GeometryError::InvalidKnotMultiplicity);
            }
        } else {
            run = 1;
        }
    }
    Ok(())
}

fn decided_order(outcome: PredicateOutcome<Ordering>) -> GeometryResult<Ordering> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(GeometryError::PredicateUnresolved { needed, stage })
        }
    }
}

pub(crate) fn certified_atan2(y: Real, x: Real) -> GeometryResult<Real> {
    let y_order = decided_order(compare_reals(&y, &Real::zero()))?;
    let x_order = decided_order(compare_reals(&x, &Real::zero()))?;
    match (y_order, x_order) {
        (Ordering::Equal, Ordering::Equal | Ordering::Greater) => Ok(Real::zero()),
        (Ordering::Equal, Ordering::Less) => Ok(Real::pi()),
        (Ordering::Greater, Ordering::Equal) => {
            (Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)
        }
        (Ordering::Less, Ordering::Equal) => {
            (-Real::pi() / Real::from(2)).map_err(|_| GeometryError::ProjectiveDivision)
        }
        (_, Ordering::Greater) => (y / x)
            .map_err(|_| GeometryError::ProjectiveDivision)?
            .atan()
            .map_err(|_| GeometryError::ElementaryFunction),
        (Ordering::Greater, Ordering::Less) => Ok((y / x)
            .map_err(|_| GeometryError::ProjectiveDivision)?
            .atan()
            .map_err(|_| GeometryError::ElementaryFunction)?
            + Real::pi()),
        (Ordering::Less, Ordering::Less) => Ok((y / x)
            .map_err(|_| GeometryError::ProjectiveDivision)?
            .atan()
            .map_err(|_| GeometryError::ElementaryFunction)?
            - Real::pi()),
    }
}

fn validate_arc_axes(x: &Vector3, y: &Vector3) -> GeometryResult<()> {
    let one = Real::one();
    let zero = Real::zero();
    for (actual, expected) in [
        (x.norm_squared(), &one),
        (y.norm_squared(), &one),
        (x.dot(y), &zero),
    ] {
        if decided_order(compare_reals(&actual, expected))? != Ordering::Equal {
            return Err(GeometryError::InvalidSurfaceFrame);
        }
    }
    Ok(())
}

fn validate_arc_domain(start: Real, end: Real) -> GeometryResult<ParameterDomain> {
    let domain = ParameterDomain::new(start, end)?;
    let sweep = domain.end() - domain.start();
    if decided_order(compare_reals(&sweep, &(Real::from(2) * Real::pi())))? == Ordering::Greater {
        Err(GeometryError::InvalidArcSweep)
    } else {
        Ok(domain)
    }
}

fn ellipse_arc_angle(arc: &EllipseArc3, domain: &ParameterDomain, parameter: &Real) -> Real {
    let delta = parameter - domain.start();
    if arc.direction > 0 {
        &arc.angle_at_start + delta
    } else {
        &arc.angle_at_start - delta
    }
}

fn evaluate_ellipse_arc(arc: &EllipseArc3, domain: &ParameterDomain, parameter: &Real) -> Point3 {
    let angle = ellipse_arc_angle(arc, domain, parameter);
    let sin = angle.clone().sin();
    let cos = angle.cos();
    arc.center.clone()
        + arc.x.clone() * (&arc.x_radius * cos)
        + arc.y.clone() * (&arc.y_radius * sin)
}

fn evaluate_ellipse_arc_derivative(
    arc: &EllipseArc3,
    domain: &ParameterDomain,
    parameter: &Real,
    order: usize,
) -> Vector3 {
    let angle = ellipse_arc_angle(arc, domain, parameter);
    let sin = angle.clone().sin();
    let cos = angle.cos();
    let (cos_derivative, sin_derivative) = match order % 4 {
        0 => (cos, sin),
        1 => (-sin, cos),
        2 => (-cos, -sin),
        3 => (sin, -cos),
        _ => unreachable!("modulo four"),
    };
    let derivative = arc.x.clone() * (&arc.x_radius * cos_derivative)
        + arc.y.clone() * (&arc.y_radius * sin_derivative);
    if arc.direction > 0 || order.is_multiple_of(2) {
        derivative
    } else {
        -derivative
    }
}

fn reversed_ellipse_arc(arc: &EllipseArc3, domain: &ParameterDomain) -> EllipseArc3 {
    EllipseArc3 {
        angle_at_start: ellipse_arc_angle(arc, domain, domain.end()),
        direction: -arc.direction,
        ..arc.clone()
    }
}

fn split_ellipse_arc(
    arc: &EllipseArc3,
    domain: &ParameterDomain,
    parameter: &Real,
    kind: Curve3Kind,
) -> GeometryResult<(Curve3, Curve3)> {
    let right_angle = ellipse_arc_angle(arc, domain, parameter);
    let left_geometry = arc.clone();
    let right_geometry = EllipseArc3 {
        angle_at_start: right_angle,
        ..arc.clone()
    };
    let left_domain = ParameterDomain::new(domain.start().clone(), parameter.clone())?;
    let right_domain = ParameterDomain::new(parameter.clone(), domain.end().clone())?;
    let wrap = |geometry| match kind {
        Curve3Kind::CircleArc => CurveGeometry3::CircleArc(geometry),
        Curve3Kind::EllipseArc => CurveGeometry3::EllipseArc(geometry),
        Curve3Kind::Line | Curve3Kind::RationalBezier | Curve3Kind::Nurbs => {
            unreachable!("ellipse split kind")
        }
    };
    Ok((
        Curve3::from_parts(wrap(left_geometry), left_domain),
        Curve3::from_parts(wrap(right_geometry), right_domain),
    ))
}

fn ellipse_arc_conservative_bounds(arc: &EllipseArc3) -> GeometryResult<Aabb> {
    let radius = if decided_order(compare_reals(&arc.x_radius, &arc.y_radius))? == Ordering::Greater
    {
        &arc.x_radius
    } else {
        &arc.y_radius
    };
    Ok(Aabb::new(
        Point3::new(
            &arc.center.x - radius,
            &arc.center.y - radius,
            &arc.center.z - radius,
        ),
        Point3::new(
            &arc.center.x + radius,
            &arc.center.y + radius,
            &arc.center.z + radius,
        ),
    ))
}

fn centered_cube_bounds(center: &Point3, radius: &Real) -> Aabb {
    Aabb::new(
        Point3::new(&center.x - radius, &center.y - radius, &center.z - radius),
        Point3::new(&center.x + radius, &center.y + radius, &center.z + radius),
    )
}

fn revolution_bounds(surface: &RevolutionSurface) -> GeometryResult<Aabb> {
    let bounds = surface.profile.bounds()?;
    let xs = [&bounds.mins.x, &bounds.maxs.x];
    let ys = [&bounds.mins.y, &bounds.maxs.y];
    let zs = [&bounds.mins.z, &bounds.maxs.z];
    let first_offset = Vector3::from_xyz(
        xs[0] - &surface.axis_origin.x,
        ys[0] - &surface.axis_origin.y,
        zs[0] - &surface.axis_origin.z,
    );
    let first_axial = surface.axis.dot(&first_offset);
    let first_radial = first_offset - surface.axis.clone() * first_axial.clone();
    let mut axial_min = first_axial.clone();
    let mut axial_max = first_axial;
    let mut radius_squared = first_radial.norm_squared();
    for x in xs {
        for y in ys {
            for z in zs {
                let offset = Vector3::from_xyz(
                    x - &surface.axis_origin.x,
                    y - &surface.axis_origin.y,
                    z - &surface.axis_origin.z,
                );
                let axial = surface.axis.dot(&offset);
                update_bound_min(&mut axial_min, &axial)?;
                update_bound_max(&mut axial_max, &axial)?;
                let radial = offset - surface.axis.clone() * axial;
                update_bound_max(&mut radius_squared, &radial.norm_squared())?;
            }
        }
    }
    let radius = radius_squared
        .sqrt()
        .map_err(|_| GeometryError::ElementaryFunction)?;
    let coordinate_bounds = |origin: &Real,
                             axis_component: &Real|
     -> GeometryResult<(Real, Real)> {
        let first_axial = axis_component * &axial_min;
        let second_axial = axis_component * &axial_max;
        let (axial_low, axial_high) =
            if decided_order(compare_reals(&first_axial, &second_axial))? == Ordering::Greater {
                (second_axial, first_axial)
            } else {
                (first_axial, second_axial)
            };
        let radial_factor = (Real::one() - axis_component * axis_component)
            .sqrt()
            .map_err(|_| GeometryError::ElementaryFunction)?;
        let radial_extent = &radius * radial_factor;
        Ok((
            origin + axial_low - &radial_extent,
            origin + axial_high + radial_extent,
        ))
    };
    let (min_x, max_x) = coordinate_bounds(&surface.axis_origin.x, &surface.axis.0[0])?;
    let (min_y, max_y) = coordinate_bounds(&surface.axis_origin.y, &surface.axis.0[1])?;
    let (min_z, max_z) = coordinate_bounds(&surface.axis_origin.z, &surface.axis.0[2])?;
    Ok(Aabb::new(
        Point3::new(min_x, min_y, min_z),
        Point3::new(max_x, max_y, max_z),
    ))
}

fn rotate_point_about_axis(
    point: &Point3,
    axis_origin: &Point3,
    axis: &Vector3,
    angle: &Real,
) -> Point3 {
    let relative = point - axis_origin;
    axis_origin.clone() + rotate_vector_about_axis(&relative, axis, angle)
}

fn rotate_vector_about_axis(vector: &Vector3, axis: &Vector3, angle: &Real) -> Vector3 {
    let axial = axis.clone() * axis.dot(vector);
    let radial = vector - &axial;
    let sin = angle.clone().sin();
    let cos = angle.clone().cos();
    axial + radial.clone() * cos + axis.cross(&radial) * sin
}

fn exact_surface_control_bounds(control_points: &[Vec<Point3>]) -> GeometryResult<Aabb> {
    let points = control_points
        .iter()
        .flat_map(|row| row.iter())
        .cloned()
        .collect::<Vec<_>>();
    exact_point_bounds(&points)
}

fn exact_point_bounds(points: &[Point3]) -> GeometryResult<Aabb> {
    let first = &points[0];
    let mut mins = first.clone();
    let mut maxs = first.clone();
    for point in &points[1..] {
        update_bound_min(&mut mins.x, &point.x)?;
        update_bound_min(&mut mins.y, &point.y)?;
        update_bound_min(&mut mins.z, &point.z)?;
        update_bound_max(&mut maxs.x, &point.x)?;
        update_bound_max(&mut maxs.y, &point.y)?;
        update_bound_max(&mut maxs.z, &point.z)?;
    }
    Ok(Aabb::new(mins, maxs))
}

fn update_bound_min(current: &mut Real, candidate: &Real) -> GeometryResult<()> {
    if decided_order(compare_reals(candidate, current))? == Ordering::Less {
        *current = candidate.clone();
    }
    Ok(())
}

fn update_bound_max(current: &mut Real, candidate: &Real) -> GeometryResult<()> {
    if decided_order(compare_reals(candidate, current))? == Ordering::Greater {
        *current = candidate.clone();
    }
    Ok(())
}

fn weighted_controls(points: &[Point3], weights: &[Real]) -> Vec<HomogeneousPoint3> {
    points
        .iter()
        .zip(weights)
        .map(|(point, weight)| HomogeneousPoint3::from_affine(point, weight))
        .collect()
}

fn evaluate_tensor_bezier(
    controls: &[Vec<HomogeneousPoint3>],
    parameter: &Point2,
) -> GeometryResult<Point3> {
    let u_values = controls
        .iter()
        .map(|row| {
            evaluate_homogeneous_bezier_value(row, &parameter.x)
                .ok_or(GeometryError::InvalidControlNetShape)
        })
        .collect::<GeometryResult<Vec<_>>>()?;
    evaluate_homogeneous_bezier_value(&u_values, &parameter.y)
        .ok_or(GeometryError::InvalidControlNetShape)?
        .project()
}

fn evaluate_tensor_bezier_partials(
    controls: &[Vec<HomogeneousPoint3>],
    parameter: &Point2,
) -> GeometryResult<SurfacePartials> {
    let u_values = controls
        .iter()
        .map(|row| {
            evaluate_homogeneous_bezier_value(row, &parameter.x)
                .ok_or(GeometryError::InvalidControlNetShape)
        })
        .collect::<GeometryResult<Vec<_>>>()?;
    let u_derivatives = controls
        .iter()
        .map(|row| {
            evaluate_homogeneous_bezier_derivative_value(row, &parameter.x)
                .ok_or(GeometryError::InvalidControlNetShape)
        })
        .collect::<GeometryResult<Vec<_>>>()?;
    let value = evaluate_homogeneous_bezier_value(&u_values, &parameter.y)
        .ok_or(GeometryError::InvalidControlNetShape)?;
    let u_derivative = evaluate_homogeneous_bezier_value(&u_derivatives, &parameter.y)
        .ok_or(GeometryError::InvalidControlNetShape)?;
    let v_derivative = evaluate_homogeneous_bezier_derivative_value(&u_values, &parameter.y)
        .ok_or(GeometryError::InvalidControlNetShape)?;
    Ok(SurfacePartials {
        u: project_homogeneous_derivative(&value, &u_derivative)?,
        v: project_homogeneous_derivative(&value, &v_derivative)?,
    })
}

fn evaluate_tensor_nurbs(
    surface: &NurbsSurface,
    domain: &SurfaceDomain,
    parameter: &Point2,
) -> GeometryResult<Point3> {
    let (value, _, _) = evaluate_tensor_nurbs_homogeneous(surface, domain, parameter)?;
    value.project()
}

fn evaluate_tensor_nurbs_partials(
    surface: &NurbsSurface,
    domain: &SurfaceDomain,
    parameter: &Point2,
) -> GeometryResult<SurfacePartials> {
    let (value, u_derivative, v_derivative) =
        evaluate_tensor_nurbs_homogeneous(surface, domain, parameter)?;
    Ok(SurfacePartials {
        u: project_homogeneous_derivative(&value, &u_derivative)?,
        v: project_homogeneous_derivative(&value, &v_derivative)?,
    })
}

fn evaluate_tensor_nurbs_homogeneous(
    surface: &NurbsSurface,
    domain: &SurfaceDomain,
    parameter: &Point2,
) -> GeometryResult<(HomogeneousPoint3, HomogeneousPoint3, HomogeneousPoint3)> {
    let SurfaceParameterDomain::Closed(u_domain) = &domain.u else {
        return Err(GeometryError::InvalidParameterDomain);
    };
    let SurfaceParameterDomain::Closed(v_domain) = &domain.v else {
        return Err(GeometryError::InvalidParameterDomain);
    };
    let controls = surface.homogeneous_controls();
    let u_count = controls[0].len();
    let v_count = controls.len();
    let u_span = find_span(
        &parameter.x,
        u_domain,
        surface.u_degree,
        u_count,
        &surface.u_knots,
    )?;
    let v_span = find_span(
        &parameter.y,
        v_domain,
        surface.v_degree,
        v_count,
        &surface.v_knots,
    )?;
    let u_jets = controls
        .iter()
        .map(|row| {
            evaluate_homogeneous_de_boor_jet(
                row,
                &surface.u_knots,
                surface.u_degree,
                u_span,
                &parameter.x,
            )
        })
        .collect::<GeometryResult<Vec<_>>>()?;
    let u_values = u_jets
        .iter()
        .map(|(value, _)| value.clone())
        .collect::<Vec<_>>();
    let u_derivatives = u_jets
        .into_iter()
        .map(|(_, derivative)| derivative)
        .collect::<Vec<_>>();
    let (value, v_derivative) = evaluate_homogeneous_de_boor_jet(
        &u_values,
        &surface.v_knots,
        surface.v_degree,
        v_span,
        &parameter.y,
    )?;
    let (u_derivative, _) = evaluate_homogeneous_de_boor_jet(
        &u_derivatives,
        &surface.v_knots,
        surface.v_degree,
        v_span,
        &parameter.y,
    )?;
    Ok((value, u_derivative, v_derivative))
}

fn evaluate_homogeneous_bezier(
    controls: &[HomogeneousPoint3],
    parameter: &Real,
) -> GeometryResult<Point3> {
    evaluate_homogeneous_bezier_value(controls, parameter)
        .ok_or(GeometryError::TooFewControlPoints)?
        .project()
}

fn evaluate_homogeneous_bezier_value(
    controls: &[HomogeneousPoint3],
    parameter: &Real,
) -> Option<HomogeneousPoint3> {
    let mut level = controls.to_vec();
    while level.len() > 1 {
        level = level
            .windows(2)
            .map(|pair| pair[0].lerp(&pair[1], parameter))
            .collect();
    }
    level.pop()
}

fn evaluate_homogeneous_bezier_derivative_value(
    controls: &[HomogeneousPoint3],
    parameter: &Real,
) -> Option<HomogeneousPoint3> {
    if controls.len() < 2 {
        return None;
    }
    let degree = Real::from((controls.len() - 1) as u64);
    let derivative_controls = controls
        .windows(2)
        .map(|pair| HomogeneousPoint3 {
            x: (&pair[1].x - &pair[0].x) * &degree,
            y: (&pair[1].y - &pair[0].y) * &degree,
            z: (&pair[1].z - &pair[0].z) * &degree,
            w: (&pair[1].w - &pair[0].w) * &degree,
        })
        .collect::<Vec<_>>();
    evaluate_homogeneous_bezier_value(&derivative_controls, parameter)
}

fn evaluate_rational_bezier_derivative(
    curve: &RationalBezier3,
    parameter: &Real,
    order: usize,
) -> GeometryResult<Vector3> {
    let mut derivative_controls = curve.homogeneous_controls().to_vec();
    let degree = derivative_controls.len() - 1;
    let mut jets = Vec::with_capacity(order + 1);
    jets.push(
        evaluate_homogeneous_bezier_value(&derivative_controls, parameter)
            .ok_or(GeometryError::TooFewControlPoints)?,
    );
    for derivative_order in 1..=order {
        if derivative_order > degree {
            jets.push(homogeneous_zero());
            continue;
        }
        let factor = Real::from((degree - derivative_order + 1) as u64);
        derivative_controls = derivative_controls
            .windows(2)
            .map(|pair| homogeneous_difference_scaled(&pair[1], &pair[0], &factor))
            .collect();
        jets.push(
            evaluate_homogeneous_bezier_value(&derivative_controls, parameter)
                .ok_or(GeometryError::TooFewControlPoints)?,
        );
    }
    project_homogeneous_derivative_order(&jets, order)
}

fn find_span(
    parameter: &Real,
    domain: &ParameterDomain,
    degree: usize,
    control_count: usize,
    knots: &[Real],
) -> GeometryResult<usize> {
    if decided_order(compare_reals(parameter, domain.end()))? == Ordering::Equal {
        return Ok(control_count - 1);
    }
    for span in degree..control_count {
        let after_start = decided_order(compare_reals(&knots[span], parameter))?;
        let before_end = decided_order(compare_reals(parameter, &knots[span + 1]))?;
        if matches!(after_start, Ordering::Less | Ordering::Equal) && before_end == Ordering::Less {
            return Ok(span);
        }
    }
    Err(GeometryError::InvalidParameterDomain)
}

fn evaluate_homogeneous_de_boor(
    controls: &[HomogeneousPoint3],
    knots: &[Real],
    degree: usize,
    span: usize,
    parameter: &Real,
) -> GeometryResult<Point3> {
    evaluate_homogeneous_de_boor_jet(controls, knots, degree, span, parameter)?
        .0
        .project()
}

fn evaluate_homogeneous_de_boor_derivative(
    controls: &[HomogeneousPoint3],
    knots: &[Real],
    degree: usize,
    span: usize,
    parameter: &Real,
    order: usize,
) -> GeometryResult<Vector3> {
    let jets = evaluate_homogeneous_de_boor_jets(controls, knots, degree, span, parameter, order)?;
    project_homogeneous_derivative_order(&jets, order)
}

fn evaluate_homogeneous_de_boor_jet(
    controls: &[HomogeneousPoint3],
    knots: &[Real],
    degree: usize,
    span: usize,
    parameter: &Real,
) -> GeometryResult<(HomogeneousPoint3, HomogeneousPoint3)> {
    let jets = evaluate_homogeneous_de_boor_jets(controls, knots, degree, span, parameter, 1)?;
    Ok((jets[0].clone(), jets[1].clone()))
}

fn evaluate_homogeneous_de_boor_jets(
    controls: &[HomogeneousPoint3],
    knots: &[Real],
    degree: usize,
    span: usize,
    parameter: &Real,
    order: usize,
) -> GeometryResult<Vec<HomogeneousPoint3>> {
    let zero = homogeneous_zero();
    let mut level = controls[(span - degree)..=span]
        .iter()
        .cloned()
        .map(|value| {
            let mut jets = vec![zero.clone(); order + 1];
            jets[0] = value;
            jets
        })
        .collect::<Vec<_>>();
    for stage in 1..=degree {
        for local in (stage..=degree).rev() {
            let knot_index = span - degree + local;
            let denominator = &knots[knot_index + degree - stage + 1] - &knots[knot_index];
            let alpha = ((parameter - &knots[knot_index]) / &denominator)
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            let alpha_derivative =
                (Real::one() / &denominator).map_err(|_| GeometryError::ProjectiveDivision)?;
            let left = level[local - 1].clone();
            let right = level[local].clone();
            let mut jets = Vec::with_capacity(order + 1);
            jets.push(left[0].lerp(&right[0], &alpha));
            for derivative_order in 1..=order {
                let base = left[derivative_order].lerp(&right[derivative_order], &alpha);
                let factor = Real::from(derivative_order as u64) * &alpha_derivative;
                jets.push(homogeneous_add(
                    &base,
                    &homogeneous_difference_scaled(
                        &right[derivative_order - 1],
                        &left[derivative_order - 1],
                        &factor,
                    ),
                ));
            }
            level[local] = jets;
        }
    }
    Ok(level[degree].clone())
}

fn homogeneous_zero() -> HomogeneousPoint3 {
    HomogeneousPoint3 {
        x: Real::zero(),
        y: Real::zero(),
        z: Real::zero(),
        w: Real::zero(),
    }
}

fn homogeneous_difference_scaled(
    left: &HomogeneousPoint3,
    right: &HomogeneousPoint3,
    factor: &Real,
) -> HomogeneousPoint3 {
    HomogeneousPoint3 {
        x: (&left.x - &right.x) * factor,
        y: (&left.y - &right.y) * factor,
        z: (&left.z - &right.z) * factor,
        w: (&left.w - &right.w) * factor,
    }
}

fn homogeneous_add(left: &HomogeneousPoint3, right: &HomogeneousPoint3) -> HomogeneousPoint3 {
    HomogeneousPoint3 {
        x: &left.x + &right.x,
        y: &left.y + &right.y,
        z: &left.z + &right.z,
        w: &left.w + &right.w,
    }
}

fn project_homogeneous_derivative_order(
    jets: &[HomogeneousPoint3],
    order: usize,
) -> GeometryResult<Vector3> {
    let mut derivatives = Vec::with_capacity(order + 1);
    derivatives.push(divide_vector_by_real(
        Vector3::from_xyz(jets[0].x.clone(), jets[0].y.clone(), jets[0].z.clone()),
        &jets[0].w,
    )?);
    for derivative_order in 1..=order {
        let mut numerator = Vector3::from_xyz(
            jets[derivative_order].x.clone(),
            jets[derivative_order].y.clone(),
            jets[derivative_order].z.clone(),
        );
        for weight_order in 1..=derivative_order {
            let coefficient =
                binomial_real(derivative_order, weight_order)? * &jets[weight_order].w;
            numerator =
                numerator - derivatives[derivative_order - weight_order].clone() * coefficient;
        }
        derivatives.push(divide_vector_by_real(numerator, &jets[0].w)?);
    }
    Ok(derivatives.pop().expect("positive derivative order"))
}

fn divide_vector_by_real(vector: Vector3, denominator: &Real) -> GeometryResult<Vector3> {
    Ok(Vector3::from_xyz(
        (vector.0[0].clone() / denominator).map_err(|_| GeometryError::ProjectiveDivision)?,
        (vector.0[1].clone() / denominator).map_err(|_| GeometryError::ProjectiveDivision)?,
        (vector.0[2].clone() / denominator).map_err(|_| GeometryError::ProjectiveDivision)?,
    ))
}

fn binomial_real(n: usize, k: usize) -> GeometryResult<Real> {
    let k = k.min(n - k);
    let mut result = Real::one();
    for index in 1..=k {
        result = (result * Real::from((n + 1 - index) as u64) / Real::from(index as u64))
            .map_err(|_| GeometryError::ProjectiveDivision)?;
    }
    Ok(result)
}

fn project_homogeneous_derivative(
    value: &HomogeneousPoint3,
    derivative: &HomogeneousPoint3,
) -> GeometryResult<Vector3> {
    let denominator = &value.w * &value.w;
    Ok(Vector3::from_xyz(
        ((&derivative.x * &value.w - &value.x * &derivative.w) / &denominator)
            .map_err(|_| GeometryError::ProjectiveDivision)?,
        ((&derivative.y * &value.w - &value.y * &derivative.w) / &denominator)
            .map_err(|_| GeometryError::ProjectiveDivision)?,
        ((&derivative.z * &value.w - &value.z * &derivative.w) / denominator)
            .map_err(|_| GeometryError::ProjectiveDivision)?,
    ))
}

fn require_interior_parameter(domain: &ParameterDomain, parameter: &Real) -> GeometryResult<()> {
    let after_start = decided_order(compare_reals(parameter, domain.start()))?;
    let before_end = decided_order(compare_reals(parameter, domain.end()))?;
    if after_start == Ordering::Greater && before_end == Ordering::Less {
        Ok(())
    } else {
        Err(GeometryError::SplitAtBoundary)
    }
}

fn split_homogeneous_bezier(
    controls: &[HomogeneousPoint3],
    parameter: &Real,
) -> (Vec<HomogeneousPoint3>, Vec<HomogeneousPoint3>) {
    let mut level = controls.to_vec();
    let mut left = Vec::with_capacity(controls.len());
    let mut right = Vec::with_capacity(controls.len());
    left.push(level[0].clone());
    right.push(level[level.len() - 1].clone());
    while level.len() > 1 {
        level = level
            .windows(2)
            .map(|pair| pair[0].lerp(&pair[1], parameter))
            .collect();
        left.push(level[0].clone());
        right.push(level[level.len() - 1].clone());
    }
    right.reverse();
    (left, right)
}

fn rational_bezier_from_homogeneous(controls: Vec<HomogeneousPoint3>) -> GeometryResult<Curve3> {
    let mut points = Vec::with_capacity(controls.len());
    let mut weights = Vec::with_capacity(controls.len());
    for control in controls {
        points.push(control.project()?);
        weights.push(control.w);
    }
    Curve3::rational_bezier(points, weights)
}

fn split_rational_bezier_surface_u(
    surface: &RationalBezierSurface,
    parameter: &Real,
) -> GeometryResult<(Surface, Surface)> {
    let mut left = Vec::with_capacity(surface.control_points.len());
    let mut right = Vec::with_capacity(surface.control_points.len());
    for row in surface.homogeneous_controls() {
        let (left_row, right_row) = split_homogeneous_bezier(row, parameter);
        left.push(left_row);
        right.push(right_row);
    }
    Ok((
        rational_bezier_surface_from_homogeneous(left)?,
        rational_bezier_surface_from_homogeneous(right)?,
    ))
}

fn split_rational_bezier_surface_v(
    surface: &RationalBezierSurface,
    parameter: &Real,
) -> GeometryResult<(Surface, Surface)> {
    let transposed = transpose_homogeneous_grid(surface.homogeneous_controls());
    let mut left = Vec::with_capacity(transposed.len());
    let mut right = Vec::with_capacity(transposed.len());
    for column in &transposed {
        let (left_column, right_column) = split_homogeneous_bezier(column, parameter);
        left.push(left_column);
        right.push(right_column);
    }
    Ok((
        rational_bezier_surface_from_homogeneous(transpose_homogeneous_grid(&left))?,
        rational_bezier_surface_from_homogeneous(transpose_homogeneous_grid(&right))?,
    ))
}

fn rational_bezier_surface_from_homogeneous(
    controls: Vec<Vec<HomogeneousPoint3>>,
) -> GeometryResult<Surface> {
    let (points, weights) = project_homogeneous_grid(controls)?;
    Surface::rational_bezier(points, weights)
}

fn split_nurbs_surface_u(
    surface: &NurbsSurface,
    parameter: &Real,
) -> GeometryResult<(Surface, Surface)> {
    let mut left = Vec::with_capacity(surface.control_points.len());
    let mut right = Vec::with_capacity(surface.control_points.len());
    let mut left_knots = None;
    let mut right_knots = None;
    for row in surface.homogeneous_controls() {
        let (left_row, row_left_knots, right_row, row_right_knots) = split_nurbs_homogeneous(
            surface.u_degree,
            row.clone(),
            surface.u_knots.clone(),
            parameter,
        )?;
        left.push(left_row);
        right.push(right_row);
        left_knots.get_or_insert(row_left_knots);
        right_knots.get_or_insert(row_right_knots);
    }
    Ok((
        nurbs_surface_from_homogeneous(
            surface.u_degree,
            surface.v_degree,
            left,
            left_knots.expect("nonempty control net"),
            surface.v_knots.clone(),
        )?,
        nurbs_surface_from_homogeneous(
            surface.u_degree,
            surface.v_degree,
            right,
            right_knots.expect("nonempty control net"),
            surface.v_knots.clone(),
        )?,
    ))
}

fn split_nurbs_surface_v(
    surface: &NurbsSurface,
    parameter: &Real,
) -> GeometryResult<(Surface, Surface)> {
    let transposed = transpose_homogeneous_grid(surface.homogeneous_controls());
    let mut left = Vec::with_capacity(transposed.len());
    let mut right = Vec::with_capacity(transposed.len());
    let mut left_knots = None;
    let mut right_knots = None;
    for column in transposed {
        let (left_column, column_left_knots, right_column, column_right_knots) =
            split_nurbs_homogeneous(surface.v_degree, column, surface.v_knots.clone(), parameter)?;
        left.push(left_column);
        right.push(right_column);
        left_knots.get_or_insert(column_left_knots);
        right_knots.get_or_insert(column_right_knots);
    }
    Ok((
        nurbs_surface_from_homogeneous(
            surface.u_degree,
            surface.v_degree,
            transpose_homogeneous_grid(&left),
            surface.u_knots.clone(),
            left_knots.expect("nonempty control net"),
        )?,
        nurbs_surface_from_homogeneous(
            surface.u_degree,
            surface.v_degree,
            transpose_homogeneous_grid(&right),
            surface.u_knots.clone(),
            right_knots.expect("nonempty control net"),
        )?,
    ))
}

fn nurbs_surface_from_homogeneous(
    u_degree: usize,
    v_degree: usize,
    controls: Vec<Vec<HomogeneousPoint3>>,
    u_knots: Vec<Real>,
    v_knots: Vec<Real>,
) -> GeometryResult<Surface> {
    let (points, weights) = project_homogeneous_grid(controls)?;
    Surface::nurbs(u_degree, v_degree, points, weights, u_knots, v_knots)
}

fn project_homogeneous_grid(
    controls: Vec<Vec<HomogeneousPoint3>>,
) -> GeometryResult<AffineControlGrid> {
    let mut points = Vec::with_capacity(controls.len());
    let mut weights = Vec::with_capacity(controls.len());
    for row in controls {
        let mut point_row = Vec::with_capacity(row.len());
        let mut weight_row = Vec::with_capacity(row.len());
        for control in row {
            point_row.push(control.project()?);
            weight_row.push(control.w);
        }
        points.push(point_row);
        weights.push(weight_row);
    }
    Ok((points, weights))
}

fn transpose_homogeneous_grid(controls: &[Vec<HomogeneousPoint3>]) -> Vec<Vec<HomogeneousPoint3>> {
    (0..controls[0].len())
        .map(|u| controls.iter().map(|row| row[u].clone()).collect())
        .collect()
}

fn split_nurbs_curve(curve: &NurbsCurve3, parameter: &Real) -> GeometryResult<(Curve3, Curve3)> {
    let (left_controls, left_knots, right_controls, right_knots) = split_nurbs_homogeneous(
        curve.degree,
        curve.homogeneous_controls().to_vec(),
        curve.knots.clone(),
        parameter,
    )?;
    Ok((
        nurbs_from_homogeneous(curve.degree, left_controls, left_knots)?,
        nurbs_from_homogeneous(curve.degree, right_controls, right_knots)?,
    ))
}

fn split_nurbs_homogeneous(
    degree: usize,
    mut controls: Vec<HomogeneousPoint3>,
    mut knots: Vec<Real>,
    parameter: &Real,
) -> GeometryResult<HomogeneousNurbsSplit> {
    let mut multiplicity = knot_multiplicity(&knots, parameter)?;
    if multiplicity > degree {
        return Err(GeometryError::InvalidKnotMultiplicity);
    }
    while multiplicity < degree {
        let span = find_raw_span(parameter, degree, controls.len(), &knots)?;
        let (next_controls, next_knots) =
            insert_nurbs_knot_once(&controls, &knots, degree, span, multiplicity, parameter)?;
        controls = next_controls;
        knots = next_knots;
        multiplicity += 1;
    }
    let span = find_raw_span(parameter, degree, controls.len(), &knots)?;
    let shared = span - degree;
    let left_controls = controls[..=shared].to_vec();
    let right_controls = controls[shared..].to_vec();
    let mut left_knots = knots[..=span].to_vec();
    left_knots.push(parameter.clone());
    let mut right_knots = Vec::with_capacity(knots.len() - (span - degree));
    right_knots.push(parameter.clone());
    right_knots.extend_from_slice(&knots[(span - degree + 1)..]);
    Ok((left_controls, left_knots, right_controls, right_knots))
}

fn knot_multiplicity(knots: &[Real], parameter: &Real) -> GeometryResult<usize> {
    let mut count = 0;
    for knot in knots {
        if decided_order(compare_reals(knot, parameter))? == Ordering::Equal {
            count += 1;
        }
    }
    Ok(count)
}

fn find_raw_span(
    parameter: &Real,
    degree: usize,
    control_count: usize,
    knots: &[Real],
) -> GeometryResult<usize> {
    for span in degree..control_count {
        let after_start = decided_order(compare_reals(&knots[span], parameter))?;
        let before_end = decided_order(compare_reals(parameter, &knots[span + 1]))?;
        if matches!(after_start, Ordering::Less | Ordering::Equal) && before_end == Ordering::Less {
            return Ok(span);
        }
    }
    Err(GeometryError::InvalidParameterDomain)
}

fn insert_nurbs_knot_once(
    controls: &[HomogeneousPoint3],
    knots: &[Real],
    degree: usize,
    span: usize,
    multiplicity: usize,
    parameter: &Real,
) -> GeometryResult<(Vec<HomogeneousPoint3>, Vec<Real>)> {
    let old_last = controls.len() - 1;
    let mut next = Vec::with_capacity(controls.len() + 1);
    for index in 0..=old_last + 1 {
        let value = if index <= span - degree {
            controls[index].clone()
        } else if index > span - multiplicity {
            controls[index - 1].clone()
        } else {
            let denominator = &knots[index + degree] - &knots[index];
            let alpha = ((parameter - &knots[index]) / denominator)
                .map_err(|_| GeometryError::ProjectiveDivision)?;
            controls[index - 1].lerp(&controls[index], &alpha)
        };
        next.push(value);
    }
    let mut next_knots = Vec::with_capacity(knots.len() + 1);
    next_knots.extend_from_slice(&knots[..=span]);
    next_knots.push(parameter.clone());
    next_knots.extend_from_slice(&knots[(span + 1)..]);
    Ok((next, next_knots))
}

fn nurbs_from_homogeneous(
    degree: usize,
    controls: Vec<HomogeneousPoint3>,
    knots: Vec<Real>,
) -> GeometryResult<Curve3> {
    let mut points = Vec::with_capacity(controls.len());
    let mut weights = Vec::with_capacity(controls.len());
    for control in controls {
        points.push(control.project()?);
        weights.push(control.w);
    }
    Curve3::nurbs(degree, points, weights, knots)
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperlimit::point3_equal;
    use hyperreal::Rational;

    fn r(value: i32) -> Real {
        Real::from(value)
    }

    fn q(numerator: i64, denominator: u64) -> Real {
        Real::new(Rational::fraction(numerator, denominator).unwrap())
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    fn assert_points_equal(left: &Point3, right: &Point3) {
        assert_eq!(point3_equal(left, right).value(), Some(true));
    }

    #[test]
    fn line_uses_real_domain_and_exact_evaluation() {
        let curve = Curve3::line(p(0, 0, 0), p(2, 4, 6)).unwrap();
        assert_eq!(curve.kind(), Curve3Kind::Line);
        assert_points_equal(&curve.point_at(&q(1, 2)).unwrap(), &p(1, 2, 3));
        assert_eq!(
            curve.point_at(&r(2)),
            Err(GeometryError::ParameterOutsideDomain)
        );
    }

    #[test]
    fn rational_bezier_evaluates_homogeneously() {
        let curve =
            Curve3::rational_bezier(vec![p(0, 0, 0), p(2, 4, 6)], vec![r(1), r(1)]).unwrap();
        assert_points_equal(&curve.point_at(&q(1, 2)).unwrap(), &p(1, 2, 3));
        let derivative = curve.derivative_at(&q(1, 3), 1).unwrap();
        assert_eq!(derivative.vector().0, [r(2), r(4), r(6)]);
        let bounds = curve.bounds().unwrap();
        assert_points_equal(&bounds.mins, &p(0, 0, 0));
        assert_points_equal(&bounds.maxs, &p(2, 4, 6));
    }

    #[test]
    fn every_curve_family_exposes_exact_positive_order_derivatives() {
        let line = Curve3::line(p(0, 0, 0), p(2, 4, 6)).unwrap();
        assert_eq!(
            line.derivative_at(&q(1, 2), 2).unwrap().vector().0,
            Vector3::zero().0
        );
        assert_eq!(
            line.derivative_at(&q(1, 2), 0).unwrap_err(),
            GeometryError::InvalidDerivativeOrder
        );

        let polynomial = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 0, 0), p(4, 0, 0)],
            vec![r(1), r(1), r(1)],
        )
        .unwrap();
        assert_points_equal(
            &Point3::from(
                polynomial
                    .derivative_at(&q(1, 2), 1)
                    .unwrap()
                    .vector()
                    .clone(),
            ),
            &p(4, 0, 0),
        );
        assert_points_equal(
            &Point3::from(
                polynomial
                    .derivative_at(&q(1, 2), 2)
                    .unwrap()
                    .vector()
                    .clone(),
            ),
            &p(4, 0, 0),
        );
        assert_eq!(
            polynomial.derivative_at(&q(1, 2), 3).unwrap().vector().0,
            Vector3::zero().0
        );
        let rational = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)],
            vec![r(1), r(2), r(1)],
        )
        .unwrap();
        assert_points_equal(
            &Point3::from(rational.derivative_at(&r(0), 2).unwrap().vector().clone()),
            &p(-20, 0, 0),
        );

        let nurbs = Curve3::nurbs(
            2,
            vec![p(0, 0, 0), p(1, 0, 0), p(4, 0, 0)],
            vec![r(1), r(1), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        )
        .unwrap();
        assert_points_equal(
            &Point3::from(nurbs.derivative_at(&q(1, 2), 2).unwrap().vector().clone()),
            &p(4, 0, 0),
        );
        assert_eq!(
            nurbs.derivative_at(&q(1, 2), 3).unwrap().vector().0,
            Vector3::zero().0
        );

        let half_pi = (Real::pi() / r(2)).unwrap();
        let circle = Curve3::circle_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            r(2),
            Real::zero(),
            half_pi,
        )
        .unwrap();
        for (order, expected) in [
            (1, p(0, 2, 0)),
            (2, p(-2, 0, 0)),
            (3, p(0, -2, 0)),
            (4, p(2, 0, 0)),
        ] {
            assert_points_equal(
                &Point3::from(
                    circle
                        .derivative_at(&Real::zero(), order)
                        .unwrap()
                        .vector()
                        .clone(),
                ),
                &expected,
            );
        }
    }

    #[test]
    fn nurbs_uses_its_exact_active_domain() {
        let curve = Curve3::nurbs(
            1,
            vec![p(0, 0, 0), p(2, 0, 0), p(4, 0, 0)],
            vec![r(1), r(1), r(1)],
            vec![r(0), r(0), r(1), r(2), r(2)],
        )
        .unwrap();
        assert_eq!(curve.domain().start(), &r(0));
        assert_eq!(curve.domain().end(), &r(2));
        assert_points_equal(&curve.point_at(&q(3, 2)).unwrap(), &p(3, 0, 0));
        assert_eq!(
            curve.derivative_at(&q(3, 2), 1).unwrap().vector().0,
            [r(2), r(0), r(0)]
        );
    }

    #[test]
    fn reversal_split_and_line_parameter_location_are_exact() {
        let line = Curve3::line(p(1, 2, 3), p(5, 10, 15)).unwrap();
        let parameter = q(1, 4);
        let point = line.point_at(&parameter).unwrap();
        let CurveParameterLocation::Parameters(parameters) = line.parameters_of(&point).unwrap()
        else {
            panic!("line point must have one parameter");
        };
        assert_eq!(parameters.len(), 1);
        assert_eq!(
            compare_reals(&parameters[0], &parameter).value(),
            Some(Ordering::Equal)
        );
        assert!(matches!(
            line.parameters_of(&p(2, 5, 6)).unwrap(),
            CurveParameterLocation::None
        ));
        let reversed = line.reversed().unwrap();
        assert_points_equal(
            &line.point_at(&parameter).unwrap(),
            &reversed.point_at(&q(3, 4)).unwrap(),
        );

        let curve = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(2, 4, 0), p(4, 0, 0)],
            vec![r(1), r(2), r(1)],
        )
        .unwrap();
        let (left, right) = curve.split_at(&q(1, 2)).unwrap();
        assert_points_equal(&left.end().unwrap(), &right.start().unwrap());
        assert_points_equal(
            &left.point_at(&q(1, 2)).unwrap(),
            &curve.point_at(&q(1, 4)).unwrap(),
        );
        assert_points_equal(
            &right.point_at(&q(1, 2)).unwrap(),
            &curve.point_at(&q(3, 4)).unwrap(),
        );
        let middle = curve.subcurve(&q(1, 4), &q(3, 4)).unwrap();
        assert_points_equal(&middle.start().unwrap(), &curve.point_at(&q(1, 4)).unwrap());
        assert_points_equal(&middle.end().unwrap(), &curve.point_at(&q(3, 4)).unwrap());
        assert_points_equal(
            &middle.point_at(&q(1, 2)).unwrap(),
            &curve.point_at(&q(1, 2)).unwrap(),
        );

        let nurbs = Curve3::nurbs(
            2,
            vec![p(0, 0, 0), p(1, 2, 0), p(3, 2, 0), p(4, 0, 0)],
            vec![r(1), r(1), r(1), r(1)],
            vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        )
        .unwrap();
        let (left, right) = nurbs.split_at(&r(1)).unwrap();
        assert_points_equal(&left.end().unwrap(), &right.start().unwrap());
        assert_points_equal(
            &left.point_at(&q(1, 2)).unwrap(),
            &nurbs.point_at(&q(1, 2)).unwrap(),
        );
        assert_points_equal(
            &right.point_at(&q(3, 2)).unwrap(),
            &nurbs.point_at(&q(3, 2)).unwrap(),
        );
        let middle = nurbs.subcurve(&q(1, 2), &q(3, 2)).unwrap();
        assert_eq!(middle.domain().start(), &q(1, 2));
        assert_eq!(middle.domain().end(), &q(3, 2));
        assert_points_equal(
            &middle.point_at(&r(1)).unwrap(),
            &nurbs.point_at(&r(1)).unwrap(),
        );
    }

    #[test]
    fn rational_bezier_and_arc_parameter_location_is_certified() {
        let rational = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 2, 0), p(2, 0, 0)],
            vec![r(1), r(1), r(1)],
        )
        .unwrap();
        let query = rational.point_at(&q(1, 2)).unwrap();
        let CurveParameterLocation::Parameters(parameters) =
            rational.parameters_of(&query).unwrap()
        else {
            panic!("represented Bézier point must retain its parameter");
        };
        assert_eq!(parameters.len(), 1);
        assert_eq!(
            compare_reals(&parameters[0], &q(1, 2)).value(),
            Some(Ordering::Equal)
        );
        assert!(matches!(
            rational.parameters_of(&p(8, 8, 8)).unwrap(),
            CurveParameterLocation::None
        ));

        let constant =
            Curve3::rational_bezier(vec![p(3, 4, 5), p(3, 4, 5)], vec![r(1), r(2)]).unwrap();
        assert!(matches!(
            constant.parameters_of(&p(3, 4, 5)).unwrap(),
            CurveParameterLocation::EntireDomain
        ));
        let algebraic = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(0, 0, 0), p(1, 0, 0)],
            vec![r(1), r(1), r(1)],
        )
        .unwrap();
        assert_eq!(
            algebraic
                .parameters_of(&Point3::new(q(1, 2), r(0), r(0)))
                .unwrap_err(),
            GeometryError::UnrepresentableParameter
        );

        let full_circle = Curve3::circle_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            r(2),
            Real::zero(),
            Real::from(2) * Real::pi(),
        )
        .unwrap();
        let CurveParameterLocation::Parameters(seam) =
            full_circle.parameters_of(&p(2, 0, 0)).unwrap()
        else {
            panic!("full-circle seam must retain both exact parameters");
        };
        assert_eq!(seam.len(), 2);
        let parameter = (Real::pi() / r(3)).unwrap();
        let point = full_circle.point_at(&parameter).unwrap();
        let CurveParameterLocation::Parameters(parameters) =
            full_circle.parameters_of(&point).unwrap()
        else {
            panic!("arc point must retain its exact angular parameter");
        };
        assert_eq!(
            compare_reals(&parameters[0], &parameter).value(),
            Some(Ordering::Equal)
        );

        let nurbs = Curve3::nurbs(
            1,
            vec![p(0, 0, 0), p(2, 0, 0), p(4, 0, 0)],
            vec![r(1), r(1), r(1)],
            vec![r(0), r(0), r(1), r(2), r(2)],
        )
        .unwrap();
        let parameter = q(3, 2);
        let point = nurbs.point_at(&parameter).unwrap();
        let CurveParameterLocation::Parameters(parameters) = nurbs.parameters_of(&point).unwrap()
        else {
            panic!("NURBS point must retain its global parameter");
        };
        assert_eq!(parameters.len(), 1);
        assert_eq!(
            compare_reals(&parameters[0], &parameter).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn parameterized_plane_retains_authored_frame() {
        let surface = Surface::plane(p(1, 2, 3), Vector3::x(), Vector3::z()).unwrap();
        let parameter = Point2::new(r(4), r(5));
        assert_points_equal(&surface.point_at(&parameter).unwrap(), &p(5, 2, 8));
        let partials = surface.partials_at(&parameter).unwrap();
        assert_eq!(partials.u().0, Vector3::x().0);
        assert_eq!(partials.v().0, Vector3::z().0);
    }

    #[test]
    fn tensor_rational_bezier_surface_is_exact_and_bounded_by_its_control_net() {
        let surface = Surface::rational_bezier(
            vec![vec![p(0, 0, 0), p(2, 0, 0)], vec![p(0, 4, 0), p(2, 4, 2)]],
            vec![vec![r(1), r(1)], vec![r(1), r(1)]],
        )
        .unwrap();
        assert_eq!(surface.kind(), SurfaceKind::RationalBezier);
        let middle = Point2::new(q(1, 2), q(1, 2));
        assert_points_equal(
            &surface.point_at(&middle).unwrap(),
            &Point3::new(r(1), r(2), q(1, 2)),
        );
        let partials = surface.partials_at(&middle).unwrap();
        assert_points_equal(&Point3::from(partials.u().clone()), &p(2, 0, 1));
        assert_points_equal(&Point3::from(partials.v().clone()), &p(0, 4, 1));
        let SurfaceBounds::Bounded(bounds) = surface.bounds().unwrap() else {
            panic!("finite spline surface must be bounded");
        };
        assert_points_equal(&bounds.mins, &p(0, 0, 0));
        assert_points_equal(&bounds.maxs, &p(2, 4, 2));
    }

    #[test]
    fn extrusion_and_revolution_surfaces_retain_exact_authored_parameters() {
        let extrusion = Surface::extrusion(
            Curve3::line(p(0, 0, 0), p(2, 0, 0)).unwrap(),
            Vector3::from_xyz(r(0), r(3), r(0)),
        )
        .unwrap();
        assert_eq!(extrusion.kind(), SurfaceKind::Extrusion);
        let parameter = Point2::new(q(1, 2), r(2));
        assert_points_equal(&extrusion.point_at(&parameter).unwrap(), &p(1, 6, 0));
        let partials = extrusion.partials_at(&parameter).unwrap();
        assert_points_equal(&Point3::from(partials.u().clone()), &p(2, 0, 0));
        assert_points_equal(&Point3::from(partials.v().clone()), &p(0, 3, 0));
        assert!(matches!(
            extrusion.bounds().unwrap(),
            SurfaceBounds::Unbounded
        ));

        let revolution = Surface::revolution(
            Curve3::line(p(2, 0, -1), p(2, 0, 1)).unwrap(),
            Point3::origin(),
            Vector3::z(),
        )
        .unwrap();
        assert_eq!(revolution.kind(), SurfaceKind::Revolution);
        let half_pi = (Real::pi() / r(2)).unwrap();
        let parameter = Point2::new(half_pi, q(1, 2));
        assert_points_equal(&revolution.point_at(&parameter).unwrap(), &p(0, 2, 0));
        let partials = revolution.partials_at(&parameter).unwrap();
        assert_points_equal(&Point3::from(partials.u().clone()), &p(-2, 0, 0));
        assert_points_equal(&Point3::from(partials.v().clone()), &p(0, 0, 2));
        let SurfaceBounds::Bounded(bounds) = revolution.bounds().unwrap() else {
            panic!("finite revolution must have conservative bounds");
        };
        assert_points_equal(&bounds.mins, &p(-2, -2, -1));
        assert_points_equal(&bounds.maxs, &p(2, 2, 1));

        let translated_axis = Surface::revolution(
            Curve3::line(p(0, 4, 3), p(3, 4, 3)).unwrap(),
            p(1, 2, 3),
            Vector3::x(),
        )
        .unwrap();
        let SurfaceBounds::Bounded(bounds) = translated_axis.bounds().unwrap() else {
            panic!("an arbitrarily oriented finite revolution must remain bounded");
        };
        assert_points_equal(&bounds.mins, &p(0, 0, 1));
        assert_points_equal(&bounds.maxs, &p(3, 4, 5));
    }

    #[test]
    fn transverse_planes_retain_exact_revolution_profile_sections() {
        let profile = Curve3::line(p(2, 0, -1), p(4, 0, 1)).unwrap();
        let revolution = Surface::revolution(profile, Point3::origin(), Vector3::z()).unwrap();
        let plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        let SurfaceSurfaceIntersection::Curve(section) =
            plane.intersect_surface(&revolution).unwrap()
        else {
            panic!("one transverse profile point must retain one revolution circle");
        };
        assert_points_equal(&section.curve().start().unwrap(), &p(3, 0, 0));
        assert_eq!(
            section
                .first_pcurve()
                .materialize()
                .unwrap()
                .curve()
                .family(),
            CurveFamily2::CircularArc
        );
        assert_eq!(
            section
                .second_pcurve()
                .materialize()
                .unwrap()
                .curve()
                .family(),
            CurveFamily2::Line
        );
        for parameter in [Real::zero(), (Real::pi() / r(2)).unwrap()] {
            let spatial = section.curve().point_at(&parameter).unwrap();
            let plane_parameter = section.first_pcurve().point_at(&parameter).unwrap();
            let revolution_parameter = section.second_pcurve().point_at(&parameter).unwrap();
            assert_eq!(
                compare_reals(&revolution_parameter.y, &q(1, 2)).value(),
                Some(Ordering::Equal)
            );
            assert_points_equal(&plane.point_at(&plane_parameter).unwrap(), &spatial);
            assert_points_equal(
                &revolution.point_at(&revolution_parameter).unwrap(),
                &spatial,
            );
        }
        let SurfaceSurfaceIntersection::Curve(swapped) =
            revolution.intersect_surface(&plane).unwrap()
        else {
            panic!("operand reversal must retain the revolution section");
        };
        let parameter = (Real::pi() / r(2)).unwrap();
        let spatial = swapped.curve().point_at(&parameter).unwrap();
        assert_points_equal(
            &revolution
                .point_at(&swapped.first_pcurve().point_at(&parameter).unwrap())
                .unwrap(),
            &spatial,
        );
        assert_points_equal(
            &plane
                .point_at(&swapped.second_pcurve().point_at(&parameter).unwrap())
                .unwrap(),
            &spatial,
        );

        let semicircle = Curve3::circle_arc(
            p(3, 0, 0),
            Vector3::x(),
            Vector3::z(),
            Real::one(),
            Real::zero(),
            Real::pi(),
        )
        .unwrap();
        let double = Surface::revolution(semicircle, Point3::origin(), Vector3::z()).unwrap();
        let SurfaceSurfaceIntersection::Curves(sections) =
            double.intersect_surface(&plane).unwrap()
        else {
            panic!("two profile contacts must retain two revolution circles");
        };
        assert_eq!(sections.len(), 2);
        assert_points_equal(&sections[0].curve().start().unwrap(), &p(4, 0, 0));
        assert_points_equal(&sections[1].curve().start().unwrap(), &p(2, 0, 0));

        let disjoint = Surface::plane(p(0, 0, 2), Vector3::x(), Vector3::y()).unwrap();
        assert!(matches!(
            revolution.intersect_surface(&disjoint).unwrap(),
            SurfaceSurfaceIntersection::None
        ));
        let singular = Surface::revolution(
            Curve3::line(p(0, 0, -1), p(0, 0, 1)).unwrap(),
            Point3::origin(),
            Vector3::z(),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Point(point) = singular.intersect_surface(&plane).unwrap()
        else {
            panic!("a transverse revolution-axis contact must retain its singular point");
        };
        assert_points_equal(&point, &Point3::origin());

        let coincident_profile = Surface::revolution(
            Curve3::line(p(2, 0, 0), p(4, 0, 0)).unwrap(),
            Point3::origin(),
            Vector3::z(),
        )
        .unwrap();
        assert_eq!(
            coincident_profile.intersect_surface(&plane).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
        let axial = Surface::plane(Point3::origin(), Vector3::x(), Vector3::z()).unwrap();
        assert_eq!(
            revolution.intersect_surface(&axial).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
    }

    #[test]
    fn tensor_nurbs_surface_uses_both_exact_active_domains_and_partials() {
        let surface = Surface::nurbs(
            1,
            1,
            vec![vec![p(0, 0, 0), p(2, 0, 0)], vec![p(0, 4, 0), p(2, 4, 2)]],
            vec![vec![r(1), r(1)], vec![r(1), r(1)]],
            vec![r(0), r(0), r(2), r(2)],
            vec![r(1), r(1), r(3), r(3)],
        )
        .unwrap();
        assert_eq!(surface.kind(), SurfaceKind::Nurbs);
        let SurfaceParameterDomain::Closed(u_domain) = surface.domain().u() else {
            panic!("NURBS u domain must be closed");
        };
        let SurfaceParameterDomain::Closed(v_domain) = surface.domain().v() else {
            panic!("NURBS v domain must be closed");
        };
        assert_eq!(
            compare_reals(u_domain.end(), &r(2)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(v_domain.start(), &r(1)).value(),
            Some(Ordering::Equal)
        );
        let middle = Point2::new(r(1), r(2));
        assert_points_equal(
            &surface.point_at(&middle).unwrap(),
            &Point3::new(r(1), r(2), q(1, 2)),
        );
        let partials = surface.partials_at(&middle).unwrap();
        assert_points_equal(
            &Point3::from(partials.u().clone()),
            &Point3::new(r(1), r(0), q(1, 2)),
        );
        assert_points_equal(
            &Point3::from(partials.v().clone()),
            &Point3::new(r(0), r(2), q(1, 2)),
        );
    }

    #[test]
    fn tensor_surface_subdivision_preserves_exact_images_on_both_axes() {
        let bezier = Surface::rational_bezier(
            vec![vec![p(0, 0, 0), p(2, 0, 0)], vec![p(0, 4, 0), p(2, 4, 2)]],
            vec![vec![r(1), r(1)], vec![r(1), r(1)]],
        )
        .unwrap();
        let (u_left, u_right) = bezier.split_u_at(&q(1, 2)).unwrap();
        assert_points_equal(
            &u_left.point_at(&Point2::new(q(1, 2), q(3, 4))).unwrap(),
            &bezier.point_at(&Point2::new(q(1, 4), q(3, 4))).unwrap(),
        );
        assert_points_equal(
            &u_right.point_at(&Point2::new(q(1, 2), q(3, 4))).unwrap(),
            &bezier.point_at(&Point2::new(q(3, 4), q(3, 4))).unwrap(),
        );
        let (v_left, v_right) = bezier.split_v_at(&q(1, 2)).unwrap();
        assert_points_equal(
            &v_left.point_at(&Point2::new(q(3, 4), q(1, 2))).unwrap(),
            &bezier.point_at(&Point2::new(q(3, 4), q(1, 4))).unwrap(),
        );
        assert_points_equal(
            &v_right.point_at(&Point2::new(q(3, 4), q(1, 2))).unwrap(),
            &bezier.point_at(&Point2::new(q(3, 4), q(3, 4))).unwrap(),
        );
        let iso_u = bezier.iso_curve(SurfaceIsoAxis::U, &q(3, 4)).unwrap();
        assert_points_equal(
            &iso_u.point_at(&q(1, 3)).unwrap(),
            &bezier.point_at(&Point2::new(q(1, 3), q(3, 4))).unwrap(),
        );
        let iso_v = bezier.iso_curve(SurfaceIsoAxis::V, &q(1, 3)).unwrap();
        assert_points_equal(
            &iso_v.point_at(&q(3, 4)).unwrap(),
            &bezier.point_at(&Point2::new(q(1, 3), q(3, 4))).unwrap(),
        );

        let nurbs = Surface::nurbs(
            2,
            2,
            vec![
                vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)],
                vec![p(0, 1, 0), p(1, 1, 1), p(2, 1, 0)],
                vec![p(0, 2, 0), p(1, 2, 0), p(2, 2, 0)],
            ],
            vec![
                vec![r(1), r(1), r(1)],
                vec![r(1), r(1), r(1)],
                vec![r(1), r(1), r(1)],
            ],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        )
        .unwrap();
        let (u_left, u_right) = nurbs.split_u_at(&q(1, 2)).unwrap();
        for (surface, parameter) in [(&u_left, q(1, 4)), (&u_right, q(3, 4))] {
            assert_points_equal(
                &surface
                    .point_at(&Point2::new(parameter.clone(), q(2, 3)))
                    .unwrap(),
                &nurbs.point_at(&Point2::new(parameter, q(2, 3))).unwrap(),
            );
        }
        let (v_left, v_right) = nurbs.split_v_at(&q(1, 2)).unwrap();
        for (surface, parameter) in [(&v_left, q(1, 4)), (&v_right, q(3, 4))] {
            assert_points_equal(
                &surface
                    .point_at(&Point2::new(q(1, 3), parameter.clone()))
                    .unwrap(),
                &nurbs.point_at(&Point2::new(q(1, 3), parameter)).unwrap(),
            );
        }
        let iso_u = nurbs.iso_curve(SurfaceIsoAxis::U, &q(2, 3)).unwrap();
        assert_points_equal(
            &iso_u.point_at(&q(1, 3)).unwrap(),
            &nurbs.point_at(&Point2::new(q(1, 3), q(2, 3))).unwrap(),
        );
        let iso_v = nurbs.iso_curve(SurfaceIsoAxis::V, &q(1, 3)).unwrap();
        assert_points_equal(
            &iso_v.point_at(&q(2, 3)).unwrap(),
            &nurbs.point_at(&Point2::new(q(1, 3), q(2, 3))).unwrap(),
        );
    }

    #[test]
    fn tensor_surface_validation_and_affine_retention_are_explicit() {
        assert_eq!(
            Surface::rational_bezier(
                vec![vec![p(0, 0, 0), p(1, 0, 0)], vec![p(0, 1, 0)]],
                vec![vec![r(1), r(1)], vec![r(1)]],
            )
            .unwrap_err(),
            GeometryError::InvalidControlNetShape
        );
        assert_eq!(
            Surface::rational_bezier(
                vec![vec![p(0, 0, 0), p(1, 0, 0)], vec![p(0, 1, 0), p(1, 1, 0)]],
                vec![vec![r(1), r(1)], vec![r(1)]],
            )
            .unwrap_err(),
            GeometryError::SurfaceWeightShapeMismatch
        );
        assert_eq!(
            Surface::extrusion(
                Curve3::line(p(0, 0, 0), p(1, 0, 0)).unwrap(),
                Vector3::zero(),
            )
            .unwrap_err(),
            GeometryError::DegenerateExtrusionDirection
        );
        assert_eq!(
            Surface::revolution(
                Curve3::line(p(1, 0, 0), p(1, 0, 1)).unwrap(),
                Point3::origin(),
                Vector3::from_xyz(r(0), r(0), r(2)),
            )
            .unwrap_err(),
            GeometryError::InvalidRevolutionAxis
        );

        let surface = Surface::rational_bezier(
            vec![vec![p(0, 0, 0), p(2, 0, 0)], vec![p(0, 4, 0), p(2, 4, 2)]],
            vec![vec![r(1), r(1)], vec![r(1), r(1)]],
        )
        .unwrap();
        let translated = surface
            .transformed(&Matrix4::affine_translation([r(3), r(-2), r(5)]), false)
            .unwrap();
        let parameter = Point2::new(q(1, 4), q(3, 4));
        let expected = surface.point_at(&parameter).unwrap() + Vector3::from_xyz(r(3), r(-2), r(5));
        assert_points_equal(&translated.point_at(&parameter).unwrap(), &expected);

        let reflected = surface
            .transformed(
                &Matrix4::from_row_major([
                    r(-1),
                    r(0),
                    r(0),
                    r(0),
                    r(0),
                    r(1),
                    r(0),
                    r(0),
                    r(0),
                    r(0),
                    r(1),
                    r(0),
                    r(0),
                    r(0),
                    r(0),
                    r(1),
                ]),
                true,
            )
            .unwrap();
        let mirrored_original = surface.point_at(&Point2::new(q(3, 4), q(3, 4))).unwrap();
        let expected = Point3::new(
            -mirrored_original.x,
            mirrored_original.y,
            mirrored_original.z,
        );
        assert_points_equal(&reflected.point_at(&parameter).unwrap(), &expected);
    }

    #[test]
    fn linear_tensor_surfaces_intersect_parallel_planes_in_exact_iso_curves() {
        let rational = Surface::rational_bezier(
            vec![
                vec![p(0, 0, 0), p(1, 2, 0), p(2, 0, 0)],
                vec![p(0, 0, 2), p(1, 2, 2), p(2, 0, 2)],
            ],
            vec![vec![r(1), r(2), r(1)], vec![r(1), r(2), r(1)]],
        )
        .unwrap();
        let middle_plane = Surface::plane(p(0, 0, 1), Vector3::x(), Vector3::y()).unwrap();
        let SurfaceSurfaceIntersection::Curve(curve) =
            middle_plane.intersect_surface(&rational).unwrap()
        else {
            panic!("linear rational tensor patch must retain its exact middle iso-curve");
        };
        assert_eq!(curve.curve().kind(), Curve3Kind::RationalBezier);
        assert_points_equal(
            &curve.curve().point_at(&q(1, 2)).unwrap(),
            &rational.point_at(&Point2::new(q(1, 2), q(1, 2))).unwrap(),
        );
        assert_eq!(
            curve.second_pcurve().point_at(&q(1, 2)).unwrap(),
            Point2::new(q(1, 2), q(1, 2))
        );
        assert!(matches!(
            rational
                .intersect_surface(&Surface::plane(p(0, 0, 3), Vector3::x(), Vector3::y()).unwrap())
                .unwrap(),
            SurfaceSurfaceIntersection::None
        ));

        let nurbs = Surface::nurbs(
            2,
            1,
            vec![
                vec![p(0, 0, 0), p(1, 2, 0), p(2, 0, 0)],
                vec![p(0, 0, 3), p(1, 2, 3), p(2, 0, 3)],
            ],
            vec![vec![r(1), r(2), r(1)], vec![r(1), r(2), r(1)]],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            vec![r(0), r(0), r(1), r(1)],
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curve(curve) = nurbs
            .intersect_surface(&middle_plane)
            .expect("linear NURBS tensor iso-curve")
        else {
            panic!("linear NURBS tensor patch must retain one exact iso-curve");
        };
        assert_eq!(curve.curve().kind(), Curve3Kind::Nurbs);
        assert_points_equal(
            &curve.curve().point_at(&q(1, 2)).unwrap(),
            &nurbs.point_at(&Point2::new(q(1, 2), q(1, 3))).unwrap(),
        );
        assert_eq!(
            curve.first_pcurve().point_at(&q(1, 2)).unwrap(),
            Point2::new(q(1, 2), q(1, 3))
        );

        let bilinear = Surface::rational_bezier(
            vec![vec![p(0, 0, 0), p(2, 0, 0)], vec![p(0, 0, 2), p(2, 0, 2)]],
            vec![vec![r(1), r(1)], vec![r(1), r(1)]],
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curve(curve) =
            bilinear.intersect_surface(&middle_plane).unwrap()
        else {
            panic!("bilinear tensor must fall through to its exact v-iso section");
        };
        assert_eq!(
            curve.first_pcurve().point_at(&q(1, 2)).unwrap(),
            Point2::new(q(1, 2), q(1, 2))
        );

        let transverse = Surface::plane(p(1, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        let SurfaceSurfaceIntersection::Curve(section) =
            rational.intersect_surface(&transverse).unwrap()
        else {
            panic!("a represented rational-profile plane root must lift to a tensor iso-curve");
        };
        let parameter = q(1, 2);
        let tensor_parameter = section.first_pcurve().point_at(&parameter).unwrap();
        let plane_parameter = section.second_pcurve().point_at(&parameter).unwrap();
        let point = section.curve().point_at(&parameter).unwrap();
        assert_points_equal(&point, &rational.point_at(&tensor_parameter).unwrap());
        assert_points_equal(&point, &transverse.point_at(&plane_parameter).unwrap());
        assert_eq!(tensor_parameter, Point2::new(q(1, 2), q(1, 2)));
    }

    #[test]
    fn u_linear_tensor_surfaces_retain_exact_non_isoparametric_plane_sections() {
        let controls = vec![
            vec![p(0, 0, 0), p(2, 0, 0)],
            vec![p(0, 2, 1), p(2, 2, 1)],
            vec![p(0, 2, 2), p(2, 2, 2)],
        ];
        let weights = vec![vec![r(1), r(1)], vec![r(2), r(2)], vec![r(3), r(3)]];
        let plane = Surface::plane(
            p(2, 0, 0),
            Vector3::y(),
            Vector3::from_xyz(r(1), r(0), r(-1)),
        )
        .unwrap();
        let rational = Surface::rational_bezier(controls.clone(), weights.clone()).unwrap();
        let SurfaceSurfaceIntersection::Curve(section) =
            plane.intersect_surface(&rational).unwrap()
        else {
            panic!("u-linear rational tensor must retain its exact oblique section");
        };
        assert_eq!(section.curve().kind(), Curve3Kind::RationalBezier);
        let parameter = q(1, 2);
        let tensor_parameter = Point2::new(q(3, 8), parameter.clone());
        assert_eq!(
            section.second_pcurve().point_at(&parameter).unwrap(),
            tensor_parameter
        );
        assert_points_equal(
            &section.curve().point_at(&parameter).unwrap(),
            &rational.point_at(&tensor_parameter).unwrap(),
        );
        let materialized = section.second_pcurve().materialize().unwrap();
        assert_eq!(materialized.curve().family(), CurveFamily2::RationalBezier);
        assert_eq!(
            materialized.curve().point_at(&parameter).unwrap(),
            CurvePoint2::new(q(3, 8), q(1, 2))
        );

        let nurbs = Surface::nurbs(
            1,
            2,
            controls,
            weights,
            vec![r(0), r(0), r(1), r(1)],
            vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curve(section) = nurbs.intersect_surface(&plane).unwrap()
        else {
            panic!("u-linear NURBS tensor must retain its exact oblique section");
        };
        assert_eq!(section.curve().kind(), Curve3Kind::Nurbs);
        assert_eq!(
            section.first_pcurve().point_at(&parameter).unwrap(),
            Point2::new(q(3, 8), q(1, 2))
        );
        assert_points_equal(
            &section.curve().point_at(&parameter).unwrap(),
            &nurbs.point_at(&Point2::new(q(3, 8), q(1, 2))).unwrap(),
        );

        let disjoint = Surface::plane(
            p(5, 0, 0),
            Vector3::y(),
            Vector3::from_xyz(r(1), r(0), r(-1)),
        )
        .unwrap();
        assert!(matches!(
            rational.intersect_surface(&disjoint).unwrap(),
            SurfaceSurfaceIntersection::None
        ));
        let partially_trimmed = Surface::plane(
            p(1, 0, 0),
            Vector3::y(),
            Vector3::from_xyz(r(1), r(0), r(-1)),
        )
        .unwrap();
        assert_eq!(
            rational.intersect_surface(&partially_trimmed).unwrap_err(),
            GeometryError::UnrepresentableParameter,
            "certified algebraic trim roots must not be approximated into Real"
        );
    }

    #[test]
    fn partial_nurbs_tensor_sections_retain_multiple_disjoint_exact_fragments() {
        let controls = vec![
            vec![p(0, 0, 3), p(2, 0, 3)],
            vec![p(0, 1, 1), p(2, 1, 1)],
            vec![p(0, 2, 3), p(2, 2, 3)],
            vec![p(0, 3, 1), p(2, 3, 1)],
            vec![p(0, 4, 3), p(2, 4, 3)],
        ];
        let weights = vec![vec![r(1), r(1)]; controls.len()];
        let (patch, face) = crate::builder::nurbs_patch(
            1,
            1,
            controls,
            weights,
            vec![r(7), r(7), r(11), r(11)],
            vec![r(2), r(2), r(3), r(4), r(5), r(6), r(6)],
        )
        .unwrap();
        let tensor = patch.surface(patch.face(face).unwrap().surface()).unwrap();
        let plane = Surface::plane(
            p(2, 0, 0),
            Vector3::y(),
            Vector3::from_xyz(r(1), r(0), r(-1)),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curves(fragments) =
            tensor.intersect_surface(&plane).unwrap()
        else {
            panic!("piecewise-linear graph must retain two disjoint bounded fragments");
        };
        assert_eq!(fragments.len(), 2);
        for fragment in &fragments {
            for parameter in [
                fragment.curve().domain().start(),
                fragment.curve().domain().end(),
            ] {
                let tensor_parameter = fragment.first_pcurve().point_at(parameter).unwrap();
                assert_eq!(tensor_parameter.x, r(7));
                assert_points_equal(
                    &fragment.curve().point_at(parameter).unwrap(),
                    &tensor.point_at(&tensor_parameter).unwrap(),
                );
            }
        }
        let (partitioned, partition) = patch
            .split_face_by_surface_curves(face, &fragments, SurfaceIntersectionOperand::First)
            .unwrap();
        assert_eq!(partition.faces.len(), 3);
        assert_eq!(partition.traces.len(), 2);
        assert_eq!(partitioned.counts().faces, 3);
        for trace in &partition.traces {
            assert_eq!(trace.splits.len(), 1);
            assert_eq!(
                partitioned
                    .uses_of_edge(trace.splits[0].open().unwrap().face.edge)
                    .unwrap()
                    .len(),
                2
            );
        }
        let mut reversed_fragments = fragments.clone();
        reversed_fragments.reverse();
        let (reverse_partitioned, _) = patch
            .split_face_by_surface_curves(
                face,
                &reversed_fragments,
                SurfaceIntersectionOperand::First,
            )
            .unwrap();
        assert_eq!(
            reverse_partitioned.to_json().unwrap(),
            partitioned.to_json().unwrap()
        );
        let partitioned_json = partitioned.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&partitioned_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            partitioned_json
        );

        let (split, record) = patch
            .split_face_by_surface_curve(face, fragments[0].curve(), fragments[0].first_pcurve())
            .unwrap();
        assert_eq!(split.counts().faces, 2);
        let graph_use = split
            .edge_use(record.open().unwrap().face.edge_uses[0])
            .unwrap();
        let graph_pcurve = split.pcurve(graph_use.pcurve()).unwrap();
        let CurveGeometry2::Nurbs(graph) = graph_pcurve.curve().geometry() else {
            panic!("cross-span partial graph must retain one NURBS pcurve");
        };
        let mut forged_controls = graph.control_points().to_vec();
        forged_controls[1] = CurvePoint2::new(
            forged_controls[1].x().clone() + Real::one(),
            forged_controls[1].y().clone(),
        );
        let forged = Pcurve::new(
            Curve2::try_nurbs(
                graph.degree(),
                forged_controls,
                graph.weights().to_vec(),
                graph.knots().to_vec(),
            )
            .unwrap(),
        );
        let mut edit = split.edit();
        edit.replace_pcurve(graph_use.pcurve(), forged).unwrap();
        assert!(
            edit.commit().is_err(),
            "partial graph validation must reject an endpoint-preserving control forgery"
        );
        let json = split.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn crossing_tensor_surface_curves_are_atomized_by_exact_pcurve_contacts() {
        let controls = vec![vec![p(0, 0, 0), p(2, 0, 0)], vec![p(0, 2, 0), p(2, 2, 0)]];
        let weights = vec![vec![Real::one(), Real::one()]; 2];
        let (patch, face) = crate::builder::rational_bezier_patch(controls, weights).unwrap();
        let surface = patch.surface(patch.face(face).unwrap().surface()).unwrap();
        let x_plane = Surface::plane(p(1, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        let y_plane = Surface::plane(p(0, 1, 0), Vector3::x(), Vector3::z()).unwrap();
        let SurfaceSurfaceIntersection::Curve(x_trace) =
            surface.intersect_surface(&x_plane).unwrap()
        else {
            panic!("x selection must retain one exact tensor iso-curve");
        };
        let SurfaceSurfaceIntersection::Curve(y_trace) =
            surface.intersect_surface(&y_plane).unwrap()
        else {
            panic!("y selection must retain one exact tensor iso-curve");
        };

        let traces = [*x_trace, *y_trace];
        let (partitioned, partition) = patch
            .split_face_by_surface_curves(face, &traces, SurfaceIntersectionOperand::First)
            .unwrap();
        assert_eq!(partition.faces.len(), 4);
        assert_eq!(partition.traces.len(), 2);
        assert_eq!(
            partition
                .traces
                .iter()
                .map(|trace| trace.segments.len())
                .sum::<usize>(),
            3
        );
        assert_eq!(
            partition
                .traces
                .iter()
                .map(|trace| trace.splits.len())
                .sum::<usize>(),
            3
        );
        let shared_crossing_vertices = partition
            .traces
            .iter()
            .flat_map(|trace| &trace.splits)
            .flat_map(|split| {
                let edge = partitioned.edge(split.open().unwrap().face.edge).unwrap();
                [edge.start(), edge.end()]
            })
            .filter(|vertex| {
                let point = partitioned.vertex(*vertex).unwrap().point();
                point3_equal(point, &p(1, 1, 0)).value() == Some(true)
            })
            .collect::<std::collections::HashSet<_>>();
        assert_eq!(shared_crossing_vertices.len(), 1);

        let (reversed, _) = patch
            .split_face_by_surface_curves(
                face,
                &[traces[1].clone(), traces[0].clone()],
                SurfaceIntersectionOperand::First,
            )
            .unwrap();
        let json = partitioned.to_json().unwrap();
        assert_eq!(reversed.to_json().unwrap(), json);
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn closed_planar_surface_curve_authors_identity_shared_outer_and_inner_wires() {
        let (model, solid) = crate::builder::cuboid(p(0, 0, 0), p(10, 10, 10)).unwrap();
        let (face, surface) = model
            .faces()
            .find_map(|(face, record)| {
                let surface = model.surface(record.surface()).unwrap();
                surface.plane_origin().and_then(|origin| {
                    (point3_equal(origin, &p(0, 0, 10)).value() == Some(true))
                        .then_some((face, surface))
                })
            })
            .expect("cuboid has a top planar face");
        let SurfaceGeometry::Plane(plane) = &surface.data.geometry else {
            unreachable!("selected surface is planar");
        };
        let loop_curve = Curve3::rational_bezier(
            vec![
                p(5, 3, 10),
                p(8, 3, 10),
                p(8, 7, 10),
                p(5, 7, 10),
                p(2, 7, 10),
                p(2, 3, 10),
                p(5, 3, 10),
            ],
            vec![Real::one(); 7],
        )
        .unwrap();
        let trace = SurfaceIntersectionCurve::new(
            loop_curve.clone(),
            SurfaceIntersectionPcurve::plane_projection(loop_curve.clone(), plane),
            SurfaceIntersectionPcurve::plane_projection(loop_curve.clone(), plane),
        );
        let materialized = trace.first_pcurve().materialize().unwrap();
        let loop_path =
            hypercurve::CurvePath2::try_new(vec![materialized.curve().clone()]).unwrap();
        assert!(
            loop_path
                .bezier_boundary_loop()
                .unwrap()
                .boundary_loop()
                .signed_area()
                .unwrap()
                .is_some()
        );
        let original_area = model.face_area(face).unwrap();
        let original_volume = model.solid_volume(solid).unwrap();
        let (partitioned, split) = model
            .split_face_by_surface_curve(face, trace.curve(), trace.first_pcurve())
            .unwrap();
        let closed = split.closed().expect("closed trace returns a loop split");
        assert_eq!(closed.first_face, face);
        assert_eq!(partitioned.face(face).unwrap().inner().len(), 1);
        assert_eq!(
            partitioned.face(closed.second_face).unwrap().outer(),
            Some(closed.second_wire)
        );
        for edge in closed.edges {
            assert_eq!(partitioned.uses_of_edge(edge).unwrap().len(), 2);
        }
        assert_eq!(
            compare_reals(
                &(partitioned.face_area(closed.first_face).unwrap()
                    + partitioned.face_area(closed.second_face).unwrap()),
                &original_area,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&partitioned.solid_volume(solid).unwrap(), &original_volume,).value(),
            Some(Ordering::Equal)
        );
        let json = partitioned.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );

        let reversed_circle = loop_curve.reversed().unwrap();
        let reversed_trace = SurfaceIntersectionCurve::new(
            reversed_circle.clone(),
            SurfaceIntersectionPcurve::plane_projection(reversed_circle.clone(), plane),
            SurfaceIntersectionPcurve::plane_projection(reversed_circle.clone(), plane),
        );
        let (reversed, _) = model
            .split_face_by_surface_curve(
                face,
                reversed_trace.curve(),
                reversed_trace.first_pcurve(),
            )
            .unwrap();
        assert_eq!(reversed.to_json().unwrap(), json);
    }

    #[test]
    fn nested_closed_surface_curves_partition_descendants_independent_of_order_and_direction() {
        let (model, solid) = crate::builder::cuboid(p(0, 0, 0), p(10, 10, 10)).unwrap();
        let (face, surface) = model
            .faces()
            .find_map(|(face, record)| {
                let surface = model.surface(record.surface()).unwrap();
                surface.plane_origin().and_then(|origin| {
                    (point3_equal(origin, &p(0, 0, 10)).value() == Some(true))
                        .then_some((face, surface))
                })
            })
            .expect("cuboid has a top planar face");
        let SurfaceGeometry::Plane(plane) = &surface.data.geometry else {
            unreachable!("selected surface is planar");
        };
        let trace = |controls: Vec<Point3>| {
            let curve = Curve3::rational_bezier(controls, vec![Real::one(); 7]).unwrap();
            SurfaceIntersectionCurve::new(
                curve.clone(),
                SurfaceIntersectionPcurve::plane_projection(curve.clone(), plane),
                SurfaceIntersectionPcurve::plane_projection(curve, plane),
            )
        };
        let outer = trace(vec![
            p(5, 2, 10),
            p(8, 2, 10),
            p(8, 8, 10),
            p(5, 8, 10),
            p(2, 8, 10),
            p(2, 2, 10),
            p(5, 2, 10),
        ]);
        let inner = trace(vec![
            p(5, 4, 10),
            p(6, 4, 10),
            p(6, 6, 10),
            p(5, 6, 10),
            p(4, 6, 10),
            p(4, 4, 10),
            p(5, 4, 10),
        ]);
        let original_area = model.face_area(face).unwrap();
        let original_volume = model.solid_volume(solid).unwrap();
        let (partitioned, partition) = model
            .split_face_by_surface_curves(
                face,
                &[inner.clone(), outer.clone()],
                SurfaceIntersectionOperand::First,
            )
            .unwrap();
        assert_eq!(partition.faces.len(), 3);
        assert_eq!(partition.traces.len(), 2);
        assert!(
            partition
                .traces
                .iter()
                .all(|trace| trace.splits.len() == 1 && trace.splits[0].closed().is_some())
        );
        for split in partition.traces.iter().flat_map(|trace| &trace.splits) {
            for edge in split.closed().unwrap().edges {
                assert_eq!(partitioned.uses_of_edge(edge).unwrap().len(), 2);
            }
        }
        let partitioned_area = partition
            .faces
            .iter()
            .map(|face| partitioned.face_area(*face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(&partitioned_area, &original_area).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&partitioned.solid_volume(solid).unwrap(), &original_volume,).value(),
            Some(Ordering::Equal)
        );

        let reverse_trace = |source: &SurfaceIntersectionCurve| {
            let curve = source.curve().reversed().unwrap();
            SurfaceIntersectionCurve::new(
                curve.clone(),
                SurfaceIntersectionPcurve::plane_projection(curve.clone(), plane),
                SurfaceIntersectionPcurve::plane_projection(curve, plane),
            )
        };
        let (reversed, _) = model
            .split_face_by_surface_curves(
                face,
                &[reverse_trace(&outer), reverse_trace(&inner)],
                SurfaceIntersectionOperand::First,
            )
            .unwrap();
        let json = partitioned.to_json().unwrap();
        assert_eq!(reversed.to_json().unwrap(), json);
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn boundary_trace_atomizes_a_closed_curve_into_unambiguous_descendant_segments() {
        let (model, solid) = crate::builder::cuboid(p(0, 0, 0), p(10, 10, 10)).unwrap();
        let (face, surface) = model
            .faces()
            .find_map(|(face, record)| {
                let surface = model.surface(record.surface()).unwrap();
                surface.plane_origin().and_then(|origin| {
                    (point3_equal(origin, &p(0, 0, 10)).value() == Some(true))
                        .then_some((face, surface))
                })
            })
            .expect("cuboid has a top planar face");
        let SurfaceGeometry::Plane(plane) = &surface.data.geometry else {
            unreachable!("selected surface is planar");
        };
        let retain = |curve: Curve3| {
            SurfaceIntersectionCurve::new(
                curve.clone(),
                SurfaceIntersectionPcurve::plane_projection(curve.clone(), plane),
                SurfaceIntersectionPcurve::plane_projection(curve, plane),
            )
        };
        let line = retain(Curve3::line(p(5, 0, 10), p(5, 10, 10)).unwrap());
        let closed = retain(
            Curve3::rational_bezier(
                vec![
                    p(5, 3, 10),
                    p(8, 3, 10),
                    p(8, 7, 10),
                    p(5, 7, 10),
                    p(2, 7, 10),
                    p(2, 3, 10),
                    p(5, 3, 10),
                ],
                vec![Real::one(); 7],
            )
            .unwrap(),
        );
        let original_area = model.face_area(face).unwrap();
        let original_volume = model.solid_volume(solid).unwrap();
        let (partitioned, partition) = model
            .split_face_by_surface_curves(
                face,
                &[closed.clone(), line.clone()],
                SurfaceIntersectionOperand::First,
            )
            .unwrap();
        assert_eq!(partition.faces.len(), 4);
        assert_eq!(partition.traces.len(), 2);
        assert_eq!(
            partition
                .traces
                .iter()
                .map(|trace| trace.segments.len())
                .sum::<usize>(),
            3
        );
        assert!(
            partition
                .traces
                .iter()
                .flat_map(|trace| &trace.splits)
                .all(|split| split.open().is_some())
        );
        let partitioned_area = partition
            .faces
            .iter()
            .map(|face| partitioned.face_area(*face).unwrap())
            .fold(Real::zero(), |sum, area| sum + area);
        assert_eq!(
            compare_reals(&partitioned_area, &original_area).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&partitioned.solid_volume(solid).unwrap(), &original_volume,).value(),
            Some(Ordering::Equal)
        );

        let (reversed, _) = model
            .split_face_by_surface_curves(
                face,
                &[line.reversed().unwrap(), closed.reversed().unwrap()],
                SurfaceIntersectionOperand::First,
            )
            .unwrap();
        let json = partitioned.to_json().unwrap();
        assert_eq!(reversed.to_json().unwrap(), json);
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn v_linear_tensor_surfaces_retain_noniso_sections_and_native_domains() {
        let controls = vec![
            vec![p(0, 0, 0), p(1, 2, 0), p(2, 2, 0)],
            vec![p(0, 0, 2), p(1, 2, 2), p(2, 2, 2)],
        ];
        let weights = vec![vec![r(1), r(2), r(3)], vec![r(1), r(2), r(3)]];
        let plane = Surface::plane(
            p(2, 0, 0),
            Vector3::y(),
            Vector3::from_xyz(r(1), r(0), r(-1)),
        )
        .unwrap();
        let rational = Surface::rational_bezier(controls.clone(), weights.clone()).unwrap();
        let SurfaceSurfaceIntersection::Curve(section) =
            rational.intersect_surface(&plane).unwrap()
        else {
            panic!("v-linear rational tensor must retain its exact graph section");
        };
        assert_eq!(section.curve().kind(), Curve3Kind::RationalBezier);
        assert_eq!(
            section.first_pcurve().point_at(&q(1, 2)).unwrap(),
            Point2::new(q(1, 2), q(3, 8))
        );
        assert_points_equal(
            &section.curve().point_at(&q(1, 2)).unwrap(),
            &rational.point_at(&Point2::new(q(1, 2), q(3, 8))).unwrap(),
        );

        let nurbs = Surface::nurbs(
            2,
            1,
            controls.clone(),
            weights.clone(),
            vec![r(2), r(2), r(2), r(5), r(5), r(5)],
            vec![r(7), r(7), r(11), r(11)],
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curve(section) = plane.intersect_surface(&nurbs).unwrap()
        else {
            panic!("v-linear NURBS tensor must retain its native graph section");
        };
        assert_eq!(section.curve().domain().start(), &r(2));
        assert_eq!(section.curve().domain().end(), &r(5));
        assert_eq!(
            section.second_pcurve().point_at(&q(7, 2)).unwrap(),
            Point2::new(q(7, 2), q(17, 2))
        );
        let materialized = section.second_pcurve().materialize().unwrap();
        assert_eq!(
            materialized.correspondence().affine_coefficients(),
            Some((&r(3), &r(2)))
        );
        assert_eq!(
            materialized.curve().point_at(&q(1, 2)).unwrap(),
            CurvePoint2::new(q(7, 2), q(17, 2))
        );

        let (rational_patch, rational_face) =
            crate::builder::rational_bezier_patch(controls.clone(), weights.clone()).unwrap();
        let rational_surface = rational_patch
            .surface(rational_patch.face(rational_face).unwrap().surface())
            .unwrap();
        let SurfaceSurfaceIntersection::Curve(rational_section) =
            rational_surface.intersect_surface(&plane).unwrap()
        else {
            panic!("v-linear rational patch must retain its graph section");
        };
        let (rational_split, _) = rational_patch
            .split_face_by_surface_curve(
                rational_face,
                rational_section.curve(),
                rational_section.first_pcurve(),
            )
            .unwrap();
        assert_eq!(rational_split.counts().faces, 2);
        let rational_json = rational_split.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&rational_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            rational_json
        );

        let (nurbs_patch, nurbs_face) = crate::builder::nurbs_patch(
            2,
            1,
            controls,
            weights,
            vec![r(2), r(2), r(2), r(5), r(5), r(5)],
            vec![r(7), r(7), r(11), r(11)],
        )
        .unwrap();
        let nurbs_surface = nurbs_patch
            .surface(nurbs_patch.face(nurbs_face).unwrap().surface())
            .unwrap();
        let SurfaceSurfaceIntersection::Curve(nurbs_section) =
            nurbs_surface.intersect_surface(&plane).unwrap()
        else {
            panic!("v-linear NURBS patch must retain its native graph section");
        };
        let (nurbs_split, _) = nurbs_patch
            .split_face_by_surface_curve(
                nurbs_face,
                nurbs_section.curve(),
                nurbs_section.first_pcurve(),
            )
            .unwrap();
        assert_eq!(nurbs_split.counts().faces, 2);
        let nurbs_json = nurbs_split.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&nurbs_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            nurbs_json
        );
    }

    #[test]
    fn rational_tensor_graph_sections_split_and_revalidate_trimmed_topology() {
        let controls = vec![
            vec![p(0, 0, 0), p(2, 0, 0)],
            vec![p(0, 2, 1), p(2, 2, 1)],
            vec![p(0, 2, 2), p(2, 2, 2)],
        ];
        let weights = vec![vec![r(1), r(1)], vec![r(2), r(2)], vec![r(3), r(3)]];
        let (patch, face) = crate::builder::rational_bezier_patch(controls, weights).unwrap();
        let surface = patch.surface(patch.face(face).unwrap().surface()).unwrap();
        let plane = Surface::plane(
            p(2, 0, 0),
            Vector3::y(),
            Vector3::from_xyz(r(1), r(0), r(-1)),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curve(section) = surface.intersect_surface(&plane).unwrap()
        else {
            panic!("translation tensor must retain one exact graph section");
        };
        let (split, record) = patch
            .split_face_by_surface_curve(face, section.curve(), section.first_pcurve())
            .unwrap();
        assert_eq!(split.counts().faces, 2);
        assert_eq!(record.first_face(), face);
        let graph_use = split
            .edge_use(record.open().unwrap().face.edge_uses[0])
            .unwrap();
        let graph_pcurve = split.pcurve(graph_use.pcurve()).unwrap();
        let CurveGeometry2::RationalBezier(graph) = graph_pcurve.curve().geometry() else {
            panic!("split tensor section must retain one rational graph pcurve");
        };
        let mut forged_controls = graph.control_points().to_vec();
        forged_controls[1] = CurvePoint2::new(
            forged_controls[1].x().clone() + Real::one(),
            forged_controls[1].y().clone(),
        );
        let forged = Pcurve::new(Curve2::from(
            RationalBezier2::try_new(forged_controls, graph.weights().to_vec()).unwrap(),
        ));
        let mut edit = split.edit();
        edit.replace_pcurve(graph_use.pcurve(), forged).unwrap();
        assert!(
            edit.commit().is_err(),
            "endpoint-preserving graph forgery must fail full control-net validation"
        );
        let json = split.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn single_span_nurbs_tensor_graph_sections_retain_native_domain_through_topology() {
        let controls = vec![
            vec![p(0, 0, 0), p(2, 0, 0)],
            vec![p(0, 2, 1), p(2, 2, 1)],
            vec![p(0, 2, 2), p(2, 2, 2)],
        ];
        let weights = vec![vec![r(1), r(1)], vec![r(2), r(2)], vec![r(3), r(3)]];
        let (patch, face) = crate::builder::nurbs_patch(
            1,
            2,
            controls,
            weights,
            vec![r(7), r(7), r(11), r(11)],
            vec![r(2), r(2), r(2), r(5), r(5), r(5)],
        )
        .unwrap();
        let surface = patch.surface(patch.face(face).unwrap().surface()).unwrap();
        let plane = Surface::plane(
            p(2, 0, 0),
            Vector3::y(),
            Vector3::from_xyz(r(1), r(0), r(-1)),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curve(section) = surface.intersect_surface(&plane).unwrap()
        else {
            panic!("single-span NURBS tensor must retain one graph section");
        };
        assert_eq!(section.curve().domain().start(), &r(2));
        assert_eq!(section.curve().domain().end(), &r(5));
        let materialized = section.first_pcurve().materialize().unwrap();
        assert_eq!(
            materialized.correspondence().affine_coefficients(),
            Some((&r(3), &r(2)))
        );
        assert_eq!(
            materialized.curve().point_at(&q(1, 2)).unwrap(),
            CurvePoint2::new(q(17, 2), q(7, 2))
        );
        let (split, _) = patch
            .split_face_by_surface_curve(face, section.curve(), section.first_pcurve())
            .unwrap();
        assert_eq!(split.counts().faces, 2);
        let json = split.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );
    }

    #[test]
    fn multi_span_nurbs_tensor_graph_sections_materialize_split_and_replay_exactly() {
        let controls = vec![
            vec![p(0, 0, 0), p(2, 0, 0)],
            vec![p(0, 2, 1), p(2, 2, 1)],
            vec![p(0, 3, 1), p(2, 3, 1)],
            vec![p(0, 1, 2), p(2, 1, 2)],
        ];
        let weights = vec![
            vec![r(1), r(1)],
            vec![r(2), r(2)],
            vec![r(3), r(3)],
            vec![r(1), r(1)],
        ];
        let (patch, face) = crate::builder::nurbs_patch(
            1,
            2,
            controls,
            weights,
            vec![r(7), r(7), r(11), r(11)],
            vec![r(2), r(2), r(2), r(3), r(5), r(5), r(5)],
        )
        .unwrap();
        let surface = patch.surface(patch.face(face).unwrap().surface()).unwrap();
        let plane = Surface::plane(
            p(2, 0, 0),
            Vector3::y(),
            Vector3::from_xyz(r(1), r(0), r(-1)),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curve(section) = surface.intersect_surface(&plane).unwrap()
        else {
            panic!("multi-span NURBS tensor must retain one graph section");
        };
        assert_eq!(section.curve().domain().start(), &r(2));
        assert_eq!(section.curve().domain().end(), &r(5));
        let materialized = section.first_pcurve().materialize().unwrap();
        assert_eq!(materialized.curve().family(), CurveFamily2::Nurbs);
        assert_eq!(
            materialized.correspondence().affine_coefficients(),
            Some((&r(1), &r(0)))
        );
        for parameter in [r(2), r(3), r(4), r(5)] {
            let surface_parameter = materialized.curve().point_at(&parameter).unwrap();
            assert_points_equal(
                &section.curve().point_at(&parameter).unwrap(),
                &surface
                    .point_at(&Point2::new(
                        surface_parameter.x().clone(),
                        surface_parameter.y().clone(),
                    ))
                    .unwrap(),
            );
        }
        let (split, record) = patch
            .split_face_by_surface_curve(face, section.curve(), section.first_pcurve())
            .unwrap();
        assert_eq!(split.counts().faces, 2);
        let graph_use = split
            .edge_use(record.open().unwrap().face.edge_uses[0])
            .unwrap();
        let graph_pcurve = split.pcurve(graph_use.pcurve()).unwrap();
        let CurveGeometry2::Nurbs(graph) = graph_pcurve.curve().geometry() else {
            panic!("multi-span tensor split must retain one NURBS graph pcurve");
        };
        let mut forged_controls = graph.control_points().to_vec();
        forged_controls[1] = CurvePoint2::new(
            forged_controls[1].x().clone() + Real::one(),
            forged_controls[1].y().clone(),
        );
        let forged = Pcurve::new(
            Curve2::try_nurbs(
                graph.degree(),
                forged_controls,
                graph.weights().to_vec(),
                graph.knots().to_vec(),
            )
            .unwrap(),
        );
        let mut edit = split.edit();
        edit.replace_pcurve(graph_use.pcurve(), forged).unwrap();
        assert!(
            edit.commit().is_err(),
            "endpoint-preserving NURBS graph forgery must fail full control-net validation"
        );
        let json = split.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            json
        );

        let (v_patch, v_face) = crate::builder::nurbs_patch(
            2,
            1,
            vec![
                vec![p(0, 0, 0), p(1, 2, 0), p(1, 3, 0), p(2, 1, 0)],
                vec![p(0, 0, 2), p(1, 2, 2), p(1, 3, 2), p(2, 1, 2)],
            ],
            vec![vec![r(1), r(2), r(3), r(1)], vec![r(1), r(2), r(3), r(1)]],
            vec![r(2), r(2), r(2), r(3), r(5), r(5), r(5)],
            vec![r(7), r(7), r(11), r(11)],
        )
        .unwrap();
        let v_surface = v_patch
            .surface(v_patch.face(v_face).unwrap().surface())
            .unwrap();
        let SurfaceSurfaceIntersection::Curve(v_section) =
            v_surface.intersect_surface(&plane).unwrap()
        else {
            panic!("multi-span v-linear NURBS tensor must retain one graph section");
        };
        let v_materialized = v_section.first_pcurve().materialize().unwrap();
        assert_eq!(v_materialized.curve().family(), CurveFamily2::Nurbs);
        assert_eq!(
            v_materialized.correspondence().affine_coefficients(),
            Some((&r(1), &r(0)))
        );
        let (v_split, _) = v_patch
            .split_face_by_surface_curve(v_face, v_section.curve(), v_section.first_pcurve())
            .unwrap();
        assert_eq!(v_split.counts().faces, 2);
        let v_json = v_split.to_json().unwrap();
        assert_eq!(
            crate::RawModel::from_json(&v_json)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap(),
            v_json
        );
    }

    #[test]
    fn extrusion_surfaces_intersect_planes_in_exact_native_curves_and_lines() {
        let profile = Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 2, 0), p(2, 0, 0)],
            vec![r(1), r(2), r(1)],
        )
        .unwrap();
        let extrusion = Surface::extrusion(profile.clone(), Vector3::z()).unwrap();
        let oblique = Surface::plane(
            Point3::origin(),
            Vector3::from_xyz(r(1), r(0), r(1)),
            Vector3::y(),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curve(section) =
            extrusion.intersect_surface(&oblique).unwrap()
        else {
            panic!("transverse plane must retain the projected native profile");
        };
        assert_eq!(section.curve().kind(), Curve3Kind::RationalBezier);
        let parameter = q(1, 2);
        let profile_point = profile.point_at(&parameter).unwrap();
        let expected = Point3::new(profile_point.x.clone(), profile_point.y, profile_point.x);
        assert_points_equal(&section.curve().point_at(&parameter).unwrap(), &expected);
        assert_eq!(
            section.first_pcurve().point_at(&parameter).unwrap(),
            Point2::new(parameter.clone(), expected.z.clone())
        );
        let extrusion_clip = section
            .first_pcurve()
            .clipping_carriers()
            .unwrap()
            .expect("rational extrusion section has an exact planar carrier")
            .pop()
            .expect("one rational extrusion span");
        let extrusion_parameter = extrusion_clip.curve.point_at(&parameter).unwrap();
        assert_eq!(
            compare_reals(extrusion_parameter.x(), &parameter).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(extrusion_parameter.y(), &expected.z).value(),
            Some(Ordering::Equal)
        );
        let plane_clip = section
            .second_pcurve()
            .clipping_carriers()
            .unwrap()
            .expect("projected rational section has an exact planar carrier")
            .pop()
            .expect("one projected rational span");
        let plane_parameter = plane_clip.curve.point_at(&parameter).unwrap();
        let retained_plane_parameter = section.second_pcurve().point_at(&parameter).unwrap();
        assert_eq!(
            compare_reals(plane_parameter.x(), &retained_plane_parameter.x).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(plane_parameter.y(), &retained_plane_parameter.y).value(),
            Some(Ordering::Equal)
        );

        let nurbs_profile = Curve3::nurbs(
            1,
            vec![p(0, 0, 0), p(1, 2, 0), p(2, 0, 0)],
            vec![r(1), r(2), r(1)],
            vec![r(2), r(2), r(3), r(4), r(4)],
        )
        .unwrap();
        let nurbs_extrusion = Surface::extrusion(nurbs_profile.clone(), Vector3::z()).unwrap();
        let SurfaceSurfaceIntersection::Curve(nurbs_section) =
            nurbs_extrusion.intersect_surface(&oblique).unwrap()
        else {
            panic!("transverse NURBS extrusion must retain one exact section");
        };
        let carriers = nurbs_section
            .first_pcurve()
            .clipping_carriers()
            .unwrap()
            .expect("NURBS extrusion section decomposes into rational trim carriers");
        assert_eq!(carriers.len(), 2);
        for carrier in &carriers {
            for local in [Real::zero(), Real::one()] {
                let spatial_parameter = &carrier.spatial_scale * &local + &carrier.spatial_offset;
                let materialized = carrier.curve.point_at(&local).unwrap();
                let retained = nurbs_section
                    .first_pcurve()
                    .point_at(&spatial_parameter)
                    .unwrap();
                assert_eq!(
                    compare_reals(materialized.x(), &retained.x).value(),
                    Some(Ordering::Equal)
                );
                assert_eq!(
                    compare_reals(materialized.y(), &retained.y).value(),
                    Some(Ordering::Equal)
                );
            }
        }
        let trim_points = [
            CurvePoint2::new(q(5, 2), r(-100)),
            CurvePoint2::new(q(7, 2), r(-100)),
            CurvePoint2::new(q(7, 2), r(100)),
            CurvePoint2::new(q(5, 2), r(100)),
        ];
        let trim_contour = hypercurve::Contour2::try_new(
            (0..trim_points.len())
                .map(|index| {
                    LineSeg2::try_new(
                        trim_points[index].clone(),
                        trim_points[(index + 1) % trim_points.len()].clone(),
                    )
                    .map(Segment2::Line)
                })
                .collect::<Result<Vec<_>, _>>()
                .unwrap(),
        )
        .unwrap();
        let trim_region = hypercurve::LineArcRegion2::from_material_contours(vec![trim_contour]);
        let mut retained_ranges = Vec::new();
        for carrier in &carriers {
            for fragment in carrier
                .curve
                .trim_inside_region_with_parameters(&trim_region, &CurvePolicy::certified())
                .unwrap()
            {
                let (local_start, local_end) = fragment
                    .represented_parameter_range()
                    .expect("linear region contacts have represented parameters");
                retained_ranges.push((
                    &carrier.spatial_scale * local_start + &carrier.spatial_offset,
                    &carrier.spatial_scale * local_end + &carrier.spatial_offset,
                ));
            }
        }
        assert_eq!(retained_ranges.len(), 2);
        for ((actual_start, actual_end), (expected_start, expected_end)) in retained_ranges
            .into_iter()
            .zip([(q(5, 2), r(3)), (r(3), q(7, 2))])
        {
            assert_eq!(
                compare_reals(&actual_start, &expected_start).value(),
                Some(Ordering::Equal)
            );
            assert_eq!(
                compare_reals(&actual_end, &expected_end).value(),
                Some(Ordering::Equal)
            );
        }

        let ruled = Surface::extrusion(Curve3::line(p(0, 0, 0), p(2, 0, 0)).unwrap(), Vector3::z())
            .unwrap();
        let x_one = Surface::plane(p(1, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        let SurfaceSurfaceIntersection::Line(line) = x_one.intersect_surface(&ruled).unwrap()
        else {
            panic!("parallel plane/profile point must lift to one extrusion line");
        };
        assert_points_equal(&line.point, &p(1, 0, 0));
        assert_points_equal(&Point3::from(line.direction), &Point3::from(Vector3::z()));
        assert!(matches!(
            ruled
                .intersect_surface(&Surface::plane(p(0, 1, 0), Vector3::x(), Vector3::z()).unwrap())
                .unwrap(),
            SurfaceSurfaceIntersection::None
        ));
        assert_eq!(
            ruled
                .intersect_surface(
                    &Surface::plane(Point3::origin(), Vector3::x(), Vector3::z()).unwrap()
                )
                .unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
    }

    #[test]
    fn degenerate_geometry_is_rejected_by_certified_predicates() {
        assert_eq!(
            Curve3::line(p(1, 2, 3), p(1, 2, 3)).unwrap_err(),
            GeometryError::DegenerateLine
        );
        assert_eq!(
            Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::x()).unwrap_err(),
            GeometryError::DegeneratePlaneBasis
        );
    }

    #[test]
    fn line_plane_intersection_distinguishes_point_none_and_contained() {
        let plane = Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap();
        let crossing = Curve3::line(p(2, 3, -5), p(2, 3, 5)).unwrap();
        match plane.intersect_curve(&crossing).unwrap() {
            CurveSurfaceIntersection::Points(points) => {
                assert_eq!(points.len(), 1);
                let intersection = &points[0];
                assert_eq!(
                    compare_reals(&intersection.parameter, &q(1, 2)).value(),
                    Some(Ordering::Equal)
                );
                assert_points_equal(&intersection.point, &p(2, 3, 0));
                assert_eq!(intersection.multiplicity, IntersectionMultiplicity::Simple);
            }
            other => panic!("expected point intersection, got {other:?}"),
        }
        assert!(matches!(
            plane
                .intersect_curve(&Curve3::line(p(0, 0, 2), p(1, 0, 2)).unwrap())
                .unwrap(),
            CurveSurfaceIntersection::None
        ));
        assert!(matches!(
            plane
                .intersect_curve(&Curve3::line(p(0, 0, 0), p(1, 0, 0)).unwrap())
                .unwrap(),
            CurveSurfaceIntersection::Contained
        ));
    }

    #[test]
    fn ellipse_plane_intersection_retains_exact_parameters_tangency_and_seams() {
        let circle = Curve3::circle_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            r(2),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        let y_axis_plane = Surface::plane(Point3::origin(), Vector3::y(), Vector3::z()).unwrap();
        let CurveSurfaceIntersection::Points(points) =
            y_axis_plane.intersect_curve(&circle).unwrap()
        else {
            panic!("diametral plane must retain two circle parameters");
        };
        assert_eq!(points.len(), 2);
        assert_eq!(
            compare_reals(&points[0].parameter, &(Real::pi() / r(2)).unwrap()).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&points[1].parameter, &(r(3) * Real::pi() / r(2)).unwrap(),).value(),
            Some(Ordering::Equal)
        );
        assert!(
            points
                .iter()
                .all(|point| point.multiplicity == IntersectionMultiplicity::Simple)
        );

        let tangent = Surface::plane(p(2, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        let CurveSurfaceIntersection::Points(points) = tangent.intersect_curve(&circle).unwrap()
        else {
            panic!("tangent plane must retain represented seam parameters");
        };
        assert_eq!(points.len(), 2);
        assert_eq!(
            compare_reals(&points[0].parameter, &Real::zero()).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&points[1].parameter, &Real::tau()).value(),
            Some(Ordering::Equal)
        );
        assert!(
            points
                .iter()
                .all(|point| point.multiplicity == IntersectionMultiplicity::Tangent)
        );

        let carrier_plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        assert!(matches!(
            carrier_plane.intersect_curve(&circle).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
        let parallel = Surface::plane(p(0, 0, 1), Vector3::x(), Vector3::y()).unwrap();
        assert!(matches!(
            parallel.intersect_curve(&circle).unwrap(),
            CurveSurfaceIntersection::None
        ));

        let ellipse = Curve3::ellipse_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            r(3),
            r(2),
            Real::zero(),
            Real::pi(),
        )
        .unwrap();
        let CurveSurfaceIntersection::Points(points) =
            y_axis_plane.intersect_curve(&ellipse).unwrap()
        else {
            panic!("finite half ellipse must retain its one represented crossing");
        };
        assert_eq!(points.len(), 1);
        assert_eq!(
            compare_reals(&points[0].parameter, &(Real::pi() / r(2)).unwrap()).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn circle_sphere_intersection_retains_exact_arc_parameters_and_multiplicity() {
        let sphere = Surface::sphere(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(5),
        )
        .unwrap();
        let secant = Curve3::circle_arc(
            p(3, 0, 0),
            Vector3::x(),
            Vector3::y(),
            r(4),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        let CurveSurfaceIntersection::Points(points) = sphere.intersect_curve(&secant).unwrap()
        else {
            panic!("offset circle must cross the sphere twice");
        };
        assert_eq!(points.len(), 2);
        assert_points_equal(&points[0].point, &p(3, 4, 0));
        assert_points_equal(&points[1].point, &p(3, -4, 0));
        assert!(
            points
                .iter()
                .all(|point| point.multiplicity == IntersectionMultiplicity::Simple)
        );

        let tangent = Curve3::circle_arc(
            p(4, 0, 0),
            Vector3::x(),
            Vector3::y(),
            r(1),
            Real::zero(),
            Real::pi(),
        )
        .unwrap();
        let CurveSurfaceIntersection::Points(points) = sphere.intersect_curve(&tangent).unwrap()
        else {
            panic!("internally tangent circle must retain one point");
        };
        assert_eq!(points.len(), 1);
        assert_points_equal(&points[0].point, &p(5, 0, 0));
        assert_eq!(points[0].multiplicity, IntersectionMultiplicity::Tangent);

        let contained = Curve3::circle_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            r(5),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        assert!(matches!(
            sphere.intersect_curve(&contained).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
        let inside = Curve3::circle_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            r(1),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        assert!(matches!(
            sphere.intersect_curve(&inside).unwrap(),
            CurveSurfaceIntersection::None
        ));
    }

    #[test]
    fn transverse_circle_cylinder_intersection_reuses_exact_circle_relation() {
        let cylinder = Surface::cylinder(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(5),
        )
        .unwrap();
        let secant = Curve3::circle_arc(
            p(3, 0, 2),
            Vector3::x(),
            Vector3::y(),
            r(4),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        let CurveSurfaceIntersection::Points(points) = cylinder.intersect_curve(&secant).unwrap()
        else {
            panic!("transverse offset circle must cross the cylinder twice");
        };
        assert_eq!(points.len(), 2);
        assert_points_equal(&points[0].point, &p(3, 4, 2));
        assert_points_equal(&points[1].point, &p(3, -4, 2));

        let tangent = Curve3::circle_arc(
            p(4, 0, -3),
            Vector3::x(),
            Vector3::y(),
            r(1),
            Real::zero(),
            Real::pi(),
        )
        .unwrap();
        let CurveSurfaceIntersection::Points(points) = cylinder.intersect_curve(&tangent).unwrap()
        else {
            panic!("transverse tangent circle must retain one point");
        };
        assert_eq!(points.len(), 1);
        assert_eq!(points[0].multiplicity, IntersectionMultiplicity::Tangent);
        assert_points_equal(&points[0].point, &p(5, 0, -3));

        let contained = Curve3::circle_arc(
            p(0, 0, 7),
            Vector3::x(),
            Vector3::y(),
            r(5),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        assert!(matches!(
            cylinder.intersect_curve(&contained).unwrap(),
            CurveSurfaceIntersection::Contained
        ));

        let oblique = Curve3::circle_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::z(),
            r(5),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        assert_eq!(
            cylinder.intersect_curve(&oblique).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
    }

    #[test]
    fn transverse_circle_cone_intersection_respects_the_upper_nappe() {
        let cone = Surface::cone(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            (Real::pi() / r(4)).unwrap(),
        )
        .unwrap();
        let secant = Curve3::circle_arc(
            p(3, 0, 5),
            Vector3::x(),
            Vector3::y(),
            r(4),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        let CurveSurfaceIntersection::Points(points) = cone.intersect_curve(&secant).unwrap()
        else {
            panic!("upper transverse circle must meet the cone twice");
        };
        assert_eq!(points.len(), 2);
        assert_points_equal(&points[0].point, &p(3, 4, 5));
        assert_points_equal(&points[1].point, &p(3, -4, 5));

        let contained = Curve3::circle_arc(
            p(0, 0, 5),
            Vector3::x(),
            Vector3::y(),
            r(5),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        assert!(matches!(
            cone.intersect_curve(&contained).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
        let below = Curve3::circle_arc(
            p(0, 0, -5),
            Vector3::x(),
            Vector3::y(),
            r(5),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        assert!(matches!(
            cone.intersect_curve(&below).unwrap(),
            CurveSurfaceIntersection::None
        ));
    }

    #[test]
    fn transverse_circle_torus_intersection_checks_both_radial_sections() {
        let torus = Surface::torus(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(3),
            r(1),
        )
        .unwrap();
        let twice_tangent = Curve3::circle_arc(
            p(3, 0, 0),
            Vector3::x(),
            Vector3::y(),
            r(1),
            Real::zero(),
            Real::pi(),
        )
        .unwrap();
        let CurveSurfaceIntersection::Points(points) =
            torus.intersect_curve(&twice_tangent).unwrap()
        else {
            panic!("center-plane circle must touch both torus radial sections");
        };
        assert_eq!(points.len(), 2);
        assert_points_equal(&points[0].point, &p(4, 0, 0));
        assert_points_equal(&points[1].point, &p(2, 0, 0));
        assert!(
            points
                .iter()
                .all(|point| point.multiplicity == IntersectionMultiplicity::Tangent)
        );

        let outer = Curve3::circle_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            r(4),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        assert!(matches!(
            torus.intersect_curve(&outer).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
        let above = Curve3::circle_arc(
            p(0, 0, 2),
            Vector3::x(),
            Vector3::y(),
            r(3),
            Real::zero(),
            Real::tau(),
        )
        .unwrap();
        assert!(matches!(
            torus.intersect_curve(&above).unwrap(),
            CurveSurfaceIntersection::None
        ));
    }

    #[test]
    fn line_sphere_and_cylinder_intersections_retain_multiplicity() {
        let sphere = Surface::sphere(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(2),
        )
        .unwrap();
        let crossing = Curve3::line(p(-3, 0, 0), p(3, 0, 0)).unwrap();
        let CurveSurfaceIntersection::Points(points) = sphere.intersect_curve(&crossing).unwrap()
        else {
            panic!("expected two sphere intersections");
        };
        assert_eq!(points.len(), 2);
        assert_points_equal(&points[0].point, &p(-2, 0, 0));
        assert_points_equal(&points[1].point, &p(2, 0, 0));
        assert!(
            points
                .iter()
                .all(|point| point.multiplicity == IntersectionMultiplicity::Simple)
        );
        let tangent = Curve3::line(p(-3, 2, 0), p(3, 2, 0)).unwrap();
        let CurveSurfaceIntersection::Points(points) = sphere.intersect_curve(&tangent).unwrap()
        else {
            panic!("expected tangent sphere intersection");
        };
        assert_eq!(points.len(), 1);
        assert_eq!(points[0].multiplicity, IntersectionMultiplicity::Tangent);
        assert_points_equal(&points[0].point, &p(0, 2, 0));

        let cylinder = Surface::cylinder(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(2),
        )
        .unwrap();
        let axial = Curve3::line(p(2, 0, -3), p(2, 0, 3)).unwrap();
        assert!(matches!(
            cylinder.intersect_curve(&axial).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
    }

    #[test]
    fn line_cone_intersection_filters_the_lower_nappe_and_retains_overlap() {
        let quarter_turn = (Real::pi() / r(4)).unwrap();
        let cone = Surface::cone(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            quarter_turn,
        )
        .unwrap();
        let crossing = Curve3::line(p(-2, 0, 1), p(2, 0, 1)).unwrap();
        let CurveSurfaceIntersection::Points(points) = cone.intersect_curve(&crossing).unwrap()
        else {
            panic!("line through the upper cone must have two intersections");
        };
        assert_eq!(points.len(), 2);
        assert_points_equal(&points[0].point, &p(-1, 0, 1));
        assert_points_equal(&points[1].point, &p(1, 0, 1));

        let lower = Curve3::line(p(-2, 0, -1), p(2, 0, -1)).unwrap();
        assert!(matches!(
            cone.intersect_curve(&lower).unwrap(),
            CurveSurfaceIntersection::None
        ));
        let generator = Curve3::line(p(-2, 0, -2), p(2, 0, 2)).unwrap();
        let CurveSurfaceIntersection::Overlap(domain) = cone.intersect_curve(&generator).unwrap()
        else {
            panic!("generator crossing the apex must retain its upper interval");
        };
        assert_eq!(
            compare_reals(domain.start(), &q(1, 2)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(domain.end(), &r(1)).value(),
            Some(Ordering::Equal)
        );
        let contained = Curve3::line(p(1, 0, 1), p(2, 0, 2)).unwrap();
        assert!(matches!(
            cone.intersect_curve(&contained).unwrap(),
            CurveSurfaceIntersection::Contained
        ));
    }

    #[test]
    fn plane_plane_intersection_is_exact() {
        let x_two = Surface::plane(p(2, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        let y_three = Surface::plane(p(0, 3, 0), Vector3::z(), Vector3::x()).unwrap();
        match x_two.intersect_surface(&y_three).unwrap() {
            SurfaceSurfaceIntersection::Line(line) => {
                assert_points_equal(&line.point, &p(2, 3, 0));
                for (actual, expected) in line.direction.0.iter().zip(Vector3::z().0.iter()) {
                    assert_eq!(
                        compare_reals(actual, expected).value(),
                        Some(Ordering::Equal)
                    );
                }
            }
            other => panic!("expected line intersection, got {other:?}"),
        }
        let x_four = Surface::plane(p(4, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        assert!(matches!(
            x_two.intersect_surface(&x_four).unwrap(),
            SurfaceSurfaceIntersection::None
        ));
        let x_two_other_frame = Surface::plane(p(2, 5, 7), Vector3::z(), Vector3::y()).unwrap();
        assert!(matches!(
            x_two.intersect_surface(&x_two_other_frame).unwrap(),
            SurfaceSurfaceIntersection::Coincident
        ));
    }

    #[test]
    fn plane_cylinder_intersection_retains_circles_ellipses_and_axial_lines() {
        let cylinder = Surface::cylinder(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(2),
        )
        .unwrap();
        let transverse = Surface::plane(p(0, 0, 3), Vector3::x(), Vector3::y()).unwrap();
        let SurfaceSurfaceIntersection::Curve(section) =
            cylinder.intersect_surface(&transverse).unwrap()
        else {
            panic!("transverse plane must retain a full circle with both exact pcurves");
        };
        assert_points_equal(&section.curve().start().unwrap(), &p(2, 0, 3));
        let cylinder_pcurve = section.first_pcurve().materialize().unwrap();
        assert_eq!(cylinder_pcurve.curve().family(), CurveFamily2::Line);
        assert!(matches!(
            cylinder_pcurve.correspondence(),
            SurfacePcurveCorrespondence::Affine { .. }
        ));
        let plane_pcurve = section.second_pcurve().materialize().unwrap();
        assert_eq!(plane_pcurve.curve().family(), CurveFamily2::CircularArc);
        assert!(matches!(
            plane_pcurve.correspondence(),
            SurfacePcurveCorrespondence::AngularSweep { .. }
        ));

        let secant = Surface::plane(Point3::origin(), Vector3::y(), Vector3::z()).unwrap();
        let SurfaceSurfaceIntersection::Lines(lines) = secant.intersect_surface(&cylinder).unwrap()
        else {
            panic!("axial secant plane must retain two lines");
        };
        assert_eq!(lines.len(), 2);
        assert_points_equal(&lines[0].point, &p(0, 2, 0));
        assert_points_equal(&lines[1].point, &p(0, -2, 0));
        for line in lines {
            assert_eq!(
                point3_equal(&Point3::from(line.direction), &Point3::from(Vector3::z()),).value(),
                Some(true)
            );
        }

        let tangent = Surface::plane(p(2, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        let SurfaceSurfaceIntersection::Line(line) = cylinder.intersect_surface(&tangent).unwrap()
        else {
            panic!("axial tangent plane must retain one line");
        };
        assert_points_equal(&line.point, &p(2, 0, 0));
        let disjoint = Surface::plane(p(3, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        assert!(matches!(
            cylinder.intersect_surface(&disjoint).unwrap(),
            SurfaceSurfaceIntersection::None
        ));

        let oblique = Surface::plane(
            Point3::origin(),
            Vector3::x(),
            Vector3::from_xyz(Real::zero(), Real::one(), Real::one()),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Ellipse(ellipse) =
            cylinder.intersect_surface(&oblique).unwrap()
        else {
            panic!("oblique plane must retain one exact full ellipse");
        };
        assert_eq!(ellipse.kind(), Curve3Kind::EllipseArc);
        assert_points_equal(&ellipse.point_at(&Real::zero()).unwrap(), &p(2, 0, 0));
        assert_points_equal(
            &ellipse.point_at(&(Real::pi() / r(2)).unwrap()).unwrap(),
            &p(0, -2, -2),
        );
        for parameter in [
            Real::zero(),
            (Real::pi() / r(2)).unwrap(),
            Real::pi(),
            (r(3) * Real::pi() / r(2)).unwrap(),
        ] {
            let point = ellipse.point_at(&parameter).unwrap();
            assert_eq!(
                compare_reals(&(&point.x * &point.x + &point.y * &point.y), &r(4)).value(),
                Some(Ordering::Equal)
            );
            assert_eq!(
                compare_reals(&point.y, &point.z).value(),
                Some(Ordering::Equal)
            );
        }
    }

    #[test]
    fn transverse_planes_retain_exact_cone_and_torus_sections() {
        let cone = Surface::cone(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            (Real::pi() / r(4)).unwrap(),
        )
        .unwrap();
        let z_two = Surface::plane(p(0, 0, 2), Vector3::x(), Vector3::y()).unwrap();
        let SurfaceSurfaceIntersection::Curve(section) = cone.intersect_surface(&z_two).unwrap()
        else {
            panic!("transverse upper-cone cut must retain a circle and both exact pcurves");
        };
        assert_points_equal(&section.curve().start().unwrap(), &p(2, 0, 2));
        assert_eq!(
            section
                .first_pcurve()
                .materialize()
                .unwrap()
                .curve()
                .family(),
            CurveFamily2::Line
        );
        assert_eq!(
            section
                .second_pcurve()
                .materialize()
                .unwrap()
                .curve()
                .family(),
            CurveFamily2::CircularArc
        );
        let apex_plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        let SurfaceSurfaceIntersection::Point(point) = cone.intersect_surface(&apex_plane).unwrap()
        else {
            panic!("apex plane must retain the singular point");
        };
        assert_points_equal(&point, &Point3::origin());
        let below = Surface::plane(p(0, 0, -1), Vector3::x(), Vector3::y()).unwrap();
        assert!(matches!(
            cone.intersect_surface(&below).unwrap(),
            SurfaceSurfaceIntersection::None
        ));

        let torus = Surface::torus(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(3),
            r(1),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curves(circles) =
            torus.intersect_surface(&apex_plane).unwrap()
        else {
            panic!("torus center plane must retain two circles");
        };
        assert_eq!(circles.len(), 2);
        assert_points_equal(&circles[0].curve().start().unwrap(), &p(4, 0, 0));
        assert_points_equal(&circles[1].curve().start().unwrap(), &p(2, 0, 0));
        for section in &circles {
            assert_eq!(
                section
                    .first_pcurve()
                    .materialize()
                    .unwrap()
                    .curve()
                    .family(),
                CurveFamily2::Line
            );
            assert_eq!(
                section
                    .second_pcurve()
                    .materialize()
                    .unwrap()
                    .curve()
                    .family(),
                CurveFamily2::CircularArc
            );
        }
        let tangent = Surface::plane(p(0, 0, 1), Vector3::x(), Vector3::y()).unwrap();
        let SurfaceSurfaceIntersection::Curve(circle) = torus.intersect_surface(&tangent).unwrap()
        else {
            panic!("top torus plane must retain one tangent circle");
        };
        assert_points_equal(&circle.curve().start().unwrap(), &p(3, 0, 1));
        let disjoint = Surface::plane(p(0, 0, 2), Vector3::x(), Vector3::y()).unwrap();
        assert!(matches!(
            torus.intersect_surface(&disjoint).unwrap(),
            SurfaceSurfaceIntersection::None
        ));

        let axial_plane = Surface::plane(Point3::origin(), Vector3::y(), Vector3::z()).unwrap();
        let SurfaceSurfaceIntersection::Rays(rays) = cone.intersect_surface(&axial_plane).unwrap()
        else {
            panic!("an axis-containing plane must retain both upper-cone generator rays");
        };
        assert_eq!(rays.len(), 2);
        let sqrt_two = r(2).sqrt().unwrap();
        for ray in &rays {
            assert_points_equal(&ray.point, &Point3::origin());
            assert_eq!(
                compare_reals(&ray.direction.norm_squared(), &Real::one()).value(),
                Some(Ordering::Equal)
            );
            assert_eq!(
                compare_reals(&ray.minimum, &Real::zero()).value(),
                Some(Ordering::Equal)
            );
            for parameter in [Real::zero(), sqrt_two.clone()] {
                let spatial = ray.point.clone() + ray.direction.clone() * &parameter;
                assert_points_equal(
                    &cone
                        .point_at(
                            &ray.pcurve(SurfaceIntersectionOperand::First)
                                .point_at(&parameter),
                        )
                        .unwrap(),
                    &spatial,
                );
                assert_points_equal(
                    &axial_plane
                        .point_at(
                            &ray.pcurve(SurfaceIntersectionOperand::Second)
                                .point_at(&parameter),
                        )
                        .unwrap(),
                    &spatial,
                );
            }
        }
        assert_points_equal(
            &(rays[0].point.clone() + rays[0].direction.clone() * &sqrt_two),
            &p(0, -1, 1),
        );
        assert_points_equal(
            &(rays[1].point.clone() + rays[1].direction.clone() * &sqrt_two),
            &p(0, 1, 1),
        );
        let SurfaceSurfaceIntersection::Rays(swapped) =
            axial_plane.intersect_surface(&cone).unwrap()
        else {
            panic!("operand reversal must retain both cone rays");
        };
        assert_eq!(swapped.len(), 2);
        for ray in &swapped {
            let spatial = ray.point.clone() + ray.direction.clone() * &sqrt_two;
            assert_points_equal(
                &axial_plane
                    .point_at(
                        &ray.pcurve(SurfaceIntersectionOperand::First)
                            .point_at(&sqrt_two),
                    )
                    .unwrap(),
                &spatial,
            );
            assert_points_equal(
                &cone
                    .point_at(
                        &ray.pcurve(SurfaceIntersectionOperand::Second)
                            .point_at(&sqrt_two),
                    )
                    .unwrap(),
                &spatial,
            );
        }
        let offset_axial = Surface::plane(p(1, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        assert_eq!(
            cone.intersect_surface(&offset_axial).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
    }

    #[test]
    fn certified_atan2_retains_exact_axes_quadrants_and_symbolic_cancellation() {
        let quarter = (Real::pi() / r(4)).unwrap();
        let half = (Real::pi() / r(2)).unwrap();
        for (y, x, expected) in [
            (Real::zero(), Real::zero(), Real::zero()),
            (Real::zero(), Real::one(), Real::zero()),
            (Real::zero(), -Real::one(), Real::pi()),
            (Real::one(), Real::zero(), half.clone()),
            (-Real::one(), Real::zero(), -half),
            (Real::one(), Real::one(), quarter.clone()),
            (Real::one(), -Real::one(), Real::pi() - &quarter),
            (-Real::one(), -Real::one(), -Real::pi() + &quarter),
            (-Real::one(), Real::one(), -quarter),
        ] {
            assert_eq!(
                compare_reals(&certified_atan2(y, x).unwrap(), &expected).value(),
                Some(Ordering::Equal)
            );
        }

        let diagonal = (Real::one() / r(2).sqrt().unwrap()).unwrap();
        let symbolic_zero = r(2) * &diagonal * diagonal - Real::one();
        assert_eq!(
            compare_reals(
                &certified_atan2(Real::one(), symbolic_zero).unwrap(),
                &(Real::pi() / r(2)).unwrap(),
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn axial_plane_through_ring_torus_retains_two_exact_meridian_circles() {
        let torus = Surface::torus(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(3),
            r(1),
        )
        .unwrap();
        let plane = Surface::plane(Point3::origin(), Vector3::y(), Vector3::z()).unwrap();
        let SurfaceSurfaceIntersection::Curves(sections) = torus.intersect_surface(&plane).unwrap()
        else {
            panic!("an axial plane through a ring torus must retain two meridian circles");
        };
        assert_eq!(sections.len(), 2);
        assert_points_equal(&sections[0].curve().start().unwrap(), &p(0, -4, 0));
        assert_points_equal(&sections[1].curve().start().unwrap(), &p(0, 4, 0));

        for section in &sections {
            assert_eq!(section.curve().kind(), Curve3Kind::CircleArc);
            assert_eq!(
                section
                    .first_pcurve()
                    .materialize()
                    .unwrap()
                    .curve()
                    .family(),
                CurveFamily2::Line
            );
            assert_eq!(
                section
                    .second_pcurve()
                    .materialize()
                    .unwrap()
                    .curve()
                    .family(),
                CurveFamily2::CircularArc
            );
            for parameter in [Real::zero(), (Real::pi() / r(2)).unwrap(), Real::pi()] {
                let point = section.curve().point_at(&parameter).unwrap();
                assert_points_equal(
                    &torus
                        .point_at(&section.first_pcurve().point_at(&parameter).unwrap())
                        .unwrap(),
                    &point,
                );
                assert_points_equal(
                    &plane
                        .point_at(&section.second_pcurve().point_at(&parameter).unwrap())
                        .unwrap(),
                    &point,
                );
            }
        }

        let SurfaceSurfaceIntersection::Curves(swapped) = plane.intersect_surface(&torus).unwrap()
        else {
            panic!("operand reversal must retain both meridian circles");
        };
        assert_eq!(swapped.len(), 2);
        assert_eq!(
            swapped[0]
                .first_pcurve()
                .materialize()
                .unwrap()
                .curve()
                .family(),
            CurveFamily2::CircularArc
        );
        assert_eq!(
            swapped[0]
                .second_pcurve()
                .materialize()
                .unwrap()
                .curve()
                .family(),
            CurveFamily2::Line
        );

        let offset = Surface::plane(p(1, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        assert_eq!(
            torus.intersect_surface(&offset).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
        let separated = Surface::plane(p(5, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        assert!(matches!(
            torus.intersect_surface(&separated).unwrap(),
            SurfaceSurfaceIntersection::None
        ));
        let tangent = Surface::plane(p(4, 0, 0), Vector3::y(), Vector3::z()).unwrap();
        let SurfaceSurfaceIntersection::Point(point) = torus.intersect_surface(&tangent).unwrap()
        else {
            panic!("an outer-radius axial plane must retain the exact tangent point");
        };
        assert_points_equal(&point, &p(4, 0, 0));
        let oblique = Surface::plane(
            Point3::origin(),
            Vector3::x(),
            Vector3::from_xyz(Real::zero(), Real::one(), Real::one()),
        )
        .unwrap();
        assert_eq!(
            torus.intersect_surface(&oblique).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
    }

    #[test]
    fn parallel_cylinder_intersections_retain_exact_axial_lines() {
        let cylinder = Surface::cylinder(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(2),
        )
        .unwrap();
        let same_axis =
            Surface::cylinder(p(0, 0, 5), Vector3::x(), -Vector3::y(), -Vector3::z(), r(2))
                .unwrap();
        assert!(matches!(
            cylinder.intersect_surface(&same_axis).unwrap(),
            SurfaceSurfaceIntersection::Coincident
        ));
        let concentric_other_radius = Surface::cylinder(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(1),
        )
        .unwrap();
        assert!(matches!(
            cylinder
                .intersect_surface(&concentric_other_radius)
                .unwrap(),
            SurfaceSurfaceIntersection::None
        ));

        let secant =
            Surface::cylinder(p(3, 0, 0), Vector3::x(), Vector3::y(), Vector3::z(), r(2)).unwrap();
        let SurfaceSurfaceIntersection::Lines(lines) = cylinder.intersect_surface(&secant).unwrap()
        else {
            panic!("parallel secant cylinders must retain two axial lines");
        };
        let half_sqrt_seven = (r(7).sqrt().unwrap() / r(2)).unwrap();
        assert_points_equal(
            &lines[0].point,
            &Point3::new(q(3, 2), half_sqrt_seven.clone(), Real::zero()),
        );
        assert_points_equal(
            &lines[1].point,
            &Point3::new(q(3, 2), -half_sqrt_seven, Real::zero()),
        );

        let tangent =
            Surface::cylinder(p(4, 0, 0), Vector3::x(), Vector3::y(), Vector3::z(), r(2)).unwrap();
        let SurfaceSurfaceIntersection::Line(line) = cylinder.intersect_surface(&tangent).unwrap()
        else {
            panic!("externally tangent cylinders must retain one axial line");
        };
        assert_points_equal(&line.point, &p(2, 0, 0));
        let disjoint =
            Surface::cylinder(p(5, 0, 0), Vector3::x(), Vector3::y(), Vector3::z(), r(2)).unwrap();
        assert!(matches!(
            cylinder.intersect_surface(&disjoint).unwrap(),
            SurfaceSurfaceIntersection::None
        ));

        let skew = Surface::cylinder(
            Point3::origin(),
            Vector3::y(),
            Vector3::z(),
            Vector3::x(),
            r(2),
        )
        .unwrap();
        assert_eq!(
            cylinder.intersect_surface(&skew).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
    }

    #[test]
    fn coaxial_sphere_cylinder_intersections_retain_exact_circles() {
        let cylinder = Surface::cylinder(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(2),
        )
        .unwrap();
        let sphere = Surface::sphere(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(3),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curves(circles) =
            sphere.intersect_surface(&cylinder).unwrap()
        else {
            panic!("aligned coaxial carriers must retain two circles with exact pcurves");
        };
        let sqrt_five = r(5).sqrt().unwrap();
        assert_points_equal(
            &circles[0].curve().start().unwrap(),
            &Point3::new(r(2), Real::zero(), sqrt_five.clone()),
        );
        assert_points_equal(
            &circles[1].curve().start().unwrap(),
            &Point3::new(r(2), Real::zero(), -sqrt_five),
        );
        for circle in &circles {
            let parameter = Real::pi();
            let spatial = circle.curve().point_at(&parameter).unwrap();
            let sphere_point = sphere
                .point_at(&circle.first_pcurve().point_at(&parameter).unwrap())
                .unwrap();
            let cylinder_point = cylinder
                .point_at(&circle.second_pcurve().point_at(&parameter).unwrap())
                .unwrap();
            assert_points_equal(&sphere_point, &spatial);
            assert_points_equal(&cylinder_point, &spatial);
            assert!(circle.first_pcurve().materialize().is_ok());
            assert!(circle.second_pcurve().materialize().is_ok());
        }
        let SurfaceSurfaceIntersection::Curves(swapped) =
            cylinder.intersect_surface(&sphere).unwrap()
        else {
            panic!("operand reversal must retain the same two exact circles");
        };
        let parameter = Real::pi();
        assert_points_equal(
            &cylinder
                .point_at(&swapped[0].first_pcurve().point_at(&parameter).unwrap())
                .unwrap(),
            &swapped[0].curve().point_at(&parameter).unwrap(),
        );
        assert_points_equal(
            &sphere
                .point_at(&swapped[0].second_pcurve().point_at(&parameter).unwrap())
                .unwrap(),
            &swapped[0].curve().point_at(&parameter).unwrap(),
        );
        let rotated_parameters = Surface::sphere(
            Point3::origin(),
            Vector3::y(),
            -Vector3::x(),
            Vector3::z(),
            r(3),
        )
        .unwrap();
        assert!(matches!(
            cylinder.intersect_surface(&rotated_parameters).unwrap(),
            SurfaceSurfaceIntersection::Circles(_)
        ));

        let tangent = Surface::sphere(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(2),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curve(circle) =
            cylinder.intersect_surface(&tangent).unwrap()
        else {
            panic!("equal-radius aligned carriers must retain one tangent circle");
        };
        assert_points_equal(&circle.curve().start().unwrap(), &p(2, 0, 0));
        assert!(circle.first_pcurve().materialize().is_ok());
        assert!(circle.second_pcurve().materialize().is_ok());
        let contained = Surface::sphere(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(1),
        )
        .unwrap();
        assert!(matches!(
            cylinder.intersect_surface(&contained).unwrap(),
            SurfaceSurfaceIntersection::None
        ));
        let off_axis =
            Surface::sphere(p(1, 0, 0), Vector3::x(), Vector3::y(), Vector3::z(), r(3)).unwrap();
        assert_eq!(
            cylinder.intersect_surface(&off_axis).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
    }

    #[test]
    fn coaxial_sphere_cone_intersections_retain_native_slant_circles() {
        let semi_angle = q(3, 4).atan().unwrap();
        let cone = Surface::cone(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            semi_angle.clone(),
        )
        .unwrap();
        let centered = Surface::sphere(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(2),
        )
        .unwrap();
        let SurfaceSurfaceIntersection::Curve(circle) = centered.intersect_surface(&cone).unwrap()
        else {
            panic!("an apex-centered sphere must cut one exact cone latitude");
        };
        assert_points_equal(
            &circle.curve().start().unwrap(),
            &Point3::new(q(6, 5), Real::zero(), q(8, 5)),
        );
        for parameter in [Real::zero(), Real::pi()] {
            let spatial = circle.curve().point_at(&parameter).unwrap();
            assert_points_equal(
                &centered
                    .point_at(&circle.first_pcurve().point_at(&parameter).unwrap())
                    .unwrap(),
                &spatial,
            );
            assert_points_equal(
                &cone
                    .point_at(&circle.second_pcurve().point_at(&parameter).unwrap())
                    .unwrap(),
                &spatial,
            );
        }

        let two_circle_sphere =
            Surface::sphere(p(0, 0, 5), Vector3::x(), Vector3::y(), Vector3::z(), r(4)).unwrap();
        let SurfaceSurfaceIntersection::Curves(circles) =
            two_circle_sphere.intersect_surface(&cone).unwrap()
        else {
            panic!("a coaxial sphere spanning both cone sheets must retain two upper circles");
        };
        assert_eq!(circles.len(), 2);
        let parameter = q(1, 3);
        for circle in &circles {
            let spatial = circle.curve().point_at(&parameter).unwrap();
            assert_points_equal(
                &two_circle_sphere
                    .point_at(&circle.first_pcurve().point_at(&parameter).unwrap())
                    .unwrap(),
                &spatial,
            );
            assert_points_equal(
                &cone
                    .point_at(&circle.second_pcurve().point_at(&parameter).unwrap())
                    .unwrap(),
                &spatial,
            );
        }

        let tangent =
            Surface::sphere(p(0, 0, 5), Vector3::x(), Vector3::y(), Vector3::z(), r(3)).unwrap();
        let SurfaceSurfaceIntersection::Curve(tangent_circle) =
            tangent.intersect_surface(&cone).unwrap()
        else {
            panic!("zero discriminant must retain one tangent latitude circle");
        };
        assert_points_equal(
            &tangent_circle.curve().start().unwrap(),
            &Point3::new(q(12, 5), Real::zero(), q(16, 5)),
        );

        let separated =
            Surface::sphere(p(0, 0, 5), Vector3::x(), Vector3::y(), Vector3::z(), r(2)).unwrap();
        assert!(matches!(
            separated.intersect_surface(&cone).unwrap(),
            SurfaceSurfaceIntersection::None
        ));
        let apex_only =
            Surface::sphere(p(0, 0, -5), Vector3::x(), Vector3::y(), Vector3::z(), r(5)).unwrap();
        let SurfaceSurfaceIntersection::Point(point) = apex_only.intersect_surface(&cone).unwrap()
        else {
            panic!("the lower coaxial sphere must retain its isolated apex contact");
        };
        assert_points_equal(&point, &Point3::origin());

        let mixed_dimension =
            Surface::sphere(p(0, 0, 5), Vector3::x(), Vector3::y(), Vector3::z(), r(5)).unwrap();
        assert_eq!(
            mixed_dimension.intersect_surface(&cone).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
        let off_axis =
            Surface::sphere(p(1, 0, 0), Vector3::x(), Vector3::y(), Vector3::z(), r(2)).unwrap();
        assert_eq!(
            off_axis.intersect_surface(&cone).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );

        let rotated_parameters = Surface::sphere(
            Point3::origin(),
            Vector3::y(),
            -Vector3::x(),
            Vector3::z(),
            r(2),
        )
        .unwrap();
        assert!(matches!(
            cone.intersect_surface(&rotated_parameters).unwrap(),
            SurfaceSurfaceIntersection::Circle(_)
        ));
    }

    #[test]
    fn coaxial_cylinder_cone_intersection_retains_native_slant_circle() {
        let semi_angle = q(3, 4).atan().unwrap();
        let cone = Surface::cone(
            p(0, 0, 5),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            semi_angle,
        )
        .unwrap();
        let cylinder =
            Surface::cylinder(p(0, 0, 1), Vector3::x(), Vector3::y(), Vector3::z(), r(3)).unwrap();
        let SurfaceSurfaceIntersection::Curve(circle) = cylinder.intersect_surface(&cone).unwrap()
        else {
            panic!("aligned coaxial cylinder/cone carriers must retain both pcurves");
        };
        assert_points_equal(
            &circle.curve().start().unwrap(),
            &Point3::new(r(3), Real::zero(), r(9)),
        );
        for parameter in [Real::zero(), Real::pi()] {
            let spatial = circle.curve().point_at(&parameter).unwrap();
            assert_points_equal(
                &cylinder
                    .point_at(&circle.first_pcurve().point_at(&parameter).unwrap())
                    .unwrap(),
                &spatial,
            );
            assert_points_equal(
                &cone
                    .point_at(&circle.second_pcurve().point_at(&parameter).unwrap())
                    .unwrap(),
                &spatial,
            );
        }
        let SurfaceSurfaceIntersection::Curve(swapped) = cone.intersect_surface(&cylinder).unwrap()
        else {
            panic!("operand reversal must retain the same native circle");
        };
        let parameter = q(2, 3);
        assert_points_equal(
            &cone
                .point_at(&swapped.first_pcurve().point_at(&parameter).unwrap())
                .unwrap(),
            &swapped.curve().point_at(&parameter).unwrap(),
        );

        let rotated_parameters =
            Surface::cylinder(p(0, 0, 1), Vector3::y(), -Vector3::x(), Vector3::z(), r(3)).unwrap();
        assert!(matches!(
            cone.intersect_surface(&rotated_parameters).unwrap(),
            SurfaceSurfaceIntersection::Circle(_)
        ));
        let off_axis =
            Surface::cylinder(p(1, 0, 1), Vector3::x(), Vector3::y(), Vector3::z(), r(3)).unwrap();
        assert_eq!(
            cone.intersect_surface(&off_axis).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
        let skew =
            Surface::cylinder(p(0, 0, 1), Vector3::y(), Vector3::z(), Vector3::x(), r(3)).unwrap();
        assert_eq!(
            cone.intersect_surface(&skew).unwrap_err(),
            GeometryError::UnsupportedIntersection
        );
    }

    #[test]
    fn plane_sphere_and_sphere_sphere_intersections_retain_exact_circles() {
        let sphere = Surface::sphere(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(2),
        )
        .unwrap();
        let equator = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        let SurfaceSurfaceIntersection::Curve(circle) = equator.intersect_surface(&sphere).unwrap()
        else {
            panic!("equatorial plane must retain a circle with both exact pcurves");
        };
        assert_eq!(circle.curve().kind(), Curve3Kind::CircleArc);
        assert_points_equal(&circle.curve().start().unwrap(), &p(2, 0, 0));
        assert_eq!(
            circle
                .first_pcurve()
                .materialize()
                .unwrap()
                .curve()
                .family(),
            CurveFamily2::CircularArc
        );
        assert_eq!(
            circle
                .second_pcurve()
                .materialize()
                .unwrap()
                .curve()
                .family(),
            CurveFamily2::Line
        );
        let SurfaceSurfaceIntersection::Curve(swapped) =
            sphere.intersect_surface(&equator).unwrap()
        else {
            panic!("operand reversal must retain the same exact sphere latitude");
        };
        assert_eq!(
            swapped
                .first_pcurve()
                .materialize()
                .unwrap()
                .curve()
                .family(),
            CurveFamily2::Line
        );
        assert_eq!(
            swapped
                .second_pcurve()
                .materialize()
                .unwrap()
                .curve()
                .family(),
            CurveFamily2::CircularArc
        );

        let tangent_plane = Surface::plane(p(0, 0, 2), Vector3::x(), Vector3::y()).unwrap();
        let SurfaceSurfaceIntersection::Point(point) =
            sphere.intersect_surface(&tangent_plane).unwrap()
        else {
            panic!("tangent plane must produce one point");
        };
        assert_points_equal(&point, &p(0, 0, 2));
        let disjoint_plane = Surface::plane(p(0, 0, 3), Vector3::x(), Vector3::y()).unwrap();
        assert!(matches!(
            sphere.intersect_surface(&disjoint_plane).unwrap(),
            SurfaceSurfaceIntersection::None
        ));

        let second =
            Surface::sphere(p(2, 0, 0), Vector3::x(), Vector3::y(), Vector3::z(), r(2)).unwrap();
        let SurfaceSurfaceIntersection::Circle(circle) = sphere.intersect_surface(&second).unwrap()
        else {
            panic!("overlapping spheres must produce a circle");
        };
        let point = circle.start().unwrap();
        assert_eq!(
            compare_reals(&point.x, &r(1)).value(),
            Some(Ordering::Equal)
        );
        let first_radius_squared = Vector3::from(point.clone()).norm_squared();
        let second_radius_squared = (&point - &p(2, 0, 0)).norm_squared();
        assert_eq!(
            compare_reals(&first_radius_squared, &r(4)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&second_radius_squared, &r(4)).value(),
            Some(Ordering::Equal)
        );

        let tangent =
            Surface::sphere(p(4, 0, 0), Vector3::x(), Vector3::y(), Vector3::z(), r(2)).unwrap();
        let SurfaceSurfaceIntersection::Point(point) = sphere.intersect_surface(&tangent).unwrap()
        else {
            panic!("tangent spheres must produce one point");
        };
        assert_points_equal(&point, &p(2, 0, 0));
        assert!(matches!(
            sphere.intersect_surface(&sphere).unwrap(),
            SurfaceSurfaceIntersection::Coincident
        ));
    }

    #[test]
    fn analytic_surfaces_use_exact_domains_evaluation_and_partials() {
        let cylinder =
            Surface::cylinder(p(1, 2, 3), Vector3::x(), Vector3::y(), Vector3::z(), r(2)).unwrap();
        assert_eq!(cylinder.kind(), SurfaceKind::Cylinder);
        assert_points_equal(
            &cylinder.point_at(&Point2::new(r(0), r(5))).unwrap(),
            &p(3, 2, 8),
        );
        let partials = cylinder.partials_at(&Point2::new(r(0), r(5))).unwrap();
        assert_points_equal(&Point3::from(partials.u().clone()), &p(0, 2, 0));
        assert_points_equal(&Point3::from(partials.v().clone()), &p(0, 0, 1));

        let sphere = Surface::sphere(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(2),
        )
        .unwrap();
        assert_points_equal(
            &sphere.point_at(&Point2::new(r(0), r(0))).unwrap(),
            &p(2, 0, 0),
        );
        let partials = sphere.partials_at(&Point2::new(r(0), r(0))).unwrap();
        assert_points_equal(&Point3::from(partials.u().clone()), &p(0, 2, 0));
        assert_points_equal(&Point3::from(partials.v().clone()), &p(0, 0, 2));
        assert_points_equal(
            &Point3::from(sphere.normal_at(&Point2::new(r(0), r(0))).unwrap()),
            &p(1, 0, 0),
        );
        assert_eq!(
            sphere
                .normal_at(&Point2::new(r(0), (Real::pi() / r(2)).unwrap()))
                .unwrap_err(),
            GeometryError::SingularSurfaceParameter
        );
        assert_eq!(
            sphere.point_at(&Point2::new(r(0), Real::pi())).unwrap_err(),
            GeometryError::SurfaceParameterOutsideDomain
        );

        let cone = Surface::cone(
            p(1, 2, 3),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            (Real::pi() / r(4)).unwrap(),
        )
        .unwrap();
        assert_points_equal(
            &cone.point_at(&Point2::new(r(7), r(0))).unwrap(),
            &p(1, 2, 3),
        );
        assert_eq!(
            cone.normal_at(&Point2::new(r(7), r(0))).unwrap_err(),
            GeometryError::SingularSurfaceParameter
        );
        assert_eq!(
            cone.point_at(&Point2::new(r(0), r(-1))).unwrap_err(),
            GeometryError::SurfaceParameterOutsideDomain
        );

        let torus = Surface::torus(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            r(4),
            r(1),
        )
        .unwrap();
        assert_points_equal(
            &torus.point_at(&Point2::new(r(0), r(0))).unwrap(),
            &p(5, 0, 0),
        );
    }

    #[test]
    fn analytic_surface_constructors_reject_invalid_exact_geometry() {
        assert_eq!(
            Surface::cylinder(
                Point3::origin(),
                Vector3::x(),
                Vector3::x(),
                Vector3::z(),
                r(1),
            )
            .unwrap_err(),
            GeometryError::InvalidSurfaceFrame
        );
        assert_eq!(
            Surface::sphere(
                Point3::origin(),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                r(0),
            )
            .unwrap_err(),
            GeometryError::InvalidRadius
        );
        assert_eq!(
            Surface::torus(
                Point3::origin(),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                r(2),
                r(2),
            )
            .unwrap_err(),
            GeometryError::InvalidTorusRadii
        );
    }

    #[test]
    fn named_circle_and_ellipse_arcs_retain_exact_angle_semantics() {
        let half_pi = (Real::pi() / r(2)).unwrap();
        let circle = Curve3::circle_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            r(2),
            Real::zero(),
            half_pi.clone(),
        )
        .unwrap();
        assert_eq!(circle.kind(), Curve3Kind::CircleArc);
        assert_points_equal(&circle.start().unwrap(), &p(2, 0, 0));
        assert_points_equal(&circle.end().unwrap(), &p(0, 2, 0));
        assert_points_equal(
            &Point3::from(
                circle
                    .derivative_at(&Real::zero(), 1)
                    .unwrap()
                    .vector()
                    .clone(),
            ),
            &p(0, 2, 0),
        );
        let reversed = circle.reversed().unwrap();
        assert_points_equal(&reversed.start().unwrap(), &circle.end().unwrap());
        assert_points_equal(&reversed.end().unwrap(), &circle.start().unwrap());
        let split = (half_pi.clone() / r(2)).unwrap();
        let (left, right) = circle.split_at(&split).unwrap();
        assert_points_equal(&left.end().unwrap(), &right.start().unwrap());

        let ellipse = Curve3::ellipse_arc(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            r(3),
            r(2),
            Real::zero(),
            half_pi,
        )
        .unwrap();
        assert_eq!(ellipse.kind(), Curve3Kind::EllipseArc);
        assert_points_equal(&ellipse.start().unwrap(), &p(3, 0, 0));
        assert_points_equal(&ellipse.end().unwrap(), &p(0, 2, 0));

        assert_eq!(
            Curve3::circle_arc(
                Point3::origin(),
                Vector3::x(),
                Vector3::y(),
                r(1),
                Real::zero(),
                r(3) * Real::pi(),
            )
            .unwrap_err(),
            GeometryError::InvalidArcSweep
        );
    }
}
