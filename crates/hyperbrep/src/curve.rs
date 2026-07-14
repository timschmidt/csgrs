//! Exact retained spatial curves owned by HyperBREP.
//!
//! Hypercurve owns planar geometry. Model-space curves used by BREP edges live
//! here so a 3D carrier cannot leak into the planar API. Rational Bezier and
//! NURBS evaluation stay homogeneous until the final affine projection, and
//! clones share the weighted control net.

use std::cell::OnceCell;
use std::cmp::Ordering;
use std::fmt;
use std::rc::Rc;

use hyperlimit::Point3;
use hyperreal::Real;

/// Stable application source for a retained spatial curve.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct BrepCurveSource3 {
    id: u64,
    version: u64,
}

/// Spatial curve family owned by HyperBREP.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum BrepCurveFamily3 {
    /// Finite line segment.
    Line,
    /// Arbitrary-degree rational Bezier curve.
    RationalBezier,
    /// Rational B-spline/NURBS curve.
    Nurbs,
}

/// Spatial curve operation attached to a typed failure.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum BrepCurveOperation3 {
    /// Carrier construction and invariant validation.
    Construction,
    /// Exact point evaluation.
    Evaluation,
}

/// Specific reason an exact spatial curve operation failed.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BrepCurveErrorKind3 {
    /// At least two control points are required.
    TooFewControlPoints,
    /// Control-point and weight counts differ.
    WeightCountMismatch,
    /// The NURBS degree is zero or is not below the control-point count.
    InvalidDegree,
    /// The knot count is not `control_count + degree + 1`.
    InvalidKnotCount,
    /// Exact knot ordering could not be certified as nondecreasing.
    InvalidKnotOrder,
    /// The active knot domain is empty or could not be ordered.
    InvalidParameterDomain,
    /// The requested parameter lies outside the exact public domain.
    ParameterOutsideDomain,
    /// Homogeneous projection or de Boor interpolation required division by a
    /// zero or uncertified denominator.
    ProjectiveDivision,
}

/// Contextual exact spatial curve failure.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepCurveError3 {
    operation: BrepCurveOperation3,
    family: BrepCurveFamily3,
    source: Option<BrepCurveSource3>,
    kind: BrepCurveErrorKind3,
}

/// Result of an exact spatial curve operation.
pub type BrepCurveResult3<T> = Result<T, BrepCurveError3>;

/// Exact public parameter domain of a retained spatial curve.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepCurveParameterDomain3 {
    start: Real,
    end: Real,
}

/// Exact finite 3D line segment.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepLineSegment3 {
    start: Point3,
    end: Point3,
}

/// Clone-shared arbitrary-degree rational Bezier curve in model space.
#[derive(Clone, Debug)]
pub struct BrepRationalBezier3 {
    data: Rc<BrepRationalBezierData3>,
}

#[derive(Debug)]
struct BrepRationalBezierData3 {
    control_points: Vec<Point3>,
    weights: Vec<Real>,
    homogeneous_controls: OnceCell<Vec<HomogeneousPoint3>>,
}

/// Clone-shared exact NURBS curve in model space.
#[derive(Clone, Debug)]
pub struct BrepNurbsCurve3 {
    data: Rc<BrepNurbsData3>,
}

#[derive(Debug)]
struct BrepNurbsData3 {
    degree: usize,
    control_points: Vec<Point3>,
    weights: Vec<Real>,
    knots: Vec<Real>,
    domain: BrepCurveParameterDomain3,
    homogeneous_controls: OnceCell<Vec<HomogeneousPoint3>>,
}

/// Geometry carried by a top-level exact spatial curve.
#[derive(Clone, Debug)]
pub enum BrepCurveGeometry3 {
    /// Finite line segment.
    Line(Box<BrepLineSegment3>),
    /// Arbitrary-degree rational Bezier curve.
    RationalBezier(BrepRationalBezier3),
    /// Rational B-spline/NURBS curve.
    Nurbs(BrepNurbsCurve3),
}

/// Top-level exact spatial curve with stable provenance.
#[derive(Clone, Debug)]
pub struct BrepCurve3 {
    geometry: BrepCurveGeometry3,
    source: Option<BrepCurveSource3>,
}

#[derive(Clone, Debug, PartialEq)]
struct HomogeneousPoint3 {
    x: Real,
    y: Real,
    z: Real,
    w: Real,
}

impl BrepCurveSource3 {
    /// Constructs a source at version zero.
    pub const fn new(id: u64) -> Self {
        Self::with_version(id, 0)
    }

    /// Constructs a source with an explicit version.
    pub const fn with_version(id: u64, version: u64) -> Self {
        Self { id, version }
    }

    /// Returns the opaque source id.
    pub const fn id(self) -> u64 {
        self.id
    }

    /// Returns the source version captured by retained facts.
    pub const fn version(self) -> u64 {
        self.version
    }
}

impl BrepCurveError3 {
    fn new(
        operation: BrepCurveOperation3,
        family: BrepCurveFamily3,
        source: Option<BrepCurveSource3>,
        kind: BrepCurveErrorKind3,
    ) -> Self {
        Self {
            operation,
            family,
            source,
            kind,
        }
    }

    /// Returns the failed operation.
    pub const fn operation(&self) -> BrepCurveOperation3 {
        self.operation
    }

    /// Returns the spatial curve family involved in the failure.
    pub const fn family(&self) -> BrepCurveFamily3 {
        self.family
    }

    /// Returns stable source provenance when supplied.
    pub const fn source_id(&self) -> Option<BrepCurveSource3> {
        self.source
    }

    /// Returns the specific failure reason.
    pub const fn kind(&self) -> &BrepCurveErrorKind3 {
        &self.kind
    }
}

impl fmt::Display for BrepCurveError3 {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "spatial {:?} {:?} failed: {:?}",
            self.family, self.operation, self.kind
        )
    }
}

impl std::error::Error for BrepCurveError3 {}

impl BrepCurveParameterDomain3 {
    /// Returns the inclusive domain start.
    pub const fn start(&self) -> &Real {
        &self.start
    }

    /// Returns the inclusive domain end.
    pub const fn end(&self) -> &Real {
        &self.end
    }
}

impl BrepLineSegment3 {
    /// Constructs a finite exact line segment.
    pub const fn new(start: Point3, end: Point3) -> Self {
        Self { start, end }
    }

    /// Returns the exact start point.
    pub const fn start(&self) -> &Point3 {
        &self.start
    }

    /// Returns the exact end point.
    pub const fn end(&self) -> &Point3 {
        &self.end
    }

    fn point_at(&self, parameter: &Real) -> Point3 {
        self.start.lerp(&self.end, parameter)
    }
}

impl BrepRationalBezier3 {
    /// Constructs an exact arbitrary-degree rational Bezier curve.
    pub fn try_new(control_points: Vec<Point3>, weights: Vec<Real>) -> BrepCurveResult3<Self> {
        validate_control_net(&control_points, &weights, BrepCurveFamily3::RationalBezier)?;
        Ok(Self {
            data: Rc::new(BrepRationalBezierData3 {
                control_points,
                weights,
                homogeneous_controls: OnceCell::new(),
            }),
        })
    }

    /// Returns the polynomial degree.
    pub fn degree(&self) -> usize {
        self.data.control_points.len() - 1
    }

    /// Returns exact affine control points.
    pub fn control_points(&self) -> &[Point3] {
        &self.data.control_points
    }

    /// Returns exact projective weights.
    pub fn weights(&self) -> &[Real] {
        &self.data.weights
    }

    /// Returns whether weighted controls have already been retained.
    pub fn is_homogeneous_control_net_cached(&self) -> bool {
        self.data.homogeneous_controls.get().is_some()
    }

    /// Evaluates a point exactly over `[0, 1]` with homogeneous de Casteljau.
    pub fn point_at(&self, parameter: &Real) -> BrepCurveResult3<Point3> {
        validate_unit_parameter(parameter, BrepCurveFamily3::RationalBezier, None)?;
        evaluate_homogeneous_bezier(self.homogeneous_controls(), parameter).ok_or_else(|| {
            BrepCurveError3::new(
                BrepCurveOperation3::Evaluation,
                BrepCurveFamily3::RationalBezier,
                None,
                BrepCurveErrorKind3::ProjectiveDivision,
            )
        })
    }

    fn homogeneous_controls(&self) -> &[HomogeneousPoint3] {
        self.data
            .homogeneous_controls
            .get_or_init(|| weighted_controls(self.control_points(), self.weights()))
    }
}

impl PartialEq for BrepRationalBezier3 {
    fn eq(&self, other: &Self) -> bool {
        self.control_points() == other.control_points() && self.weights() == other.weights()
    }
}

impl BrepNurbsCurve3 {
    /// Constructs an exact finite-domain NURBS curve.
    pub fn try_new(
        degree: usize,
        control_points: Vec<Point3>,
        weights: Vec<Real>,
        knots: Vec<Real>,
    ) -> BrepCurveResult3<Self> {
        validate_control_net(&control_points, &weights, BrepCurveFamily3::Nurbs)?;
        if degree == 0 || degree >= control_points.len() {
            return Err(construction_error(
                BrepCurveFamily3::Nurbs,
                BrepCurveErrorKind3::InvalidDegree,
            ));
        }
        if knots.len() != control_points.len() + degree + 1 {
            return Err(construction_error(
                BrepCurveFamily3::Nurbs,
                BrepCurveErrorKind3::InvalidKnotCount,
            ));
        }
        for adjacent in knots.windows(2) {
            if !matches!(
                compare_reals(&adjacent[0], &adjacent[1]),
                Some(Ordering::Less | Ordering::Equal)
            ) {
                return Err(construction_error(
                    BrepCurveFamily3::Nurbs,
                    BrepCurveErrorKind3::InvalidKnotOrder,
                ));
            }
        }
        let start = knots[degree].clone();
        let end = knots[control_points.len()].clone();
        if compare_reals(&start, &end) != Some(Ordering::Less) {
            return Err(construction_error(
                BrepCurveFamily3::Nurbs,
                BrepCurveErrorKind3::InvalidParameterDomain,
            ));
        }
        Ok(Self {
            data: Rc::new(BrepNurbsData3 {
                degree,
                control_points,
                weights,
                knots,
                domain: BrepCurveParameterDomain3 { start, end },
                homogeneous_controls: OnceCell::new(),
            }),
        })
    }

    /// Returns the spline degree.
    pub fn degree(&self) -> usize {
        self.data.degree
    }

    /// Returns exact affine control points.
    pub fn control_points(&self) -> &[Point3] {
        &self.data.control_points
    }

    /// Returns exact projective weights.
    pub fn weights(&self) -> &[Real] {
        &self.data.weights
    }

    /// Returns the authored knot vector.
    pub fn knots(&self) -> &[Real] {
        &self.data.knots
    }

    /// Returns the exact active knot domain.
    pub fn parameter_domain(&self) -> &BrepCurveParameterDomain3 {
        &self.data.domain
    }

    /// Returns whether weighted controls have already been retained.
    pub fn is_homogeneous_control_net_cached(&self) -> bool {
        self.data.homogeneous_controls.get().is_some()
    }

    /// Evaluates an exact model-space point with homogeneous de Boor.
    pub fn point_at(&self, parameter: &Real) -> BrepCurveResult3<Point3> {
        validate_parameter(
            parameter,
            self.parameter_domain(),
            BrepCurveFamily3::Nurbs,
            None,
        )?;
        let span = self.find_span(parameter).ok_or_else(|| {
            BrepCurveError3::new(
                BrepCurveOperation3::Evaluation,
                BrepCurveFamily3::Nurbs,
                None,
                BrepCurveErrorKind3::InvalidParameterDomain,
            )
        })?;
        evaluate_homogeneous_de_boor(
            self.homogeneous_controls(),
            self.knots(),
            self.degree(),
            span,
            parameter,
        )
        .ok_or_else(|| {
            BrepCurveError3::new(
                BrepCurveOperation3::Evaluation,
                BrepCurveFamily3::Nurbs,
                None,
                BrepCurveErrorKind3::ProjectiveDivision,
            )
        })
    }

    fn homogeneous_controls(&self) -> &[HomogeneousPoint3] {
        self.data
            .homogeneous_controls
            .get_or_init(|| weighted_controls(self.control_points(), self.weights()))
    }

    fn find_span(&self, parameter: &Real) -> Option<usize> {
        let control_count = self.control_points().len();
        if compare_reals(parameter, self.parameter_domain().end()) == Some(Ordering::Equal) {
            return Some(control_count - 1);
        }
        (self.degree()..control_count).find(|&span| {
            matches!(
                compare_reals(&self.knots()[span], parameter),
                Some(Ordering::Less | Ordering::Equal)
            ) && compare_reals(parameter, &self.knots()[span + 1]) == Some(Ordering::Less)
        })
    }
}

impl PartialEq for BrepNurbsCurve3 {
    fn eq(&self, other: &Self) -> bool {
        self.degree() == other.degree()
            && self.control_points() == other.control_points()
            && self.weights() == other.weights()
            && self.knots() == other.knots()
    }
}

impl BrepCurveGeometry3 {
    /// Returns the spatial family.
    pub const fn family(&self) -> BrepCurveFamily3 {
        match self {
            Self::Line(_) => BrepCurveFamily3::Line,
            Self::RationalBezier(_) => BrepCurveFamily3::RationalBezier,
            Self::Nurbs(_) => BrepCurveFamily3::Nurbs,
        }
    }
}

impl PartialEq for BrepCurveGeometry3 {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Line(first), Self::Line(second)) => first == second,
            (Self::RationalBezier(first), Self::RationalBezier(second)) => first == second,
            (Self::Nurbs(first), Self::Nurbs(second)) => first == second,
            _ => false,
        }
    }
}

impl BrepCurve3 {
    /// Wraps spatial geometry with optional stable source/version provenance.
    pub const fn new(geometry: BrepCurveGeometry3, source: Option<BrepCurveSource3>) -> Self {
        Self { geometry, source }
    }

    /// Returns retained model-space geometry.
    pub const fn geometry(&self) -> &BrepCurveGeometry3 {
        &self.geometry
    }

    /// Returns the spatial family.
    pub const fn family(&self) -> BrepCurveFamily3 {
        self.geometry.family()
    }

    /// Returns stable source/version provenance when supplied.
    pub const fn source(&self) -> Option<BrepCurveSource3> {
        self.source
    }

    /// Returns the exact public parameter domain.
    pub fn parameter_domain(&self) -> BrepCurveParameterDomain3 {
        match self.geometry() {
            BrepCurveGeometry3::Line(_) | BrepCurveGeometry3::RationalBezier(_) => {
                BrepCurveParameterDomain3 {
                    start: Real::zero(),
                    end: Real::one(),
                }
            }
            BrepCurveGeometry3::Nurbs(curve) => curve.parameter_domain().clone(),
        }
    }

    /// Evaluates an exact model-space point without demoting coordinates.
    pub fn point_at(&self, parameter: &Real) -> BrepCurveResult3<Point3> {
        let result = match self.geometry() {
            BrepCurveGeometry3::Line(line) => {
                validate_unit_parameter(parameter, BrepCurveFamily3::Line, self.source)?;
                Ok(line.point_at(parameter))
            }
            BrepCurveGeometry3::RationalBezier(curve) => curve.point_at(parameter),
            BrepCurveGeometry3::Nurbs(curve) => curve.point_at(parameter),
        };
        result.map_err(|mut error| {
            error.source = self.source;
            error
        })
    }
}

impl PartialEq for BrepCurve3 {
    fn eq(&self, other: &Self) -> bool {
        self.geometry == other.geometry && self.source == other.source
    }
}

impl From<BrepLineSegment3> for BrepCurve3 {
    fn from(value: BrepLineSegment3) -> Self {
        Self::new(BrepCurveGeometry3::Line(Box::new(value)), None)
    }
}

impl From<BrepRationalBezier3> for BrepCurve3 {
    fn from(value: BrepRationalBezier3) -> Self {
        Self::new(BrepCurveGeometry3::RationalBezier(value), None)
    }
}

impl From<BrepNurbsCurve3> for BrepCurve3 {
    fn from(value: BrepNurbsCurve3) -> Self {
        Self::new(BrepCurveGeometry3::Nurbs(value), None)
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

    fn project(&self) -> Option<Point3> {
        Some(Point3::new(
            (&self.x / &self.w).ok()?,
            (&self.y / &self.w).ok()?,
            (&self.z / &self.w).ok()?,
        ))
    }
}

fn construction_error(family: BrepCurveFamily3, kind: BrepCurveErrorKind3) -> BrepCurveError3 {
    BrepCurveError3::new(BrepCurveOperation3::Construction, family, None, kind)
}

fn validate_control_net(
    control_points: &[Point3],
    weights: &[Real],
    family: BrepCurveFamily3,
) -> BrepCurveResult3<()> {
    if control_points.len() < 2 {
        return Err(construction_error(
            family,
            BrepCurveErrorKind3::TooFewControlPoints,
        ));
    }
    if control_points.len() != weights.len() {
        return Err(construction_error(
            family,
            BrepCurveErrorKind3::WeightCountMismatch,
        ));
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

fn evaluate_homogeneous_bezier(controls: &[HomogeneousPoint3], parameter: &Real) -> Option<Point3> {
    let mut level = controls.to_vec();
    while level.len() > 1 {
        level = level
            .windows(2)
            .map(|pair| pair[0].lerp(&pair[1], parameter))
            .collect();
    }
    level.first()?.project()
}

fn evaluate_homogeneous_de_boor(
    controls: &[HomogeneousPoint3],
    knots: &[Real],
    degree: usize,
    span: usize,
    parameter: &Real,
) -> Option<Point3> {
    let mut level = controls[(span - degree)..=span].to_vec();
    for stage in 1..=degree {
        for local in (stage..=degree).rev() {
            let knot_index = span - degree + local;
            let denominator = &knots[knot_index + degree - stage + 1] - &knots[knot_index];
            let alpha = ((parameter - &knots[knot_index]) / denominator).ok()?;
            level[local] = level[local - 1].lerp(&level[local], &alpha);
        }
    }
    level[degree].project()
}

fn validate_unit_parameter(
    parameter: &Real,
    family: BrepCurveFamily3,
    source: Option<BrepCurveSource3>,
) -> BrepCurveResult3<()> {
    validate_parameter(
        parameter,
        &BrepCurveParameterDomain3 {
            start: Real::zero(),
            end: Real::one(),
        },
        family,
        source,
    )
}

fn validate_parameter(
    parameter: &Real,
    domain: &BrepCurveParameterDomain3,
    family: BrepCurveFamily3,
    source: Option<BrepCurveSource3>,
) -> BrepCurveResult3<()> {
    let after_start = matches!(
        compare_reals(parameter, domain.start()),
        Some(Ordering::Equal | Ordering::Greater)
    );
    let before_end = matches!(
        compare_reals(parameter, domain.end()),
        Some(Ordering::Equal | Ordering::Less)
    );
    if after_start && before_end {
        Ok(())
    } else {
        Err(BrepCurveError3::new(
            BrepCurveOperation3::Evaluation,
            family,
            source,
            BrepCurveErrorKind3::ParameterOutsideDomain,
        ))
    }
}

fn compare_reals(first: &Real, second: &Real) -> Option<Ordering> {
    first.partial_cmp(second)
}
