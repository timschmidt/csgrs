//! Errors shared by exact geometry and validated model construction.

use std::fmt;

use hyperlimit::{Escalation, RefinementNeed};

/// Failure while constructing or evaluating exact geometry.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GeometryError {
    /// A closed parameter interval was empty or reversed.
    InvalidParameterDomain,
    /// A requested parameter lies outside the curve's exact domain.
    ParameterOutsideDomain,
    /// A curve has too few control points.
    TooFewControlPoints,
    /// Control-point and weight counts differ.
    WeightCountMismatch,
    /// A tensor-product surface control net is empty, ragged, or too small.
    InvalidControlNetShape,
    /// A tensor-product surface control net and weight net have different shapes.
    SurfaceWeightShapeMismatch,
    /// A rational curve weight was not certified strictly positive.
    InvalidWeight,
    /// A NURBS degree is zero or not smaller than the control-point count.
    InvalidDegree,
    /// A NURBS knot vector has the wrong length.
    InvalidKnotCount,
    /// A NURBS knot vector is not certified nondecreasing.
    InvalidKnotOrder,
    /// A finite non-periodic NURBS knot vector is not clamped at both ends.
    UnclampedNurbs,
    /// A NURBS knot multiplicity exceeds the supported degree contract.
    InvalidKnotMultiplicity,
    /// A projective evaluation required division by an uncertified denominator.
    ProjectiveDivision,
    /// An exact elementary function rejected a value or exhausted its budget.
    ElementaryFunction,
    /// A model transform was not certified affine.
    NonAffineTransform,
    /// An affine model transform was not certified invertible.
    SingularTransform,
    /// A geometry family cannot yet be retained by the requested transform.
    UnsupportedTransform,
    /// Exact homogeneous point transformation failed.
    TransformFailure,
    /// A line's endpoints denote the same mathematical point.
    DegenerateLine,
    /// Circular or elliptic arc angle bounds do not define a supported sweep.
    InvalidArcSweep,
    /// Ellipse radii are not certified strictly positive.
    InvalidEllipseRadii,
    /// A plane basis is linearly dependent.
    DegeneratePlaneBasis,
    /// An extrusion direction is not certified nonzero.
    DegenerateExtrusionDirection,
    /// A revolution axis is not certified to be a unit vector.
    InvalidRevolutionAxis,
    /// An analytic surface frame is not certified orthonormal.
    InvalidSurfaceFrame,
    /// An analytic surface radius is not certified strictly positive.
    InvalidRadius,
    /// A cone semi-angle is not certified inside `(0, pi/2)`.
    InvalidConeAngle,
    /// Torus radii do not define a positive non-self-intersecting ring torus.
    InvalidTorusRadii,
    /// A surface parameter lies outside its canonical nonperiodic domain.
    SurfaceParameterOutsideDomain,
    /// Surface first partials are linearly dependent at this parameter.
    SingularSurfaceParameter,
    /// The requested derivative is not implemented for this curve family.
    UnsupportedDerivative,
    /// Curve derivatives use positive orders; order zero denotes a point.
    InvalidDerivativeOrder,
    /// The requested subdivision is not implemented for this curve family.
    UnsupportedSubdivision,
    /// A subdivision parameter lies at a curve-domain boundary.
    SplitAtBoundary,
    /// Exact inverse parameter location is not implemented for this family.
    UnsupportedParameterLocation,
    /// A certified location exists but is not representable by `Real`.
    UnrepresentableParameter,
    /// The requested intersection family combination is unsupported.
    UnsupportedIntersection,
    /// Exact measurement is not implemented for this validated geometry family.
    UnsupportedMeasurement,
    /// The pcurve family cannot yet form an authoritative face contour.
    UnsupportedPcurveContour,
    /// A point is not on the requested finite curve.
    PointNotOnCurve,
    /// Hypercurve rejected or could not certify a planar curve operation.
    PlanarCurve(hypercurve::ExactCurveError),
    /// Hypercurve rejected planar curve construction.
    PlanarCurveConstruction(hypercurve::CurveError),
    /// Hypercurve could not certify an exact planar classification.
    PlanarClassificationUnresolved(hypercurve::UncertaintyReason),
    /// An exact decision was not certified by the active predicate pipeline.
    PredicateUnresolved {
        /// Additional predicate capability that was requested.
        needed: RefinementNeed,
        /// Last escalation stage reached.
        stage: Escalation,
    },
}

impl fmt::Display for GeometryError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameterDomain => formatter.write_str("invalid parameter domain"),
            Self::ParameterOutsideDomain => {
                formatter.write_str("parameter lies outside the exact curve domain")
            }
            Self::TooFewControlPoints => formatter.write_str("too few control points"),
            Self::WeightCountMismatch => {
                formatter.write_str("control-point and weight counts differ")
            }
            Self::InvalidControlNetShape => {
                formatter.write_str("invalid tensor-product surface control-net shape")
            }
            Self::SurfaceWeightShapeMismatch => {
                formatter.write_str("surface control-point and weight nets have different shapes")
            }
            Self::InvalidWeight => {
                formatter.write_str("rational curve weights must be certified positive")
            }
            Self::InvalidDegree => formatter.write_str("invalid NURBS degree"),
            Self::InvalidKnotCount => formatter.write_str("invalid NURBS knot count"),
            Self::InvalidKnotOrder => {
                formatter.write_str("NURBS knots are not certified nondecreasing")
            }
            Self::UnclampedNurbs => formatter.write_str("NURBS knot vector is not clamped"),
            Self::InvalidKnotMultiplicity => formatter.write_str("invalid NURBS knot multiplicity"),
            Self::ProjectiveDivision => {
                formatter.write_str("projective evaluation denominator is not certified nonzero")
            }
            Self::ElementaryFunction => {
                formatter.write_str("exact elementary function evaluation failed")
            }
            Self::NonAffineTransform => formatter.write_str("model transform is not affine"),
            Self::SingularTransform => formatter.write_str("model transform is singular"),
            Self::UnsupportedTransform => formatter.write_str("geometry transform is unsupported"),
            Self::TransformFailure => formatter.write_str("exact point transformation failed"),
            Self::DegenerateLine => formatter.write_str("line endpoints are mathematically equal"),
            Self::InvalidArcSweep => formatter.write_str("invalid arc sweep"),
            Self::InvalidEllipseRadii => formatter.write_str("invalid ellipse radii"),
            Self::DegeneratePlaneBasis => {
                formatter.write_str("plane parameter directions are linearly dependent")
            }
            Self::DegenerateExtrusionDirection => {
                formatter.write_str("extrusion direction must be nonzero")
            }
            Self::InvalidRevolutionAxis => {
                formatter.write_str("revolution axis must be a unit vector")
            }
            Self::InvalidSurfaceFrame => {
                formatter.write_str("analytic surface frame is not orthonormal")
            }
            Self::InvalidRadius => formatter.write_str("surface radius must be positive"),
            Self::InvalidConeAngle => formatter.write_str("invalid cone semi-angle"),
            Self::InvalidTorusRadii => formatter.write_str("invalid ring-torus radii"),
            Self::SurfaceParameterOutsideDomain => {
                formatter.write_str("parameter lies outside the exact surface domain")
            }
            Self::SingularSurfaceParameter => formatter.write_str("surface parameter is singular"),
            Self::UnsupportedDerivative => formatter.write_str("curve derivative is unsupported"),
            Self::InvalidDerivativeOrder => {
                formatter.write_str("curve derivative order must be positive")
            }
            Self::UnsupportedSubdivision => formatter.write_str("curve subdivision is unsupported"),
            Self::SplitAtBoundary => {
                formatter.write_str("curve subdivision requires an interior parameter")
            }
            Self::UnsupportedParameterLocation => {
                formatter.write_str("curve parameter location is unsupported")
            }
            Self::UnrepresentableParameter => {
                formatter.write_str("certified curve parameter is not representable by Real")
            }
            Self::UnsupportedIntersection => {
                formatter.write_str("geometry intersection is unsupported")
            }
            Self::UnsupportedMeasurement => {
                formatter.write_str("exact measurement is unsupported for this geometry")
            }
            Self::UnsupportedPcurveContour => {
                formatter.write_str("pcurve family is unsupported in face contours")
            }
            Self::PointNotOnCurve => formatter.write_str("point is not on the finite curve"),
            Self::PlanarCurve(error) => write!(formatter, "planar curve operation failed: {error}"),
            Self::PlanarCurveConstruction(error) => {
                write!(formatter, "planar curve construction failed: {error}")
            }
            Self::PlanarClassificationUnresolved(reason) => {
                write!(formatter, "planar classification unresolved: {reason:?}")
            }
            Self::PredicateUnresolved { needed, stage } => {
                write!(
                    formatter,
                    "exact predicate unresolved at {stage:?}; needed {needed:?}"
                )
            }
        }
    }
}

impl std::error::Error for GeometryError {}

impl From<hypercurve::ExactCurveError> for GeometryError {
    fn from(value: hypercurve::ExactCurveError) -> Self {
        Self::PlanarCurve(value)
    }
}

impl From<hypercurve::CurveError> for GeometryError {
    fn from(value: hypercurve::CurveError) -> Self {
        Self::PlanarCurveConstruction(value)
    }
}

/// Result of an exact geometry operation.
pub type GeometryResult<T> = Result<T, GeometryError>;
