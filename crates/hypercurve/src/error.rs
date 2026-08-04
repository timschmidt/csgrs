//! Error types for curve construction and topology operations.

use std::fmt;

use crate::{CurveFamily2, UncertaintyReason};

/// Result alias used by `hypercurve`.
pub type CurveResult<T> = Result<T, CurveError>;

/// Result of a top-level exact curve operation.
pub type ExactCurveResult<T> = Result<T, ExactCurveError>;

/// Exact kernel operation that failed or could not be certified.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveOperation2 {
    /// Validate and construct a curve carrier.
    Construction,
    /// Decompose a spline into exact Bezier spans.
    BezierDecomposition,
    /// Insert an exact knot without changing a spline's image.
    KnotInsertion,
    /// Remove one exact knot occurrence without changing a spline's parameterized image.
    KnotRemoval,
    /// Elevate a curve's polynomial degree without changing its parameterized image.
    DegreeElevation,
    /// Interpolate exact point constraints into a spline carrier.
    Interpolation,
    /// Split or trim an exact curve in its public parameter domain.
    Subdivision,
    /// Replace one path vertex with an exact line chamfer.
    Chamfer,
    /// Replace one path vertex with an exact tangent circular fillet.
    Fillet,
    /// Construct an exact parallel boundary offset.
    Offset,
    /// Reverse exact traversal while preserving the curve image.
    Reversal,
    /// Apply an exact geometry-preserving transform.
    Transformation,
    /// Promote retained curve evidence into native topology.
    NativeTopology,
    /// Evaluate an exact curve at a retained parameter.
    Evaluation,
    /// Classify an exact geometric property or relation.
    Classification,
    /// Intersect exact curves.
    Intersection,
    /// Build an exact curve arrangement.
    Arrangement,
    /// Evaluate a regularized region Boolean operation.
    Boolean,
}

/// Context for an exact operation that could not certify a required branch.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactCurveBlocker {
    operation: CurveOperation2,
    family: CurveFamily2,
    reason: UncertaintyReason,
}

impl ExactCurveBlocker {
    /// Constructs a contextual exact-operation blocker.
    pub(crate) const fn new(
        operation: CurveOperation2,
        family: CurveFamily2,
        reason: UncertaintyReason,
    ) -> Self {
        Self {
            operation,
            family,
            reason,
        }
    }

    /// Returns the blocked operation.
    pub const fn operation(self) -> CurveOperation2 {
        self.operation
    }

    /// Returns the curve family involved in the blocked operation.
    pub const fn family(self) -> CurveFamily2 {
        self.family
    }

    /// Returns the exact predicate or capability reason.
    pub const fn reason(self) -> UncertaintyReason {
        self.reason
    }
}

/// Contextual failure from a top-level exact curve operation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactCurveError {
    /// Input or retained state violated a curve invariant.
    Invalid {
        /// Operation that detected the invalid state.
        operation: CurveOperation2,
        /// Curve family being processed.
        family: CurveFamily2,
        /// Underlying invariant failure.
        cause: CurveError,
    },
    /// Exact processing stopped before making a topology decision.
    Blocked(ExactCurveBlocker),
}

impl ExactCurveError {
    /// Wraps a low-level curve error with operation context.
    pub(crate) const fn invalid(
        operation: CurveOperation2,
        family: CurveFamily2,
        cause: CurveError,
    ) -> Self {
        Self::Invalid {
            operation,
            family,
            cause,
        }
    }

    /// Constructs a blocker with operation context.
    pub(crate) const fn blocked(
        operation: CurveOperation2,
        family: CurveFamily2,
        reason: UncertaintyReason,
    ) -> Self {
        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))
    }

    /// Returns the operation that failed.
    pub const fn operation(&self) -> CurveOperation2 {
        match self {
            Self::Invalid { operation, .. } => *operation,
            Self::Blocked(blocker) => blocker.operation(),
        }
    }

    /// Returns the affected curve family.
    pub const fn family(&self) -> CurveFamily2 {
        match self {
            Self::Invalid { family, .. } => *family,
            Self::Blocked(blocker) => blocker.family(),
        }
    }

    pub(crate) fn with_operation(self, operation: CurveOperation2) -> Self {
        match self {
            Self::Invalid { family, cause, .. } => Self::invalid(operation, family, cause),
            Self::Blocked(blocker) => Self::blocked(operation, blocker.family(), blocker.reason()),
        }
    }
}

impl fmt::Display for ExactCurveBlocker {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "exact {:?} for {:?} was blocked by {:?}",
            self.operation, self.family, self.reason
        )?;
        Ok(())
    }
}

impl fmt::Display for ExactCurveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Invalid {
                operation,
                family,
                cause,
            } => {
                write!(f, "invalid {:?} during exact {:?}", family, operation)?;
                write!(f, ": {cause}")
            }
            Self::Blocked(blocker) => blocker.fmt(f),
        }
    }
}

impl std::error::Error for ExactCurveError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Invalid { cause, .. } => Some(cause),
            Self::Blocked(_) => None,
        }
    }
}

/// Errors returned by curve constructors and early topology scaffolding.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CurveError {
    /// A line segment has equal endpoints.
    ZeroLengthLine,
    /// A circular arc has zero radius.
    ZeroRadiusArc,
    /// A rational Bezier weight is structurally zero.
    ZeroRationalBezierWeight,
    /// A general rational Bezier control/weight layout is invalid.
    InvalidRationalBezier,
    /// A requested degree elevation is lower than the source degree or overflows.
    InvalidDegreeElevation,
    /// NURBS interpolation points, parameters, weights, or knots are inconsistent.
    InvalidNurbsInterpolation,
    /// The exact NURBS interpolation coefficient matrix is singular.
    SingularNurbsInterpolation {
        /// Pivot at which singularity was certified.
        pivot: usize,
    },
    /// Exact interpolation solve division is unsupported by the current scalar package.
    UnsupportedNurbsInterpolationDivision {
        /// Pivot or solution column that required the unsupported division.
        index: usize,
    },
    /// Exact replay certified a nonzero NURBS interpolation residual.
    InconsistentNurbsInterpolationSolution {
        /// Constraint row whose solved coordinate did not replay.
        row: usize,
    },
    /// A NURBS homogeneous denominator is exactly zero at an evaluation parameter.
    ZeroNurbsDenominator,
    /// A circular arc start/end pair does not lie on a common supplied circle.
    RadiusMismatch,
    /// Circular-arc endpoint/orientation evidence does not define a valid finite sweep.
    InvalidArcSweep,
    /// A bulge value is too close to zero to choose line versus arc semantics.
    AmbiguousBulge,
    /// A curve string needs at least two vertices or one segment.
    InsufficientVertices,
    /// A curve string cannot be empty when built through checked constructors.
    EmptyCurveString,
    /// A top-level exact curve path cannot be empty.
    EmptyCurvePath,
    /// Adjacent curve-string segments are not connected.
    DisconnectedCurveString,
    /// Adjacent curve-string segment connectivity could not be classified.
    AmbiguousCurveStringConnection,
    /// Adjacent top-level curves do not share an endpoint.
    DisconnectedCurvePath,
    /// A closed-boundary operation received an open top-level curve path.
    OpenCurvePath,
    /// Nested and top-level retained source identities disagree.
    ConflictingCurveSource,
    /// Polyline reconstruction options contain non-finite or unsupported values.
    InvalidReconstructionOptions,
    /// Bezier flattening options cannot certify a positive error budget.
    InvalidFlatteningOptions,
    /// Bezier parallel verification options cannot certify a positive error budget.
    InvalidBezierOffsetOptions,
    /// Exact region-offset corner options are outside their defined domain.
    InvalidOffsetOptions,
    /// Exact fillet or chamfer design options are outside their defined domain.
    InvalidCornerOptions,
    /// Finite projection options contain non-finite or unsupported values.
    InvalidFiniteProjectionOptions,
    /// Edge-preview tolerances are negative or non-finite.
    InvalidPreviewOptions,
    /// Retained import record metadata is inconsistent or non-finite.
    InvalidImportRecord,
    /// A finite affine transform is not a nonsingular planar similarity.
    InvalidSimilarityTransform,
    /// A planar affine transform has a singular or undecidable linear part.
    InvalidAffineTransform,
    /// A Bezier parameter is certified outside the closed unit interval.
    InvalidBezierParameter,
    /// A Bezier segment range is outside the stored path facts.
    InvalidBezierRange,
    /// A native curve parameter is certified outside its accepted domain.
    InvalidCurveParameter,
    /// A native curve trim range is empty, reversed, or outside the source path.
    InvalidCurveRange,
    /// A requested fillet is not tangent to both retained source curves in traversal order.
    InvalidFilletTangency,
    /// A certified positive-length overlap has a zero parameter range.
    DegenerateOverlapRange,
    /// A Bezier parameter polynomial is structurally invalid.
    InvalidBezierPolynomial,
    /// A Bezier algebraic parameter does not certify one isolated root.
    InvalidBezierAlgebraicParameter,
    /// A requested Bezier arc length is certified outside the curve length range.
    InvalidBezierArcLengthTarget,
    /// A polynomial B-spline has invalid degree, knot, or control-net structure.
    InvalidBSpline,
    /// One-period spline controls or knot breaks cannot define a periodic carrier.
    InvalidPeriodicSpline,
    /// A wrapped-parameter operation was requested from a non-periodic curve.
    CurveIsNotPeriodic,
    /// A declared periodic spline does not close exactly at its canonical seam.
    PeriodicSplineSeamMismatch,
    /// Polyline reconstruction needs coordinates with finite `f64` approximations.
    NonFiniteReconstructionPoint,
    /// Finite projection needs coordinates with finite `f64` approximations.
    NonFiniteProjectionPoint,
    /// A topology pipeline referenced inconsistent internal state.
    Topology(String),
    /// A `Real` division or elementary operation failed.
    Real(String),
}

impl fmt::Display for CurveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroLengthLine => write!(f, "line segment has zero length"),
            Self::ZeroRadiusArc => write!(f, "circular arc has zero radius"),
            Self::ZeroRationalBezierWeight => {
                write!(f, "rational Bezier weight is structurally zero")
            }
            Self::InvalidRationalBezier => write!(f, "invalid rational Bezier control layout"),
            Self::InvalidDegreeElevation => write!(f, "invalid curve degree elevation target"),
            Self::InvalidNurbsInterpolation => write!(f, "invalid NURBS interpolation inputs"),
            Self::SingularNurbsInterpolation { pivot } => {
                write!(f, "NURBS interpolation matrix is singular at pivot {pivot}")
            }
            Self::UnsupportedNurbsInterpolationDivision { index } => write!(
                f,
                "NURBS interpolation exact division is unsupported at index {index}"
            ),
            Self::InconsistentNurbsInterpolationSolution { row } => {
                write!(
                    f,
                    "NURBS interpolation solution failed at constraint row {row}"
                )
            }
            Self::ZeroNurbsDenominator => {
                write!(f, "NURBS denominator is zero at the evaluation parameter")
            }
            Self::RadiusMismatch => write!(f, "arc endpoints do not share the supplied radius"),
            Self::InvalidArcSweep => write!(f, "circular arc sweep evidence is invalid"),
            Self::AmbiguousBulge => write!(f, "bulge sign or zero status is ambiguous"),
            Self::InsufficientVertices => write!(f, "curve string has insufficient vertices"),
            Self::EmptyCurveString => write!(f, "curve string has no segments"),
            Self::EmptyCurvePath => write!(f, "curve path has no curves"),
            Self::DisconnectedCurveString => write!(f, "curve string segments are disconnected"),
            Self::AmbiguousCurveStringConnection => {
                write!(f, "curve string segment connectivity is ambiguous")
            }
            Self::DisconnectedCurvePath => {
                write!(f, "adjacent curve path endpoints are disconnected")
            }
            Self::OpenCurvePath => write!(f, "curve path is not closed"),
            Self::ConflictingCurveSource => {
                write!(f, "top-level and retained curve source identities conflict")
            }
            Self::InvalidReconstructionOptions => {
                write!(f, "polyline reconstruction options are invalid")
            }
            Self::InvalidFlatteningOptions => write!(f, "Bezier flattening options are invalid"),
            Self::InvalidBezierOffsetOptions => {
                write!(f, "Bezier parallel verification options are invalid")
            }
            Self::InvalidOffsetOptions => write!(f, "region offset options are invalid"),
            Self::InvalidCornerOptions => write!(f, "corner edit options are invalid"),
            Self::InvalidFiniteProjectionOptions => {
                write!(f, "finite projection options are invalid")
            }
            Self::InvalidPreviewOptions => write!(f, "curve preview options are invalid"),
            Self::InvalidImportRecord => write!(f, "retained import record is invalid"),
            Self::InvalidSimilarityTransform => {
                write!(f, "affine transform is not a planar similarity")
            }
            Self::InvalidAffineTransform => write!(f, "invalid planar affine transform"),
            Self::InvalidBezierParameter => {
                write!(f, "Bezier parameter is outside the closed unit interval")
            }
            Self::InvalidBezierRange => write!(f, "Bezier segment range is invalid"),
            Self::InvalidCurveParameter => write!(f, "curve parameter is invalid"),
            Self::InvalidCurveRange => write!(f, "curve trim range is invalid"),
            Self::InvalidFilletTangency => {
                write!(
                    f,
                    "fillet is not tangent to the source curves in traversal order"
                )
            }
            Self::DegenerateOverlapRange => {
                write!(f, "positive-length overlap has a zero parameter range")
            }
            Self::InvalidBezierPolynomial => write!(f, "Bezier parameter polynomial is invalid"),
            Self::InvalidBezierAlgebraicParameter => {
                write!(
                    f,
                    "Bezier algebraic parameter does not isolate exactly one root"
                )
            }
            Self::InvalidBezierArcLengthTarget => {
                write!(
                    f,
                    "Bezier arc-length target is outside the certified length range"
                )
            }
            Self::InvalidBSpline => write!(f, "B-spline degree, knots, or controls are invalid"),
            Self::InvalidPeriodicSpline => {
                write!(f, "periodic spline controls or knot breaks are invalid")
            }
            Self::CurveIsNotPeriodic => write!(f, "curve does not have periodic semantics"),
            Self::PeriodicSplineSeamMismatch => {
                write!(
                    f,
                    "periodic spline does not close exactly at its canonical seam"
                )
            }
            Self::NonFiniteReconstructionPoint => {
                write!(f, "polyline reconstruction point is not finite")
            }
            Self::NonFiniteProjectionPoint => write!(f, "finite projection point is not finite"),
            Self::Topology(message) => write!(f, "topology operation failed: {message}"),
            Self::Real(message) => write!(f, "Real operation failed: {message}"),
        }
    }
}

impl std::error::Error for CurveError {}

impl From<hyperreal::Problem> for CurveError {
    fn from(value: hyperreal::Problem) -> Self {
        Self::Real(value.to_string())
    }
}
