//! Error types for curve construction and topology operations.

use std::fmt;

/// Result alias used by `hypercurve`.
pub type CurveResult<T> = Result<T, CurveError>;

/// Errors returned by curve constructors and early topology scaffolding.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CurveError {
    /// A line segment has equal endpoints.
    ZeroLengthLine,
    /// A circular arc has zero radius.
    ZeroRadiusArc,
    /// A rational Bezier weight is structurally zero.
    ZeroRationalBezierWeight,
    /// A circular arc start/end pair does not lie on a common supplied circle.
    RadiusMismatch,
    /// A bulge value is too close to zero to choose line versus arc semantics.
    AmbiguousBulge,
    /// A curve string needs at least two vertices or one segment.
    InsufficientVertices,
    /// A curve string cannot be empty when built through checked constructors.
    EmptyCurveString,
    /// Adjacent curve-string segments are not connected.
    DisconnectedCurveString,
    /// Adjacent curve-string segment connectivity could not be classified.
    AmbiguousCurveStringConnection,
    /// Polyline reconstruction options contain non-finite or unsupported values.
    InvalidReconstructionOptions,
    /// Bezier flattening options cannot certify a positive error budget.
    InvalidFlatteningOptions,
    /// Finite projection options contain non-finite or unsupported values.
    InvalidFiniteProjectionOptions,
    /// Retained import record metadata is inconsistent or non-finite.
    InvalidImportRecord,
    /// Retained planar face trim metadata is inconsistent.
    InvalidPlanarFace,
    /// Retained analytic surface frame metadata is inconsistent.
    InvalidAnalyticSurfaceFrame,
    /// A finite affine transform is not a nonsingular planar similarity.
    InvalidSimilarityTransform,
    /// A Bezier parameter is certified outside the closed unit interval.
    InvalidBezierParameter,
    /// A Bezier segment range is outside the stored path facts.
    InvalidBezierRange,
    /// A Bezier parameter polynomial is structurally invalid.
    InvalidBezierPolynomial,
    /// A Bezier algebraic parameter does not certify one isolated root.
    InvalidBezierAlgebraicParameter,
    /// A requested Bezier arc length is certified outside the curve length range.
    InvalidBezierArcLengthTarget,
    /// A polynomial B-spline has invalid degree, knot, or control-net structure.
    InvalidBSpline,
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
            Self::RadiusMismatch => write!(f, "arc endpoints do not share the supplied radius"),
            Self::AmbiguousBulge => write!(f, "bulge sign or zero status is ambiguous"),
            Self::InsufficientVertices => write!(f, "curve string has insufficient vertices"),
            Self::EmptyCurveString => write!(f, "curve string has no segments"),
            Self::DisconnectedCurveString => write!(f, "curve string segments are disconnected"),
            Self::AmbiguousCurveStringConnection => {
                write!(f, "curve string segment connectivity is ambiguous")
            }
            Self::InvalidReconstructionOptions => {
                write!(f, "polyline reconstruction options are invalid")
            }
            Self::InvalidFlatteningOptions => write!(f, "Bezier flattening options are invalid"),
            Self::InvalidFiniteProjectionOptions => {
                write!(f, "finite projection options are invalid")
            }
            Self::InvalidImportRecord => write!(f, "retained import record is invalid"),
            Self::InvalidPlanarFace => write!(f, "retained planar face is invalid"),
            Self::InvalidAnalyticSurfaceFrame => {
                write!(f, "retained analytic surface frame is invalid")
            }
            Self::InvalidSimilarityTransform => {
                write!(f, "affine transform is not a planar similarity")
            }
            Self::InvalidBezierParameter => {
                write!(f, "Bezier parameter is outside the closed unit interval")
            }
            Self::InvalidBezierRange => write!(f, "Bezier segment range is invalid"),
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
