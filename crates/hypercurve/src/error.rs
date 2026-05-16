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
    /// A circular arc start/end pair does not lie on a common supplied circle.
    RadiusMismatch,
    /// A bulge value is too close to zero to choose line versus arc semantics.
    AmbiguousBulge,
    /// Cavalier-compatible bulge import only accepts arcs up to a half circle.
    UnsupportedBulge,
    /// A curve string needs at least two vertices or one segment.
    InsufficientVertices,
    /// A curve string cannot be empty when built through checked constructors.
    EmptyCurveString,
    /// Adjacent curve-string segments are not connected.
    DisconnectedCurveString,
    /// Adjacent curve-string segment connectivity could not be classified.
    AmbiguousCurveStringConnection,
    /// A topology pipeline referenced inconsistent internal state.
    Topology(String),
    /// A scalar division or elementary scalar operation failed.
    Scalar(String),
}

impl fmt::Display for CurveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroLengthLine => write!(f, "line segment has zero length"),
            Self::ZeroRadiusArc => write!(f, "circular arc has zero radius"),
            Self::RadiusMismatch => write!(f, "arc endpoints do not share the supplied radius"),
            Self::AmbiguousBulge => write!(f, "bulge sign or zero status is ambiguous"),
            Self::UnsupportedBulge => {
                write!(f, "Cavalier-compatible bulge exceeds half-circle support")
            }
            Self::InsufficientVertices => write!(f, "curve string has insufficient vertices"),
            Self::EmptyCurveString => write!(f, "curve string has no segments"),
            Self::DisconnectedCurveString => write!(f, "curve string segments are disconnected"),
            Self::AmbiguousCurveStringConnection => {
                write!(f, "curve string segment connectivity is ambiguous")
            }
            Self::Topology(message) => write!(f, "topology operation failed: {message}"),
            Self::Scalar(message) => write!(f, "scalar operation failed: {message}"),
        }
    }
}

impl std::error::Error for CurveError {}

impl From<hyperlattice::Problem> for CurveError {
    fn from(value: hyperlattice::Problem) -> Self {
        Self::Scalar(value.to_string())
    }
}
