//! Errors produced by CSG grammar and native-curve operations.

use hypercurve::{CurveError, ExactCurveError};

/// Failure to produce a certified native curve-region Boolean.
#[derive(Clone, Debug, thiserror::Error, PartialEq)]
pub enum CurveBooleanError {
    /// Hypercurve rejected the topology operation.
    #[error(transparent)]
    Curve(#[from] CurveError),
    /// A higher-order exact curve operation failed with retained context.
    #[error(transparent)]
    ExactCurve(#[from] ExactCurveError),
}

/// Failure in a CSG construction or modeling operation.
#[derive(Debug, Clone, thiserror::Error, PartialEq)]
#[non_exhaustive]
pub enum ValidationError {
    /// A length or radius that must be nonnegative was proved negative.
    #[error("{name} must be nonnegative")]
    NegativeParameter { name: &'static str },
    /// A tessellation count does not meet the constructor's minimum.
    #[error("{name} must be at least {min}, got {actual}")]
    InvalidResolution {
        name: &'static str,
        min: usize,
        actual: usize,
    },
    /// The requested analytic primitive is outside the supported representation.
    #[error("unsupported primitive: {reason}")]
    UnsupportedPrimitive { reason: &'static str },
    /// A necessary predicate has no admissible proof under the selected policy.
    #[error("{predicate} is undecided under {policy:?}")]
    PredicateUndecided {
        predicate: &'static str,
        policy: hyperlimit::PredicatePolicy,
    },
    /// A mesh buffer length or byte size cannot be represented on this platform.
    #[error("primitive buffer capacity exceeds platform limits")]
    CapacityOverflow,
    /// Reserving a primitive buffer failed before a complete mesh was published.
    #[error("could not allocate primitive buffer for {elements} elements")]
    AllocationFailed { elements: usize },
    /// A checked scalar or lattice operation failed, retaining its native cause.
    #[error(transparent)]
    Scalar(#[from] hyperreal::Problem),
    /// A native mesh operation failed, retaining its typed cause.
    #[error("{0}")]
    Mesh(#[from] hypermesh::HypermeshError),
    /// Operation requires corresponding loops with the same number of vertices.
    #[error("operation requires loops with the same number of vertices, {left} != {right}")]
    MismatchedVertexCount { left: usize, right: usize },
    /// A required index is outside a collection.
    #[error("index {index} is out of range for length {len}")]
    IndexOutOfRangeWithLen { index: usize, len: usize },
    /// Feature arguments do not describe supported geometry.
    #[error("invalid arguments")]
    InvalidArguments,
    /// A named integer field is below the supported minimum.
    #[error("{name} must not be less than {min}")]
    FieldLessThan { name: &'static str, min: i32 },
    /// A native geometry dependency rejected the requested construction.
    #[error("{0}")]
    Geometry(String),
}

/// Stable failure categories exposed by modeling operations and bindings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ValidationErrorKind {
    InvalidInput,
    Unsupported,
    Undecided,
    ResourceExhausted,
    Unbounded,
    Geometry,
}

impl ValidationErrorKind {
    /// Machine-readable category used by JavaScript exceptions.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::InvalidInput => "invalid_input",
            Self::Unsupported => "unsupported",
            Self::Undecided => "undecided",
            Self::ResourceExhausted => "resource_exhausted",
            Self::Unbounded => "unbounded",
            Self::Geometry => "geometry_failure",
        }
    }
}

impl ValidationError {
    pub const fn kind(&self) -> ValidationErrorKind {
        match self {
            Self::Mesh(error) => match error {
                hypermesh::HypermeshError::PredicateUndecided { .. }
                | hypermesh::HypermeshError::ConvexHullPredicate { .. }
                | hypermesh::HypermeshError::UnknownClassification => {
                    ValidationErrorKind::Undecided
                },
                hypermesh::HypermeshError::CapacityOverflow { .. }
                | hypermesh::HypermeshError::WindingOverflow => {
                    ValidationErrorKind::ResourceExhausted
                },
                hypermesh::HypermeshError::PointAtInfinity
                | hypermesh::HypermeshError::UnboundedProjectiveTriangle { .. }
                | hypermesh::HypermeshError::UnboundedBooleanOutput { .. } => {
                    ValidationErrorKind::Unbounded
                },
                hypermesh::HypermeshError::OpenOutput { .. }
                | hypermesh::HypermeshError::SurfaceArrangementFailed { .. } => {
                    ValidationErrorKind::Geometry
                },
                _ => ValidationErrorKind::InvalidInput,
            },
            Self::UnsupportedPrimitive { .. } => ValidationErrorKind::Unsupported,
            Self::PredicateUndecided { .. }
            | Self::Scalar(hyperreal::Problem::UnknownZero) => ValidationErrorKind::Undecided,
            Self::CapacityOverflow
            | Self::AllocationFailed { .. }
            | Self::Scalar(hyperreal::Problem::Exhausted) => {
                ValidationErrorKind::ResourceExhausted
            },
            Self::Geometry(_) | Self::Scalar(_) => ValidationErrorKind::Geometry,
            _ => ValidationErrorKind::InvalidInput,
        }
    }
}
