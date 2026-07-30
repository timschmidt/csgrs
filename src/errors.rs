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

/// Failure to construct a native curve-region offset.
#[derive(Clone, Debug, thiserror::Error, PartialEq, Eq)]
pub enum CurveOffsetError {
    /// Hypercurve rejected an exact curve-region offset operation.
    #[error(transparent)]
    ExactCurve(#[from] ExactCurveError),
}

/// Validation failure in a CSG feature constructor.
#[derive(Debug, Clone, thiserror::Error, PartialEq)]
#[non_exhaustive]
pub enum ValidationError {
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
