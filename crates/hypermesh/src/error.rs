//! Error types for hypermesh operations.

use std::error::Error;
use std::fmt;

/// Result alias used by fallible hypermesh APIs.
pub type HypermeshResult<T> = Result<T, HypermeshError>;

/// Errors reported by exact geometric routines.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum HypermeshError {
    /// A triangle or polygon references a vertex index outside the input slice.
    VertexIndexOutOfBounds {
        /// Requested vertex index.
        index: usize,
        /// Number of vertices in the input slice.
        vertex_count: usize,
    },
    /// A mesh operation needs at least one point.
    EmptyInput,
    /// A scalar predicate could not certify its sign within the configured
    /// exact-real refinement budget.
    UnknownClassification,
    /// A homogeneous point had zero or unknown homogeneous scale.
    PointAtInfinity,
    /// OBJ text could not be parsed.
    InvalidObj {
        /// One-based line number.
        line: usize,
        /// Parse failure detail.
        reason: String,
    },
    /// File input failed.
    Io {
        /// Path that was requested.
        path: String,
        /// I/O failure detail.
        reason: String,
    },
}

impl fmt::Display for HypermeshError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::VertexIndexOutOfBounds {
                index,
                vertex_count,
            } => write!(
                f,
                "vertex index {index} is out of bounds for {vertex_count} vertices"
            ),
            Self::EmptyInput => f.write_str("input mesh set has no positions"),
            Self::UnknownClassification => f.write_str("could not certify scalar sign"),
            Self::PointAtInfinity => f.write_str("homogeneous point is at infinity"),
            Self::InvalidObj { line, reason } => {
                write!(f, "invalid OBJ at line {line}: {reason}")
            }
            Self::Io { path, reason } => write!(f, "could not read {path}: {reason}"),
        }
    }
}

impl Error for HypermeshError {}
