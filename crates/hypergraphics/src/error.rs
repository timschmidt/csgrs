//! Error types for projection, mesh conversion, and backend upload.

use std::error::Error as StdError;
use std::fmt;

/// Crate-local result type.
pub type Result<T> = std::result::Result<T, Error>;

/// Errors produced by hypergraphics.
#[derive(Debug)]
pub enum Error {
    /// A primitive float input was NaN or infinite.
    NonFinitePrimitive {
        /// Name of the rejected value.
        value: &'static str,
    },
    /// A hyperreal value could not be exported to a finite primitive float.
    NonFiniteProjection {
        /// Name of the rejected value.
        value: &'static str,
    },
    /// A primitive value was finite as f64 but could not be represented as f32.
    F32Overflow {
        /// Name of the rejected value.
        value: &'static str,
    },
    /// The requested triangle index does not exist in a triangle mesh.
    TriangleIndexOutOfBounds {
        /// Requested triangle index.
        index: usize,
        /// Number of triangles in the mesh.
        triangle_count: usize,
    },
    /// The operation requires a triangle mesh.
    RequiresTriangles,
    /// Hyperreal or hyperlattice arithmetic rejected an operation.
    Arithmetic(hyperlattice::Problem),
    /// Hypertri rejected or could not triangulate the polygon.
    Triangulation(hypertri::Error),
    /// OpenGL shader/program/buffer setup failed.
    Backend(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinitePrimitive { value } => {
                write!(f, "{value} must be a finite primitive float")
            }
            Self::NonFiniteProjection { value } => {
                write!(
                    f,
                    "{value} could not be projected to a finite primitive float"
                )
            }
            Self::F32Overflow { value } => {
                write!(f, "{value} could not be represented as finite f32")
            }
            Self::TriangleIndexOutOfBounds {
                index,
                triangle_count,
            } => write!(
                f,
                "triangle index {index} is out of bounds for {triangle_count} triangles"
            ),
            Self::RequiresTriangles => write!(f, "operation requires a triangle mesh"),
            Self::Arithmetic(problem) => write!(f, "hyperreal arithmetic failed: {problem}"),
            Self::Triangulation(error) => write!(f, "triangulation failed: {error}"),
            Self::Backend(error) => write!(f, "backend error: {error}"),
        }
    }
}

impl StdError for Error {}

impl From<hyperlattice::Problem> for Error {
    fn from(value: hyperlattice::Problem) -> Self {
        Self::Arithmetic(value)
    }
}

impl From<hypertri::Error> for Error {
    fn from(value: hypertri::Error) -> Self {
        Self::Triangulation(value)
    }
}
