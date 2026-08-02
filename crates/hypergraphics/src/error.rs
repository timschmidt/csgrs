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
    /// A viewport dimension is zero or negative.
    NonPositiveViewportExtent {
        /// Name of the rejected dimension.
        value: &'static str,
    },
    /// The viewport's finite extents cannot form a stable finite aspect ratio.
    InvalidViewportAspect,
    /// A camera parameter cannot form a valid perspective projection.
    InvalidCameraParameter {
        /// Name of the rejected parameter.
        value: &'static str,
    },
    /// An exact predicate could not decide under the requested policy.
    #[cfg(feature = "exact")]
    IndeterminatePredicate {
        /// Name of the predicate or parameter being decided.
        predicate: &'static str,
        /// Additional capability required to decide it.
        needed: hyperlimit::RefinementNeed,
        /// Last escalation stage attempted.
        stage: hyperlimit::Escalation,
    },
    /// A mesh has more vertices than OpenGL's signed draw-count field accepts.
    VertexCountOverflow {
        /// Rejected vertex count.
        count: usize,
    },
    /// A flat vertex buffer does not contain a whole number of vertices.
    InvalidVertexDataLength {
        /// Number of scalar values in the buffer.
        values: usize,
        /// Required scalar values per vertex.
        stride: usize,
    },
    /// A requested grid cannot be represented or allocated safely.
    #[cfg(feature = "exact")]
    GridSizeOverflow {
        /// Requested number of steps on either side of the origin.
        half_steps: u32,
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
    #[cfg(feature = "exact")]
    Arithmetic(hyperlattice::Problem),
    /// Hypertri rejected or could not triangulate the polygon.
    #[cfg(feature = "exact")]
    Triangulation(hypertri::Error),
    /// OpenGL shader/program/buffer setup failed.
    Backend(String),
    /// The active graphics context lacks a capability required by the backend.
    UnsupportedBackend {
        /// Required graphics capability.
        capability: &'static str,
    },
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
            Self::NonPositiveViewportExtent { value } => {
                write!(f, "viewport {value} must be positive")
            }
            Self::InvalidViewportAspect => {
                write!(
                    f,
                    "viewport dimensions must form a stable finite aspect ratio"
                )
            }
            Self::InvalidCameraParameter { value } => {
                write!(f, "camera {value} is outside its valid projection domain")
            }
            #[cfg(feature = "exact")]
            Self::IndeterminatePredicate {
                predicate,
                needed,
                stage,
            } => write!(
                f,
                "{predicate} remained undecided after {stage:?}; additional {needed:?} is required"
            ),
            Self::VertexCountOverflow { count } => {
                write!(f, "vertex count {count} exceeds the OpenGL draw limit")
            }
            Self::InvalidVertexDataLength { values, stride } => write!(
                f,
                "vertex buffer contains {values} values, which is not divisible by stride {stride}"
            ),
            #[cfg(feature = "exact")]
            Self::GridSizeOverflow { half_steps } => write!(
                f,
                "grid with {half_steps} steps per half-axis is too large to allocate"
            ),
            Self::TriangleIndexOutOfBounds {
                index,
                triangle_count,
            } => write!(
                f,
                "triangle index {index} is out of bounds for {triangle_count} triangles"
            ),
            Self::RequiresTriangles => write!(f, "operation requires a triangle mesh"),
            #[cfg(feature = "exact")]
            Self::Arithmetic(problem) => write!(f, "hyperreal arithmetic failed: {problem}"),
            #[cfg(feature = "exact")]
            Self::Triangulation(error) => write!(f, "triangulation failed: {error}"),
            Self::Backend(error) => write!(f, "backend error: {error}"),
            Self::UnsupportedBackend { capability } => {
                write!(f, "graphics context does not support required {capability}")
            }
        }
    }
}

impl StdError for Error {
    fn source(&self) -> Option<&(dyn StdError + 'static)> {
        match self {
            #[cfg(feature = "exact")]
            Self::Arithmetic(problem) => Some(problem),
            #[cfg(feature = "exact")]
            Self::Triangulation(error) => Some(error),
            _ => None,
        }
    }
}

#[cfg(feature = "exact")]
impl From<hyperlattice::Problem> for Error {
    fn from(value: hyperlattice::Problem) -> Self {
        Self::Arithmetic(value)
    }
}

#[cfg(feature = "exact")]
impl From<hypertri::Error> for Error {
    fn from(value: hypertri::Error) -> Self {
        Self::Triangulation(value)
    }
}
