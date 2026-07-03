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
    /// A scalar predicate could not certify its sign through exact predicate
    /// routes without choosing a precision budget.
    UnknownClassification,
    /// Certified output extraction found boundary edges.
    OpenOutput {
        /// Number of undirected edges used by exactly one triangle.
        boundary_edges: usize,
        /// Number of undirected edges used by more than two triangles.
        non_manifold_edges: usize,
    },
    /// A homogeneous point had zero or unknown homogeneous scale.
    PointAtInfinity,
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
            Self::OpenOutput {
                boundary_edges,
                non_manifold_edges,
            } => write!(
                f,
                "output has boundary: {boundary_edges} boundary edges, {non_manifold_edges} non-manifold edges"
            ),
            Self::PointAtInfinity => f.write_str("homogeneous point is at infinity"),
        }
    }
}

impl Error for HypermeshError {}
