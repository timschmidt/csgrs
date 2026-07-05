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
    /// An individual input mesh has no positions or no triangles.
    EmptyMesh {
        /// Index of the empty mesh in the input slice.
        mesh_index: usize,
    },
    /// A source triangle is degenerate and cannot bound a PWN surface.
    DegenerateTriangle {
        /// Index of the mesh containing the triangle.
        mesh_index: usize,
        /// Index of the triangle within that mesh.
        triangle_index: usize,
    },
    /// An input mesh has boundary edges and is not closed.
    OpenInput {
        /// Index of the open mesh in the input slice.
        mesh_index: usize,
        /// Number of undirected edges used by exactly one triangle.
        boundary_edges: usize,
    },
    /// A scalar predicate could not certify its sign through exact predicate
    /// routes without choosing a precision budget.
    UnknownClassification,
    /// Subdivision could not construct a certified child-cell reference point.
    ReferencePropagationFailed,
    /// A subdividable task exhausted the configured depth budget before a
    /// certified leaf could be produced.
    SubdivisionDepthLimit {
        /// Depth at which subdivision stopped.
        depth: usize,
        /// Number of polygons remaining in the uncertified task.
        polygon_count: usize,
    },
    /// Certified output extraction found boundary edges.
    OpenOutput {
        /// Number of undirected edges used by exactly one triangle.
        boundary_edges: usize,
        /// Number of undirected edges used by more than two triangles.
        non_manifold_edges: usize,
    },
    /// Exact output T-junction/crossing resolution exhausted its pass budget.
    OutputResolutionLimit {
        /// Maximum number of resolution passes allowed.
        pass_limit: usize,
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
            Self::EmptyInput => f.write_str("input mesh set is empty"),
            Self::EmptyMesh { mesh_index } => {
                write!(f, "input mesh {mesh_index} has no positions or triangles")
            }
            Self::DegenerateTriangle {
                mesh_index,
                triangle_index,
            } => write!(
                f,
                "input mesh {mesh_index} triangle {triangle_index} is degenerate"
            ),
            Self::OpenInput {
                mesh_index,
                boundary_edges,
            } => write!(
                f,
                "input mesh {mesh_index} has {boundary_edges} boundary edges"
            ),
            Self::UnknownClassification => f.write_str("could not certify scalar sign"),
            Self::ReferencePropagationFailed => {
                f.write_str("could not construct a certified subdivision reference")
            }
            Self::SubdivisionDepthLimit {
                depth,
                polygon_count,
            } => write!(
                f,
                "subdivision reached depth {depth} with {polygon_count} uncertified polygons"
            ),
            Self::OpenOutput {
                boundary_edges,
                non_manifold_edges,
            } => write!(
                f,
                "output has boundary: {boundary_edges} boundary edges, {non_manifold_edges} non-manifold edges"
            ),
            Self::OutputResolutionLimit { pass_limit } => {
                write!(
                    f,
                    "output resolution did not converge within {pass_limit} passes"
                )
            }
            Self::PointAtInfinity => f.write_str("homogeneous point is at infinity"),
        }
    }
}

impl Error for HypermeshError {}
