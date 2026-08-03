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
    /// A query supplied a point slice different from the one used to build an
    /// acceleration structure.
    PointCountMismatch {
        /// Point count retained by the acceleration structure.
        expected: usize,
        /// Point count supplied to the query.
        actual: usize,
    },
    /// Boolean output triangles and their provenance rows are not parallel.
    TriangleSourceCountMismatch {
        /// Number of output triangles.
        triangles: usize,
        /// Number of source-provenance rows.
        sources: usize,
    },
    /// A collection size required by an operation cannot be represented.
    CapacityOverflow {
        /// Operation whose output size overflowed.
        operation: &'static str,
    },
    /// Winding vectors with different dimensions were combined.
    WindingDimensionMismatch {
        /// Required component count.
        expected: usize,
        /// Supplied component count.
        actual: usize,
    },
    /// Checked winding arithmetic exceeded the `i32` representation.
    WindingOverflow,
    /// A convex hull requires a three-dimensional point set.
    DegeneratePointSet,
    /// A numeric predicate could not be decided under the selected policy.
    PredicateUndecided {
        /// Predicate or construction decision that failed.
        predicate: &'static str,
    },
    /// Convex hull construction could not certify a required predicate.
    ConvexHullPredicate {
        /// Hull stage that required the undecidable predicate.
        stage: &'static str,
    },
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
    /// An input mesh has nonzero signed boundary and therefore does not define
    /// a closed piecewise-winding-number surface.
    NonPwnInput {
        /// Index of the mesh with inconsistent directed edge multiplicities.
        mesh_index: usize,
        /// Number of geometric edge classes whose forward and reverse uses do
        /// not cancel.
        unbalanced_edges: usize,
    },
    /// A mesh presented for convex certification has a vertex outside one of
    /// its outward-oriented supporting half-spaces.
    NonConvexInput,
    /// A predicate or certified construction could not be decided through the
    /// strict exact-predicate routes without choosing a precision budget or an
    /// approximate fallback.
    ///
    /// This is the public boundary for arbitrary undecidable computable
    /// `hyperreal::Real` inputs under bounded refinement: if the implementation
    /// cannot certify the required sign, incidence, or witness exactly, it
    /// returns this error instead of silently using an approximate answer.
    UnknownClassification,
    /// Subdivision could not construct a certified child-cell reference point
    /// by the enabled exact reference-propagation path family.
    ReferencePropagationFailed,
    /// A task with a remaining exact root-basis arrangement split exhausted the
    /// configured depth budget before a certified leaf could be produced.
    SubdivisionDepthLimit {
        /// Depth at which subdivision stopped.
        depth: usize,
        /// Number of polygons remaining in the uncertified task.
        polygon_count: usize,
    },
    /// Certified output extraction found singleton or directionally
    /// unbalanced edges.
    OpenOutput {
        /// Number of undirected edges used by exactly one triangle.
        boundary_edges: usize,
        /// Number of geometric edge classes whose forward and reverse uses do
        /// not cancel.
        unbalanced_edges: usize,
        /// Number of undirected edges used by more than two triangles.
        non_manifold_edges: usize,
    },
    /// Exact planar subdivision of overlapping coplanar output faces failed.
    OutputPlanarizationFailed {
        /// Planar triangulation stage that could not be completed.
        reason: &'static str,
    },
    /// Exact source-face arrangement or bounded-cell triangulation failed.
    SurfaceArrangementFailed {
        /// Arrangement invariant or triangulation stage that failed.
        reason: &'static str,
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
            Self::PointCountMismatch { expected, actual } => write!(
                f,
                "point acceleration structure contains {expected} points but query supplied {actual}"
            ),
            Self::TriangleSourceCountMismatch { triangles, sources } => write!(
                f,
                "Boolean output contains {triangles} triangles but {sources} provenance rows"
            ),
            Self::CapacityOverflow { operation } => {
                write!(f, "{operation} output size exceeds addressable capacity")
            }
            Self::WindingDimensionMismatch { expected, actual } => write!(
                f,
                "winding vector has {actual} components; expected {expected}"
            ),
            Self::WindingOverflow => f.write_str("winding arithmetic overflow"),
            Self::DegeneratePointSet => {
                f.write_str("convex hull input does not span three dimensions")
            }
            Self::PredicateUndecided { predicate } => {
                write!(f, "predicate could not be decided: {predicate}")
            }
            Self::ConvexHullPredicate { stage } => {
                write!(f, "convex hull could not certify predicate during {stage}")
            }
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
            Self::NonPwnInput {
                mesh_index,
                unbalanced_edges,
            } => write!(
                f,
                "input mesh {mesh_index} has {unbalanced_edges} directed edge imbalances"
            ),
            Self::NonConvexInput => f.write_str("input mesh is not convex"),
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
                unbalanced_edges,
                non_manifold_edges,
            } => write!(
                f,
                "output has boundary: {boundary_edges} singleton edges, {unbalanced_edges} directed edge imbalances, {non_manifold_edges} non-manifold edges"
            ),
            Self::OutputPlanarizationFailed { reason } => {
                write!(f, "could not planarize coplanar output faces: {reason}")
            }
            Self::SurfaceArrangementFailed { reason } => {
                write!(f, "could not construct exact surface arrangement: {reason}")
            }
            Self::PointAtInfinity => f.write_str("homogeneous point is at infinity"),
        }
    }
}

impl Error for HypermeshError {}
