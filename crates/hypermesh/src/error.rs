//! Typed blockers for exact mesh import, validation, and kernel execution.

use std::fmt;

/// Stable category for a mesh blocker.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactMeshBlockerKind {
    /// Coordinate buffer length is not divisible by three.
    VertexBufferArity,
    /// Index buffer length is not divisible by three.
    IndexBufferArity,
    /// Primitive coordinate was NaN or infinite.
    NonFiniteCoordinate,
    /// Primitive coordinate could not be converted to `hyperreal::Real`.
    CoordinateImportFailed,
    /// Triangle index referenced a missing vertex.
    IndexOutOfBounds,
    /// Triangle repeats a vertex or is exactly collinear.
    DegenerateTriangle,
    /// Two faces use the same directed edge.
    DuplicateDirectedEdge,
    /// An undirected edge has only one incident face.
    BoundaryEdge,
    /// An undirected edge has more than two incident faces.
    NonManifoldEdge,
    /// A vertex link is not a single disk or circle.
    NonManifoldVertexLink,
    /// Duplicate triangle vertex set.
    DuplicateTriangle,
    /// Retained exact facts or acceleration structures did not replay against
    /// the supplied source mesh objects.
    StaleFactReplay,
    /// Requested exact operation is not yet certified by the exact stack.
    UnsupportedExactOperation,
}

/// One fatal validation or import blocker.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactMeshBlocker {
    /// Stable category.
    pub kind: ExactMeshBlockerKind,
    /// Human-readable detail.
    pub message: String,
    /// Optional vertex index.
    pub vertex: Option<usize>,
    /// Optional face index.
    pub face: Option<usize>,
    /// Optional coordinate index in a flat coordinate buffer.
    pub coordinate: Option<usize>,
    /// Optional undirected edge endpoints.
    pub edge: Option<[usize; 2]>,
}

impl ExactMeshBlocker {
    /// Build a blocker with no object location.
    pub fn new(kind: ExactMeshBlockerKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
            vertex: None,
            face: None,
            coordinate: None,
            edge: None,
        }
    }

    /// Attach a vertex index.
    pub const fn with_vertex(mut self, vertex: usize) -> Self {
        self.vertex = Some(vertex);
        self
    }

    /// Attach a face index.
    pub const fn with_face(mut self, face: usize) -> Self {
        self.face = Some(face);
        self
    }

    /// Attach a flat coordinate index.
    pub const fn with_coordinate(mut self, coordinate: usize) -> Self {
        self.coordinate = Some(coordinate);
        self
    }

    /// Attach an undirected edge.
    pub const fn with_edge(mut self, edge: [usize; 2]) -> Self {
        self.edge = Some(edge);
        self
    }
}

/// Error returned when mesh construction has one or more fatal blockers.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactMeshError {
    /// Blockers collected before construction stopped.
    pub blockers: Vec<ExactMeshBlocker>,
}

impl ExactMeshError {
    /// Build an error from blockers.
    pub fn new(blockers: Vec<ExactMeshBlocker>) -> Self {
        Self { blockers }
    }

    /// Build an error containing one blocker.
    pub fn one(blocker: ExactMeshBlocker) -> Self {
        Self {
            blockers: vec![blocker],
        }
    }
}

impl fmt::Display for ExactMeshError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.blockers.as_slice() {
            [] => write!(f, "mesh validation failed"),
            [blocker] => write!(f, "{}", blocker.message),
            blockers => write!(f, "{} mesh blockers", blockers.len()),
        }
    }
}

impl std::error::Error for ExactMeshError {}
