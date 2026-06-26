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
    /// An exact predicate or exact comparison could not produce a decided
    /// value.
    UndecidablePredicate,
    /// A retained exact construction artifact is missing, internally
    /// inconsistent, or failed its construction-family audit.
    ExactConstructionFailure,
    /// Retained exact facts or acceleration structures did not replay against
    /// the supplied source mesh objects.
    StaleFactReplay,
    /// A certified exact support path reached a topology case whose exact cell
    /// materializer is not available.
    UnsupportedCellMaterializer,
    /// Requested policy requires exact evidence that was not retained or could
    /// not be certified.
    MissingRequiredEvidence,
    /// Requested exact operation is not yet certified by the exact stack.
    UnsupportedExactOperation,
}

/// Source operand named by a kernel blocker, when the blocker comes from a mesh pair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactMeshSourceSide {
    /// The left/input-first mesh.
    Left,
    /// The right/input-second mesh.
    Right,
}

/// One fatal validation or import blocker.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactMeshBlocker {
    /// Stable category.
    pub(crate) kind: ExactMeshBlockerKind,
    /// Human-readable detail.
    pub(crate) message: String,
    /// Optional vertex index.
    pub(crate) vertex: Option<usize>,
    /// Optional face index.
    pub(crate) face: Option<usize>,
    /// Optional coordinate index in a flat coordinate buffer.
    pub(crate) coordinate: Option<usize>,
    /// Optional undirected edge endpoints.
    pub(crate) edge: Option<[usize; 2]>,
    /// Optional source operand for pair-stage blockers.
    pub(crate) source_side: Option<ExactMeshSourceSide>,
}

impl ExactMeshBlocker {
    /// Build a blocker with no object location.
    pub(crate) fn new(kind: ExactMeshBlockerKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
            vertex: None,
            face: None,
            coordinate: None,
            edge: None,
            source_side: None,
        }
    }

    /// Attach a source operand side.
    pub(crate) const fn with_source_side(mut self, source_side: ExactMeshSourceSide) -> Self {
        self.source_side = Some(source_side);
        self
    }

    /// Attach a vertex index.
    pub(crate) const fn with_vertex(mut self, vertex: usize) -> Self {
        self.vertex = Some(vertex);
        self
    }

    /// Attach a face index.
    pub(crate) const fn with_face(mut self, face: usize) -> Self {
        self.face = Some(face);
        self
    }

    /// Attach a flat coordinate index.
    pub(crate) const fn with_coordinate(mut self, coordinate: usize) -> Self {
        self.coordinate = Some(coordinate);
        self
    }

    /// Attach an undirected edge.
    pub(crate) const fn with_edge(mut self, edge: [usize; 2]) -> Self {
        self.edge = Some(edge);
        self
    }

    /// Stable blocker category.
    pub const fn kind(&self) -> ExactMeshBlockerKind {
        self.kind
    }

    /// Human-readable blocker detail.
    pub fn message(&self) -> &str {
        &self.message
    }

    /// Retained vertex provenance, when the blocker names one vertex.
    pub const fn vertex(&self) -> Option<usize> {
        self.vertex
    }

    /// Retained face provenance, when the blocker names one face.
    pub const fn face(&self) -> Option<usize> {
        self.face
    }

    /// Flat coordinate-buffer provenance, when the blocker names one coordinate.
    pub const fn coordinate(&self) -> Option<usize> {
        self.coordinate
    }

    /// Retained undirected-edge provenance, when the blocker names one edge.
    pub const fn edge(&self) -> Option<[usize; 2]> {
        self.edge
    }

    /// Source operand provenance, when the blocker came from a mesh-pair stage.
    pub const fn source_side(&self) -> Option<ExactMeshSourceSide> {
        self.source_side
    }
}

/// Error returned when mesh construction has one or more fatal blockers.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactMeshError {
    /// Blockers collected before construction stopped.
    pub(crate) blockers: Vec<ExactMeshBlocker>,
}

impl ExactMeshError {
    /// Build an error from blockers.
    pub(crate) fn new(blockers: Vec<ExactMeshBlocker>) -> Self {
        Self { blockers }
    }

    /// Build an error containing one blocker.
    pub(crate) fn one(blocker: ExactMeshBlocker) -> Self {
        Self {
            blockers: vec![blocker],
        }
    }

    /// Blockers collected before construction stopped.
    pub fn blockers(&self) -> &[ExactMeshBlocker] {
        &self.blockers
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
