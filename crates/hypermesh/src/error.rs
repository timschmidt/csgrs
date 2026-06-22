//! Diagnostics for exact mesh import and validation.

use std::fmt;

/// Diagnostic severity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Severity {
    /// Informational note.
    Info,
    /// Suspicious but accepted input.
    Warning,
    /// Rejected input or invalid topology.
    Error,
}

/// Stable category for a mesh diagnostic.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DiagnosticKind {
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
    /// Requested exact operation is not yet certified by the exact stack.
    UnsupportedExactOperation,
}

/// One validation or import diagnostic.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MeshDiagnostic {
    /// Severity.
    pub severity: Severity,
    /// Stable category.
    pub kind: DiagnosticKind,
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

impl MeshDiagnostic {
    /// Build a diagnostic with no object location.
    pub fn new(severity: Severity, kind: DiagnosticKind, message: impl Into<String>) -> Self {
        Self {
            severity,
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

/// Error returned when mesh construction has one or more fatal diagnostics.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MeshError {
    /// Diagnostics collected before construction stopped.
    pub diagnostics: Vec<MeshDiagnostic>,
}

impl MeshError {
    /// Build an error from diagnostics.
    pub fn new(diagnostics: Vec<MeshDiagnostic>) -> Self {
        Self { diagnostics }
    }

    /// Build an error containing one diagnostic.
    pub fn one(diagnostic: MeshDiagnostic) -> Self {
        Self {
            diagnostics: vec![diagnostic],
        }
    }
}

impl fmt::Display for MeshError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.diagnostics.as_slice() {
            [] => write!(f, "mesh validation failed"),
            [diagnostic] => write!(f, "{}", diagnostic.message),
            diagnostics => write!(f, "{} mesh diagnostics", diagnostics.len()),
        }
    }
}

impl std::error::Error for MeshError {}

/// Public exact-kernel error name for mesh construction and materialization.
///
/// This aliases the existing diagnostic container so downstream code can move
/// to exact-kernel naming without losing source-compatible diagnostics.
pub type ExactMeshError = MeshError;

/// Public exact-kernel blocker name for a single retained diagnostic.
///
/// Blockers carry stable categories plus source vertex, face, edge, or flat
/// coordinate locations when the failing kernel stage can identify them.
pub type ExactMeshBlocker = MeshDiagnostic;
