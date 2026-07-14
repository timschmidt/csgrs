//! BREP topology carriers.
//!
//! These types are deliberately simple value records. Validation lives in the
//! report layer so topology can be transported between crates without silently
//! running repair or tolerance-merging logic.

use hyperlimit::Point3;

use crate::report::{BrepShellClosureReport, BrepTopologyValidationReport};
use crate::surface::{BrepSurface, BrepSurfaceId};

/// Stable vertex identifier.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct BrepVertexId(pub u64);

/// Stable edge identifier.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct BrepEdgeId(pub u64);

/// Stable loop identifier.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct BrepLoopId(pub u64);

/// Stable face identifier.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct BrepFaceId(pub u64);

/// Exact BREP vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepVertex {
    /// Stable vertex identifier.
    pub id: BrepVertexId,
    /// Exact model-space position.
    pub point: Point3,
}

impl BrepVertex {
    /// Construct an exact BREP vertex.
    pub const fn new(id: BrepVertexId, point: Point3) -> Self {
        Self { id, point }
    }
}

/// Topological edge between two exact vertices.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BrepEdge {
    /// Stable edge identifier.
    pub id: BrepEdgeId,
    /// Start vertex identifier for the edge's canonical direction.
    pub start: BrepVertexId,
    /// End vertex identifier for the edge's canonical direction.
    pub end: BrepVertexId,
}

impl BrepEdge {
    /// Construct a topological edge.
    pub const fn new(id: BrepEdgeId, start: BrepVertexId, end: BrepVertexId) -> Self {
        Self { id, start, end }
    }

    /// Returns whether the edge references the same vertex twice.
    pub const fn is_degenerate(self) -> bool {
        self.start.0 == self.end.0
    }
}

/// Orientation of an edge use inside a loop.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum BrepEdgeOrientation {
    /// Coedge follows the edge's canonical direction.
    Forward,
    /// Coedge traverses the edge opposite to its canonical direction.
    Reversed,
}

/// One oriented edge use in a trim/topology loop.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BrepCoedge {
    /// Referenced topological edge.
    pub edge: BrepEdgeId,
    /// Orientation of this use.
    pub orientation: BrepEdgeOrientation,
}

impl BrepCoedge {
    /// Construct an oriented edge use.
    pub const fn new(edge: BrepEdgeId, orientation: BrepEdgeOrientation) -> Self {
        Self { edge, orientation }
    }
}

/// A BREP loop made of oriented edge uses.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BrepLoop {
    /// Stable loop identifier.
    pub id: BrepLoopId,
    /// Ordered oriented edge uses.
    pub coedges: Vec<BrepCoedge>,
}

impl BrepLoop {
    /// Construct a BREP loop.
    pub fn new(id: BrepLoopId, coedges: Vec<BrepCoedge>) -> Self {
        Self { id, coedges }
    }

    /// Returns whether the loop has no edge uses.
    pub fn is_empty(&self) -> bool {
        self.coedges.is_empty()
    }
}

/// A face bound to one retained surface and one or more loops.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BrepFace {
    /// Stable face identifier.
    pub id: BrepFaceId,
    /// Referenced retained surface.
    pub surface: BrepSurfaceId,
    /// Outer trim/topology loop.
    pub outer: BrepLoop,
    /// Inner loops, if any.
    pub inner: Vec<BrepLoop>,
}

impl BrepFace {
    /// Construct a face with an outer loop and no holes.
    pub fn new(id: BrepFaceId, surface: BrepSurfaceId, outer: BrepLoop) -> Self {
        Self {
            id,
            surface,
            outer,
            inner: Vec::new(),
        }
    }

    /// Construct a face with an outer loop and inner loops.
    pub fn with_inner(
        id: BrepFaceId,
        surface: BrepSurfaceId,
        outer: BrepLoop,
        inner: Vec<BrepLoop>,
    ) -> Self {
        Self {
            id,
            surface,
            outer,
            inner,
        }
    }

    /// Iterate over all loops on the face.
    pub fn loops(&self) -> impl Iterator<Item = &BrepLoop> {
        core::iter::once(&self.outer).chain(self.inner.iter())
    }
}

/// Retained shell evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepShell {
    /// Exact vertices referenced by edges.
    pub vertices: Vec<BrepVertex>,
    /// Topological edges referenced by loops.
    pub edges: Vec<BrepEdge>,
    /// Retained surfaces referenced by faces.
    pub surfaces: Vec<BrepSurface>,
    /// Faces in this shell.
    pub faces: Vec<BrepFace>,
}

impl BrepShell {
    /// Validate shell topology and surface inventory without repairing it.
    ///
    /// The audit reports topological closure, nonmanifold edge use, invalid
    /// references, and supported-surface readiness. It does not infer missing
    /// geometry or merge nearby vertices. Oriented edge uses assemble loops,
    /// loops bound faces, and closed shells require paired manifold edge uses.
    pub fn audit_closure(&self) -> BrepShellClosureReport {
        BrepShellClosureReport::from_shell(self)
    }

    /// Validate retained topology graph identity and incidence.
    pub fn validate_topology(&self) -> BrepTopologyValidationReport {
        BrepTopologyValidationReport::from_shell(self)
    }
}
