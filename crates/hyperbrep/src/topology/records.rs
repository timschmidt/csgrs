//! Private topology records and public read-only accessors.

use super::*;

/// Direction of an edge use relative to its edge's canonical parameterization.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Direction {
    /// Traverse from the edge's start vertex to its end vertex.
    Forward,
    /// Traverse from the edge's end vertex to its start vertex.
    Reversed,
}

impl Direction {
    /// Returns the opposite traversal direction.
    pub const fn reversed(self) -> Self {
        match self {
            Self::Forward => Self::Reversed,
            Self::Reversed => Self::Forward,
        }
    }
}

/// Orientation of a face or shell relative to its authored geometry.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Orientation {
    /// Use the authored orientation.
    Forward,
    /// Reverse the authored orientation.
    Reversed,
}

impl Orientation {
    /// Returns the opposite surface orientation.
    pub const fn reversed(self) -> Self {
        match self {
            Self::Forward => Self::Reversed,
            Self::Reversed => Self::Forward,
        }
    }
}

/// Exact location of a point relative to a solid.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SolidPointLocation {
    /// The point belongs to the solid interior.
    Inside,
    /// The point does not belong to the solid.
    Outside,
    /// The point lies on a face, edge, or vertex.
    Boundary,
}

/// Canonical topological vertex.
#[derive(Clone, Debug)]
pub struct Vertex {
    pub(super) point: GeometricPoint3,
}

impl Vertex {
    /// Returns the exact model-space point.
    pub const fn point(&self) -> &GeometricPoint3 {
        &self.point
    }

    pub(crate) fn represented_point(&self) -> Result<&Point3, GeometryError> {
        self.point
            .try_coordinates()
            .ok_or(GeometryError::UnrepresentablePoint)
    }
}

/// Canonical topological edge.
#[derive(Clone, Debug)]
pub struct Edge {
    pub(super) start: VertexId,
    pub(super) end: VertexId,
    pub(super) curve: Curve3Id,
    pub(super) domain: ParameterRange,
    pub(super) endpoint_certificate: bool,
}

impl Edge {
    /// Returns the canonical start vertex.
    pub const fn start(&self) -> VertexId {
        self.start
    }

    /// Returns the canonical end vertex.
    pub const fn end(&self) -> VertexId {
        self.end
    }

    /// Returns the spatial curve.
    pub const fn curve(&self) -> Curve3Id {
        self.curve
    }

    /// Returns the exact edge interval on its spatial curve.
    pub const fn domain(&self) -> &ParameterRange {
        &self.domain
    }
}

/// One oriented use of an edge by a wire.
#[derive(Clone, Debug)]
pub struct EdgeUse {
    pub(super) edge: EdgeId,
    pub(super) direction: Direction,
    pub(super) pcurve: PcurveId,
    pub(super) parameter_correspondence: ParameterCorrespondence,
    pub(super) image_certificate: bool,
}

impl EdgeUse {
    /// Returns the referenced edge.
    pub const fn edge(&self) -> EdgeId {
        self.edge
    }

    /// Returns the traversal direction.
    pub const fn direction(&self) -> Direction {
        self.direction
    }

    /// Returns the face-local pcurve.
    pub const fn pcurve(&self) -> PcurveId {
        self.pcurve
    }

    /// Returns the pcurve-to-edge parameter correspondence.
    pub const fn parameter_correspondence(&self) -> &ParameterCorrespondence {
        &self.parameter_correspondence
    }
}

/// Ordered closed boundary.
#[derive(Clone, Debug)]
pub struct Wire {
    pub(super) edge_uses: Vec<EdgeUseId>,
    pub(super) simplicity_certificate: bool,
}

impl Wire {
    /// Returns ordered edge uses.
    pub fn edge_uses(&self) -> &[EdgeUseId] {
        &self.edge_uses
    }
}

/// Boundary representation of one oriented surface region.
#[derive(Clone, Debug)]
pub(super) enum FaceBoundary {
    /// The complete closed support surface, with no artificial seam topology.
    WholeSurface,
    /// One outer wire and zero or more inner wires.
    Trimmed { outer: WireId, inner: Vec<WireId> },
}

/// Oriented surface region.
#[derive(Clone, Debug)]
pub struct Face {
    pub(super) surface: SurfaceId,
    pub(super) orientation: Orientation,
    pub(super) boundary: FaceBoundary,
}

impl Face {
    /// Returns the support surface.
    pub const fn surface(&self) -> SurfaceId {
        self.surface
    }

    /// Returns face orientation relative to its support surface.
    pub const fn orientation(&self) -> Orientation {
        self.orientation
    }

    /// Returns the outer boundary, or `None` for a complete closed surface.
    pub const fn outer(&self) -> Option<WireId> {
        match &self.boundary {
            FaceBoundary::WholeSurface => None,
            FaceBoundary::Trimmed { outer, .. } => Some(*outer),
        }
    }

    /// Returns inner boundaries. Complete closed surfaces have none.
    pub fn inner(&self) -> &[WireId] {
        match &self.boundary {
            FaceBoundary::WholeSurface => &[],
            FaceBoundary::Trimmed { inner, .. } => inner,
        }
    }

    /// Returns whether this face is the complete closed support surface.
    pub const fn is_whole_surface(&self) -> bool {
        matches!(self.boundary, FaceBoundary::WholeSurface)
    }

    pub(in crate::topology) fn boundary_wires(&self) -> impl Iterator<Item = &WireId> {
        let outer = match &self.boundary {
            FaceBoundary::WholeSurface => None,
            FaceBoundary::Trimmed { outer, .. } => Some(outer),
        };
        outer.into_iter().chain(self.inner())
    }
}

/// Edge-connected collection of oriented faces.
#[derive(Clone, Debug)]
pub struct Shell {
    pub(super) faces: Vec<FaceId>,
}

impl Shell {
    /// Returns shell faces.
    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }
}

/// Volumetric region bounded by one outer shell.
#[derive(Clone, Debug)]
pub struct Solid {
    pub(super) outer: ShellId,
    pub(super) voids: Vec<ShellId>,
}

impl Solid {
    /// Returns the outer shell.
    pub const fn outer(&self) -> ShellId {
        self.outer
    }

    /// Returns void shells.
    pub fn voids(&self) -> &[ShellId] {
        &self.voids
    }
}

/// Counts of canonical model records.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ModelCounts {
    /// Number of vertices.
    pub vertices: usize,
    /// Number of spatial curves.
    pub curves: usize,
    /// Number of pcurves.
    pub pcurves: usize,
    /// Number of surfaces.
    pub surfaces: usize,
    /// Number of edges.
    pub edges: usize,
    /// Number of edge uses.
    pub edge_uses: usize,
    /// Number of wires.
    pub wires: usize,
    /// Number of faces.
    pub faces: usize,
    /// Number of shells.
    pub shells: usize,
    /// Number of solids.
    pub solids: usize,
}
