use hyperbrep::{EdgeUseRef, FaceRef, SolidRef, VertexRef, WireRef};

/// One explicitly selected native solid occurrence, with its output name.
#[derive(Clone, Copy, Debug)]
pub struct FacetedBody<'a> {
    /// Source body; coordinates are interpreted as metres.
    pub solid: SolidRef<'a>,
    /// Name encoded in the output FACETED_BREP entity.
    pub name: &'a str,
}
/// A native vertex and its exact CARTESIAN_POINT encoding.
#[derive(Clone, Copy, Debug)]
pub struct WrittenVertex<'a> {
    /// Source identity in this body's native model.
    pub source: VertexRef<'a>,
    /// Output point entity ID.
    pub entity: u64,
}
/// A face reparameterized onto an exact boundary-derived plane frame.
#[derive(Clone, Copy, Debug)]
pub struct WrittenFace<'a> {
    /// Original native face, including its authored parameterization.
    pub source: FaceRef<'a>,
    /// Output FACE_SURFACE entity ID.
    pub entity: u64,
    /// Output PLANE entity ID. Its frame follows the directed outer boundary.
    pub plane: u64,
}
/// An exact wire encoded as an implicitly closed polygon.
#[derive(Clone, Copy, Debug)]
pub struct WrittenWire<'a> {
    /// Original native wire.
    pub source: WireRef<'a>,
    /// Output POLY_LOOP ID.
    pub polygon: u64,
    /// Output FACE_OUTER_BOUND or FACE_BOUND ID.
    pub bound: u64,
}
/// A native directed edge use encoded as an implicit straight polygon segment.
#[derive(Clone, Copy, Debug)]
pub struct WrittenEdgeUse<'a> {
    /// Source use, including spatial edge identity and original parameter relation.
    pub source: EdgeUseRef<'a>,
    /// Output POLY_LOOP ID.
    pub polygon: u64,
    /// Zero-based segment index; the last segment closes the polygon.
    pub segment: usize,
}
/// Complete source accounting for one selected body occurrence.
#[derive(Debug)]
pub struct WrittenBody<'a> {
    pub(super) source: SolidRef<'a>,
    pub(super) entity: u64,
    pub(super) shell: u64,
    pub(super) vertices: Vec<WrittenVertex<'a>>,
    pub(super) faces: Vec<WrittenFace<'a>>,
    pub(super) wires: Vec<WrittenWire<'a>>,
    pub(super) edge_uses: Vec<WrittenEdgeUse<'a>>,
}
impl<'a> WrittenBody<'a> {
    /// Native source; repeated input occurrences remain distinct output bodies.
    pub fn source(&self) -> SolidRef<'a> {
        self.source
    }
    /// Output FACETED_BREP ID.
    pub fn entity(&self) -> u64 {
        self.entity
    }
    /// Output CLOSED_SHELL ID.
    pub fn shell(&self) -> u64 {
        self.shell
    }
    /// Every used source vertex and shared output point, in first-use order.
    pub fn vertices(&self) -> &[WrittenVertex<'a>] {
        &self.vertices
    }
    /// Every source face and exact output plane; native UV charts are reparameterized.
    pub fn faces(&self) -> &[WrittenFace<'a>] {
        &self.faces
    }
    /// Every outer and inner wire in face traversal order.
    pub fn wires(&self) -> &[WrittenWire<'a>] {
        &self.wires
    }
    /// Every source edge use, including closing segments and both sides of shared edges.
    pub fn edge_uses(&self) -> &[WrittenEdgeUse<'a>] {
        &self.edge_uses
    }
}

/// Exact selected geometry plus a complete source-to-STEP encoding report.
///
/// Faceted STEP stores straight edges implicitly and uses new plane charts.
/// Native curve parameters, pcurves, history and unrelated document entities are
/// not serialized by this explicitly selected geometry operation. Retain a native
/// archive when those representations must survive unchanged.
#[derive(Debug)]
pub struct FacetedStep<'a> {
    pub(super) text: String,
    pub(super) representation: u64,
    pub(super) bodies: Vec<WrittenBody<'a>>,
}
impl<'a> FacetedStep<'a> {
    /// Complete deterministic Part 21 exchange, with one faceted representation/product.
    pub fn text(&self) -> &str {
        &self.text
    }
    /// Releases the independent output string and drops borrowed source accounting.
    pub fn into_text(self) -> String {
        self.text
    }
    /// Output FACETED_BREP_SHAPE_REPRESENTATION ID, suitable for selected re-import.
    pub fn representation(&self) -> u64 {
        self.representation
    }
    /// Source encoding reports in input occurrence order.
    pub fn bodies(&self) -> &[WrittenBody<'a>] {
        &self.bodies
    }
}
