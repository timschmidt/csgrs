use super::ContextUnits;
use hyperbrep::{
    EdgeUseKey, EdgeUseRef, FaceKey, FaceRef, Model, SolidKey, SolidRef, VertexKey, VertexRef,
    WireKey,
};

/// A mapped source Cartesian point and its model-bound native vertex.
#[derive(Clone, Copy, Debug)]
pub struct SourceVertex<'a> {
    /// Original CARTESIAN_POINT ID. Exact aliases share a native vertex.
    pub source: u64,
    /// Native vertex in this imported body.
    pub vertex: VertexRef<'a>,
}
/// A mapped source face and its model-bound native face.
#[derive(Clone, Copy, Debug)]
pub struct SourceFace<'a> {
    /// Original FACE_SURFACE ID.
    pub source: u64,
    /// Native face in this imported body.
    pub face: FaceRef<'a>,
}
/// Source polygon segment and its bound native face-local edge use.
#[derive(Clone, Copy, Debug)]
pub struct SourceEdgeUse<'a> {
    /// FACE_BOUND or FACE_OUTER_BOUND ID.
    pub bound: u64,
    /// POLY_LOOP ID.
    pub polygon: u64,
    /// Zero-based segment in the original polygon, including its closing segment.
    pub segment: usize,
    /// Source point at the start of the directed face traversal.
    pub from_point: u64,
    /// Source point at the end of the directed face traversal.
    pub to_point: u64,
    /// Native face-local use in the imported body.
    pub edge_use: EdgeUseRef<'a>,
}
#[derive(Debug)]
pub(super) struct UseSource {
    pub(super) bound: u64,
    pub(super) polygon: u64,
    pub(super) segment: usize,
    pub(super) from_point: u64,
    pub(super) to_point: u64,
    pub(super) key: EdgeUseKey,
}
/// One strictly validated body, with source ownership retained outside the kernel.
#[derive(Debug)]
pub struct ImportedBody {
    pub(super) source: u64,
    pub(super) name: String,
    pub(super) model: Model,
    pub(super) solid: SolidKey,
    pub(super) vertices: Vec<(u64, VertexKey)>,
    pub(super) faces: Vec<(u64, FaceKey)>,
    pub(super) loops: Vec<(u64, u64, WireKey)>,
    pub(super) edge_uses: Vec<UseSource>,
}
impl ImportedBody {
    /// Source FACETED_BREP ID.
    pub fn source(&self) -> u64 {
        self.source
    }
    /// Authored body name.
    pub fn name(&self) -> &str {
        &self.name
    }
    /// Native immutable model in metre coordinates.
    pub fn model(&self) -> &Model {
        &self.model
    }
    /// Bound solid reference; usable by ordinary HyperBRep queries and edits.
    pub fn solid(&self) -> SolidRef<'_> {
        self.model
            .solid(&self.solid)
            .expect("key belongs to published model")
    }
    /// Source points, including every exactly coincident source alias.
    pub fn vertices(&self) -> impl ExactSizeIterator<Item = SourceVertex<'_>> {
        self.vertices.iter().map(|(source, key)| SourceVertex {
            source: *source,
            vertex: self.model.vertex(key).expect("published key"),
        })
    }
    /// Source faces in shell order.
    pub fn faces(&self) -> impl ExactSizeIterator<Item = SourceFace<'_>> {
        self.faces.iter().map(|(source, key)| SourceFace {
            source: *source,
            face: self.model.face(key).expect("published key"),
        })
    }
    /// Bound and poly-loop source IDs with their native wire, including holes.
    pub fn loops(&self) -> impl ExactSizeIterator<Item = (u64, u64, hyperbrep::WireRef<'_>)> {
        self.loops.iter().map(|(bound, polygon, key)| {
            (
                *bound,
                *polygon,
                self.model.wire(key).expect("published key"),
            )
        })
    }
    /// Complete segment provenance, including reversed bounds and shared edges.
    pub fn edge_uses(&self) -> impl ExactSizeIterator<Item = SourceEdgeUse<'_>> {
        self.edge_uses.iter().map(|item| SourceEdgeUse {
            bound: item.bound,
            polygon: item.polygon,
            segment: item.segment,
            from_point: item.from_point,
            to_point: item.to_point,
            edge_use: self.model.edge_use(&item.key).expect("published key"),
        })
    }
}
/// Exact typed import of an explicitly selected representation.
///
/// This is not a whole-file or assembly import. Every unselected/unsupported
/// source record remains in the input [`StepFile`](super::StepFile) and is listed separately.
#[derive(Debug)]
pub struct ImportedRepresentation {
    pub(super) source: u64,
    pub(super) name: String,
    pub(super) units: ContextUnits,
    pub(super) bodies: Vec<ImportedBody>,
    pub(super) interpreted: Vec<u64>,
    pub(super) retained: Vec<u64>,
    pub(super) placements: Vec<u64>,
}
impl ImportedRepresentation {
    /// Selected FACETED_BREP_SHAPE_REPRESENTATION ID.
    pub fn source(&self) -> u64 {
        self.source
    }
    /// Authored representation name.
    pub fn name(&self) -> &str {
        &self.name
    }
    /// Source context, exact conversions and unapplied uncertainty metadata.
    pub fn units(&self) -> &ContextUnits {
        &self.units
    }
    /// Validated bodies in representation item order.
    pub fn bodies(&self) -> &[ImportedBody] {
        &self.bodies
    }
    /// Sorted distinct IDs interpreted while importing the selected representation.
    pub fn interpreted_entities(&self) -> &[u64] {
        &self.interpreted
    }
    /// Source-order IDs retained raw, including other representations/products.
    pub fn retained_entities(&self) -> &[u64] {
        &self.retained
    }
    /// Placement items validated as metadata. They do not transform other items.
    pub fn placement_items(&self) -> &[u64] {
        &self.placements
    }
}
