use super::ContextUnits;
use hyperbrep::{
    EdgeKey, EdgeRef, EdgeUseKey, EdgeUseRef, FaceKey, FaceRef, Model, Point2, Real, SolidKey,
    SolidRef, Vector2, VertexKey, VertexRef, WireKey,
};

/// The source identity authorizing a native vertex.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VertexSource {
    /// A faceted polygon point; exact coordinate aliases may share a vertex.
    CartesianPoint {
        /// Original CARTESIAN_POINT ID.
        point: u64,
    },
    /// Explicit STEP topology; distinct vertex entities are never welded.
    VertexPoint {
        /// Original VERTEX_POINT ID.
        vertex: u64,
        /// Referenced CARTESIAN_POINT ID.
        point: u64,
    },
}
/// A mapped source point/topological vertex and its model-bound native vertex.
#[derive(Clone, Copy, Debug)]
pub struct SourceVertex<'a> {
    /// Original point and, when present, explicit topological vertex identity.
    pub source: VertexSource,
    /// Native vertex in this imported body.
    pub vertex: VertexRef<'a>,
}
/// A mapped source face and its model-bound native face.
#[derive(Clone, Copy, Debug)]
pub struct SourceFace<'a> {
    /// Original FACE_SURFACE or ADVANCED_FACE ID.
    pub source: u64,
    /// Native face in this imported body.
    pub face: FaceRef<'a>,
    /// Original support identity and complete native-to-source parameter map.
    pub parameters: &'a SourceSurfaceParameters,
}
/// Exact affine map from a native surface chart to its authored STEP chart.
///
/// Angular coordinates use the source context's plane-angle unit. Plane
/// coordinates and cylinder V use its length unit. A revolved LINE's V is the
/// original line parameter, including its magnitude, sign and shifted origin.
/// Spline extrusions retain generator U and V multiplying the complete source
/// extrusion vector; length conversion does not rescale these parameters.
/// Periodic lifts are retained; this metadata neither clamps parameters nor
/// asserts membership in the finite face.
#[derive(Clone, Debug)]
pub struct SourceSurfaceParameters {
    /// Original support-surface entity ID. Different native cone branches may
    /// share this ID while retaining different maps.
    pub surface: u64,
    /// Source UV corresponding to native (0, 0).
    pub origin: Point2,
    /// Source UV change for one native U unit.
    pub u: Vector2,
    /// Source UV change for one native V unit.
    pub v: Vector2,
}
impl SourceSurfaceParameters {
    /// Maps native parameters without rounding or periodic reduction.
    pub fn map(&self, point: &Point2) -> Point2 {
        self.origin.clone() + self.u.clone() * &point.x + self.v.clone() * &point.y
    }
}
#[derive(Debug)]
pub(super) struct FaceSource {
    pub source: u64,
    pub key: FaceKey,
    pub parameters: SourceSurfaceParameters,
}
/// Source description of one directed boundary use.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EdgeUseSource {
    /// A segment of a source POLY_LOOP, including its closing segment.
    PolygonSegment {
        /// Zero-based segment in the original polygon, including closing segment.
        segment: usize,
        /// Directed start CARTESIAN_POINT ID after bound orientation.
        from_point: u64,
        /// Directed end CARTESIAN_POINT ID after bound orientation.
        to_point: u64,
    },
    /// An explicit edge use after applying the bound's orientation.
    OrientedEdge {
        /// Original ORIENTED_EDGE ID.
        oriented_edge: u64,
        /// Referenced EDGE_CURVE ID.
        edge_curve: u64,
        /// Directed start VERTEX_POINT ID after bound orientation.
        from_vertex: u64,
        /// Directed end VERTEX_POINT ID after bound orientation.
        to_vertex: u64,
    },
}
/// Source boundary use and its bound native face-local edge use.
#[derive(Clone, Copy, Debug)]
pub struct SourceEdgeUse<'a> {
    /// FACE_BOUND or FACE_OUTER_BOUND ID.
    pub bound: u64,
    /// POLY_LOOP or EDGE_LOOP ID.
    pub loop_id: u64,
    /// Original segment/oriented edge with directed source endpoint identities.
    pub source: EdgeUseSource,
    /// Native face-local use in the imported body.
    pub edge_use: EdgeUseRef<'a>,
}
#[derive(Debug)]
pub(super) struct UseSource {
    pub(super) bound: u64,
    pub(super) loop_id: u64,
    pub(super) source: EdgeUseSource,
    pub(super) key: EdgeUseKey,
}
/// Exact affine map from the directed edge fraction `s` in `[0,1]` to a source
/// curve parameter: `start + (end - start) * s`. For native parameter `t`,
/// `s = (t - edge.domain.start) / (edge.domain.end - edge.domain.start)`.
/// Native lines use `[0,1]`; native circles use an increasing angular interval.
#[derive(Clone, Debug)]
pub struct SourceCurveParameters {
    /// Source LINE, CIRCLE, B-spline, PCURVE, SURFACE_CURVE or SEAM_CURVE ID.
    pub curve: u64,
    /// Parameter at the original EDGE_CURVE start vertex, in source units.
    pub start: Real,
    /// Parameter at the original EDGE_CURVE end vertex; may be less than start.
    /// A closed circle retains a distinct endpoint one exact source period away.
    pub end: Real,
}
/// An explicit source EDGE_CURVE and its independently checked representations.
#[derive(Clone, Copy, Debug)]
pub struct SourceEdge<'a> {
    /// Original EDGE_CURVE ID.
    pub source: u64,
    /// Original edge_geometry ID.
    pub geometry: u64,
    /// Source curve supplying the edge geometry's parameterization.
    pub master: u64,
    /// Complete source curve parameter maps, including every supplied pcurve.
    pub parameters: &'a [SourceCurveParameters],
    /// Native edge, directed from the original start to the original end vertex.
    pub edge: EdgeRef<'a>,
}
#[derive(Debug)]
pub(super) struct EdgeSource {
    pub source: u64,
    pub geometry: u64,
    pub master: u64,
    pub parameters: Vec<SourceCurveParameters>,
    pub key: EdgeKey,
}
/// One strictly validated body, with source ownership retained outside the kernel.
#[derive(Debug)]
pub struct ImportedBody {
    pub(super) source: u64,
    pub(super) name: String,
    pub(super) model: Model,
    pub(super) solid: SolidKey,
    pub(super) vertices: Vec<(VertexSource, VertexKey)>,
    pub(super) faces: Vec<FaceSource>,
    pub(super) loops: Vec<(u64, u64, WireKey)>,
    pub(super) edge_uses: Vec<UseSource>,
    pub(super) edges: Vec<EdgeSource>,
}
impl ImportedBody {
    /// Source FACETED_BREP or MANIFOLD_SOLID_BREP ID.
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
        self.faces.iter().map(|item| SourceFace {
            source: item.source,
            face: self.model.face(&item.key).expect("published key"),
            parameters: &item.parameters,
        })
    }
    /// Bound and loop source IDs with their native wire, including holes.
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
            loop_id: item.loop_id,
            source: item.source,
            edge_use: self.model.edge_use(&item.key).expect("published key"),
        })
    }
    /// Explicit EDGE_CURVE records in first-use order. Faceted polygons express
    /// edges implicitly and retain their evidence in [`Self::edge_uses`].
    pub fn explicit_edges(&self) -> impl ExactSizeIterator<Item = SourceEdge<'_>> {
        self.edges.iter().map(|item| SourceEdge {
            source: item.source,
            geometry: item.geometry,
            master: item.master,
            parameters: &item.parameters,
            edge: self.model.edge(&item.key).expect("published key"),
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
    /// Selected supported shape representation ID.
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
