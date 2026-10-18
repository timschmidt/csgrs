use hyperbrep::{
    EdgeRef, EdgeUseRef, FaceRef, Orientation, Point2, Real, SolidRef, Vector2, VertexRef, WireRef,
};

/// A native vertex with its distinct topological and coordinate STEP entities.
#[derive(Clone, Copy, Debug)]
pub struct WrittenAdvancedVertex<'a> {
    /// Original model-bound identity.
    pub source: VertexRef<'a>,
    /// Output VERTEX_POINT ID.
    pub entity: u64,
    /// Output CARTESIAN_POINT ID.
    pub point: u64,
}

/// A native edge and the exact parameterization of its emitted LINE or CIRCLE.
#[derive(Clone, Debug)]
pub struct WrittenAdvancedEdge<'a> {
    /// Original model-bound edge, with its retained native parameter interval.
    pub source: EdgeRef<'a>,
    /// Output EDGE_CURVE ID.
    pub entity: u64,
    /// Output LINE, CIRCLE or rational B-spline ID.
    pub geometry: u64,
    /// Emitted curve parameter at the native edge's start vertex.
    pub parameter_start: Real,
    /// Emitted curve parameter at the native edge's end vertex.
    /// Interpolation uses the normalized native edge fraction. Negative traversal
    /// decreases this parameter; a full circle retains a complete unwrapped turn.
    pub parameter_end: Real,
}

/// Exact affine map from an authored native surface chart to the output chart.
///
/// The two vectors are the columns of the linear part. Offsets retain shifted
/// native parameters, including unwrapped circular angles and seam identities.
/// Plane output coordinates are in the selected length unit; cylinder U is in
/// radians and V is in that length unit. Both torus coordinates are in radians.
/// A conical surface of revolution uses radians and the emitted unit-line
/// parameter; its exact longitude/slant offsets remain in this map.
/// Spline extrusions retain the generator parameter and V multiplier of the
/// full extrusion vector. Length conversion changes the vector, not V.
/// This is encoding metadata, not a surface.
#[derive(Clone, Debug)]
pub struct WrittenSurfaceParameters {
    /// Output UV at native (0, 0), even when outside the finite source chart.
    pub origin: Point2,
    /// Output UV change for one native U unit.
    pub u: Vector2,
    /// Output UV change for one native V unit.
    pub v: Vector2,
}
impl WrittenSurfaceParameters {
    /// Maps a native parameter without converting through floating point.
    pub fn map(&self, point: &Point2) -> Point2 {
        self.origin.clone() + self.u.clone() * &point.x + self.v.clone() * &point.y
    }
}

/// An authored face and its emitted analytic support and chart map.
#[derive(Clone, Debug)]
pub struct WrittenAdvancedFace<'a> {
    /// Original model-bound face, including its orientation and pcurves.
    pub source: FaceRef<'a>,
    /// Output ADVANCED_FACE ID.
    pub entity: u64,
    /// Output PLANE, CYLINDRICAL_SURFACE, TOROIDAL_SURFACE or conical
    /// SURFACE_OF_REVOLUTION, or spline SURFACE_OF_LINEAR_EXTRUSION ID.
    pub surface: u64,
    /// Native-to-output surface parameter map.
    pub parameters: WrittenSurfaceParameters,
    /// Emitted orientation relative to the output surface's normal. Chart
    /// reversal can make this differ from the source face's orientation.
    pub orientation: Orientation,
}

/// An authored outer/inner wire and its explicit directed edge loop.
#[derive(Clone, Copy, Debug)]
pub struct WrittenAdvancedWire<'a> {
    /// Original model-bound wire.
    pub source: WireRef<'a>,
    /// Output EDGE_LOOP ID.
    pub entity: u64,
    /// Output FACE_OUTER_BOUND or FACE_BOUND ID.
    pub bound: u64,
}

/// One directed use of a shared edge, including repeated uses on a seam.
#[derive(Clone, Copy, Debug)]
pub struct WrittenAdvancedEdgeUse<'a> {
    /// Original use and its native pcurve/correspondence.
    pub source: EdgeUseRef<'a>,
    /// Output ORIENTED_EDGE ID; source direction is preserved.
    pub entity: u64,
}

/// Complete encoding report for one explicitly selected solid occurrence.
#[derive(Debug)]
pub struct WrittenAdvancedBody<'a> {
    pub(super) source: SolidRef<'a>,
    pub(super) entity: u64,
    pub(super) shell: u64,
    pub(super) vertices: Vec<WrittenAdvancedVertex<'a>>,
    pub(super) edges: Vec<WrittenAdvancedEdge<'a>>,
    pub(super) faces: Vec<WrittenAdvancedFace<'a>>,
    pub(super) wires: Vec<WrittenAdvancedWire<'a>>,
    pub(super) edge_uses: Vec<WrittenAdvancedEdgeUse<'a>>,
}
impl<'a> WrittenAdvancedBody<'a> {
    /// Original source solid in its immutable model.
    pub fn source(&self) -> SolidRef<'a> {
        self.source
    }
    /// Output MANIFOLD_SOLID_BREP ID.
    pub fn entity(&self) -> u64 {
        self.entity
    }
    /// Output CLOSED_SHELL ID.
    pub fn shell(&self) -> u64 {
        self.shell
    }
    /// Every used native vertex, in first-use order; coordinates do not weld IDs.
    pub fn vertices(&self) -> &[WrittenAdvancedVertex<'a>] {
        &self.vertices
    }
    /// Every used native edge and emitted carrier map, in first-use order.
    pub fn edges(&self) -> &[WrittenAdvancedEdge<'a>] {
        &self.edges
    }
    /// Every face in native shell order.
    pub fn faces(&self) -> &[WrittenAdvancedFace<'a>] {
        &self.faces
    }
    /// Every outer and inner wire in face traversal order.
    pub fn wires(&self) -> &[WrittenAdvancedWire<'a>] {
        &self.wires
    }
    /// Every directed use, including both sides of a shared edge or seam.
    pub fn edge_uses(&self) -> &[WrittenAdvancedEdgeUse<'a>] {
        &self.edge_uses
    }
}

/// Exact selected advanced geometry with source-to-output topology/parameter maps.
///
/// Direct spatial curves encode the boundary; native pcurves, unused geometry,
/// history and document records are not serialized. Keep a native archive when
/// those representations must survive. Each input occurrence remains distinct.
#[derive(Debug)]
pub struct AdvancedStep<'a> {
    pub(super) text: String,
    pub(super) representation: u64,
    pub(super) bodies: Vec<WrittenAdvancedBody<'a>>,
}
impl<'a> AdvancedStep<'a> {
    /// Complete deterministic Part 21 exchange.
    pub fn text(&self) -> &str {
        &self.text
    }
    /// Releases the independent text and drops borrowed source accounting.
    pub fn into_text(self) -> String {
        self.text
    }
    /// Output ADVANCED_BREP_SHAPE_REPRESENTATION ID for selected re-import.
    pub fn representation(&self) -> u64 {
        self.representation
    }
    /// Body encoding reports, in selected occurrence order.
    pub fn bodies(&self) -> &[WrittenAdvancedBody<'a>] {
        &self.bodies
    }
}
