//! Fact carriers for exact hypermesh objects.
//!
//! These are retained structural facts, not proofs by themselves. Exact
//! predicate reports are stored beside facts when a topological claim depends
//! on a predicate. This mirrors Yap's separation of geometric objects,
//! arithmetic kernels, and certified predicate decisions.

use super::provenance::PredicateUse;
use super::scalar::ExactReal;

/// Facts known for one mesh vertex.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VertexFacts {
    /// Vertex index in the exact mesh.
    pub index: usize,
    /// Whether all coordinates are exact rational values in `hyperreal`.
    pub fixed_coordinates_exact_rational: bool,
    /// Whether the point has known sparse coordinate support.
    pub sparse_support: bool,
    /// Number of incident faces.
    pub incident_faces: usize,
    /// Number of incident undirected edges.
    pub incident_edges: usize,
    /// Certified combinatorial shape of the vertex link.
    pub link: VertexLinkKind,
}

/// Combinatorial shape of one vertex link.
///
/// This is a topology fact, not a geometric predicate. CGAL-style
/// triangulation data structures validate manifoldness by the local shape of
/// each vertex star; see Boissonnat, Devillers, Pion, Teillaud, and Yvinec,
/// "Triangulations in CGAL," *Computational Geometry* 22.1-3 (2002). Hypermesh
/// stores the classification explicitly so boolean stages can reject a local
/// nonmanifold mutation before it becomes a global mesh.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VertexLinkKind {
    /// No incident faces.
    Isolated,
    /// Link is a single cycle, as expected for a closed 2-manifold vertex.
    Circle,
    /// Link is a single path, as expected for a boundary 2-manifold vertex.
    Disk,
    /// Link has multiple components, branching, or otherwise invalid degree.
    NonManifold,
}

/// Facts known for one undirected edge.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EdgeFacts {
    /// Canonical edge endpoints.
    pub vertices: [usize; 2],
    /// Incident face count.
    pub incident_faces: usize,
    /// Number of faces using each directed orientation.
    pub directed_uses: [usize; 2],
}

impl EdgeFacts {
    /// Return whether the edge has exactly two opposing incident faces.
    pub const fn is_closed_manifold_edge(&self) -> bool {
        self.incident_faces == 2 && self.directed_uses[0] == 1 && self.directed_uses[1] == 1
    }
}

/// Facts known for one triangle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TriangleFacts {
    /// Face index.
    pub face: usize,
    /// Vertex indices.
    pub vertices: [usize; 3],
    /// Whether predicate validation proved a non-degenerate triangle.
    pub non_degenerate: bool,
    /// Predicate certificates used while checking degeneracy.
    pub degeneracy_predicates: Vec<PredicateUse>,
}

/// Facts known for one oriented face.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OrientedFaceFacts {
    /// Directed triangle edges.
    pub directed_edges: [[usize; 2]; 3],
}

/// Exact oriented plane equation retained for one face.
///
/// The coefficients satisfy `normal.x * x + normal.y * y + normal.z * z +
/// offset = 0` for every source vertex on the face. Hypermesh deliberately
/// stores the unnormalized determinant form rather than a unit normal: Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), emphasizes retaining numerical structure so later predicates can
/// reuse exact object facts instead of re-deriving topology from rounded
/// representatives.
#[derive(Clone, Debug, PartialEq)]
pub struct FacePlaneFacts {
    /// Oriented plane normal from the indexed triangle order.
    pub normal: [ExactReal; 3],
    /// Exact plane offset.
    pub offset: ExactReal,
}

/// Facts known for one face.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceFacts {
    /// Triangle facts.
    pub triangle: TriangleFacts,
    /// Oriented edge facts.
    pub oriented: OrientedFaceFacts,
    /// Exact oriented plane equation.
    pub plane: FacePlaneFacts,
}

/// Topology and exactness facts for a whole mesh.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MeshFacts {
    /// Number of vertices.
    pub vertex_count: usize,
    /// Number of triangular faces.
    pub face_count: usize,
    /// Number of undirected edges.
    pub edge_count: usize,
    /// Euler characteristic `V - E + F`.
    pub euler_characteristic: isize,
    /// Number of boundary edges.
    pub boundary_edges: usize,
    /// Number of non-manifold undirected edges.
    pub non_manifold_edges: usize,
    /// Number of duplicate directed edges.
    pub duplicate_directed_edges: usize,
    /// Number of degenerate triangles.
    pub degenerate_triangles: usize,
    /// Number of nonmanifold vertex links.
    pub non_manifold_vertices: usize,
    /// Whether all accepted triangles and edges form a closed two-manifold.
    pub closed_manifold: bool,
    /// Whether all coordinates are exact rational values in `hyperreal`.
    pub fixed_coordinates_exact_rational: bool,
}

/// Expanded validation facts for vertices, edges, and faces.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshValidationFacts {
    /// Whole-mesh summary.
    pub mesh: MeshFacts,
    /// Per-vertex facts.
    pub vertices: Vec<VertexFacts>,
    /// Per-edge facts.
    pub edges: Vec<EdgeFacts>,
    /// Per-face facts.
    pub faces: Vec<FaceFacts>,
}
