//! Borrowed exact views of retained mesh data.

use super::bounds::PreparedMeshBounds;
use super::error::ExactMeshError;
use super::intersection::{MeshFacePairClassification, classify_mesh_face_pair_unchecked};
use super::mesh::{ExactAffineTransform3, Triangle};
use super::{ExactMesh, ExactMeshValidationError};
use hyperlimit::Point3;
use hyperreal::Real;

/// Borrowed exact view of an [`ExactMesh`].
#[derive(Clone, Copy, Debug)]
pub struct ExactMeshRef<'a> {
    mesh: &'a ExactMesh,
}

/// Preferred borrowed exact mesh view type.
pub type MeshView<'a> = ExactMeshRef<'a>;

/// Borrowed face view.
#[derive(Clone, Copy, Debug)]
pub struct FaceRef<'a> {
    mesh: &'a ExactMesh,
    index: usize,
}

/// Borrowed triangle view.
#[derive(Clone, Copy, Debug)]
pub struct TriangleRef<'a> {
    mesh: &'a ExactMesh,
    index: usize,
}

/// Borrowed edge view.
#[derive(Clone, Copy, Debug)]
pub struct EdgeRef<'a> {
    mesh: &'a ExactMesh,
    index: usize,
}

/// Borrowed exact mesh view with prepared broad-phase acceleration facts.
#[derive(Debug)]
pub struct PreparedMeshView<'a> {
    view: ExactMeshRef<'a>,
    bounds: PreparedMeshBounds<'a>,
}

/// Borrowed pair view with replay-validated broad-phase acceleration facts.
#[derive(Debug)]
pub struct PreparedMeshPairView<'a, 'b> {
    left: ExactMeshRef<'a>,
    right: ExactMeshRef<'b>,
    candidate_pairs: Vec<[usize; 2]>,
}

/// Borrowed pair view with cached graph-driving face-pair classifications.
#[derive(Debug)]
pub(crate) struct PreparedMeshPairGraphClassifications<'a, 'b> {
    left: ExactMeshRef<'a>,
    right: ExactMeshRef<'b>,
    classifications: Vec<MeshFacePairClassification>,
}

impl<'a> ExactMeshRef<'a> {
    /// Borrow an exact mesh as a replayable view.
    pub const fn new(mesh: &'a ExactMesh) -> Self {
        Self { mesh }
    }

    /// Return the underlying mesh.
    pub const fn mesh(self) -> &'a ExactMesh {
        self.mesh
    }

    /// Return exact vertices.
    pub fn vertices(self) -> &'a [Point3] {
        self.mesh.vertices()
    }

    /// Return triangle index rows.
    pub fn triangles(self) -> &'a [Triangle] {
        self.mesh.triangles()
    }

    /// Retained vertex count.
    pub const fn vertex_count(self) -> usize {
        self.mesh.facts().mesh.vertex_count
    }

    /// Retained face count.
    pub const fn face_count(self) -> usize {
        self.mesh.facts().mesh.face_count
    }

    /// Retained undirected edge count.
    pub const fn edge_count(self) -> usize {
        self.mesh.facts().mesh.edge_count
    }

    /// Retained Euler characteristic `V - E + F`.
    pub const fn euler_characteristic(self) -> isize {
        self.mesh.facts().mesh.euler_characteristic
    }

    /// Retained boundary edge count.
    pub const fn boundary_edge_count(self) -> usize {
        self.mesh.facts().mesh.boundary_edges
    }

    /// Retained non-manifold edge count.
    pub const fn non_manifold_edge_count(self) -> usize {
        self.mesh.facts().mesh.non_manifold_edges
    }

    /// Retained non-manifold vertex-link count.
    pub const fn non_manifold_vertex_count(self) -> usize {
        self.mesh.facts().mesh.non_manifold_vertices
    }

    /// Retained degenerate triangle count.
    pub const fn degenerate_triangle_count(self) -> usize {
        self.mesh.facts().mesh.degenerate_triangles
    }

    /// Whether retained facts certify a closed two-manifold mesh.
    pub const fn is_closed_manifold(self) -> bool {
        self.mesh.facts().mesh.closed_manifold
    }

    /// Whether retained facts record exact rational coordinates for every vertex.
    pub const fn has_exact_rational_coordinates(self) -> bool {
        self.mesh.facts().mesh.fixed_coordinates_exact_rational
    }

    /// Replay retained bounds, topology facts, and provenance against the source mesh.
    pub fn validate_retained_state(self) -> Result<(), ExactMeshValidationError> {
        self.mesh.validate_retained_state()
    }

    /// Replay retained exact bounds against the source mesh.
    pub fn validate_retained_bounds(self) -> Result<(), ExactMeshValidationError> {
        self.mesh.validate_retained_bounds()
    }

    /// Borrow one face by index.
    pub fn face(self, index: usize) -> Option<FaceRef<'a>> {
        (index < self.mesh.triangles().len()).then_some(FaceRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Borrow one triangle by index.
    pub fn triangle(self, index: usize) -> Option<TriangleRef<'a>> {
        (index < self.mesh.triangles().len()).then_some(TriangleRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Borrow one retained edge by index.
    pub fn edge(self, index: usize) -> Option<EdgeRef<'a>> {
        (index < self.mesh.facts().edges.len()).then_some(EdgeRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Iterate borrowed faces.
    pub fn faces(self) -> impl Iterator<Item = FaceRef<'a>> + 'a {
        (0..self.mesh.triangles().len()).map(move |index| FaceRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Iterate borrowed triangles.
    pub fn triangle_refs(self) -> impl Iterator<Item = TriangleRef<'a>> + 'a {
        (0..self.mesh.triangles().len()).map(move |index| TriangleRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Iterate retained edges.
    pub fn edges(self) -> impl Iterator<Item = EdgeRef<'a>> + 'a {
        (0..self.mesh.facts().edges.len()).map(move |index| EdgeRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Prepare replay-validated broad-phase facts for repeated pair queries.
    pub fn prepare_broad_phase(self) -> Result<PreparedMeshView<'a>, ExactMeshValidationError> {
        self.validate_retained_bounds()?;
        Ok(PreparedMeshView {
            view: self,
            bounds: self.mesh.bounds().prepare(),
        })
    }

    /// Prepare replay-validated broad-phase facts for one repeated mesh pair.
    pub fn prepare_pair_broad_phase<'b>(
        self,
        right: ExactMeshRef<'b>,
    ) -> Result<PreparedMeshPairView<'a, 'b>, ExactMeshValidationError> {
        let left = self.prepare_broad_phase()?;
        let right = right.prepare_broad_phase()?;
        Ok(left.prepare_pair_broad_phase(&right))
    }

    /// Materialize this view after an exact affine transform.
    pub fn transform(self, transform: &ExactAffineTransform3) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.transform(transform)
    }

    /// Materialize this view after a row-major exact homogeneous affine transform.
    pub fn transform_by(self, matrix: [[Real; 4]; 4]) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.transform_by(matrix)
    }

    /// Materialize this view with every triangle orientation reversed.
    pub fn inverse(self) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.inverse()
    }

    /// Materialize the exact closed union of this view and `right`.
    pub fn union(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.union(right.mesh)
    }

    /// Materialize the exact closed intersection of this view and `right`.
    pub fn intersection(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.intersection(right.mesh)
    }

    /// Materialize the exact closed difference of this view minus `right`.
    pub fn difference(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.difference(right.mesh)
    }

    /// Materialize the exact closed symmetric difference of this view and `right`.
    pub fn xor(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.xor(right.mesh)
    }
}

impl<'a> PreparedMeshView<'a> {
    /// Return the underlying borrowed mesh view.
    pub const fn view(&self) -> ExactMeshRef<'a> {
        self.view
    }

    /// Prepare cached broad-phase candidate face pairs for one mesh pair.
    pub fn prepare_pair_broad_phase<'b>(
        &self,
        right: &PreparedMeshView<'b>,
    ) -> PreparedMeshPairView<'a, 'b> {
        let plan = self.bounds.candidate_face_pair_plan(&right.bounds);
        let mut candidate_pairs = Vec::with_capacity(plan.candidate_pair_capacity_hint());
        let result = self.bounds.try_visit_candidate_face_pairs_with_plan(
            &right.bounds,
            plan,
            &mut |pair| {
                candidate_pairs.push(pair);
                Ok::<(), ()>(())
            },
        );
        debug_assert!(result.is_ok());
        PreparedMeshPairView {
            left: self.view,
            right: right.view,
            candidate_pairs,
        }
    }
}

impl<'a, 'b> PreparedMeshPairView<'a, 'b> {
    /// Return the left mesh view.
    pub const fn left(&self) -> ExactMeshRef<'a> {
        self.left
    }

    /// Return the right mesh view.
    pub const fn right(&self) -> ExactMeshRef<'b> {
        self.right
    }

    /// Cached broad-phase candidate face pairs in left/right face-index order.
    pub fn candidate_face_pairs(&self) -> &[[usize; 2]] {
        &self.candidate_pairs
    }

    /// Classify cached broad-phase candidates that must continue to graph construction.
    pub(crate) fn classify_graph_face_pairs(&self) -> PreparedMeshPairGraphClassifications<'a, 'b> {
        let left = self.left.mesh();
        let right = self.right.mesh();
        let mut classifications = Vec::new();
        for [left_face, right_face] in self.candidate_pairs.iter().copied() {
            let classification =
                classify_mesh_face_pair_unchecked(left, left_face, right, right_face);
            if classification.needs_graph_construction() {
                classifications.push(classification);
            }
        }
        PreparedMeshPairGraphClassifications {
            left: self.left,
            right: self.right,
            classifications,
        }
    }
}

impl<'a, 'b> PreparedMeshPairGraphClassifications<'a, 'b> {
    /// Return the left mesh view.
    pub(crate) const fn left(&self) -> ExactMeshRef<'a> {
        self.left
    }

    /// Return the right mesh view.
    pub(crate) const fn right(&self) -> ExactMeshRef<'b> {
        self.right
    }

    /// Cached retained graph-driving classifications in left/right face-index order.
    pub(crate) fn classifications(&self) -> &[MeshFacePairClassification] {
        &self.classifications
    }
}

impl<'a> FaceRef<'a> {
    /// Face index in the source mesh.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Triangle vertex indices for this face.
    pub fn vertex_indices(self) -> [usize; 3] {
        self.mesh.triangles()[self.index].0
    }

    /// Retained exact oriented plane normal.
    pub fn plane_normal(self) -> &'a [Real; 3] {
        &self.mesh.facts().faces[self.index].plane.normal
    }

    /// Retained exact oriented plane offset.
    pub fn plane_offset(self) -> &'a Real {
        &self.mesh.facts().faces[self.index].plane.offset
    }

    /// Retained exact oriented plane coefficients.
    pub fn plane_coefficients(self) -> (&'a [Real; 3], &'a Real) {
        (self.plane_normal(), self.plane_offset())
    }

    /// Exact face vertices.
    pub fn vertices(self) -> [&'a Point3; 3] {
        triangle_vertices(self.mesh, self.vertex_indices())
    }
}

impl<'a> TriangleRef<'a> {
    /// Triangle index in the source mesh.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Triangle vertex indices.
    pub fn vertex_indices(self) -> [usize; 3] {
        self.mesh.triangles()[self.index].0
    }

    /// Retained exact oriented plane normal.
    pub fn plane_normal(self) -> &'a [Real; 3] {
        &self.mesh.facts().faces[self.index].plane.normal
    }

    /// Retained exact oriented plane offset.
    pub fn plane_offset(self) -> &'a Real {
        &self.mesh.facts().faces[self.index].plane.offset
    }

    /// Retained exact oriented plane coefficients.
    pub fn plane_coefficients(self) -> (&'a [Real; 3], &'a Real) {
        (self.plane_normal(), self.plane_offset())
    }

    /// Exact triangle vertices.
    pub fn vertices(self) -> [&'a Point3; 3] {
        triangle_vertices(self.mesh, self.vertex_indices())
    }
}

impl<'a> EdgeRef<'a> {
    /// Edge index in the retained edge-fact table.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Retained endpoint vertex indices.
    pub fn vertex_indices(self) -> [usize; 2] {
        self.mesh.facts().edges[self.index].vertices
    }

    /// Retained incident face count.
    pub fn incident_face_count(self) -> usize {
        self.mesh.facts().edges[self.index].incident_faces
    }

    /// Retained directed use counts for the canonical edge orientation.
    pub fn directed_use_counts(self) -> [usize; 2] {
        self.mesh.facts().edges[self.index].directed_uses
    }

    /// Whether retained facts classify this edge as a closed-manifold edge.
    pub fn is_closed_manifold_edge(self) -> bool {
        let facts = &self.mesh.facts().edges[self.index];
        facts.is_closed_manifold_edge()
    }

    /// Exact edge endpoints.
    pub fn vertices(self) -> [&'a Point3; 2] {
        let [a, b] = self.vertex_indices();
        [&self.mesh.vertices()[a], &self.mesh.vertices()[b]]
    }
}

fn triangle_vertices(mesh: &ExactMesh, triangle: [usize; 3]) -> [&Point3; 3] {
    let [a, b, c] = triangle;
    [
        &mesh.vertices()[a],
        &mesh.vertices()[b],
        &mesh.vertices()[c],
    ]
}
