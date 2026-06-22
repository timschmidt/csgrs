//! Borrowed exact views of retained mesh data.

use super::bounds::{MeshBounds, PreparedMeshBounds};
use super::error::MeshError;
use super::facts::{EdgeFacts, FaceFacts, FacePlaneFacts, MeshValidationFacts};
use super::mesh::{ExactAffineTransform3, Triangle};
use super::validation::ValidationPolicy;
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
#[derive(Clone, Debug)]
pub struct PreparedMeshView<'a> {
    view: ExactMeshRef<'a>,
    bounds: PreparedMeshBounds<'a>,
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

    /// Return exact mesh bounds.
    pub const fn bounds(self) -> &'a MeshBounds {
        self.mesh.bounds()
    }

    /// Return retained validation facts.
    pub const fn facts(self) -> &'a MeshValidationFacts {
        self.mesh.facts()
    }

    /// Return the validation policy attached to this mesh.
    pub const fn validation_policy(self) -> ValidationPolicy {
        self.mesh.validation_policy()
    }

    /// Replay retained bounds, topology facts, and provenance against the source mesh.
    pub fn validate_retained_state(self) -> Result<(), ExactMeshValidationError> {
        self.mesh.validate_retained_state()
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

    /// Return exact broad-phase candidate face pairs for this view and `right`.
    ///
    /// The retained bounds on both source meshes are replayed before the AABB
    /// scheduler is allowed to discard face pairs. Returned pairs are only
    /// broad-phase candidates; narrow exact predicates still decide topology.
    pub fn candidate_face_pairs(
        self,
        right: ExactMeshRef<'_>,
    ) -> Result<Vec<[usize; 2]>, ExactMeshValidationError> {
        let left = self.prepare_broad_phase()?;
        let right = right.prepare_broad_phase()?;
        Ok(left.candidate_face_pairs(&right))
    }

    /// Prepare replay-validated broad-phase facts for repeated pair queries.
    pub fn prepare_broad_phase(self) -> Result<PreparedMeshView<'a>, ExactMeshValidationError> {
        self.validate_retained_state()?;
        Ok(PreparedMeshView {
            view: self,
            bounds: self.mesh.bounds().prepare(),
        })
    }

    /// Materialize this view after an exact affine transform.
    pub fn transform(self, transform: &ExactAffineTransform3) -> Result<ExactMesh, MeshError> {
        self.mesh.transform(transform)
    }

    /// Materialize this view after a row-major exact homogeneous affine transform.
    pub fn transform_by(self, matrix: [[Real; 4]; 4]) -> Result<ExactMesh, MeshError> {
        self.mesh.transform_by(matrix)
    }

    /// Materialize this view with every triangle orientation reversed.
    pub fn inverse(self) -> Result<ExactMesh, MeshError> {
        self.mesh.inverse()
    }

    /// Materialize the exact closed union of this view and `right`.
    pub fn union(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, MeshError> {
        self.mesh.union(right.mesh)
    }

    /// Materialize the exact closed intersection of this view and `right`.
    pub fn intersection(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, MeshError> {
        self.mesh.intersection(right.mesh)
    }

    /// Materialize the exact closed difference of this view minus `right`.
    pub fn difference(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, MeshError> {
        self.mesh.difference(right.mesh)
    }

    /// Materialize the exact closed symmetric difference of this view and `right`.
    pub fn xor(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, MeshError> {
        self.mesh.xor(right.mesh)
    }
}

impl<'a> PreparedMeshView<'a> {
    /// Return the underlying borrowed mesh view.
    pub const fn view(&self) -> ExactMeshRef<'a> {
        self.view
    }

    /// Return prepared retained bounds.
    pub const fn bounds(&self) -> &PreparedMeshBounds<'a> {
        &self.bounds
    }

    /// Return exact broad-phase candidate face pairs for this view and `right`.
    pub fn candidate_face_pairs(&self, right: &PreparedMeshView<'_>) -> Vec<[usize; 2]> {
        self.bounds.candidate_face_pairs(&right.bounds)
    }
}

impl<'a> FaceRef<'a> {
    /// Face index in the source mesh.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Triangle row for this face.
    pub fn triangle(self) -> &'a Triangle {
        &self.mesh.triangles()[self.index]
    }

    /// Retained face facts.
    pub fn facts(self) -> &'a FaceFacts {
        &self.mesh.facts().faces[self.index]
    }

    /// Retained exact oriented face plane.
    pub fn plane(self) -> &'a FacePlaneFacts {
        &self.facts().plane
    }

    /// Exact face vertices.
    pub fn vertices(self) -> [&'a Point3; 3] {
        triangle_vertices(self.mesh, self.triangle())
    }
}

impl<'a> TriangleRef<'a> {
    /// Triangle index in the source mesh.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Triangle row.
    pub fn triangle(self) -> &'a Triangle {
        &self.mesh.triangles()[self.index]
    }

    /// Retained face facts for this triangle.
    pub fn facts(self) -> &'a FaceFacts {
        &self.mesh.facts().faces[self.index]
    }

    /// Retained exact oriented face plane.
    pub fn plane(self) -> &'a FacePlaneFacts {
        &self.facts().plane
    }

    /// Exact triangle vertices.
    pub fn vertices(self) -> [&'a Point3; 3] {
        triangle_vertices(self.mesh, self.triangle())
    }
}

impl<'a> EdgeRef<'a> {
    /// Edge index in the retained edge-fact table.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Retained edge facts.
    pub fn facts(self) -> &'a EdgeFacts {
        &self.mesh.facts().edges[self.index]
    }

    /// Exact edge endpoints.
    pub fn vertices(self) -> [&'a Point3; 2] {
        let [a, b] = self.facts().vertices;
        [&self.mesh.vertices()[a], &self.mesh.vertices()[b]]
    }
}

fn triangle_vertices<'a>(mesh: &'a ExactMesh, triangle: &Triangle) -> [&'a Point3; 3] {
    let [a, b, c] = triangle.0;
    [
        &mesh.vertices()[a],
        &mesh.vertices()[b],
        &mesh.vertices()[c],
    ]
}
