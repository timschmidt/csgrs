//! Adapters from the transitional `csgrs::Mesh` carrier into `hypermesh`.
//!
//! `Mesh` stores hyperreal polygons while the CSG pipeline is being
//! ported. This module keeps the exact handoff explicit: triangles are
//! tessellated, vertices are deduplicated by canonical coordinate keys, and the
//! flat stream is handed to `hypermesh::ExactMesh` for topology
//! validation. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997),
//! <https://doi.org/10.1016/0925-7721(95)00040-2>: approximate input channels
//! may propose exact objects only through audited conversion boundaries.

use std::fmt::Debug;

use hashbrown::HashMap;
use hyperlattice::{Point3, Real, Vector3};

use crate::{
    polygon::Polygon,
    triangulated::{IndexedTriangleMesh3D, IndexedTriangulated3D, Triangulated3D},
    vertex::Vertex,
};

use super::Mesh;

/// Error returned while importing or materializing a transitional `csgrs::Mesh`
/// through hypermesh.
#[derive(Debug, thiserror::Error)]
pub enum HypermeshHandoffError {
    /// The mesh could not be imported or validated as an exact hypermesh.
    #[error("hypermesh exact import failed: {0}")]
    Mesh(::hypermesh::kernel::ExactMeshError),
    /// The exact boolean operation failed.
    #[error("hypermesh boolean operation failed: {0}")]
    Boolean(::hypermesh::kernel::ExactMeshError),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum HypermeshBooleanOp {
    Union,
    Difference,
    Intersection,
}

/// Primitive-float buffers ready for `hypermesh::ExactMesh` import.
///
/// The coordinates are a lossy boundary view over the current `csgrs` carrier;
/// consumers that need certified topology should construct an
/// [`hypermesh::ExactMesh`] from these rows instead of making decisions
/// on the raw `f64` stream.
#[derive(Clone, Debug, PartialEq)]
pub struct HypermeshBuffers {
    /// Flat `x, y, z` coordinate rows.
    pub positions: Vec<Real>,
    /// Flat triangle index rows.
    pub indices: Vec<usize>,
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
struct PositionKey([String; 3]);

impl PositionKey {
    fn new(position: &Point3) -> Self {
        Self([
            canonical_coordinate_key(&position.x),
            canonical_coordinate_key(&position.y),
            canonical_coordinate_key(&position.z),
        ])
    }
}

impl<M: Clone + Send + Sync + Debug> Mesh<M> {
    /// Build deduplicated flat triangle buffers for `hypermesh`.
    ///
    /// This differs from renderer/export buffers, which may intentionally keep
    /// per-face duplicate vertices for normal seams. Exact mesh validation needs
    /// topology-bearing vertex identity, so equal finite coordinates are
    /// canonicalized to shared indices before import into `hypermesh`.
    pub fn to_hypermesh_buffers(&self) -> HypermeshBuffers {
        let tri_mesh = self.triangulate();
        let mut positions = Vec::with_capacity(tri_mesh.polygons.len() * 9);
        let mut indices = Vec::with_capacity(tri_mesh.polygons.len() * 3);
        let mut vertex_indices = HashMap::<PositionKey, usize>::new();

        for polygon in &tri_mesh.polygons {
            for vertex in &polygon.vertices {
                let key = PositionKey::new(&vertex.position);
                let index = *vertex_indices.entry(key).or_insert_with(|| {
                    let index = positions.len() / 3;
                    positions.extend_from_slice(&[
                        canonical_coordinate(&vertex.position.x),
                        canonical_coordinate(&vertex.position.y),
                        canonical_coordinate(&vertex.position.z),
                    ]);
                    index
                });
                indices.push(index);
            }
        }

        HypermeshBuffers { positions, indices }
    }

    /// Convert this transitional `csgrs` mesh into an exact `hypermesh` mesh.
    ///
    /// The default constructor requires a closed two-manifold. Use
    /// [`Mesh::to_hypermesh_surface_exact`] for open surfaces that should be
    /// validated as boundary-allowed surface meshes.
    pub fn to_hypermesh_exact(
        &self,
    ) -> Result<::hypermesh::ExactMesh, ::hypermesh::kernel::ExactMeshError> {
        let buffers = self.to_hypermesh_buffers();
        ::hypermesh::ExactMesh::from_real_triangles(&buffers.positions, &buffers.indices)
    }

    /// Convert this mesh into a boundary-allowed exact `hypermesh` surface mesh.
    pub fn to_hypermesh_surface_exact(
        &self,
    ) -> Result<::hypermesh::ExactMesh, ::hypermesh::kernel::ExactMeshError> {
        let buffers = self.to_hypermesh_buffers();
        if !self.polygons.is_empty() && buffers.indices.is_empty() {
            return ::hypermesh::ExactMesh::from_real_surface_triangles(
                &buffers.positions,
                &[0],
            );
        }
        ::hypermesh::ExactMesh::from_real_surface_triangles(
            &buffers.positions,
            &buffers.indices,
        )
    }

    /// Compute a mesh boolean through exact `hypermesh`.
    ///
    /// The finite `csgrs` polygons are imported as audited exact meshes and the
    /// returned exact output is replayed as finite `Mesh` triangles. This keeps
    /// topology decisions on the exact side, with conversion boundary evidence
    /// preserved as required by Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), and
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>.
    pub(crate) fn boolean_via_hypermesh(
        &self,
        other: &Self,
        operation: HypermeshBooleanOp,
    ) -> Result<Self, HypermeshHandoffError> {
        let left = self
            .to_hypermesh_exact()
            .map_err(HypermeshHandoffError::Mesh)?;
        let right = other
            .to_hypermesh_exact()
            .map_err(HypermeshHandoffError::Mesh)?;
        let result = match operation {
            HypermeshBooleanOp::Union => left.union(&right),
            HypermeshBooleanOp::Difference => left.difference(&right),
            HypermeshBooleanOp::Intersection => left.intersection(&right),
        };
        let result = result.map_err(HypermeshHandoffError::Boolean)?;
        Ok(Self::from_hypermesh_exact(&result, self.metadata.clone()))
    }

    fn from_hypermesh_exact(mesh: &::hypermesh::ExactMesh, metadata: M) -> Self {
        let mut polygons = Vec::with_capacity(mesh.triangle_count());

        mesh.visit_triangles(|[a, b, c]| {
            polygons.push(Polygon::new(vec![a, b, c], metadata.clone()));
        });

        Self::from_polygons(&polygons, metadata)
    }
}

impl Triangulated3D for ::hypermesh::ExactMesh {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for triangle in self.triangle_indices() {
            let Some(a) = exact_vertex_position(self, triangle[0]) else {
                continue;
            };
            let Some(b) = exact_vertex_position(self, triangle[1]) else {
                continue;
            };
            let Some(c) = exact_vertex_position(self, triangle[2]) else {
                continue;
            };
            let normal = hyper_triangle_unit_normal(&a, &b, &c).unwrap_or_else(Vector3::z);
            f([
                Vertex::new(a, normal.clone()),
                Vertex::new(b, normal.clone()),
                Vertex::new(c, normal),
            ]);
        }
    }
}

impl IndexedTriangulated3D for ::hypermesh::ExactMesh {
    fn indexed_triangles(&self) -> IndexedTriangleMesh3D {
        let positions = self
            .vertices()
            .iter()
            .map(|point| Point3::new(point.x.clone(), point.y.clone(), point.z.clone()))
            .collect::<Vec<_>>();
        let mut normals = Vec::with_capacity(self.triangle_count());
        let mut faces = Vec::with_capacity(self.triangle_count());

        for (normal_index, triangle) in self.triangle_indices().enumerate() {
            if triangle.iter().any(|&index| index >= positions.len()) {
                continue;
            }
            let a = positions[triangle[0]].clone();
            let b = positions[triangle[1]].clone();
            let c = positions[triangle[2]].clone();
            let normal = hyper_triangle_unit_normal(&a, &b, &c).unwrap_or_else(Vector3::z);
            normals.push(normal);
            faces.push([
                (triangle[0], normal_index),
                (triangle[1], normal_index),
                (triangle[2], normal_index),
            ]);
        }

        IndexedTriangleMesh3D {
            positions,
            normals,
            faces,
        }
    }
}

fn canonical_coordinate(value: &Real) -> Real {
    if matches!(value.refine_sign_until(128), Some(hyperreal::RealSign::Zero)) {
        Real::zero()
    } else {
        value.clone()
    }
}

fn canonical_coordinate_key(value: &Real) -> String {
    canonical_coordinate(value).to_string()
}

fn exact_vertex_position(mesh: &::hypermesh::ExactMesh, vertex: usize) -> Option<Point3> {
    mesh.vertices()
        .get(vertex)
        .map(|point| Point3::new(point.x.clone(), point.y.clone(), point.z.clone()))
}

fn hyper_triangle_unit_normal(a: &Point3, b: &Point3, c: &Point3) -> Option<Vector3> {
    (b - a).unit_cross_checked(&(c - a)).ok()
}
