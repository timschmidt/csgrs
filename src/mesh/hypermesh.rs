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

/// Error returned while packaging a transitional `csgrs::Mesh` for hypermesh consumers.
#[derive(Debug, thiserror::Error)]
pub enum HypermeshHandoffError {
    /// The mesh could not be imported or validated as an exact hypermesh.
    #[error("hypermesh exact import failed: {0}")]
    Mesh(#[from] ::hypermesh::MeshError),
    /// The exact mesh imported successfully, but package replay failed.
    #[error("hypermesh handoff package failed: {0:?}")]
    Package(::hypermesh::ExactMeshHandoffPackageError),
    /// The exact boolean operation failed.
    #[error("hypermesh boolean operation failed: {0}")]
    Boolean(String),
}

impl From<::hypermesh::ExactMeshHandoffPackageError> for HypermeshHandoffError {
    fn from(error: ::hypermesh::ExactMeshHandoffPackageError) -> Self {
        Self::Package(error)
    }
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
    /// The default policy requires a closed two-manifold. Use
    /// [`Mesh::to_hypermesh_exact_with_policy`] for open surfaces that should be
    /// validated as boundary-allowed surface meshes rather than closed solids.
    pub fn to_hypermesh_exact(
        &self,
    ) -> Result<::hypermesh::ExactMesh, ::hypermesh::MeshError> {
        self.to_hypermesh_exact_with_policy(::hypermesh::ValidationPolicy::CLOSED)
    }

    /// Convert this mesh into an exact `hypermesh` mesh with an explicit policy.
    pub fn to_hypermesh_exact_with_policy(
        &self,
        policy: ::hypermesh::ValidationPolicy,
    ) -> Result<::hypermesh::ExactMesh, ::hypermesh::MeshError> {
        let buffers = self.to_hypermesh_buffers();
        if !self.polygons.is_empty() && buffers.indices.is_empty() {
            return Err(::hypermesh::MeshError::one(::hypermesh::MeshDiagnostic::new(
                ::hypermesh::Severity::Error,
                ::hypermesh::DiagnosticKind::DegenerateTriangle,
                "csgrs mesh produced no finite triangles for hypermesh import",
            )));
        }
        ::hypermesh::ExactMesh::from_real_triangles_with_policy(
            &buffers.positions,
            &buffers.indices,
            policy,
        )
    }

    /// Build a report-bearing hypermesh handoff package for downstream consumers.
    ///
    /// This is the preferred boundary for systems that need more than raw
    /// triangles. The package includes retained-state audit, consumer-readiness
    /// routing, exact surface/solid reports when available, and an explicitly
    /// lossy primitive-float view for display/export. Keeping these artifacts
    /// bundled but replayable follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997),
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>: exact topology evidence
    /// and approximate representatives must not be collapsed into one implicit
    /// buffer contract.
    pub fn to_hypermesh_handoff_package(
        &self,
    ) -> Result<::hypermesh::ExactMeshHandoffPackage, HypermeshHandoffError> {
        self.to_hypermesh_handoff_package_with_policy(::hypermesh::ValidationPolicy::CLOSED)
    }

    /// Build a report-bearing package for an open or closed surface consumer.
    ///
    /// This keeps callers that do not depend directly on `hypermesh` from
    /// naming validation policies just to request the query/export surface
    /// domain.
    pub fn to_hypermesh_surface_handoff_package(
        &self,
    ) -> Result<::hypermesh::ExactMeshHandoffPackage, HypermeshHandoffError> {
        self.to_hypermesh_handoff_package_with_policy(
            ::hypermesh::ValidationPolicy::ALLOW_BOUNDARY,
        )
    }

    /// Build a hypermesh handoff package with an explicit validation policy.
    pub fn to_hypermesh_handoff_package_with_policy(
        &self,
        policy: ::hypermesh::ValidationPolicy,
    ) -> Result<::hypermesh::ExactMeshHandoffPackage, HypermeshHandoffError> {
        let mesh = self.to_hypermesh_exact_with_policy(policy)?;
        Ok(::hypermesh::ExactMeshHandoffPackage::from_mesh(&mesh)?)
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
        operation: ::hypermesh::ExactBooleanOperation,
    ) -> Result<Self, HypermeshHandoffError> {
        let left = self
            .to_hypermesh_exact_with_policy(::hypermesh::ValidationPolicy::CLOSED)
            .map_err(|error| HypermeshHandoffError::Mesh(error))?;
        let right = other
            .to_hypermesh_exact_with_policy(::hypermesh::ValidationPolicy::CLOSED)
            .map_err(|error| HypermeshHandoffError::Mesh(error))?;
        let result = ::hypermesh::boolean_exact_with_boundary_policy(
            &left,
            &right,
            operation,
            ::hypermesh::ValidationPolicy::CLOSED,
            ::hypermesh::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .map_err(|error| HypermeshHandoffError::Boolean(format!("{error:?}")))?;
        Ok(Self::from_hypermesh_exact(
            &result.mesh,
            self.metadata.clone(),
        ))
    }

    fn from_hypermesh_exact(mesh: &::hypermesh::ExactMesh, metadata: M) -> Self {
        let mut polygons = Vec::with_capacity(mesh.triangles().len());

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
        let Ok(view) = self.approximate_f64_view() else {
            return;
        };

        for triangle in view.indices.chunks_exact(3) {
            let Some(a) = view_position(&view.positions, triangle[0]) else {
                continue;
            };
            let Some(b) = view_position(&view.positions, triangle[1]) else {
                continue;
            };
            let Some(c) = view_position(&view.positions, triangle[2]) else {
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
        let Ok(view) = self.approximate_f64_view() else {
            return IndexedTriangleMesh3D {
                positions: Vec::new(),
                normals: Vec::new(),
                faces: Vec::new(),
            };
        };

        let positions = view
            .positions
            .chunks_exact(3)
            .map(|coords| {
                Point3::try_from_f64_array([coords[0], coords[1], coords[2]])
                    .unwrap_or_else(|_| Point3::origin())
            })
            .collect::<Vec<_>>();
        let mut normals = Vec::with_capacity(view.indices.len() / 3);
        let mut faces = Vec::with_capacity(view.indices.len() / 3);

        for (normal_index, triangle) in view.indices.chunks_exact(3).enumerate() {
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

fn view_position(positions: &[f64], vertex: usize) -> Option<Point3> {
    let offset = vertex.checked_mul(3)?;
    if offset + 2 >= positions.len() {
        return None;
    }
    Point3::try_from_f64_array([
        positions[offset],
        positions[offset + 1],
        positions[offset + 2],
    ])
    .ok()
}

fn hyper_triangle_unit_normal(a: &Point3, b: &Point3, c: &Point3) -> Option<Vector3> {
    (b - a).unit_cross_checked(&(c - a)).ok()
}
