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
use hypermesh::prelude::{Manifold as HypermeshManifold, OpType as HypermeshOpType};

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
    Mesh(#[from] ::hypermesh::exact::MeshError),
    /// The exact mesh imported successfully, but package replay failed.
    #[error("hypermesh handoff package failed: {0:?}")]
    Package(::hypermesh::exact::ExactMeshHandoffPackageError),
}

impl From<::hypermesh::exact::ExactMeshHandoffPackageError> for HypermeshHandoffError {
    fn from(error: ::hypermesh::exact::ExactMeshHandoffPackageError) -> Self {
        Self::Package(error)
    }
}

/// Primitive-float buffers ready for `hypermesh::ExactMesh` import.
///
/// The coordinates are a lossy boundary view over the current `csgrs` carrier;
/// consumers that need certified topology should construct an
/// [`hypermesh::exact::ExactMesh`] from these rows instead of making decisions
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
    ) -> Result<::hypermesh::exact::ExactMesh, ::hypermesh::exact::MeshError> {
        self.to_hypermesh_exact_with_policy(::hypermesh::exact::ValidationPolicy::CLOSED)
    }

    /// Convert this mesh into an exact `hypermesh` mesh with an explicit policy.
    pub fn to_hypermesh_exact_with_policy(
        &self,
        policy: ::hypermesh::exact::ValidationPolicy,
    ) -> Result<::hypermesh::exact::ExactMesh, ::hypermesh::exact::MeshError> {
        let buffers = self.to_hypermesh_buffers();
        if !self.polygons.is_empty() && buffers.indices.is_empty() {
            return Err(::hypermesh::exact::MeshError::one(
                ::hypermesh::exact::MeshDiagnostic::new(
                    ::hypermesh::exact::Severity::Error,
                    ::hypermesh::exact::DiagnosticKind::DegenerateTriangle,
                    "csgrs mesh produced no finite triangles for hypermesh import",
                ),
            ));
        }
        ::hypermesh::exact::ExactMesh::from_real_triangles_with_policy(
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
    ) -> Result<::hypermesh::exact::ExactMeshHandoffPackage, HypermeshHandoffError> {
        self.to_hypermesh_handoff_package_with_policy(
            ::hypermesh::exact::ValidationPolicy::CLOSED,
        )
    }

    /// Build a report-bearing package for an open or closed surface consumer.
    ///
    /// This keeps callers that do not depend directly on `hypermesh` from
    /// naming validation policies just to request the query/export surface
    /// domain.
    pub fn to_hypermesh_surface_handoff_package(
        &self,
    ) -> Result<::hypermesh::exact::ExactMeshHandoffPackage, HypermeshHandoffError> {
        self.to_hypermesh_handoff_package_with_policy(
            ::hypermesh::exact::ValidationPolicy::ALLOW_BOUNDARY,
        )
    }

    /// Build a hypermesh handoff package with an explicit validation policy.
    pub fn to_hypermesh_handoff_package_with_policy(
        &self,
        policy: ::hypermesh::exact::ValidationPolicy,
    ) -> Result<::hypermesh::exact::ExactMeshHandoffPackage, HypermeshHandoffError> {
        let mesh = self.to_hypermesh_exact_with_policy(policy)?;
        Ok(::hypermesh::exact::ExactMeshHandoffPackage::from_mesh(&mesh)?)
    }

    /// Compute a mesh boolean through `hypermesh` instead of a csgrs-owned tree.
    ///
    /// `csgrs::Mesh` still exposes transitional finite polygon storage, so this
    /// method packages both operands through the `hypermesh` boundary, requests
    /// a report-bearing boolean, and reconstructs finite polygons only for the
    /// legacy `Mesh` API surface. The retained report makes the current
    /// primitive-float adapter explicit while exact solid booleans continue to
    /// migrate into `hypermesh`, following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997),
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>.
    pub(crate) fn boolean_via_hypermesh(
        &self,
        other: &Self,
        operation: HypermeshOpType,
    ) -> Result<Self, String> {
        let left = self.to_hypermesh_legacy_manifold()?;
        let right = other.to_hypermesh_legacy_manifold()?;
        let result = ::hypermesh::compute_boolean_with_report(&left, &right, operation)?;
        Ok(Self::from_hypermesh_legacy_manifold(
            &result.mesh,
            self.metadata.clone(),
        ))
    }

    fn to_hypermesh_legacy_manifold(&self) -> Result<HypermeshManifold, String> {
        let buffers = self.to_hypermesh_buffers();
        let positions = buffers
            .positions
            .iter()
            .map(|value| {
                value
                    .to_f64_lossy()
                    .filter(|value| value.is_finite())
                    .ok_or_else(|| {
                        "legacy hypermesh boolean requires finite f64 coordinates".to_string()
                    })
            })
            .collect::<Result<Vec<_>, _>>()?;
        HypermeshManifold::new(&positions, &buffers.indices)
    }

    fn from_hypermesh_legacy_manifold(manifold: &HypermeshManifold, metadata: M) -> Self {
        let mut polygons = Vec::with_capacity(manifold.nf);

        for face in manifold.hs.chunks_exact(3) {
            let a = legacy_manifold_position(manifold, face[0].tail);
            let b = legacy_manifold_position(manifold, face[1].tail);
            let c = legacy_manifold_position(manifold, face[2].tail);
            let normal = hyper_triangle_unit_normal(&a, &b, &c).unwrap_or_else(Vector3::z);
            polygons.push(Polygon::new(
                vec![
                    Vertex::new(a, normal.clone()),
                    Vertex::new(b, normal.clone()),
                    Vertex::new(c, normal),
                ],
                metadata.clone(),
            ));
        }

        Self::from_polygons(&polygons, metadata)
    }
}

impl Triangulated3D for ::hypermesh::exact::ExactMesh {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        let Ok(view) = self.approximate_f64_view() else {
            return;
        };

        for triangle in view.indices.chunks_exact(3) {
            let a = view_position(&view.positions, triangle[0]);
            let b = view_position(&view.positions, triangle[1]);
            let c = view_position(&view.positions, triangle[2]);
            let normal = hyper_triangle_unit_normal(&a, &b, &c).unwrap_or_else(Vector3::z);
            f([
                Vertex::new(a, normal.clone()),
                Vertex::new(b, normal.clone()),
                Vertex::new(c, normal),
            ]);
        }
    }
}

impl IndexedTriangulated3D for ::hypermesh::exact::ExactMesh {
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

fn view_position(positions: &[f64], vertex: usize) -> Point3 {
    let offset = vertex * 3;
    Point3::try_from_f64_array([
        positions[offset],
        positions[offset + 1],
        positions[offset + 2],
    ])
    .unwrap_or_else(|_| Point3::origin())
}

fn legacy_manifold_position(manifold: &HypermeshManifold, vertex: usize) -> Point3 {
    let point = manifold.ps[vertex];
    Point3::try_from_f64_array([point.x, point.y, point.z])
        .unwrap_or_else(|_| Point3::origin())
}

fn hyper_triangle_unit_normal(a: &Point3, b: &Point3, c: &Point3) -> Option<Vector3> {
    (b - a).unit_cross_checked(&(c - a)).ok()
}
