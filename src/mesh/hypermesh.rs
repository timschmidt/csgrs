//! Adapters from the transitional `csgrs::Mesh` carrier into `hypermesh`.
//!
//! This module keeps the exact adapter boundary explicit: polygons are
//! tessellated, vertices are deduplicated by canonical coordinate keys, and the
//! flat stream is passed to `hypermesh::InputMesh` for boolean operations. This
//! follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997),
//! <https://doi.org/10.1016/0925-7721(95)00040-2>: approximate input channels
//! may propose exact objects only through audited conversion boundaries.

use std::collections::BTreeMap;
use std::fmt::Debug;

use hashbrown::HashMap;
use hyperlattice::{Point3, Real, Vector3};
use hypermesh::{BooleanOp, EmberConfig, InputMesh, Triangle, TriangleSoup};

use crate::{
    polygon::Polygon,
    triangulated::{IndexedTriangleMesh3D, IndexedTriangulated3D, Triangulated3D},
    vertex::Vertex,
};

use super::Mesh;

/// Error returned while importing or materializing a transitional `csgrs::Mesh`
/// through hypermesh.
#[derive(Debug, thiserror::Error)]
pub enum HypermeshError {
    /// The mesh could not be prepared by hypermesh.
    #[error("hypermesh exact import failed: {0}")]
    Mesh(::hypermesh::HypermeshError),
    /// The exact boolean operation failed.
    #[error("hypermesh boolean operation failed: {0}")]
    Boolean(::hypermesh::HypermeshError),
    /// The boolean output could not be triangulated for csgrs.
    #[error("hypermesh materialization failed: {0}")]
    Materialize(::hypermesh::HypermeshError),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum HypermeshBooleanOp {
    Union,
    Difference,
    Intersection,
    Xor,
}

/// Finite exact buffers ready for `hypermesh::InputMesh` import.
///
/// Rational coordinates remain exact. Non-rational computable coordinates are
/// sampled at the finite `csgrs` compatibility boundary and promoted to exact
/// binary rationals before they enter `hypermesh`.
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
            position.x.to_string(),
            position.y.to_string(),
            position.z.to_string(),
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
        let mut canonical_positions =
            Vec::<Point3>::with_capacity(tri_mesh.polygons.len() * 3);
        let mut indices = Vec::with_capacity(tri_mesh.polygons.len() * 3);
        let mut vertex_indices = HashMap::<PositionKey, Vec<usize>>::new();

        for polygon in &tri_mesh.polygons {
            for vertex in &polygon.vertices {
                let position = canonical_position(&vertex.position);
                let key = PositionKey::new(&position);
                let candidates = vertex_indices.entry(key).or_default();
                let index = candidates
                    .iter()
                    .copied()
                    .find(|index| canonical_positions[*index] == position)
                    .unwrap_or_else(|| {
                        let index = canonical_positions.len();
                        canonical_positions.push(position);
                        candidates.push(index);
                        index
                    });
                indices.push(index);
            }
        }

        let positions = canonical_positions
            .into_iter()
            .flat_map(|point| [point.x, point.y, point.z])
            .collect();

        HypermeshBuffers { positions, indices }
    }

    /// Convert this `csgrs` mesh into an exact `hypermesh` input mesh.
    ///
    /// The returned mesh is validated against hypermesh's closed-PWN input
    /// contract. Use [`Mesh::to_hypermesh_triangle_mesh`] when a caller only
    /// needs the indexed triangle carrier and does not intend to run a boolean.
    pub fn to_hypermesh_exact(&self) -> ::hypermesh::HypermeshResult<InputMesh> {
        let buffers = self.to_hypermesh_buffers();
        let mesh = input_mesh_from_buffers(&buffers);
        ::hypermesh::prepare_input(&[mesh.as_ref()])?;
        Ok(mesh)
    }

    /// Convert this mesh into a validated indexed `hypermesh` triangle carrier.
    ///
    /// This validates indices and source-triangle degeneracy but deliberately
    /// does not claim that open surfaces satisfy hypermesh's Boolean input
    /// contract.
    pub fn to_hypermesh_triangle_mesh(&self) -> ::hypermesh::HypermeshResult<InputMesh> {
        let buffers = self.to_hypermesh_buffers();
        let mesh = input_mesh_from_buffers(&buffers);
        if mesh.positions.is_empty() {
            return Ok(mesh);
        }
        validate_surface_input(&mesh)?;
        Ok(mesh)
    }

    /// Convert this mesh without requiring closed-PWN topology.
    #[deprecated(
        since = "0.23.0",
        note = "use to_hypermesh_triangle_mesh; open surfaces are not valid hypermesh Boolean inputs"
    )]
    pub fn to_hypermesh_surface_exact(&self) -> ::hypermesh::HypermeshResult<InputMesh> {
        self.to_hypermesh_triangle_mesh()
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
    ) -> Result<Self, HypermeshError> {
        let left = self.to_hypermesh_exact().map_err(HypermeshError::Mesh)?;
        let right = other.to_hypermesh_exact().map_err(HypermeshError::Mesh)?;
        let op = match operation {
            HypermeshBooleanOp::Union => BooleanOp::Union,
            HypermeshBooleanOp::Difference => BooleanOp::Difference,
            HypermeshBooleanOp::Intersection => BooleanOp::Intersection,
            HypermeshBooleanOp::Xor => BooleanOp::SymmetricDifference,
        };
        let refs = [left.as_ref(), right.as_ref()];
        let result = ::hypermesh::boolean_operation(&refs, op, EmberConfig::default())
            .map_err(HypermeshError::Boolean)?;
        let soup = ::hypermesh::triangulate_and_resolve_certified(&result)
            .map_err(HypermeshError::Materialize)?;
        Ok(Self::from_hypermesh_soup(&soup, self.metadata.clone()))
    }

    fn from_hypermesh_soup(soup: &TriangleSoup, metadata: M) -> Self {
        let mut polygons = Vec::with_capacity(soup.triangles.len());

        for triangle in &soup.triangles {
            let Some(a) = soup_vertex_position(soup, triangle[0]) else {
                continue;
            };
            let Some(b) = soup_vertex_position(soup, triangle[1]) else {
                continue;
            };
            let Some(c) = soup_vertex_position(soup, triangle[2]) else {
                continue;
            };
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

impl Triangulated3D for InputMesh {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for triangle in &self.triangles {
            let [ia, ib, ic] = triangle.indices();
            let Some(a) = self.positions.get(ia).cloned() else {
                continue;
            };
            let Some(b) = self.positions.get(ib).cloned() else {
                continue;
            };
            let Some(c) = self.positions.get(ic).cloned() else {
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

impl IndexedTriangulated3D for InputMesh {
    fn indexed_triangles(&self) -> IndexedTriangleMesh3D {
        let positions = self.positions.clone();
        let mut normals = Vec::with_capacity(self.triangles.len());
        let mut faces = Vec::with_capacity(self.triangles.len());

        for (normal_index, triangle) in self.triangles.iter().enumerate() {
            let triangle = triangle.indices();
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

impl Triangulated3D for TriangleSoup {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for triangle in &self.triangles {
            let Some(a) = soup_vertex_position(self, triangle[0]) else {
                continue;
            };
            let Some(b) = soup_vertex_position(self, triangle[1]) else {
                continue;
            };
            let Some(c) = soup_vertex_position(self, triangle[2]) else {
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

impl IndexedTriangulated3D for TriangleSoup {
    fn indexed_triangles(&self) -> IndexedTriangleMesh3D {
        let positions = self
            .vertices
            .iter()
            .map(|point| Point3::new(point.x.clone(), point.y.clone(), point.z.clone()))
            .collect::<Vec<_>>();
        let mut normals = Vec::with_capacity(self.triangles.len());
        let mut faces = Vec::with_capacity(self.triangles.len());

        for (normal_index, triangle) in self.triangles.iter().enumerate() {
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
    } else if value.is_rational() {
        value.clone()
    } else if let Some(finite) = value.to_f64_lossy().filter(|value| value.is_finite()) {
        Real::try_from(finite).unwrap_or_else(|_| value.clone())
    } else {
        value.clone()
    }
}

fn canonical_position(position: &Point3) -> Point3 {
    Point3::new(
        canonical_coordinate(&position.x),
        canonical_coordinate(&position.y),
        canonical_coordinate(&position.z),
    )
}

fn input_mesh_from_buffers(buffers: &HypermeshBuffers) -> InputMesh {
    let positions = buffers
        .positions
        .chunks_exact(3)
        .map(|chunk| Point3::new(chunk[0].clone(), chunk[1].clone(), chunk[2].clone()))
        .collect::<Vec<_>>();
    let triangles = buffers
        .indices
        .chunks_exact(3)
        .map(|chunk| Triangle::new(chunk[0], chunk[1], chunk[2]))
        .collect::<Vec<_>>();
    InputMesh::new(positions, triangles)
}

fn validate_surface_input(mesh: &InputMesh) -> ::hypermesh::HypermeshResult<()> {
    for (triangle_index, triangle) in mesh.triangles.iter().enumerate() {
        let [a, b, c] = triangle.indices();
        for index in [a, b, c] {
            if index >= mesh.positions.len() {
                return Err(::hypermesh::HypermeshError::VertexIndexOutOfBounds {
                    index,
                    vertex_count: mesh.positions.len(),
                });
            }
        }
        if !::hypermesh::Plane::from_points(
            &mesh.positions[a],
            &mesh.positions[b],
            &mesh.positions[c],
        )
        .is_valid()
        {
            return Err(::hypermesh::HypermeshError::DegenerateTriangle {
                mesh_index: 0,
                triangle_index,
            });
        }
    }
    Ok(())
}

pub(crate) fn hypermesh_edges(mesh: &InputMesh) -> Vec<[usize; 2]> {
    let mut edges = Vec::with_capacity(mesh.triangles.len() * 3);
    for triangle in &mesh.triangles {
        let [a, b, c] = triangle.indices();
        edges.push([a, b]);
        edges.push([b, c]);
        edges.push([c, a]);
    }
    edges
}

pub(crate) fn hypermesh_is_closed_manifold(mesh: &InputMesh) -> bool {
    let mut edge_counts = BTreeMap::<(usize, usize), usize>::new();
    for [a, b] in hypermesh_edges(mesh) {
        if a >= mesh.positions.len() || b >= mesh.positions.len() || a == b {
            return false;
        }
        let key = if a < b { (a, b) } else { (b, a) };
        *edge_counts.entry(key).or_insert(0) += 1;
    }
    !edge_counts.is_empty() && edge_counts.values().all(|count| *count == 2)
}

fn soup_vertex_position(soup: &TriangleSoup, vertex: usize) -> Option<Point3> {
    soup.vertices
        .get(vertex)
        .map(|point| Point3::new(point.x.clone(), point.y.clone(), point.z.clone()))
}

fn hyper_triangle_unit_normal(a: &Point3, b: &Point3, c: &Point3) -> Option<Vector3> {
    (b - a).unit_cross_checked(&(c - a)).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::hreal_from_f64;
    use crate::vertex::Vertex;

    #[test]
    fn sphere_materializes_as_closed_hypermesh_input() {
        let radius = hreal_from_f64(1.0).expect("finite test radius");
        let sphere = Mesh::sphere(radius, 16, 8, ());
        let surface = sphere
            .to_hypermesh_triangle_mesh()
            .expect("sphere triangles should form a valid indexed carrier");
        let pole_count = surface
            .positions
            .iter()
            .filter(|point| {
                point.x.refine_sign_until(128) == Some(hyperreal::RealSign::Zero)
                    && point.z.refine_sign_until(128) == Some(hyperreal::RealSign::Zero)
            })
            .count();
        assert_eq!(pole_count, 2);
        assert_eq!(surface.positions.len(), 114);
        assert_eq!(surface.triangles.len(), 224);

        let input = sphere
            .to_hypermesh_exact()
            .expect("sphere should prepare as a closed hypermesh input");

        assert_eq!(input.triangles.len(), 224);
        assert!(hypermesh_is_closed_manifold(&input));
    }

    #[test]
    fn hypermesh_buffers_do_not_weld_distinct_exact_nearby_positions() {
        let zero = Real::zero();
        let one = Real::one();
        let epsilon = (one.clone() / Real::from(1_000_000_000_000_000_u64)).unwrap();
        let normal = Vector3::z();
        let first = Polygon::new(
            vec![
                Vertex::new(
                    Point3::new(zero.clone(), zero.clone(), zero.clone()),
                    normal.clone(),
                ),
                Vertex::new(
                    Point3::new(one.clone(), zero.clone(), zero.clone()),
                    normal.clone(),
                ),
                Vertex::new(
                    Point3::new(zero.clone(), one.clone(), zero.clone()),
                    normal.clone(),
                ),
            ],
            (),
        );
        let second = Polygon::new(
            vec![
                Vertex::new(
                    Point3::new(epsilon.clone(), zero.clone(), zero.clone()),
                    normal.clone(),
                ),
                Vertex::new(
                    Point3::new(one.clone(), zero.clone(), zero.clone()),
                    normal.clone(),
                ),
                Vertex::new(Point3::new(zero.clone(), one.clone(), zero), normal),
            ],
            (),
        );
        let buffers = Mesh::from_polygons(&[first, second], ()).to_hypermesh_buffers();
        let positions = buffers
            .positions
            .chunks_exact(3)
            .map(|row| Point3::new(row[0].clone(), row[1].clone(), row[2].clone()))
            .collect::<Vec<_>>();

        assert_eq!(positions.len(), 4);
        assert!(positions.iter().any(|point| point.x == epsilon));
    }
}
