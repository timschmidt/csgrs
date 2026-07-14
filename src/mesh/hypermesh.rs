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
    mesh::Polygon,
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
    /// Hypermesh returned a triangle without a valid source polygon.
    #[error("hypermesh returned invalid source triangle {triangle} for mesh {mesh}")]
    InvalidSource { mesh: isize, triangle: isize },
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

pub(super) struct HypermeshAdapterInput {
    pub(super) buffers: HypermeshBuffers,
    source_polygons: Vec<usize>,
    position_ids: HashMap<u64, usize>,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct PositionKey([Option<u64>; 3]);

impl PositionKey {
    fn new(position: &Point3) -> Self {
        Self([
            position.x.to_f64_lossy().map(f64::to_bits),
            position.y.to_f64_lossy().map(f64::to_bits),
            position.z.to_f64_lossy().map(f64::to_bits),
        ])
    }
}

enum PositionIndices {
    One(usize),
    Collisions(Vec<usize>),
}

impl PositionIndices {
    fn find(&self, positions: &[Point3], position: &Point3) -> Option<usize> {
        match self {
            Self::One(index) => (positions[*index] == *position).then_some(*index),
            Self::Collisions(indices) => indices
                .iter()
                .copied()
                .find(|index| positions[*index] == *position),
        }
    }

    fn insert_collision(&mut self, index: usize) {
        match self {
            Self::One(first) => {
                let first = *first;
                *self = Self::Collisions(vec![first, index]);
            },
            Self::Collisions(indices) => indices.push(index),
        }
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
        self.build_hypermesh_input(false).buffers
    }

    pub(super) fn build_hypermesh_input(&self, retain_sources: bool) -> HypermeshAdapterInput {
        let triangle_capacity = self
            .polygons
            .iter()
            .map(|polygon| polygon.vertices.len().saturating_sub(2))
            .sum::<usize>();
        let mut canonical_positions = Vec::<Point3>::with_capacity(triangle_capacity * 3);
        let mut indices = Vec::with_capacity(triangle_capacity * 3);
        let mut vertex_indices = HashMap::<PositionKey, PositionIndices>::new();
        let mut position_ids = HashMap::<u64, usize>::new();
        let mut source_polygons =
            Vec::with_capacity(if retain_sources { triangle_capacity } else { 0 });

        let mut push_triangle = |vertices: &[Vertex]| {
            for vertex in vertices {
                let position = canonical_position(&vertex.position);
                let key = PositionKey::new(&position);
                let index = match vertex_indices.entry(key) {
                    hashbrown::hash_map::Entry::Vacant(entry) => {
                        let index = canonical_positions.len();
                        canonical_positions.push(position);
                        entry.insert(PositionIndices::One(index));
                        index
                    },
                    hashbrown::hash_map::Entry::Occupied(mut entry) => {
                        if let Some(index) = entry.get().find(&canonical_positions, &position)
                        {
                            index
                        } else {
                            let index = canonical_positions.len();
                            canonical_positions.push(position);
                            entry.get_mut().insert_collision(index);
                            index
                        }
                    },
                };
                position_ids.insert(vertex.position_id, index);
                indices.push(index);
            }
        };

        for (polygon_index, polygon) in self.polygons.iter().enumerate() {
            if polygon.vertices.len() == 3 {
                push_triangle(&polygon.vertices);
                if retain_sources {
                    source_polygons.push(polygon_index);
                }
                continue;
            }
            for triangle in polygon.triangulate() {
                push_triangle(&triangle);
                if retain_sources {
                    source_polygons.push(polygon_index);
                }
            }
        }

        let positions = canonical_positions
            .into_iter()
            .flat_map(|point| [point.x, point.y, point.z])
            .collect();

        HypermeshAdapterInput {
            buffers: HypermeshBuffers { positions, indices },
            source_polygons,
            position_ids,
        }
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

    pub(super) fn to_hypermesh_connectivity_mesh(
        &self,
    ) -> ::hypermesh::HypermeshResult<(InputMesh, HashMap<u64, usize>)> {
        let input = self.build_hypermesh_input(false);
        let mesh = input_mesh_from_buffers(&input.buffers);
        if !mesh.positions.is_empty() {
            validate_surface_input(&mesh)?;
        }
        Ok((mesh, input.position_ids))
    }

    /// Compute a mesh boolean through exact `hypermesh`.
    ///
    /// The finite `csgrs` polygons are imported as audited exact meshes and the
    /// returned exact output is replayed as finite `Mesh` triangles. This keeps
    /// topology decisions on the exact side, with conversion boundary evidence
    /// preserved as required by Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), and
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>.
    pub(super) fn boolean_via_hypermesh(
        &self,
        other: &Self,
        operation: HypermeshBooleanOp,
        left_input: HypermeshAdapterInput,
        right_input: HypermeshAdapterInput,
    ) -> Result<Self, HypermeshError> {
        let left = input_mesh_from_buffers(&left_input.buffers);
        let right = input_mesh_from_buffers(&right_input.buffers);
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
        self.materialize_hypermesh_soup(
            other,
            &soup,
            &left_input.source_polygons,
            &right_input.source_polygons,
        )
    }

    fn materialize_hypermesh_soup(
        &self,
        other: &Self,
        soup: &TriangleSoup,
        left_source_polygons: &[usize],
        right_source_polygons: &[usize],
    ) -> Result<Self, HypermeshError> {
        let mut polygons = Vec::with_capacity(soup.triangles.len());

        for (triangle, source) in soup.triangles.iter().zip(&soup.sources) {
            let source_index = usize::try_from(source.triangle).map_err(|_| {
                HypermeshError::InvalidSource {
                    mesh: source.mesh,
                    triangle: source.triangle,
                }
            })?;
            let polygon = match source.mesh {
                0 => left_source_polygons
                    .get(source_index)
                    .and_then(|index| self.polygons.get(*index)),
                1 => source_index
                    .checked_sub(left_source_polygons.len())
                    .and_then(|index| right_source_polygons.get(index))
                    .and_then(|index| other.polygons.get(*index)),
                _ => None,
            }
            .ok_or(HypermeshError::InvalidSource {
                mesh: source.mesh,
                triangle: source.triangle,
            })?;
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
                polygon.metadata.clone(),
            ));
        }

        Ok(Self::from_polygons(polygons))
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
    if matches!(value.refine_sign_until(-128), Some(hyperreal::RealSign::Zero)) {
        Real::zero()
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
                point.x.refine_sign_until(-128) == Some(hyperreal::RealSign::Zero)
                    && point.z.refine_sign_until(-128) == Some(hyperreal::RealSign::Zero)
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
        let buffers = Mesh::from_polygons(vec![first, second]).to_hypermesh_buffers();
        let positions = buffers
            .positions
            .chunks_exact(3)
            .map(|row| Point3::new(row[0].clone(), row[1].clone(), row[2].clone()))
            .collect::<Vec<_>>();

        assert_eq!(positions.len(), 4);
        assert!(positions.iter().any(|point| point.x == epsilon));
    }
}
