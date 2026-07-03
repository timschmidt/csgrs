//! Input mesh conversion into polygon soup.

use hyperlattice::{Point3, Real};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Aabb, axis_ref, compare_real};
use crate::polygon::{ConvexPolygon, make_triangle};

/// Input triangle: three vertex indices.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Triangle {
    /// First vertex index.
    pub v0: usize,
    /// Second vertex index.
    pub v1: usize,
    /// Third vertex index.
    pub v2: usize,
}

impl Triangle {
    /// Constructs an input triangle.
    pub const fn new(v0: usize, v1: usize, v2: usize) -> Self {
        Self { v0, v1, v2 }
    }

    /// Returns the indices as an array.
    pub const fn indices(self) -> [usize; 3] {
        [self.v0, self.v1, self.v2]
    }
}

/// Owned input mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct InputMesh {
    /// Vertex positions.
    pub positions: Vec<Point3>,
    /// Triangle indices.
    pub triangles: Vec<Triangle>,
}

impl InputMesh {
    /// Creates an owned input mesh.
    pub fn new(positions: Vec<Point3>, triangles: Vec<Triangle>) -> Self {
        Self {
            positions,
            triangles,
        }
    }

    /// Returns a borrowed mesh view.
    pub fn as_ref(&self) -> MeshRef<'_> {
        MeshRef {
            positions: &self.positions,
            triangles: &self.triangles,
        }
    }
}

/// Borrowed input mesh view.
#[derive(Clone, Copy, Debug)]
pub struct MeshRef<'a> {
    /// Borrowed positions.
    pub positions: &'a [Point3],
    /// Borrowed triangles.
    pub triangles: &'a [Triangle],
}

/// Output vertex in external primitive space.
#[derive(Clone, Debug, PartialEq)]
pub struct OutputVertex {
    /// X coordinate.
    pub x: Real,
    /// Y coordinate.
    pub y: Real,
    /// Z coordinate.
    pub z: Real,
}

/// Working polygon soup.
#[derive(Clone, Debug, PartialEq)]
pub struct PolygonSoup {
    /// Polygons produced from input triangles.
    pub polygons: Vec<ConvexPolygon>,
    /// Exact bounds across all source positions.
    pub bounds: Aabb,
    /// Number of input meshes.
    pub num_meshes: usize,
}

impl PolygonSoup {
    /// Recomputes exact bounds from polygon vertices.
    pub fn compute_bounds_from_vertices(&mut self) -> HypermeshResult<()> {
        let mut vertices = Vec::new();
        for polygon in &self.polygons {
            vertices.extend(polygon.vertices()?);
        }
        self.bounds = bounds_for_positions(&vertices)?;
        Ok(())
    }
}

/// Prepares one borrowed mesh from position and triangle slices.
pub fn prepare_input(positions: &[Point3], triangles: &[Triangle]) -> HypermeshResult<PolygonSoup> {
    prepare_input_refs(&[MeshRef {
        positions,
        triangles,
    }])
}

/// Prepares owned meshes by delegating to the borrowed slice API.
pub fn prepare_input_meshes(meshes: &[InputMesh]) -> HypermeshResult<PolygonSoup> {
    let refs = meshes.iter().map(InputMesh::as_ref).collect::<Vec<_>>();
    prepare_input_refs(&refs)
}

/// Prepares borrowed mesh views into a combined polygon soup.
pub fn prepare_input_refs(meshes: &[MeshRef<'_>]) -> HypermeshResult<PolygonSoup> {
    let all_positions = meshes
        .iter()
        .flat_map(|mesh| mesh.positions.iter().cloned())
        .collect::<Vec<_>>();
    let bounds = bounds_for_positions(&all_positions)?;

    let mut polygons = Vec::new();
    let mut polygon_index = 0isize;
    for (mesh_index, mesh) in meshes.iter().enumerate() {
        for triangle in mesh.triangles {
            let [i0, i1, i2] = triangle.indices();
            let p0 = mesh
                .positions
                .get(i0)
                .ok_or(HypermeshError::VertexIndexOutOfBounds {
                    index: i0,
                    vertex_count: mesh.positions.len(),
                })?;
            let p1 = mesh
                .positions
                .get(i1)
                .ok_or(HypermeshError::VertexIndexOutOfBounds {
                    index: i1,
                    vertex_count: mesh.positions.len(),
                })?;
            let p2 = mesh
                .positions
                .get(i2)
                .ok_or(HypermeshError::VertexIndexOutOfBounds {
                    index: i2,
                    vertex_count: mesh.positions.len(),
                })?;
            let mut polygon = make_triangle(p0, p1, p2, mesh_index as isize, polygon_index);
            if !polygon.support.is_valid() {
                continue;
            }
            polygon.delta_w = vec![0; meshes.len()];
            polygon.delta_w[mesh_index] = 1;
            polygons.push(polygon);
            polygon_index += 1;
        }
    }

    Ok(PolygonSoup {
        polygons,
        bounds,
        num_meshes: meshes.len(),
    })
}

fn bounds_for_positions(positions: &[Point3]) -> HypermeshResult<Aabb> {
    let first = positions.first().ok_or(HypermeshError::EmptyInput)?;
    let mut min = first.clone();
    let mut max = first.clone();

    for position in &positions[1..] {
        for axis in 0..3 {
            if compare_real(axis_ref(position, axis), axis_ref(&min, axis))?.is_lt() {
                *crate::geometry::axis_mut(&mut min, axis) = axis_ref(position, axis).clone();
            }
            if compare_real(axis_ref(position, axis), axis_ref(&max, axis))?.is_gt() {
                *crate::geometry::axis_mut(&mut max, axis) = axis_ref(position, axis).clone();
            }
        }
    }

    Ok(Aabb::new(min, max))
}
