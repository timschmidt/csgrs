//! Input mesh conversion into polygon soup.

use std::collections::HashMap;

use hyperlattice::{Point3, Real};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Aabb, axis_ref, compare_real};
use crate::polygon::{ConvexPolygon, make_triangle, make_triangle_with_deferred_edges};

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
        self.bounds = bounds_for_positions(vertices.iter())?;
        Ok(())
    }
}

/// Prepares borrowed mesh views into a combined polygon soup.
pub fn prepare_input(meshes: &[MeshRef<'_>]) -> HypermeshResult<PolygonSoup> {
    prepare_input_with_certified_convex_inputs(meshes, &vec![false; meshes.len()])
}

pub(crate) fn prepare_input_with_certified_convex_inputs(
    meshes: &[MeshRef<'_>],
    certified_convex_inputs: &[bool],
) -> HypermeshResult<PolygonSoup> {
    prepare_input_with_edge_mode(meshes, certified_convex_inputs, false)
}

pub(crate) fn prepare_input_with_deferred_edges(
    meshes: &[MeshRef<'_>],
    certified_convex_inputs: &[bool],
) -> HypermeshResult<PolygonSoup> {
    prepare_input_with_edge_mode(meshes, certified_convex_inputs, true)
}

fn prepare_input_with_edge_mode(
    meshes: &[MeshRef<'_>],
    certified_convex_inputs: &[bool],
    defer_edges: bool,
) -> HypermeshResult<PolygonSoup> {
    crate::trace_dispatch!("prepare-input", "start");
    if certified_convex_inputs.len() != meshes.len() {
        return Err(HypermeshError::UnknownClassification);
    }
    validate_non_empty_mesh_views(meshes)?;

    let bounds = bounds_for_positions(meshes.iter().flat_map(|mesh| mesh.positions.iter()))?;
    crate::trace_dispatch!("prepare-input", "bounds-computed");

    let mut polygons = Vec::new();
    let mut polygon_index = 0isize;
    for (mesh_index, mesh) in meshes.iter().enumerate() {
        for (triangle_index, triangle) in mesh.triangles.iter().enumerate() {
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
            let mut polygon = if defer_edges && certified_convex_inputs[mesh_index] {
                make_triangle_with_deferred_edges(p0, p1, p2, mesh_index as isize, polygon_index)
            } else {
                make_triangle(p0, p1, p2, mesh_index as isize, polygon_index)
            }
            .with_source_triangle_edge_identities(mesh_index, [i0, i1, i2]);
            if !polygon.support.is_valid() {
                return Err(HypermeshError::DegenerateTriangle {
                    mesh_index,
                    triangle_index,
                });
            }
            polygon.delta_w = vec![0; meshes.len()];
            polygon.delta_w[mesh_index] = 1;
            polygons.push(polygon);
            polygon_index += 1;
        }
        if !certified_convex_inputs[mesh_index] {
            let edge_balance = classify_indexed_edge_balance(mesh);
            if edge_balance.boundary_edges != 0 {
                return Err(HypermeshError::OpenInput {
                    mesh_index,
                    boundary_edges: edge_balance.boundary_edges,
                });
            }
            if edge_balance.unbalanced_edges != 0 {
                return Err(HypermeshError::NonPwnInput {
                    mesh_index,
                    unbalanced_edges: edge_balance.unbalanced_edges,
                });
            }
        }
    }

    crate::trace_dispatch!("prepare-input", "complete");
    Ok(PolygonSoup {
        polygons,
        bounds,
        num_meshes: meshes.len(),
    })
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct PositionBucket([Option<u64>; 3]);

fn classify_indexed_edge_balance(mesh: &MeshRef<'_>) -> EdgeBalance {
    let mut canonical_positions: Vec<&Point3> = Vec::with_capacity(mesh.positions.len());
    let mut buckets = HashMap::<PositionBucket, Vec<usize>>::new();
    let mut canonical_indices = Vec::with_capacity(mesh.positions.len());
    for position in mesh.positions {
        let key = PositionBucket([
            position.x.to_f64_lossy().map(f64::to_bits),
            position.y.to_f64_lossy().map(f64::to_bits),
            position.z.to_f64_lossy().map(f64::to_bits),
        ]);
        let candidates = buckets.entry(key).or_default();
        let canonical = candidates
            .iter()
            .copied()
            .find(|index| *canonical_positions[*index] == *position)
            .unwrap_or_else(|| {
                let index = canonical_positions.len();
                canonical_positions.push(position);
                candidates.push(index);
                index
            });
        canonical_indices.push(canonical);
    }

    let mut edge_uses = HashMap::<(usize, usize), [usize; 2]>::new();
    for triangle in mesh.triangles {
        let [a, b, c] = triangle.indices().map(|index| canonical_indices[index]);
        for [start, end] in [[a, b], [b, c], [c, a]] {
            let (key, direction) = if start < end {
                ((start, end), 0)
            } else {
                ((end, start), 1)
            };
            edge_uses.entry(key).or_default()[direction] += 1;
        }
    }

    edge_uses
        .values()
        .fold(EdgeBalance::default(), |mut balance, uses| {
            if uses[0] + uses[1] == 1 {
                balance.boundary_edges += 1;
            }
            if uses[0] != uses[1] {
                balance.unbalanced_edges += 1;
            }
            balance
        })
}

fn validate_non_empty_mesh_views(meshes: &[MeshRef<'_>]) -> HypermeshResult<()> {
    if meshes.is_empty() {
        return Err(HypermeshError::EmptyInput);
    }
    for (mesh_index, mesh) in meshes.iter().enumerate() {
        if mesh.positions.is_empty() || mesh.triangles.is_empty() {
            return Err(HypermeshError::EmptyMesh { mesh_index });
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct EdgeBalance {
    pub(crate) boundary_edges: usize,
    pub(crate) unbalanced_edges: usize,
}

pub(crate) fn classify_edge_balance(edges: &[[Point3; 2]]) -> EdgeBalance {
    let mut balance = EdgeBalance::default();
    let mut visited = vec![false; edges.len()];
    for (index, edge) in edges.iter().enumerate() {
        if visited[index] {
            continue;
        }

        let mut forward_uses = 0usize;
        let mut reverse_uses = 0usize;
        for (other_index, other) in edges.iter().enumerate() {
            if !undirected_edges_match(edge, other) {
                continue;
            }
            visited[other_index] = true;
            if edge == other {
                forward_uses += 1;
            } else {
                reverse_uses += 1;
            }
        }

        if forward_uses + reverse_uses == 1 {
            balance.boundary_edges += 1;
        }
        if forward_uses != reverse_uses {
            balance.unbalanced_edges += 1;
        }
    }
    balance
}

fn undirected_edges_match(left: &[Point3; 2], right: &[Point3; 2]) -> bool {
    (left[0] == right[0] && left[1] == right[1]) || (left[0] == right[1] && left[1] == right[0])
}

fn bounds_for_positions<'a>(
    positions: impl IntoIterator<Item = &'a Point3>,
) -> HypermeshResult<Aabb> {
    let mut positions = positions.into_iter();
    let first = positions.next().ok_or(HypermeshError::EmptyInput)?;
    let mut min = first.clone();
    let mut max = first.clone();

    for position in positions {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn indexed_edge_balance_canonicalizes_coincident_input_vertices() {
        let geometric = [
            Point3::new(Real::zero(), Real::zero(), Real::zero()),
            Point3::new(Real::one(), Real::zero(), Real::zero()),
            Point3::new(Real::zero(), Real::one(), Real::zero()),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
        ];
        let faces = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
        let mut positions = Vec::new();
        let mut triangles = Vec::new();
        for face in faces {
            let start = positions.len();
            positions.extend(face.map(|index| geometric[index].clone()));
            triangles.push(Triangle::new(start, start + 1, start + 2));
        }
        let mesh = InputMesh::new(positions, triangles);

        assert_eq!(
            classify_indexed_edge_balance(&mesh.as_ref()),
            EdgeBalance::default()
        );
        prepare_input(&[mesh.as_ref()]).expect("closed coincident-index tetrahedron");
    }
}
