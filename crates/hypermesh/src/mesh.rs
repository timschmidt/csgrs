//! Input mesh conversion into polygon soup.

use std::collections::HashMap;
use std::sync::Arc;

use hyperlattice::{Point3, Real, RealSign};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Aabb, Plane, axis_ref, compare_real};
use crate::polygon::{
    ConvexPolygon, InputTrianglePlanes, exact_axis_aligned_triangle_support,
    make_indexed_triangle_with_deferred_edges,
    make_indexed_triangle_with_deferred_edges_and_input_planes, make_triangle,
    make_triangle_with_input_planes,
};
use crate::storage_hash::StorageHashMap;

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

/// Validates borrowed mesh views and builds a combined polygon soup.
pub fn build_polygon_soup(meshes: &[MeshRef<'_>]) -> HypermeshResult<PolygonSoup> {
    build_polygon_soup_with_edge_mode(meshes, None, None, false)
}

/// Validates a closed PWN mesh and certifies that every vertex lies in every
/// outward-oriented face half-space.
///
/// A successful result may be retained by mesh owners as a reusable convexity
/// fact for subsequent Boolean operations.
pub fn certify_convex_mesh(mesh: MeshRef<'_>) -> HypermeshResult<()> {
    let soup = build_polygon_soup(&[mesh])?;
    for polygon in &soup.polygons {
        for point in mesh.positions {
            if crate::predicate::classify_point(point, &polygon.support)?
                == crate::geometry::Classification::Positive
            {
                return Err(HypermeshError::NonConvexInput);
            }
        }
    }
    Ok(())
}

pub(crate) fn build_polygon_soup_with_certified_convex_inputs(
    meshes: &[MeshRef<'_>],
    certified_convex_inputs: &[bool],
    input_planes: Option<&[&[InputTrianglePlanes]]>,
) -> HypermeshResult<PolygonSoup> {
    build_polygon_soup_with_edge_mode(meshes, Some(certified_convex_inputs), input_planes, false)
}

pub(crate) fn build_polygon_soup_with_deferred_edges(
    meshes: &[MeshRef<'_>],
    certified_convex_inputs: &[bool],
    input_planes: Option<&[&[InputTrianglePlanes]]>,
) -> HypermeshResult<PolygonSoup> {
    build_polygon_soup_with_edge_mode(meshes, Some(certified_convex_inputs), input_planes, true)
}

fn build_polygon_soup_with_edge_mode(
    meshes: &[MeshRef<'_>],
    certified_convex_inputs: Option<&[bool]>,
    input_planes: Option<&[&[InputTrianglePlanes]]>,
    defer_edges: bool,
) -> HypermeshResult<PolygonSoup> {
    crate::trace_dispatch!("build-polygon-soup", "start");
    if certified_convex_inputs.is_some_and(|certified| certified.len() != meshes.len()) {
        return Err(HypermeshError::UnknownClassification);
    }
    if input_planes.is_some_and(|planes| {
        planes.len() != meshes.len()
            || planes
                .iter()
                .zip(meshes)
                .any(|(planes, mesh)| planes.len() != mesh.triangles.len())
    }) {
        return Err(HypermeshError::UnknownClassification);
    }
    validate_non_empty_mesh_views(meshes)?;

    let bounds = bounds_for_positions(meshes.iter().flat_map(|mesh| mesh.positions.iter()))?;
    crate::trace_dispatch!("build-polygon-soup", "bounds-computed");

    let mut polygons = Vec::new();
    let mut polygon_index = 0isize;
    for (mesh_index, mesh) in meshes.iter().enumerate() {
        let input_is_certified_convex =
            certified_convex_inputs.is_some_and(|certified| certified[mesh_index]);
        let retained_positions = (defer_edges && input_is_certified_convex)
            .then(|| Arc::<[Point3]>::from(mesh.positions));
        // Bound the admission scan before retaining an approximate position
        // cache. A missed axis face only skips the fast path, and every hint
        // is revalidated exactly when its support plane is constructed.
        let sample_count = mesh.triangles.len().min(64);
        let predominantly_axis_aligned = retained_positions.is_some()
            && (0..sample_count).all(|sample| {
                let triangle_index = sample * mesh.triangles.len() / sample_count;
                approximate_triangle_axis(mesh.positions, mesh.triangles[triangle_index].indices())
                    .is_some()
            });
        let (approximate_positions, approximate_positions_are_exact_dyadic) =
            if predominantly_axis_aligned {
                let exact_dyadic = mesh
                    .positions
                    .iter()
                    .map(|point| {
                        let coordinates = [&point.x, &point.y, &point.z];
                        let [Some(x), Some(y), Some(z)] =
                            coordinates.map(Real::to_f64_exact_dyadic)
                        else {
                            return None;
                        };
                        Some([x, y, z])
                    })
                    .collect::<Option<Vec<_>>>();
                match exact_dyadic {
                    Some(positions) => (Some(positions), true),
                    None => (
                        mesh.positions
                            .iter()
                            .map(|point| {
                                Some([
                                    point.x.to_f64_lossy()?,
                                    point.y.to_f64_lossy()?,
                                    point.z.to_f64_lossy()?,
                                ])
                            })
                            .collect::<Option<Vec<_>>>(),
                        false,
                    ),
                }
            } else {
                (None, false)
            };
        let mut axis_support_planes: StorageHashMap<(usize, usize, bool), Plane> =
            StorageHashMap::default();
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
            let supplied_planes = input_planes
                .and_then(|planes| planes.get(mesh_index))
                .and_then(|planes| planes.get(triangle_index))
                .cloned();
            let mut polygon = match (retained_positions.as_ref(), supplied_planes) {
                (Some(positions), Some(planes)) => {
                    make_indexed_triangle_with_deferred_edges_and_input_planes(
                        Arc::clone(positions),
                        [i0, i1, i2],
                        planes,
                        mesh_index as isize,
                        polygon_index,
                    )
                }
                (Some(positions), None) => {
                    let axis_hint = approximate_positions.as_ref().and_then(|points| {
                        let [p0, p1, p2] = [points[i0], points[i1], points[i2]];
                        let axis =
                            (0..3).find(|&axis| p0[axis] == p1[axis] && p0[axis] == p2[axis])?;
                        let orientation = if approximate_positions_are_exact_dyadic {
                            let u = (axis + 1) % 3;
                            let v = (axis + 2) % 3;
                            Real::certified_affine_det2_sign_exact_dyadic_f64(
                                [p0[u], p0[v]],
                                [p1[u], p1[v]],
                                [p2[u], p2[v]],
                            )
                        } else {
                            None
                        };
                        Some((axis, orientation))
                    });
                    let support_hint = axis_hint.and_then(|(axis, orientation)| {
                        let orientation_positive = match orientation {
                            Some(RealSign::Negative) => false,
                            Some(RealSign::Positive) => true,
                            Some(RealSign::Zero) | None => {
                                return exact_axis_aligned_triangle_support(
                                    p0,
                                    p1,
                                    p2,
                                    axis,
                                    orientation,
                                );
                            }
                        };
                        let coordinate_identity =
                            axis_ref(p0, axis).exact_rational_ref()?.storage_identity();
                        let key = (axis, coordinate_identity, orientation_positive);
                        if let Some(support) = axis_support_planes.get(&key) {
                            return Some(support.clone());
                        }
                        let support =
                            exact_axis_aligned_triangle_support(p0, p1, p2, axis, orientation)?;
                        axis_support_planes.insert(key, support.clone());
                        Some(support)
                    });
                    make_indexed_triangle_with_deferred_edges(
                        Arc::clone(positions),
                        [i0, i1, i2],
                        support_hint,
                        mesh_index as isize,
                        polygon_index,
                    )
                }
                (None, Some(planes)) => make_triangle_with_input_planes(
                    p0,
                    p1,
                    p2,
                    planes,
                    mesh_index as isize,
                    polygon_index,
                ),
                (None, None) => make_triangle(p0, p1, p2, mesh_index as isize, polygon_index),
            }
            .with_source_triangle_edge_identities(mesh_index, [i0, i1, i2]);
            if !polygon.support.is_valid() {
                return Err(HypermeshError::DegenerateTriangle {
                    mesh_index,
                    triangle_index,
                });
            }
            if !defer_edges {
                polygon.delta_w = vec![0; meshes.len()];
                polygon.delta_w[mesh_index] = 1;
            }
            polygons.push(polygon);
            polygon_index += 1;
        }
        if !input_is_certified_convex {
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

    crate::trace_dispatch!("build-polygon-soup", "complete");
    Ok(PolygonSoup {
        polygons,
        bounds,
        num_meshes: meshes.len(),
    })
}

fn approximate_triangle_axis(positions: &[Point3], indices: [usize; 3]) -> Option<usize> {
    let points = indices.map(|index| positions.get(index));
    let [Some(p0), Some(p1), Some(p2)] = points else {
        return None;
    };
    let points = [p0, p1, p2].map(|point| {
        Some([
            point.x.to_f64_lossy()?,
            point.y.to_f64_lossy()?,
            point.z.to_f64_lossy()?,
        ])
    });
    let [Some(p0), Some(p1), Some(p2)] = points else {
        return None;
    };
    (0..3).find(|&axis| p0[axis] == p1[axis] && p0[axis] == p2[axis])
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
    let first_coordinates = [&first.x, &first.y, &first.z];
    let mut exact_dyadic_bounds = match first_coordinates.map(Real::to_f64_exact_dyadic) {
        [Some(x), Some(y), Some(z)] => Some(([x, y, z], [x, y, z])),
        _ => None,
    };

    for position in positions {
        if let Some((min_f64, max_f64)) = &mut exact_dyadic_bounds {
            let coordinates = [&position.x, &position.y, &position.z];
            if let [Some(x), Some(y), Some(z)] = coordinates.map(Real::to_f64_exact_dyadic) {
                for (axis, value) in [x, y, z].into_iter().enumerate() {
                    if value < min_f64[axis] {
                        min_f64[axis] = value;
                        *crate::geometry::axis_mut(&mut min, axis) =
                            axis_ref(position, axis).clone();
                    }
                    if value > max_f64[axis] {
                        max_f64[axis] = value;
                        *crate::geometry::axis_mut(&mut max, axis) =
                            axis_ref(position, axis).clone();
                    }
                }
                continue;
            }
            exact_dyadic_bounds = None;
        }
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
    use crate::polygon::RetainedVertexCycle;
    use hyperlattice::Rational;

    #[test]
    fn bounds_exact_dyadic_scan_falls_back_for_later_general_rational() {
        let one_third = Real::new(Rational::fraction(1, 3).unwrap());
        let points = [
            Point3::new(Real::from(4), Real::from(-2), Real::from(8)),
            Point3::new(Real::from(-3), Real::from(5), Real::from(1)),
            Point3::new(one_third.clone(), Real::from(-7), Real::from(9)),
        ];

        assert_eq!(
            bounds_for_positions(&points).unwrap(),
            Aabb::new(
                Point3::new(Real::from(-3), Real::from(-7), Real::from(1)),
                Point3::new(Real::from(4), Real::from(5), Real::from(9)),
            ),
        );
    }

    #[test]
    fn deferred_certified_triangles_share_one_indexed_position_pool() {
        let positions = vec![
            Point3::new(Real::zero(), Real::zero(), Real::zero()),
            Point3::new(Real::one(), Real::zero(), Real::zero()),
            Point3::new(Real::zero(), Real::one(), Real::zero()),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
        ];
        let mesh = InputMesh::new(
            positions.clone(),
            vec![Triangle::new(0, 1, 2), Triangle::new(0, 3, 1)],
        );
        let soup = build_polygon_soup_with_deferred_edges(&[mesh.as_ref()], &[true], None).unwrap();

        let (
            Some(RetainedVertexCycle::IndexedTriangle {
                positions: first,
                indices: first_indices,
            }),
            Some(RetainedVertexCycle::IndexedTriangle {
                positions: second,
                indices: second_indices,
            }),
        ) = (
            &soup.polygons[0].known_vertices,
            &soup.polygons[1].known_vertices,
        )
        else {
            panic!("certified deferred triangles must retain indexed vertices");
        };

        assert!(Arc::ptr_eq(first, second));
        assert_eq!(*first_indices, [0, 1, 2]);
        assert_eq!(*second_indices, [0, 3, 1]);
        assert_eq!(soup.polygons[0].vertices().unwrap(), positions[..3]);
    }

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
        build_polygon_soup(&[mesh.as_ref()]).expect("closed coincident-index tetrahedron");
    }
}
