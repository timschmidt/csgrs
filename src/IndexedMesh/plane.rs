//! IndexedMesh-Optimized Plane Operations
//!
//! This module implements robust geometric operations for planes optimized for
//! IndexedMesh's indexed connectivity model while maintaining compatibility
//! with the regular Mesh plane operations.

use crate::IndexedMesh::{IndexedPolygon, vertex::IndexedVertex};
use crate::float_types::{EPSILON, Real};
use nalgebra::{Isometry3, Matrix4, Point3, Rotation3, Translation3, Vector3};
use robust;
use std::collections::HashMap;
use std::fmt::Debug;

/// **Plane-Edge Cache Key**
///
/// Proper cache key that includes both the edge vertices and the plane information
/// to ensure intersection vertices are only shared for the same plane-edge combination.
/// This prevents the critical bug where different planes incorrectly share intersection
/// vertices for the same edge.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct PlaneEdgeCacheKey {
    /// Canonical edge representation (smaller index first)
    edge: (usize, usize),
    /// Plane normal quantized to avoid floating-point precision issues
    plane_normal_quantized: (i64, i64, i64),
    /// Plane offset quantized to avoid floating-point precision issues
    plane_offset_quantized: i64,
}

impl PlaneEdgeCacheKey {
    /// Create a cache key for a specific plane-edge combination
    pub fn new(plane: &Plane, idx_i: usize, idx_j: usize) -> Self {
        // Create canonical edge key (smaller index first)
        let edge = if idx_i < idx_j {
            (idx_i, idx_j)
        } else {
            (idx_j, idx_i)
        };

        // Quantize plane parameters to avoid floating-point precision issues
        // **CRITICAL FIX**: Increased precision from 1e6 to 1e12 to prevent incorrect vertex sharing
        // that was causing gaps in complex CSG operations. The previous 1e6 scale was too coarse
        // and merged vertices that should remain separate, creating visible gaps.
        const QUANTIZATION_SCALE: Real = 1e12;
        let plane_normal_quantized = (
            (plane.normal.x * QUANTIZATION_SCALE).round() as i64,
            (plane.normal.y * QUANTIZATION_SCALE).round() as i64,
            (plane.normal.z * QUANTIZATION_SCALE).round() as i64,
        );
        let plane_offset_quantized = (plane.w * QUANTIZATION_SCALE).round() as i64;

        PlaneEdgeCacheKey {
            edge,
            plane_normal_quantized,
            plane_offset_quantized,
        }
    }
}

// Plane classification constants (matching mesh::plane constants)
pub const COPLANAR: i8 = 0;
pub const FRONT: i8 = 1;
pub const BACK: i8 = 2;
pub const SPANNING: i8 = 3;

/// IndexedMesh-Optimized Plane
///
/// A plane representation optimized for IndexedMesh operations.
/// Maintains the same mathematical properties as the regular Plane
/// but with enhanced functionality for indexed operations.
#[derive(Debug, Clone, PartialEq)]
pub struct Plane {
    /// Unit normal vector of the plane
    pub normal: Vector3<Real>,
    /// Distance from origin along normal (plane equation: n·p = w)
    pub w: Real,
}

impl Plane {
    /// Create a new plane from normal vector and distance
    pub fn from_normal(normal: Vector3<Real>, w: Real) -> Self {
        let normalized = normal.normalize();
        Plane {
            normal: normalized,
            w,
        }
    }

    /// Create a plane from three points
    /// The normal direction follows the right-hand rule: (p2-p1) × (p3-p1)
    pub fn from_points(p1: Point3<Real>, p2: Point3<Real>, p3: Point3<Real>) -> Self {
        let v1 = p2 - p1;
        let v2 = p3 - p1;
        let normal = v1.cross(&v2);

        if normal.norm_squared() < Real::EPSILON * Real::EPSILON {
            // Degenerate triangle, return default plane
            return Plane {
                normal: Vector3::z(),
                w: 0.0,
            };
        }

        let normal = normal.normalize();
        let w = normal.dot(&p1.coords);
        Plane { normal, w }
    }

    /// Create a plane from vertices (for compatibility with regular Mesh)
    pub fn from_vertices(vertices: Vec<crate::mesh::vertex::Vertex>) -> Self {
        if vertices.len() < 3 {
            return Plane {
                normal: Vector3::z(),
                w: 0.0,
            };
        }

        let p1 = vertices[0].pos;
        let p2 = vertices[1].pos;
        let p3 = vertices[2].pos;
        Self::from_points(p1, p2, p3)
    }

    /// Create a plane from IndexedVertex vertices (optimized for IndexedMesh)
    /// Uses the same robust algorithm as Mesh::from_vertices for consistency
    pub fn from_indexed_vertices(vertices: Vec<IndexedVertex>) -> Self {
        let n = vertices.len();
        if n < 3 {
            return Plane {
                normal: Vector3::z(),
                w: 0.0,
            };
        }

        let reference_plane = Plane {
            normal: (vertices[1].pos - vertices[0].pos)
                .cross(&(vertices[2].pos - vertices[0].pos))
                .normalize(),
            w: vertices[0].pos.coords.dot(
                &(vertices[1].pos - vertices[0].pos)
                    .cross(&(vertices[2].pos - vertices[0].pos))
                    .normalize(),
            ),
        };

        if n == 3 {
            return reference_plane;
        }

        // Find the longest chord (farthest pair of points) - same as Mesh implementation
        let Some((i0, i1, _)) = (0..n)
            .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
            .map(|(i, j)| {
                let d2 = (vertices[i].pos - vertices[j].pos).norm_squared();
                (i, j, d2)
            })
            .max_by(|a, b| a.2.total_cmp(&b.2))
        else {
            return reference_plane;
        };

        let p0 = vertices[i0].pos;
        let p1 = vertices[i1].pos;
        let dir = p1 - p0;
        if dir.norm_squared() < EPSILON * EPSILON {
            return reference_plane; // everything almost coincident
        }

        // Find vertex farthest from the line p0-p1
        let Some((i2, max_area2)) = vertices
            .iter()
            .enumerate()
            .filter(|(idx, _)| *idx != i0 && *idx != i1)
            .map(|(idx, v)| {
                let a2 = (v.pos - p0).cross(&dir).norm_squared(); // ∝ area²
                (idx, a2)
            })
            .max_by(|a, b| a.1.total_cmp(&b.1))
        else {
            return reference_plane;
        };

        let i2 = if max_area2 > EPSILON * EPSILON {
            i2
        } else {
            return reference_plane; // all vertices collinear
        };
        let p2 = vertices[i2].pos;

        // Build plane using the optimal triangle
        let mut plane_hq = Self::from_points(p0, p1, p2);

        // Construct the reference normal for the original polygon using Newell's Method
        let reference_normal = vertices.iter().zip(vertices.iter().cycle().skip(1)).fold(
            Vector3::zeros(),
            |acc, (curr, next)| {
                acc + (curr.pos - Point3::origin()).cross(&(next.pos - Point3::origin()))
            },
        );

        // Orient the plane to match original winding
        if plane_hq.normal().dot(&reference_normal) < 0.0 {
            plane_hq.flip(); // flip in-place to agree with winding
        }

        plane_hq
    }

    /// Get the plane normal (matches regular Mesh API)
    pub const fn normal(&self) -> Vector3<Real> {
        self.normal
    }

    /// Get the offset (distance from origin) (matches regular Mesh API)
    pub const fn offset(&self) -> Real {
        self.w
    }

    /// Flip the plane (reverse normal and distance)
    pub fn flip(&mut self) {
        self.normal = -self.normal;
        self.w = -self.w;
    }

    /// Return a flipped copy of this plane
    pub fn flipped(&self) -> Self {
        Plane {
            normal: -self.normal,
            w: -self.w,
        }
    }

    /// Classify a point relative to the plane using robust geometric predicates
    /// This matches the regular Mesh API but uses the IndexedMesh (normal, w) representation
    pub fn orient_point(&self, point: &Point3<Real>) -> i8 {
        // For robust geometric classification, we need three points on the plane
        // Generate them from the normal and offset
        let p0 = Point3::from(self.normal * (self.w / self.normal.norm_squared()));

        // Build an orthonormal basis {u, v} that spans the plane
        let mut u = if self.normal.z.abs() > self.normal.x.abs()
            || self.normal.z.abs() > self.normal.y.abs()
        {
            // normal is closer to ±Z ⇒ cross with X
            Vector3::x().cross(&self.normal)
        } else {
            // otherwise cross with Z
            Vector3::z().cross(&self.normal)
        };
        u.normalize_mut();
        let v = self.normal.cross(&u).normalize();

        // Use p0, p0+u, p0+v as the three defining points
        let point_a = p0;
        let point_b = p0 + u;
        let point_c = p0 + v;

        // Use robust orient3d predicate (same as regular Mesh)
        let sign = robust::orient3d(
            robust::Coord3D {
                x: point_a.x,
                y: point_a.y,
                z: point_a.z,
            },
            robust::Coord3D {
                x: point_b.x,
                y: point_b.y,
                z: point_b.z,
            },
            robust::Coord3D {
                x: point_c.x,
                y: point_c.y,
                z: point_c.z,
            },
            robust::Coord3D {
                x: point.x,
                y: point.y,
                z: point.z,
            },
        );

        if sign > EPSILON as f64 {
            BACK
        } else if sign < -(EPSILON as f64) {
            FRONT
        } else {
            COPLANAR
        }
    }

    /// Classify an IndexedPolygon with respect to the plane.
    /// Returns a bitmask of COPLANAR, FRONT, and BACK.
    /// This method matches the regular Mesh classify_polygon method.
    pub fn classify_polygon<S: Clone>(
        &self,
        polygon: &IndexedPolygon<S>,
        vertices: &[IndexedVertex],
    ) -> i8 {
        // Match the regular Mesh approach: check each vertex individually
        // This is more robust than plane-to-plane comparison
        let mut polygon_type: i8 = 0;

        for &vertex_idx in &polygon.indices {
            if vertex_idx < vertices.len() {
                let classification = self.orient_point(&vertices[vertex_idx].pos);
                polygon_type |= classification;
            }
        }

        polygon_type
    }

    /// Splits an IndexedPolygon by this plane, returning four buckets:
    /// `(coplanar_front, coplanar_back, front, back)`.
    /// This method matches the regular Mesh split_polygon implementation.
    #[allow(clippy::type_complexity)]
    pub fn split_polygon<S: Clone + Send + Sync + Debug>(
        &self,
        polygon: &IndexedPolygon<S>,
        vertices: &mut Vec<IndexedVertex>,
    ) -> (
        Vec<IndexedPolygon<S>>,
        Vec<IndexedPolygon<S>>,
        Vec<IndexedPolygon<S>>,
        Vec<IndexedPolygon<S>>,
    ) {
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();
        let mut front = Vec::new();
        let mut back = Vec::new();

        let normal = self.normal();

        // Classify each vertex of the polygon
        let types: Vec<i8> = polygon
            .indices
            .iter()
            .map(|&idx| {
                if idx < vertices.len() {
                    self.orient_point(&vertices[idx].pos)
                } else {
                    COPLANAR
                }
            })
            .collect();

        let polygon_type = types.iter().fold(0, |acc, &t| acc | t);

        // Dispatch the easy cases
        match polygon_type {
            COPLANAR => {
                if normal.dot(&polygon.plane.normal()) > 0.0 {
                    coplanar_front.push(polygon.clone());
                } else {
                    coplanar_back.push(polygon.clone());
                }
            },
            FRONT => front.push(polygon.clone()),
            BACK => back.push(polygon.clone()),

            // True spanning – do the split
            _ => {
                let mut split_front = Vec::<IndexedVertex>::new();
                let mut split_back = Vec::<IndexedVertex>::new();

                for i in 0..polygon.indices.len() {
                    // j is the vertex following i, we modulo by len to wrap around to the first vertex after the last
                    let j = (i + 1) % polygon.indices.len();
                    let type_i = types[i];
                    let type_j = types[j];
                    let idx_i = polygon.indices[i];
                    let idx_j = polygon.indices[j];

                    if idx_i >= vertices.len() || idx_j >= vertices.len() {
                        continue;
                    }

                    let vertex_i = &vertices[idx_i];
                    let vertex_j = &vertices[idx_j];

                    // If current vertex is definitely not behind plane, it goes to split_front
                    if type_i != BACK {
                        split_front.push(*vertex_i);
                    }
                    // If current vertex is definitely not in front, it goes to split_back
                    if type_i != FRONT {
                        split_back.push(*vertex_i);
                    }

                    // If the edge between these two vertices crosses the plane,
                    // compute intersection and add that intersection to both sets
                    if (type_i | type_j) == SPANNING {
                        let denom = normal.dot(&(vertex_j.pos - vertex_i.pos));
                        // Avoid dividing by zero
                        if denom.abs() > EPSILON {
                            let intersection =
                                (self.offset() - normal.dot(&vertex_i.pos.coords)) / denom;
                            let vertex_new = vertex_i.interpolate(vertex_j, intersection);
                            split_front.push(vertex_new);
                            split_back.push(vertex_new);
                        }
                    }
                }

                // Build new polygons from the front/back vertex lists
                // if they have at least 3 vertices
                if split_front.len() >= 3 {
                    // Add new vertices to the vertex array and get their indices
                    let mut front_indices = Vec::new();
                    for vertex in split_front {
                        vertices.push(vertex);
                        front_indices.push(vertices.len() - 1);
                    }
                    // **CRITICAL FIX**: Use original polygon plane instead of recomputing
                    // Recomputing the plane from split vertices can introduce numerical errors
                    // that cause gaps. Regular Mesh uses the original plane logic.
                    front.push(IndexedPolygon::new(
                        front_indices,
                        polygon.plane.clone(),
                        polygon.metadata.clone(),
                    ));
                }
                if split_back.len() >= 3 {
                    // Add new vertices to the vertex array and get their indices
                    let mut back_indices = Vec::new();
                    for vertex in split_back {
                        vertices.push(vertex);
                        back_indices.push(vertices.len() - 1);
                    }
                    // **CRITICAL FIX**: Use original polygon plane instead of recomputing
                    // Recomputing the plane from split vertices can introduce numerical errors
                    // that cause gaps. Regular Mesh uses the original plane logic.
                    back.push(IndexedPolygon::new(
                        back_indices,
                        polygon.plane.clone(),
                        polygon.metadata.clone(),
                    ));
                }
            },
        }

        (coplanar_front, coplanar_back, front, back)
    }

    /// Returns (T, T_inv), where:
    /// - `T` maps a point on this plane into XY plane (z=0) with the plane's normal going to +Z
    /// - `T_inv` is the inverse transform, mapping back
    ///
    /// **Mathematical Foundation**: This implements an orthonormal transformation:
    /// 1. **Rotation Matrix**: R = rotation_between(plane_normal, +Z)
    /// 2. **Translation**: Translate so plane passes through origin
    /// 3. **Combined Transform**: T = T₂ · R · T₁
    ///
    /// The transformation preserves distances and angles, enabling 2D algorithms
    /// to be applied to 3D planar geometry.
    pub fn to_xy_transform(&self) -> (Matrix4<Real>, Matrix4<Real>) {
        // Normal
        let n = self.normal();
        let n_len = n.norm();
        if n_len < EPSILON {
            // Degenerate plane, return identity
            return (Matrix4::identity(), Matrix4::identity());
        }

        // Normalize
        let norm_dir = n / n_len;

        // Rotate plane.normal -> +Z
        let rot = Rotation3::rotation_between(&norm_dir, &Vector3::z())
            .unwrap_or_else(Rotation3::identity);
        let iso_rot = Isometry3::from_parts(Translation3::identity(), rot.into());

        // We want to translate so that the plane's reference point
        //    (some point p0 with n·p0 = w) lands at z=0 in the new coords.
        // p0 = (plane.w / (n·n)) * n
        let denom = n.dot(&n);
        let p0_3d = norm_dir * (self.offset() / denom);
        let p0_rot = iso_rot.transform_point(&Point3::from(p0_3d));

        // We want p0_rot.z = 0, so we shift by -p0_rot.z
        let shift_z = -p0_rot.z;
        let iso_trans = Translation3::new(0.0, 0.0, shift_z);

        let transform_to_xy = iso_trans.to_homogeneous() * iso_rot.to_homogeneous();

        // Inverse for going back
        let transform_from_xy = transform_to_xy
            .try_inverse()
            .unwrap_or_else(Matrix4::identity);

        (transform_to_xy, transform_from_xy)
    }

    /// Split an IndexedPolygon by this plane for BSP operations
    /// Returns (coplanar_front, coplanar_back, front, back)
    /// This version properly handles spanning polygons by creating intersection vertices
    /// **FIXED**: Now uses plane-aware cache keys to prevent incorrect vertex sharing
    #[allow(clippy::type_complexity)]
    pub fn split_indexed_polygon_with_cache<S: Clone + Send + Sync + Debug>(
        &self,
        polygon: &IndexedPolygon<S>,
        vertices: &mut Vec<IndexedVertex>,
        edge_cache: &mut HashMap<PlaneEdgeCacheKey, usize>,
    ) -> (
        Vec<IndexedPolygon<S>>,
        Vec<IndexedPolygon<S>>,
        Vec<IndexedPolygon<S>>,
        Vec<IndexedPolygon<S>>,
    ) {
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();
        let mut front = Vec::new();
        let mut back = Vec::new();

        // Check if planes are coplanar first (optimization)
        // Use very strict criteria for coplanar detection to avoid false positives
        let poly_plane = &polygon.plane;
        let normal_dot = self.normal.dot(&poly_plane.normal);

        // Only treat as coplanar if:
        // 1. Normals are extremely close (almost exactly the same direction)
        // 2. Distances from origin are very close
        if normal_dot.abs() > 0.999999 && (self.w - poly_plane.w).abs() < EPSILON {
            // Planes are effectively coplanar
            if normal_dot > 0.0 {
                coplanar_front.push(polygon.clone());
            } else {
                coplanar_back.push(polygon.clone());
            }
            return (coplanar_front, coplanar_back, front, back);
        }

        // Not coplanar - need to check individual vertices for spanning case
        let mut types: Vec<i8> = Vec::new();
        let mut has_front = false;
        let mut has_back = false;

        // Classify all vertices
        for &idx in &polygon.indices {
            if idx >= vertices.len() {
                // Invalid vertex index - treat as coplanar
                types.push(COPLANAR);
                continue;
            }
            let vertex_type = self.orient_point(&vertices[idx].pos);
            types.push(vertex_type);

            if vertex_type == FRONT {
                has_front = true;
            } else if vertex_type == BACK {
                has_back = true;
            }
        }

        let polygon_type = if has_front && has_back {
            SPANNING
        } else if has_front {
            FRONT
        } else if has_back {
            BACK
        } else {
            COPLANAR
        };

        // Dispatch based on classification
        match polygon_type {
            COPLANAR => {
                // All vertices coplanar - check orientation relative to this plane
                if self.normal().dot(&polygon.plane.normal()) > 0.0 {
                    coplanar_front.push(polygon.clone());
                } else {
                    coplanar_back.push(polygon.clone());
                }
            },
            FRONT => front.push(polygon.clone()),
            BACK => back.push(polygon.clone()),
            SPANNING => {
                // **CRITICAL FIX**: Implement exact same algorithm as regular Mesh split_polygon
                // This ensures manifold topology preservation by maintaining correct vertex ordering
                let mut front_indices: Vec<usize> = Vec::new();
                let mut back_indices: Vec<usize> = Vec::new();

                for i in 0..polygon.indices.len() {
                    let j = (i + 1) % polygon.indices.len();
                    let idx_i = polygon.indices[i];
                    let idx_j = polygon.indices[j];

                    if idx_i >= vertices.len() || idx_j >= vertices.len() {
                        continue;
                    }

                    let type_i = types[i];
                    let type_j = types[j];

                    // **STEP 1**: Add current vertex to appropriate side(s) - EXACT MATCH to regular Mesh
                    if type_i != BACK {
                        front_indices.push(idx_i);
                    }
                    if type_i != FRONT {
                        back_indices.push(idx_i);
                    }

                    // **STEP 2**: Handle edge intersection - EXACT MATCH to regular Mesh
                    // If the edge between these two vertices crosses the plane,
                    // compute intersection and add that intersection to both sets
                    if (type_i | type_j) == SPANNING {
                        let cache_key = PlaneEdgeCacheKey::new(self, idx_i, idx_j);

                        let intersection_idx = if let Some(&cached_idx) = edge_cache.get(&cache_key) {
                            // Reuse cached intersection vertex
                            cached_idx
                        } else {
                            // Compute new intersection vertex - EXACT MATCH to regular Mesh
                            let vertex_i = &vertices[idx_i];
                            let vertex_j = &vertices[idx_j];
                            let denom = self.normal().dot(&(vertex_j.pos - vertex_i.pos));
                            // Avoid dividing by zero - EXACT MATCH to regular Mesh
                            if denom.abs() > EPSILON {
                                let t = (self.offset() - self.normal().dot(&vertex_i.pos.coords))
                                    / denom;
                                let intersection_vertex = vertex_i.interpolate(vertex_j, t);

                                // Add to vertex array and cache the index
                                vertices.push(intersection_vertex);
                                let new_idx = vertices.len() - 1;
                                edge_cache.insert(cache_key, new_idx);
                                new_idx
                            } else {
                                // Degenerate case - use first vertex
                                idx_i
                            }
                        };

                        // **CRITICAL**: Add intersection to BOTH polygons - EXACT MATCH to regular Mesh
                        front_indices.push(intersection_idx);
                        back_indices.push(intersection_idx);
                    }
                }

                // Create new polygons with proper vertex sharing
                if front_indices.len() >= 3 {
                    front.push(IndexedPolygon::new(
                        front_indices,
                        polygon.plane.clone(),
                        polygon.metadata.clone(),
                    ));
                }

                if back_indices.len() >= 3 {
                    back.push(IndexedPolygon::new(
                        back_indices,
                        polygon.plane.clone(),
                        polygon.metadata.clone(),
                    ));
                }
            },
            _ => {
                // Fallback - shouldn't happen
                coplanar_front.push(polygon.clone());
            },
        }

        (coplanar_front, coplanar_back, front, back)
    }

    /// Determine the orientation of another plane relative to this plane
    /// Uses a more robust geometric approach similar to Mesh implementation
    /// **CRITICAL FIX**: Properly handles inverted planes with opposite normals
    pub fn orient_plane(&self, other_plane: &Plane) -> i8 {
        // First check if planes are coplanar by comparing normals and distances
        let normal_dot = self.normal.dot(&other_plane.normal);
        let distance_diff = (self.w - other_plane.w).abs();

        // **CRITICAL FIX**: Check for opposite orientation first
        // If normals are nearly opposite (dot product close to -1), they're inverted planes
        if normal_dot < -0.999 {
            // Planes have opposite normals - this is the inverted case
            if distance_diff < EPSILON {
                // Same distance but opposite normals - this is a flipped coplanar plane
                // The inverted plane should be classified as BACK relative to the original
                return BACK;
            } else {
                // Different distances and opposite normals
                return if self.w > other_plane.w { FRONT } else { BACK };
            }
        }

        if normal_dot.abs() > 0.999 && distance_diff < EPSILON {
            // Planes are coplanar - need to determine relative orientation
            if normal_dot > 0.0 {
                // Same orientation - check which side of self the other plane's point lies
                // Use a point on the other plane relative to self's origin
                let test_distance = other_plane.w - self.normal.dot(&Point3::origin().coords);
                if test_distance > EPSILON {
                    FRONT
                } else if test_distance < -EPSILON {
                    BACK
                } else {
                    COPLANAR
                }
            } else {
                // This case should now be handled above, but keep for safety
                BACK
            }
        } else {
            // Planes are not coplanar - use normal comparison
            if normal_dot > EPSILON {
                FRONT
            } else if normal_dot < -EPSILON {
                BACK
            } else {
                COPLANAR
            }
        }
    }
}

/// Conversion from mesh::plane::Plane to IndexedMesh::plane::Plane
impl From<crate::mesh::plane::Plane> for Plane {
    fn from(mesh_plane: crate::mesh::plane::Plane) -> Self {
        let normal = mesh_plane.normal();
        let w = normal.dot(&mesh_plane.point_a.coords);
        Plane { normal, w }
    }
}

/// Conversion to mesh::plane::Plane for compatibility
impl From<Plane> for crate::mesh::plane::Plane {
    fn from(indexed_plane: Plane) -> Self {
        // Create three points on the plane
        let origin_on_plane = indexed_plane.normal * indexed_plane.w;
        let u = if indexed_plane.normal.x.abs() < 0.9 {
            Vector3::x().cross(&indexed_plane.normal).normalize()
        } else {
            Vector3::y().cross(&indexed_plane.normal).normalize()
        };
        let v = indexed_plane.normal.cross(&u);

        let point_a = Point3::from(origin_on_plane);
        let point_b = Point3::from(origin_on_plane + u);
        let point_c = Point3::from(origin_on_plane + v);

        crate::mesh::plane::Plane {
            point_a,
            point_b,
            point_c,
        }
    }
}

// External function for BSP operations that need to split polygons
// **FIXED**: Updated to use plane-aware cache keys
pub fn split_indexed_polygon<S: Clone + Send + Sync + Debug>(
    plane: &Plane,
    polygon: &IndexedPolygon<S>,
    vertices: &mut Vec<IndexedVertex>,
    edge_cache: &mut HashMap<PlaneEdgeCacheKey, usize>,
) -> (
    Vec<IndexedPolygon<S>>,
    Vec<IndexedPolygon<S>>,
    Vec<IndexedPolygon<S>>,
    Vec<IndexedPolygon<S>>,
) {
    plane.split_indexed_polygon_with_cache(polygon, vertices, edge_cache)
}
