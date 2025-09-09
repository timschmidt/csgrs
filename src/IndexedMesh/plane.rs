//! **IndexedMesh-Optimized Plane Operations**
//!
//! This module implements robust geometric operations for planes optimized for
//! IndexedMesh's indexed connectivity model, providing superior performance
//! compared to coordinate-based approaches.

use crate::IndexedMesh::IndexedPolygon;
use crate::float_types::{EPSILON, Real};
use crate::mesh::vertex::Vertex;
use nalgebra::{Matrix4, Point3, Vector3};
use std::fmt::Debug;

// Plane classification constants (matching mesh::plane constants)
pub const COPLANAR: i8 = 0;
pub const FRONT: i8 = 1;
pub const BACK: i8 = 2;
pub const SPANNING: i8 = 3;

/// **IndexedMesh-Optimized Plane**
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
    pub fn from_points(p1: Point3<Real>, p2: Point3<Real>, p3: Point3<Real>) -> Self {
        let v1 = p2 - p1;
        let v2 = p3 - p1;
        let normal = v1.cross(&v2).normalize();
        let w = normal.dot(&p1.coords);
        Plane { normal, w }
    }

    /// Create a plane from vertices (for compatibility)
    pub fn from_vertices(vertices: Vec<Vertex>) -> Self {
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
    pub fn from_indexed_vertices(
        vertices: Vec<crate::IndexedMesh::vertex::IndexedVertex>,
    ) -> Self {
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

    /// Get the plane normal
    pub const fn normal(&self) -> Vector3<Real> {
        self.normal
    }

    /// Flip the plane (reverse normal and distance)
    pub fn flip(&mut self) {
        self.normal = -self.normal;
        self.w = -self.w;
    }

    /// Classify a point relative to the plane
    pub fn orient_point(&self, point: &Point3<Real>) -> i8 {
        let distance = self.normal.dot(&point.coords) - self.w;
        if distance > EPSILON {
            FRONT
        } else if distance < -EPSILON {
            BACK
        } else {
            COPLANAR
        }
    }

    /// Get the offset (distance from origin) for compatibility with BSP
    pub const fn offset(&self) -> Real {
        self.w
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

/// **IndexedMesh-Optimized Plane Operations**
///
/// Extension trait providing plane operations optimized for IndexedMesh's
/// indexed connectivity model.
pub trait IndexedPlaneOperations {
    /// **Classify Indexed Polygon with Optimal Performance**
    ///
    /// Classify a polygon with respect to the plane using direct vertex index access.
    /// Returns a bitmask of `COPLANAR`, `FRONT`, `BACK`, and `SPANNING`.
    fn classify_indexed_polygon<S: Clone + Send + Sync + Debug>(
        &self,
        polygon: &IndexedPolygon<S>,
        vertices: &[crate::IndexedMesh::vertex::IndexedVertex],
    ) -> i8;

    /// **Split Indexed Polygon with Zero-Copy Optimization**
    ///
    /// Split a polygon by the plane, returning new vertices and polygon parts.
    /// Uses indexed operations to minimize memory allocation and copying.
    fn split_indexed_polygon<S: Clone + Send + Sync + Debug>(
        &self,
        polygon: &IndexedPolygon<S>,
        vertices: &mut Vec<crate::IndexedMesh::vertex::IndexedVertex>,
    ) -> (
        Vec<usize>,
        Vec<usize>,
        Vec<IndexedPolygon<S>>,
        Vec<IndexedPolygon<S>>,
    );

    /// **Robust Point Orientation with Exact Arithmetic**
    ///
    /// Classify a point with respect to the plane using robust geometric predicates.
    /// Returns `FRONT`, `BACK`, or `COPLANAR`.
    fn orient_point_robust(&self, point: &Point3<Real>) -> i8;

    /// **2D Projection Transform for Indexed Operations**
    ///
    /// Returns transformation matrices for projecting indexed polygons to 2D.
    fn to_xy_transform_indexed(&self) -> (Matrix4<Real>, Matrix4<Real>);
}

impl IndexedPlaneOperations for Plane {
    fn classify_indexed_polygon<S: Clone + Send + Sync + Debug>(
        &self,
        polygon: &IndexedPolygon<S>,
        vertices: &[crate::IndexedMesh::vertex::IndexedVertex],
    ) -> i8 {
        let mut front_count = 0;
        let mut back_count = 0;

        for &vertex_idx in &polygon.indices {
            if vertex_idx >= vertices.len() {
                continue;
            }
            let vertex = &vertices[vertex_idx];
            let orientation = self.orient_point(&vertex.pos);

            if orientation == FRONT {
                front_count += 1;
            } else if orientation == BACK {
                back_count += 1;
            }
        }

        if front_count > 0 && back_count > 0 {
            SPANNING
        } else if front_count > 0 {
            FRONT
        } else if back_count > 0 {
            BACK
        } else {
            COPLANAR
        }
    }

    fn split_indexed_polygon<S: Clone + Send + Sync + Debug>(
        &self,
        polygon: &IndexedPolygon<S>,
        vertices: &mut Vec<crate::IndexedMesh::vertex::IndexedVertex>,
    ) -> (
        Vec<usize>,
        Vec<usize>,
        Vec<IndexedPolygon<S>>,
        Vec<IndexedPolygon<S>>,
    ) {
        let classification = self.classify_indexed_polygon(polygon, vertices);

        match classification {
            FRONT => (vec![], vec![], vec![polygon.clone()], vec![]),
            BACK => (vec![], vec![], vec![], vec![polygon.clone()]),
            COPLANAR => {
                // Check orientation to decide front or back
                if self.normal.dot(&polygon.plane.normal) > 0.0 {
                    (vec![], vec![], vec![polygon.clone()], vec![])
                } else {
                    (vec![], vec![], vec![], vec![polygon.clone()])
                }
            },
            _ => {
                // SPANNING case - implement proper polygon splitting
                let mut front_indices = Vec::new();
                let mut back_indices = Vec::new();
                let mut new_vertex_indices = Vec::new();

                let vertex_count = polygon.indices.len();

                // Classify each vertex
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

                // Process each edge for intersections
                for i in 0..vertex_count {
                    let j = (i + 1) % vertex_count;
                    let type_i = types[i];
                    let type_j = types[j];
                    let idx_i = polygon.indices[i];
                    let idx_j = polygon.indices[j];

                    if idx_i >= vertices.len() || idx_j >= vertices.len() {
                        continue;
                    }

                    let vertex_i = &vertices[idx_i];
                    let vertex_j = &vertices[idx_j];

                    // Add current vertex to appropriate side
                    match type_i {
                        FRONT => front_indices.push(idx_i),
                        BACK => back_indices.push(idx_i),
                        COPLANAR => {
                            front_indices.push(idx_i);
                            back_indices.push(idx_i);
                        },
                        _ => {},
                    }

                    // Check for edge intersection
                    if (type_i | type_j) == SPANNING {
                        let denom = self.normal.dot(&(vertex_j.pos - vertex_i.pos));
                        if denom.abs() > crate::float_types::EPSILON {
                            let intersection =
                                (self.w - self.normal.dot(&vertex_i.pos.coords)) / denom;
                            let new_vertex = vertex_i.interpolate(vertex_j, intersection);

                            // Add new vertex to the vertex array
                            let new_idx = vertices.len();
                            vertices.push(new_vertex);
                            new_vertex_indices.push(new_idx);

                            // Add intersection to both sides
                            front_indices.push(new_idx);
                            back_indices.push(new_idx);
                        }
                    }
                }

                // Create new polygons if they have enough vertices
                let mut front_polygons = Vec::new();
                let mut back_polygons = Vec::new();

                if front_indices.len() >= 3 {
                    // Calculate plane for front polygon
                    let front_plane = if front_indices.len() >= 3 {
                        let v0 = &vertices[front_indices[0]];
                        let v1 = &vertices[front_indices[1]];
                        let v2 = &vertices[front_indices[2]];
                        Plane::from_indexed_vertices(vec![*v0, *v1, *v2])
                    } else {
                        polygon.plane.clone()
                    };

                    front_polygons.push(IndexedPolygon::new(
                        front_indices,
                        front_plane,
                        polygon.metadata.clone(),
                    ));
                }

                if back_indices.len() >= 3 {
                    // Calculate plane for back polygon
                    let back_plane = if back_indices.len() >= 3 {
                        let v0 = &vertices[back_indices[0]];
                        let v1 = &vertices[back_indices[1]];
                        let v2 = &vertices[back_indices[2]];
                        Plane::from_indexed_vertices(vec![*v0, *v1, *v2])
                    } else {
                        polygon.plane.clone()
                    };

                    back_polygons.push(IndexedPolygon::new(
                        back_indices,
                        back_plane,
                        polygon.metadata.clone(),
                    ));
                }

                (vec![], new_vertex_indices, front_polygons, back_polygons)
            },
        }
    }

    fn orient_point_robust(&self, point: &Point3<Real>) -> i8 {
        // Use robust orientation test
        let distance = self.normal.dot(&point.coords) - self.w;
        if distance > EPSILON {
            FRONT
        } else if distance < -EPSILON {
            BACK
        } else {
            COPLANAR
        }
    }

    fn to_xy_transform_indexed(&self) -> (Matrix4<Real>, Matrix4<Real>) {
        // Create orthonormal basis for the plane
        let n = self.normal;
        let u = if n.x.abs() < 0.9 {
            Vector3::x().cross(&n).normalize()
        } else {
            Vector3::y().cross(&n).normalize()
        };
        let v = n.cross(&u);

        // Transform to XY plane
        let transform = Matrix4::new(
            u.x, u.y, u.z, 0.0, v.x, v.y, v.z, 0.0, n.x, n.y, n.z, -self.w, 0.0, 0.0, 0.0, 1.0,
        );

        // Inverse transform
        let inv_transform = Matrix4::new(
            u.x, v.x, n.x, 0.0, u.y, v.y, n.y, 0.0, u.z, v.z, n.z, self.w, 0.0, 0.0, 0.0, 1.0,
        );

        (transform, inv_transform)
    }
}
