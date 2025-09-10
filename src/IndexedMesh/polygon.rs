//! **IndexedMesh Polygon Operations**
//!
//! Optimized polygon operations specifically designed for IndexedMesh's indexed connectivity model.
//! This module provides index-aware polygon operations that leverage shared vertex storage
//! and eliminate redundant vertex copying for maximum performance.

use crate::IndexedMesh::IndexedMesh;
use crate::IndexedMesh::plane::Plane;
use crate::IndexedMesh::vertex::IndexedVertex;
use crate::float_types::{Real, parry3d::bounding_volume::Aabb};
use geo::{LineString, Polygon as GeoPolygon, coord};
use nalgebra::{Point3, Vector3};
use std::sync::OnceLock;
use std::fmt::Debug;

/// **IndexedPolygon: Zero-Copy Polygon for IndexedMesh**
///
/// Represents a polygon using vertex indices instead of storing vertex data directly.
/// This eliminates vertex duplication and enables efficient mesh operations.
#[derive(Debug, Clone)]
pub struct IndexedPolygon<S: Clone> {
    /// Indices into the mesh's vertex array
    pub indices: Vec<usize>,

    /// The plane on which this polygon lies
    pub plane: Plane,

    /// Lazily-computed bounding box
    pub bounding_box: OnceLock<Aabb>,

    /// Generic metadata
    pub metadata: Option<S>,
}

impl<S: Clone + PartialEq> PartialEq for IndexedPolygon<S> {
    fn eq(&self, other: &Self) -> bool {
        self.indices == other.indices
            && self.plane == other.plane
            && self.metadata == other.metadata
    }
}

impl<S: Clone + Send + Sync + Debug> IndexedPolygon<S> {
    /// Create a new IndexedPolygon from vertex indices
    pub fn new(indices: Vec<usize>, plane: Plane, metadata: Option<S>) -> Self {
        assert!(indices.len() >= 3, "degenerate indexed polygon");

        IndexedPolygon {
            indices,
            plane,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }



    /// **Index-Aware Bounding Box Computation**
    ///
    /// Compute bounding box using vertex indices, accessing shared vertex storage.
    pub fn bounding_box<T: Clone + Send + Sync + std::fmt::Debug>(
        &self,
        mesh: &IndexedMesh<T>,
    ) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            let mut mins = Point3::new(Real::MAX, Real::MAX, Real::MAX);
            let mut maxs = Point3::new(-Real::MAX, -Real::MAX, -Real::MAX);

            for &idx in &self.indices {
                if idx < mesh.vertices.len() {
                    let pos = mesh.vertices[idx].pos;
                    mins.x = mins.x.min(pos.x);
                    mins.y = mins.y.min(pos.y);
                    mins.z = mins.z.min(pos.z);
                    maxs.x = maxs.x.max(pos.x);
                    maxs.y = maxs.y.max(pos.y);
                    maxs.z = maxs.z.max(pos.z);
                }
            }
            Aabb::new(mins, maxs)
        })
    }

    /// **Index-Aware Polygon Flipping**
    ///
    /// Reverse winding order and flip plane normal using indexed operations.
    ///
    /// **CRITICAL**: Unlike Mesh, we cannot flip shared vertex normals without
    /// affecting other polygons. This is the correct approach for IndexedMesh.
    /// Vertex normals should be recomputed globally after CSG operations.
    pub fn flip(&mut self) {
        // Reverse vertex indices to flip winding order
        self.indices.reverse();

        // Flip the plane normal
        self.plane.flip();
    }

    /// Flip this polygon and also flip the normals of its vertices
    ///
    /// **CRITICAL FIX**: For IndexedMesh, we CANNOT flip shared vertex normals
    /// as they are used by multiple polygons. This was causing inconsistent
    /// normals and broken CSG operations. Only flip polygon winding and plane.
    pub fn flip_with_vertices(&mut self, _vertices: &mut [IndexedVertex]) {
        // Reverse vertex indices to flip winding order
        self.indices.reverse();

        // Flip the plane normal
        self.plane.flip();

        // **FIXED**: DO NOT flip shared vertex normals - this breaks IndexedMesh
        // Vertex normals will be recomputed correctly after CSG operations
        // via compute_vertex_normals() which considers all adjacent polygons
    }

    /// **Index-Aware Edge Iterator**
    ///
    /// Returns iterator over edge pairs using vertex indices.
    /// Each edge is represented as (start_index, end_index).
    pub fn edge_indices(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.indices
            .iter()
            .zip(self.indices.iter().cycle().skip(1))
            .map(|(&start, &end)| (start, end))
    }

    /// **Index-Aware Triangulation**
    ///
    /// Triangulate polygon using indexed vertices for maximum efficiency.
    /// Returns triangle indices instead of copying vertex data.
    pub fn triangulate_indices<T: Clone + Send + Sync + std::fmt::Debug>(
        &self,
        mesh: &IndexedMesh<T>,
    ) -> Vec<[usize; 3]> {
        if self.indices.len() < 3 {
            return Vec::new();
        }

        // Already a triangle
        if self.indices.len() == 3 {
            return vec![[self.indices[0], self.indices[1], self.indices[2]]];
        }

        // For more complex polygons, we need to project to 2D and triangulate
        let normal_3d = self.plane.normal().normalize();
        let (u, v) = build_orthonormal_basis(normal_3d);

        // Get first vertex as origin
        if self.indices[0] >= mesh.vertices.len() {
            return Vec::new();
        }
        let origin_3d = mesh.vertices[self.indices[0]].pos;

        #[cfg(feature = "earcut")]
        {
            // Project vertices to 2D
            let mut vertices_2d = Vec::with_capacity(self.indices.len());
            for &idx in &self.indices {
                if idx < mesh.vertices.len() {
                    let pos = mesh.vertices[idx].pos;
                    let offset = pos.coords - origin_3d.coords;
                    let x = offset.dot(&u);
                    let y = offset.dot(&v);
                    vertices_2d.push(coord! {x: x, y: y});
                }
            }

            use geo::TriangulateEarcut;
            let triangulation = GeoPolygon::new(LineString::new(vertices_2d), Vec::new())
                .earcut_triangles_raw();

            // Convert triangle indices back to mesh indices
            let mut triangles = Vec::with_capacity(triangulation.triangle_indices.len() / 3);
            for tri_chunk in triangulation.triangle_indices.chunks_exact(3) {
                if tri_chunk.iter().all(|&idx| idx < self.indices.len()) {
                    triangles.push([
                        self.indices[tri_chunk[0]],
                        self.indices[tri_chunk[1]],
                        self.indices[tri_chunk[2]],
                    ]);
                }
            }
            triangles
        }

        #[cfg(feature = "delaunay")]
        {
            // Similar implementation for delaunay triangulation
            // Project to 2D with spade's constraints
            #[allow(clippy::excessive_precision)]
            const MIN_ALLOWED_VALUE: Real = 1.793662034335766e-43;

            let mut vertices_2d = Vec::with_capacity(self.indices.len());
            for &idx in &self.indices {
                if idx < mesh.vertices.len() {
                    let pos = mesh.vertices[idx].pos;
                    let offset = pos.coords - origin_3d.coords;
                    let x = offset.dot(&u);
                    let y = offset.dot(&v);

                    let x_clamped = if x.abs() < MIN_ALLOWED_VALUE { 0.0 } else { x };
                    let y_clamped = if y.abs() < MIN_ALLOWED_VALUE { 0.0 } else { y };

                    if x.is_finite()
                        && y.is_finite()
                        && x_clamped.is_finite()
                        && y_clamped.is_finite()
                    {
                        vertices_2d.push(coord! {x: x_clamped, y: y_clamped});
                    }
                }
            }

            use geo::TriangulateSpade;
            let polygon_2d = GeoPolygon::new(LineString::new(vertices_2d), Vec::new());

            if let Ok(tris) = polygon_2d.constrained_triangulation(Default::default()) {
                // Convert back to mesh indices
                let mut triangles = Vec::with_capacity(tris.len());
                for _tri2d in tris {
                    // Map 2D triangle vertices back to original indices
                    // This is a simplified mapping - in practice, you'd need to
                    // match the 2D coordinates back to the original vertex indices
                    if self.indices.len() >= 3 {
                        // For now, use simple fan triangulation as fallback
                        for i in 1..self.indices.len() - 1 {
                            triangles.push([
                                self.indices[0],
                                self.indices[i],
                                self.indices[i + 1],
                            ]);
                        }
                    }
                }
                triangles
            } else {
                Vec::new()
            }
        }
    }

    /// **Index-Aware Subdivision**
    ///
    /// Subdivide polygon triangles using indexed operations.
    /// Creates new vertices at midpoints and adds them to the mesh.
    /// Returns triangle indices referencing both existing and newly created vertices.
    pub fn subdivide_indices<T: Clone + Send + Sync + std::fmt::Debug>(
        &self,
        mesh: &mut IndexedMesh<T>,
        subdivisions: core::num::NonZeroU32,
    ) -> Vec<[usize; 3]> {
        let base_triangles = self.triangulate_indices(mesh);
        let mut result = Vec::new();

        for tri_indices in base_triangles {
            let mut queue = vec![tri_indices];

            for _ in 0..subdivisions.get() {
                let mut next_level = Vec::new();
                for tri in queue {
                    // Subdivide this triangle by creating midpoint vertices
                    let subdivided = self.subdivide_triangle_indices(mesh, tri);
                    next_level.extend(subdivided);
                }
                queue = next_level;
            }
            result.extend(queue);
        }

        result
    }

    /// **Helper: Subdivide Single Triangle with Indices**
    ///
    /// Subdivide a single triangle into 4 smaller triangles by creating midpoint vertices.
    /// Adds new vertices to the mesh and returns triangle indices.
    fn subdivide_triangle_indices<T: Clone + Send + Sync + std::fmt::Debug>(
        &self,
        mesh: &mut IndexedMesh<T>,
        tri: [usize; 3],
    ) -> Vec<[usize; 3]> {
        // Get the three vertices of the triangle
        if tri[0] >= mesh.vertices.len() || tri[1] >= mesh.vertices.len() || tri[2] >= mesh.vertices.len() {
            return vec![tri]; // Return original if indices are invalid
        }

        let v0 = mesh.vertices[tri[0]];
        let v1 = mesh.vertices[tri[1]];
        let v2 = mesh.vertices[tri[2]];

        // Create midpoint vertices
        let v01 = v0.interpolate(&v1, 0.5);
        let v12 = v1.interpolate(&v2, 0.5);
        let v20 = v2.interpolate(&v0, 0.5);

        // Add new vertices to the mesh and get their indices
        let idx01 = mesh.vertices.len();
        mesh.vertices.push(v01);

        let idx12 = mesh.vertices.len();
        mesh.vertices.push(v12);

        let idx20 = mesh.vertices.len();
        mesh.vertices.push(v20);

        // Return 4 new triangles using the original and midpoint vertices
        vec![
            [tri[0], idx01, idx20],     // Corner triangle 0
            [idx01, tri[1], idx12],     // Corner triangle 1
            [idx20, idx12, tri[2]],     // Corner triangle 2
            [idx01, idx12, idx20],      // Center triangle
        ]
    }

    /// **Index-Aware Normal Calculation**
    ///
    /// Calculate polygon normal using indexed vertices.
    pub fn calculate_normal<T: Clone + Send + Sync + std::fmt::Debug>(
        &self,
        mesh: &IndexedMesh<T>,
    ) -> Vector3<Real> {
        let n = self.indices.len();
        if n < 3 {
            return Vector3::z();
        }

        let mut normal = Vector3::zeros();

        for i in 0..n {
            let current_idx = self.indices[i];
            let next_idx = self.indices[(i + 1) % n];

            if current_idx < mesh.vertices.len() && next_idx < mesh.vertices.len() {
                let current = mesh.vertices[current_idx].pos;
                let next = mesh.vertices[next_idx].pos;

                normal.x += (current.y - next.y) * (current.z + next.z);
                normal.y += (current.z - next.z) * (current.x + next.x);
                normal.z += (current.x - next.x) * (current.y + next.y);
            }
        }

        let mut poly_normal = normal.normalize();

        // Ensure consistency with plane normal
        if poly_normal.dot(&self.plane.normal()) < 0.0 {
            poly_normal = -poly_normal;
        }

        poly_normal
    }

    /// Metadata accessors
    pub const fn metadata(&self) -> Option<&S> {
        self.metadata.as_ref()
    }

    pub const fn metadata_mut(&mut self) -> Option<&mut S> {
        self.metadata.as_mut()
    }

    pub fn set_metadata(&mut self, data: S) {
        self.metadata = Some(data);
    }

    /// **Set New Normal (Index-Aware)**
    ///
    /// Recompute this polygon's normal from all vertices, then set all vertices' normals to match (flat shading).
    /// This modifies the mesh's vertex normals directly using indexed operations.
    /// This method matches the regular Mesh polygon.set_new_normal() method.
    pub fn set_new_normal<T: Clone + Send + Sync + std::fmt::Debug>(
        &mut self,
        mesh: &mut IndexedMesh<T>,
    ) {
        // Calculate the new normal
        let new_normal = self.calculate_normal(mesh);

        // Update the plane normal
        self.plane.normal = new_normal;

        // Set all referenced vertices' normals to match the plane (flat shading)
        for &idx in &self.indices {
            if let Some(vertex) = mesh.vertices.get_mut(idx) {
                vertex.normal = new_normal;
            }
        }
    }

    /// **Index-Aware Edge Iterator with Vertex References**
    ///
    /// Returns iterator over edge pairs with actual vertex references.
    /// This matches the regular Mesh polygon.edges() method signature.
    pub fn edges<'a, T: Clone + Send + Sync + std::fmt::Debug>(
        &'a self,
        mesh: &'a IndexedMesh<T>,
    ) -> impl Iterator<Item = (&'a crate::IndexedMesh::vertex::IndexedVertex, &'a crate::IndexedMesh::vertex::IndexedVertex)> + 'a {
        self.indices
            .iter()
            .zip(self.indices.iter().cycle().skip(1))
            .filter_map(move |(&start_idx, &end_idx)| {
                if let (Some(start_vertex), Some(end_vertex)) = (
                    mesh.vertices.get(start_idx),
                    mesh.vertices.get(end_idx)
                ) {
                    Some((start_vertex, end_vertex))
                } else {
                    None
                }
            })
    }

    /// **Index-Aware Triangulation with Vertex Data**
    ///
    /// Triangulate polygon returning actual triangles (not just indices).
    /// This matches the regular Mesh polygon.triangulate() method signature.
    pub fn triangulate<T: Clone + Send + Sync + std::fmt::Debug>(
        &self,
        mesh: &IndexedMesh<T>,
    ) -> Vec<[crate::IndexedMesh::vertex::IndexedVertex; 3]> {
        let triangle_indices = self.triangulate_indices(mesh);

        triangle_indices
            .into_iter()
            .filter_map(|[i0, i1, i2]| {
                if let (Some(v0), Some(v1), Some(v2)) = (
                    mesh.vertices.get(i0),
                    mesh.vertices.get(i1),
                    mesh.vertices.get(i2)
                ) {
                    Some([*v0, *v1, *v2])
                } else {
                    None
                }
            })
            .collect()
    }

    /// **Index-Aware Triangle Subdivision with Vertex Data**
    ///
    /// Subdivide polygon triangles returning actual triangles (not just indices).
    /// This matches the regular Mesh polygon.subdivide_triangles() method signature.
    pub fn subdivide_triangles<T: Clone + Send + Sync + std::fmt::Debug>(
        &self,
        mesh: &IndexedMesh<T>,
        subdivisions: core::num::NonZeroU32,
    ) -> Vec<[crate::IndexedMesh::vertex::IndexedVertex; 3]> {
        // Get base triangles
        let base_triangles = self.triangulate(mesh);

        // Subdivide each triangle
        let mut result = Vec::new();
        for triangle in base_triangles {
            let mut current_triangles = vec![triangle];

            // Apply subdivision levels
            for _ in 0..subdivisions.get() {
                let mut next_triangles = Vec::new();
                for tri in current_triangles {
                    next_triangles.extend(subdivide_triangle(tri));
                }
                current_triangles = next_triangles;
            }

            result.extend(current_triangles);
        }

        result
    }

    /// **Convert Subdivision Triangles to IndexedPolygons**
    ///
    /// Convert subdivision triangles back to polygons for CSG operations.
    /// Each triangle becomes a triangular polygon with the same metadata.
    /// This matches the regular Mesh polygon.subdivide_to_polygons() method signature.
    pub fn subdivide_to_polygons<T: Clone + Send + Sync + std::fmt::Debug>(
        &self,
        mesh: &mut IndexedMesh<T>,
        subdivisions: core::num::NonZeroU32,
    ) -> Vec<IndexedPolygon<S>> {
        // Use subdivide_indices to get triangle indices (vertices already added to mesh)
        let triangle_indices = self.subdivide_indices(mesh, subdivisions);

        triangle_indices
            .into_iter()
            .filter_map(|indices| {
                // Validate indices
                if indices.len() == 3 && indices.iter().all(|&idx| idx < mesh.vertices.len()) {
                    // Create plane from the triangle vertices
                    let v0 = mesh.vertices[indices[0]];
                    let v1 = mesh.vertices[indices[1]];
                    let v2 = mesh.vertices[indices[2]];

                    let plane = crate::IndexedMesh::plane::Plane::from_indexed_vertices(
                        vec![v0, v1, v2]
                    );

                    Some(IndexedPolygon::new(indices.to_vec(), plane, self.metadata.clone()))
                } else {
                    None
                }
            })
            .collect()
    }

    /// **Convert IndexedPolygon to Regular Polygon**
    ///
    /// Convert this indexed polygon to a regular polygon by resolving
    /// vertex indices to actual vertex positions.
    ///
    /// # Parameters
    /// - `vertices`: The vertex array to resolve indices against
    ///
    /// # Returns
    /// A regular Polygon with resolved vertex positions
    ///
    /// **⚠️ DEPRECATED**: This method creates a dependency on the regular Mesh module.
    /// Use native IndexedPolygon operations instead for better performance and memory efficiency.
    #[deprecated(since = "0.20.1", note = "Use native IndexedPolygon operations instead of converting to regular Polygon")]
    pub fn to_regular_polygon(&self, vertices: &[crate::IndexedMesh::vertex::IndexedVertex]) -> crate::mesh::polygon::Polygon<S> {
        let resolved_vertices: Vec<crate::mesh::vertex::Vertex> = self.indices.iter()
            .filter_map(|&idx| {
                if idx < vertices.len() {
                    // IndexedVertex has pos field, regular Vertex needs position and normal
                    let pos = vertices[idx].pos;
                    let normal = Vector3::zeros(); // Default normal, will be recalculated
                    Some(crate::mesh::vertex::Vertex::new(pos, normal))
                } else {
                    None
                }
            })
            .collect();

        crate::mesh::polygon::Polygon::new(resolved_vertices, self.metadata.clone())
    }






}

/// Build orthonormal basis for 2D projection
pub fn build_orthonormal_basis(n: Vector3<Real>) -> (Vector3<Real>, Vector3<Real>) {
    let n = n.normalize();

    let other = if n.x.abs() < n.y.abs() && n.x.abs() < n.z.abs() {
        Vector3::x()
    } else if n.y.abs() < n.z.abs() {
        Vector3::y()
    } else {
        Vector3::z()
    };

    let v = n.cross(&other).normalize();
    let u = v.cross(&n).normalize();

    (u, v)
}





/// **Helper function to subdivide a triangle**
///
/// Subdivides a single triangle into 4 smaller triangles by adding midpoint vertices.
/// This matches the regular Mesh polygon.subdivide_triangle() helper function.
pub fn subdivide_triangle(tri: [crate::IndexedMesh::vertex::IndexedVertex; 3]) -> Vec<[crate::IndexedMesh::vertex::IndexedVertex; 3]> {
    let v01 = tri[0].interpolate(&tri[1], 0.5);
    let v12 = tri[1].interpolate(&tri[2], 0.5);
    let v20 = tri[2].interpolate(&tri[0], 0.5);

    vec![
        [tri[0], v01, v20],     // Corner triangle 0
        [v01, tri[1], v12],     // Corner triangle 1 - FIXED: Now matches Mesh ordering
        [v20, v12, tri[2]],     // Corner triangle 2 - FIXED: Now matches Mesh ordering
        [v01, v12, v20],        // Center triangle
    ]
}
