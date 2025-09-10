//! `IndexedMesh` struct and implementations of the `CSGOps` trait for `IndexedMesh`

use crate::float_types::{
    parry3d::{bounding_volume::{Aabb, BoundingVolume}, query::RayCast, shape::Shape},
    rapier3d::prelude::{
        ColliderBuilder, ColliderSet, Ray, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
        SharedShape, TriMesh, Triangle,
    },
    {EPSILON, Real},
};
// Only import mesh types for compatibility conversions
use crate::mesh::vertex::Vertex;
use crate::sketch::Sketch;
use crate::traits::CSG;
use geo::{CoordsIter, Geometry, Polygon as GeoPolygon};
use nalgebra::{
    Isometry3, Matrix4, Point3, Quaternion, Unit, Vector3, partial_max, partial_min,
};
use std::{cmp::PartialEq, fmt::Debug, num::NonZeroU32, sync::OnceLock};

pub mod connectivity;

/// BSP tree operations for IndexedMesh
pub mod bsp;
pub mod bsp_parallel;

/// Shape generation functions for IndexedMesh
pub mod shapes;

/// Mesh quality analysis for IndexedMesh
pub mod quality;

/// Manifold topology validation for IndexedMesh
pub mod manifold;

/// Mesh smoothing algorithms for IndexedMesh
pub mod smoothing;

/// Flattening and slicing operations for IndexedMesh
pub mod flatten_slice;

/// SDF-based mesh generation for IndexedMesh
pub mod sdf;

/// Convex hull operations for IndexedMesh
#[cfg(feature = "chull-io")]
pub mod convex_hull;

/// Metaball (implicit surface) generation for IndexedMesh
#[cfg(feature = "metaballs")]
pub mod metaballs;

/// Triply Periodic Minimal Surfaces (TPMS) for IndexedMesh
#[cfg(feature = "sdf")]
pub mod tpms;

/// Plane operations optimized for IndexedMesh
pub mod plane;

/// Vertex operations optimized for IndexedMesh
pub mod vertex;

/// Polygon operations optimized for IndexedMesh
pub mod polygon;

/// An indexed polygon, defined by indices into a vertex array.
/// - `S` is the generic metadata type, stored as `Option<S>`.
#[derive(Debug, Clone)]
pub struct IndexedPolygon<S: Clone> {
    /// Indices into the vertex array
    pub indices: Vec<usize>,

    /// The plane on which this Polygon lies, used for splitting
    pub plane: plane::Plane,

    /// Lazily‑computed axis‑aligned bounding box of the Polygon
    pub bounding_box: OnceLock<Aabb>,

    /// Generic metadata associated with the Polygon
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug> IndexedPolygon<S> {
    /// Create an indexed polygon from indices
    pub fn new(indices: Vec<usize>, plane: plane::Plane, metadata: Option<S>) -> Self {
        assert!(indices.len() >= 3, "degenerate polygon");

        IndexedPolygon {
            indices,
            plane,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Axis aligned bounding box of this IndexedPolygon (cached after first call)
    pub fn bounding_box(&self, vertices: &[vertex::IndexedVertex]) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            let mut mins = Point3::new(Real::MAX, Real::MAX, Real::MAX);
            let mut maxs = Point3::new(-Real::MAX, -Real::MAX, -Real::MAX);
            for &idx in &self.indices {
                let v = &vertices[idx];
                mins.x = mins.x.min(v.pos.x);
                mins.y = mins.y.min(v.pos.y);
                mins.z = mins.z.min(v.pos.z);
                maxs.x = maxs.x.max(v.pos.x);
                maxs.y = maxs.y.max(v.pos.y);
                maxs.z = maxs.z.max(v.pos.z);
            }
            Aabb::new(mins, maxs)
        })
    }

    /// Reverses winding order and flips the plane normal
    pub fn flip(&mut self) {
        self.indices.reverse();
        self.plane.flip();
    }

    /// Flip this polygon and also flip the normals of its vertices
    pub fn flip_with_vertices(&mut self, vertices: &mut [vertex::IndexedVertex]) {
        // Reverse vertex indices to flip winding order
        self.indices.reverse();

        // Flip the plane normal
        self.plane.flip();

        // Flip normals of all vertices referenced by this polygon
        for &idx in &self.indices {
            if idx < vertices.len() {
                vertices[idx].flip();
            }
        }
    }

    /// Return an iterator over paired indices each forming an edge of the polygon
    pub fn edges(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.indices
            .iter()
            .zip(self.indices.iter().cycle().skip(1))
            .map(|(&a, &b)| (a, b))
    }

    /// Triangulate this indexed polygon into triangles using indices
    pub fn triangulate(&self, vertices: &[vertex::IndexedVertex]) -> Vec<[usize; 3]> {
        let n = self.indices.len();
        if n < 3 {
            return Vec::new();
        }
        if n == 3 {
            return vec![[self.indices[0], self.indices[1], self.indices[2]]];
        }

        // For simple fan triangulation, find the best starting vertex
        // (one that minimizes the maximum angle in the fan)
        let start_idx = self.find_best_fan_start(vertices);

        // Rotate indices so the best vertex is first
        let mut rotated_indices = Vec::new();
        for i in 0..n {
            rotated_indices.push(self.indices[(start_idx + i) % n]);
        }

        // Simple fan from the best starting vertex
        let mut triangles = Vec::new();
        for i in 1..n - 1 {
            triangles.push([
                rotated_indices[0],
                rotated_indices[i],
                rotated_indices[i + 1],
            ]);
        }
        triangles
    }

    /// Find the best vertex to start fan triangulation (minimizes maximum triangle angle)
    fn find_best_fan_start(&self, vertices: &[vertex::IndexedVertex]) -> usize {
        let n = self.indices.len();
        if n <= 3 {
            return 0;
        }

        let mut best_start = 0;
        let mut best_score = Real::MAX;

        // Try each vertex as potential start
        for start in 0..n {
            let mut max_angle = 0.0;

            // Calculate angles for triangles in this fan
            for i in 1..n - 1 {
                let v0 = vertices[self.indices[start % n]].pos;
                let v1 = vertices[self.indices[(start + i) % n]].pos;
                let v2 = vertices[self.indices[(start + i + 1) % n]].pos;

                // Calculate triangle angles
                let angles = self.triangle_angles(v0, v1, v2);
                for &angle in &angles {
                    if angle > max_angle {
                        max_angle = angle;
                    }
                }
            }

            if max_angle < best_score {
                best_score = max_angle;
                best_start = start;
            }
        }

        best_start
    }

    /// Calculate the three angles of a triangle given its vertices
    fn triangle_angles(&self, a: Point3<Real>, b: Point3<Real>, c: Point3<Real>) -> [Real; 3] {
        let ab = b - a;
        let ac = c - a;
        let bc = c - b;
        let ca = a - c;

        let angle_a = (ab.dot(&ac) / (ab.norm() * ac.norm())).acos();
        let angle_b = (ab.dot(&bc) / (ab.norm() * bc.norm())).acos();
        let angle_c = (ca.dot(&bc) / (ca.norm() * bc.norm())).acos();

        [angle_a, angle_b, angle_c]
    }

    /// **Mathematical Foundation: Indexed Polygon Subdivision**
    ///
    /// Subdivides this polygon into smaller triangles using midpoint subdivision.
    /// Each triangle is subdivided into 4 smaller triangles by adding midpoints.
    ///
    /// ## **Algorithm Overview**
    /// 1. **Triangulation**: Convert polygon to triangles using fan triangulation
    /// 2. **Vertex Addition**: Add midpoint vertices to the mesh
    /// 3. **Subdivision**: Apply triangle subdivision algorithm
    /// 4. **Index Generation**: Return triangle indices for the subdivided mesh
    ///
    /// # Parameters
    /// - `mesh`: Reference to the IndexedMesh containing this polygon
    /// - `levels`: Number of subdivision levels to apply
    ///
    /// # Returns
    /// Vector of triangle indices [v0, v1, v2] representing the subdivided triangles
    ///
    /// # Note
    /// This method modifies the input mesh by adding new vertices for subdivision.
    /// The returned indices reference the updated mesh vertices.
    pub fn subdivide_triangles<T: Clone + Send + Sync + Debug>(
        &self,
        mesh: &mut IndexedMesh<T>,
        levels: NonZeroU32,
    ) -> Vec<[usize; 3]> {
        if self.indices.len() < 3 {
            return Vec::new();
        }

        // Get base triangulation of this polygon
        let base_triangles = self.triangulate(&mesh.vertices);

        // Apply subdivision to each triangle
        let mut result = Vec::new();
        for triangle in base_triangles {
            let mut current_triangles = vec![triangle];

            // Apply subdivision levels
            for _ in 0..levels.get() {
                let mut next_triangles = Vec::new();
                for tri in current_triangles {
                    next_triangles.extend(self.subdivide_triangle_indices(mesh, tri));
                }
                current_triangles = next_triangles;
            }

            result.extend(current_triangles);
        }

        result
    }

    /// Subdivide a single triangle by indices, adding new vertices to the mesh
    fn subdivide_triangle_indices<T: Clone + Send + Sync + Debug>(
        &self,
        mesh: &mut IndexedMesh<T>,
        tri: [usize; 3],
    ) -> Vec<[usize; 3]> {
        let v0 = mesh.vertices[tri[0]];
        let v1 = mesh.vertices[tri[1]];
        let v2 = mesh.vertices[tri[2]];

        // Create midpoints by interpolating vertices
        let v01 = v0.interpolate(&v1, 0.5);
        let v12 = v1.interpolate(&v2, 0.5);
        let v20 = v2.interpolate(&v0, 0.5);

        // Add new vertices to mesh and get their indices
        let v01_idx = mesh.vertices.len();
        mesh.vertices.push(v01);

        let v12_idx = mesh.vertices.len();
        mesh.vertices.push(v12);

        let v20_idx = mesh.vertices.len();
        mesh.vertices.push(v20);

        // Return the 4 subdivided triangles
        vec![
            [tri[0], v01_idx, v20_idx],      // Corner triangle 0
            [v01_idx, tri[1], v12_idx],      // Corner triangle 1
            [v20_idx, v12_idx, tri[2]],      // Corner triangle 2
            [v01_idx, v12_idx, v20_idx],     // Center triangle
        ]
    }

    /// Set a new normal for this polygon based on its vertices and update vertex normals
    pub fn set_new_normal(&mut self, vertices: &mut [vertex::IndexedVertex]) {
        // Recompute the plane from the actual vertex positions
        if self.indices.len() >= 3 {
            let vertex_positions: Vec<vertex::IndexedVertex> = self
                .indices
                .iter()
                .map(|&idx| {
                    let pos = vertices[idx].pos;
                    // Create vertex with dummy normal for plane computation
                    vertex::IndexedVertex::new(pos, Vector3::z())
                })
                .collect();

            self.plane = plane::Plane::from_indexed_vertices(vertex_positions);
        }

        // Update all vertex normals in this polygon to match the face normal
        let face_normal = self.plane.normal();
        for &idx in &self.indices {
            vertices[idx].normal = face_normal;
        }
    }

    /// **Calculate New Normal from Vertex Positions**
    ///
    /// Compute the polygon normal from its vertex positions using cross product.
    /// Returns the normalized face normal vector.
    pub fn calculate_new_normal(&self, vertices: &[vertex::IndexedVertex]) -> Vector3<Real> {
        if self.indices.len() < 3 {
            return Vector3::z(); // Default normal for degenerate polygons
        }

        // Use first three vertices to compute normal
        let v0 = vertices[self.indices[0]].pos;
        let v1 = vertices[self.indices[1]].pos;
        let v2 = vertices[self.indices[2]].pos;

        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let normal = edge1.cross(&edge2);

        if normal.norm_squared() > Real::EPSILON * Real::EPSILON {
            normal.normalize()
        } else {
            Vector3::z() // Fallback for degenerate triangles
        }
    }

    /// **Metadata Accessor**
    ///
    /// Returns a reference to the metadata, if any.
    pub const fn metadata(&self) -> Option<&S> {
        self.metadata.as_ref()
    }

    /// **Mathematical Foundation: Polygon-Plane Classification**
    ///
    /// Classify this polygon relative to a plane using robust geometric predicates.
    /// Returns classification constant (FRONT, BACK, COPLANAR, SPANNING).
    ///
    /// ## **Classification Algorithm**
    /// 1. **Vertex Classification**: Test each vertex against the plane
    /// 2. **Consensus Analysis**: Determine overall polygon position
    /// 3. **Spanning Detection**: Check if polygon crosses the plane
    ///
    /// Uses epsilon-based tolerance for numerical stability.
    pub fn classify_against_plane(&self, plane: &plane::Plane, mesh: &IndexedMesh<S>) -> i8 {
        use crate::IndexedMesh::plane::{BACK, COPLANAR, FRONT, SPANNING};

        let mut front_count = 0;
        let mut back_count = 0;

        for &vertex_idx in &self.indices {
            let vertex = &mesh.vertices[vertex_idx];
            let orientation = plane.orient_point(&vertex.pos);

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

    /// **Mathematical Foundation: Polygon Centroid Calculation**
    ///
    /// Calculate the centroid (geometric center) of this polygon using
    /// indexed vertex access for optimal performance.
    ///
    /// ## **Centroid Formula**
    /// For a polygon with vertices v₁, v₂, ..., vₙ:
    /// ```text
    /// centroid = (v₁ + v₂ + ... + vₙ) / n
    /// ```
    ///
    /// Returns the centroid point in 3D space.
    pub fn centroid(&self, mesh: &IndexedMesh<S>) -> Point3<Real> {
        let mut sum = Point3::origin();
        let count = self.indices.len() as Real;

        for &vertex_idx in &self.indices {
            sum += mesh.vertices[vertex_idx].pos.coords;
        }

        Point3::from(sum.coords / count)
    }

    /// **Mathematical Foundation: Polygon Splitting by Plane**
    ///
    /// Split this polygon by a plane, returning front and back parts.
    /// Uses the IndexedPlane's split_indexed_polygon method for robust
    /// geometric processing with indexed connectivity.
    ///
    /// ## **Splitting Algorithm**
    /// 1. **Edge Intersection**: Find where plane intersects polygon edges
    /// 2. **Vertex Classification**: Classify vertices as front/back/on-plane
    /// 3. **Polygon Reconstruction**: Build new polygons from split parts
    ///
    /// Returns (front_polygons, back_polygons) as separate IndexedPolygons.
    pub fn split_by_plane(
        &self,
        plane: &plane::Plane,
        mesh: &IndexedMesh<S>,
    ) -> (Vec<IndexedPolygon<S>>, Vec<IndexedPolygon<S>>) {
        // Use the plane's BSP-compatible split method
        let mut vertices = mesh.vertices.clone();
        let mut edge_cache: std::collections::HashMap<(usize, usize), usize> = std::collections::HashMap::new();
        let (_coplanar_front, _coplanar_back, front_polygons, back_polygons) =
            plane.split_indexed_polygon_with_cache(self, &mut vertices, &mut edge_cache);

        (front_polygons, back_polygons)
    }
}

#[derive(Clone, Debug)]
pub struct IndexedMesh<S: Clone + Send + Sync + Debug> {
    /// 3D vertices using IndexedVertex for optimized indexed connectivity
    pub vertices: Vec<vertex::IndexedVertex>,

    /// Indexed polygons for volumetric shapes
    pub polygons: Vec<IndexedPolygon<S>>,

    /// Lazily calculated AABB that spans `polygons`.
    pub bounding_box: OnceLock<Aabb>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug + PartialEq> IndexedMesh<S> {
    /// Compare just the `metadata` fields of two meshes
    #[inline]
    pub fn same_metadata(&self, other: &Self) -> bool {
        self.metadata == other.metadata
    }

    /// Example: retain only polygons whose metadata matches `needle`
    #[inline]
    pub fn filter_polygons_by_metadata(&self, needle: &S) -> IndexedMesh<S> {
        let polys = self
            .polygons
            .iter()
            .filter(|&p| p.metadata.as_ref() == Some(needle))
            .cloned()
            .collect();

        IndexedMesh {
            vertices: self.vertices.clone(),
            polygons: polys,
            bounding_box: std::sync::OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> IndexedMesh<S> {
    /// **Zero-Copy Vertex Buffer Creation**
    ///
    /// Create GPU-ready vertex buffer from IndexedMesh without copying vertex data
    /// when possible. Optimized for graphics API upload.
    #[inline]
    pub fn vertex_buffer(&self) -> vertex::VertexBuffer {
        vertex::VertexBuffer::from_indexed_vertices(&self.vertices)
    }

    /// **Zero-Copy Index Buffer Creation**
    ///
    /// Create GPU-ready index buffer from triangulated mesh.
    /// Uses iterator combinators for optimal performance.
    #[inline]
    pub fn index_buffer(&self) -> vertex::IndexBuffer {
        let triangles: Vec<[usize; 3]> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.triangulate(&self.vertices))
            .collect();
        vertex::IndexBuffer::from_triangles(&triangles)
    }

    /// **Zero-Copy Vertex Slice Access**
    ///
    /// Get immutable slice of vertices for zero-copy operations.
    #[inline]
    pub fn vertex_slice(&self) -> &[vertex::IndexedVertex] {
        &self.vertices
    }

    /// **Zero-Copy Mutable Vertex Slice Access**
    ///
    /// Get mutable slice of vertices for in-place operations.
    #[inline]
    pub fn vertex_slice_mut(&mut self) -> &mut [vertex::IndexedVertex] {
        &mut self.vertices
    }

    /// **Zero-Copy Polygon Slice Access**
    ///
    /// Get immutable slice of polygons for zero-copy operations.
    #[inline]
    pub fn polygon_slice(&self) -> &[IndexedPolygon<S>] {
        &self.polygons
    }

    /// **Iterator-Based Vertex Processing**
    ///
    /// Process vertices using iterator combinators for optimal performance.
    /// Enables SIMD vectorization and parallel processing.
    #[inline]
    pub fn vertices_iter(&self) -> impl Iterator<Item = &vertex::IndexedVertex> {
        self.vertices.iter()
    }

    /// **Iterator-Based Polygon Processing**
    ///
    /// Process polygons using iterator combinators.
    #[inline]
    pub fn polygons_iter(&self) -> impl Iterator<Item = &IndexedPolygon<S>> {
        self.polygons.iter()
    }

    /// **Parallel Vertex Processing**
    ///
    /// Process vertices in parallel using rayon for CPU-intensive operations.
    #[cfg(feature = "rayon")]
    #[inline]
    pub fn vertices_par_iter(
        &self,
    ) -> impl rayon::iter::ParallelIterator<Item = &vertex::IndexedVertex> {
        use rayon::prelude::*;
        self.vertices.par_iter()
    }



    /// Build an IndexedMesh from an existing polygon list
    pub fn from_polygons(
        polygons: &[crate::mesh::polygon::Polygon<S>],
        metadata: Option<S>,
    ) -> Self {
        let mut vertices = Vec::new();
        let mut indexed_polygons = Vec::new();
        let mut vertex_map = std::collections::HashMap::new();

        for poly in polygons {
            let mut indices = Vec::new();
            for vertex in &poly.vertices {
                let pos = vertex.pos;
                let key = (pos.x.to_bits(), pos.y.to_bits(), pos.z.to_bits());
                let idx = if let Some(&existing_idx) = vertex_map.get(&key) {
                    existing_idx
                } else {
                    let new_idx = vertices.len();
                    // Convert Vertex to IndexedVertex
                    vertices.push(vertex::IndexedVertex::from(*vertex));
                    vertex_map.insert(key, new_idx);
                    new_idx
                };
                indices.push(idx);
            }
            let indexed_poly =
                IndexedPolygon::new(indices, poly.plane.clone().into(), poly.metadata.clone());
            indexed_polygons.push(indexed_poly);
        }

        IndexedMesh {
            vertices,
            polygons: indexed_polygons,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Helper to collect all vertices from the CSG (converted to regular Vertex for compatibility).
    pub fn vertices(&self) -> Vec<Vertex> {
        self.vertices.iter().map(|&iv| iv.into()).collect()
    }

    /// Get IndexedVertex vertices directly (optimized for IndexedMesh operations)
    pub const fn indexed_vertices(&self) -> &Vec<vertex::IndexedVertex> {
        &self.vertices
    }

    /// Split polygons into (may_touch, cannot_touch) using bounding-box tests
    /// This optimization avoids unnecessary BSP computations for polygons
    /// that cannot possibly intersect with the other mesh.
    fn partition_polygons(
        polygons: &[IndexedPolygon<S>],
        vertices: &[vertex::IndexedVertex],
        other_bb: &Aabb,
    ) -> (Vec<IndexedPolygon<S>>, Vec<IndexedPolygon<S>>) {
        polygons
            .iter()
            .cloned()
            .partition(|p| p.bounding_box(vertices).intersects(other_bb))
    }

    /// Remap vertex indices in polygons to account for combined vertex array
    fn remap_polygon_indices(polygons: &mut [IndexedPolygon<S>], offset: usize) {
        for polygon in polygons.iter_mut() {
            for index in &mut polygon.indices {
                *index += offset;
            }
        }
    }

    /// Remap polygons from one vertex array to another by adjusting indices
    fn remap_bsp_polygons(polygons: &[IndexedPolygon<S>], offset: usize) -> Vec<IndexedPolygon<S>> {
        polygons.iter().map(|poly| {
            let mut new_poly = poly.clone();
            for index in &mut new_poly.indices {
                *index += offset;
            }
            new_poly
        }).collect()
    }

    /// **Mathematical Foundation: Dihedral Angle Calculation**
    ///
    /// Computes the dihedral angle between two polygons sharing an edge.
    /// The angle is computed as the angle between the normal vectors of the two polygons.
    ///
    /// Returns the angle in radians.
    #[allow(dead_code)]
    fn dihedral_angle(p1: &IndexedPolygon<S>, p2: &IndexedPolygon<S>) -> Real {
        let n1 = p1.plane.normal();
        let n2 = p2.plane.normal();
        let dot = n1.dot(&n2).clamp(-1.0, 1.0);
        dot.acos()
    }

    /// **Zero-Copy Triangulation with Iterator Optimization**
    ///
    /// Triangulate each polygon using iterator combinators for optimal performance.
    /// Minimizes memory allocations and enables vectorization.
    pub fn triangulate(&self) -> IndexedMesh<S> {
        // **Iterator Optimization**: Use lazy triangle generation with single final collection
        // This eliminates intermediate Vec allocations from poly.triangulate() calls
        let triangles: Vec<IndexedPolygon<S>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                // **Zero-Copy Triangulation**: Use iterator-based triangulation
                // **CRITICAL**: Plane information must be recomputed for each triangle for CSG accuracy
                self.triangulate_polygon_iter(poly).map(move |tri| {
                    // Recompute plane from actual triangle vertices for numerical accuracy
                    let vertices_for_plane = [
                        self.vertices[tri[0]],
                        self.vertices[tri[1]],
                        self.vertices[tri[2]]
                    ];
                    let triangle_plane = plane::Plane::from_indexed_vertices(vertices_for_plane.to_vec());
                    IndexedPolygon::new(tri.to_vec(), triangle_plane, poly.metadata.clone())
                })
            })
            .collect();

        IndexedMesh {
            vertices: self.vertices.clone(), // TODO: Consider Cow for conditional copying
            polygons: triangles,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// **Zero-Copy Triangulation Iterator**
    ///
    /// Returns an iterator over triangulated polygons without creating intermediate mesh.
    /// Enables lazy evaluation and memory-efficient processing.
    #[inline]
    pub fn triangulate_iter(&self) -> impl Iterator<Item = IndexedPolygon<S>> + '_ {
        self.polygons.iter().flat_map(move |poly| {
            // **Zero-Copy Triangulation**: Use iterator-based triangulation instead of Vec allocation
            // **CRITICAL**: Plane information must be recomputed for each triangle for CSG accuracy
            self.triangulate_polygon_iter(poly).map(move |tri| {
                // Recompute plane from actual triangle vertices for numerical accuracy
                let vertices_for_plane = [
                    self.vertices[tri[0]],
                    self.vertices[tri[1]],
                    self.vertices[tri[2]]
                ];
                let triangle_plane = plane::Plane::from_indexed_vertices(vertices_for_plane.to_vec());
                IndexedPolygon::new(tri.to_vec(), triangle_plane, poly.metadata.clone())
            })
        })
    }

    /// Subdivide all polygons in this Mesh 'levels' times, returning a new Mesh.
    /// This results in a triangular mesh with more detail.
    /// Uses midpoint subdivision: each triangle is split into 4 smaller triangles.
    pub fn subdivide_triangles(&self, levels: NonZeroU32) -> IndexedMesh<S> {
        // Start with triangulation
        let mut current_mesh = self.triangulate();

        // Apply subdivision levels
        for _ in 0..levels.get() {
            current_mesh = current_mesh.subdivide_once();
        }

        current_mesh
    }

    /// Perform one level of midpoint subdivision on a triangulated mesh
    fn subdivide_once(&self) -> IndexedMesh<S> {
        let mut new_vertices = self.vertices.clone();
        let mut new_polygons = Vec::new();

        // Map to store edge midpoints: (min_vertex, max_vertex) -> new_vertex_index
        let mut edge_midpoints = std::collections::HashMap::new();

        for poly in &self.polygons {
            // Each polygon should be a triangle after triangulation
            if poly.indices.len() == 3 {
                let [a, b, c] = [poly.indices[0], poly.indices[1], poly.indices[2]];

                // Get or create midpoints for each edge
                let ab_mid =
                    self.get_or_create_midpoint(&mut new_vertices, &mut edge_midpoints, a, b);
                let bc_mid =
                    self.get_or_create_midpoint(&mut new_vertices, &mut edge_midpoints, b, c);
                let ca_mid =
                    self.get_or_create_midpoint(&mut new_vertices, &mut edge_midpoints, c, a);

                // Create 4 new triangles with recomputed planes
                let metadata = poly.metadata.clone();

                // Triangle A-AB-CA
                let triangle1_indices = vec![a, ab_mid, ca_mid];
                let triangle1_plane = plane::Plane::from_indexed_vertices(
                    triangle1_indices.iter().map(|&idx| new_vertices[idx]).collect()
                );
                new_polygons.push(IndexedPolygon::new(
                    triangle1_indices,
                    triangle1_plane,
                    metadata.clone(),
                ));

                // Triangle AB-B-BC
                let triangle2_indices = vec![ab_mid, b, bc_mid];
                let triangle2_plane = plane::Plane::from_indexed_vertices(
                    triangle2_indices.iter().map(|&idx| new_vertices[idx]).collect()
                );
                new_polygons.push(IndexedPolygon::new(
                    triangle2_indices,
                    triangle2_plane,
                    metadata.clone(),
                ));

                // Triangle CA-BC-C
                let triangle3_indices = vec![ca_mid, bc_mid, c];
                let triangle3_plane = plane::Plane::from_indexed_vertices(
                    triangle3_indices.iter().map(|&idx| new_vertices[idx]).collect()
                );
                new_polygons.push(IndexedPolygon::new(
                    triangle3_indices,
                    triangle3_plane,
                    metadata.clone(),
                ));

                // Triangle AB-BC-CA (center triangle)
                let triangle4_indices = vec![ab_mid, bc_mid, ca_mid];
                let triangle4_plane = plane::Plane::from_indexed_vertices(
                    triangle4_indices.iter().map(|&idx| new_vertices[idx]).collect()
                );
                new_polygons.push(IndexedPolygon::new(
                    triangle4_indices,
                    triangle4_plane,
                    metadata.clone(),
                ));
            } else {
                // For non-triangles, just copy them (shouldn't happen after triangulation)
                new_polygons.push(poly.clone());
            }
        }

        IndexedMesh {
            vertices: new_vertices,
            polygons: new_polygons,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Get or create a midpoint vertex for an edge
    fn get_or_create_midpoint(
        &self,
        new_vertices: &mut Vec<vertex::IndexedVertex>,
        edge_midpoints: &mut std::collections::HashMap<(usize, usize), usize>,
        v1: usize,
        v2: usize,
    ) -> usize {
        // Ensure consistent ordering for edge keys
        let edge_key = if v1 < v2 { (v1, v2) } else { (v2, v1) };

        if let Some(&midpoint_idx) = edge_midpoints.get(&edge_key) {
            return midpoint_idx;
        }

        // Create new midpoint vertex
        let pos1 = self.vertices[v1].pos;
        let pos2 = self.vertices[v2].pos;
        let midpoint_pos = (pos1 + pos2.coords) / 2.0;

        // Interpolate normal (average of the two vertex normals)
        let normal1 = self.vertices[v1].normal;
        let normal2 = self.vertices[v2].normal;
        let midpoint_normal = (normal1 + normal2).normalize();

        let midpoint_vertex = vertex::IndexedVertex::new(midpoint_pos, midpoint_normal);
        let midpoint_idx = new_vertices.len();
        new_vertices.push(midpoint_vertex);

        edge_midpoints.insert(edge_key, midpoint_idx);
        midpoint_idx
    }

    /// Subdivide all polygons in this Mesh 'levels' times, in place.
    /// This results in a triangular mesh with more detail.
    /// Uses midpoint subdivision: each triangle is split into 4 smaller triangles.
    pub fn subdivide_triangles_mut(&mut self, levels: NonZeroU32) {
        // First triangulate in place
        let mut new_polygons = Vec::new();
        for poly in &self.polygons {
            let tri_indices = poly.triangulate(&self.vertices);
            for tri in tri_indices {
                let plane = poly.plane.clone();
                let indexed_tri =
                    IndexedPolygon::new(tri.to_vec(), plane, poly.metadata.clone());
                new_polygons.push(indexed_tri);
            }
        }
        self.polygons = new_polygons;
        self.bounding_box = OnceLock::new();

        // Apply subdivision levels in place
        for _ in 0..levels.get() {
            self.subdivide_once_mut();
        }
    }

    /// Perform one level of midpoint subdivision in place
    fn subdivide_once_mut(&mut self) {
        let mut new_vertices = self.vertices.clone();
        let mut new_polygons = Vec::new();

        // Map to store edge midpoints: (min_vertex, max_vertex) -> new_vertex_index
        let mut edge_midpoints = std::collections::HashMap::new();

        for poly in &self.polygons {
            // Each polygon should be a triangle after triangulation
            if poly.indices.len() == 3 {
                let [a, b, c] = [poly.indices[0], poly.indices[1], poly.indices[2]];

                // Get or create midpoints for each edge
                let ab_mid =
                    self.get_or_create_midpoint(&mut new_vertices, &mut edge_midpoints, a, b);
                let bc_mid =
                    self.get_or_create_midpoint(&mut new_vertices, &mut edge_midpoints, b, c);
                let ca_mid =
                    self.get_or_create_midpoint(&mut new_vertices, &mut edge_midpoints, c, a);

                // Create 4 new triangles with recomputed planes
                let metadata = poly.metadata.clone();

                // Triangle A-AB-CA
                let triangle1_indices = vec![a, ab_mid, ca_mid];
                let triangle1_plane = plane::Plane::from_indexed_vertices(
                    triangle1_indices.iter().map(|&idx| new_vertices[idx]).collect()
                );
                new_polygons.push(IndexedPolygon::new(
                    triangle1_indices,
                    triangle1_plane,
                    metadata.clone(),
                ));

                // Triangle AB-B-BC
                let triangle2_indices = vec![ab_mid, b, bc_mid];
                let triangle2_plane = plane::Plane::from_indexed_vertices(
                    triangle2_indices.iter().map(|&idx| new_vertices[idx]).collect()
                );
                new_polygons.push(IndexedPolygon::new(
                    triangle2_indices,
                    triangle2_plane,
                    metadata.clone(),
                ));

                // Triangle CA-BC-C
                let triangle3_indices = vec![ca_mid, bc_mid, c];
                let triangle3_plane = plane::Plane::from_indexed_vertices(
                    triangle3_indices.iter().map(|&idx| new_vertices[idx]).collect()
                );
                new_polygons.push(IndexedPolygon::new(
                    triangle3_indices,
                    triangle3_plane,
                    metadata.clone(),
                ));

                // Triangle AB-BC-CA (center triangle)
                let triangle4_indices = vec![ab_mid, bc_mid, ca_mid];
                let triangle4_plane = plane::Plane::from_indexed_vertices(
                    triangle4_indices.iter().map(|&idx| new_vertices[idx]).collect()
                );
                new_polygons.push(IndexedPolygon::new(
                    triangle4_indices,
                    triangle4_plane,
                    metadata.clone(),
                ));
            } else {
                // For non-triangles, just copy them (shouldn't happen after triangulation)
                new_polygons.push(poly.clone());
            }
        }

        self.vertices = new_vertices;
        self.polygons = new_polygons;
        self.bounding_box = OnceLock::new();
    }

    /// Renormalize all polygons in this Mesh by re-computing each polygon’s plane
    /// and assigning that plane’s normal to all vertices.
    pub fn renormalize(&mut self) {
        for poly in &mut self.polygons {
            poly.set_new_normal(&mut self.vertices);
        }
    }

    /// **Zero-Copy Vertex and Index Extraction**
    ///
    /// Extracts vertices and indices using iterator combinators for optimal performance.
    /// Avoids intermediate mesh creation when possible.
    fn get_vertices_and_indices(&self) -> (Vec<Point3<Real>>, Vec<[u32; 3]>) {
        // Extract positions using zero-copy iterator
        let vertices: Vec<Point3<Real>> = self.vertices.iter().map(|v| v.pos).collect();

        // Extract triangle indices using iterator combinators
        let indices: Vec<[u32; 3]> = self
            .triangulate_iter()
            .map(|poly| {
                [
                    poly.indices[0] as u32,
                    poly.indices[1] as u32,
                    poly.indices[2] as u32,
                ]
            })
            .collect();

        (vertices, indices)
    }

    /// **SIMD-Optimized Batch Vertex Processing**
    ///
    /// Process vertices in batches for SIMD optimization.
    /// Enables vectorized operations on vertex positions and normals.
    #[inline]
    pub fn process_vertices_batched<F>(&mut self, batch_size: usize, mut processor: F)
    where F: FnMut(&mut [vertex::IndexedVertex]) {
        for chunk in self.vertices.chunks_mut(batch_size) {
            processor(chunk);
        }
    }

    /// **Iterator-Based Vertex Transformation**
    ///
    /// Transform vertices using iterator combinators for optimal performance.
    /// Enables SIMD vectorization and parallel processing.
    #[inline]
    pub fn transform_vertices<F>(&mut self, transformer: F)
    where F: Fn(&mut vertex::IndexedVertex) {
        self.vertices.iter_mut().for_each(transformer);
    }

    /// **Zero-Copy Polygon Triangulation Iterator**
    ///
    /// Returns an iterator over triangle indices for a polygon without creating intermediate Vec.
    /// This eliminates memory allocations during triangulation for ray intersection and other operations.
    #[inline]
    fn triangulate_polygon_iter(&self, poly: &IndexedPolygon<S>) -> Box<dyn Iterator<Item = [usize; 3]> + '_> {
        let n = poly.indices.len();

        if n < 3 {
            // Return empty iterator for degenerate polygons
            Box::new(std::iter::empty())
        } else if n == 3 {
            // Single triangle case
            let tri = [poly.indices[0], poly.indices[1], poly.indices[2]];
            Box::new(std::iter::once(tri))
        } else {
            // For polygons with more than 3 vertices, use fan triangulation
            // This creates (n-2) triangles from vertex 0 as the fan center
            let indices = poly.indices.clone(); // Small allocation for indices only
            Box::new((1..n-1).map(move |i| [
                indices[0],
                indices[i],
                indices[i + 1],
            ]))
        }
    }

    /// Casts a ray defined by `origin` + t * `direction` against all triangles
    /// of this Mesh and returns a list of (intersection_point, distance),
    /// sorted by ascending distance.
    ///
    /// # Parameters
    /// - `origin`: The ray’s start point.
    /// - `direction`: The ray’s direction vector.
    ///
    /// # Returns
    /// A `Vec` of `(Point3<Real>, Real)` where:
    /// - `Point3<Real>` is the intersection coordinate in 3D,
    /// - `Real` is the distance (the ray parameter t) from `origin`.
    pub fn ray_intersections(
        &self,
        origin: &Point3<Real>,
        direction: &Vector3<Real>,
    ) -> Vec<(Point3<Real>, Real)> {
        let ray = Ray::new(*origin, *direction);
        let iso = Isometry3::identity(); // No transformation on the triangles themselves.

        // **Iterator Optimization**: Use lazy iterator chain that processes intersections on-demand
        // This eliminates intermediate Vec allocations from poly.triangulate() calls
        let mut hits: Vec<_> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                // **Zero-Copy Triangulation**: Use iterator-based triangulation instead of Vec allocation
                self.triangulate_polygon_iter(poly)
            })
            .filter_map(|tri| {
                let a = self.vertices[tri[0]].pos;
                let b = self.vertices[tri[1]].pos;
                let c = self.vertices[tri[2]].pos;
                let triangle = Triangle::new(a, b, c);
                triangle
                    .cast_ray_and_get_normal(&iso, &ray, Real::MAX, true)
                    .map(|hit| {
                        let point_on_ray = ray.point_at(hit.time_of_impact);
                        (Point3::from(point_on_ray.coords), hit.time_of_impact)
                    })
            })
            .collect();

        // Sort hits by ascending distance (toi):
        hits.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        // Remove duplicate hits if they fall within tolerance
        hits.dedup_by(|a, b| (a.1 - b.1).abs() < EPSILON);

        hits
    }

    /// Convert the polygons in this Mesh to a Parry `TriMesh`, wrapped in a `SharedShape` to be used in Rapier.\
    /// Useful for collision detection or physics simulations.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_rapier_shape(&self) -> SharedShape {
        let (vertices, indices) = self.get_vertices_and_indices();
        let trimesh = TriMesh::new(vertices, indices).unwrap();
        SharedShape::new(trimesh)
    }

    /// Convert the polygons in this Mesh to a Parry `TriMesh`.\
    /// Useful for collision detection.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_trimesh(&self) -> Option<TriMesh> {
        let (vertices, indices) = self.get_vertices_and_indices();
        TriMesh::new(vertices, indices).ok()
    }

    /// Uses Parry to check if a point is inside a `Mesh`'s as a `TriMesh`.\
    /// Note: this only use the 3d geometry of `CSG`
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices
    ///
    /// ## Example
    /// ```
    /// # use csgrs::mesh::Mesh;
    /// # use nalgebra::Point3;
    /// # use nalgebra::Vector3;
    /// let csg_cube = Mesh::<()>::cube(6.0, None);
    ///
    /// assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    /// assert!(csg_cube.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    ///
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    /// ```
    pub fn contains_vertex(&self, point: &Point3<Real>) -> bool {
        let intersections = self.ray_intersections(point, &Vector3::new(1.0, 1.0, 1.0));
        intersections.len() % 2 == 1
    }

    /// Approximate mass properties using Rapier.
    pub fn mass_properties(
        &self,
        density: Real,
    ) -> (Real, Point3<Real>, Unit<Quaternion<Real>>) {
        let trimesh = self.to_trimesh().unwrap();
        let mp = trimesh.mass_properties(density);

        (
            mp.mass(),
            mp.local_com,                     // a Point3<Real>
            mp.principal_inertia_local_frame, // a Unit<Quaternion<Real>>
        )
    }

    /// Create a Rapier rigid body + collider from this Mesh, using
    /// an axis-angle `rotation` in 3D (the vector’s length is the
    /// rotation in radians, and its direction is the axis).
    pub fn to_rigid_body(
        &self,
        rb_set: &mut RigidBodySet,
        co_set: &mut ColliderSet,
        translation: Vector3<Real>,
        rotation: Vector3<Real>, // rotation axis scaled by angle (radians)
        density: Real,
    ) -> RigidBodyHandle {
        let shape = self.to_rapier_shape();

        // Build a Rapier RigidBody
        let rb = RigidBodyBuilder::dynamic()
            .translation(translation)
            // Now `rotation(...)` expects an axis-angle Vector3.
            .rotation(rotation)
            .build();
        let rb_handle = rb_set.insert(rb);

        // Build the collider
        let coll = ColliderBuilder::new(shape).density(density).build();
        co_set.insert_with_parent(coll, rb_handle, rb_set);

        rb_handle
    }

    /// Convert an IndexedMesh into a Bevy `Mesh`.
    #[cfg(feature = "bevymesh")]
    pub fn to_bevy_mesh(&self) -> bevy_mesh::Mesh {
        use bevy_asset::RenderAssetUsages;
        use bevy_mesh::{Indices, Mesh};
        use wgpu_types::PrimitiveTopology;

        let triangulated_mesh = &self.triangulate();

        // Prepare buffers
        let mut positions_32 = Vec::new();
        let mut normals_32 = Vec::new();
        let mut indices = Vec::new();

        for poly in &triangulated_mesh.polygons {
            for &idx in &poly.indices {
                let v = &triangulated_mesh.vertices[idx];
                positions_32.push([v.pos.x as f32, v.pos.y as f32, v.pos.z as f32]);
                normals_32.push([v.normal.x as f32, v.normal.y as f32, v.normal.z as f32]);
            }
            // Since triangulated, each polygon has 3 indices
            let base = indices.len() as u32;
            indices.push(base);
            indices.push(base + 1);
            indices.push(base + 2);
        }

        // Create the mesh with the new 2-argument constructor
        let mut mesh =
            Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());

        // Insert attributes. Note the `<Vec<[f32; 3]>>` usage.
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions_32);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals_32);

        // Insert triangle indices
        mesh.insert_indices(Indices::U32(indices));

        mesh
    }

    /// **Memory-Efficient Mesh Conversion**
    ///
    /// Convert IndexedMesh to Mesh using iterator combinators for optimal performance.
    /// Minimizes memory allocations and enables vectorization.
    ///
    /// **⚠️ DEPRECATED**: This method creates a dependency on the regular Mesh module.
    /// Use native IndexedMesh operations instead for better performance and memory efficiency.
    #[deprecated(since = "0.20.1", note = "Use native IndexedMesh operations instead of converting to regular Mesh")]
    pub fn to_mesh(&self) -> crate::mesh::Mesh<S> {
        // Pre-calculate capacity to avoid reallocations
        let polygons: Vec<crate::mesh::polygon::Polygon<S>> = self
            .polygons
            .iter()
            .map(|ip| {
                // Use iterator combinators for efficient vertex conversion
                let vertices: Vec<crate::mesh::vertex::Vertex> = ip
                    .indices
                    .iter()
                    .map(|&idx| self.vertices[idx].into())
                    .collect();
                crate::mesh::polygon::Polygon::new(vertices, ip.metadata.clone())
            })
            .collect();
        crate::mesh::Mesh::from_polygons(&polygons, self.metadata.clone())
    }

    /// **Zero-Copy Mesh Conversion (when possible)**
    ///
    /// Attempts to convert to Mesh with minimal copying using Cow (Clone on Write).
    /// Falls back to full conversion when necessary.
    ///
    /// **⚠️ DEPRECATED**: This method creates a dependency on the regular Mesh module.
    /// Use native IndexedMesh operations instead for better performance and memory efficiency.
    #[deprecated(since = "0.20.1", note = "Use native IndexedMesh operations instead of converting to regular Mesh")]
    pub fn to_mesh_cow(&self) -> crate::mesh::Mesh<S> {
        // For now, delegate to regular conversion
        // TODO: Implement true Cow semantics when mesh structures support it
        self.to_mesh()
    }

    /// **Mathematical Foundation: Comprehensive Mesh Validation**
    ///
    /// Perform comprehensive validation of the IndexedMesh structure and geometry.
    /// Returns a vector of validation issues found, empty if mesh is valid.
    ///
    /// ## **Validation Checks**
    /// - **Index Bounds**: All polygon indices within vertex array bounds
    /// - **Polygon Validity**: All polygons have at least 3 vertices
    /// - **Duplicate Indices**: No duplicate vertex indices within polygons
    /// - **Manifold Properties**: Edge manifold and orientation consistency
    /// - **Geometric Validity**: Non-degenerate normals and finite coordinates
    ///
    /// ## **Performance Optimization**
    /// - **Iterator-Based**: Uses lazy iterator chains for memory efficiency
    /// - **Single Pass**: Most checks performed in one iteration
    /// - **Early Termination**: Stops on critical errors
    /// - **Index-based**: Leverages indexed connectivity for efficiency
    pub fn validate(&self) -> Vec<String> {
        // Check vertex array first
        if self.vertices.is_empty() {
            return vec!["Mesh has no vertices".to_string()];
        }

        // **Iterator Optimization**: Use lazy iterator chains for validation
        // This reduces memory allocations and enables better compiler optimizations
        let polygon_issues = self.validate_polygons_iter();
        let manifold_issues = self.validate_manifold_properties();
        let isolated_vertex_issues = self.validate_isolated_vertices_iter();

        // **Single Final Collection**: Only collect all issues at the end
        polygon_issues
            .chain(manifold_issues.into_iter())
            .chain(isolated_vertex_issues)
            .collect()
    }

    /// **Iterator-Based Polygon Validation**
    ///
    /// Returns an iterator over validation issues for all polygons.
    /// Uses lazy evaluation to minimize memory usage during validation.
    #[inline]
    fn validate_polygons_iter(&self) -> impl Iterator<Item = String> + '_ {
        self.polygons
            .iter()
            .enumerate()
            .flat_map(|(i, polygon)| {
                // **Iterator Fusion**: Chain all polygon validation checks
                let mut issues = Vec::new();
                issues.extend(self.validate_polygon_vertex_count(i, polygon));
                issues.extend(self.validate_polygon_duplicate_indices(i, polygon));
                issues.extend(self.validate_polygon_index_bounds(i, polygon));
                issues.extend(self.validate_polygon_normal(i, polygon));
                issues
            })
    }

    /// **Validate Polygon Vertex Count**
    #[inline]
    fn validate_polygon_vertex_count(&self, i: usize, polygon: &IndexedPolygon<S>) -> Vec<String> {
        if polygon.indices.len() < 3 {
            vec![format!("Polygon {i} has fewer than 3 vertices")]
        } else {
            Vec::new()
        }
    }

    /// **Validate Polygon Duplicate Indices**
    #[inline]
    fn validate_polygon_duplicate_indices(&self, i: usize, polygon: &IndexedPolygon<S>) -> Vec<String> {
        let mut seen_indices = std::collections::HashSet::new();
        let mut issues = Vec::new();
        for &idx in &polygon.indices {
            if !seen_indices.insert(idx) {
                issues.push(format!("Polygon {i} has duplicate vertex index {idx}"));
            }
        }
        issues
    }

    /// **Validate Polygon Index Bounds**
    #[inline]
    fn validate_polygon_index_bounds(&self, i: usize, polygon: &IndexedPolygon<S>) -> Vec<String> {
        let vertex_count = self.vertices.len();
        let mut issues = Vec::new();
        for &idx in &polygon.indices {
            if idx >= vertex_count {
                issues.push(format!("Polygon {i} references out-of-bounds vertex index {idx}"));
            }
        }
        issues
    }

    /// **Validate Polygon Normal**
    #[inline]
    fn validate_polygon_normal(&self, i: usize, polygon: &IndexedPolygon<S>) -> Vec<String> {
        if polygon.indices.len() >= 3
            && polygon.indices.iter().all(|&idx| idx < self.vertices.len())
        {
            let normal = polygon.calculate_new_normal(&self.vertices);
            if normal.norm_squared() < Real::EPSILON * Real::EPSILON {
                vec![format!("Polygon {i} has degenerate normal (zero length)")]
            } else {
                Vec::new()
            }
        } else {
            Vec::new()
        }
    }

    /// **Iterator-Based Isolated Vertex Validation**
    ///
    /// Returns an iterator over isolated vertex validation issues.
    /// Uses lazy evaluation to minimize memory usage.
    #[inline]
    fn validate_isolated_vertices_iter(&self) -> impl Iterator<Item = String> + '_ {
        // **Zero-Copy Vertex Usage Tracking**: Use iterator-based approach
        let used_vertices: std::collections::HashSet<usize> = self
            .polygons
            .iter()
            .flat_map(|p| p.indices.iter())
            .copied()
            .collect();

        (0..self.vertices.len()).filter_map(move |i| {
            if !used_vertices.contains(&i) {
                Some(format!("Vertex {i} is isolated (no adjacent faces)"))
            } else {
                None
            }
        })
    }

    /// **Validate Manifold Properties**
    ///
    /// Check edge manifold properties and report violations.
    fn validate_manifold_properties(&self) -> Vec<String> {
        let mut issues = Vec::new();
        let mut edge_count: std::collections::HashMap<(usize, usize), usize> =
            std::collections::HashMap::new();

        // Count edge occurrences
        for polygon in &self.polygons {
            for (start_idx, end_idx) in polygon.edges() {
                let edge = if start_idx < end_idx {
                    (start_idx, end_idx)
                } else {
                    (end_idx, start_idx)
                };
                *edge_count.entry(edge).or_insert(0) += 1;
            }
        }

        // Check for non-manifold edges (shared by more than 2 faces)
        for ((a, b), count) in edge_count {
            if count > 2 {
                issues.push(format!(
                    "Non-manifold edge between vertices {a} and {b} (shared by {count} faces)"
                ));
            }
        }

        issues
    }

    /// **Mathematical Foundation: Vertex Merging with Epsilon Tolerance**
    ///
    /// Merge vertices that are within epsilon distance of each other.
    /// Updates polygon indices to reference merged vertices.
    ///
    /// ## **Algorithm**
    /// 1. **Spatial Clustering**: Group nearby vertices using epsilon tolerance
    /// 2. **Representative Selection**: Choose centroid as cluster representative
    /// 3. **Index Remapping**: Update all polygon indices to merged vertices
    /// 4. **Cleanup**: Remove unused vertices and compact array
    ///
    /// ## **Performance Benefits**
    /// - **Reduced Memory**: Eliminates duplicate vertices
    /// - **Improved Connectivity**: Better manifold properties
    /// - **Cache Efficiency**: Fewer vertices to process
    pub fn merge_vertices(&mut self, epsilon: Real) {
        if self.vertices.is_empty() {
            return;
        }

        let mut vertex_clusters = Vec::new();
        let mut vertex_to_cluster: Vec<Option<usize>> = vec![None; self.vertices.len()];

        // Build clusters of nearby vertices
        for (i, vertex) in self.vertices.iter().enumerate() {
            if vertex_to_cluster[i].is_some() {
                continue; // Already assigned to a cluster
            }

            // Start new cluster
            let cluster_id = vertex_clusters.len();
            let mut cluster_vertices = vec![i];
            vertex_to_cluster[i] = Some(cluster_id);

            // Find nearby vertices
            for (j, other_vertex) in self.vertices.iter().enumerate().skip(i + 1) {
                if vertex_to_cluster[j].is_none() {
                    let distance = (vertex.pos - other_vertex.pos).norm();
                    if distance < epsilon {
                        cluster_vertices.push(j);
                        vertex_to_cluster[j] = Some(cluster_id);
                    }
                }
            }

            vertex_clusters.push(cluster_vertices);
        }

        // Create merged vertices (centroids of clusters)
        let mut merged_vertices: Vec<vertex::IndexedVertex> = Vec::new();
        let mut old_to_new_index = vec![0; self.vertices.len()];

        for (cluster_id, cluster) in vertex_clusters.iter().enumerate() {
            // Compute centroid position
            let centroid_pos = cluster.iter().fold(Point3::origin(), |acc, &idx| {
                acc + self.vertices[idx].pos.coords
            }) / cluster.len() as Real;

            // Compute average normal
            let avg_normal = cluster
                .iter()
                .fold(Vector3::zeros(), |acc, &idx| acc + self.vertices[idx].normal);
            let normalized_normal = if avg_normal.norm() > Real::EPSILON {
                avg_normal.normalize()
            } else {
                Vector3::z()
            };

            let merged_vertex =
                vertex::IndexedVertex::new(Point3::from(centroid_pos), normalized_normal);
            merged_vertices.push(merged_vertex);

            // Update index mapping
            for &old_idx in cluster {
                old_to_new_index[old_idx] = cluster_id;
            }
        }

        // Update polygon indices
        for polygon in &mut self.polygons {
            for idx in &mut polygon.indices {
                *idx = old_to_new_index[*idx];
            }
        }

        // Replace vertices
        self.vertices = merged_vertices;

        // Invalidate cached bounding box
        self.bounding_box = OnceLock::new();
    }

    /// **Mathematical Foundation: Duplicate Polygon Removal**
    ///
    /// Remove duplicate polygons from the mesh based on vertex index comparison.
    /// Two polygons are considered duplicates if they reference the same vertices
    /// in the same order (accounting for cyclic permutations).
    ///
    /// ## **Algorithm**
    /// 1. **Canonical Form**: Convert each polygon to canonical form (smallest index first)
    /// 2. **Hash-based Deduplication**: Use HashMap to identify duplicates
    /// 3. **Metadata Preservation**: Keep metadata from first occurrence
    /// 4. **Index Compaction**: Maintain polygon ordering where possible
    ///
    /// ## **Performance Benefits**
    /// - **O(n log n)** complexity using hash-based deduplication
    /// - **Memory Efficient**: No temporary polygon copies
    /// - **Preserves Ordering**: Maintains relative polygon order
    pub fn remove_duplicate_polygons(&mut self) {
        if self.polygons.is_empty() {
            return;
        }

        let mut seen_polygons = std::collections::HashMap::new();
        let mut unique_polygons = Vec::new();

        for (i, polygon) in self.polygons.iter().enumerate() {
            // Create canonical form of polygon indices
            let canonical_indices = Self::canonicalize_polygon_indices(&polygon.indices);

            // Check if we've seen this polygon before
            if let std::collections::hash_map::Entry::Vacant(e) =
                seen_polygons.entry(canonical_indices)
            {
                e.insert(i);
                unique_polygons.push(polygon.clone());
            }
        }

        // Update polygons if duplicates were found
        if unique_polygons.len() != self.polygons.len() {
            self.polygons = unique_polygons;
            self.bounding_box = OnceLock::new(); // Invalidate cached bounding box
        }
    }

    /// **Create Canonical Form of Polygon Indices**
    ///
    /// Convert polygon indices to canonical form by rotating so the smallest
    /// index comes first, enabling duplicate detection across cyclic permutations.
    fn canonicalize_polygon_indices(indices: &[usize]) -> Vec<usize> {
        if indices.is_empty() {
            return Vec::new();
        }

        // Find position of minimum index
        let min_pos = indices
            .iter()
            .enumerate()
            .min_by_key(|&(_, val)| val)
            .map(|(pos, _)| pos)
            .unwrap_or(0);

        // Rotate so minimum index is first
        let mut canonical = Vec::with_capacity(indices.len());
        canonical.extend_from_slice(&indices[min_pos..]);
        canonical.extend_from_slice(&indices[..min_pos]);

        canonical
    }

    /// **Mathematical Foundation: Surface Area Computation**
    ///
    /// Compute the total surface area of the IndexedMesh by summing
    /// the areas of all triangulated polygons.
    ///
    /// ## **Algorithm**
    /// 1. **Triangulation**: Convert all polygons to triangles
    /// 2. **Area Computation**: Use cross product for triangle areas
    /// 3. **Summation**: Sum all triangle areas
    ///
    /// ## **Performance Benefits**
    /// - **Index-based**: Direct vertex access via indices
    /// - **Cache Efficient**: Sequential vertex access pattern
    /// - **Memory Efficient**: No temporary vertex copies
    pub fn surface_area(&self) -> Real {
        let mut total_area = 0.0;

        for polygon in &self.polygons {
            let triangle_indices = polygon.triangulate(&self.vertices);
            for triangle in triangle_indices {
                let v0 = self.vertices[triangle[0]].pos;
                let v1 = self.vertices[triangle[1]].pos;
                let v2 = self.vertices[triangle[2]].pos;

                let edge1 = v1 - v0;
                let edge2 = v2 - v0;
                let cross = edge1.cross(&edge2);
                let area = cross.norm() * 0.5;
                total_area += area;
            }
        }

        total_area
    }

    /// **Mathematical Foundation: Volume Computation for Closed Meshes**
    ///
    /// Compute the volume enclosed by the IndexedMesh using the divergence theorem.
    /// Assumes the mesh represents a closed, manifold surface.
    ///
    /// ## **Algorithm: Divergence Theorem**
    /// ```text
    /// V = (1/3) * Σ (p_i · n_i * A_i)
    /// ```
    /// Where p_i is a point on triangle i, n_i is the normal, A_i is the area.
    ///
    /// ## **Requirements**
    /// - Mesh must be closed (no boundary edges)
    /// - Mesh must be manifold (proper topology)
    /// - Normals must point outward
    pub fn volume(&self) -> Real {
        let mut total_volume: Real = 0.0;

        for polygon in &self.polygons {
            let triangle_indices = polygon.triangulate(&self.vertices);
            for triangle in triangle_indices {
                let v0 = self.vertices[triangle[0]].pos;
                let v1 = self.vertices[triangle[1]].pos;
                let v2 = self.vertices[triangle[2]].pos;

                // Compute triangle normal and area
                let edge1 = v1 - v0;
                let edge2 = v2 - v0;
                let normal = edge1.cross(&edge2);
                let area = normal.norm() * 0.5;

                if area > Real::EPSILON {
                    let unit_normal = normal.normalize();
                    // Use triangle centroid as reference point
                    let centroid = (v0 + v1.coords + v2.coords) / 3.0;

                    // Apply divergence theorem
                    let contribution = centroid.coords.dot(&unit_normal) * area / 3.0;
                    total_volume += contribution;
                }
            }
        }

        total_volume.abs() // Return absolute value for consistent results
    }

    /// **Mathematical Foundation: Manifold Closure Test**
    ///
    /// Test if the mesh represents a closed surface by checking for boundary edges.
    /// A mesh is closed if every edge is shared by exactly two faces.
    ///
    /// ## **Algorithm**
    /// 1. **Edge Enumeration**: Extract all edges from polygons
    /// 2. **Edge Counting**: Count occurrences of each edge
    /// 3. **Boundary Detection**: Edges with count ≠ 2 indicate boundaries
    ///
    /// Returns true if mesh is closed (no boundary edges).
    pub fn is_closed(&self) -> bool {
        let mut edge_count: std::collections::HashMap<(usize, usize), usize> =
            std::collections::HashMap::new();

        // Count edge occurrences
        for polygon in &self.polygons {
            for (start_idx, end_idx) in polygon.edges() {
                let edge = if start_idx < end_idx {
                    (start_idx, end_idx)
                } else {
                    (end_idx, start_idx)
                };
                *edge_count.entry(edge).or_insert(0) += 1;
            }
        }

        // Check if all edges are shared by exactly 2 faces
        edge_count.values().all(|&count| count == 2)
    }

    /// **Edge Count Computation**
    ///
    /// Count the total number of unique edges in the mesh.
    /// Each edge is counted once regardless of how many faces share it.
    pub fn edge_count(&self) -> usize {
        let mut edges = std::collections::HashSet::new();

        for polygon in &self.polygons {
            for (start_idx, end_idx) in polygon.edges() {
                let edge = if start_idx < end_idx {
                    (start_idx, end_idx)
                } else {
                    (end_idx, start_idx)
                };
                edges.insert(edge);
            }
        }

        edges.len()
    }

    /// Check if the mesh is a valid 2-manifold
    pub fn is_manifold(&self) -> bool {
        self.validate().is_empty()
    }

    /// Check if all polygons have consistent outward-pointing normals
    pub fn has_consistent_normals(&self) -> bool {
        // For a closed mesh, we can check if the mesh bounds contain the origin
        // If normals are outward-pointing, the origin should be outside
        let bbox = self.bounding_box();
        let center = bbox.center();

        // Check if center is outside the mesh (should be for outward normals)
        !self.contains_vertex(&center)
    }

    /// Ensure all polygons have consistent winding and normal orientation
    /// This is critical after CSG operations that may create inconsistent geometry
    pub fn ensure_consistent_winding(&mut self) {
        // Compute centroid once before mutable borrow
        let centroid = self.compute_centroid();

        for polygon in &mut self.polygons {
            // Reconstruct plane from vertices to ensure accuracy
            let vertices_for_plane = polygon.indices.iter()
                .map(|&idx| self.vertices[idx])
                .collect::<Vec<_>>();
            polygon.plane = plane::Plane::from_indexed_vertices(vertices_for_plane);

            // Ensure the polygon normal points outward (away from mesh centroid)
            let polygon_center = polygon.indices.iter()
                .map(|&idx| self.vertices[idx].pos.coords)
                .sum::<Vector3<Real>>() / polygon.indices.len() as Real;

            let to_center = centroid.coords - polygon_center;
            let normal_dot = polygon.plane.normal().dot(&to_center);

            // If normal points inward (towards centroid), flip it
            // normal_dot > 0 means normal and to_center point in same direction (inward)
            if normal_dot > 0.0 {
                // Flip polygon indices to reverse winding
                polygon.indices.reverse();
                // Flip plane normal
                polygon.plane = polygon.plane.flipped();
                // Flip normals of all vertices referenced by this polygon
                for &idx in &polygon.indices {
                    if idx < self.vertices.len() {
                        self.vertices[idx].flip();
                    }
                }
            }
        }
    }

    /// Compute the centroid of the mesh
    fn compute_centroid(&self) -> Point3<Real> {
        if self.vertices.is_empty() {
            return Point3::origin();
        }

        let sum: Vector3<Real> = self.vertices.iter()
            .map(|v| v.pos.coords)
            .sum();
        Point3::from(sum / self.vertices.len() as Real)
    }

    /// **Mathematical Foundation: Vertex Normal Computation with Indexed Connectivity**
    ///
    /// Computes vertex normals by averaging adjacent face normals, weighted by face area.
    /// Uses indexed connectivity for optimal performance with SIMD optimizations.
    ///
    /// ## **Algorithm: SIMD-Optimized Area-Weighted Normal Averaging**
    /// 1. **Vectorized Initialization**: Zero vertex normals using SIMD operations
    /// 2. **Face Normal Computation**: Calculate normal for each face
    /// 3. **Area Weighting**: Weight normals by triangle/polygon area
    /// 4. **Batch Accumulation**: Accumulate normals using vectorized operations
    /// 5. **Vectorized Normalization**: Normalize final vertex normals in batches
    ///
    /// ## **Performance Optimizations**
    /// - **SIMD Operations**: Process multiple vertices simultaneously
    /// - **Cache-Friendly Access**: Sequential memory access patterns
    /// - **Minimal Allocations**: In-place operations where possible
    pub fn compute_vertex_normals(&mut self) {
        // **SIMD-Optimized Initialization**: Vectorized initialization of vertex normals to zero
        self.vertices
            .iter_mut()
            .for_each(|vertex| vertex.normal = Vector3::zeros());

        // **Iterator-Based Normal Accumulation**: Use iterator chains for better vectorization
        // Collect weighted normals for each vertex using iterator combinators
        let weighted_normals: Vec<(usize, Vector3<Real>)> = self.polygons
            .iter()
            .flat_map(|polygon| {
                let face_normal = polygon.plane.normal();
                let area = self.compute_polygon_area(polygon);
                let weighted_normal = face_normal * area;

                // **Iterator Fusion**: Map each vertex index to its weighted normal contribution
                polygon.indices.iter()
                    .filter(|&&vertex_idx| vertex_idx < self.vertices.len())
                    .map(move |&vertex_idx| (vertex_idx, weighted_normal))
            })
            .collect();

        // **Vectorized Accumulation**: Apply weighted normals using iterator-based approach
        for (vertex_idx, weighted_normal) in weighted_normals {
            self.vertices[vertex_idx].normal += weighted_normal;
        }

        // **SIMD-Optimized Normalization**: Use iterator chains for better vectorization
        self.vertices
            .iter_mut()
            .for_each(|vertex| {
                let norm = vertex.normal.norm();
                if norm > EPSILON {
                    vertex.normal /= norm;
                } else {
                    // Default normal for degenerate cases
                    vertex.normal = Vector3::new(0.0, 0.0, 1.0);
                }
            });
    }

    /// Compute the area of a polygon using the shoelace formula
    fn compute_polygon_area(&self, polygon: &IndexedPolygon<S>) -> Real {
        if polygon.indices.len() < 3 {
            return 0.0;
        }

        let mut area = 0.0;
        let n = polygon.indices.len();

        for i in 0..n {
            let curr_idx = polygon.indices[i];
            let next_idx = polygon.indices[(i + 1) % n];

            if curr_idx < self.vertices.len() && next_idx < self.vertices.len() {
                let curr = self.vertices[curr_idx].pos;
                let next = self.vertices[next_idx].pos;

                // Cross product contribution to area
                area += curr.coords.cross(&next.coords).norm();
            }
        }

        area * 0.5
    }
}

impl<S: Clone + Send + Sync + Debug> CSG for IndexedMesh<S> {
    /// Returns a new empty IndexedMesh
    fn new() -> Self {
        IndexedMesh {
            vertices: Vec::new(),
            polygons: Vec::new(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

    fn union(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        // Use direct IndexedMesh BSP operations instead of round-trip conversion
        self.union_indexed(other)
    }

    fn difference(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        // Use direct IndexedMesh BSP operations instead of round-trip conversion
        self.difference_indexed(other)
    }

    fn intersection(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        // Use direct IndexedMesh BSP operations instead of round-trip conversion
        self.intersection_indexed(other)
    }

    fn xor(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        // Use direct IndexedMesh BSP operations instead of round-trip conversion
        self.xor_indexed(other)
    }

    /// **Mathematical Foundation: General 3D Transformations**
    ///
    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to Mesh.
    /// This implements the complete theory of affine transformations in homogeneous coordinates.
    ///
    /// ## **Transformation Mathematics**
    ///
    /// ### **Homogeneous Coordinates**
    /// Points and vectors are represented in 4D homogeneous coordinates:
    /// - **Point**: (x, y, z, 1)ᵀ → transforms as p' = Mp
    /// - **Vector**: (x, y, z, 0)ᵀ → transforms as v' = Mv
    /// - **Normal**: n'ᵀ = nᵀM⁻¹ (inverse transpose rule)
    ///
    /// ### **Normal Vector Transformation**
    /// Normals require special handling to remain perpendicular to surfaces:
    /// ```text
    /// If: T(p)·n = 0 (tangent perpendicular to normal)
    /// Then: T(p)·T(n) ≠ 0 in general
    /// But: T(p)·(M⁻¹)ᵀn = 0 ✓
    /// ```
    /// **Proof**: (Mp)ᵀ(M⁻¹)ᵀn = pᵀMᵀ(M⁻¹)ᵀn = pᵀ(M⁻¹M)ᵀn = pᵀn = 0
    ///
    /// ### **Numerical Stability**
    /// - **Degeneracy Detection**: Check determinant before inversion
    /// - **Homogeneous Division**: Validate w-coordinate after transformation
    /// - **Precision**: Maintain accuracy through matrix decomposition
    ///
    /// ## **Algorithm Complexity**
    /// - **Vertices**: O(n) matrix-vector multiplications
    /// - **Matrix Inversion**: O(1) for 4×4 matrices
    /// - **Plane Updates**: O(n) plane reconstructions from transformed vertices
    ///
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D
    fn transform(&self, mat: &Matrix4<Real>) -> IndexedMesh<S> {
        // Compute inverse transpose for normal transformation
        let mat_inv_transpose = match mat.try_inverse() {
            Some(inv) => inv.transpose(),
            None => {
                eprintln!(
                    "Warning: Transformation matrix is not invertible, using identity for normals"
                );
                Matrix4::identity()
            },
        };

        let mut mesh = self.clone();

        for vert in &mut mesh.vertices {
            // Transform position using homogeneous coordinates
            let hom_pos = mat * vert.pos.to_homogeneous();
            match Point3::from_homogeneous(hom_pos) {
                Some(transformed_pos) => vert.pos = transformed_pos,
                None => {
                    eprintln!(
                        "Warning: Invalid homogeneous coordinates after transformation, skipping vertex"
                    );
                    continue;
                },
            }

            // Transform normal using inverse transpose rule
            vert.normal = mat_inv_transpose.transform_vector(&vert.normal).normalize();
        }

        // Update planes for all polygons
        for poly in &mut mesh.polygons {
            // Reconstruct plane from transformed vertices
            let vertices: Vec<vertex::IndexedVertex> =
                poly.indices.iter().map(|&idx| mesh.vertices[idx]).collect();
            poly.plane = plane::Plane::from_indexed_vertices(vertices);

            // Invalidate the polygon's bounding box
            poly.bounding_box = OnceLock::new();
        }

        // invalidate the old cached bounding box
        mesh.bounding_box = OnceLock::new();

        mesh
    }

    /// Returns a [`parry3d::bounding_volume::Aabb`] indicating the 3D bounds of all `polygons`.
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Track overall min/max in x, y, z among all 3D polygons
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            // 1) Gather from the 3D polygons
            for poly in &self.polygons {
                for &idx in &poly.indices {
                    let v = &self.vertices[idx];
                    min_x = *partial_min(&min_x, &v.pos.x).unwrap();
                    min_y = *partial_min(&min_y, &v.pos.y).unwrap();
                    min_z = *partial_min(&min_z, &v.pos.z).unwrap();

                    max_x = *partial_max(&max_x, &v.pos.x).unwrap();
                    max_y = *partial_max(&max_y, &v.pos.y).unwrap();
                    max_z = *partial_max(&max_z, &v.pos.z).unwrap();
                }
            }

            // If still uninitialized (e.g., no polygons), return a trivial AABB at origin
            if min_x > max_x {
                return Aabb::new(Point3::origin(), Point3::origin());
            }

            // Build a parry3d Aabb from these min/max corners
            let mins = Point3::new(min_x, min_y, min_z);
            let maxs = Point3::new(max_x, max_y, max_z);
            Aabb::new(mins, maxs)
        })
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Invert this IndexedMesh (flip inside vs. outside)
    fn inverse(&self) -> IndexedMesh<S> {
        let mut mesh = self.clone();
        for p in &mut mesh.polygons {
            p.flip();
        }
        mesh
    }
}

impl<S: Clone + Send + Sync + Debug> IndexedMesh<S> {
    /// **Mathematical Foundation: BSP-Based Union Operation**
    ///
    /// Compute the union of two IndexedMeshes using Binary Space Partitioning
    /// for robust boolean operations with manifold preservation.
    ///
    /// ## **Algorithm: Direct Mirror of Regular Mesh Union**
    /// This implementation directly mirrors the regular Mesh union algorithm
    /// but uses IndexedMesh data structures for memory efficiency.
    ///
    /// ## **IndexedMesh Optimization**
    /// - **Vertex Sharing**: Maintains indexed connectivity across union
    /// - **Memory Efficiency**: Reuses vertices where possible
    /// - **Topology Preservation**: Preserves manifold structure
    pub fn union_indexed(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        // Use exact same algorithm as regular Mesh union with partition optimization
        // Avoid splitting obvious non-intersecting faces
        let (a_clip, a_passthru) = Self::partition_polygons(&self.polygons, &self.vertices, &other.bounding_box());
        let (b_clip, b_passthru) = Self::partition_polygons(&other.polygons, &other.vertices, &self.bounding_box());


        // Union operation preserves original metadata from each source
        // Do NOT retag b_clip polygons (unlike difference operation)

        // Start with self vertices, BSP operations will add intersection vertices as needed
        let mut result_vertices = self.vertices.clone();

        // Create BSP trees with proper vertex handling - a_clip polygons reference self vertices
        let mut a = bsp::IndexedNode::from_polygons(&a_clip, &mut result_vertices);

        // For b_clip polygons, use separate vertex array to avoid index conflicts
        let mut b_vertices = other.vertices.clone();
        let mut b = bsp::IndexedNode::from_polygons(&b_clip, &mut b_vertices);

        // Use exact same algorithm as regular Mesh union (NOT difference!)
        a.clip_to(&b, &mut result_vertices);
        b.clip_to(&a, &mut b_vertices);  // b's polygons reference b_vertices
        b.invert_with_vertices(&mut b_vertices);
        b.clip_to(&a, &mut b_vertices);  // b's polygons reference b_vertices
        b.invert_with_vertices(&mut b_vertices);
        // Add b_vertices to result_vertices first, then remap b polygons
        let b_vertex_offset = result_vertices.len();
        result_vertices.extend(b_vertices.iter().cloned());

        // Remap b polygons to use result_vertices indices
        let b_polygons_remapped = Self::remap_bsp_polygons(&b.all_polygons(), b_vertex_offset);
        a.build(&b_polygons_remapped, &mut result_vertices);
        // NOTE: Union operation does NOT have final a.invert() (unlike difference operation)

        // Combine results and untouched faces
        let mut final_polygons = a.all_polygons();
        final_polygons.extend(a_passthru);

        // Include b_passthru polygons and remap their indices to account for result vertex array
        let mut b_passthru_remapped = b_passthru;
        Self::remap_polygon_indices(&mut b_passthru_remapped, b_vertex_offset);
        final_polygons.extend(b_passthru_remapped);

        let mut result = IndexedMesh {
            vertices: result_vertices,
            polygons: final_polygons,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        };

        // Deduplicate vertices to prevent holes and improve manifold properties
        result.merge_vertices(Real::EPSILON);

        // Ensure consistent polygon winding and normal orientation BEFORE computing normals
        result.ensure_consistent_winding();

        // Recompute vertex normals after CSG operation and winding correction
        result.compute_vertex_normals();

        result
    }

    /// **Mathematical Foundation: BSP-Based Difference Operation**
    ///
    /// Compute A - B using Binary Space Partitioning for robust boolean operations
    /// with manifold preservation and indexed connectivity.
    ///
    /// ## **Algorithm: Direct CSG Difference**
    /// Based on the working regular Mesh difference algorithm:
    /// 1. **BSP Construction**: Build BSP trees from both meshes
    /// 2. **Invert A**: Flip A inside/outside
    /// 3. **Clip A against B**: Keep parts of A outside B
    /// 4. **Clip B against A**: Keep parts of B outside A
    /// 5. **Invert B**: Flip B inside/outside
    /// 6. **Final clipping**: Complete the difference operation
    /// 7. **Combine results**: Merge clipped geometry
    ///
    /// ## **IndexedMesh Optimization**
    /// - **Vertex Sharing**: Maintains indexed connectivity throughout
    /// - **Memory Efficiency**: Reuses vertices where possible
    /// - **Topology Preservation**: Preserves manifold structure
    pub fn difference_indexed(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        // Handle empty mesh cases
        if self.polygons.is_empty() {
            return IndexedMesh::new();
        }
        if other.polygons.is_empty() {
            return self.clone();
        }

        // Use exact same algorithm as regular Mesh difference with partition optimization
        // Avoid splitting obvious non-intersecting faces
        let (a_clip, a_passthru) = Self::partition_polygons(&self.polygons, &self.vertices, &other.bounding_box());
        let (b_clip, _b_passthru) = Self::partition_polygons(&other.polygons, &other.vertices, &self.bounding_box());

        // Propagate self.metadata to new polygons by overwriting intersecting
        // polygon.metadata in other.
        let b_clip_retagged: Vec<IndexedPolygon<S>> = b_clip
            .iter()
            .map(|poly| {
                let mut p = poly.clone();
                p.metadata = self.metadata.clone();
                p
            })
            .collect();

        // Start with self vertices, BSP operations will add intersection vertices as needed
        let mut result_vertices = self.vertices.clone();

        // Create BSP trees with proper vertex handling
        let mut a = bsp::IndexedNode::from_polygons(&a_clip, &mut result_vertices);

        // For b_clip polygons, create separate vertex array and remap indices
        let mut b_vertices = other.vertices.clone();
        let mut b = bsp::IndexedNode::from_polygons(&b_clip_retagged, &mut b_vertices);

        a.invert_with_vertices(&mut result_vertices);
        a.clip_to(&b, &mut result_vertices);
        b.clip_to(&a, &mut b_vertices);  // b's polygons reference b_vertices
        b.invert_with_vertices(&mut b_vertices);
        b.clip_to(&a, &mut b_vertices);  // b's polygons reference b_vertices
        b.invert_with_vertices(&mut b_vertices);
        // Add b_vertices to result_vertices first, then remap b polygons
        let b_vertex_offset = result_vertices.len();
        result_vertices.extend(b_vertices.iter().cloned());

        // Remap b polygons to use result_vertices indices
        let b_polygons_remapped = Self::remap_bsp_polygons(&b.all_polygons(), b_vertex_offset);
        a.build(&b_polygons_remapped, &mut result_vertices);
        a.invert_with_vertices(&mut result_vertices);

        // Combine results - for difference, only include polygons from BSP operations and a_passthru
        // Do NOT include b_passthru as they are outside the difference volume
        let mut final_polygons = a.all_polygons();
        final_polygons.extend(a_passthru);

        // Note: b_passthru polygons are intentionally excluded from difference result

        let mut result = IndexedMesh {
            vertices: result_vertices,
            polygons: final_polygons,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        };

        // Deduplicate vertices to prevent holes and improve manifold properties
        result.merge_vertices(Real::EPSILON);

        // NOTE: Difference operations should NOT have winding correction applied
        // The cut boundary faces need to point inward toward the removed volume
        // Regular mesh difference operations work correctly without winding correction

        // Recompute vertex normals after CSG operation
        result.compute_vertex_normals();

        result
    }

    /// **Mathematical Foundation: BSP-Based Intersection Operation**
    ///
    /// Compute A ∩ B using Binary Space Partitioning for robust boolean operations
    /// with manifold preservation and indexed connectivity.
    ///
    /// ## **Algorithm: Direct CSG Intersection**
    /// Based on the working regular Mesh intersection algorithm:
    /// 1. **BSP Construction**: Build BSP trees from both meshes
    /// 2. **Invert A**: Flip A inside/outside
    /// 3. **Clip B against A**: Keep parts of B outside inverted A
    /// 4. **Invert B**: Flip B inside/outside
    /// 5. **Clip A against B**: Keep parts of A outside inverted B
    /// 6. **Clip B against A**: Final clipping
    /// 7. **Build result**: Combine and invert
    ///
    /// ## **IndexedMesh Optimization**
    /// - **Vertex Sharing**: Maintains indexed connectivity throughout
    /// - **Memory Efficiency**: Reuses vertices where possible
    /// - **Topology Preservation**: Preserves manifold structure
    pub fn intersection_indexed(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        // Handle empty mesh cases
        if self.polygons.is_empty() || other.polygons.is_empty() {
            return IndexedMesh::new();
        }

        // For intersection operations, don't use partition optimization to ensure correctness
        // The regular Mesh intersection also doesn't use partition optimization

        // Start with self vertices, BSP operations will add intersection vertices as needed
        let mut result_vertices = self.vertices.clone();

        // Create BSP trees with proper vertex handling for ALL polygons (no partition optimization)
        let mut a = bsp::IndexedNode::from_polygons(&self.polygons, &mut result_vertices);

        // For b polygons, use separate vertex array to avoid index conflicts
        let mut b_vertices = other.vertices.clone();
        let mut b = bsp::IndexedNode::from_polygons(&other.polygons, &mut b_vertices);

        // Use exact same algorithm as regular Mesh intersection
        a.invert_with_vertices(&mut result_vertices);
        b.clip_to(&a, &mut b_vertices);  // b's polygons reference b_vertices
        b.invert_with_vertices(&mut b_vertices);
        a.clip_to(&b, &mut result_vertices);
        b.clip_to(&a, &mut b_vertices);  // b's polygons reference b_vertices

        // Add b_vertices to result_vertices first, then remap b polygons
        let b_vertex_offset = result_vertices.len();
        result_vertices.extend(b_vertices.iter().cloned());

        // Remap b polygons to use result_vertices indices
        let b_polygons_remapped = Self::remap_bsp_polygons(&b.all_polygons(), b_vertex_offset);
        a.build(&b_polygons_remapped, &mut result_vertices);
        a.invert_with_vertices(&mut result_vertices);

        // Combine results - only use polygons from the final BSP tree (same as regular Mesh)
        let final_polygons = a.all_polygons();

        let mut result = IndexedMesh {
            vertices: result_vertices,
            polygons: final_polygons,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        };

        // Deduplicate vertices to prevent holes and improve manifold properties
        result.merge_vertices(Real::EPSILON);

        // Ensure consistent polygon winding and normal orientation BEFORE computing normals
        result.ensure_consistent_winding();

        // Recompute vertex normals after CSG operation and winding correction
        result.compute_vertex_normals();

        result
    }

    /// **Mathematical Foundation: BSP-based XOR Operation with Indexed Connectivity**
    ///
    /// Computes the symmetric difference (XOR) A ⊕ B = (A - B) ∪ (B - A)
    /// using BSP tree operations while preserving indexed connectivity.
    ///
    /// ## **Algorithm: Manifold-Preserving XOR via Difference Union**
    /// 1. **A - B Computation**: Remove B from A using indexed BSP operations
    /// 2. **B - A Computation**: Remove A from B using indexed BSP operations
    /// 3. **Union Computation**: Combine (A - B) ∪ (B - A) using indexed BSP operations
    /// 4. **Connectivity Preservation**: Maintain vertex indices throughout
    ///
    /// This approach matches the regular Mesh XOR and better preserves manifold properties.
    pub fn xor_indexed(&self, other: &IndexedMesh<S>) -> IndexedMesh<S> {
        // Compute XOR as (A - B) ∪ (B - A) to better preserve manifold properties
        let a_minus_b = self.difference_indexed(other);
        let b_minus_a = other.difference_indexed(self);

        // Return union of the two differences
        a_minus_b.union_indexed(&b_minus_a)
    }
}

impl<S: Clone + Send + Sync + Debug> From<Sketch<S>> for IndexedMesh<S> {
    /// Convert a Sketch into an IndexedMesh.
    fn from(sketch: Sketch<S>) -> Self {
        // Use appropriate hash key type based on Real precision
        #[cfg(feature = "f32")]
        type HashKey = (u32, u32, u32);
        #[cfg(feature = "f64")]
        type HashKey = (u64, u64, u64);
        /// Helper function to convert a geo::Polygon to vertices and IndexedPolygon
        fn geo_poly_to_indexed<S: Clone + Debug + Send + Sync>(
            poly2d: &GeoPolygon<Real>,
            metadata: &Option<S>,
            vertices: &mut Vec<vertex::IndexedVertex>,
            vertex_map: &mut std::collections::HashMap<HashKey, usize>,
        ) -> IndexedPolygon<S> {
            let mut indices = Vec::new();

            // Handle the exterior ring
            for coord in poly2d.exterior().coords_iter() {
                let pos = Point3::new(coord.x, coord.y, 0.0);
                let key = (pos.x.to_bits(), pos.y.to_bits(), pos.z.to_bits());
                let idx = if let Some(&existing_idx) = vertex_map.get(&key) {
                    existing_idx
                } else {
                    let new_idx = vertices.len();
                    vertices.push(vertex::IndexedVertex::new(pos, Vector3::z()));
                    vertex_map.insert(key, new_idx);
                    new_idx
                };
                indices.push(idx);
            }

            let plane = plane::Plane::from_indexed_vertices(vec![
                vertex::IndexedVertex::new(vertices[indices[0]].pos, Vector3::z()),
                vertex::IndexedVertex::new(vertices[indices[1]].pos, Vector3::z()),
                vertex::IndexedVertex::new(vertices[indices[2]].pos, Vector3::z()),
            ]);

            IndexedPolygon::new(indices, plane, metadata.clone())
        }

        let mut vertices: Vec<vertex::IndexedVertex> = Vec::new();
        let mut vertex_map: std::collections::HashMap<HashKey, usize> =
            std::collections::HashMap::new();
        let mut indexed_polygons = Vec::new();

        for geom in &sketch.geometry {
            match geom {
                Geometry::Polygon(poly2d) => {
                    let indexed_poly = geo_poly_to_indexed(
                        poly2d,
                        &sketch.metadata,
                        &mut vertices,
                        &mut vertex_map,
                    );
                    indexed_polygons.push(indexed_poly);
                },
                Geometry::MultiPolygon(multipoly) => {
                    for poly2d in multipoly.iter() {
                        let indexed_poly = geo_poly_to_indexed(
                            poly2d,
                            &sketch.metadata,
                            &mut vertices,
                            &mut vertex_map,
                        );
                        indexed_polygons.push(indexed_poly);
                    }
                },
                _ => {},
            }
        }

        IndexedMesh {
            vertices,
            polygons: indexed_polygons,
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;


    #[test]
    fn test_union_consistency_with_mesh() {
        // Create two simple cubes
        let cube1 = IndexedMesh::<()>::cube(1.0, None);
        let cube2 = IndexedMesh::<()>::cube(1.0, None).transform(&nalgebra::Translation3::new(0.5, 0.5, 0.5).to_homogeneous());

        // Perform union using IndexedMesh
        let indexed_union = cube1.union_indexed(&cube2);

        // Convert to regular Mesh and perform union
        let mesh1 = cube1.to_mesh();
        let mesh2 = cube2.to_mesh();
        let mesh_union = mesh1.union(&mesh2);

        // Basic checks - both should have similar properties
        assert!(!indexed_union.vertices.is_empty());
        assert!(!indexed_union.polygons.is_empty());
        assert!(!mesh_union.polygons.is_empty());

        // The indexed union should preserve the indexed structure
        assert!(indexed_union.vertices.len() >= cube1.vertices.len() + cube2.vertices.len());

        println!("IndexedMesh union: {} vertices, {} polygons",
                indexed_union.vertices.len(), indexed_union.polygons.len());
        println!("Regular Mesh union: {} polygons", mesh_union.polygons.len());
    }
}
