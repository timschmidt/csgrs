//! Mesh smoothing algorithms optimized for IndexedMesh with indexed connectivity

use crate::IndexedMesh::IndexedMesh;
use crate::float_types::Real;
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};
use std::collections::HashMap;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> IndexedMesh<S> {
    /// **Mathematical Foundation: Optimized Laplacian Mesh Smoothing with Indexed Connectivity**
    ///
    /// Implements discrete Laplacian smoothing leveraging indexed mesh representation
    /// for superior performance and memory efficiency compared to coordinate-based approaches.
    ///
    /// ## **Indexed Connectivity Advantages**
    /// - **Direct Vertex Access**: O(1) vertex lookup using indices
    /// - **Efficient Adjacency**: Pre-computed connectivity graph from build_connectivity_indexed
    /// - **Memory Locality**: Better cache performance through structured vertex access
    /// - **Precision Preservation**: No coordinate quantization or floating-point drift
    ///
    /// ## **Discrete Laplacian Operator**
    /// For each vertex v with neighbors N(v):
    /// ```text
    /// L(v) = (1/|N(v)|) · Σ(n∈N(v)) (n - v)
    /// v_new = v + λ · L(v)
    /// ```
    ///
    /// ## **Algorithm Optimization**
    /// 1. **Connectivity Reuse**: Single connectivity computation for all iterations
    /// 2. **Index-based Updates**: Direct vertex array modification
    /// 3. **Boundary Preservation**: Automatic boundary detection via valence analysis
    /// 4. **Vectorized Operations**: SIMD-friendly position updates
    ///
    /// # Parameters
    /// - `lambda`: Smoothing factor (0.0 = no smoothing, 1.0 = full neighbor averaging)
    /// - `iterations`: Number of smoothing iterations
    /// - `preserve_boundaries`: Whether to keep boundary vertices fixed
    pub fn laplacian_smooth(
        &self,
        lambda: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> IndexedMesh<S> {
        // Build connectivity once for all iterations - major performance optimization
        let (_vertex_map, adjacency) = self.build_connectivity_indexed();
        let mut smoothed_mesh = self.clone();

        for _iteration in 0..iterations {
            // Compute Laplacian updates for all vertices
            let mut position_updates: HashMap<usize, Point3<Real>> = HashMap::new();

            for (&vertex_idx, neighbors) in &adjacency {
                if vertex_idx >= smoothed_mesh.vertices.len() {
                    continue;
                }

                let current_pos = smoothed_mesh.vertices[vertex_idx].pos;

                // Boundary detection: vertices with low valence are likely on boundaries
                if preserve_boundaries && neighbors.len() < 4 {
                    position_updates.insert(vertex_idx, current_pos);
                    continue;
                }

                // Compute neighbor centroid using indexed access
                let mut neighbor_sum = Point3::origin();
                let mut valid_neighbors = 0;

                for &neighbor_idx in neighbors {
                    if neighbor_idx < smoothed_mesh.vertices.len() {
                        neighbor_sum += smoothed_mesh.vertices[neighbor_idx].pos.coords;
                        valid_neighbors += 1;
                    }
                }

                if valid_neighbors > 0 {
                    let neighbor_centroid = neighbor_sum / valid_neighbors as Real;
                    let laplacian = neighbor_centroid - current_pos;
                    let new_pos = current_pos + laplacian * lambda;
                    position_updates.insert(vertex_idx, new_pos);
                } else {
                    position_updates.insert(vertex_idx, current_pos);
                }
            }

            // Apply position updates using direct indexed access
            for (vertex_idx, new_pos) in position_updates {
                if vertex_idx < smoothed_mesh.vertices.len() {
                    smoothed_mesh.vertices[vertex_idx].pos = new_pos;
                }
            }

            // Update polygon planes and vertex normals after position changes
            smoothed_mesh.update_geometry_after_smoothing();
        }

        smoothed_mesh
    }

    /// **Mathematical Foundation: Taubin Smoothing with Indexed Connectivity**
    ///
    /// Implements Taubin's λ/μ smoothing algorithm optimized for indexed meshes.
    /// This method provides better volume preservation than pure Laplacian smoothing.
    ///
    /// ## **Taubin Algorithm**
    /// Two-step process per iteration:
    /// 1. **Smoothing step**: Apply positive λ (expansion)
    /// 2. **Shrinkage correction**: Apply negative μ (contraction)
    ///
    /// ## **Mathematical Properties**
    /// - **Volume Preservation**: Better than pure Laplacian
    /// - **Feature Preservation**: Maintains sharp edges better
    /// - **Stability**: Reduced mesh shrinkage artifacts
    ///
    /// # Parameters
    /// - `lambda`: Positive smoothing factor (typically 0.5-0.7)
    /// - `mu`: Negative shrinkage correction factor (typically -0.5 to -0.7)
    /// - `iterations`: Number of λ/μ iteration pairs
    /// - `preserve_boundaries`: Whether to keep boundary vertices fixed
    pub fn taubin_smooth(
        &self,
        lambda: Real,
        mu: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> IndexedMesh<S> {
        let (_vertex_map, adjacency) = self.build_connectivity_indexed();
        let mut smoothed_mesh = self.clone();

        for _iteration in 0..iterations {
            // Step 1: Apply λ smoothing (expansion)
            smoothed_mesh =
                smoothed_mesh.apply_laplacian_step(&adjacency, lambda, preserve_boundaries);

            // Step 2: Apply μ smoothing (contraction/correction)
            smoothed_mesh =
                smoothed_mesh.apply_laplacian_step(&adjacency, mu, preserve_boundaries);
        }

        smoothed_mesh
    }

    /// Apply a single Laplacian smoothing step with given factor
    fn apply_laplacian_step(
        &self,
        adjacency: &HashMap<usize, Vec<usize>>,
        factor: Real,
        preserve_boundaries: bool,
    ) -> IndexedMesh<S> {
        let mut result = self.clone();
        let mut position_updates: HashMap<usize, Point3<Real>> = HashMap::new();

        for (&vertex_idx, neighbors) in adjacency {
            if vertex_idx >= result.vertices.len() {
                continue;
            }

            let current_pos = result.vertices[vertex_idx].pos;

            // Boundary preservation
            if preserve_boundaries && neighbors.len() < 4 {
                position_updates.insert(vertex_idx, current_pos);
                continue;
            }

            // Compute Laplacian using indexed connectivity
            let mut neighbor_sum = Point3::origin();
            let mut valid_neighbors = 0;

            for &neighbor_idx in neighbors {
                if neighbor_idx < result.vertices.len() {
                    neighbor_sum += result.vertices[neighbor_idx].pos.coords;
                    valid_neighbors += 1;
                }
            }

            if valid_neighbors > 0 {
                let neighbor_centroid = neighbor_sum / valid_neighbors as Real;
                let laplacian = neighbor_centroid - current_pos;
                let new_pos = current_pos + laplacian * factor;
                position_updates.insert(vertex_idx, new_pos);
            } else {
                position_updates.insert(vertex_idx, current_pos);
            }
        }

        // Apply updates
        for (vertex_idx, new_pos) in position_updates {
            if vertex_idx < result.vertices.len() {
                result.vertices[vertex_idx].pos = new_pos;
            }
        }

        result.update_geometry_after_smoothing();
        result
    }

    /// **Mathematical Foundation: Bilateral Mesh Smoothing with Indexed Connectivity**
    ///
    /// Implements edge-preserving bilateral smoothing optimized for indexed meshes.
    /// This method smooths while preserving sharp features and edges.
    ///
    /// ## **Bilateral Filtering**
    /// Combines spatial and range kernels:
    /// ```text
    /// w(i,j) = exp(-||p_i - p_j||²/σ_s²) · exp(-||n_i - n_j||²/σ_r²)
    /// ```
    /// Where σ_s controls spatial smoothing and σ_r controls feature preservation.
    ///
    /// ## **Feature Preservation**
    /// - **Sharp Edges**: Preserved through normal-based weighting
    /// - **Corners**: Maintained via spatial distance weighting
    /// - **Surface Details**: Controlled by range parameter
    ///
    /// # Parameters
    /// - `sigma_spatial`: Spatial smoothing strength
    /// - `sigma_range`: Feature preservation strength (normal similarity)
    /// - `iterations`: Number of smoothing iterations
    /// - `preserve_boundaries`: Whether to keep boundary vertices fixed
    pub fn bilateral_smooth(
        &self,
        sigma_spatial: Real,
        sigma_range: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> IndexedMesh<S> {
        let (_vertex_map, adjacency) = self.build_connectivity_indexed();
        let mut smoothed_mesh = self.clone();

        // Precompute spatial and range factors for efficiency
        let spatial_factor = -1.0 / (2.0 * sigma_spatial * sigma_spatial);
        let range_factor = -1.0 / (2.0 * sigma_range * sigma_range);

        for _iteration in 0..iterations {
            let mut position_updates: HashMap<usize, Point3<Real>> = HashMap::new();

            for (&vertex_idx, neighbors) in &adjacency {
                if vertex_idx >= smoothed_mesh.vertices.len() {
                    continue;
                }

                let current_vertex = &smoothed_mesh.vertices[vertex_idx];
                let current_pos = current_vertex.pos;
                let current_normal = current_vertex.normal;

                // Boundary preservation
                if preserve_boundaries && neighbors.len() < 4 {
                    position_updates.insert(vertex_idx, current_pos);
                    continue;
                }

                // Bilateral weighted averaging
                let mut weighted_sum = Point3::origin();
                let mut weight_sum = 0.0;

                for &neighbor_idx in neighbors {
                    if neighbor_idx >= smoothed_mesh.vertices.len() {
                        continue;
                    }

                    let neighbor_vertex = &smoothed_mesh.vertices[neighbor_idx];
                    let neighbor_pos = neighbor_vertex.pos;
                    let neighbor_normal = neighbor_vertex.normal;

                    // Spatial weight based on distance
                    let spatial_dist_sq = (neighbor_pos - current_pos).norm_squared();
                    let spatial_weight = (spatial_dist_sq * spatial_factor).exp();

                    // Range weight based on normal similarity
                    let normal_diff_sq = (neighbor_normal - current_normal).norm_squared();
                    let range_weight = (normal_diff_sq * range_factor).exp();

                    let combined_weight = spatial_weight * range_weight;
                    weighted_sum += neighbor_pos.coords * combined_weight;
                    weight_sum += combined_weight;
                }

                if weight_sum > Real::EPSILON {
                    let new_pos = weighted_sum / weight_sum;
                    position_updates.insert(vertex_idx, Point3::from(new_pos));
                } else {
                    position_updates.insert(vertex_idx, current_pos);
                }
            }

            // Apply updates
            for (vertex_idx, new_pos) in position_updates {
                if vertex_idx < smoothed_mesh.vertices.len() {
                    smoothed_mesh.vertices[vertex_idx].pos = new_pos;
                }
            }

            smoothed_mesh.update_geometry_after_smoothing();
        }

        smoothed_mesh
    }

    /// Update polygon planes and vertex normals after vertex position changes
    /// This is essential for maintaining geometric consistency after smoothing
    fn update_geometry_after_smoothing(&mut self) {
        // Recompute polygon planes from updated vertex positions
        for polygon in &mut self.polygons {
            if polygon.indices.len() >= 3 {
                // Get vertices for plane computation
                let vertices: Vec<Vertex> = polygon
                    .indices
                    .iter()
                    .take(3)
                    .map(|&idx| self.vertices[idx])
                    .collect();

                if vertices.len() == 3 {
                    polygon.plane = crate::IndexedMesh::plane::Plane::from_vertices(vertices);
                }
            }

            // Invalidate cached bounding box
            polygon.bounding_box = std::sync::OnceLock::new();
        }

        // Recompute vertex normals based on adjacent faces
        self.compute_vertex_normals_from_faces();

        // Invalidate mesh bounding box
        self.bounding_box = std::sync::OnceLock::new();
    }

    /// Compute vertex normals from adjacent face normals using indexed connectivity
    pub fn compute_vertex_normals_from_faces(&mut self) {
        // Reset all vertex normals
        for vertex in &mut self.vertices {
            vertex.normal = Vector3::zeros();
        }

        // Accumulate face normals at each vertex
        for polygon in &self.polygons {
            let face_normal = polygon.plane.normal();
            for &vertex_idx in &polygon.indices {
                if vertex_idx < self.vertices.len() {
                    self.vertices[vertex_idx].normal += face_normal;
                }
            }
        }

        // Normalize accumulated normals
        for vertex in &mut self.vertices {
            if vertex.normal.norm_squared() > Real::EPSILON * Real::EPSILON {
                vertex.normal = vertex.normal.normalize();
            }
        }
    }
}
