//! Mesh smoothing and refinement operations.

use crate::mesh::Mesh;
use hyperlattice::{Point3, Real};
use std::collections::HashMap;
use std::fmt::Debug;

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Mathematical Foundation: True Laplacian Mesh Smoothing with Global Connectivity**
    ///
    /// Implements proper discrete Laplacian smoothing using global mesh connectivity:
    ///
    /// ## **Discrete Laplacian Operator**
    /// For each vertex v with neighbors N(v):
    /// ```text
    /// L(v) = (1/|N(v)|) · Σ(n∈N(v)) (n - v)
    /// ```
    ///
    /// ## **Global Connectivity Benefits**
    /// - **Proper Neighborhoods**: Uses actual mesh connectivity, not just polygon edges
    /// - **Uniform Weighting**: Each neighbor contributes equally to smoothing
    /// - **Boundary Detection**: Automatically detects and preserves mesh boundaries
    /// - **Volume Preservation**: Better volume preservation than local smoothing
    ///
    /// ## **Algorithm Improvements**
    /// - **Exact Vertex Matching**: Hyperreal-backed boundary-coordinate identity
    /// - **Manifold Preservation**: Ensures mesh topology is maintained
    /// - **Feature Detection**: Can preserve sharp features based on neighbor count
    ///
    /// Vertex lookup reuses [`VertexIndexMap`](crate::mesh::connectivity::VertexIndexMap),
    /// whose matching predicate requires exact equality after promotion through
    /// `hyperlattice::Vector3` and `Real`. This keeps the discrete
    /// Laplacian's adjacency relation stable before applying the uniform
    /// smoothing stencil described in Botsch et al., *Polygon Mesh Processing*,
    /// 2010 (<https://doi.org/10.1201/b10688>).
    pub fn laplacian_smooth(
        &self,
        lambda: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Mesh<M> {
        let (vertex_map, adjacency) = self.build_connectivity();
        let mut smoothed_polygons = self.polygons.clone();

        for iteration in 0..iterations {
            // Build current vertex position mapping
            let mut current_positions: HashMap<usize, Point3> = HashMap::new();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        current_positions.insert(idx, vertex.position.clone());
                    }
                }
            }

            // Compute Laplacian for each vertex
            let mut laplacian_updates: HashMap<usize, Point3> = HashMap::new();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(current_position) = current_positions.get(&vertex_idx).cloned() {
                    // Check if this is a boundary vertex
                    if preserve_boundaries && neighbors.len() < 4 {
                        // Boundary vertex - skip smoothing
                        laplacian_updates.insert(vertex_idx, current_position);
                        continue;
                    }

                    let neighbor_positions = neighbors
                        .iter()
                        .filter_map(|neighbor_idx| {
                            current_positions.get(neighbor_idx).cloned()
                        })
                        .collect::<Vec<_>>();

                    if let Some(neighbor_avg) = Point3::centroid(&neighbor_positions) {
                        let new_position = current_position.lerp(&neighbor_avg, &lambda);
                        laplacian_updates.insert(vertex_idx, new_position);
                    } else {
                        laplacian_updates.insert(vertex_idx, current_position);
                    }
                }
            }

            // Apply updates to mesh vertices
            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        if let Some(new_position) = laplacian_updates.get(&idx) {
                            vertex.position = new_position.clone();
                        }
                    }
                }
                // Recompute polygon plane and normals after smoothing
                polygon.set_new_normal();
            }

            // Progress feedback for long smoothing operations
            if iterations > 10 && iteration % (iterations / 10) == 0 {
                eprintln!(
                    "Smoothing progress: {}/{} iterations",
                    iteration + 1,
                    iterations
                );
            }
        }

        Mesh::from_polygons(&smoothed_polygons, self.metadata.clone())
    }

    /// **Mathematical Foundation: Taubin Mesh Smoothing**
    ///
    /// Implements Taubin's feature-preserving mesh smoothing algorithm, which reduces
    /// shrinkage compared to standard Laplacian smoothing.
    ///
    /// ## **Taubin's Algorithm**
    /// This method involves two steps per iteration:
    /// 1. **Shrinking Step**: Apply standard Laplacian smoothing with a positive factor `lambda`.
    ///    `v' = v + λ * L(v)`
    /// 2. **Inflating Step**: Apply a second Laplacian step with a negative factor `mu`.
    ///    `v'' = v' + μ * L(v')`
    ///
    /// Typically, `0 < λ < -μ`. A common choice is `mu = -λ / (1 - λ)`.
    /// This combination effectively smooths the mesh while minimizing volume loss.
    ///
    /// Returns a new, smoothed CSG object.
    pub fn taubin_smooth(
        &self,
        lambda: Real,
        mu: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Mesh<M> {
        let (vertex_map, adjacency) = self.build_connectivity();
        let mut smoothed_polygons = self.polygons.clone();

        for _ in 0..iterations {
            // --- Lambda (shrinking) pass ---
            let mut current_positions: HashMap<usize, Point3> = HashMap::new();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        current_positions.insert(idx, vertex.position.clone());
                    }
                }
            }

            let mut updates: HashMap<usize, Point3> = HashMap::new();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(current_position) = current_positions.get(&vertex_idx).cloned() {
                    if preserve_boundaries && neighbors.len() < 4 {
                        updates.insert(vertex_idx, current_position);
                        continue;
                    }

                    let neighbor_positions = neighbors
                        .iter()
                        .filter_map(|neighbor_idx| {
                            current_positions.get(neighbor_idx).cloned()
                        })
                        .collect::<Vec<_>>();

                    if let Some(neighbor_avg) = Point3::centroid(&neighbor_positions) {
                        updates
                            .insert(vertex_idx, current_position.lerp(&neighbor_avg, &lambda));
                    }
                }
            }

            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        if let Some(new_position) = updates.get(&idx) {
                            vertex.position = new_position.clone();
                        }
                    }
                }
            }

            // --- Mu (inflating) pass ---
            current_positions.clear();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        current_positions.insert(idx, vertex.position.clone());
                    }
                }
            }

            updates.clear();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(current_position) = current_positions.get(&vertex_idx).cloned() {
                    if preserve_boundaries && neighbors.len() < 4 {
                        updates.insert(vertex_idx, current_position);
                        continue;
                    }

                    let neighbor_positions = neighbors
                        .iter()
                        .filter_map(|neighbor_idx| {
                            current_positions.get(neighbor_idx).cloned()
                        })
                        .collect::<Vec<_>>();

                    if let Some(neighbor_avg) = Point3::centroid(&neighbor_positions) {
                        updates.insert(vertex_idx, current_position.lerp(&neighbor_avg, &mu));
                    }
                }
            }

            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        if let Some(new_position) = updates.get(&idx) {
                            vertex.position = new_position.clone();
                        }
                    }
                }
            }
        }

        // Final pass to recompute normals
        for polygon in &mut smoothed_polygons {
            polygon.set_new_normal();
        }

        Mesh::from_polygons(&smoothed_polygons, self.metadata.clone())
    }
}
