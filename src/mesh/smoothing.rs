//! Mesh smoothing and refinement operations.

use crate::mesh::Mesh;
use crate::mesh::connectivity::VertexIndexMap;
use hashbrown::HashMap;
use hyperlattice::{Point3, Real};
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
        let polygon_vertex_indices = self.polygon_vertex_indices(&vertex_map);
        let mut smoothed_polygons = self.polygons.clone();
        let mut positions = vertex_map.index_to_position.clone();

        for iteration in 0..iterations {
            positions = smoothing_pass(&positions, &adjacency, &lambda, preserve_boundaries);

            // Progress feedback for long smoothing operations
            if iterations > 10 && iteration % (iterations / 10) == 0 {
                eprintln!(
                    "Smoothing progress: {}/{} iterations",
                    iteration + 1,
                    iterations
                );
            }
        }

        apply_positions(&mut smoothed_polygons, &polygon_vertex_indices, &positions);
        Mesh::from_polygons(smoothed_polygons)
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
        let polygon_vertex_indices = self.polygon_vertex_indices(&vertex_map);
        let mut smoothed_polygons = self.polygons.clone();
        let mut positions = vertex_map.index_to_position.clone();

        for _ in 0..iterations {
            positions = smoothing_pass(&positions, &adjacency, &lambda, preserve_boundaries);
            positions = smoothing_pass(&positions, &adjacency, &mu, preserve_boundaries);
        }

        apply_positions(&mut smoothed_polygons, &polygon_vertex_indices, &positions);
        Mesh::from_polygons(smoothed_polygons)
    }

    fn polygon_vertex_indices(&self, vertex_map: &VertexIndexMap) -> Vec<Vec<Option<usize>>> {
        self.polygons
            .iter()
            .map(|polygon| {
                polygon
                    .vertices
                    .iter()
                    .map(|vertex| vertex_map.find_index(&vertex.position))
                    .collect()
            })
            .collect()
    }
}

fn smoothing_pass(
    positions: &HashMap<usize, Point3>,
    adjacency: &HashMap<usize, Vec<usize>>,
    factor: &Real,
    preserve_boundaries: bool,
) -> HashMap<usize, Point3> {
    positions
        .iter()
        .map(|(&vertex_index, current)| {
            let Some(neighbors) = adjacency.get(&vertex_index) else {
                return (vertex_index, current.clone());
            };
            if preserve_boundaries && neighbors.len() < 4 {
                return (vertex_index, current.clone());
            }

            let neighbor_positions = neighbors
                .iter()
                .filter_map(|neighbor| positions.get(neighbor).cloned())
                .collect::<Vec<_>>();
            let updated = Point3::centroid(&neighbor_positions)
                .map_or_else(|| current.clone(), |average| current.lerp(&average, factor));
            (vertex_index, updated)
        })
        .collect()
}

fn apply_positions<M: Clone + Send + Sync>(
    polygons: &mut [crate::mesh::Polygon<M>],
    polygon_vertex_indices: &[Vec<Option<usize>>],
    positions: &HashMap<usize, Point3>,
) {
    for (polygon, indices) in polygons.iter_mut().zip(polygon_vertex_indices) {
        let mut vertices = polygon.vertices_mut();
        for (vertex, index) in vertices.iter_mut().zip(indices) {
            if let Some(position) = index.and_then(|index| positions.get(&index)) {
                vertex.position = position.clone();
            }
        }
    }
}
