//! Mesh connectivity helpers for exact vertex indexing and topology analysis.

use crate::float_types::{Real, hpoints_exactly_equal};
use crate::mesh::Mesh;
use hashbrown::HashMap;
use nalgebra::Point3;
use std::fmt::Debug;

/// **Mathematical Foundation: Robust Vertex Indexing for Mesh Connectivity**
///
/// Handles boundary-coordinate comparison exactly:
/// - **Exact Matching**: Considers vertices identical only when promoted
///   hyperreal coordinates prove equality
/// - **Global Indexing**: Maintains consistent vertex indices across mesh
///
/// The identity predicate promotes candidate positions into `hyperlattice`
/// vectors and requires exact zero squared distance in `hyperreal::Real`. That
/// keeps mesh-topology equivalence decisions on the exact-aware side of the API
/// boundary, following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
#[derive(Debug, Clone)]
pub struct VertexIndexMap {
    /// Maps vertex positions to global indices.
    pub position_to_index: Vec<(Point3<Real>, usize)>,
    /// Maps global indices to representative positions.
    pub index_to_position: HashMap<usize, Point3<Real>>,
}

impl VertexIndexMap {
    /// Create a new exact vertex index map.
    ///
    /// No tolerance is stored or accepted: two boundary positions share an index
    /// only when Hyper proves their promoted coordinates are equal. That keeps
    /// connectivity equivalence on the exact-aware side of Yap's exact
    /// geometric computation boundary model (1997).
    pub fn new() -> Self {
        Self {
            position_to_index: Vec::new(),
            index_to_position: HashMap::new(),
        }
    }

    /// Get the existing index for an exactly equal position.
    pub fn find_index(&self, position: &Point3<Real>) -> Option<usize> {
        self.position_to_index
            .iter()
            .find_map(|(existing_position, existing_index)| {
                hpoints_exactly_equal(position, existing_position).then_some(*existing_index)
            })
    }

    /// Get or create an index for a vertex position
    pub fn get_or_create_index(&mut self, position: Point3<Real>) -> usize {
        if let Some(existing_index) = self.find_index(&position) {
            return existing_index;
        }

        // Create new index
        let new_index = self.position_to_index.len();
        self.position_to_index.push((position, new_index));
        self.index_to_position.insert(new_index, position);
        new_index
    }

    /// Get the position for a given index
    pub fn get_position(&self, index: usize) -> Option<Point3<Real>> {
        self.index_to_position.get(&index).copied()
    }

    /// Get total number of unique vertices
    pub fn vertex_count(&self) -> usize {
        self.position_to_index.len()
    }

    /// Get all vertex positions and their indices (for iteration)
    pub const fn get_vertex_positions(&self) -> &Vec<(Point3<Real>, usize)> {
        &self.position_to_index
    }
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Mathematical Foundation: Robust Mesh Connectivity Analysis**
    ///
    /// Build a proper vertex adjacency graph from accepted `hypermesh` topology:
    ///
    /// ## **Vertex Matching Algorithm**
    /// 1. **Exact Import**: `hypermesh` validates the current triangle stream
    ///    with a boundary-allowed surface policy
    /// 2. **Global Indexing**: Each exact hypermesh vertex keeps its global index
    /// 3. **Edge Facts**: Retained `MeshValidationFacts::edges` supply
    ///    bidirectional connectivity
    /// 4. **Manifold Validation**: Closed-manifold decisions are delegated to
    ///    `hypermesh` rather than repeated here
    ///
    /// Keeping adjacency on retained hypermesh edge facts avoids a second
    /// approximate topology model in `csgrs`. This follows Yap, "Towards Exact
    /// Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), and the CGAL
    /// triangulation-data-structure separation of vertices, edges, and faces
    /// described by Boissonnat et al., *Computational Geometry* 22.1-3 (2002).
    ///
    /// Returns (vertex_map, adjacency_graph) for robust mesh processing.
    pub fn build_connectivity(&self) -> (VertexIndexMap, HashMap<usize, Vec<usize>>) {
        let mut vertex_map = VertexIndexMap::new();
        let mut adjacency: HashMap<usize, Vec<usize>> = HashMap::new();

        let Ok(mesh) = self.to_hypermesh_exact_with_policy(
            ::hypermesh::exact::ValidationPolicy::ALLOW_BOUNDARY,
        ) else {
            return (vertex_map, adjacency);
        };
        if mesh.validate_retained_state().is_err() {
            return (vertex_map, adjacency);
        }
        let Ok(view) = mesh.approximate_f64_view() else {
            return (vertex_map, adjacency);
        };

        for (index, coords) in view.positions.chunks_exact(3).enumerate() {
            let position = Point3::new(coords[0], coords[1], coords[2]);
            vertex_map.position_to_index.push((position, index));
            vertex_map.index_to_position.insert(index, position);
        }

        for edge in &mesh.facts().edges {
            let [a, b] = edge.vertices;
            add_adjacency_edge(&mut adjacency, a, b);
        }

        for (vertex_idx, neighbors) in adjacency.iter_mut() {
            neighbors.sort_unstable();
            neighbors.dedup();
            neighbors.retain(|&neighbor| neighbor != *vertex_idx);
        }

        (vertex_map, adjacency)
    }
}

fn add_adjacency_edge(adjacency: &mut HashMap<usize, Vec<usize>>, a: usize, b: usize) {
    adjacency.entry(a).or_default().push(b);
    adjacency.entry(b).or_default().push(a);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vertex_index_map_shares_only_exact_positions() {
        let mut map = VertexIndexMap::new();
        let base = Point3::new(0.0, 0.0, 0.0);
        let exact = Point3::new(0.0, 0.0, 0.0);
        let near = Point3::new(1.0e-12, 0.0, 0.0);
        let far = Point3::new(2.0, 0.0, 0.0);

        let base_index = map.get_or_create_index(base);
        assert_eq!(map.get_or_create_index(exact), base_index);
        assert_ne!(map.get_or_create_index(near), base_index);
        assert_ne!(map.get_or_create_index(far), base_index);
    }

    #[test]
    fn vertex_index_map_rejects_nonfinite_positions() {
        let mut map = VertexIndexMap::new();
        let base = Point3::new(0.0, 0.0, 0.0);
        let nonfinite = Point3::new(Real::NAN, 0.0, 0.0);

        let base_index = map.get_or_create_index(base);
        assert_eq!(map.find_index(&nonfinite), None);
        assert_ne!(map.get_or_create_index(nonfinite), base_index);
    }
}
