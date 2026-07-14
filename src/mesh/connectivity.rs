//! Mesh connectivity helpers for exact vertex indexing and topology analysis.

use crate::mesh::Mesh;
use crate::mesh::hypermesh::hypermesh_edges;
use crate::vertex::Vertex;
use hashbrown::HashMap;
use hyperlattice::Point3;
use std::fmt::Debug;

/// **Mathematical Foundation: Robust Vertex Indexing for Mesh Connectivity**
///
/// Handles boundary-coordinate comparison exactly:
/// - **Exact Matching**: Considers vertices identical only when promoted
///   hyperreal coordinates prove equality
/// - **Global Indexing**: Maintains consistent vertex indices across mesh
///
/// The identity predicate promotes candidate positions into `hyperlattice`
/// vectors and requires exact zero squared distance in `Real`. That
/// keeps mesh-topology equivalence decisions on the exact-aware side of the API
/// boundary, following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
#[derive(Debug, Clone)]
pub struct VertexIndexMap {
    /// Maps vertex positions to global indices.
    pub position_to_index: Vec<(Point3, usize)>,
    /// Maps global indices to representative positions.
    pub index_to_position: HashMap<usize, Point3>,
    position_id_to_index: HashMap<u64, usize>,
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
            position_id_to_index: HashMap::new(),
        }
    }

    /// Get the existing index for an exactly equal position.
    pub fn find_index(&self, position: &Point3) -> Option<usize> {
        self.position_to_index
            .iter()
            .find_map(|(existing_position, existing_index)| {
                let position = hyperlimit::Point3::new(
                    position.x.clone(),
                    position.y.clone(),
                    position.z.clone(),
                );
                let existing_position = hyperlimit::Point3::new(
                    existing_position.x.clone(),
                    existing_position.y.clone(),
                    existing_position.z.clone(),
                );
                matches!(
                    hyperlimit::point3_equal(&position, &existing_position).value(),
                    Some(true)
                )
                .then_some(*existing_index)
            })
    }

    pub(crate) fn find_vertex_index(&self, vertex: &Vertex) -> Option<usize> {
        self.position_id_to_index
            .get(&vertex.position_id)
            .copied()
            .or_else(|| self.find_index(&vertex.position))
    }

    /// Get or create an index for a vertex position
    pub fn get_or_create_index(&mut self, position: Point3) -> usize {
        if let Some(existing_index) = self.find_index(&position) {
            return existing_index;
        }

        // Create new index
        let new_index = self.position_to_index.len();
        self.position_to_index.push((position.clone(), new_index));
        self.index_to_position.insert(new_index, position);
        new_index
    }

    /// Get the position for a given index
    pub fn get_position(&self, index: usize) -> Option<Point3> {
        self.index_to_position.get(&index).cloned()
    }

    /// Get total number of unique vertices
    pub const fn vertex_count(&self) -> usize {
        self.position_to_index.len()
    }

    /// Get all vertex positions and their indices (for iteration)
    pub const fn get_vertex_positions(&self) -> &Vec<(Point3, usize)> {
        &self.position_to_index
    }
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Mathematical Foundation: Robust Mesh Connectivity Analysis**
    ///
    /// Build a proper vertex adjacency graph from the exact `hypermesh` adapter:
    ///
    /// ## **Vertex Matching Algorithm**
    /// 1. **Exact Import**: `hypermesh` validates the current triangle stream
    ///    as a boundary-allowed surface adapter
    /// 2. **Global Indexing**: Each exact hypermesh input vertex keeps its
    ///    global index
    /// 3. **Edge Facts**: Triangle indices expose bidirectional connectivity
    ///    without primitive-coordinate matching
    /// 4. **Manifold Validation**: Closed-manifold decisions use the same exact
    ///    imported triangle stream
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

        let Ok((mesh, position_id_to_index)) = self.to_hypermesh_connectivity_mesh() else {
            return (vertex_map, adjacency);
        };
        vertex_map.position_id_to_index = position_id_to_index;
        for (index, point) in mesh.positions.iter().enumerate() {
            vertex_map.position_to_index.push((point.clone(), index));
            vertex_map.index_to_position.insert(index, point.clone());
        }

        for [a, b] in hypermesh_edges(&mesh) {
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
    use crate::hyper_math::{Real, hreal_from_f64};

    fn r(value: f64) -> Real {
        hreal_from_f64(value).expect("test values must be finite")
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn vertex_index_map_shares_only_exact_positions() {
        let mut map = VertexIndexMap::new();
        let base = p3(0.0, 0.0, 0.0);
        let exact = p3(0.0, 0.0, 0.0);
        let near = p3(1.0e-12, 0.0, 0.0);
        let far = p3(2.0, 0.0, 0.0);

        let base_index = map.get_or_create_index(base);
        assert_eq!(map.get_or_create_index(exact), base_index);
        assert_ne!(map.get_or_create_index(near), base_index);
        assert_ne!(map.get_or_create_index(far), base_index);
    }
}
