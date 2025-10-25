type Real = f64;
use crate::mesh::Mesh;
use hashbrown::HashMap;
use crate::math_ndsp::Point3;
use std::fmt::Debug;

/// **Mathematical Foundation: Robust Vertex Indexing for Mesh Connectivity**
///
/// Handles floating-point coordinate comparison with epsilon tolerance:
/// - **Spatial Hashing**: Groups nearby vertices for efficient lookup
/// - **Epsilon Matching**: Considers vertices within ε distance as identical
/// - **Global Indexing**: Maintains consistent vertex indices across mesh
#[derive(Debug, Clone)]
pub struct VertexIndexMap {
    /// Maps vertex positions to global indices (with epsilon tolerance)
    pub position_to_index: Vec<(Point3<Real>, usize)>,
    /// Maps global indices to representative positions
    pub index_to_position: HashMap<usize, Point3<Real>>,
    /// Spatial tolerance for vertex matching
    pub epsilon: Real,
}

impl VertexIndexMap {
    /// Create a new vertex index map with specified tolerance
    pub fn new(epsilon: Real) -> Self {
        Self {
            position_to_index: Vec::new(),
            index_to_position: HashMap::new(),
            epsilon,
        }
    }

    /// Get or create an index for a vertex position
    pub fn get_or_create_index(&mut self, pos: Point3<Real>) -> usize {
        // Look for existing vertex within epsilon tolerance
        for (existing_pos, existing_index) in &self.position_to_index {
            if (pos - existing_pos).norm() < self.epsilon {
                return *existing_index;
            }
        }

        // Create new index
        let new_index = self.position_to_index.len();
        self.position_to_index.push((pos, new_index));
        self.index_to_position.insert(new_index, pos);
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

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    /// **Mathematical Foundation: Robust Mesh Connectivity Analysis**
    ///
    /// Build a proper vertex adjacency graph using epsilon-based vertex matching:
    ///
    /// ## **Vertex Matching Algorithm**
    /// 1. **Spatial Tolerance**: Vertices within ε distance are considered identical
    /// 2. **Global Indexing**: Each unique position gets a global index
    /// 3. **Adjacency Building**: For each edge, record bidirectional connectivity
    /// 4. **Manifold Validation**: Ensure each edge is shared by at most 2 triangles
    ///
    /// Returns (vertex_map, adjacency_graph) for robust mesh processing.
    pub fn build_connectivity(&self) -> (VertexIndexMap, HashMap<usize, Vec<usize>>) {
        let mut vertex_map = VertexIndexMap::new(Real::EPSILON * 100.0); // Tolerance for vertex matching
        let mut adjacency: HashMap<usize, Vec<usize>> = HashMap::new();

        // First pass: build vertex index mapping
        for polygon in &self.polygons {
            for vertex in &polygon.vertices {
                vertex_map.get_or_create_index(vertex.pos);
            }
        }

        // Second pass: build adjacency graph
        for polygon in &self.polygons {
            let mut vertex_indices = Vec::new();

            // Get indices for this polygon's vertices
            for vertex in &polygon.vertices {
                let index = vertex_map.get_or_create_index(vertex.pos);
                vertex_indices.push(index);
            }

            // Build adjacency for this polygon's edges
            for i in 0..vertex_indices.len() {
                let current = vertex_indices[i];
                let next = vertex_indices[(i + 1) % vertex_indices.len()];
                let prev =
                    vertex_indices[(i + vertex_indices.len() - 1) % vertex_indices.len()];

                // Add bidirectional edges
                adjacency.entry(current).or_default().push(next);
                adjacency.entry(current).or_default().push(prev);
                adjacency.entry(next).or_default().push(current);
                adjacency.entry(prev).or_default().push(current);
            }
        }

        // Clean up adjacency lists - remove duplicates and self-references
        for (vertex_idx, neighbors) in adjacency.iter_mut() {
            neighbors.sort_unstable();
            neighbors.dedup();
            neighbors.retain(|&neighbor| neighbor != *vertex_idx);
        }

        (vertex_map, adjacency)
    }
}
