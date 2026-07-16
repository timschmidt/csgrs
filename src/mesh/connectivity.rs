//! Mesh connectivity helpers for exact vertex indexing and topology analysis.

use crate::mesh::Mesh;
use crate::vertex::Vertex;
use hashbrown::HashMap;
use hyperlattice::Point3;
use std::cell::RefCell;
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

#[derive(Clone, Debug)]
struct CachedConnectivity {
    geometry_identity: Vec<u64>,
    vertex_map: VertexIndexMap,
    adjacency: HashMap<usize, Vec<usize>>,
}

thread_local! {
    static CONNECTIVITY_FACTS: RefCell<Vec<CachedConnectivity>> = const { RefCell::new(Vec::new()) };
}

const CONNECTIVITY_FACT_CAPACITY: usize = 8;

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
    /// 1. **Exact Import**: the audited `hypermesh` adapter canonicalizes the
    ///    current triangle stream without requiring Boolean-surface validation
    /// 2. **Global Indexing**: Each exact hypermesh input vertex keeps its
    ///    global index
    /// 3. **Edge Facts**: Triangle indices expose bidirectional connectivity
    ///    without primitive-coordinate matching
    /// 4. **Shared Carrier**: Closed-manifold decisions use the same exact
    ///    imported triangle stream, with their additional validation performed
    ///    by [`Mesh::is_manifold`]
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
        if let Some(cached) = CONNECTIVITY_FACTS.with_borrow(|facts| {
            facts
                .iter()
                .rev()
                .find(|fact| self.geometry_identity_matches(&fact.geometry_identity))
                .cloned()
        }) {
            return (cached.vertex_map, cached.adjacency);
        }

        let (vertex_map, adjacency) = self.build_connectivity_uncached();
        self.retain_connectivity(vertex_map.clone(), adjacency.clone());
        (vertex_map, adjacency)
    }

    fn build_connectivity_uncached(&self) -> (VertexIndexMap, HashMap<usize, Vec<usize>>) {
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

        adjacency.reserve(mesh.positions.len());
        for triangle in &mesh.triangles {
            let [a, b, c] = triangle.indices();
            for [a, b] in [[a, b], [b, c], [c, a]] {
                add_adjacency_edge(&mut adjacency, a, b);
            }
        }

        for (vertex_idx, neighbors) in adjacency.iter_mut() {
            neighbors.sort_unstable();
            neighbors.dedup();
            neighbors.retain(|&neighbor| neighbor != *vertex_idx);
        }

        (vertex_map, adjacency)
    }

    pub(super) fn cache_connectivity(&self) {
        let (vertex_map, adjacency) = self.build_connectivity_uncached();
        self.retain_connectivity(vertex_map, adjacency);
    }

    fn retain_connectivity(
        &self,
        vertex_map: VertexIndexMap,
        adjacency: HashMap<usize, Vec<usize>>,
    ) {
        let geometry_identity = self.geometry_identity();
        CONNECTIVITY_FACTS.with_borrow_mut(|facts| {
            if let Some(index) = facts
                .iter()
                .position(|fact| fact.geometry_identity == geometry_identity)
            {
                facts.remove(index);
            }
            if facts.len() == CONNECTIVITY_FACT_CAPACITY {
                facts.remove(0);
            }
            facts.push(CachedConnectivity {
                geometry_identity,
                vertex_map,
                adjacency,
            });
        });
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
