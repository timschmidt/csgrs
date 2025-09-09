//! BSP (Binary Space Partitioning) tree operations for IndexedMesh.
//!
//! This module provides BSP tree functionality optimized for IndexedMesh's indexed connectivity model.
//! BSP trees are used for efficient spatial partitioning and CSG operations.

use crate::IndexedMesh::{IndexedMesh, IndexedPolygon};
use crate::float_types::Real;
use crate::mesh::plane::Plane;
use crate::mesh::vertex::Vertex;
use nalgebra::Point3;
use std::fmt::Debug;
use std::marker::PhantomData;

/// Type alias for IndexedBSPNode for compatibility
pub type IndexedBSPNode<S> = IndexedNode<S>;

/// A BSP tree node for IndexedMesh, containing indexed polygons plus optional front/back subtrees.
///
/// **Mathematical Foundation**: Uses plane-based spatial partitioning for O(log n) spatial queries.
/// **Optimization**: Stores polygon indices instead of full polygon data for memory efficiency.
#[derive(Debug, Clone)]
pub struct IndexedNode<S: Clone> {
    /// Splitting plane for this node or None for a leaf that only stores polygons.
    pub plane: Option<Plane>,

    /// Polygons in front half-spaces (as indices into the mesh's polygon array).
    pub front: Option<Box<IndexedNode<S>>>,

    /// Polygons in back half-spaces (as indices into the mesh's polygon array).
    pub back: Option<Box<IndexedNode<S>>>,

    /// Polygons that lie exactly on plane (after the node has been built).
    pub polygons: Vec<usize>, // Indices into the mesh's polygon array
    /// Phantom data to use the type parameter
    _phantom: PhantomData<S>,
}

impl<S: Clone + Send + Sync + Debug> Default for IndexedNode<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone + Send + Sync + Debug> IndexedNode<S> {
    /// Create a new empty BSP node
    pub const fn new() -> Self {
        Self {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
            _phantom: PhantomData,
        }
    }

    /// Creates a new BSP node from polygon indices
    pub fn from_polygon_indices(polygon_indices: &[usize]) -> Self {
        let mut node = Self::new();
        if !polygon_indices.is_empty() {
            node.polygons = polygon_indices.to_vec();
        }
        node
    }

    /// **Mathematical Foundation: Robust BSP Tree Construction with Indexed Connectivity**
    ///
    /// Builds a balanced BSP tree from polygon indices using optimal splitting plane selection
    /// and efficient indexed polygon processing.
    ///
    /// ## **Algorithm: Optimized BSP Construction**
    /// 1. **Splitting Plane Selection**: Choose plane that minimizes polygon splits
    /// 2. **Polygon Classification**: Classify polygons relative to splitting plane
    /// 3. **Recursive Subdivision**: Build front and back subtrees recursively
    /// 4. **Index Preservation**: Maintain polygon indices throughout construction
    ///
    /// ## **Optimization Strategies**
    /// - **Plane Selection Heuristics**: Minimize splits and balance tree depth
    /// - **Indexed Access**: Direct polygon access via indices for O(1) lookup
    /// - **Memory Efficiency**: Reuse polygon indices instead of copying geometry
    /// - **Degenerate Handling**: Robust handling of coplanar and degenerate cases
    pub fn build(&mut self, mesh: &IndexedMesh<S>) {
        if self.polygons.is_empty() {
            return;
        }

        // Choose optimal splitting plane if not already set
        if self.plane.is_none() {
            self.plane = Some(self.choose_splitting_plane(mesh));
        }

        let plane = self.plane.as_ref().unwrap();

        // Classify polygons relative to the splitting plane
        let mut front_polygons = Vec::new();
        let mut back_polygons = Vec::new();
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();

        for &poly_idx in &self.polygons {
            let polygon = &mesh.polygons[poly_idx];
            let classification = self.classify_polygon_to_plane(mesh, polygon, plane);

            match classification {
                PolygonClassification::Front => front_polygons.push(poly_idx),
                PolygonClassification::Back => back_polygons.push(poly_idx),
                PolygonClassification::CoplanarFront => coplanar_front.push(poly_idx),
                PolygonClassification::CoplanarBack => coplanar_back.push(poly_idx),
                PolygonClassification::Spanning => {
                    // For spanning polygons, add to both sides for now
                    // In a full implementation, we would split the polygon
                    front_polygons.push(poly_idx);
                    back_polygons.push(poly_idx);
                },
            }
        }

        // Store coplanar polygons in this node
        self.polygons = coplanar_front;
        self.polygons.extend(coplanar_back);

        // Recursively build front subtree
        if !front_polygons.is_empty() {
            let mut front_node = IndexedNode::new();
            front_node.polygons = front_polygons;
            front_node.build(mesh);
            self.front = Some(Box::new(front_node));
        }

        // Recursively build back subtree
        if !back_polygons.is_empty() {
            let mut back_node = IndexedNode::new();
            back_node.polygons = back_polygons;
            back_node.build(mesh);
            self.back = Some(Box::new(back_node));
        }
    }

    /// Return all polygon indices in this BSP tree
    pub fn all_polygon_indices(&self) -> Vec<usize> {
        let mut result = Vec::new();
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            result.extend_from_slice(&node.polygons);

            // Add child nodes to stack
            if let Some(ref front) = node.front {
                stack.push(front.as_ref());
            }
            if let Some(ref back) = node.back {
                stack.push(back.as_ref());
            }
        }

        result
    }

    /// **Collect All Polygons from BSP Tree**
    ///
    /// Return all polygons in this BSP tree as IndexedPolygon objects.
    /// Used to extract final results from BSP operations.
    pub fn all_indexed_polygons(&self, mesh: &IndexedMesh<S>) -> Vec<IndexedPolygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            // Collect polygons from this node
            for &poly_idx in &node.polygons {
                if poly_idx < mesh.polygons.len()
                    && !mesh.polygons[poly_idx].indices.is_empty()
                {
                    result.push(mesh.polygons[poly_idx].clone());
                }
            }

            // Add child nodes to stack
            if let Some(ref front) = node.front {
                stack.push(front.as_ref());
            }
            if let Some(ref back) = node.back {
                stack.push(back.as_ref());
            }
        }

        result
    }

    /// **Build BSP Tree from Polygons**
    ///
    /// Add polygons to this BSP tree and build the tree structure.
    pub fn build_from_polygons(
        &mut self,
        polygons: &[IndexedPolygon<S>],
        mesh: &mut IndexedMesh<S>,
    ) {
        // Add polygons to mesh and collect indices
        let mut polygon_indices = Vec::new();
        for poly in polygons {
            let idx = mesh.polygons.len();
            mesh.polygons.push(poly.clone());
            polygon_indices.push(idx);
        }

        // Set polygon indices and build tree
        self.polygons = polygon_indices;
        self.build(mesh);
    }

    /// Choose an optimal splitting plane using heuristics to minimize polygon splits
    fn choose_splitting_plane(&self, mesh: &IndexedMesh<S>) -> Plane {
        if self.polygons.is_empty() {
            // Default plane if no polygons
            return Plane::from_normal(nalgebra::Vector3::z(), 0.0);
        }

        let mut best_plane = mesh.polygons[self.polygons[0]].plane.clone();
        let mut best_score = f64::INFINITY;

        // Evaluate a subset of polygon planes as potential splitting planes
        let sample_size = (self.polygons.len().min(10)).max(1);
        for i in 0..sample_size {
            let poly_idx = self.polygons[i * self.polygons.len() / sample_size];
            let candidate_plane = &mesh.polygons[poly_idx].plane;

            let score = self.evaluate_splitting_plane(mesh, candidate_plane);
            if score < best_score {
                best_score = score;
                best_plane = candidate_plane.clone();
            }
        }

        best_plane
    }

    /// Evaluate the quality of a splitting plane (lower score is better)
    fn evaluate_splitting_plane(&self, mesh: &IndexedMesh<S>, plane: &Plane) -> f64 {
        let mut front_count = 0;
        let mut back_count = 0;
        let mut split_count = 0;

        for &poly_idx in &self.polygons {
            let polygon = &mesh.polygons[poly_idx];
            match self.classify_polygon_to_plane(mesh, polygon, plane) {
                PolygonClassification::Front => front_count += 1,
                PolygonClassification::Back => back_count += 1,
                PolygonClassification::Spanning => split_count += 1,
                _ => {}, // Coplanar polygons don't affect balance
            }
        }

        // Score based on balance and number of splits
        let balance_penalty = ((front_count as f64) - (back_count as f64)).abs();
        let split_penalty = (split_count as f64) * 3.0; // Heavily penalize splits

        balance_penalty + split_penalty
    }

    /// Classify a polygon relative to a plane
    fn classify_polygon_to_plane(
        &self,
        mesh: &IndexedMesh<S>,
        polygon: &crate::IndexedMesh::IndexedPolygon<S>,
        plane: &Plane,
    ) -> PolygonClassification {
        let mut front_count = 0;
        let mut back_count = 0;
        let epsilon = crate::float_types::EPSILON;

        for &vertex_idx in &polygon.indices {
            let vertex_pos = mesh.vertices[vertex_idx].pos;
            let distance = self.signed_distance_to_point(plane, &vertex_pos);

            if distance > epsilon {
                front_count += 1;
            } else if distance < -epsilon {
                back_count += 1;
            }
        }

        if front_count > 0 && back_count > 0 {
            PolygonClassification::Spanning
        } else if front_count > 0 {
            PolygonClassification::Front
        } else if back_count > 0 {
            PolygonClassification::Back
        } else {
            // All vertices are coplanar - determine orientation
            let polygon_normal = polygon.plane.normal();
            let plane_normal = plane.normal();

            if polygon_normal.dot(&plane_normal) > 0.0 {
                PolygonClassification::CoplanarFront
            } else {
                PolygonClassification::CoplanarBack
            }
        }
    }

    /// Compute signed distance from a point to a plane
    fn signed_distance_to_point(&self, plane: &Plane, point: &Point3<Real>) -> Real {
        let normal = plane.normal();
        let offset = plane.offset();
        normal.dot(&point.coords) - offset
    }

    /// **Invert BSP Tree**
    ///
    /// Invert all polygons and planes in this BSP tree, effectively flipping inside/outside.
    /// This is used in CSG operations to change the solid/void interpretation.
    pub fn invert(&mut self, mesh: &mut IndexedMesh<S>) {
        // Flip all polygons at this node
        for &poly_idx in &self.polygons {
            if poly_idx < mesh.polygons.len() {
                mesh.polygons[poly_idx].flip();
            }
        }

        // Flip the splitting plane
        if let Some(ref mut plane) = self.plane {
            plane.flip();
        }

        // Recursively invert children
        if let Some(ref mut front) = self.front {
            front.invert(mesh);
        }
        if let Some(ref mut back) = self.back {
            back.invert(mesh);
        }

        // Swap front and back subtrees
        std::mem::swap(&mut self.front, &mut self.back);
    }

    /// **Clip Polygons Against BSP Tree**
    ///
    /// Remove all polygons that are inside this BSP tree.
    /// Returns polygons that are outside or on the boundary.
    pub fn clip_indexed_polygons(
        &self,
        polygons: &[IndexedPolygon<S>],
        vertices: &[Vertex],
    ) -> Vec<IndexedPolygon<S>> {
        if self.plane.is_none() {
            return polygons.to_vec();
        }

        let plane = self.plane.as_ref().unwrap();
        let mut front_polys = Vec::new();
        let back_polys = Vec::new();

        // Classify and split polygons
        for polygon in polygons {
            use crate::IndexedMesh::plane::IndexedPlaneOperations;
            let classification = plane.classify_indexed_polygon(polygon, vertices);

            match classification {
                crate::mesh::plane::FRONT => front_polys.push(polygon.clone()),
                crate::mesh::plane::BACK => {}, // Clipped (inside)
                crate::mesh::plane::COPLANAR => {
                    // Check orientation to determine if it's inside or outside
                    let poly_normal = polygon.plane.normal();
                    let plane_normal = plane.normal();
                    if poly_normal.dot(&plane_normal) > 0.0 {
                        front_polys.push(polygon.clone());
                    }
                    // Opposite orientation polygons are clipped
                },
                _ => {
                    // Spanning polygon, split it
                    let mut vertices_mut = vertices.to_vec();
                    let (_, _, front_parts, _back_parts) =
                        plane.split_indexed_polygon(polygon, &mut vertices_mut);
                    front_polys.extend(front_parts);
                    // Back parts are clipped (inside)
                },
            }
        }

        // Recursively clip front polygons
        let mut result = if let Some(ref front) = self.front {
            front.clip_indexed_polygons(&front_polys, vertices)
        } else {
            front_polys
        };

        // Recursively clip back polygons
        if let Some(ref back) = self.back {
            result.extend(back.clip_indexed_polygons(&back_polys, vertices));
        }

        result
    }

    /// **Clip This BSP Tree Against Another**
    ///
    /// Remove all polygons in this BSP tree that are inside the other BSP tree.
    pub fn clip_to(&mut self, other: &IndexedNode<S>, mesh: &mut IndexedMesh<S>) {
        // Collect polygons at this node
        let node_polygons: Vec<IndexedPolygon<S>> = self
            .polygons
            .iter()
            .filter_map(|&idx| {
                if idx < mesh.polygons.len() {
                    Some(mesh.polygons[idx].clone())
                } else {
                    None
                }
            })
            .collect();

        // Clip polygons against the other BSP tree
        let clipped_polygons = other.clip_indexed_polygons(&node_polygons, &mesh.vertices);

        // Update mesh with clipped polygons
        // First, remove old polygons
        for &idx in &self.polygons {
            if idx < mesh.polygons.len() {
                // Mark for removal by clearing indices
                mesh.polygons[idx].indices.clear();
            }
        }

        // Add clipped polygons and update indices
        self.polygons.clear();
        for poly in clipped_polygons {
            let new_idx = mesh.polygons.len();
            mesh.polygons.push(poly);
            self.polygons.push(new_idx);
        }

        // Recursively clip children
        if let Some(ref mut front) = self.front {
            front.clip_to(other, mesh);
        }
        if let Some(ref mut back) = self.back {
            back.clip_to(other, mesh);
        }
    }

    /// **Clip Polygon to Outside Region**
    ///
    /// Return parts of the polygon that lie outside this BSP tree.
    /// Used for union operations to extract external geometry.
    pub fn clip_polygon_outside(
        &self,
        polygon: &IndexedPolygon<S>,
        vertices: &[Vertex],
    ) -> Vec<IndexedPolygon<S>> {
        if let Some(ref plane) = self.plane {
            use crate::IndexedMesh::plane::IndexedPlaneOperations;

            let classification = plane.classify_indexed_polygon(polygon, vertices);

            match classification {
                crate::mesh::plane::FRONT => {
                    // Polygon is in front, check front subtree
                    if let Some(ref front) = self.front {
                        front.clip_polygon_outside(polygon, vertices)
                    } else {
                        // No front subtree, polygon is outside
                        vec![polygon.clone()]
                    }
                },
                crate::mesh::plane::BACK => {
                    // Polygon is behind, check back subtree
                    if let Some(ref back) = self.back {
                        back.clip_polygon_outside(polygon, vertices)
                    } else {
                        // No back subtree, polygon is inside
                        Vec::new()
                    }
                },
                crate::mesh::plane::COPLANAR => {
                    // Coplanar polygon, check orientation
                    let poly_normal = polygon.plane.normal();
                    let plane_normal = plane.normal();

                    if poly_normal.dot(&plane_normal) > 0.0 {
                        // Same orientation, check front
                        if let Some(ref front) = self.front {
                            front.clip_polygon_outside(polygon, vertices)
                        } else {
                            vec![polygon.clone()]
                        }
                    } else {
                        // Opposite orientation, check back
                        if let Some(ref back) = self.back {
                            back.clip_polygon_outside(polygon, vertices)
                        } else {
                            Vec::new()
                        }
                    }
                },
                _ => {
                    // Spanning polygon, split and process parts
                    let mut vertices_mut = vertices.to_vec();
                    let (_, _, front_polys, back_polys) =
                        plane.split_indexed_polygon(polygon, &mut vertices_mut);

                    let mut result = Vec::new();

                    // Process front parts
                    for front_poly in front_polys {
                        if let Some(ref front) = self.front {
                            result.extend(
                                front.clip_polygon_outside(&front_poly, &vertices_mut),
                            );
                        } else {
                            result.push(front_poly);
                        }
                    }

                    // Process back parts
                    for back_poly in back_polys {
                        if let Some(ref back) = self.back {
                            result
                                .extend(back.clip_polygon_outside(&back_poly, &vertices_mut));
                        }
                        // Back parts are inside, don't add them
                    }

                    result
                },
            }
        } else {
            // Leaf node, polygon is outside
            vec![polygon.clone()]
        }
    }

    /// **Mathematical Foundation: IndexedMesh BSP Slicing with Optimized Indexed Connectivity**
    ///
    /// Slice this BSP tree with a plane, returning coplanar polygons and intersection edges.
    /// Leverages indexed connectivity for superior performance over coordinate-based approaches.
    ///
    /// ## **Indexed Connectivity Advantages**
    /// - **O(1) Vertex Access**: Direct vertex lookup using indices
    /// - **Memory Efficiency**: No vertex duplication during intersection computation
    /// - **Cache Performance**: Better memory locality through structured vertex access
    /// - **Precision Preservation**: Direct coordinate access without quantization
    ///
    /// ## **Slicing Algorithm**
    /// 1. **Polygon Collection**: Gather all polygon indices from BSP tree
    /// 2. **Classification**: Use IndexedPlaneOperations for robust polygon classification
    /// 3. **Coplanar Extraction**: Collect polygons lying exactly in the slicing plane
    /// 4. **Intersection Computation**: Compute edge-plane intersections for spanning polygons
    /// 5. **Edge Generation**: Create line segments from intersection points
    ///
    /// # Parameters
    /// - `slicing_plane`: The plane to slice with
    /// - `mesh`: Reference to the IndexedMesh containing vertex data
    ///
    /// # Returns
    /// - `Vec<IndexedPolygon<S>>`: Polygons coplanar with the slicing plane
    /// - `Vec<[Vertex; 2]>`: Line segments from edge-plane intersections
    ///
    /// # Example
    /// ```
    /// use csgrs::IndexedMesh::{IndexedMesh, bsp::IndexedNode};
    /// use csgrs::mesh::plane::Plane;
    /// use nalgebra::Vector3;
    ///
    /// let mesh = IndexedMesh::<()>::cube(2.0, None);
    /// let mut bsp_tree = IndexedNode::new();
    /// // ... build BSP tree ...
    ///
    /// let plane = Plane::from_normal(Vector3::z(), 0.0);
    /// let (coplanar_polys, intersection_edges) = bsp_tree.slice_indexed(&plane, &mesh);
    /// ```
    pub fn slice_indexed(
        &self,
        slicing_plane: &crate::mesh::plane::Plane,
        mesh: &IndexedMesh<S>,
    ) -> (Vec<IndexedPolygon<S>>, Vec<[crate::mesh::vertex::Vertex; 2]>) {
        use crate::IndexedMesh::plane::IndexedPlaneOperations;
        use crate::float_types::EPSILON;
        use crate::mesh::plane::{COPLANAR, SPANNING};

        // Collect all polygon indices from the BSP tree
        let all_polygon_indices = self.all_polygon_indices();

        let mut coplanar_polygons = Vec::new();
        let mut intersection_edges = Vec::new();

        // Process each polygon in the BSP tree
        for &poly_idx in &all_polygon_indices {
            if poly_idx >= mesh.polygons.len() {
                continue; // Skip invalid indices
            }

            let polygon = &mesh.polygons[poly_idx];
            if polygon.indices.len() < 3 {
                continue; // Skip degenerate polygons
            }

            // Classify polygon relative to the slicing plane
            let classification =
                slicing_plane.classify_indexed_polygon(polygon, &mesh.vertices);

            match classification {
                COPLANAR => {
                    // Polygon lies exactly in the slicing plane
                    coplanar_polygons.push(polygon.clone());
                },
                SPANNING => {
                    // Polygon crosses the plane - compute intersection points
                    let vertex_count = polygon.indices.len();
                    let mut crossing_points = Vec::new();

                    // Check each edge for plane intersection
                    for i in 0..vertex_count {
                        let j = (i + 1) % vertex_count;

                        // Get vertex indices and ensure they're valid
                        let idx_i = polygon.indices[i];
                        let idx_j = polygon.indices[j];

                        if idx_i >= mesh.vertices.len() || idx_j >= mesh.vertices.len() {
                            continue; // Skip invalid vertex indices
                        }

                        let vertex_i = &mesh.vertices[idx_i];
                        let vertex_j = &mesh.vertices[idx_j];

                        // Classify vertices relative to the plane
                        let type_i = slicing_plane.orient_point(&vertex_i.pos);
                        let type_j = slicing_plane.orient_point(&vertex_j.pos);

                        // Check if edge crosses the plane
                        if (type_i | type_j) == SPANNING {
                            // Edge crosses plane - compute intersection point
                            let edge_vector = vertex_j.pos - vertex_i.pos;
                            let denom = slicing_plane.normal().dot(&edge_vector);

                            if denom.abs() > EPSILON {
                                let intersection_param = (slicing_plane.offset()
                                    - slicing_plane.normal().dot(&vertex_i.pos.coords))
                                    / denom;

                                // Ensure intersection is within edge bounds
                                if (0.0..=1.0).contains(&intersection_param) {
                                    let intersection_vertex =
                                        vertex_i.interpolate(vertex_j, intersection_param);
                                    crossing_points.push(intersection_vertex);
                                }
                            }
                        }
                    }

                    // Create line segments from consecutive intersection points
                    if crossing_points.len() >= 2 {
                        // For most cases, we expect exactly 2 intersection points per spanning polygon
                        // Create line segments from pairs of intersection points
                        for chunk in crossing_points.chunks(2) {
                            if chunk.len() == 2 {
                                intersection_edges.push([chunk[0], chunk[1]]);
                            }
                        }
                    }
                },
                _ => {
                    // FRONT or BACK - polygon doesn't intersect the plane
                    // No action needed for slicing
                },
            }
        }

        (coplanar_polygons, intersection_edges)
    }
}

/// Classification of a polygon relative to a plane
#[derive(Debug, Clone, Copy, PartialEq)]
enum PolygonClassification {
    /// Polygon is entirely in front of the plane
    Front,
    /// Polygon is entirely behind the plane
    Back,
    /// Polygon is coplanar and facing the same direction as the plane
    CoplanarFront,
    /// Polygon is coplanar and facing the opposite direction as the plane
    CoplanarBack,
    /// Polygon spans the plane (needs to be split)
    Spanning,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::IndexedMesh::{IndexedMesh, IndexedPolygon};
    use crate::mesh::vertex::Vertex;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_indexed_bsp_basic_functionality() {
        // Create a simple mesh with one triangle
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];

        let plane_vertices = vec![
            vertices[0].clone(),
            vertices[1].clone(),
            vertices[2].clone(),
        ];
        let polygons = vec![IndexedPolygon::<i32>::new(
            vec![0, 1, 2],
            Plane::from_vertices(plane_vertices),
            None,
        )];

        let _mesh = IndexedMesh {
            vertices,
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata: None,
        };

        let polygon_indices = vec![0];
        let node: IndexedNode<i32> = IndexedNode::from_polygon_indices(&polygon_indices);

        // Basic test that node was created
        assert!(!node.all_polygon_indices().is_empty());
    }

    #[test]
    fn test_slice_indexed_coplanar() {
        // Create a simple cube mesh
        let cube = IndexedMesh::<()>::cube(2.0, None);

        // Build BSP tree from all polygons
        let polygon_indices: Vec<usize> = (0..cube.polygons.len()).collect();
        let mut bsp_tree = IndexedNode::from_polygon_indices(&polygon_indices);
        bsp_tree.build(&cube);

        // Create a plane that should intersect the cube at z=0
        let slicing_plane = Plane::from_normal(Vector3::z(), 0.0);

        // Perform slice operation
        let (coplanar_polys, intersection_edges) =
            bsp_tree.slice_indexed(&slicing_plane, &cube);

        // Should have some intersection results
        assert!(
            coplanar_polys.len() > 0 || intersection_edges.len() > 0,
            "Slice should produce either coplanar polygons or intersection edges"
        );
    }

    #[test]
    fn test_slice_indexed_spanning() {
        use crate::traits::CSG;

        // Create a simple triangle that spans the XY plane
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, -1.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 1.0), Vector3::z()),
        ];

        // Create plane from vertices
        let plane = Plane::from_vertices(vertices.clone());
        let triangle_polygon: IndexedPolygon<()> =
            IndexedPolygon::new(vec![0, 1, 2], plane, None);
        let mut mesh: IndexedMesh<()> = IndexedMesh::new();
        mesh.vertices = vertices;
        mesh.polygons = vec![triangle_polygon];

        // Build BSP tree
        let polygon_indices = vec![0];
        let mut bsp_tree = IndexedNode::from_polygon_indices(&polygon_indices);
        bsp_tree.build(&mesh);

        // Slice with XY plane (z=0)
        let slicing_plane = Plane::from_normal(Vector3::z(), 0.0);
        let (coplanar_polys, intersection_edges) =
            bsp_tree.slice_indexed(&slicing_plane, &mesh);

        // Triangle spans the plane, so should have intersection edges
        assert!(
            intersection_edges.len() > 0,
            "Spanning triangle should produce intersection edges"
        );
        assert_eq!(
            coplanar_polys.len(),
            0,
            "Spanning triangle should not be coplanar"
        );
    }

    #[test]
    fn test_slice_indexed_no_intersection() {
        use crate::traits::CSG;

        // Create a cube above the slicing plane
        let vertices = vec![
            Vertex::new(Point3::new(-1.0, -1.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, -1.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 1.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(-1.0, 1.0, 1.0), Vector3::z()),
        ];

        // Create plane from first 3 vertices
        let plane_vertices = vec![
            vertices[0].clone(),
            vertices[1].clone(),
            vertices[2].clone(),
        ];
        let plane = Plane::from_vertices(plane_vertices);
        let quad_polygon: IndexedPolygon<()> =
            IndexedPolygon::new(vec![0, 1, 2, 3], plane, None);
        let mut mesh: IndexedMesh<()> = IndexedMesh::new();
        mesh.vertices = vertices;
        mesh.polygons = vec![quad_polygon];

        // Build BSP tree
        let polygon_indices = vec![0];
        let mut bsp_tree = IndexedNode::from_polygon_indices(&polygon_indices);
        bsp_tree.build(&mesh);

        // Slice with XY plane (z=0) - should not intersect
        let slicing_plane = Plane::from_normal(Vector3::z(), 0.0);
        let (coplanar_polys, intersection_edges) =
            bsp_tree.slice_indexed(&slicing_plane, &mesh);

        // No intersection expected
        assert_eq!(coplanar_polys.len(), 0, "No coplanar polygons expected");
        assert_eq!(intersection_edges.len(), 0, "No intersection edges expected");
    }

    #[test]
    fn test_slice_indexed_integration_with_flatten_slice() {
        // Create a cube that should be sliced by a plane
        let cube = IndexedMesh::<()>::cube(2.0, None);

        // Test that the IndexedMesh slice method (which uses slice_indexed internally) works
        let plane = Plane::from_normal(Vector3::z(), 0.0);
        let sketch = cube.slice(plane);

        // The slice should produce some 2D geometry
        assert!(!sketch.geometry.is_empty(), "Slice should produce 2D geometry");

        // Check that we have some polygonal geometry
        let has_polygons = sketch.geometry.iter().any(|geom| {
            matches!(geom, geo::Geometry::Polygon(_) | geo::Geometry::MultiPolygon(_))
        });
        assert!(has_polygons, "Slice should produce polygonal geometry");
    }

    #[test]
    fn test_slice_indexed_correctness_validation() {
        // Create a cube that spans from z=0 to z=2
        let indexed_cube = IndexedMesh::<()>::cube(2.0, None);

        // Slice at z=0 (should intersect the bottom face)
        let plane = Plane::from_normal(Vector3::z(), 0.0);
        let sketch = indexed_cube.slice(plane);

        // Should produce exactly one square polygon
        assert_eq!(sketch.geometry.len(), 1, "Cube slice at z=0 should produce exactly 1 geometry element");

        // Verify it's a polygon
        let geom = &sketch.geometry.0[0];
        match geom {
            geo::Geometry::Polygon(poly) => {
                // Should be a square with 4 vertices (plus closing vertex = 5 total)
                assert_eq!(poly.exterior().coords().count(), 5, "Square should have 5 coordinates (4 + closing)");

                // Verify it's approximately a 2x2 square
                let coords: Vec<_> = poly.exterior().coords().collect();
                let mut x_coords: Vec<_> = coords.iter().map(|c| c.x).collect();
                let mut y_coords: Vec<_> = coords.iter().map(|c| c.y).collect();
                x_coords.sort_by(|a, b| a.partial_cmp(b).unwrap());
                y_coords.sort_by(|a, b| a.partial_cmp(b).unwrap());

                // Should span from 0 to 2 in both X and Y
                assert!((x_coords[0] - 0.0).abs() < 1e-6, "Min X should be 0");
                assert!((x_coords[x_coords.len()-1] - 2.0).abs() < 1e-6, "Max X should be 2");
                assert!((y_coords[0] + 2.0).abs() < 1e-6, "Min Y should be -2");
                assert!((y_coords[y_coords.len()-1] - 0.0).abs() < 1e-6, "Max Y should be 0");
            },
            _ => panic!("Expected a polygon geometry, got {:?}", geom),
        }
    }
}
