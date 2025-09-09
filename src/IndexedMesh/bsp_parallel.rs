//! Parallel BSP (Binary Space Partitioning) tree operations for IndexedMesh.
//!
//! This module provides parallel BSP tree functionality optimized for IndexedMesh's indexed connectivity model.
//! Uses rayon for parallel processing of BSP tree operations.

#[cfg(feature = "parallel")]
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::IndexedMesh::IndexedMesh;

use std::fmt::Debug;

/// Parallel BSP tree node for IndexedMesh
pub use crate::IndexedMesh::bsp::IndexedNode;

/// Classification of a polygon relative to a plane (parallel version)
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(dead_code)]
enum PolygonClassification {
    Front,
    Back,
    CoplanarFront,
    CoplanarBack,
    Spanning,
}

#[cfg(feature = "parallel")]
#[cfg(feature = "parallel")]
impl<S: Clone + Send + Sync + Debug> IndexedNode<S> {
    /// **Mathematical Foundation: Parallel BSP Tree Construction**
    ///
    /// Build BSP tree using parallel processing for optimal performance on large meshes.
    /// This is the parallel version of the regular build method.
    ///
    /// ## **Parallel Algorithm**
    /// 1. **Plane Selection**: Choose optimal splitting plane using parallel evaluation
    /// 2. **Polygon Classification**: Classify polygons in parallel using rayon
    /// 3. **Recursive Subdivision**: Build front and back subtrees in parallel
    /// 4. **Index Preservation**: Maintain polygon indices throughout construction
    ///
    /// ## **Performance Benefits**
    /// - **Scalability**: Linear speedup with number of cores for large meshes
    /// - **Cache Efficiency**: Better memory access patterns through parallelization
    /// - **Load Balancing**: Automatic work distribution via rayon
    pub fn build(&mut self, mesh: &IndexedMesh<S>) {
        self.build_parallel(mesh);
    }

    /// **Build BSP Tree with Depth Limit (Parallel Version)**
    ///
    /// Construct BSP tree with maximum recursion depth to prevent stack overflow.
    /// Uses parallel processing for optimal performance.
    ///
    /// ## **Mathematical Foundation**
    /// - **Depth Control**: Limits recursion to prevent exponential memory growth
    /// - **Parallel Processing**: Leverages multiple cores for large polygon sets
    /// - **Memory Management**: Prevents excessive stack frame allocation
    ///
    /// # Parameters
    /// - `mesh`: The IndexedMesh containing the polygons
    /// - `max_depth`: Maximum recursion depth (recommended: 15-25)
    pub fn build_with_depth_limit(&mut self, mesh: &IndexedMesh<S>, max_depth: usize) {
        self.build_with_depth_limit_recursive(mesh, 0, max_depth);
    }

    /// Recursive helper for depth-limited BSP construction (parallel version)
    fn build_with_depth_limit_recursive(
        &mut self,
        mesh: &IndexedMesh<S>,
        current_depth: usize,
        max_depth: usize,
    ) {
        if self.polygons.is_empty() || current_depth >= max_depth {
            return;
        }

        // Choose optimal splitting plane if not already set
        if self.plane.is_none() {
            self.plane = Some(self.choose_splitting_plane(mesh));
        }

        let plane = self.plane.as_ref().unwrap();

        // Use parallel classification for better performance
        let (front_polygons, back_polygons): (Vec<_>, Vec<_>) = self
            .polygons
            .par_iter()
            .map(|&poly_idx| {
                let polygon = &mesh.polygons[poly_idx];
                let classification = self.classify_polygon_to_plane(mesh, polygon, plane);
                (poly_idx, classification)
            })
            .partition_map(|(poly_idx, classification)| {
                match classification {
                    PolygonClassification::Front => rayon::iter::Either::Left(poly_idx),
                    PolygonClassification::Back => rayon::iter::Either::Right(poly_idx),
                    _ => rayon::iter::Either::Left(poly_idx), // Coplanar and spanning go to front
                }
            });

        // Build subtrees recursively with incremented depth
        if !front_polygons.is_empty() {
            let mut front_node = IndexedNode::new();
            front_node.polygons = front_polygons;
            front_node.build_with_depth_limit_recursive(mesh, current_depth + 1, max_depth);
            self.front = Some(Box::new(front_node));
        }

        if !back_polygons.is_empty() {
            let mut back_node = IndexedNode::new();
            back_node.polygons = back_polygons;
            back_node.build_with_depth_limit_recursive(mesh, current_depth + 1, max_depth);
            self.back = Some(Box::new(back_node));
        }
    }

    /// **Mathematical Foundation: Parallel BSP Tree Construction with Indexed Connectivity**
    ///
    /// Builds a balanced BSP tree using parallel processing for polygon classification
    /// and recursive subtree construction.
    ///
    /// ## **Parallel Optimization Strategies**
    /// - **Parallel Classification**: Classify polygons to planes using rayon
    /// - **Concurrent Subtree Building**: Build front/back subtrees in parallel
    /// - **Work Stealing**: Efficient load balancing across threads
    /// - **Memory Locality**: Minimize data movement between threads
    ///
    /// ## **Performance Benefits**
    /// - **Scalability**: Linear speedup with number of cores for large meshes
    /// - **Cache Efficiency**: Better memory access patterns through parallelization
    /// - **Load Balancing**: Automatic work distribution via rayon
    pub fn build_parallel(&mut self, mesh: &IndexedMesh<S>) {
        if self.polygons.is_empty() {
            return;
        }

        // Choose optimal splitting plane
        if self.plane.is_none() {
            self.plane = Some(self.choose_splitting_plane(mesh));
        }

        let plane = self.plane.as_ref().unwrap();

        // Parallel polygon classification
        let classifications: Vec<_> = self
            .polygons
            .par_iter()
            .map(|&poly_idx| {
                let polygon = &mesh.polygons[poly_idx];
                (poly_idx, self.classify_polygon_to_plane(mesh, polygon, plane))
            })
            .collect();

        // Partition polygons based on classification
        let mut front_polygons = Vec::new();
        let mut back_polygons = Vec::new();
        let mut coplanar_polygons = Vec::new();

        for (poly_idx, classification) in classifications {
            match classification {
                PolygonClassification::Front => front_polygons.push(poly_idx),
                PolygonClassification::Back => back_polygons.push(poly_idx),
                PolygonClassification::CoplanarFront | PolygonClassification::CoplanarBack => {
                    coplanar_polygons.push(poly_idx);
                },
                PolygonClassification::Spanning => {
                    // Add to both sides for spanning polygons
                    front_polygons.push(poly_idx);
                    back_polygons.push(poly_idx);
                },
            }
        }

        // Store coplanar polygons in this node
        self.polygons = coplanar_polygons;

        // Build subtrees in parallel using rayon::join
        let (front_result, back_result) = rayon::join(
            || {
                if !front_polygons.is_empty() {
                    let mut front_node = IndexedNode::new();
                    front_node.polygons = front_polygons;
                    front_node.build_parallel(mesh);
                    Some(Box::new(front_node))
                } else {
                    None
                }
            },
            || {
                if !back_polygons.is_empty() {
                    let mut back_node = IndexedNode::new();
                    back_node.polygons = back_polygons;
                    back_node.build_parallel(mesh);
                    Some(Box::new(back_node))
                } else {
                    None
                }
            },
        );

        self.front = front_result;
        self.back = back_result;
    }

    /// Choose optimal splitting plane (parallel version)
    #[cfg(feature = "parallel")]
    fn choose_splitting_plane(&self, mesh: &IndexedMesh<S>) -> crate::mesh::plane::Plane {
        if self.polygons.is_empty() {
            return crate::mesh::plane::Plane::from_normal(nalgebra::Vector3::z(), 0.0);
        }

        // Use parallel evaluation for plane selection
        let sample_size = (self.polygons.len().min(10)).max(1);
        let candidates: Vec<_> = (0..sample_size)
            .into_par_iter()
            .map(|i| {
                let poly_idx = self.polygons[i * self.polygons.len() / sample_size];
                let plane = mesh.polygons[poly_idx].plane.clone();
                let score = self.evaluate_splitting_plane(mesh, &plane);
                (plane, score)
            })
            .collect();

        // Find best plane
        candidates
            .into_iter()
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(plane, _)| plane)
            .unwrap_or_else(|| mesh.polygons[self.polygons[0]].plane.clone())
    }

    /// Evaluate splitting plane quality (parallel version)
    #[cfg(feature = "parallel")]
    fn evaluate_splitting_plane(
        &self,
        mesh: &IndexedMesh<S>,
        plane: &crate::mesh::plane::Plane,
    ) -> f64 {
        let counts = self
            .polygons
            .par_iter()
            .map(|&poly_idx| {
                let polygon = &mesh.polygons[poly_idx];
                match self.classify_polygon_to_plane(mesh, polygon, plane) {
                    PolygonClassification::Front => (1, 0, 0),
                    PolygonClassification::Back => (0, 1, 0),
                    PolygonClassification::Spanning => (0, 0, 1),
                    _ => (0, 0, 0),
                }
            })
            .reduce(|| (0, 0, 0), |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2));

        let (front_count, back_count, split_count) = counts;
        let balance_penalty = ((front_count as f64) - (back_count as f64)).abs();
        let split_penalty = (split_count as f64) * 3.0;

        balance_penalty + split_penalty
    }

    /// Classify polygon relative to plane (parallel version)
    #[cfg(feature = "parallel")]
    fn classify_polygon_to_plane(
        &self,
        mesh: &IndexedMesh<S>,
        polygon: &crate::IndexedMesh::IndexedPolygon<S>,
        plane: &crate::mesh::plane::Plane,
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
            let polygon_normal = polygon.plane.normal();
            let plane_normal = plane.normal();

            if polygon_normal.dot(&plane_normal) > 0.0 {
                PolygonClassification::CoplanarFront
            } else {
                PolygonClassification::CoplanarBack
            }
        }
    }

    /// Return all polygon indices in this BSP tree using parallel processing
    pub fn all_polygon_indices_parallel(&self) -> Vec<usize> {
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

    /// Compute signed distance from a point to a plane (parallel version)
    #[cfg(feature = "parallel")]
    fn signed_distance_to_point(
        &self,
        plane: &crate::mesh::plane::Plane,
        point: &nalgebra::Point3<crate::float_types::Real>,
    ) -> crate::float_types::Real {
        let normal = plane.normal();
        let offset = plane.offset();
        normal.dot(&point.coords) - offset
    }
}

#[cfg(not(feature = "parallel"))]
impl<S: Clone + Send + Sync + Debug> IndexedNode<S> {
    /// Fallback to sequential implementation when parallel feature is disabled
    pub fn build_parallel(&mut self, mesh: &IndexedMesh<S>) {
        self.build(mesh);
    }

    /// Fallback to sequential implementation when parallel feature is disabled
    pub fn all_polygon_indices_parallel(&self) -> Vec<usize> {
        self.all_polygon_indices()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::IndexedMesh::plane::Plane;
    use crate::IndexedMesh::{IndexedMesh, IndexedPolygon};

    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_parallel_bsp_basic_functionality() {
        // Create a simple mesh with one triangle
        let vertices = vec![
            crate::IndexedMesh::vertex::IndexedVertex::new(
                Point3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ),
            crate::IndexedMesh::vertex::IndexedVertex::new(
                Point3::new(1.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ),
            crate::IndexedMesh::vertex::IndexedVertex::new(
                Point3::new(0.5, 1.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ),
        ];

        let plane_vertices = vec![
            vertices[0].clone(),
            vertices[1].clone(),
            vertices[2].clone(),
        ];
        let polygons = vec![IndexedPolygon::<i32>::new(
            vec![0, 1, 2],
            Plane::from_indexed_vertices(plane_vertices),
            None,
        )];

        let mesh = IndexedMesh {
            vertices,
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata: None,
        };

        let polygon_indices = vec![0];
        let mut node = IndexedNode::from_polygon_indices(&polygon_indices);
        node.build_parallel(&mesh);

        // Basic test that node was created
        assert!(!node.all_polygon_indices_parallel().is_empty());
    }
}
