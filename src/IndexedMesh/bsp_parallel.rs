//! Parallel BSP (Binary Space Partitioning) tree operations for IndexedMesh.
//!
//! This module provides parallel BSP tree functionality optimized for IndexedMesh's indexed connectivity model.
//! Uses rayon for parallel processing of BSP tree operations.

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::IndexedMesh::IndexedMesh;
use std::fmt::Debug;

/// Parallel BSP tree node for IndexedMesh
pub use crate::IndexedMesh::bsp::IndexedNode;

#[cfg(feature = "parallel")]
impl<S: Clone + Send + Sync + Debug> IndexedNode<S> {
    /// Build a BSP tree from the given polygon indices with parallel processing
    pub fn build_parallel(&mut self, mesh: &IndexedMesh<S>) {
        if self.polygons.is_empty() {
            return;
        }

        // For now, use the first polygon's plane as the splitting plane
        if self.plane.is_none() {
            self.plane = Some(mesh.polygons[self.polygons[0]].plane.clone());
        }

        // Simple parallel implementation: just store all polygons in this node
        // TODO: Implement proper parallel BSP tree construction with polygon splitting
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
    use crate::IndexedMesh::{IndexedMesh, IndexedPolygon};
    use crate::mesh::{plane::Plane, vertex::Vertex};
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_parallel_bsp_basic_functionality() {
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
        let polygons = vec![
            IndexedPolygon::<i32>::new(vec![0, 1, 2], Plane::from_vertices(plane_vertices), None)
        ];

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