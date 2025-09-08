//! BSP (Binary Space Partitioning) tree operations for IndexedMesh.
//!
//! This module provides BSP tree functionality optimized for IndexedMesh's indexed connectivity model.
//! BSP trees are used for efficient spatial partitioning and CSG operations.

use crate::mesh::plane::Plane;
use crate::IndexedMesh::IndexedMesh;
use std::fmt::Debug;
use std::marker::PhantomData;

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
    pub fn new() -> Self {
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

    /// Build a BSP tree from the given polygon indices with access to the mesh
    pub fn build(&mut self, mesh: &IndexedMesh<S>) {
        if self.polygons.is_empty() {
            return;
        }

        // For now, use the first polygon's plane as the splitting plane
        if self.plane.is_none() {
            self.plane = Some(mesh.polygons[self.polygons[0]].plane.clone());
        }

        // Simple implementation: just store all polygons in this node
        // TODO: Implement proper BSP tree construction with polygon splitting
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
        let polygons = vec![
            IndexedPolygon::<i32>::new(vec![0, 1, 2], Plane::from_vertices(plane_vertices), None)
        ];

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
}