//! Binary Space Partitioning (BSP) tree implementation
//!
//! This module provides BSP tree operations with dependency inversion,
//! allowing for different algorithm implementations (serial/parallel).

pub mod node;
pub mod traits;

#[cfg(not(feature = "parallel"))]
pub mod serial;

#[cfg(feature = "parallel")]
pub mod parallel;

// Re-export core types for backward compatibility
pub use node::Node;
pub use traits::{BalancedSplittingStrategy, BspOps, SplittingPlaneStrategy};

#[cfg(not(feature = "parallel"))]
pub use serial::SerialBspOps;

#[cfg(feature = "parallel")]
pub use parallel::ParallelBspOps;

// Backward compatibility implementations on Node
use crate::mesh::plane::Plane;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use std::fmt::Debug;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Creates a new BSP node from polygons
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        let mut node = Self::new();
        if !polygons.is_empty() {
            node.build(polygons);
        }
        node
    }

    /// Invert all polygons in the BSP tree
    #[cfg(not(feature = "parallel"))]
    pub fn invert(&mut self) {
        let ops = SerialBspOps::new();
        ops.invert(self);
    }

    #[cfg(feature = "parallel")]
    pub fn invert(&mut self) {
        let ops = ParallelBspOps::new();
        ops.invert(self);
    }

    /// Pick the best splitting plane using the default strategy
    pub fn pick_best_splitting_plane(&self, polygons: &[Polygon<S>]) -> Plane {
        let strategy = BalancedSplittingStrategy::default();
        strategy.pick_best_splitting_plane(polygons)
    }

    /// Recursively remove all polygons that are inside this BSP tree
    #[cfg(not(feature = "parallel"))]
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        let ops = SerialBspOps::new();
        ops.clip_polygons(self, polygons)
    }

    #[cfg(feature = "parallel")]
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        let ops = ParallelBspOps::new();
        ops.clip_polygons(self, polygons)
    }

    /// Remove all polygons in this BSP tree that are inside the other BSP tree
    #[cfg(not(feature = "parallel"))]
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        let ops = SerialBspOps::new();
        ops.clip_to(self, bsp);
    }

    #[cfg(feature = "parallel")]
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        let ops = ParallelBspOps::new();
        ops.clip_to(self, bsp);
    }

    /// Return all polygons in this BSP tree
    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        #[cfg(not(feature = "parallel"))]
        let ops = SerialBspOps::new();
        #[cfg(feature = "parallel")]
        let ops = ParallelBspOps::new();

        ops.all_polygons(self)
    }

    /// Build a BSP tree from the given polygons
    #[cfg(not(feature = "parallel"))]
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        let ops = SerialBspOps::new();
        ops.build(self, polygons);
    }

    #[cfg(feature = "parallel")]
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        let ops = ParallelBspOps::new();
        ops.build(self, polygons);
    }

    /// Slices this BSP node with the given plane
    #[cfg(not(feature = "parallel"))]
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let ops = SerialBspOps::new();
        ops.slice(self, slicing_plane)
    }

    #[cfg(feature = "parallel")]
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let ops = ParallelBspOps::new();
        ops.slice(self, slicing_plane)
    }
}
