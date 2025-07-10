//! Traits for convex hull operations.

use crate::mesh::Mesh;
use std::fmt::Debug;

/// Trait for convex hull and related operations.
pub trait ConvexHullOps<S: Clone + Debug + Send + Sync> {
    /// Computes the convex hull of the mesh.
    fn convex_hull(&self, mesh: &Mesh<S>) -> Mesh<S>;

    /// Computes the Minkowski sum of two meshes.
    fn minkowski_sum(&self, mesh: &Mesh<S>, other: &Mesh<S>) -> Mesh<S>;
}
