//! Traits for mesh smoothing operations.

use crate::float_types::Real;
use crate::mesh::Mesh;
use std::fmt::Debug;

/// Trait for mesh smoothing and refinement operations.
pub trait SmoothingOps<S: Clone + Debug + Send + Sync> {
    /// Applies Laplacian smoothing to the mesh.
    fn laplacian_smooth(
        &self,
        mesh: &Mesh<S>,
        lambda: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Mesh<S>;

    /// Applies Taubin smoothing to the mesh.
    fn taubin_smooth(
        &self,
        mesh: &Mesh<S>,
        lambda: Real,
        mu: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Mesh<S>;

    /// Adaptively refines the mesh based on geometric criteria.
    fn adaptive_refine(
        &self,
        mesh: &Mesh<S>,
        quality_threshold: Real,
        max_edge_length: Real,
        curvature_threshold_deg: Real,
    ) -> Mesh<S>;

    /// Removes poor-quality triangles from the mesh.
    fn remove_poor_triangles(&self, mesh: &Mesh<S>, min_quality: Real) -> Mesh<S>;
}
