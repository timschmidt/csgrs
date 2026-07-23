//! Sparse-grid exact surface triangle mesh construction.

use crate::{
    ExactVoxelSurfaceTriangleMesh, HypervoxelResult, SparseVoxelGrid,
    exact_voxel_surface_triangle_mesh_from_faces, extract_exposed_faces,
};

/// Builds an exact indexed surface triangle mesh from sparse storage.
pub fn sparse_exact_surface_triangle_mesh(
    grid: &SparseVoxelGrid,
) -> HypervoxelResult<ExactVoxelSurfaceTriangleMesh> {
    let faces = extract_exposed_faces(grid)?;
    exact_voxel_surface_triangle_mesh_from_faces(&faces)
}
