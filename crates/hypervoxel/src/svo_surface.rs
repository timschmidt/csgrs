//! Exact surfaces from Hyper-owned SVO-DAG storage.

use crate::{
    ExactVoxelFace, ExactVoxelSurfaceTriangleMesh, HypervoxelResult, SvoVoxelGrid,
    exact_voxel_surface_triangle_mesh_from_faces, extract_exposed_faces,
};

/// Expands an SVO-DAG and extracts its exact exposed faces.
pub fn extract_svo_exposed_faces(grid: &SvoVoxelGrid) -> HypervoxelResult<Vec<ExactVoxelFace>> {
    extract_exposed_faces(&grid.to_sparse_grid()?)
}

/// Builds an exact indexed surface triangle mesh from an SVO-DAG.
pub fn svo_exact_surface_triangle_mesh(
    grid: &SvoVoxelGrid,
) -> HypervoxelResult<ExactVoxelSurfaceTriangleMesh> {
    let faces = extract_svo_exposed_faces(grid)?;
    exact_voxel_surface_triangle_mesh_from_faces(&faces)
}
