//! Page-backed exact voxel-surface triangle mesh construction.

use crate::{
    ChunkPagedSparseGrid, ExactVoxelSurfaceTriangleMesh, HypervoxelResult,
    exact_voxel_surface_triangle_mesh_from_faces, extract_chunk_paged_exposed_faces,
};

/// Builds an exact indexed triangle mesh from chunk-paged sparse storage.
pub fn chunk_paged_exact_surface_triangle_mesh(
    grid: &ChunkPagedSparseGrid,
) -> HypervoxelResult<ExactVoxelSurfaceTriangleMesh> {
    let faces = extract_chunk_paged_exposed_faces(grid)?;
    exact_voxel_surface_triangle_mesh_from_faces(&faces)
}
