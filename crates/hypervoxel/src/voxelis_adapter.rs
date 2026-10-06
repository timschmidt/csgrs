//! Feature-gated conversion from legacy `voxelis` storage.
//!
//! The adapter maps legacy leaf values into Hyper's integer-addressed storage.
//! It does not retain adapter lineage or manufacture source-geometry evidence.

use glam::IVec3;
use voxelis::{
    Lod, VoxInterner,
    spatial::{VoxOpsConfig, VoxOpsRead, VoxTree},
};

use crate::{
    ChunkPagedSparseGrid, ChunkShape, ExactVoxelSurfaceTriangleMesh, GridFrame, HypervoxelError,
    HypervoxelResult, MaterialRegionId, SparseVoxelGrid, VoxelAddress, VoxelCell,
    chunk_paged_exact_surface_triangle_mesh,
};

/// Materializes a legacy `voxelis::VoxTree<u8>` into Hyper chunk pages.
///
/// Zero values remain implicit empty cells; nonzero values become material
/// region identifiers with the same numeric value.
pub fn materialize_legacy_voxelis_u8_chunk_paged_storage(
    tree: &VoxTree<u8>,
    interner: &VoxInterner<u8>,
    frame: GridFrame,
    shape: ChunkShape,
) -> HypervoxelResult<ChunkPagedSparseGrid> {
    let frame_depth = frame.depth();
    if tree.max_depth(Lod::new(0)).max() != frame_depth {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "legacy voxel tree depth does not match the target frame",
        });
    }

    let cells_per_axis = checked_cells_per_axis(frame_depth)?;
    let mut sparse = SparseVoxelGrid::new(frame);
    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                let value = tree
                    .get(interner, ivec3_from_xyz([x, y, z])?)
                    .unwrap_or_default();
                if value != 0 {
                    sparse.set(
                        VoxelAddress::new(frame_depth, [x, y, z])?,
                        VoxelCell::material(MaterialRegionId(u32::from(value))),
                    )?;
                }
            }
        }
    }

    ChunkPagedSparseGrid::from_sparse_grid(&sparse, shape)
}

/// Materializes legacy storage and builds its exact exposed triangle surface.
pub fn materialize_legacy_voxelis_u8_exact_surface_triangle_mesh(
    tree: &VoxTree<u8>,
    interner: &VoxInterner<u8>,
    frame: GridFrame,
    shape: ChunkShape,
) -> HypervoxelResult<(ChunkPagedSparseGrid, ExactVoxelSurfaceTriangleMesh)> {
    let paged = materialize_legacy_voxelis_u8_chunk_paged_storage(tree, interner, frame, shape)?;
    let surface = chunk_paged_exact_surface_triangle_mesh(&paged)?;
    Ok((paged, surface))
}

fn checked_cells_per_axis(depth: u8) -> HypervoxelResult<u64> {
    1_u64
        .checked_shl(u32::from(depth))
        .ok_or(HypervoxelError::AddressOverflow)
}

fn ivec3_from_xyz(xyz: [u64; 3]) -> HypervoxelResult<IVec3> {
    Ok(IVec3::new(
        i32::try_from(xyz[0]).map_err(|_| HypervoxelError::AddressOverflow)?,
        i32::try_from(xyz[1]).map_err(|_| HypervoxelError::AddressOverflow)?,
        i32::try_from(xyz[2]).map_err(|_| HypervoxelError::AddressOverflow)?,
    ))
}
