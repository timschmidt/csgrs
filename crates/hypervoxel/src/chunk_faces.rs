//! Exact exposed-face extraction over chunk-paged sparse storage.

use crate::{
    ChunkPagedSparseGrid, ExactVoxelFace, GreedyFacePatch, HypervoxelError, HypervoxelResult,
    OccupancyState, VoxelAddress, VoxelFaceSide, greedy_face_patches,
};

/// Extracts exact exposed faces from a chunk-paged sparse grid.
pub fn extract_chunk_paged_exposed_faces(
    grid: &ChunkPagedSparseGrid,
) -> HypervoxelResult<Vec<ExactVoxelFace>> {
    let mut faces = Vec::new();
    for (_, page) in grid.pages() {
        for (address, cell) in page.iter() {
            if cell.occupancy == OccupancyState::Empty {
                continue;
            }
            if matches!(
                cell.occupancy,
                OccupancyState::Unknown | OccupancyState::LossyAdapterValue
            ) {
                return Err(HypervoxelError::InvalidSourceGeometry {
                    reason: "cannot extract an exact shell from uncertain chunk cells",
                });
            }

            let cell_bounds = address.bounds(grid.frame())?;
            for side in FACE_SIDES {
                let exposed = match neighbor_address(*address, side) {
                    None => true,
                    Some(neighbor) => match grid.get(neighbor)?.occupancy {
                        OccupancyState::Empty => true,
                        OccupancyState::Unknown | OccupancyState::LossyAdapterValue => {
                            return Err(HypervoxelError::InvalidSourceGeometry {
                                reason: "chunk shell exposure depends on an uncertain neighbor",
                            });
                        }
                        _ => false,
                    },
                };
                if exposed {
                    faces.push(ExactVoxelFace {
                        address: *address,
                        side,
                        cell_bounds: cell_bounds.clone(),
                    });
                }
            }
        }
    }
    Ok(faces)
}

/// Returns deterministic greedy patches over a chunk-paged grid shell.
pub fn chunk_paged_greedy_face_patches(
    grid: &ChunkPagedSparseGrid,
) -> HypervoxelResult<Vec<GreedyFacePatch>> {
    let faces = extract_chunk_paged_exposed_faces(grid)?;
    Ok(greedy_face_patches(&faces))
}

const FACE_SIDES: [VoxelFaceSide; 6] = [
    VoxelFaceSide::XNeg,
    VoxelFaceSide::XPos,
    VoxelFaceSide::YNeg,
    VoxelFaceSide::YPos,
    VoxelFaceSide::ZNeg,
    VoxelFaceSide::ZPos,
];

fn neighbor_address(address: VoxelAddress, side: VoxelFaceSide) -> Option<VoxelAddress> {
    let cells = 1_u64 << address.depth;
    let offset = side.integer_normal();
    let mut xyz = address.xyz;
    for axis in 0..3 {
        match offset[axis] {
            -1 if xyz[axis] == 0 => return None,
            -1 => xyz[axis] -= 1,
            1 if xyz[axis] + 1 >= cells => return None,
            1 => xyz[axis] += 1,
            _ => {}
        }
    }
    VoxelAddress::new(address.depth, xyz).ok()
}
