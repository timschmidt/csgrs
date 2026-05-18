//! Conservative LOD selection over exact voxel addresses.
//!
//! A Hyper LOD cell is not an averaged material value. It is a coarser exact
//! address plus conservative facts over the stored descendants that selected
//! it. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997: preserve object-level combinatorial
//! structure and expose what has actually been proved.

use std::collections::BTreeMap;

use crate::{
    HypervoxelError, HypervoxelResult, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts,
    VoxelCell,
};

/// One selected LOD cell and its conservative aggregate.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LodCellSelection {
    /// Coarse selected address.
    pub address: VoxelAddress,
    /// Aggregate facts over stored descendants represented by this address.
    pub aggregate: VoxelAggregateFacts,
}

/// Deterministic LOD selection report.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LodSelectionReport {
    /// Requested target depth.
    pub target_depth: u8,
    /// Selected cells in deterministic address order.
    pub cells: Vec<LodCellSelection>,
}

/// Selects conservative LOD cells by grouping stored cells under ancestors.
pub fn select_lod_cells(
    grid: &SparseVoxelGrid,
    target_depth: u8,
) -> HypervoxelResult<LodSelectionReport> {
    if target_depth > grid.frame().depth() {
        return Err(HypervoxelError::DepthOutsideFrame {
            depth: target_depth,
            frame_depth: grid.frame().depth(),
        });
    }

    let mut groups = BTreeMap::<VoxelAddress, Vec<VoxelCell>>::new();
    for (address, cell) in grid.iter() {
        let ancestor = ancestor_at_depth(*address, target_depth)?;
        groups.entry(ancestor).or_default().push(*cell);
    }

    Ok(LodSelectionReport {
        target_depth,
        cells: groups
            .into_iter()
            .map(|(address, cells)| LodCellSelection {
                address,
                aggregate: VoxelAggregateFacts::from_cells(cells.iter()),
            })
            .collect(),
    })
}

fn ancestor_at_depth(address: VoxelAddress, target_depth: u8) -> HypervoxelResult<VoxelAddress> {
    if target_depth > address.depth {
        return Err(HypervoxelError::DepthOutsideFrame {
            depth: target_depth,
            frame_depth: address.depth,
        });
    }
    let shift = address.depth - target_depth;
    VoxelAddress::new(
        target_depth,
        [
            address.xyz[0] >> shift,
            address.xyz[1] >> shift,
            address.xyz[2] >> shift,
        ],
    )
}
