//! Minimal exact sparse-grid storage wrapper.
//!
//! This is intentionally the simple semantic map backend; the SVO-DAG backend
//! lives in `svo` behind the same exact cell/address contract.

use std::collections::BTreeMap;

use crate::{
    GridFrame, HypervoxelResult, OccupancyState, VoxelAddress, VoxelAggregateFacts, VoxelCell,
};

/// Sparse semantic grid over exact voxel addresses.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseVoxelGrid {
    frame: GridFrame,
    cells: BTreeMap<VoxelAddress, VoxelCell>,
}

impl SparseVoxelGrid {
    /// Creates an empty grid.
    pub fn new(frame: GridFrame) -> Self {
        Self {
            frame,
            cells: BTreeMap::new(),
        }
    }

    /// Returns the grid frame.
    pub fn frame(&self) -> &GridFrame {
        &self.frame
    }

    /// Returns a cell, defaulting to exact empty when absent.
    pub fn get(&self, address: VoxelAddress) -> HypervoxelResult<VoxelCell> {
        if address.depth > self.frame.depth() {
            return Err(crate::HypervoxelError::DepthOutsideFrame {
                depth: address.depth,
                frame_depth: self.frame.depth(),
            });
        }
        Ok(*self.cells.get(&address).unwrap_or(&VoxelCell::empty()))
    }

    /// Sets a cell after validating the address belongs to the frame.
    pub fn set(&mut self, address: VoxelAddress, cell: VoxelCell) -> HypervoxelResult<()> {
        if address.depth > self.frame.depth() {
            return Err(crate::HypervoxelError::DepthOutsideFrame {
                depth: address.depth,
                frame_depth: self.frame.depth(),
            });
        }
        if cell.occupancy == OccupancyState::Empty {
            self.cells.remove(&address)
        } else {
            self.cells.insert(address, cell)
        };
        Ok(())
    }

    /// Returns aggregate facts over explicitly stored cells.
    ///
    /// Empty absent cells are not expanded; callers that need whole-grid facts
    /// should use finite-frame aggregate reports or an SVO-DAG aggregate.
    pub fn stored_aggregate(&self) -> VoxelAggregateFacts {
        VoxelAggregateFacts::from_cells(self.cells.values())
    }

    /// Iterates over explicitly stored non-empty cells in deterministic order.
    pub fn iter(&self) -> impl Iterator<Item = (&VoxelAddress, &VoxelCell)> {
        self.cells.iter()
    }

    /// Returns the number of explicitly stored non-empty cells.
    pub fn len(&self) -> usize {
        self.cells.len()
    }

    /// Returns whether no non-empty cells are stored.
    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }
}
