//! Minimal exact sparse-grid storage wrapper.
//!
//! This is intentionally a semantic storage wrapper, not a full SVO-DAG port.
//! It gives the first `hypervoxel` APIs a tested exact contract while the
//! harvested `voxelis` interner/tree implementation is ported underneath.

use std::collections::BTreeMap;

use crate::{
    GridFrame, HypervoxelResult, OccupancyState, VoxelAddress, VoxelAggregateFacts, VoxelCell,
};

/// Report returned by a voxel edit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelEditReport {
    /// Edited address.
    pub address: VoxelAddress,
    /// Previous cell, if present.
    pub previous: Option<VoxelCell>,
    /// New cell.
    pub current: VoxelCell,
}

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
    pub fn set(
        &mut self,
        address: VoxelAddress,
        cell: VoxelCell,
    ) -> HypervoxelResult<VoxelEditReport> {
        if address.depth > self.frame.depth() {
            return Err(crate::HypervoxelError::DepthOutsideFrame {
                depth: address.depth,
                frame_depth: self.frame.depth(),
            });
        }
        let previous = if cell.occupancy == OccupancyState::Empty {
            self.cells.remove(&address)
        } else {
            self.cells.insert(address, cell)
        };
        Ok(VoxelEditReport {
            address,
            previous,
            current: cell,
        })
    }

    /// Returns aggregate facts over explicitly stored cells.
    ///
    /// Empty absent cells are not expanded; callers that need whole-grid facts
    /// should query the relevant subtree once the SVO-DAG backend lands.
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
