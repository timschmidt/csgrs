//! Continuous-field voxel intake.
//!
//! Continuous implicit/SDF fields are owned by crates such as `hypersdf`;
//! `hypervoxel` owns grid frames, cell payloads, aggregate facts, and storage.
//! This module accepts already-classified cells without retaining producer
//! lineage or constructing audit reports.

use std::collections::BTreeSet;

use crate::{
    GridFrame, HypervoxelError, HypervoxelResult, OccupancyState, PreparedVoxelGrid,
    SparseVoxelGrid, VoxelAddress, VoxelCell,
};

/// One externally classified continuous-field cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContinuousFieldVoxelCell {
    /// Exact voxel address for this classified cell.
    pub address: VoxelAddress,
    /// Conservative cell payload supplied by the continuous-field owner.
    pub cell: VoxelCell,
}

impl ContinuousFieldVoxelCell {
    /// Constructs one externally classified cell.
    pub const fn new(address: VoxelAddress, cell: VoxelCell) -> Self {
        Self { address, cell }
    }
}

/// A frame and externally classified continuous-field cells ready for intake.
#[derive(Clone, Debug, PartialEq)]
pub struct ContinuousFieldVoxelBatch {
    /// Exact target grid frame.
    pub frame: GridFrame,
    /// Explicit classified cells.
    pub cells: Vec<ContinuousFieldVoxelCell>,
}

impl ContinuousFieldVoxelBatch {
    /// Materializes supplied cells without imposing a dense-cover requirement.
    pub fn materialize_sparse_grid(&self) -> HypervoxelResult<PreparedVoxelGrid<SparseVoxelGrid>> {
        let mut grid = SparseVoxelGrid::new(self.frame.clone());
        for row in &self.cells {
            grid.set(row.address, row.cell)?;
        }
        let aggregate = grid.stored_aggregate();
        Ok(PreparedVoxelGrid::new(self.frame.clone(), grid, aggregate))
    }

    /// Materializes only a complete, unique, exact finest-depth frame cover.
    pub fn materialize_exact_sparse_grid(
        &self,
    ) -> HypervoxelResult<PreparedVoxelGrid<SparseVoxelGrid>> {
        let expected = frame_cell_count(&self.frame).ok_or(
            HypervoxelError::InvalidContinuousFieldMaterialization {
                reason: "frame cell count exceeds addressable storage",
            },
        )?;
        if self.cells.len() != expected {
            return Err(HypervoxelError::InvalidContinuousFieldMaterialization {
                reason: "supplied cells do not cover the complete frame",
            });
        }

        let mut seen = BTreeSet::new();
        for row in &self.cells {
            if row.address.depth != self.frame.depth() {
                return Err(HypervoxelError::InvalidContinuousFieldMaterialization {
                    reason: "supplied cell is not at the frame depth",
                });
            }
            if !seen.insert(row.address) {
                return Err(HypervoxelError::InvalidContinuousFieldMaterialization {
                    reason: "supplied cells contain duplicate addresses",
                });
            }
            if matches!(
                row.cell.occupancy,
                OccupancyState::Unknown | OccupancyState::LossyAdapterValue
            ) {
                return Err(HypervoxelError::InvalidContinuousFieldMaterialization {
                    reason: "supplied cell contains unknown or lossy evidence",
                });
            }
        }

        self.materialize_sparse_grid()
    }
}

/// Builds a finest-depth address for a continuous-field intake cell.
pub fn continuous_field_address(
    frame: &GridFrame,
    xyz: [u64; 3],
) -> HypervoxelResult<VoxelAddress> {
    VoxelAddress::new(frame.depth(), xyz)
}

fn frame_cell_count(frame: &GridFrame) -> Option<usize> {
    let cells_per_axis = frame.cells_per_axis();
    cells_per_axis
        .checked_mul(cells_per_axis)
        .and_then(|area| area.checked_mul(cells_per_axis))
        .and_then(|volume| usize::try_from(volume).ok())
}
