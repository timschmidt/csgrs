//! Deterministic sparse-grid edit batches.
//!
//! `voxelis` uses batched edits for performance. This semantic layer keeps that
//! storage idea, but every edit is still
//! validated against the exact grid frame.

use crate::{HypervoxelResult, SparseVoxelGrid, VoxelAddress, VoxelCell};

/// One pending voxel edit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoxelEdit {
    /// Address to edit.
    pub address: VoxelAddress,
    /// Cell to store. Empty cells remove explicit storage.
    pub cell: VoxelCell,
}

/// Deterministic batch of sparse voxel edits.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct VoxelEditBatch {
    edits: Vec<VoxelEdit>,
}

impl VoxelEditBatch {
    /// Creates an empty edit batch.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds one edit to the end of the batch.
    pub fn push(&mut self, address: VoxelAddress, cell: VoxelCell) {
        self.edits.push(VoxelEdit { address, cell });
    }

    /// Returns the number of queued edits.
    pub fn len(&self) -> usize {
        self.edits.len()
    }

    /// Returns whether the batch has no queued edits.
    pub fn is_empty(&self) -> bool {
        self.edits.is_empty()
    }

    /// Iterates queued edits in application order.
    pub fn iter(&self) -> impl Iterator<Item = &VoxelEdit> {
        self.edits.iter()
    }

    /// Applies edits in insertion order.
    pub fn apply_to(&self, grid: &mut SparseVoxelGrid) -> HypervoxelResult<()> {
        for edit in &self.edits {
            grid.set(edit.address, edit.cell)?;
        }
        Ok(())
    }
}
