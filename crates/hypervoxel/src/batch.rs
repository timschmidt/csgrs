//! Deterministic sparse-grid edit batches.
//!
//! `voxelis` uses batched edits for performance. The Hyper port keeps that
//! storage idea, but the public contract is semantic: every edit is still
//! validated against the exact grid frame, and the report preserves the
//! previous/current cell facts needed to replay or audit the mutation.

use crate::{HypervoxelResult, SparseVoxelGrid, VoxelAddress, VoxelCell, VoxelEditReport};

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

/// Summary for a validated edit batch.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelEditBatchReport {
    /// Per-edit reports in application order.
    pub edits: Vec<VoxelEditReport>,
    /// Number of queued edits that were applied.
    pub applied_edits: usize,
    /// Whether this report contains at least one applied edit.
    ///
    /// An empty batch is a valid no-op, but it is not replay evidence for an
    /// object mutation. Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997, treats exactness as a property of
    /// replayable object-level operations; this flag prevents an empty
    /// performance batch from being promoted as exact edit evidence.
    pub has_applied_edits: bool,
    /// Number of edits that stored non-empty explicit cells.
    pub stored_explicit_cells: usize,
    /// Number of edits that removed previously explicit cells.
    pub removed_explicit_cells: usize,
    /// Number of edits that left semantic storage unchanged.
    pub semantic_noops: usize,
    /// Number of applied current cells that are not exact-ready evidence.
    pub non_exact_current_cells: usize,
    /// Whether every edit was frame-validated and exact-ready.
    ///
    /// This is the batch-edit counterpart to Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997: a performance
    /// batch is exact only when each object-level mutation remains validated,
    /// semantically coherent, and replayable in order. Unknown/lossy edits are
    /// still valid storage mutations, but this report keeps them out of exact
    /// replay.
    pub exact_batch_replay_ready: bool,
}

impl VoxelEditBatchReport {
    /// Builds a batch summary from per-edit reports.
    pub fn from_edits(edits: Vec<VoxelEditReport>) -> Self {
        let applied_edits = edits.len();
        let has_applied_edits = applied_edits > 0;
        let stored_explicit_cells = edits
            .iter()
            .filter(|report| report.stored_explicit_cell)
            .count();
        let removed_explicit_cells = edits
            .iter()
            .filter(|report| report.removed_explicit_cell)
            .count();
        let semantic_noops = edits.iter().filter(|report| report.semantic_noop).count();
        let non_exact_current_cells = edits
            .iter()
            .filter(|report| !report.exact_edit_replay_ready)
            .count();
        let exact_batch_replay_ready = has_applied_edits
            && edits.iter().all(|report| report.frame_validated)
            && non_exact_current_cells == 0;
        Self {
            edits,
            applied_edits,
            has_applied_edits,
            stored_explicit_cells,
            removed_explicit_cells,
            semantic_noops,
            non_exact_current_cells,
            exact_batch_replay_ready,
        }
    }
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

    /// Applies edits in insertion order and returns per-edit reports.
    pub fn apply_to(&self, grid: &mut SparseVoxelGrid) -> HypervoxelResult<Vec<VoxelEditReport>> {
        self.edits
            .iter()
            .map(|edit| grid.set(edit.address, edit.cell))
            .collect()
    }

    /// Applies edits in insertion order and returns a replay summary.
    pub fn apply_with_report(
        &self,
        grid: &mut SparseVoxelGrid,
    ) -> HypervoxelResult<VoxelEditBatchReport> {
        self.apply_to(grid).map(VoxelEditBatchReport::from_edits)
    }
}
