//! Minimal exact sparse-grid storage wrapper.
//!
//! This is intentionally the simple semantic map backend; the SVO-DAG backend
//! lives in `svo` behind the same exact cell/address contract. As
//! Yap argues in "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997, exact systems should make object-level decisions
//! explicit; edit reports therefore expose whether storage was inserted,
//! removed, or left unchanged instead of making callers infer that from maps.

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
    /// Whether the address was validated against the grid frame.
    pub frame_validated: bool,
    /// Whether this edit stored a non-empty explicit cell.
    pub stored_explicit_cell: bool,
    /// Whether this edit removed a previously explicit cell.
    pub removed_explicit_cell: bool,
    /// Whether this edit left semantic storage unchanged.
    pub semantic_noop: bool,
    /// Whether this edit can be replayed as exact voxel-state evidence.
    ///
    /// Storage accepts unknown and lossy cells because those are valid Hyper
    /// evidence states. They are not exact replay states, though. This flag is
    /// the single-edit version of the batch readiness gate and follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997: a representation update cannot upgrade undecided or
    /// approximate object facts into exact facts.
    pub exact_edit_replay_ready: bool,
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
        let stored_explicit_cell = cell.occupancy != OccupancyState::Empty;
        let removed_explicit_cell = cell.occupancy == OccupancyState::Empty && previous.is_some();
        let semantic_noop = previous.unwrap_or_else(VoxelCell::empty) == cell;
        let exact_edit_replay_ready = cell.report().exact_cell_evidence_ready;
        Ok(VoxelEditReport {
            address,
            previous,
            current: cell,
            frame_validated: true,
            stored_explicit_cell,
            removed_explicit_cell,
            semantic_noop,
            exact_edit_replay_ready,
        })
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
