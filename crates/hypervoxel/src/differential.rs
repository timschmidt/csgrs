//! Semantic differential reports for voxel storage fixtures.
//!
//! Differential checks are useful when harvesting `voxelis` storage ideas, but
//! they must compare semantics that actually match. This module compares exact
//! address/cell facts between two semantic grids and reports mismatches without
//! granting exact status to any lossy legacy voxelizer. That boundary is the
//! same one advocated by Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997: predicates and object facts are
//! checked explicitly instead of inferred from a nearby implementation.

use std::collections::BTreeSet;

use crate::{SparseVoxelGrid, VoxelAddress};

/// Differential report between two sparse semantic grids.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SparseGridDiffReport {
    /// Addresses present only in the left grid.
    pub only_left: Vec<VoxelAddress>,
    /// Addresses present only in the right grid.
    pub only_right: Vec<VoxelAddress>,
    /// Addresses present in both grids but carrying different cells.
    pub differing_cells: Vec<VoxelAddress>,
}

impl SparseGridDiffReport {
    /// Returns whether the compared grids are semantically identical.
    pub fn is_equal(&self) -> bool {
        self.only_left.is_empty() && self.only_right.is_empty() && self.differing_cells.is_empty()
    }
}

/// Compares two sparse grids by exact address and cell payload.
pub fn diff_sparse_grids(left: &SparseVoxelGrid, right: &SparseVoxelGrid) -> SparseGridDiffReport {
    let left_addresses = left
        .iter()
        .map(|(address, _)| *address)
        .collect::<BTreeSet<_>>();
    let right_addresses = right
        .iter()
        .map(|(address, _)| *address)
        .collect::<BTreeSet<_>>();

    let only_left = left_addresses
        .difference(&right_addresses)
        .copied()
        .collect::<Vec<_>>();
    let only_right = right_addresses
        .difference(&left_addresses)
        .copied()
        .collect::<Vec<_>>();
    let differing_cells = left_addresses
        .intersection(&right_addresses)
        .copied()
        .filter(|address| left.get(*address).ok() != right.get(*address).ok())
        .collect::<Vec<_>>();

    SparseGridDiffReport {
        only_left,
        only_right,
        differing_cells,
    }
}
