//! Semantic differential reports for voxel storage fixtures.
//!
//! Differential checks are useful when harvesting `voxelis` storage ideas, but
//! they must compare semantics that actually match. This module compares exact
//! address/cell facts between two semantic grids and reports mismatches without
//! granting exact status to any lossy legacy voxelizer. Predicates and object
//! facts are checked explicitly instead of inferred from a nearby
//! implementation.

use std::collections::BTreeSet;

use crate::{SparseVoxelGrid, VoxelAddress};

/// Differential report between two sparse semantic grids.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SparseGridDiffReport {
    /// Whether both grids use the same exact frame.
    pub frame_matches: bool,
    /// Number of distinct addresses compared across both grids.
    pub compared_addresses: usize,
    /// Whether at least one address was compared.
    ///
    /// Comparing two empty sparse maps is a precise no-difference report, but
    /// it is not evidence that a ported backend preserved any object-level
    /// voxel facts. This bit keeps empty fixture comparisons from becoming
    /// vacuous equivalence certificates.
    pub has_compared_addresses: bool,
    /// Addresses present only in the left grid.
    pub only_left: Vec<VoxelAddress>,
    /// Addresses present only in the right grid.
    pub only_right: Vec<VoxelAddress>,
    /// Addresses present in both grids but carrying different cells.
    pub differing_cells: Vec<VoxelAddress>,
    /// Total number of address or payload mismatches.
    pub mismatch_count: usize,
    /// Whether the two grids are non-vacuously semantically identical.
    ///
    /// A backend matches only when at least one exact address and cell fact was
    /// compared in the same grid frame, not because a lossy fixture looks
    /// visually close or two empty maps share a frame.
    pub semantic_equivalence_ready: bool,
}

impl SparseGridDiffReport {
    /// Returns whether the compared grids are semantically identical.
    pub fn is_equal(&self) -> bool {
        self.semantic_equivalence_ready
    }
}

/// Compares two sparse grids by exact address and cell payload.
pub fn diff_sparse_grids(left: &SparseVoxelGrid, right: &SparseVoxelGrid) -> SparseGridDiffReport {
    let frame_matches = left.frame() == right.frame();
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
    let compared_addresses = left_addresses.union(&right_addresses).count();
    let mismatch_count =
        usize::from(!frame_matches) + only_left.len() + only_right.len() + differing_cells.len();

    SparseGridDiffReport {
        frame_matches,
        compared_addresses,
        has_compared_addresses: compared_addresses > 0,
        only_left,
        only_right,
        differing_cells,
        mismatch_count,
        semantic_equivalence_ready: compared_addresses > 0 && frame_matches && mismatch_count == 0,
    }
}
