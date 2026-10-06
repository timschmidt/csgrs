//! Semantic differential reports for chunk-paged sparse storage.
//!
//! Page layout is useful when porting `voxelis`-style storage, but page equality
//! is not voxel equality. This module compares exact retained addresses and
//! cells while separately reporting page coverage, so a backend can be checked
//! for semantic equivalence without treating layout as topology.

use std::collections::BTreeSet;

use crate::{ChunkAddress, ChunkPagedSparseGrid, VoxelAddress};

/// Differential report between two chunk-paged sparse grids.
///
/// Page counters expose the storage schedule, while address and cell
/// mismatches remain the semantic facts. The page-set partition describes
/// storage coverage, not an approximate geometric predicate; exact claims are
/// grounded in retained object structure.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPagedSparseGridDiffReport {
    /// Whether both grids use the same exact frame.
    pub frame_matches: bool,
    /// Whether both grids use the same chunk shape.
    pub shape_matches: bool,
    /// Occupied pages in the left grid.
    pub left_pages: usize,
    /// Occupied pages in the right grid.
    pub right_pages: usize,
    /// Page addresses present in both layouts.
    pub shared_pages: usize,
    /// Page addresses present only in the left layout.
    pub only_left_pages: Vec<ChunkAddress>,
    /// Page addresses present only in the right layout.
    pub only_right_pages: Vec<ChunkAddress>,
    /// Number of distinct explicit cell addresses compared across both grids.
    pub compared_addresses: usize,
    /// Whether at least one address was compared.
    pub has_compared_addresses: bool,
    /// Addresses present only in the left grid.
    pub only_left: Vec<VoxelAddress>,
    /// Addresses present only in the right grid.
    pub only_right: Vec<VoxelAddress>,
    /// Addresses present in both grids but carrying different cells.
    pub differing_cells: Vec<VoxelAddress>,
    /// Total semantic or structural mismatch count.
    pub mismatch_count: usize,
    /// Whether the page-level audit itself can be consumed as exact evidence.
    pub exact_page_diff_ready: bool,
    /// Whether the two grids are non-vacuously semantically identical.
    pub semantic_equivalence_ready: bool,
}

impl ChunkPagedSparseGridDiffReport {
    /// Returns whether the compared paged grids are semantically identical.
    pub fn is_equal(&self) -> bool {
        self.semantic_equivalence_ready
    }
}

/// Compares two chunk-paged sparse grids by exact address and cell payload.
///
/// The function reports page coverage separately from semantic equality. Two
/// layouts with different chunk shapes are not exact page-diff ready, even if
/// their retained cells match, because page counters no longer describe the
/// same partition. Address/cell comparison still proceeds so callers can see
/// whether object-level voxel facts match independently from layout shape.
pub fn diff_chunk_paged_sparse_grids(
    left: &ChunkPagedSparseGrid,
    right: &ChunkPagedSparseGrid,
) -> ChunkPagedSparseGridDiffReport {
    let frame_matches = left.frame() == right.frame();
    let shape_matches = left.shape() == right.shape();

    let left_pages = left
        .pages()
        .map(|(chunk, _)| *chunk)
        .collect::<BTreeSet<_>>();
    let right_pages = right
        .pages()
        .map(|(chunk, _)| *chunk)
        .collect::<BTreeSet<_>>();
    let shared_pages = left_pages.intersection(&right_pages).count();
    let only_left_pages = left_pages
        .difference(&right_pages)
        .copied()
        .collect::<Vec<_>>();
    let only_right_pages = right_pages
        .difference(&left_pages)
        .copied()
        .collect::<Vec<_>>();

    let left_addresses = left
        .pages()
        .flat_map(|(_, page)| page.iter().map(|(address, _)| *address))
        .collect::<BTreeSet<_>>();
    let right_addresses = right
        .pages()
        .flat_map(|(_, page)| page.iter().map(|(address, _)| *address))
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
    let mismatch_count = usize::from(!frame_matches)
        + usize::from(!shape_matches)
        + only_left.len()
        + only_right.len()
        + differing_cells.len();
    let exact_page_diff_ready = frame_matches
        && shape_matches
        && left.report().exact_address_replay_ready
        && right.report().exact_address_replay_ready
        && left.report().exact_payload_replay_ready
        && right.report().exact_payload_replay_ready
        && !left.report().has_unknown
        && !right.report().has_unknown
        && !left.report().has_lossy
        && !right.report().has_lossy;
    let semantic_equivalence_ready =
        exact_page_diff_ready && compared_addresses > 0 && mismatch_count == 0;

    ChunkPagedSparseGridDiffReport {
        frame_matches,
        shape_matches,
        left_pages: left_pages.len(),
        right_pages: right_pages.len(),
        shared_pages,
        only_left_pages,
        only_right_pages,
        compared_addresses,
        has_compared_addresses: compared_addresses > 0,
        only_left,
        only_right,
        differing_cells,
        mismatch_count,
        exact_page_diff_ready,
        semantic_equivalence_ready,
    }
}
