//! Conservative LOD selection over exact voxel addresses.
//!
//! A Hyper LOD cell is not an averaged material value. It is a coarser exact
//! address plus conservative facts over the stored descendants that selected
//! it. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997: preserve object-level combinatorial
//! structure and expose what has actually been proved.

use std::collections::BTreeMap;

use crate::{
    AggregateCertainty, HypervoxelError, HypervoxelResult, SparseVoxelGrid, VoxelAddress,
    VoxelAggregateFacts, VoxelCell,
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
    /// Number of selected coarse cells.
    pub selected_cells: usize,
    /// Whether at least one coarse cell was selected.
    ///
    /// An empty sparse grid can produce a precise empty LOD selection, but it
    /// is not evidence that any descendant aggregate was certified. This
    /// non-vacuous gate follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997: exact consumers should see what
    /// object facts were actually proved rather than inheriting truth from an
    /// empty count.
    pub has_selected_cells: bool,
    /// Number of selected cells whose descendant aggregate is exact.
    pub exact_aggregate_cells: usize,
    /// Number of selected cells whose descendant aggregate is certified.
    pub certified_aggregate_cells: usize,
    /// Number of selected cells whose descendant aggregate preserves uncertainty.
    pub unknown_aggregate_cells: usize,
    /// Number of selected cells whose descendant aggregate contains lossy adapter values.
    pub lossy_aggregate_cells: usize,
    /// Whether at least one selected descendant aggregate exists and all are exact or certified.
    ///
    /// This is deliberately not a claim that a coarse voxel equals an averaged
    /// material or topology value. It is a readiness flag for consuming the
    /// selected descendants as conservative LOD evidence. Empty selections,
    /// unknown packets, and lossy descendant packets block the flag, following
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997.
    pub certified_lod_aggregate_ready: bool,
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

    let cells = groups
        .into_iter()
        .map(|(address, cells)| LodCellSelection {
            address,
            aggregate: VoxelAggregateFacts::from_cells(cells.iter()),
        })
        .collect::<Vec<_>>();
    let selected_cells = cells.len();
    let exact_aggregate_cells = cells
        .iter()
        .filter(|cell| cell.aggregate.certainty == AggregateCertainty::Exact)
        .count();
    let certified_aggregate_cells = cells
        .iter()
        .filter(|cell| cell.aggregate.certainty == AggregateCertainty::Certified)
        .count();
    let unknown_aggregate_cells = cells
        .iter()
        .filter(|cell| cell.aggregate.certainty == AggregateCertainty::Unknown)
        .count();
    let lossy_aggregate_cells = cells
        .iter()
        .filter(|cell| cell.aggregate.certainty == AggregateCertainty::Lossy)
        .count();
    let has_selected_cells = selected_cells > 0;
    let certified_lod_aggregate_ready =
        has_selected_cells && selected_cells == exact_aggregate_cells + certified_aggregate_cells;

    Ok(LodSelectionReport {
        target_depth,
        selected_cells,
        has_selected_cells,
        exact_aggregate_cells,
        certified_aggregate_cells,
        unknown_aggregate_cells,
        lossy_aggregate_cells,
        certified_lod_aggregate_ready,
        cells,
    })
}

impl LodSelectionReport {
    /// Returns the aggregate over selected LOD cells.
    ///
    /// This helper keeps the report-level certificate counts and the aggregate
    /// packet in one place. It remains a conservative aggregate over selected
    /// descendants, not a smoothing or majority-vote LOD material.
    pub fn selected_aggregate(&self) -> VoxelAggregateFacts {
        VoxelAggregateFacts::from_aggregates(self.cells.iter().map(|cell| &cell.aggregate))
    }
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
