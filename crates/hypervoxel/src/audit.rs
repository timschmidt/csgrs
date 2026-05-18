//! Voxelization audit reports.
//!
//! A voxelization report should make its accounting auditable: stored cells,
//! implied empty cells, boundary/unknown/lossy cells, freshness, and adapter
//! replay status are separate facts. This mirrors Yap's exact geometric
//! computation guidance in "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997: a geometric pipeline should expose
//! predicate outcomes and uncertainty instead of hiding them inside a numeric
//! representation.

use crate::{FreshnessStatus, OccupancyState, SparseVoxelGrid, VoxelizationReport};

/// Audited voxelization cell accounting.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelizationAudit {
    /// Number of cells in the full finite frame.
    pub total_frame_cells: u64,
    /// Number of explicitly stored non-empty cells.
    pub stored_cells: usize,
    /// Number of stored filled cells.
    pub filled_cells: usize,
    /// Number of stored boundary cells.
    pub boundary_cells: usize,
    /// Number of stored unknown cells.
    pub unknown_cells: usize,
    /// Number of stored lossy cells.
    pub lossy_cells: usize,
    /// Number of implied empty cells in the finite frame.
    pub implied_empty_cells: u64,
    /// Freshness status against source provenance.
    pub freshness: FreshnessStatus,
    /// Whether an adapter was exactly replayed.
    pub exact_adapter_replay: bool,
    /// Number of cells with certified predicate outcomes before policy lowering.
    pub predicate_certified_cells: usize,
    /// Number of cells whose predicate outcome was unknown before policy lowering.
    pub predicate_unknown_cells: usize,
    /// Whether this audit can be consumed as exact voxelization accounting.
    ///
    /// Yap's EGC model requires exact consumers to see whether a result depends
    /// on unresolved predicates or adapter values. The audit is exact-ready
    /// only when every classified frame cell has certified predicate evidence
    /// and no stored cell carries explicit unknown or lossy occupancy. A lossy
    /// legacy adapter also blocks readiness even when the cell counts look
    /// clean, because adapter provenance is part of the represented object
    /// boundary in Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997.
    pub exact_audit_ready: bool,
}

impl VoxelizationAudit {
    /// Builds an audit report from a grid and its voxelization report.
    pub fn from_grid_and_report(grid: &SparseVoxelGrid, report: &VoxelizationReport) -> Self {
        let total_frame_cells = report.frame.cells_per_axis().pow(3);
        let mut filled_cells = 0_usize;
        let mut boundary_cells = 0_usize;
        let mut unknown_cells = 0_usize;
        let mut lossy_cells = 0_usize;
        for (_, cell) in grid.iter() {
            match cell.occupancy {
                OccupancyState::Filled => filled_cells += 1,
                OccupancyState::Boundary => boundary_cells += 1,
                OccupancyState::Unknown => unknown_cells += 1,
                OccupancyState::LossyAdapterValue => lossy_cells += 1,
                OccupancyState::Empty | OccupancyState::Mixed => {}
            }
        }
        let predicate_certified_cells = report.predicate_certificates.certified_cells();
        let predicate_unknown_cells = report.predicate_certificates.unknown_cells;
        let adapter_replay_ready = report
            .legacy_adapter
            .as_ref()
            .is_none_or(|adapter| adapter.exact_replay_ready());
        let exact_adapter_replay = report
            .legacy_adapter
            .as_ref()
            .is_some_and(|adapter| adapter.exact_replay_ready());
        let exact_audit_ready = unknown_cells == 0
            && lossy_cells == 0
            && adapter_replay_ready
            && predicate_unknown_cells == 0
            && predicate_certified_cells as u64 == total_frame_cells;

        Self {
            total_frame_cells,
            stored_cells: grid.len(),
            filled_cells,
            boundary_cells,
            unknown_cells,
            lossy_cells,
            implied_empty_cells: total_frame_cells.saturating_sub(grid.len() as u64),
            freshness: report.freshness(),
            exact_adapter_replay,
            predicate_certified_cells,
            predicate_unknown_cells,
            exact_audit_ready,
        }
    }

    /// Returns whether the audit contains explicit uncertainty or lossy cells.
    pub fn has_uncertainty(&self) -> bool {
        self.unknown_cells > 0 || self.lossy_cells > 0
    }
}
