//! Conservative cell handoff reports for voxel/grid consumers.
//!
//! `hypervoxel` owns sampled-grid storage and aggregate facts; `hypersdf`
//! supplies continuous-field evidence. This module keeps that boundary narrow:
//! it packages exact/certified cell classifications and summary counts, but it
//! does not allocate a voxel tree or turn preview samples into occupancy. The
//! design follows Yap's EGC rule that uncertain cells remain explicit unknowns.

use crate::status::{SdfCellClassificationReport, SdfCellLocation, SdfFreshness, SdfMetricStatus};

/// Conservative cell-classification package for grid/voxel consumers.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfVoxelHandoffReport {
    /// Metric claim of the source expression.
    pub metric_status: SdfMetricStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
    /// Number of cells in the package.
    pub cell_count: usize,
    /// Number of cells certified completely inside.
    pub certified_inside_count: usize,
    /// Number of cells certified to touch or cross the boundary.
    pub boundary_count: usize,
    /// Number of cells certified completely outside.
    pub certified_outside_count: usize,
    /// Number of cells that remain unknown.
    pub unknown_count: usize,
    /// Per-cell classification reports.
    pub cells: Vec<SdfCellClassificationReport>,
}

impl SdfVoxelHandoffReport {
    /// Build a handoff package from cell reports.
    pub fn from_cells(
        cells: Vec<SdfCellClassificationReport>,
        metric_status: SdfMetricStatus,
        freshness: SdfFreshness,
    ) -> Self {
        let mut certified_inside_count = 0_usize;
        let mut boundary_count = 0_usize;
        let mut certified_outside_count = 0_usize;
        let mut unknown_count = 0_usize;
        for cell in &cells {
            match cell.location {
                SdfCellLocation::ConservativeInside => certified_inside_count += 1,
                SdfCellLocation::Boundary => boundary_count += 1,
                SdfCellLocation::ConservativeOutside => certified_outside_count += 1,
                SdfCellLocation::Unknown => unknown_count += 1,
            }
        }
        Self {
            metric_status,
            freshness,
            cell_count: cells.len(),
            certified_inside_count,
            boundary_count,
            certified_outside_count,
            unknown_count,
            cells,
        }
    }

    /// Returns whether every cell has certified non-unknown evidence.
    pub const fn all_cells_classified(&self) -> bool {
        self.unknown_count == 0
    }

    /// Validate summary counts against contained cell reports.
    pub fn is_self_consistent(&self) -> bool {
        let mut certified_inside_count = 0_usize;
        let mut boundary_count = 0_usize;
        let mut certified_outside_count = 0_usize;
        let mut unknown_count = 0_usize;
        for cell in &self.cells {
            if !cell.is_self_consistent() {
                return false;
            }
            match cell.location {
                SdfCellLocation::ConservativeInside => certified_inside_count += 1,
                SdfCellLocation::Boundary => boundary_count += 1,
                SdfCellLocation::ConservativeOutside => certified_outside_count += 1,
                SdfCellLocation::Unknown => unknown_count += 1,
            }
        }
        self.cell_count == self.cells.len()
            && self.certified_inside_count == certified_inside_count
            && self.boundary_count == boundary_count
            && self.certified_outside_count == certified_outside_count
            && self.unknown_count == unknown_count
    }
}
