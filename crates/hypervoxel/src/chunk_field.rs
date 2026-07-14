//! Field-sample side-table audits over chunk-paged sparse storage.
//!
//! Voxel field payloads are compact sample IDs. Chunk pages can accelerate the
//! scan, but certified scalar intervals still come only from side-table records
//! and exact/certified comparisons.

use std::collections::BTreeSet;

use crate::{
    ChunkPagedSparseGrid, FieldAggregateFacts, FieldSampleId, FieldSampleQuery, HypervoxelResult,
    OccupancyState, VoxelCell, VoxelPayload, VoxelSideTables,
};

/// Page-backed field-sample audit.
///
/// The embedded [`FieldSampleQuery`] and [`FieldAggregateFacts`] retain the
/// canonical field side-table semantics. Page counters describe how chunk
/// storage scheduled the scan. Certified interval bounds are represented and
/// checked as object evidence rather than inferred from sampled floats or
/// storage layout.
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkPagedFieldAuditReport {
    /// Field sample references and missing side-table evidence.
    pub query: FieldSampleQuery,
    /// Certified/certain aggregate facts over referenced field samples.
    pub aggregate: FieldAggregateFacts,
    /// Number of occupied pages scanned.
    pub tested_pages: usize,
    /// Number of explicit non-empty cells scanned.
    pub tested_cells: usize,
    /// Number of scanned cells carrying field-sample payloads.
    pub field_payload_cells: usize,
    /// Number of scanned cells carrying non-field payloads.
    pub non_field_payload_cells: usize,
    /// Number of scanned cells with explicit unknown occupancy.
    pub unknown_cells: usize,
    /// Number of scanned cells from lossy adapters.
    pub lossy_cells: usize,
    /// Distinct field sample IDs observed in deterministic order.
    pub referenced_samples: BTreeSet<FieldSampleId>,
    /// Whether this page-backed field audit is complete certified evidence.
    pub exact_paged_field_audit_ready: bool,
}

/// Audits field-sample side-table references through chunk-paged storage.
///
/// This is the page-backed counterpart to [`crate::query_field_samples`] plus
/// [`FieldAggregateFacts::from_grid`]. Unknown and lossy cells are readiness
/// blockers even when they do not carry field-sample payloads, because the
/// page-backed audit is meant to certify the explicit stored cell set being
/// handed to a downstream field or physics consumer.
pub fn audit_chunk_paged_field_samples(
    grid: &ChunkPagedSparseGrid,
    side_tables: &VoxelSideTables,
) -> HypervoxelResult<ChunkPagedFieldAuditReport> {
    let mut referenced = BTreeSet::new();
    let mut missing_records = BTreeSet::new();
    let mut missing_bounds = BTreeSet::new();
    let mut cells = Vec::<&VoxelCell>::new();
    let mut tested_pages = 0_usize;
    let mut tested_cells = 0_usize;
    let mut field_payload_cells = 0_usize;
    let mut non_field_payload_cells = 0_usize;
    let mut unknown_cells = 0_usize;
    let mut lossy_cells = 0_usize;

    for (_, page) in grid.pages() {
        tested_pages += 1;
        for (_, cell) in page.iter() {
            if cell.occupancy == OccupancyState::Empty {
                continue;
            }
            tested_cells += 1;
            cells.push(cell);
            unknown_cells += usize::from(cell.occupancy == OccupancyState::Unknown);
            lossy_cells += usize::from(cell.occupancy == OccupancyState::LossyAdapterValue);
            if let VoxelPayload::FieldSample(sample) = cell.payload {
                field_payload_cells += 1;
                referenced.insert(sample);
                match side_tables.field_sample(sample) {
                    Some(record) if record.lower.is_some() && record.upper.is_some() => {}
                    Some(_) => {
                        missing_bounds.insert(sample);
                    }
                    None => {
                        missing_records.insert(sample);
                    }
                }
            } else {
                non_field_payload_cells += 1;
            }
        }
    }

    let query = FieldSampleQuery {
        referenced: referenced.clone(),
        missing_records,
        missing_bounds,
    };
    let aggregate = FieldAggregateFacts::from_cells(cells, side_tables)?;
    let exact_paged_field_audit_ready = grid.report().exact_chunk_storage_ready
        && tested_cells > 0
        && unknown_cells == 0
        && lossy_cells == 0
        && query.is_fully_resolved()
        && aggregate.certified_field_bounds_ready;

    Ok(ChunkPagedFieldAuditReport {
        query,
        aggregate,
        tested_pages,
        tested_cells,
        field_payload_cells,
        non_field_payload_cells,
        unknown_cells,
        lossy_cells,
        referenced_samples: referenced,
        exact_paged_field_audit_ready,
    })
}
