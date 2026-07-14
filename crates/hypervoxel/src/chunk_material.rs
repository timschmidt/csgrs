//! Material side-table audits over chunk-paged sparse storage.
//!
//! Material payloads in `hypervoxel` are compact region handles. Chunk paging
//! can accelerate scans, but it must not turn a payload ID into a material law
//! or hide missing side-table evidence.

use crate::{
    ChunkPagedSparseGrid, MaterialRegionMetadataReport, MaterialRegionQuery, OccupancyState,
    VoxelPayload, VoxelSideTables, report_material_region_metadata,
};
use std::collections::BTreeSet;

/// Page-backed material-region audit.
///
/// The embedded [`MaterialRegionQuery`] and [`MaterialRegionMetadataReport`]
/// keep the same side-table semantics as the sparse-grid path. Page counters
/// expose the storage schedule used to gather those references. Exact payload
/// facts and unresolved metadata stay explicit rather than being inferred from
/// layout, display labels, or colors; an identifier is not a property model.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPagedMaterialAuditReport {
    /// Material references and missing side-table records.
    pub query: MaterialRegionQuery,
    /// Metadata completeness audit for referenced material records.
    pub metadata: MaterialRegionMetadataReport,
    /// Number of occupied pages scanned.
    pub tested_pages: usize,
    /// Number of explicit non-empty cells scanned.
    pub tested_cells: usize,
    /// Number of scanned cells carrying material-region payloads.
    pub material_payload_cells: usize,
    /// Number of scanned cells carrying non-material payloads.
    pub non_material_payload_cells: usize,
    /// Number of scanned cells with explicit unknown occupancy.
    pub unknown_cells: usize,
    /// Number of scanned cells from lossy adapters.
    pub lossy_cells: usize,
    /// Distinct material regions observed in deterministic order.
    pub referenced_regions: BTreeSet<crate::MaterialRegionId>,
    /// Whether this page-backed material audit is complete exact evidence.
    pub exact_paged_material_audit_ready: bool,
}

/// Audits material-region side-table references through chunk-paged storage.
///
/// This is the page-backed counterpart to [`crate::query_material_regions`]
/// followed by [`crate::report_material_region_metadata`]. Unknown and lossy
/// cells are counted as blockers for exact readiness even when their payloads
/// do not name material regions, because a downstream material consumer should
/// not treat an uncertain cell set as complete material evidence.
pub fn audit_chunk_paged_material_regions(
    grid: &ChunkPagedSparseGrid,
    side_tables: &VoxelSideTables,
) -> ChunkPagedMaterialAuditReport {
    let mut referenced = BTreeSet::new();
    let mut missing_records = BTreeSet::new();
    let mut tested_pages = 0_usize;
    let mut tested_cells = 0_usize;
    let mut material_payload_cells = 0_usize;
    let mut non_material_payload_cells = 0_usize;
    let mut unknown_cells = 0_usize;
    let mut lossy_cells = 0_usize;

    for (_, page) in grid.pages() {
        tested_pages += 1;
        for (_, cell) in page.iter() {
            if cell.occupancy == OccupancyState::Empty {
                continue;
            }
            tested_cells += 1;
            unknown_cells += usize::from(cell.occupancy == OccupancyState::Unknown);
            lossy_cells += usize::from(cell.occupancy == OccupancyState::LossyAdapterValue);
            if let VoxelPayload::MaterialRegion(region) = cell.payload {
                material_payload_cells += 1;
                referenced.insert(region);
                if side_tables.material(region).is_none() {
                    missing_records.insert(region);
                }
            } else {
                non_material_payload_cells += 1;
            }
        }
    }

    let query = MaterialRegionQuery {
        referenced: referenced.clone(),
        missing_records,
    };
    let metadata = report_material_region_metadata(&query, side_tables);
    let exact_paged_material_audit_ready = grid.report().exact_chunk_storage_ready
        && tested_cells > 0
        && unknown_cells == 0
        && lossy_cells == 0
        && query.is_fully_resolved()
        && metadata.is_complete();

    ChunkPagedMaterialAuditReport {
        query,
        metadata,
        tested_pages,
        tested_cells,
        material_payload_cells,
        non_material_payload_cells,
        unknown_cells,
        lossy_cells,
        referenced_regions: referenced,
        exact_paged_material_audit_ready,
    }
}
