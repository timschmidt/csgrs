//! Exact domain handoff certificates for chunk-paged sparse storage.
//!
//! The generic [`crate::VoxelHandoffManifest`] is intentionally small: it
//! checks source freshness, aggregate certainty, and side-table link counts.
//! This module builds those manifest facts from the concrete page backend
//! instead of asking a caller to count links by hand. It replays the pages into
//! deterministic snapshot bytes and audits material, field, and process
//! side-table references before naming the handoff exact.
//!
//! This follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997: an optimized or serialized representation can be an
//! exact component only when the represented object facts and unresolved
//! blockers remain explicit. The side-table boundary is also kept explicit in
//! the STEP AP242 spirit (ISO 10303-242:2022): product/process/material
//! references are evidence links, not implicit geometry.

use crate::{
    ChunkPagedFieldAuditReport, ChunkPagedMaterialAuditReport, ChunkPagedProcessStateAuditReport,
    ChunkPagedSnapshotReplay, ChunkPagedSparseGrid, ChunkPagedSparseStorageReport, FreshnessStatus,
    GridSource, HypervoxelResult, VoxelHandoffDomain, VoxelHandoffManifest, VoxelHandoffReport,
    VoxelSideTables, audit_chunk_paged_field_samples, audit_chunk_paged_material_regions,
    audit_chunk_paged_process_states, chunk_paged_binary_snapshot_v1,
};

/// Page-backed domain handoff certificate.
///
/// This report is the chunk-paged counterpart to
/// [`crate::VoxelHandoffManifest::report`]. It retains the actual page-storage
/// report, deterministic binary snapshot replay, and all side-table audits
/// used to populate the generic manifest. A downstream crate can therefore
/// inspect *why* a handoff is exact, stale, incomplete, or blocked by unknown
/// cells instead of receiving only a boolean.
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkPagedHandoffReport {
    /// Destination domain.
    pub domain: VoxelHandoffDomain,
    /// Source freshness computed against the expected source version.
    pub freshness: FreshnessStatus,
    /// Exact page-storage evidence.
    pub storage: ChunkPagedSparseStorageReport,
    /// Deterministic binary snapshot replay evidence.
    pub snapshot: ChunkPagedSnapshotReplay,
    /// Material side-table audit over the paged cells.
    pub material: ChunkPagedMaterialAuditReport,
    /// Field-sample side-table audit over the paged cells.
    pub field: ChunkPagedFieldAuditReport,
    /// Process-state side-table audit over the paged cells.
    pub process: ChunkPagedProcessStateAuditReport,
    /// Number of side-table links referenced by material/field/process payloads.
    pub required_side_table_links: usize,
    /// Number of referenced links with records present in the supplied side tables.
    pub supplied_side_table_links: usize,
    /// Number of referenced links with complete metadata for Hyper-owned audits.
    pub complete_side_table_links: usize,
    /// Whether every referenced side-table link has complete evidence.
    pub side_table_evidence_ready: bool,
    /// Handoff report produced by the generic manifest gate from these facts.
    pub domain_report: VoxelHandoffReport,
    /// Whether this paged artifact may be consumed as exact handoff evidence.
    pub exact_paged_handoff_ready: bool,
}

/// Certifies a chunk-paged sparse grid for downstream domain handoff.
///
/// The function deliberately accepts source freshness inputs rather than
/// reading them from the [`crate::GridFrame`]. A storage snapshot may be a
/// derived artifact with its own source/version; exact consumers need that
/// artifact freshness replayed explicitly. The readiness gate requires:
///
/// - exact chunk page/storage replay,
/// - deterministic binary snapshot replay over the same explicit cells,
/// - complete side-table evidence for every referenced material, field, and
///   process payload,
/// - a current generic domain handoff report with non-empty exact aggregate
///   evidence.
pub fn certify_chunk_paged_handoff(
    grid: &ChunkPagedSparseGrid,
    side_tables: &VoxelSideTables,
    domain: VoxelHandoffDomain,
    source: Option<GridSource>,
    expected_source: Option<GridSource>,
) -> HypervoxelResult<ChunkPagedHandoffReport> {
    let snapshot = chunk_paged_binary_snapshot_v1(grid, side_tables)?;
    let material = audit_chunk_paged_material_regions(grid, side_tables);
    let field = audit_chunk_paged_field_samples(grid, side_tables)?;
    let process = audit_chunk_paged_process_states(grid, side_tables);

    let required_side_table_links = material.referenced_regions.len()
        + field.referenced_samples.len()
        + process.referenced_states.len();
    let supplied_side_table_links = supplied_material_links(&material)
        + supplied_field_links(&field)
        + process.resolved_records;
    let complete_side_table_links = complete_material_links(&material)
        + complete_field_links(&field)
        + complete_process_links(&process);
    let side_table_evidence_ready = required_side_table_links == complete_side_table_links;

    let manifest = VoxelHandoffManifest {
        domain,
        source,
        expected_source,
        required_side_table_links,
        supplied_side_table_links: complete_side_table_links,
        aggregate: grid.report().aggregate.clone(),
    };
    let domain_report = manifest.report();
    let freshness = domain_report.freshness;
    let exact_paged_handoff_ready = grid.report().exact_chunk_storage_ready
        && snapshot.exact_paged_snapshot_ready
        && side_table_evidence_ready
        && domain_report.exact_handoff_ready;

    Ok(ChunkPagedHandoffReport {
        domain,
        freshness,
        storage: grid.report().clone(),
        snapshot,
        material,
        field,
        process,
        required_side_table_links,
        supplied_side_table_links,
        complete_side_table_links,
        side_table_evidence_ready,
        domain_report,
        exact_paged_handoff_ready,
    })
}

fn supplied_material_links(report: &ChunkPagedMaterialAuditReport) -> usize {
    report.metadata.resolved_records
}

fn complete_material_links(report: &ChunkPagedMaterialAuditReport) -> usize {
    let mut blocked = report.metadata.missing_records.clone();
    blocked.extend(report.metadata.records_missing_density.iter().copied());
    blocked.extend(report.metadata.empty_labels.iter().copied());
    blocked.extend(report.metadata.empty_provenance.iter().copied());
    report.referenced_regions.len() - blocked.len()
}

fn supplied_field_links(report: &ChunkPagedFieldAuditReport) -> usize {
    report.referenced_samples.len()
        - report.query.missing_records.len()
        - report.query.missing_bounds.len()
}

fn complete_field_links(report: &ChunkPagedFieldAuditReport) -> usize {
    let mut blocked = report.query.missing_records.clone();
    blocked.extend(report.query.missing_bounds.iter().copied());
    report.referenced_samples.len() - blocked.len()
}

fn complete_process_links(report: &ChunkPagedProcessStateAuditReport) -> usize {
    let mut blocked = report.missing_records.clone();
    blocked.extend(report.empty_labels.iter().copied());
    blocked.extend(report.empty_provenance.iter().copied());
    report.referenced_states.len() - blocked.len()
}
