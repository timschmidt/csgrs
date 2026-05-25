//! Process-state side-table audits over chunk-paged sparse storage.
//!
//! Process-state payloads are compact provenance handles. They are useful for
//! CAM, fabrication, and simulation pipelines, but `hypervoxel` must not infer
//! process meaning from an integer ID or from chunk layout.

use std::collections::BTreeSet;

use crate::{ChunkPagedSparseGrid, OccupancyState, ProcessStateId, VoxelPayload, VoxelSideTables};

/// Page-backed process-state side-table audit.
///
/// This report validates that process-state payload IDs resolve to non-empty
/// side-table labels and provenance while exposing the page scan that found
/// them. Following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997, the voxel layer keeps the object
/// reference and unresolved evidence explicit instead of interpreting process
/// physics. The provenance boundary follows the manufacturing traceability
/// principle in ISO 10303-242:2022 (STEP AP242): product/process references
/// should remain explicit identifiers, not implicit geometry or display state.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPagedProcessStateAuditReport {
    /// Number of occupied pages scanned.
    pub tested_pages: usize,
    /// Number of explicit non-empty cells scanned.
    pub tested_cells: usize,
    /// Number of scanned cells carrying process-state payloads.
    pub process_payload_cells: usize,
    /// Number of scanned cells carrying non-process payloads.
    pub non_process_payload_cells: usize,
    /// Distinct process states referenced by cells.
    pub referenced_states: BTreeSet<ProcessStateId>,
    /// Whether at least one process state was referenced.
    pub has_process_states: bool,
    /// Referenced process states missing from the side table.
    pub missing_records: BTreeSet<ProcessStateId>,
    /// Referenced process states whose side-table record has an empty label.
    pub empty_labels: BTreeSet<ProcessStateId>,
    /// Referenced process states whose side-table record has empty provenance.
    pub empty_provenance: BTreeSet<ProcessStateId>,
    /// Number of referenced process states with side-table records.
    pub resolved_records: usize,
    /// Number of scanned cells with explicit unknown occupancy.
    pub unknown_cells: usize,
    /// Number of scanned cells from lossy adapters.
    pub lossy_cells: usize,
    /// Whether this page-backed process-state audit is complete exact evidence.
    pub exact_paged_process_audit_ready: bool,
}

impl ChunkPagedProcessStateAuditReport {
    /// Returns whether all referenced process-state records are complete.
    pub fn is_complete(&self) -> bool {
        self.has_process_states
            && self.missing_records.is_empty()
            && self.empty_labels.is_empty()
            && self.empty_provenance.is_empty()
    }
}

/// Audits process-state side-table references through chunk-paged storage.
///
/// Unknown and lossy cells block exact readiness even when they do not carry a
/// process-state payload. A process consumer should not treat an uncertain
/// explicit cell set as complete process evidence.
pub fn audit_chunk_paged_process_states(
    grid: &ChunkPagedSparseGrid,
    side_tables: &VoxelSideTables,
) -> ChunkPagedProcessStateAuditReport {
    let mut tested_pages = 0_usize;
    let mut tested_cells = 0_usize;
    let mut process_payload_cells = 0_usize;
    let mut non_process_payload_cells = 0_usize;
    let mut referenced_states = BTreeSet::new();
    let mut missing_records = BTreeSet::new();
    let mut empty_labels = BTreeSet::new();
    let mut empty_provenance = BTreeSet::new();
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
            if let VoxelPayload::ProcessState(state) = cell.payload {
                process_payload_cells += 1;
                referenced_states.insert(state);
                match side_tables.process_state(state) {
                    Some(record) => {
                        if record.label.trim().is_empty() {
                            empty_labels.insert(state);
                        }
                        if record.provenance.trim().is_empty() {
                            empty_provenance.insert(state);
                        }
                    }
                    None => {
                        missing_records.insert(state);
                    }
                }
            } else {
                non_process_payload_cells += 1;
            }
        }
    }

    let has_process_states = !referenced_states.is_empty();
    let resolved_records = referenced_states.len() - missing_records.len();
    let exact_paged_process_audit_ready = grid.report().exact_chunk_storage_ready
        && tested_cells > 0
        && unknown_cells == 0
        && lossy_cells == 0
        && has_process_states
        && missing_records.is_empty()
        && empty_labels.is_empty()
        && empty_provenance.is_empty();

    ChunkPagedProcessStateAuditReport {
        tested_pages,
        tested_cells,
        process_payload_cells,
        non_process_payload_cells,
        referenced_states,
        has_process_states,
        missing_records,
        empty_labels,
        empty_provenance,
        resolved_records,
        unknown_cells,
        lossy_cells,
        exact_paged_process_audit_ready,
    }
}
