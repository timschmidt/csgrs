//! Report-bearing compressed storage route manifests.
//!
//! `voxelis` is useful because its octree/DAG, chunk paging, batching, and
//! compact payload ideas scale to large sparse worlds. Hyper still needs an
//! exact semantic boundary around those storage choices. These reports describe
//! compression and paging decisions without letting a storage codec redefine
//! occupancy, material, or aggregate truth. The separation is the same
//! object-structure rule emphasized by Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997.

use crate::{AggregateCertainty, ChunkShape, LegacyAdapterKind, LegacyAdapterStatus};

/// Compressed sparse-grid storage family.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CompressedStorageKind {
    /// Deterministic sparse map fixture.
    SparseMap,
    /// Sparse voxel octree.
    SparseVoxelOctree,
    /// Hash-consed sparse voxel DAG.
    SparseVoxelDag,
    /// Deterministic run-length snapshot stream.
    RunLengthSnapshot,
    /// Chunk-paged sparse storage.
    ChunkPaged,
}

/// Replay status for a compressed storage route.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum StorageReplayStatus {
    /// Payloads and aggregate facts can be replayed exactly.
    Exact,
    /// Aggregate facts are certified but some payload detail is external.
    Certified,
    /// Storage is display/fixture-only and cannot prove exact replay.
    Lossy,
    /// Replay status has not been established.
    Unknown,
}

/// Manifest for a compressed storage route.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompressedStorageManifest {
    /// Storage family.
    pub kind: CompressedStorageKind,
    /// Number of logically stored non-empty cells.
    pub stored_cells: usize,
    /// Number of physical nodes/pages/runs used by the route.
    pub physical_records: usize,
    /// Optional chunk shape for paged routes.
    pub chunk_shape: Option<ChunkShape>,
    /// Whether exact aggregate facts are stored or replayable.
    pub preserves_aggregate_facts: bool,
    /// Whether payload IDs are stored without lossy reinterpretation.
    pub preserves_payload_ids: bool,
    /// Whether domain side-table links are stored or recoverable.
    pub preserves_side_table_links: bool,
}

/// Report derived from a compressed storage manifest.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompressedStorageReport {
    /// Storage family.
    pub kind: CompressedStorageKind,
    /// Logical non-empty cell count.
    pub stored_cells: usize,
    /// Physical record count.
    pub physical_records: usize,
    /// Conservative replay status.
    pub replay_status: StorageReplayStatus,
    /// Aggregate certainty exposed by this route.
    pub aggregate_certainty: AggregateCertainty,
    /// Explicit legacy/import-export adapter status.
    pub adapter: LegacyAdapterStatus,
}

/// Memory-budget manifest for a voxel storage route.
///
/// Memory diagnostics are harvested from `voxelis` as reporting ideas only:
/// budget pressure must not change exact grid semantics or silently discard
/// payload/aggregate facts. The report therefore states whether the route is
/// within budget and whether any over-budget state is still exact/certified.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelMemoryBudgetManifest {
    /// Storage family being measured.
    pub kind: CompressedStorageKind,
    /// Number of bytes allocated or estimated for this route.
    pub estimated_bytes: usize,
    /// Caller-supplied budget in bytes.
    pub budget_bytes: usize,
    /// Whether payload IDs and aggregate facts are retained when over budget.
    pub preserves_exact_semantics_when_over_budget: bool,
}

/// Report for a voxel storage memory budget.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelMemoryBudgetReport {
    /// Storage family being measured.
    pub kind: CompressedStorageKind,
    /// Number of bytes allocated or estimated for this route.
    pub estimated_bytes: usize,
    /// Caller-supplied budget in bytes.
    pub budget_bytes: usize,
    /// Saturating number of bytes over budget.
    pub over_budget_bytes: usize,
    /// Whether the route is within the budget.
    pub within_budget: bool,
    /// Whether exact semantics survive the reported memory pressure.
    pub exact_semantics_preserved: bool,
}

impl CompressedStorageManifest {
    /// Builds a report for a storage route without inspecting encoded bytes.
    pub fn report(&self) -> CompressedStorageReport {
        let replay_status = if self.preserves_aggregate_facts
            && self.preserves_payload_ids
            && self.preserves_side_table_links
        {
            StorageReplayStatus::Exact
        } else if self.preserves_aggregate_facts && self.preserves_payload_ids {
            StorageReplayStatus::Certified
        } else if self.preserves_aggregate_facts {
            StorageReplayStatus::Unknown
        } else {
            StorageReplayStatus::Lossy
        };
        let exact_replay = matches!(replay_status, StorageReplayStatus::Exact);
        CompressedStorageReport {
            kind: self.kind,
            stored_cells: self.stored_cells,
            physical_records: self.physical_records,
            replay_status,
            aggregate_certainty: match replay_status {
                StorageReplayStatus::Exact => AggregateCertainty::Exact,
                StorageReplayStatus::Certified => AggregateCertainty::Certified,
                StorageReplayStatus::Lossy => AggregateCertainty::Lossy,
                StorageReplayStatus::Unknown => AggregateCertainty::Unknown,
            },
            adapter: if exact_replay {
                LegacyAdapterStatus::exact(LegacyAdapterKind::ImportExport, "compressed storage")
            } else {
                LegacyAdapterStatus::lossy(LegacyAdapterKind::ImportExport, "compressed storage")
            },
        }
    }
}

impl VoxelMemoryBudgetManifest {
    /// Builds a memory-budget report without changing storage semantics.
    pub fn report(&self) -> VoxelMemoryBudgetReport {
        let over_budget_bytes = self.estimated_bytes.saturating_sub(self.budget_bytes);
        let within_budget = over_budget_bytes == 0;
        VoxelMemoryBudgetReport {
            kind: self.kind,
            estimated_bytes: self.estimated_bytes,
            budget_bytes: self.budget_bytes,
            over_budget_bytes,
            within_budget,
            exact_semantics_preserved: within_budget
                || self.preserves_exact_semantics_when_over_budget,
        }
    }
}
