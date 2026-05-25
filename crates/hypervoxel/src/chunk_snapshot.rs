//! Deterministic snapshot replay over chunk-paged sparse storage.
//!
//! Chunk pages are storage partitions. Snapshot bytes are interchange fixtures.
//! Neither is allowed to become topology evidence by convention, so this module
//! replays exact page contents into the canonical sparse-grid snapshot path and
//! reports the page evidence beside the snapshot replay report.

use crate::{
    ChunkPagedSparseGrid, DeterministicSnapshot, DeterministicSnapshotReport, HypervoxelResult,
    OccupancyState, SnapshotFormat, SparseVoxelGrid, VoxelSideTables,
};

/// Page-backed deterministic snapshot output and replay evidence.
///
/// The snapshot bytes are produced by the existing canonical sparse-grid
/// serializer after exact page replay. The page counters describe the storage
/// schedule used to reconstruct that semantic grid. This follows Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997:
/// optimized representations and serialized artifacts are acceptable only when
/// the exact object facts they preserve remain explicit. The deterministic
/// replay discipline is also in the spirit of Knuth, *The Art of Computer
/// Programming*, Vol. 3, 2nd ed., Addison-Wesley, 1998: stable ordering is a
/// data-structure fact, not a substitute for semantic equality.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPagedSnapshotReplay {
    /// Deterministic snapshot produced after replaying chunk pages.
    pub snapshot: DeterministicSnapshot,
    /// Semantic replay report for the snapshot bytes.
    pub snapshot_report: DeterministicSnapshotReport,
    /// Number of occupied pages replayed.
    pub replayed_pages: usize,
    /// Number of explicit non-empty cells replayed.
    pub replayed_cells: usize,
    /// Number of replayed cells carrying unknown occupancy.
    pub unknown_cells: usize,
    /// Number of replayed cells from lossy adapters.
    pub lossy_cells: usize,
    /// Whether the replayed sparse grid has the same explicit cell count as
    /// the page backend reported and the snapshot retained nonempty cell/run
    /// evidence for nonempty input.
    pub exact_cell_count_replay: bool,
    /// Whether this page-backed snapshot can be consumed as exact replay
    /// evidence for the chosen snapshot format.
    pub exact_paged_snapshot_ready: bool,
}

/// Produces a deterministic binary snapshot by replaying exact chunk pages.
pub fn chunk_paged_binary_snapshot_v1(
    grid: &ChunkPagedSparseGrid,
    side_tables: &VoxelSideTables,
) -> HypervoxelResult<ChunkPagedSnapshotReplay> {
    snapshot_from_pages(grid, side_tables, SnapshotFormat::BinaryV1)
}

/// Produces a deterministic run-length binary snapshot by replaying exact
/// chunk pages.
///
/// RLE snapshots intentionally omit full frame and side-table records, so the
/// returned [`ChunkPagedSnapshotReplay::exact_paged_snapshot_ready`] remains
/// false even when page replay and run records are exact.
pub fn chunk_paged_run_length_snapshot_v1(
    grid: &ChunkPagedSparseGrid,
) -> HypervoxelResult<ChunkPagedSnapshotReplay> {
    snapshot_from_pages(
        grid,
        &VoxelSideTables::default(),
        SnapshotFormat::RunLengthBinaryV1,
    )
}

fn snapshot_from_pages(
    grid: &ChunkPagedSparseGrid,
    side_tables: &VoxelSideTables,
    format: SnapshotFormat,
) -> HypervoxelResult<ChunkPagedSnapshotReplay> {
    let mut replayed = SparseVoxelGrid::new(grid.frame().clone());
    let mut replayed_pages = 0_usize;
    let mut replayed_cells = 0_usize;
    let mut unknown_cells = 0_usize;
    let mut lossy_cells = 0_usize;

    for (_, page) in grid.pages() {
        replayed_pages += 1;
        for (address, cell) in page.iter() {
            replayed_cells += 1;
            unknown_cells += usize::from(cell.occupancy == OccupancyState::Unknown);
            lossy_cells += usize::from(cell.occupancy == OccupancyState::LossyAdapterValue);
            replayed.set(*address, *cell)?;
        }
    }

    let snapshot = match format {
        SnapshotFormat::BinaryV1 => DeterministicSnapshot::binary_v1(&replayed, side_tables),
        SnapshotFormat::RunLengthBinaryV1 => DeterministicSnapshot::run_length_binary_v1(&replayed),
        SnapshotFormat::TextV1 => DeterministicSnapshot::text_v1(&replayed, side_tables),
    };
    let snapshot_report = snapshot.report();
    let exact_cell_count_replay = replayed_cells == grid.len()
        && match format {
            SnapshotFormat::RunLengthBinaryV1 => {
                replayed_cells == 0 || snapshot_report.has_cell_records
            }
            _ => snapshot_report.serialized_cell_records == replayed_cells,
        };
    let exact_paged_snapshot_ready = grid.report().exact_chunk_storage_ready
        && exact_cell_count_replay
        && unknown_cells == 0
        && lossy_cells == 0
        && snapshot_report.exact_snapshot_replay_ready;

    Ok(ChunkPagedSnapshotReplay {
        snapshot,
        snapshot_report,
        replayed_pages,
        replayed_cells,
        unknown_cells,
        lossy_cells,
        exact_cell_count_replay,
        exact_paged_snapshot_ready,
    })
}
