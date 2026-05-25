//! Exact chunk-paged sparse storage.
//!
//! This module is the first Hyper-owned chunk paging backend rather than a
//! manifest-only description of paging. It borrows the useful storage idea
//! from voxel engines such as `voxelis`: group sparse cells by integer pages so
//! repeated queries can avoid scanning the whole sparse map. The paging
//! boundary is deliberately semantic, not geometric. Pages are derived only
//! from [`crate::VoxelAddress`] integer coordinates and [`crate::ChunkShape`];
//! metric coordinates still live in [`crate::GridFrame`].
//!
//! This follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997: an optimized representation may accelerate lookup,
//! but it must not change the object-level facts that exact predicates and
//! reports consume. Page reports therefore expose exact address replay,
//! payload readiness, unknown/lossy blockers, and aggregate facts instead of
//! asking callers to trust a compressed layout name.

use std::collections::BTreeMap;

use crate::{
    ChunkAddress, ChunkLocalAddress, ChunkPageSummary, ChunkShape, GridFrame, HypervoxelError,
    HypervoxelResult, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts, VoxelCell,
};

/// Exact cells stored in one chunk page.
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkPagedSparsePage {
    /// Integer chunk address for this page.
    pub chunk: ChunkAddress,
    /// Explicit non-empty cells in deterministic address order.
    cells: BTreeMap<VoxelAddress, VoxelCell>,
    /// Exact local address decomposition for each stored cell.
    locals: BTreeMap<VoxelAddress, ChunkLocalAddress>,
}

/// Report for one chunk page.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPagedSparsePageReport {
    /// Integer chunk address for this page.
    pub chunk: ChunkAddress,
    /// Number of explicit non-empty cells in this page.
    pub stored_cells: usize,
    /// Number of stored cells whose depth is the owning frame depth.
    pub finest_depth_cells: usize,
    /// Number of stored cells at a coarser address depth than the frame.
    pub non_finest_depth_cells: usize,
    /// Whether every local coordinate is inside the page extent.
    pub local_addresses_in_bounds: bool,
    /// Whether every page/local split recomposes to the original address.
    pub exact_local_recompose_ready: bool,
    /// Whether every payload in the page is exact-ready.
    pub exact_payload_replay_ready: bool,
    /// Whether any page cell carries explicit unknown evidence.
    pub has_unknown: bool,
    /// Whether any page cell came from a lossy adapter.
    pub has_lossy: bool,
    /// Aggregate facts over the explicit cells in this page.
    pub aggregate: VoxelAggregateFacts,
    /// Whether this page can be consumed as exact chunk-storage evidence.
    pub exact_page_replay_ready: bool,
}

/// Report for a chunk-paged sparse storage backend.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPagedSparseStorageReport {
    /// Chunk shape used for page partitioning.
    pub shape: ChunkShape,
    /// Deterministic page summary derived from exact integer addresses.
    pub summary: ChunkPageSummary,
    /// Number of stored cells whose depth is the owning frame depth.
    pub finest_depth_cells: usize,
    /// Number of stored cells at a coarser address depth than the frame.
    pub non_finest_depth_cells: usize,
    /// Whether all page/local decompositions are exact integer replays.
    pub exact_address_replay_ready: bool,
    /// Whether all explicit payloads are exact-ready.
    pub exact_payload_replay_ready: bool,
    /// Whether any stored cell carries unknown evidence.
    pub has_unknown: bool,
    /// Whether any stored cell came from a lossy adapter.
    pub has_lossy: bool,
    /// Aggregate facts over all explicit cells.
    pub aggregate: VoxelAggregateFacts,
    /// Whether this chunk-paged backend can replay exact sparse storage.
    pub exact_chunk_storage_ready: bool,
}

/// Sparse grid cells grouped by exact integer chunk pages.
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkPagedSparseGrid {
    frame: GridFrame,
    shape: ChunkShape,
    pages: BTreeMap<ChunkAddress, ChunkPagedSparsePage>,
    report: ChunkPagedSparseStorageReport,
}

impl ChunkPagedSparseGrid {
    /// Builds chunk-paged storage by replaying a sparse grid's explicit cells.
    ///
    /// Empty cells remain implicit, matching [`SparseVoxelGrid`]. The builder
    /// validates every stored address against the source frame and retains the
    /// page/local decomposition as evidence; it does not infer occupancy from
    /// page density or from metric bounds.
    pub fn from_sparse_grid(grid: &SparseVoxelGrid, shape: ChunkShape) -> HypervoxelResult<Self> {
        let mut pages: BTreeMap<ChunkAddress, ChunkPagedSparsePage> = BTreeMap::new();
        let mut finest_depth_cells = 0_usize;
        let mut non_finest_depth_cells = 0_usize;
        let mut exact_address_replay_ready = true;
        let mut exact_payload_replay_ready = true;
        let mut has_unknown = false;
        let mut has_lossy = false;

        for (address, cell) in grid.iter() {
            validate_address_in_frame(*address, grid.frame())?;
            if address.depth == grid.frame().depth() {
                finest_depth_cells += 1;
            } else {
                non_finest_depth_cells += 1;
            }

            let local = ChunkAddress::split(*address, shape);
            exact_address_replay_ready &= local.local_in_bounds && local.exact_recompose_ready;
            exact_payload_replay_ready &= cell.report().exact_cell_evidence_ready;
            has_unknown |= cell.report().has_unknown;
            has_lossy |= cell.report().has_lossy;

            let page = pages
                .entry(local.chunk)
                .or_insert_with(|| ChunkPagedSparsePage::new(local.chunk));
            page.cells.insert(*address, *cell);
            page.locals.insert(*address, local);
        }

        let summary =
            ChunkPageSummary::from_addresses(shape, grid.iter().map(|(address, _)| *address));
        let aggregate = grid.stored_aggregate();
        let exact_chunk_storage_ready = summary.exact_integer_partition
            && summary.exact_page_cover_ready
            && exact_address_replay_ready
            && exact_payload_replay_ready
            && !has_unknown
            && !has_lossy;
        let report = ChunkPagedSparseStorageReport {
            shape,
            summary,
            finest_depth_cells,
            non_finest_depth_cells,
            exact_address_replay_ready,
            exact_payload_replay_ready,
            has_unknown,
            has_lossy,
            aggregate,
            exact_chunk_storage_ready,
        };
        Ok(Self {
            frame: grid.frame().clone(),
            shape,
            pages,
            report,
        })
    }

    /// Returns the grid frame represented by this paged backend.
    pub fn frame(&self) -> &GridFrame {
        &self.frame
    }

    /// Returns the chunk shape used by this paged backend.
    pub fn shape(&self) -> ChunkShape {
        self.shape
    }

    /// Returns the deterministic storage report built with the pages.
    pub fn report(&self) -> &ChunkPagedSparseStorageReport {
        &self.report
    }

    /// Returns the number of occupied chunk pages.
    pub fn page_count(&self) -> usize {
        self.pages.len()
    }

    /// Returns the number of explicitly stored non-empty cells.
    pub fn len(&self) -> usize {
        self.report.summary.stored_cells
    }

    /// Returns whether no non-empty cells are explicitly stored.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Looks up one exact address, returning exact empty when the page or cell
    /// is absent.
    pub fn get(&self, address: VoxelAddress) -> HypervoxelResult<VoxelCell> {
        validate_address_in_frame(address, &self.frame)?;
        let chunk = ChunkAddress::containing(address, self.shape);
        Ok(self
            .pages
            .get(&chunk)
            .and_then(|page| page.cells.get(&address))
            .copied()
            .unwrap_or_else(VoxelCell::empty))
    }

    /// Returns a page by exact chunk address.
    pub fn page(&self, chunk: ChunkAddress) -> Option<&ChunkPagedSparsePage> {
        self.pages.get(&chunk)
    }

    /// Iterates over occupied pages in deterministic order.
    pub fn pages(&self) -> impl Iterator<Item = (&ChunkAddress, &ChunkPagedSparsePage)> {
        self.pages.iter()
    }
}

impl ChunkPagedSparsePage {
    fn new(chunk: ChunkAddress) -> Self {
        Self {
            chunk,
            cells: BTreeMap::new(),
            locals: BTreeMap::new(),
        }
    }

    /// Returns explicit cells in this page in deterministic address order.
    pub fn iter(&self) -> impl Iterator<Item = (&VoxelAddress, &VoxelCell)> {
        self.cells.iter()
    }

    /// Returns the number of explicit cells in this page.
    pub fn len(&self) -> usize {
        self.cells.len()
    }

    /// Returns whether this page has no explicit cells.
    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    /// Returns exact replay evidence for this page.
    pub fn report(&self, frame: &GridFrame) -> ChunkPagedSparsePageReport {
        let mut finest_depth_cells = 0_usize;
        let mut non_finest_depth_cells = 0_usize;
        let mut local_addresses_in_bounds = true;
        let mut exact_local_recompose_ready = true;
        let mut exact_payload_replay_ready = true;
        let mut has_unknown = false;
        let mut has_lossy = false;

        for (address, cell) in &self.cells {
            if address.depth == frame.depth() {
                finest_depth_cells += 1;
            } else {
                non_finest_depth_cells += 1;
            }
            if let Some(local) = self.locals.get(address) {
                local_addresses_in_bounds &= local.local_in_bounds;
                exact_local_recompose_ready &= local.exact_recompose_ready;
            } else {
                local_addresses_in_bounds = false;
                exact_local_recompose_ready = false;
            }
            let cell_report = cell.report();
            exact_payload_replay_ready &= cell_report.exact_cell_evidence_ready;
            has_unknown |= cell_report.has_unknown;
            has_lossy |= cell_report.has_lossy;
        }

        let aggregate = VoxelAggregateFacts::from_cells(self.cells.values());
        let exact_page_replay_ready = !self.cells.is_empty()
            && local_addresses_in_bounds
            && exact_local_recompose_ready
            && exact_payload_replay_ready
            && !has_unknown
            && !has_lossy;
        ChunkPagedSparsePageReport {
            chunk: self.chunk,
            stored_cells: self.cells.len(),
            finest_depth_cells,
            non_finest_depth_cells,
            local_addresses_in_bounds,
            exact_local_recompose_ready,
            exact_payload_replay_ready,
            has_unknown,
            has_lossy,
            aggregate,
            exact_page_replay_ready,
        }
    }
}

fn validate_address_in_frame(address: VoxelAddress, frame: &GridFrame) -> HypervoxelResult<()> {
    if address.depth > frame.depth() {
        return Err(HypervoxelError::DepthOutsideFrame {
            depth: address.depth,
            frame_depth: frame.depth(),
        });
    }
    Ok(())
}
