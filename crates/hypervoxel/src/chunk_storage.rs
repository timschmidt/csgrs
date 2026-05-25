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

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use hyperlimit::{PredicateOutcome, classify_aabb3_intersection};

use crate::query::voxel_neighbors6;
use crate::{
    AabbBroadPhaseCandidate, AabbBroadPhaseQuery, ChunkAddress, ChunkLocalAddress,
    ChunkPageSummary, ChunkShape, ExactAabb3, GridFrame, HypervoxelError, HypervoxelResult,
    OccupancyState, QueryRegion, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts, VoxelCell,
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

/// Exact page-pruned region aggregate query report.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPagedRegionAggregateReport {
    /// Queried integer address region.
    pub region: QueryRegion,
    /// Number of occupied pages inspected by the page filter.
    pub tested_pages: usize,
    /// Pages certified disjoint from the region by integer page bounds.
    pub rejected_pages: usize,
    /// Pages whose integer bounds may overlap the region.
    pub candidate_pages: usize,
    /// Candidate pages whose depth differs from the query depth and therefore
    /// could not be rejected by same-depth page bounds.
    pub cross_depth_candidate_pages: usize,
    /// Explicit cells tested after page filtering.
    pub tested_cells: usize,
    /// Explicit cells whose addresses are inside the query region.
    pub matched_cells: usize,
    /// Whether all page decisions were exact same-depth integer range tests.
    pub exact_page_filter_ready: bool,
    /// Aggregate facts for matched explicit cells.
    pub aggregate: VoxelAggregateFacts,
    /// Whether this query has non-vacuous exact page/storage evidence.
    pub exact_region_query_ready: bool,
}

/// Exact page-pruned AABB broad-phase report.
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkPagedAabbBroadPhaseReport {
    /// Query AABB.
    pub query: ExactAabb3,
    /// Occupied pages tested by the page AABB filter.
    pub tested_pages: usize,
    /// Pages certified disjoint from the query AABB.
    pub rejected_pages: usize,
    /// Pages whose exact AABB intersects or touches the query AABB.
    pub candidate_pages: usize,
    /// Pages whose exact page/query relation was undecided.
    pub unknown_pages: usize,
    /// Exact per-cell broad-phase result after page pruning.
    pub cells: AabbBroadPhaseQuery,
    /// Whether page-level pruning was fully certified.
    pub exact_page_filter_ready: bool,
    /// Whether both page-level and cell-level broad phase evidence are exact
    /// and non-vacuous.
    pub exact_paged_broad_phase_ready: bool,
}

/// Exact connected-component traversal report over chunk-paged storage.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPagedConnectedComponentReport {
    /// Seed address.
    pub seed: VoxelAddress,
    /// Reached explicit non-empty addresses in deterministic order.
    pub addresses: Vec<VoxelAddress>,
    /// Whether the seed reached at least one non-empty cell.
    pub has_reached_cells: bool,
    /// Number of neighbor edges tested by the traversal.
    pub neighbor_edges: usize,
    /// Number of neighbor checks whose target page existed.
    pub page_hits: usize,
    /// Number of neighbor checks whose target page was absent.
    pub page_misses: usize,
    /// Number of neighbor checks that crossed an integer page boundary.
    pub cross_page_edges: usize,
    /// Number of in-page or candidate-page neighbors that were exact empty.
    pub empty_neighbors: usize,
    /// Whether any reached cell carries explicit unknown evidence.
    pub has_unknown: bool,
    /// Whether any reached cell came from a lossy adapter.
    pub has_lossy: bool,
    /// Aggregate facts over reached cells.
    pub aggregate: VoxelAggregateFacts,
    /// Whether this traversal is exact connected-component evidence.
    pub exact_component_ready: bool,
}

/// Exact bounded Manhattan-distance traversal over chunk-paged storage.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChunkPagedManhattanBandReport {
    /// Seed address.
    pub seed: VoxelAddress,
    /// Maximum six-neighbor distance explored.
    pub max_distance: u32,
    /// Reached explicit non-empty addresses and exact lattice distances.
    pub distances: BTreeMap<VoxelAddress, u32>,
    /// Whether the seed reached at least one non-empty cell.
    pub has_reached_cells: bool,
    /// Number of neighbor edges tested by the traversal.
    pub neighbor_edges: usize,
    /// Number of neighbor checks whose target page existed.
    pub page_hits: usize,
    /// Number of neighbor checks whose target page was absent.
    pub page_misses: usize,
    /// Number of neighbor checks that crossed an integer page boundary.
    pub cross_page_edges: usize,
    /// Number of in-page or candidate-page neighbors that were exact empty.
    pub empty_neighbors: usize,
    /// Whether any reached cell carries explicit unknown evidence.
    pub has_unknown: bool,
    /// Whether any reached cell came from a lossy adapter.
    pub has_lossy: bool,
    /// Aggregate facts over reached cells.
    pub aggregate: VoxelAggregateFacts,
    /// Whether this traversal is exact bounded Manhattan-distance evidence.
    pub exact_distance_band_ready: bool,
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

    /// Returns aggregate facts for explicitly stored cells in a query region,
    /// using exact integer page bounds to skip disjoint pages.
    ///
    /// The page filter is an acceleration stage only. It may prove a page
    /// disjoint from a same-depth [`QueryRegion`], but aggregate membership is
    /// still decided by the exact [`QueryRegion::contains`] address predicate
    /// for every candidate cell. This is the storage-query analogue of Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997: the optimized representation proposes less work, while
    /// retained integer addresses decide the object facts. The hierarchical
    /// page layout follows the spatial subdivision role described by Samet,
    /// *The Design and Analysis of Spatial Data Structures*, Addison-Wesley,
    /// 1990, but without floating bounding boxes or tolerance predicates.
    pub fn query_region_aggregate(
        &self,
        region: &QueryRegion,
    ) -> HypervoxelResult<ChunkPagedRegionAggregateReport> {
        if region.depth > self.frame.depth() {
            return Err(HypervoxelError::DepthOutsideFrame {
                depth: region.depth,
                frame_depth: self.frame.depth(),
            });
        }

        let mut tested_pages = 0_usize;
        let mut rejected_pages = 0_usize;
        let mut candidate_pages = 0_usize;
        let mut cross_depth_candidate_pages = 0_usize;
        let mut tested_cells = 0_usize;
        let mut matched_cells = 0_usize;
        let mut matched = Vec::new();

        for page in self.pages.values() {
            tested_pages += 1;
            match page_relation_to_region(page.chunk, self.shape, region) {
                PageRegionRelation::Disjoint => {
                    rejected_pages += 1;
                    continue;
                }
                PageRegionRelation::Candidate { cross_depth } => {
                    candidate_pages += 1;
                    cross_depth_candidate_pages += usize::from(cross_depth);
                }
            }

            for (address, cell) in &page.cells {
                tested_cells += 1;
                if region.contains(*address) {
                    matched_cells += 1;
                    matched.push(cell);
                }
            }
        }

        let exact_page_filter_ready = cross_depth_candidate_pages == 0;
        let aggregate = VoxelAggregateFacts::from_cells(matched);
        let exact_region_query_ready = self.report.exact_chunk_storage_ready
            && exact_page_filter_ready
            && matched_cells > 0
            && !aggregate.has_unknown
            && !aggregate.has_lossy;
        Ok(ChunkPagedRegionAggregateReport {
            region: region.clone(),
            tested_pages,
            rejected_pages,
            candidate_pages,
            cross_depth_candidate_pages,
            tested_cells,
            matched_cells,
            exact_page_filter_ready,
            aggregate,
            exact_region_query_ready,
        })
    }

    /// Returns exact AABB broad-phase candidates using page AABBs first.
    ///
    /// Page AABBs are exact constructions from integer chunk/page coordinates
    /// and [`GridFrame`] cell bounds. A page certified disjoint from the query
    /// is skipped; intersecting and undecided pages are scanned at the
    /// retained-cell level, where each stored cell is still classified by its
    /// exact cell AABB. This is a report-bearing broad/narrow split in the
    /// spirit of Bentley and Ottmann, "Algorithms for Reporting and Counting
    /// Geometric Intersections," *IEEE Transactions on Computers* C-28.9
    /// (1979), but with Yap's exact-geometric-computation discipline: page
    /// pruning can reduce work, never decide topology by tolerance.
    pub fn query_aabb_broad_phase(
        &self,
        query: &ExactAabb3,
    ) -> HypervoxelResult<ChunkPagedAabbBroadPhaseReport> {
        let query_min = point3(&query.min);
        let query_max = point3(&query.max);
        let mut tested_pages = 0_usize;
        let mut rejected_pages = 0_usize;
        let mut candidate_pages = 0_usize;
        let mut unknown_pages = 0_usize;
        let mut tested_cells = 0_usize;
        let mut candidates = Vec::new();
        let mut rejected_addresses = Vec::new();
        let mut unknown_addresses = Vec::new();

        for page in self.pages.values() {
            tested_pages += 1;
            let page_bounds = page_aabb(page.chunk, self.shape, &self.frame)?;
            match classify_aabb3_intersection(
                &query_min,
                &query_max,
                &point3(&page_bounds.min),
                &point3(&page_bounds.max),
            ) {
                PredicateOutcome::Decided { value, .. } if !value.intersects() => {
                    rejected_pages += 1;
                    continue;
                }
                PredicateOutcome::Decided { .. } => candidate_pages += 1,
                PredicateOutcome::Unknown { .. } => unknown_pages += 1,
            }

            for (address, _) in &page.cells {
                tested_cells += 1;
                let bounds = ExactAabb3::from(address.bounds(&self.frame)?);
                match classify_aabb3_intersection(
                    &query_min,
                    &query_max,
                    &point3(&bounds.min),
                    &point3(&bounds.max),
                ) {
                    PredicateOutcome::Decided { value, .. } if value.intersects() => {
                        candidates.push(AabbBroadPhaseCandidate {
                            address: *address,
                            bounds,
                            relation: value,
                        });
                    }
                    PredicateOutcome::Decided { .. } => rejected_addresses.push(*address),
                    PredicateOutcome::Unknown { .. } => unknown_addresses.push(*address),
                }
            }
        }

        let cells = AabbBroadPhaseQuery {
            query: query.clone(),
            tested_cells,
            has_tested_cells: tested_cells > 0,
            certified_broad_phase_ready: tested_cells > 0 && unknown_addresses.is_empty(),
            candidates,
            rejected_addresses,
            unknown_addresses,
        };
        let exact_page_filter_ready = unknown_pages == 0;
        let exact_paged_broad_phase_ready =
            exact_page_filter_ready && cells.certified_broad_phase_ready;
        Ok(ChunkPagedAabbBroadPhaseReport {
            query: query.clone(),
            tested_pages,
            rejected_pages,
            candidate_pages,
            unknown_pages,
            cells,
            exact_page_filter_ready,
            exact_paged_broad_phase_ready,
        })
    }

    /// Returns the six-connected component of explicit non-empty cells.
    ///
    /// This is the chunk-paged counterpart to prepared sparse-grid component
    /// queries. The traversal uses exact integer 6-neighbor adjacency and page
    /// membership only as a storage shortcut: a missing page proves all
    /// explicit sparse cells in that page are absent, while present pages still
    /// require exact address lookup. The lattice connectivity model follows
    /// Rosenfeld and Pfaltz, "Sequential Operations in Digital Picture
    /// Processing," *JACM* 13(4), 1966. As in Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997, unknown and lossy
    /// cells remain explicit blockers instead of being coerced into exact
    /// topology evidence.
    pub fn query_connected_component(
        &self,
        seed: VoxelAddress,
    ) -> HypervoxelResult<ChunkPagedConnectedComponentReport> {
        validate_address_in_frame(seed, &self.frame)?;
        let mut page_hits = 0_usize;
        let mut page_misses = 0_usize;
        let seed_cell = self.get_with_page_probe(seed, &mut page_hits, &mut page_misses)?;
        if seed_cell.occupancy == OccupancyState::Empty {
            return Ok(ChunkPagedConnectedComponentReport {
                seed,
                addresses: Vec::new(),
                has_reached_cells: false,
                neighbor_edges: 0,
                page_hits,
                page_misses,
                cross_page_edges: 0,
                empty_neighbors: 0,
                has_unknown: false,
                has_lossy: false,
                aggregate: VoxelAggregateFacts::from_cells(std::iter::empty::<&VoxelCell>()),
                exact_component_ready: false,
            });
        }

        let mut seen = BTreeSet::new();
        let mut reached = BTreeMap::new();
        let mut queue = VecDeque::new();
        let mut neighbor_edges = 0_usize;
        let mut cross_page_edges = 0_usize;
        let mut empty_neighbors = 0_usize;
        let mut has_unknown = false;
        let mut has_lossy = false;

        seen.insert(seed);
        queue.push_back(seed);
        reached.insert(seed, seed_cell);

        while let Some(address) = queue.pop_front() {
            let source_page = ChunkAddress::containing(address, self.shape);
            for neighbor in voxel_neighbors6(address) {
                neighbor_edges += 1;
                if ChunkAddress::containing(neighbor, self.shape) != source_page {
                    cross_page_edges += 1;
                }
                if !seen.insert(neighbor) {
                    continue;
                }
                let cell = self.get_with_page_probe(neighbor, &mut page_hits, &mut page_misses)?;
                if cell.occupancy == OccupancyState::Empty {
                    empty_neighbors += 1;
                    continue;
                }
                let cell_report = cell.report();
                has_unknown |= cell_report.has_unknown;
                has_lossy |= cell_report.has_lossy;
                reached.insert(neighbor, cell);
                queue.push_back(neighbor);
            }
        }

        let aggregate = VoxelAggregateFacts::from_cells(reached.values());
        let has_reached_cells = !reached.is_empty();
        let exact_component_ready = has_reached_cells
            && self.report.exact_chunk_storage_ready
            && !has_unknown
            && !has_lossy
            && aggregate.certainty != crate::AggregateCertainty::Unknown
            && aggregate.certainty != crate::AggregateCertainty::Lossy;
        Ok(ChunkPagedConnectedComponentReport {
            seed,
            addresses: reached.keys().copied().collect(),
            has_reached_cells,
            neighbor_edges,
            page_hits,
            page_misses,
            cross_page_edges,
            empty_neighbors,
            has_unknown,
            has_lossy,
            aggregate,
            exact_component_ready,
        })
    }

    /// Returns a bounded six-connected Manhattan-distance band over explicit
    /// non-empty cells.
    ///
    /// The metric is exact integer graph distance on the 6-neighbor voxel
    /// lattice, matching Rosenfeld and Pfaltz, "Distance functions on digital
    /// pictures," *Pattern Recognition* 1(1), 1968. Chunk pages only
    /// accelerate absence checks; a present page does not certify a neighbor
    /// until the exact address lookup succeeds. This preserves the
    /// precision-driven model in Yap, "Towards Exact Geometric Computation,"
    /// by keeping unknown/lossy cells and storage page evidence visible in the
    /// report instead of letting an accelerator decide topology.
    pub fn query_manhattan_band(
        &self,
        seed: VoxelAddress,
        max_distance: u32,
    ) -> HypervoxelResult<ChunkPagedManhattanBandReport> {
        validate_address_in_frame(seed, &self.frame)?;
        let mut page_hits = 0_usize;
        let mut page_misses = 0_usize;
        let seed_cell = self.get_with_page_probe(seed, &mut page_hits, &mut page_misses)?;
        if seed_cell.occupancy == OccupancyState::Empty {
            return Ok(ChunkPagedManhattanBandReport {
                seed,
                max_distance,
                distances: BTreeMap::new(),
                has_reached_cells: false,
                neighbor_edges: 0,
                page_hits,
                page_misses,
                cross_page_edges: 0,
                empty_neighbors: 0,
                has_unknown: false,
                has_lossy: false,
                aggregate: VoxelAggregateFacts::from_cells(std::iter::empty::<&VoxelCell>()),
                exact_distance_band_ready: false,
            });
        }

        let mut seen = BTreeSet::new();
        let mut distances = BTreeMap::new();
        let mut reached = BTreeMap::new();
        let mut queue = VecDeque::new();
        let mut neighbor_edges = 0_usize;
        let mut cross_page_edges = 0_usize;
        let mut empty_neighbors = 0_usize;
        let mut has_unknown = seed_cell.report().has_unknown;
        let mut has_lossy = seed_cell.report().has_lossy;

        seen.insert(seed);
        distances.insert(seed, 0);
        reached.insert(seed, seed_cell);
        queue.push_back((seed, 0_u32));

        while let Some((address, distance)) = queue.pop_front() {
            if distance == max_distance {
                continue;
            }
            let source_page = ChunkAddress::containing(address, self.shape);
            for neighbor in voxel_neighbors6(address) {
                neighbor_edges += 1;
                if ChunkAddress::containing(neighbor, self.shape) != source_page {
                    cross_page_edges += 1;
                }
                if !seen.insert(neighbor) {
                    continue;
                }
                let cell = self.get_with_page_probe(neighbor, &mut page_hits, &mut page_misses)?;
                if cell.occupancy == OccupancyState::Empty {
                    empty_neighbors += 1;
                    continue;
                }
                let next_distance = distance.saturating_add(1);
                let cell_report = cell.report();
                has_unknown |= cell_report.has_unknown;
                has_lossy |= cell_report.has_lossy;
                distances.insert(neighbor, next_distance);
                reached.insert(neighbor, cell);
                queue.push_back((neighbor, next_distance));
            }
        }

        let aggregate = VoxelAggregateFacts::from_cells(reached.values());
        let has_reached_cells = !distances.is_empty();
        let exact_distance_band_ready = has_reached_cells
            && self.report.exact_chunk_storage_ready
            && !has_unknown
            && !has_lossy
            && aggregate.certainty != crate::AggregateCertainty::Unknown
            && aggregate.certainty != crate::AggregateCertainty::Lossy;
        Ok(ChunkPagedManhattanBandReport {
            seed,
            max_distance,
            distances,
            has_reached_cells,
            neighbor_edges,
            page_hits,
            page_misses,
            cross_page_edges,
            empty_neighbors,
            has_unknown,
            has_lossy,
            aggregate,
            exact_distance_band_ready,
        })
    }

    fn get_with_page_probe(
        &self,
        address: VoxelAddress,
        page_hits: &mut usize,
        page_misses: &mut usize,
    ) -> HypervoxelResult<VoxelCell> {
        validate_address_in_frame(address, &self.frame)?;
        let chunk = ChunkAddress::containing(address, self.shape);
        if let Some(page) = self.pages.get(&chunk) {
            *page_hits += 1;
            Ok(page
                .cells
                .get(&address)
                .copied()
                .unwrap_or_else(VoxelCell::empty))
        } else {
            *page_misses += 1;
            Ok(VoxelCell::empty())
        }
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

fn page_aabb(
    chunk: ChunkAddress,
    shape: ChunkShape,
    frame: &GridFrame,
) -> HypervoxelResult<ExactAabb3> {
    if chunk.depth > frame.depth() {
        return Err(HypervoxelError::DepthOutsideFrame {
            depth: chunk.depth,
            frame_depth: frame.depth(),
        });
    }
    let shift = shape.log2_cells.min(chunk.depth);
    let extent = 1_u64 << shift;
    let min_xyz = [
        chunk.xyz[0] << shift,
        chunk.xyz[1] << shift,
        chunk.xyz[2] << shift,
    ];
    let max_xyz = [
        min_xyz[0] + extent - 1,
        min_xyz[1] + extent - 1,
        min_xyz[2] + extent - 1,
    ];
    let min_bounds = VoxelAddress::new(chunk.depth, min_xyz)?.bounds(frame)?;
    let max_bounds = VoxelAddress::new(chunk.depth, max_xyz)?.bounds(frame)?;
    Ok(ExactAabb3 {
        min: min_bounds.min,
        max: max_bounds.max,
    })
}

fn point3(values: &[hyperreal::Real; 3]) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(values[0].clone(), values[1].clone(), values[2].clone())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PageRegionRelation {
    Disjoint,
    Candidate { cross_depth: bool },
}

fn page_relation_to_region(
    chunk: ChunkAddress,
    shape: ChunkShape,
    region: &QueryRegion,
) -> PageRegionRelation {
    if chunk.depth != region.depth {
        return PageRegionRelation::Candidate { cross_depth: true };
    }

    let shift = shape.log2_cells.min(chunk.depth);
    let extent = 1_u64 << shift;
    let page_min = [
        chunk.xyz[0] << shift,
        chunk.xyz[1] << shift,
        chunk.xyz[2] << shift,
    ];
    let page_max = [
        page_min[0] + extent - 1,
        page_min[1] + extent - 1,
        page_min[2] + extent - 1,
    ];

    if (0..3).any(|axis| page_max[axis] < region.min[axis] || page_min[axis] > region.max[axis]) {
        PageRegionRelation::Disjoint
    } else {
        PageRegionRelation::Candidate { cross_depth: false }
    }
}
