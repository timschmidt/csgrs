//! Prepared query helpers for semantic sparse voxel grids.

use std::collections::{BTreeMap, VecDeque};

use hyperlimit::{Aabb3Intersection, Point3, PredicateOutcome, classify_aabb3_intersection};
use rustc_hash::FxHashSet;

use crate::{
    ExactAabb3, OccupancyState, PreparedVoxelGrid, SparseVoxelGrid, VoxelAddress,
    VoxelAggregateFacts, VoxelCell,
};

/// Occupancy query result for one address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OccupancyQuery {
    /// Queried address.
    pub address: VoxelAddress,
    /// Cell at that address, defaulting to exact empty when absent.
    pub cell: VoxelCell,
    /// Whether the returned cell is exact occupancy evidence.
    ///
    /// Sparse absence means exact empty in the current grid frame, but explicit
    /// unknown and lossy adapter cells remain non-ready evidence.
    pub exact_cell_evidence_ready: bool,
}

/// Coarse query region used by prepared APIs.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QueryRegion {
    /// Minimum inclusive address.
    pub min: [u64; 3],
    /// Maximum inclusive address.
    pub max: [u64; 3],
    /// Query depth.
    pub depth: u8,
}

/// Grid-neighbor result for a six-connected lattice query.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NeighborQuery {
    /// Center address.
    pub address: VoxelAddress,
    /// Existing in-frame six-neighbor addresses in deterministic axis order.
    pub neighbors: Vec<VoxelAddress>,
    /// Whether the neighbor list is exact integer-grid adjacency evidence.
    pub exact_neighbors_ready: bool,
}

/// Connected component over explicitly stored, non-empty sparse cells.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConnectedComponentQuery {
    /// Seed address.
    pub seed: VoxelAddress,
    /// Reached addresses in deterministic order.
    pub addresses: Vec<VoxelAddress>,
    /// Whether the component contains at least one stored non-empty cell.
    ///
    /// An empty result is a precise "seed is empty" report, but it is not
    /// component evidence. This flag prevents an empty traversal from
    /// masquerading as a certified connected object.
    pub has_reached_cells: bool,
    /// Whether the component contains only exact stored cells.
    ///
    /// Unknown and lossy cells are traversable as explicit non-empty evidence
    /// for conservative reachability, but they prevent the component from
    /// being promoted to exact topology.
    pub exact_component_ready: bool,
}

/// Deterministic Manhattan-distance band over explicitly stored non-empty cells.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ManhattanDistanceBand {
    /// Seed address.
    pub seed: VoxelAddress,
    /// Maximum distance explored.
    pub max_distance: u32,
    /// Address-to-distance map in deterministic key order.
    pub distances: BTreeMap<VoxelAddress, u32>,
    /// Whether the traversal reached at least one stored non-empty cell.
    ///
    /// A zero-sized band from an empty seed is a valid query result, but it is
    /// not usable as exact distance evidence for a retained object.
    pub has_reached_cells: bool,
    /// Whether the band is exact address-space distance evidence.
    ///
    /// The traversal is a discrete Manhattan metric over the 6-neighbor voxel
    /// lattice. The result is exact-ready only when no reached cell carries
    /// unknown or lossy occupancy.
    pub exact_distance_band_ready: bool,
}

/// One exact AABB broad-phase candidate from a prepared sparse grid.
#[derive(Clone, Debug, PartialEq)]
pub struct AabbBroadPhaseCandidate {
    /// Candidate cell address.
    pub address: VoxelAddress,
    /// Exact cell bounds.
    pub bounds: ExactAabb3,
    /// Certified relation between the query AABB and this cell AABB.
    pub relation: Aabb3Intersection,
}

/// Exact AABB broad-phase report over explicitly stored non-empty cells.
///
/// This is a candidate filter, not a topology mutation. It uses
/// `hyperlimit::classify_aabb3_intersection` for exact broad-phase interval
/// classification.
/// Disjoint cells can be rejected; touching and overlapping cells remain
/// narrow-phase candidates; undecided predicate outcomes stay explicit.
#[derive(Clone, Debug, PartialEq)]
pub struct AabbBroadPhaseQuery {
    /// Query AABB.
    pub query: ExactAabb3,
    /// Number of stored non-empty cells tested by the broad-phase predicate.
    pub tested_cells: usize,
    /// Whether at least one stored non-empty cell was tested.
    ///
    /// An empty broad-phase scan is a precise empty-result report, but it is
    /// not evidence that any retained object relation was certified. Exact
    /// consumers should not infer evidence from vacuous counts.
    pub has_tested_cells: bool,
    /// Stored non-empty cells whose exact AABBs intersect the query.
    pub candidates: Vec<AabbBroadPhaseCandidate>,
    /// Stored non-empty cells certified disjoint from the query.
    pub rejected_addresses: Vec<VoxelAddress>,
    /// Stored non-empty cells whose broad-phase relation was undecided.
    pub unknown_addresses: Vec<VoxelAddress>,
    /// Whether at least one stored cell was tested and every relation was certified.
    ///
    /// Broad-phase acceleration is exact evidence only when there is a tested
    /// object relation and every tested cell has a decided AABB relation.
    /// Undecided or absent relations remain explicit.
    pub certified_broad_phase_ready: bool,
}

impl AabbBroadPhaseQuery {
    /// Returns whether the broad-phase pass non-vacuously certified every stored cell.
    pub fn is_fully_decided(&self) -> bool {
        self.certified_broad_phase_ready
    }
}

impl QueryRegion {
    /// Returns whether an address belongs to this region.
    pub fn contains(&self, address: VoxelAddress) -> bool {
        address.depth == self.depth
            && (0..3).all(|axis| {
                address.xyz[axis] >= self.min[axis] && address.xyz[axis] <= self.max[axis]
            })
    }
}

/// Returns in-frame six-neighbors for an address in deterministic axis order.
///
/// This is a combinatorial grid operation, not a metric approximation.
/// Topological adjacency is decided from exact integer addresses before any
/// geometric export.
pub fn voxel_neighbors6(address: VoxelAddress) -> Vec<VoxelAddress> {
    let cells = 1_u64 << address.depth;
    let mut neighbors = Vec::with_capacity(6);
    for (axis, delta) in [(0_usize, -1_i8), (0, 1), (1, -1), (1, 1), (2, -1), (2, 1)] {
        let mut xyz = address.xyz;
        match delta {
            -1 if xyz[axis] == 0 => continue,
            -1 => xyz[axis] -= 1,
            1 if xyz[axis] + 1 >= cells => continue,
            1 => xyz[axis] += 1,
            _ => {}
        }
        if let Ok(neighbor) = VoxelAddress::new(address.depth, xyz) {
            neighbors.push(neighbor);
        }
    }
    neighbors
}

/// Sparse-grid query extensions for prepared grids.
pub trait PreparedSparseVoxelGridExt {
    /// Queries one address.
    fn query_occupancy(&self, address: VoxelAddress) -> crate::HypervoxelResult<OccupancyQuery>;

    /// Returns aggregate facts for stored cells in a region.
    fn query_region_aggregate(
        &self,
        region: &QueryRegion,
    ) -> crate::HypervoxelResult<VoxelAggregateFacts>;

    /// Returns all explicitly stored non-empty addresses in deterministic order.
    fn stored_non_empty_addresses(&self) -> Vec<VoxelAddress>;

    /// Returns in-frame six-neighbors for one address.
    fn query_neighbors6(&self, address: VoxelAddress) -> NeighborQuery;

    /// Returns the connected component of stored non-empty cells containing `seed`.
    fn query_connected_component(
        &self,
        seed: VoxelAddress,
    ) -> crate::HypervoxelResult<ConnectedComponentQuery>;

    /// Returns a bounded six-connected Manhattan-distance band over stored non-empty cells.
    fn query_manhattan_band(
        &self,
        seed: VoxelAddress,
        max_distance: u32,
    ) -> crate::HypervoxelResult<ManhattanDistanceBand>;

    /// Returns exact AABB broad-phase candidates over stored non-empty cells.
    fn query_aabb_broad_phase(
        &self,
        query: &ExactAabb3,
    ) -> crate::HypervoxelResult<AabbBroadPhaseQuery>;
}

impl PreparedSparseVoxelGridExt for PreparedVoxelGrid<SparseVoxelGrid> {
    fn query_occupancy(&self, address: VoxelAddress) -> crate::HypervoxelResult<OccupancyQuery> {
        let cell = self.storage.get(address)?;
        Ok(OccupancyQuery {
            address,
            cell,
            exact_cell_evidence_ready: exact_cell_ready(cell),
        })
    }

    fn query_region_aggregate(
        &self,
        region: &QueryRegion,
    ) -> crate::HypervoxelResult<VoxelAggregateFacts> {
        let cells = self
            .storage
            .iter()
            .filter(|(address, _)| region.contains(**address))
            .map(|(_, cell)| cell)
            .collect::<Vec<_>>();
        Ok(VoxelAggregateFacts::from_cells(cells))
    }

    fn stored_non_empty_addresses(&self) -> Vec<VoxelAddress> {
        self.storage
            .iter()
            .filter(|(_, cell)| cell.occupancy != OccupancyState::Empty)
            .map(|(address, _)| *address)
            .collect()
    }

    fn query_neighbors6(&self, address: VoxelAddress) -> NeighborQuery {
        NeighborQuery {
            address,
            neighbors: voxel_neighbors6(address),
            exact_neighbors_ready: true,
        }
    }

    fn query_connected_component(
        &self,
        seed: VoxelAddress,
    ) -> crate::HypervoxelResult<ConnectedComponentQuery> {
        let band = self.query_manhattan_band(seed, u32::MAX)?;
        Ok(ConnectedComponentQuery {
            seed,
            addresses: band.distances.keys().copied().collect(),
            has_reached_cells: band.has_reached_cells,
            exact_component_ready: band.has_reached_cells && band.exact_distance_band_ready,
        })
    }

    fn query_manhattan_band(
        &self,
        seed: VoxelAddress,
        max_distance: u32,
    ) -> crate::HypervoxelResult<ManhattanDistanceBand> {
        let seed_cell = self.storage.get(seed)?;
        let mut distances = BTreeMap::new();
        if seed_cell.occupancy == OccupancyState::Empty {
            return Ok(ManhattanDistanceBand {
                seed,
                max_distance,
                distances,
                has_reached_cells: false,
                exact_distance_band_ready: false,
            });
        }

        let mut seen = FxHashSet::default();
        let mut queue = VecDeque::new();
        let mut exact_distance_band_ready = exact_cell_ready(seed_cell);
        seen.insert(seed);
        distances.insert(seed, 0);
        queue.push_back((seed, 0_u32));

        while let Some((address, distance)) = queue.pop_front() {
            if distance == max_distance {
                continue;
            }
            for neighbor in voxel_neighbors6(address) {
                if !seen.insert(neighbor) {
                    continue;
                }
                let cell = self.storage.get(neighbor)?;
                if cell.occupancy == OccupancyState::Empty {
                    continue;
                }
                exact_distance_band_ready &= exact_cell_ready(cell);
                let next_distance = distance.saturating_add(1);
                distances.insert(neighbor, next_distance);
                queue.push_back((neighbor, next_distance));
            }
        }

        let has_reached_cells = !distances.is_empty();

        Ok(ManhattanDistanceBand {
            seed,
            max_distance,
            has_reached_cells,
            distances,
            exact_distance_band_ready: has_reached_cells && exact_distance_band_ready,
        })
    }

    fn query_aabb_broad_phase(
        &self,
        query: &ExactAabb3,
    ) -> crate::HypervoxelResult<AabbBroadPhaseQuery> {
        let query_min = point3(&query.min);
        let query_max = point3(&query.max);
        let mut candidates = Vec::new();
        let mut rejected_addresses = Vec::new();
        let mut unknown_addresses = Vec::new();

        let addresses = self.stored_non_empty_addresses();
        let tested_cells = addresses.len();
        for address in addresses {
            let bounds = ExactAabb3::from(address.bounds(&self.frame)?);
            match classify_aabb3_intersection(
                &query_min,
                &query_max,
                &point3(&bounds.min),
                &point3(&bounds.max),
            ) {
                PredicateOutcome::Decided { value, .. } if value.intersects() => {
                    candidates.push(AabbBroadPhaseCandidate {
                        address,
                        bounds,
                        relation: value,
                    });
                }
                PredicateOutcome::Decided { .. } => rejected_addresses.push(address),
                PredicateOutcome::Unknown { .. } => unknown_addresses.push(address),
            }
        }

        Ok(AabbBroadPhaseQuery {
            query: query.clone(),
            tested_cells,
            has_tested_cells: tested_cells > 0,
            certified_broad_phase_ready: tested_cells > 0 && unknown_addresses.is_empty(),
            candidates,
            rejected_addresses,
            unknown_addresses,
        })
    }
}

fn point3(values: &[hyperreal::Real; 3]) -> Point3 {
    Point3::new(values[0].clone(), values[1].clone(), values[2].clone())
}

fn exact_cell_ready(cell: VoxelCell) -> bool {
    cell.report().exact_cell_evidence_ready
}
