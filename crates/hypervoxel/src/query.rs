//! Prepared query helpers for semantic sparse voxel grids.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use crate::{
    FreshnessStatus, GridAabbHandoff, OccupancyState, PreparedVoxelGrid, SparseVoxelGrid,
    VoxelAddress, VoxelAggregateFacts, VoxelCell,
};

/// Occupancy query result for one address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OccupancyQuery {
    /// Queried address.
    pub address: VoxelAddress,
    /// Cell at that address, defaulting to exact empty when absent.
    pub cell: VoxelCell,
}

/// Coarse query region used by initial prepared APIs.
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
}

/// Connected component over explicitly stored, non-empty sparse cells.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConnectedComponentQuery {
    /// Seed address.
    pub seed: VoxelAddress,
    /// Reached addresses in deterministic order.
    pub addresses: Vec<VoxelAddress>,
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
}

/// Prepared acceleration evidence retained beside a sparse query handle.
///
/// This report intentionally describes prepared *facts* rather than promising a
/// floating-point acceleration structure. Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 1997, separates numerical
/// approximation from exact object-level predicates; here that means an AABB
/// handoff, Morton order, or predicate replay cache is only usable as exact
/// evidence when its provenance says so.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedQueryReport {
    /// Number of explicitly stored cells covered by the prepared handle.
    pub stored_cells: usize,
    /// Number of explicitly stored non-empty cells.
    pub non_empty_cells: usize,
    /// Exact aggregate facts retained with the prepared handle.
    pub aggregate: VoxelAggregateFacts,
    /// Freshness of the optional voxelization/import report.
    pub freshness: FreshnessStatus,
    /// Exact AABB handoffs for explicitly stored non-empty cells.
    pub aabb_handoffs: Vec<GridAabbHandoff>,
    /// Whether the prepared handle can replay exact source predicates.
    pub predicate_replay_available: bool,
    /// Number of prepared cache entries that can be reused without changing semantics.
    pub cache_entries: usize,
    /// Conservative estimate of how many exact cell reads the cache can avoid.
    pub estimated_saved_cell_reads: usize,
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
/// This is a combinatorial grid operation, not a metric approximation. The
/// distinction follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry*, 1997: topological adjacency is decided from exact
/// integer addresses before any geometric export happens.
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

    /// Returns prepared-query evidence and cache-payoff accounting.
    fn prepared_query_report(
        &self,
        predicate_replay_available: bool,
    ) -> crate::HypervoxelResult<PreparedQueryReport>;

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
}

impl PreparedSparseVoxelGridExt for PreparedVoxelGrid<SparseVoxelGrid> {
    fn query_occupancy(&self, address: VoxelAddress) -> crate::HypervoxelResult<OccupancyQuery> {
        Ok(OccupancyQuery {
            address,
            cell: self.storage.get(address)?,
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

    fn prepared_query_report(
        &self,
        predicate_replay_available: bool,
    ) -> crate::HypervoxelResult<PreparedQueryReport> {
        let stored_cells = self.storage.iter().count();
        let non_empty = self.stored_non_empty_addresses();
        let aabb_handoffs = non_empty
            .iter()
            .copied()
            .map(|address| GridAabbHandoff::from_address(&self.frame, address))
            .collect::<crate::HypervoxelResult<Vec<_>>>()?;
        let cache_entries = aabb_handoffs.len() + usize::from(predicate_replay_available);
        let estimated_saved_cell_reads = cache_entries;

        Ok(PreparedQueryReport {
            stored_cells,
            non_empty_cells: non_empty.len(),
            aggregate: self.aggregate.clone(),
            freshness: self
                .report
                .as_ref()
                .map_or(FreshnessStatus::Unknown, |report| report.freshness()),
            aabb_handoffs,
            predicate_replay_available,
            cache_entries,
            estimated_saved_cell_reads,
        })
    }

    fn query_neighbors6(&self, address: VoxelAddress) -> NeighborQuery {
        NeighborQuery {
            address,
            neighbors: voxel_neighbors6(address),
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
            });
        }

        let mut seen = BTreeSet::new();
        let mut queue = VecDeque::new();
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
                if self.storage.get(neighbor)?.occupancy == OccupancyState::Empty {
                    continue;
                }
                let next_distance = distance.saturating_add(1);
                distances.insert(neighbor, next_distance);
                queue.push_back((neighbor, next_distance));
            }
        }

        Ok(ManhattanDistanceBand {
            seed,
            max_distance,
            distances,
        })
    }
}
