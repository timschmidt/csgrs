//! Prepared query helpers for semantic sparse voxel grids.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use hyperlimit::{
    Aabb3Intersection, PredicateOutcome, classify_aabb3_intersection, geometry::Point3,
};

use crate::{
    AggregateCertainty, ExactAabb3, FreshnessStatus, GridAabbHandoff, OccupancyState,
    PreparedVoxelGrid, SparseVoxelGrid, VoxelAddress, VoxelAggregateFacts, VoxelCell,
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
    /// unknown and lossy adapter cells remain non-ready evidence. This follows
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997: exact consumers must see undecided or approximate object
    /// facts instead of treating them as topology.
    pub exact_cell_evidence_ready: bool,
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
    /// component evidence. Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997, frames exactness at the object
    /// level; this flag prevents an empty traversal from masquerading as a
    /// certified connected object.
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
    /// not usable as exact distance evidence for a retained object. The
    /// non-vacuous evidence gate follows Yap's exact-object distinction, while
    /// the lattice distance itself follows Rosenfeld and Pfaltz, "Distance
    /// functions on digital pictures," *Pattern Recognition* 1(1), 1968.
    pub has_reached_cells: bool,
    /// Whether the band is exact address-space distance evidence.
    ///
    /// The traversal is a discrete Manhattan metric over the 6-neighbor voxel
    /// lattice, following the digital-distance role introduced by Rosenfeld
    /// and Pfaltz, "Distance functions on digital pictures," *Pattern
    /// Recognition* 1(1), 1968. The result is exact-ready only when no reached
    /// cell carries unknown or lossy occupancy.
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
/// `hyperlimit::classify_aabb3_intersection`, whose 3D AABB classifier follows
/// Yap's exact-geometric-computation boundary and the broad-phase interval role
/// used by Bentley and Ottmann, "Algorithms for Reporting and Counting
/// Geometric Intersections," *IEEE Transactions on Computers* C-28.9 (1979).
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
    /// not evidence that any retained object relation was certified. Keeping
    /// this bit separate follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997: exact consumers should see the
    /// object-level evidence boundary rather than infer it from vacuous counts.
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
    /// object relation and every tested cell has a decided AABB relation. This
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997, and the broad-phase/narrow-phase split used by
    /// Bentley and Ottmann, "Algorithms for Reporting and Counting Geometric
    /// Intersections," *IEEE Transactions on Computers* C-28.9 (1979):
    /// undecided or absent relation evidence remains explicit rather than
    /// being promoted by convention.
    pub certified_broad_phase_ready: bool,
}

impl AabbBroadPhaseQuery {
    /// Returns whether the broad-phase pass non-vacuously certified every stored cell.
    pub fn is_fully_decided(&self) -> bool {
        self.certified_broad_phase_ready
    }
}

/// Prepared acceleration evidence retained beside a sparse query handle.
///
/// This report intentionally describes prepared *facts* rather than promising a
/// floating-point acceleration structure. Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 1997, separates numerical
/// approximation from exact object-level predicates; here that means an AABB
/// handoff, Morton order, or predicate replay cache is only usable as exact
/// evidence when its provenance says so. The retained report frame is checked
/// against the prepared grid frame so a reused cache cannot borrow freshness
/// from a different object.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedQueryReport {
    /// Number of explicitly stored cells covered by the prepared handle.
    pub stored_cells: usize,
    /// Number of explicitly stored non-empty cells.
    pub non_empty_cells: usize,
    /// Whether at least one non-empty cell produced query evidence.
    ///
    /// Prepared predicate replay can make a cache reusable, but it is not by
    /// itself a voxel query result. Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997, requires exact
    /// object claims to remain tied to represented object evidence; this flag
    /// prevents an empty prepared handle from becoming exact evidence through a
    /// vacuous aggregate.
    pub has_query_evidence: bool,
    /// Exact aggregate facts retained with the prepared handle.
    pub aggregate: VoxelAggregateFacts,
    /// Freshness of the optional voxelization/import report.
    pub freshness: FreshnessStatus,
    /// Whether the retained report was built for this prepared grid frame.
    pub report_frame_matches: bool,
    /// Exact AABB handoffs for explicitly stored non-empty cells.
    pub aabb_handoffs: Vec<GridAabbHandoff>,
    /// Whether the prepared handle can replay exact source predicates.
    pub predicate_replay_available: bool,
    /// Number of prepared cache entries that can be reused without changing semantics.
    pub cache_entries: usize,
    /// Conservative estimate of how many exact cell reads the cache can avoid.
    pub estimated_saved_cell_reads: usize,
    /// Whether this prepared handle can be consumed as exact query evidence.
    ///
    /// Prepared caches are only evidence when their source report is current,
    /// the retained frame matches, exact predicate replay is available, and the
    /// aggregate packet contains non-empty exact cell evidence with no unknown
    /// or lossy state. That mirrors Yap's exact-object boundary: a cache can
    /// save work, but it cannot repair stale provenance, uncertified topology,
    /// or absent query evidence.
    pub exact_query_evidence_ready: bool,
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
        let has_query_evidence = !non_empty.is_empty();

        let (freshness, report_frame_matches) =
            self.report
                .as_ref()
                .map_or((FreshnessStatus::Unknown, false), |report| {
                    if report.frame == self.frame {
                        (report.freshness(), true)
                    } else {
                        (FreshnessStatus::Stale, false)
                    }
                });

        let exact_query_evidence_ready = freshness == FreshnessStatus::Current
            && has_query_evidence
            && report_frame_matches
            && predicate_replay_available
            && self.aggregate.certainty == AggregateCertainty::Exact
            && !self.aggregate.has_unknown
            && !self.aggregate.has_lossy;

        Ok(PreparedQueryReport {
            stored_cells,
            non_empty_cells: non_empty.len(),
            has_query_evidence,
            aggregate: self.aggregate.clone(),
            freshness,
            report_frame_matches,
            aabb_handoffs,
            predicate_replay_available,
            cache_entries,
            estimated_saved_cell_reads,
            exact_query_evidence_ready,
        })
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

        let exact_distance_band_ready = distances
            .keys()
            .copied()
            .map(|address| self.storage.get(address))
            .collect::<crate::HypervoxelResult<Vec<_>>>()?
            .into_iter()
            .all(exact_cell_ready);

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
