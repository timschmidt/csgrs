//! Exact address-space distance-field previews.
//!
//! These routines are preview/query helpers, not continuous signed-distance
//! geometry. They compute integer Manhattan distances on the voxel lattice so
//! the result is an exact combinatorial field. Continuous SDF export can be
//! added later as a named lossy or certified adapter. The separation follows
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7(1-2), 1997.

use std::collections::BTreeMap;

use crate::{OccupancyState, QueryRegion, SparseVoxelGrid, VoxelAddress};

/// Exact integer distance sample for one voxel address.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DistanceSample {
    /// Sampled address.
    pub address: VoxelAddress,
    /// Exact Manhattan distance to the nearest explicitly stored non-empty cell.
    pub manhattan_distance: Option<u64>,
    /// Whether the sampled address itself is stored non-empty.
    pub occupied: bool,
}

/// Exact signed integer distance sample.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SignedDistanceSample {
    /// Sampled address.
    pub address: VoxelAddress,
    /// Signed Manhattan distance. Occupied cells are non-positive, empty cells are non-negative.
    pub signed_manhattan_distance: Option<i64>,
    /// Whether the sampled address itself is stored non-empty.
    pub occupied: bool,
}

/// Distance-field preview over a query region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DistanceFieldPreview {
    /// Region sampled.
    pub region: QueryRegion,
    /// Number of explicitly stored non-empty source cells used as distance sites.
    pub source_cells: usize,
    /// Whether the preview has at least one source cell to measure from.
    ///
    /// A distance transform without sites is a well-defined empty search
    /// result, but it is not exact distance evidence for downstream planning.
    /// Keeping that distinction explicit follows Rosenfeld and Pfaltz,
    /// "Distance functions on digital pictures," *Pattern Recognition* 1(1),
    /// 1968, and Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997.
    pub has_distance_source: bool,
    /// Samples in deterministic address order.
    pub samples: Vec<DistanceSample>,
    /// Whether all samples are exact address-space distance evidence.
    ///
    /// This is not a continuous SDF certificate. It only certifies the integer
    /// Manhattan preview over known voxel occupancy, following Rosenfeld and
    /// Pfaltz, "Distance functions on digital pictures," *Pattern Recognition*
    /// 1(1), 1968, and Yap's exact-geometric-computation rule that approximate
    /// geometry must not be smuggled into exact topology.
    pub exact_address_distance_ready: bool,
}

/// Signed distance-field preview over a query region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SignedDistanceFieldPreview {
    /// Region sampled.
    pub region: QueryRegion,
    /// Number of explicitly stored non-empty source cells used as distance sites.
    pub source_cells: usize,
    /// Whether the preview has at least one source cell to measure from.
    pub has_distance_source: bool,
    /// Samples in deterministic address order.
    pub samples: Vec<SignedDistanceSample>,
    /// Whether all signed samples are exact address-space distance evidence.
    pub exact_address_distance_ready: bool,
    /// Whether this preview may be consumed as a continuous signed-distance field.
    ///
    /// This is always false for the current integer-lattice helper; continuous
    /// SDFs must enter through a named preview/export adapter report.
    pub continuous_sdf_ready: bool,
}

/// Samples exact address-space Manhattan distances in a query region.
pub fn sample_manhattan_distance_field(
    grid: &SparseVoxelGrid,
    region: QueryRegion,
) -> crate::HypervoxelResult<DistanceFieldPreview> {
    let occupied = grid
        .iter()
        .filter(|(_, cell)| cell.occupancy != OccupancyState::Empty)
        .map(|(address, _)| *address)
        .collect::<Vec<_>>();
    let mut samples = BTreeMap::new();
    for z in region.min[2]..=region.max[2] {
        for y in region.min[1]..=region.max[1] {
            for x in region.min[0]..=region.max[0] {
                let address = VoxelAddress::new(region.depth, [x, y, z])?;
                let occupied_here = grid.get(address)?.occupancy != OccupancyState::Empty;
                let manhattan_distance = occupied
                    .iter()
                    .filter(|candidate| candidate.depth == address.depth)
                    .map(|candidate| manhattan(address.xyz, candidate.xyz))
                    .min();
                samples.insert(
                    address,
                    DistanceSample {
                        address,
                        manhattan_distance,
                        occupied: occupied_here,
                    },
                );
            }
        }
    }
    let source_cells = occupied.len();
    let has_distance_source = source_cells > 0;
    let exact_address_distance_ready = has_distance_source
        && grid
            .iter()
            .all(|(_, cell)| exact_distance_source_ready(cell.occupancy));

    Ok(DistanceFieldPreview {
        region,
        source_cells,
        has_distance_source,
        samples: samples.into_values().collect(),
        exact_address_distance_ready,
    })
}

/// Samples signed address-space Manhattan distances in a query region.
///
/// This is still an integer lattice preview: it is useful for masks and
/// process fixtures, but it is not a continuous signed distance field.
pub fn sample_signed_manhattan_distance_field(
    grid: &SparseVoxelGrid,
    region: QueryRegion,
) -> crate::HypervoxelResult<SignedDistanceFieldPreview> {
    let unsigned = sample_manhattan_distance_field(grid, region.clone())?;
    let exact_address_distance_ready = unsigned.exact_address_distance_ready;
    let samples = unsigned
        .samples
        .into_iter()
        .map(|sample| SignedDistanceSample {
            address: sample.address,
            signed_manhattan_distance: sample.manhattan_distance.map(|distance| {
                if sample.occupied {
                    -(distance as i64)
                } else {
                    distance as i64
                }
            }),
            occupied: sample.occupied,
        })
        .collect();
    Ok(SignedDistanceFieldPreview {
        region,
        source_cells: unsigned.source_cells,
        has_distance_source: unsigned.has_distance_source,
        samples,
        exact_address_distance_ready,
        continuous_sdf_ready: false,
    })
}

fn manhattan(left: [u64; 3], right: [u64; 3]) -> u64 {
    left[0].abs_diff(right[0]) + left[1].abs_diff(right[1]) + left[2].abs_diff(right[2])
}

fn exact_distance_source_ready(occupancy: OccupancyState) -> bool {
    !matches!(
        occupancy,
        OccupancyState::Unknown | OccupancyState::LossyAdapterValue
    )
}
