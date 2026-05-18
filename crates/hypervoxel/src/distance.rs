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
    /// Samples in deterministic address order.
    pub samples: Vec<DistanceSample>,
}

/// Signed distance-field preview over a query region.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SignedDistanceFieldPreview {
    /// Region sampled.
    pub region: QueryRegion,
    /// Samples in deterministic address order.
    pub samples: Vec<SignedDistanceSample>,
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
    Ok(DistanceFieldPreview {
        region,
        samples: samples.into_values().collect(),
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
    Ok(SignedDistanceFieldPreview { region, samples })
}

fn manhattan(left: [u64; 3], right: [u64; 3]) -> u64 {
    left[0].abs_diff(right[0]) + left[1].abs_diff(right[1]) + left[2].abs_diff(right[2])
}
