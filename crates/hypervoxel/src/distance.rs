//! Exact address-space distance-field previews.
//!
//! These routines are preview/query helpers, not continuous signed-distance
//! geometry. They compute integer Manhattan distances on the voxel lattice so
//! the result is an exact combinatorial field. Continuous SDF export remains a
//! separately named lossy or certified adapter.

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
    pub has_distance_source: bool,
    /// Samples in deterministic address order.
    pub samples: Vec<DistanceSample>,
    /// Whether all samples are exact address-space distance evidence.
    ///
    /// This is not a continuous SDF certificate. It certifies only the integer
    /// Manhattan preview over known voxel occupancy.
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
    let distances = exact_manhattan_transform(&occupied, &region)?;
    let dimensions = if distances.is_empty() {
        [0; 3]
    } else {
        [
            usize::try_from(region.max[0] - region.min[0] + 1)
                .map_err(|_| crate::HypervoxelError::AddressOverflow)?,
            usize::try_from(region.max[1] - region.min[1] + 1)
                .map_err(|_| crate::HypervoxelError::AddressOverflow)?,
            usize::try_from(region.max[2] - region.min[2] + 1)
                .map_err(|_| crate::HypervoxelError::AddressOverflow)?,
        ]
    };
    let mut samples = Vec::with_capacity(distances.len());
    // VoxelAddress derives lexicographic order over [x, y, z]. Emit that order
    // directly instead of rebuilding it through an ordered map.
    for x in region.min[0]..=region.max[0] {
        for y in region.min[1]..=region.max[1] {
            for z in region.min[2]..=region.max[2] {
                let address = VoxelAddress::new(region.depth, [x, y, z])?;
                let distance_index = transform_index([x, y, z], &region, dimensions);
                let manhattan_distance = distances[distance_index];
                samples.push(DistanceSample {
                    address,
                    manhattan_distance,
                    occupied: manhattan_distance == Some(0),
                });
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
        samples,
        exact_address_distance_ready,
    })
}

/// Computes the exact lower envelope of integer L1-distance cones over a box.
///
/// A source outside the query box is projected coordinate-wise onto the box.
/// For every point inside the box, its distance to that source is the source's
/// distance to the projection plus the in-box distance from the projection.
/// Six linear sweeps therefore replace the previous source-by-sample scan
/// without changing the integer metric or its evidence boundary.
fn exact_manhattan_transform(
    sources: &[VoxelAddress],
    region: &QueryRegion,
) -> crate::HypervoxelResult<Vec<Option<u64>>> {
    if (0..3).any(|axis| region.min[axis] > region.max[axis]) {
        return Ok(Vec::new());
    }
    VoxelAddress::new(region.depth, region.min)?;
    VoxelAddress::new(region.depth, region.max)?;

    let dimensions = [
        usize::try_from(region.max[0] - region.min[0] + 1)
            .map_err(|_| crate::HypervoxelError::AddressOverflow)?,
        usize::try_from(region.max[1] - region.min[1] + 1)
            .map_err(|_| crate::HypervoxelError::AddressOverflow)?,
        usize::try_from(region.max[2] - region.min[2] + 1)
            .map_err(|_| crate::HypervoxelError::AddressOverflow)?,
    ];
    let sample_count = dimensions
        .iter()
        .try_fold(1_usize, |count, &dimension| count.checked_mul(dimension))
        .ok_or(crate::HypervoxelError::AddressOverflow)?;
    let mut distances = vec![u64::MAX; sample_count];

    for source in sources.iter().filter(|source| source.depth == region.depth) {
        let projected = [
            source.xyz[0].clamp(region.min[0], region.max[0]),
            source.xyz[1].clamp(region.min[1], region.max[1]),
            source.xyz[2].clamp(region.min[2], region.max[2]),
        ];
        let index = transform_index(projected, region, dimensions);
        distances[index] = distances[index].min(manhattan(source.xyz, projected));
    }

    let [width, height, depth] = dimensions;
    for z in 0..depth {
        for y in 0..height {
            let start = (z * height + y) * width;
            relax_line(&mut distances, start, width, 1);
        }
    }
    for z in 0..depth {
        for x in 0..width {
            let start = z * height * width + x;
            relax_line(&mut distances, start, height, width);
        }
    }
    for y in 0..height {
        for x in 0..width {
            let start = y * width + x;
            relax_line(&mut distances, start, depth, width * height);
        }
    }

    Ok(distances
        .into_iter()
        .map(|distance| (distance != u64::MAX).then_some(distance))
        .collect())
}

fn transform_index(xyz: [u64; 3], region: &QueryRegion, [width, height, _]: [usize; 3]) -> usize {
    let x = (xyz[0] - region.min[0]) as usize;
    let y = (xyz[1] - region.min[1]) as usize;
    let z = (xyz[2] - region.min[2]) as usize;
    (z * height + y) * width + x
}

fn relax_line(distances: &mut [u64], start: usize, length: usize, stride: usize) {
    for offset in 1..length {
        let previous = start + (offset - 1) * stride;
        let current = start + offset * stride;
        distances[current] = distances[current].min(distances[previous].saturating_add(1));
    }
    for offset in (0..length.saturating_sub(1)).rev() {
        let current = start + offset * stride;
        let next = current + stride;
        distances[current] = distances[current].min(distances[next].saturating_add(1));
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{GridFrame, MaterialRegionId, VoxelCell};

    #[test]
    fn separable_transform_matches_all_pairs_with_sources_outside_region() {
        let region = QueryRegion {
            min: [2, 3, 1],
            max: [5, 6, 4],
            depth: 4,
        };
        let sources = [
            VoxelAddress::new(4, [0, 0, 0]).unwrap(),
            VoxelAddress::new(4, [9, 4, 2]).unwrap(),
            VoxelAddress::new(4, [3, 12, 8]).unwrap(),
            VoxelAddress::new(3, [2, 3, 1]).unwrap(),
        ];
        let transformed = exact_manhattan_transform(&sources, &region).unwrap();

        let mut index = 0;
        for z in region.min[2]..=region.max[2] {
            for y in region.min[1]..=region.max[1] {
                for x in region.min[0]..=region.max[0] {
                    let expected = sources
                        .iter()
                        .filter(|source| source.depth == region.depth)
                        .map(|source| manhattan(source.xyz, [x, y, z]))
                        .min();
                    assert_eq!(transformed[index], expected);
                    index += 1;
                }
            }
        }
    }

    #[test]
    fn distance_preview_preserves_address_order_and_exact_occupancy() {
        let mut grid = SparseVoxelGrid::new(GridFrame::builder().depth(2).build().unwrap());
        for xyz in [[0, 1, 0], [1, 0, 1]] {
            grid.set(
                VoxelAddress::new(2, xyz).unwrap(),
                VoxelCell::material(MaterialRegionId(1)),
            )
            .unwrap();
        }
        let preview = sample_manhattan_distance_field(
            &grid,
            QueryRegion {
                min: [0, 0, 0],
                max: [1, 1, 1],
                depth: 2,
            },
        )
        .unwrap();

        assert!(
            preview
                .samples
                .windows(2)
                .all(|pair| pair[0].address < pair[1].address)
        );
        for sample in &preview.samples {
            let stored = grid.get(sample.address).unwrap().occupancy != OccupancyState::Empty;
            assert_eq!(sample.occupied, stored);
            assert_eq!(sample.manhattan_distance == Some(0), stored);
        }
    }
}
