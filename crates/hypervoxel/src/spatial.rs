//! Spatial aggregate facts for sparse voxel grids.
//!
//! Occupancy aggregates say what is known about cell values. Spatial aggregate
//! facts say where those values live: exact enclosing bounds, root child
//! presence, stored-cell counts, and optional source freshness. Keeping these
//! facts separate follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997: spatial/combinatorial evidence is
//! preserved as structured object data instead of being inferred later from a
//! lossy mesh, preview, or floating-point bounding box.

use hyperreal::{CertifiedRealOrdering, Real};

use crate::{
    ExactAabb3, FreshnessStatus, HypervoxelError, HypervoxelResult, OccupancyState,
    SparseVoxelGrid, VoxelAddress, VoxelizationReport,
};

/// Exact spatial facts for a sparse grid or aggregate region.
#[derive(Clone, Debug, PartialEq)]
pub struct VoxelSpatialAggregateFacts {
    /// Number of explicitly stored non-empty cells.
    pub stored_cells: usize,
    /// Root-octant child presence mask.
    pub child_presence_mask: u8,
    /// Exact enclosing AABB of stored non-empty cells, if any.
    pub exact_bounds: Option<ExactAabb3>,
    /// Optional source freshness copied from a voxelization report.
    pub freshness: FreshnessStatus,
}

impl VoxelSpatialAggregateFacts {
    /// Builds spatial aggregate facts for all stored non-empty cells.
    pub fn from_grid(
        grid: &SparseVoxelGrid,
        report: Option<&VoxelizationReport>,
    ) -> HypervoxelResult<Self> {
        let mut stored_cells = 0_usize;
        let mut child_presence_mask = 0_u8;
        let mut exact_bounds: Option<ExactAabb3> = None;

        for (address, cell) in grid.iter() {
            if cell.occupancy == OccupancyState::Empty {
                continue;
            }
            stored_cells += 1;
            child_presence_mask |= root_child_bit(*address);
            let bounds: ExactAabb3 = address.bounds(grid.frame())?.into();
            exact_bounds = Some(match exact_bounds {
                Some(current) => union_aabb(&current, &bounds)?,
                None => bounds,
            });
        }

        Ok(Self {
            stored_cells,
            child_presence_mask,
            exact_bounds,
            freshness: report
                .map(VoxelizationReport::freshness)
                .unwrap_or(FreshnessStatus::Unknown),
        })
    }

    /// Returns whether the root octant is present in this aggregate.
    pub fn has_child(&self, child_index: u8) -> bool {
        child_index < 8 && (self.child_presence_mask & (1 << child_index)) != 0
    }
}

fn root_child_bit(address: VoxelAddress) -> u8 {
    if address.depth == 0 {
        return 0b1111_1111;
    }
    let level = address.depth - 1;
    let x = ((address.xyz[0] >> level) & 1) as u8;
    let y = ((address.xyz[1] >> level) & 1) as u8;
    let z = ((address.xyz[2] >> level) & 1) as u8;
    1 << (x | (y << 1) | (z << 2))
}

fn union_aabb(left: &ExactAabb3, right: &ExactAabb3) -> HypervoxelResult<ExactAabb3> {
    let mut min = left.min.clone();
    let mut max = left.max.clone();
    for axis in 0..3 {
        if certified_cmp(&right.min[axis], &min[axis])? == std::cmp::Ordering::Less {
            min[axis] = right.min[axis].clone();
        }
        if certified_cmp(&right.max[axis], &max[axis])? == std::cmp::Ordering::Greater {
            max[axis] = right.max[axis].clone();
        }
    }
    Ok(ExactAabb3 { min, max })
}

fn certified_cmp(left: &Real, right: &Real) -> HypervoxelResult<std::cmp::Ordering> {
    match left.certified_cmp_until(right, -128) {
        CertifiedRealOrdering::Known { ordering, .. } => Ok(ordering),
        CertifiedRealOrdering::Unknown { .. } => Err(HypervoxelError::UnknownScalarOrdering {
            field: "spatial aggregate bounds",
        }),
    }
}
