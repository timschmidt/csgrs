//! Optional lowering into `hypervoxel` continuous-field intake records.
//!
//! `hypersdf` owns the continuous field and its exact cell classifications.
//! When the `hypervoxel-adapter` feature is enabled, those classifications can
//! be materialized as `hypervoxel` intake rows without duplicating voxel
//! storage or frame semantics in this crate.

use hypervoxel::{
    ContinuousFieldVoxelBatch, ContinuousFieldVoxelCell, GridFrame, HypervoxelError,
    HypervoxelResult, MaterialRegionId, VoxelCell, VoxelPayload, continuous_field_address,
};

use crate::{SdfVoxelBatch, SdfVoxelOccupancy};

/// Converts an SDF voxel batch into a `hypervoxel` continuous-field batch.
pub fn continuous_field_batch_from_sdf(
    batch: &SdfVoxelBatch,
    frame: GridFrame,
    material: MaterialRegionId,
) -> HypervoxelResult<ContinuousFieldVoxelBatch> {
    let mut cells = Vec::with_capacity(batch.cells.len());
    for cell in &batch.cells {
        let address = continuous_field_address(
            &frame,
            [
                u64::from(cell.index[0]),
                u64::from(cell.index[1]),
                u64::from(cell.index[2]),
            ],
        )?;
        cells.push(ContinuousFieldVoxelCell::new(
            address,
            voxel_cell_from_sdf(cell.occupancy, material),
        ));
    }
    Ok(ContinuousFieldVoxelBatch { frame, cells })
}

fn voxel_cell_from_sdf(occupancy: SdfVoxelOccupancy, material: MaterialRegionId) -> VoxelCell {
    match occupancy {
        SdfVoxelOccupancy::Empty => VoxelCell::empty(),
        SdfVoxelOccupancy::Filled => VoxelCell::material(material),
        SdfVoxelOccupancy::Boundary => VoxelCell::boundary(VoxelPayload::MaterialRegion(material)),
        SdfVoxelOccupancy::Unknown => VoxelCell::unknown(),
    }
}

/// Error alias used by callers that only enable this adapter module.
pub type SdfHypervoxelAdapterError = HypervoxelError;
