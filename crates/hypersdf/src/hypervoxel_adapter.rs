//! Optional lowering into `hypervoxel` continuous-field intake records.
//!
//! `hypersdf` owns the continuous field and its exact cell classifications.
//! When the `hypervoxel-adapter` feature is enabled, those classifications can
//! be materialized as `hypervoxel` intake rows without duplicating voxel
//! storage or frame semantics in this crate.

use hypervoxel::{
    ContinuousFieldVoxelCell, ContinuousFieldVoxelManifest, GridFrame, GridSource, HypervoxelError,
    HypervoxelResult, MaterialRegionId, VoxelCell, VoxelPayload, continuous_field_address,
};

use crate::{SdfHypervoxelHandoffReport, SdfVoxelOccupancy};

/// Converts an SDF voxel handoff report into a `hypervoxel` intake manifest.
pub fn continuous_field_manifest_from_sdf(
    report: &SdfHypervoxelHandoffReport,
    frame: GridFrame,
    material: MaterialRegionId,
) -> HypervoxelResult<ContinuousFieldVoxelManifest> {
    let mut cells = Vec::with_capacity(report.cells.len());
    for cell in &report.cells {
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
    let source = report
        .grid
        .source
        .as_ref()
        .map(|source| GridSource::new(source.id.clone(), source.version));
    Ok(ContinuousFieldVoxelManifest {
        frame,
        source: source.clone(),
        expected_source: source,
        expected_cell_count: report.cell_count,
        cells,
    })
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
