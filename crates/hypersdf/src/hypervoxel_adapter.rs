//! Optional lowering into `hypervoxel` continuous-field intake records.
//!
//! `hypersdf` owns the continuous field and its exact cell classifications.
//! When the `hypervoxel-adapter` feature is enabled, those classifications can
//! be materialized as `hypervoxel` intake rows without duplicating voxel
//! storage or frame semantics in this crate.

use core::cmp::Ordering;
use hyperlimit::PredicateOutcome;
use hypervoxel::{
    ContinuousFieldVoxelBatch, ContinuousFieldVoxelCell, GridFrame, HypervoxelError,
    HypervoxelResult, LengthUnit, MaterialRegionId, VoxelCell, VoxelPayload,
    continuous_field_address,
};

use crate::policy::compare_reals_for_construction;
use crate::{SdfVoxelBatch, SdfVoxelLengthUnit, SdfVoxelOccupancy};

/// Converts an SDF voxel batch into a `hypervoxel` continuous-field batch.
///
/// The target must have exactly the source origin, cell pitches and declared
/// units. Equality is certified under the strict construction policy; an
/// undecided comparison is an error, not permission to relocate evidence.
/// Source dimensions must fit the target, and every row must lie in the source
/// grid. Rectangular and partial batches remain supported; only Hypervoxel's
/// exact materializer requires a complete target cover. This validates frame
/// transport, not the authenticity of caller-constructed occupancy labels.
pub fn continuous_field_batch_from_sdf(
    batch: &SdfVoxelBatch,
    frame: GridFrame,
    material: MaterialRegionId,
) -> HypervoxelResult<ContinuousFieldVoxelBatch> {
    validate_frame(batch, &frame)?;
    let mut cells = Vec::with_capacity(batch.cells.len());
    for cell in &batch.cells {
        if cell
            .index
            .iter()
            .zip(batch.grid.dimensions)
            .any(|(i, n)| *i >= n)
        {
            return Err(HypervoxelError::InvalidContinuousFieldMaterialization {
                reason: "supplied cell is outside the source SDF grid",
            });
        }
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

fn validate_frame(batch: &SdfVoxelBatch, frame: &GridFrame) -> HypervoxelResult<()> {
    if batch
        .grid
        .dimensions
        .iter()
        .any(|n| *n == 0 || u64::from(*n) > frame.cells_per_axis())
    {
        return Err(HypervoxelError::InvalidContinuousFieldMaterialization {
            reason: "source SDF grid dimensions do not fit the target frame",
        });
    }
    let units = match batch.grid.units {
        SdfVoxelLengthUnit::Unitless => LengthUnit::Unitless,
        SdfVoxelLengthUnit::Meter => LengthUnit::Meter,
        SdfVoxelLengthUnit::Millimeter => LengthUnit::Millimeter,
        SdfVoxelLengthUnit::Micrometer => LengthUnit::Micrometer,
        SdfVoxelLengthUnit::Nanometer => LengthUnit::Nanometer,
    };
    if units != frame.units() {
        return Err(HypervoxelError::InvalidContinuousFieldMaterialization {
            reason: "source SDF grid and target frame have different units",
        });
    }
    let origin = [
        &batch.grid.origin.x,
        &batch.grid.origin.y,
        &batch.grid.origin.z,
    ];
    let step = [&batch.grid.step.x, &batch.grid.step.y, &batch.grid.step.z];
    for axis in 0..3 {
        for (source, target) in [
            (origin[axis], &frame.origin()[axis]),
            (step[axis], frame.pitch(axis)),
        ] {
            match compare_reals_for_construction(source, target) {
                PredicateOutcome::Decided {
                    value: Ordering::Equal,
                    ..
                } => {}
                PredicateOutcome::Decided { .. } => {
                    return Err(HypervoxelError::InvalidContinuousFieldMaterialization {
                        reason: "source SDF grid and target frame describe different cells",
                    });
                }
                PredicateOutcome::Unknown { .. } => {
                    return Err(HypervoxelError::UnknownOrdering { axis });
                }
            }
        }
    }
    Ok(())
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
