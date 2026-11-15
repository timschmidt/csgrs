#![cfg(feature = "hypervoxel-adapter")]
use hyperlimit::Point3;
use hypersdf::{
    Sdf, SdfExpr, SdfVoxelCellGrid, SdfVoxelOccupancy, continuous_field_batch_from_sdf,
};
use hypervoxel::{GridFrame, HypervoxelError, LengthUnit, MaterialRegionId};

#[test]
fn filled_cell_cannot_be_relocated_without_reclassification() {
    let field = Sdf::new(SdfExpr::x());
    let source = SdfVoxelCellGrid::new(
        Point3::new((-2).into(), 0.into(), 0.into()),
        Point3::new(1.into(), 1.into(), 1.into()),
        [1; 3],
    );
    let batch = field.classify_voxel_grid(source).unwrap();
    assert_eq!(batch.cells[0].occupancy, SdfVoxelOccupancy::Filled);
    let wrong = GridFrame::new(
        [10.into(), 0.into(), 0.into()],
        [1.into(), 1.into(), 1.into()],
        0,
        LengthUnit::Unitless,
    )
    .unwrap();
    assert!(matches!(
        continuous_field_batch_from_sdf(&batch, wrong, MaterialRegionId(7)),
        Err(HypervoxelError::InvalidContinuousFieldMaterialization { .. })
    ));
}

use hyperreal::{Rational, Real};
use hypersdf::{SdfVoxelBatch, SdfVoxelCell, SdfVoxelLengthUnit};
use hypervoxel::{OccupancyState, VoxelPayload};

fn batch() -> SdfVoxelBatch {
    SdfVoxelBatch {
        grid: SdfVoxelCellGrid::new(
            Point3::new((-2).into(), 3.into(), 5.into()),
            Point3::new(1.into(), 2.into(), 3.into()),
            [2; 3],
        ),
        cells: (0..2)
            .flat_map(|z| {
                (0..2).flat_map(move |y| {
                    (0..2).map(move |x| SdfVoxelCell {
                        index: [x, y, z],
                        occupancy: if x == 0 {
                            SdfVoxelOccupancy::Filled
                        } else {
                            SdfVoxelOccupancy::Empty
                        },
                    })
                })
            })
            .collect(),
    }
}
fn frame(b: &SdfVoxelBatch, units: LengthUnit, depth: u8) -> GridFrame {
    GridFrame::new(
        [
            b.grid.origin.x.clone(),
            b.grid.origin.y.clone(),
            b.grid.origin.z.clone(),
        ],
        [
            b.grid.step.x.clone(),
            b.grid.step.y.clone(),
            b.grid.step.z.clone(),
        ],
        depth,
        units,
    )
    .unwrap()
}
fn invalid(result: Result<hypervoxel::ContinuousFieldVoxelBatch, HypervoxelError>) {
    assert!(matches!(
        result,
        Err(HypervoxelError::InvalidContinuousFieldMaterialization { .. })
    ));
}

#[test]
fn every_declared_unit_preserves_cells_and_materialization() {
    let units = [
        (SdfVoxelLengthUnit::Unitless, LengthUnit::Unitless),
        (SdfVoxelLengthUnit::Meter, LengthUnit::Meter),
        (SdfVoxelLengthUnit::Millimeter, LengthUnit::Millimeter),
        (SdfVoxelLengthUnit::Micrometer, LengthUnit::Micrometer),
        (SdfVoxelLengthUnit::Nanometer, LengthUnit::Nanometer),
    ];
    for (source_unit, target_unit) in units {
        let mut b = batch();
        b.grid.units = source_unit;
        let target = frame(&b, target_unit, 1);
        let result = continuous_field_batch_from_sdf(&b, target, MaterialRegionId(17)).unwrap();
        let grid = result.materialize_exact_sparse_grid().unwrap();
        assert_eq!(grid.frame().units(), target_unit);
        for row in &result.cells {
            assert_eq!(grid.get(row.address).unwrap(), row.cell);
            let [x, y, z] = row.address.xyz;
            let bounds = row.address.bounds(grid.frame()).unwrap();
            assert_eq!(
                bounds.min,
                [
                    Real::from(x as i64 - 2),
                    Real::from(3 + 2 * y),
                    Real::from(5 + 3 * z)
                ]
            );
            assert_eq!(
                bounds.max,
                [
                    Real::from(x as i64 - 1),
                    Real::from(5 + 2 * y),
                    Real::from(8 + 3 * z)
                ]
            );
            if row.cell.occupancy == OccupancyState::Filled {
                assert_eq!(
                    row.cell.payload,
                    VoxelPayload::MaterialRegion(MaterialRegionId(17))
                );
            }
        }
        for (_, wrong_unit) in units {
            if wrong_unit != target_unit {
                invalid(continuous_field_batch_from_sdf(
                    &b,
                    frame(&b, wrong_unit, 1),
                    MaterialRegionId(17),
                ));
            }
        }
    }
}

#[test]
fn all_origin_and_pitch_axes_reject_nearby_distinct_cells() {
    let b = batch();
    let epsilon = Real::from(Rational::fraction(1, 1u64 << 60).unwrap());
    for axis in 0..3 {
        for change_pitch in [false, true] {
            let good = frame(&b, LengthUnit::Unitless, 1);
            let mut origin = good.origin().clone();
            let mut pitch = good.pitches().clone();
            if change_pitch {
                pitch[axis] = &pitch[axis] + &epsilon;
            } else {
                origin[axis] = &origin[axis] + &epsilon;
            }
            let bad = GridFrame::new(origin, pitch, 1, LengthUnit::Unitless).unwrap();
            invalid(continuous_field_batch_from_sdf(
                &b,
                bad,
                MaterialRegionId(7),
            ));
            let retry = continuous_field_batch_from_sdf(&b, good, MaterialRegionId(7)).unwrap();
            assert_eq!(retry.cells.len(), 8);
        }
    }
}

#[test]
fn independently_constructed_algebraic_coordinates_are_accepted() {
    let mut b = batch();
    b.grid.origin.x = Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap();
    b.grid.step.y = Real::from(5).sqrt().unwrap();
    let f = GridFrame::new(
        [
            Real::from(3).sqrt().unwrap() + Real::from(2).sqrt().unwrap(),
            3.into(),
            5.into(),
        ],
        [1.into(), Real::from(5).sqrt().unwrap(), 3.into()],
        1,
        LengthUnit::Unitless,
    )
    .unwrap();
    continuous_field_batch_from_sdf(&b, f, MaterialRegionId(7))
        .unwrap()
        .materialize_exact_sparse_grid()
        .unwrap();
}

#[path = "support/uncertainty.rs"]
mod uncertainty;
#[test]
fn undecided_equality_does_not_relocate_evidence() {
    let b = batch();
    let zero = uncertainty::terminal_zero(Real::zero());
    let mut origin = frame(&b, LengthUnit::Unitless, 1).origin().clone();
    origin[2] = &origin[2] + zero;
    let bad = GridFrame::new(
        origin,
        [1.into(), 2.into(), 3.into()],
        1,
        LengthUnit::Unitless,
    )
    .unwrap();
    assert!(matches!(
        continuous_field_batch_from_sdf(&b, bad, MaterialRegionId(7)),
        Err(HypervoxelError::UnknownOrdering { axis: 2 })
    ));
}

#[test]
fn invalid_grid_or_source_indices_fail_even_when_target_can_hold_them() {
    let b = batch();
    let f = frame(&b, LengthUnit::Unitless, 2);
    for dimensions in [
        [0, 2, 2],
        [2, 0, 2],
        [2, 2, 0],
        [5, 2, 2],
        [2, 5, 2],
        [2, 2, 5],
    ] {
        let mut bad = b.clone();
        bad.grid.dimensions = dimensions;
        bad.cells.clear();
        invalid(continuous_field_batch_from_sdf(
            &bad,
            f.clone(),
            MaterialRegionId(7),
        ));
    }
    for axis in 0..3 {
        let mut bad = b.clone();
        bad.cells[0].index[axis] = 2;
        invalid(continuous_field_batch_from_sdf(
            &bad,
            f.clone(),
            MaterialRegionId(7),
        ));
    }
    let mut bad = b.clone();
    bad.grid.step.x = (-1).into();
    invalid(continuous_field_batch_from_sdf(
        &bad,
        f,
        MaterialRegionId(7),
    ));
}

#[test]
fn rectangular_partial_and_unknown_batches_remain_explicit() {
    let mut b = batch();
    b.grid.dimensions = [3, 2, 1];
    b.cells = vec![SdfVoxelCell {
        index: [2, 1, 0],
        occupancy: SdfVoxelOccupancy::Unknown,
    }];
    let f = frame(&b, LengthUnit::Unitless, 2);
    let result = continuous_field_batch_from_sdf(&b, f.clone(), MaterialRegionId(7)).unwrap();
    assert!(result.materialize_exact_sparse_grid().is_err());
    let grid = result.materialize_sparse_grid().unwrap();
    assert_eq!(
        grid.get(result.cells[0].address).unwrap().occupancy,
        OccupancyState::Unknown
    );
    b.cells.clear();
    let empty = continuous_field_batch_from_sdf(&b, f, MaterialRegionId(7)).unwrap();
    assert!(empty.materialize_sparse_grid().unwrap().is_empty());
    assert!(empty.materialize_exact_sparse_grid().is_err());
}

#[test]
fn malformed_empty_batch_does_not_hide_wrong_frame() {
    let mut b = batch();
    b.cells.clear();
    invalid(continuous_field_batch_from_sdf(
        &b,
        GridFrame::unit(1).unwrap(),
        MaterialRegionId(7),
    ));
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn rational_frame_admission_uses_six_exact_comparisons() {
    let b = batch();
    let f = frame(&b, LengthUnit::Unitless, 1);
    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();
    continuous_field_batch_from_sdf(&b, f, MaterialRegionId(7)).unwrap();
    let trace = hyperreal::dispatch_trace::snapshot_trace();
    assert_eq!(trace.operation_count("hyperlimit", "compare_reals"), 6);
    assert_eq!(
        trace.path_count("hyperlimit", "compare_reals", "exact-rational"),
        6
    );
    assert_eq!(trace.correlation_summary().approximation_events, 0);
}

#[test]
fn fresh_process_reconstructs_and_materializes_valid_frames() {
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "every_declared_unit_preserves_cells_and_materialization",
            "--nocapture",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(String::from_utf8_lossy(&output.stdout).contains("1 passed"));
}
