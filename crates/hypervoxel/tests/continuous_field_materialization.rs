use hypervoxel::{
    ContinuousFieldVoxelBatch, ContinuousFieldVoxelCell, GridFrame, HypervoxelError,
    MaterialRegionId, VoxelAddress, VoxelCell, continuous_field_address,
};
use proptest::prelude::*;

fn frame(depth: u8) -> GridFrame {
    GridFrame::unit(depth).unwrap()
}

fn full_rows(frame: &GridFrame) -> Vec<ContinuousFieldVoxelCell> {
    let cells_per_axis = frame.cells_per_axis();
    let mut rows = Vec::new();
    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                rows.push(ContinuousFieldVoxelCell::new(
                    continuous_field_address(frame, [x, y, z]).unwrap(),
                    VoxelCell::material(MaterialRegionId(7)),
                ));
            }
        }
    }
    rows
}

fn batch(frame: GridFrame, cells: Vec<ContinuousFieldVoxelCell>) -> ContinuousFieldVoxelBatch {
    ContinuousFieldVoxelBatch { frame, cells }
}

#[test]
fn exact_continuous_field_rows_materialize_a_complete_cover() {
    let frame = frame(2);
    let grid = batch(frame.clone(), full_rows(&frame))
        .materialize_exact_sparse_grid()
        .unwrap();

    assert_eq!(grid.frame(), &frame);
    assert_eq!(grid.len(), 64);
}

#[test]
fn strict_materialization_rejects_duplicate_incomplete_unknown_and_lossy_rows() {
    let frame = frame(1);
    let rows = full_rows(&frame);

    let mut duplicate_rows = rows.clone();
    duplicate_rows[1] = duplicate_rows[0];
    assert_eq!(
        batch(frame.clone(), duplicate_rows).materialize_exact_sparse_grid(),
        Err(HypervoxelError::InvalidContinuousFieldMaterialization {
            reason: "supplied cells contain duplicate addresses",
        })
    );

    assert_eq!(
        batch(frame.clone(), rows[..rows.len() - 1].to_vec()).materialize_exact_sparse_grid(),
        Err(HypervoxelError::InvalidContinuousFieldMaterialization {
            reason: "supplied cells do not cover the complete frame",
        })
    );

    let mut unknown_rows = rows.clone();
    unknown_rows[0].cell = VoxelCell::unknown();
    assert_eq!(
        batch(frame.clone(), unknown_rows).materialize_exact_sparse_grid(),
        Err(HypervoxelError::InvalidContinuousFieldMaterialization {
            reason: "supplied cell contains unknown or lossy evidence",
        })
    );

    let mut lossy_rows = rows;
    lossy_rows[0].cell = VoxelCell::lossy_adapter_value(99);
    assert_eq!(
        batch(frame, lossy_rows).materialize_exact_sparse_grid(),
        Err(HypervoxelError::InvalidContinuousFieldMaterialization {
            reason: "supplied cell contains unknown or lossy evidence",
        })
    );
}

#[test]
fn strict_materialization_rejects_parent_depth_rows() {
    let frame = frame(2);
    let mut rows = full_rows(&frame);
    rows[0] = ContinuousFieldVoxelCell::new(
        VoxelAddress::new(1, [0, 0, 0]).unwrap(),
        VoxelCell::material(MaterialRegionId(1)),
    );

    assert_eq!(
        batch(frame, rows).materialize_exact_sparse_grid(),
        Err(HypervoxelError::InvalidContinuousFieldMaterialization {
            reason: "supplied cell is not at the frame depth",
        })
    );
}

proptest! {
    #[test]
    fn generated_strict_materialization_requires_full_unique_exact_finest_cover(
        depth in 1_u8..4,
        drop_last in any::<bool>(),
        duplicate_first in any::<bool>(),
        unknown_first in any::<bool>(),
    ) {
        let frame = frame(depth);
        let mut rows = full_rows(&frame);
        if drop_last {
            rows.pop();
        }
        if duplicate_first && rows.len() > 1 {
            rows[1] = rows[0];
        }
        if unknown_first && !rows.is_empty() {
            rows[0].cell = VoxelCell::unknown();
        }

        let should_admit = !drop_last && !duplicate_first && !unknown_first;
        prop_assert_eq!(
            batch(frame, rows).materialize_exact_sparse_grid().is_ok(),
            should_admit
        );
    }
}
