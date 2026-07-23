#![no_main]

use hypervoxel::{
    ContinuousFieldVoxelBatch, ContinuousFieldVoxelCell, GridFrame, MaterialRegionId, VoxelCell,
    continuous_field_address,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: (u8, u8, bool, bool, bool, bool)| {
    let (depth_raw, mutate_index, _stale, drop_last, duplicate_first, non_exact_first) = data;
    let depth = (depth_raw % 3) + 1;
    let frame = GridFrame::builder().depth(depth).build().unwrap();
    let cells_per_axis = frame.cells_per_axis();
    let mut rows = Vec::new();
    for z in 0..cells_per_axis {
        for y in 0..cells_per_axis {
            for x in 0..cells_per_axis {
                rows.push(ContinuousFieldVoxelCell::new(
                    continuous_field_address(&frame, [x, y, z]).unwrap(),
                    VoxelCell::material(MaterialRegionId(1)),
                ));
            }
        }
    }
    if drop_last {
        rows.pop();
    }
    if duplicate_first && rows.len() > 1 {
        let index = usize::from(mutate_index) % rows.len();
        rows[index] = rows[0];
    }
    if non_exact_first && !rows.is_empty() {
        rows[0].cell = VoxelCell::unknown();
    }

    let batch = ContinuousFieldVoxelBatch { frame, cells: rows };
    let result = batch.materialize_exact_sparse_grid();
    if !drop_last && !duplicate_first && !non_exact_first {
        assert!(result.is_ok());
    }
});
