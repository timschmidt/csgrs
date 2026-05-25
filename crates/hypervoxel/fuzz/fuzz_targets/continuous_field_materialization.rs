#![no_main]

use hypervoxel::{
    ContinuousFieldVoxelCell, ContinuousFieldVoxelManifest, GridFrame, GridSource,
    MaterialRegionId, VoxelCell, continuous_field_address,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: (u8, u8, bool, bool, bool, bool)| {
    let (depth_raw, mutate_index, stale, drop_last, duplicate_first, non_exact_first) = data;
    let depth = (depth_raw % 3) + 1;
    let frame = GridFrame::builder()
        .depth(depth)
        .source(GridSource::new("fuzz:continuous-field", 1))
        .build()
        .unwrap();
    let source = frame.source().cloned();
    let expected_source = if stale {
        Some(GridSource::new("fuzz:continuous-field", 2))
    } else {
        source.clone()
    };
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

    let manifest = ContinuousFieldVoxelManifest {
        frame,
        source,
        expected_source,
        expected_cell_count: rows.len(),
        cells: rows,
    };
    let report = manifest.report();
    assert_eq!(
        report.exact_materialization_ready,
        manifest.materialize_exact_sparse_grid().is_ok()
    );
    if report.exact_materialization_ready {
        assert!(report.materialization_blockers.is_empty());
        assert!(report.complete_expected_cover);
        assert!(report.complete_frame_cover);
    } else {
        assert!(!report.materialization_blockers.is_empty());
    }
});
