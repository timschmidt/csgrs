use hypervoxel::{
    ContinuousFieldMaterializationBlocker, ContinuousFieldVoxelCell, ContinuousFieldVoxelManifest,
    FreshnessStatus, GridFrame, GridSource, HypervoxelError, MaterialRegionId, VoxelAddress,
    VoxelCell, continuous_field_address,
};
use proptest::prelude::*;

fn sourced_frame(depth: u8, version: u64) -> GridFrame {
    GridFrame::builder()
        .depth(depth)
        .source(GridSource::new("sdf:direct-materialization", version))
        .build()
        .unwrap()
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

fn manifest(
    frame: GridFrame,
    source: Option<GridSource>,
    expected_source: Option<GridSource>,
    cells: Vec<ContinuousFieldVoxelCell>,
) -> ContinuousFieldVoxelManifest {
    ContinuousFieldVoxelManifest {
        expected_cell_count: cells.len(),
        frame,
        source,
        expected_source,
        cells,
    }
}

#[test]
fn exact_continuous_field_rows_materialize_only_after_full_current_replay() {
    let frame = sourced_frame(2, 11);
    let source = frame.source().cloned();
    let rows = full_rows(&frame);
    let manifest = manifest(frame.clone(), source.clone(), source, rows);

    let report = manifest.report();
    assert_eq!(report.freshness, FreshnessStatus::Current);
    assert!(report.complete_expected_cover);
    assert!(report.complete_frame_cover);
    assert!(report.materialization_blockers.is_empty());
    assert!(report.exact_materialization_ready);

    let prepared = manifest.materialize_exact_sparse_grid().unwrap();
    assert_eq!(prepared.storage.frame(), &frame);
    assert_eq!(prepared.storage.len(), 64);
    assert!(prepared.report.as_ref().unwrap().exact_topology_ready());
    assert!(prepared.report.as_ref().unwrap().source_replay_ready());
}

#[test]
fn strict_materialization_rejects_stale_duplicate_incomplete_unknown_and_lossy_rows() {
    let frame = sourced_frame(1, 5);
    let source = frame.source().cloned();
    let stale_source = Some(GridSource::new("sdf:direct-materialization", 4));
    let rows = full_rows(&frame);

    let stale = manifest(frame.clone(), stale_source, source.clone(), rows.clone()).report();
    assert!(
        stale
            .materialization_blockers
            .contains(&ContinuousFieldMaterializationBlocker::SourceNotCurrent)
    );

    let mut duplicate_rows = rows.clone();
    duplicate_rows[1] = duplicate_rows[0];
    let duplicate = manifest(
        frame.clone(),
        source.clone(),
        source.clone(),
        duplicate_rows,
    )
    .report();
    assert!(
        duplicate
            .materialization_blockers
            .contains(&ContinuousFieldMaterializationBlocker::DuplicateAddresses)
    );
    assert!(!duplicate.exact_materialization_ready);

    let incomplete_rows = rows[..rows.len() - 1].to_vec();
    let incomplete = manifest(
        frame.clone(),
        source.clone(),
        source.clone(),
        incomplete_rows,
    )
    .report();
    assert!(
        incomplete
            .materialization_blockers
            .contains(&ContinuousFieldMaterializationBlocker::IncompleteFrameCover)
    );

    let mut unknown_rows = rows.clone();
    unknown_rows[0].cell = VoxelCell::unknown();
    let unknown = manifest(frame.clone(), source.clone(), source.clone(), unknown_rows).report();
    assert!(
        unknown
            .materialization_blockers
            .contains(&ContinuousFieldMaterializationBlocker::NonExactCellEvidence)
    );
    assert!(
        unknown
            .materialization_blockers
            .contains(&ContinuousFieldMaterializationBlocker::UncertifiedPredicates)
    );

    let mut lossy_rows = rows;
    lossy_rows[0].cell = VoxelCell::lossy_adapter_value(99);
    let lossy_manifest = manifest(frame, source.clone(), source, lossy_rows);
    assert_eq!(
        lossy_manifest.materialize_exact_sparse_grid(),
        Err(HypervoxelError::InvalidContinuousFieldMaterialization {
            reason: "unknown or lossy cell evidence is present"
        })
    );
}

#[test]
fn strict_materialization_rejects_parent_depth_rows_even_when_address_is_in_frame() {
    let frame = sourced_frame(2, 1);
    let source = frame.source().cloned();
    let mut rows = full_rows(&frame);
    rows[0] = ContinuousFieldVoxelCell::new(
        VoxelAddress::new(1, [0, 0, 0]).unwrap(),
        VoxelCell::material(MaterialRegionId(1)),
    );
    let report = manifest(frame, source.clone(), source, rows).report();

    assert!(!report.finest_depth_only);
    assert!(
        report
            .materialization_blockers
            .contains(&ContinuousFieldMaterializationBlocker::NonFinestDepthRows)
    );
}

proptest! {
    #[test]
    fn generated_strict_materialization_requires_full_unique_current_finest_cover(
        depth in 1_u8..4,
        stale in any::<bool>(),
        drop_last in any::<bool>(),
        duplicate_first in any::<bool>(),
        unknown_first in any::<bool>(),
    ) {
        let frame = sourced_frame(depth, 9);
        let source = frame.source().cloned();
        let expected_source = if stale {
            Some(GridSource::new("sdf:direct-materialization", 10))
        } else {
            source.clone()
        };
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
        let manifest = manifest(frame, source, expected_source, rows);
        let report = manifest.report();

        let should_admit = !stale && !drop_last && !duplicate_first && !unknown_first;
        prop_assert_eq!(report.exact_materialization_ready, should_admit);
        prop_assert_eq!(manifest.materialize_exact_sparse_grid().is_ok(), should_admit);
    }
}
