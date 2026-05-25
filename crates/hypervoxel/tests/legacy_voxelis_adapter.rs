#![cfg(feature = "legacy-voxelis")]

use glam::IVec3;
use hypervoxel::{
    ChunkShape, GridFrame, LegacyAdapterKind, MaterialRegionId, OccupancyState, SparseVoxelGrid,
    VoxelAddress, VoxelCell, compare_legacy_voxelis_u8_samples,
    materialize_legacy_voxelis_u8_chunk_paged_storage,
};
use proptest::prelude::*;
use voxelis::{
    MaxDepth, VoxInterner,
    spatial::{VoxOpsWrite, VoxTree},
};

#[test]
fn legacy_voxelis_storage_diff_accepts_only_sampled_semantic_equivalence() {
    let frame = GridFrame::builder().depth(2).build().unwrap();
    let mut expected = SparseVoxelGrid::new(frame);
    let a = VoxelAddress::new(2, [1, 0, 0]).unwrap();
    let b = VoxelAddress::new(2, [3, 3, 3]).unwrap();
    expected
        .set(a, VoxelCell::material(MaterialRegionId(7)))
        .unwrap();
    expected
        .set(b, VoxelCell::material(MaterialRegionId(2)))
        .unwrap();

    let mut interner = VoxInterner::<u8>::with_memory_budget(4096);
    let mut tree = VoxTree::<u8>::new(MaxDepth::new(2));
    assert!(tree.set(&mut interner, IVec3::new(1, 0, 0), 7));
    assert!(tree.set(&mut interner, IVec3::new(3, 3, 3), 2));

    let empty = VoxelAddress::new(2, [0, 0, 0]).unwrap();
    let report =
        compare_legacy_voxelis_u8_samples(&tree, &interner, &expected, [a, b, empty]).unwrap();
    assert_eq!(report.sampled_addresses, 3);
    assert_eq!(report.compared_addresses, 3);
    assert!(report.has_compared_addresses);
    assert!(report.legacy_depth_matches_frame);
    assert!(report.differing_cells.is_empty());
    assert_eq!(report.mismatch_count, 0);
    assert!(report.sampled_storage_equivalence_ready);
    assert_eq!(report.adapter.kind, LegacyAdapterKind::VoxelisStorage);
    assert!(!report.adapter.exact_replay);
    assert!(!report.exact_voxelization_ready);
}

#[test]
fn legacy_voxelis_chunk_paged_materialization_exhaustively_replays_leaf_storage() {
    let frame = GridFrame::builder().depth(3).build().unwrap();
    let shape = ChunkShape::new(1).unwrap();
    let mut interner = VoxInterner::<u8>::with_memory_budget(16_384);
    let mut tree = VoxTree::<u8>::new(MaxDepth::new(3));
    let cells = [
        ([0, 0, 0], 1_u8),
        ([1, 0, 0], 7_u8),
        ([2, 3, 4], 9_u8),
        ([7, 7, 7], 255_u8),
    ];
    for (xyz, value) in cells {
        assert!(tree.set(&mut interner, IVec3::new(xyz[0], xyz[1], xyz[2]), value));
    }

    let (paged, report) =
        materialize_legacy_voxelis_u8_chunk_paged_storage(&tree, &interner, frame, shape).unwrap();

    assert_eq!(report.frame_depth, 3);
    assert_eq!(report.legacy_depth, 3);
    assert!(report.legacy_depth_matches_frame);
    assert_eq!(report.scanned_cells, 512);
    assert_eq!(report.replayed_cells, 512);
    assert_eq!(report.materialized_cells, 4);
    assert_eq!(report.material_region_cells, 4);
    assert_eq!(report.empty_cells, 508);
    assert_eq!(report.paging_mismatch_cells, 0);
    assert_eq!(report.storage.summary.stored_cells, 4);
    assert!(report.storage.exact_chunk_storage_ready);
    assert!(report.exhaustive_chunk_port_ready);
    assert_eq!(report.adapter.kind, LegacyAdapterKind::VoxelisStorage);
    assert!(!report.adapter.exact_replay);
    assert!(!report.exact_voxelization_ready);

    for (xyz, value) in cells {
        let address = VoxelAddress::new(3, [xyz[0] as u64, xyz[1] as u64, xyz[2] as u64]).unwrap();
        assert_eq!(
            paged.get(address).unwrap(),
            VoxelCell::material(MaterialRegionId(u32::from(value)))
        );
    }
    assert_eq!(
        paged
            .get(VoxelAddress::new(3, [6, 6, 6]).unwrap())
            .unwrap()
            .occupancy,
        OccupancyState::Empty
    );
}

#[test]
fn legacy_voxelis_chunk_paged_materialization_rejects_depth_mismatch_as_evidence() {
    let frame = GridFrame::builder().depth(3).build().unwrap();
    let mut interner = VoxInterner::<u8>::with_memory_budget(4096);
    let mut tree = VoxTree::<u8>::new(MaxDepth::new(2));
    assert!(tree.set(&mut interner, IVec3::new(1, 1, 1), 4));

    let (paged, report) = materialize_legacy_voxelis_u8_chunk_paged_storage(
        &tree,
        &interner,
        frame,
        ChunkShape::new(2).unwrap(),
    )
    .unwrap();

    assert_eq!(report.frame_depth, 3);
    assert_eq!(report.legacy_depth, 2);
    assert!(!report.legacy_depth_matches_frame);
    assert_eq!(report.scanned_cells, 0);
    assert_eq!(report.replayed_cells, 0);
    assert_eq!(report.materialized_cells, 0);
    assert_eq!(report.storage.summary.stored_cells, 0);
    assert!(!report.storage.exact_chunk_storage_ready);
    assert!(!report.exhaustive_chunk_port_ready);
    assert!(!report.exact_voxelization_ready);
    assert!(paged.is_empty());
}

#[test]
fn legacy_voxelis_storage_diff_rejects_empty_mismatched_and_non_leaf_samples() {
    let frame = GridFrame::builder().depth(2).build().unwrap();
    let mut expected = SparseVoxelGrid::new(frame);
    let filled = VoxelAddress::new(2, [1, 0, 0]).unwrap();
    expected
        .set(filled, VoxelCell::material(MaterialRegionId(7)))
        .unwrap();

    let mut interner = VoxInterner::<u8>::with_memory_budget(4096);
    let mut tree = VoxTree::<u8>::new(MaxDepth::new(2));
    assert!(tree.set(&mut interner, IVec3::new(1, 0, 0), 8));

    let empty_report = compare_legacy_voxelis_u8_samples(&tree, &interner, &expected, []).unwrap();
    assert_eq!(empty_report.compared_addresses, 0);
    assert!(!empty_report.has_compared_addresses);
    assert!(!empty_report.sampled_storage_equivalence_ready);

    let non_leaf = VoxelAddress::new(1, [0, 0, 0]).unwrap();
    let mismatch_report =
        compare_legacy_voxelis_u8_samples(&tree, &interner, &expected, [filled, non_leaf]).unwrap();
    assert_eq!(mismatch_report.compared_addresses, 1);
    assert_eq!(mismatch_report.skipped_non_leaf_addresses, vec![non_leaf]);
    assert_eq!(mismatch_report.differing_cells, vec![filled]);
    assert_eq!(mismatch_report.mismatch_count, 2);
    assert!(!mismatch_report.sampled_storage_equivalence_ready);
    assert!(!mismatch_report.exact_voxelization_ready);
}

proptest! {
    #[test]
    fn generated_legacy_voxelis_storage_samples_only_accept_matching_leaf_semantics(
        depth in 1_u8..5,
        x in 0_u64..32,
        y in 0_u64..32,
        z in 0_u64..32,
        legacy_value in 1_u8..=u8::MAX,
        mismatch_delta in 0_u8..=u8::MAX,
    ) {
        let cells = 1_u64 << depth;
        let xyz = [x % cells, y % cells, z % cells];
        let address = VoxelAddress::new(depth, xyz).unwrap();
        let expected_value = legacy_value.wrapping_add(mismatch_delta);
        let expect_match = expected_value == legacy_value;

        let mut expected = SparseVoxelGrid::new(GridFrame::builder().depth(depth).build().unwrap());
        expected
            .set(
                address,
                VoxelCell::material(MaterialRegionId(u32::from(expected_value))),
            )
            .unwrap();

        let mut interner = VoxInterner::<u8>::with_memory_budget(8192);
        let mut tree = VoxTree::<u8>::new(MaxDepth::new(depth));
        prop_assert!(tree.set(
            &mut interner,
            IVec3::new(xyz[0] as i32, xyz[1] as i32, xyz[2] as i32),
            legacy_value,
        ));

        let report =
            compare_legacy_voxelis_u8_samples(&tree, &interner, &expected, [address]).unwrap();
        prop_assert_eq!(report.sampled_addresses, 1);
        prop_assert_eq!(report.compared_addresses, 1);
        prop_assert!(report.has_compared_addresses);
        prop_assert_eq!(report.differing_cells.is_empty(), expect_match);
        prop_assert_eq!(report.sampled_storage_equivalence_ready, expect_match);
        prop_assert_eq!(report.exact_voxelization_ready, false);
        prop_assert_eq!(report.adapter.kind, LegacyAdapterKind::VoxelisStorage);
    }

    #[test]
    fn generated_legacy_voxelis_storage_samples_reject_non_leaf_evidence(
        depth in 1_u8..5,
        x in 0_u64..32,
        y in 0_u64..32,
        z in 0_u64..32,
        legacy_value in 1_u8..=u8::MAX,
    ) {
        let cells = 1_u64 << depth;
        let leaf_xyz = [x % cells, y % cells, z % cells];
        let leaf = VoxelAddress::new(depth, leaf_xyz).unwrap();
        let non_leaf = leaf.parent().unwrap();

        let mut expected = SparseVoxelGrid::new(GridFrame::builder().depth(depth).build().unwrap());
        expected
            .set(
                leaf,
                VoxelCell::material(MaterialRegionId(u32::from(legacy_value))),
            )
            .unwrap();

        let mut interner = VoxInterner::<u8>::with_memory_budget(8192);
        let mut tree = VoxTree::<u8>::new(MaxDepth::new(depth));
        prop_assert!(tree.set(
            &mut interner,
            IVec3::new(leaf_xyz[0] as i32, leaf_xyz[1] as i32, leaf_xyz[2] as i32),
            legacy_value,
        ));

        let report =
            compare_legacy_voxelis_u8_samples(&tree, &interner, &expected, [non_leaf]).unwrap();
        prop_assert_eq!(report.sampled_addresses, 1);
        prop_assert_eq!(report.compared_addresses, 0);
        prop_assert_eq!(report.skipped_non_leaf_addresses, vec![non_leaf]);
        prop_assert!(!report.has_compared_addresses);
        prop_assert!(!report.sampled_storage_equivalence_ready);
        prop_assert!(!report.exact_voxelization_ready);
    }

    #[test]
    fn generated_legacy_voxelis_chunk_paged_materialization_replays_all_leaf_cells(
        depth in 1_u8..4,
        x in 0_u64..8,
        y in 0_u64..8,
        z in 0_u64..8,
        value in 1_u8..=u8::MAX,
        shape_log2 in 0_u8..3,
    ) {
        let cells = 1_u64 << depth;
        let xyz = [x % cells, y % cells, z % cells];
        let address = VoxelAddress::new(depth, xyz).unwrap();
        let frame = GridFrame::builder().depth(depth).build().unwrap();
        let shape = ChunkShape::new(shape_log2).unwrap();

        let mut interner = VoxInterner::<u8>::with_memory_budget(8192);
        let mut tree = VoxTree::<u8>::new(MaxDepth::new(depth));
        prop_assert!(tree.set(
            &mut interner,
            IVec3::new(xyz[0] as i32, xyz[1] as i32, xyz[2] as i32),
            value,
        ));

        let (paged, report) =
            materialize_legacy_voxelis_u8_chunk_paged_storage(&tree, &interner, frame, shape).unwrap();
        let expected_cells = usize::try_from(cells.pow(3)).unwrap();
        prop_assert_eq!(report.scanned_cells, expected_cells);
        prop_assert_eq!(report.replayed_cells, expected_cells);
        prop_assert_eq!(report.materialized_cells, 1);
        prop_assert_eq!(report.empty_cells, expected_cells - 1);
        prop_assert_eq!(report.paging_mismatch_cells, 0);
        prop_assert!(report.exhaustive_chunk_port_ready);
        prop_assert_eq!(
            paged.get(address).unwrap(),
            VoxelCell::material(MaterialRegionId(u32::from(value)))
        );
        prop_assert!(!report.exact_voxelization_ready);
    }
}
