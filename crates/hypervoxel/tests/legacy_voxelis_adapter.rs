#![cfg(feature = "legacy-voxelis")]

use glam::IVec3;
use hypervoxel::{
    GridFrame, LegacyAdapterKind, MaterialRegionId, SparseVoxelGrid, VoxelAddress, VoxelCell,
    compare_legacy_voxelis_u8_samples,
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
}
