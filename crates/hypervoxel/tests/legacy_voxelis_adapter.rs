#![cfg(feature = "legacy-voxelis")]

use glam::IVec3;
use hypervoxel::{
    ChunkShape, GridFrame, MaterialRegionId, VoxelAddress, VoxelCell,
    materialize_legacy_voxelis_u8_chunk_paged_storage,
    materialize_legacy_voxelis_u8_exact_surface_triangle_mesh,
};
use proptest::prelude::*;
use voxelis::{
    MaxDepth, VoxInterner,
    spatial::{VoxOpsWrite, VoxTree},
};

#[test]
fn legacy_storage_materializes_directly_into_chunk_pages() {
    let frame = GridFrame::builder().depth(3).build().unwrap();
    let shape = ChunkShape::new(1).unwrap();
    let mut interner = VoxInterner::<u8>::with_memory_budget(16_384);
    let mut tree = VoxTree::<u8>::new(MaxDepth::new(3));
    let cells = [([0, 0, 0], 1_u8), ([2, 3, 4], 9_u8), ([7, 7, 7], 255_u8)];

    for (xyz, value) in cells {
        assert!(tree.set(&mut interner, IVec3::new(xyz[0], xyz[1], xyz[2]), value,));
    }

    let paged =
        materialize_legacy_voxelis_u8_chunk_paged_storage(&tree, &interner, frame, shape).unwrap();
    assert_eq!(paged.len(), cells.len());
    for (xyz, value) in cells {
        let address = VoxelAddress::new(3, xyz.map(|coordinate| coordinate as u64)).unwrap();
        assert_eq!(
            paged.get(address).unwrap(),
            VoxelCell::material(MaterialRegionId(u32::from(value)))
        );
    }
}

#[test]
fn legacy_storage_rejects_depth_mismatch() {
    let frame = GridFrame::builder().depth(3).build().unwrap();
    let interner = VoxInterner::<u8>::with_memory_budget(4096);
    let tree = VoxTree::<u8>::new(MaxDepth::new(2));
    assert!(
        materialize_legacy_voxelis_u8_chunk_paged_storage(
            &tree,
            &interner,
            frame,
            ChunkShape::new(2).unwrap(),
        )
        .is_err()
    );
}

#[test]
fn legacy_storage_builds_a_direct_exact_surface_mesh() {
    let frame = GridFrame::builder().depth(2).build().unwrap();
    let mut interner = VoxInterner::<u8>::with_memory_budget(4096);
    let mut tree = VoxTree::<u8>::new(MaxDepth::new(2));
    assert!(tree.set(&mut interner, IVec3::new(1, 1, 1), 4));

    let (paged, surface) = materialize_legacy_voxelis_u8_exact_surface_triangle_mesh(
        &tree,
        &interner,
        frame,
        ChunkShape::new(1).unwrap(),
    )
    .unwrap();
    assert_eq!(paged.len(), 1);
    assert_eq!(surface.vertices.len(), 8);
    assert_eq!(surface.triangles.len(), 12);
}

#[test]
fn empty_legacy_storage_has_no_surface_mesh() {
    let frame = GridFrame::builder().depth(2).build().unwrap();
    let interner = VoxInterner::<u8>::with_memory_budget(4096);
    let tree = VoxTree::<u8>::new(MaxDepth::new(2));
    assert!(
        materialize_legacy_voxelis_u8_exact_surface_triangle_mesh(
            &tree,
            &interner,
            frame,
            ChunkShape::new(1).unwrap(),
        )
        .is_err()
    );
}

proptest! {
    #[test]
    fn generated_leaf_values_round_trip_through_direct_materialization(
        x in 0_i32..4,
        y in 0_i32..4,
        z in 0_i32..4,
        value in 1_u8..=u8::MAX,
    ) {
        let frame = GridFrame::builder().depth(2).build().unwrap();
        let mut interner = VoxInterner::<u8>::with_memory_budget(4096);
        let mut tree = VoxTree::<u8>::new(MaxDepth::new(2));
        prop_assert!(tree.set(&mut interner, IVec3::new(x, y, z), value));

        let paged = materialize_legacy_voxelis_u8_chunk_paged_storage(
            &tree,
            &interner,
            frame,
            ChunkShape::new(1).unwrap(),
        ).unwrap();
        let address = VoxelAddress::new(2, [x as u64, y as u64, z as u64]).unwrap();
        prop_assert_eq!(
            paged.get(address).unwrap(),
            VoxelCell::material(MaterialRegionId(u32::from(value)))
        );
    }
}
