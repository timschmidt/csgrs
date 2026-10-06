#![no_main]

use hyperreal::Real;
use hypervoxel::{
    ChunkPagedSparseGrid, ChunkShape, ContinuousFieldVoxelBatch, ContinuousFieldVoxelCell,
    ExactBox, GridFrame, MaterialRegionId, SparseVoxelGrid, SvoVoxelGrid, VoxelAddress, VoxelCell,
    VoxelEditBatch, VoxelizationPolicy,
    continuous_field_address, exact_voxel_surface_triangle_mesh_from_faces,
    extract_chunk_paged_exposed_faces, extract_exposed_faces, greedy_face_patches,
    lossy_quad_mesh_from_faces, voxelize_exact_box,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: (u8, u64, u64, u64)| {
    let (depth_raw, x, y, z) = data;
    let depth = depth_raw % 9;
    let frame = GridFrame::unit(depth).unwrap();
    let cells = 1_u64 << depth;
    let address = VoxelAddress::new(depth, [x % cells, y % cells, z % cells]).unwrap();

    assert_eq!(
        VoxelAddress::from_morton_code(depth, address.morton_code()).unwrap(),
        address
    );
    assert_eq!(
        VoxelAddress::from_child_path(&address.child_path()).unwrap(),
        address
    );
    let bounds = address.bounds(&frame).unwrap();
    for axis in 0..3 {
        assert_eq!(bounds.extent(axis), Real::from(1));
    }

    let material = VoxelCell::material(MaterialRegionId((depth_raw % 7) as u32));
    let mut sparse = SparseVoxelGrid::new(frame.clone());
    sparse.set(address, material).unwrap();
    assert_eq!(sparse.get(address).unwrap(), material);

    let chunk_log2 = depth.min(3);
    let paged =
        ChunkPagedSparseGrid::from_sparse_grid(&sparse, ChunkShape::new(chunk_log2).unwrap())
            .unwrap();
    assert_eq!(paged.get(address).unwrap(), material);

    let svo = SvoVoxelGrid::from_sparse_grid(&sparse).unwrap();
    assert_eq!(svo.get(address).unwrap(), material);
    assert_eq!(svo.to_sparse_grid().unwrap(), sparse);

    assert_eq!(sparse.query_occupancy(address).unwrap().cell, material);
    assert!(
        sparse
            .query_connected_component(address)
            .unwrap()
            .addresses
            .contains(&address)
    );

    let faces = extract_exposed_faces(&sparse).unwrap();
    assert_eq!(faces.len(), 6);
    assert_eq!(extract_chunk_paged_exposed_faces(&paged).unwrap(), faces);
    assert!(!greedy_face_patches(&faces).is_empty());
    assert_eq!(
        lossy_quad_mesh_from_faces(&faces).unwrap().triangles.len(),
        12
    );
    assert_eq!(
        exact_voxel_surface_triangle_mesh_from_faces(&faces)
            .unwrap()
            .triangles
            .len(),
        12
    );

    let mut batch_grid = SparseVoxelGrid::new(frame.clone());
    let mut batch = VoxelEditBatch::new();
    batch.push(address, material);
    batch.push(address, VoxelCell::empty());
    batch.apply_to(&mut batch_grid).unwrap();
    assert!(batch_grid.is_empty());

    let intake_depth = (depth_raw % 3) + 1;
    let intake_frame = GridFrame::unit(intake_depth).unwrap();
    let intake_cells = intake_frame.cells_per_axis();
    let mut rows = Vec::new();
    for iz in 0..intake_cells {
        for iy in 0..intake_cells {
            for ix in 0..intake_cells {
                rows.push(ContinuousFieldVoxelCell::new(
                    continuous_field_address(&intake_frame, [ix, iy, iz]).unwrap(),
                    material,
                ));
            }
        }
    }
    let intake = ContinuousFieldVoxelBatch {
        frame: intake_frame,
        cells: rows,
    }
    .materialize_exact_sparse_grid()
    .unwrap();
    assert_eq!(intake.len() as u64, intake_cells.pow(3));

    let voxel_depth = (depth_raw % 3) + 2;
    let voxel_frame = GridFrame::unit(voxel_depth).unwrap();
    let voxel_cells = 1_u64 << voxel_depth;
    let lo = 1_u64;
    let hi = voxel_cells - 1;
    let solid = ExactBox::new(
        [Real::from(lo), Real::from(lo), Real::from(lo)],
        [Real::from(hi), Real::from(hi), Real::from(hi)],
    );
    let (grid, report) = voxelize_exact_box(
        voxel_frame,
        &solid,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert_eq!(report.unknown_cells, 0);
    assert!(grid.len() <= report.aggregate.child_count);
});
