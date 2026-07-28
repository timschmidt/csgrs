use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Rational, Real};
use hypervoxel::{
    ExactBox, GridFrame, MaterialRegionId, QueryRegion, SparseVoxelGrid, SvoVoxelGrid,
    VoxelAddress, VoxelCell, VoxelizationPolicy, exact_voxel_surface_triangle_mesh_from_faces,
    extract_exposed_faces, lossy_quad_mesh_from_faces, voxelize_exact_box,
};

fn r(n: i32) -> Real {
    n.into()
}

fn frame(depth: u8) -> GridFrame {
    GridFrame::builder()
        .origin([r(0), r(0), r(0)])
        .pitch([
            Rational::fraction(1, 8).unwrap().into(),
            Rational::fraction(1, 8).unwrap().into(),
            Rational::fraction(1, 8).unwrap().into(),
        ])
        .depth(depth)
        .build()
        .unwrap()
}

fn populated_sparse_grid(depth: u8) -> SparseVoxelGrid {
    let mut grid = SparseVoxelGrid::new(frame(depth));
    let cells = 1_u64 << depth;
    for i in 0..cells.min(64) {
        grid.set(
            VoxelAddress::new(depth, [i, (i * 3) % cells, (i * 7) % cells]).unwrap(),
            VoxelCell::material(MaterialRegionId((i % 4) as u32)),
        )
        .unwrap();
    }
    grid
}

fn bench_cell_bounds(c: &mut Criterion) {
    let frame = frame(8);
    let address = VoxelAddress::new(8, [173, 91, 207]).unwrap();
    c.bench_function("exact_cell_bounds", |b| {
        b.iter(|| black_box(address).bounds(black_box(&frame)).unwrap())
    });
}

fn bench_sparse_edits(c: &mut Criterion) {
    let frame = frame(8);
    let address = VoxelAddress::new(8, [73, 91, 107]).unwrap();
    c.bench_function("sparse_set_remove", |b| {
        b.iter_batched(
            || SparseVoxelGrid::new(frame.clone()),
            |mut grid| {
                grid.set(address, VoxelCell::material(MaterialRegionId(7)))
                    .unwrap();
                grid.set(address, VoxelCell::empty()).unwrap();
                black_box(grid)
            },
            BatchSize::SmallInput,
        )
    });
}

fn bench_exact_box_voxelization(c: &mut Criterion) {
    let frame = frame(6);
    let solid = ExactBox::new([r(2), r(2), r(2)], [r(6), r(6), r(6)]);
    c.bench_function("exact_box_voxelization", |b| {
        b.iter(|| {
            voxelize_exact_box(
                black_box(frame.clone()),
                black_box(&solid),
                MaterialRegionId(3),
                VoxelizationPolicy::conservative_cover(),
            )
            .unwrap()
        })
    });
}

fn bench_svo_compaction_and_expansion(c: &mut Criterion) {
    let sparse = populated_sparse_grid(8);
    c.bench_function("sparse_to_svo", |b| {
        b.iter(|| SvoVoxelGrid::from_sparse_grid(black_box(&sparse)).unwrap())
    });

    let svo = SvoVoxelGrid::from_sparse_grid(&sparse).unwrap();
    c.bench_function("svo_to_sparse", |b| {
        b.iter(|| black_box(&svo).to_sparse_grid().unwrap())
    });
}

fn bench_surface_paths(c: &mut Criterion) {
    let frame = GridFrame::builder().depth(5).build().unwrap();
    let solid = ExactBox::new([r(4), r(4), r(4)], [r(20), r(20), r(20)]);
    let (grid, _) = voxelize_exact_box(
        frame,
        &solid,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    c.bench_function("extract_exposed_faces", |b| {
        b.iter(|| extract_exposed_faces(black_box(&grid)).unwrap())
    });

    let faces = extract_exposed_faces(&grid).unwrap();
    c.bench_function("exact_surface_triangle_mesh", |b| {
        b.iter(|| exact_voxel_surface_triangle_mesh_from_faces(black_box(&faces)).unwrap())
    });
    c.bench_function("lossy_preview_quad_mesh", |b| {
        b.iter(|| lossy_quad_mesh_from_faces(black_box(&faces)).unwrap())
    });
}

fn bench_grid_queries(c: &mut Criterion) {
    let grid = populated_sparse_grid(8);
    let region = QueryRegion {
        min: [0, 0, 0],
        max: [127, 127, 127],
        depth: 8,
    };
    c.bench_function("region_aggregate", |b| {
        b.iter(|| grid.query_region_aggregate(black_box(&region)))
    });
}

fn bench_hypermesh_exact_adapter(c: &mut Criterion) {
    #[cfg(feature = "hypermesh-adapter")]
    {
        use hypermesh::{InputMesh, Point3, Real, Triangle};
        use hypervoxel::adapt_hypermesh_exact_solid;

        let point = |x, y, z| Point3::new(Real::from(x), Real::from(y), Real::from(z));
        let mesh = InputMesh::new(
            vec![
                point(0, 0, 0),
                point(2, 0, 0),
                point(0, 2, 0),
                point(0, 0, 2),
            ],
            vec![
                Triangle::new(0, 2, 1),
                Triangle::new(0, 1, 3),
                Triangle::new(1, 2, 3),
                Triangle::new(2, 0, 3),
            ],
        );
        c.bench_function("hypermesh_exact_solid_adapter", |b| {
            b.iter(|| adapt_hypermesh_exact_solid(black_box(&mesh)).unwrap())
        });
    }
    #[cfg(not(feature = "hypermesh-adapter"))]
    {
        let _ = c;
    }
}

criterion_group!(
    benches,
    bench_cell_bounds,
    bench_sparse_edits,
    bench_exact_box_voxelization,
    bench_svo_compaction_and_expansion,
    bench_surface_paths,
    bench_grid_queries,
    bench_hypermesh_exact_adapter,
);
criterion_main!(benches);
