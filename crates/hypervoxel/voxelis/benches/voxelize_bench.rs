use criterion::{criterion_group, criterion_main, Criterion};
use voxelis::{obj_reader::Obj, voxelizer::Voxelizer};

fn prepare_obj() -> Obj {
    let path = "../ad-altum/assets/statue_03.obj";
    Obj::parse(path)
}

fn benchmark_voxelize(c: &mut Criterion) {
    let obj = prepare_obj();
    let mut voxelizer = Voxelizer::new(obj);
    voxelizer.prepare_chunks();
    let face_chunk_map = voxelizer.build_face_to_chunk_map();

    c.bench_function("voxelizing", |b| {
        b.iter(|| {
            // voxelizer.clear();
            voxelizer.voxelize_mesh(&face_chunk_map);
        })
    });
}

criterion_group!(benches, benchmark_voxelize);
criterion_main!(benches);
