use criterion::{criterion_group, criterion_main, Criterion};
use voxelis::voxtree::{calculate_voxels_per_axis, VoxTree};

const MAX_LOD_LEVEL: usize = 7;

fn prepare_voxtree() -> VoxTree<MAX_LOD_LEVEL> {
    let mut voxtree = VoxTree::<MAX_LOD_LEVEL>::new();

    let voxels_per_axis = calculate_voxels_per_axis(MAX_LOD_LEVEL);

    for x in 0..voxels_per_axis {
        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                // Initialize voxels with some data, e.g., set all to 1
                voxtree.set_value(0, x as u8, y as u8, z as u8, 1);
            }
        }
    }

    voxtree
}

fn benchmark_update_lods_sequential(c: &mut Criterion) {
    c.bench_function("update_lods_sequential", |b| {
        let mut voxtree = prepare_voxtree();
        b.iter(|| {
            voxtree.update_lods_sequential();
        })
    });
}

fn benchmark_update_lods_parallel_clone(c: &mut Criterion) {
    c.bench_function("update_lods_parallel_clone", |b| {
        let mut voxtree = prepare_voxtree();
        b.iter(|| {
            voxtree.update_lods_parallel_clone();
        })
    });
}

criterion_group!(
    benches,
    benchmark_update_lods_sequential,
    benchmark_update_lods_parallel_clone,
);
criterion_main!(benches);
