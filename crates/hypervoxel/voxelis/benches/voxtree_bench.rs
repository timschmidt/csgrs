use std::hint::black_box;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use glam::{IVec3, Vec3};
use rand::Rng;

use voxelis::{
    Batch, Lod, MaxDepth, VoxInterner,
    spatial::{VoxOpsBatch, VoxOpsMesh, VoxOpsRead, VoxOpsState, VoxOpsWrite, VoxTree},
    world::VoxChunk,
};

macro_rules! fill_sum {
    ($size:expr, $tree:expr, $interner:expr) => {
        for x in 0..$size as i32 {
            for y in 0..$size as i32 {
                for z in 0..$size as i32 {
                    $tree.set(
                        &mut $interner,
                        black_box(IVec3::new(x, y, z)),
                        black_box((x + y + z) % i32::MAX),
                    );
                }
            }
        }
    };
}

pub fn generate_test_sphere(
    tree: &mut VoxTree,
    interner: &mut VoxInterner<i32>,
    size: u32,
    value: i32,
) {
    let radius = (size / 2) as i32;
    let r1 = radius - 1;

    let (cx, cy, cz) = (r1, r1, r1);
    let radius_squared = radius * radius;

    for y in 0..size as i32 {
        for z in 0..size as i32 {
            for x in 0..size as i32 {
                let dx = (x - cx).abs();
                let dy = (y - cy).abs();
                let dz = (z - cz).abs();

                let distance_squared = dx * dx + dy * dy + dz * dz;

                if distance_squared <= radius_squared {
                    tree.set(interner, black_box(IVec3::new(x, y, z)), black_box(value));
                }
            }
        }
    }
}

pub fn generate_test_sphere_for_batch(
    batch: &mut Batch<i32>,
    interner: &mut VoxInterner<i32>,
    size: u32,
    value: i32,
) {
    let radius = (size / 2) as i32;
    let r1 = radius - 1;

    let (cx, cy, cz) = (r1, r1, r1);
    let radius_squared = radius * radius;

    for y in 0..size as i32 {
        for z in 0..size as i32 {
            for x in 0..size as i32 {
                let dx = (x - cx).abs();
                let dy = (y - cy).abs();
                let dz = (z - cz).abs();

                let distance_squared = dx * dx + dy * dy + dz * dz;

                if distance_squared <= radius_squared {
                    batch.set(interner, IVec3::new(x, y, z), value);
                }
            }
        }
    }
}

pub fn chunk_generate_test_sphere(
    chunk: &mut VoxChunk,
    interner: &mut VoxInterner<i32>,
    size: u32,
    value: i32,
) {
    let radius = (size / 2) as i32;
    let r1 = radius - 1;

    let (cx, cy, cz) = (r1, r1, r1);
    let radius_squared = radius * radius;

    for y in 0..size as i32 {
        for z in 0..size as i32 {
            for x in 0..size as i32 {
                let dx = (x - cx).abs();
                let dy = (y - cy).abs();
                let dz = (z - cz).abs();

                let distance_squared = dx * dx + dy * dy + dz * dz;

                if distance_squared <= radius_squared {
                    chunk.set(interner, IVec3::new(x, y, z), value);
                }
            }
        }
    }
}

pub fn generate_test_sphere_sum(tree: &mut VoxTree, interner: &mut VoxInterner<i32>, size: u32) {
    let radius = (size / 2) as i32;
    let r1 = radius - 1;

    let (cx, cy, cz) = (r1, r1, r1);
    let radius_squared = radius * radius;

    for y in 0..size as i32 {
        for z in 0..size as i32 {
            for x in 0..size as i32 {
                let dx = (x - cx).abs();
                let dy = (y - cy).abs();
                let dz = (z - cz).abs();

                let distance_squared = dx * dx + dy * dy + dz * dz;

                if distance_squared <= radius_squared {
                    tree.set(
                        interner,
                        black_box(IVec3::new(x, y, z)),
                        black_box((x + y + z) % i32::MAX),
                    );
                }
            }
        }
    }
}

#[derive(Copy, Clone)]
enum BenchType {
    Single,
    Batch,
}

impl BenchType {
    pub fn to_string(self) -> &'static str {
        match self {
            Self::Single => "single",
            Self::Batch => "batch",
        }
    }
}

fn benchmark_voxtree(c: &mut Criterion) {
    let depths: Vec<(u32, MaxDepth)> = (3..=6)
        .map(|depth| {
            let voxels_per_axis = 1 << depth;
            (voxels_per_axis as u32, MaxDepth::new(depth))
        })
        .collect();

    let bench_types = [BenchType::Single, BenchType::Batch];

    {
        let depth = depths[0].1;
        c.bench_function("voxtree_create", |b| {
            b.iter(|| {
                let _ = black_box(VoxTree::new(black_box(depth)));
            });
        });
    }

    {
        let mut group = c.benchmark_group("voxtree_fill");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024);

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                tree.fill(&mut interner, black_box(v));

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            batches[0].fill(&mut interner, 1);
                            batches[1].fill(&mut interner, 2);

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    };
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_fill_then_set_single_voxel");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                let next_v = if v + 1 != 0 { v + 1 } else { 1 };

                                tree.fill(&mut interner, black_box(v));
                                tree.set(
                                    &mut interner,
                                    black_box(IVec3::new(0, 0, 0)),
                                    black_box(next_v),
                                );

                                v = next_v;
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            batches[0].fill(&mut interner, 1);
                            batches[0].set(&mut interner, IVec3::new(0, 0, 0), 2);

                            batches[1].fill(&mut interner, 2);
                            batches[1].set(&mut interner, IVec3::new(0, 0, 0), 3);

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    };
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_single_voxel");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                let next_v = if v + 1 != 0 { v + 1 } else { 1 };

                                tree.set(
                                    &mut interner,
                                    black_box(IVec3::new(0, 0, 0)),
                                    black_box(next_v),
                                );

                                v = next_v;
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            batches[0].set(&mut interner, IVec3::new(0, 0, 0), 1);
                            batches[1].set(&mut interner, IVec3::new(0, 0, 0), 2);

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_uniform");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 24);

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for y in 0..size as i32 {
                                    for z in 0..size as i32 {
                                        for x in 0..size as i32 {
                                            tree.set(
                                                &mut interner,
                                                black_box(IVec3::new(x, y, z)),
                                                black_box(v),
                                            );
                                        }
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        batches[0].set(&mut interner, IVec3::new(x, y, z), 1);
                                        batches[1].set(&mut interner, IVec3::new(x, y, z), 2);
                                    }
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_uniform_half");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 24);

                    let half_size = size / 2;

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for y in 0..half_size as i32 {
                                    for z in 0..size as i32 {
                                        for x in 0..size as i32 {
                                            tree.set(
                                                &mut interner,
                                                black_box(IVec3::new(x, y, z)),
                                                black_box(v),
                                            );
                                        }
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..half_size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        batches[0].set(&mut interner, IVec3::new(x, y, z), 1);
                                        batches[1].set(&mut interner, IVec3::new(x, y, z), 2);
                                    }
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_sum");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 25);

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for y in 0..size as i32 {
                                    for z in 0..size as i32 {
                                        for x in 0..size as i32 {
                                            tree.set(
                                                &mut interner,
                                                black_box(IVec3::new(x, y, z)),
                                                black_box((x + y + z + v) % i32::MAX),
                                            );
                                        }
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        let position = IVec3::new(x, y, z);
                                        batches[0].set(
                                            &mut interner,
                                            position,
                                            (x + y + z + 1) % i32::MAX,
                                        );
                                        batches[1].set(
                                            &mut interner,
                                            position,
                                            (x + y + z + 1000) % i32::MAX,
                                        );
                                    }
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_checkerboard");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 25);

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for y in 0..size as i32 {
                                    for z in 0..size as i32 {
                                        for x in 0..size as i32 {
                                            if (x + y + z) % 2 == 0 {
                                                tree.set(
                                                    &mut interner,
                                                    black_box(IVec3::new(x, y, z)),
                                                    black_box(v),
                                                );
                                            }
                                        }
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        if (x + y + z) % 2 == 0 {
                                            batches[0].set(&mut interner, IVec3::new(x, y, z), 1);
                                            batches[1].set(&mut interner, IVec3::new(x, y, z), 2);
                                        }
                                    }
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_sparse_fill");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 25);

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for y in (0..size as i32).step_by(4) {
                                    for z in (0..size as i32).step_by(4) {
                                        for x in (0..size as i32).step_by(4) {
                                            tree.set(
                                                &mut interner,
                                                black_box(IVec3::new(x, y, z)),
                                                black_box(v),
                                            );
                                        }
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in (0..size as i32).step_by(4) {
                                for z in (0..size as i32).step_by(4) {
                                    for x in (0..size as i32).step_by(4) {
                                        batches[0].set(&mut interner, IVec3::new(x, y, z), 1);
                                        batches[1].set(&mut interner, IVec3::new(x, y, z), 2);
                                    }
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_gradient_fill");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 256);

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for x in 0..size as i32 {
                                    let value = (x + v) % 256;
                                    for y in 0..size as i32 {
                                        for z in 0..size as i32 {
                                            tree.set(
                                                &mut interner,
                                                black_box(IVec3::new(x, y, z)),
                                                black_box(value),
                                            );
                                        }
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for x in 0..size as i32 {
                                let value1 = (x + 1) % 256;
                                let value2 = (x + 2) % 256;
                                for y in 0..size as i32 {
                                    for z in 0..size as i32 {
                                        batches[0].set(&mut interner, IVec3::new(x, y, z), value1);
                                        batches[1].set(&mut interner, IVec3::new(x, y, z), value2);
                                    }
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_hollow_cube");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 25);

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for y in 0..size as i32 {
                                    for z in 0..size as i32 {
                                        for x in 0..size as i32 {
                                            let is_edge = x == 0
                                                || x == (size as i32) - 1
                                                || y == 0
                                                || y == (size as i32) - 1
                                                || z == 0
                                                || z == (size as i32) - 1;
                                            if is_edge {
                                                tree.set(
                                                    &mut interner,
                                                    black_box(IVec3::new(x, y, z)),
                                                    black_box(v),
                                                );
                                            }
                                        }
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        let is_edge = x == 0
                                            || x == (size as i32) - 1
                                            || y == 0
                                            || y == (size as i32) - 1
                                            || z == 0
                                            || z == (size as i32) - 1;
                                        if is_edge {
                                            batches[0].set(&mut interner, IVec3::new(x, y, z), 1);
                                            batches[1].set(&mut interner, IVec3::new(x, y, z), 2);
                                        }
                                    }
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_diagonal");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 25);

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for y in 0..size as i32 {
                                    for z in 0..size as i32 {
                                        for x in 0..size as i32 {
                                            if x == y && x == z {
                                                tree.set(
                                                    &mut interner,
                                                    black_box(IVec3::new(x, y, z)),
                                                    black_box(v),
                                                );
                                            }
                                        }
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        if x == y && x == z {
                                            batches[0].set(&mut interner, IVec3::new(x, y, z), 1);
                                            batches[1].set(&mut interner, IVec3::new(x, y, z), 2);
                                        }
                                    }
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_sphere");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let radius = (size / 2) as i32;
                    let r1 = radius - 1;

                    let (cx, cy, cz) = (r1, r1, r1);
                    let radius_squared = radius * radius;

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for y in 0..size as i32 {
                                    for z in 0..size as i32 {
                                        for x in 0..size as i32 {
                                            let dx = (x - cx).abs();
                                            let dy = (y - cy).abs();
                                            let dz = (z - cz).abs();

                                            let distance_squared = dx * dx + dy * dy + dz * dz;

                                            if distance_squared <= radius_squared {
                                                tree.set(
                                                    &mut interner,
                                                    black_box(IVec3::new(x, y, z)),
                                                    black_box(v),
                                                );
                                            }
                                        }
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        let dx = (x - cx).abs();
                                        let dy = (y - cy).abs();
                                        let dz = (z - cz).abs();

                                        let distance_squared = dx * dx + dy * dy + dz * dz;

                                        if distance_squared <= radius_squared {
                                            batches[0].set(&mut interner, IVec3::new(x, y, z), 1);
                                            batches[1].set(&mut interner, IVec3::new(x, y, z), 2);
                                        }
                                    }
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_terrain_surface_only");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

                    let mut noise = fastnoise_lite::FastNoiseLite::new();
                    noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        let y = ((noise.get_noise_2d(
                                            x as f32 / size as f32,
                                            z as f32 / size as f32,
                                        ) + 1.0)
                                            / 2.0)
                                            * size as f32;
                                        let y = y as i32;
                                        debug_assert!(y < size as i32);
                                        tree.set(
                                            &mut interner,
                                            black_box(IVec3::new(x, y, z)),
                                            black_box(v),
                                        );
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    let y = ((noise.get_noise_2d(
                                        x as f32 / size as f32,
                                        z as f32 / size as f32,
                                    ) + 1.0)
                                        / 2.0)
                                        * size as f32;
                                    let y = y as i32;
                                    debug_assert!(y < size as i32);
                                    batches[0].set(&mut interner, IVec3::new(x, y, z), 1);
                                    batches[1].set(&mut interner, IVec3::new(x, y, z), 2);
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_terrain_surface_and_below");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 14);

                    let mut noise = fastnoise_lite::FastNoiseLite::new();
                    noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

                    match bench_type {
                        BenchType::Single => {
                            let mut v = 1;

                            b.iter(|| {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        let height = ((noise.get_noise_2d(
                                            x as f32 / size as f32,
                                            z as f32 / size as f32,
                                        ) + 1.0)
                                            / 2.0)
                                            * size as f32;
                                        let height = height as i32;
                                        debug_assert!(height < size as i32);
                                        for y in 0..=height {
                                            tree.set(
                                                &mut interner,
                                                black_box(IVec3::new(x, y, z)),
                                                black_box(v),
                                            );
                                        }
                                    }
                                }

                                v += 1;
                                if v == 0 {
                                    v = 1;
                                }
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    let height = ((noise.get_noise_2d(
                                        x as f32 / size as f32,
                                        z as f32 / size as f32,
                                    ) + 1.0)
                                        / 2.0)
                                        * size as f32;
                                    let height = height as i32;
                                    debug_assert!(height < size as i32);
                                    for y in 0..=height {
                                        batches[0].set(&mut interner, IVec3::new(x, y, z), 1);
                                        batches[1].set(&mut interner, IVec3::new(x, y, z), 2);
                                    }
                                }
                            }

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_random_position_same_value");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 24);

                    let mut rng = rand::rng();

                    let value = 1;

                    match bench_type {
                        BenchType::Single => {
                            b.iter(|| {
                                let x = rng.random_range(0..size as i32);
                                let y = rng.random_range(0..size as i32);
                                let z = rng.random_range(0..size as i32);

                                tree.set(
                                    &mut interner,
                                    black_box(IVec3::new(x, y, z)),
                                    black_box(value),
                                );
                            });
                        }
                        BenchType::Batch => {
                            // TODO(aljen): fix this case
                            // let mut batch = tree.create_batch();

                            b.iter(|| {
                                let x = rng.random_range(0..size as i32);
                                let y = rng.random_range(0..size as i32);
                                let z = rng.random_range(0..size as i32);

                                let mut batch = tree.create_batch();

                                batch.set(&mut interner, IVec3::new(x, y, z), value);

                                tree.apply_batch(&mut interner, black_box(&batch));

                                // batch.set(&mut interner, IVec3::new(x, y, z), 0);
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_set_random_position_and_value");

        for &(size, depth) in depths.iter() {
            for bench_type in bench_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), bench_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut tree = VoxTree::new(depth);
                    let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 185);

                    let mut rng = rand::rng();

                    match bench_type {
                        BenchType::Single => {
                            b.iter(|| {
                                let x = rng.random_range(0..size as i32);
                                let y = rng.random_range(0..size as i32);
                                let z = rng.random_range(0..size as i32);
                                let value = rng.random_range(1..i32::MAX);

                                tree.set(
                                    &mut interner,
                                    black_box(IVec3::new(x, y, z)),
                                    black_box(value),
                                );
                            });
                        }
                        BenchType::Batch => {
                            let mut batch = tree.create_batch();

                            b.iter(|| {
                                let x = rng.random_range(0..size as i32);
                                let y = rng.random_range(0..size as i32);
                                let z = rng.random_range(0..size as i32);
                                let value = rng.random_range(1..i32::MAX);

                                batch.set(&mut interner, IVec3::new(x, y, z), value);

                                tree.apply_batch(&mut interner, black_box(&batch));

                                batch.set(&mut interner, IVec3::new(x, y, z), 0);
                            });
                        }
                    }
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_get_empty");

        for &(size, depth) in depths.iter() {
            group.bench_with_input(size.to_string(), &depth, |b, &depth| {
                let tree = VoxTree::new(depth);
                let interner = VoxInterner::<i32>::with_memory_budget(1024);

                b.iter(|| {
                    for y in 0..size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                let _ =
                                    black_box(tree.get(&interner, black_box(IVec3::new(x, y, z))));
                            }
                        }
                    }
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_get_sphere_uniform");

        for &(size, depth) in depths.iter() {
            group.bench_with_input(size.to_string(), &depth, |b, &depth| {
                let mut tree = VoxTree::new(depth);
                let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 14);

                generate_test_sphere(&mut tree, &mut interner, size, 1);

                b.iter(|| {
                    for y in 0..size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                let _ =
                                    black_box(tree.get(&interner, black_box(IVec3::new(x, y, z))));
                            }
                        }
                    }
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_get_sphere_sum");

        for &(size, depth) in depths.iter() {
            group.bench_with_input(size.to_string(), &depth, |b, &depth| {
                let mut tree = VoxTree::new(depth);
                let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 14);

                generate_test_sphere_sum(&mut tree, &mut interner, size);

                b.iter(|| {
                    for y in 0..size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                let _ =
                                    black_box(tree.get(&interner, black_box(IVec3::new(x, y, z))));
                            }
                        }
                    }
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_get_full_uniform");

        for &(size, depth) in depths.iter() {
            group.bench_with_input(size.to_string(), &depth, |b, &depth| {
                let mut tree = VoxTree::new(depth);
                let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

                tree.fill(&mut interner, 1);

                b.iter(|| {
                    for y in 0..size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                let _ =
                                    black_box(tree.get(&interner, black_box(IVec3::new(x, y, z))));
                            }
                        }
                    }
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_get_full_sum");

        for &(size, depth) in depths.iter() {
            group.bench_with_input(size.to_string(), &depth, |b, &depth| {
                let mut tree = VoxTree::new(depth);
                let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 24);

                fill_sum!(size, tree, interner);

                b.iter(|| {
                    for y in 0..size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                let _ =
                                    black_box(tree.get(&interner, black_box(IVec3::new(x, y, z))));
                            }
                        }
                    }
                })
            });
        }

        group.finish();
    }

    {
        c.bench_function("voxtree_is_empty_empty", |b| {
            let depth = depths[0].1;
            let tree = VoxTree::new(depth);

            b.iter(|| {
                let _ = black_box(tree.is_empty());
            });
        });
    }

    {
        c.bench_function("voxtree_is_empty_not_empty", |b| {
            let depth = depths[0].1;
            let mut tree = VoxTree::new(depth);
            let mut interner = VoxInterner::<i32>::with_memory_budget(1024);
            tree.fill(&mut interner, 1);

            b.iter(|| {
                let _ = black_box(tree.is_empty());
            });
        });
    }

    {
        let mut group = c.benchmark_group("voxtree_clear_empty");

        for &(size, depth) in depths.iter() {
            group.bench_with_input(size.to_string(), &depth, |b, &depth| {
                let mut tree = VoxTree::new(depth);
                let mut interner = VoxInterner::<i32>::with_memory_budget(1024);

                b.iter(|| {
                    tree.clear(black_box(&mut interner));
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_clear_sphere");

        for &(size, depth) in depths.iter() {
            group.bench_with_input(size.to_string(), &depth, |b, &depth| {
                let mut tree = VoxTree::new(depth);
                let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 22);

                let mut batch = tree.create_batch();
                generate_test_sphere_for_batch(&mut batch, &mut interner, size, 1);

                b.iter(|| {
                    tree.apply_batch(&mut interner, black_box(&batch));
                    tree.clear(&mut interner);
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_clear_filled");

        for &(size, depth) in depths.iter() {
            group.bench_with_input(size.to_string(), &depth, |b, &depth| {
                let mut tree = VoxTree::new(depth);
                let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

                b.iter(|| {
                    tree.fill(&mut interner, 1);
                    tree.clear(&mut interner);
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_to_vec_empty");

        for &(size, depth) in depths.iter() {
            let tree = VoxTree::new(depth);
            let interner = VoxInterner::<i32>::with_memory_budget(1024);

            for lod in 0..depth.max() {
                let bench_id = BenchmarkId::new(size.to_string(), format!("LOD_{}", lod));
                group.bench_with_input(bench_id, &depth, |b, _| {
                    let lod = Lod::new(lod);

                    b.iter(|| {
                        let _ = black_box(tree.to_vec(&interner, black_box(lod)));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_to_vec_sphere");

        for &(size, depth) in depths.iter() {
            let mut tree = VoxTree::new(depth);
            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 14);

            let mut batch = tree.create_batch();
            generate_test_sphere_for_batch(&mut batch, &mut interner, size, 1);
            tree.apply_batch(&mut interner, &batch);

            for lod in 0..depth.max() {
                let bench_id = BenchmarkId::new(size.to_string(), format!("LOD_{}", lod));
                group.bench_with_input(bench_id, &depth, |b, _| {
                    let lod = Lod::new(lod);

                    b.iter(|| {
                        let _ = black_box(tree.to_vec(&interner, black_box(lod)));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_to_vec_uniform");

        for &(size, depth) in depths.iter() {
            let mut tree = VoxTree::new(depth);
            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            tree.fill(&mut interner, 1);

            for lod in 0..depth.max() {
                let bench_id = BenchmarkId::new(size.to_string(), format!("LOD_{}", lod));
                group.bench_with_input(bench_id, &depth, |b, _| {
                    let lod = Lod::new(lod);

                    b.iter(|| {
                        let _ = black_box(tree.to_vec(&interner, black_box(lod)));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_to_vec_sum");

        for &(size, depth) in depths.iter() {
            let mut tree = VoxTree::new(depth);
            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 24);

            fill_sum!(size, tree, interner);

            for lod in 0..depth.max() {
                let bench_id = BenchmarkId::new(size.to_string(), format!("LOD_{}", lod));
                group.bench_with_input(bench_id, &depth, |b, _| {
                    let lod = Lod::new(lod);

                    b.iter(|| {
                        let _ = black_box(tree.to_vec(&interner, black_box(lod)));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_to_vec_terrain");

        for &(size, depth) in depths.iter() {
            let mut tree = VoxTree::new(depth);
            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut noise = fastnoise_lite::FastNoiseLite::new();
            noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

            let mut batch = tree.create_batch();

            for x in 0..size as i32 {
                for z in 0..size as i32 {
                    let y = ((noise.get_noise_2d(x as f32 / size as f32, z as f32 / size as f32)
                        + 1.0)
                        / 2.0)
                        * size as f32;
                    let y = y as i32;
                    debug_assert!(y < size as i32);
                    batch.set(&mut interner, IVec3::new(x, y, z), 1);
                }
            }

            tree.apply_batch(&mut interner, &batch);

            for lod in 0..depth.max() {
                let bench_id = BenchmarkId::new(size.to_string(), format!("LOD_{}", lod));
                group.bench_with_input(bench_id, &depth, |b, _| {
                    let lod = Lod::new(lod);

                    b.iter(|| {
                        let _ = black_box(tree.to_vec(&interner, black_box(lod)));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_naive_mesh_sphere");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            chunk_generate_test_sphere(&mut chunk, &mut interner, size, 1);

            for lod in 0..depth.max() {
                let bench_id = BenchmarkId::new(size.to_string(), format!("LOD_{}", lod));
                group.bench_with_input(bench_id, &depth, |b, _| {
                    let offset = Vec3::ZERO;
                    let mut vertices = Vec::new();
                    let mut normals = Vec::new();
                    let mut indices = Vec::new();

                    let lod = Lod::new(lod);

                    b.iter(|| {
                        vertices.clear();
                        normals.clear();
                        indices.clear();

                        chunk.generate_mesh_arrays(
                            &interner,
                            black_box(&mut vertices),
                            black_box(&mut normals),
                            black_box(&mut indices),
                            black_box(offset),
                            black_box(lod),
                        );
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_naive_mesh_terrain");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut noise = fastnoise_lite::FastNoiseLite::new();
            noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

            let mut batch = chunk.create_batch();

            for x in 0..size as i32 {
                for z in 0..size as i32 {
                    let y = ((noise.get_noise_2d(x as f32 / size as f32, z as f32 / size as f32)
                        + 1.0)
                        / 2.0)
                        * size as f32;
                    let y = y as i32;
                    debug_assert!(y < size as i32);
                    batch.set(&mut interner, IVec3::new(x, y, z), 1);
                }
            }

            chunk.apply_batch(&mut interner, &batch);

            for lod in 0..depth.max() {
                let bench_id = BenchmarkId::new(size.to_string(), format!("LOD_{}", lod));
                group.bench_with_input(bench_id, &depth, |b, _| {
                    let offset = Vec3::ZERO;
                    let mut vertices = Vec::new();
                    let mut normals = Vec::new();
                    let mut indices = Vec::new();

                    let lod = Lod::new(lod);

                    b.iter(|| {
                        vertices.clear();
                        normals.clear();
                        indices.clear();

                        chunk.generate_mesh_arrays(
                            &interner,
                            black_box(&mut vertices),
                            black_box(&mut normals),
                            black_box(&mut indices),
                            black_box(offset),
                            black_box(lod),
                        );
                    });
                });
            }
        }

        group.finish();
    }
}

criterion_group!(benches, benchmark_voxtree);
criterion_main!(benches);
