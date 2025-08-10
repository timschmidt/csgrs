use std::hint::black_box;

use criterion::{
    BenchmarkGroup, BenchmarkId, Criterion, criterion_group, criterion_main,
    measurement::Measurement,
};
use glam::{IVec3, Vec3};
use rand::Rng;

use voxelis::{
    Batch, Lod, MaxDepth, VoxInterner,
    spatial::{
        VoxOpsBatch, VoxOpsBulkWrite, VoxOpsConfig, VoxOpsMesh, VoxOpsRead, VoxOpsState,
        VoxOpsWrite, VoxTree,
    },
    utils::{
        common::to_vec,
        mesh::MeshData,
        shapes::{
            generate_checkerboard_batch, generate_corners_batch, generate_diagonal_batch,
            generate_hollow_cube_batch, generate_perlin_3d_batch, generate_sparse_fill_batch,
            generate_sphere_batch, generate_terrain_batch, generate_terrain_batch_3_mats,
        },
    },
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
    tree: &mut VoxTree<i32>,
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

pub fn generate_test_sphere_for_batch(batch: &mut Batch<i32>, size: u32, value: i32) {
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
                    batch.just_set(IVec3::new(x, y, z), value);
                }
            }
        }
    }
}

pub fn chunk_generate_test_sphere(
    chunk: &mut VoxChunk<i32>,
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

pub fn generate_test_sphere_sum(
    tree: &mut VoxTree<i32>,
    interner: &mut VoxInterner<i32>,
    size: u32,
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

#[derive(Copy, Clone)]
enum MeshType {
    Naive,
    Greedy,
}

impl BenchType {
    pub fn to_string(self) -> &'static str {
        match self {
            Self::Single => "single",
            Self::Batch => "batch",
        }
    }
}

impl MeshType {
    pub fn to_string(self) -> &'static str {
        match self {
            Self::Naive => "naive",
            Self::Greedy => "greedy",
        }
    }
}

fn benchmark_meshing<M: Measurement>(
    group: &mut BenchmarkGroup<'_, M>,
    size: u32,
    depth: MaxDepth,
    max_lod: u8,
    mesh_types: &[MeshType],
    interner: &VoxInterner<i32>,
    chunk: &VoxChunk<i32>,
) {
    for lod in 0..max_lod {
        for mesh_type in mesh_types.iter() {
            let bench_id = BenchmarkId::new(
                size.to_string(),
                format!("LOD_{lod}/{}", mesh_type.to_string()),
            );

            let offset = Vec3::ZERO;
            let mut mesh_data = MeshData::default();

            let lod = Lod::new(lod);

            match mesh_type {
                MeshType::Naive => {
                    group.bench_with_input(bench_id, &depth, |b, _| {
                        b.iter(|| {
                            mesh_data.clear();

                            chunk.generate_naive_mesh_arrays(
                                interner,
                                black_box(&mut mesh_data),
                                black_box(offset),
                                black_box(lod),
                            );

                            #[cfg(feature = "tracy")]
                            tracy_client::frame_mark();
                        });
                    });
                }
                MeshType::Greedy => {
                    group.bench_with_input(bench_id, &depth, |b, _| {
                        b.iter(|| {
                            mesh_data.clear();

                            chunk.generate_greedy_mesh_arrays(
                                interner,
                                black_box(&mut mesh_data),
                                black_box(offset),
                                black_box(lod),
                            );

                            #[cfg(feature = "tracy")]
                            tracy_client::frame_mark();
                        });
                    });
                }
            }
        }
    }
}

fn benchmark_to_vec<M: Measurement>(
    group: &mut BenchmarkGroup<'_, M>,
    size: u32,
    depth: MaxDepth,
    max_lod: u8,
    interner: &VoxInterner<i32>,
    tree: &VoxTree<i32>,
) {
    for lod in 0..max_lod {
        let bench_id = BenchmarkId::new(size.to_string(), format!("LOD_{lod}"));
        group.bench_with_input(bench_id, &depth, |b, _| {
            let lod = Lod::new(lod);

            let max_depth = tree.max_depth(lod);

            b.iter(|| {
                let _ = black_box(to_vec(
                    interner,
                    black_box(&tree.get_root_id()),
                    black_box(max_depth),
                ));
            });
        });
    }
}

fn benchmark_voxtree(c: &mut Criterion) {
    const MIN_DEPTH: u8 = 3;
    const MAX_DEPTH: u8 = 6;

    let min_depth_env = std::env::var("VOXTREE_MIN_DEPTH");
    let min_depth = match min_depth_env {
        Ok(val) => val.parse::<u8>().unwrap_or(MIN_DEPTH),
        Err(_) => MIN_DEPTH,
    };
    let max_depth_env = std::env::var("VOXTREE_MAX_DEPTH");
    let max_depth = match max_depth_env {
        Ok(val) => val.parse::<u8>().unwrap_or(MAX_DEPTH),
        Err(_) => MAX_DEPTH,
    };
    let bench_types_env = std::env::var("VOXTREE_BENCHMARK_TYPES");
    let bench_types = match bench_types_env {
        Ok(val) => {
            if val == "single" {
                vec![BenchType::Single]
            } else if val == "batch" {
                vec![BenchType::Batch]
            } else {
                vec![BenchType::Single, BenchType::Batch]
            }
        }
        Err(_) => vec![BenchType::Single, BenchType::Batch],
    };
    let mesh_types_env = std::env::var("VOXTREE_MESH_TYPES");
    let mesh_types = match mesh_types_env {
        Ok(val) => {
            if val == "naive" {
                vec![MeshType::Naive]
            } else if val == "greedy" {
                vec![MeshType::Greedy]
            } else {
                vec![MeshType::Naive, MeshType::Greedy]
            }
        }
        Err(_) => vec![MeshType::Naive, MeshType::Greedy],
    };
    let max_lod_env = std::env::var("VOXTREE_MAX_LOD");
    let max_lod = match max_lod_env {
        Ok(val) => val.parse::<u8>().unwrap_or(max_depth),
        Err(_) => max_depth,
    };

    let max_lod = max_lod.clamp(1, max_depth);

    let depths: Vec<(u32, MaxDepth)> = (min_depth..=max_depth)
        .map(|depth| {
            let voxels_per_axis = 1 << depth;
            (voxels_per_axis as u32, MaxDepth::new(depth))
        })
        .collect();

    {
        let depth = depths[0].1;
        c.bench_function("voxtree_create", |b| {
            b.iter(|| {
                let _ = black_box(VoxTree::<i32>::new(black_box(depth)));

                #[cfg(feature = "tracy")]
                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            batches[0].just_fill(1);
                            batches[1].just_fill(2);

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            batches[0].just_fill(1);
                            batches[0].just_set(IVec3::new(0, 0, 0), 2);

                            batches[1].just_fill(2);
                            batches[1].just_set(IVec3::new(0, 0, 0), 3);

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            batches[0].just_set(IVec3::new(0, 0, 0), 1);
                            batches[1].just_set(IVec3::new(0, 0, 0), 2);

                            let mut batch_idx = 0;

                            b.iter(|| {
                                tree.apply_batch(&mut interner, black_box(&batches[batch_idx]));

                                if batch_idx == 0 {
                                    batch_idx = 1;
                                } else {
                                    batch_idx = 0;
                                }

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        batches[0].just_set(IVec3::new(x, y, z), 1);
                                        batches[1].just_set(IVec3::new(x, y, z), 2);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..half_size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        batches[0].just_set(IVec3::new(x, y, z), 1);
                                        batches[1].just_set(IVec3::new(x, y, z), 2);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        let position = IVec3::new(x, y, z);
                                        batches[0].just_set(position, (x + y + z + 1) % i32::MAX);
                                        batches[1]
                                            .just_set(position, (x + y + z + 1000) % i32::MAX);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        if (x + y + z) % 2 == 0 {
                                            batches[0].just_set(IVec3::new(x, y, z), 1);
                                            batches[1].just_set(IVec3::new(x, y, z), 2);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in (0..size as i32).step_by(4) {
                                for z in (0..size as i32).step_by(4) {
                                    for x in (0..size as i32).step_by(4) {
                                        batches[0].just_set(IVec3::new(x, y, z), 1);
                                        batches[1].just_set(IVec3::new(x, y, z), 2);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for x in 0..size as i32 {
                                let value1 = (x + 1) % 256;
                                let value2 = (x + 2) % 256;
                                for y in 0..size as i32 {
                                    for z in 0..size as i32 {
                                        batches[0].just_set(IVec3::new(x, y, z), value1);
                                        batches[1].just_set(IVec3::new(x, y, z), value2);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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
                                            batches[0].just_set(IVec3::new(x, y, z), 1);
                                            batches[1].just_set(IVec3::new(x, y, z), 2);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batches = [tree.create_batch(), tree.create_batch()];

                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    for x in 0..size as i32 {
                                        if x == y && x == z {
                                            batches[0].just_set(IVec3::new(x, y, z), 1);
                                            batches[1].just_set(IVec3::new(x, y, z), 2);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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
                                            batches[0].just_set(IVec3::new(x, y, z), 1);
                                            batches[1].just_set(IVec3::new(x, y, z), 2);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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
                                    batches[0].just_set(IVec3::new(x, y, z), 1);
                                    batches[1].just_set(IVec3::new(x, y, z), 2);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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
                                        batches[0].just_set(IVec3::new(x, y, z), 1);
                                        batches[1].just_set(IVec3::new(x, y, z), 2);
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                batch.just_set(IVec3::new(x, y, z), value);

                                tree.apply_batch(&mut interner, black_box(&batch));

                                // batch.just_set(IVec3::new(x, y, z), 0);

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
                            });
                        }
                        BenchType::Batch => {
                            let mut batch = tree.create_batch();

                            b.iter(|| {
                                let x = rng.random_range(0..size as i32);
                                let y = rng.random_range(0..size as i32);
                                let z = rng.random_range(0..size as i32);
                                let value = rng.random_range(1..i32::MAX);

                                batch.just_set(IVec3::new(x, y, z), value);

                                tree.apply_batch(&mut interner, black_box(&batch));

                                batch.just_set(IVec3::new(x, y, z), 0);

                                #[cfg(feature = "tracy")]
                                tracy_client::frame_mark();
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

                    #[cfg(feature = "tracy")]
                    tracy_client::frame_mark();
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

                    #[cfg(feature = "tracy")]
                    tracy_client::frame_mark();
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

                    #[cfg(feature = "tracy")]
                    tracy_client::frame_mark();
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

                    #[cfg(feature = "tracy")]
                    tracy_client::frame_mark();
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

                    #[cfg(feature = "tracy")]
                    tracy_client::frame_mark();
                })
            });
        }

        group.finish();
    }

    {
        c.bench_function("voxtree_is_empty_empty", |b| {
            let depth = depths[0].1;
            let tree = VoxTree::<i32>::new(depth);

            b.iter(|| {
                let _ = black_box(tree.is_empty());

                #[cfg(feature = "tracy")]
                tracy_client::frame_mark();
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

                #[cfg(feature = "tracy")]
                tracy_client::frame_mark();
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

                    #[cfg(feature = "tracy")]
                    tracy_client::frame_mark();
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
                generate_test_sphere_for_batch(&mut batch, size, 1);

                b.iter(|| {
                    tree.apply_batch(&mut interner, black_box(&batch));
                    tree.clear(&mut interner);

                    #[cfg(feature = "tracy")]
                    tracy_client::frame_mark();
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

                    #[cfg(feature = "tracy")]
                    tracy_client::frame_mark();
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

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_to_vec(&mut group, size, depth, max_lod, &interner, &tree);
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_to_vec_sphere");

        for &(size, depth) in depths.iter() {
            let mut tree = VoxTree::new(depth);
            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 14);

            let mut batch = tree.create_batch();
            generate_test_sphere_for_batch(&mut batch, size, 1);
            tree.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_to_vec(&mut group, size, depth, max_lod, &interner, &tree);
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_to_vec_uniform");

        for &(size, depth) in depths.iter() {
            let mut tree = VoxTree::new(depth);
            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            tree.fill(&mut interner, 1);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_to_vec(&mut group, size, depth, max_lod, &interner, &tree);
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_to_vec_sum");

        for &(size, depth) in depths.iter() {
            let mut tree = VoxTree::new(depth);
            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024 * 24);

            fill_sum!(size, tree, interner);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_to_vec(&mut group, size, depth, max_lod, &interner, &tree);
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
                    batch.just_set(IVec3::new(x, y, z), 1);
                }
            }

            tree.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_to_vec(&mut group, size, depth, max_lod, &interner, &tree);
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_sphere");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            let half_size = size as i32 / 2;
            let center = IVec3::new(half_size, half_size, half_size);
            let radius = (size / 2) as i32;
            let value = 1;

            generate_sphere_batch(&mut batch, center, radius, value);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_terrain_surface");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            let offset = Vec3::ZERO;
            let scale = 26.0;
            let voxel_size = 1.28 / size as f32;

            generate_terrain_batch(&mut batch, voxel_size, scale, offset, true);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_terrain_full");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            let offset = Vec3::ZERO;
            let scale = 26.0;
            let voxel_size = 1.28 / size as f32;

            generate_terrain_batch(&mut batch, voxel_size, scale, offset, false);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_terrain_full_3_mats");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            let offset = Vec3::ZERO;
            let scale = 26.0;
            let voxel_size = 1.28 / size as f32;

            generate_terrain_batch_3_mats(&mut batch, voxel_size, scale, offset, false);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_corners_all");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            generate_corners_batch(&mut batch, [true; 8], false);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_corners_all_unique");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            generate_corners_batch(&mut batch, [true; 8], true);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_corners_min");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            generate_corners_batch(
                &mut batch,
                [true, false, false, false, false, false, false, false],
                false,
            );

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_corners_max");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            generate_corners_batch(
                &mut batch,
                [false, false, false, false, false, false, false, true],
                false,
            );

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_checkerboard");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            generate_checkerboard_batch(&mut batch);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_sparse_fill");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            generate_sparse_fill_batch(&mut batch);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_hollow_cube");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            generate_hollow_cube_batch(&mut batch);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_diagonal");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            generate_diagonal_batch(&mut batch);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_perlin3d");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            let mut batch = chunk.create_batch();

            let offset = Vec3::ZERO;
            let scale = 183.0;
            let voxel_size = 1.28 / size as f32;
            let threshold = 0.51;

            generate_perlin_3d_batch(&mut batch, voxel_size, scale, offset, threshold);

            chunk.apply_batch(&mut interner, &batch);

            let max_lod = max_lod.clamp(1, depth.max());

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("voxtree_mesh_uniform");

        for &(size, depth) in depths.iter() {
            let mut chunk = VoxChunk::with_position(1.28, depth, 0, 0, 0);

            let mut interner = VoxInterner::<i32>::with_memory_budget(1024 * 1024);

            chunk.fill(&mut interner, 1);

            benchmark_meshing(
                &mut group,
                size,
                depth,
                max_lod,
                &mesh_types,
                &interner,
                &chunk,
            );
        }

        group.finish();
    }
}

criterion_group!(benches, benchmark_voxtree);
criterion_main!(benches);
