use std::hint::black_box;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use glam::{IVec3, Vec3};
use rand::Rng;

use voxelis::{
    Batch,
    spatial::{
        Octree, OctreeOpsBatch, OctreeOpsMesh, OctreeOpsRead, OctreeOpsState, OctreeOpsWrite,
    },
    storage::NodeStore,
    world::Chunk,
};

macro_rules! fill_sum {
    ($size:expr, $octree:expr, $storage:expr) => {
        for x in 0..$size as i32 {
            for y in 0..$size as i32 {
                for z in 0..$size as i32 {
                    $octree.set(
                        &mut $storage,
                        black_box(IVec3::new(x, y, z)),
                        black_box((x + y + z) % i32::MAX),
                    );
                }
            }
        }
    };
}

pub fn generate_test_sphere(
    octree: &mut Octree,
    store: &mut NodeStore<i32>,
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
                    octree.set(store, black_box(IVec3::new(x, y, z)), black_box(value));
                }
            }
        }
    }
}

pub fn chunk_generate_test_sphere(
    chunk: &mut Chunk,
    store: &mut NodeStore<i32>,
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
                    chunk.set(store, IVec3::new(x, y, z), value);
                }
            }
        }
    }
}

pub fn generate_test_sphere_sum(octree: &mut Octree, store: &mut NodeStore<i32>, size: u32) {
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
                    octree.set(
                        store,
                        black_box(IVec3::new(x, y, z)),
                        black_box((x + y + z) % i32::MAX),
                    );
                }
            }
        }
    }
}

#[derive(Copy, Clone)]
enum OctreeType {
    Static,
    Dynamic,
}

impl OctreeType {
    pub fn to_string(self) -> &'static str {
        match self {
            Self::Static => "static",
            Self::Dynamic => "dynamic",
        }
    }
}

fn benchmark_octree(c: &mut Criterion) {
    let depths: Vec<(u32, u8)> = (3..=6)
        .map(|depth| {
            let voxels_per_axis = 1 << depth;
            (voxels_per_axis as u32, depth as u8)
        })
        .collect();

    let octree_types = [OctreeType::Static, OctreeType::Dynamic];

    {
        let mut group = c.benchmark_group("octree_create");

        for octree_type in octree_types.iter() {
            let depth = depths[0].1;
            group.bench_with_input(octree_type.to_string(), &depth, |b, &depth| {
                b.iter(|| {
                    let _ = match octree_type {
                        OctreeType::Static => black_box(Octree::make_static(black_box(depth))),
                        OctreeType::Dynamic => black_box(Octree::make_dynamic(black_box(depth))),
                    };
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_fill");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024);

                    let mut v = 1;

                    b.iter(|| {
                        octree.fill(&mut store, black_box(v));

                        v += 1;
                        if v == 0 {
                            v = 1;
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_fill_then_set_single_voxel");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                    let mut v = 1;
                    b.iter(|| {
                        let next_v = if v + 1 != 0 { v + 1 } else { 1 };

                        octree.fill(&mut store, black_box(v));
                        octree.set(
                            &mut store,
                            black_box(IVec3::new(0, 0, 0)),
                            black_box(next_v),
                        );

                        v = next_v;
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_single_voxel");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                    let mut v = 1;
                    b.iter(|| {
                        let next_v = if v + 1 != 0 { v + 1 } else { 1 };

                        octree.set(
                            &mut store,
                            black_box(IVec3::new(0, 0, 0)),
                            black_box(next_v),
                        );

                        v = next_v;
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_single_voxel");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    batches[0].set(&mut store, IVec3::new(0, 0, 0), 1);
                    batches[1].set(&mut store, IVec3::new(0, 0, 0), 2);

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_uniform");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 24);

                    let mut v = 1;
                    b.iter(|| {
                        for y in 0..size as i32 {
                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    octree.set(
                                        &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_uniform");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 24);

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    for y in 0..size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                batches[0].set(&mut store, IVec3::new(x, y, z), 1);
                                batches[1].set(&mut store, IVec3::new(x, y, z), 2);
                            }
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_uniform_half");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 24);

                    let half_size = size / 2;

                    let mut v = 1;
                    b.iter(|| {
                        for y in 0..half_size as i32 {
                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    octree.set(
                                        &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_uniform_half");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 24);

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    let half_size = size / 2;

                    for y in 0..half_size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                batches[0].set(&mut store, IVec3::new(x, y, z), 1);
                                batches[1].set(&mut store, IVec3::new(x, y, z), 2);
                            }
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_sum");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let mut v = 1;
                    b.iter(|| {
                        for y in 0..size as i32 {
                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    octree.set(
                                        &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_sum");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    for y in 0..size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                let position = IVec3::new(x, y, z);
                                batches[0].set(&mut store, position, (x + y + z + 1) % i32::MAX);
                                batches[1].set(&mut store, position, (x + y + z + 1000) % i32::MAX);
                            }
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });

                    // #[cfg(feature = "memory_stats")]
                    // println!("stats: {:#?}", store.stats());
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_checkerboard");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let mut v = 1;
                    b.iter(|| {
                        for y in 0..size as i32 {
                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    if (x + y + z) % 2 == 0 {
                                        octree.set(
                                            &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_checkerboard");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    for y in 0..size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                if (x + y + z) % 2 == 0 {
                                    batches[0].set(&mut store, IVec3::new(x, y, z), 1);
                                    batches[1].set(&mut store, IVec3::new(x, y, z), 2);
                                }
                            }
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });

                    // #[cfg(feature = "memory_stats")]
                    // println!("stats: {:#?}", store.stats());
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_sparse_fill");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let mut v = 1;
                    b.iter(|| {
                        for y in (0..size as i32).step_by(4) {
                            for z in (0..size as i32).step_by(4) {
                                for x in (0..size as i32).step_by(4) {
                                    octree.set(
                                        &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_sparse_fill");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    for y in (0..size as i32).step_by(4) {
                        for z in (0..size as i32).step_by(4) {
                            for x in (0..size as i32).step_by(4) {
                                batches[0].set(&mut store, IVec3::new(x, y, z), 1);
                                batches[1].set(&mut store, IVec3::new(x, y, z), 2);
                            }
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });

                    // #[cfg(feature = "memory_stats")]
                    // println!("stats: {:#?}", store.stats());
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_gradient_fill");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 256);

                    let mut v = 1;
                    b.iter(|| {
                        for x in 0..size as i32 {
                            let value = (x + v) % 256;
                            for y in 0..size as i32 {
                                for z in 0..size as i32 {
                                    octree.set(
                                        &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_gradient_fill");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    for x in 0..size as i32 {
                        let value1 = (x + 1) % 256;
                        let value2 = (x + 2) % 256;
                        for y in 0..size as i32 {
                            for z in 0..size as i32 {
                                batches[0].set(&mut store, IVec3::new(x, y, z), value1);
                                batches[1].set(&mut store, IVec3::new(x, y, z), value2);
                            }
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });

                    // #[cfg(feature = "memory_stats")]
                    // println!("stats: {:#?}", store.stats());
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_hollow_cube");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

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
                                        octree.set(
                                            &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_hollow_cube");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

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
                                    batches[0].set(&mut store, IVec3::new(x, y, z), 1);
                                    batches[1].set(&mut store, IVec3::new(x, y, z), 2);
                                }
                            }
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });

                    // #[cfg(feature = "memory_stats")]
                    // println!("stats: {:#?}", store.stats());
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_diagonal");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let mut v = 1;
                    b.iter(|| {
                        for y in 0..size as i32 {
                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    if x == y && x == z {
                                        octree.set(
                                            &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_diagonal");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    for y in 0..size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                if x == y && x == z {
                                    batches[0].set(&mut store, IVec3::new(x, y, z), 1);
                                    batches[1].set(&mut store, IVec3::new(x, y, z), 2);
                                }
                            }
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });

                    // #[cfg(feature = "memory_stats")]
                    // println!("stats: {:#?}", store.stats());
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_sphere");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let radius = (size / 2) as i32;
                    let r1 = radius - 1;

                    let (cx, cy, cz) = (r1, r1, r1);
                    let radius_squared = radius * radius;

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
                                        octree.set(
                                            &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_sphere");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 25);

                    let radius = (size / 2) as i32;
                    let r1 = radius - 1;

                    let (cx, cy, cz) = (r1, r1, r1);
                    let radius_squared = radius * radius;

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    for y in 0..size as i32 {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                let dx = (x - cx).abs();
                                let dy = (y - cy).abs();
                                let dz = (z - cz).abs();

                                let distance_squared = dx * dx + dy * dy + dz * dz;

                                if distance_squared <= radius_squared {
                                    batches[0].set(&mut store, IVec3::new(x, y, z), 1);
                                    batches[1].set(&mut store, IVec3::new(x, y, z), 2);
                                }
                            }
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });

                    // #[cfg(feature = "memory_stats")]
                    // println!("stats: {:#?}", store.stats());
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_terrain");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                    let mut v = 1;
                    let mut noise = fastnoise_lite::FastNoiseLite::new();
                    noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

                    b.iter(|| {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                let y = ((noise
                                    .get_noise_2d(x as f32 / size as f32, z as f32 / size as f32)
                                    + 1.0)
                                    / 2.0)
                                    * size as f32;
                                let y = y as i32;
                                debug_assert!(y < size as i32);
                                octree.set(
                                    &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_terrain");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                    let mut noise = fastnoise_lite::FastNoiseLite::new();
                    noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    for z in 0..size as i32 {
                        for x in 0..size as i32 {
                            let y = ((noise
                                .get_noise_2d(x as f32 / size as f32, z as f32 / size as f32)
                                + 1.0)
                                / 2.0)
                                * size as f32;
                            let y = y as i32;
                            debug_assert!(y < size as i32);
                            batches[0].set(&mut store, IVec3::new(x, y, z), 1);
                            batches[1].set(&mut store, IVec3::new(x, y, z), 2);
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_terrain_full");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 14);

                    let mut v = 1;
                    let mut noise = fastnoise_lite::FastNoiseLite::new();
                    noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

                    b.iter(|| {
                        for z in 0..size as i32 {
                            for x in 0..size as i32 {
                                let height = ((noise
                                    .get_noise_2d(x as f32 / size as f32, z as f32 / size as f32)
                                    + 1.0)
                                    / 2.0)
                                    * size as f32;
                                let height = height as i32;
                                debug_assert!(height < size as i32);
                                for y in 0..=height {
                                    octree.set(
                                        &mut store,
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
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_batch_set_terrain_full");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 14);

                    let mut noise = fastnoise_lite::FastNoiseLite::new();
                    noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

                    let mut batches: Vec<Batch<i32>> =
                        vec![octree.create_batch(), octree.create_batch()];

                    for z in 0..size as i32 {
                        for x in 0..size as i32 {
                            let height = ((noise
                                .get_noise_2d(x as f32 / size as f32, z as f32 / size as f32)
                                + 1.0)
                                / 2.0)
                                * size as f32;
                            let height = height as i32;
                            debug_assert!(height < size as i32);
                            for y in 0..=height {
                                batches[0].set(&mut store, IVec3::new(x, y, z), 1);
                                batches[1].set(&mut store, IVec3::new(x, y, z), 2);
                            }
                        }
                    }

                    let mut batch_idx = 0;

                    b.iter(|| {
                        octree.apply_batch(&mut store, black_box(&batches[batch_idx]));

                        if batch_idx == 0 {
                            batch_idx = 1;
                        } else {
                            batch_idx = 0;
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_rand_uniform");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 24);

                    let mut rng = rand::rng();
                    let mut v = 1;

                    b.iter(|| {
                        let value = 1;
                        let x = rng.random_range(0..size as i32);
                        let y = rng.random_range(0..size as i32);
                        let z = rng.random_range(0..size as i32);
                        octree.set(&mut store, black_box(IVec3::new(x, y, z)), black_box(value));
                        v += 1;
                        if v == 0 {
                            v = 1;
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_set_rand_rand");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 185);

                    let mut rng = rand::rng();
                    let mut v = 1;

                    b.iter(|| {
                        let x = rng.random_range(0..size as i32);
                        let y = rng.random_range(0..size as i32);
                        let z = rng.random_range(0..size as i32);
                        let value = rng.random_range(1..i32::MAX);
                        octree.set(&mut store, black_box(IVec3::new(x, y, z)), black_box(value));
                        v += 1;
                        if v == 0 {
                            v = 1;
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_get_empty");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let store = NodeStore::<i32>::with_memory_budget(1024);

                    b.iter(|| {
                        for y in 0..size as i32 {
                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    let _ = black_box(
                                        octree.get(&store, black_box(IVec3::new(x, y, z))),
                                    );
                                }
                            }
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_get_sphere_uniform");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 14);

                    generate_test_sphere(&mut octree, &mut store, size, 1);

                    b.iter(|| {
                        for y in 0..size as i32 {
                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    let _ = black_box(
                                        octree.get(&store, black_box(IVec3::new(x, y, z))),
                                    );
                                }
                            }
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_get_sphere_sum");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 14);

                    generate_test_sphere_sum(&mut octree, &mut store, size);

                    b.iter(|| {
                        for y in 0..size as i32 {
                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    let _ = black_box(
                                        octree.get(&store, black_box(IVec3::new(x, y, z))),
                                    );
                                }
                            }
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_get_full_uniform");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                    octree.fill(&mut store, 1);

                    b.iter(|| {
                        for y in 0..size as i32 {
                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    let _ = black_box(
                                        octree.get(&store, black_box(IVec3::new(x, y, z))),
                                    );
                                }
                            }
                        }
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_get_full_sum");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 24);

                    fill_sum!(size, octree, store);

                    b.iter(|| {
                        for y in 0..size as i32 {
                            for z in 0..size as i32 {
                                for x in 0..size as i32 {
                                    let _ = black_box(
                                        octree.get(&store, black_box(IVec3::new(x, y, z))),
                                    );
                                }
                            }
                        }
                    })
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_is_empty_empty");

        for octree_type in octree_types.iter() {
            let depth = depths[0].1;
            group.bench_with_input(octree_type.to_string(), &depth, |b, &depth| {
                let octree = match octree_type {
                    OctreeType::Static => Octree::make_static(depth),
                    OctreeType::Dynamic => Octree::make_dynamic(depth),
                };

                b.iter(|| {
                    let _ = black_box(octree.is_empty());
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_is_empty_not_empty");

        for octree_type in octree_types.iter() {
            let depth = depths[0].1;
            group.bench_with_input(octree_type.to_string(), &depth, |b, &depth| {
                let mut octree = match octree_type {
                    OctreeType::Static => Octree::make_static(depth),
                    OctreeType::Dynamic => Octree::make_dynamic(depth),
                };
                let mut store = NodeStore::<i32>::with_memory_budget(1024);
                octree.fill(&mut store, 1);

                b.iter(|| {
                    let _ = black_box(octree.is_empty());
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_clear_empty");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024);

                    b.iter(|| {
                        octree.clear(black_box(&mut store));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_clear_sphere");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 22);

                    b.iter(|| {
                        generate_test_sphere(&mut octree, &mut store, size, 1);
                        octree.clear(&mut store);
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_clear_full");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                    b.iter(|| {
                        octree.fill(&mut store, 1);
                        octree.clear(&mut store);
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_to_vec_empty");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let store = NodeStore::<i32>::with_memory_budget(1024);

                    b.iter(|| {
                        let _ = black_box(octree.to_vec(&store));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_to_vec_sphere");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 14);

                    generate_test_sphere(&mut octree, &mut store, size, 1);

                    b.iter(|| {
                        let _ = black_box(octree.to_vec(&store));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_to_vec_uniform");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                    octree.fill(&mut store, 1);

                    b.iter(|| {
                        let _ = black_box(octree.to_vec(&store));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_to_vec_sum");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024 * 24);

                    fill_sum!(size, octree, store);

                    b.iter(|| {
                        let _ = black_box(octree.to_vec(&store));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("octree_to_vec_terrain");

        for &(size, depth) in depths.iter() {
            for octree_type in octree_types.iter() {
                let bench_id = BenchmarkId::new(size.to_string(), octree_type.to_string());
                group.bench_with_input(bench_id, &depth, |b, &depth| {
                    let mut octree = match octree_type {
                        OctreeType::Static => Octree::make_static(depth),
                        OctreeType::Dynamic => Octree::make_dynamic(depth),
                    };
                    let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                    let mut noise = fastnoise_lite::FastNoiseLite::new();
                    noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

                    for x in 0..size as i32 {
                        for z in 0..size as i32 {
                            let y = ((noise
                                .get_noise_2d(x as f32 / size as f32, z as f32 / size as f32)
                                + 1.0)
                                / 2.0)
                                * size as f32;
                            let y = y as i32;
                            debug_assert!(y < size as i32);
                            octree.set(&mut store, IVec3::new(x, y, z), 1);
                        }
                    }

                    b.iter(|| {
                        let _ = black_box(octree.to_vec(&store));
                    });
                });
            }
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("svodag_naive_mesh_sphere");

        for &(size, depth) in depths.iter() {
            group.bench_with_input(BenchmarkId::from_parameter(size), &depth, |b, &depth| {
                let mut chunk = Chunk::with_position(1.28, depth as usize, 0, 0, 0);

                let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                chunk_generate_test_sphere(&mut chunk, &mut store, size, 1);

                let offset = Vec3::ZERO;
                let mut vertices = Vec::new();
                let mut normals = Vec::new();
                let mut indices = Vec::new();

                b.iter(|| {
                    vertices.clear();
                    normals.clear();
                    indices.clear();

                    chunk.generate_mesh_arrays(
                        &store,
                        black_box(&mut vertices),
                        black_box(&mut normals),
                        black_box(&mut indices),
                        black_box(offset),
                    );
                });
            });
        }

        group.finish();
    }

    {
        let mut group = c.benchmark_group("svodag_naive_mesh_terrain");

        for &(size, depth) in depths.iter() {
            group.bench_with_input(BenchmarkId::from_parameter(size), &depth, |b, &depth| {
                let mut chunk = Chunk::with_position(1.28, depth as usize, 0, 0, 0);

                let mut store = NodeStore::<i32>::with_memory_budget(1024 * 1024);

                let mut noise = fastnoise_lite::FastNoiseLite::new();
                noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

                for x in 0..size as i32 {
                    for z in 0..size as i32 {
                        let y = ((noise
                            .get_noise_2d(x as f32 / size as f32, z as f32 / size as f32)
                            + 1.0)
                            / 2.0)
                            * size as f32;
                        let y = y as i32;
                        debug_assert!(y < size as i32);
                        chunk.set(&mut store, IVec3::new(x, y, z), 1);
                    }
                }

                let offset = Vec3::ZERO;
                let mut vertices = Vec::new();
                let mut normals = Vec::new();
                let mut indices = Vec::new();

                b.iter(|| {
                    vertices.clear();
                    normals.clear();
                    indices.clear();

                    chunk.generate_mesh_arrays(
                        &store,
                        black_box(&mut vertices),
                        black_box(&mut normals),
                        black_box(&mut indices),
                        black_box(offset),
                    );
                });
            });
        }

        group.finish();
    }

    // c.bench_function("svodag_total_memory_size", |b| {
    //     let mut octree = SvoDag::new(max_depth, None);
    //     fill_uniform(voxels_per_axis, &mut octree);
    //     b.iter(|| {
    //         black_box(octree.total_memory_size());
    //     })
    // });

    // c.bench_function("svodag_serialize_to_vec", |b| {
    //     let mut octree = SvoDag::new(max_depth);
    //     fill_octree(voxels_per_axis, &mut octree);
    //     b.iter(|| {
    //         black_box(octree.serialize_to_vec().unwrap());
    //     })
    // });

    // c.bench_function("svodag_serialize_empty", |b| {
    //     let octree = SvoDag::new(max_depth);
    //     b.iter(|| {
    //         let mut buffer = Vec::new();
    //         octree.serialize(&mut buffer).unwrap();
    //         black_box(buffer);
    //     })
    // });

    // c.bench_function("svodag_serialize_sphere", |b| {
    //     let mut octree = SvoDag::new(max_depth);
    //     insert_sphere(voxels_per_axis, &mut octree);
    //     b.iter(|| {
    //         let mut buffer = Vec::new();
    //         octree.serialize(&mut buffer).unwrap();
    //         black_box(buffer);
    //     })
    // });

    // c.bench_function("svodag_serialize_full", |b| {
    //     let mut octree = SvoDag::new(max_depth);
    //     fill_octree(voxels_per_axis, &mut octree);
    //     b.iter(|| {
    //         let mut buffer = Vec::new();
    //         octree.serialize(&mut buffer).unwrap();
    //         black_box(buffer);
    //     })
    // });

    // c.bench_function("svodag_deserialize_full", |b| {
    //     let mut octree = SvoDag::new(max_depth);
    //     fill_octree(voxels_per_axis, &mut octree);
    //     let buffer = octree.serialize_to_vec().unwrap();
    //     b.iter(|| {
    //         let mut new_octree = SvoDag::new(max_depth);
    //         new_octree.deserialize(&mut buffer.as_slice()).unwrap();
    //         black_box(new_octree);
    //     })
    // });
}

criterion_group!(benches, benchmark_octree);
criterion_main!(benches);
