use criterion::{black_box, criterion_group, criterion_main, Criterion};
use glam::IVec3;
use voxelis::spatial::{Octree, Voxel};

fn fill_octree(voxels_per_axis: i32, octree: &mut Octree<u8>) {
    for x in 0..voxels_per_axis {
        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                octree.set(
                    black_box(IVec3::new(x, y, z)),
                    black_box(Voxel { value: 1 }),
                );
            }
        }
    }
}

pub fn generate_test_sphere(
    octree: &mut Octree<u8>,
    voxels_per_axis: i32,
    center: IVec3,
    radius: i32,
    value: u8,
) {
    let (cx, cy, cz) = (center.x, center.y, center.z);
    let radius_squared = radius * radius;

    for y in 0..voxels_per_axis {
        for z in 0..voxels_per_axis {
            for x in 0..voxels_per_axis {
                let dx = x as i32 - cx;
                let dy = y as i32 - cy;
                let dz = z as i32 - cz;

                let distance_squared = dx * dx + dy * dy + dz * dz;

                if distance_squared <= radius_squared {
                    octree.set(IVec3::new(x, y, z), Voxel { value });
                }
            }
        }
    }
}

fn insert_sphere(voxels_per_axis: i32, octree: &mut Octree<u8>) {
    let r1 = (voxels_per_axis / 2) - 1;
    let r = voxels_per_axis / 2;
    generate_test_sphere(octree, voxels_per_axis, IVec3::new(r1, r1, r1), r, 1);
}

fn benchmark_octree(c: &mut Criterion) {
    let max_depth = 6;
    let voxels_per_axis = 1 << max_depth;

    c.bench_function("octree_set_value", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        b.iter(|| {
            for x in 0..voxels_per_axis {
                for y in 0..voxels_per_axis {
                    for z in 0..voxels_per_axis {
                        octree.set(
                            black_box(IVec3::new(x, y, z)),
                            black_box(Voxel { value: 1 }),
                        );
                    }
                }
            }
        })
    });

    c.bench_function("octree_set_sum", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        b.iter(|| {
            for x in 0..voxels_per_axis {
                for y in 0..voxels_per_axis {
                    for z in 0..voxels_per_axis {
                        octree.set(
                            black_box(IVec3::new(x, y, z)),
                            black_box(Voxel {
                                value: ((x + y + z) % 255) as u8,
                            }),
                        );
                    }
                }
            }
        })
    });

    c.bench_function("octree_get_empty", |b| {
        let octree = Octree::<u8>::new(max_depth);
        b.iter(|| {
            for x in 0..voxels_per_axis {
                for y in 0..voxels_per_axis {
                    for z in 0..voxels_per_axis {
                        black_box(octree.get(IVec3::new(x, y, z)));
                    }
                }
            }
        })
    });

    c.bench_function("octree_get_sphere", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        insert_sphere(voxels_per_axis, &mut octree);
        b.iter(|| {
            for x in 0..voxels_per_axis {
                for y in 0..voxels_per_axis {
                    for z in 0..voxels_per_axis {
                        black_box(octree.get(IVec3::new(x, y, z)));
                    }
                }
            }
        })
    });

    c.bench_function("octree_get_full", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        fill_octree(voxels_per_axis, &mut octree);
        b.iter(|| {
            for x in 0..voxels_per_axis {
                for y in 0..voxels_per_axis {
                    for z in 0..voxels_per_axis {
                        black_box(octree.get(IVec3::new(x, y, z)));
                    }
                }
            }
        })
    });

    c.bench_function("octree_is_empty_empty", |b| {
        let octree = Octree::<u8>::new(max_depth);
        b.iter(|| {
            black_box(octree.is_empty());
        })
    });

    c.bench_function("octree_is_empty_sphere", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        insert_sphere(voxels_per_axis, &mut octree);
        b.iter(|| {
            black_box(octree.is_empty());
        })
    });

    c.bench_function("octree_is_empty_full", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        fill_octree(voxels_per_axis, &mut octree);
        b.iter(|| {
            black_box(octree.is_empty());
        })
    });

    c.bench_function("octree_is_full_empty", |b| {
        let octree = Octree::<u8>::new(max_depth);
        b.iter(|| {
            black_box(octree.is_full());
        })
    });

    c.bench_function("octree_is_full_sphere", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        insert_sphere(voxels_per_axis, &mut octree);
        b.iter(|| {
            black_box(octree.is_full());
        })
    });

    c.bench_function("octree_is_full_full", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        fill_octree(voxels_per_axis, &mut octree);
        b.iter(|| {
            black_box(octree.is_full());
        })
    });

    c.bench_function("octree_clear_empty", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        b.iter(|| {
            octree.clear();
        })
    });

    c.bench_function("octree_clear_sphere", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        insert_sphere(voxels_per_axis, &mut octree);
        b.iter(|| {
            insert_sphere(voxels_per_axis, &mut octree);
            octree.clear();
        })
    });

    c.bench_function("octree_clear_full", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        b.iter(|| {
            fill_octree(voxels_per_axis, &mut octree);
            octree.clear();
        })
    });

    c.bench_function("octree_iter_empty", |b| {
        let octree = Octree::<u8>::new(max_depth);
        b.iter(|| {
            for voxel in octree.iter() {
                black_box(voxel);
            }
        })
    });

    c.bench_function("octree_iter_sphere", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        insert_sphere(voxels_per_axis, &mut octree);
        b.iter(|| {
            for voxel in octree.iter() {
                black_box(voxel);
            }
        })
    });

    c.bench_function("octree_iter_full", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        fill_octree(voxels_per_axis, &mut octree);
        b.iter(|| {
            for voxel in octree.iter() {
                black_box(voxel);
            }
        })
    });

    c.bench_function("octree_to_vec_empty", |b| {
        let octree = Octree::<u8>::new(max_depth);
        b.iter(|| {
            black_box(octree.to_vec());
        })
    });

    c.bench_function("octree_to_vec_sphere", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        insert_sphere(voxels_per_axis, &mut octree);
        b.iter(|| {
            black_box(octree.to_vec());
        })
    });

    c.bench_function("octree_to_vec_full", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        fill_octree(voxels_per_axis, &mut octree);
        b.iter(|| {
            black_box(octree.to_vec());
        })
    });

    c.bench_function("octree_for_each_mut_empty", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        b.iter(|| {
            octree.for_each_mut(|_, value| {
                *value += 1;
            });
        })
    });

    c.bench_function("octree_for_each_mut_sphere", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        insert_sphere(voxels_per_axis, &mut octree);
        b.iter(|| {
            octree.for_each_mut(|_, value| {
                *value += 1;
            });
        })
    });

    c.bench_function("octree_for_each_mut_full", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        fill_octree(voxels_per_axis, &mut octree);
        b.iter(|| {
            octree.for_each_mut(|_, value| {
                *value += 1;
            });
        })
    });

    c.bench_function("octree_total_memory_size", |b| {
        let mut octree = Octree::<u8>::new(max_depth);
        fill_octree(voxels_per_axis, &mut octree);
        b.iter(|| {
            black_box(octree.total_memory_size());
        })
    });
}

criterion_group!(benches, benchmark_octree);
criterion_main!(benches);
