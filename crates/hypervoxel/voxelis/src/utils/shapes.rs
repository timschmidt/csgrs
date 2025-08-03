use glam::{IVec3, Vec3};

use crate::{Batch, Lod, spatial::VoxOpsConfig};

pub fn generate_corners<T: VoxOpsConfig>(tree: &T, corners: [bool; 8]) -> Batch<i32> {
    let mut batch = Batch::<i32>::new(tree.max_depth(Lod::new(0)));

    generate_corners_batch(&mut batch, corners);

    batch
}

/// Generates a batch of corner voxels for a voxel tree.
/// Corners are specified as a boolean array, where each element corresponds to a corner voxel.
/// The order of corners is:
/// * 0: bottom-left-back
/// * 1: bottom-right-back
/// * 2: bottom-left-front
/// * 3: bottom-right-front
/// * 4: top-left-back
/// * 5: top-right-back
/// * 6: top-left-front
/// * 7: top-right-front
pub fn generate_corners_batch(batch: &mut Batch<i32>, corners: [bool; 8]) {
    let voxels_per_axis = batch.voxels_per_axis(Lod::new(0)) as i32;

    let max = voxels_per_axis - 1;

    // bottom-left-back corner
    if corners[0] {
        batch.just_set(IVec3::new(0, 0, 0), 1);
    }

    // bottom-right-back corner
    if corners[1] {
        batch.just_set(IVec3::new(max, 0, 0), 1);
    }

    // bottom-left-front corner
    if corners[2] {
        batch.just_set(IVec3::new(0, 0, max), 1);
    }

    // bottom-right-front corner
    if corners[3] {
        batch.just_set(IVec3::new(max, 0, max), 1);
    }

    // top-left-back corner
    if corners[4] {
        batch.just_set(IVec3::new(0, max, 0), 1);
    }

    // top-right-back corner
    if corners[5] {
        batch.just_set(IVec3::new(max, max, 0), 1);
    }

    // top-left-front corner
    if corners[6] {
        batch.just_set(IVec3::new(0, max, max), 1);
    }

    // top-right-front corner
    if corners[7] {
        batch.just_set(IVec3::new(max, max, max), 1);
    }
}

pub fn generate_sphere<T: VoxOpsConfig>(
    tree: &T,
    center: IVec3,
    radius: i32,
    value: i32,
) -> Batch<i32> {
    let mut batch = Batch::<i32>::new(tree.max_depth(Lod::new(0)));

    generate_sphere_batch(&mut batch, center, radius, value);

    batch
}

pub fn generate_sphere_batch(batch: &mut Batch<i32>, center: IVec3, radius: i32, value: i32) {
    debug_assert!(radius > 0);

    let (cx, cy, cz) = (center.x, center.y, center.z);
    let radius_squared = radius * radius;

    let voxels_per_axis = batch.voxels_per_axis(Lod::new(0)) as i32;

    let mut position = IVec3::ZERO;

    for y in 0..voxels_per_axis {
        position.y = y;
        for z in 0..voxels_per_axis {
            position.z = z;
            for x in 0..voxels_per_axis {
                let dx = x - cx;
                let dy = y - cy;
                let dz = z - cz;

                let distance_squared = dx * dx + dy * dy + dz * dz;

                if distance_squared <= radius_squared {
                    position.x = x;
                    batch.just_set(position, value);
                }
            }
        }
    }
}

pub fn generate_checkerboard<T: VoxOpsConfig>(tree: &T) -> Batch<i32> {
    let lod = Lod::new(0);

    let mut batch = Batch::<i32>::new(tree.max_depth(lod));

    generate_checkerboard_batch(&mut batch);

    batch
}

pub fn generate_checkerboard_batch(batch: &mut Batch<i32>) {
    let lod = Lod::new(0);
    let voxels_per_axis = batch.voxels_per_axis(lod) as i32;

    for y in 0..voxels_per_axis {
        for z in 0..voxels_per_axis {
            for x in 0..voxels_per_axis {
                if (x + y + z) % 2 == 0 {
                    batch.just_set(IVec3::new(x, y, z), 1);
                }
            }
        }
    }
}

pub fn generate_sparse_fill<T: VoxOpsConfig>(tree: &T) -> Batch<i32> {
    let lod = Lod::new(0);

    let mut batch = Batch::<i32>::new(tree.max_depth(lod));

    generate_sparse_fill_batch(&mut batch);

    batch
}

pub fn generate_sparse_fill_batch(batch: &mut Batch<i32>) {
    let lod = Lod::new(0);
    let voxels_per_axis = batch.voxels_per_axis(lod) as i32;

    for y in (0..voxels_per_axis).step_by(4) {
        for z in (0..voxels_per_axis).step_by(4) {
            for x in (0..voxels_per_axis).step_by(4) {
                batch.just_set(IVec3::new(x, y, z), 1);
            }
        }
    }
}

pub fn generate_hollow_cube<T: VoxOpsConfig>(tree: &T) -> Batch<i32> {
    let lod = Lod::new(0);

    let mut batch = Batch::<i32>::new(tree.max_depth(lod));

    generate_hollow_cube_batch(&mut batch);

    batch
}

pub fn generate_hollow_cube_batch(batch: &mut Batch<i32>) {
    let lod = Lod::new(0);
    let voxels_per_axis = batch.voxels_per_axis(lod) as i32;

    for y in 0..voxels_per_axis {
        for z in 0..voxels_per_axis {
            for x in 0..voxels_per_axis {
                let is_edge = x == 0
                    || x == voxels_per_axis - 1
                    || y == 0
                    || y == voxels_per_axis - 1
                    || z == 0
                    || z == voxels_per_axis - 1;
                if is_edge {
                    batch.just_set(IVec3::new(x, y, z), 1);
                }
            }
        }
    }
}

pub fn generate_diagonal<T: VoxOpsConfig>(tree: &T) -> Batch<i32> {
    let lod = Lod::new(0);

    let mut batch = Batch::<i32>::new(tree.max_depth(lod));

    generate_diagonal_batch(&mut batch);

    batch
}

pub fn generate_diagonal_batch(batch: &mut Batch<i32>) {
    let lod = Lod::new(0);
    let voxels_per_axis = batch.voxels_per_axis(lod) as i32;

    for y in 0..voxels_per_axis {
        for z in 0..voxels_per_axis {
            for x in 0..voxels_per_axis {
                if x == y && x == z {
                    batch.just_set(IVec3::new(x, y, z), 1);
                }
            }
        }
    }
}

pub fn generate_terrain<T: VoxOpsConfig>(
    tree: &T,
    voxel_size: f32,
    scale: f32,
    offset: Vec3,
    surface_only: bool,
) -> Batch<i32> {
    let mut batch = Batch::<i32>::new(tree.max_depth(Lod::new(0)));

    generate_terrain_batch(&mut batch, voxel_size, scale, offset, surface_only);

    batch
}

pub fn generate_terrain_batch(
    batch: &mut Batch<i32>,
    voxel_size: f32,
    scale: f32,
    offset: Vec3,
    surface_only: bool,
) {
    let lod = Lod::new(0);
    let voxels_per_axis = batch.voxels_per_axis(lod) as i32;

    let mut noise = fastnoise_lite::FastNoiseLite::new();
    noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

    for z in 0..voxels_per_axis {
        for x in 0..voxels_per_axis {
            let noise_value = noise.get_noise_2d(
                (offset.x + (x as f32 * voxel_size)) * scale,
                (offset.z + (z as f32 * voxel_size)) * scale,
            );
            let noise_value = (noise_value + 1.0) / 2.0;
            let y = noise_value * voxels_per_axis as f32;
            let y = y as u32;

            let local_y = (y % voxels_per_axis as u32) as i32;

            if surface_only {
                assert!(local_y < voxels_per_axis);
                batch.just_set(IVec3::new(x, local_y, z), 1);
            } else {
                for local_y in 0..=local_y {
                    assert!(local_y < voxels_per_axis);
                    batch.just_set(IVec3::new(x, local_y, z), 1);
                }
            }
        }
    }
}

pub fn generate_perlin_3d<T: VoxOpsConfig>(
    tree: &T,
    voxel_size: f32,
    scale: f32,
    offset: Vec3,
    threshold: f32,
) -> Batch<i32> {
    let lod = Lod::new(0);

    let mut batch = Batch::<i32>::new(tree.max_depth(lod));

    generate_perlin_3d_batch(&mut batch, voxel_size, scale, offset, threshold);

    batch
}

pub fn generate_perlin_3d_batch(
    batch: &mut Batch<i32>,
    voxel_size: f32,
    scale: f32,
    offset: Vec3,
    threshold: f32,
) {
    let lod = Lod::new(0);
    let voxels_per_axis = batch.voxels_per_axis(lod) as i32;

    let mut noise = fastnoise_lite::FastNoiseLite::new();
    noise.set_noise_type(Some(fastnoise_lite::NoiseType::OpenSimplex2));

    for y in 0..voxels_per_axis {
        for z in 0..voxels_per_axis {
            for x in 0..voxels_per_axis {
                let noise_value = noise.get_noise_3d(
                    (offset.x + (x as f32 * voxel_size)) * scale,
                    (offset.y + (y as f32 * voxel_size)) * scale,
                    (offset.z + (z as f32 * voxel_size)) * scale,
                );
                let noise_value = (noise_value + 1.0) / 2.0;
                if noise_value >= threshold {
                    batch.just_set(IVec3::new(x, y, z), 1);
                }
            }
        }
    }
}
