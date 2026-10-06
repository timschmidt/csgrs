use std::hint::black_box;
use std::time::Instant;

use glam::{DVec3, IVec3};
use voxelis::{MaxDepth, io::Obj};
use voxelis_voxelize::Voxelizer;

fn cube_mesh() -> Obj {
    let vertices = vec![
        DVec3::new(0.0, 0.0, 0.0),
        DVec3::new(0.75, 0.0, 0.0),
        DVec3::new(0.75, 0.75, 0.0),
        DVec3::new(0.0, 0.75, 0.0),
        DVec3::new(0.0, 0.0, 0.75),
        DVec3::new(0.75, 0.0, 0.75),
        DVec3::new(0.75, 0.75, 0.75),
        DVec3::new(0.0, 0.75, 0.75),
    ];
    let faces = vec![
        IVec3::new(1, 2, 3),
        IVec3::new(1, 3, 4),
        IVec3::new(5, 7, 6),
        IVec3::new(5, 8, 7),
        IVec3::new(1, 5, 6),
        IVec3::new(1, 6, 2),
        IVec3::new(2, 6, 7),
        IVec3::new(2, 7, 3),
        IVec3::new(3, 7, 8),
        IVec3::new(3, 8, 4),
        IVec3::new(4, 8, 5),
        IVec3::new(4, 5, 1),
    ];
    Obj {
        vertices,
        faces,
        aabb: (DVec3::ZERO, DVec3::splat(0.75)),
        size: DVec3::splat(0.75),
    }
}

fn main() {
    const ITERATIONS: u32 = 10;
    let mut warmup = Voxelizer::empty(MaxDepth::new(3), 1.0, cube_mesh(), 64 * 1024 * 1024);
    warmup.voxelize();
    black_box(warmup.model.chunks.len());

    let started = Instant::now();
    let mut chunk_checksum = 0_usize;

    for _ in 0..ITERATIONS {
        let mut voxelizer = Voxelizer::empty(MaxDepth::new(3), 1.0, cube_mesh(), 64 * 1024 * 1024);
        voxelizer.voxelize();
        chunk_checksum += black_box(voxelizer.model.chunks.len());
    }

    let elapsed = started.elapsed();
    println!(
        "immediate_cube_voxelize: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), chunks={chunk_checksum}",
        elapsed / ITERATIONS
    );
}
