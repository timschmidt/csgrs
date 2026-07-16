use std::hint::black_box;
use std::time::Instant;

use glam::DVec3;
use voxelis_math::triangle_cube_intersection;

fn main() {
    let triangle = (
        DVec3::new(0.0, 0.0, 0.0),
        DVec3::new(20.0, 0.0, 0.0),
        DVec3::new(0.0, 20.0, 0.0),
    );
    let mut cubes = Vec::with_capacity(2_048);
    for y in -6..26 {
        for x in -6..26 {
            let min = DVec3::new(x as f64, y as f64, -0.5);
            cubes.push((min, min + DVec3::ONE));

            let far_min = DVec3::new(x as f64, y as f64, 4.0);
            cubes.push((far_min, far_min + DVec3::ONE));
        }
    }

    let iterations = if cfg!(debug_assertions) { 1 } else { 1_000 };
    let started = Instant::now();
    let mut hits = 0_usize;
    for _ in 0..iterations {
        for cube in &cubes {
            hits += usize::from(triangle_cube_intersection(
                black_box(triangle),
                black_box(*cube),
            ));
        }
    }
    println!(
        "triangle_cube_mixed queries={} hits={} ms={:.3}",
        iterations * cubes.len(),
        hits,
        started.elapsed().as_secs_f64() * 1_000.0
    );
}
