//! Small primitive consumer: construct, transform, and query one exact solid.
//!
//! This binary links only CSGRS primitive construction, affine transport,
//! exact bounds, and native triangle traversal. It is the feature-minimal end
//! of the size attribution; `composed` adds profiles, extrusion, Booleans, and
//! export. The workload is argument-driven so dead-code elimination cannot
//! reduce the measured artifact to metadata.

use std::hint::black_box;

use csgrs::{Real, TriangleMesh, solid, solid::SolidExt};

/// One completed primitive workflow. Returns the traversed triangle count and
/// whether exact bounds were available.
fn workload(width: i64) -> Result<(usize, bool), Box<dyn std::error::Error>> {
    let body: TriangleMesh = solid::cube(Real::from(black_box(width)))?;
    let placed = body.translated(Real::from(1), Real::from(2), Real::from(3))?;
    // Disambiguated from the inherent Hypermesh method of the same name.
    let bounds = SolidExt::exact_bounds(&placed)?;
    let mut triangles = 0usize;
    placed.visit_native_triangles(|corners| {
        black_box(&corners);
        triangles += 1;
    });
    Ok((triangles, bounds.is_some()))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let width = std::env::args()
        .nth(1)
        .and_then(|value| value.parse::<i64>().ok())
        .unwrap_or(12);
    let repeats = std::env::args()
        .nth(2)
        .and_then(|value| value.parse::<u32>().ok())
        .unwrap_or(1)
        .max(1);

    // One warm iteration precedes the timed loop; caches are process-local.
    let mut last = workload(width)?;
    #[cfg(not(target_arch = "wasm32"))]
    let started = std::time::Instant::now();
    for _ in 0..repeats {
        last = workload(width)?;
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let elapsed = started.elapsed().as_nanos();
        println!(
            "repeats={repeats} elapsed_ns={elapsed} ns_per_iteration={}",
            elapsed / u128::from(repeats)
        );
    }
    println!("triangles={} bounded={}", last.0, last.1);
    Ok(())
}
