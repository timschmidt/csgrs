//! Composed CAD consumer: profile, extrusion, Boolean, bounds, and export.
//!
//! The workload is the drilled-body workflow from `examples/basic.rs`: an
//! exact cube, a square profile extruded into a tool, a regularized
//! difference, exact bounds, the operation's certainty receipt, and both
//! binary STL and OBJ materialization. Every stage consumes its result, so the
//! measured artifact retains the curve grammar, the mesh Boolean kernel, and
//! two export formats.

use std::hint::black_box;

use csgrs::{
    GeometryCertainty, GeometryContext, Real, TriangleMesh, curve,
    io::{obj::to_obj, stl::to_stl_binary},
    solid,
    solid::SolidExt,
};

/// One completed composed workflow. Returns the exported byte counts, the
/// result triangle count, and the weakest consumed certainty.
fn workload(
    width: i64,
) -> Result<(usize, usize, usize, GeometryCertainty), Box<dyn std::error::Error>> {
    let body: TriangleMesh = solid::cube(Real::from(black_box(width)))?;
    let profile = curve::square(Real::from(4))?;
    let tool = curve::try_extrude(&profile, Real::from(16), &GeometryContext::STRICT)?;
    let certainty = tool.certainty;
    let placed = tool
        .into_value()
        .translated(Real::from(4), Real::from(4), Real::from(-2))?;
    let part = body.try_difference(&placed)?;
    // Disambiguated from the inherent Hypermesh method of the same name.
    let bounds = SolidExt::exact_bounds(&part)?;
    assert!(bounds.is_some(), "drilled body has exact bounds");
    let stl = to_stl_binary(&part, "drilled_cube")?;
    let obj = to_obj(&part, "drilled_cube")?;
    let mut triangles = 0usize;
    part.visit_native_triangles(|corners| {
        black_box(&corners);
        triangles += 1;
    });
    Ok((stl.len(), obj.len(), triangles, certainty))
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
    println!(
        "stl_bytes={} obj_bytes={} triangles={} certainty={:?}",
        last.0, last.1, last.2, last.3
    );
    Ok(())
}
