//! Composed CAD consumer: profile, extrusion, Boolean, bounds, and export.
//!
//! The default workload follows the drilled-body construction in `examples/basic.rs`:
//! an exact cube, square-profile extrusion, regularized difference, strict bounds,
//! native triangle traversal, and binary STL/OBJ materialization. The printed receipt
//! comes from extrusion alone; it is not an aggregate chain receipt. Runtime inputs
//! and consumed outputs exercise these paths for artifact attribution, whose actual
//! retained code must still be measured.

use std::hint::black_box;

use csgrs::{
    GeometryCertainty, GeometryContext, Real, TriangleMesh, curve,
    io::{obj::to_obj, stl::to_stl_binary},
    solid,
    solid::SolidExt,
};

/// Subtracts a fixed 4×4×16 translated tool from a cube with integer side length.
/// Returns STL/OBJ byte lengths, traversed triangle count and the extrusion's local
/// certainty. Translation/difference/bounds convenience calls expose no separate
/// receipt here. Requires nonempty result bounds; that assertion panics on emptiness.
/// Other operation/export errors propagate boxed. Exports are materialized and dropped,
/// not reimported or compared for source fidelity; all owned temporaries drop on return.
fn workload(
    width: i64,
) -> Result<(usize, usize, usize, GeometryCertainty), Box<dyn std::error::Error>> {
    let body: TriangleMesh = solid::cube(Real::from(black_box(width)))?;
    let profile = curve::square(Real::from(4))?;
    let tool = curve::try_extrude(&profile, Real::from(16), GeometryContext::STRICT)?;
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

/// Parses width/repetitions, warms once and runs the full composition repeatedly.
/// Missing/malformed arguments default to 12/1; repetitions clamp to one. Native
/// elapsed time includes all workflow work and destruction, excluding warmup/printing.
/// WASM omits the timer. Errors/panics stop execution; printed certainty is only the
/// last extrusion receipt and is not asserted or combined across stages/repetitions.
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
