//! Raster-tracing consumer: decoded grayscale pixels to an exact region.
//!
//! `csgrs::curve::from_image` consumes an already-decoded `GrayImage`, so this
//! consumer constructs the buffer itself without calling an image decoder. It
//! exists to attribute the raster-tracing path's artifact cost and inspect whether
//! codec code reaches a linked consumer. Exactness is relative to thresholded unit
//! pixel cells, not an inferred continuous image boundary.

use std::hint::black_box;

use csgrs::curve;
use image::{GrayImage, Luma};

/// Fills an interior pixel pattern, traces at threshold 128 and queries strict bounds.
/// Pixels with `(x + y) % 7 != 0` are white; the one-pixel border remains black.
/// Returns whether the region is nonempty through its optional bounds. Tracing and
/// bounds errors propagate boxed; allocation/work has no explicit budget here.
/// Current entry-point inputs clamp side to at least three. The buffer and region
/// are destroyed before returning, so their full construction lifetime is timed.
fn workload(side: u32) -> Result<bool, Box<dyn std::error::Error>> {
    let mut raster = GrayImage::from_pixel(side, side, Luma([0]));
    for y in 1..side.saturating_sub(1) {
        for x in 1..side.saturating_sub(1) {
            if (x + y) % 7 != 0 {
                raster.put_pixel(x, y, Luma([255]));
            }
        }
    }
    let region = curve::from_image(black_box(&raster), 128)?;
    Ok(curve::bounding_box(&region)?.is_some())
}

/// Parses side/repetitions, warms once, then repeatedly constructs and traces a raster.
/// Missing/malformed arguments default to 32/1, clamped to at least 3/1. Native timing
/// excludes warmup, parsing and printing and truncates nanoseconds per iteration;
/// WASM omits timing. Errors stop execution; the final bounds flag is not asserted.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let side = std::env::args()
        .nth(1)
        .and_then(|value| value.parse::<u32>().ok())
        .unwrap_or(32)
        .max(3);
    let repeats = std::env::args()
        .nth(2)
        .and_then(|value| value.parse::<u32>().ok())
        .unwrap_or(1)
        .max(1);

    // One warm iteration precedes the timed loop; caches are process-local.
    let mut last = workload(side)?;
    #[cfg(not(target_arch = "wasm32"))]
    let started = std::time::Instant::now();
    for _ in 0..repeats {
        last = workload(side)?;
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let elapsed = started.elapsed().as_nanos();
        println!(
            "repeats={repeats} elapsed_ns={elapsed} ns_per_iteration={}",
            elapsed / u128::from(repeats)
        );
    }
    println!("bounded={last}");
    Ok(())
}
