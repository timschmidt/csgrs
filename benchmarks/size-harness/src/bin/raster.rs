//! Raster-tracing consumer: decoded grayscale pixels to an exact region.
//!
//! `csgrs::curve::from_image` consumes an already-decoded `GrayImage`, so this
//! consumer constructs the buffer itself and never links an image decoder. It
//! exists to attribute the raster-tracing path's artifact cost and to detect
//! whether the `image` dependency's codec features reach a linked consumer.

use std::hint::black_box;

use csgrs::curve;
use image::{GrayImage, Luma};

/// One completed raster-tracing workflow. Returns whether the traced region
/// has exact bounds, which is only decidable once the contours are built.
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
    Ok(curve::try_bounding_box(&region)?.is_some())
}

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
