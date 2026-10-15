//! Focused cost of corrected polygons; no comparison with the old incorrect set.
use csgrs::{GeometryCertainty, GeometryContext, Real, curve};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
use std::time::Instant;

struct Counted;
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static REQUESTED: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

fn allocated(bytes: usize) {
    ALLOCATIONS.fetch_add(1, Relaxed);
    REQUESTED.fetch_add(bytes, Relaxed);
    let live = LIVE.fetch_add(bytes, Relaxed) + bytes;
    PEAK.fetch_max(live, Relaxed);
}

unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            allocated(layout.size());
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            allocated(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Relaxed);
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, old: Layout, size: usize) -> *mut u8 {
        let result = unsafe { System.realloc(pointer, old, size) };
        if !result.is_null() {
            LIVE.fetch_sub(old.size(), Relaxed);
            allocated(size);
        }
        result
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;

fn complete(shape: &str, offset: u32) {
    let (w, h, r, n) = match shape {
        "octagon" => (
            Real::from(4) + Real::from(offset),
            Real::from(2),
            (Real::one() / Real::from(2)).unwrap(),
            1,
        ),
        "benchmark" => (
            Real::from(12) + Real::from(offset),
            Real::from(8),
            Real::from(2),
            8,
        ),
        "readme" => (
            Real::try_from(2.4).unwrap() + Real::from(offset),
            Real::try_from(1.5).unwrap(),
            Real::try_from(0.28).unwrap(),
            12,
        ),
        _ => panic!("unknown workload"),
    };
    let result = curve::rounded_rectangle_with_context(
        w.clone(),
        h.clone(),
        r.clone(),
        n,
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Certified);
    let profiles = result.value.boundary_profiles().unwrap();
    assert_eq!(profiles.len(), 1);
    let (material, holes) = profiles[0].linear_contours().unwrap().unwrap();
    assert!(holes.is_empty());
    assert_eq!(material.segments().len(), 4 * (n + 1));
    for expected in [
        [Real::zero(), r.clone()],
        [r.clone(), Real::zero()],
        [&w - &r, Real::zero()],
        [w.clone(), r.clone()],
        [w.clone(), &h - &r],
        [&w - &r, h.clone()],
        [r.clone(), h.clone()],
        [Real::zero(), &h - &r],
    ] {
        assert!(
            material
                .segments()
                .iter()
                .any(|s| s.start().x() == &expected[0] && s.start().y() == &expected[1])
        );
    }
    if n == 1 {
        assert_eq!(
            result.value.filled_area().unwrap(),
            Some(w * h - Real::from(2) * &r * r)
        );
    }
    black_box(result);
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let shape = &args[1];
    let cache = &args[2];
    let iterations: usize = args[3].parse().unwrap();
    assert!(cache == "warm" || cache == "miss");
    let offset = |i: usize| if cache == "warm" { 0 } else { (i % 128) as u32 };
    // 128 distinct rings exceed the 64-entry FIFO. These are ring-cache misses,
    // not a claim that all scalar/kernel caches are cold.
    for i in 0..128 {
        complete(shape, offset(i));
    }
    let initial_live = LIVE.load(Relaxed);
    let initial_allocations = ALLOCATIONS.load(Relaxed);
    let initial_requested = REQUESTED.load(Relaxed);
    PEAK.store(initial_live, Relaxed);
    let start = Instant::now();
    for i in 0..iterations {
        complete(shape, offset(i));
    }
    let nanos = start.elapsed().as_nanos();
    let allocations = ALLOCATIONS.load(Relaxed) - initial_allocations;
    let requested = REQUESTED.load(Relaxed) - initial_requested;
    let retained = LIVE.load(Relaxed) as i128 - initial_live as i128;
    let peak = PEAK.load(Relaxed) - initial_live;
    println!(
        "{{\"shape\":\"{shape}\",\"cache\":\"{cache}\",\"iterations\":{iterations},\"completed\":{iterations},\"policy\":\"strict\",\"certainty\":\"Certified\",\"oracle\":\"exact tangents and one-chord area\",\"nanos\":{nanos},\"allocations\":{allocations},\"requested_bytes\":{requested},\"retained_byte_delta\":{retained},\"peak_live_bytes_above_start\":{peak}}}"
    );
}
