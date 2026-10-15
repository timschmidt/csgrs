//! Focused cost probe, identical source against the preserved baseline and candidate.
use csgrs::{Real, adapter::RawCurveRegion};
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

fn complete(shape: &str, width: u32) {
    let w = Real::from(width);
    let (region, expected) = match shape {
        "rectangle" => (
            RawCurveRegion::rectangle(w.clone(), Real::from(3)).unwrap(),
            Real::from(3) * w,
        ),
        "circle4" => (
            RawCurveRegion::circle(w.clone(), 4).unwrap(),
            Real::from(2) * &w * w,
        ),
        "polygon" => (
            RawCurveRegion::polygon(&[
                [Real::zero(), Real::zero()],
                [w.clone(), Real::zero()],
                [w.clone(), Real::from(3)],
                [Real::zero(), Real::from(3)],
            ])
            .unwrap(),
            Real::from(3) * w,
        ),
        _ => panic!("unknown workload"),
    };
    assert_eq!(region.native().filled_area().unwrap(), Some(expected));
    black_box(region);
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let shape = &args[1];
    let cache = &args[2];
    let iterations: usize = args[3].parse().unwrap();
    assert!(cache == "warm" || cache == "miss");
    let width = |i: usize| {
        if cache == "warm" {
            1
        } else {
            (i % 128 + 1) as u32
        }
    };
    // 128 unique rings exceed the 64-entry FIFO. Every measured miss case
    // cycles in that order; warm cases repeat one fully materialized ring.
    for i in 0..128 {
        complete(shape, width(i));
    }
    let initial_live = LIVE.load(Relaxed);
    let initial_allocations = ALLOCATIONS.load(Relaxed);
    let initial_requested = REQUESTED.load(Relaxed);
    PEAK.store(initial_live, Relaxed);
    let start = Instant::now();
    for i in 0..iterations {
        complete(shape, width(i));
    }
    let nanos = start.elapsed().as_nanos();
    let allocations = ALLOCATIONS.load(Relaxed) - initial_allocations;
    let requested = REQUESTED.load(Relaxed) - initial_requested;
    let retained = LIVE.load(Relaxed) as i128 - initial_live as i128;
    let peak = PEAK.load(Relaxed) - initial_live;
    println!(
        "{{\"shape\":\"{shape}\",\"cache\":\"{cache}\",\"iterations\":{iterations},\"completed\":{iterations},\"policy\":\"strict\",\"certainty\":\"Certified\",\"oracle\":\"exact area\",\"nanos\":{nanos},\"allocations\":{allocations},\"requested_bytes\":{requested},\"retained_byte_delta\":{retained},\"peak_live_bytes_above_start\":{peak}}"
    );
}
