//! Standalone measurement only; this allocator is never linked into the library.
use csgrs::{Real, curve, errors::ValidationError};
use hypercurve::CurveRegion2;
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};

struct CountingAllocator;
static ALLOCS: AtomicUsize = AtomicUsize::new(0);
static REALLOCS: AtomicUsize = AtomicUsize::new(0);
static ALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);
static REALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

fn added(bytes: usize) {
    let live = LIVE.fetch_add(bytes, Relaxed) + bytes;
    PEAK.fetch_max(live, Relaxed);
}

fn allocated(pointer: *mut u8, bytes: usize) -> *mut u8 {
    if !pointer.is_null() {
        ALLOCS.fetch_add(1, Relaxed);
        ALLOC_BYTES.fetch_add(bytes, Relaxed);
        added(bytes);
    }
    pointer
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        allocated(unsafe { System.alloc(layout) }, layout.size())
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        allocated(unsafe { System.alloc_zeroed(layout) }, layout.size())
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) };
        LIVE.fetch_sub(layout.size(), Relaxed);
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let result = unsafe { System.realloc(pointer, layout, new_size) };
        if !result.is_null() {
            REALLOCS.fetch_add(1, Relaxed);
            REALLOC_BYTES.fetch_add(new_size, Relaxed);
            if new_size >= layout.size() {
                added(new_size - layout.size());
            } else {
                LIVE.fetch_sub(layout.size() - new_size, Relaxed);
            }
        }
        result
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

#[derive(Clone, Copy)]
struct Stats {
    allocations: usize,
    reallocations: usize,
    allocated_bytes: usize,
    reallocated_bytes: usize,
    live: usize,
}

fn stats() -> Stats {
    Stats {
        allocations: ALLOCS.load(Relaxed),
        reallocations: REALLOCS.load(Relaxed),
        allocated_bytes: ALLOC_BYTES.load(Relaxed),
        reallocated_bytes: REALLOC_BYTES.load(Relaxed),
        live: LIVE.load(Relaxed),
    }
}

fn calibrate() {
    let layout = Layout::from_size_align(4096, 8).unwrap();
    let before = stats();
    let pointer = unsafe { std::alloc::alloc(layout) };
    assert!(!pointer.is_null());
    black_box(pointer);
    let during = stats();
    unsafe { std::alloc::dealloc(pointer, layout) };
    let after = stats();
    assert_eq!(during.allocations - before.allocations, 1);
    assert_eq!(during.allocated_bytes - before.allocated_bytes, 4096);
    assert_eq!(during.live - before.live, 4096);
    assert_eq!(after.live, before.live);
}

trait Attempt {
    fn attempt(self) -> Result<CurveRegion2, ValidationError>;
}
impl Attempt for CurveRegion2 {
    fn attempt(self) -> Result<CurveRegion2, ValidationError> {
        Ok(self)
    }
}
impl Attempt for Result<CurveRegion2, ValidationError> {
    fn attempt(self) -> Result<CurveRegion2, ValidationError> {
        self
    }
}

fn r(value: i32) -> Real {
    value.into()
}

fn build(case: &str) -> CurveRegion2 {
    let value = match case {
        "involute_gear" => curve::involute_gear(r(2), 20, r(20), r(0), r(0), 4),
        "cycloidal_gear" => curve::cycloidal_gear(r(2), 12, r(2), r(0), 4),
        "involute_rack" => curve::involute_rack(r(2), 4, r(20), r(0), r(0)),
        "cycloidal_rack" => curve::cycloidal_rack(r(2), 4, r(0), 8),
        "airfoil_naca4" => curve::airfoil_naca4(r(2), r(4), r(12), r(20), 80),
        _ => panic!("unknown fixture"),
    };
    let value = value.attempt().expect("original constructor must complete");
    assert!(!value.is_empty());
    value
}

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let case = &args[1];
    let temperature = &args[2];
    assert!(temperature == "cold" || temperature == "warm");
    calibrate();
    if temperature == "warm" {
        drop(black_box(build(case)));
    }
    let before = stats();
    PEAK.store(before.live, Relaxed);
    let value = black_box(build(case));
    let after = stats();
    let peak = PEAK.load(Relaxed);
    let components = value.len();
    drop(value);
    let retained = stats().live as i128 - before.live as i128;
    println!(
        "case,temperature,allocations,reallocations,allocated_bytes,reallocated_bytes,live_delta_bytes,peak_additional_requested_bytes,retained_delta_bytes,components"
    );
    println!(
        "{case},{temperature},{},{},{},{},{},{},{retained},{components}",
        after.allocations - before.allocations,
        after.reallocations - before.reallocations,
        after.allocated_bytes - before.allocated_bytes,
        after.reallocated_bytes - before.reallocated_bytes,
        after.live as i128 - before.live as i128,
        peak - before.live,
    );
}
