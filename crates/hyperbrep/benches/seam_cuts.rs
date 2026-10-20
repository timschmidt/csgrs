//! Completed seam-cut workflows; exact checks, archive materialization and replay
//! stay inside each measured operation. New completions have absolute costs; the baseline rejects these seam cuts.
#[allow(dead_code)]
#[path = "../tests/support/seam_cuts.rs"]
mod acceptance;
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Instant;

struct CountingAllocator;
static COUNTING: AtomicBool = AtomicBool::new(false);
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
fn record(size: usize) {
    if COUNTING.load(Ordering::Relaxed) {
        CALLS.fetch_add(1, Ordering::Relaxed);
        BYTES.fetch_add(size, Ordering::Relaxed);
    }
}
// SAFETY: The original pointers and layouts pass unchanged to System; the
// independent counters neither inspect allocations nor change their lifetime.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        record(layout.size());
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        record(layout.size());
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        record(size);
        unsafe { System.realloc(ptr, layout, size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

fn measure(name: &str, iterations: u32, outcome: &str, mut operation: impl FnMut() -> u64) {
    let expected = operation();
    CALLS.store(0, Ordering::Relaxed);
    BYTES.store(0, Ordering::Relaxed);
    COUNTING.store(true, Ordering::Relaxed);
    let started = Instant::now();
    for _ in 0..iterations {
        assert_eq!(black_box(operation()), expected);
    }
    let elapsed = started.elapsed();
    COUNTING.store(false, Ordering::Relaxed);
    println!(
        "{{\"case\":\"{name}\",\"iterations\":{iterations},\"outcome\":\"{outcome}\",\"receipt\":{expected},\"elapsed_ns\":{},\"allocation_calls\":{},\"requested_bytes\":{}}}",
        elapsed.as_nanos(),
        CALLS.load(Ordering::Relaxed),
        BYTES.load(Ordering::Relaxed)
    );
}
fn main() {
    for (name, kind) in [
        ("seams/torus_axial", 0),
        ("seams/torus_longitudinal", 1),
        ("seams/cone_longitudinal", 2),
    ] {
        measure(name, 3, "certified", || acceptance::original(kind));
    }
    measure("seams/similarity_matrix", 1, "certified", || {
        acceptance::transforms() as u64
    });
    measure("seams/boundary_provenance", 3, "certified", || {
        acceptance::boundary_provenance() as u64
    });
    measure("seams/closed_boundary", 3, "certified", || {
        acceptance::closed_boundary_provenance() as u64
    });
}
