//! Absolute costs of the new complete-image query, with cold construction,
//! reused input and a separately labeled rejection. No topology is published.
#[path = "../tests/support/edge_image_queries.rs"]
mod fixture;
use hyperbrep::{BuildError, Real};
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

fn measure(
    name: &str,
    iterations: u32,
    outcome: &str,
    expected: Result<(), BuildError>,
    mut operation: impl FnMut() -> Result<(), BuildError>,
) {
    assert_eq!(operation(), expected);
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
        "{}",
        serde_json::json!({
            "case": name, "iterations": iterations, "outcome": outcome,
            "elapsed_ns": elapsed.as_nanos(),
            "allocation_calls": CALLS.load(Ordering::Relaxed),
            "requested_bytes": BYTES.load(Ordering::Relaxed),
        })
    );
}

fn main() {
    fixture::parameter_and_radius_failures();
    fixture::unresolved_is_not_success();
    fixture::unsupported_is_not_endpoint_agreement();
    measure(
        "edge_images/fresh_cylinder",
        1_000,
        "certified",
        Ok(()),
        || fixture::cylinder(Real::one(), Real::zero(), Real::tau()).check(),
    );
    let prepared = fixture::cylinder(Real::one(), Real::zero(), Real::tau());
    measure(
        "edge_images/reused_cylinder",
        10_000,
        "certified",
        Ok(()),
        || prepared.check(),
    );
    let invalid = fixture::cylinder(Real::from(2), Real::zero(), Real::tau() * Real::from(2));
    measure(
        "edge_images/reject_wrong_radius",
        1_000,
        "invalid correspondence",
        Err(BuildError::EdgeUseSupportMismatch),
        || invalid.check(),
    );
}
