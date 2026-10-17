//! Complete normalized-region construction, exact STEP exchange and continued modeling.
//! Newly admitted complete workflows have absolute cost; failed paths are not speed baselines.
#[path = "../tests/support/region_extrusion.rs"]
mod acceptance;
use hypercurve::OffsetCap;
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

fn measure(name: &str, iterations: u32, mut operation: impl FnMut() -> (usize, usize, u64)) {
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
        "{{\"case\":\"{name}\",\"iterations\":{iterations},\"outcome\":\"certified\",\"step_bytes\":{},\"native_bytes\":{},\"receipt\":{},\"elapsed_ns\":{},\"allocation_calls\":{},\"requested_bytes\":{}}}",
        expected.0,
        expected.1,
        expected.2,
        elapsed.as_nanos(),
        CALLS.load(Ordering::Relaxed),
        BYTES.load(Ordering::Relaxed)
    );
}
fn main() {
    for cap in [OffsetCap::Butt, OffsetCap::Round] {
        for reverse in [false, true] {
            measure(
                &format!("region_exchange/{cap:?}/reverse_{reverse}"),
                3,
                || {
                    let (step, native) = acceptance::round_trip(cap, reverse).unwrap();
                    let receipt = step
                        .bytes()
                        .chain(native.bytes())
                        .fold(0xcbf29ce484222325_u64, |hash, byte| {
                            (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
                        });
                    (step.len(), native.len(), receipt)
                },
            );
        }
    }
}
