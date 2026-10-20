//! Completed similarity workflows; exact checks, archive materialization and replay
//! stay inside each measured operation. No comparison with the old failing scale path.
#[allow(dead_code)]
#[path = "../tests/support/similarity.rs"]
mod acceptance;
use hyperbrep::{Real, builder};
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
fn digest(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
    })
}
fn main() {
    let source = builder::cylinder(5.into(), 7.into()).unwrap();
    for (name, scale, reflected) in [
        (
            "similarity/rational_replay",
            acceptance::ratio(1, 1000),
            false,
        ),
        (
            "similarity/reflected_replay",
            acceptance::ratio(127, 5000),
            true,
        ),
        (
            "similarity/radical_replay",
            Real::from(2).sqrt().unwrap(),
            false,
        ),
    ] {
        let transform = acceptance::similarity(&scale, reflected);
        let expected_volume = Real::from(175) * Real::pi() * &scale * &scale * &scale;
        measure(name, 100, "certified", || {
            let moved = source.transformed(&transform).unwrap();
            let replay = acceptance::reload(moved.model());
            acceptance::equal(
                &replay.solids().next().unwrap().volume().unwrap(),
                &expected_volume,
            );
            acceptance::check_images(&replay);
            digest(replay.to_json().unwrap().as_bytes())
        });
    }
    measure("similarity/primitive_matrix", 1, "certified", || {
        acceptance::primitives();
        40
    });
    measure("similarity/clipped_void_matrix", 1, "certified", || {
        acceptance::clipped_and_voids();
        64
    });
    measure("similarity/selected_three_cycles", 2, "certified", || {
        acceptance::selected_and_repeated();
        3
    });
    measure("similarity/periodic_near", 1, "certified", || {
        acceptance::periodic_and_near();
        6
    });
    measure("similarity/profile_families", 1, "certified", || {
        acceptance::profile_families();
        83
    });
    measure(
        "similarity/rejection_recovery",
        50,
        "typed-rejection-and-certified-recovery",
        || {
            acceptance::rejection_and_recovery();
            4
        },
    );
}
