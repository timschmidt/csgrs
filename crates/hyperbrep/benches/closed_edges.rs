//! Absolute costs of new closed-edge operations, including exact checks and
//! complete archive equality inside the measured loop. Input models are reused.
use hyperbrep::{ArchiveLimits, Curve3Kind, Model, RawModel, Real};
#[path = "../tests/support/closed_cylinder.rs"]
mod fixture;
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
// SAFETY: System receives the original layout/pointer; counters do not access allocations.
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

fn digest(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x100000001b3)
    })
}

fn measure(name: &str, mut operation: impl FnMut() -> Model) {
    let expected_model = operation();
    fixture::check(&fixture::replay(&expected_model));
    let expected = expected_model.to_json().unwrap();
    let counts = expected_model.counts();
    println!(
        "fixture name={name} vertices={} edges={} edge_uses={} faces={} archive_bytes={} archive_fnv1a64={:016x}",
        counts.vertices,
        counts.edges,
        counts.edge_uses,
        counts.faces,
        expected.len(),
        digest(expected.as_bytes())
    );
    let iterations = 100;
    CALLS.store(0, Ordering::Relaxed);
    BYTES.store(0, Ordering::Relaxed);
    COUNTING.store(true, Ordering::Relaxed);
    let started = Instant::now();
    let mut checksum = 0;
    for _ in 0..iterations {
        let model = operation();
        fixture::check(&model);
        assert_eq!(model.counts(), counts);
        let actual = model.to_json().unwrap();
        assert_eq!(actual, expected);
        checksum += black_box(actual.len());
    }
    let elapsed = started.elapsed();
    COUNTING.store(false, Ordering::Relaxed);
    println!(
        "allocation_metadata name={name} calls={} requested_bytes={}",
        CALLS.load(Ordering::Relaxed),
        BYTES.load(Ordering::Relaxed)
    );
    println!(
        "{name} {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
}

fn main() {
    let model = fixture::build(Real::zero(), 0, false).unwrap();
    let archive = model.to_json().unwrap();
    let shifted = fixture::build(Real::zero(), -2, true)
        .unwrap()
        .to_json()
        .unwrap();
    measure("closed_edges/build:", || {
        fixture::build(Real::zero(), 0, false).unwrap()
    });
    measure("closed_edges/replay:", || {
        RawModel::from_json(black_box(&archive), &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap()
    });
    measure("closed_edges/split_circle:", || {
        model
            .edges()
            .find(|e| e.curve().kind() == Curve3Kind::CircleArc)
            .unwrap()
            .split(Real::pi().into())
            .unwrap()
            .into_model()
    });
    measure("closed_edges/split_seam:", || {
        model
            .edges()
            .find(|e| e.curve().kind() == Curve3Kind::Line)
            .unwrap()
            .split((Real::one() / Real::from(2)).unwrap().into())
            .unwrap()
            .into_model()
    });
    measure("closed_edges/reversed_shifted_replay:", || {
        RawModel::from_json(black_box(&shifted), &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap()
    });
}
