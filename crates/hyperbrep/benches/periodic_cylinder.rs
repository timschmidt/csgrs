//! Indexed rectangle coverage costs and native cylinder replay costs are separate.
use hyperbrep::{ArchiveLimits, BuildError, GeometryError, RawModel};
#[path = "../tests/support/periodic_cylinder.rs"]
mod fixture;
#[path = "../src/topology/validation/rectangular_cover.rs"]
mod rectangular_cover;
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

fn measure(name: &str, iterations: usize, mut operation: impl FnMut() -> usize) {
    CALLS.store(0, Ordering::Relaxed);
    BYTES.store(0, Ordering::Relaxed);
    COUNTING.store(true, Ordering::Relaxed);
    let started = Instant::now();
    let mut checksum = 0;
    for _ in 0..iterations {
        checksum += black_box(operation());
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
        elapsed / iterations as u32
    );
}

fn digest(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x100000001b3)
    })
}
fn main() {
    for (label, n, iterations) in [
        ("periodic_cover/staircase_1000:", 1_000, 1000),
        ("periodic_cover/staircase_10000:", 10_000, 100),
    ] {
        let mut rectangles = Vec::new();
        for i in 0..n {
            rectangles.push([i, n, i, i + 1]);
            if i + 1 < n {
                rectangles.push([i, i + 1, i + 1, n]);
            }
        }
        println!(
            "fixture name={label} rectangles={} implicit_cells={}",
            rectangles.len(),
            n * n
        );
        measure(label, iterations, || {
            assert!(rectangular_cover::exactly_covers(n, n, black_box(&rectangles)).unwrap());
            rectangles.len()
        });
    }
    for (label, rectangles, levels) in [
        ("periodic_cover/one_side_replay:", vec![[0, 4, 0, 1]], 1),
        (
            "periodic_cover/unequal_sides_replay:",
            vec![[0, 1, 0, 1], [1, 4, 0, 1]],
            1,
        ),
        (
            "periodic_cover/t_junction_replay:",
            vec![[0, 1, 0, 2], [1, 4, 0, 1], [1, 4, 1, 2]],
            2,
        ),
    ] {
        let model = fixture::build(&rectangles, levels, 0).unwrap();
        fixture::check(&fixture::replay(&model));
        let expected = model.to_json().unwrap();
        println!(
            "fixture name={label} vertices={} edges={} faces={} archive_bytes={} archive_fnv1a64={:016x}",
            model.counts().vertices,
            model.counts().edges,
            model.counts().faces,
            expected.len(),
            digest(expected.as_bytes())
        );
        measure(label, 100, || {
            let model = RawModel::from_json(black_box(&expected), &ArchiveLimits::default())
                .unwrap()
                .validate()
                .unwrap();
            fixture::check(&model);
            let actual = model.to_json().unwrap();
            assert_eq!(actual, expected);
            actual.len()
        });
    }
    measure("periodic_cover/one_side_construct:", 100, || {
        let model = fixture::build(black_box(&[[0, 4, 0, 1]]), 1, 0).unwrap();
        fixture::check(&model);
        model.counts().edges
    });
}
