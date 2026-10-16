//! New supplied-pcurve workload costs; all output is materialized and checked.
#[allow(dead_code)]
#[path = "../tests/support/analytic_cylinder.rs"]
mod cylinder;
#[path = "../tests/support/supplied_pcurves.rs"]
mod fixture;
use hyperbrep::{BuildError, Real};
use hyperbrep_io::step::{ErrorKind, ImportLimits, StepFile};
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
fn completed(file: &StepFile<'_>) -> u64 {
    let imported = file
        .import_representation(10, &ImportLimits::default(), &cylinder::context())
        .result
        .unwrap()
        .unwrap();
    assert_eq!(imported.bodies().len(), 1);
    assert_eq!(
        imported.interpreted_entities().len() + imported.retained_entities().len(),
        file.syntax().entities().len()
    );
    let body = &imported.bodies()[0];
    cylinder::equal(
        &body.solid().volume().unwrap(),
        &(Real::from(175) * Real::pi()),
    );
    for edge in body.explicit_edges() {
        let maps = match edge.geometry {
            200 | 230 => 4,
            260 => 3,
            _ => 1,
        };
        assert_eq!(edge.parameters.len(), maps);
        assert!(edge.parameters.iter().any(|map| map.curve == edge.master));
        assert_eq!(edge.edge.uses().count(), 2);
    }
    body.model()
        .to_json()
        .unwrap()
        .as_bytes()
        .iter()
        .fold(0xcbf29ce484222325, |hash, byte| {
            (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
        })
}
fn main() {
    fixture::source_maps();
    fixture::representations();
    fixture::composition();
    fixture::failures();
    fixture::admission();
    let full = include_str!("../tests/fixtures/supplied-cylinder.step");
    let partial = fixture::supplied(&cylinder::partitioned());
    for (fresh, reused, source) in [
        ("pcurves/full_fresh", "pcurves/full_reused", full),
        (
            "pcurves/partial_fresh",
            "pcurves/partial_reused",
            partial.as_str(),
        ),
    ] {
        measure(fresh, 100, "certified", || {
            completed(&cylinder::read(source))
        });
        let file = cylinder::read(source);
        measure(reused, 100, "certified", || completed(&file));
    }
    measure("pcurves/edit_replay_composition", 20, "certified", || {
        fixture::composition();
        1
    });
    let bad = cylinder::argument(full, 204, 1, "(2.,7.000000000001)");
    measure(
        "pcurves/reject_nonmaster_height",
        100,
        "invalid input",
        || {
            assert!(matches!(
                cylinder::import(&bad).unwrap_err().kind,
                ErrorKind::Build(BuildError::EdgeUseSupportMismatch)
            ));
            1
        },
    );
}
