use hyperbrep::{OperationContext, Real, WorkBudget};
use hyperbrep_io::step::{ErrorKind, ImportLimits, Resource, StepFile, StepReadOptions};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Instant;

#[path = "../tests/support/mod.rs"]
mod support;

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
// SAFETY: delegation preserves System's layout/pointer contract. Counters do not access memory.
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
    bytes.iter().fold(0xcbf29ce484222325, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
    })
}
fn faceted(
    name: &str,
    source: &str,
    expected: (usize, usize, usize, usize, i32),
    iterations: usize,
) {
    let (bodies, vertices, edges, faces, volume_mm3) = expected;
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    });
    let volume = (Real::from(volume_mm3) / Real::from(1_000_000_000)).unwrap();
    let run = || {
        let file =
            StepFile::read(black_box(source.as_bytes()), &StepReadOptions::default()).unwrap();
        let imported = file
            .import_faceted_representation(502, &ImportLimits::default(), &context)
            .result
            .unwrap()
            .unwrap();
        assert_eq!(imported.bodies().len(), bodies);
        assert_eq!(
            imported.interpreted_entities().len() + imported.retained_entities().len(),
            file.syntax().entities().len()
        );
        let mut output = String::new();
        for body in imported.bodies() {
            let counts = body.model().counts();
            assert_eq!(
                (
                    counts.vertices,
                    counts.edges,
                    counts.faces,
                    counts.edge_uses
                ),
                (vertices, edges, faces, 2 * edges)
            );
            assert_eq!(body.edge_uses().count(), 2 * edges);
            assert_eq!(body.solid().volume().unwrap(), volume);
            output.push_str(&body.model().to_json().unwrap());
        }
        output
    };
    // First output is a determinism reference, not an independent geometry oracle.
    // Correctness expectations above come from the authored dimensional recipes.
    let expected = run();
    println!(
        "fixture name={name} input_bytes={} output_bytes={} output_fnv1a64={:016x}",
        source.len(),
        expected.len(),
        digest(expected.as_bytes())
    );
    measure(name, iterations, || {
        let output = run();
        assert_eq!(output, expected);
        output.len()
    });
}
fn units(name: &str, depth: usize, iterations: usize) {
    let source = support::unit_chain(depth, "1.");
    println!(
        "fixture name={name} input_bytes={} depth={depth}",
        source.len()
    );
    measure(name, iterations, || {
        let file =
            StepFile::read(black_box(source.as_bytes()), &StepReadOptions::default()).unwrap();
        assert_eq!(
            file.context_units(1, &ImportLimits::default())
                .unwrap()
                .length
                .scale_to_si,
            Real::one()
        );
        file.syntax().entities().len()
    });
}
fn main() {
    faceted(
        "step/box:",
        include_str!("../tests/fixtures/faceted-box.step"),
        (1, 8, 12, 6, 24),
        100,
    );
    faceted(
        "step/two_bodies:",
        include_str!("../tests/fixtures/faceted-two-boxes.step"),
        (2, 8, 12, 6, 24),
        50,
    );
    faceted(
        "step/through_hole:",
        include_str!("../tests/fixtures/faceted-through-hole.step"),
        (1, 16, 24, 10, 20),
        30,
    );
    faceted(
        "step/oblique_tetrahedron:",
        include_str!("../tests/fixtures/faceted-tetrahedron.step"),
        (1, 4, 6, 4, 12),
        50,
    );
    units("step/units_2000:", 2000, 10);
    units("step/units_20000:", 20_000, 3);
    let source = support::unit_chain(100, "1.E100");
    let file = StepFile::read(source.as_bytes(), &StepReadOptions::default()).unwrap();
    measure("step/reject_scale_bits:", 100, || {
        assert!(matches!(
            file.context_units(
                1,
                &ImportLimits {
                    unit_scale_bits: 512,
                    ..ImportLimits::default()
                }
            )
            .unwrap_err()
            .kind,
            ErrorKind::Limit {
                resource: Resource::UnitScaleBits,
                ..
            }
        ));
        1
    });
}
