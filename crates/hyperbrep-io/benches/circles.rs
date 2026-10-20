use hyperbrep::Real;
use hyperbrep_io::step::{ErrorKind, ImportLimits, ImportedRepresentation};
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
#[allow(dead_code)]
#[path = "../tests/support/analytic_cylinder.rs"]
mod fixture;

fn materialize(
    imported: ImportedRepresentation,
    vertices: usize,
    edges: usize,
    faces: usize,
    span: &Real,
) -> String {
    assert_eq!(imported.bodies().len(), 1);
    let body = &imported.bodies()[0];
    fixture::check(body.model());
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
    for edge in body.explicit_edges() {
        assert_eq!(edge.parameters.len(), 1);
        let map = &edge.parameters[0];
        assert_eq!(map.curve, edge.geometry);
        assert_eq!(edge.master, edge.geometry);
        if edge.edge.curve().kind() == hyperbrep::Curve3Kind::CircleArc {
            fixture::equal(&(&map.end - &map.start), span);
        } else {
            fixture::equal(&map.start, &Real::zero());
            fixture::equal(&map.end, &(Real::from(7) / Real::from(2)).unwrap());
        }
    }
    body.model().to_json().unwrap()
}
fn digest(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
    })
}
fn main() {
    let partial = fixture::partitioned();
    for (name, text, vertices, edges, faces, span) in [
        ("closed", fixture::CYLINDER, 2, 3, 3, Real::tau()),
        ("partial", partial.as_str(), 4, 6, 4, Real::pi()),
    ] {
        // Initial materialization establishes the expected artifact. Timed runs have
        // warm shared constants; each import constructs a fresh native model.
        let cold = || {
            materialize(
                fixture::import(text).unwrap(),
                vertices,
                edges,
                faces,
                &span,
            )
        };
        let expected = cold();
        println!(
            "fixture name={name} input_bytes={} output_bytes={} output_fnv1a64={:016x}",
            text.len(),
            expected.len(),
            digest(expected.as_bytes())
        );
        measure(&format!("circles/{name}_fresh_document:"), 100, || {
            let actual = cold();
            assert_eq!(actual, expected);
            actual.len()
        });
        let parsed = fixture::read(text);
        measure(&format!("circles/{name}_reused_document:"), 100, || {
            let attempt =
                parsed.import_representation(10, &ImportLimits::default(), &fixture::context());
            let imported = attempt.result.unwrap().unwrap();
            assert_eq!(
                imported.interpreted_entities().len() + imported.retained_entities().len(),
                parsed.syntax().entities().len()
            );
            let actual = materialize(imported, vertices, edges, faces, &span);
            assert_eq!(actual, expected);
            actual.len()
        });
    }
    measure("circles/edits_replay_composition:", 25, || {
        fixture::composition();
        1
    });
    let bad = fixture::argument(fixture::CYLINDER, 23, 1, "(3.,4.000000000001,7.)");
    measure("circles/reject_off_circle:", 1000, || {
        let error = fixture::import(&bad).unwrap_err();
        assert_eq!(error.entity, Some(24));
        assert!(matches!(
            error.kind,
            ErrorKind::Invalid("point is not on the exact source circle")
        ));
        1
    });
}
