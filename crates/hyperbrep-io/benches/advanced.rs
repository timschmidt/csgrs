use hyperbrep::{OperationContext, Real, WorkBudget};
use hyperbrep_io::step::{ErrorKind, ImportLimits, StepFile, StepReadOptions};
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
fn digest(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
    })
}

const BOX: &str = include_str!("../tests/fixtures/occt-advanced-box.step");
const BOX_3D: &str = include_str!("../tests/fixtures/occt-advanced-box-3d.step");
const HOLE: &str = include_str!("../tests/fixtures/occt-advanced-hole.step");
fn argument(text: &str, id: u64, index: usize, value: &str) -> String {
    let file = StepFile::read(text.as_bytes(), &StepReadOptions::default()).unwrap();
    let span = file
        .syntax()
        .entity(id)
        .unwrap()
        .components()
        .next()
        .unwrap()
        .arguments()
        .nth(index)
        .unwrap()
        .span();
    let mut result = text.to_string();
    result.replace_range(span.start..span.end, value);
    result
}
fn main() {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    });
    let reparam = argument(&argument(BOX, 39, 1, "(7.,0.)"), 40, 2, "2.");
    let mut two = argument(BOX, 10, 1, "(#11,#15,#9000)");
    two.insert_str(
        two.rfind("ENDSEC;").unwrap(),
        "#9000=MANIFOLD_SOLID_BREP('second',#16);\n",
    );
    for (label, text, bodies, vertices, edges, faces, volume, maps) in [
        ("advanced/box_3d:", BOX_3D, 1, 8, 12, 6, 24, 1),
        ("advanced/box_pcurves:", BOX, 1, 8, 12, 6, 24, 4),
        ("advanced/hole:", HOLE, 1, 16, 24, 10, 20, 4),
        ("advanced/two_bodies:", two.as_str(), 2, 8, 12, 6, 24, 4),
        (
            "advanced/reparameterized:",
            reparam.as_str(),
            1,
            8,
            12,
            6,
            24,
            4,
        ),
    ] {
        let volume = (Real::from(volume) / Real::from(1_000_000_000)).unwrap();
        let run = || {
            let file =
                StepFile::read(black_box(text.as_bytes()), &StepReadOptions::default()).unwrap();
            let imported = file
                .import_representation(10, &ImportLimits::default(), &context)
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
                let c = body.model().counts();
                assert_eq!(
                    (c.vertices, c.edges, c.faces, c.edge_uses),
                    (vertices, edges, faces, 2 * edges)
                );
                assert_eq!(body.solid().volume().unwrap(), volume);
                assert_eq!(body.explicit_edges().count(), edges);
                for edge in body.explicit_edges() {
                    assert_eq!(edge.parameters.len(), maps);
                }
                if text == reparam {
                    let edge = body.explicit_edges().find(|e| e.source == 21).unwrap();
                    assert_eq!(edge.master, 31);
                    assert_eq!(
                        edge.parameters[0].start,
                        (Real::from(-7) / Real::from(2)).unwrap()
                    );
                    assert_eq!(
                        edge.parameters[0].end,
                        (Real::from(-3) / Real::from(2)).unwrap()
                    );
                }
                output.push_str(&body.model().to_json().unwrap());
            }
            output
        };
        let expected = run();
        println!(
            "fixture name={label} input_bytes={} output_bytes={} output_fnv1a64={:016x}",
            text.len(),
            expected.len(),
            digest(expected.as_bytes())
        );
        measure(label, 100, || {
            let actual = run();
            assert_eq!(actual, expected);
            actual.len()
        });
    }
    let bad = argument(BOX, 53, 1, "(-1.,0.)");
    measure("advanced/reject_alternate_sense:", 1000, || {
        let file = StepFile::read(black_box(bad.as_bytes()), &StepReadOptions::default()).unwrap();
        let error = file
            .import_representation(10, &ImportLimits::default(), &context)
            .result
            .unwrap()
            .unwrap_err();
        assert_eq!(error.entity, Some(26));
        assert!(matches!(error.kind, ErrorKind::Invalid(_)));
        1
    });
}
