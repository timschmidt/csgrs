use hyperbrep::{OperationContext, Point2, Point3, Real, WorkBudget, builder};
use hyperbrep_io::step::{StepBody, StepWriteOptions, WriteErrorKind, write_faceted};
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
fn ratio(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn digest(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
    })
}
fn export(
    name: &str,
    inputs: &[StepBody<'_>],
    expected: (usize, usize, usize, usize),
    iterations: usize,
) {
    let context = OperationContext::strict(WorkBudget {
        refinements: 0,
        promotions: 0,
    });
    let options = StepWriteOptions::new("benchmark.step", "2026-10-15T00:00:00");
    let run = || {
        let attempt = write_faceted(black_box(inputs), &options, &context);
        let output = attempt.result.unwrap().unwrap();
        assert_eq!(output.bodies().len(), inputs.len());
        for body in output.bodies() {
            assert_eq!(
                (
                    body.vertices().len(),
                    body.faces().len(),
                    body.wires().len(),
                    body.edge_uses().len()
                ),
                expected
            );
        }
        output.into_text()
    };
    let expected = run();
    println!(
        "fixture name={name} bodies={} output_bytes={} output_fnv1a64={:016x}",
        inputs.len(),
        expected.len(),
        digest(expected.as_bytes())
    );
    measure(name, iterations, || {
        let output = run();
        assert_eq!(output, expected);
        output.len()
    });
}
fn main() {
    let box_body = builder::cuboid(
        Point3::origin(),
        Point3::new(ratio(2, 1000), ratio(3, 1000), ratio(4, 1000)),
    )
    .unwrap();
    let solid = box_body.bodies().next().unwrap();
    assert_eq!(solid.volume().unwrap(), ratio(24, 1_000_000_000));
    let input = StepBody { solid, name: "box" };
    export("export/box:", &[input], (8, 6, 6, 24), 1000);
    let profile = |points: &[(i32, i32)]| {
        points
            .iter()
            .map(|&(x, y)| Point2::new(ratio(x, 1000), ratio(y, 1000)))
            .collect::<Vec<_>>()
    };
    let hole = builder::extrude_region(
        &profile(&[(0, 0), (4, 0), (4, 3), (0, 3)]),
        &[profile(&[(1, 1), (3, 1), (3, 2), (1, 2)])],
        Real::zero(),
        ratio(2, 1000),
    )
    .unwrap();
    let solid = hole.bodies().next().unwrap();
    assert_eq!(solid.volume().unwrap(), ratio(20, 1_000_000_000));
    export(
        "export/hole:",
        &[StepBody {
            solid,
            name: "hole",
        }],
        (16, 10, 12, 48),
        500,
    );
    let concave = builder::extrude(
        &profile(&[(0, 0), (3, 0), (3, 1), (1, 1), (1, 3), (0, 3)]),
        Real::zero(),
        ratio(2, 1000),
    )
    .unwrap();
    let solid = concave.bodies().next().unwrap();
    assert_eq!(solid.volume().unwrap(), ratio(10, 1_000_000_000));
    export(
        "export/concave:",
        &[StepBody {
            solid,
            name: "concave",
        }],
        (12, 8, 8, 36),
        500,
    );
    let split = box_body
        .model()
        .edges()
        .next()
        .unwrap()
        .split(ratio(1, 8).into())
        .unwrap();
    export(
        "export/split:",
        &[StepBody {
            solid: split.model().solids().next().unwrap(),
            name: "split",
        }],
        (9, 6, 6, 26),
        1000,
    );
    export("export/32_bodies:", &[input; 32], (8, 6, 6, 24), 30);
    let third = builder::cuboid(
        Point3::origin(),
        Point3::new(ratio(1, 3), Real::one(), Real::one()),
    )
    .unwrap();
    let inputs = [StepBody {
        solid: third.bodies().next().unwrap(),
        name: "third",
    }];
    let context = OperationContext::strict(WorkBudget {
        refinements: 0,
        promotions: 0,
    });
    let options = StepWriteOptions::new("third.step", "2026-10-15T00:00:00");
    measure("export/reject_third:", 1000, || {
        assert!(matches!(
            write_faceted(black_box(&inputs), &options, &context)
                .result
                .unwrap()
                .unwrap_err()
                .kind,
            WriteErrorKind::NonTerminatingDecimal
        ));
        1
    });
}
