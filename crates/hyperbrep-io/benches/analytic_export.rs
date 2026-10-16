//! Absolute costs for exact analytic export, checked and fully materialized.
#[allow(dead_code)]
#[path = "../tests/support/analytic_export.rs"]
mod acceptance;
use acceptance::cylinder;
use hyperbrep::{Real, builder};
use hyperbrep_io::step::{StepBody, StepWriteOptions, WriteErrorKind, write_advanced};
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
fn export(name: &str, inputs: &[StepBody<'_>], iterations: u32) {
    let options = StepWriteOptions::new("benchmark.step", "2026-10-16T00:00:00");
    let output = acceptance::export(inputs, &options);
    acceptance::reimport(&output);
    let expected = output.text().to_owned();
    let counts = output
        .bodies()
        .iter()
        .map(|b| {
            (
                b.vertices().len(),
                b.edges().len(),
                b.faces().len(),
                b.wires().len(),
                b.edge_uses().len(),
            )
        })
        .collect::<Vec<_>>();
    measure(name, iterations, "certified", || {
        let result = write_advanced(black_box(inputs), &options, &acceptance::context())
            .result
            .unwrap()
            .unwrap();
        assert_eq!(result.bodies().len(), inputs.len());
        for (body, counts) in result.bodies().iter().zip(&counts) {
            assert_eq!(
                (
                    body.vertices().len(),
                    body.edges().len(),
                    body.faces().len(),
                    body.wires().len(),
                    body.edge_uses().len()
                ),
                *counts
            );
            assert!(body.edges().iter().all(|e| e.source.uses().count() == 2));
        }
        assert_eq!(result.text(), expected);
        digest(result.into_text().as_bytes())
    });
}
fn main() {
    let native = builder::cylinder(5.into(), 7.into()).unwrap();
    let input = StepBody {
        solid: native.bodies().next().unwrap(),
        name: "native",
    };
    export("analytic_export/native_cylinder", &[input], 500);
    export("analytic_export/32_bodies", &[input; 32], 15);
    let partial = cylinder::partitioned();
    for (name, text, n) in [
        ("analytic_export/closed_imported", cylinder::CYLINDER, 500),
        ("analytic_export/partial_imported", partial.as_str(), 300),
        (
            "analytic_export/radical_imported",
            include_str!("../tests/fixtures/radical-frame-cylinder.step"),
            200,
        ),
    ] {
        let imported = cylinder::import(text).unwrap();
        export(
            name,
            &[StepBody {
                solid: imported.bodies()[0].solid(),
                name: "imported",
            }],
            n,
        );
    }
    let profile = |points: &[(i32, i32)]| {
        points
            .iter()
            .map(|&(x, y)| hyperbrep::Point2::new(x.into(), y.into()))
            .collect::<Vec<_>>()
    };
    let hole = builder::extrude_region(
        &profile(&[(0, 0), (4, 0), (4, 3), (0, 3)]),
        &[profile(&[(1, 1), (3, 1), (3, 2), (1, 2)])],
        Real::zero(),
        2.into(),
    )
    .unwrap();
    export(
        "analytic_export/planar_hole",
        &[StepBody {
            solid: hole.bodies().next().unwrap(),
            name: "hole",
        }],
        300,
    );
    measure("analytic_export/edit_roundtrip", 20, "certified", || {
        acceptance::frames_and_edits();
        1
    });
    let third = builder::cylinder((Real::one() / Real::from(3)).unwrap(), Real::one()).unwrap();
    measure(
        "analytic_export/reject_third",
        500,
        "unsupported representation",
        || {
            assert!(matches!(
                write_advanced(
                    &[StepBody {
                        solid: third.bodies().next().unwrap(),
                        name: "third"
                    }],
                    &StepWriteOptions::new("third.step", "2026-10-16T00:00:00"),
                    &acceptance::context()
                )
                .result
                .unwrap()
                .unwrap_err()
                .kind,
                WriteErrorKind::NonTerminatingDecimal
            ));
            1
        },
    );
}
