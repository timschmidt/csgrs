//! Steady-state allocation profile for representative Hyperlimit predicates.
//!
//! Run with:
//!
//! ```text
//! cargo run --release --all-features --example allocation_profile
//! ```
//!
//! Inputs and one warm-up query are created before counting starts. The table
//! therefore measures query-time allocations, including returned reports and
//! batch output vectors, rather than benchmark-fixture construction.

use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use hyperlimit::{
    Plane3, Point2, Point3, PredicatePolicy, classify_point_aabb3,
    classify_segment_triangle3_intersection_report, classify_segment3_intersection, incircle2,
    insphere3, intersect_three_planes, orient2, orient2_batch, orient3,
};
use hyperreal::Real;

const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
const DEFAULT_ITERATIONS: usize = 256;

struct CountingAllocator;

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

static ENABLED: AtomicBool = AtomicBool::new(false);
static ALLOC_CALLS: AtomicUsize = AtomicUsize::new(0);
static DEALLOC_CALLS: AtomicUsize = AtomicUsize::new(0);
static REALLOC_CALLS: AtomicUsize = AtomicUsize::new(0);
static ALLOCATED_BYTES: AtomicUsize = AtomicUsize::new(0);
static DEALLOCATED_BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);
static PEAK_LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() && ENABLED.load(Ordering::Relaxed) {
            ALLOC_CALLS.fetch_add(1, Ordering::Relaxed);
            ALLOCATED_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            add_live_bytes(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        if ENABLED.load(Ordering::Relaxed) {
            DEALLOC_CALLS.fetch_add(1, Ordering::Relaxed);
            DEALLOCATED_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            subtract_live_bytes(layout.size());
        }
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let replacement = unsafe { System.realloc(pointer, layout, new_size) };
        if !replacement.is_null() && ENABLED.load(Ordering::Relaxed) {
            REALLOC_CALLS.fetch_add(1, Ordering::Relaxed);
            ALLOCATED_BYTES.fetch_add(new_size, Ordering::Relaxed);
            DEALLOCATED_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            if new_size >= layout.size() {
                add_live_bytes(new_size - layout.size());
            } else {
                subtract_live_bytes(layout.size() - new_size);
            }
        }
        replacement
    }
}

fn add_live_bytes(bytes: usize) {
    let live = LIVE_BYTES.fetch_add(bytes, Ordering::Relaxed) + bytes;
    let mut peak = PEAK_LIVE_BYTES.load(Ordering::Relaxed);
    while live > peak {
        match PEAK_LIVE_BYTES.compare_exchange_weak(
            peak,
            live,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(observed) => peak = observed,
        }
    }
}

fn subtract_live_bytes(bytes: usize) {
    let _ = LIVE_BYTES.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |live| {
        Some(live.saturating_sub(bytes))
    });
}

#[derive(Clone, Copy)]
struct AllocationStats {
    allocations: usize,
    deallocations: usize,
    reallocations: usize,
    allocated_bytes: usize,
    deallocated_bytes: usize,
    peak_live_bytes: usize,
    live_bytes: usize,
}

impl AllocationStats {
    fn snapshot() -> Self {
        Self {
            allocations: ALLOC_CALLS.load(Ordering::Relaxed),
            deallocations: DEALLOC_CALLS.load(Ordering::Relaxed),
            reallocations: REALLOC_CALLS.load(Ordering::Relaxed),
            allocated_bytes: ALLOCATED_BYTES.load(Ordering::Relaxed),
            deallocated_bytes: DEALLOCATED_BYTES.load(Ordering::Relaxed),
            peak_live_bytes: PEAK_LIVE_BYTES.load(Ordering::Relaxed),
            live_bytes: LIVE_BYTES.load(Ordering::Relaxed),
        }
    }
}

struct CountingGuard;

impl CountingGuard {
    fn start() -> Self {
        for counter in [
            &ALLOC_CALLS,
            &DEALLOC_CALLS,
            &REALLOC_CALLS,
            &ALLOCATED_BYTES,
            &DEALLOCATED_BYTES,
            &LIVE_BYTES,
            &PEAK_LIVE_BYTES,
        ] {
            counter.store(0, Ordering::Relaxed);
        }
        ENABLED.store(true, Ordering::SeqCst);
        Self
    }
}

impl Drop for CountingGuard {
    fn drop(&mut self) {
        ENABLED.store(false, Ordering::SeqCst);
    }
}

fn measure<T>(iterations: usize, mut operation: impl FnMut() -> T) -> AllocationStats {
    drop(black_box(operation()));

    let guard = CountingGuard::start();
    for _ in 0..iterations {
        drop(black_box(operation()));
    }
    let stats = AllocationStats::snapshot();
    drop(guard);
    stats
}

fn exact_point2(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn exact_point3(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn symbolic(value: i32) -> Real {
    Real::pi() + Real::from(value)
}

fn symbolic_point2(x: i32, y: i32) -> Point2 {
    Point2::new(symbolic(x), symbolic(y))
}

fn print_row(name: &str, iterations: usize, stats: AllocationStats) {
    let divisor = iterations as f64;
    println!(
        "| {name} | {:.2} | {:.1} | {:.2} | {} | {} |",
        stats.allocations as f64 / divisor,
        stats.allocated_bytes as f64 / divisor,
        stats.reallocations as f64 / divisor,
        stats.peak_live_bytes,
        stats.live_bytes,
    );
    black_box((stats.deallocations, stats.deallocated_bytes));
}

fn main() {
    let iterations = std::env::args()
        .nth(1)
        .map(|value| {
            value
                .parse::<usize>()
                .expect("iteration count must be a positive integer")
        })
        .unwrap_or(DEFAULT_ITERATIONS);
    assert!(iterations > 0, "iteration count must be positive");

    let a2 = exact_point2(0, 0);
    let b2 = exact_point2(5, 0);
    let c2 = exact_point2(1, 3);
    let d2 = exact_point2(2, 1);
    let a3 = exact_point3(0, 0, 0);
    let b3 = exact_point3(4, 0, 0);
    let c3 = exact_point3(0, 4, 0);
    let d3 = exact_point3(0, 0, 4);
    let e3 = exact_point3(1, 1, 1);
    let segment_a = exact_point3(1, 1, -1);
    let segment_b = exact_point3(1, 1, 1);

    let symbolic_a = symbolic_point2(0, 0);
    let symbolic_b = symbolic_point2(5, 0);
    let symbolic_c = symbolic_point2(1, 3);

    let x_plane = Plane3::new(exact_point3(1, 0, 0), Real::from(-2));
    let y_plane = Plane3::new(exact_point3(0, 1, 0), Real::from(-3));
    let z_plane = Plane3::new(exact_point3(0, 0, 1), Real::from(-4));

    let batch: Vec<_> = (0_i32..512)
        .map(|index| {
            let y = index % 17 - 8;
            (
                exact_point2(-10, -10),
                exact_point2(10, 10),
                exact_point2(index % 19 - 9, y),
            )
        })
        .collect();

    let rows = vec![
        (
            "orient2/exact-rational",
            measure(iterations, || orient2(&a2, &b2, &c2, POLICY)),
        ),
        (
            "orient2/symbolic-pi-offset",
            measure(iterations, || {
                orient2(&symbolic_a, &symbolic_b, &symbolic_c, POLICY)
            }),
        ),
        (
            "orient3/exact-rational",
            measure(iterations, || orient3(&a3, &b3, &c3, &d3, POLICY)),
        ),
        (
            "incircle2/exact-rational",
            measure(iterations, || incircle2(&a2, &b2, &c2, &d2, POLICY)),
        ),
        (
            "insphere3/exact-rational",
            measure(iterations, || insphere3(&a3, &b3, &c3, &d3, &e3, POLICY)),
        ),
        (
            "point-aabb3/exact-rational",
            measure(iterations, || classify_point_aabb3(&a3, &d3, &e3, POLICY)),
        ),
        (
            "segment3/exact-rational",
            measure(iterations, || {
                classify_segment3_intersection(&a3, &d3, &segment_a, &segment_b, POLICY)
            }),
        ),
        (
            "segment-triangle3/report",
            measure(iterations, || {
                classify_segment_triangle3_intersection_report(
                    &segment_a, &segment_b, &a3, &b3, &c3, POLICY,
                )
            }),
        ),
        (
            "three-plane/intersection",
            measure(iterations, || {
                intersect_three_planes(&x_plane, &y_plane, &z_plane)
            }),
        ),
        (
            "orient2-batch/512",
            measure(iterations, || orient2_batch(&batch, POLICY)),
        ),
    ];
    #[cfg(feature = "parallel")]
    let rows = {
        let mut rows = rows;
        rows.push((
            "orient2-batch-parallel/512",
            measure(iterations, || {
                hyperlimit::orient2_batch_parallel(&batch, POLICY)
            }),
        ));
        rows
    };

    println!("# Hyperlimit steady-state allocation profile\n");
    println!("Iterations per row: {iterations}\n");
    println!(
        "| Query | allocations/op | bytes/op | reallocations/op | peak live bytes | retained bytes |"
    );
    println!("|---|---:|---:|---:|---:|---:|");
    for (name, stats) in rows {
        print_row(name, iterations, stats);
    }
}
