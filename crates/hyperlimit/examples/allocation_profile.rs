//! Allocation-event diagnostics for warmed Hyperlimit predicate calls.
//!
//! From the workspace root, run with an optional positive iteration count:
//!
//! ```text
//! cargo run -p hyperlimit --release --all-features --example allocation_profile -- 256
//! ```
//!
//! Builds fixtures and performs one warm-up per row before enabling counters.
//! Each measured call includes destruction of its returned value, including
//! report and batch output allocations. Batch rates are per whole batch.
//!
//! Counts requested sizes through a System allocator wrapper, process-wide,
//! including worker activity. Pre-window allocations are not inventoried, while
//! their later frees are counted and the live balance saturates at zero. Peak
//! and retained columns are window estimates, not complete-lifetime memory,
//! allocator overhead, resident memory, or transient realloc storage. Concurrent
//! unrelated activity can affect results; snapshots load counters independently.
//! Uses APPROXIMATE_512 without validating outcomes, so this is not correctness
//! or exact-completion evidence. No timing or resource limit is measured here.

use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use hyperlimit::{
    Plane3, Point2, Point3, PredicatePolicy, classify_point_aabb3,
    classify_segment_triangle3_intersection_report, classify_segment3_intersection, incircle2,
    insphere3, intersect_three_planes, orient2, orient2_batch, orient3,
};
use hyperreal::Real;

/// Allow terminal approximation in measured predicates; outcomes are consumed but not validated.
const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
/// Number of measured calls per row when no positional iteration count is supplied.
const DEFAULT_ITERATIONS: usize = 256;

/// System allocator wrapper counting successful allocation/reallocation events while enabled.
///
/// Counters are process-global, not scoped to the calling thread or allocation
/// ownership. Deallocations during the window include blocks created before it;
/// the saturating live counter is therefore an observation-window estimate, not
/// an exact allocation-lifetime or process-resident-memory measurement.
struct CountingAllocator;

/// Install the counting System wrapper for this example process, including worker-thread allocations.
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

/// Global switch sampled by allocator hooks; enabled windows must not overlap or nest.
static ENABLED: AtomicBool = AtomicBool::new(false);
/// Number of successful alloc calls observed while counting is enabled.
static ALLOC_CALLS: AtomicUsize = AtomicUsize::new(0);
/// Number of dealloc calls observed while enabled, regardless of when the block was allocated.
static DEALLOC_CALLS: AtomicUsize = AtomicUsize::new(0);
/// Number of successful realloc calls observed while counting is enabled.
static REALLOC_CALLS: AtomicUsize = AtomicUsize::new(0);
/// Requested bytes from successful alloc calls plus full new sizes from successful realloc calls.
static ALLOCATED_BYTES: AtomicUsize = AtomicUsize::new(0);
/// Layout bytes passed to dealloc plus full old sizes from successful realloc calls.
static DEALLOCATED_BYTES: AtomicUsize = AtomicUsize::new(0);
/// Window-relative requested-byte balance, starting at zero and saturating on subtraction.
static LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);
/// Largest observed window-relative live balance; excludes allocator overhead and transient realloc storage.
static PEAK_LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    /// Delegate to System and count a nonnull allocation if counting is enabled afterward.
    ///
    /// Updates requested bytes and the live/peak estimate; failed allocations are not
    /// counted. Relaxed counters do not form an atomic multi-counter snapshot.
    ///
    /// # Safety
    ///
    /// The caller must satisfy [`GlobalAlloc::alloc`]'s layout contract. The same layout
    /// is forwarded to System; the returned pointer retains System's ownership rules.
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() && ENABLED.load(Ordering::Relaxed) {
            ALLOC_CALLS.fetch_add(1, Ordering::Relaxed);
            ALLOCATED_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            add_live_bytes(layout.size());
        }
        pointer
    }

    /// Count the supplied block size if enabled, then release the block through System.
    ///
    /// Subtraction saturates even for blocks allocated before the counting window.
    ///
    /// # Safety
    ///
    /// The pointer and layout must satisfy [`GlobalAlloc::dealloc`]'s allocation-origin,
    /// layout, and liveness requirements for this System-backed allocator.
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        if ENABLED.load(Ordering::Relaxed) {
            DEALLOC_CALLS.fetch_add(1, Ordering::Relaxed);
            DEALLOCATED_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            subtract_live_bytes(layout.size());
        }
        unsafe { System.dealloc(pointer, layout) };
    }

    /// Delegate resizing to System and count only a nonnull replacement while enabled.
    ///
    /// Records the full old/new sizes as byte traffic, adjusting live balance by their
    /// difference. Does not observe any simultaneous old/new storage inside System.
    ///
    /// # Safety
    ///
    /// The pointer, layout, and new size must satisfy [`GlobalAlloc::realloc`]'s contract.
    /// After success the returned pointer, rather than the old allocation, owns the block.
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

/// Add requested bytes and raise the estimated peak with a relaxed compare/exchange loop.
///
/// The individual counters are atomic, but the combined observation is not a
/// transaction. Addition has no explicit overflow guard; counts must fit usize.
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

/// Saturating-subtract observed released bytes with a relaxed compare/exchange loop.
///
/// Avoids unsigned underflow when releases include pre-window allocations, at the
/// cost of losing an exact accounting balance. Retries after concurrent changes.
fn subtract_live_bytes(bytes: usize) {
    // Keep the relaxed saturating update compatible with both the MSRV and
    // toolchains that have deprecated fetch_update in favor of try_update.
    let mut live = LIVE_BYTES.load(Ordering::Relaxed);
    loop {
        match LIVE_BYTES.compare_exchange_weak(
            live,
            live.saturating_sub(bytes),
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(observed) => live = observed,
        }
    }
}

/// Snapshot of allocator events and requested-byte estimates for one observed window.
///
/// Counts exclude fixture construction and the uncounted warm-up when used by
/// measure. Returned operation values are dropped inside the measured window.
/// Live/peak values are not complete-lifetime memory or allocator-overhead metrics.
#[derive(Clone, Copy)]
struct AllocationStats {
    /// Successful allocation calls counted in the window.
    allocations: usize,
    /// Deallocation calls counted in the window, including releases of older blocks.
    deallocations: usize,
    /// Successful resize calls counted in the window.
    reallocations: usize,
    /// Sum of requested allocation sizes and full new resize sizes.
    allocated_bytes: usize,
    /// Sum of deallocation layout sizes and full old resize sizes.
    deallocated_bytes: usize,
    /// Peak observed window-relative requested-byte balance.
    peak_live_bytes: usize,
    /// Remaining saturating window-relative balance at snapshot time.
    live_bytes: usize,
}

impl AllocationStats {
    /// Load each relaxed counter independently into a value record.
    ///
    /// Concurrent allocator events can produce a mixed-time snapshot; callers need
    /// quiescent relevant work for a consistent end-of-window reading.
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

/// Disable the global counting switch on drop, including ordinary unwinding.
///
/// Does not own or isolate allocations. Guards must not overlap or nest, and
/// counted worker work must finish before taking its final snapshot.
struct CountingGuard;

impl CountingGuard {
    /// Zero all counters, then enable counting globally.
    ///
    /// Requires no other active measurement or concurrent reset. Previously allocated
    /// blocks are not inventoried, and in-flight allocator calls are not synchronized
    /// with the reset; this is a diagnostic window, not a lifetime accounting barrier.
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
    /// Disable counting globally without resetting counters or waiting for outstanding allocator activity.
    fn drop(&mut self) {
        ENABLED.store(false, Ordering::SeqCst);
    }
}

/// Warm a closure once, then count iterations of calls and returned-value destruction.
///
/// Black-boxes each result and drops it while counting is enabled. Takes a snapshot
/// before dropping the guard. Fixture construction and the warm-up are uncounted;
/// other process allocation activity can still affect the counters. Requires an
/// exclusive measurement window and completion of relevant workers before return.
/// Does not assert successful geometry or certainty and allows zero measured calls.
/// If the closure panics, ordinary unwinding drops the guard but returns no stats.
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

/// Build a planar profiling fixture from exact integer coordinates before measurement.
fn exact_point2(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

/// Build a spatial profiling fixture from exact integer coordinates before measurement.
fn exact_point3(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

/// Construct pi plus an exact integer to exercise shared symbolic offsets.
fn symbolic(value: i32) -> Real {
    Real::pi() + Real::from(value)
}

/// Build a planar profiling fixture translated by (pi, pi).
fn symbolic_point2(x: i32, y: i32) -> Point2 {
    Point2::new(symbolic(x), symbolic(y))
}

/// Print per-call allocation/byte/reallocation rates and whole-window peak/remaining balance.
///
/// Requires a positive iteration count for meaningful rates. Converts counters to
/// f64 for display only; deallocation counts/bytes are consumed but not printed.
/// The displayed retained-byte value is the estimated window balance, not a
/// measurement of all live allocations owned by the predicate.
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

/// Profile ten warmed predicate/batch rows and print a Markdown table; parallel adds an eleventh.
///
/// Accepts an optional positive usize iteration count (default 256), ignoring later
/// arguments. Builds integer/symbolic fixtures and the 512-element batch before
/// measurement. Uses APPROXIMATE_512 and does not inspect result validity or receipts.
/// Each batch counts as one operation in the displayed rates, not 512 operations.
///
/// # Panics
///
/// Panics for an unparseable or zero iteration count; measured operations may also
/// panic. Printed figures alone do not establish correctness or resource compliance.
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
