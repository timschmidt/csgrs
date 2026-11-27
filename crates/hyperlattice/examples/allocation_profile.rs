//! Allocator-call observations for four carrier operations across 22 labeled scalar recipes.
//!
//! The labels name intended representation recipes; only the list length is asserted,
//! not private tags or exhaustive class coverage. Each row constructs fixtures and
//! runs one warm-up before resetting counters. Counted iterations include result
//! destruction. One warm-up does not establish steady state, and later cache growth
//! can remain visible. This example performs no independent geometry/certainty check.
//!
//! The process-global System wrapper counts successful allocation/reallocation calls
//! and observed frees while enabled, including other threads. Requested-byte traffic
//! adds layout sizes and full new realloc sizes, not allocator overhead or RSS.
//! Live balance starts at zero and saturates on subtraction; a free of pre-window
//! storage can erase balance rather than record a negative delta. Peaks omit any
//! simultaneous old/new storage inside System::realloc. These are window-relative
//! bookkeeping counters, not complete-lifetime retained/temporary memory evidence.
//!
//! Counters use usize and unchecked growth; snapshots load them independently with
//! relaxed ordering. Overlapping/nested windows and concurrent reset/event attribution
//! are not supported measurement protocols. No work, precision, memory, or execution-
//! time limit is enforced. Default iterations are 128; supply one positive usize as
//! the first argument. The shell wrapper selects a release/all-feature build, while
//! this source can also be compiled separately for bounded instrumentation probes.

use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use hyperlattice::{Complex, Matrix3, Matrix4, Point3, Rational, Real, Vector3};

/// Fallback counted iterations per row when no first command-line argument is supplied; excludes the additional warm-up call.
const DEFAULT_ITERATIONS: usize = 128;

/// System allocator wrapper that records process-global events during an enabled observation window.
///
/// Does not identify allocation ownership, inventory pre-existing storage, track
/// System internals, or enforce a memory budget. Its counters are instrumentation,
/// not a replacement allocation/error policy for lattice consumers.
struct CountingAllocator;

/// Example-process global allocator; delegates storage operations to System while optionally observing events.
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

/// Process-global event gate, initially false; relaxed hook reads are not an atomic reset/snapshot boundary across threads.
static ENABLED: AtomicBool = AtomicBool::new(false);
/// Number of successful alloc calls observed while enabled; realloc calls have a separate counter.
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
/// Observed dealloc calls while enabled, including frees whose allocation predates the window.
static DEALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
/// Successful realloc calls observed while enabled; failed realloc attempts do not increment this counter.
static REALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
/// Sum of successful allocation layout sizes and full successful realloc new sizes; not net growth or OS-resident bytes.
static ALLOCATED_BYTES: AtomicUsize = AtomicUsize::new(0);
/// Zero-based window balance with saturating subtraction and unchecked additions; omits initial live storage and cannot represent negative deltas.
static LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);
/// Largest observed LIVE_BYTES candidate during this window; excludes allocator-internal temporary/overhead storage.
static PEAK_LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    /// Delegate allocation to System, then count a nonnull result if the gate is enabled.
    ///
    /// Records one call, layout.size requested bytes, and the same live-balance increase.
    /// The enabled test happens after allocation. Failed allocation is returned unchanged.
    ///
    /// # Safety
    ///
    /// The caller must satisfy GlobalAlloc::alloc's layout/allocation requirements;
    /// this wrapper adds no validation and retains System's allocation contract.
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() && ENABLED.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
            ALLOCATED_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            add_live_bytes(layout.size());
        }
        pointer
    }

    /// Count a gated free and saturating live-balance decrease before delegating to System.
    ///
    /// No allocation-origin check occurs, so freeing pre-window storage is also counted.
    ///
    /// # Safety
    ///
    /// The pointer/layout must satisfy GlobalAlloc::dealloc and identify a currently
    /// allocated System block with its matching layout; the wrapper does not check them.
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        if ENABLED.load(Ordering::Relaxed) {
            DEALLOCATIONS.fetch_add(1, Ordering::Relaxed);
            subtract_live_bytes(layout.size());
        }
        unsafe { System.dealloc(pointer, layout) };
    }

    /// Delegate resizing to System, then count a nonnull result if the gate is enabled.
    ///
    /// Adds the full new_size to requested-byte traffic, but adjusts live balance only
    /// by the signed size difference (saturating on shrink). Does not observe whether
    /// System moved the block or temporarily retained both allocations. Failure is
    /// returned without changing counters for this attempt.
    ///
    /// # Safety
    ///
    /// The pointer, old layout, and new size must satisfy GlobalAlloc::realloc's
    /// requirements for a live System allocation; no local validation is performed.
    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let replacement = unsafe { System.realloc(pointer, layout, new_size) };
        if !replacement.is_null() && ENABLED.load(Ordering::Relaxed) {
            REALLOCATIONS.fetch_add(1, Ordering::Relaxed);
            ALLOCATED_BYTES.fetch_add(new_size, Ordering::Relaxed);
            if new_size >= layout.size() {
                add_live_bytes(new_size - layout.size());
            } else {
                subtract_live_bytes(layout.size() - new_size);
            }
        }
        replacement
    }
}

/// Add bytes with a relaxed fetch-add, then raise the peak using a weak compare/exchange loop.
///
/// The returned balance plus bytes is the peak candidate. Counters have no overflow
/// protection or total-memory semantics; concurrent events/resets can affect the
/// observed window. This helper performs no allocation itself.
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

/// Subtract bytes with a relaxed saturating compare/exchange loop, retrying on contention.
///
/// A free larger than the current balance records zero, losing any negative delta.
/// This accommodates untracked pre-window allocations but is not exact ownership
/// accounting or proof that no retained storage remains.
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

/// Copyable snapshot of six independently loaded window counters.
///
/// This packet is not an atomic cross-counter snapshot or a complete-lifetime memory
/// receipt. Counts/sizes may wrap at usize limits, and live subtraction saturates.
#[derive(Clone, Copy)]
struct AllocationStats {
    /// Successful gated allocation-call count, excluding reallocations.
    allocations: usize,
    /// Gated free-call count, regardless of the allocation window of origin.
    deallocations: usize,
    /// Successful gated resize-call count, excluding failed attempts.
    reallocations: usize,
    /// Requested-byte traffic including each realloc full new size; not net or resident memory.
    allocated_bytes: usize,
    /// Peak of the zero-based live bookkeeping balance across the whole counted window.
    peak_live_bytes: usize,
    /// End-of-window saturating live bookkeeping balance; zero does not prove no retained memory.
    live_bytes: usize,
}

impl AllocationStats {
    /// Load the six counters independently with relaxed ordering in field order; does not disable counting or synchronize concurrent events.
    fn snapshot() -> Self {
        Self {
            allocations: ALLOCATIONS.load(Ordering::Relaxed),
            deallocations: DEALLOCATIONS.load(Ordering::Relaxed),
            reallocations: REALLOCATIONS.load(Ordering::Relaxed),
            allocated_bytes: ALLOCATED_BYTES.load(Ordering::Relaxed),
            peak_live_bytes: PEAK_LIVE_BYTES.load(Ordering::Relaxed),
            live_bytes: LIVE_BYTES.load(Ordering::Relaxed),
        }
    }
}

/// RAII gate for one intended non-overlapping observation window.
///
/// Start resets global state; Drop disables it rather than restoring an earlier
/// state. Nested/overlapping guards interfere, and in-flight allocator operations
/// are not quiesced. Ordinary unwinding after start drops the guard.
struct CountingGuard;

impl CountingGuard {
    /// Reset every counter with relaxed stores, then enable observation with a SeqCst store.
    ///
    /// Does not check that another window is inactive or inventory live allocations.
    /// An event racing reset/enable need not belong cleanly to this intended window.
    fn start() -> Self {
        for counter in [
            &ALLOCATIONS,
            &DEALLOCATIONS,
            &REALLOCATIONS,
            &ALLOCATED_BYTES,
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
    /// Disable the global gate with a SeqCst store, without taking a snapshot or restoring an outer guard state.
    fn drop(&mut self) {
        ENABLED.store(false, Ordering::SeqCst);
    }
}

/// Warm once, then count iterations calls, black-box and drop each result, and snapshot before disabling.
///
/// Fixtures already held by the closure are not inventoried. Result destruction is
/// inside the window; the closure/environment is destroyed after the explicit guard
/// drop. One warm-up is not a steady-state guarantee. A panic produces no returned
/// snapshot, with guard cleanup during ordinary unwinding after start. Zero iterations
/// still warms once and returns reset counters; main separately requires positivity.
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

/// Print per-iteration call/traffic averages and undivided peak/end window balances.
///
/// Converts counts and iterations to f64 for formatted averages (two decimal places
/// for calls, one for bytes). Peak/end columns are integer totals over the window,
/// not per-operation measurements. The end-live-delta label denotes a nonnegative
/// saturating balance, not a signed lifetime delta. Requires positive iterations for
/// meaningful averages; main enforces this, but this helper does not.
fn print_row(name: &str, operation: &str, iterations: usize, stats: AllocationStats) {
    let divisor = iterations as f64;
    println!(
        "| {name} | {operation} | {:.2} | {:.2} | {:.1} | {:.2} | {} | {} |",
        stats.allocations as f64 / divisor,
        stats.deallocations as f64 / divisor,
        stats.allocated_bytes as f64 / divisor,
        stats.reallocations as f64 / divisor,
        stats.peak_live_bytes,
        stats.live_bytes,
    );
}

/// Construct an exact rational fixture; expect panics on an invalid denominator such as zero.
fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

/// Build 22 labeled finite scalar recipes for the four carrier workloads.
///
/// Includes rational, pi forms, constant products/offsets, roots, exponentials/logs,
/// rational-angle trig, and opaque sin(1). Labels are descriptive rather than
/// validated private class tags; One names the rational representation recipe 3/2,
/// not the numerical value one. Fixed-domain expects panic on construction failure.
/// This function does not establish exhaustive symbolic/algebraic coverage.
fn representation_values() -> Vec<(&'static str, Real)> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().expect("positive radicand");
    let ln_two = Real::from(2).ln().expect("positive logarithm input");
    let ln_three = Real::from(3).ln().expect("positive logarithm input");

    vec![
        ("One", fraction(3, 2)),
        ("Pi", pi.clone()),
        ("PiPow", pi_squared.clone()),
        ("PiInv", pi.clone().inverse().expect("pi is nonzero")),
        ("PiExp", &pi * &e),
        ("PiInvExp", (&e / &pi).expect("pi is nonzero")),
        ("PiSqrt", &pi * &sqrt_two),
        ("ConstProduct", &pi_squared * &e),
        ("ConstOffset", &pi - Real::from(3)),
        ("ConstProductSqrt", &(&pi_squared * &e) * &sqrt_two),
        ("Sqrt", sqrt_two),
        ("Exp", Real::from(2).exp().expect("finite exponential")),
        ("Ln", ln_three.clone()),
        (
            "LnAffine",
            (Real::from(2) * &e).ln().expect("positive logarithm input"),
        ),
        ("LnProduct", &ln_two * &ln_three),
        ("Log10", Real::from(2).log10().expect("positive input")),
        ("Log2", Real::from(3).log2().expect("positive input")),
        (
            "Pow10",
            fraction(1, 7)
                .exp10()
                .expect("finite rational base-ten power"),
        ),
        (
            "Pow2",
            fraction(1, 7)
                .exp2()
                .expect("finite rational base-two power"),
        ),
        ("SinPi", fraction(1, 5).sin_pi()),
        (
            "TanPi",
            fraction(1, 5)
                .tan_pi()
                .expect("one fifth of a turn is not a tangent pole"),
        ),
        ("Irrational", Real::one().sin()),
    ]
}

/// Parse the first optional positive usize, construct fixtures, and print 88 allocation rows.
///
/// Missing input defaults to 128; invalid parsing or zero panics, and additional
/// arguments are ignored. Type-size output is inline Rust layout size, excluding
/// heap referents. For each recipe, measures complex self-product, a tuple of vector
/// self-dot/self-cross, Matrix3 determinant, and affine Matrix4 point transformation.
/// The tuple is one counted operation, not separate dot/cross rows. Inputs are built
/// outside their windows. The transform must return Ok; no independent exact output,
/// private representation, topology, or certainty assertion qualifies the measurements.
fn main() {
    let iterations = std::env::args()
        .nth(1)
        .map(|value| value.parse().expect("iteration count must be an integer"))
        .unwrap_or(DEFAULT_ITERATIONS);
    assert!(iterations > 0, "iteration count must be positive");

    let values = representation_values();
    assert_eq!(values.len(), 22, "update the Real allocation corpus");
    println!("Hyperlattice allocation profile ({iterations} iterations per row)\n");
    println!(
        "Type sizes: Real={} Complex={} Vector3={} Matrix3={} Matrix4={} bytes\n",
        size_of::<Real>(),
        size_of::<Complex>(),
        size_of::<Vector3>(),
        size_of::<Matrix3>(),
        size_of::<Matrix4>(),
    );
    println!(
        "| Certificate | Operation | allocs/op | deallocs/op | bytes/op | reallocs/op | peak live bytes | end live delta |"
    );
    println!("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |");

    for (name, value) in values {
        let complex = Complex::new(value.clone(), Real::one());
        print_row(
            name,
            "complex multiply",
            iterations,
            measure(iterations, || &complex * &complex),
        );

        let vector = Vector3::new([value.clone(), Real::one(), Real::from(2)]);
        print_row(
            name,
            "vector dot/cross",
            iterations,
            measure(iterations, || (vector.dot(&vector), vector.cross(&vector))),
        );

        let matrix3 = Matrix3::new([
            [value.clone(), Real::one(), Real::zero()],
            [Real::zero(), Real::from(2), Real::one()],
            [Real::one(), Real::zero(), Real::from(3)],
        ]);
        print_row(
            name,
            "matrix3 determinant",
            iterations,
            measure(iterations, || matrix3.determinant()),
        );

        let matrix4 = Matrix4::affine_translation([value.clone(), Real::from(2), Real::from(3)]);
        let point = Point3::new(value, Real::one(), Real::from(2));
        print_row(
            name,
            "matrix4 point transform",
            iterations,
            measure(iterations, || {
                matrix4
                    .transform_point3(&point)
                    .expect("affine transform remains finite")
            }),
        );
    }
}
