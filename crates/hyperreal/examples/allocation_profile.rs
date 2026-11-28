//! Counting-allocator profile of 20 fixed exact-real construction recipes.
//!
//! From the workspace root, use a release build for allocation measurements:
//!
//! ```text
//! cargo run -p hyperreal --release --example allocation_profile -- 64
//! ```
//!
//! The optional positive `u64` argument defaults to 64 lifecycles per recipe.
//! Rows contain totals, not per-iteration averages. Recipe names describe intended
//! scalar forms; this executable neither asserts their actual dispatch tags nor
//! covers every certificate family (for example, Pow2 and Pow10 have no recipes).
//!
//! Each recipe first warms construction, a certified interval at precision -128,
//! and a lossy float view. The measured loop additionally queries detailed facts
//! and constructs an inverse. Those extra operations may populate shared caches;
//! retained bytes are a process-wide net change, not necessarily a leak or bytes
//! owned by the dropped value. Earlier rows also influence later cache state.
//!
//! The allocator tracks requested Rust allocation sizes, not allocator overhead,
//! resident memory, stack use or allocations bypassing the global allocator.
//! Successful reallocations count as both a deallocation and an allocation, even
//! in place; their internal simultaneous storage is not captured by the peak.
//! Relaxed counters are independent observations, not a coherent concurrent
//! snapshot. Finite counters assume no overflow. The iteration count and numeric
//! precision are not total-work, memory or cancellation bounds. This is a
//! measurement tool, not a mathematical accuracy or representation-coverage test.

use std::{
    alloc::{GlobalAlloc, Layout, System},
    hint::black_box,
    io::{self, Write},
    sync::atomic::{AtomicU64, Ordering},
};

use hyperreal::{Rational, Real};

/// Forward Rust global allocations to the system allocator while recording requested sizes.
/// Instrumentation does not allocate; callers retain the GlobalAlloc layout and pointer
/// obligations. Failed allocation/reallocation attempts leave the counters unchanged.
struct CountingAllocator;

/// Successful allocation events, including the replacement side of successful reallocations.
static ALLOCATIONS: AtomicU64 = AtomicU64::new(0);
/// Deallocation events, including the old block of successful reallocations.
static DEALLOCATIONS: AtomicU64 = AtomicU64::new(0);
/// Successful reallocations, whether or not the pointer changes.
static REALLOCATIONS: AtomicU64 = AtomicU64::new(0);
/// Cumulative requested bytes for successful allocations and replacement blocks.
static ALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);
/// Cumulative layout bytes released, including replaced blocks.
static DEALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);
/// Requested bytes currently accounted as live; excludes allocator bookkeeping.
static LIVE_BYTES: AtomicU64 = AtomicU64::new(0);
/// Maximum observed live-byte count since the current measurement reset.
static PEAK_LIVE_BYTES: AtomicU64 = AtomicU64::new(0);

/// Process-wide allocator used by this executable, including its output and runtime support.
#[global_allocator]
static GLOBAL_ALLOCATOR: CountingAllocator = CountingAllocator;

/// Raise the peak with relaxed compare/exchange until this candidate is covered.
/// Concurrent updates may require retries; there is no fixed retry bound or snapshot lock.
fn update_peak(candidate: u64) {
    let mut peak = PEAK_LIVE_BYTES.load(Ordering::Relaxed);
    while candidate > peak {
        match PEAK_LIVE_BYTES.compare_exchange_weak(
            peak,
            candidate,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(observed) => peak = observed,
        }
    }
}

/// Count one successful allocation, add its requested bytes, and offer the new live total.
/// The size must fit u64 and counters must not overflow; this hook does not allocate.
fn record_allocation(size: usize) {
    let size = u64::try_from(size).expect("allocation size fits u64");
    ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
    ALLOCATED_BYTES.fetch_add(size, Ordering::Relaxed);
    let live = LIVE_BYTES.fetch_add(size, Ordering::Relaxed) + size;
    update_peak(live);
}

/// Count a released block and subtract its layout size from live bytes.
/// Requires a matching recorded allocation and representable counters; no pointer is inspected.
fn record_deallocation(size: usize) {
    let size = u64::try_from(size).expect("deallocation size fits u64");
    DEALLOCATIONS.fetch_add(1, Ordering::Relaxed);
    DEALLOCATED_BYTES.fetch_add(size, Ordering::Relaxed);
    LIVE_BYTES.fetch_sub(size, Ordering::Relaxed);
}

/// Delegate allocation and pointer validity to System, instrumenting only successful changes.
/// The unsafe contract is preserved by forwarding each original layout and pointer unchanged.
unsafe impl GlobalAlloc for CountingAllocator {
    /// Forward an allocation request and count its layout size only for a non-null result.
    ///
    /// # Safety
    /// The caller must satisfy GlobalAlloc::alloc layout requirements.
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            record_allocation(layout.size());
        }
        pointer
    }

    /// Forward a zero-initialized allocation request and count only a non-null result.
    ///
    /// # Safety
    /// The caller must satisfy GlobalAlloc::alloc_zeroed layout requirements.
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            record_allocation(layout.size());
        }
        pointer
    }

    /// Subtract the recorded layout bytes, then free the original block through System.
    ///
    /// # Safety
    /// The pointer and layout must identify a live allocation owned by this allocator.
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        record_deallocation(layout.size());
        unsafe { System.dealloc(pointer, layout) };
    }

    /// Forward resizing, leaving accounting unchanged on failure. On success, count a
    /// reallocation and replace the old layout size by the full new requested size.
    /// The sequential accounting cannot observe temporary overlap inside System.
    ///
    /// # Safety
    /// The pointer, old layout and nonzero new size must satisfy GlobalAlloc::realloc.
    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let resized = unsafe { System.realloc(pointer, layout, new_size) };
        if !resized.is_null() {
            REALLOCATIONS.fetch_add(1, Ordering::Relaxed);
            record_deallocation(layout.size());
            record_allocation(new_size);
        }
        resized
    }
}

/// Copyable sample of independently loaded process counters; excludes the separately reset peak.
/// Derived Clone and Copy duplicate numeric observations without sampling again.
#[derive(Clone, Copy)]
struct Snapshot {
    /// Cumulative successful allocation events at this load.
    allocations: u64,
    /// Cumulative deallocation events at this load.
    deallocations: u64,
    /// Cumulative successful resize events at this load.
    reallocations: u64,
    /// Cumulative requested bytes allocated at this load.
    allocated_bytes: u64,
    /// Cumulative layout bytes released at this load.
    deallocated_bytes: u64,
    /// Current requested live-byte total at this load.
    live_bytes: u64,
}

/// Load the six cumulative counters in field order using relaxed atomics.
/// The returned Copy value is suitable for this sequential profiler; another allocating
/// thread could change counters between loads, so it is not an atomic process snapshot.
fn snapshot() -> Snapshot {
    Snapshot {
        allocations: ALLOCATIONS.load(Ordering::Relaxed),
        deallocations: DEALLOCATIONS.load(Ordering::Relaxed),
        reallocations: REALLOCATIONS.load(Ordering::Relaxed),
        allocated_bytes: ALLOCATED_BYTES.load(Ordering::Relaxed),
        deallocated_bytes: DEALLOCATED_BYTES.load(Ordering::Relaxed),
        live_bytes: LIVE_BYTES.load(Ordering::Relaxed),
    }
}

/// Finite corpus of scalar expressions selected for allocation profiling.
/// Names are intended certificate labels, not observed dispatch results. Derived
/// Clone and Copy duplicate only this selector; Real values are constructed separately.
#[derive(Clone, Copy)]
enum Recipe {
    /// The rational 3/2, whose unit basis carries an exact outer rational scale.
    One,
    /// The constant pi.
    Pi,
    /// The product pi*pi.
    PiPow,
    /// The reciprocal 1/pi.
    PiInv,
    /// The product pi*e.
    PiExp,
    /// The quotient e/pi.
    PiInvExp,
    /// The product pi*sqrt(2).
    PiSqrt,
    /// The product pi^2*e.
    ConstProduct,
    /// The positive difference pi-3.
    ConstOffset,
    /// The product pi^2*e*sqrt(2).
    ConstProductSqrt,
    /// The positive square root of two.
    Sqrt,
    /// The exponential exp(2).
    Exp,
    /// The natural logarithm ln(3).
    Ln,
    /// The natural logarithm ln(2*e), mathematically 1+ln(2).
    LnAffine,
    /// The product ln(2)*ln(3).
    LnProduct,
    /// The base-ten logarithm of two.
    Log10,
    /// The base-two logarithm of three.
    Log2,
    /// The sine of pi/5.
    SinPi,
    /// The tangent of pi/5.
    TanPi,
    /// The sine of one radian, used as an opaque computable recipe.
    Irrational,
}

/// The 20 recipes in CSV order, each measured once after its own partial warm-up.
/// Order affects shared-cache state; this list is not an exhaustive Class inventory.
const RECIPES: [Recipe; 20] = [
    Recipe::One,
    Recipe::Pi,
    Recipe::PiPow,
    Recipe::PiInv,
    Recipe::PiExp,
    Recipe::PiInvExp,
    Recipe::PiSqrt,
    Recipe::ConstProduct,
    Recipe::ConstOffset,
    Recipe::ConstProductSqrt,
    Recipe::Sqrt,
    Recipe::Exp,
    Recipe::Ln,
    Recipe::LnAffine,
    Recipe::LnProduct,
    Recipe::Log10,
    Recipe::Log2,
    Recipe::SinPi,
    Recipe::TanPi,
    Recipe::Irrational,
];

/// Map each fixed selector to its CSV label and authored exact scalar expression.
impl Recipe {
    /// Return the fixed CSV label without inspecting or evaluating a Real.
    const fn name(self) -> &'static str {
        match self {
            Self::One => "One",
            Self::Pi => "Pi",
            Self::PiPow => "PiPow",
            Self::PiInv => "PiInv",
            Self::PiExp => "PiExp",
            Self::PiInvExp => "PiInvExp",
            Self::PiSqrt => "PiSqrt",
            Self::ConstProduct => "ConstProduct",
            Self::ConstOffset => "ConstOffset",
            Self::ConstProductSqrt => "ConstProductSqrt",
            Self::Sqrt => "Sqrt",
            Self::Exp => "Exp",
            Self::Ln => "Ln",
            Self::LnAffine => "LnAffine",
            Self::LnProduct => "LnProduct",
            Self::Log10 => "Log10",
            Self::Log2 => "Log2",
            Self::SinPi => "SinPi",
            Self::TanPi => "TanPi",
            Self::Irrational => "Irrational",
        }
    }

    /// Construct the selected finite, nonzero real using exact rational inputs and public
    /// operations. Construction may simplify or populate shared caches; it does not
    /// assert a particular Class tag or force every later evaluation. Panics if an
    /// expected valid domain operation fails; this helper does not accept arbitrary inputs.
    fn real(self) -> Real {
        match self {
            Self::One => Real::new(Rational::fraction(3, 2).unwrap()),
            Self::Pi => Real::pi(),
            Self::PiPow => {
                let pi = Real::pi();
                &pi * &pi
            }
            Self::PiInv => Real::pi().inverse().expect("pi is nonzero"),
            Self::PiExp => Real::pi() * Real::e(),
            Self::PiInvExp => (Real::e() / Real::pi()).expect("pi is nonzero"),
            Self::PiSqrt => Real::pi() * Real::from(2).sqrt().expect("positive radicand"),
            Self::ConstProduct => {
                let pi = Real::pi();
                (&pi * &pi) * Real::e()
            }
            Self::ConstOffset => Real::pi() - Real::from(3),
            Self::ConstProductSqrt => {
                let pi = Real::pi();
                (&pi * &pi) * Real::e() * Real::from(2).sqrt().expect("positive radicand")
            }
            Self::Sqrt => Real::from(2).sqrt().expect("positive radicand"),
            Self::Exp => Real::from(2).exp().expect("finite exponential"),
            Self::Ln => Real::from(3).ln().expect("positive input"),
            Self::LnAffine => (Real::from(2) * Real::e())
                .ln()
                .expect("positive logarithm input"),
            Self::LnProduct => {
                Real::from(2).ln().expect("positive input")
                    * Real::from(3).ln().expect("positive input")
            }
            Self::Log10 => Real::from(2).log10().expect("positive input"),
            Self::Log2 => Real::from(3).log2().expect("positive input"),
            Self::SinPi => Real::new(Rational::fraction(1, 5).unwrap()).sin_pi(),
            Self::TanPi => Real::new(Rational::fraction(1, 5).unwrap())
                .tan_pi()
                .expect("not a tangent pole"),
            Self::Irrational => Real::one().sin(),
        }
    }
}

/// Aggregate allocation changes across all measured lifecycles of one recipe.
/// These process-wide requested-byte metrics do not attribute ownership to a Real.
struct Measurement {
    /// Allocation-event total, including successful reallocations.
    allocations: u64,
    /// Deallocation-event total, including successful reallocations.
    deallocations: u64,
    /// Successful resize total, also included in both event columns.
    reallocations: u64,
    /// Total requested allocation bytes, including full replacement sizes.
    allocated_bytes: u64,
    /// Total released layout bytes, including replaced blocks.
    deallocated_bytes: u64,
    /// Observed peak requested live bytes above the starting total, saturated at zero.
    peak_live_delta_bytes: u64,
    /// Signed final minus initial live bytes; includes shared-cache growth or release.
    retained_bytes: i128,
}

/// Warm construction, an interval at -128, and float conversion, then snapshot and
/// reset the peak. Each iteration constructs a fresh value, queries detailed facts,
/// requires its interval, requests a lossy float and constructs/drops an inverse
/// before dropping the value. No numeric query is made on the returned inverse.
///
/// Warm-up discards the interval Option; measured interval and inverse failures
/// panic. Float Options are discarded throughout. Zero iterations is allowed here
/// (the CLI rejects it). Snapshots include temporary query results but exclude CSV
/// formatting; shared caches can survive every value. No cancellation or total-work
/// limit is installed. Counter subtraction assumes no overflow or concurrent reset.
fn measure(recipe: Recipe, iterations: u64) -> Measurement {
    // Populate shared constants and process-lifetime caches outside the measured
    // window. Each measured value and all of its owned accelerators are dropped.
    let warm = recipe.real();
    black_box(warm.certified_rational_interval(-128));
    black_box(warm.to_f64_lossy());
    drop(warm);

    let before = snapshot();
    PEAK_LIVE_BYTES.store(before.live_bytes, Ordering::Relaxed);

    for _ in 0..iterations {
        let value = recipe.real();
        black_box(value.detailed_facts());
        black_box(
            value
                .certified_rational_interval(-128)
                .expect("finite representative is certifiable"),
        );
        black_box(value.to_f64_lossy());
        black_box(value.inverse_ref().expect("representative is nonzero"));
        drop(value);
    }

    let after = snapshot();
    Measurement {
        allocations: after.allocations - before.allocations,
        deallocations: after.deallocations - before.deallocations,
        reallocations: after.reallocations - before.reallocations,
        allocated_bytes: after.allocated_bytes - before.allocated_bytes,
        deallocated_bytes: after.deallocated_bytes - before.deallocated_bytes,
        peak_live_delta_bytes: PEAK_LIVE_BYTES
            .load(Ordering::Relaxed)
            .saturating_sub(before.live_bytes),
        retained_bytes: i128::from(after.live_bytes) - i128::from(before.live_bytes),
    }
}

/// Parse the first argument as a positive u64 lifecycle count (default 64), ignore
/// later arguments, and emit a header plus one CSV row per recipe. The length check
/// requires 20 recipes but does not verify certificate coverage. Invalid/zero counts,
/// failed expected scalar operations, or stdout write errors panic. Rows preserve
/// corpus order and report totals across iterations, with measurements excluding
/// their own CSV writes.
fn main() {
    let iterations = std::env::args()
        .nth(1)
        .map(|text| text.parse::<u64>().expect("iteration count must be u64"))
        .unwrap_or(64);
    assert!(iterations > 0, "iteration count must be positive");
    assert_eq!(RECIPES.len(), 20, "update every representation profile");

    let stdout = io::stdout();
    let mut output = stdout.lock();
    writeln!(
        output,
        "representation,iterations,allocations,deallocations,reallocations,allocated_bytes,deallocated_bytes,peak_live_delta_bytes,retained_bytes"
    )
    .expect("write profile heading");

    for recipe in RECIPES {
        let result = measure(recipe, iterations);
        writeln!(
            output,
            "{},{},{},{},{},{},{},{},{}",
            recipe.name(),
            iterations,
            result.allocations,
            result.deallocations,
            result.reallocations,
            result.allocated_bytes,
            result.deallocated_bytes,
            result.peak_live_delta_bytes,
            result.retained_bytes,
        )
        .expect("write profile row");
    }
}
