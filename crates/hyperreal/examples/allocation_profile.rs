//! Counting-allocator memory profile for every optimized `Real` certificate.
//!
//! Run a release build so debug-only arithmetic does not distort the counts:
//!
//! ```text
//! cargo run --release --example allocation_profile -- 64
//! ```
//!
//! The optional argument is the number of measured lifecycles per
//! representation. Shared process caches are warmed before measurement.

use std::{
    alloc::{GlobalAlloc, Layout, System},
    hint::black_box,
    io::{self, Write},
    sync::atomic::{AtomicU64, Ordering},
};

use hyperreal::{Rational, Real};

struct CountingAllocator;

static ALLOCATIONS: AtomicU64 = AtomicU64::new(0);
static DEALLOCATIONS: AtomicU64 = AtomicU64::new(0);
static REALLOCATIONS: AtomicU64 = AtomicU64::new(0);
static ALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);
static DEALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);
static LIVE_BYTES: AtomicU64 = AtomicU64::new(0);
static PEAK_LIVE_BYTES: AtomicU64 = AtomicU64::new(0);

#[global_allocator]
static GLOBAL_ALLOCATOR: CountingAllocator = CountingAllocator;

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

fn record_allocation(size: usize) {
    let size = u64::try_from(size).expect("allocation size fits u64");
    ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
    ALLOCATED_BYTES.fetch_add(size, Ordering::Relaxed);
    let live = LIVE_BYTES.fetch_add(size, Ordering::Relaxed) + size;
    update_peak(live);
}

fn record_deallocation(size: usize) {
    let size = u64::try_from(size).expect("deallocation size fits u64");
    DEALLOCATIONS.fetch_add(1, Ordering::Relaxed);
    DEALLOCATED_BYTES.fetch_add(size, Ordering::Relaxed);
    LIVE_BYTES.fetch_sub(size, Ordering::Relaxed);
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            record_allocation(layout.size());
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            record_allocation(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        record_deallocation(layout.size());
        unsafe { System.dealloc(pointer, layout) };
    }

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

#[derive(Clone, Copy)]
struct Snapshot {
    allocations: u64,
    deallocations: u64,
    reallocations: u64,
    allocated_bytes: u64,
    deallocated_bytes: u64,
    live_bytes: u64,
}

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

#[derive(Clone, Copy)]
enum Recipe {
    One,
    Pi,
    PiPow,
    PiInv,
    PiExp,
    PiInvExp,
    PiSqrt,
    ConstProduct,
    ConstOffset,
    ConstProductSqrt,
    Sqrt,
    Exp,
    Ln,
    LnAffine,
    LnProduct,
    Log10,
    Log2,
    SinPi,
    TanPi,
    Irrational,
}

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

impl Recipe {
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

struct Measurement {
    allocations: u64,
    deallocations: u64,
    reallocations: u64,
    allocated_bytes: u64,
    deallocated_bytes: u64,
    peak_live_delta_bytes: u64,
    retained_bytes: i128,
}

fn measure(recipe: Recipe, iterations: u64) -> Measurement {
    // Populate shared constants and process-lifetime caches outside the measured
    // window. Each measured value and all of its owned accelerators are dropped.
    let warm = recipe.real();
    black_box(warm.certified_dyadic_interval(-128));
    black_box(warm.to_f64_lossy());
    drop(warm);

    let before = snapshot();
    PEAK_LIVE_BYTES.store(before.live_bytes, Ordering::Relaxed);

    for _ in 0..iterations {
        let value = recipe.real();
        black_box(value.detailed_facts());
        black_box(
            value
                .certified_dyadic_interval(-128)
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
