use hyperreal::Rational;
use num::{BigInt, BigUint};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
use std::time::Instant;

struct AuditAllocator;
static TRACK: AtomicBool = AtomicBool::new(false);
static CALLS: AtomicU64 = AtomicU64::new(0);
static BYTES: AtomicU64 = AtomicU64::new(0);
unsafe impl GlobalAlloc for AuditAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if TRACK.load(Relaxed) {
            CALLS.fetch_add(1, Relaxed);
            BYTES.fetch_add(layout.size() as u64, Relaxed);
        }
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        if TRACK.load(Relaxed) {
            CALLS.fetch_add(1, Relaxed);
            BYTES.fetch_add(layout.size() as u64, Relaxed);
        }
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        if TRACK.load(Relaxed) {
            CALLS.fetch_add(1, Relaxed);
            BYTES.fetch_add(size as u64, Relaxed);
        }
        unsafe { System.realloc(ptr, layout, size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}
#[global_allocator]
static ALLOCATOR: AuditAllocator = AuditAllocator;

fn input(case: &str) -> Rational {
    let max = || Rational::try_from(f64::MAX).unwrap();
    let dyadic = |n: BigUint, shift: usize| Rational::from_bigint_fraction(
        BigInt::from(n), BigUint::from(1_u8) << shift).unwrap();
    match case {
        "exact_word" => Rational::fraction(3, 4).unwrap(),
        "exact_wide" => Rational::try_from(2.0_f64.powi(700)).unwrap(),
        "exact_subnormal" => Rational::try_from(f64::from_bits(17)).unwrap(),
        "inexact_word" => Rational::fraction((1_i64 << 54) + 1, 8).unwrap(),
        "inexact_wide" => dyadic((BigUint::from(1_u8) << 200) + BigUint::from(1_u8), 180),
        "non_dyadic" => Rational::fraction(1, 3).unwrap(),
        "non_dyadic_wide" => Rational::from_bigint_fraction(
            BigInt::from((BigUint::from(1_u8) << 4096) + BigUint::from(1_u8)),
            (BigUint::from(1_u8) << 4096) + BigUint::from(3_u8)).unwrap(),
        "near_max_below" => max() - Rational::one(),
        "near_max_above" => max() + Rational::one(),
        "near_max_negative" => -(max() - Rational::one()),
        "near_max_deep" => max() - dyadic(BigUint::from(1_u8), 4096),
        "overflow" => dyadic(BigUint::from(1_u8) << 1024, 0),
        "unsupported_subnormal" => dyadic(BigUint::from(3_u8), 1076),
        "exact_max" => max(),
        _ => panic!("unknown case"),
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let case = &args[1];
    let mode = &args[2];
    let iterations: u64 = args[3].parse().unwrap();
    let input = input(case);
    let run = || black_box(black_box(&input).to_f64_enclosure());
    for _ in 0..8 { black_box(run()); }
    CALLS.store(0, Relaxed);
    BYTES.store(0, Relaxed);
    TRACK.store(mode == "alloc", Relaxed);
    let start = Instant::now();
    for _ in 0..iterations { black_box(run()); }
    let elapsed = start.elapsed().as_nanos();
    TRACK.store(false, Relaxed);
    println!("{{\"iterations\":{iterations},\"ns\":{elapsed},\"allocations\":{},\"requested_bytes\":{}}}",
        CALLS.load(Relaxed), BYTES.load(Relaxed));
}
