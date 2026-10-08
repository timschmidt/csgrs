use hyperreal::{Computable, Rational};
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

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let numerator: i64 = args[1].parse().unwrap();
    let denominator: u64 = args[2].parse().unwrap();
    let precision: i32 = args[3].parse().unwrap();
    let mode = &args[4];
    let iterations: u64 = args[5].parse().unwrap();
    let input = Rational::fraction(numerator, denominator).unwrap();
    let make = || Computable::rational(black_box(input.clone())).exp();
    // Initialize shared constants to equal depth on both sides, including cold
    // object runs; this measures fresh object work, not first-process startup.
    black_box(Computable::one().exp().approx(-512));
    black_box(Computable::rational(Rational::new(2)).ln().approx(-512));
    let cached = make();
    black_box(cached.approx(precision));
    let run = || {
        if mode == "constructor" {
            black_box(make());
        } else if mode == "warm" {
            black_box(cached.approx(black_box(precision)));
        } else {
            black_box(make().approx(black_box(precision)));
        }
    };
    for _ in 0..8 { run(); }
    CALLS.store(0, Relaxed);
    BYTES.store(0, Relaxed);
    TRACK.store(mode == "alloc", Relaxed);
    let start = Instant::now();
    for _ in 0..iterations { run(); }
    let elapsed = start.elapsed().as_nanos();
    TRACK.store(false, Relaxed);
    println!("{{\"n\":{numerator},\"d\":{denominator},\"p\":{precision},\"mode\":\"{mode}\",\"iterations\":{iterations},\"ns\":{elapsed},\"allocations\":{},\"requested_bytes\":{}}}",
        CALLS.load(Relaxed), BYTES.load(Relaxed));
}
