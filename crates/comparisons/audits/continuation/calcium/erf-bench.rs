use hyperreal::{Computable, Rational};
use num::{BigInt, BigUint, One};
use std::{hint::black_box, time::Instant};

#[cfg(feature = "allocation-count")]
mod allocation {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::sync::atomic::{AtomicUsize, Ordering};
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    static BYTES: AtomicUsize = AtomicUsize::new(0);
    struct Count;
    #[global_allocator]
    static GLOBAL: Count = Count;
    unsafe impl GlobalAlloc for Count {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let p = unsafe { System.alloc(layout) };
            if !p.is_null() {
                CALLS.fetch_add(1, Ordering::Relaxed);
                BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            }
            p
        }
        unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
            let p = unsafe { System.alloc_zeroed(layout) };
            if !p.is_null() {
                CALLS.fetch_add(1, Ordering::Relaxed);
                BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            }
            p
        }
        unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
            unsafe { System.dealloc(p, layout) }
        }
        unsafe fn realloc(&self, p: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            let result = unsafe { System.realloc(p, layout, size) };
            if !result.is_null() {
                CALLS.fetch_add(1, Ordering::Relaxed);
                BYTES.fetch_add(size, Ordering::Relaxed);
            }
            result
        }
    }
    pub fn counters() -> (usize, usize) {
        (CALLS.load(Ordering::Relaxed), BYTES.load(Ordering::Relaxed))
    }
}

fn build(case: &str) -> Computable {
    let rational = |n| Computable::rational(Rational::new(n));
    let third = || Computable::rational(Rational::fraction(1, 3).unwrap());
    match case {
        "erf-zero" => rational(0).erf(),
        "erf-third" => third().erf(),
        "erf-negative" => third().negate().erf(),
        "erf-tiny" => Computable::rational(Rational::from_bigint_fraction(
            BigInt::one(), BigUint::one() << 512usize).unwrap()).erf(),
        "erf-tail" => rational(8).erf(),
        "erfc-third" => third().erfc(),
        "erfc-negative" => third().negate().erfc(),
        "erfc-tail" => rational(8).erfc(),
        "normal-cdf" => third().pnorm(),
        "complement" => third().erf().add(third().erfc()).add(rational(-1)),
        "ordinary-add" => rational(2).sqrt().add(rational(3).sqrt()).add(rational(5).sqrt()),
        "ordinary-sqrt" => rational(2).sqrt(),
        _ => panic!("unknown case {case}"),
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let case = &args[1];
    let lifecycle = &args[2];
    let iterations: usize = args[3].parse().unwrap();
    assert!(iterations > 0 && ["construct", "fresh", "warm", "refine"].contains(&lifecycle.as_str()));
    let prepared = build(case);
    for _ in 0..8 {
        black_box(build(case).approx(-256));
        black_box(prepared.approx(-256));
    }
    #[cfg(feature = "allocation-count")]
    let before = allocation::counters();
    let start = Instant::now();
    for _ in 0..iterations {
        match lifecycle.as_str() {
            "construct" => { black_box(build(black_box(case))); }
            "fresh" => { black_box(build(black_box(case)).approx(-256)); }
            "warm" => { black_box(black_box(&prepared).approx(-256)); }
            "refine" => {
                let value = build(black_box(case));
                for p in [-32, -128, -512] { black_box(value.approx(p)); }
            }
            _ => unreachable!(),
        }
    }
    let elapsed_ns = start.elapsed().as_nanos();
    #[cfg(feature = "allocation-count")]
    let (alloc_calls, allocated_bytes) = {
        let after = allocation::counters();
        (after.0 - before.0, after.1 - before.1)
    };
    #[cfg(not(feature = "allocation-count"))]
    let (alloc_calls, allocated_bytes) = (0, 0);
    println!("{{\"case\":\"{case}\",\"lifecycle\":\"{lifecycle}\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed_ns},\"alloc_calls\":{alloc_calls},\"allocated_bytes\":{allocated_bytes}}}");
}
