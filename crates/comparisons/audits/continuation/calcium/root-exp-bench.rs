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

fn operand(case: &str) -> Computable {
    let rational = |n| Computable::rational(Rational::new(n));
    let sqrt2 = || rational(2).sqrt();
    match case {
        "exp-third" => Computable::rational(Rational::fraction(1, 3).unwrap()).exp(),
        "exp-sqrt2" => sqrt2().exp(),
        "exp-sine" => rational(1).sin().exp(),
        "exp-negative-sine" => rational(1).sin().negate().exp(),
        "exp-multiradical" => sqrt2().add(rational(3).sqrt()).exp(),
        "exp-large" => sqrt2().multiply(rational(32)).exp(),
        "exp-tiny" => Computable::rational(
            Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << 512usize).unwrap(),
        )
        .exp(),
        "exp-offset" => rational(1).sin().exp().multiply(rational(8)),
        "exp-opaque-zero" => {
            let x = sqrt2();
            x.clone()
                .sin()
                .square()
                .add(x.cos().square())
                .add(rational(-1))
                .exp()
        }
        "ordinary-sqrt" => rational(2),
        "ordinary-sine-root" => rational(1).sin().add(rational(2)),
        "perfect-square" => rational(9),
        _ => panic!("unknown case {case}"),
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let case = &args[1];
    let lifecycle = &args[2];
    let iterations: usize = args[3].parse().unwrap();
    assert!(
        iterations > 0
            && [
                "construct",
                "fresh",
                "warm",
                "refine",
                "hot-operand",
                "construct-hot"
            ]
            .contains(&lifecycle.as_str())
    );
    let prepared = operand(case).sqrt();
    let hot = operand(case);
    black_box(hot.approx(-768));
    for _ in 0..8 {
        black_box(operand(case).sqrt().approx(-256));
        black_box(prepared.approx(-256));
        black_box(hot.clone().sqrt().approx(-256));
    }
    #[cfg(feature = "allocation-count")]
    let before = allocation::counters();
    let start = Instant::now();
    for _ in 0..iterations {
        match lifecycle.as_str() {
            "construct" => {
                black_box(operand(black_box(case)).sqrt());
            }
            "construct-hot" => {
                black_box(black_box(&hot).clone().sqrt());
            }
            "fresh" => {
                black_box(operand(black_box(case)).sqrt().approx(-256));
            }
            "warm" => {
                black_box(black_box(&prepared).approx(-256));
            }
            "hot-operand" => {
                black_box(black_box(&hot).clone().sqrt().approx(-256));
            }
            "refine" => {
                let value = operand(black_box(case)).sqrt();
                for p in [-32, -128, -512] {
                    black_box(value.approx(p));
                }
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
    println!(
        "{{\"case\":\"{case}\",\"lifecycle\":\"{lifecycle}\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed_ns},\"alloc_calls\":{alloc_calls},\"allocated_bytes\":{allocated_bytes}}}"
    );
}
