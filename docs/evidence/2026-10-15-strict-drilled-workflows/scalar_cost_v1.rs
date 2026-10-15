//! Bounded scalar completion and cost probe; unresolved baseline is counted.
use hyperreal::{CertifiedRealSign, Rational, Real, RealSign};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
use std::time::Instant;

struct Counted;
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static REQUESTED: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

fn allocated(bytes: usize) {
    ALLOCATIONS.fetch_add(1, Relaxed);
    REQUESTED.fetch_add(bytes, Relaxed);
    let live = LIVE.fetch_add(bytes, Relaxed) + bytes;
    PEAK.fetch_max(live, Relaxed);
}

unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            allocated(layout.size());
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            allocated(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Relaxed);
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, old: Layout, size: usize) -> *mut u8 {
        let result = unsafe { System.realloc(pointer, old, size) };
        if !result.is_null() {
            LIVE.fetch_sub(old.size(), Relaxed);
            allocated(size);
        }
        result
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;

fn factors(case: &str, index: usize) -> [Real; 4] {
    if case == "rational" {
        let n = 1_000_000 + (index % 8) as i64;
        return [n, n + 1, n + 1, n + 2].map(Real::from);
    }
    if case == "cancel" {
        let origin = Real::from(6) - Real::from(2).sqrt().unwrap();
        let [a, b] = [1, 3].map(|n| {
            Real::from(6)
                - Real::from(2)
                    * ((Real::pi() * Real::from(n)) / Real::from(8))
                        .unwrap()
                        .sin()
                - &origin
        });
        return [a.clone(), b.clone(), b, a];
    }
    let a = Real::from(2) + (Real::pi() / Real::from(8)).unwrap().sin();
    let b = Real::from(3) + Real::e().sin();
    match case {
        "unequal" => {
            let shifted = &a + (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
            [a, b.clone(), b, shifted]
        }
        "same_sign" => [a.clone(), b.clone(), -b, a],
        _ => panic!("unknown case"),
    }
}

fn evaluate(case: &str, method: usize, factors: &[Real; 4], index: usize) -> usize {
    let [a, b, c, d] = factors;
    let result = match method {
        0 => Real::diff_of_products(a, b, c, d),
        1 => Real::signed_product_sum([true, false], [[a, b], [c, d]]),
        2 => Real::active_signed_product_sum([true, false], [[a, b], [c, d]]),
        _ => unreachable!(),
    };
    if case == "rational" {
        // n(n+1)-(n+1)(n+2) = -2(n+1), independently derived.
        let expected = -2 * (1_000_001 + (index % 8) as i64);
        assert_eq!(result.exact_rational(), Some(Rational::from(expected)));
    }
    let expected = match case {
        "cancel" => RealSign::Zero,
        "same_sign" => RealSign::Positive,
        _ => RealSign::Negative,
    };
    let certified = match black_box(result.certified_sign_until(-512)) {
        CertifiedRealSign::Known { sign, .. } => {
            assert_eq!(sign, expected);
            1
        }
        CertifiedRealSign::Unknown { .. } => {
            assert_eq!(case, "cancel");
            0
        }
    };
    drop(result);
    certified
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let candidate = match args[1].as_str() {
        "candidate" => true,
        "baseline" => false,
        _ => panic!("unknown revision"),
    };
    let iterations: usize = args[2].parse().unwrap();
    assert!((1..=4096).contains(&iterations));
    for case in ["rational", "cancel", "unequal", "same_sign"] {
        for method in 0..3 {
            for prepared in [false, true] {
                let retained = factors(case, 0);
                let perform = |index| {
                    if prepared {
                        evaluate(case, method, &retained, 0)
                    } else {
                        evaluate(case, method, &factors(case, index), index)
                    }
                };
                for index in 0..32 {
                    black_box(perform(index));
                }
                let live = LIVE.load(Relaxed);
                ALLOCATIONS.store(0, Relaxed);
                REQUESTED.store(0, Relaxed);
                PEAK.store(live, Relaxed);
                let started = Instant::now();
                let mut certified = 0;
                for index in 0..iterations {
                    certified += black_box(perform(index));
                }
                let elapsed = started.elapsed().as_nanos();
                let allocations = ALLOCATIONS.load(Relaxed);
                let requested = REQUESTED.load(Relaxed);
                let peak = PEAK.load(Relaxed).saturating_sub(live);
                let retained_bytes = LIVE.load(Relaxed) as i128 - live as i128;
                let expected = if case == "cancel" && !candidate {
                    0
                } else {
                    iterations
                };
                assert_eq!(certified, expected);
                println!(
                    "{{\"case\":\"{case}\",\"method\":{method},\"prepared\":{prepared},\"iterations\":{iterations},\"certified\":{certified},\"unknown\":{},\"nanoseconds\":{elapsed},\"allocations\":{allocations},\"requested_bytes\":{requested},\"peak_live_bytes\":{peak},\"retained_bytes\":{retained_bytes}}}",
                    iterations - certified
                );
            }
        }
    }
}
