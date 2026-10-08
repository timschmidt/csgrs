use hyperreal::{CertifiedRealEquality, Computable, Rational, Real, RealSign};
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

enum Query {
    Equality(Real, Real, i32),
    Sign(Computable, i32),
}

fn epsilon(bits: usize) -> Rational {
    Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << bits).unwrap()
}

fn build(case: &str) -> Query {
    match case {
        "log-identity" | "algebraic-identity" => {
            let x = Real::from(1) + Real::from(2).sqrt().unwrap();
            let expanded = Real::from(3) + Real::from(2) * Real::from(2).sqrt().unwrap();
            if case == "algebraic-identity" {
                Query::Equality(&x * &x, expanded, -512)
            } else {
                Query::Equality(
                    Real::from(2) * x.ln().unwrap(),
                    expanded.ln().unwrap(),
                    -512,
                )
            }
        }
        "log-near-nonzero" | "log-near-unknown" => {
            let bits = if case == "log-near-nonzero" { 128 } else { 512 };
            let x = Computable::rational(Rational::new(17) + epsilon(bits)).ln();
            let y = Computable::rational(Rational::new(17)).ln();
            Query::Sign(x.add(y.negate()), if bits == 128 { -256 } else { -128 })
        }
        "log-algebraic-near-unknown" => {
            let x = Computable::rational(Rational::new(2))
                .sqrt()
                .add(Computable::one());
            let y = x.clone().square().add(Computable::rational(epsilon(512)));
            Query::Sign(
                y.ln().add(
                    x.ln()
                        .multiply(Computable::rational(Rational::new(2)))
                        .negate(),
                ),
                -128,
            )
        }
        "log-multiquadratic-unknown" => {
            let root = |d| Computable::rational(Rational::new(d)).sqrt();
            let x = root(2).add(root(3)).add(root(5));
            let y = root(6)
                .add(root(10))
                .add(root(15))
                .multiply(Computable::rational(Rational::new(2)))
                .add(Computable::rational(Rational::new(10) + epsilon(512)));
            Query::Sign(
                y.ln().add(
                    x.ln()
                        .multiply(Computable::rational(Rational::new(2)))
                        .negate(),
                ),
                -128,
            )
        }
        "log-transcendental-unknown" => {
            let x = Computable::pi()
                .add(Computable::rational(epsilon(512)))
                .ln();
            Query::Sign(x.add(Computable::pi().ln().negate()), -128)
        }
        "nonlog-unknown" => {
            let x = Computable::rational(Rational::new(2)).sqrt();
            Query::Sign(
                x.clone()
                    .sin()
                    .square()
                    .add(x.cos().square())
                    .add(Computable::one().negate()),
                -256,
            )
        }
        "ordinary-log" => Query::Sign(
            Computable::rational(Rational::fraction(3, 2).unwrap()).ln(),
            -256,
        ),
        "ordinary-rational" => Query::Equality(Real::from(17), Real::from(19), -256),
        "exp-cancellation" => Query::Sign(
            Computable::rational(epsilon(80))
                .exp()
                .add(Computable::one().negate()),
            -256,
        ),
        _ => panic!("unknown case: {case}"),
    }
}

fn evaluate(query: &Query) -> usize {
    match query {
        Query::Equality(a, b, floor) => match a.certified_eq_until(b, *floor) {
            CertifiedRealEquality::Equal { .. } => 0,
            CertifiedRealEquality::NotEqual { .. } => 1,
            CertifiedRealEquality::Unknown { .. } => 2,
        },
        Query::Sign(x, floor) => match x.sign_until(*floor) {
            Some(RealSign::Zero) => 0,
            Some(RealSign::Positive) => 1,
            None => 2,
            Some(RealSign::Negative) => panic!("false negative sign"),
        },
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let case = &args[1];
    let lifecycle = &args[2];
    let iterations: usize = args[3].parse().unwrap();
    assert!(iterations > 0 && ["fresh", "warm"].contains(&lifecycle.as_str()));
    let prepared = build(case);
    for _ in 0..8 {
        black_box(evaluate(&build(case)));
        black_box(evaluate(&prepared));
    }
    #[cfg(feature = "allocation-count")]
    let (before_calls, before_bytes) = allocation::counters();
    let mut outcomes = [0_u64; 3];
    let start = Instant::now();
    for _ in 0..iterations {
        let outcome = if lifecycle == "fresh" {
            evaluate(black_box(&build(case)))
        } else {
            evaluate(black_box(&prepared))
        };
        outcomes[outcome] += 1;
    }
    let elapsed_ns = start.elapsed().as_nanos();
    #[cfg(feature = "allocation-count")]
    let (calls, bytes) = {
        let (calls, bytes) = allocation::counters();
        (calls - before_calls, bytes - before_bytes)
    };
    #[cfg(not(feature = "allocation-count"))]
    let (calls, bytes) = (0_usize, 0_usize);
    if case == "log-identity" || case == "nonlog-unknown" {
        assert_eq!(outcomes[1], 0, "false inequality of an exact identity");
    } else if case == "algebraic-identity" {
        assert_eq!(outcomes[0], iterations as u64);
    } else {
        assert_eq!(outcomes[0], 0, "false equality for a nonzero control");
    }
    println!(
        "{{\"case\":\"{case}\",\"lifecycle\":\"{lifecycle}\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed_ns},\"alloc_calls\":{calls},\"allocated_bytes\":{bytes},\"outcomes\":{outcomes:?}}}"
    );
}
