use hyperreal::{CertifiedRealSign, Rational, Real, RealSign};
use num::{BigInt, BigRational, BigUint, One, Zero};
use std::hint::black_box;
use std::time::Instant;

include!("baseline.rs"); // Mechanically extracted unchanged from the frozen actual caller.

#[cfg(feature = "allocation-profile")]
mod allocation {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering::Relaxed};
    pub static ENABLED: AtomicBool = AtomicBool::new(false);
    pub static CALLS: AtomicUsize = AtomicUsize::new(0);
    pub static BYTES: AtomicUsize = AtomicUsize::new(0);
    pub static LIVE: AtomicUsize = AtomicUsize::new(0);
    pub static PEAK: AtomicUsize = AtomicUsize::new(0);
    pub struct Counter;
    fn add(size: usize) {
        CALLS.fetch_add(1, Relaxed);
        BYTES.fetch_add(size, Relaxed);
        let live = LIVE.fetch_add(size, Relaxed) + size;
        PEAK.fetch_max(live, Relaxed);
    }
    unsafe impl GlobalAlloc for Counter {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let p = unsafe { System.alloc(layout) };
            if !p.is_null() && ENABLED.load(Relaxed) {
                add(layout.size());
            }
            p
        }
        unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
            if ENABLED.load(Relaxed) {
                LIVE.fetch_sub(layout.size(), Relaxed);
            }
            unsafe { System.dealloc(p, layout) }
        }
        unsafe fn realloc(&self, p: *mut u8, layout: Layout, new: usize) -> *mut u8 {
            let q = unsafe { System.realloc(p, layout, new) };
            if !q.is_null() && ENABLED.load(Relaxed) {
                LIVE.fetch_sub(layout.size(), Relaxed);
                add(new);
            }
            q
        }
    }
}
#[cfg(feature = "allocation-profile")]
#[global_allocator]
static ALLOCATOR: allocation::Counter = allocation::Counter;

fn trim(mut p: Vec<Real>) -> Vec<Real> {
    while p.len() > 1 && p.last().is_some_and(exact_real_is_zero) {
        p.pop();
    }
    p
}

fn add_halves(a: &[Real], b: &[Real]) -> Vec<Real> {
    let mut result = a.to_vec();
    result.resize_with(a.len().max(b.len()), Real::zero);
    for (r, x) in result.iter_mut().zip(b) {
        *r += x;
    }
    result
}

// Independent systems implementation of the three-product identity, not a donor port.
// Unbalanced inputs use the unchanged base case; no power-of-two padding is required.
fn karatsuba(a: &[Real], b: &[Real], cutoff: usize) -> Vec<Real> {
    let short = a.len().min(b.len());
    let long = a.len().max(b.len());
    if short <= cutoff || short * 2 <= long {
        return multiply_exact_polynomials(a, b);
    }
    let middle = long / 2;
    let (a0, a1) = a.split_at(middle);
    let (b0, b1) = b.split_at(middle);
    let low = karatsuba(a0, b0, cutoff);
    let high = karatsuba(a1, b1, cutoff);
    let mut cross = karatsuba(&add_halves(a0, a1), &add_halves(b0, b1), cutoff);
    cross.resize_with(cross.len().max(low.len()).max(high.len()), Real::zero);
    for (c, x) in cross.iter_mut().zip(&low) {
        *c -= x;
    }
    for (c, x) in cross.iter_mut().zip(&high) {
        *c -= x;
    }
    let mut result = vec![Real::zero(); a.len() + b.len() - 1];
    for (r, x) in result.iter_mut().zip(low) {
        *r += x;
    }
    for (r, x) in result[middle..].iter_mut().zip(cross) {
        *r += x;
    }
    for (r, x) in result[2 * middle..].iter_mut().zip(high) {
        *r += x;
    }
    trim(result)
}

// Pilot gate only: deliberately conservative and not a production threshold claim.
// numerator()/denominator() may canonicalize a deferred rational, so this cost
// belongs inside measured dispatch; it is not free representation metadata.
fn qualifies(a: &[Real], b: &[Real]) -> bool {
    let short = a.len().min(b.len());
    if short < 32 || short * 2 <= a.len().max(b.len()) {
        return false;
    }
    [a, b].into_iter().all(|p| {
        let mut expensive = 0;
        for c in p {
            let Some(r) = c.exact_rational_ref() else {
                return false;
            };
            if r.numerator().bits().max(r.denominator().bits()) >= 512 {
                expensive += 1;
            }
        }
        expensive * 4 >= p.len() * 3
    })
}

type Multiply = fn(&[Real], &[Real]) -> Vec<Real>;
fn k8(a: &[Real], b: &[Real]) -> Vec<Real> {
    karatsuba(a, b, 8)
}
fn k16(a: &[Real], b: &[Real]) -> Vec<Real> {
    karatsuba(a, b, 16)
}
fn k32(a: &[Real], b: &[Real]) -> Vec<Real> {
    karatsuba(a, b, 32)
}
fn gated(a: &[Real], b: &[Real]) -> Vec<Real> {
    if qualifies(a, b) {
        k16(a, b)
    } else {
        multiply_exact_polynomials(a, b)
    }
}
const ALGORITHMS: [(&str, Multiply); 5] = [
    ("baseline", multiply_exact_polynomials),
    ("k8", k8),
    ("k16", k16),
    ("k32", k32),
    ("gated", gated),
];

fn next(state: &mut u64) -> u64 {
    *state ^= *state << 13;
    *state ^= *state >> 7;
    *state ^= *state << 17;
    *state
}
fn values(n: usize, bits: usize, density: &str, seed: u64) -> Vec<BigRational> {
    let mut state = seed;
    (0..n)
        .map(|i| {
            if density == "sparse" && i % 8 != 0 && i + 1 != n {
                return BigRational::zero();
            }
            let mut magnitude = BigUint::zero();
            for shift in (0..bits).step_by(64) {
                let width = (bits - shift).min(64);
                let word = next(&mut state) & (u64::MAX >> (64 - width));
                magnitude |= BigUint::from(word) << shift;
            }
            if bits > 0 {
                magnitude |= BigUint::one() << (bits - 1);
            }
            let mut numerator = BigInt::from(magnitude);
            if next(&mut state) & 1 == 1 {
                numerator = -numerator;
            }
            let denominator = if density == "fraction" {
                BigInt::from([17, 19, 23, 29, 31, 37, 41][i % 7])
            } else {
                BigInt::one()
            };
            BigRational::new(numerator, denominator)
        })
        .collect()
}
fn as_real(r: &BigRational) -> Real {
    Real::from(
        Rational::from_bigint_fraction(r.numer().clone(), r.denom().to_biguint().unwrap()).unwrap(),
    )
}
fn oracle(a: &[BigRational], b: &[BigRational]) -> Vec<Real> {
    if a.is_empty() || b.is_empty() {
        return vec![Real::zero()];
    }
    let mut c = vec![BigRational::zero(); a.len() + b.len() - 1];
    for (i, x) in a.iter().enumerate() {
        for (j, y) in b.iter().enumerate() {
            c[i + j] += x * y;
        }
    }
    while c.len() > 1 && c.last().is_some_and(Zero::is_zero) {
        c.pop();
    }
    c.iter().map(as_real).collect()
}
fn inputs(
    n: usize,
    m: usize,
    bits: usize,
    density: &str,
    seed: u64,
) -> (Vec<Real>, Vec<Real>, Vec<Real>) {
    let a = values(n, bits, density, seed);
    let b = values(m, bits, density, seed + 391);
    let expected = oracle(&a, &b);
    (
        a.iter().map(as_real).collect(),
        b.iter().map(as_real).collect(),
        expected,
    )
}

fn checks() {
    let mut count = 0;
    for n in [0, 1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65] {
        for m in [0, 1, 2, 3, 8, 9, 16, 17, 32, 33, 64, 65] {
            for seed in [1, 17, 137] {
                let (a, b, expected) =
                    inputs(n, m, 7, if seed == 17 { "sparse" } else { "dense" }, seed);
                for (name, f) in ALGORITHMS {
                    assert_eq!(f(&a, &b), expected, "{name} n={n} m={m} seed={seed}");
                    count += 1;
                }
            }
        }
    }
    println!("PASS\tshape-integer-grid\t{count}");
    let before = count;
    for n in [9, 17, 32, 33, 64, 65] {
        for bits in [32, 512, 2048] {
            for density in ["dense", "sparse", "fraction"] {
                let (a, b, expected) = inputs(n, n - 1, bits, density, 93);
                for (name, f) in ALGORITHMS {
                    assert_eq!(f(&a, &b), expected, "{name} {n}/{bits}/{density}");
                    count += 1;
                }
            }
        }
    }
    println!("PASS\tlarge-rational-grid\t{}", count - before);
    let before = count;
    for n in [1, 9, 17, 33] {
        let (a, b, e) = inputs(n, n, 5, "dense", 17);
        let s2 = Real::from(2).sqrt().unwrap();
        let s3 = Real::from(3).sqrt().unwrap();
        let s6 = Real::from(6).sqrt().unwrap();
        let a: Vec<_> = a.iter().map(|x| x * &s2).collect();
        let b: Vec<_> = b.iter().map(|x| x * &s3).collect();
        let e: Vec<_> = e.iter().map(|x| x * &s6).collect();
        assert!(!qualifies(&a, &b));
        for (name, f) in ALGORITHMS {
            assert_eq!(f(&a, &b), e, "symbolic {name} {n}");
            count += 1;
        }
    }
    for n in [1, 9, 17, 33, 65] {
        for zero_kind in 0..3 {
            let (mut a, mut b, _) = inputs(n, n, 32, "dense", 37);
            if zero_kind == 0 {
                a.fill(Real::zero());
            }
            if zero_kind == 1 {
                a[n - 1] = Real::zero();
                b[n - 1] = Real::zero();
            }
            if zero_kind == 2 {
                for i in 0..n {
                    b[i] = -&a[i];
                }
            }
            let expected = multiply_exact_polynomials(&a, &b);
            for (name, f) in ALGORITHMS {
                assert_eq!(f(&a, &b), expected, "cancel/zero {name} {n} {zero_kind}");
                count += 1;
            }
        }
    }
    println!("PASS\tsymbolic-and-zero-boundaries\t{}", count - before);
    println!("SUMMARY\t{count}");
}

#[repr(C)]
struct Timespec {
    seconds: i64,
    nanos: i64,
}
unsafe extern "C" {
    fn clock_gettime(clock: i32, value: *mut Timespec) -> i32;
}
fn cpu() -> f64 {
    let mut time = Timespec {
        seconds: 0,
        nanos: 0,
    };
    assert_eq!(unsafe { clock_gettime(2, &mut time) }, 0); // Linux CLOCK_PROCESS_CPUTIME_ID.
    time.seconds as f64 + time.nanos as f64 * 1e-9
}
fn batch(f: Multiply, iterations: usize, a: &[Real], b: &[Real], expected: &[Real]) -> (f64, f64) {
    let wall = Instant::now();
    let started = cpu();
    for _ in 0..iterations {
        let result = black_box(f(black_box(a), black_box(b)));
        assert_eq!(result, expected);
    }
    (cpu() - started, wall.elapsed().as_secs_f64())
}
fn bench(n: usize, m: usize, bits: usize, density: &str, seconds: f64, rounds: usize) {
    assert!(
        !cfg!(feature = "allocation-profile"),
        "allocation instrumentation must not affect timing"
    );
    let (a, b, e) = inputs(n, m, bits, density, 137);
    let mut batches = Vec::new();
    println!("INPUT\t{n}\t{m}\t{bits}\t{density}\t{}", qualifies(&a, &b));
    for (name, f) in ALGORITHMS {
        assert_eq!(f(&a, &b), e);
        let warm = cpu();
        while cpu() - warm < seconds {
            batch(f, 1, &a, &b, &e);
        }
        let mut count = 1;
        loop {
            let (c, w) = batch(f, count, &a, &b, &e);
            if c >= seconds {
                println!("CALIBRATE\t{name}\t{count}\t{c:.9}\t{w:.9}");
                break;
            }
            count *= 2;
            assert!(count <= 16_777_216);
        }
        batches.push(count);
    }
    for round in 0..rounds + 2 {
        for position in 0..ALGORITHMS.len() {
            let index = (round + position) % ALGORITHMS.len();
            let (name, f) = ALGORITHMS[index];
            let (c, w) = batch(f, batches[index], &a, &b, &e);
            println!(
                "{}\t{round}\t{position}\t{name}\t{}\t{c:.9}\t{w:.9}",
                if round < 2 { "WARMUP" } else { "MEASURE" },
                batches[index]
            );
            if round >= 2 {
                assert!(c >= seconds * 0.5, "measured batch too short");
            }
        }
    }
    println!("PASS\tbench");
}
#[cfg(feature = "allocation-profile")]
fn memory(n: usize, m: usize, bits: usize, density: &str) {
    use allocation::*;
    use std::sync::atomic::Ordering::Relaxed;
    let (a, b, e) = inputs(n, m, bits, density, 137);
    // Only rational workloads: inputs have been canonicalized and are never
    // freed in the measured window. Drop result before disabling the counter.
    for (name, f) in ALGORITHMS {
        assert_eq!(f(&a, &b), e);
        CALLS.store(0, Relaxed);
        BYTES.store(0, Relaxed);
        LIVE.store(0, Relaxed);
        PEAK.store(0, Relaxed);
        ENABLED.store(true, Relaxed);
        let product = f(&a, &b);
        let retained = LIVE.load(Relaxed);
        drop(product);
        ENABLED.store(false, Relaxed);
        assert_eq!(
            LIVE.load(Relaxed),
            0,
            "allocation accounting escaped measured window"
        );
        println!(
            "MEMORY\t{name}\t{}\t{}\t{}\t{retained}",
            CALLS.load(Relaxed),
            BYTES.load(Relaxed),
            PEAK.load(Relaxed)
        );
    }
}
fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("check") {
        checks();
        return;
    }
    let n = args[1].parse().unwrap();
    let m = args[2].parse().unwrap();
    let bits = args[3].parse().unwrap();
    match args[0].as_str() {
        "bench" => bench(
            n,
            m,
            bits,
            &args[4],
            args[5].parse().unwrap(),
            args[6].parse().unwrap(),
        ),
        #[cfg(feature = "allocation-profile")]
        "memory" => memory(n, m, bits, &args[4]),
        _ => panic!("unknown mode"),
    }
}
