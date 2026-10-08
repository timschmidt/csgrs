use hyperreal::Rational;
use num::{BigInt, BigUint, One};
use std::{alloc::{GlobalAlloc, Layout, System}, env, hint::black_box,
    sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed}, time::Instant};

struct Meter;
static TRACK: AtomicBool = AtomicBool::new(false);
static BYTES: AtomicU64 = AtomicU64::new(0);
static CALLS: AtomicU64 = AtomicU64::new(0);
fn record(n: usize) {
    if TRACK.load(Relaxed) {
        BYTES.fetch_add(n as u64, Relaxed);
        CALLS.fetch_add(1, Relaxed);
    }
}
unsafe impl GlobalAlloc for Meter {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        record(layout.size());
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        record(layout.size());
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        record(size);
        unsafe { System.realloc(ptr, layout, size) }
    }
}
#[global_allocator] static ALLOCATOR: Meter = Meter;

fn inputs(kind: &str, n: usize, bits: usize, seed: usize) -> Vec<Rational> {
    (1..=n).map(|i| {
        let common = (BigUint::one() << bits) - BigUint::one();
        let noise = (BigUint::from(6364136223846793005_u64)
            * BigUint::from(i + seed * 100003) + BigUint::from(1442695040888963407_u64)) & &common;
        let den = match kind {
            "equal" => common,
            "dyadic" => BigUint::one() << (1 + (i * 13 + seed) % bits),
            "nested" => BigUint::from(3_u8).pow((1 + (i * 13 + seed) % bits) as u32),
            "mixed" => ((BigUint::one() << (bits / 2)) + BigUint::one())
                * (((BigUint::one() << (bits - 1)) + noise) | BigUint::one()),
            "independent" => ((BigUint::one() << (bits - 1)) + noise) | BigUint::one(),
            _ => panic!("unknown family"),
        };
        let num = BigInt::from(i + seed) * if (i + seed) % 2 == 0 { 1 } else { -1 };
        Rational::from_bigint_fraction(num, den).unwrap()
    }).collect()
}
// Same public Hyper wide-GCD primitive on both schedule controls. The current
// mean additionally owns private mixed-width/dyadic/equal-denominator paths.
fn lcm(a: BigUint, b: &BigUint) -> BigUint {
    let gcd = Rational::gcd_magnitudes(&a, b);
    (a / gcd) * b
}
fn tree_lcm(xs: &[&Rational]) -> BigUint {
    match xs.len() {
        0 => BigUint::one(),
        1 => xs[0].denominator().clone(),
        n => lcm(tree_lcm(&xs[..n/2]), &tree_lcm(&xs[n/2..])),
    }
}
fn lcm_mean(xs: &[&Rational], balanced: bool) -> Rational {
    let d = if balanced { tree_lcm(xs) } else {
        xs.iter().fold(BigUint::one(), |d, q| lcm(d, q.denominator()))
    };
    let mut positive = BigUint::ZERO;
    let mut negative = BigUint::ZERO;
    for q in xs {
        let magnitude = q.numerator() * (&d / q.denominator());
        if q.is_negative() { negative += magnitude; } else { positive += magnitude; }
    }
    Rational::from_bigint_fraction(BigInt::from(positive) - BigInt::from(negative),
        d * BigUint::from(xs.len())).unwrap()
}
fn balanced_sum(xs: &[&Rational]) -> Rational {
    match xs.len() {
        0 => Rational::zero(), 1 => xs[0].clone(),
        n => balanced_sum(&xs[..n/2]) + balanced_sum(&xs[n/2..]),
    }
}
fn to_gmp(q: &Rational) -> rug::Rational {
    // Hyper Display uses mixed fractions; GMP ignores whitespace. Never use
    // cross-library Display/parse as a supposedly neutral scalar interchange.
    let mut n = rug::Integer::from_str_radix(&q.numerator().to_string(), 10).unwrap();
    if q.is_negative() { n = -n; }
    let d = rug::Integer::from_str_radix(&q.denominator().to_string(), 10).unwrap();
    rug::Rational::from((n, d))
}
fn main() {
    let args: Vec<_> = env::args().collect();
    let (algorithm, kind) = (args[1].as_str(), args[2].as_str());
    let n = args[3].parse().unwrap();
    let bits = args[4].parse().unwrap();
    let seed = args[5].parse().unwrap();
    let xs = inputs(kind, n, bits, seed);
    let refs: Vec<_> = xs.iter().collect();
    let start = Instant::now();
    TRACK.store(args.get(6).is_none_or(|mode| mode != "time-only"), Relaxed);
    let result = black_box(match algorithm {
        "current" => Rational::mean_refs(black_box(&refs)).unwrap(),
        "seq-lcm" => lcm_mean(black_box(&refs), false),
        "balanced-lcm" => lcm_mean(black_box(&refs), true),
        "balanced-add" => balanced_sum(black_box(&refs)) / Rational::from(n as i64),
        _ => panic!("unknown algorithm"),
    });
    TRACK.store(false, Relaxed);
    let elapsed = start.elapsed();
    let bytes = BYTES.load(Relaxed);
    let calls = CALLS.load(Relaxed);
    // Independent GMP rational arithmetic, outside the measurement window.
    let mut oracle = rug::Rational::new();
    for q in &xs { oracle += to_gmp(q); }
    oracle /= n as u32;
    assert_eq!(to_gmp(&result), oracle);
    println!("{algorithm}\t{kind}\t{n}\t{bits}\t{seed}\t{:.3}\t{bytes}\t{calls}",
        elapsed.as_secs_f64() * 1e6);
}
