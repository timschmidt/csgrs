use hyperreal::{CertifiedRealEquality, Rational, Real};
use num::{BigInt, BigUint, One};
use std::{hint::black_box, time::Instant};

#[allow(dead_code)]
mod existing_counter {
    include!("root-exp-bench.rs");
    #[cfg(feature = "allocation-count")]
    pub fn counters() -> (usize, usize) { allocation::counters() }
}

fn pair(case: &str, sharing: &str) -> (Real, Real) {
    let (kind, depth) = case.split_once('-').unwrap();
    let depth: usize = depth.parse().unwrap();
    assert!([1, 8, 32, 128].contains(&depth));
    let base = || Real::from(2).sqrt().unwrap();
    let sine = |mut value: Real, count| {
        for _ in 0..count { value = value.sin(); }
        value
    };
    let (a, b) = match sharing {
        "independent" => (sine(base(), depth), sine(base(), depth)),
        "shared" => { let x = sine(base(), depth); (x.clone(), x) }
        "common-tail" => {
            let x = sine(base(), depth / 2);
            (sine(x.clone(), depth - depth / 2), sine(x, depth - depth / 2))
        }
        _ => panic!("unknown sharing"),
    };
    let delta = match kind {
        "identity" => Real::from(0),
        "unresolved" => Real::from(Rational::from_bigint_fraction(
            BigInt::one(), BigUint::one() << 2048usize).unwrap()),
        _ => panic!("unknown case"),
    };
    (a.exp().unwrap().sqrt().unwrap(),
        (b * Real::from(Rational::fraction(1, 2).unwrap()) + delta).exp().unwrap())
}

fn query(pair: &(Real, Real)) -> usize {
    match pair.0.certified_eq_until(&pair.1, -256) {
        CertifiedRealEquality::Equal { .. } => 0,
        CertifiedRealEquality::NotEqual { .. } => 1,
        CertifiedRealEquality::Unknown { .. } => 2,
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let case = &args[1]; let sharing = &args[2];
    let workers: usize = args[3].parse().unwrap();
    assert!(workers > 0 && workers <= 256);
    let prepared = pair(case, sharing);
    // Warm numeric operands/shared constants on the parent only. Each measured
    // worker has new TLS. Thread creation/destruction is outside query clocks.
    for _ in 0..8 { black_box(query(&prepared)); }
    // Initialize main-thread scoped-thread bookkeeping and the OS thread cache.
    std::thread::scope(|scope| scope.spawn(|| black_box(0)).join().unwrap());
    let mut first_ns = 0u128; let mut warm_ns = 0u128; let mut clock_ns = 0u128;
    let mut first_calls = 0usize; let mut first_bytes = 0usize;
    let mut warm_calls = 0usize; let mut warm_bytes = 0usize;
    let mut outcomes = [0usize; 3];
    let start = Instant::now();
    for _ in 0..workers {
        let (cold, hot, clock, result, fc, fb, wc, wb) = std::thread::scope(|scope| {
            scope.spawn(|| {
                let start = Instant::now(); black_box(&prepared);
                let clock = start.elapsed().as_nanos();
                #[cfg(feature = "allocation-count")]
                let before = existing_counter::counters();
                let start = Instant::now();
                let result = black_box(query(black_box(&prepared)));
                let cold = start.elapsed().as_nanos();
                #[cfg(feature = "allocation-count")]
                let middle = existing_counter::counters();
                let start = Instant::now();
                for _ in 0..16 { assert_eq!(black_box(query(black_box(&prepared))), result); }
                let hot = start.elapsed().as_nanos();
                #[cfg(feature = "allocation-count")]
                let (fc, fb, wc, wb) = {
                    let after = existing_counter::counters();
                    (middle.0 - before.0, middle.1 - before.1,
                        after.0 - middle.0, after.1 - middle.1)
                };
                #[cfg(not(feature = "allocation-count"))]
                let (fc, fb, wc, wb) = (0, 0, 0, 0);
                (cold, hot, clock, result, fc, fb, wc, wb)
            }).join().unwrap()
        });
        first_ns += cold; warm_ns += hot; clock_ns += clock;
        first_calls += fc; first_bytes += fb; warm_calls += wc; warm_bytes += wb;
        outcomes[result] += 1;
    }
    let lifecycle_ns = start.elapsed().as_nanos();
    println!("{{\"case\":\"{case}\",\"sharing\":\"{sharing}\",\"workers\":{workers},\"warm_queries_per_worker\":16,\"first_ns\":{first_ns},\"warm_ns\":{warm_ns},\"clock_ns\":{clock_ns},\"lifecycle_ns\":{lifecycle_ns},\"first_calls\":{first_calls},\"first_bytes\":{first_bytes},\"warm_calls\":{warm_calls},\"warm_bytes\":{warm_bytes},\"outcomes\":{outcomes:?}}}");
}
