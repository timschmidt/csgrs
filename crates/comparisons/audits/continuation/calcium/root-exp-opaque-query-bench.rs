use hyperreal::{CertifiedRealEquality, CertifiedRealSign, Rational, Real, RealSign};
use num::{BigInt, BigUint, One};
use std::{hint::black_box, time::Instant};

// Reuse the already qualified counter without modifying the frozen older
// harness. This private module's main/build functions are not called here.
#[allow(dead_code)]
mod existing_counter {
    include!("root-exp-bench.rs");
    #[cfg(feature = "allocation-count")]
    pub fn counters() -> (usize, usize) {
        allocation::counters()
    }
}

fn pair(case: &str) -> (Real, Real) {
    let (kind, depth) = case.split_once('-').unwrap();
    let depth: usize = depth.parse().unwrap();
    assert!([1, 8, 32, 128].contains(&depth));
    let argument = || {
        let mut x = Real::from(2).sqrt().unwrap();
        for _ in 0..depth { x = x.sin(); }
        x
    };
    // Independently rebuilt opaque sine graphs. Their mathematical values are
    // identical; recognition may require a paired-DAG structural comparison.
    let left = argument().exp().unwrap().sqrt().unwrap();
    let offset = match kind {
        "identity" => Real::from(0),
        "unresolved" => Real::from(Rational::from_bigint_fraction(
            BigInt::one(), BigUint::one() << 2048usize).unwrap()),
        _ => panic!("unknown case"),
    };
    let right = (argument() * Real::from(Rational::fraction(1, 2).unwrap()) + offset)
        .exp().unwrap();
    (left, right)
}

fn equal(left: &Real, right: &Real) -> usize {
    match left.certified_eq_until(right, -256) {
        CertifiedRealEquality::Equal { .. } => 0,
        CertifiedRealEquality::NotEqual { .. } => 1,
        CertifiedRealEquality::Unknown { .. } => 2,
    }
}
fn sign(difference: &Real) -> usize {
    match difference.certified_sign_until(-256) {
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        } => 0,
        CertifiedRealSign::Known { .. } => 1,
        CertifiedRealSign::Unknown { .. } => 2,
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let case = &args[1];
    let lifecycle = &args[2];
    let iterations: usize = args[3].parse().unwrap();
    assert!(
        iterations > 0 && ["fresh", "warm-pair", "warm-difference"].contains(&lifecycle.as_str())
    );
    let prepared = pair(case);
    let difference = &prepared.0 - &prepared.1;
    for _ in 0..8 {
        black_box(equal(&prepared.0, &prepared.1));
        black_box(sign(&difference));
        let (a, b) = pair(case);
        black_box(equal(&a, &b));
    }
    #[cfg(feature = "allocation-count")]
    let before = existing_counter::counters();
    let mut outcomes = [0usize; 3];
    let start = Instant::now();
    for _ in 0..iterations {
        let result = match lifecycle.as_str() {
            "fresh" => {
                let (a, b) = pair(black_box(case));
                equal(&a, &b)
            }
            "warm-pair" => equal(black_box(&prepared.0), black_box(&prepared.1)),
            "warm-difference" => sign(black_box(&difference)),
            _ => unreachable!(),
        };
        outcomes[black_box(result)] += 1;
    }
    let elapsed_ns = start.elapsed().as_nanos();
    #[cfg(feature = "allocation-count")]
    let (alloc_calls, allocated_bytes) = {
        let after = existing_counter::counters();
        (after.0 - before.0, after.1 - before.1)
    };
    #[cfg(not(feature = "allocation-count"))]
    let (alloc_calls, allocated_bytes) = (0, 0);
    println!(
        "{{\"case\":\"{case}\",\"lifecycle\":\"{lifecycle}\",\"iterations\":{iterations},\"elapsed_ns\":{elapsed_ns},\"alloc_calls\":{alloc_calls},\"allocated_bytes\":{allocated_bytes},\"outcomes\":{outcomes:?}}}"
    );
}
