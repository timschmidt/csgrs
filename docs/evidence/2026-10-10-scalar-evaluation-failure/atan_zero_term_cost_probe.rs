use hyperreal::{Computable, Rational};
use serde_json::json;
use std::{hint::black_box, time::Instant};

fn accumulate(mut checksum: u64, text: &str) -> u64 {
    for byte in text.bytes() {
        checksum = checksum.wrapping_mul(31).wrapping_add(u64::from(byte));
    }
    checksum.wrapping_mul(31).wrapping_add(255)
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let case = &args[0];
    let count: usize = args[1].parse().unwrap();
    let (bits, precision, reciprocal, dense, cached_roots): (usize, i32, bool, bool, usize) =
        match case.as_str() {
            "small_integer" => (3, -128, false, false, 0),
            "small_reciprocal" => (3, -128, true, false, 0),
            "medium_integer" => (65_537, 0, false, false, 0),
            "medium_reciprocal" => (65_537, 0, true, false, 0),
            "wide_integer" => (1_000_001, 0, false, false, 0),
            "wide_reciprocal" => (1_000_001, 0, true, false, 0),
            "dense_integer" => (1_000_000, 0, false, true, 0),
            "dense_reciprocal" => (1_000_000, 0, true, true, 0),
            "cached_integer_one" => (1_000_001, 0, false, false, 1),
            "cached_reciprocal_one" => (1_000_001, 0, true, false, 1),
            "cached_integer_many" => (1_000_001, 0, false, false, 64),
            "cached_reciprocal_many" => (1_000_001, 0, true, false, 64),
            _ => panic!("unknown workload"),
        };
    Computable::pi().approx(-192).unwrap();
    let one = Computable::one().approx(0).unwrap();
    let magnitude = if dense {
        (&one << bits) - 1_i32
    } else {
        (&one << (bits - 1)) + 1_i32
    };
    assert_eq!(magnitude.bits(), bits as u64);
    let make = |index: usize| {
        let negative = cached_roots == 1 || index % 2 == 1;
        let rational = if reciprocal {
            Rational::from_bigint_fraction(
                if negative { -one.clone() } else { one.clone() },
                magnitude.to_biguint().unwrap(),
            )
            .unwrap()
        } else {
            Rational::from_bigint(if negative {
                -magnitude.clone()
            } else {
                magnitude.clone()
            })
        };
        Computable::rational(rational).atan()
    };
    let references: Vec<_> = (0..2)
        .map(|index| make(index).approx(precision).unwrap().to_string())
        .collect();
    let mut expected_checksum = 0_u64;
    let mut expected_bytes = 0_usize;
    for index in 0..count {
        let reference = &references[if cached_roots == 1 { 0 } else { index % 2 }];
        expected_checksum = accumulate(expected_checksum, reference);
        expected_bytes += reference.len();
    }
    // Each uncached iteration receives an independently coarse-warmed root.
    // Completed-cache controls retain either one negative root or 64 alternating
    // signed roots; both layouts remain separate frozen workloads.
    let values: Vec<_> = (0..if cached_roots == 0 {
        count
    } else {
        cached_roots
    })
        .map(|index| {
            let value = make(index);
            value
                .approx(if cached_roots == 0 { 4 } else { precision })
                .unwrap();
            value
        })
        .collect();
    let mut checksum = 0_u64;
    let mut output_bytes = 0_usize;
    let start = Instant::now();
    for index in 0..count {
        let answer = black_box(values[index % values.len()].approx(precision).unwrap());
        let text = answer.to_string();
        output_bytes += text.len();
        checksum = accumulate(checksum, &text);
    }
    let elapsed = start.elapsed().as_nanos();
    assert_eq!(checksum, expected_checksum);
    assert_eq!(output_bytes, expected_bytes);
    println!(
        "{}",
        json!({"case":case,"iterations":count,"bits":bits,"precision":precision,
        "reciprocal":reciprocal,"dense":dense,"cached_roots":cached_roots,
        "elapsed_ns":elapsed.to_string(),"checksum":checksum,"output_bytes":output_bytes,"references":references})
    );
}
