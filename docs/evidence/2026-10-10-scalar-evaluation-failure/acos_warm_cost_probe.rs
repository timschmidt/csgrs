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
    let (bits, precision, negative, cached) = match case.as_str() {
        "small_positive" => (8, -64, false, false),
        "small_negative" => (8, -64, true, false),
        "mid_positive" => (4097, 0, false, false),
        "mid_negative" => (4097, 0, true, false),
        "wide_positive" => (1_000_001, 0, false, false),
        "wide_negative" => (1_000_001, 0, true, false),
        "cached_positive" => (1_000_001, 0, false, true),
        "cached_negative" => (1_000_001, 0, true, true),
        _ => panic!("unknown workload"),
    };
    Computable::pi().approx(-96).unwrap();
    let denominator = Computable::one().approx(0).unwrap() << (bits - 1);
    let numerator = &denominator - 1_i32;
    let numerator = if negative { -numerator } else { numerator };
    let rational =
        Rational::from_bigint_fraction(numerator, denominator.to_biguint().unwrap()).unwrap();
    let make = || Computable::rational(rational.clone()).acos();
    let reference_root = make();
    let reference = reference_root.approx(precision).unwrap().to_string();
    let expected_checksum = (0..count).fold(0_u64, |sum, _| accumulate(sum, &reference));
    // Warm each independent root only at coarse precision before timing a
    // finer miss. Cached controls instead reuse one fully completed root.
    let values: Vec<_> = if cached {
        vec![reference_root]
    } else {
        (0..count)
            .map(|_| {
                let value = make();
                value.approx(1).unwrap();
                value
            })
            .collect()
    };
    let mut checksum = 0_u64;
    let mut output_bytes = 0_usize;
    let start = Instant::now();
    for index in 0..count {
        let answer = black_box(
            values[if cached { 0 } else { index }]
                .approx(precision)
                .unwrap(),
        );
        let text = answer.to_string();
        output_bytes += text.len();
        checksum = accumulate(checksum, &text);
    }
    let elapsed = start.elapsed().as_nanos();
    assert_eq!(checksum, expected_checksum);
    assert_eq!(output_bytes, reference.len() * count);
    println!(
        "{}",
        json!({"case":case,"iterations":count,"bits":bits,
        "precision":precision,"negative":negative,"cached":cached,
        "elapsed_ns":elapsed.to_string(),"checksum":checksum,
        "output_bytes":output_bytes,"reference":reference})
    );
}
