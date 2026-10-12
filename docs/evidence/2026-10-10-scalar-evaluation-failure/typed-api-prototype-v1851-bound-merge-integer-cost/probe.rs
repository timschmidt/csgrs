use hyperreal::{Computable, Rational};
use serde_json::json;
use std::{hint::black_box, time::Instant};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let case = &args[0];
    let count: usize = args[1].parse().unwrap();
    let (bits, precision, halfway) = match case.as_str() {
        "small_fine" => (8, -128, false),
        "small_coarse" => (8, 1, false),
        "small_identity" => (65, 0, false),
        "wide_coarse" => (1_000_001, 1_000_000, false),
        "wide_gap" => (1_000_001, i32::MAX, false),
        "wide_halfway" => (1_000_001, 1_000_001, true),
        "wide_fine" => (65_537, -128, false),
        "wide_identity" => (65_537, 0, false),
        _ => panic!("unknown workload"),
    };
    let zero = Computable::zero().approx(0).unwrap();
    let power = Computable::one().approx(0).unwrap() << (bits - 1);
    let positive =
        Computable::rational(Rational::from_bigint(if halfway { power } else { power + 1 }));
    let negative = positive.clone().negate();
    let values = [positive, negative];
    let expected: Vec<_> = values
        .iter()
        .map(|value| value.approx(precision).unwrap())
        .collect();
    // Construction, lazy initialization, and reference answers are outside the
    // timed loop. Exact integer leaves keep no numeric cache across these calls.
    let expected_checksum = (0..count).fold(0_u64, |sum, index| {
        let answer = &expected[index % 2];
        sum.wrapping_add(answer.bits() * 3 + u64::from(answer < &zero))
    });
    let mut checksum = 0_u64;
    let start = Instant::now();
    for index in 0..count {
        let answer = black_box(values[index % 2].approx(precision).unwrap());
        checksum = checksum.wrapping_add(answer.bits() * 3 + u64::from(answer < zero));
    }
    let elapsed = start.elapsed().as_nanos();
    assert_eq!(checksum, expected_checksum);
    println!(
        "{}",
        json!({"case":case,"iterations":count,"bits":bits,"precision":precision,
            "elapsed_ns":elapsed.to_string(),"checksum":checksum,
            "expected_bits":[expected[0].bits(),expected[1].bits()]}),
    );
}
