//! Focused out-of-line Q30 cost probe; not a firmware cycle/HIL benchmark.

use alumina_foc::{FocError, Q30, Q30Interval};
use hyperinterval::{FixedInterval, IntervalError};
use std::hint::black_box;
use std::time::Instant;

type Output = Result<(i32, i32), u8>;
type Input = [i32; 4];

#[inline(never)]
fn shared(operation: u8, input: Input) -> Output {
    type I = FixedInterval<i32, 30>;
    let [a, b, c, d] = input;
    let x = I::new(a, b).unwrap();
    let y = I::new(c, d).unwrap();
    let result = match operation {
        0 => x.checked_add(&y),
        1 => x.checked_sub(&y),
        2 => x.checked_mul(&y),
        3 => x.checked_div(&y),
        _ => x.square(),
    };
    result
        .map(|value| value.bounds().unwrap())
        .map_err(|error| match error {
            IntervalError::Overflow => 1,
            IntervalError::ContainsZero => 2,
            _ => panic!("unexpected error: {error}"),
        })
}

#[inline(never)]
fn legacy(operation: u8, input: Input) -> Output {
    let [a, b, c, d] = input;
    let x = Q30Interval::new(Q30::from_bits(a), Q30::from_bits(b)).unwrap();
    let y = Q30Interval::new(Q30::from_bits(c), Q30::from_bits(d)).unwrap();
    let result = match operation {
        0 => x.checked_add(y),
        1 => x.checked_sub(y),
        2 => x.checked_mul(y),
        3 => x.checked_div(y),
        _ => x.checked_square(),
    };
    result
        .map(|value| (value.lower().bits(), value.upper().bits()))
        .map_err(|error| match error {
            FocError::Range => 1,
            FocError::DivideByZero => 2,
            _ => panic!("unexpected error: {error:?}"),
        })
}

fn measure(
    operation: u8,
    inputs: &[Input],
    backend: fn(u8, Input) -> Output,
    iterations: usize,
) -> (u128, u64) {
    let start = Instant::now();
    let mut checksum = 0_u64;
    for _ in 0..iterations {
        for input in inputs {
            let (lower, upper) =
                black_box(backend(black_box(operation), black_box(*input))).unwrap();
            checksum = checksum.wrapping_add((lower as u64).rotate_left(17) ^ upper as u64);
        }
    }
    (start.elapsed().as_nanos(), checksum)
}

fn main() {
    let inputs = [
        [1 << 27, 1 << 28, 1 << 29, 3 << 28],
        [-(1 << 28), -(1 << 27), -(3 << 28), -(1 << 29)],
        [-(1 << 28), 1 << 28, 1 << 29, 3 << 28],
        [1 << 29, 1 << 29, 1 << 30, 1 << 30],
    ];
    let names = ["add", "subtract", "multiply", "divide", "square"];
    println!("operation,sample,backend,elapsed_ns,completed_operations,checksum");
    for (operation, name) in names.into_iter().enumerate() {
        let operation = operation as u8;
        for input in inputs {
            assert_eq!(shared(operation, input), legacy(operation, input));
            assert!(shared(operation, input).is_ok());
        }
        for backend in [legacy as fn(u8, Input) -> Output, shared] {
            black_box(measure(operation, &inputs, backend, 4096));
        }
        for sample in 0..30 {
            let order = if sample % 2 == 0 {
                [
                    ("legacy", legacy as fn(u8, Input) -> Output),
                    ("shared", shared),
                ]
            } else {
                [
                    ("shared", shared as fn(u8, Input) -> Output),
                    ("legacy", legacy),
                ]
            };
            let mut checksums = Vec::new();
            for (backend_name, backend) in order {
                let (elapsed, checksum) = measure(operation, &inputs, backend, 16_384);
                println!("{name},{sample},{backend_name},{elapsed},65536,{checksum}");
                checksums.push(checksum);
            }
            assert_eq!(checksums[0], checksums[1]);
        }
    }
}
