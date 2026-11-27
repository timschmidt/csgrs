//! Completed-action timing and allocation requests for the two FOC providers.
//!
//! Full outputs are compared before timing; digests keep all endpoints observable
//! inside each measured workload. These finite fixture measurements do not prove
//! independent arithmetic correctness, general zero allocation or firmware timing.
use alumina_foc::FocError;
use interval_foc_conformance::samples::{self, ACTIONS, Action, Output, Sample};
use std::{hint::black_box, time::Instant};
#[path = "../../../allocations.rs"]
mod allocations;

/// Preflight 64 inputs for five actions, then emit twenty samples per action/provider pair.
///
/// Precompute inputs and compare unwrapped full outputs outside the timer/counter.
/// For each action, finish native before shared; provider order does not alternate.
/// Warm each provider with all 64 inputs, then measure eight passes (512 action
/// executions) per sample. Chain is one reported action but executes five transforms.
/// Each sample resets the digest and includes endpoint hashing in both measurements.
///
/// The timer surrounds allocations::measure, so counter reset/enable/disable/read
/// overhead is timed. Both clock calls and CSV printing are outside allocation
/// recording. Rows report total nanoseconds, request count/bytes and a hexadecimal
/// digest. Any operation error, unequal preflight result or recorded allocation
/// panics. CLI arguments are unused; no statistical conclusion or timeout is provided.
fn main() {
    let inputs = core::array::from_fn::<_, 64, _>(|i| samples::sample(i as u32));
    for action in ACTIONS {
        for &input in &inputs {
            assert_eq!(
                samples::shared(action, input).unwrap(),
                samples::native(action, input).unwrap()
            );
        }
    }
    println!(
        "implementation,action,sample,completed_operations,elapsed_ns,allocation_calls,allocated_bytes,checksum"
    );
    /// Common function-pointer signature retaining complete wrappers and typed failures.
    type Provider = fn(Action, Sample) -> Result<Output, FocError>;
    for action in ACTIONS {
        for (name, provider) in [
            ("native", samples::native as Provider),
            ("shared", samples::shared as Provider),
        ] {
            for &input in &inputs {
                black_box(provider(black_box(action), black_box(input)).unwrap());
            }
            for sample in 0..20 {
                let mut hash = 0xcbf29ce484222325;
                let start = Instant::now();
                let (_, calls, bytes) = allocations::measure(|| {
                    for _ in 0..8 {
                        for &input in &inputs {
                            hash =
                                black_box(provider(black_box(action), black_box(input)).unwrap())
                                    .digest(hash);
                        }
                    }
                });
                let elapsed = start.elapsed().as_nanos();
                assert_eq!((calls, bytes), (0, 0));
                println!("{name},{action:?},{sample},512,{elapsed},{calls},{bytes},{hash:016x}");
            }
        }
    }
}
