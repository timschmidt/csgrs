//! Completed exact box-width and selection queries; no rounded-width decisions.
//!
//! Fixtures and preflight checks precede sampling. Results are materialized and
//! consumed/dropped in each action, except explicit ownership cycles that retain the
//! same source. Reported bytes count cumulative allocator requests, not live/peak
//! memory; inline sizes exclude heap payloads. The global counter assumes no nested
//! measurement or unrelated concurrent allocations. Errors and failed assertions
//! panic rather than becoming successful or failed timing rows. Printed rows cover
//! completed batches only; the program installs no timeout or cancellation bound.

use hyperlattice_interval::hyperinterval::{
    DyadicBudget, DyadicInterval, F64Enclosure, FixedInterval, RationalInterval,
};
use hyperlattice_interval::{IntervalArithmetic, IntervalVector};
use std::{hint::black_box, time::Instant};
/// Process-global allocation-request observer shared by the conformance executables.
/// Requires a single nonnested scope; cumulative requested bytes are not peak memory.
mod allocations;
/// Check widths 10,20,16,0 and selected axis one for a four-lane fixture.
/// Expected widths are constructed through the same backend width API and compared
/// exactly; this checks agreement with literal endpoint differences, not an independent
/// implementation of width arithmetic. Preflight prints source/width-array inline sizes.
///
/// Warm each of widths and widest_axis 64 times, then record 20 batches of 128 calls.
/// Every timed selection still asserts Some(1); timed width arrays are black-boxed
/// and dropped without repeating the full preflight comparison. A context is created
/// per call. Timing includes counter overhead and result drops; printing is outside.
/// Primitive batches assert zero allocation requests.
fn family<I: IntervalArithmetic>(
    name: &str,
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
    primitive: bool,
) {
    let source =
        IntervalVector::new([scalar(-3, 7), scalar(0, 20), scalar(0, 16), scalar(4, 4)]).unwrap();
    assert_eq!(source.widest_axis(&mut context()).unwrap(), Some(1));
    let widths = source.widths(&mut context()).unwrap().unwrap();
    for (w, n) in widths.iter().zip([10, 20, 16, 0]) {
        let expected = scalar(0, n).width(&mut context()).unwrap().unwrap();
        assert_eq!(
            I::compare_widths(w, &expected, &mut context()).unwrap(),
            core::cmp::Ordering::Equal
        );
    }
    eprintln!(
        "{name},source_inline_bytes={},width_array_inline_bytes={}",
        std::mem::size_of_val(&source),
        std::mem::size_of_val(&widths)
    );
    for action in ["widths", "widest_axis"] {
        let mut operation = || {
            if action == "widths" {
                black_box(black_box(&source).widths(&mut context()).unwrap());
            } else {
                assert_eq!(
                    black_box(black_box(&source).widest_axis(&mut context()).unwrap()),
                    Some(1)
                );
            }
        };
        for _ in 0..64 {
            operation();
        }
        for sample in 0..20 {
            let start = Instant::now();
            let (_, calls, bytes) = allocations::measure(|| {
                for _ in 0..128 {
                    operation();
                }
            });
            let elapsed = start.elapsed().as_nanos();
            if primitive {
                assert_eq!((calls, bytes), (0, 0));
            }
            println!("{name},{action},{sample},128,{elapsed},{calls},{bytes}");
        }
    }
}
/// Emit the CSV header, then run fixed Q16, qualified f64, rational and dyadic cases.
/// The small integer endpoints are exactly representable in every selected family.
/// Primitive cases assert zero allocation requests; dyadic queries receive fresh
/// 256-bit/1,000,000-work contexts. Unsupported float environments or any unexpected
/// native failure panic. CSV goes to stdout and fixture/inline-size evidence to stderr.
fn main() {
    println!(
        "backend,action,sample,completed_operations,elapsed_ns,allocation_calls,allocated_bytes"
    );
    family(
        "fixed_q16",
        |a, b| FixedInterval::<i32, 16>::new(a << 16, b << 16).unwrap(),
        || (),
        true,
    );
    family(
        "strict_f64",
        |a, b| F64Enclosure::new(a.into(), b.into()).unwrap(),
        || (),
        true,
    );
    family(
        "rational",
        |a, b| RationalInterval::new(a.into(), b.into()).unwrap(),
        || (),
        false,
    );
    family(
        "dyadic",
        |a, b| DyadicInterval::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(256, 1_000_000),
        false,
    );
}
