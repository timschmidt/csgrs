//! Completed exact box-width and selection queries; no rounded-width decisions.
use hyperlattice_interval::hyperinterval::{
    DyadicBudget, DyadicInterval, F64Enclosure, FixedInterval, RationalInterval,
};
use hyperlattice_interval::{IntervalArithmetic, IntervalVector};
use std::{hint::black_box, time::Instant};
mod allocations;
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
