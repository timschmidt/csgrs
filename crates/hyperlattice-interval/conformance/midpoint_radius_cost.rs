//! Completed retained decompositions and explicit center/radius projections.
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
use hyperlattice_interval::{IntervalArithmetic, IntervalMatrix, IntervalVector};
use std::{hint::black_box, time::Instant};
/// Process-global allocation-request observer shared by the conformance executables.
/// Requires a single nonnested scope; cumulative requested bytes are not peak memory.
mod allocations;

/// Warm an action 64 times, then print 20 samples of 128 completed invocations.
/// Timing surrounds the allocation wrapper, including its reset/read overhead.
/// The closure determines whether an invocation projects/drops a result or moves a
/// retained definition through an ownership cycle. zero_alloc asserts both counters
/// are zero after the timer; printing occurs afterward. Panics propagate.
fn measure(name: &str, action: &str, zero_alloc: bool, mut operation: impl FnMut()) {
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
        if zero_alloc {
            assert_eq!((calls, bytes), (0, 0));
        }
        println!("{name},{action},{sample},128,{elapsed},{calls},{bytes}");
    }
}
/// Retain four interval lanes and the same entries as a 2x2 matrix.
/// Literal midpoint values are 2,4,1,0 and radii 5,1,2,0, all exactly representable
/// in these fixtures. Check both projections before measuring, and report inline
/// source/form sizes without interpreting those sizes as total memory.
///
/// Two actions repeatedly move source bounds into a decomposition and back through
/// Option, with no clone or projection inside the ownership cycle. They assert zero
/// allocation requests for every backend and verify the returned source afterward.
/// Four projection actions query vector/matrix midpoint/radius separately, create a
/// context per query and drop results; only primitive projections require zero
/// allocations. The setup clones and final ownership checks occur outside measurement.
fn family<I: IntervalArithmetic + PartialEq + core::fmt::Debug>(
    name: &str,
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
    primitive: bool,
) {
    let vector =
        IntervalVector::new([scalar(-3, 7), scalar(3, 5), scalar(-1, 3), scalar(0, 0)]).unwrap();
    let matrix =
        IntervalMatrix::new([[scalar(-3, 7), scalar(3, 5)], [scalar(-1, 3), scalar(0, 0)]])
            .unwrap();
    let v = vector.clone().midpoint_radius();
    let m = matrix.clone().midpoint_radius();
    assert_eq!(
        v.midpoint_enclosure(&mut context()).unwrap(),
        IntervalVector::new([scalar(2, 2), scalar(4, 4), scalar(1, 1), scalar(0, 0)]).unwrap()
    );
    assert_eq!(
        v.radius_enclosure(&mut context()).unwrap(),
        IntervalVector::new([scalar(5, 5), scalar(1, 1), scalar(2, 2), scalar(0, 0)]).unwrap()
    );
    assert_eq!(
        m.midpoint_enclosure(&mut context()).unwrap(),
        IntervalMatrix::new([[scalar(2, 2), scalar(4, 4)], [scalar(1, 1), scalar(0, 0)]]).unwrap()
    );
    assert_eq!(
        m.radius_enclosure(&mut context()).unwrap(),
        IntervalMatrix::new([[scalar(5, 5), scalar(1, 1)], [scalar(2, 2), scalar(0, 0)]]).unwrap()
    );
    eprintln!(
        "{name},source_vector_bytes={},form_vector_bytes={},source_matrix_bytes={},form_matrix_bytes={}",
        core::mem::size_of_val(&vector),
        core::mem::size_of_val(&v),
        core::mem::size_of_val(&matrix),
        core::mem::size_of_val(&m)
    );
    let mut owned = Some(vector);
    measure(name, "vector_retain_reconstruct", true, || {
        let form = black_box(owned.take().unwrap()).midpoint_radius();
        owned = Some(black_box(black_box(form).into_intervals()));
    });
    assert_eq!(owned.unwrap(), v.clone().into_intervals());
    let mut owned = Some(matrix);
    measure(name, "matrix_retain_reconstruct", true, || {
        let form = black_box(owned.take().unwrap()).midpoint_radius();
        owned = Some(black_box(black_box(form).into_intervals()));
    });
    assert_eq!(owned.unwrap(), m.clone().into_intervals());
    measure(name, "vector_center", primitive, || {
        black_box(black_box(&v).midpoint_enclosure(&mut context()).unwrap());
    });
    measure(name, "vector_radius", primitive, || {
        black_box(black_box(&v).radius_enclosure(&mut context()).unwrap());
    });
    measure(name, "matrix_center", primitive, || {
        black_box(black_box(&m).midpoint_enclosure(&mut context()).unwrap());
    });
    measure(name, "matrix_radius", primitive, || {
        black_box(black_box(&m).radius_enclosure(&mut context()).unwrap());
    });
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
