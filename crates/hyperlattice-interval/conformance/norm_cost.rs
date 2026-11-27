//! Completed norm/measure/margin queries with independent exact expectations.
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

/// Check vector norms, square-matrix norms, logarithmic measures and margins.
/// The four vector lanes have absolute-value ranges `[0,2]`, `[3,4]`, `[3,5]`, `[0,2]`,
/// giving norm ranges `[6,13]` and `[3,5]`. Matrix diagonal `[-8,-7]` and off-diagonal
/// `[-1,2]` yield induced norms `[7,14]`, signed measures `[-8,-1]`, dominance margins
/// `[1,8]` and contraction margins `[-13,-6]`. Negative contraction margins are valid
/// completed query results; this fixture does not assert a contraction certificate.
/// I-A is checked entrywise, with diagonal `[8,9]` and off-diagonal `[-2,1]`.
///
/// After these literal preflight checks, warm each of eleven queries 64 times and
/// record 20 batches of 128 materialized/dropped results. Each query calls the context
/// factory; the timer includes counter setup/reads and output drops, excludes printing,
/// and asserts zero allocation requests for primitive families. Inline vector/matrix
/// sizes exclude backend heap storage. The action labels are internal fixed choices;
/// an unrecognized one is an unreachable-branch panic.
fn family<I: IntervalArithmetic + core::fmt::Debug + PartialEq>(
    name: &str,
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
    primitive: bool,
) {
    let vector =
        IntervalVector::new([scalar(-2, 1), scalar(3, 4), scalar(-5, -3), scalar(0, 2)]).unwrap();
    let matrix = IntervalMatrix::new(core::array::from_fn::<_, 4, _>(|r| {
        core::array::from_fn::<_, 4, _>(|c| {
            if r == c {
                scalar(-8, -7)
            } else {
                scalar(-1, 2)
            }
        })
    }))
    .unwrap();
    assert_eq!(vector.norm_one(&mut context()).unwrap(), scalar(6, 13));
    assert_eq!(vector.norm_infinity(&mut context()).unwrap(), scalar(3, 5));
    assert_eq!(matrix.norm_one(&mut context()).unwrap(), scalar(7, 14));
    assert_eq!(matrix.norm_infinity(&mut context()).unwrap(), scalar(7, 14));
    assert_eq!(matrix.measure_one(&mut context()).unwrap(), scalar(-8, -1));
    assert_eq!(
        matrix.measure_infinity(&mut context()).unwrap(),
        scalar(-8, -1)
    );
    let expected = IntervalVector::new(core::array::from_fn(|_| scalar(1, 8))).unwrap();
    assert_eq!(
        matrix.row_dominance_margins(&mut context()).unwrap(),
        expected
    );
    assert_eq!(
        matrix.column_dominance_margins(&mut context()).unwrap(),
        expected
    );
    let remainder = matrix.identity_minus(&mut context()).unwrap();
    for r in 0..4 {
        for c in 0..4 {
            assert_eq!(
                remainder.rows().unwrap()[r][c],
                if r == c { scalar(8, 9) } else { scalar(-2, 1) }
            );
        }
    }
    assert_eq!(
        matrix.contraction_margin_one(&mut context()).unwrap(),
        scalar(-13, -6)
    );
    assert_eq!(
        matrix.contraction_margin_infinity(&mut context()).unwrap(),
        scalar(-13, -6)
    );
    eprintln!(
        "{name},vector_inline_bytes={},matrix_inline_bytes={}",
        std::mem::size_of_val(&vector),
        std::mem::size_of_val(&matrix)
    );
    for action in [
        "vector_one",
        "vector_infinity",
        "matrix_one",
        "matrix_infinity",
        "measure_one",
        "measure_infinity",
        "row_margins",
        "column_margins",
        "identity_minus",
        "contraction_one",
        "contraction_infinity",
    ] {
        let mut operation = || match action {
            "vector_one" => {
                black_box(black_box(&vector).norm_one(&mut context()).unwrap());
            }
            "vector_infinity" => {
                black_box(black_box(&vector).norm_infinity(&mut context()).unwrap());
            }
            "matrix_one" => {
                black_box(black_box(&matrix).norm_one(&mut context()).unwrap());
            }
            "matrix_infinity" => {
                black_box(black_box(&matrix).norm_infinity(&mut context()).unwrap());
            }
            "measure_one" => {
                black_box(black_box(&matrix).measure_one(&mut context()).unwrap());
            }
            "measure_infinity" => {
                black_box(black_box(&matrix).measure_infinity(&mut context()).unwrap());
            }
            "row_margins" => {
                black_box(
                    black_box(&matrix)
                        .row_dominance_margins(&mut context())
                        .unwrap(),
                );
            }
            "column_margins" => {
                black_box(
                    black_box(&matrix)
                        .column_dominance_margins(&mut context())
                        .unwrap(),
                );
            }
            "identity_minus" => {
                black_box(black_box(&matrix).identity_minus(&mut context()).unwrap());
            }
            "contraction_one" => {
                black_box(
                    black_box(&matrix)
                        .contraction_margin_one(&mut context())
                        .unwrap(),
                );
            }
            "contraction_infinity" => {
                black_box(
                    black_box(&matrix)
                        .contraction_margin_infinity(&mut context())
                        .unwrap(),
                );
            }
            _ => unreachable!(),
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
