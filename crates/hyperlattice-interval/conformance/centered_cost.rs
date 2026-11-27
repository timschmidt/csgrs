//! Retained authored definitions and complete endpoint projections.
//!
//! Fixtures and preflight checks precede sampling. Results are materialized and
//! consumed/dropped in each action, except explicit ownership cycles that retain the
//! same source. Reported bytes count cumulative allocator requests, not live/peak
//! memory; inline sizes exclude heap payloads. The global counter assumes no nested
//! measurement or unrelated concurrent allocations. Errors and failed assertions
//! panic rather than becoming successful or failed timing rows. Printed rows cover
//! completed batches only; the program installs no timeout or cancellation bound.

use hyperlattice_interval::hyperinterval::{
    DyadicBudget, DyadicInterval, F32Enclosure, F64Enclosure, FixedInterval, RationalInterval,
};
use hyperlattice_interval::{
    CenteredMatrix, CenteredVector, IntervalArithmetic, IntervalMatrix, IntervalVector,
};
use std::{hint::black_box, time::Instant};
/// Process-global allocation-request observer shared by the conformance executables.
/// Requires a single nonnested scope; cumulative requested bytes are not peak memory.
mod allocations;

/// Check authored boxes with center i-3 and radius i mod 3 for lanes i=0..N.
/// Literal endpoints are center±radius. Also check a 2x3 authored matrix whose every
/// entry has center three, radius one and enclosure `[2,4]`. The explicit subtract/add/
/// hull path shares native arithmetic; agreement alone is not an independent oracle.
/// Configured N=4 or 64 keeps index casts and fixture endpoint arithmetic representable.
/// Print inline sizes for carriers, component pairs and translated definitions;
/// these omit any backend heap payloads.
///
/// Six numeric action labels identify completed workloads:
/// - 0: move the same owned center/radius pair into a validated definition and back.
/// - 1: project the retained authored vector to endpoint intervals.
/// - 2: explicitly compute center-radius, center+radius and their hull in one context.
/// - 3: project the fixed 2x3 authored matrix, independent of the reported vector N.
/// - 4: construct and drop an exact translated affine source from the authored form.
/// - 5: project the already prepared translated source.
///
/// Each action warms 32 times, then records 20 batches of 128 calls. Unlike the other
/// cost programs, the timer starts inside allocations::measure, so counter reset/
/// enable/read overhead is excluded from elapsed time, while clock calls lie inside
/// the allocation scope. Context construction and output drops remain inside the
/// action. Owned components survive action zero and are checked after all samples;
/// no clone occurs inside that ownership cycle. Primitive cases assert zero requests
/// for every action. Printing and setup clones are outside measurement.
fn family<I: IntervalArithmetic + PartialEq + core::fmt::Debug, const N: usize>(
    name: &str,
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
    primitive: bool,
) {
    let center =
        IntervalVector::new(core::array::from_fn(|i| scalar(i as i32 - 3, i as i32 - 3))).unwrap();
    let radius = IntervalVector::new(core::array::from_fn(|i| {
        scalar((i % 3) as i32, (i % 3) as i32)
    }))
    .unwrap();
    let form = CenteredVector::<I, N>::new(center.clone(), radius.clone()).unwrap();
    let expected = IntervalVector::new(core::array::from_fn(|i| {
        scalar(i as i32 - 3 - (i % 3) as i32, i as i32 - 3 + (i % 3) as i32)
    }))
    .unwrap();
    assert_eq!(form.enclosure(&mut context()).unwrap(), expected);
    let mut reference_context = context();
    let lower = center.sub(&radius, &mut reference_context).unwrap();
    let upper = center.add(&radius, &mut reference_context).unwrap();
    assert_eq!(
        lower.hull(&upper, &mut reference_context).unwrap(),
        expected
    );
    let matrix = CenteredMatrix::new(
        IntervalMatrix::new(core::array::from_fn(|_| {
            core::array::from_fn(|_| scalar(3, 3))
        }))
        .unwrap(),
        IntervalMatrix::new(core::array::from_fn(|_| {
            core::array::from_fn(|_| scalar(1, 1))
        }))
        .unwrap(),
    )
    .unwrap();
    let expected_matrix = IntervalMatrix::<I, 2, 3>::new(core::array::from_fn(|_| {
        core::array::from_fn(|_| scalar(2, 4))
    }))
    .unwrap();
    assert_eq!(matrix.enclosure(&mut context()).unwrap(), expected_matrix);
    eprintln!(
        "{name},{N},vector_bytes={},definition_bytes={},pair_bytes={},matrix_definition_bytes={}",
        core::mem::size_of_val(&center),
        core::mem::size_of_val(&form),
        core::mem::size_of::<(IntervalVector<I, N>, IntervalVector<I, N>)>(),
        core::mem::size_of_val(&matrix)
    );
    let mut owned = Some((center.clone(), radius.clone()));
    let translated = form.affine_source(&mut context()).unwrap();
    assert_eq!(translated.enclosure(&mut context()).unwrap(), expected);
    eprintln!(
        "{name},{N},translated_bytes={}",
        core::mem::size_of_val(&translated)
    );
    for action in 0..6 {
        let mut execute = || match action {
            0 => {
                let (c, r) = black_box(owned.take().unwrap());
                let form = CenteredVector::new(c, r).unwrap();
                owned = Some(black_box(form.into_parts().unwrap()));
            }
            1 => {
                black_box(black_box(&form).enclosure(&mut context()).unwrap());
            }
            2 => {
                let (c, r) = (black_box(&center), black_box(&radius));
                let mut context = context();
                let low = c.sub(r, &mut context).unwrap();
                let high = c.add(r, &mut context).unwrap();
                black_box(low.hull(&high, &mut context).unwrap());
            }
            4 => {
                black_box(black_box(&form).affine_source(&mut context()).unwrap());
            }
            5 => {
                black_box(black_box(&translated).enclosure(&mut context()).unwrap());
            }
            _ => {
                black_box(black_box(&matrix).enclosure(&mut context()).unwrap());
            }
        };
        for _ in 0..32 {
            execute();
        }
        for sample in 0..20 {
            let (elapsed, calls, bytes) = allocations::measure(|| {
                let start = Instant::now();
                for _ in 0..128 {
                    execute();
                }
                start.elapsed().as_nanos()
            });
            if primitive {
                assert_eq!((calls, bytes), (0, 0));
            }
            println!("{name},{N},{action},{sample},128,{elapsed},{calls},{bytes}");
        }
    }
    assert_eq!(owned.unwrap(), form.into_parts().unwrap());
}

/// Emit the centered-cost CSV header and run N=4 and 64 for six scalar families.
/// The four primitive families are i32 Q16, i64 Q32, f32 and f64; all fixture integers
/// are exactly representable. Rational and dyadic cases retain exact integer inputs;
/// each dyadic query receives a fresh 256-bit/1,000,000-work context. Unsupported
/// floating environments, failed native operations or assertions panic. Reported N
/// selects vector size; each matrix action still uses the same 2x3 shape.
fn main() {
    println!("family,dimension,action,sample,repetitions,nanoseconds,allocations,bytes");
    family::<_, 4>(
        "i32q16",
        |a, b| FixedInterval::<i32, 16>::new(a << 16, b << 16).unwrap(),
        || (),
        true,
    );
    family::<_, 64>(
        "i32q16",
        |a, b| FixedInterval::<i32, 16>::new(a << 16, b << 16).unwrap(),
        || (),
        true,
    );
    family::<_, 4>(
        "i64q32",
        |a, b| FixedInterval::<i64, 32>::new(i64::from(a) << 32, i64::from(b) << 32).unwrap(),
        || (),
        true,
    );
    family::<_, 64>(
        "i64q32",
        |a, b| FixedInterval::<i64, 32>::new(i64::from(a) << 32, i64::from(b) << 32).unwrap(),
        || (),
        true,
    );
    family::<_, 4>(
        "f32",
        |a, b| F32Enclosure::new(a as f32, b as f32).unwrap(),
        || (),
        true,
    );
    family::<_, 64>(
        "f32",
        |a, b| F32Enclosure::new(a as f32, b as f32).unwrap(),
        || (),
        true,
    );
    family::<_, 4>(
        "f64",
        |a, b| F64Enclosure::new(f64::from(a), f64::from(b)).unwrap(),
        || (),
        true,
    );
    family::<_, 64>(
        "f64",
        |a, b| F64Enclosure::new(f64::from(a), f64::from(b)).unwrap(),
        || (),
        true,
    );
    family::<_, 4>(
        "rational",
        |a, b| RationalInterval::new(a.into(), b.into()).unwrap(),
        || (),
        false,
    );
    family::<_, 64>(
        "rational",
        |a, b| RationalInterval::new(a.into(), b.into()).unwrap(),
        || (),
        false,
    );
    family::<_, 4>(
        "dyadic",
        |a, b| DyadicInterval::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(256, 1_000_000),
        false,
    );
    family::<_, 64>(
        "dyadic",
        |a, b| DyadicInterval::new(a.into(), b.into(), 0).unwrap(),
        || DyadicBudget::new(256, 1_000_000),
        false,
    );
}
