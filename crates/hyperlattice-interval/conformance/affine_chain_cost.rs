//! Completed runtime affine queries/restrictions, with bounded inline storage.
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
use hyperlattice_interval::{
    AffineChain, AffineMap, AffineSource, IntervalArithmetic, IntervalMatrix, IntervalVector,
};
use std::{hint::black_box, time::Instant};
/// Process-global allocation-request observer shared by the conformance executables.
/// Requires a single nonnested scope; cumulative requested bytes are not peak memory.
mod allocations;

/// Check the independent recurrence, then time projection and owned restriction.
/// For n repeated maps (x,y)->(x+y+1,y-1), y_n=y-n and
/// x_n=x+n*y+n-n(n-1)/2. The fixture source is `[-2,1]` by `[-1,3]`;
/// restricting its original parameters to (0,0) gives (n-n(n-1)/2,-n).
/// Depth is caller-supplied fixture metadata, paired with the constructed definition;
/// arbitrary depths/inputs are not validated by this helper.
///
/// Preflight checks both full and restricted projections. Timed restriction constructs
/// and drops the restricted definition without projecting it. Each action calls the
/// context factory, warms 64 times, then completes 20 batches of 128 calls. Timing
/// includes counter setup/reads, context construction and output destruction, but
/// excludes fixture setup, preflight and printing. Primitive batches require zero
/// allocation requests. Inline image size excludes retained heap payloads.
/// This specialization stores at most 32 maps inline; map count and inline carrier
/// capacity do not bound exact scalar heap storage or total query work.
fn measure<I, S>(
    name: &str,
    depth: i32,
    image: &AffineChain<I, S, 2, 32>,
    scalar: &impl Fn(i32, i32) -> I,
    context: &mut impl FnMut() -> I::Context,
    primitive: bool,
) where
    I: IntervalArithmetic + core::fmt::Debug + PartialEq,
    S: AffineSource<I, 2, Parameters = IntervalVector<I, 2>>,
{
    // For n identical maps (x,y)->(x+y+1,y-1):
    // y_n=y-n; x_n=x+n*y+n-n(n-1)/2. This is independent of the kernel.
    let offset = depth - depth * (depth - 1) / 2;
    let expected = IntervalVector::new([
        scalar(-2 - depth + offset, 1 + 3 * depth + offset),
        scalar(-1 - depth, 3 - depth),
    ])
    .unwrap();
    assert_eq!(image.enclosure(&mut context()).unwrap(), expected);
    let selection = IntervalVector::new([scalar(0, 0), scalar(0, 0)]).unwrap();
    let restricted = image
        .restrict_parameters(&selection, &mut context())
        .unwrap();
    assert_eq!(
        restricted.enclosure(&mut context()).unwrap().lanes(),
        Some(&[scalar(offset, offset), scalar(-depth, -depth)])
    );
    eprintln!(
        "{name},depth={depth},inline_bytes={},enclosure={expected:?}",
        std::mem::size_of_val(image)
    );
    for action in ["enclosure", "restriction"] {
        let mut operation = || {
            if action == "enclosure" {
                black_box(black_box(image).enclosure(&mut context()).unwrap());
            } else {
                black_box(
                    black_box(image)
                        .restrict_parameters(black_box(&selection), &mut context())
                        .unwrap(),
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
            println!("{name},{depth},{action},{sample},128,{elapsed},{calls},{bytes}");
        }
    }
}

/// Build fresh capacity-32 runtime chains at depths zero, one, four, sixteen and 32.
/// Each starts from a cloned common source and receives exactly depth copies of the
/// point map (x,y)->(x+y+1,y-1). Construction/cloning precede measurement. Depth zero
/// still queries or restricts the retained Cartesian source, rather than inventing
/// an identity map. All chain projections use independent recurrence expectations.
fn family<I: IntervalArithmetic + core::fmt::Debug + PartialEq>(
    name: &str,
    scalar: impl Fn(i32, i32) -> I,
    mut context: impl FnMut() -> I::Context,
    primitive: bool,
) {
    let make_map = || {
        AffineMap::new(
            IntervalMatrix::new([[scalar(1, 1), scalar(1, 1)], [scalar(0, 0), scalar(1, 1)]])
                .unwrap(),
            IntervalVector::new([scalar(1, 1), scalar(-1, -1)]).unwrap(),
        )
        .unwrap()
    };
    let source = IntervalVector::new([scalar(-2, 1), scalar(-1, 3)]).unwrap();
    for depth in [0, 1, 4, 16, 32] {
        let mut chain = AffineChain::<_, _, 2, 32>::new(source.clone());
        for _ in 0..depth {
            chain.push(make_map()).unwrap();
        }
        measure(name, depth, &chain, &scalar, &mut context, primitive);
    }
}

/// Emit the CSV header, then run fixed Q16, qualified f64, rational and dyadic cases.
/// The small integer endpoints are exactly representable in every selected family.
/// Primitive cases assert zero allocation requests; dyadic queries receive fresh
/// 256-bit/1,000,000-work contexts. Unsupported float environments or any unexpected
/// native failure panic. CSV goes to stdout and fixture/inline-size evidence to stderr.
fn main() {
    println!(
        "backend,depth,action,sample,completed_operations,elapsed_ns,allocation_calls,allocated_bytes"
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
