//! Completed runtime affine queries/restrictions, with bounded inline storage.
use hyperlattice_interval::hyperinterval::{
    DyadicBudget, DyadicInterval, F64Enclosure, FixedInterval, RationalInterval,
};
use hyperlattice_interval::{
    AffineChain, AffineMap, AffineSource, IntervalArithmetic, IntervalMatrix, IntervalVector,
};
use std::{hint::black_box, time::Instant};
mod allocations;

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
