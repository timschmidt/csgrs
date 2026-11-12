//! Completed 2x2 interval products, including materialized outputs and drops.
//! The scalar-loop control is a cost comparison, not an independent oracle.
use hyperlattice_interval::hyperinterval::{
    DyadicBudget, DyadicInterval, F64Enclosure, FixedInterval, RationalInterval,
};
use hyperlattice_interval::{IntervalArithmetic, IntervalLatticeError, IntervalMatrix};
use std::{hint::black_box, time::Instant};
mod allocations;

type M<I> = IntervalMatrix<I, 2, 2>;

#[inline(never)]
fn shared<I: IntervalArithmetic>(
    a: &M<I>,
    b: &M<I>,
    context: &mut I::Context,
) -> Result<M<I>, IntervalLatticeError> {
    a.mul_matrix(b, context)
}

#[inline(never)]
fn scalar_control<I: IntervalArithmetic>(
    a: &M<I>,
    b: &M<I>,
    context: &mut I::Context,
) -> Result<M<I>, IntervalLatticeError> {
    let a = a.rows().unwrap();
    let b = b.rows().unwrap();
    let mut values = core::array::from_fn(|_| core::array::from_fn(|_| I::zero().unwrap()));
    for r in 0..2 {
        for c in 0..2 {
            for (k, row) in b.iter().enumerate() {
                let product = a[r][k].mul(&row[c], context)?;
                values[r][c] = values[r][c].add(&product, context)?;
            }
        }
    }
    M::new(values)
}

fn run<I: IntervalArithmetic + core::fmt::Debug + PartialEq>(
    name: &str,
    a: M<I>,
    b: M<I>,
    mut context: impl FnMut() -> I::Context,
    primitive: bool,
) {
    let expected = scalar_control(&a, &b, &mut context()).unwrap();
    assert_eq!(shared(&a, &b, &mut context()).unwrap(), expected);
    eprintln!("completed output {name}: {expected:?}");
    type Operation<I> = fn(
        &M<I>,
        &M<I>,
        &mut <I as IntervalArithmetic>::Context,
    ) -> Result<M<I>, IntervalLatticeError>;
    let paths: [(&str, Operation<I>); 2] = [
        ("scalar_loop", scalar_control::<I>),
        ("cartesian", shared::<I>),
    ];
    for (_, path) in paths {
        for _ in 0..64 {
            black_box(path(black_box(&a), black_box(&b), &mut context()).unwrap());
        }
    }
    for sample in 0..20 {
        for index in 0..2 {
            let (path, operation) = paths[(index + sample) % 2];
            let start = Instant::now();
            let (_, allocations, bytes) = allocations::measure(|| {
                for _ in 0..128 {
                    black_box(operation(black_box(&a), black_box(&b), &mut context()).unwrap());
                }
            });
            let elapsed = start.elapsed().as_nanos();
            if primitive {
                assert_eq!((allocations, bytes), (0, 0));
            }
            println!("{name},{sample},{path},128,{elapsed},{allocations},{bytes}");
        }
    }
}

fn main() {
    println!("case,sample,path,completed_operations,elapsed_ns,allocation_calls,allocated_bytes");
    let ranges = [[(1, 3), (-2, 1)], [(0, 2), (-4, -1)]];
    let fixed = ranges
        .map(|row| row.map(|(a, b)| FixedInterval::<i32, 16>::new(a << 14, b << 14).unwrap()));
    let matrix = M::new(fixed).unwrap();
    run("fixed_q16", matrix, matrix, || (), true);
    let float = ranges.map(|row| {
        row.map(|(a, b)| F64Enclosure::new(f64::from(a) / 4., f64::from(b) / 4.).unwrap())
    });
    let matrix = M::new(float).unwrap();
    run("strict_f64", matrix, matrix, || (), true);
    let rational =
        ranges.map(|row| row.map(|(a, b)| RationalInterval::new(a.into(), b.into()).unwrap()));
    let matrix = M::new(rational).unwrap();
    run("small_rational", matrix.clone(), matrix, || (), false);
    let wide = RationalInterval::new(
        "12345678901234567890123456789/17".parse().unwrap(),
        "98765432109876543210987654321/19".parse().unwrap(),
    )
    .unwrap();
    let matrix = M::new(core::array::from_fn(|_| {
        core::array::from_fn(|_| wide.clone())
    }))
    .unwrap();
    run("wide_rational", matrix.clone(), matrix, || (), false);
    let dyadic =
        ranges.map(|row| row.map(|(a, b)| DyadicInterval::new(a.into(), b.into(), -2).unwrap()));
    let matrix = M::new(dyadic).unwrap();
    run(
        "dyadic",
        matrix.clone(),
        matrix,
        || DyadicBudget::new(256, 1_000_000),
        false,
    );
}
