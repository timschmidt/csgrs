//! Completed 2x2 interval products, including materialized outputs and drops.
//! The scalar-loop control is a cost comparison, not an independent oracle.
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
use hyperlattice_interval::{IntervalArithmetic, IntervalLatticeError, IntervalMatrix};
use std::{hint::black_box, time::Instant};
/// Process-global allocation-request observer shared by the conformance executables.
/// Requires a single nonnested scope; cumulative requested bytes are not peak memory.
mod allocations;

/// Square two-by-two Cartesian carrier shared by the native and scalar-loop paths.
type M<I> = IntervalMatrix<I, 2, 2>;

#[inline(never)]
/// Forward one native matrix product through the caller's context in a separate frame.
/// The production kernel owns validation, empty semantics, ordered dot products and
/// failure propagation; this wrapper supplies no independent arithmetic expectation.
fn shared<I: IntervalArithmetic>(
    a: &M<I>,
    b: &M<I>,
    context: &mut I::Context,
) -> Result<M<I>, IntervalLatticeError> {
    a.mul_matrix(b, context)
}

#[inline(never)]
/// Evaluate the nonempty fixture product with explicit row/column/shared-index loops.
/// Initialize all four zeros before multiplication, then multiply and add in increasing
/// shared index. This intentionally uses the same scalar operations as the native
/// path and is a cost control, not an independent correctness oracle. It assumes
/// nonempty rows and an available scalar zero (unwrap panics otherwise); multiply,
/// add and final carrier-construction errors propagate through Result.
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

/// Compare native output with the scalar-loop control, then measure both paths.
/// Print the completed expected carrier on stderr. Warm each path 64 times, alternate
/// which runs first on successive samples, and record 20 batches of 128 calls per
/// path. Each call constructs a context and materializes/black-boxes/drops its output.
/// The timer includes allocation-counter setup/reads but excludes fixtures, initial
/// equality and printing; primitive batches assert zero allocation requests. Timed
/// iterations require success without repeating full output equality. Identical input
/// references still denote independent interval operands, not correlated matrix squaring.
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
    /// Common function-pointer signature so both multiplication paths use the same
    /// call boundary and mutable arithmetic context in the sampling loop.
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

/// Run five cases after printing the common CSV header.
/// Fixed Q16, qualified f64 and dyadic use the listed small ranges divided by four;
/// small_rational uses the unscaled integer ranges, so cross-family timings do not
/// all measure identical operands. wide_rational fills every entry with one large
/// positive rational interval. Each product interprets its two matrix operands
/// independently. Fixed/float cases require zero allocation requests; dyadic calls
/// receive fresh 256-bit/1,000,000-work contexts. Fixture/conversion failures panic.
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
