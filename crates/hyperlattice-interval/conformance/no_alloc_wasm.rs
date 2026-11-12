//! Clean no_std cdylib consumer: linking succeeds without a global allocator.
#![no_std]
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{AffineChain, AffineMap, IntervalMatrix, IntervalVector};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn affine_correlation_probe(input: i32) -> i32 {
    type I = FixedInterval<i32, 0>;
    let s = input.clamp(-256, 256);
    let point = |x| I::point(x).unwrap();
    let duplicate = AffineMap::new(
        IntervalMatrix::new([[point(2)], [point(2)]]).unwrap(),
        IntervalVector::new([point(3), point(-4)]).unwrap(),
    )
    .unwrap();
    let difference = AffineMap::new(
        IntervalMatrix::new([[point(1), point(-1)]]).unwrap(),
        IntervalVector::new([point(0)]).unwrap(),
    )
    .unwrap();
    let source = IntervalVector::new([I::new(s, s + 1).unwrap()]).unwrap();
    let image = difference.image(duplicate.image(source));
    let bounds = image.enclosure(&mut ()).unwrap().lanes().unwrap()[0]
        .bounds()
        .unwrap();
    assert_eq!(bounds, (7, 7));
    bounds.0
}

#[unsafe(no_mangle)]
pub extern "C" fn interval_affine_probe(input: i32) -> i32 {
    type I = FixedInterval<i32, 0>;
    let s = input.clamp(-256, 256);
    let point = |x| I::point(x).unwrap();
    let a = IntervalMatrix::new([[point(2), point(-1)], [point(1), point(1)]]).unwrap();
    let x = IntervalVector::new([I::new(s, s + 1).unwrap(), I::new(-1, 2).unwrap()]).unwrap();
    let b = IntervalVector::new([point(3), point(-4)]).unwrap();
    let result = a.affine_enclosure(&x, &b, &mut ()).unwrap();
    let [(lo0, hi0), (lo1, hi1)] = result.into_lanes().unwrap().map(|i| i.bounds().unwrap());
    assert_eq!((lo0, hi0, lo1, hi1), (2 * s + 1, 2 * s + 6, s - 5, s - 1));
    lo0 + 3 * hi0 + 5 * lo1 + 7 * hi1
}

#[unsafe(no_mangle)]
pub extern "C" fn affine_chain_probe(input: i32, depth: u32) -> i32 {
    type I = FixedInterval<i32, 0>;
    let p = |n| I::point(n).unwrap();
    let s = input.clamp(-256, 256);
    let depth = (depth % 33) as i32;
    let source = IntervalVector::new([I::new(s, s + 1).unwrap(), I::new(-1, 2).unwrap()]).unwrap();
    let mut chain = AffineChain::<_, _, 2, 32>::new(source);
    for _ in 0..depth {
        chain
            .push(
                AffineMap::new(
                    IntervalMatrix::new([[p(1), p(1)], [p(0), p(1)]]).unwrap(),
                    IntervalVector::new([p(1), p(-1)]).unwrap(),
                )
                .unwrap(),
            )
            .unwrap();
    }
    let result = chain.enclosure(&mut ()).unwrap();
    let [(lo0, hi0), (lo1, hi1)] = result.into_lanes().unwrap().map(|i| i.bounds().unwrap());
    let offset = depth - depth * (depth - 1) / 2;
    assert_eq!(
        (lo0, hi0, lo1, hi1),
        (
            s - depth + offset,
            s + 1 + 2 * depth + offset,
            -1 - depth,
            2 - depth
        )
    );
    lo0 + 3 * hi0 + 5 * lo1 + 7 * hi1
}

#[unsafe(no_mangle)]
pub extern "C" fn interval_norm_probe(input: i32) -> i32 {
    type I = FixedInterval<i32, 0>;
    let s = input.clamp(2, 256);
    let p = |a, b| I::new(a, b).unwrap();
    let matrix =
        IntervalMatrix::new([[p(-s - 2, -s - 2), p(1, 2)], [p(-1, 1), p(-s - 1, -s - 1)]]).unwrap();
    let one = matrix.norm_one(&mut ()).unwrap().bounds().unwrap();
    let infinity = matrix.norm_infinity(&mut ()).unwrap().bounds().unwrap();
    let measure_one = matrix.measure_one(&mut ()).unwrap().bounds().unwrap();
    let measure_infinity = matrix.measure_infinity(&mut ()).unwrap().bounds().unwrap();
    let rows = matrix.row_dominance_margins(&mut ()).unwrap();
    let columns = matrix.column_dominance_margins(&mut ()).unwrap();
    assert_eq!(one, (s + 2, s + 3));
    assert_eq!(infinity, (s + 3, s + 4));
    assert_eq!(measure_one, (-s, 1 - s));
    assert_eq!(measure_infinity, (-s - 1, -s));
    assert_eq!(
        rows.norm_one(&mut ()).unwrap().bounds(),
        Some((2 * s, 2 * s + 2))
    );
    assert_eq!(
        columns.norm_infinity(&mut ()).unwrap().bounds(),
        Some((s + 1, s + 2))
    );
    one.0
        + 3 * one.1
        + 5 * infinity.0
        + 7 * infinity.1
        + 11 * measure_one.0
        + 13 * measure_one.1
        + 17 * measure_infinity.0
        + 19 * measure_infinity.1
}
