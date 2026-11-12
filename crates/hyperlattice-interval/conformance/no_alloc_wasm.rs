//! Clean no_std cdylib consumer: linking succeeds without a global allocator.
#![no_std]
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{AffineMap, IntervalMatrix, IntervalVector};

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
