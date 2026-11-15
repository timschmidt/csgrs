#![cfg(feature = "dyadic")]
use hyperlattice_interval::{
    IntervalVector,
    hyperinterval::{DyadicBudget, DyadicInterval},
};
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
#[test]
fn normalized_product_extremes_feed_dot_and_scale() {
    for (left, right, expected) in [
        (d(1, 2, i32::MAX), d(1, 1, 1), d(2, 4, i32::MAX)),
        (
            d(-1, 2, -1_073_741_824),
            d(-1, 2, -1_073_741_825),
            d(-1, 2, i32::MIN),
        ),
    ] {
        let mut budget = DyadicBudget::new(4096, 1_000_000_000);
        let a = IntervalVector::new([left]).unwrap();
        let b = IntervalVector::new([right]).unwrap();
        let output = a.dot(&b, &mut budget).unwrap();
        assert_eq!(output, expected);
        let generated = IntervalVector::new([output]).unwrap();
        let scaled = generated.scale(&d(1, 1, 0), &mut budget).unwrap();
        assert_eq!(scaled.lanes().unwrap(), &[expected]);
    }
}
