//! Extreme stored exponents remain usable through Cartesian dot and scale operations.
//!
//! Exact endpoint normalization can repair a temporary product exponent outside i32.
//! These two finite witnesses exercise both directions without materializing the
//! represented power of two or claiming that every such product is representable.

#![cfg(feature = "dyadic")]
use hyperlattice_interval::{
    IntervalVector,
    hyperinterval::{DyadicBudget, DyadicInterval},
};
/// Construct the closed interval `[a*2^e, b*2^e]` and unwrap fixture validation.
/// The constructor may normalize the stored mantissas and exponent.
fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
#[test]
/// Check exponent repair through a one-lane dot product, then reuse its output.
/// At the upper limit, moving one exponent bit into the mantissas yields `[2,4]`
/// at i32::MAX. Below the lower limit, the product hull `[-2,4]` has a common
/// factor two, yielding `[-1,2]` at i32::MIN. Scaling the generated vector by one
/// preserves each expected interval. Each case shares one adequate size/work
/// budget across dot and scale; this does not test exhausted-budget behavior.
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
