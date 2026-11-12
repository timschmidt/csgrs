//! Independent directed MPFR arctangent and GMP output-grid conversion.
use hyperinterval::DyadicInterval;
use rug::{
    Float, Rational,
    float::{Constant, Round},
};
#[path = "grid_oracle.rs"]
mod grid_oracle;
pub use grid_oracle::q;
pub fn finite(result: &DyadicInterval, lower: Rational, upper: Rational, precision: i32) {
    let (lo, hi) = grid_oracle::bounds(lower, upper, precision);
    let (a, b, e) = result.bounds().unwrap();
    assert_eq!(q(a, e), lo);
    assert_eq!(q(b, e), hi);
}
pub fn verify(result: &DyadicInterval, a: &Rational, b: &Rational, precision: i32) {
    crate::mpfr_cache::cleanup_on_thread_exit();
    let (mut lo, _) = Float::with_val_round(4096, a, Round::Down);
    let (mut hi, _) = Float::with_val_round(4096, b, Round::Up);
    lo.atan_round(Round::Down);
    hi.atan_round(Round::Up);
    finite(
        result,
        lo.to_rational().unwrap(),
        hi.to_rational().unwrap(),
        precision,
    );
}
pub fn half_pi() -> (Rational, Rational) {
    crate::mpfr_cache::cleanup_on_thread_exit();
    let (lo, _) = Float::with_val_round(4096, Constant::Pi, Round::Down);
    let (hi, _) = Float::with_val_round(4096, Constant::Pi, Round::Up);
    (lo.to_rational().unwrap() / 2, hi.to_rational().unwrap() / 2)
}
