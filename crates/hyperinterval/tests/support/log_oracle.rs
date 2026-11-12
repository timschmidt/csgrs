//! Independent directed MPFR logarithm and exact GMP grid rounding.
use hyperinterval::{DyadicInterval, ExtendedInterval, ExtendedIntervalView as View};
use rug::{Float, Rational, float::Round};
#[path = "grid_oracle.rs"]
mod grid_oracle;
pub use grid_oracle::q;
pub fn verify(
    result: &ExtendedInterval<DyadicInterval>,
    a: &Rational,
    b: &Rational,
    precision: i32,
) {
    crate::mpfr_cache::cleanup_on_thread_exit();
    let (mut upper, _) = Float::with_val_round(4096, b, Round::Up);
    upper.ln_round(Round::Up);
    if a <= &0 {
        let (_, expected) =
            grid_oracle::bounds(Rational::from(0), upper.to_rational().unwrap(), precision);
        let View::Below(point) = result.view() else {
            panic!("expected lower-unbounded image");
        };
        let (a, b, e) = point.bounds().unwrap();
        assert_eq!(a, b);
        assert_eq!(q(b, e), expected);
    } else {
        let (mut lower, _) = Float::with_val_round(4096, a, Round::Down);
        lower.ln_round(Round::Down);
        finite(
            result,
            lower.to_rational().unwrap(),
            upper.to_rational().unwrap(),
            precision,
        );
    }
}
fn finite(
    result: &ExtendedInterval<DyadicInterval>,
    lower: Rational,
    upper: Rational,
    precision: i32,
) {
    let (lo, hi) = grid_oracle::bounds(lower, upper, precision);
    let (a, b, e) = result.as_finite().unwrap().bounds().unwrap();
    assert_eq!(q(a, e), lo);
    assert_eq!(q(b, e), hi);
}
pub fn log_two_multiple(result: &ExtendedInterval<DyadicInterval>, k: i64, precision: i32) {
    crate::mpfr_cache::cleanup_on_thread_exit();
    let mut lo = Float::with_val(4096, 2);
    let mut hi = lo.clone();
    lo.ln_round(Round::Down);
    hi.ln_round(Round::Up);
    let a = lo.to_rational().unwrap() * k;
    let b = hi.to_rational().unwrap() * k;
    if k >= 0 {
        finite(result, a, b, precision)
    } else {
        finite(result, b, a, precision)
    }
}
