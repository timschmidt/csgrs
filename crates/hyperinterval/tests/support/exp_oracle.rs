//! Independent directed MPFR exponential and exact GMP grid conversion.
use hyperinterval::DyadicInterval;
use rug::{Float, Rational, float::Round};
#[path = "dyadic_oracle.rs"]
mod dyadic_oracle;
pub use dyadic_oracle::q;

pub fn verify(result: &DyadicInterval, a: &Rational, b: &Rational, precision: i32) {
    let (mut lower, _) = Float::with_val_round(4096, a, Round::Down);
    let (mut upper, _) = Float::with_val_round(4096, b, Round::Up);
    lower.exp_round(Round::Down);
    upper.exp_round(Round::Up);
    let mut lo = lower.to_rational().unwrap();
    let mut hi = upper.to_rational().unwrap();
    if precision >= 0 {
        lo >>= precision as u32;
        hi >>= precision as u32;
    } else {
        lo <<= precision.unsigned_abs();
        hi <<= precision.unsigned_abs();
    }
    let expected_lower = q(lo.floor().numer(), precision);
    let expected_upper = q(hi.ceil().numer(), precision);
    let (actual_lower, actual_upper, e) = result.bounds().unwrap();
    assert_eq!(
        q(actual_lower, e),
        expected_lower,
        "lower exp({a}), p={precision}"
    );
    assert_eq!(
        q(actual_upper, e),
        expected_upper,
        "upper exp({b}), p={precision}"
    );
}
