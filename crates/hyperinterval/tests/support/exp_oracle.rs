//! Independent directed MPFR exponential and exact GMP grid conversion.
use hyperinterval::DyadicInterval;
use rug::{Float, Rational, float::Round};
#[path = "grid_oracle.rs"]
mod grid_oracle;
pub use grid_oracle::q;

pub fn verify(result: &DyadicInterval, a: &Rational, b: &Rational, precision: i32) {
    let (mut lower, _) = Float::with_val_round(4096, a, Round::Down);
    let (mut upper, _) = Float::with_val_round(4096, b, Round::Up);
    lower.exp_round(Round::Down);
    upper.exp_round(Round::Up);
    let (expected_lower, expected_upper) = grid_oracle::bounds(
        lower.to_rational().unwrap(),
        upper.to_rational().unwrap(),
        precision,
    );
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
