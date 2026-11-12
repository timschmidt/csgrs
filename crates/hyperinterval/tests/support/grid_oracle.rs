//! Exact directed GMP grid conversion shared by transcendental oracles.
use rug::Rational;
#[path = "dyadic_oracle.rs"]
mod dyadic_oracle;
pub use dyadic_oracle::q;
pub fn bounds(lower: Rational, upper: Rational, precision: i32) -> (Rational, Rational) {
    let mut lo = lower;
    let mut hi = upper;
    if precision >= 0 {
        lo >>= precision as u32;
        hi >>= precision as u32;
    } else {
        lo <<= precision.unsigned_abs();
        hi <<= precision.unsigned_abs();
    }
    let expected_lower = q(lo.floor().numer(), precision);
    let expected_upper = q(hi.ceil().numer(), precision);
    (expected_lower, expected_upper)
}
