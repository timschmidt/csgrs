//! Exact value transfer to independent GMP without floating conversion.
use rug::{Integer, Rational as Q};

pub fn q(mantissa: &impl core::fmt::Display, exponent: i32) -> Q {
    let mut value = Q::from(mantissa.to_string().parse::<Integer>().unwrap());
    if exponent >= 0 {
        value <<= exponent as u32;
    } else {
        value >>= exponent.unsigned_abs();
    }
    value
}
