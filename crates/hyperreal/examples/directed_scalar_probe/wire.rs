//! Exact machine-readable fractions for benchmark/probe JSON.
//!
//! Rational's human Display may produce mixed numbers, which are unsuitable for
//! independent fraction readers. This encoder preserves the exact value as one
//! signed numerator and one positive denominator, without decimal approximation.

use hyperreal::Rational;
use num::bigint::Sign;

/// Encode the stored numerator magnitude, denominator and negative sign as n/d.
/// Include /1 for integers and encode zero without a minus sign. Relies on Rational
/// normalization and allocates text proportional to the integer digit lengths; no
/// length/resource limit is imposed here.
pub fn rational(value: &Rational) -> String {
    let sign = if value.sign() == Sign::Minus { "-" } else { "" };
    format!("{sign}{}/{}", value.numerator(), value.denominator())
}

#[cfg(test)]
/// Finite wire-format regression corpus, including negative improper and large fractions.
mod tests {
    use super::*;

    #[test]
    /// For eight fixed exact fractions, require unchanged canonical text and successful
    /// round trips through Hyperreal and Rug. Retain the contrasting human Display
    /// assertion -7/3 -> -2 1/3. This checks transport for these cases, not all parsers
    /// or arbitrary-size resource behavior.
    fn signed_and_mixed_fractions_round_trip_through_independent_readers() {
        for text in [
            "0/1",
            "1/1",
            "-1/1",
            "7/3",
            "-7/3",
            "1/17",
            "-1/17",
            "-10000000000000000000000000000000000000001/3",
        ] {
            let value = text.parse::<Rational>().unwrap();
            let encoded = rational(&value);
            assert_eq!(encoded, text);
            assert_eq!(encoded.parse::<Rational>().unwrap(), value);
            assert_eq!(
                encoded.parse::<rug::Rational>().unwrap(),
                text.parse::<rug::Rational>().unwrap()
            );
        }
        assert_eq!(Rational::fraction(-7, 3).unwrap().to_string(), "-2 1/3");
    }
}
