//! Machine-readable fractions; Rational's human Display may use mixed numbers.
use hyperreal::Rational;
use num::bigint::Sign;

pub fn rational(value: &Rational) -> String {
    let sign = if value.sign() == Sign::Minus { "-" } else { "" };
    format!("{sign}{}/{}", value.numerator(), value.denominator())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
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
