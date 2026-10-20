use super::{Error, ErrorKind, Resource, error::check};

/// Caller-selected bounds before materializing an exact decimal rational.
#[derive(Clone, Copy, Debug)]
pub struct NumberLimits {
    /// Maximum significand digits, including leading and trailing zeros.
    pub digits: usize,
    /// Maximum absolute authored exponent. Zero does not bypass this charge.
    pub exponent: usize,
}
impl Default for NumberLimits {
    fn default() -> Self {
        Self {
            digits: 4096,
            exponent: 4096,
        }
    }
}

/// A validated numeric lexeme. Syntax parsing never converts it to a float.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Number<'a>(pub(super) &'a str);

impl<'a> Number<'a> {
    /// Original spelling, including sign, decimal point and exponent.
    pub fn lexeme(self) -> &'a str {
        self.0
    }

    /// Materializes an exact rational through Hyperreal after admission.
    ///
    /// This bounds textual digit/exponent amplification, not total arithmetic
    /// work or all allocations in the scalar implementation. It never uses
    /// `Real`'s lazy scientific-notation fallback.
    pub fn to_real(self, limits: &NumberLimits) -> Result<hyperreal::Real, Error> {
        let (mantissa, exponent) = self.0.split_once('E').unwrap_or((self.0, "0"));
        let digits = mantissa.bytes().filter(u8::is_ascii_digit).count();
        check(Resource::DecimalDigits, digits, limits.digits, 0)?;
        let exponent = exponent.trim_start_matches(['+', '-']);
        let mut magnitude = 0usize;
        for digit in exponent.bytes() {
            magnitude = magnitude
                .checked_mul(10)
                .and_then(|n| n.checked_add(usize::from(digit - b'0')))
                .ok_or_else(|| {
                    Error::new(
                        0,
                        ErrorKind::Limit {
                            resource: Resource::DecimalExponent,
                            limit: limits.exponent,
                            observed: usize::MAX,
                        },
                    )
                })?;
        }
        check(Resource::DecimalExponent, magnitude, limits.exponent, 0)?;
        let value = self
            .0
            .parse::<hyperreal::Rational>()
            .map_err(|problem| Error::new(0, ErrorKind::Numeric(problem)))?;
        Ok(hyperreal::Real::new(value))
    }
}
