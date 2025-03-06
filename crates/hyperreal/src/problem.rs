// We need to refer to these types in the documentation
#[allow(unused_imports)]
use crate::{Rational, Real};

/// Problems when either parsing or attempting Arithmetic with [`Real`] numbers
/// also can occur when trying to make or convert to a [`Rational`]

#[derive(Copy, Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Problem {
    /// Unspecified problem while parsing an expression
    ParseError,
    /// Tried to take the Square Root of a Negative, these values are Imaginary
    SqrtNegative,
    /// Tried to divide by Zero, also arises if attempting to make a fraction with a zero
    /// denominator
    DivideByZero,
    /// The specified identifier in an expression was not found
    NotFound,
    /// The expression has too few parameters to evaluate
    InsufficientParameters,
    /// Tried to convert a floating point NaN, which has no equivalent
    /// or evaluated the Logarithm of a non-positive value
    NotANumber,
    /// Tried to convert a floating point Infinity which has no equivalent
    Infinity,
    /// When parsing a fraction either the numerator or denominator weren't decimal digits
    BadFraction,
    /// When parsing a decimal there was non-digits on one or both sides of the decimal point
    BadDecimal,
    /// When parsing an integer there were non-digits in the text
    BadInteger,
    /// The integer was outside the range for the chosen type
    OutOfRange,
    /// The rational was not an integer
    NotAnInteger,
    /// Operation was rejected because it was likely to consume all available resources
    Exhausted,
}

use std::fmt;

impl fmt::Display for Problem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

impl std::error::Error for Problem {}
