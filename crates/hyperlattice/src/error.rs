use std::error::Error;
use std::fmt;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

/// Shared cancellation signal used by abort-aware APIs.
///
/// With the hyperreal backend, the signal is attached to cloned `Real` values
/// before operations that may evaluate unknown computable reals. The approx
/// backend accepts the same API as a no-op.
pub type AbortSignal = Arc<AtomicBool>;

/// Error type returned by fallible scalar, complex, vector, and matrix APIs.
///
/// Most variants mirror errors from `hyperreal::Problem`; `UnknownZero` is
/// crate-owned and indicates that a checked operation could not prove a divisor
/// or pivot was non-zero.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Problem {
    /// Parsing a numeric value failed.
    ParseError,
    /// A square root was requested for a definitely negative value.
    SqrtNegative,
    /// Division by a definitely zero value was requested.
    DivideByZero,
    /// A requested value could not be found.
    NotFound,
    /// A numeric operation did not receive enough parameters.
    InsufficientParameters,
    /// A conversion or operation produced a NaN.
    NotANumber,
    /// A conversion or operation produced infinity.
    Infinity,
    /// Fraction construction failed.
    BadFraction,
    /// Decimal construction failed.
    BadDecimal,
    /// Integer construction or conversion failed.
    BadInteger,
    /// A conversion was outside the supported numeric range.
    OutOfRange,
    /// An integer-only operation received a non-integer value.
    NotAnInteger,
    /// Evaluation exhausted the available precision or work budget.
    Exhausted,
    /// A checked operation could not determine whether a value was zero.
    UnknownZero,
}

#[cfg(feature = "hyperreal-backend")]
impl From<hyperreal::Problem> for Problem {
    fn from(problem: hyperreal::Problem) -> Self {
        match problem {
            hyperreal::Problem::ParseError => Self::ParseError,
            hyperreal::Problem::SqrtNegative => Self::SqrtNegative,
            hyperreal::Problem::DivideByZero => Self::DivideByZero,
            hyperreal::Problem::NotFound => Self::NotFound,
            hyperreal::Problem::InsufficientParameters => Self::InsufficientParameters,
            hyperreal::Problem::NotANumber => Self::NotANumber,
            hyperreal::Problem::Infinity => Self::Infinity,
            hyperreal::Problem::BadFraction => Self::BadFraction,
            hyperreal::Problem::BadDecimal => Self::BadDecimal,
            hyperreal::Problem::BadInteger => Self::BadInteger,
            hyperreal::Problem::OutOfRange => Self::OutOfRange,
            hyperreal::Problem::NotAnInteger => Self::NotAnInteger,
            hyperreal::Problem::Exhausted => Self::Exhausted,
            _ => Self::Exhausted,
        }
    }
}

impl fmt::Display for Problem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

impl Error for Problem {}

/// Result type used by fallible operations in this crate.
pub type BlasResult<T> = Result<T, Problem>;

/// Result type used by APIs that reject unknown-zero conditions.
pub type CheckedBlasResult<T> = BlasResult<T>;
