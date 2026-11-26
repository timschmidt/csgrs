//! Error types for predicate evaluation.
//!
//! This public diagnostic vocabulary is separate from the [`crate::PredicateOutcome`]
//! returned by the main classifiers. Unknown predicate outcomes are not
//! automatically converted into these errors. Messages are static strings;
//! these variants do not retain a typed scalar cause or a resource receipt.

use core::fmt;

/// Public result alias for operations using the predicate error vocabulary.
pub type Result<T> = core::result::Result<T, PredicateError>;

/// Errors that can occur while evaluating a predicate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PredicateError {
    /// Strict predicate escalation requires a Real predicate capability that is unavailable.
    CapabilityUnavailable(
        /// Name or explanation of the missing capability, supplied by the producer.
        &'static str,
    ),
    /// Real refinement or exact evaluation failed.
    Real(
        /// Static scalar diagnostic; no underlying error object is retained.
        &'static str,
    ),
}

impl fmt::Display for PredicateError {
    /// Prefix the stored message with its capability or scalar-error category.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CapabilityUnavailable(capability) => {
                write!(f, "predicate capability unavailable: {capability}")
            }
            Self::Real(message) => write!(f, "predicate Real error: {message}"),
        }
    }
}

#[cfg(feature = "std")]
/// Integrate with standard errors without an underlying source or backtrace payload.
impl std::error::Error for PredicateError {}
