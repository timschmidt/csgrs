//! Error types for predicate evaluation.

use core::fmt;

/// Crate-local result type.
pub type Result<T> = core::result::Result<T, PredicateError>;

/// Errors that can occur while evaluating a predicate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PredicateError {
    /// The selected policy requires a backend capability that is unavailable.
    CapabilityUnavailable(&'static str),
    /// The backend rejected refinement or exact evaluation.
    Backend(&'static str),
}

impl fmt::Display for PredicateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CapabilityUnavailable(capability) => {
                write!(f, "predicate capability unavailable: {capability}")
            }
            Self::Backend(message) => write!(f, "predicate backend error: {message}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for PredicateError {}
