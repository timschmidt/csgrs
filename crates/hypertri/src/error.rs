use std::fmt;

/// Crate-local result type.
pub type Result<T> = std::result::Result<T, Error>;

/// Errors returned by triangulation APIs.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Error {
    /// A numeric predicate could not be decided within the configured exact
    /// policy.
    PredicateUndecided {
        /// Predicate or Real decision that failed.
        predicate: &'static str,
    },
    /// Input data violates the API contract.
    InvalidInput {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// The current implementation could not find an ear in a polygon ring.
    NoEarFound,
    /// The requested feature is not compiled or cannot be satisfied by the
    /// active exact policy.
    UnsupportedFeature {
        /// Feature that is not available in the current build or policy.
        feature: &'static str,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PredicateUndecided { predicate } => {
                write!(f, "predicate could not be decided: {predicate}")
            }
            Self::InvalidInput { reason } => write!(f, "invalid input: {reason}"),
            Self::NoEarFound => write!(f, "no valid polygon ear could be found"),
            Self::UnsupportedFeature { feature } => write!(f, "unsupported feature: {feature}"),
        }
    }
}

impl std::error::Error for Error {}
