use hyperlimit::{Escalation, RefinementNeed};
use hyperreal::Problem;

/// A failed transformation or construction; never a successful zero coordinate.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Input violates a stated domain precondition.
    InvalidInput(&'static str),
    /// A native scalar operation failed.
    Scalar(Problem),
    /// A domain/branch decision could not be established under the policy.
    Unknown {
        /// The unresolved domain claim.
        claim: &'static str,
        /// Required additional predicate capability.
        needed: RefinementNeed,
        /// Stage at which the predicate stopped.
        stage: Escalation,
    },
    /// A named Hyperspatial resource limit was reached.
    ResourceExhausted(&'static str),
    /// Cooperative cancellation was requested.
    Cancelled,
    /// Coordinates and operation use different reference frames or epochs.
    ReferenceMismatch,
    /// Local coordinates belong to a different prepared anchor.
    LocalFrameMismatch,
    /// The selected inverse has multiple solutions or needs a pole convention.
    AmbiguousInverse(&'static str),
}

impl From<Problem> for Error {
    fn from(value: Problem) -> Self {
        Self::Scalar(value)
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Scalar(problem) => Some(problem),
            _ => None,
        }
    }
}

/// Fallible native operation result.
pub type Result<T> = std::result::Result<T, Error>;
