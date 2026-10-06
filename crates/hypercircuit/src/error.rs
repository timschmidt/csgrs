//! Error types for exact circuit carriers.

use std::fmt::{Display, Formatter};

/// Result alias used by `hypercircuit`.
pub type CircuitResult<T> = Result<T, CircuitError>;

/// Errors surfaced by circuit construction and replay.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CircuitError {
    /// A stable id was empty.
    EmptyIdentifier,
    /// A stamp referenced an unknown net.
    MissingNet,
    /// A net-voltage or branch-current unknown was declared more than once.
    DuplicateUnknown,
    /// A replay candidate had the wrong length.
    CandidateLengthMismatch,
    /// A replay residual sign could not be certified.
    UnknownResidual,
    /// A resistance value was certified negative.
    NegativeResistance,
    /// A requested circuit instance was not present in the circuit.
    MissingInstance,
    /// A requested net was not present in the declarative circuit.
    MissingConnectionNet,
    /// A pin already had a retained net binding.
    DuplicatePinBinding,
    /// A linear MNA system is singular and has no unique solution.
    SingularLinearSystem,
    /// Exact pivot classification could not decide whether a candidate is zero.
    IndeterminateLinearPivot,
    /// Exact scalar division failed during linear elimination.
    LinearSolveArithmetic,
}

impl Display for CircuitError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(match self {
            Self::EmptyIdentifier => "circuit identifier is empty",
            Self::MissingNet => "circuit stamp references a missing net",
            Self::DuplicateUnknown => "MNA unknown is declared more than once",
            Self::CandidateLengthMismatch => "replay candidate has the wrong length",
            Self::UnknownResidual => "replay residual could not be certified",
            Self::NegativeResistance => "resistance is negative",
            Self::MissingInstance => "circuit instance is missing",
            Self::MissingConnectionNet => "connection net is missing",
            Self::DuplicatePinBinding => "instance pin is already connected",
            Self::SingularLinearSystem => "linear MNA system is singular",
            Self::IndeterminateLinearPivot => "linear MNA pivot is indeterminate",
            Self::LinearSolveArithmetic => "linear MNA elimination arithmetic failed",
        })
    }
}

impl std::error::Error for CircuitError {}
