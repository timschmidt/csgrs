//! Error types for exact circuit carriers.

/// Result alias used by `hypercircuit`.
pub type CircuitResult<T> = Result<T, CircuitError>;

/// Errors surfaced by circuit construction and replay.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CircuitError {
    /// A stable id was empty.
    EmptyIdentifier,
    /// A stamp referenced an unknown net.
    MissingNet,
    /// A replay candidate had the wrong length.
    CandidateLengthMismatch,
    /// A replay residual sign could not be certified.
    UnknownResidual,
    /// A resistance value was certified negative.
    NegativeResistance,
}
