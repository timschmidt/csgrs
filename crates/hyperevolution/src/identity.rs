//! Stable search identifiers.

/// Error returned by id constructors.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EvolutionError {
    /// A stable id was empty.
    EmptyIdentifier,
}

/// Result alias used by `hyperevolution`.
pub type EvolutionResult<T> = Result<T, EvolutionError>;

/// Stable candidate identifier.
#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct CandidateId(String);

impl CandidateId {
    /// Creates a non-empty candidate id.
    pub fn new(value: impl Into<String>) -> EvolutionResult<Self> {
        let value = value.into();
        if value.is_empty() {
            return Err(EvolutionError::EmptyIdentifier);
        }
        Ok(Self(value))
    }

    /// Returns the id text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}
