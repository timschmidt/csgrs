//! Error types for part graph construction and queries.

/// Result alias used by `hyperparts`.
pub type PartsResult<T> = Result<T, PartsError>;

/// Errors surfaced by the source-attributed part graph.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PartsError {
    /// A stable id field was empty.
    EmptyIdentifier,
    /// A source authority or locator was empty.
    EmptySource,
    /// A voltage envelope was not certified to have `min <= max`.
    InvalidVoltageEnvelope,
    /// An assertion interval had `min > max` or an undecidable ordering.
    InvalidAssertionRange,
    /// A requested part family was not present.
    MissingPart,
    /// A requested variant was not present.
    MissingVariant,
    /// A requested terminal was not present.
    MissingTerminal,
}
