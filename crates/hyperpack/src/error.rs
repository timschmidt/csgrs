//! Error types for packing models and replay.

/// Result alias used by `hyperpack`.
pub type PackResult<T> = Result<T, PackError>;

/// Errors surfaced by exact packing carriers.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PackError {
    /// A stable id was empty.
    EmptyIdentifier,
    /// A dimension was not certified positive.
    NonPositiveDimension,
    /// A placement referenced an unknown item.
    MissingItem,
}
