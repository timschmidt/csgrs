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
    /// A support area-ratio policy had an invalid denominator.
    InvalidSupportRatio,
    /// A load, weight, or capacity value was certified negative.
    NegativeLoadValue,
    /// A requested clearance was certified negative.
    NegativeClearance,
    /// More than one declared item used the same stable id.
    DuplicateItem,
    /// More than one declared bin used the same stable id.
    DuplicateBin,
    /// More than one weight claim was supplied for an item.
    DuplicateItemWeight,
    /// More than one load limit was supplied for an item.
    DuplicateLoadLimit,
    /// A placement referenced an unknown item.
    MissingItem,
    /// A placement referenced an unknown bin.
    MissingBin,
}
