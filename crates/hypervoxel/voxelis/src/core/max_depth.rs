//! Module `core::max_depth`
//!
//! Defines the [`MaxDepth`] struct, a compact and type-safe representation of the maximum allowed depth for an octree or voxel structure.
//!
//! # Usage
//!
//! Use [`MaxDepth`] to avoid confusion and bugs related to raw integer depth parameters. This type enforces invariants and provides clear API boundaries.
//!
//! # Examples
//!
//! ```rust
//! use voxelis::MaxDepth;
//!
//! let depth = MaxDepth::new(6);
//! assert_eq!(depth.max(), 6);
//! ```

use crate::storage::node::MAX_ALLOWED_DEPTH;

/// A representation of maximum depth.
///
/// # Examples
///
/// ```rust
/// use voxelis::MaxDepth;
///
/// let depth = MaxDepth::new(6);
/// assert_eq!(depth.max(), 6);
/// ```
#[repr(transparent)]
#[derive(Copy, Clone)]
pub struct MaxDepth(u8);

impl From<MaxDepth> for u8 {
    #[inline]
    fn from(id: MaxDepth) -> u8 {
        id.0
    }
}

impl TryFrom<u8> for MaxDepth {
    type Error = &'static str;

    #[inline]
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        if value < MAX_ALLOWED_DEPTH as u8 {
            Ok(Self(value))
        } else {
            Err("Max depth exceeds allowed limit")
        }
    }
}

impl MaxDepth {
    /// Creates a new [`MaxDepth`].
    ///
    /// # Parameters
    ///
    /// - `max`: Maximum allowed depth (`u8`). Must be less than [`MAX_ALLOWED_DEPTH`].
    ///
    /// # Panics
    ///
    /// Panics if `max >= MAX_ALLOWED_DEPTH`.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use voxelis::MaxDepth;
    ///
    /// let depth = MaxDepth::new(6);
    /// assert_eq!(depth.max(), 6);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn new(max: u8) -> Self {
        assert!(
            max < MAX_ALLOWED_DEPTH as u8,
            "Max depth exceeds allowed limit"
        );
        Self(max)
    }

    /// Returns the maximum depth as `u8`.
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::MaxDepth;
    ///
    /// let depth = MaxDepth::new(6);
    /// assert_eq!(depth.max(), 6);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn max(&self) -> u8 {
        self.0
    }

    /// Returns the maximum depth as a `usize`.
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::MaxDepth;
    ///
    /// let depth = MaxDepth::new(6);
    /// assert_eq!(depth.as_usize(), 6usize);
    /// ```
    #[must_use]
    pub const fn as_usize(&self) -> usize {
        self.0 as usize
    }
}

/// Display implementation for [`MaxDepth`] that provides a human-readable representation
impl std::fmt::Display for MaxDepth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.max())
    }
}

/// Debug implementation for [`MaxDepth`] that provides a human-readable representation
impl std::fmt::Debug for MaxDepth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.max())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let depth = MaxDepth::new(6);
        assert_eq!(depth.max(), 6);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Max depth exceeds allowed limit")]
    fn test_max_allowed_depth() {
        let _ = MaxDepth::new(MAX_ALLOWED_DEPTH as u8);
    }

    #[test]
    fn test_display_and_debug() {
        let depth = MaxDepth::new(6);
        assert_eq!(format!("{depth}"), "6");
        assert_eq!(format!("{depth:?}"), "6");
    }

    #[test]
    #[should_panic(expected = "Max depth exceeds allowed limit")]
    fn test_new_current_eq_max() {
        let _ = MaxDepth::new(MAX_ALLOWED_DEPTH as u8);
    }

    #[test]
    fn test_zero_zero_depth() {
        let depth = MaxDepth::new(0);
        assert_eq!(depth.max(), 0);
    }

    #[test]
    fn test_maximum_valid_depth() {
        let max = MAX_ALLOWED_DEPTH as u8 - 1;
        let depth = MaxDepth::new(max);
        assert_eq!(depth.max(), max);
    }

    #[test]
    fn test_try_from_ok() {
        let val: u8 = 5;
        let depth = MaxDepth::try_from(val);
        assert!(depth.is_ok());
        assert_eq!(depth.unwrap().max(), 5);
    }

    #[test]
    fn test_try_from_err() {
        let val: u8 = MAX_ALLOWED_DEPTH as u8;
        let depth = MaxDepth::try_from(val);
        assert!(depth.is_err());
    }
}
