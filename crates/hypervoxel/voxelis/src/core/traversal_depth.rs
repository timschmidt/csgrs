//! Module `core::traversal_depth`
//!
//! Defines the [`TraversalDepth`] struct, a compact representation of current and maximum depth in a voxel storage node.
//!
//! # Layout
//!
//! Internally represented as a 16-bit value:
//!
//! ```text
//! ┌15──────8┬7────────0┐
//! │ CURRENT │ MAX      │
//! └─────────┴──────────┘
//! ```
//!
//! - Bits 15-8: Current depth value (`u8`)
//! - Bits 7-0: Maximum allowed depth (`u8`)
//!
//! # Examples
//!
//! ```rust
//! use voxelis::TraversalDepth;
//!
//! let depth = TraversalDepth::new(3, 6);
//! assert_eq!(depth.current(), 3);
//! assert_eq!(depth.max(), 6);
//! ```

use crate::interner::MAX_ALLOWED_DEPTH;

/// A combined representation of current and maximum depth.
///
/// Internally stored as a 16-bit value: high byte for current, low byte for max.
///
/// # Examples
///
/// ```rust
/// use voxelis::TraversalDepth;
///
/// let depth = TraversalDepth::new(0, 6);
/// assert_eq!(depth.current(), 0);
/// assert_eq!(depth.max(), 6);
/// ```
#[repr(transparent)]
#[derive(Copy, Clone)]
pub struct TraversalDepth(u16);

impl TraversalDepth {
    /// Creates a new [`TraversalDepth`].
    ///
    /// # Panics
    /// - If `current > max`.
    /// - If `max >= MAX_ALLOWED_DEPTH`.
    ///
    /// # Parameters
    /// - `current` - Current depth value (`u8`).
    /// - `max` - Maximum allowed depth (`u8`), must be less than `MAX_ALLOWED_DEPTH`.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use voxelis::TraversalDepth;
    ///
    /// let depth = TraversalDepth::new(3, 6);
    /// assert_eq!(depth.current(), 3);
    /// assert_eq!(depth.max(), 6);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn new(current: u8, max: u8) -> Self {
        assert!(
            current <= max,
            "Current depth cannot be greater than max depth"
        );
        assert!(
            max < MAX_ALLOWED_DEPTH as u8,
            "Max depth exceeds allowed limit"
        );
        Self(((current as u16) << 8) | max as u16)
    }

    /// Returns the current depth.
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::TraversalDepth;
    ///
    /// let depth = TraversalDepth::new(3, 6);
    /// assert_eq!(depth.current(), 3);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn current(&self) -> u8 {
        (self.0 >> 8) as u8
    }

    /// Returns the maximum depth.
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::TraversalDepth;
    ///
    /// let depth = TraversalDepth::new(3, 6);
    /// assert_eq!(depth.max(), 6);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn max(&self) -> u8 {
        (self.0 & 0xFF) as u8
    }

    /// Returns a new [`TraversalDepth`] with current incremented by 1.
    ///
    /// # Panics
    ///
    /// Panics if the new current exceeds max.
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::TraversalDepth;
    ///
    /// let depth = TraversalDepth::new(3, 6);
    /// let incremented = depth.increment();
    /// assert_eq!(incremented.current(), 4);
    /// assert_eq!(incremented.max(), 6);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn increment(&self) -> Self {
        Self::new(self.current() + 1, self.max())
    }

    /// Returns a new [`TraversalDepth`] with current decremented by 1.
    ///
    /// # Panics
    ///
    /// Panics if current is zero.
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::TraversalDepth;
    ///
    /// let depth = TraversalDepth::new(3, 6);
    /// let decremented = depth.decrement();
    /// assert_eq!(decremented.current(), 2);
    /// assert_eq!(decremented.max(), 6);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn decrement(&self) -> Self {
        assert!(self.current() > 0, "Current depth cannot be less than zero");
        Self::new(self.current() - 1, self.max())
    }
}

/// Display implementation for [`TraversalDepth`] that provides a human-readable representation
impl std::fmt::Display for TraversalDepth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}", self.current(), self.max())
    }
}

/// Debug implementation for [`TraversalDepth`] that provides a human-readable representation
impl std::fmt::Debug for TraversalDepth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}", self.current(), self.max())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let depth = TraversalDepth::new(3, 6);
        assert_eq!(depth.current(), 3);
        assert_eq!(depth.max(), 6);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Max depth exceeds allowed limit")]
    fn test_max_allowed_depth() {
        let _ = TraversalDepth::new(3, MAX_ALLOWED_DEPTH as u8);
    }

    #[test]
    fn test_increment() {
        let depth = TraversalDepth::new(3, 6);
        let incremented = depth.increment();
        assert_eq!(incremented.current(), 4);
        assert_eq!(incremented.max(), 6);
    }

    #[test]
    fn test_decrement() {
        let depth = TraversalDepth::new(3, 6);
        let decremented = depth.decrement();
        assert_eq!(decremented.current(), 2);
        assert_eq!(decremented.max(), 6);
    }

    #[test]
    fn test_display_and_debug() {
        let depth = TraversalDepth::new(3, 6);
        assert_eq!(format!("{depth}"), "3/6");
        assert_eq!(format!("{depth:?}"), "3/6");
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Current depth cannot be greater than max depth")]
    fn test_new_current_greater_than_max() {
        let _ = TraversalDepth::new(10, 5);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Current depth cannot be greater than max depth")]
    fn test_increment_overflow() {
        let depth = TraversalDepth::new(5, 5);
        let _ = depth.increment();
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Current depth cannot be less than zero")]
    fn test_decrement_underflow() {
        let depth = TraversalDepth::new(0, 5);
        let _ = depth.decrement();
    }

    #[test]
    #[should_panic(expected = "Max depth exceeds allowed limit")]
    fn test_new_current_eq_max() {
        let depth = TraversalDepth::new(7, 7);
        assert_eq!(depth.current(), 7);
        assert_eq!(depth.max(), 7);
    }

    #[test]
    fn test_zero_zero_depth() {
        let depth = TraversalDepth::new(0, 0);
        assert_eq!(depth.current(), 0);
        assert_eq!(depth.max(), 0);
    }

    #[test]
    fn test_maximum_valid_depth() {
        let max = MAX_ALLOWED_DEPTH as u8 - 1;
        let depth = TraversalDepth::new(max, max);
        assert_eq!(depth.current(), max);
        assert_eq!(depth.max(), max);
    }

    #[test]
    fn test_increment_to_max() {
        let max = MAX_ALLOWED_DEPTH as u8 - 1;
        let depth = TraversalDepth::new(max - 1, max);
        let inc = depth.increment();
        assert_eq!(inc.current(), max);
    }

    #[test]
    fn test_decrement_to_zero() {
        let depth = TraversalDepth::new(1, 5);
        let dec = depth.decrement();
        assert_eq!(dec.current(), 0);
    }
}
